"""Self-labels for the track perception net without a map: geometry seen later by the same sensor.

    uv run --group bags python -m racer_pnp_perception.selflabel --bags <bag dir> [<bag dir> ...] \\
        [--out /Users/cameronjim/code/car-data/pnp/selflabels] [--moving-mps 0.5] \\
        [--split /Users/cameronjim/code/car/evaluation/prereg/pnp-v0.2-split.yaml]

The slam_toolbox map of the oval came out doubled (no loop closure, no gyro), so labels from a map
and a localised pose are not available. Over a few metres, scan-to-scan ICP drift is centimetres,
which is enough for local labels:

1. Motion. Planar motion of the rear axle per scan interval from ICP (`compute_motion`, the
   matching and quality flags of racer_pnp_dynamics.icp_motion: RMS at most 0.02 m, inlier share
   at least 0.7), with tools/safety_replay/motion.py doing the matching.
2. Submap. For scan i, the points of the scans from 1.0 m of travel behind to 4.0 m ahead (by the
   accumulated ICP distance, at most 80 scans), each moved into the base_link frame of scan i by
   chaining the motions, thinned to a 0.03 m voxel grid. The window is dropped when any interval
   in it failed the quality flag, or when the chained yaw exceeds 2 pi.
3. Label. The geometric tracer (`geometric.trace`) on the submap, in scan i's frame. Kept only
   when valid_count is at least 12, the label's own intrusion against the submap is at most 0.03,
   its clearance deficit p90 is at most 0.05 m and station 0 and the travel direction follow the
   contract (the centreline point nearest the car, direction within 90 degrees of the heading).
4. Driven path (label free, a second gate and an evaluation number). The car's own path over the
   next 2.0 m (chained ICP, rear axle positions) must lie inside the labelled corridor shrunk by
   the half body width on at least 95 percent of its points, otherwise the label is dropped.
   `path_containment` scores any predicted track the same way.

With `--split` only the scans inside a segment of the pre-registered split are candidates, and a
label whose submap window (first to last scan of the window) is not inside ONE segment is dropped
with reason `segment_edge`. Every kept label then carries its segment, its role and the bag time
of the first and last scan of its window (`split.py` has the time axis). Bags the split does not
name, or marks unused, are skipped.

Output per bag: `<out>/<bag name>.npz` (the scan tensor exactly as `preprocess.scan_to_tensor`
gives it, the label [16, 5] in the net's output layout with log sigma from the tracer, the scan
stamp and index, the gate numbers, the driven path) and `<out>/manifest.json` (counts kept and
dropped per reason, git sha). Never committed.

The label is geometry seen later by the same sensor. It is not the steering anyone chose, and
`/drive` and `/drive_raw` are not used anywhere here.
"""

from __future__ import annotations

import argparse
import json
import logging
import math
import multiprocessing
import subprocess
import sys
from collections import Counter
from dataclasses import dataclass, field
from pathlib import Path
from typing import Any

import numpy as np
from numpy.typing import NDArray
from racer_track_perception import geometric, preprocess, selfcheck
from racer_track_perception import track as tr
from racer_track_perception.vehicle import LiveVehicle, find_repo_root, load_live_vehicle
from scipy.spatial import cKDTree

from racer_pnp_perception.eval_bags import clearance_deficit
from racer_pnp_perception.split import Segment, Split, load_split

log = logging.getLogger(__name__)

DEFAULT_OUT = Path("/Users/cameronjim/code/car-data/pnp/selflabels")

# Motion (the flags of racer_pnp_dynamics.icp_motion).
SCAN_DT_S = 0.1
DT_TOLERANCE_S = 0.03
ICP_MAX_RANGE_M = 4.0
ICP_MIN_RANGE_M = 0.12
INLIER_M = 0.05
RMS_MAX_M = 0.02
INLIER_SHARE_MIN = 0.7

# Submap.
BEHIND_M = 1.0
AHEAD_M = 4.0
MAX_WINDOW_SCANS = 80
MAX_BEHIND_SCANS = 15
MAX_AHEAD_SCANS = MAX_WINDOW_SCANS - 1 - MAX_BEHIND_SCANS
VOXEL_M = 0.03
MAX_CHAINED_YAW_RAD = 2.0 * math.pi

# Label gates.
MIN_VALID_COUNT = 12
MAX_INTRUSION = 0.03
MAX_DEFICIT_P90_M = 0.05
STATION0_TOL_M = 0.01

# Driven path.
PATH_LENGTH_M = 2.0
PATH_SPACING_M = 0.1
PATH_POINTS = round(PATH_LENGTH_M / PATH_SPACING_M) + 1
MIN_PATH_CONTAINMENT = 0.95
# A driven-path point this far before the first station or past the last one is still measured
# against the extension of that end segment; further out it counts as outside the corridor.
PATH_END_EXTENSION_M = 0.25

SIGMA_MAX_M = 0.25

REASONS = (
    "icp_failed",
    "yaw_wrap",
    "path_short",
    "valid_count",
    "contract",
    "intrusion",
    "deficit",
    "path_outside",
    "segment_edge",
)


# ---------------------------------------------------------------------------------------------
# Scans and motion
# ---------------------------------------------------------------------------------------------


@dataclass(frozen=True)
class ScanRec:
    """One scan in the shape tools/safety_replay/motion.py reads (ranges, beam geometry)."""

    stamp_s: float
    angle_min: float
    angle_increment: float
    range_min: float
    range_max: float
    ranges: Any


@dataclass(frozen=True)
class ScanSet:
    """Scans of one bag with a common beam layout."""

    stamp_s: NDArray[np.float64]  # [n]
    ranges: NDArray[np.float32]  # [n, beams]
    angle_min: float
    angle_increment: float
    range_min: float
    range_max: float
    # Recorder log time of each scan in seconds after the first scan's (the split's time axis).
    # None for a scan cache written before the split existed.
    log_s: NDArray[np.float64] | None = None

    def __len__(self) -> int:
        return len(self.stamp_s)

    def bag_time_s(self) -> NDArray[np.float64]:
        """Bag time of every scan: the log time axis of the split, or (without it) the header
        stamp after the first scan's."""
        if self.log_s is not None:
            return self.log_s
        return self.stamp_s - self.stamp_s[0] if len(self) else self.stamp_s

    def rec(self, i: int) -> ScanRec:
        return ScanRec(
            float(self.stamp_s[i]),
            self.angle_min,
            self.angle_increment,
            self.range_min,
            self.range_max,
            self.ranges[i],
        )

    def recs(self) -> list[ScanRec]:
        return [self.rec(i) for i in range(len(self))]


def _tools() -> None:
    tools = find_repo_root() / "tools"
    if str(tools) not in sys.path:
        sys.path.insert(0, str(tools))


def _motion_module() -> Any:
    _tools()
    from safety_replay import motion

    return motion


def motion_mount_and_body(live: LiveVehicle) -> tuple[Any, Any]:
    m = _motion_module()
    mount = m.Mount(live.mount.x_m, live.mount.y_m, live.mount.yaw_rad)
    body = m.Body(live.body.x_max_m, -live.body.x_min_m, live.body.half_width_m)
    return mount, body


@dataclass(frozen=True)
class MotionSeries:
    """One entry per scan interval i (scan i to scan i + 1): the rear axle at scan i + 1 in the
    base_link frame at scan i."""

    dx_m: NDArray[np.float64]
    dy_m: NDArray[np.float64]
    dyaw_rad: NDArray[np.float64]
    rms_m: NDArray[np.float64]
    inlier_share: NDArray[np.float64]
    ok: NDArray[np.bool_]  # computed and passed the quality gate


def scan_motion(
    earlier: ScanRec,
    later: ScanRec,
    mount: Any,
    ignore: frozenset[tuple[int, int]] | None = None,
    init: tuple[float, float, float] = (0.0, 0.0, 0.0),
) -> tuple[float, float, float, float, float]:
    """(dx, dy, dyaw, rms, inlier share) of the rear axle at `later` in the frame at `earlier`.

    `motion.icp` starts from the identity, which in a lane (two parallel walls) leaves the motion
    along the lane unobserved and stops early: on the 2026-11-12 bag its distance was 0.78 of the
    wheel odometry. The later scan is first moved by `init` (the previous interval's motion, a
    constant velocity guess) and the ICP estimates the remainder, which brings the distance to
    0.91 of the wheel odometry on the same stretch. Raises ValueError when a scan has too few
    usable returns."""
    m = _motion_module()
    a = m.scan_points(earlier, mount, ICP_MIN_RANGE_M, ICP_MAX_RANGE_M, ignore)
    b = m.scan_points(later, mount, ICP_MIN_RANGE_M, ICP_MAX_RANGE_M, ignore)
    c0, s0 = math.cos(init[2]), math.sin(init[2])
    r0 = np.array([[c0, -s0], [s0, c0]])
    t0 = np.array(init[:2], dtype=np.float64)
    r1, t1, rms = m.icp(b @ r0.T + t0, a, inlier_m=INLIER_M)
    rotation = r1 @ r0
    translation = r1 @ t0 + t1
    d, _ = cKDTree(a).query(b @ rotation.T + translation)
    share = float(np.mean(d < INLIER_M))
    dyaw = float(np.arctan2(rotation[1, 0], rotation[0, 0]))
    return float(translation[0]), float(translation[1]), dyaw, rms, share


def quality_ok(rms: float, share: float) -> bool:
    return bool(rms <= RMS_MAX_M and share >= INLIER_SHARE_MIN)


_WORK: dict[str, Any] = {}
BLOCK_INTERVALS = 64


def _block(indices: list[int]) -> list[tuple[float, ...]]:
    """Consecutive intervals, each started from the previous interval's motion when that passed
    the quality flags (from the identity at the start of the block and after a failure)."""
    scans: ScanSet = _WORK["scans"]
    out: list[tuple[float, ...]] = []
    init = (0.0, 0.0, 0.0)
    for i in indices:
        try:
            r = scan_motion(scans.rec(i), scans.rec(i + 1), _WORK["mount"], _WORK["ignore"], init)
        except ValueError:
            out.append((math.nan,) * 5)
            init = (0.0, 0.0, 0.0)
            continue
        out.append(r)
        init = r[:3] if quality_ok(r[3], r[4]) else (0.0, 0.0, 0.0)
    return out


def _blocks(todo: list[int]) -> list[list[int]]:
    blocks: list[list[int]] = []
    for i in todo:
        if blocks and blocks[-1][-1] == i - 1 and len(blocks[-1]) < BLOCK_INTERVALS:
            blocks[-1].append(i)
        else:
            blocks.append([i])
    return blocks


def compute_motion(
    scans: ScanSet,
    live: LiveVehicle,
    want: NDArray[np.bool_] | None = None,
    workers: int = 1,
) -> MotionSeries:
    """Motion for every interval where `want[i]` is true (all by default), the rest not ok."""
    n = max(len(scans) - 1, 0)
    want = np.ones(n, dtype=bool) if want is None else np.asarray(want, dtype=bool)
    m = _motion_module()
    mount, body = motion_mount_and_body(live)
    ignore = m.static_self_returns(scans.recs(), mount, body)
    todo = [i for i in range(n) if want[i]]
    blocks = _blocks(todo)
    _WORK.update(scans=scans, mount=mount, ignore=ignore)
    if workers > 1:  # pragma: no cover - process pool, exercised by the real run only
        with multiprocessing.get_context("fork").Pool(workers) as pool:
            results = pool.map(_block, blocks, chunksize=1)
    else:
        results = [_block(b) for b in blocks]
    out = np.full((n, 5), np.nan)
    for block, rows in zip(blocks, results, strict=True):
        for i, r in zip(block, rows, strict=True):
            out[i] = r
    dt = np.diff(scans.stamp_s) if n else np.zeros(0)
    with np.errstate(invalid="ignore"):
        ok = (
            np.isfinite(out).all(axis=1)
            & (np.abs(dt - SCAN_DT_S) < DT_TOLERANCE_S)
            & (out[:, 3] <= RMS_MAX_M)
            & (out[:, 4] >= INLIER_SHARE_MIN)
        )
    return MotionSeries(out[:, 0], out[:, 1], out[:, 2], out[:, 3], out[:, 4], ok)


def extend_motion(
    series: MotionSeries,
    scans: ScanSet,
    live: LiveVehicle,
    want: NDArray[np.bool_],
    workers: int = 1,
) -> MotionSeries:
    """`series` with the intervals that are wanted but were never computed (NaN) filled in. The
    cached intervals are kept as they are."""
    missing = want & ~np.isfinite(series.rms_m)
    if not missing.any():
        return series
    new = compute_motion(scans, live, missing, workers)
    take = missing & np.isfinite(new.rms_m)

    def merge(old: NDArray[Any], fresh: NDArray[Any]) -> NDArray[Any]:
        return np.where(take, fresh, old)

    return MotionSeries(
        merge(series.dx_m, new.dx_m),
        merge(series.dy_m, new.dy_m),
        merge(series.dyaw_rad, new.dyaw_rad),
        merge(series.rms_m, new.rms_m),
        merge(series.inlier_share, new.inlier_share),
        np.where(take, new.ok, series.ok),
    )


def save_motion(path: Path, series: MotionSeries) -> None:
    path.parent.mkdir(parents=True, exist_ok=True)
    np.savez_compressed(path, **{k: getattr(series, k) for k in series.__dataclass_fields__})


def load_motion(path: Path) -> MotionSeries:
    with np.load(path) as z:
        return MotionSeries(**{k: z[k] for k in MotionSeries.__dataclass_fields__})


# ---------------------------------------------------------------------------------------------
# Chaining
# ---------------------------------------------------------------------------------------------


@dataclass(frozen=True)
class Chain:
    """Poses of every scan in the frame of the first scan of its unbroken run (an interval that
    failed the quality flag starts a new run), and the travelled distance along the chain."""

    poses: NDArray[np.float64]  # [n, 3] x, y, yaw
    run: NDArray[np.int64]  # [n] run id, constant while no interval failed
    travel_m: NDArray[np.float64]  # [n] cumulative ICP distance (not reset at a failure)
    ok: NDArray[np.bool_]  # [n - 1]


def chain_motion(series: MotionSeries) -> Chain:
    """Chain the per interval motions (each in the frame at its start) into poses."""
    n = len(series.ok) + 1
    poses = np.zeros((n, 3))
    run = np.zeros(n, dtype=np.int64)
    travel = np.zeros(n)
    for i in range(n - 1):
        if not series.ok[i]:
            poses[i + 1] = 0.0
            run[i + 1] = run[i] + 1
            travel[i + 1] = travel[i]
            continue
        x, y, yaw = poses[i]
        c, s = math.cos(yaw), math.sin(yaw)
        dx, dy = float(series.dx_m[i]), float(series.dy_m[i])
        poses[i + 1] = (x + c * dx - s * dy, y + s * dx + c * dy, yaw + float(series.dyaw_rad[i]))
        run[i + 1] = run[i]
        travel[i + 1] = travel[i] + math.hypot(dx, dy)
    return Chain(poses, run, travel, np.asarray(series.ok, dtype=bool))


def relative_poses(chain: Chain, i: int, lo: int, hi: int) -> NDArray[np.float64]:
    """Poses of scans lo..hi inclusive in the base_link frame of scan i: [hi - lo + 1, 3]. Valid
    only inside one run."""
    xi, yi, ai = chain.poses[i]
    c, s = math.cos(ai), math.sin(ai)
    seg = chain.poses[lo : hi + 1]
    dx, dy = seg[:, 0] - xi, seg[:, 1] - yi
    out = np.empty((len(seg), 3))
    out[:, 0] = c * dx + s * dy
    out[:, 1] = -s * dx + c * dy
    out[:, 2] = seg[:, 2] - ai
    return out


def window_bounds(chain: Chain, i: int) -> tuple[int, int]:
    """First and last scan of the submap window of scan i: 1.0 m of travel behind to 4.0 m ahead,
    at most 80 scans."""
    n = len(chain.travel_m)
    t = chain.travel_m
    lo = int(np.searchsorted(t, t[i] - BEHIND_M, side="left"))
    hi = int(np.searchsorted(t, t[i] + AHEAD_M, side="right")) - 1
    return max(lo, i - MAX_BEHIND_SCANS, 0), min(hi, i + MAX_AHEAD_SCANS, n - 1)


def window_status(chain: Chain, i: int, lo: int, hi: int) -> str | None:
    """None when the window can be used, else the drop reason."""
    if not bool(chain.ok[lo:hi].all()):
        return "icp_failed"
    rel = relative_poses(chain, i, lo, hi)
    if float(np.abs(rel[:, 2]).max()) > MAX_CHAINED_YAW_RAD:
        return "yaw_wrap"
    return None


# ---------------------------------------------------------------------------------------------
# Submap
# ---------------------------------------------------------------------------------------------


def voxel_thin(points: NDArray[np.float64], voxel_m: float = VOXEL_M) -> NDArray[np.float64]:
    """One point per voxel of `voxel_m`: the mean of the points in it."""
    if len(points) == 0:
        return points
    ij = np.floor(points / voxel_m).astype(np.int64)
    ij -= ij.min(axis=0)
    key = ij[:, 0] * (int(ij[:, 1].max()) + 1) + ij[:, 1]
    _, inverse, counts = np.unique(key, return_inverse=True, return_counts=True)
    out = np.empty((len(counts), 2))
    out[:, 0] = np.bincount(inverse, weights=points[:, 0]) / counts
    out[:, 1] = np.bincount(inverse, weights=points[:, 1]) / counts
    return out


def build_submap(
    points: list[NDArray[np.floating]], rel_poses: NDArray[np.float64]
) -> NDArray[np.float64]:
    """Points of several scans (each in its own base_link) moved into one frame by the pose of
    each scan in that frame ([n, 3] x, y, yaw), thinned to the voxel grid."""
    parts = []
    for p, (x, y, yaw) in zip(points, rel_poses, strict=True):
        if len(p) == 0:
            continue
        c, s = math.cos(yaw), math.sin(yaw)
        q = np.asarray(p, dtype=np.float64)
        parts.append(np.stack([c * q[:, 0] - s * q[:, 1] + x, s * q[:, 0] + c * q[:, 1] + y], 1))
    if not parts:
        return np.zeros((0, 2))
    return voxel_thin(np.vstack(parts))


# ---------------------------------------------------------------------------------------------
# Driven path and containment
# ---------------------------------------------------------------------------------------------


def driven_path(chain: Chain, i: int, hi: int) -> NDArray[np.float64] | None:
    """The rear axle positions over the next PATH_LENGTH_M of travel in the frame of scan i,
    resampled to PATH_POINTS points PATH_SPACING_M of path length apart; None when the car did
    not travel that far within scans i..hi or the chain broke."""
    if hi <= i or not bool(chain.ok[i:hi].all()):
        return None
    rel = relative_poses(chain, i, i, hi)[:, :2]
    s = chain.travel_m[i : hi + 1] - chain.travel_m[i]
    if s[-1] < PATH_LENGTH_M:
        return None
    target = np.linspace(0.0, PATH_LENGTH_M, PATH_POINTS)
    return np.stack([np.interp(target, s, rel[:, 0]), np.interp(target, s, rel[:, 1])], axis=1)


def path_containment(
    track: tr.LocalTrackData, path_xy: NDArray[np.floating], body_half_width_m: float
) -> float:
    """Share of the path points that lie inside the track's corridor shrunk by the half body
    width. NaN with fewer than two valid stations or no path points. A point beyond the valid
    stations (more than PATH_END_EXTENSION_M past either end) is outside."""
    n = track.valid_count
    path = np.asarray(path_xy, dtype=np.float64).reshape(-1, 2)
    if n < 2 or len(path) == 0:
        return math.nan
    cx = track.x_m[:n].astype(np.float64)
    cy = track.y_m[:n].astype(np.float64)
    hl = track.half_width_left_m[:n].astype(np.float64)
    hr = track.half_width_right_m[:n].astype(np.float64)
    ex, ey = cx[1:] - cx[:-1], cy[1:] - cy[:-1]
    seg_len = np.maximum(np.hypot(ex, ey), 1e-9)
    rx = path[:, 0][:, None] - cx[:-1][None, :]
    ry = path[:, 1][:, None] - cy[:-1][None, :]
    t = (rx * ex[None, :] + ry * ey[None, :]) / seg_len[None, :] ** 2
    lateral = (ex[None, :] * ry - ey[None, :] * rx) / seg_len[None, :]
    ext = PATH_END_EXTENSION_M / seg_len
    t_lo = np.zeros(n - 1)
    t_hi = np.ones(n - 1)
    t_lo[0] = -ext[0]
    t_hi[-1] = 1.0 + ext[-1]
    on = (t >= t_lo[None, :]) & (t <= t_hi[None, :])
    pick = np.argmin(np.where(on, np.abs(lateral), np.inf), axis=1)
    rows = np.arange(len(path))
    found = on[rows, pick]
    tt = np.clip(t[rows, pick], 0.0, 1.0)
    lat = lateral[rows, pick]
    left = hl[:-1][pick] + tt * (hl[1:][pick] - hl[:-1][pick])
    right = hr[:-1][pick] + tt * (hr[1:][pick] - hr[:-1][pick])
    inside = found & (lat <= left - body_half_width_m) & (-lat <= right - body_half_width_m)
    return float(inside.mean())


# ---------------------------------------------------------------------------------------------
# Labels
# ---------------------------------------------------------------------------------------------


def contract_ok(track: tr.LocalTrackData) -> bool:
    """Station 0 is the centreline point nearest the car and the travel direction is within 90
    degrees of the heading (+x)."""
    n = track.valid_count
    if n < 2:
        return False
    x = track.x_m[:n].astype(np.float64)
    y = track.y_m[:n].astype(np.float64)
    d = np.hypot(x, y)
    if d[0] > d.min() + STATION0_TOL_M:
        return False
    return bool(x[min(3, n - 1)] - x[0] > 0.0)


def label_array(track: tr.LocalTrackData) -> NDArray[np.float32]:
    """The track as [16, 5] in the net's output layout (x, y, half widths, log sigma)."""
    out = np.zeros((tr.K_STATIONS, 5), dtype=np.float32)
    out[:, 0] = track.x_m
    out[:, 1] = track.y_m
    out[:, 2] = track.half_width_left_m
    out[:, 3] = track.half_width_right_m
    out[:, 4] = np.log(np.maximum(track.sigma_m.astype(np.float64), 1e-3))
    return out


@dataclass
class LabelOutcome:
    """The label of one scan or the reason it was dropped, with the numbers that were computed."""

    reason: str | None
    track: tr.LocalTrackData | None = None
    submap: NDArray[np.float64] | None = None
    path: NDArray[np.float64] | None = None
    intrusion: float = math.nan
    deficit_p90_m: float = math.nan
    containment: float = math.nan


def label_scan(
    chain: Chain,
    points: list[NDArray[np.floating]],
    i: int,
    live: LiveVehicle,
    cfg: geometric.GeometricConfig | None = None,
) -> LabelOutcome:
    """Run steps 2 to 4 for scan i. `points[j]` are the base_link points of scan j."""
    lo, hi = window_bounds(chain, i)
    reason = window_status(chain, i, lo, hi)
    if reason is not None:
        return LabelOutcome(reason)
    path = driven_path(chain, i, hi)
    if path is None:
        return LabelOutcome("path_short")
    sub = build_submap(points[lo : hi + 1], relative_poses(chain, i, lo, hi))
    track = geometric.trace(sub, live.body.half_width_m, SIGMA_MAX_M, cfg)
    out = LabelOutcome(None, track, sub, path)
    if track.valid_count < MIN_VALID_COUNT:
        out.reason = "valid_count"
        return out
    if not contract_ok(track):
        out.reason = "contract"
        return out
    out.intrusion = selfcheck.intrusion_fraction(track, sub)
    if not out.intrusion <= MAX_INTRUSION:  # NaN fails
        out.reason = "intrusion"
        return out
    deficit = clearance_deficit(track, sub)
    out.deficit_p90_m = float(np.percentile(deficit, 90)) if len(deficit) else math.nan
    if not out.deficit_p90_m <= MAX_DEFICIT_P90_M:
        out.reason = "deficit"
        return out
    out.containment = path_containment(track, path, live.body.half_width_m)
    if not out.containment >= MIN_PATH_CONTAINMENT:
        out.reason = "path_outside"
    return out


@dataclass
class SelfLabels:
    """The kept labels of one bag (arrays are aligned) and the drop counts."""

    index: NDArray[np.int64]
    stamp_s: NDArray[np.float64]
    tensor: NDArray[np.float32]  # [m, 1, 2, 360]
    label: NDArray[np.float32]  # [m, 16, 5]
    valid_count: NDArray[np.int64]
    intrusion: NDArray[np.float64]
    deficit_p90_m: NDArray[np.float64]
    path_containment: NDArray[np.float64]
    path_xy: NDArray[np.float64]  # [m, PATH_POINTS, 2]
    candidates: int
    dropped: dict[str, int]
    # Set with a split: per label the bag time of the scan and of the first and last scan of its
    # submap window, and the segment name and role the window lies in. Per segment the counts.
    bag_t_s: NDArray[np.float64] | None = None
    window_first_s: NDArray[np.float64] | None = None
    window_last_s: NDArray[np.float64] | None = None
    segment: NDArray[np.str_] | None = None
    role: NDArray[np.str_] | None = None
    per_segment: dict[str, dict[str, Any]] = field(default_factory=dict)

    def __len__(self) -> int:
        return len(self.index)

    def save(self, path: Path) -> None:
        path.parent.mkdir(parents=True, exist_ok=True)
        extra: dict[str, Any] = {}
        if self.segment is not None:
            extra = {
                "bag_t_s": self.bag_t_s,
                "window_first_s": self.window_first_s,
                "window_last_s": self.window_last_s,
                "segment": self.segment,
                "role": self.role,
            }
        np.savez_compressed(
            path,
            **extra,
            index=self.index,
            stamp_s=self.stamp_s,
            tensor=self.tensor,
            label=self.label,
            valid_count=self.valid_count,
            intrusion=self.intrusion,
            deficit_p90_m=self.deficit_p90_m,
            path_containment=self.path_containment,
            path_xy=self.path_xy,
        )


def scan_points_list(scans: ScanSet, live: LiveVehicle, want: NDArray[np.bool_]) -> list[Any]:
    """Body-filtered base_link points per scan (empty for scans nobody needs)."""
    empty = np.zeros((0, 2), dtype=np.float32)
    return [
        preprocess.scan_to_points(
            scans.ranges[i],
            scans.angle_min,
            scans.angle_increment,
            scans.range_min,
            scans.range_max,
            live.mount,
            live.body,
        )
        if want[i]
        else empty
        for i in range(len(scans))
    ]


@dataclass(frozen=True)
class Segmenter:
    """The split as seen from one bag: which segment a scan, or a submap window, lies in."""

    split: Split
    bag: str
    bag_t_s: NDArray[np.float64]

    def scan_segment(self, i: int) -> Segment | None:
        return self.split.locate(self.bag, float(self.bag_t_s[i]), float(self.bag_t_s[i]))

    def window_segment(self, lo: int, hi: int) -> Segment | None:
        return self.split.locate(self.bag, float(self.bag_t_s[lo]), float(self.bag_t_s[hi]))


def make_labels(
    scans: ScanSet,
    series: MotionSeries,
    live: LiveVehicle,
    candidates: NDArray[np.bool_],
    cfg: geometric.GeometricConfig | None = None,
    segmenter: Segmenter | None = None,
) -> SelfLabels:
    """Self-labels for every candidate scan (the moving scans). With a `segmenter`, a window not
    inside one segment is dropped as `segment_edge` before anything else is computed."""
    chain = chain_motion(series)
    n = len(scans)
    need = np.zeros(n, dtype=bool)
    for i in np.flatnonzero(candidates):
        lo, hi = window_bounds(chain, int(i))
        need[lo : hi + 1] = True
    points = scan_points_list(scans, live, need)
    kept: dict[str, list[Any]] = {
        k: []
        for k in ("index", "tensor", "label", "valid", "intrusion", "deficit", "contain", "path")
    }
    dropped: Counter[str] = Counter()
    seg_meta: list[tuple[float, float, float, str, str]] = []
    per_segment: dict[str, dict[str, Any]] = {}

    def seg_stats(seg: Segment) -> dict[str, Any]:
        return per_segment.setdefault(
            seg.name,
            {"role": seg.role, "moving_scans": 0, "kept": 0, "dropped": dict.fromkeys(REASONS, 0)},
        )

    for i in np.flatnonzero(candidates):
        own = segmenter.scan_segment(int(i)) if segmenter is not None else None
        if own is not None:
            seg_stats(own)["moving_scans"] += 1
        window = None
        if segmenter is not None:
            lo, hi = window_bounds(chain, int(i))
            window = segmenter.window_segment(lo, hi)
            if window is None:
                dropped["segment_edge"] += 1
                if own is not None:
                    seg_stats(own)["dropped"]["segment_edge"] += 1
                continue
        out = label_scan(chain, points, int(i), live, cfg)
        if out.reason is not None:
            dropped[out.reason] += 1
            if own is not None:
                seg_stats(own)["dropped"][out.reason] += 1
            continue
        if segmenter is not None:
            assert window is not None
            seg_stats(window)["kept"] += 1
            seg_meta.append(
                (
                    float(segmenter.bag_t_s[i]),
                    float(segmenter.bag_t_s[lo]),
                    float(segmenter.bag_t_s[hi]),
                    window.name,
                    window.role,
                )
            )
        assert out.track is not None and out.path is not None
        kept["index"].append(int(i))
        kept["tensor"].append(
            preprocess.scan_to_tensor(
                scans.ranges[i],
                scans.angle_min,
                scans.angle_increment,
                scans.range_min,
                scans.range_max,
                live.mount,
                live.body,
            )
        )
        kept["label"].append(label_array(out.track))
        kept["valid"].append(out.track.valid_count)
        kept["intrusion"].append(out.intrusion)
        kept["deficit"].append(out.deficit_p90_m)
        kept["contain"].append(out.containment)
        kept["path"].append(out.path)
    m = len(kept["index"])
    index = np.array(kept["index"], dtype=np.int64)
    return SelfLabels(
        index=index,
        stamp_s=scans.stamp_s[index] if m else np.zeros(0),
        tensor=np.stack(kept["tensor"]) if m else np.zeros((0, 1, 2, preprocess.N_BINS), "f4"),
        label=np.stack(kept["label"]) if m else np.zeros((0, tr.K_STATIONS, 5), "f4"),
        valid_count=np.array(kept["valid"], dtype=np.int64),
        intrusion=np.array(kept["intrusion"], dtype=np.float64),
        deficit_p90_m=np.array(kept["deficit"], dtype=np.float64),
        path_containment=np.array(kept["contain"], dtype=np.float64),
        path_xy=np.stack(kept["path"]) if m else np.zeros((0, PATH_POINTS, 2)),
        candidates=int(candidates.sum()),
        dropped={r: int(dropped.get(r, 0)) for r in REASONS},
        bag_t_s=np.array([r[0] for r in seg_meta], dtype=np.float64) if segmenter else None,
        window_first_s=np.array([r[1] for r in seg_meta], dtype=np.float64) if segmenter else None,
        window_last_s=np.array([r[2] for r in seg_meta], dtype=np.float64) if segmenter else None,
        segment=np.array([r[3] for r in seg_meta], dtype=np.str_) if segmenter else None,
        role=np.array([r[4] for r in seg_meta], dtype=np.str_) if segmenter else None,
        per_segment=per_segment,
    )


# ---------------------------------------------------------------------------------------------
# Bags
# ---------------------------------------------------------------------------------------------

CURRENT_UNITS_FROM = "2026-11-12"


def bag_in_current_units(name: str) -> bool:
    """Bags recorded from 2026-11-12 are in the units of vehicle_params 0.14.0, the earlier ones
    in those of 0.13.0 (speeds 1.401 times the true speed). Only the moving test depends on it."""
    return name >= CURRENT_UNITS_FROM


# The moving test of a bag with no /odom/wheel (pre-registration v0.3 section 3.2, constants file
# `perception.moving_speed_min_mps`): a scan counts as moving when the ICP ground speed is at least
# this. Implemented once, here in the bag reader: such a bag's speed samples ARE the ICP ground
# speed at each scan stamp, so every consumer (`moving_scans`, the gp1 speed bins) reads it
# through the same arrays it reads wheel speed from.
ICP_MOVING_MPS = 0.2
SPEED_SOURCE_ODOM = "odom"
SPEED_SOURCE_ICP = "icp"


def icp_ground_speed(scans: ScanSet, series: MotionSeries) -> NDArray[np.float64]:
    """Ground speed at every scan from the ICP motion of the interval that ends at it: the
    distance the rear axle moved between the previous scan and this one over their stamp
    difference. Scan 0 and a scan whose interval has no ICP result (NaN) read 0.0 m/s."""
    n = len(scans)
    out = np.zeros(n)
    if n < 2:
        return out
    dt = np.diff(scans.stamp_s)
    with np.errstate(invalid="ignore", divide="ignore"):
        v = np.hypot(series.dx_m, series.dy_m) / dt
    out[1:] = np.where(np.isfinite(v) & (dt > 0.0), v, 0.0)
    return out


def load_scanset(
    bag_dir: Path,
    cache_dir: Path | None = None,
    live: LiveVehicle | None = None,
    workers: int = 1,
) -> tuple[ScanSet, Any, Any]:
    """The scans of a bag, and the speed samples (stamp, true speed) used to say when the car is
    moving: /odom/wheel, or for a bag without odometry the ICP ground speed at each scan
    (`icp_ground_speed`, ICP over every scan interval, cached next to the scans). The bag is read
    once and cached as an npz under `cache_dir`; `speed_source` in the cache says which."""
    cache = cache_dir / f"{bag_dir.name}.scans.npz" if cache_dir is not None else None
    if cache is not None and cache.is_file():
        with np.load(cache) as z:
            scans = ScanSet(
                z["stamp_s"],
                z["ranges"],
                float(z["angle_min"]),
                float(z["angle_increment"]),
                float(z["range_min"]),
                float(z["range_max"]),
                z["log_s"] if "log_s" in z.files else None,
            )
            return scans, z["speed_t"], z["speed_mps"]
    _tools()
    from follower_replay.bag import read_bag
    from safety_replay.calibration import RECORDED_0_13_0, convert_bag, load_current

    data = read_bag(bag_dir)
    if not bag_in_current_units(bag_dir.name):
        data = convert_bag(data, RECORDED_0_13_0, load_current())
    first = data.scans[0]
    scans = ScanSet(
        np.array([s.stamp_s for s in data.scans], dtype=np.float64),
        np.array([s.ranges for s in data.scans], dtype=np.float32),
        float(first.angle_min),
        float(first.angle_increment),
        float(first.range_min),
        float(first.range_max),
        np.array([s.log_time_s for s in data.scans], dtype=np.float64) - float(first.log_time_s),
    )
    if data.odom:
        speed_t = np.array([o.stamp_s for o in data.odom])
        speed = np.array([o.speed_mps for o in data.odom])
        source = SPEED_SOURCE_ODOM
    else:
        live = live or load_live_vehicle()
        motion_path = cache_dir / f"{bag_dir.name}.motion.npz" if cache_dir is not None else None
        if motion_path is not None and motion_path.is_file():
            series = extend_motion(
                load_motion(motion_path), scans, live, np.ones(len(scans) - 1, bool), workers
            )
        else:
            log.info("%s: no /odom/wheel, ICP over all %d intervals", bag_dir.name, len(scans) - 1)
            series = compute_motion(scans, live, None, workers)
        if motion_path is not None:
            save_motion(motion_path, series)
        speed_t = scans.stamp_s.copy()
        speed = icp_ground_speed(scans, series)
        source = SPEED_SOURCE_ICP
    if cache is not None:
        cache.parent.mkdir(parents=True, exist_ok=True)
        np.savez(
            cache,
            stamp_s=scans.stamp_s,
            ranges=scans.ranges,
            angle_min=scans.angle_min,
            angle_increment=scans.angle_increment,
            range_min=scans.range_min,
            range_max=scans.range_max,
            speed_t=speed_t,
            speed_mps=speed,
            speed_source=source,
            log_s=scans.log_s,
            log_t0_s=float(first.log_time_s),
        )
    return scans, speed_t, speed


def speed_source(cache_dir: Path, bag_name: str) -> str:
    """Where the moving test of a cached bag reads its speed from: `odom` (/odom/wheel; a cache
    written before this field existed is odom or /drive, never ICP) or `icp`."""
    cache = cache_dir / f"{bag_name}.scans.npz"
    with np.load(cache) as z:
        return str(z["speed_source"]) if "speed_source" in z.files else SPEED_SOURCE_ODOM


def moving_scans(
    scans: ScanSet, speed_t: NDArray[np.float64], speed: NDArray[np.float64], threshold_mps: float
) -> NDArray[np.bool_]:
    from racer_pnp_perception.eval_bags import moving_mask

    return moving_mask(scans.stamp_s, speed_t, speed, threshold_mps)


def intervals_to_match(candidates: NDArray[np.bool_]) -> NDArray[np.bool_]:
    """The scan intervals inside the window of any candidate scan (by scan count, the widest
    window): everything else needs no ICP."""
    n = len(candidates)
    want = np.zeros(max(n - 1, 0), dtype=bool)
    for i in np.flatnonzero(candidates):
        want[max(0, int(i) - MAX_BEHIND_SCANS) : min(n - 1, int(i) + MAX_AHEAD_SCANS)] = True
    return want


def git_sha() -> str:
    try:
        return subprocess.run(
            ["git", "rev-parse", "HEAD"],
            cwd=find_repo_root(),
            capture_output=True,
            text=True,
            check=True,
        ).stdout.strip()
    except (subprocess.SubprocessError, OSError, FileNotFoundError):
        return "unknown"


def label_bag(
    bag_dir: Path,
    out_dir: Path,
    moving_mps: float,
    workers: int,
    live: LiveVehicle,
    split: Split | None = None,
) -> dict[str, Any]:
    scans, speed_t, speed = load_scanset(bag_dir, out_dir / "cache", live, workers)
    moving = moving_scans(scans, speed_t, speed, moving_mps)
    segmenter = None
    if split is not None:
        segmenter = Segmenter(split, bag_dir.name, scans.bag_time_s())
        inside = np.array([segmenter.scan_segment(i) is not None for i in range(len(scans))], bool)
        moving = moving & inside
    motion_path = out_dir / "cache" / f"{bag_dir.name}.motion.npz"
    want = intervals_to_match(moving)
    if motion_path.is_file():
        series = extend_motion(load_motion(motion_path), scans, live, want, workers)
    else:
        log.info("%s: ICP over %d of %d intervals", bag_dir.name, int(want.sum()), len(want))
        series = compute_motion(scans, live, want, workers)
    save_motion(motion_path, series)
    labels = make_labels(scans, series, live, moving, segmenter=segmenter)
    labels.save(out_dir / f"{bag_dir.name}.npz")
    log.info("%s: kept %d of %d moving scans", bag_dir.name, len(labels), labels.candidates)
    entry: dict[str, Any] = {
        "scans": len(scans),
        "moving_scans": labels.candidates,
        "moving_threshold_mps": moving_mps,
        "kept": len(labels),
        "kept_share": len(labels) / labels.candidates if labels.candidates else math.nan,
        "dropped": labels.dropped,
        "icp_intervals": int(want.sum()),
        "icp_ok": int(series.ok[want].sum()),
    }
    if split is not None:
        seg_scans = {
            name: int(
                sum(
                    1
                    for i in range(len(scans))
                    if (g := segmenter.scan_segment(i)) is not None and g.name == name
                )
            )
            for name in labels.per_segment
        }
        entry["segments"] = {
            name: {
                "role": v["role"],
                "scans": seg_scans[name],
                **{k: v[k] for k in ("moving_scans", "kept", "dropped")},
            }
            for name, v in labels.per_segment.items()
        }
        entry["split_sha256"] = split.sha256
    return entry


def main(argv: list[str] | None = None) -> int:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--bags", nargs="+", type=Path, required=True)
    parser.add_argument("--out", type=Path, default=DEFAULT_OUT)
    parser.add_argument("--moving-mps", type=float, default=0.5)
    parser.add_argument(
        "--workers", type=int, default=max(1, (multiprocessing.cpu_count() or 2) - 2)
    )
    parser.add_argument(
        "--split", type=Path, default=None, help="pre-registered split YAML (see split.py)"
    )
    args = parser.parse_args(argv)
    logging.basicConfig(level=logging.INFO, format="%(message)s")
    live = load_live_vehicle()
    split = load_split(args.split) if args.split is not None else None
    manifest: dict[str, Any] = {"git_sha": git_sha(), "bags": {}}
    for bag in args.bags:
        if split is not None and split.bags.get(bag.name) is None:
            log.info("%s: not in the split, skipped", bag.name)
            continue
        if split is not None and bag.name in split.bags:
            b = split.bags[bag.name]
            if b.whole_bag_role == "unused":
                log.info("%s: unused in the split, skipped", bag.name)
                continue
        manifest["bags"][bag.name] = label_bag(
            bag, args.out, args.moving_mps, args.workers, live, split
        )
    args.out.mkdir(parents=True, exist_ok=True)
    manifest_path = args.out / "manifest.json"
    if manifest_path.is_file():
        previous = json.loads(manifest_path.read_text())
        previous["bags"].update(manifest["bags"])
        previous["git_sha"] = manifest["git_sha"]
        manifest = previous
    manifest_path.write_text(json.dumps(manifest, indent=2))
    print(json.dumps(manifest, indent=2))
    return 0


if __name__ == "__main__":
    sys.exit(main())
