"""Run the real perception code over a segment's scans, and check it against the recording.

The perceiver is `racer_track_perception.core.TrackPerceiver`, the object track_perception_node
calls for every scan, built the way training/racer_pnp_perception/.../eval_bags.py builds it
(`load_live_vehicle` for the vehicle, `load_bundle` + `CircConv1dV0` for the net). Nothing of the
algorithm is copied here.

The self-test (`check_fidelity`): replaying the geometric tracer over the recorded scans must give
the recorded /pnp/local_track, matched by header stamp, to within 1e-5 m on every station. That
holds only when the tracer is the one that recorded the bag: the 2026-11-12 bag was recorded with
geometric-v0 (difference 0.0 there), and the tracer on main has moved on to v1, so against that
bag the check now reports the v0 to v1 difference. Replay such a bag with the old tracer by
pointing PNP_REPLAY_PERCEPTION_SRC at its racer_track_perception/src.
"""

from __future__ import annotations

import logging
import os
import sys
from dataclasses import dataclass, field
from pathlib import Path
from typing import Any

import numpy as np
from numpy.typing import NDArray

from pnp_replay import metrics
from pnp_replay.bag import Recording, scan_ranges

log = logging.getLogger(__name__)

REPO_ROOT = Path(__file__).resolve().parents[2]
# PNP_REPLAY_PERCEPTION_SRC points at another checkout's racer_track_perception/src, to replay a
# bag with the tracer that recorded it (the --check-fidelity self-test needs the same tracer).
PERCEPTION_SRC = Path(
    os.environ.get("PNP_REPLAY_PERCEPTION_SRC")
    or REPO_ROOT / "ros_ws" / "src" / "racer_track_perception" / "src"
)
FIDELITY_TOLERANCE_M = 1e-5
STAMP_MATCH_S = 1e-3


def _import_core() -> Any:
    """racer_track_perception, imported from the repo by path (it is a ROS package, not
    installed in the tools environment)."""
    if str(PERCEPTION_SRC) not in sys.path:
        sys.path.insert(0, str(PERCEPTION_SRC))
    from racer_track_perception import core

    return core


@dataclass
class Perceiver:
    """A TrackPerceiver with the vehicle it was built for and what it is called in reports."""

    impl: Any
    vehicle: Any
    backend: str
    model_id: str


def build_perceiver(backend: str, bundle_dir: Path | None = None) -> Perceiver:
    """The perceiver for --backend geometric|net. The net needs the track_perception bundle
    (refused, as on the car, on any mismatch with the live vehicle_params)."""
    core = _import_core()
    from racer_track_perception.vehicle import load_live_vehicle

    live = load_live_vehicle(REPO_ROOT)
    if backend == core.BACKEND_GEOMETRIC:
        impl = core.TrackPerceiver(core.BACKEND_GEOMETRIC, live)
    elif backend == core.BACKEND_NET:
        if bundle_dir is None:
            raise ValueError("--backend net needs --perception-bundle")
        from racer_track_perception.bundle import load_bundle
        from racer_track_perception.runtime import CircConv1dV0

        bundle = load_bundle(bundle_dir, "track_perception", live)
        impl = core.TrackPerceiver(
            core.BACKEND_NET,
            live,
            net=CircConv1dV0.from_npz(bundle.payload_path("weights.npz")),
            model_id=bundle.model_id,
        )
    else:
        raise ValueError(f"backend must be geometric or net, got {backend!r}")
    return Perceiver(impl=impl, vehicle=live, backend=backend, model_id=impl.model_id)


@dataclass
class Tracks:
    """Replayed tracks for the scans `scan_index` of a recording, as arrays of 16 stations."""

    scan_index: NDArray[np.int64]
    x: NDArray[np.float32]
    y: NDArray[np.float32]
    hl: NDArray[np.float32]
    hr: NDArray[np.float32]
    sigma: NDArray[np.float32]
    valid: NDArray[np.int64]
    residual: NDArray[np.float64]
    intrusion: NDArray[np.float64]
    points: list[NDArray[np.float32]] = field(default_factory=list)

    def __len__(self) -> int:
        return len(self.scan_index)

    def subset(self, mask: NDArray[np.bool_]) -> Tracks:
        keep = np.nonzero(mask)[0]
        return Tracks(
            scan_index=self.scan_index[keep],
            x=self.x[keep],
            y=self.y[keep],
            hl=self.hl[keep],
            hr=self.hr[keep],
            sigma=self.sigma[keep],
            valid=self.valid[keep],
            residual=self.residual[keep],
            intrusion=self.intrusion[keep],
            points=[self.points[i] for i in keep],
        )


def run_perceiver(perceiver: Perceiver, rec: Recording, scan_index: NDArray[np.int64]) -> Tracks:
    """Process the scans `scan_index` of the recording, as the node processes each scan. Also
    keeps each scan's body filtered points (for the clearance metric)."""
    from racer_track_perception import preprocess

    xs, ys, hls, hrs, sgs, vs, rs, its, pts = [], [], [], [], [], [], [], [], []
    mount, body = perceiver.vehicle.mount, perceiver.vehicle.body
    for i in scan_index:
        args = (
            scan_ranges(rec, int(i)),
            float(rec["scan_angle_min"][i]),
            float(rec["scan_angle_increment"][i]),
            float(rec["scan_range_min"][i]),
            float(rec["scan_range_max"][i]),
        )
        track = perceiver.impl.process(*args)
        xs.append(track.x_m)
        ys.append(track.y_m)
        hls.append(track.half_width_left_m)
        hrs.append(track.half_width_right_m)
        sgs.append(track.sigma_m)
        vs.append(track.valid_count)
        rs.append(track.boundary_residual_m)
        its.append(track.intrusion_fraction)
        pts.append(preprocess.scan_to_points(*args, mount, body))
    n = len(scan_index)
    stack = lambda a: np.stack(a).astype(np.float32) if n else np.zeros((0, 16), np.float32)
    return Tracks(
        scan_index=np.asarray(scan_index, dtype=np.int64),
        x=stack(xs),
        y=stack(ys),
        hl=stack(hls),
        hr=stack(hrs),
        sigma=stack(sgs),
        valid=np.array(vs, dtype=np.int64),
        residual=np.array(rs, dtype=np.float64),
        intrusion=np.array(its, dtype=np.float64),
        points=pts,
    )


@dataclass
class Fidelity:
    matched: int
    unmatched_recorded: int
    max_centreline_diff_m: float
    max_width_diff_m: float
    valid_count_mismatches: int
    ok: bool

    def as_dict(self) -> dict[str, Any]:
        return {
            "matched": self.matched,
            "unmatched_recorded": self.unmatched_recorded,
            "max_centreline_diff_m": self.max_centreline_diff_m,
            "max_half_width_diff_m": self.max_width_diff_m,
            "valid_count_mismatches": self.valid_count_mismatches,
            "tolerance_m": FIDELITY_TOLERANCE_M,
            "ok": self.ok,
        }


def check_fidelity(
    rec: Recording,
    tracks: Tracks,
    t_from: float,
    t_to: float,
    tolerance_m: float = FIDELITY_TOLERANCE_M,
) -> Fidelity:
    """Compare replayed tracks with the recorded /pnp/local_track whose header stamp lies in
    [t_from, t_to], matching each recorded track to the replayed one with the same scan header
    stamp. A recorded track with no replayed scan of that stamp is counted, not compared. The
    check fails when nothing matched, or a valid_count differs, or a station differs by more than
    the tolerance."""
    stamps = rec["scan_stamp"][tracks.scan_index]
    order = np.argsort(stamps)
    sorted_stamps = stamps[order]
    in_range = (rec["trk_stamp"] >= t_from) & (rec["trk_stamp"] <= t_to)
    max_c, max_w, mism, matched, missing = 0.0, 0.0, 0, 0, 0
    for k in np.nonzero(in_range)[0]:
        stamp = rec["trk_stamp"][k]
        j = int(np.searchsorted(sorted_stamps, stamp))
        cands = [c for c in (j - 1, j) if 0 <= c < len(sorted_stamps)]
        best = min(cands, key=lambda c: abs(sorted_stamps[c] - stamp)) if cands else None
        if best is None or abs(sorted_stamps[best] - stamp) > STAMP_MATCH_S:
            missing += 1
            continue
        r = int(order[best])
        matched += 1
        n = int(min(rec["trk_valid"][k], tracks.valid[r]))
        if int(rec["trk_valid"][k]) != int(tracks.valid[r]):
            mism += 1
        if n:
            dx = rec["trk_x"][k, :n].astype(np.float64) - tracks.x[r, :n]
            dy = rec["trk_y"][k, :n].astype(np.float64) - tracks.y[r, :n]
            max_c = max(max_c, float(np.hypot(dx, dy).max()))
            dl = np.abs(rec["trk_hl"][k, :n].astype(np.float64) - tracks.hl[r, :n]).max()
            dr = np.abs(rec["trk_hr"][k, :n].astype(np.float64) - tracks.hr[r, :n]).max()
            max_w = max(max_w, float(dl), float(dr))
    ok = matched > 0 and mism == 0 and max_c <= tolerance_m and max_w <= tolerance_m
    return Fidelity(matched, missing, max_c, max_w, mism, bool(ok))


def perception_summary(
    rec: Recording,
    tracks: Tracks,
    moving_threshold_mps: float = metrics.MOVING_SPEED_MPS,
    stride: int = 1,
) -> tuple[dict[str, Any], dict[str, NDArray[Any]]]:
    """Perception metrics over the replayed scans whose wheel speed (interpolated at the scan's
    arrival) is above the threshold, every `stride`-th of those. Returns the metrics and the
    per-scan arrays they came from (so segments can be pooled)."""
    t = rec["scan_t"][tracks.scan_index]
    speed = metrics.speed_at(t, rec["odom_t"], rec["odom_v"])
    moving = np.nonzero(metrics.moving_mask(speed, moving_threshold_mps))[0][::stride]
    deficits = [
        metrics.clearance_deficits(
            tracks.x[i],
            tracks.y[i],
            tracks.hl[i],
            tracks.hr[i],
            int(tracks.valid[i]),
            tracks.points[i],
        )
        for i in moving
    ]
    arrays = {
        "valid": tracks.valid[moving],
        "intrusion": tracks.intrusion[moving],
        "residual": tracks.residual[moving],
        "deficits": np.concatenate(deficits) if deficits else np.zeros(0),
    }
    out = metrics.perception_metrics(
        arrays["valid"], arrays["intrusion"], arrays["residual"], arrays["deficits"]
    )
    out["moving_threshold_mps"] = moving_threshold_mps
    out["scan_stride"] = stride
    return out, arrays


def pool(parts: list[dict[str, NDArray[Any]]]) -> dict[str, Any]:
    """Perception metrics over the concatenated per-scan arrays of several segments."""
    cat = {k: np.concatenate([p[k] for p in parts]) for k in parts[0]} if parts else None
    if cat is None:
        return {}
    return metrics.perception_metrics(
        cat["valid"], cat["intrusion"], cat["residual"], cat["deficits"]
    )
