"""Planar motion of the car per scan interval from scan-to-scan ICP.

Reuses tools/safety_replay/motion.py (`scan_points`, `icp`, `static_self_returns`). For each pair
of consecutive scans the motion of the rear axle (dx, dy, dyaw) is expressed in `base_link` at the
earlier scan. A quality flag keeps matches with a small mean residual and a high inlier share.
This gives yaw rate and ground speed at 10 Hz, the only yaw supervision there is (no IMU).
"""

from __future__ import annotations

import logging
import multiprocessing
from collections.abc import Sequence
from dataclasses import dataclass
from pathlib import Path

import numpy as np
from scipy.spatial import cKDTree

from .bags import Run, ScanRecord, motion
from .vehicle import Vehicle

LOG = logging.getLogger(__name__)

SCAN_DT_S = 0.1
DT_TOLERANCE_S = 0.03
MAX_RANGE_M = 4.0
INLIER_M = 0.05
RMS_MAX_M = 0.02  # a mean residual above this is a bad match (the 5 cm band caps at 0.05)
INLIER_SHARE_MIN = 0.7


@dataclass(frozen=True)
class MotionSeries:
    """One entry per scan interval i (scan i to scan i + 1)."""

    t_s: np.ndarray  # stamp of scan i
    dt_s: np.ndarray
    dx_m: np.ndarray
    dy_m: np.ndarray
    dyaw_rad: np.ndarray
    rms_m: np.ndarray
    inlier_share: np.ndarray
    ok: np.ndarray  # bool: computed and passed the quality gate

    def ground_speed_mps(self) -> np.ndarray:
        return np.hypot(self.dx_m, self.dy_m) / self.dt_s

    def yaw_rate_rad_per_s(self) -> np.ndarray:
        return self.dyaw_rad / self.dt_s


def mount_of(vehicle: Vehicle) -> motion.Mount:
    return motion.Mount(vehicle.lidar_x_m, vehicle.lidar_y_m, vehicle.lidar_yaw_rad)


def body_of(vehicle: Vehicle) -> motion.Body:
    return motion.Body(vehicle.body_front_x_m, vehicle.body_rear_x_m, vehicle.body_half_width_m)


def scan_motion(
    earlier: ScanRecord,
    later: ScanRecord,
    mount: motion.Mount,
    ignore: frozenset[tuple[int, int]] | None = None,
) -> tuple[float, float, float, float, float]:
    """(dx, dy, dyaw, rms, inlier share) of the rear axle at `later` in the frame at `earlier`.
    Raises ValueError when a scan has too few usable returns."""
    a = motion.scan_points(earlier, mount, 0.12, MAX_RANGE_M, ignore)
    b = motion.scan_points(later, mount, 0.12, MAX_RANGE_M, ignore)
    rotation, translation, rms = motion.icp(b, a, inlier_m=INLIER_M)
    moved = b @ rotation.T + translation
    d, _ = cKDTree(a).query(moved)
    share = float(np.mean(d < INLIER_M))
    dyaw = float(np.arctan2(rotation[1, 0], rotation[0, 0]))
    return float(translation[0]), float(translation[1]), dyaw, rms, share


_WORK: dict[str, object] = {}


def _one(i: int) -> tuple[float, ...]:
    scans: Sequence[ScanRecord] = _WORK["scans"]  # type: ignore[assignment]
    try:
        return scan_motion(
            scans[i],
            scans[i + 1],
            _WORK["mount"],
            _WORK["ignore"],  # type: ignore[arg-type]
        )
    except ValueError:
        return (np.nan,) * 5


def compute_motion(
    scans: Sequence[ScanRecord],
    vehicle: Vehicle,
    want: np.ndarray | None = None,
    workers: int = 1,
    ignore: frozenset[tuple[int, int]] | None = None,
) -> MotionSeries:
    """Motion for every interval where `want[i]` is true (all by default), the rest not ok."""
    n = max(len(scans) - 1, 0)
    want = np.ones(n, dtype=bool) if want is None else np.asarray(want, dtype=bool)
    mount = mount_of(vehicle)
    if ignore is None:
        ignore = motion.static_self_returns(scans, mount, body_of(vehicle))
    todo = [i for i in range(n) if want[i]]
    _WORK.update(scans=scans, mount=mount, ignore=ignore)
    if workers > 1:  # pragma: no cover - process pool, exercised by the real run only
        with multiprocessing.get_context("fork").Pool(workers) as pool:
            results = pool.map(_one, todo, chunksize=64)
    else:
        results = [_one(i) for i in todo]
    out = np.full((n, 5), np.nan)
    for i, r in zip(todo, results, strict=True):
        out[i] = r
    stamps = np.array([s.stamp_s for s in scans])
    dt = np.diff(stamps) if n else np.zeros(0)
    with np.errstate(invalid="ignore"):
        ok = (
            np.isfinite(out).all(axis=1)
            & (np.abs(dt - SCAN_DT_S) < DT_TOLERANCE_S)
            & (out[:, 3] <= RMS_MAX_M)
            & (out[:, 4] >= INLIER_SHARE_MIN)
        )
    return MotionSeries(
        t_s=stamps[:n],
        dt_s=dt,
        dx_m=out[:, 0],
        dy_m=out[:, 1],
        dyaw_rad=out[:, 2],
        rms_m=out[:, 3],
        inlier_share=out[:, 4],
        ok=ok,
    )


def driving_intervals(run: Run, threshold_mps: float = 0.05) -> np.ndarray:
    """Which scan intervals of a run are worth matching: the car is commanded or measured moving."""
    t = np.array([s.stamp_s for s in run.scans])
    if len(t) < 2:
        return np.zeros(0, dtype=bool)
    idx = np.clip(np.searchsorted(run.t_s, t[:-1]), 0, len(run.t_s) - 1)
    moving = np.nan_to_num(run.speed_cmd_mps[idx], nan=0.0) > threshold_mps
    moving |= np.nan_to_num(run.wheel_speed_mps[idx], nan=0.0) > threshold_mps
    return moving


def save_motion(path: Path, m: MotionSeries) -> None:
    path.parent.mkdir(parents=True, exist_ok=True)
    np.savez_compressed(path, **{k: getattr(m, k) for k in m.__dataclass_fields__})


def load_motion(path: Path) -> MotionSeries:
    with np.load(path) as z:
        return MotionSeries(**{k: z[k] for k in MotionSeries.__dataclass_fields__})


def compose(dx: np.ndarray, dy: np.ndarray, dyaw: np.ndarray) -> np.ndarray:
    """Chain per interval motions (each in the frame at its start) into poses (x, y, yaw) of the
    last frame in the first frame, one row per prefix, starting with the identity."""
    poses = np.zeros((len(dx) + 1, 3))
    for i in range(len(dx)):
        x, y, yaw = poses[i]
        c, s = np.cos(yaw), np.sin(yaw)
        poses[i + 1] = (x + c * dx[i] - s * dy[i], y + s * dx[i] + c * dy[i], yaw + dyaw[i])
    return poses
