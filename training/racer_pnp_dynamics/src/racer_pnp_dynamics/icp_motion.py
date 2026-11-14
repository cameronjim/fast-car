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

# Where the matcher starts. "identity" is T1, the v0.2 truth (it under-reads yaw rate at speed,
# docs/notes/yaw-rate-scan-matching-2026-11-13.md). "wheel" is T2, the v0.3 truth: the matcher
# starts from the wheel motion over the interval (wheel_seed.py). Same matcher, same flags.
SEED_IDENTITY = "identity"
SEED_WHEEL = "wheel"
SEEDS = (SEED_IDENTITY, SEED_WHEEL)


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
    seeded: np.ndarray | None = None  # bool: the match started from a wheel seed (T2 only)

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
    init: tuple[float, float] | None = None,
) -> tuple[float, float, float, float, float]:
    """(dx, dy, dyaw, rms, inlier share) of the rear axle at `later` in the frame at `earlier`.
    `init` is an optional (heading change, forward distance) to start from instead of the
    identity (the wheel seed); the result is the whole motion, not a correction. Raises
    ValueError when a scan has too few usable returns."""
    from yaw_rate.matching import init_motion  # tools/ is on sys.path through .bags

    a = motion.scan_points(earlier, mount, 0.12, MAX_RANGE_M, ignore)
    b = motion.scan_points(later, mount, 0.12, MAX_RANGE_M, ignore)
    r0, t0 = (np.eye(2), np.zeros(2)) if init is None else init_motion(init[0], init[1])
    rotation, translation, rms = motion.icp(b @ r0.T + t0, a, inlier_m=INLIER_M)
    rotation, translation = rotation @ r0, rotation @ t0 + translation
    moved = b @ rotation.T + translation
    d, _ = cKDTree(a).query(moved)
    share = float(np.mean(d < INLIER_M))
    dyaw = float(np.arctan2(rotation[1, 0], rotation[0, 0]))
    return float(translation[0]), float(translation[1]), dyaw, rms, share


_WORK: dict[str, object] = {}


def _one(i: int) -> tuple[float, ...]:
    scans: Sequence[ScanRecord] = _WORK["scans"]  # type: ignore[assignment]
    init = _WORK.get("init")
    start = None
    if init is not None and np.isfinite(init[i]).all():  # type: ignore[index]
        start = (float(init[i][0]), float(init[i][1]))  # type: ignore[index]
    try:
        return scan_motion(
            scans[i],
            scans[i + 1],
            _WORK["mount"],
            _WORK["ignore"],  # type: ignore[arg-type]
            start,
        )
    except ValueError:
        return (np.nan,) * 5


def compute_motion(
    scans: Sequence[ScanRecord],
    vehicle: Vehicle,
    want: np.ndarray | None = None,
    workers: int = 1,
    ignore: frozenset[tuple[int, int]] | None = None,
    seed: str = SEED_IDENTITY,
    run: Run | None = None,
) -> MotionSeries:
    """Motion for every interval where `want[i]` is true (all by default), the rest not ok.

    `seed` is "identity" (T1, the matcher starts from no motion) or "wheel" (T2, it starts from
    the wheel motion over each interval, which needs `run` for the wheel speed and the gated
    steering). Everything else, the matcher and the quality flags, is the same."""
    if seed not in SEEDS:
        raise ValueError(f"seed must be one of {SEEDS}, got {seed!r}")
    if seed == SEED_WHEEL and run is None:
        raise ValueError("seed 'wheel' needs the run (wheel speed and /drive steering)")
    n = max(len(scans) - 1, 0)
    want = np.ones(n, dtype=bool) if want is None else np.asarray(want, dtype=bool)
    mount = mount_of(vehicle)
    if ignore is None:
        ignore = motion.static_self_returns(scans, mount, body_of(vehicle))
    todo = [i for i in range(n) if want[i]]
    stamps = np.array([s.stamp_s for s in scans])
    init = None
    if seed == SEED_WHEEL:
        from .wheel_seed import interval_seed  # not at the top: wheel_seed imports windows

        dyaw0, path0 = interval_seed(run, vehicle, stamps[:n], stamps[1 : n + 1])  # type: ignore[arg-type]
        init = np.stack([dyaw0, path0], axis=1)
    _WORK.update(scans=scans, mount=mount, ignore=ignore, init=init)
    if workers > 1:  # pragma: no cover - process pool, exercised by the real run only
        with multiprocessing.get_context("fork").Pool(workers) as pool:
            results = pool.map(_one, todo, chunksize=64)
    else:
        results = [_one(i) for i in todo]
    out = np.full((n, 5), np.nan)
    for i, r in zip(todo, results, strict=True):
        out[i] = r
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
        seeded=None if init is None else (np.isfinite(init).all(axis=1) & want),
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


def motion_cache_path(cache_dir: Path, run_name: str, seed: str = SEED_IDENTITY) -> Path:
    """Cache file of a run's motion. The seed is part of the key, so T1 and T2 never share a file;
    the identity name is the one the v0.2 caches already have."""
    if seed not in SEEDS:
        raise ValueError(f"seed must be one of {SEEDS}, got {seed!r}")
    suffix = "motion" if seed == SEED_IDENTITY else f"motion-{seed}"
    return Path(cache_dir) / f"{run_name}.{suffix}.npz"


def save_motion(path: Path, m: MotionSeries) -> None:
    path.parent.mkdir(parents=True, exist_ok=True)
    arrays = {k: getattr(m, k) for k in m.__dataclass_fields__ if getattr(m, k) is not None}
    np.savez_compressed(path, **arrays)


def load_motion(path: Path) -> MotionSeries:
    with np.load(path) as z:
        return MotionSeries(**{k: z[k] for k in MotionSeries.__dataclass_fields__ if k in z.files})


def compose(dx: np.ndarray, dy: np.ndarray, dyaw: np.ndarray) -> np.ndarray:
    """Chain per interval motions (each in the frame at its start) into poses (x, y, yaw) of the
    last frame in the first frame, one row per prefix, starting with the identity."""
    poses = np.zeros((len(dx) + 1, 3))
    for i in range(len(dx)):
        x, y, yaw = poses[i]
        c, s = np.cos(yaw), np.sin(yaw)
        poses[i + 1] = (x + c * dx[i] - s * dy[i], y + s * dx[i] + c * dy[i], yaw + dyaw[i])
    return poses
