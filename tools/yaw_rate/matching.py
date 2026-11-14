"""Scan-to-scan ICP yaw with the quality flags of the dynamics package.

The matcher is tools/safety_replay/motion.py (point-to-point ICP, brute-force nearest neighbours,
the car's own static returns masked out). The flags are those of
training/racer_pnp_dynamics/icp_motion.py: mean residual at most 0.02 m, inlier share (matched
within 5 cm) at least 0.7, and the stamp gap within 30 ms of the nominal. scipy is not a
dependency of tools, so the inlier share uses numpy nearest neighbours.

Frames: rear axle at the origin, x forward, y left; the LiDAR sits at yaw pi on the car and
`motion.scan_points` applies the mount, so a left turn is a positive yaw change.
"""

from __future__ import annotations

import logging
import math
import multiprocessing
from dataclasses import dataclass

import numpy as np
from follower_replay.bag import ScanRecord
from numpy.typing import NDArray
from safety_replay import motion

from .bagio import Recording, scan_ranges
from .vehicle import Vehicle

log = logging.getLogger(__name__)

SCAN_DT_S = 0.1
DT_TOLERANCE_S = 0.03  # per single step; scaled by the step count for i to i+2
MAX_RANGE_M = 4.0
MIN_RANGE_M = 0.12
INLIER_M = 0.05
RMS_MAX_M = 0.02
INLIER_SHARE_MIN = 0.7

Array = NDArray[np.float64]


@dataclass(frozen=True)
class Steps:
    """Matches of scan i to scan i + lag for every i in `index` (arrays aligned with `index`)."""

    index: NDArray[np.int64]
    lag: int
    t0: Array  # stamp of scan i
    t1: Array  # stamp of scan i + lag
    dyaw: Array  # NaN when the match failed
    rms: Array
    share: Array
    ok: NDArray[np.bool_]  # computed and passed the quality flags


def mount_of(vehicle: Vehicle) -> motion.Mount:
    return motion.Mount(vehicle.lidar_x_m, vehicle.lidar_y_m, vehicle.lidar_yaw_rad)


def body_of(vehicle: Vehicle) -> motion.Body:
    return motion.Body(vehicle.body_front_x_m, vehicle.body_rear_x_m, vehicle.body_half_width_m)


def scan_record(rec: Recording, i: int) -> ScanRecord:
    return ScanRecord(
        stamp_s=float(rec["scan_stamp"][i]),
        angle_min=float(rec["scan_angle_min"][i]),
        angle_increment=float(rec["scan_angle_increment"][i]),
        range_min=float(rec["scan_range_min"][i]),
        range_max=float(rec["scan_range_max"][i]),
        ranges=tuple(float(r) for r in scan_ranges(rec, i)),
    )


def self_returns(rec: Recording, vehicle: Vehicle, sample: int = 400) -> frozenset[tuple[int, int]]:
    """The car's own static returns, from an even sample of the bag's scans."""
    n = len(rec["scan_stamp"])
    picked = np.unique(np.linspace(0, n - 1, min(sample, n)).astype(int))
    scans = [scan_record(rec, int(i)) for i in picked]
    return motion.static_self_returns(scans, mount_of(vehicle), body_of(vehicle), sample=len(scans))


def _nearest_distance(points: Array, reference: Array) -> Array:
    d2 = ((points[:, None, :] - reference[None, :, :]) ** 2).sum(axis=2)
    return np.sqrt(d2.min(axis=1))


def init_motion(dyaw_rad: float, distance_m: float) -> tuple[np.ndarray, np.ndarray]:
    """(R, t) of a rear-axle motion along an arc: `distance_m` forward, heading change
    `dyaw_rad` (the chord leaves at half the heading change)."""
    c, s = math.cos(dyaw_rad), math.sin(dyaw_rad)
    half = 0.5 * dyaw_rad
    return np.array([[c, -s], [s, c]]), distance_m * np.array([math.cos(half), math.sin(half)])


def scan_motion(
    earlier: ScanRecord,
    later: ScanRecord,
    mount: motion.Mount,
    ignore: frozenset[tuple[int, int]] | None = None,
    init: tuple[float, float] | None = None,
) -> tuple[float, float, float, float, float]:
    """(dx, dy, dyaw, rms, inlier share) of the rear axle at `later` in the frame at `earlier`.
    `init` is an optional (heading change, forward distance) to start from instead of the
    identity (the wheel twist's motion over the interval); the result is the whole motion.
    Raises ValueError when a scan has too few usable returns."""
    a = motion.scan_points(earlier, mount, MIN_RANGE_M, MAX_RANGE_M, ignore)
    b = motion.scan_points(later, mount, MIN_RANGE_M, MAX_RANGE_M, ignore)
    r0, t0 = (np.eye(2), np.zeros(2)) if init is None else init_motion(init[0], init[1])
    rotation, translation, rms = motion.icp(b @ r0.T + t0, a, inlier_m=INLIER_M)
    rotation, translation = rotation @ r0, rotation @ t0 + translation
    moved = b @ rotation.T + translation
    share = float(np.mean(_nearest_distance(moved, a) < INLIER_M))
    dyaw = float(np.arctan2(rotation[1, 0], rotation[0, 0]))
    return float(translation[0]), float(translation[1]), dyaw, rms, share


_WORK: dict[str, object] = {}


def _one(args: tuple[int, int, tuple[float, float] | None]) -> tuple[float, float, float]:
    i, lag, init = args
    rec: Recording = _WORK["rec"]  # type: ignore[assignment]
    try:
        _dx, _dy, dyaw, rms, share = scan_motion(
            scan_record(rec, i),
            scan_record(rec, i + lag),
            _WORK["mount"],  # type: ignore[arg-type]
            _WORK["ignore"],  # type: ignore[arg-type]
            init,
        )
    except ValueError:
        return (np.nan, np.nan, np.nan)
    return (dyaw, rms, share)


def match_steps(
    rec: Recording,
    index: NDArray[np.int64],
    lag: int,
    vehicle: Vehicle,
    ignore: frozenset[tuple[int, int]] | None = None,
    workers: int = 1,
    init: Array | None = None,
) -> Steps:
    """Match scan i to scan i + lag for every i in `index`. `init`, shape (len(index), 2), is the
    (heading change, forward distance) each match starts from (identity when None)."""
    index = np.asarray(index, dtype=np.int64)
    mount = mount_of(vehicle)
    if ignore is None:
        ignore = self_returns(rec, vehicle)
    _WORK.update(rec=rec, mount=mount, ignore=ignore)
    jobs = [
        (int(i), lag, None if init is None else (float(init[k, 0]), float(init[k, 1])))
        for k, i in enumerate(index)
    ]
    if workers > 1:  # pragma: no cover - process pool, exercised by the real run only
        with multiprocessing.get_context("fork").Pool(workers) as pool:
            results = pool.map(_one, jobs, chunksize=32)
    else:
        results = [_one(j) for j in jobs]
    out = np.array(results, dtype=float).reshape(len(jobs), 3)
    stamps = rec["scan_stamp"]
    t0 = stamps[index]
    t1 = stamps[index + lag]
    with np.errstate(invalid="ignore"):
        ok = (
            np.isfinite(out).all(axis=1)
            & (np.abs((t1 - t0) - lag * SCAN_DT_S) < DT_TOLERANCE_S * lag)
            & (out[:, 1] <= RMS_MAX_M)
            & (out[:, 2] >= INLIER_SHARE_MIN)
        )
    return Steps(index, lag, t0, t1, out[:, 0], out[:, 1], out[:, 2], ok)
