"""What the car actually did around a brake, measured from the LiDAR alone.

No odometry was recorded before 2026-11 (and none exists in the simulator), so the car's motion
comes from 2D scan matching: point-to-point ICP between two scans of the same static scene gives
the rigid motion of the car between them. Over the 0.1 to 1.0 s around a brake the scene is the
foam lane, which is static, and the motion is a few tens of centimetres, so a plain ICP from the
identity converges. Brute-force nearest neighbours (numpy, no scipy): a scan has a few hundred
returns within the range used.

Frames: rear axle at the origin, x forward, y left (REP-103), the LiDAR at (mount_x, mount_y) with
mount yaw `yaw` (pi on the car), as in vehicle_params.
"""

from __future__ import annotations

from dataclasses import dataclass

import numpy as np
from follower_replay.bag import ScanRecord


@dataclass(frozen=True)
class Mount:
    x_m: float
    y_m: float
    yaw_rad: float


@dataclass(frozen=True)
class Body:
    """The car's footprint about the rear axle, no margin: what touches the wall."""

    front_x_m: float  # wheelbase + front overhang
    rear_x_m: float  # rear overhang (behind the axle, positive)
    half_width_m: float


def scan_points(
    scan: ScanRecord, mount: Mount, min_range_m: float = 0.0, max_range_m: float = float("inf")
) -> np.ndarray:
    """Usable returns of a scan as an (N, 2) array in the rear-axle frame."""
    r = np.asarray(scan.ranges, dtype=float)
    bearing = scan.angle_min + np.arange(len(r)) * scan.angle_increment + mount.yaw_rad
    ok = np.isfinite(r) & (r > 0.0)
    if np.isfinite(scan.range_min):
        ok &= r >= scan.range_min
    if np.isfinite(scan.range_max):
        ok &= r <= scan.range_max
    ok &= (r >= min_range_m) & (r <= max_range_m)
    r = r[ok]
    b = bearing[ok]
    return np.stack([r * np.cos(b) + mount.x_m, r * np.sin(b) + mount.y_m], axis=1)


def box_clearance(points: np.ndarray, body: Body) -> float:
    """Smallest distance from any point to the body rectangle (0 for a point on or inside it);
    +inf with no points."""
    if len(points) == 0:
        return float("inf")
    x = points[:, 0]
    y = points[:, 1]
    dx = np.maximum(np.maximum(-body.rear_x_m - x, x - body.front_x_m), 0.0)
    dy = np.maximum(np.abs(y) - body.half_width_m, 0.0)
    return float(np.hypot(dx, dy).min())


def _nearest(points: np.ndarray, reference: np.ndarray) -> tuple[np.ndarray, np.ndarray]:
    d2 = ((points[:, None, :] - reference[None, :, :]) ** 2).sum(axis=2)
    j = d2.argmin(axis=1)
    return np.sqrt(d2[np.arange(len(points)), j]), j


def icp(
    source: np.ndarray, target: np.ndarray, iterations: int = 40, inlier_m: float = 0.05
) -> tuple[np.ndarray, np.ndarray, float]:
    """Rigid (R, t) with R @ source + t ~ target. Returns R, t and the RMS distance of the
    matched points (capped at inlier_m each). Raises ValueError with too few points."""
    if len(source) < 10 or len(target) < 10:
        raise ValueError("ICP needs at least 10 points in each scan")
    rotation = np.eye(2)
    translation = np.zeros(2)
    for k in range(iterations):
        moved = source @ rotation.T + translation
        d, j = _nearest(moved, target)
        # Loose gating first (the start can be several centimetres off), then the inlier band.
        threshold = max(inlier_m, float(np.percentile(d, 80))) if k < 10 else inlier_m
        keep = d < threshold
        if keep.sum() < 10:
            break
        a = moved[keep]
        b = target[j[keep]]
        ca = a.mean(axis=0)
        cb = b.mean(axis=0)
        u, _, vt = np.linalg.svd((a - ca).T @ (b - cb))
        step = vt.T @ u.T
        if np.linalg.det(step) < 0:
            vt[1] *= -1
            step = vt.T @ u.T
        shift = cb - step @ ca
        rotation = step @ rotation
        translation = step @ translation + shift
        if float(np.hypot(*shift)) < 1e-6:
            break
    d, _ = _nearest(source @ rotation.T + translation, target)
    return rotation, translation, float(np.sqrt(np.mean(np.minimum(d, inlier_m) ** 2)))


def car_motion(
    earlier: ScanRecord, later: ScanRecord, mount: Mount, max_range_m: float = 4.0
) -> tuple[float, float, float, float]:
    """Where the rear axle is at `later`, in the car frame at `earlier`: (x, y, heading change,
    ICP rms). Returns are taken from 0.12 m (clear of the car's own parts) to max_range_m."""
    a = scan_points(earlier, mount, 0.12, max_range_m)
    b = scan_points(later, mount, 0.12, max_range_m)
    rotation, translation, rms = icp(b, a)
    return (
        float(translation[0]),
        float(translation[1]),
        float(np.arctan2(rotation[1, 0], rotation[0, 0])),
        rms,
    )
