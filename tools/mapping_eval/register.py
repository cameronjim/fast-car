"""Scan-to-map registration: where the car really was, from a saved map instead of from the
previous scan.

Frame-to-frame ICP between scans reads forward travel low in a featureless corridor (the 2026-11-12
note measured 0.73 of speed * dt), so it cannot calibrate wheel speed. Registering each scan
against a finished map has no such drift: the map's ends and corners pin the pose along the lane.
The headings and positions it gives are what yaw_evidence.py compares the wheel odometry with.

Numpy only. Poses are base_link (the rear axle) in the map frame; the laser sits at the vehicle's
sensors.lidar mount.
"""

from __future__ import annotations

import math
from dataclasses import dataclass

import numpy as np
from raceline.map_track import DistanceField, MapGrid

TRIM_M = 0.25  # scan points further than this from any wall are outliers (people, glitches)
HUBER_M = 0.05


@dataclass(frozen=True)
class Registration:
    x: float
    y: float
    yaw: float
    rms_m: float  # over the inlier points
    inlier_fraction: float


def scan_points_base(
    ranges: np.ndarray,
    angle_min: float,
    angle_inc: float,
    mount: tuple[float, float, float],
    range_min: float = 0.15,
    range_max: float = 12.0,
) -> np.ndarray:
    """Scan endpoints (N, 2) in the base frame."""
    theta = angle_min + angle_inc * np.arange(len(ranges))
    ok = np.isfinite(ranges) & (ranges > range_min) & (ranges < range_max - 0.05)
    mx, my, myaw = mount
    r = ranges[ok]
    return np.column_stack([mx + r * np.cos(theta[ok] + myaw), my + r * np.sin(theta[ok] + myaw)])


def _gradient(field: DistanceField, x: np.ndarray, y: np.ndarray, eps: float):
    d = field.at(x, y)
    gx = (field.at(x + eps, y) - field.at(x - eps, y)) / (2 * eps)
    gy = (field.at(x, y + eps) - field.at(x, y - eps)) / (2 * eps)
    return d, gx, gy


def _transform(points: np.ndarray, pose: tuple[float, float, float]):
    c, s = math.cos(pose[2]), math.sin(pose[2])
    x = c * points[:, 0] - s * points[:, 1] + pose[0]
    y = s * points[:, 0] + c * points[:, 1] + pose[1]
    return x, y


def score(field: DistanceField, points: np.ndarray, pose: tuple[float, float, float]) -> float:
    """Mean trimmed distance to the nearest wall (lower is better)."""
    x, y = _transform(points, pose)
    d = np.minimum(field.at(x, y), TRIM_M)
    return float(d.mean())


def refine(
    field: DistanceField,
    points: np.ndarray,
    pose: tuple[float, float, float],
    iterations: int = 15,
) -> Registration:
    """Gauss-Newton on (x, y, yaw) with trimmed, Huber-weighted distance residuals."""
    x0, y0, th = pose
    eps = 0.5 * field.grid.resolution_m
    for _ in range(iterations):
        px, py = _transform(points, (x0, y0, th))
        d, gx, gy = _gradient(field, px, py, eps)
        inlier = d < TRIM_M
        if inlier.sum() < 20:
            break
        # d(point)/d(yaw) = R'(yaw) p = (-(py - y0), px - x0)
        jth = gx * (-(py - y0)) + gy * (px - x0)
        jac = np.column_stack([gx, gy, jth])[inlier]
        res = d[inlier]
        w = np.where(res < HUBER_M, 1.0, HUBER_M / np.maximum(res, 1e-9))
        a = jac.T @ (jac * w[:, None]) + 1e-6 * np.eye(3)
        b = jac.T @ (res * w)
        step = -np.linalg.solve(a, b)
        # The distance field is the UNSIGNED distance, so the gradient points away from the
        # nearest wall cell and a Gauss-Newton step is a descent step on d, which is what we want.
        step = np.clip(step, [-0.1, -0.1, -0.1], [0.1, 0.1, 0.1])
        x0, y0, th = x0 + step[0], y0 + step[1], th + step[2]
        if np.abs(step).max() < 1e-4:
            break
    px, py = _transform(points, (x0, y0, th))
    d = field.at(px, py)
    inlier = d < TRIM_M
    rms = float(np.sqrt((d[inlier] ** 2).mean())) if inlier.any() else math.nan
    return Registration(x0, y0, th, rms, float(inlier.mean()))


def register_scan(
    field: DistanceField,
    points: np.ndarray,
    prediction: tuple[float, float, float],
    search_xy_m: float = 0.06,
    search_yaw_rad: float = 0.06,
) -> Registration:
    """Best of a 3 x 3 x 3 neighbourhood of the predicted pose, refined. The neighbourhood guards
    against the Gauss-Newton step settling on the wrong side of a thin wall."""
    best = None
    best_score = math.inf
    for dx in (-search_xy_m, 0.0, search_xy_m):
        for dy in (-search_xy_m, 0.0, search_xy_m):
            for dth in (-search_yaw_rad, 0.0, search_yaw_rad):
                cand = (prediction[0] + dx, prediction[1] + dy, prediction[2] + dth)
                sc = score(field, points, cand)
                if sc < best_score:
                    best, best_score = cand, sc
    return refine(field, points, best)


def track_scans(
    grid: MapGrid,
    scans: list[np.ndarray],
    odom_poses: np.ndarray,
    start_pose: tuple[float, float, float] = (0.0, 0.0, 0.0),
) -> list[Registration]:
    """Register consecutive scans, each seeded with the previous registration moved by the odometry
    increment. odom_poses (N, 3) are the dead-reckoned (x, y, yaw) at the scans; only their
    increments are used, so a wrong yaw SCALE in the odometry costs a worse seed, never a bias in
    the answer (the registration decides)."""
    field = DistanceField(grid)
    out: list[Registration] = []
    prev = start_pose
    for i, pts in enumerate(scans):
        if i == 0:
            pred = prev
        else:
            # Increment in the previous odom frame, applied in the previous registered frame.
            ox, oy, oth = odom_poses[i - 1]
            nx, ny, nth = odom_poses[i]
            c, s = math.cos(oth), math.sin(oth)
            dx = c * (nx - ox) + s * (ny - oy)
            dy = -s * (nx - ox) + c * (ny - oy)
            dth = nth - oth
            cp, sp = math.cos(prev[2]), math.sin(prev[2])
            pred = (prev[0] + cp * dx - sp * dy, prev[1] + sp * dx + cp * dy, prev[2] + dth)
        reg = register_scan(field, pts, pred)
        out.append(reg)
        prev = (reg.x, reg.y, reg.yaw)
    return out
