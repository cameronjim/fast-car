"""An independent sweep generator for the deskew tests: ray casting per beam from the pose the
car has at THAT beam's time. It shares nothing with racer_track_perception.deskew (no arc helper,
no mount transform, no rebin): the car pose comes from a closed form of its own and each beam is
cast against wall segments. Used by test_deskew.py and test_deskew_relay.py.

World frame: the rear axle pose at the time of the sweep's first beam (time 0) is the origin with
x forward. The car drives a constant arc: speed v, yaw rate w (the closed form for w = 0 is a
straight line).
"""

from __future__ import annotations

import math

import numpy as np

Segment = tuple[float, float, float, float]

# A 3.0 x 2.0 m room with a box in it: ranges stay under about 2.2 m, so one 0.5 degree bin is at
# most 2 cm across and the nearest-return grid cannot by itself exceed a 2 cm tolerance.
ROOM: tuple[Segment, ...] = (
    (-1.2, -1.0, 1.8, -1.0),
    (1.8, -1.0, 1.8, 1.0),
    (1.8, 1.0, -1.2, 1.0),
    (-1.2, 1.0, -1.2, -1.0),
    # a box 0.3 x 0.3 m
    (0.9, -0.15, 1.2, -0.15),
    (1.2, -0.15, 1.2, 0.15),
    (1.2, 0.15, 0.9, 0.15),
    (0.9, 0.15, 0.9, -0.15),
)


def pose_at(v: float, w: float, t: float) -> tuple[float, float, float]:
    """(x, y, yaw) of the rear axle after t seconds on the arc, in the world frame."""
    if abs(w) < 1e-12:
        return v * t, 0.0, 0.0
    return v / w * math.sin(w * t), v / w * (1.0 - math.cos(w * t)), w * t


def ray_hit(ox: float, oy: float, angle: float, walls: tuple[Segment, ...]) -> float:
    """Distance along the ray to the nearest wall, inf when none."""
    dx, dy = math.cos(angle), math.sin(angle)
    best = math.inf
    for x1, y1, x2, y2 in walls:
        ex, ey = x2 - x1, y2 - y1
        denom = dx * ey - dy * ex
        if abs(denom) < 1e-12:
            continue
        sx, sy = x1 - ox, y1 - oy
        t = (sx * ey - sy * ex) / denom
        u = (sx * dy - sy * dx) / denom
        if t > 1e-9 and -1e-9 <= u <= 1.0 + 1e-9:
            best = min(best, t)
    return best


def lidar_origin(base: tuple[float, float, float], mount) -> tuple[float, float, float]:
    """(x, y, yaw) of the LiDAR for a base pose and a mount (x, y, yaw)."""
    bx, by, byaw = base
    c, s = math.cos(byaw), math.sin(byaw)
    return (
        bx + c * mount.x_m - s * mount.y_m,
        by + s * mount.x_m + c * mount.y_m,
        byaw + mount.yaw_rad,
    )


def sweep(
    mount,
    v: float,
    w: float,
    n: int = 720,
    angle_min: float = -math.pi,
    angle_increment: float = 2.0 * math.pi / 719.0,
    time_increment: float = 1.0e-4 * 10 / 7.2,  # about 0.1 s over 720 beams
    time_order: str = "index_ascending",
    walls: tuple[Segment, ...] = ROOM,
    start: tuple[float, float, float] = (0.0, 0.0, 0.0),
    t_start: float = 0.0,
) -> np.ndarray:
    """The ranges of one sweep. The beam seen first in time is index 0 (`index_ascending`) or the
    last index (`index_descending`). `start` is the rear axle pose in the room at t_start (the
    first beam's time); the car then drives the arc."""
    ranges = np.full(n, np.inf)
    for i in range(n):
        k = i if time_order == "index_ascending" else n - 1 - i
        t = k * time_increment  # seconds after the first beam
        x0, y0, th0 = start
        px, py, pth = pose_at(v, w, t)
        c, s = math.cos(th0), math.sin(th0)
        base = (x0 + c * px - s * py, y0 + s * px + c * py, th0 + pth)
        ox, oy, oyaw = lidar_origin(base, mount)
        a = angle_min + i * angle_increment
        ranges[i] = ray_hit(ox, oy, oyaw + a, walls)
    return ranges


def reference_truth_distance(
    ranges: np.ndarray,
    mount,
    v: float,
    w: float,
    t_ref: float,
    angle_min: float,
    angle_increment: float,
    walls: tuple[Segment, ...] = ROOM,
    start: tuple[float, float, float] = (0.0, 0.0, 0.0),
    t_start: float = 0.0,
) -> np.ndarray:
    """Distance of each finite output point to the nearest true wall, the point being the beam's
    range along its own bearing from the LiDAR at the reference pose (time t_ref, same clock as
    sweep()'s t_start, the time of the first beam). Walls are taken as they are in the room, so a point is judged against
    where the LiDAR REALLY was at t_ref."""
    x0, y0, th0 = start
    px, py, pth = pose_at(v, w, t_ref - t_start)
    c, s = math.cos(th0), math.sin(th0)
    base = (x0 + c * px - s * py, y0 + s * px + c * py, th0 + pth)
    ox, oy, oyaw = lidar_origin(base, mount)
    out = []
    for i, r in enumerate(ranges):
        if not math.isfinite(r):
            continue
        a = oyaw + angle_min + i * angle_increment
        qx, qy = ox + r * math.cos(a), oy + r * math.sin(a)
        out.append(min(_point_segment(qx, qy, seg) for seg in walls))
    return np.array(out)


def _point_segment(px: float, py: float, seg: Segment) -> float:
    x1, y1, x2, y2 = seg
    ex, ey = x2 - x1, y2 - y1
    u = max(0.0, min(1.0, ((px - x1) * ex + (py - y1) * ey) / (ex * ex + ey * ey)))
    return math.hypot(px - (x1 + u * ex), py - (y1 + u * ey))
