"""Vectorised ray casting against wall segments, and the LiDAR scan built on it.

`cast_rays` is the primitive (many rays, many segments). `lidar_scan` builds a 720-beam, 360
degree scan the way the real driver reports one: beam i has laser bearing
angle_min + i * angle_increment, the laser frame is rotated by the mount yaw, and the head sits at
the mount offset, so ranges are from the head. Each beam can be cast from the pose the car has at
the beam's own time (motion distortion).
"""

from __future__ import annotations

import math
from dataclasses import dataclass

import numpy as np
from numpy.typing import NDArray
from racer_track_perception.vehicle import LidarMount

NO_RETURN = np.inf


def cast_rays(
    ox: NDArray[np.float64],
    oy: NDArray[np.float64],
    angle: NDArray[np.float64],
    segments: NDArray[np.float64],
    max_range_m: float,
) -> NDArray[np.float64]:
    """Distance from each ray origin along its angle to the nearest segment, inf if none within
    `max_range_m`. ox, oy, angle are [B]; segments is [S, 4] (x0, y0, x1, y1)."""
    ox = np.asarray(ox, dtype=np.float64)
    oy = np.asarray(oy, dtype=np.float64)
    ang = np.asarray(angle, dtype=np.float64)
    out = np.full(ang.shape, NO_RETURN)
    if segments.shape[0] == 0 or ang.size == 0:
        return out
    dx = np.cos(ang)[:, None]
    dy = np.sin(ang)[:, None]
    px, py = segments[:, 0][None, :], segments[:, 1][None, :]
    ex, ey = (segments[:, 2] - segments[:, 0])[None, :], (segments[:, 3] - segments[:, 1])[None, :]
    rx = px - ox[:, None]
    ry = py - oy[:, None]
    denom = dx * ey - dy * ex
    with np.errstate(divide="ignore", invalid="ignore"):
        s = (rx * ey - ry * ex) / denom
        u = (rx * dy - ry * dx) / denom
    hit = (np.abs(denom) > 1e-12) & (s > 0.0) & (u >= 0.0) & (u <= 1.0) & (s <= max_range_m)
    s = np.where(hit, s, NO_RETURN)
    return s.min(axis=1)


def cull_segments(
    segments: NDArray[np.float64], x: float, y: float, radius_m: float
) -> NDArray[np.float64]:
    """The segments that can be within `radius_m` of (x, y) (by midpoint and half length)."""
    mx = 0.5 * (segments[:, 0] + segments[:, 2])
    my = 0.5 * (segments[:, 1] + segments[:, 3])
    half = 0.5 * np.hypot(segments[:, 2] - segments[:, 0], segments[:, 3] - segments[:, 1])
    keep = np.hypot(mx - x, my - y) <= radius_m + half
    return segments[keep]


@dataclass(frozen=True)
class ScanGeometry:
    """The beam layout of a scan."""

    n_beams: int = 720
    angle_min_rad: float = -math.pi
    angle_increment_rad: float = 0.008739
    range_min_m: float = 0.05
    range_max_m: float = 12.0

    def laser_bearings(self) -> NDArray[np.float64]:
        return self.angle_min_rad + np.arange(self.n_beams) * self.angle_increment_rad


@dataclass(frozen=True)
class Motion:
    """Constant body twist over the sweep: forward speed (m/s), yaw rate (rad/s), the sweep
    duration, and the direction beams are taken in (+1: beam 0 first, -1: last beam first)."""

    speed_mps: float = 0.0
    yaw_rate_rps: float = 0.0
    sweep_s: float = 0.1
    sweep_sign: int = 1


def beam_times(n_beams: int, motion: Motion) -> NDArray[np.float64]:
    frac = np.arange(n_beams) / float(n_beams)
    if motion.sweep_sign < 0:
        frac = 1.0 - frac
    return frac * motion.sweep_s


def base_pose_at(
    pose: tuple[float, float, float], motion: Motion, t: NDArray[np.float64]
) -> tuple[NDArray[np.float64], NDArray[np.float64], NDArray[np.float64]]:
    """base_link pose (x, y, yaw) in the world at times `t` after the sweep start, for a
    constant twist (the chord of the arc: exact for a constant yaw rate)."""
    x0, y0, yaw0 = pose
    w = motion.yaw_rate_rps
    half_turn = 0.5 * w * t
    chord = motion.speed_mps * t * np.sinc(half_turn / math.pi)  # sin(wt/2) / (wt/2) * v t
    mid_heading = yaw0 + half_turn
    return x0 + chord * np.cos(mid_heading), y0 + chord * np.sin(mid_heading), yaw0 + w * t


def lidar_scan(
    pose: tuple[float, float, float],
    mount: LidarMount,
    geometry: ScanGeometry,
    segments: NDArray[np.float64],
    motion: Motion | None = None,
) -> NDArray[np.float64]:
    """Ranges [n_beams] (inf = no return, no noise) of a scan taken from base_link `pose`.

    The head is at the mount offset in the vehicle frame and the laser frame is rotated by the
    mount yaw. With `motion`, beam i is cast from the pose at its own time.
    """
    motion = motion or Motion()
    t = beam_times(geometry.n_beams, motion)
    bx, by, byaw = base_pose_at(pose, motion, t)
    c, s = np.cos(byaw), np.sin(byaw)
    hx = bx + c * mount.x_m - s * mount.y_m
    hy = by + s * mount.x_m + c * mount.y_m
    angle = byaw + mount.yaw_rad + geometry.laser_bearings()
    near = cull_segments(segments, pose[0], pose[1], geometry.range_max_m + 1.0)
    return cast_rays(hx, hy, angle, near, geometry.range_max_m)
