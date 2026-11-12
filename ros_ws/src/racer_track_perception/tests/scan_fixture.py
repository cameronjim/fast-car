"""A synthetic scan of a straight 1 m lane, shared by the core and node tests."""

from __future__ import annotations

import math

import numpy as np
from racer_track_perception.vehicle import LiveVehicle

SCAN = {
    "angle_min": -math.pi,
    "angle_increment": 2 * math.pi / 720,
    "range_min": 0.05,
    "range_max": 12.0,
}


def corridor_ranges(live: LiveVehicle) -> np.ndarray:
    """Ranges from the head, in the laser frame (mount yaw applied), of walls at y = +-0.5."""
    n = 720
    bearings = SCAN["angle_min"] + np.arange(n) * SCAN["angle_increment"]
    vb = bearings + live.mount.yaw_rad
    r = np.full(n, np.inf)
    s, c = np.sin(vb), np.cos(vb)
    for wall_y in (0.5, -0.5):
        with np.errstate(divide="ignore", invalid="ignore"):
            t = (wall_y - live.mount.y_m) / s
        ok = t > 0
        r[ok] = np.minimum(r[ok], t[ok])
    r[np.abs(c) < 1e-9] = np.inf
    r[r > 8.0] = np.inf
    return r
