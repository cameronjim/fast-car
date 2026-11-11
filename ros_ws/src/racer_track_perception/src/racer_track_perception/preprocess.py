"""LiDAR scan to the net's input tensor and to body-frame points (the contract's input definition).

A beam with laser bearing b = angle_min + i * angle_increment has vehicle bearing
wrap(b + mount_yaw) in [-pi, pi). Bin j covers vehicle bearing
[-pi + j * 2pi/360, -pi + (j + 1) * 2pi/360), so bin 180 starts straight ahead and bins increase
counter-clockwise (to the left). A beam is valid when its range is finite and within the scan's
[range_min, range_max]. Its point in base_link is (mount_x + r cos vb, mount_y + r sin vb); a beam
whose point lies inside the body rectangle grown by BODY_FILTER_MARGIN_M is dropped. Channel 0 of
bin j is min(r, RANGE_CLIP_M) / RANGE_CLIP_M over the valid beams in the bin (1.0 if none), channel
1 is 1.0 if the bin has a valid beam. Ranges are from the LiDAR head, not from base_link.

Nothing here raises on NaN, inf, empty or all-invalid scans.
"""

from __future__ import annotations

import math
from typing import Any

import numpy as np
from numpy.typing import NDArray

from racer_track_perception.vehicle import BodyFootprint, LidarMount

N_BINS = 360
RANGE_CLIP_M = 6.0
BODY_FILTER_MARGIN_M = 0.02
_BIN_WIDTH_RAD = 2.0 * math.pi / N_BINS
# Keeps a beam exactly on a bin edge (straight ahead) in the upper bin despite float rounding.
_EDGE_EPS = 1e-9


def _valid_beams(
    ranges: Any,
    angle_min: float,
    angle_increment: float,
    range_min: float,
    range_max: float,
    mount: LidarMount,
    body: BodyFootprint,
) -> tuple[NDArray[np.float64], NDArray[np.float64], NDArray[np.float64], NDArray[np.float64]]:
    """(r, vehicle bearing, x, y) of the beams that are valid and outside the body."""
    empty = np.zeros(0, dtype=np.float64)
    try:
        r = np.asarray(ranges, dtype=np.float64).reshape(-1)
        a0 = float(angle_min)
        da = float(angle_increment)
        r_lo = float(range_min)
        r_hi = float(range_max)
    except (TypeError, ValueError):
        return empty, empty, empty, empty
    if r.size == 0 or not (math.isfinite(a0) and math.isfinite(da)):
        return empty, empty, empty, empty
    with np.errstate(invalid="ignore"):
        valid = np.isfinite(r) & (r >= r_lo) & (r <= r_hi)
    if not valid.any():
        return empty, empty, empty, empty
    idx = np.nonzero(valid)[0]
    r = r[idx]
    bearing = a0 + idx.astype(np.float64) * da + mount.yaw_rad
    bearing = np.mod(bearing + math.pi, 2.0 * math.pi) - math.pi
    x = mount.x_m + r * np.cos(bearing)
    y = mount.y_m + r * np.sin(bearing)
    m = BODY_FILTER_MARGIN_M
    inside = (
        (x >= body.x_min_m - m) & (x <= body.x_max_m + m) & (np.abs(y) <= body.half_width_m + m)
    )
    keep = ~inside
    return r[keep], bearing[keep], x[keep], y[keep]


def scan_to_tensor(
    ranges: Any,
    angle_min: float,
    angle_increment: float,
    range_min: float,
    range_max: float,
    mount: LidarMount,
    body: BodyFootprint,
) -> NDArray[np.float32]:
    """The net input, float32 [1, 2, 360]: channel 0 min-pooled clipped range, channel 1 validity."""
    r, bearing, _x, _y = _valid_beams(
        ranges, angle_min, angle_increment, range_min, range_max, mount, body
    )
    out = np.zeros((1, 2, N_BINS), dtype=np.float32)
    out[0, 0, :] = 1.0
    if r.size == 0:
        return out
    bins = np.floor((bearing + math.pi) / _BIN_WIDTH_RAD + _EDGE_EPS).astype(np.int64)
    bins = np.clip(bins, 0, N_BINS - 1)
    pooled = np.ones(N_BINS, dtype=np.float64)
    np.minimum.at(pooled, bins, np.minimum(r, RANGE_CLIP_M) / RANGE_CLIP_M)
    out[0, 0, :] = pooled.astype(np.float32)
    out[0, 1, bins] = 1.0
    return out


def scan_to_points(
    ranges: Any,
    angle_min: float,
    angle_increment: float,
    range_min: float,
    range_max: float,
    mount: LidarMount,
    body: BodyFootprint,
) -> NDArray[np.float32]:
    """Valid, body-filtered beams as float32 [N, 2] points (x, y) in base_link."""
    _r, _b, x, y = _valid_beams(
        ranges, angle_min, angle_increment, range_min, range_max, mount, body
    )
    return np.stack([x, y], axis=1).astype(np.float32)
