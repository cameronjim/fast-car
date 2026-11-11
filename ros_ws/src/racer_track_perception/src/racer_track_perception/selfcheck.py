"""Label-free self-checks of a predicted track against the scan that produced it.

* `boundary_residual_m`: over valid stations within 2.0 m of the car, the median distance from
  each predicted boundary point (the centreline point plus or minus the half width along the
  centreline normal) to the nearest scan point.
* `intrusion_fraction`: of the scan points within 2.5 m of the car and ahead of it (x > 0), the
  share lying inside the predicted corridor (between the two boundaries over the valid stations)
  by more than 0.05 m.

Both are NaN when they cannot be computed (too few valid stations, no scan points).
"""

from __future__ import annotations

import math

import numpy as np
from numpy.typing import NDArray

from racer_track_perception.track import LocalTrackData

STATION_RANGE_M = 2.0
POINT_RANGE_M = 2.5
INSIDE_MARGIN_M = 0.05


def _tangents(x: NDArray[np.float64], y: NDArray[np.float64]) -> NDArray[np.float64]:
    """Unit tangents along the polyline of the valid stations (needs at least two)."""
    if len(x) == 2:
        d = np.array([[x[1] - x[0], y[1] - y[0]]] * 2)
    else:
        d = np.stack([np.gradient(x), np.gradient(y)], axis=1)
    norm = np.hypot(d[:, 0], d[:, 1])
    return d / np.maximum(norm, 1e-9)[:, None]


def boundary_residual_m(track: LocalTrackData, points: NDArray[np.floating]) -> float:
    n = track.valid_count
    if n < 2 or len(points) == 0:
        return math.nan
    x = track.x_m[:n].astype(np.float64)
    y = track.y_m[:n].astype(np.float64)
    tan = _tangents(x, y)
    near = np.hypot(x, y) <= STATION_RANGE_M
    if not near.any():
        return math.nan
    nx, ny = -tan[:, 1], tan[:, 0]
    hl = track.half_width_left_m[:n].astype(np.float64)
    hr = track.half_width_right_m[:n].astype(np.float64)
    bx = np.concatenate([(x + hl * nx)[near], (x - hr * nx)[near]])
    by = np.concatenate([(y + hl * ny)[near], (y - hr * ny)[near]])
    px = points[:, 0].astype(np.float64)
    py = points[:, 1].astype(np.float64)
    d2 = (bx[:, None] - px[None, :]) ** 2 + (by[:, None] - py[None, :]) ** 2
    return float(np.median(np.sqrt(d2.min(axis=1))))


def intrusion_fraction(track: LocalTrackData, points: NDArray[np.floating]) -> float:
    n = track.valid_count
    if n < 2 or len(points) == 0:
        return math.nan
    p = points.astype(np.float64)
    near = (p[:, 0] > 0.0) & (np.hypot(p[:, 0], p[:, 1]) <= POINT_RANGE_M)
    p = p[near]
    if len(p) == 0:
        return math.nan
    cx = track.x_m[:n].astype(np.float64)
    cy = track.y_m[:n].astype(np.float64)
    hl = track.half_width_left_m[:n].astype(np.float64)
    hr = track.half_width_right_m[:n].astype(np.float64)
    ex, ey = cx[1:] - cx[:-1], cy[1:] - cy[:-1]
    seg_len2 = np.maximum(ex * ex + ey * ey, 1e-12)
    rx = p[:, 0][:, None] - cx[:-1][None, :]
    ry = p[:, 1][:, None] - cy[:-1][None, :]
    t = (rx * ex[None, :] + ry * ey[None, :]) / seg_len2[None, :]
    lateral = (ex[None, :] * ry - ey[None, :] * rx) / np.sqrt(seg_len2)[None, :]
    on = (t >= 0.0) & (t <= 1.0)
    pick = np.argmin(np.where(on, np.abs(lateral), np.inf), axis=1)
    rows = np.arange(len(p))
    ok = on[rows, pick]
    tt = t[rows, pick]
    lat = lateral[rows, pick]
    left = hl[:-1][pick] + tt * (hl[1:][pick] - hl[:-1][pick])
    right = hr[:-1][pick] + tt * (hr[1:][pick] - hr[:-1][pick])
    depth = np.minimum(left - lat, lat + right)
    inside = ok & (depth > INSIDE_MARGIN_M)
    return float(inside.sum()) / float(len(p))


def apply_self_check(track: LocalTrackData, points: NDArray[np.floating]) -> LocalTrackData:
    """Fill the two self-check fields of `track` in place and return it."""
    track.boundary_residual_m = boundary_residual_m(track, points)
    track.intrusion_fraction = intrusion_fraction(track, points)
    return track
