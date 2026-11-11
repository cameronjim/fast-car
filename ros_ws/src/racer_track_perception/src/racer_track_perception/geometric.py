"""The geometric corridor tracer: a classical, deterministic track estimate from scan points.

Input: scan points in base_link (body filtered, `preprocess.scan_to_points`). Output: the same 16
stations as the net, as a `LocalTrackData`.

1. Station 0: in a strip |x| < STRIP_HALF_M across the car, the nearest point on the left (y > 0,
   up to FEELER_MAX_M) and on the right. Both found: the centre is the midpoint and the half
   widths are half the gap. One found: the missing side takes the last known half width (or
   `default_half_width_m`). None: no track.
2. March: step STEP_M along the current heading, recentre the same way in the frame of that step
   (strip along the heading, feelers along its normal). The new heading is the direction from the
   previous centre to the new one, its change limited to MAX_TURN_RAD per step.
3. Stop when both feelers find nothing, or when the centre would come within the body half width
   of a scan point.
4. sigma is SIGMA_TWO_WALLS_M where both walls were seen, SIGMA_ONE_WALL_M where one was.
5. Resample so stations are exactly STATION_SPACING_M apart along the traced polyline.

Refinement over the plain procedure (measured in eval_bags and eval_sim, see the README of the
released model): a first pass from heading 0 estimates the lane direction at the car, and a second
pass starts from that heading so station 0 sits on the normal through the car (the label's
definition: the centreline point nearest the car) and its width is measured across the lane, not
across the car's own heading.
"""

from __future__ import annotations

import math
from dataclasses import dataclass

import numpy as np
from numpy.typing import NDArray

from racer_track_perception import track as tr

STEP_M = 0.20
STRIP_HALF_M = 0.15
FEELER_MAX_M = 2.0
MAX_TURN_RAD = 0.5
SIGMA_TWO_WALLS_M = 0.05
SIGMA_ONE_WALL_M = 0.15
# Path length to march: every step is at least STEP_M long, so 15 steps give the 3.0 m the 16
# stations need.
N_STEPS = tr.K_STATIONS - 1
# Points further than this from the car cannot take part in a 3 m march with a 2 m feeler.
MAX_POINT_RANGE_M = 6.0


@dataclass(frozen=True)
class GeometricConfig:
    default_half_width_m: float = 0.5
    two_pass: bool = True
    heading_blend: float = 1.0  # 1.0: the plain direction of the last step; lower smooths it


@dataclass
class _Raw:
    x: list[float]
    y: list[float]
    hl: list[float]
    hr: list[float]
    sigma: list[float]


def _march(
    px: NDArray[np.float64],
    py: NDArray[np.float64],
    heading0: float,
    cfg: GeometricConfig,
    body_half_width_m: float,
) -> _Raw:
    raw = _Raw([], [], [], [], [])
    cx = cy = 0.0
    heading = heading0
    last_hw = cfg.default_half_width_m
    r2_body = body_half_width_m * body_half_width_m
    for step in range(N_STEPS + 1):
        if step > 0:
            cx += STEP_M * math.cos(heading)
            cy += STEP_M * math.sin(heading)
        c, s = math.cos(heading), math.sin(heading)
        rx, ry = px - cx, py - cy
        along = rx * c + ry * s
        lat = -rx * s + ry * c
        strip = np.abs(along) <= STRIP_HALF_M
        left = strip & (lat > 0.0) & (lat <= FEELER_MAX_M)
        right = strip & (lat < 0.0) & (lat >= -FEELER_MAX_M)
        has_left, has_right = bool(left.any()), bool(right.any())
        if not (has_left or has_right):
            break
        dl = float(lat[left].min()) if has_left else 0.0
        dr = float(-lat[right].max()) if has_right else 0.0
        if has_left and has_right:
            shift = 0.5 * (dl - dr)
            hw = 0.5 * (dl + dr)
            last_hw = hw
            sigma = SIGMA_TWO_WALLS_M
        elif has_left:
            shift = dl - last_hw
            hw = last_hw
            sigma = SIGMA_ONE_WALL_M
        else:
            shift = last_hw - dr
            hw = last_hw
            sigma = SIGMA_ONE_WALL_M
        nx, ny = -s, c
        new_x, new_y = cx + shift * nx, cy + shift * ny
        if float(np.min((px - new_x) ** 2 + (py - new_y) ** 2)) < r2_body:
            break
        if raw.x:
            measured = math.atan2(new_y - raw.y[-1], new_x - raw.x[-1])
            turn = (measured - heading + math.pi) % (2.0 * math.pi) - math.pi
            turn *= cfg.heading_blend
            turn = max(-MAX_TURN_RAD, min(MAX_TURN_RAD, turn))
            heading += turn
        cx, cy = new_x, new_y
        raw.x.append(cx)
        raw.y.append(cy)
        raw.hl.append(hw)
        raw.hr.append(hw)
        raw.sigma.append(sigma)
    return raw


def _resample(raw: _Raw) -> tuple[NDArray[np.float64], int]:
    """Stations exactly STATION_SPACING_M apart along the raw polyline: array [K, 5] of x, y, hl,
    hr, sigma (rows past the traced length are not meaningful) and the number of valid rows."""
    out = np.zeros((tr.K_STATIONS, 5), dtype=np.float64)
    n_raw = len(raw.x)
    if n_raw == 0:
        return out, 0
    pts = np.array([raw.x, raw.y, raw.hl, raw.hr, raw.sigma], dtype=np.float64).T
    seg = np.hypot(np.diff(pts[:, 0]), np.diff(pts[:, 1]))
    s = np.concatenate([[0.0], np.cumsum(seg)])
    total = float(s[-1])
    n_valid = min(math.floor(total / tr.STATION_SPACING_M + 1e-9) + 1, tr.K_STATIONS)
    targets = np.arange(n_valid) * tr.STATION_SPACING_M
    for col in range(4):
        out[:n_valid, col] = np.interp(targets, s, pts[:, col])
    # Conservative sigma: the larger of the two raw points either side of the station.
    hi = np.minimum(np.searchsorted(s, targets, side="left"), n_raw - 1)
    lo = np.maximum(hi - 1, 0)
    out[:n_valid, 4] = np.maximum(pts[hi, 4], pts[lo, 4])
    return out, n_valid


def trace(
    points: NDArray[np.floating],
    body_half_width_m: float,
    sigma_max_m: float,
    cfg: GeometricConfig | None = None,
) -> tr.LocalTrackData:
    """Trace the corridor ahead of the car from scan points in base_link. Never raises."""
    cfg = cfg or GeometricConfig()
    try:
        pts = np.asarray(points, dtype=np.float64).reshape(-1, 2)
    except (TypeError, ValueError):
        return tr.no_track()
    if len(pts) == 0:
        return tr.no_track()
    pts = pts[np.isfinite(pts).all(axis=1)]
    pts = pts[np.hypot(pts[:, 0], pts[:, 1]) <= MAX_POINT_RANGE_M]
    if len(pts) == 0:
        return tr.no_track()
    px, py = pts[:, 0], pts[:, 1]
    raw = _march(px, py, 0.0, cfg, body_half_width_m)
    if cfg.two_pass and len(raw.x) >= 2:
        j = min(3, len(raw.x) - 1)
        heading = math.atan2(raw.y[j] - raw.y[0], raw.x[j] - raw.x[0])
        second = _march(px, py, heading, cfg, body_half_width_m)
        if len(second.x) >= len(raw.x) - 1:
            raw = second
    stations, n_valid = _resample(raw)
    sigma = stations[:, 4].copy()
    sigma[n_valid:] = tr.INVALID_SIGMA_M
    return tr.assemble(
        stations[:, 0],
        stations[:, 1],
        stations[:, 2],
        stations[:, 3],
        sigma,
        sigma_max_m,
        valid_cap=n_valid,
    )
