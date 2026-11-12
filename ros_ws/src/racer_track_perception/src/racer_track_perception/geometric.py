"""The geometric corridor tracer (geometric-v1): a classical, deterministic track estimate from
scan points.

Input: scan points in base_link (body filtered, `preprocess.scan_to_points`). Output: the same 16
stations as the net, as a `LocalTrackData`.

1. March: from the origin, predict the centre one STEP_M along the current heading, then twice
   recentre it on true Euclidean geometry. Candidates are scan points within WINDOW_M along the
   heading and within FEELER_MAX_M of the predicted centre by Euclidean distance; those to the
   left (lateral > 0) and those to the right are separate sets, and the nearest point of each set
   by Euclidean distance is the wall point (`pL`, `pR`, at distances `dl`, `dr`).
2. Two walls: the centre moves by 0.5 * (dl - dr) along the unit vector from `pR` to `pL`, and the
   half width is 0.5 * (dl + dr). One wall: the centre is placed the remembered half width from
   that wall point along the line from the centre to it. A two-wall half width outside
   [hw_min_m, hw_max_m] is not trusted: the nearer wall alone is used. Neither wall: stop.
3. The remembered half width is the smallest two-wall half width measured so far in this march
   (the default until one is measured). The distance to the nearest scan point never
   underestimates the distance to a wall, and overestimates it where the wall is sparse or
   hidden (the end of the island, beyond a corner), so the smallest measurement is the best one;
   remembering the last one carried the overestimate into the one-wall stations after the island.
4. The heading is the direction perpendicular to that line (the line to the left wall rotated by
   -90 degrees, along travel), its change limited to max_turn_rad and scaled by heading_blend.
5. Stop when the centre would come within the body half width of a scan point.
6. sigma is SIGMA_TWO_WALLS_M where both walls were seen, SIGMA_ONE_WALL_M where one was.
7. Resample so stations are exactly STATION_SPACING_M apart along the traced polyline.

Why v0 was wrong: it measured the wall distance along the normal of the marching heading inside a
+/-0.15 m strip. In a corner the heading lags the wall direction, so the wall is measured at a
slant, the distance comes out too long and the centre lands a full half width from the wall along
that slanted normal, 0.2 to 0.3 m too close to the outer wall while the message still claims the
full half width. On the 2026-11-12 bag, 99 percent of the scan points the self-check counted as
intruding lay on the outer-wall side (median depth 0.26 m). Measuring the Euclidean distance to
the nearest wall point and moving along the line between the two wall points does not depend on
the lag of the heading.

Measured on that bag (replay of the gap follow laps, 305 scans, and the 148 moving scans of the
four PnP runs), v0 -> v1: intrusion mean 0.087 -> 0.007 and 0.140 -> 0.017; share over 0.10
34 -> 0.3 percent and 74 -> 0 percent; clearance deficit p90 0.199 -> 0.022 m and 0.293 -> 0.036 m.
See the pull request for the older bags and the simulation.

Refinement over the plain procedure (measured in eval_bags and eval_sim, see the README of the
released model): a first pass from heading 0 estimates the lane direction at the car, and a second
pass starts from that heading so station 0 sits on the normal through the car (the label's
definition: the centreline point nearest the car) and its width is measured across the lane, not
across the car's own heading.

Known weak spot: the island end is small and sparse, so the inner boundary around it is traced
from one wall (or from two with the nearest visible island point, which is beyond the hidden
corner) at the remembered half width.
"""

from __future__ import annotations

import math
from dataclasses import dataclass

import numpy as np
from numpy.typing import NDArray

from racer_track_perception import track as tr

STEP_M = 0.20
# Half length of the window along the heading in which wall points are looked for. Swept from 0.15
# to 0.35 m with the half width memory below: at 0.15 to 0.25 m the tracer cannot follow a tight
# corner (replay of the 2026-11-12 bag, moving PnP scans, intrusion mean: 0.074 at 0.15 m without
# the memory, 0.022 at 0.25 m, 0.017 at 0.27 m, 0.010 at 0.30 m); a wider one picks wall points
# beside the corner exit that are farther than the wall (simulation, held-out family tracks, worst
# station against v0: +0.004 m at 0.27 m, +0.007 at 0.28 m, +0.013 at 0.30 m, +0.033 at 0.35 m).
WINDOW_M = 0.27
RECENTRE_ITERATIONS = 2
FEELER_MAX_M = 2.0
MAX_TURN_RAD = 0.5
SIGMA_TWO_WALLS_M = 0.05
SIGMA_ONE_WALL_M = 0.15
# Path length to march: 15 steps of STEP_M give the 3.0 m the 16 stations need. The recentring can
# move a centre back along the track, so a step is up to a few millimetres short of STEP_M in a
# bend (a 90 degree bend of radius 1 m traced 2.977 m in 15 steps); the extra steps make up the
# difference and the resampling drops the surplus.
EXTRA_STEPS = 2
N_STEPS = tr.K_STATIONS - 1 + EXTRA_STEPS
# Plausible half widths of a lane. On the 2026-11-11 bags (lane 0.8 to 1.3 m) the unguarded tracer
# paired a near wall with a far, unrelated one (the island's stub, the room) in one scan in three,
# and the centreline then spiralled off the lane; with this test the mean distance to the net's
# centreline fell from 0.28 to 0.09 m, the share of scans over 0.3 m apart from 36 to 0 percent
# and the median boundary residual from 0.107 to 0.059 m, with no loss in simulation. The upper
# value is half the widest lane the simulator trains on (1.6 m): a wider lane is traced by its
# nearer wall alone.
DEFAULT_HW_MIN_M = 0.3
DEFAULT_HW_MAX_M = 0.8
# Points further than this from the car cannot take part in a 3 m march with a 2 m feeler.
MAX_POINT_RANGE_M = 6.0


@dataclass(frozen=True)
class GeometricConfig:
    default_half_width_m: float = 0.5
    two_pass: bool = True
    heading_blend: float = 1.0  # 1.0: the plain direction of the last step; lower smooths it
    max_turn_rad: float = MAX_TURN_RAD
    # A two-wall midpoint whose half width is outside [hw_min_m, hw_max_m] is not trusted (the far
    # "wall" is another part of the track): the nearer wall alone is used. inf disables the test.
    hw_min_m: float = DEFAULT_HW_MIN_M
    hw_max_m: float = DEFAULT_HW_MAX_M
    # Half length of the window along the heading in which wall points are looked for.
    window_m: float = WINDOW_M


@dataclass
class _Raw:
    x: list[float]
    y: list[float]
    hl: list[float]
    hr: list[float]
    sigma: list[float]


def _nearest(dist: NDArray[np.float64], mask: NDArray[np.bool_]) -> tuple[int, float] | None:
    """Index and distance of the nearest masked point, or None when the mask is empty."""
    if not mask.any():
        return None
    idx = np.flatnonzero(mask)
    k = int(np.argmin(dist[idx]))
    return int(idx[k]), float(dist[idx[k]])


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
    hw = last_hw
    seen_min = math.inf
    sigma = SIGMA_ONE_WALL_M
    for step in range(N_STEPS + 1):
        if step > 0:
            cx += STEP_M * math.cos(heading)
            cy += STEP_M * math.sin(heading)
        found = False
        for _ in range(RECENTRE_ITERATIONS):
            c, s = math.cos(heading), math.sin(heading)
            rx, ry = px - cx, py - cy
            along = rx * c + ry * s
            lat = -rx * s + ry * c
            dist = np.hypot(rx, ry)
            near = (np.abs(along) <= cfg.window_m) & (dist <= FEELER_MAX_M)
            nl = _nearest(dist, near & (lat > 0.0))
            nr = _nearest(dist, near & (lat < 0.0))
            if nl is None and nr is None:
                break
            if (
                nl is not None
                and nr is not None
                and not (cfg.hw_min_m <= 0.5 * (nl[1] + nr[1]) <= cfg.hw_max_m)
            ):
                # Implausible width: keep only the nearer wall.
                if nl[1] <= nr[1]:
                    nr = None
                else:
                    nl = None
            if nl is not None and nr is not None:
                il, dl = nl
                ir, dr = nr
                ux, uy = px[il] - px[ir], py[il] - py[ir]
                norm = math.hypot(ux, uy)
                if norm < 1e-9:
                    ux, uy = -s, c
                else:
                    ux, uy = ux / norm, uy / norm
                shift = 0.5 * (dl - dr)
                cx += shift * ux
                cy += shift * uy
                hw = 0.5 * (dl + dr)
                seen_min = min(seen_min, hw)
                last_hw = seen_min
                sigma = SIGMA_TWO_WALLS_M
            elif nl is not None:
                il, dl = nl
                ux, uy = (px[il] - cx) / max(dl, 1e-9), (py[il] - cy) / max(dl, 1e-9)
                cx, cy = px[il] - last_hw * ux, py[il] - last_hw * uy
                hw = last_hw
                sigma = SIGMA_ONE_WALL_M
            else:
                assert nr is not None
                ir, dr = nr
                ux, uy = -(px[ir] - cx) / max(dr, 1e-9), -(py[ir] - cy) / max(dr, 1e-9)
                cx, cy = px[ir] + last_hw * ux, py[ir] + last_hw * uy
                hw = last_hw
                sigma = SIGMA_ONE_WALL_M
            # The tangent is u (pointing to the left wall) rotated by -90 degrees, along travel.
            tx, ty = uy, -ux
            if tx * c + ty * s < 0.0:
                tx, ty = -tx, -ty
            turn = (math.atan2(ty, tx) - heading + math.pi) % (2.0 * math.pi) - math.pi
            turn *= cfg.heading_blend
            heading += max(-cfg.max_turn_rad, min(cfg.max_turn_rad, turn))
            found = True
        if not found:
            break
        if float(np.min((px - cx) ** 2 + (py - cy) ** 2)) < r2_body:
            break
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
