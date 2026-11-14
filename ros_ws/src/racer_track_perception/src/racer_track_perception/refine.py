"""Scan-grounded refinement of a net output (`io.arch.type` circ_conv1d_v0_refine_v1).

Design: claude-docs/13-plug-and-play.md 15.5 and the principle of 13.2: the scan is the ground, the
net is a prior. The net's 16 stations are corrected against the scan points of the same scan;
stations the scan does not support keep the net's value, which is the only place the net adds
anything (past the corner, beyond what the scan can see).

Per station, with the heading taken from the net's own polyline (so a bad station never changes
the heading used for its neighbours):

* Walls on both sides. The nearest scan point on each side, by Euclidean distance, among the
  points within `window_m` along the heading (the wall rule of `geometric-v1`, the very same
  helper). The station moves along the line joining the two points until both distances are
  equal; the half widths are those distances. As in `geometric-v1`, a pair whose mean distance
  is not a plausible lane half width (`hw_min_m` to `hw_max_m`) is not trusted as a pair: the
  nearer wall alone is used (the far point is another part of the track).
* A wall on one side only. The station keeps its place and that side's half width becomes
  min(net half width, measured distance). If the measured distance is under `min_half_m` the
  station moves away from the wall until the distance is `min_half_m` or the net's other half
  width is used up; the other boundary stays where the net put it.
* No scan support: unchanged.
* A station never claims more half width than the scan shows (design 15.5, pre-registration
  v0.3 section 3.1): after the move, a side's half width is at most the Euclidean distance from
  the station to the nearest scan point on that side, at any distance along the heading (a point
  exactly on the heading counts for both sides). The wall rules above look only within
  `window_m` along the heading, so this cap is what keeps a point beside the corner exit, or
  ahead of it, out of the claimed corridor. It is the property the property test (L2) holds the
  refinement to, and it never narrows a correct lane (there the nearest point is the wall).

A station never moves more than `max_shift_m`. The moved polyline is then resampled to 0.20 m
from the point nearest the base_link origin (the 4.1 semantics: station 0 is the centreline
point nearest the car), sigma is carried (the larger of the two raw values either side of a
station) and `valid_count` is recomputed. A scan without any supporting point returns the input
unchanged.

The module is numpy only and ROS free, like the rest of the package, and never raises.
"""

from __future__ import annotations

import logging
import math
from dataclasses import dataclass

import numpy as np
from numpy.typing import NDArray

from racer_track_perception import geometric, selfcheck
from racer_track_perception import track as tr

# Constants of the bundle contract `circ_conv1d_v0_refine_v1`: a change of any of them is a new
# arch type, not a new setting.
WINDOW_M = 0.30
MAX_SHIFT_M = 0.35
MIN_HALF_M = geometric.DEFAULT_HW_MIN_M
HW_MIN_M = geometric.DEFAULT_HW_MIN_M
HW_MAX_M = geometric.DEFAULT_HW_MAX_M
FEELER_MAX_M = geometric.FEELER_MAX_M
# The last station may lie this far past the end of the moved polyline (the recentring shortens a
# bend by millimetres; one station must not be lost to that). The extension follows the last
# segment.
END_TOLERANCE_M = 0.05
_EPS = 1e-9

log = logging.getLogger(__name__)


@dataclass(frozen=True)
class RefineConfig:
    window_m: float = WINDOW_M
    max_shift_m: float = MAX_SHIFT_M
    min_half_m: float = MIN_HALF_M
    hw_min_m: float = HW_MIN_M
    hw_max_m: float = HW_MAX_M
    feeler_max_m: float = FEELER_MAX_M
    sigma_max_m: float = 0.25


def _copy(track: tr.LocalTrackData) -> tr.LocalTrackData:
    return tr.LocalTrackData(
        x_m=track.x_m.copy(),
        y_m=track.y_m.copy(),
        half_width_left_m=track.half_width_left_m.copy(),
        half_width_right_m=track.half_width_right_m.copy(),
        sigma_m=track.sigma_m.copy(),
        valid_count=track.valid_count,
        boundary_residual_m=track.boundary_residual_m,
        intrusion_fraction=track.intrusion_fraction,
    )


def _clean_points(points: object) -> NDArray[np.float64]:
    try:
        pts = np.asarray(points, dtype=np.float64).reshape(-1, 2)
    except (TypeError, ValueError):
        return np.zeros((0, 2))
    return pts[np.isfinite(pts).all(axis=1)]


def _refine_station(
    c: NDArray[np.float64],
    t: NDArray[np.float64],
    hl_net: float,
    hr_net: float,
    pts: NDArray[np.float64],
    cfg: RefineConfig,
) -> tuple[NDArray[np.float64], float, float, bool]:
    """(new centre, left half width, right half width, whether the scan supported the station)
    for one station at `c` with unit heading `t`."""
    rel = pts - c
    along = rel @ t
    lat = -rel[:, 0] * t[1] + rel[:, 1] * t[0]
    dist = np.hypot(rel[:, 0], rel[:, 1])
    near = (np.abs(along) <= cfg.window_m) & (dist <= cfg.feeler_max_m)
    nl = geometric._nearest(dist, near & (lat > 0.0))
    nr = geometric._nearest(dist, near & (lat < 0.0))
    if nl is None and nr is None:
        return c, hl_net, hr_net, False
    c_new, hl, hr = _walls(c, t, hl_net, hr_net, pts, dist, nl, nr, cfg)
    rel = pts - c_new
    d_new = np.hypot(rel[:, 0], rel[:, 1])
    lat_new = -rel[:, 0] * t[1] + rel[:, 1] * t[0]
    left = d_new[lat_new >= 0.0]
    right = d_new[lat_new <= 0.0]
    if len(left):
        hl = min(hl, float(left.min()))
    if len(right):
        hr = min(hr, float(right.min()))
    return c_new, hl, hr, True


def _walls(
    c: NDArray[np.float64],
    t: NDArray[np.float64],
    hl_net: float,
    hr_net: float,
    pts: NDArray[np.float64],
    dist: NDArray[np.float64],
    nl: tuple[int, float] | None,
    nr: tuple[int, float] | None,
    cfg: RefineConfig,
) -> tuple[NDArray[np.float64], float, float]:
    """The two-wall and one-wall rules of the module docstring for one station."""
    if (
        nl is not None
        and nr is not None
        and not (cfg.hw_min_m <= 0.5 * (nl[1] + nr[1]) <= cfg.hw_max_m)
    ):
        # Implausible lane width: the pair is not one lane. Keep the nearer wall only.
        if nl[1] <= nr[1]:
            nr = None
        else:
            nl = None
    if nl is not None and nr is not None:
        pl, pr = pts[nl[0]], pts[nr[0]]
        dl, dr = nl[1], nr[1]
        u = pl - pr
        norm = float(np.hypot(u[0], u[1]))
        u = u / norm if norm > _EPS else np.array([-t[1], t[0]])
        shift = 0.5 * (dl - dr)
        if abs(shift) > cfg.max_shift_m:
            shift = math.copysign(cfg.max_shift_m, shift)
        c_new = c + shift * u
        mean = 0.5 * (dl + dr)
        hl = min(mean, float(np.hypot(*(pl - c_new))))
        hr = min(mean, float(np.hypot(*(pr - c_new))))
        return c_new, hl, hr
    if nl is not None:
        p, d = pts[nl[0]], nl[1]
        away = c - p
        side_net, other_net = hl_net, hr_net
    else:
        assert nr is not None
        p, d = pts[nr[0]], nr[1]
        away = c - p
        side_net, other_net = hr_net, hl_net
    shift = 0.0
    c_new = c
    if d < cfg.min_half_m:
        shift = min(cfg.min_half_m - d, max(other_net, 0.0), cfg.max_shift_m)
        norm = float(np.hypot(away[0], away[1]))
        direction = (
            away / norm
            if norm > _EPS
            else (np.array([t[1], -t[0]]) if nl is not None else np.array([-t[1], t[0]]))
        )
        c_new = c + shift * direction
    side = shift + min(side_net, d)
    other = max(other_net - shift, 0.0)
    return (c_new, side, other) if nl is not None else (c_new, other, side)


def _resample(
    x: NDArray[np.float64],
    y: NDArray[np.float64],
    hl: NDArray[np.float64],
    hr: NDArray[np.float64],
    sigma: NDArray[np.float64],
) -> tuple[NDArray[np.float64], int] | None:
    """Stations 0.20 m apart along the polyline, from the point nearest the origin. Array [K, 5]
    (x, y, hl, hr, sigma) and the number of valid rows; None when the polyline has no length."""
    keep = np.concatenate([[True], np.hypot(np.diff(x), np.diff(y)) > _EPS])
    x, y, hl, hr, sigma = x[keep], y[keep], hl[keep], hr[keep], sigma[keep]
    if len(x) < 2:
        return None
    ex, ey = np.diff(x), np.diff(y)
    seg = np.hypot(ex, ey)
    s = np.concatenate([[0.0], np.cumsum(seg)])
    total = float(s[-1])
    # Nearest point of the polyline to the origin.
    t_proj = np.clip(-(x[:-1] * ex + y[:-1] * ey) / (seg * seg), 0.0, 1.0)
    px, py = x[:-1] + t_proj * ex, y[:-1] + t_proj * ey
    best = int(np.argmin(px * px + py * py))
    s0 = float(s[best] + t_proj[best] * seg[best])
    n_valid = min(
        math.floor((total - s0 + END_TOLERANCE_M) / tr.STATION_SPACING_M) + 1, tr.K_STATIONS
    )
    targets = s0 + np.arange(n_valid) * tr.STATION_SPACING_M
    out = np.zeros((tr.K_STATIONS, 5), dtype=np.float64)
    inside = np.minimum(targets, total)
    out[:n_valid, 0] = np.interp(inside, s, x)
    out[:n_valid, 1] = np.interp(inside, s, y)
    over = targets - inside
    if over.any():  # a short extension along the last segment
        out[:n_valid, 0] += over * ex[-1] / seg[-1]
        out[:n_valid, 1] += over * ey[-1] / seg[-1]
    out[:n_valid, 2] = np.interp(inside, s, hl)
    out[:n_valid, 3] = np.interp(inside, s, hr)
    hi = np.minimum(np.searchsorted(s, inside, side="left"), len(s) - 1)
    lo = np.maximum(hi - 1, 0)
    out[:n_valid, 4] = np.maximum(sigma[hi], sigma[lo])
    return out, n_valid


def refine_track(
    track: tr.LocalTrackData,
    scan_points_xy: NDArray[np.floating],
    cfg: RefineConfig | None = None,
) -> tr.LocalTrackData:
    """Scan-grounded correction of a net output (see the module docstring). Returns a new
    `LocalTrackData`; the input is not modified. Never raises and never returns a non-finite
    value in the first `valid_count` entries (on any failure the input is returned as it was)."""
    cfg = cfg or RefineConfig()
    n = int(track.valid_count)
    if n < 2 or not track.arrays_ok():
        return _copy(track)
    try:
        pts = _clean_points(scan_points_xy)
        if len(pts) == 0:
            return _copy(track)
        x = track.x_m[:n].astype(np.float64)
        y = track.y_m[:n].astype(np.float64)
        hl = track.half_width_left_m[:n].astype(np.float64)
        hr = track.half_width_right_m[:n].astype(np.float64)
        sg = track.sigma_m[:n].astype(np.float64)
        tan = selfcheck._tangents(x, y)
        new = np.zeros((n, 4), dtype=np.float64)
        supported = False
        for i in range(n):
            c, left, right, hit = _refine_station(
                np.array([x[i], y[i]]), tan[i], float(hl[i]), float(hr[i]), pts, cfg
            )
            new[i] = (c[0], c[1], left, right)
            supported = supported or hit
        if not supported:
            return _copy(track)
        resampled = _resample(new[:, 0], new[:, 1], new[:, 2], new[:, 3], sg)
        if resampled is None:
            return _copy(track)
        stations, n_valid = resampled
        sigma = stations[:, 4].copy()
        sigma[n_valid:] = tr.INVALID_SIGMA_M
        out = tr.assemble(
            stations[:, 0],
            stations[:, 1],
            stations[:, 2],
            stations[:, 3],
            sigma,
            max(cfg.sigma_max_m, float(sg.max()) + 1e-6),
            valid_cap=n_valid,
        )
        if out.valid_count < 1 or not out.arrays_ok():
            return _copy(track)
        return out
    except Exception:  # a refinement that fails must not take the net output down with it
        log.exception("scan refinement failed; keeping the net output")
        return _copy(track)
