"""Arc-length self-label stations with rounded corners (labeller flag `--arc-length-stations`).

The geometric tracer (`geometric.trace`, the default label) resamples its marched polyline at
0.20 m of arc along that polyline. Its centre turns most of a square corner within one marching
step, so the chord between two resampled stations across a corner is shorter than 0.20 m (the
2026-11-23 session labels: 2.925 m from station 0 to 15 instead of 3.0) and the polyline through
the stations has near-square corners (about 1.5 rad of heading change at one station). The net's
scan refinement, and every consumer of the output contract, read stations as points 0.20 m apart
along the polyline through them, so those labels put far stations up to about 0.07 m along the
track from where the contract says they are.

This module relabels the same trace:

1. The raw marched polyline (`trace_raw`: the tracer's march and two-pass start, before its
   resampling and before the cut at 16 stations, so the surplus of the extra steps is kept).
2. Densified at `DENSE_DS_M` of arc. The centre is smoothed by a Gaussian of `smooth_m` (metres
   of arc) with point-reflected ends (a straight end stays straight and unshortened); this rounds
   a square corner to the radius the corridor allows. Each half width moves with the centre: a
   lateral shift d to the left gives left - d and right + d, so the corridor walls stay where the
   tracer measured them.
3. Station 0 is the point of the smoothed polyline nearest the car (the contract). Each next
   station is the first point along the polyline whose straight-line distance from the previous
   station is `tr.STATION_SPACING_M`, so the stations are exactly 0.20 m apart as the contract
   (and the refinement's resampling) reads them. sigma per station is the largest raw sigma
   since the previous station.

The track then goes through the labeller's unchanged gates (valid count, contract, intrusion,
clearance deficit, driven path). numpy only, never raises (an unusable trace gives no track).
"""

from __future__ import annotations

import math

import numpy as np
from numpy.typing import NDArray
from racer_track_perception import geometric
from racer_track_perception import track as tr

DENSE_DS_M = 0.01


def trace_raw(
    points: NDArray[np.floating],
    body_half_width_m: float,
    cfg: geometric.GeometricConfig | None = None,
) -> NDArray[np.float64] | None:
    """The tracer's raw marched polyline [n, 5] (x, y, hl, hr, sigma), exactly as
    `geometric.trace` marches it before resampling, or None without one."""
    cfg = cfg or geometric.GeometricConfig()
    try:
        pts = np.asarray(points, dtype=np.float64).reshape(-1, 2)
    except (TypeError, ValueError):
        return None
    pts = pts[np.isfinite(pts).all(axis=1)]
    pts = pts[np.hypot(pts[:, 0], pts[:, 1]) <= geometric.MAX_POINT_RANGE_M]
    if len(pts) == 0:
        return None
    px, py = pts[:, 0], pts[:, 1]
    raw = geometric._march(px, py, 0.0, cfg, body_half_width_m)
    if cfg.two_pass and len(raw.x) >= 2:
        j = min(3, len(raw.x) - 1)
        heading = math.atan2(raw.y[j] - raw.y[0], raw.x[j] - raw.x[0])
        second = geometric._march(px, py, heading, cfg, body_half_width_m)
        if len(second.x) >= len(raw.x) - 1:
            raw = second
    if len(raw.x) < 2:
        return None
    return np.array([raw.x, raw.y, raw.hl, raw.hr, raw.sigma], dtype=np.float64).T


def densify(raw: NDArray[np.float64]) -> NDArray[np.float64] | None:
    """[m, 5] samples every DENSE_DS_M of arc along the raw polyline (sigma: the larger of the
    two raw points either side)."""
    keep = np.concatenate([[True], np.hypot(*np.diff(raw[:, :2], axis=0).T) > 1e-9])
    raw = raw[keep]
    if len(raw) < 2:
        return None
    s = np.concatenate([[0.0], np.cumsum(np.hypot(*np.diff(raw[:, :2], axis=0).T))])
    t = np.arange(0.0, s[-1] + 1e-12, DENSE_DS_M)
    out = np.empty((len(t), 5))
    for col in range(4):
        out[:, col] = np.interp(t, s, raw[:, col])
    hi = np.minimum(np.searchsorted(s, t, side="left"), len(s) - 1)
    lo = np.maximum(hi - 1, 0)
    out[:, 4] = np.maximum(raw[hi, 4], raw[lo, 4])
    return out


def smooth_centre(dense: NDArray[np.float64], smooth_m: float) -> NDArray[np.float64]:
    """The dense polyline with its centre Gaussian-smoothed over `smooth_m` of arc (point
    reflection at both ends) and the half widths moved with it."""
    if smooth_m <= 0.0 or len(dense) < 3:
        return dense.copy()
    k = max(1, round(smooth_m / DENSE_DS_M))
    half = min(3 * k, len(dense) - 1)
    xy = dense[:, :2]
    head = 2.0 * xy[0] - xy[half:0:-1]
    tail = 2.0 * xy[-1] - xy[-2 : -half - 2 : -1]
    padded = np.vstack([head, xy, tail])
    off = np.arange(-half, half + 1)
    w = np.exp(-0.5 * (off / k) ** 2)
    w /= w.sum()
    sm = np.stack([np.convolve(padded[:, c], w, mode="valid") for c in (0, 1)], axis=1)
    # Lateral shift of each smoothed point from its original along the smoothed normal (left +).
    tang = np.gradient(sm, axis=0)
    norm = np.hypot(tang[:, 0], tang[:, 1])
    norm[norm < 1e-12] = 1.0
    nx, ny = -tang[:, 1] / norm, tang[:, 0] / norm
    d = (sm[:, 0] - xy[:, 0]) * nx + (sm[:, 1] - xy[:, 1]) * ny
    out = dense.copy()
    out[:, :2] = sm
    out[:, 2] = np.maximum(dense[:, 2] - d, 0.0)
    out[:, 3] = np.maximum(dense[:, 3] + d, 0.0)
    return out


def chord_stations(dense: NDArray[np.float64]) -> tuple[NDArray[np.float64], int]:
    """[K, 5] stations from the point nearest the origin, each tr.STATION_SPACING_M in a straight
    line from the previous one along the polyline, and the number found."""
    out = np.zeros((tr.K_STATIONS, 5))
    xy = dense[:, :2]
    j = int(np.argmin(np.hypot(xy[:, 0], xy[:, 1])))
    out[0] = dense[j]
    n = 1
    cur = xy[j].copy()
    seg = j  # the current station lies on segment [seg, seg + 1]
    last = j
    r = tr.STATION_SPACING_M
    while n < tr.K_STATIONS:
        m, a, found = seg, cur, False
        while m + 1 < len(xy):
            b = xy[m + 1]
            if math.hypot(b[0] - cur[0], b[1] - cur[1]) >= r - 1e-9:
                f = _circle_crossing(cur, a, b, r)
                p = a + f * (b - a)
                seg_len = math.hypot(*(xy[m + 1] - xy[m]))
                g = math.hypot(*(p - xy[m])) / seg_len if seg_len > 1e-12 else 0.0
                row = dense[m] + g * (dense[m + 1] - dense[m])
                row[:2] = p
                row[4] = float(np.max(dense[last : m + 2, 4]))
                out[n] = row
                n += 1
                cur, seg, last, found = p, m, m, True
                break
            a = b
            m += 1
        if not found:
            break
    return out, n


def _circle_crossing(
    c: NDArray[np.float64], a: NDArray[np.float64], b: NDArray[np.float64], r: float
) -> float:
    """f in [0, 1] with |a + f (b - a) - c| = r (a inside the circle, b on or outside)."""
    d = b - a
    e = a - c
    qa = float(d @ d)
    if qa < 1e-18:
        return 1.0
    qb = 2.0 * float(e @ d)
    qc = float(e @ e) - r * r
    disc = max(qb * qb - 4.0 * qa * qc, 0.0)
    return float(min(max((-qb + math.sqrt(disc)) / (2.0 * qa), 0.0), 1.0))


def arc_track(
    points: NDArray[np.floating],
    body_half_width_m: float,
    sigma_max_m: float,
    smooth_m: float,
    cfg: geometric.GeometricConfig | None = None,
) -> tr.LocalTrackData:
    """The arc-length, corner-rounded label track of a submap (module docstring)."""
    raw = trace_raw(points, body_half_width_m, cfg)
    if raw is None:
        return tr.no_track()
    dense = densify(raw)
    if dense is None:
        return tr.no_track()
    stations, n = chord_stations(smooth_centre(dense, smooth_m))
    sigma = stations[:, 4].copy()
    sigma[n:] = tr.INVALID_SIGMA_M
    return tr.assemble(
        stations[:, 0],
        stations[:, 1],
        stations[:, 2],
        stations[:, 3],
        sigma,
        sigma_max_m,
        valid_cap=n,
    )
