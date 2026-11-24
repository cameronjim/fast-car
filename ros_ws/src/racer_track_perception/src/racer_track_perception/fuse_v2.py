"""`fused-v2`: the geometric tracer's corridor edges, the net's centre, half widths by ray cast
(`io.arch.type` circ_conv1d_v0_fused_v2). Brief 55, owner decision 27.

`fused-v1` kept the tracer's edge points but a consumer rebuilds the edges from the OUTPUT's own
normals (`selfcheck._tangents`, the rule `corridor.polygon` applies), and those turn when the
centre moves, so part of the corridor was lost. `fused-v2` measures the half widths along the
output's own normals instead:

1. centre: the `fused-v1` rule (`fuse._fuse` with `append_far` off: reach, edge margin, blend and
   the optional shift slope limit), giving one shifted centre per tracer station;
2. edges: the tracer's own edge polylines (station plus the left half width and minus the right
   half width along the tracer's own normals). They never move. Both ends may be extended along
   their end tangents by `edge_end_extension_m`;
3. stations: the shifted centre polyline (its end extended along its end tangent by the same
   `edge_end_extension_m`, so a polyline that the shift shortened by millimetres does not lose a
   station) resampled at 0.20 m from the point nearest base_link (design 13 section 4.1);
4. half widths: from every output station along the normal a consumer computes (left positive) to
   the first crossing of the left edge polyline, and along the opposite direction to the first
   crossing of the right one. A ray that misses ends the valid stations there (none valid at
   station 0 gives the tracer's track);
5. with `append_far` the net's stations beyond the last one are appended as in `fused-v1`, with the
   net's own half widths.

Every rebuilt edge point of stations 0 to n-1 therefore lies on the tracer's edge polyline (or its
allowed extension): the polygon a consumer rebuilds has its vertices on the tracer's polygon
boundary. Sigma is the larger of the two tracer sigmas either side of the station.

A net output that is empty or not finite returns the tracer's track and is reported as
`FuseOutcome.NET_UNUSABLE`. The module is numpy only and ROS free and never raises.
"""

from __future__ import annotations

import logging
import math
from dataclasses import dataclass

import numpy as np
from numpy.typing import NDArray

from racer_track_perception import fuse, selfcheck
from racer_track_perception import track as tr
from racer_track_perception.fuse import FuseOutcome

log = logging.getLogger(__name__)

MAX_EDGE_EXTENSION_M = 0.20  # the registered upper end of the free parameter
NO_SLOPE_LIMIT = 1.0e6  # `max_shift_slope: none` of the yaml grid, stored as a number
_EPS = 1e-12


@dataclass(frozen=True)
class FuseV2Config(fuse.FuseConfig):
    """`FuseConfig` plus the edge extension. `max_shift_slope` of `NO_SLOPE_LIMIT` or more means
    no limit (the yaml's `none`)."""

    edge_end_extension_m: float = 0.0

    def validate(self) -> None:
        super().validate()
        value = self.edge_end_extension_m
        if isinstance(value, bool) or not isinstance(value, (int, float)):
            raise ValueError(f"fuse.edge_end_extension_m must be a number, got {value!r}")  # noqa: TRY004
        if not math.isfinite(value) or not 0.0 <= value <= MAX_EDGE_EXTENSION_M:
            raise ValueError(
                f"fuse.edge_end_extension_m must be in [0, {MAX_EDGE_EXTENSION_M}], got {value!r}"
            )

    def as_dict(self) -> dict[str, object]:
        return {**super().as_dict(), "edge_end_extension_m": float(self.edge_end_extension_m)}

    @classmethod
    def from_dict(cls, raw: object) -> FuseV2Config:
        """The config a manifest carries; every key is required, none may be extra."""
        if not isinstance(raw, dict):
            raise ValueError(f"io.arch.fuse must be a mapping, got {type(raw).__name__}")  # noqa: TRY004
        want = {*fuse.FuseConfig().as_dict(), "edge_end_extension_m"}
        if set(raw) != want:
            raise ValueError(f"io.arch.fuse has keys {sorted(raw)}, needs exactly {sorted(want)}")
        cfg = cls(
            reach_m=raw["reach_m"],
            edge_margin_m=raw["edge_margin_m"],
            blend=raw["blend"],
            append_far=raw["append_far"],
            max_shift_slope=raw["max_shift_slope"],
            edge_end_extension_m=raw["edge_end_extension_m"],
        )
        cfg.validate()
        return cfg


def _extend(
    x: NDArray[np.float64], y: NDArray[np.float64], ext: float, start: bool
) -> tuple[NDArray[np.float64], NDArray[np.float64]]:
    """The polyline with its end (and, with `start`, its start) pushed out along the end tangent."""
    if ext <= 0.0 or len(x) < 2:
        return x, y
    ex, ey = x[-1] - x[-2], y[-1] - y[-2]
    n = math.hypot(ex, ey)
    if n > _EPS:
        x = np.append(x, x[-1] + ext * ex / n)
        y = np.append(y, y[-1] + ext * ey / n)
    if start:
        sx, sy = x[0] - x[1], y[0] - y[1]
        n = math.hypot(sx, sy)
        if n > _EPS:
            x = np.insert(x, 0, x[0] + ext * sx / n)
            y = np.insert(y, 0, y[0] + ext * sy / n)
    return x, y


def _first_hit(
    px: float,
    py: float,
    dx: float,
    dy: float,
    ax: NDArray[np.float64],
    ay: NDArray[np.float64],
    bx: NDArray[np.float64],
    by: NDArray[np.float64],
) -> float | None:
    """Smallest t >= 0 with (px, py) + t (dx, dy) on a segment (a, b), or None."""
    ex, ey = bx - ax, by - ay
    den = dx * ey - dy * ex
    ok = np.abs(den) > _EPS
    safe = np.where(ok, den, 1.0)
    wx, wy = ax - px, ay - py
    t = (wx * ey - wy * ex) / safe
    u = (wx * dy - wy * dx) / safe
    tol = 1e-9
    hit = ok & (t >= -tol) & (u >= -tol) & (u <= 1.0 + tol)
    if not hit.any():
        return None
    return float(max(np.min(t[hit]), 0.0))


def _resample_centre(
    cx: NDArray[np.float64], cy: NDArray[np.float64]
) -> tuple[NDArray[np.float64], NDArray[np.float64], NDArray[np.float64]]:
    """Stations 0.20 m apart along the polyline (cx, cy) from its point nearest base_link, at most
    K_STATIONS, and the fractional vertex index each station lies at."""
    ax, ay = cx[:-1], cy[:-1]
    ex, ey = np.diff(cx), np.diff(cy)
    seg2 = ex * ex + ey * ey
    t = np.where(seg2 > _EPS, (-ax * ex - ay * ey) / np.maximum(seg2, _EPS), 0.0)
    t = np.clip(t, 0.0, 1.0)
    d = np.hypot(ax + t * ex, ay + t * ey)
    j = int(np.argmin(d))
    sx, sy = float(ax[j] + t[j] * ex[j]), float(ay[j] + t[j] * ey[j])
    px = np.concatenate([[sx], cx[j + 1 :]])
    py = np.concatenate([[sy], cy[j + 1 :]])
    vidx = np.concatenate([[j + float(t[j])], np.arange(j + 1, len(cx), dtype=np.float64)])
    seg = np.hypot(np.diff(px), np.diff(py))
    s = np.concatenate([[0.0], np.cumsum(seg)])
    n = min(math.floor(float(s[-1]) / tr.STATION_SPACING_M + 1e-9) + 1, tr.K_STATIONS)
    targets = np.arange(n) * tr.STATION_SPACING_M
    return np.interp(targets, s, px), np.interp(targets, s, py), np.interp(targets, s, vidx)


def fuse_track_v2_with_outcome(
    geo: tr.LocalTrackData, net: tr.LocalTrackData, cfg: FuseV2Config | None = None
) -> tuple[tr.LocalTrackData, FuseOutcome]:
    """`fuse_track_v2` and what happened. Never raises; on any failure the tracer's track."""
    cfg = cfg or FuseV2Config()
    ng = int(geo.valid_count)
    if ng < 1 or not geo.arrays_ok():
        return fuse._copy(geo), FuseOutcome.TRACER_EMPTY
    if not fuse._net_usable(net):
        return fuse._copy(geo), FuseOutcome.NET_UNUSABLE
    try:
        return _fuse_v2(geo, net, cfg), FuseOutcome.FUSED
    except Exception:  # a fusion that fails must not take the tracer's track down with it
        log.exception("track fusion v2 failed; keeping the tracer's track")
        return fuse._copy(geo), FuseOutcome.NET_UNUSABLE


def fuse_track_v2(
    geo: tr.LocalTrackData, net: tr.LocalTrackData, cfg: FuseV2Config | None = None
) -> tr.LocalTrackData:
    """The tracer's corridor with the net's centre, widths by ray cast (module docstring)."""
    return fuse_track_v2_with_outcome(geo, net, cfg)[0]


def _fuse_v2(
    geo: tr.LocalTrackData, net: tr.LocalTrackData, cfg: FuseV2Config
) -> tr.LocalTrackData:
    ng = int(geo.valid_count)
    if ng < 2:
        return fuse._copy(geo)  # no polyline, no normal: the tracer's station as it is
    ext = float(cfg.edge_end_extension_m)
    # 1. the fused-v1 centre (append_far off: only the first ng shifted centres are wanted)
    v1_cfg = fuse.FuseConfig(
        reach_m=cfg.reach_m,
        edge_margin_m=cfg.edge_margin_m,
        blend=cfg.blend,
        append_far=False,
        max_shift_slope=cfg.max_shift_slope,
    )
    v1 = fuse._fuse(geo, net, v1_cfg)
    cx = v1.x_m[:ng].astype(np.float64)
    cy = v1.y_m[:ng].astype(np.float64)
    # 2. the tracer's own edge polylines, with the rule `corridor.polygon` applies
    gx = geo.x_m[:ng].astype(np.float64)
    gy = geo.y_m[:ng].astype(np.float64)
    gt = selfcheck._tangents(gx, gy)
    gnx, gny = -gt[:, 1], gt[:, 0]
    hl = geo.half_width_left_m[:ng].astype(np.float64)
    hr = geo.half_width_right_m[:ng].astype(np.float64)
    lx, ly = _extend(gx + hl * gnx, gy + hl * gny, ext, start=True)
    rx, ry = _extend(gx - hr * gnx, gy - hr * gny, ext, start=True)
    # 3. the stations
    cxe, cye = _extend(cx, cy, ext, start=False)
    sx, sy, vidx = _resample_centre(cxe, cye)
    n = len(sx)
    if n < 2:
        return fuse._copy(geo)
    # 4. half widths along the normal a consumer computes. The consumer's normal at the last
    # station depends on how many stations there are, so a miss that shortens the run is followed
    # by a recast of the shorter run until it is stable (each pass loses at least one station).
    new_hl = np.zeros(n)
    new_hr = np.zeros(n)
    m = n
    while True:
        ot = selfcheck._tangents(sx[:m], sy[:m])
        onx, ony = -ot[:, 1], ot[:, 0]
        n_ok = 0
        for k in range(m):
            left = _first_hit(sx[k], sy[k], onx[k], ony[k], lx[:-1], ly[:-1], lx[1:], ly[1:])
            right = _first_hit(sx[k], sy[k], -onx[k], -ony[k], rx[:-1], ry[:-1], rx[1:], ry[1:])
            if left is None or right is None:
                break
            new_hl[k], new_hr[k] = left, right
            n_ok = k + 1
        if n_ok == m or n_ok < 2:
            break
        m = n_ok
    if n_ok < 2:
        return fuse._copy(geo)
    # sigma: the larger tracer sigma either side of the station
    sg = geo.sigma_m[:ng].astype(np.float64)
    lo = np.clip(np.floor(vidx + 1e-9).astype(int), 0, ng - 1)
    hi = np.clip(np.ceil(vidx - 1e-9).astype(int), 0, ng - 1)
    sigma = np.maximum(sg[lo], sg[hi])

    x_out = list(sx[:n_ok])
    y_out = list(sy[:n_ok])
    l_out = list(new_hl[:n_ok])
    r_out = list(new_hr[:n_ok])
    s_out = list(sigma[:n_ok])
    if cfg.append_far and n_ok == n and n < tr.K_STATIONS:
        fuse._append_far(geo, net, (x_out, y_out, l_out, r_out, s_out), ot[n_ok - 1])

    n_out = len(x_out)
    k = tr.K_STATIONS
    arrays = [np.zeros(k, dtype=np.float64) for _ in range(5)]
    for a, src in zip(arrays, (x_out, y_out, l_out, r_out, s_out), strict=True):
        a[:n_out] = src
    arrays[4][n_out:] = tr.INVALID_SIGMA_M
    out = tr.assemble(
        arrays[0],
        arrays[1],
        arrays[2],
        arrays[3],
        arrays[4],
        float(np.max(arrays[4][:n_out])) + 1e-6,
        valid_cap=n_out,
    )
    if out.valid_count < 1 or not out.arrays_ok():
        return fuse._copy(geo)
    out.boundary_residual_m = geo.boundary_residual_m
    out.intrusion_fraction = geo.intrusion_fraction
    return out
