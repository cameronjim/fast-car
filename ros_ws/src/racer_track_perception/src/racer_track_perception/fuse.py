"""`fused-v1`: the geometric tracer's corridor with the net's centre (`io.arch.type`
circ_conv1d_v0_fused_v1).

Design: claude-docs/13-plug-and-play.md 16.4. The tracer (`geometric.trace`) supplies the stations
and the two corridor edges; the refined net (`refine.refine_track` on the net output) supplies the
centre where it has one within reach. Range and coverage are therefore the tracer's, the centre is
the net's, and the corridor never leaves the tracer's edges.

Per station i valid in the tracer track, with the unit normal of the tracer's own polyline (left
positive, the convention of `selfcheck`):

* the two edge points (centre plus the left half width, centre minus the right half width, along
  the normal) do not move;
* target: the point of the net's centreline (the polyline through its valid stations) nearest the
  tracer's centre, provided it is within `reach_m`; otherwise the station is left as it is;
* new centre: the tracer's centre moved along the normal towards the target by `blend`, clamped to
  stay `edge_margin_m` inside each edge (and inside the 2.0 m half width cap of the message). If
  the corridor is narrower than twice the margin the centre stays where the tracer put it;
* the half widths are recomputed as the distances from the new centre to the two (unmoved) edges.

Station count and spacing, `sigma` and `valid_count` are the tracer's. With `append_far` the net's
stations beyond the tracer's last valid one are appended (up to 16 stations in all), as the net's
refined output has them, when the first of them lies one station spacing (within
`JOIN_TOLERANCE_M`) ahead of the fused last station. A net output that is empty or not finite
returns the tracer's track unchanged and is reported as such (`FuseOutcome.NET_UNUSABLE`).

The module is numpy only and ROS free, like the rest of the package, and never raises.
"""

from __future__ import annotations

import enum
import logging
import math
from dataclasses import dataclass

import numpy as np
from numpy.typing import NDArray

from racer_track_perception import selfcheck
from racer_track_perception import track as tr

# The first appended station must lie this close to one station spacing from the fused last
# station (the 4.1 spacing contract at the join). Part of the arch contract, not a setting.
JOIN_TOLERANCE_M = 0.05
_EPS = 1e-9

log = logging.getLogger(__name__)


class FuseOutcome(enum.Enum):
    FUSED = "fused"
    NET_UNUSABLE = "net_unusable"  # empty, short of stations or not finite: the tracer's track
    TRACER_EMPTY = "tracer_empty"  # nothing to fuse into: the tracer's track (no track)


@dataclass(frozen=True)
class FuseConfig:
    reach_m: float = 0.35  # max lateral move of a station's centre towards the net
    edge_margin_m: float = 0.15  # the centre stays at least this far from each tracer edge
    blend: float = 1.0  # 1.0 = take the net's centre where in reach
    append_far: bool = True  # add the net's stations beyond the tracer's last valid one
    # metres of shift per metre of station spacing allowed between neighbouring stations (the
    # Lipschitz envelope of `limit_shift_slope`, brief 38a); chosen on development sets by the
    # rule of addendum 1 section 3.2 and frozen with the bundle
    max_shift_slope: float = 0.30

    def validate(self) -> None:
        """Raise ValueError unless every field is usable (the bundle loader refuses on it)."""
        for name in ("reach_m", "edge_margin_m", "blend", "max_shift_slope"):
            value = getattr(self, name)
            if isinstance(value, bool) or not isinstance(value, (int, float)):
                raise ValueError(f"fuse.{name} must be a number, got {value!r}")  # noqa: TRY004
            if not math.isfinite(value):
                raise ValueError(f"fuse.{name} must be finite, got {value!r}")
        if not isinstance(self.append_far, bool):
            raise ValueError(f"fuse.append_far must be a bool, got {self.append_far!r}")  # noqa: TRY004
        if self.reach_m <= 0.0:
            raise ValueError(f"fuse.reach_m must be positive, got {self.reach_m!r}")
        if self.edge_margin_m < 0.0:
            raise ValueError(f"fuse.edge_margin_m must not be negative, got {self.edge_margin_m!r}")
        if self.max_shift_slope <= 0.0:
            raise ValueError(f"fuse.max_shift_slope must be positive, got {self.max_shift_slope!r}")
        if not 0.0 <= self.blend <= 1.0:
            raise ValueError(f"fuse.blend must be in [0, 1], got {self.blend!r}")

    def as_dict(self) -> dict[str, object]:
        return {
            "reach_m": float(self.reach_m),
            "edge_margin_m": float(self.edge_margin_m),
            "blend": float(self.blend),
            "append_far": bool(self.append_far),
            "max_shift_slope": float(self.max_shift_slope),
        }

    @classmethod
    def from_dict(cls, raw: object) -> FuseConfig:
        """The config a manifest carries; every key is required, none may be extra."""
        if not isinstance(raw, dict):
            raise ValueError(f"io.arch.fuse must be a mapping, got {type(raw).__name__}")  # noqa: TRY004
        want = {"reach_m", "edge_margin_m", "blend", "append_far", "max_shift_slope"}
        if set(raw) != want:
            raise ValueError(f"io.arch.fuse has keys {sorted(raw)}, needs exactly {sorted(want)}")
        cfg = cls(
            reach_m=raw["reach_m"],
            edge_margin_m=raw["edge_margin_m"],
            blend=raw["blend"],
            append_far=raw["append_far"],
            max_shift_slope=raw["max_shift_slope"],
        )
        cfg.validate()
        return cfg


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


def _net_usable(net: tr.LocalTrackData) -> bool:
    try:
        n = int(net.valid_count)
        return n >= 1 and net.arrays_ok()
    except Exception:  # noqa: BLE001
        return False


def _nearest_on_polyline(
    px: float, py: float, nx_: NDArray[np.float64], ny_: NDArray[np.float64]
) -> tuple[float, float, float]:
    """(x, y, distance) of the point of the polyline (nx_, ny_) nearest (px, py). One station is a
    polyline of one point."""
    if len(nx_) == 1:
        return float(nx_[0]), float(ny_[0]), float(math.hypot(nx_[0] - px, ny_[0] - py))
    ax, ay = nx_[:-1], ny_[:-1]
    ex, ey = np.diff(nx_), np.diff(ny_)
    seg2 = ex * ex + ey * ey
    t = np.where(seg2 > _EPS, ((px - ax) * ex + (py - ay) * ey) / np.maximum(seg2, _EPS), 0.0)
    t = np.clip(t, 0.0, 1.0)
    qx, qy = ax + t * ex, ay + t * ey
    d = np.hypot(qx - px, qy - py)
    best = int(np.argmin(d))
    return float(qx[best]), float(qy[best]), float(d[best])


def _envelope(a: NDArray[np.float64], step: float) -> NDArray[np.float64]:
    """min over j of (a_j + step * |i - j|) for every i (a two pass distance transform)."""
    e = a.copy()
    for i in range(1, len(e)):
        e[i] = min(e[i], e[i - 1] + step)
    for i in range(len(e) - 2, -1, -1):
        e[i] = min(e[i], e[i + 1] + step)
    return e


def limit_shift_slope(d: NDArray[np.float64], slope: float) -> NDArray[np.float64]:
    """The slope limit of brief 38a: the largest shift profile that is nowhere larger in size than
    `d`, keeps its signs, and changes by at most `slope * STATION_SPACING_M` between neighbouring
    stations (stations that do not move are zeros of `d`).

    The Lipschitz envelope is taken for the positive and the negative part separately. Where a
    positive and a negative station are neighbours the two envelopes can add to up to twice the
    step; the larger of the two is then reduced until the difference is within the step (a shift
    only ever shrinks)."""
    step = slope * tr.STATION_SPACING_M
    out = np.array(d, dtype=np.float64)
    for _ in range(4 * len(out) + 4):
        new = _envelope(np.maximum(out, 0.0), step) - _envelope(np.maximum(-out, 0.0), step)
        for i in range(len(new) - 1):
            a, b = new[i], new[i + 1]
            if a * b < 0.0 and abs(a - b) > step + 1e-12:
                if abs(a) >= abs(b):
                    new[i] = math.copysign(max(step - abs(b), 0.0), a)
                else:
                    new[i + 1] = math.copysign(max(step - abs(a), 0.0), b)
        if np.array_equal(new, out):
            break
        out = new
    return out


def fuse_track_with_outcome(
    geo: tr.LocalTrackData, net: tr.LocalTrackData, cfg: FuseConfig | None = None
) -> tuple[tr.LocalTrackData, FuseOutcome]:
    """`fuse_track` and what happened. Never raises; on any failure the tracer's track."""
    cfg = cfg or FuseConfig()
    ng = int(geo.valid_count)
    if ng < 1 or not geo.arrays_ok():
        return _copy(geo), FuseOutcome.TRACER_EMPTY
    if not _net_usable(net):
        return _copy(geo), FuseOutcome.NET_UNUSABLE
    try:
        return _fuse(geo, net, cfg), FuseOutcome.FUSED
    except Exception:  # a fusion that fails must not take the tracer's track down with it
        log.exception("track fusion failed; keeping the tracer's track")
        return _copy(geo), FuseOutcome.NET_UNUSABLE


def fuse_track(
    geo: tr.LocalTrackData, net: tr.LocalTrackData, cfg: FuseConfig | None = None
) -> tr.LocalTrackData:
    """The tracer's corridor with the net's centre (see the module docstring)."""
    return fuse_track_with_outcome(geo, net, cfg)[0]


def _fuse(geo: tr.LocalTrackData, net: tr.LocalTrackData, cfg: FuseConfig) -> tr.LocalTrackData:
    ng = int(geo.valid_count)
    nn = int(net.valid_count)
    gx = geo.x_m[:ng].astype(np.float64)
    gy = geo.y_m[:ng].astype(np.float64)
    hl = geo.half_width_left_m[:ng].astype(np.float64)
    hr = geo.half_width_right_m[:ng].astype(np.float64)
    sg = geo.sigma_m[:ng].astype(np.float64)
    zx = net.x_m[:nn].astype(np.float64)
    zy = net.y_m[:nn].astype(np.float64)

    # A single station has no polyline and so no normal: it is left as the tracer put it.
    tan = selfcheck._tangents(gx, gy) if ng >= 2 else np.array([[1.0, 0.0]])
    nrm = np.stack([-tan[:, 1], tan[:, 0]], axis=1)  # left normal

    cx, cy = gx.copy(), gy.copy()
    new_hl, new_hr = hl.copy(), hr.copy()
    cap = tr.HALF_WIDTH_CAP_M
    d = np.zeros(ng)  # the shift each station would take by the rules above
    bounds = np.zeros((ng, 2))  # the margin and cap bounds of that shift
    has_bounds = np.zeros(ng, dtype=bool)
    for i in range(ng if ng >= 2 else 0):
        qx, qy, dist = _nearest_on_polyline(gx[i], gy[i], zx, zy)
        if dist > cfg.reach_m:
            continue
        if hl[i] + hr[i] < 2.0 * cfg.edge_margin_m:
            continue  # corridor too narrow to move in: the tracer's centre
        # signed lateral offset (left positive) of the target from the tracer's centre
        want = cfg.blend * ((qx - gx[i]) * nrm[i, 0] + (qy - gy[i]) * nrm[i, 1])
        lo = max(-(hr[i] - cfg.edge_margin_m), hl[i] - cap)  # moving right shrinks hr, grows hl
        hi = min(hl[i] - cfg.edge_margin_m, cap - hr[i])
        if lo > hi:
            continue
        d[i] = min(max(min(max(want, -cfg.reach_m), cfg.reach_m), lo), hi)
        bounds[i] = (lo, hi)
        has_bounds[i] = True
    applied = limit_shift_slope(d, cfg.max_shift_slope)
    for i in range(ng):
        if not has_bounds[i]:
            continue  # out of reach, a narrow corridor or a lone station: the tracer's centre
        s = min(max(applied[i], bounds[i, 0]), bounds[i, 1])
        cx[i] = gx[i] + s * nrm[i, 0]
        cy[i] = gy[i] + s * nrm[i, 1]
        new_hl[i] = hl[i] - s
        new_hr[i] = hr[i] + s

    x_out = list(cx)
    y_out = list(cy)
    l_out = list(new_hl)
    r_out = list(new_hr)
    s_out = list(sg)

    if cfg.append_far and ng < tr.K_STATIONS:
        _append_far(geo, net, (x_out, y_out, l_out, r_out, s_out), tan[-1])

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
    if out.valid_count < ng or not out.arrays_ok():
        return _copy(geo)
    out.boundary_residual_m = geo.boundary_residual_m
    out.intrusion_fraction = geo.intrusion_fraction
    return out


def _append_far(
    geo: tr.LocalTrackData,
    net: tr.LocalTrackData,
    lists: tuple[list[float], list[float], list[float], list[float], list[float]],
    last_tangent: NDArray[np.float64],
) -> None:
    """Append the net's stations beyond the fused last one, in place, when the join keeps the
    spacing contract (see the module docstring)."""
    x_out, y_out, l_out, r_out, s_out = lists
    nn = int(net.valid_count)
    lx, ly = x_out[-1], y_out[-1]
    best = -1
    best_err = math.inf
    for j in range(nn):
        dx = float(net.x_m[j]) - lx
        dy = float(net.y_m[j]) - ly
        ahead = dx * last_tangent[0] + dy * last_tangent[1] > 0.0
        err = abs(math.hypot(dx, dy) - tr.STATION_SPACING_M)
        if ahead and err <= JOIN_TOLERANCE_M and err < best_err:
            best, best_err = j, err
    if best < 0:
        return
    for j in range(best, nn):
        if len(x_out) >= tr.K_STATIONS:
            break
        x_out.append(float(net.x_m[j]))
        y_out.append(float(net.y_m[j]))
        l_out.append(float(net.half_width_left_m[j]))
        r_out.append(float(net.half_width_right_m[j]))
        s_out.append(float(net.sigma_m[j]))
