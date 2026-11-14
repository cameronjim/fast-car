"""Scan deskew (P1.4, claude-docs/13-plug-and-play.md 15.6): move every beam of one sweep to the
car's pose at a single reference time. ROS-free, numpy only.

A sweep is not instantaneous: the LiDAR reports `scan_time` and `time_increment` (the real C1
driver says about 0.072 s and 1.0e-4 s per beam), and the car moves while it turns. Beam i is
seen at time t_i from the car pose at t_i. With the motion held constant over the sweep (speed v,
yaw rate w of the rear axle, an exact arc) the point seen at t_i is carried to the pose at the
reference time t_r:

    dt = t_r - t_i, theta = w dt, T = (v/w sin theta, v/w (1 - cos theta)) (v dt, 0 when w = 0)
    p_ref = R(-theta) (p_i - T)                 p_i in base_link at t_i, p_ref at t_r

Three things the sweep does not say are inputs, never assumptions (the pre-registration, section
4 of docs/notes/preregistration-pnp-v0.3.md, has the data choose them):

* `stamp_is`: whether header.stamp is the first or the last beam's time.
* `beam_order`: whether time runs with the beam index (`index_ascending`) or against it
  (`index_descending`). The Slamtec driver stores the head's clockwise sweep reversed so that
  the angle grows counter-clockwise, which makes the second case plausible, not certain.
* the geometry of the sweep (angle_min, sign of angle_increment) is read from the message, and
  the mount yaw and offset from vehicle_params, so no sense of rotation is hard coded.

`deskew_scan` returns ranges on the input's angle grid (nearest return per bin) and the offset to
add to header.stamp so the output stamp is the reference time. A scan taken at standstill comes
back unchanged. Nothing here raises on NaN, inf, empty or all-invalid ranges.

The car's own parts (about a dozen beams per scan on the real car return from inside the body
rectangle, at about 0.17 m) are fixed in the car frame, not in the world: carrying them with the
constant-twist arc would throw them up to v T out of the body and make phantom obstacles at the
car's edge. `body_beams` finds them and `deskew_points` / `deskew_scan` leave them where they are
(an optional mask; with none given every beam is carried).
"""

from __future__ import annotations

import math
from dataclasses import dataclass

import numpy as np
from numpy.typing import NDArray

from racer_track_perception.preprocess import BODY_FILTER_MARGIN_M
from racer_track_perception.vehicle import BodyFootprint, LidarMount

STAMP_IS_VALUES = ("sweep_start", "sweep_end")
REFERENCES = ("sweep_start", "sweep_mid", "sweep_end")
BEAM_ORDERS = ("index_ascending", "index_descending")

#: Motion over the whole sweep below this (m and rad) is treated as none: the input comes back
#: untouched instead of being resampled onto a grid it already sits on.
NEGLIGIBLE_MOTION = 1e-6
#: |w dt| under this uses the series of the arc (sin x / x and (1 - cos x) / x are 0/0 at 0).
_SMALL_ANGLE_RAD = 1e-4


@dataclass(frozen=True)
class DeskewConfig:
    stamp_is: str  # "sweep_start" | "sweep_end": what header.stamp means. No default.
    beam_order: str  # "index_ascending" | "index_descending": time against the index. No default.
    reference: str = "sweep_end"  # output points are expressed at the car pose at this time
    max_twist_age_s: float = 0.10

    def __post_init__(self) -> None:
        if self.stamp_is not in STAMP_IS_VALUES:
            raise ValueError(f"stamp_is must be one of {STAMP_IS_VALUES}, got {self.stamp_is!r}")
        if self.beam_order not in BEAM_ORDERS:
            raise ValueError(f"beam_order must be one of {BEAM_ORDERS}, got {self.beam_order!r}")
        if self.reference not in REFERENCES:
            raise ValueError(f"reference must be one of {REFERENCES}, got {self.reference!r}")
        if not (math.isfinite(self.max_twist_age_s) and self.max_twist_age_s > 0.0):
            raise ValueError(f"max_twist_age_s must be positive, got {self.max_twist_age_s}")


@dataclass(frozen=True)
class Twist:
    """Motion held constant over the sweep, of the rear axle (base_link)."""

    v_mps: float
    yaw_rate_radps: float


def beam_times(n_beams: int, scan_time_s: float, time_increment_s: float) -> NDArray[np.float64]:
    """Time of each beam relative to the first beam in INDEX order (beam 0 is at 0), from the
    message's own fields: `time_increment` per beam when it is positive and finite, else
    `scan_time / n_beams`, else all zeros (no timing: nothing to compensate)."""
    n = max(int(n_beams), 0)
    step = 0.0
    if math.isfinite(time_increment_s) and time_increment_s > 0.0:
        step = float(time_increment_s)
    elif math.isfinite(scan_time_s) and scan_time_s > 0.0 and n > 0:
        step = float(scan_time_s) / n
    return np.arange(n, dtype=np.float64) * step


def _time_axis(t_beam: NDArray[np.float64]) -> tuple[float, float]:
    """(first, last) beam time of the sweep."""
    finite = t_beam[np.isfinite(t_beam)]
    if finite.size == 0:
        return 0.0, 0.0
    return float(finite.min()), float(finite.max())


def reference_time(t_beam: NDArray[np.float64], cfg: DeskewConfig) -> float:
    """The reference time on the axis of `t_beam`."""
    first, last = _time_axis(t_beam)
    return {"sweep_start": first, "sweep_mid": 0.5 * (first + last), "sweep_end": last}[
        cfg.reference
    ]


def stamp_offset(t_beam: NDArray[np.float64], cfg: DeskewConfig) -> float:
    """Seconds to add to header.stamp so that it is the reference time: the reference time minus
    the time header.stamp denotes (the first or the last beam's)."""
    first, last = _time_axis(t_beam)
    stamp_time = first if cfg.stamp_is == "sweep_start" else last
    return reference_time(t_beam, cfg) - stamp_time


def _check_twist(twist: Twist) -> None:
    if not (math.isfinite(twist.v_mps) and math.isfinite(twist.yaw_rate_radps)):
        raise ValueError(f"twist must be finite, got {twist}")


def _arc(
    v: float, w: float, dt: NDArray[np.float64]
) -> tuple[NDArray[np.float64], NDArray[np.float64], NDArray[np.float64]]:
    """(Tx, Ty, theta) of the rear axle after `dt` seconds on the arc of constant (v, w), in the
    frame of the start pose. Exact; the series is used only where sin x / x is 0/0."""
    theta = w * dt
    small = np.abs(theta) < _SMALL_ANGLE_RAD
    th2 = theta * theta
    sinc = np.where(small, 1.0 - th2 / 6.0, np.sin(theta) / np.where(small, 1.0, theta))
    vers = np.where(
        small, 0.5 * theta * (1.0 - th2 / 12.0), (1.0 - np.cos(theta)) / np.where(small, 1.0, theta)
    )
    return v * dt * sinc, v * dt * vers, theta


def deskew_points(
    ranges: NDArray[np.floating],
    angle_min: float,
    angle_increment: float,
    t_beam: NDArray[np.float64],
    twist: Twist,
    mount: LidarMount,
    cfg: DeskewConfig,
    static: NDArray[np.bool_] | None = None,
) -> NDArray[np.float64]:
    """Points [n, 2] in base_link at the reference time. Each beam's point is taken in the LiDAR
    frame at its own time, moved to base_link through the mount (vehicle_params) and carried to
    the reference pose with the constant-twist arc. A beam that is not a return (NaN, inf, zero
    or negative range) gives a NaN row. `t_beam[i]` is the time of beam i on any axis; the
    reference time is taken from that axis by `cfg.reference`. A beam flagged in `static` (the
    car's own parts) is not carried: its point stays where the mount puts it."""
    _check_twist(twist)
    r = np.asarray(ranges, dtype=np.float64).reshape(-1)
    n = r.size
    t = np.asarray(t_beam, dtype=np.float64).reshape(-1)
    if t.size != n:
        raise ValueError(f"t_beam has {t.size} entries for {n} beams")
    out = np.full((n, 2), np.nan)
    if n == 0:
        return out
    with np.errstate(invalid="ignore"):
        valid = np.isfinite(r) & (r > 0.0) & np.isfinite(t)
    if not valid.any():
        return out
    idx = np.nonzero(valid)[0]
    a = float(angle_min) + idx.astype(np.float64) * float(angle_increment)
    c_m, s_m = math.cos(mount.yaw_rad), math.sin(mount.yaw_rad)
    lx, ly = r[idx] * np.cos(a), r[idx] * np.sin(a)
    px = mount.x_m + c_m * lx - s_m * ly
    py = mount.y_m + s_m * lx + c_m * ly
    dt = reference_time(t, cfg) - t[idx]
    if static is not None:
        dt = np.where(np.asarray(static, dtype=bool).reshape(-1)[idx], 0.0, dt)
    tx, ty, theta = _arc(twist.v_mps, twist.yaw_rate_radps, dt)
    c, s = np.cos(theta), np.sin(theta)
    dx, dy = px - tx, py - ty
    out[idx, 0] = c * dx + s * dy
    out[idx, 1] = -s * dx + c * dy
    return out


def body_beams(
    ranges: NDArray[np.floating],
    angle_min: float,
    angle_increment: float,
    mount: LidarMount,
    body: BodyFootprint,
    margin_m: float = BODY_FILTER_MARGIN_M,
) -> NDArray[np.bool_]:
    """Beams of the input whose point lies inside the body rectangle grown by `margin_m` (the
    rule preprocess.scan_to_points uses to drop the car's own parts)."""
    r = np.asarray(ranges, dtype=np.float64).reshape(-1)
    out = np.zeros(r.size, dtype=bool)
    if r.size == 0:
        return out
    with np.errstate(invalid="ignore"):
        valid = np.isfinite(r) & (r > 0.0)
    a = float(angle_min) + np.arange(r.size) * float(angle_increment) + mount.yaw_rad
    x = mount.x_m + np.where(valid, r, 0.0) * np.cos(a)
    y = mount.y_m + np.where(valid, r, 0.0) * np.sin(a)
    inside = (
        (x >= body.x_min_m - margin_m)
        & (x <= body.x_max_m + margin_m)
        & (np.abs(y) <= body.half_width_m + margin_m)
    )
    return valid & inside


def rebin(
    points_base: NDArray[np.float64],
    mount: LidarMount,
    angle_min: float,
    angle_increment: float,
    n_beams: int,
    range_min: float,
    range_max: float,
) -> NDArray[np.float64]:
    """Ranges on the input's angle grid as seen from the LiDAR at the reference pose: nearest
    return per bin, inf where no point lands. A bin left empty between two filled neighbours takes
    the smaller neighbour (a one-bin hole is resampling, not free space). Points whose range is
    outside [range_min, range_max] or that fall outside the grid are dropped."""
    n = max(int(n_beams), 0)
    out = np.full(n, np.inf)
    pts = np.asarray(points_base, dtype=np.float64).reshape(-1, 2)
    if n == 0 or pts.size == 0 or not (math.isfinite(angle_min) and math.isfinite(angle_increment)):
        return out
    if angle_increment == 0.0:
        return out
    c_m, s_m = math.cos(mount.yaw_rad), math.sin(mount.yaw_rad)
    dx, dy = pts[:, 0] - mount.x_m, pts[:, 1] - mount.y_m
    lx = c_m * dx + s_m * dy
    ly = -s_m * dx + c_m * dy
    with np.errstate(invalid="ignore"):
        rng = np.hypot(lx, ly)
        ok = np.isfinite(rng) & (rng > 0.0) & (rng >= range_min) & (rng <= range_max)
    if not ok.any():
        return out
    bearing = np.arctan2(ly[ok], lx[ok])
    sign = 1.0 if angle_increment > 0.0 else -1.0
    delta = np.mod(sign * (bearing - angle_min), 2.0 * math.pi)
    bins = np.rint(delta / abs(angle_increment)).astype(np.int64)
    inside = (bins >= 0) & (bins < n)
    np.minimum.at(out, bins[inside], rng[ok][inside])
    if n >= 3:
        empty = ~np.isfinite(out)
        hole = empty[1:-1] & ~empty[:-2] & ~empty[2:]
        fill = np.minimum(out[:-2], out[2:])
        out[1:-1] = np.where(hole, fill, out[1:-1])
    return out


def _sanitise(
    ranges: NDArray[np.floating], range_min: float, range_max: float
) -> NDArray[np.float64]:
    """Input ranges with everything that is not a usable return set to inf."""
    r = np.asarray(ranges, dtype=np.float64).reshape(-1).copy()
    with np.errstate(invalid="ignore"):
        bad = ~(np.isfinite(r) & (r > 0.0) & (r >= range_min) & (r <= range_max))
    r[bad] = np.inf
    return r


def deskew_scan(
    ranges: NDArray[np.floating],
    angle_min: float,
    angle_increment: float,
    scan_time_s: float,
    time_increment_s: float,
    twist: Twist,
    mount: LidarMount,
    cfg: DeskewConfig,
    *,
    range_min: float = 0.0,
    range_max: float = math.inf,
    static: NDArray[np.bool_] | None = None,
) -> tuple[NDArray[np.float64], float]:
    """(ranges on the same grid, stamp offset in seconds to add to header.stamp so that the output
    stamp is the reference time). The output is finite or inf, never negative, never below
    `range_min` or above `range_max`. With no motion (or no timing) the sanitised input comes back
    on its own grid, not resampled. `static` flags the car's own parts (see `body_beams`), which
    are not carried."""
    _check_twist(twist)
    r = np.asarray(ranges, dtype=np.float64).reshape(-1)
    n = r.size
    t = beam_times(n, scan_time_s, time_increment_s)
    if cfg.beam_order == "index_descending":
        t = t[::-1].copy()
    offset = stamp_offset(t, cfg) if n else 0.0
    sweep_s = float(t.max()) if n else 0.0
    moved = abs(twist.v_mps) * sweep_s + abs(twist.yaw_rate_radps) * sweep_s
    if n == 0 or moved < NEGLIGIBLE_MOTION:
        return _sanitise(r, range_min, range_max), offset
    pts = deskew_points(r, angle_min, angle_increment, t, twist, mount, cfg, static)
    out = rebin(pts, mount, angle_min, angle_increment, n, range_min, range_max)
    return out, offset
