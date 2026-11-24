"""Scan nuisances: what a real room and a real LiDAR do to the clean geometry.

Drawn once per sample (`draw_nuisance`) and applied in two places: to the world before casting
(wall gaps, a room box around the track, clutter segments: `perturb_world`) and to the ranges
after casting (noise, dropout, a dead sector: `perturb_ranges`). Motion distortion is a twist the
caster takes (`raycast.Motion`), drawn here too. The label never sees any of this: it is computed
from the clean walls.
"""

from __future__ import annotations

import math
from dataclasses import dataclass

import numpy as np
from numpy.typing import NDArray

from racer_pnp_perception.raycast import Motion

RANGE_SIGMA_M = (0.005, 0.03)
DROPOUT_MAX = 0.10
DEAD_SECTOR_MAX_RAD = math.radians(40.0)
DEAD_SECTOR_CENTRE_JITTER_RAD = math.radians(45.0)
GAP_M = (0.05, 0.30)
MAX_GAPS = 5
ROOM_MARGIN_M = (1.0, 4.0)
ROOM_PROBABILITY = 0.85
MAX_CLUTTER = 6
CLUTTER_LENGTH_M = (0.1, 1.0)
MAX_TWIST_MPS = 1.5
SWEEP_S = 0.1


@dataclass(frozen=True)
class Nuisance:
    range_sigma_m: float
    dropout: float
    dead_centre_rad: float  # vehicle bearing of the dead sector's centre
    dead_width_rad: float
    n_gaps: int
    gap_widths_m: tuple[float, ...]
    room_margins_m: tuple[float, float, float, float] | None  # x-, x+, y-, y+ beyond the track
    n_clutter: int
    speed_mps: float
    sweep_sign: int
    inf_share: float  # share of dropped beams reported as +inf rather than NaN


CLEAN = Nuisance(0.0, 0.0, math.pi, 0.0, 0, (), None, 0, 0.0, 1, 0.0)


def draw_nuisance(rng: np.random.Generator, clean: bool = False) -> Nuisance:
    """Draw one sample's nuisances. `clean=True` returns CLEAN (an exact scan, no room)."""
    if clean:
        return CLEAN
    n_gaps = int(rng.integers(0, MAX_GAPS + 1))
    room = (
        tuple(float(rng.uniform(*ROOM_MARGIN_M)) for _ in range(4))
        if rng.uniform() < ROOM_PROBABILITY
        else None
    )
    return Nuisance(
        range_sigma_m=float(rng.uniform(*RANGE_SIGMA_M)),
        dropout=float(rng.uniform(0.0, DROPOUT_MAX)),
        dead_centre_rad=math.pi
        + float(rng.uniform(-DEAD_SECTOR_CENTRE_JITTER_RAD, DEAD_SECTOR_CENTRE_JITTER_RAD)),
        dead_width_rad=float(rng.uniform(0.0, DEAD_SECTOR_MAX_RAD)) if rng.uniform() < 0.7 else 0.0,
        n_gaps=n_gaps,
        gap_widths_m=tuple(float(rng.uniform(*GAP_M)) for _ in range(n_gaps)),
        room_margins_m=room,  # type: ignore[arg-type]
        n_clutter=int(rng.integers(0, MAX_CLUTTER + 1)),
        speed_mps=float(MAX_TWIST_MPS * rng.uniform() ** 2),
        sweep_sign=int(rng.choice([-1, 1])),
        inf_share=float(rng.uniform(0.0, 0.5)),
    )


def motion_for(nuisance: Nuisance, curvature_1pm: float) -> Motion:
    """The sweep's twist: speed from the nuisance, yaw rate from the local path curvature
    (positive = turning left in the direction of travel)."""
    return Motion(
        speed_mps=nuisance.speed_mps,
        yaw_rate_rps=nuisance.speed_mps * curvature_1pm,
        sweep_s=SWEEP_S,
        sweep_sign=nuisance.sweep_sign,
    )


def _box(x0: float, y0: float, x1: float, y1: float) -> NDArray[np.float64]:
    return np.array(
        [[x0, y0, x1, y0], [x1, y0, x1, y1], [x1, y1, x0, y1], [x0, y1, x0, y0]], dtype=np.float64
    )


def perturb_world(
    walls: NDArray[np.float64],
    centreline: NDArray[np.float64],
    lane_width_m: float,
    nuisance: Nuisance,
    rng: np.random.Generator,
) -> NDArray[np.float64]:
    """The walls the LiDAR sees: gaps cut in them, a room box around the track, and clutter.

    Clutter segments are placed in the room, never within half a lane width (plus 0.1 m) of the
    centreline, so the lane itself stays free.
    """
    out = walls
    mid = 0.5 * (walls[:, :2] + walls[:, 2:])
    keep = np.ones(len(walls), dtype=bool)
    for width in nuisance.gap_widths_m:
        centre = mid[int(rng.integers(0, len(walls)))]
        keep &= np.hypot(mid[:, 0] - centre[0], mid[:, 1] - centre[1]) > width / 2.0
    out = walls[keep]
    x0, y0 = float(walls[:, [0, 2]].min()), float(walls[:, [1, 3]].min())
    x1, y1 = float(walls[:, [0, 2]].max()), float(walls[:, [1, 3]].max())
    if nuisance.room_margins_m is not None:
        mxl, mxh, myl, myh = nuisance.room_margins_m
        bx0, by0, bx1, by1 = x0 - mxl, y0 - myl, x1 + mxh, y1 + myh
        out = np.vstack([out, _box(bx0, by0, bx1, by1)])
    else:
        bx0, by0, bx1, by1 = x0 - 2.5, y0 - 2.5, x1 + 2.5, y1 + 2.5
    clearance = lane_width_m / 2.0 + 0.1
    clutter = []
    tries = 0
    while len(clutter) < nuisance.n_clutter and tries < 60:
        tries += 1
        cx, cy = rng.uniform(bx0, bx1), rng.uniform(by0, by1)
        length = rng.uniform(*CLUTTER_LENGTH_M)
        ang = rng.uniform(0.0, math.pi)
        ex, ey = 0.5 * length * math.cos(ang), 0.5 * length * math.sin(ang)
        seg = np.array([cx - ex, cy - ey, cx + ex, cy + ey])
        probe = np.linspace(seg[:2], seg[2:], 7)
        d = np.hypot(
            probe[:, None, 0] - centreline[None, :, 0], probe[:, None, 1] - centreline[None, :, 1]
        )
        if d.min() > clearance:
            clutter.append(seg)
    if clutter:
        out = np.vstack([out, np.array(clutter)])
    return out


def perturb_ranges(
    ranges: NDArray[np.float64],
    laser_bearings: NDArray[np.float64],
    mount_yaw_rad: float,
    nuisance: Nuisance,
    rng: np.random.Generator,
) -> NDArray[np.float64]:
    """Range noise, per-beam dropout and the dead sector applied to cast ranges (inf stays inf).
    Dropped beams become NaN, or +inf for a share of them, like the real driver's returns."""
    out = np.array(ranges, dtype=np.float64)
    finite = np.isfinite(out)
    if nuisance.range_sigma_m > 0.0:
        out[finite] += rng.normal(0.0, nuisance.range_sigma_m, size=int(finite.sum()))
    drop = rng.uniform(size=out.shape) < nuisance.dropout
    if nuisance.dead_width_rad > 0.0:
        vehicle_bearing = laser_bearings + mount_yaw_rad
        delta = np.abs(
            np.mod(vehicle_bearing - nuisance.dead_centre_rad + math.pi, 2 * math.pi) - math.pi
        )
        drop |= delta <= nuisance.dead_width_rad / 2.0
    as_inf = rng.uniform(size=out.shape) < nuisance.inf_share
    out[drop & as_inf] = np.inf
    out[drop & ~as_inf] = np.nan
    return out


# ---------------------------------------------------------------------------------------------
# LiDAR-model randomisation (trackperc-v3, `dataset --lidar-dr`). Off by default: a sample drawn
# without it consumes no random numbers here, so every earlier dataset stays reproducible.
# ---------------------------------------------------------------------------------------------

MOUNT_YAW_ERROR_RAD = math.radians(1.5)
MOUNT_XY_ERROR_M = 0.015
RANGE_SCALE_ERROR = 0.015
RANGE_BIAS_M = 0.02
FAR_DROPOUT_MAX = 0.4
FAR_DROPOUT_PROBABILITY = 0.5
FAR_DROPOUT_RANGE_M = 6.0
DARK_PROBABILITY = 0.4
MAX_DARK_SECTORS = 3
DARK_WIDTH_RAD = (math.radians(5.0), math.radians(35.0))
DARK_RANGE_M = (0.8, 3.0)


@dataclass(frozen=True)
class LidarModel:
    """What a different unit, a different mount or a different room does to the LiDAR: the scan
    is cast from a mount that is off by (yaw, x, y) from the one preprocessing assumes, ranges are
    scaled and biased, far returns drop out more (quadratic in range up to `far_dropout` at 6 m),
    and dark sectors (low-reflectivity walls) return nothing beyond their range."""

    yaw_error_rad: float
    x_error_m: float
    y_error_m: float
    range_scale: float
    range_bias_m: float
    far_dropout: float
    # (vehicle bearing of the centre, width, range beyond which nothing returns)
    dark_sectors: tuple[tuple[float, float, float], ...]


IDEAL_LIDAR = LidarModel(0.0, 0.0, 0.0, 1.0, 0.0, 0.0, ())


def draw_lidar_model(rng: np.random.Generator) -> LidarModel:
    n_dark = int(rng.integers(1, MAX_DARK_SECTORS + 1)) if rng.uniform() < DARK_PROBABILITY else 0
    return LidarModel(
        yaw_error_rad=float(rng.uniform(-MOUNT_YAW_ERROR_RAD, MOUNT_YAW_ERROR_RAD)),
        x_error_m=float(rng.uniform(-MOUNT_XY_ERROR_M, MOUNT_XY_ERROR_M)),
        y_error_m=float(rng.uniform(-MOUNT_XY_ERROR_M, MOUNT_XY_ERROR_M)),
        range_scale=1.0 + float(rng.uniform(-RANGE_SCALE_ERROR, RANGE_SCALE_ERROR)),
        range_bias_m=float(rng.uniform(-RANGE_BIAS_M, RANGE_BIAS_M)),
        far_dropout=(
            float(rng.uniform(0.0, FAR_DROPOUT_MAX))
            if rng.uniform() < FAR_DROPOUT_PROBABILITY
            else 0.0
        ),
        dark_sectors=tuple(
            (
                float(rng.uniform(-math.pi, math.pi)),
                float(rng.uniform(*DARK_WIDTH_RAD)),
                float(rng.uniform(*DARK_RANGE_M)),
            )
            for _ in range(n_dark)
        ),
    )


def apply_lidar_model(
    ranges: NDArray[np.float64],
    vehicle_bearings: NDArray[np.float64],
    model: LidarModel,
    rng: np.random.Generator,
) -> NDArray[np.float64]:
    """Scale, bias, far dropout and dark sectors on cast ranges (NaN and inf stay as they are; a
    lost return becomes NaN). IDEAL_LIDAR returns the ranges unchanged and draws nothing."""
    out = np.array(ranges, dtype=np.float64)
    if model == IDEAL_LIDAR:
        return out
    finite = np.isfinite(out)
    out[finite] = out[finite] * model.range_scale + model.range_bias_m
    lost = np.zeros(out.shape, dtype=bool)
    if model.far_dropout > 0.0:
        frac = np.clip(np.where(finite, out, 0.0) / FAR_DROPOUT_RANGE_M, 0.0, 1.0)
        lost |= finite & (rng.uniform(size=out.shape) < model.far_dropout * frac * frac)
    for centre, width, reach in model.dark_sectors:
        delta = np.abs(np.mod(vehicle_bearings - centre + math.pi, 2 * math.pi) - math.pi)
        lost |= finite & (delta <= width / 2.0) & (out > reach)
    out[lost] = np.nan
    return out
