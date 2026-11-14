"""Isotropic scale augmentation of real self-label samples (brief 32, design 15.5, pre-registration
v0.3 section 3.1).

One real layout gives one lane width. Scaling a scan and its label together about the LiDAR
position by a factor k in [0.75, 1.4] (the range is read from `evaluation/prereg/pnp-v0.3.yaml`,
never typed here) shows the net the same scene as a lane k times as wide, and so teaches it to
read the width from the scan instead of remembering the one it saw.

Scan. The sample is the net's input tensor: channel 0 the min-pooled range over the clip, channel
1 validity. The body filter was applied when the tensor was built (at the original scale), so a
bin that was blanked by the body filter stays blank. Ranges are multiplied by k (scaling about the
LiDAR position leaves every bearing as it is), then the clip and the valid channel are recomputed:

* a bin whose scaled range is under the LiDAR's minimum or over its maximum range is no return
  (`vehicle_params` `sensors.lidar_spec.range_min_m` and `range_max_m`);
* a bin that was clipped (a return of 6 m or more, its range unknown) stays a clipped return for
  k at least 1, since its scaled range is still 6 m or more. For k under 1 its scaled range may
  be under 6 m and is not known, so the bin is made no return rather than invent a range.

Label. The 16 stations (x, y, half widths) are scaled about the same point (the LiDAR position in
base_link), `log_sigma` shifts by log k, the polyline is re-anchored at the point nearest the
base_link origin (4.1: station 0 is the centreline point nearest the car; when the scaled polyline
starts ahead of the origin its first point is that point), resampled to 0.20 m of arc, and the
stations that fall off the end of the scaled label are masked through `valid_count`
(`train.station_weights`), so they never enter the loss. Their values are the last valid row, so
nothing past the end is NaN or inf.

Sim samples are not scaled (the generator already varies width). The mirror and the bin dropout of
`real.augment_real` apply after the scaling.
"""

from __future__ import annotations

import math

import numpy as np
from numpy.typing import NDArray
from racer_track_perception import track as tr
from racer_track_perception.preprocess import RANGE_CLIP_M
from racer_track_perception.vehicle import load_generated_params

from racer_pnp_perception import prereg

# A bin at the clip is a return of RANGE_CLIP_M or more. Tensors are float32 and the clip is a
# hard min, so a clipped bin is exactly 1.0; the tolerance covers a dequantised dataset tensor.
CLIPPED = 1.0 - 1e-6
_EPS = 1e-9


def lidar_range_limits() -> tuple[float, float]:
    """(range_min_m, range_max_m) of the fitted head, from the generated vehicle_params binding."""
    spec = load_generated_params().sensors.lidar_spec
    return float(spec.range_min_m), float(spec.range_max_m)


def lidar_position() -> tuple[float, float]:
    """The LiDAR head in base_link (x, y), from the generated vehicle_params binding."""
    lidar = load_generated_params().sensors.lidar
    return float(lidar.mount_x_m), float(lidar.mount_y_m)


def draw_scales(
    rng: np.random.Generator, n: int, scale_range: tuple[float, float] | None = None
) -> NDArray[np.float64]:
    """`n` factors drawn uniformly from the pre-registered range."""
    lo, hi = scale_range or prereg.scale_augmentation_range()
    return rng.uniform(lo, hi, size=n)


def scale_tensor(
    tensor: NDArray[np.floating],
    k: NDArray[np.floating] | float,
    range_limits_m: tuple[float, float] | None = None,
) -> NDArray[np.float32]:
    """The scan tensor(s) [n, 2, 360] of the same scene scaled by `k` (a number or one per sample)
    about the LiDAR position."""
    x = np.asarray(tensor, dtype=np.float32)
    kk = np.broadcast_to(np.asarray(k, dtype=np.float64), (len(x),))[:, None]
    r_min, r_max = range_limits_m or lidar_range_limits()
    valid = x[:, 1, :] > 0.5
    clipped = valid & (x[:, 0, :] >= CLIPPED)
    r = x[:, 0, :].astype(np.float64) * RANGE_CLIP_M * kk
    keep = valid & ~(clipped & (kk < 1.0))
    keep &= clipped | ((r >= r_min) & (r <= r_max))
    out = np.empty_like(x)
    out[:, 0, :] = np.where(keep, np.minimum(r, RANGE_CLIP_M) / RANGE_CLIP_M, 1.0)
    out[:, 1, :] = keep.astype(np.float32)
    return out


def scale_label(
    label: NDArray[np.floating],
    valid_count: int,
    k: float,
    lidar_xy: tuple[float, float] | None = None,
) -> tuple[NDArray[np.float32], int]:
    """One label [16, 5] (x, y, half_width_left, half_width_right, log_sigma) scaled by `k` about
    the LiDAR position, re-anchored at the point nearest the base_link origin and resampled to
    0.20 m of arc. Returns the label and its new valid_count (the stations that fall off the end of
    the scaled label are masked by it; their rows repeat the last valid one)."""
    lx, ly = lidar_xy or lidar_position()
    n = int(min(max(valid_count, 0), tr.K_STATIONS))
    lab = np.asarray(label, dtype=np.float64)
    if n < 2:
        return np.array(label, dtype=np.float32), n
    x = lx + k * (lab[:n, 0] - lx)
    y = ly + k * (lab[:n, 1] - ly)
    hl, hr = k * lab[:n, 2], k * lab[:n, 3]
    ls = lab[:n, 4] + math.log(k)
    keep = np.concatenate([[True], np.hypot(np.diff(x), np.diff(y)) > _EPS])
    x, y, hl, hr, ls = x[keep], y[keep], hl[keep], hr[keep], ls[keep]
    if len(x) < 2:
        return np.array(label, dtype=np.float32), n
    ex, ey = np.diff(x), np.diff(y)
    seg = np.hypot(ex, ey)
    s = np.concatenate([[0.0], np.cumsum(seg)])
    total = float(s[-1])
    # The point of the polyline nearest the origin (a clamp at the start when it lies ahead).
    t_proj = np.clip(-(x[:-1] * ex + y[:-1] * ey) / (seg * seg), 0.0, 1.0)
    px, py = x[:-1] + t_proj * ex, y[:-1] + t_proj * ey
    best = int(np.argmin(px * px + py * py))
    s0 = float(s[best] + t_proj[best] * seg[best])
    n_new = min(math.floor((total - s0) / tr.STATION_SPACING_M + _EPS) + 1, tr.K_STATIONS)
    n_new = max(n_new, 1)
    targets = np.minimum(s0 + np.arange(n_new) * tr.STATION_SPACING_M, total)
    out = np.zeros((tr.K_STATIONS, 5), dtype=np.float64)
    for col, values in enumerate((x, y, hl, hr, ls)):
        out[:n_new, col] = np.interp(targets, s, values)
    out[n_new:] = out[n_new - 1]
    return out.astype(np.float32), n_new


def scale_real(
    tensor: NDArray[np.floating],
    label: NDArray[np.floating],
    valid_count: NDArray[np.integer],
    rng: np.random.Generator,
    scale_range: tuple[float, float] | None = None,
    probability: float = 1.0,
    range_limits_m: tuple[float, float] | None = None,
    lidar_xy: tuple[float, float] | None = None,
) -> tuple[NDArray[np.float32], NDArray[np.float32], NDArray[np.int64], NDArray[np.float64]]:
    """Scale each real sample (tensor [n, 2, 360], label [n, 16, 5], valid_count [n]) by a factor
    drawn uniformly from the pre-registered range, with probability `probability` (1.0: every
    sample; an unscaled sample has factor 1.0). Returns copies and the factors used."""
    n = len(tensor)
    k = draw_scales(rng, n, scale_range)
    if probability < 1.0:
        k = np.where(rng.uniform(size=n) < probability, k, 1.0)
    lidar_xy = lidar_xy or lidar_position()
    range_limits_m = range_limits_m or lidar_range_limits()
    x = scale_tensor(tensor, k, range_limits_m)
    y = np.empty((n, tr.K_STATIONS, 5), dtype=np.float32)
    v = np.empty(n, dtype=np.int64)
    for i in range(n):
        y[i], v[i] = scale_label(label[i], int(valid_count[i]), float(k[i]), lidar_xy)
    return x, y, v, k
