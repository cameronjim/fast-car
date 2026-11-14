"""2D scan matching for the start-scan lap gate. Numpy only, no ROS.

Vendored from `tools/safety_replay/motion.py` (point-to-point ICP with brute-force nearest
neighbours): a ROS package cannot import from `tools/`. Differences from the original: the ICP
starts from a given pose instead of the identity, the matched scans are voxel-thinned so a match
costs a few milliseconds, and the result carries the inlier share besides the RMS.

Frames: `base_link`, x forward, y left (REP-103). The LiDAR sits at (mount.x, mount.y) with
mount yaw `mount.yaw` (pi on the car), as in vehicle_params; the node passes the numbers from the
generated binding, nothing here is a physical constant.

`match_scans(source, target, seed)` finds the pose of the source scan's frame in the target
scan's frame: R @ source + t ~ target, returned as (x, y, yaw). With the target the start scan and
the source the current scan, (x, y, yaw) is where the car is relative to the start pose, x
along the start heading.
"""

from __future__ import annotations

import math
from dataclasses import dataclass
from typing import Any

import numpy as np
from numpy.typing import NDArray

FloatArray = NDArray[np.float64]

#: Same margin as racer_track_perception.preprocess.BODY_FILTER_MARGIN_M.
BODY_FILTER_MARGIN_M = 0.02
#: Returns closer than this to the LiDAR head are the car's own parts (motion.py uses 0.12 m).
MIN_RANGE_M = 0.12
#: Beyond this the returns are sparse and noisy: the walls of a 1 m lane are well inside.
MAX_RANGE_M = 4.0
#: Voxel size of the thinning, metres.
VOXEL_M = 0.04
MAX_POINTS = 220
#: The final RMS and inlier share are scored on denser sets than the ICP iterates on: with one
#: point per 4 cm voxel the nearest-neighbour distance of a perfect alignment is already 1 to 2 cm.
DENSE_VOXEL_M = 0.015
DENSE_MAX_POINTS = 420
MIN_POINTS = 12


@dataclass(frozen=True)
class Mount:
    """LiDAR head pose in base_link."""

    x_m: float
    y_m: float
    yaw_rad: float


@dataclass(frozen=True)
class Body:
    """Body rectangle in base_link: x from -rear_overhang_m to wheelbase_m + front_overhang_m,
    |y| up to half_width_m."""

    x_min_m: float
    x_max_m: float
    half_width_m: float


@dataclass(frozen=True)
class MatchResult:
    x_m: float
    y_m: float
    yaw_rad: float
    rms_m: float  # RMS distance of the inliers
    inlier_share: float  # share of the source points with a target point within inlier_m
    n_source: int
    #: Larger of the two free-space violation shares (see `freespace_violation_share`); the gate
    #: fills it in for the match it keeps, 0.0 when nobody measured it.
    freespace_violation: float = 0.0


def wrap(a: float) -> float:
    return (a + math.pi) % (2.0 * math.pi) - math.pi


def scan_to_points(
    ranges: Any,
    angle_min: float,
    angle_increment: float,
    range_min: float,
    range_max: float,
    mount: Mount,
    body: Body,
    max_range_m: float = MAX_RANGE_M,
) -> FloatArray:
    """Valid beams of a scan as (N, 2) base_link points, the car's body filtered out the way
    racer_track_perception.preprocess does (rectangle grown by 0.02 m), within
    [MIN_RANGE_M, max_range_m] of the LiDAR head. Never raises on NaN, inf or empty scans."""
    r = np.asarray(ranges, dtype=np.float64).reshape(-1)
    if r.size == 0 or not (math.isfinite(angle_min) and math.isfinite(angle_increment)):
        return np.zeros((0, 2))
    with np.errstate(invalid="ignore"):
        ok = np.isfinite(r) & (r > 0.0) & (r >= MIN_RANGE_M) & (r <= max_range_m)
        if math.isfinite(range_min):
            ok &= r >= range_min
        if math.isfinite(range_max):
            ok &= r <= range_max
    idx = np.nonzero(ok)[0]
    if idx.size == 0:
        return np.zeros((0, 2))
    r = r[idx]
    bearing = angle_min + idx.astype(np.float64) * angle_increment + mount.yaw_rad
    x = mount.x_m + r * np.cos(bearing)
    y = mount.y_m + r * np.sin(bearing)
    m = BODY_FILTER_MARGIN_M
    inside = (
        (x >= body.x_min_m - m) & (x <= body.x_max_m + m) & (np.abs(y) <= body.half_width_m + m)
    )
    return np.stack([x[~inside], y[~inside]], axis=1)


def thin(points: FloatArray, voxel_m: float = VOXEL_M, max_points: int = MAX_POINTS) -> FloatArray:
    """One point per voxel (the first), then at most `max_points` evenly spaced in bearing order.
    Deterministic."""
    if points.shape[0] == 0:
        return points
    key = np.round(points / voxel_m).astype(np.int64)
    _, first = np.unique(key[:, 0] * 100003 + key[:, 1], return_index=True)
    out = points[np.sort(first)]
    if out.shape[0] > max_points:
        out = out[np.linspace(0, out.shape[0] - 1, max_points).astype(np.int64)]
    return out


def _nearest(points: FloatArray, reference: FloatArray) -> tuple[FloatArray, NDArray[np.int64]]:
    # |a - b|^2 = |a|^2 + |b|^2 - 2 a.b : one matrix product instead of an (N, M, 2) temporary.
    d2 = (
        (points * points).sum(axis=1)[:, None]
        + (reference * reference).sum(axis=1)[None, :]
        - 2.0 * (points @ reference.T)
    )
    j = d2.argmin(axis=1)
    return np.sqrt(np.maximum(d2[np.arange(points.shape[0]), j], 0.0)), j


def match_scans(
    source: FloatArray,
    target: FloatArray,
    seed: tuple[float, float, float] = (0.0, 0.0, 0.0),
    iterations: int = 30,
    inlier_m: float = 0.05,
    score_source: FloatArray | None = None,
    score_target: FloatArray | None = None,
) -> MatchResult | None:
    """Point-to-point ICP of `source` onto `target` from the pose `seed` (x, y, yaw). Both are
    (N, 2) arrays already thinned. None when either has fewer than MIN_POINTS points or the
    iteration lost all its inliers. `score_source` and `score_target`, when given, are denser
    versions of the same two scans: the returned RMS and inlier share are measured on them."""
    if source.shape[0] < MIN_POINTS or target.shape[0] < MIN_POINTS:
        return None
    c, s = math.cos(seed[2]), math.sin(seed[2])
    rotation = np.array([[c, -s], [s, c]])
    translation = np.array([seed[0], seed[1]], dtype=np.float64)
    for k in range(iterations):
        moved = source @ rotation.T + translation
        d, j = _nearest(moved, target)
        # Loose gate first (the seed can be tens of centimetres off), then the inlier band.
        threshold = max(inlier_m, float(np.percentile(d, 80))) if k < 8 else inlier_m
        keep = d < threshold
        if int(keep.sum()) < MIN_POINTS:
            return None
        a = moved[keep]
        b = target[j[keep]]
        ca = a.mean(axis=0)
        cb = b.mean(axis=0)
        u, _, vt = np.linalg.svd((a - ca).T @ (b - cb))
        step = vt.T @ u.T
        if np.linalg.det(step) < 0:
            vt[1] *= -1
            step = vt.T @ u.T
        shift = cb - step @ ca
        rotation = step @ rotation
        translation = step @ translation + shift
        if (
            float(np.hypot(shift[0], shift[1])) < 1e-5
            and abs(math.atan2(step[1, 0], step[0, 0])) < 1e-6
        ):
            break
    final_source = source if score_source is None else score_source
    final_target = target if score_target is None else score_target
    d, _ = _nearest(final_source @ rotation.T + translation, final_target)
    inliers = d < inlier_m
    n_in = int(inliers.sum())
    if n_in == 0:
        return None
    return MatchResult(
        x_m=float(translation[0]),
        y_m=float(translation[1]),
        yaw_rad=float(math.atan2(rotation[1, 0], rotation[0, 0])),
        rms_m=float(np.sqrt(np.mean(d[inliers] ** 2))),
        inlier_share=n_in / final_source.shape[0],
        n_source=int(final_source.shape[0]),
    )


def surface_normals(points: FloatArray, k: int = 6, radius_m: float = 0.15) -> FloatArray:
    """Unit surface normals of a thinned scan: the direction of least spread of the `k` nearest
    points within `radius_m` (NaN where fewer than 3 neighbours). Deterministic."""
    n = points.shape[0]
    out = np.full((n, 2), np.nan)
    if n < 3:
        return out
    d2 = (
        (points * points).sum(axis=1)[:, None]
        + (points * points).sum(axis=1)[None, :]
        - 2.0 * (points @ points.T)
    )
    order = np.argsort(d2, axis=1)[:, : k + 1]
    for i in range(n):
        idx = order[i]
        near = idx[d2[i, idx] <= radius_m * radius_m]
        if near.shape[0] < 3:
            continue
        q = points[near] - points[near].mean(axis=0)
        _w, v = np.linalg.eigh(q.T @ q)
        out[i] = v[:, 0]  # eigenvalues ascending: the least spread is the surface normal
    return out


def along_support(
    source: FloatArray,
    target: FloatArray,
    normals: FloatArray,
    res: MatchResult,
    inlier_m: float = 0.05,
    min_component: float = 0.5,
) -> int:
    """How many inlier matches pin down the offset along the target frame's x axis: those whose
    target surface normal has |x component| of at least `min_component`. In a straight lane every
    wall normal is across the lane, ICP then keeps whatever x it was seeded with, and this is 0."""
    c, s = math.cos(res.yaw_rad), math.sin(res.yaw_rad)
    moved = source @ np.array([[c, -s], [s, c]]).T + np.array([res.x_m, res.y_m])
    d, j = _nearest(moved, target)
    ok = d < inlier_m
    nx = np.abs(normals[j[ok], 0])
    return int(np.sum(nx >= min_component))


def overlap_share(
    source: FloatArray, target: FloatArray, pose: tuple[float, float, float], inlier_m: float = 0.08
) -> float:
    """Share of `source` points with a target point within `inlier_m` when the source is placed at
    `pose` (x, y, yaw). One nearest-neighbour pass, no iteration: a cheap score to rank seeds."""
    if source.shape[0] == 0 or target.shape[0] == 0:
        return 0.0
    c, s = math.cos(pose[2]), math.sin(pose[2])
    moved = source @ np.array([[c, -s], [s, c]]).T + np.array([pose[0], pose[1]])
    d, _ = _nearest(moved, target)
    return float(np.mean(d < inlier_m))


def polar_profile(
    points: FloatArray,
    origin_xy: tuple[float, float],
    n_bins: int = 360,
    dilate_bins: int = 2,
) -> FloatArray:
    """Nearest return per 1 degree bearing bin around `origin_xy` (the LiDAR position in the frame
    of `points`), then the minimum over +- `dilate_bins` bins (circular). NaN where no bin in the
    window has a return: unknown, never counted as free space."""
    out = np.full(n_bins, np.nan)
    if points.shape[0] == 0 or n_bins <= 0:
        return out
    dx = points[:, 0] - origin_xy[0]
    dy = points[:, 1] - origin_xy[1]
    r = np.hypot(dx, dy)
    k = np.floor((np.arctan2(dy, dx) + math.pi) / (2.0 * math.pi) * n_bins).astype(np.int64)
    k = np.clip(k, 0, n_bins - 1)
    raw = np.full(n_bins, np.inf)
    np.minimum.at(raw, k, r)
    prof = raw.copy()
    for shift in range(1, max(dilate_bins, 0) + 1):
        prof = np.minimum(prof, np.minimum(np.roll(raw, shift), np.roll(raw, -shift)))
    out[np.isfinite(prof)] = prof[np.isfinite(prof)]
    return out


def freespace_violation_share(
    points: FloatArray,
    profile: FloatArray,
    origin_xy: tuple[float, float],
    margin_m: float = 0.15,
    max_range_m: float = 6.0,
) -> float:
    """Share of `points` (already in the profile's frame) that lie more than `margin_m` INSIDE the
    profile's range in their bin: the other scan saw through the place where this scan sees a
    wall. Points in unknown bins or beyond `max_range_m` from `origin_xy` are not counted, in the
    numerator or the denominator. 0.0 when nothing can be counted."""
    n_bins = profile.shape[0]
    if points.shape[0] == 0 or n_bins == 0:
        return 0.0
    dx = points[:, 0] - origin_xy[0]
    dy = points[:, 1] - origin_xy[1]
    r = np.hypot(dx, dy)
    k = np.floor((np.arctan2(dy, dx) + math.pi) / (2.0 * math.pi) * n_bins).astype(np.int64)
    k = np.clip(k, 0, n_bins - 1)
    seen = profile[k]
    counted = np.isfinite(seen) & (r <= max_range_m)
    n = int(counted.sum())
    if n == 0:
        return 0.0
    return float(np.sum(r[counted] < seen[counted] - margin_m)) / n
