"""Metric C of docs/notes/preregistration-pnp-v0.3.md section 4, and the paired block bootstrap.

C: for each scan of the fast run (the scans asked for, e.g. the fast corner scans), the best ICP
match among the scans of the slow run, its RMS kept; C of a set is the median of those RMS. "Best"
is the lowest RMS among the slow scans whose ICP started at the identity ends with an inlier
share (points within 5 cm of the other scan) of at least `INLIER_SHARE_MIN`; a fast scan with no
such slow scan is not scored (and is counted). The ICP is the one of tools/safety_replay/motion.py
(point to point, brute force, the car's own static returns masked, RMS capped at 5 cm per point),
the matcher tools/yaw_rate/matching.py is built on. Both runs get the same processing.
"""

from __future__ import annotations

import multiprocessing
from collections.abc import Sequence
from dataclasses import dataclass
from typing import Any

import numpy as np
from follower_replay.bag import ScanRecord
from numpy.typing import NDArray
from safety_replay import motion
from yaw_rate import matching

# Brief 33, step "Metric C": "lowest RMS with inlier share at least 0.6".
INLIER_SHARE_MIN = 0.6
# Pre-registration v0.3 section 4 and evaluation/prereg/pnp-v0.3.yaml block deskew.
BOOTSTRAP_BLOCK_S = 2.0
BOOTSTRAP_RESAMPLES = 2000
BOOTSTRAP_SEED = 0
BOOTSTRAP_ALPHA = 0.05  # 95 percent percentile interval, as the v0.2 prereg

_WORK: dict[str, Any] = {}


def points_of(
    ranges: NDArray[np.floating],
    meta: tuple[float, float, float, float],
    mount: motion.Mount,
    ignore: frozenset[tuple[int, int]] | None,
) -> NDArray[np.float64]:
    angle_min, angle_increment, range_min, range_max = meta
    scan = ScanRecord(
        stamp_s=0.0,
        angle_min=angle_min,
        angle_increment=angle_increment,
        range_min=range_min,
        range_max=range_max,
        ranges=tuple(float(r) for r in ranges),
    )
    return motion.scan_points(scan, mount, matching.MIN_RANGE_M, matching.MAX_RANGE_M, ignore)


def _nearest(points: NDArray[np.float64], ref: NDArray[np.float64]) -> NDArray[np.float64]:
    return matching._nearest_distance(points, ref)


def icp_kdtree(
    source: NDArray[np.float64],
    target: NDArray[np.float64],
    tree: Any,
    iterations: int = 40,
    inlier_m: float = 0.05,
) -> tuple[NDArray[np.float64], NDArray[np.float64], float, float]:
    """tools/safety_replay/motion.icp step for step (same gating, same Kabsch update, same stop,
    same RMS capped at inlier_m), with the nearest neighbours from a scipy cKDTree of `target`
    (exact, so the result equals motion.icp's up to float rounding; test_pnp_deskew_metric.py
    compares them). Returns R, t, the RMS, and the inlier share of the final fit."""
    if len(source) < 10 or len(target) < 10:
        raise ValueError("ICP needs at least 10 points in each scan")
    rotation = np.eye(2)
    translation = np.zeros(2)
    for k in range(iterations):
        moved = source @ rotation.T + translation
        d, j = tree.query(moved)
        threshold = max(inlier_m, float(np.percentile(d, 80))) if k < 10 else inlier_m
        keep = d < threshold
        if keep.sum() < 10:
            break
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
        if float(np.hypot(*shift)) < 1e-6:
            break
    d, _ = tree.query(source @ rotation.T + translation)
    rms = float(np.sqrt(np.mean(np.minimum(d, inlier_m) ** 2)))
    return rotation, translation, rms, float(np.mean(d < inlier_m))


def _best_for(k: int) -> tuple[float, int]:
    fast = _WORK["fast"][k]
    best = np.nan
    converged = 0
    if len(fast) < 10:
        return best, converged
    for slow, tree in zip(_WORK["slow"], _WORK["trees"], strict=True):
        if tree is None:
            continue
        try:
            _r, _t, rms, share = icp_kdtree(fast, slow, tree, inlier_m=matching.INLIER_M)
        except ValueError:
            continue
        if share >= INLIER_SHARE_MIN:
            converged += 1
            if not np.isfinite(best) or rms < best:
                best = rms
    return best, converged


def best_match_rms(
    fast_points: Sequence[NDArray[np.float64]],
    slow_points: Sequence[NDArray[np.float64]],
    workers: int = 1,
) -> tuple[NDArray[np.float64], NDArray[np.int64]]:
    """(RMS of the best slow match per fast scan, NaN when none; number of slow scans whose ICP
    converged per fast scan)."""
    from scipy.spatial import cKDTree

    slow = list(slow_points)
    trees = [cKDTree(p) if len(p) >= 10 else None for p in slow]
    _WORK.update(fast=list(fast_points), slow=slow, trees=trees)
    jobs = range(len(fast_points))
    if workers > 1:
        with multiprocessing.get_context("fork").Pool(workers) as pool:
            res = pool.map(_best_for, jobs, chunksize=4)
    else:
        res = [_best_for(k) for k in jobs]
    arr = np.array(res, dtype=np.float64).reshape(-1, 2)
    return arr[:, 0], arr[:, 1].astype(np.int64)


def median_ignoring_nan(values: NDArray[np.float64]) -> float:
    v = np.asarray(values, dtype=np.float64)
    v = v[np.isfinite(v)]
    return float(np.median(v)) if len(v) else float("nan")


@dataclass
class RatioInterval:
    ratio: float
    lo: float
    hi: float
    blocks: int
    scans: int


def block_bootstrap_median_ratio(
    numerator: NDArray[np.float64],
    denominator: NDArray[np.float64],
    block: NDArray[np.int64],
    n_resamples: int = BOOTSTRAP_RESAMPLES,
    seed: int = BOOTSTRAP_SEED,
    alpha: float = BOOTSTRAP_ALPHA,
) -> RatioInterval:
    """Paired block bootstrap of median(numerator) / median(denominator). The arrays are per
    scan and paired (only scans finite in both count). Whole blocks are resampled with
    replacement, the same blocks for both; the ratio of the two medians is taken per resample."""
    num = np.asarray(numerator, dtype=np.float64)
    den = np.asarray(denominator, dtype=np.float64)
    ok = np.isfinite(num) & np.isfinite(den)
    num, den, blk = num[ok], den[ok], np.asarray(block)[ok]
    ids, inverse = np.unique(blk, return_inverse=True)
    n_blocks = len(ids)
    if n_blocks == 0:
        return RatioInterval(np.nan, np.nan, np.nan, 0, 0)
    by_block = [np.nonzero(inverse == b)[0] for b in range(n_blocks)]
    point = float(np.median(num) / np.median(den))
    rng = np.random.default_rng(seed)
    draws = rng.integers(0, n_blocks, size=(n_resamples, n_blocks))
    ratios = np.empty(n_resamples)
    for r in range(n_resamples):
        rows = np.concatenate([by_block[b] for b in draws[r]])
        ratios[r] = np.median(num[rows]) / np.median(den[rows])
    ratios = ratios[np.isfinite(ratios)]
    lo = float(np.percentile(ratios, 100.0 * alpha / 2.0)) if len(ratios) else np.nan
    hi = float(np.percentile(ratios, 100.0 * (1.0 - alpha / 2.0))) if len(ratios) else np.nan
    return RatioInterval(point, lo, hi, n_blocks, len(num))
