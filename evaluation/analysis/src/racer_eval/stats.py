"""Interval statistics of the pre-registration: t intervals (sections 5.2, 5.3), Wilson intervals
for rates (5.2, 5.3), the paired block bootstrap of a ratio (sections 2 and 3) and the macro
averaged RMSE with its stratum merge rule (section 3)."""

from __future__ import annotations

from collections.abc import Callable, Sequence
from dataclasses import dataclass, field
from statistics import NormalDist

import numpy as np
from numpy.typing import ArrayLike, NDArray
from scipy import stats as _scipy_stats


@dataclass(frozen=True)
class Interval:
    estimate: float
    lo: float
    hi: float
    level: float = 0.95

    def contains(self, value: float) -> bool:
        return self.lo <= value <= self.hi


def _check_level(level: float) -> None:
    if not 0.0 < level < 1.0:
        raise ValueError(f"level must be in (0, 1), got {level}")


def t_interval(x: ArrayLike, level: float = 0.95) -> Interval:
    """Mean of `x` and its two sided t interval (n - 1 degrees of freedom). Needs n >= 2."""
    _check_level(level)
    a = np.asarray(x, dtype=np.float64)
    if a.ndim != 1 or len(a) < 2:
        raise ValueError("a t interval needs a 1-D sample of at least 2 values")
    mean = float(a.mean())
    half = float(
        _scipy_stats.t.ppf(0.5 + level / 2.0, len(a) - 1) * a.std(ddof=1) / np.sqrt(len(a))
    )
    return Interval(mean, mean - half, mean + half, level)


def paired_t_interval(a: ArrayLike, b: ArrayLike, level: float = 0.95) -> Interval:
    """t interval of the mean of a - b over pairs."""
    x, y = np.asarray(a, dtype=np.float64), np.asarray(b, dtype=np.float64)
    if x.shape != y.shape:
        raise ValueError("paired samples must have the same length")
    return t_interval(x - y, level)


def wilson_interval(k: int, n: int, level: float = 0.95) -> Interval:
    """Wilson score interval of the rate k / n, clipped to [0, 1]."""
    _check_level(level)
    if n <= 0 or k < 0 or k > n:
        raise ValueError(f"need 0 <= k <= n and n > 0, got k={k}, n={n}")
    z = NormalDist().inv_cdf(0.5 + level / 2.0)
    p = k / n
    denom = 1.0 + z * z / n
    centre = (p + z * z / (2.0 * n)) / denom
    half = z * np.sqrt(p * (1.0 - p) / n + z * z / (4.0 * n * n)) / denom
    # clip to [0, 1] and, against rounding at k = 0 and k = n, keep the rate inside
    lo, hi = max(0.0, float(centre - half)), min(1.0, float(centre + half))
    return Interval(p, min(lo, p), max(hi, p), level)


def rmse(values: ArrayLike) -> float:
    """Root mean square of `values`; the aggregate for block_bootstrap_ratio on errors."""
    a = np.asarray(values, dtype=np.float64)
    return float(np.sqrt(np.mean(a * a)))


def block_bootstrap_ratio(
    num: ArrayLike,
    den: ArrayLike,
    block_ids: ArrayLike,
    n_resamples: int = 2000,
    seed: int = 0,
    aggregate: Callable[[NDArray[np.float64]], float] = np.mean,
    level: float = 0.95,
) -> Interval:
    """Paired block bootstrap of aggregate(num) / aggregate(den).

    `num[i]` and `den[i]` are the per-sample values of the two models on the SAME sample i, and
    `block_ids[i]` names its block (consecutive samples of a segment, 2.0 s or 5.0 s long). Each
    resample draws blocks with replacement, recomputes BOTH aggregates on the drawn blocks and
    takes their ratio; the interval is the percentile interval of those ratios. The estimate is
    the ratio on the full data. `aggregate` is np.mean by default; pass `rmse` for RMSE ratios.
    """
    _check_level(level)
    x, y = np.asarray(num, dtype=np.float64), np.asarray(den, dtype=np.float64)
    ids = np.asarray(block_ids)
    if not (x.shape == y.shape == ids.shape) or x.ndim != 1 or len(x) == 0:
        raise ValueError("num, den and block_ids must be 1-D, non-empty and the same length")
    if n_resamples < 1:
        raise ValueError("n_resamples must be at least 1")
    _, inverse = np.unique(ids, return_inverse=True)
    blocks = [np.nonzero(inverse == b)[0] for b in range(inverse.max() + 1)]
    rng = np.random.default_rng(seed)
    ratios = np.empty(n_resamples)
    for i in range(n_resamples):
        picked = rng.integers(0, len(blocks), size=len(blocks))
        idx = np.concatenate([blocks[j] for j in picked])
        ratios[i] = aggregate(x[idx]) / aggregate(y[idx])
    alpha = (1.0 - level) / 2.0
    lo, hi = np.quantile(ratios, [alpha, 1.0 - alpha])
    return Interval(float(aggregate(x) / aggregate(y)), float(lo), float(hi), level)


@dataclass(frozen=True)
class MacroGroup:
    """One stratum after merging: its label (the lowest bin of the merged ones), the original
    strata it pools, its sample count and its pooled RMSE."""

    label: str
    members: tuple[str, ...]
    count: int
    rmse: float


@dataclass(frozen=True)
class MacroRmse:
    value: float
    groups: tuple[MacroGroup, ...]
    #: (stratum, stratum it was merged into) for every merge, in the order they happened.
    merged: tuple[tuple[str, str], ...] = field(default_factory=tuple)
    #: Lowest bins of a chain still under min_count. Nothing lower exists to merge into, so the
    #: rule of section 3 cannot be applied; they are kept as they are and listed here.
    undersized: tuple[str, ...] = field(default_factory=tuple)


def macro_rmse(
    errors: ArrayLike,
    strata: Sequence[str],
    min_count: int = 30,
    merge_order: Sequence[Sequence[str]] = (),
) -> MacroRmse:
    """Unweighted mean over strata of the per-stratum RMSE (pre-registration section 3).

    `merge_order` is one chain per direction, each listing that direction's strata from the
    LOWEST start-speed bin to the highest. A stratum with fewer than `min_count` samples is
    merged into the next lower bin of its chain (and that one into the next if it is still too
    small); the merges are returned. Every label in `strata` must be in one chain.
    """
    e = np.asarray(errors, dtype=np.float64)
    labels = np.asarray(list(strata))
    if e.ndim != 1 or e.shape != labels.shape:
        raise ValueError("errors and strata must be 1-D and the same length")
    known = {s for chain in merge_order for s in chain}
    unknown = sorted(set(labels.tolist()) - known)
    if unknown:
        raise ValueError(f"strata {unknown} are in no merge_order chain")
    groups: list[MacroGroup] = []
    merged: list[tuple[str, str]] = []
    undersized: list[str] = []
    for chain in merge_order:
        pool: list[NDArray[np.float64]] = []
        members: list[str] = []
        for i in range(len(chain) - 1, -1, -1):
            stratum = chain[i]
            values = e[labels == stratum]
            if len(values) > 0:
                pool.append(values)
                members.append(stratum)
            count = sum(len(v) for v in pool)
            if count == 0:
                continue
            if count >= min_count or i == 0:
                allv = np.concatenate(pool)
                if count < min_count:
                    undersized.append(stratum)
                groups.append(MacroGroup(stratum, tuple(members), count, rmse(allv)))
                merged.extend((m, stratum) for m in members if m != stratum)
                pool, members = [], []
    if not groups:
        raise ValueError("no samples")
    return MacroRmse(
        float(np.mean([g.rmse for g in groups])), tuple(groups), tuple(merged), tuple(undersized)
    )
