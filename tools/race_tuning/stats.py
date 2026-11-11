"""Small statistics for the harness: bootstrap CI of a mean, Wilson interval of a proportion,
and the paired ratio of two candidates over the same runs. numpy only, seeded, no global state."""

from __future__ import annotations

import math
from collections.abc import Sequence

import numpy as np


def mean_ci(
    values: Sequence[float], confidence: float = 0.95, resamples: int = 4000, seed: int = 0
) -> tuple[float, float, float]:
    """(mean, low, high): percentile bootstrap of the mean. One value gives a zero-width
    interval; no values give NaNs."""
    if len(values) == 0:
        return (math.nan, math.nan, math.nan)
    x = np.asarray(values, dtype=float)
    mean = float(x.mean())
    if len(x) == 1:
        return (mean, mean, mean)
    rng = np.random.default_rng(seed)
    draws = x[rng.integers(0, len(x), size=(resamples, len(x)))].mean(axis=1)
    alpha = (1.0 - confidence) / 2.0
    low, high = np.quantile(draws, [alpha, 1.0 - alpha])
    return (mean, float(low), float(high))


def wilson_interval(successes: int, trials: int, z: float = 1.959964) -> tuple[float, float]:
    """Wilson score interval of a proportion (the right interval at 0 or all successes). With no
    trials the whole [0, 1]."""
    if trials <= 0:
        return (0.0, 1.0)
    p = successes / trials
    denom = 1.0 + z * z / trials
    centre = (p + z * z / (2.0 * trials)) / denom
    half = z * math.sqrt(p * (1.0 - p) / trials + z * z / (4.0 * trials * trials)) / denom
    return (max(0.0, centre - half), min(1.0, centre + half))


def paired_ratio_ci(
    a: Sequence[float], b: Sequence[float], confidence: float = 0.95, seed: int = 0
) -> tuple[float, float, float]:
    """Mean of a_i / b_i with a bootstrap interval, over pairs (a_i, b_i) from the same run.
    Pairs with a non-positive or non-finite denominator are dropped."""
    if len(a) != len(b):
        raise ValueError("paired samples must have the same length")
    ratios = [x / y for x, y in zip(a, b, strict=True) if y > 0 and math.isfinite(x / y)]
    return mean_ci(ratios, confidence=confidence, seed=seed)
