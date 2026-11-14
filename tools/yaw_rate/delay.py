"""Delay of a yaw signal against a reference by cross-correlation over interval means.

A signal that LAGS the reference by d seconds has S(t) = R(t - d), so its mean over
[a + d, b + d] equals the reference mean over [a, b]. For each trial lag the signal's interval
means are computed over the shifted intervals and correlated with the reference's; the peak, with
parabolic sub-sample interpolation, is the delay (positive: the signal is late). This works for
any interval length, including the 0.4 to 0.6 s of the slam pose, as long as the intervals are
short against the yaw dynamics; the sharpness of the peak says whether they were.
"""

from __future__ import annotations

from dataclasses import dataclass
from typing import Protocol

import numpy as np
from numpy.typing import NDArray

Array = NDArray[np.float64]

LAG_RANGE_S = 0.6
LAG_STEP_S = 0.01
MIN_INTERVALS = 30


class HasMean(Protocol):
    def mean(self, a: Array, b: Array) -> Array: ...


@dataclass(frozen=True)
class LagResult:
    lag_s: float  # NaN when there were too few intervals
    peak_corr: float
    drop_at_0p1: float  # peak minus the mean correlation at +-0.1 s from the peak
    n: int
    lags: Array
    corr: Array


def parabolic_peak(x: Array, y: Array, i: int) -> float:
    """Abscissa of the vertex of the parabola through y[i-1], y[i], y[i+1] (x uniform)."""
    if i <= 0 or i >= len(y) - 1:
        return float(x[i])
    denom = y[i - 1] - 2.0 * y[i] + y[i + 1]
    if denom == 0.0:
        return float(x[i])
    shift = 0.5 * (y[i - 1] - y[i + 1]) / denom
    return float(x[i] + np.clip(shift, -1.0, 1.0) * (x[1] - x[0]))


def interval_lag(
    a: Array,
    b: Array,
    reference_rate: Array,
    signal: HasMean,
    lag_range_s: float = LAG_RANGE_S,
    lag_step_s: float = LAG_STEP_S,
    min_intervals: int = MIN_INTERVALS,
) -> LagResult:
    lags = np.arange(-lag_range_s, lag_range_s + 0.5 * lag_step_s, lag_step_s)
    corr = np.full(len(lags), np.nan)
    count = 0
    for k, lag in enumerate(lags):
        s = signal.mean(a + lag, b + lag)
        ok = np.isfinite(s) & np.isfinite(reference_rate)
        count = max(count, int(ok.sum()))
        if ok.sum() < min_intervals or np.std(s[ok]) == 0 or np.std(reference_rate[ok]) == 0:
            continue
        corr[k] = float(np.corrcoef(s[ok], reference_rate[ok])[0, 1])
    if not np.isfinite(corr).any():
        return LagResult(float("nan"), float("nan"), float("nan"), count, lags, corr)
    i = int(np.nanargmax(corr))
    lag = parabolic_peak(lags, np.where(np.isfinite(corr), corr, np.nanmin(corr)), i)
    peak = float(corr[i])
    sides = []
    for off in (-0.1, 0.1):
        j = round((lags[i] + off - lags[0]) / lag_step_s)
        if 0 <= j < len(lags) and np.isfinite(corr[j]):
            sides.append(corr[j])
    drop = peak - float(np.mean(sides)) if sides else float("nan")
    return LagResult(lag, peak, drop, count, lags, corr)
