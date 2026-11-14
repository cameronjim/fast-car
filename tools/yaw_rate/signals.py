"""Integrating a yaw signal over arbitrary intervals, and closure noise.

Two kinds of signal: a RATE series (rad/s at samples, integrated by the trapezoid rule: the wheel
model and the steering model) and a STEP series (the yaw change over each [t0, t1] of a scan
pair, spread uniformly over the step: scan matching). Both answer `integral(a, b)`, the yaw
change over [a, b], which is NaN where the signal is not valid over the whole interval. For any
sampling the integral over adjacent intervals adds up to the integral over their union.
"""

from __future__ import annotations

from dataclasses import dataclass

import numpy as np
from numpy.typing import NDArray

Array = NDArray[np.float64]

MAX_RATE_GAP_S = 0.2  # a hole in a rate series longer than this makes an interval invalid


@dataclass(frozen=True)
class RateSeries:
    """A yaw rate (rad/s) sampled at times t (ascending)."""

    t: Array
    w: Array

    def integral(self, a: Array, b: Array) -> Array:
        a = np.atleast_1d(np.asarray(a, dtype=float))
        b = np.atleast_1d(np.asarray(b, dtype=float))
        out = np.full(a.shape, np.nan)
        if len(self.t) < 2:
            return out
        dt = np.diff(self.t)
        cum = np.concatenate([[0.0], np.cumsum(0.5 * (self.w[1:] + self.w[:-1]) * dt)])
        inside = (a >= self.t[0]) & (b <= self.t[-1]) & (b >= a)
        out[inside] = np.interp(b[inside], self.t, cum) - np.interp(a[inside], self.t, cum)
        # A hole in the samples: the trapezoid across it is a guess.
        holes = np.nonzero(dt > MAX_RATE_GAP_S)[0]
        for h in holes:
            out[inside & (a < self.t[h + 1]) & (b > self.t[h])] = np.nan
        return out

    def mean(self, a: Array, b: Array) -> Array:
        return self.integral(a, b) / (np.atleast_1d(np.asarray(b)) - np.atleast_1d(np.asarray(a)))


@dataclass(frozen=True)
class StepSeries:
    """Yaw change `v[k]` over [t0[k], t1[k]] (ascending, not overlapping), NaN where the step is
    not valid (a rejected match, a missing scan)."""

    t0: Array
    t1: Array
    v: Array

    def integral(self, a: Array, b: Array) -> Array:
        a = np.atleast_1d(np.asarray(a, dtype=float))
        b = np.atleast_1d(np.asarray(b, dtype=float))
        out = np.full(a.shape, np.nan)
        if len(self.t0) == 0:
            return out
        width = self.t1 - self.t0
        first = np.searchsorted(self.t1, a, side="right")  # first step ending after a
        last = np.searchsorted(self.t0, b, side="left")  # steps before this start before b
        for i in range(len(a)):
            lo, hi = int(first[i]), int(last[i])
            if hi <= lo or b[i] < a[i]:
                continue
            # The interval must be covered by contiguous steps from its start to its end.
            if self.t0[lo] > a[i] + 1e-9 or self.t1[hi - 1] < b[i] - 1e-9:
                continue
            if hi - lo > 1 and np.any(self.t0[lo + 1 : hi] - self.t1[lo : hi - 1] > 1e-9):
                continue
            seg = self.v[lo:hi]
            if not np.all(np.isfinite(seg)):
                continue
            overlap = np.minimum(self.t1[lo:hi], b[i]) - np.maximum(self.t0[lo:hi], a[i])
            out[i] = float(np.sum(seg * np.clip(overlap, 0.0, None) / width[lo:hi]))
        return out

    def mean(self, a: Array, b: Array) -> Array:
        return self.integral(a, b) / (np.atleast_1d(np.asarray(b)) - np.atleast_1d(np.asarray(a)))


def closure_residual(step1: Array, step2: Array, direct: Array) -> Array:
    """Yaw of scan i to i+2 matched directly minus the sum of the two single steps, per i
    (`step1[i]` is i to i+1, `step2[i]` is i+1 to i+2, `direct[i]` is i to i+2). NaN where any of
    the three is invalid."""
    return direct - (step1 + step2)


def closure_rate_rms(residual: Array, span_s: Array) -> float:
    """RMS of the closure residual as a rate: each residual over the span of the direct match
    (about 0.2 s), in rad/s. NaN with no valid residual."""
    ok = np.isfinite(residual) & np.isfinite(span_s) & (span_s > 0)
    if not ok.any():
        return float("nan")
    return float(np.sqrt(np.mean((residual[ok] / span_s[ok]) ** 2)))


def per_step_noise_rate(closure_rms_rate: float) -> float:
    """The noise of ONE single-step yaw rate implied by a closure RMS, when the direct match and
    the two single steps carry independent noise of the same size: the residual has sqrt(3)
    times the step noise over a span twice the step, so the step rate noise is the closure rate
    RMS times 2 / sqrt(3)."""
    return closure_rms_rate * 2.0 / float(np.sqrt(3.0))
