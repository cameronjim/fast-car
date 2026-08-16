"""Small, robust derived-scalar helpers for the S.6 battery's summary golden records
(claude-docs/07-sim-and-sysid.md's step-response vocabulary: steady-state value, settling
time).

Deliberately NOT threshold-crossing curve fits (e.g. a literal 10%/90% rise-time crossing
search): those are ambiguous on a lightly underdamped response, and maneuvers.py's
steering_step visibly overshoots and rings before settling. "Settling time" here instead
means "the last time the signal was still more than `band` away from its OWN final sampled
value" -- well-defined for any trajectory shape, including ones that overshoot past their
eventual steady state before settling into it.
"""

from __future__ import annotations

import numpy as np


def steady_state_mean(values: np.ndarray, *, window: int) -> float:
    """Mean of the last ``window`` samples -- the steady-state estimate for a settled
    signal. Averaging (rather than taking the last sample) damps any small residual
    numerical oscillation."""
    if window <= 0:
        raise ValueError(f"window must be positive, got {window}")
    return float(np.mean(values[-window:]))


def settling_time_s(times: np.ndarray, values: np.ndarray, *, band: float) -> float:
    """First time after which ``values`` never again strays more than ``band`` from its own
    final sampled value.

    The comparison is always against ``values[-1]`` itself, so the last sample's own
    deviation is exactly 0 and therefore always "inside" the (non-negative) band -- the last
    index can never be the sample the search is looking for, so there is always at least one
    later, in-band sample to report as the settling point.
    """
    if band < 0:
        raise ValueError(f"band must be >= 0, got {band}")
    final = values[-1]
    outside = np.nonzero(np.abs(values - final) > band)[0]
    if outside.size == 0:
        return float(times[0])
    last_outside = int(outside[-1])
    return float(times[last_outside + 1])


def peak_overshoot_frac(values: np.ndarray, steady_state: float) -> float:
    """Fractional overshoot of the largest-magnitude sample relative to ``steady_state``."""
    if steady_state == 0.0:
        return float(np.max(np.abs(values)))
    peak = values[np.argmax(np.abs(values))]
    return float((abs(peak) - abs(steady_state)) / abs(steady_state))


def first_time_below(times: np.ndarray, values: np.ndarray, *, threshold: float) -> float:
    """First time ``values`` drops below ``threshold``; the last recorded time if it never
    does within the window (a valid, still-deterministic golden field -- not an error)."""
    idx = np.nonzero(values < threshold)[0]
    if idx.size == 0:
        return float(times[-1])
    return float(times[int(idx[0])])
