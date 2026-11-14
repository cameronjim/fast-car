"""Gap statistics of a timeline: percentiles and counts above a list of limits.

A "gap" is the difference of two consecutive times. Percentiles use numpy's linear
interpolation (`np.percentile(..., method="linear")`); the report says so. A count "above" a limit
is strictly above, with the tolerance ABOVE_EPS_S so that a gap that is exactly the limit is not
counted even when two stamps in float seconds differ by 0.060000000000000005.
"""

from __future__ import annotations

from collections.abc import Sequence
from dataclasses import dataclass

import numpy as np
from numpy.typing import ArrayLike, NDArray

from pnp_session.constants import ABOVE_EPS_S, GAP_LIMITS_S


@dataclass(frozen=True)
class GapStats:
    n_messages: int
    n_gaps: int
    p50_s: float | None
    p99_s: float | None
    max_s: float | None
    #: Number of gaps strictly above each limit, keyed by the limit in milliseconds ("40", "60").
    above: dict[str, int]

    def to_dict(self) -> dict[str, object]:
        return {
            "n_messages": self.n_messages,
            "n_gaps": self.n_gaps,
            "p50_s": self.p50_s,
            "p99_s": self.p99_s,
            "max_s": self.max_s,
            "above": dict(self.above),
        }


def limit_key(limit_s: float) -> str:
    """Key of a limit in a table: milliseconds, no decimals when whole ("60")."""
    ms = limit_s * 1000.0
    return str(int(round(ms))) if abs(ms - round(ms)) < 1e-9 else f"{ms:g}"


def gaps_of(times: ArrayLike) -> NDArray[np.float64]:
    t = np.asarray(times, dtype=np.float64)
    return np.diff(t) if len(t) > 1 else np.zeros(0, dtype=np.float64)


def count_above(gaps: ArrayLike, limit_s: float) -> int:
    g = np.asarray(gaps, dtype=np.float64)
    return int(np.count_nonzero(g - limit_s > ABOVE_EPS_S))


def count_below(gaps: ArrayLike, limit_s: float) -> int:
    """Gaps strictly below a limit (bunched ticks), with the same tolerance."""
    g = np.asarray(gaps, dtype=np.float64)
    return int(np.count_nonzero(limit_s - g > ABOVE_EPS_S))


def gap_stats(
    gaps: ArrayLike, n_messages: int, limits_s: Sequence[float] = GAP_LIMITS_S
) -> GapStats:
    g = np.asarray(gaps, dtype=np.float64)
    above = {limit_key(lim): count_above(g, lim) for lim in limits_s}
    if len(g) == 0:
        return GapStats(n_messages, 0, None, None, None, above)
    p50, p99 = np.percentile(g, [50.0, 99.0], method="linear")
    return GapStats(n_messages, len(g), float(p50), float(p99), float(np.max(g)), above)


def timeline_stats(times: ArrayLike, limits_s: Sequence[float] = GAP_LIMITS_S) -> GapStats:
    t = np.asarray(times, dtype=np.float64)
    return gap_stats(gaps_of(t), len(t), limits_s)


def pooled_stats(
    gap_lists: Sequence[ArrayLike], n_messages: int, limits_s: Sequence[float] = GAP_LIMITS_S
) -> GapStats:
    """Statistics of the gaps of several runs together. Gaps never span two runs: each list is
    the gaps inside one run's window."""
    parts = [np.asarray(g, dtype=np.float64) for g in gap_lists]
    joined = np.concatenate(parts) if parts else np.zeros(0, dtype=np.float64)
    return gap_stats(joined, n_messages, limits_s)
