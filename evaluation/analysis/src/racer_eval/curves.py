"""Lap time against lap index over the runs of one condition (pre-registration sections 5.2, 5.3).

Completed laps only. A run that stops early (DNF) contributes the laps it completed and counts
in the completion rate; nothing is ever imputed. Lap indices are 1-based.
"""

from __future__ import annotations

from collections.abc import Sequence
from dataclasses import dataclass, field

import numpy as np
from numpy.typing import NDArray

from racer_eval.stats import Interval, t_interval, wilson_interval

#: Laps per run of GP4 and the headline evaluation (sections 5.2 and 5.3).
N_LAPS = 10
#: "T_late = median time of laps 8 to 10" (section 5.1), 1-based and inclusive.
LATE_FIRST_LAP = 8
LATE_LAST_LAP = 10


@dataclass(frozen=True)
class RunLaps:
    """The laps of one run as the analysis sees them."""

    run_id: str
    condition: str
    #: Completed lap times in lap order (lap 1 first), seconds.
    lap_times: tuple[float, ...]
    #: Wall contacts counted in the run (people or video decide, section 4.3).
    contacts: int = 0
    #: The run stopped after this many completed laps. Laps listed beyond it are never used.
    dnf_after: int | None = None
    dnf_cause: str | None = None
    n_target: int = N_LAPS

    @property
    def usable(self) -> NDArray[np.float64]:
        """Completed laps that count: at most n_target, none beyond a DNF."""
        stop = len(self.lap_times)
        stop = min(stop, self.n_target)
        if self.dnf_after is not None:
            stop = min(stop, max(self.dnf_after, 0))
        return np.asarray(self.lap_times[:stop], dtype=np.float64)

    @property
    def completed(self) -> bool:
        """All n_target laps completed and no DNF flagged before the last one."""
        return len(self.usable) >= self.n_target


@dataclass(frozen=True)
class RunMetrics:
    t1: float
    t_late: float
    #: Delta = T1 - T_late, positive when the run got faster.
    delta: float
    #: r = T_late / T1.
    ratio: float


def run_metrics(run: RunLaps) -> RunMetrics | None:
    """T1, T_late (median of laps 8 to 10), Delta and r of a run; None when it has no lap 10."""
    laps = run.usable
    if len(laps) < LATE_LAST_LAP:
        return None
    t1 = float(laps[0])
    t_late = float(np.median(laps[LATE_FIRST_LAP - 1 : LATE_LAST_LAP]))
    return RunMetrics(t1, t_late, t1 - t_late, t_late / t1)


@dataclass(frozen=True)
class CurvePoint:
    lap: int
    n: int
    mean: float
    #: 95 percent t interval of the mean; None when n < 2.
    interval: Interval | None


@dataclass(frozen=True)
class LapCurve:
    condition: str
    points: tuple[CurvePoint, ...]
    n_runs: int
    n_completed: int
    completion: Interval
    dnf: tuple[tuple[str, str | None], ...] = field(default_factory=tuple)


def lap_curve(runs: Sequence[RunLaps], level: float = 0.95) -> LapCurve:
    """Mean lap time and t interval per lap index over the runs of ONE condition."""
    if not runs:
        raise ValueError("no runs")
    conditions = {r.condition for r in runs}
    if len(conditions) != 1:
        raise ValueError(f"runs of several conditions: {sorted(conditions)}")
    n_target = max(r.n_target for r in runs)
    points = []
    for k in range(1, n_target + 1):
        times = np.array([r.usable[k - 1] for r in runs if len(r.usable) >= k])
        if len(times) == 0:
            continue
        interval = t_interval(times, level) if len(times) >= 2 else None
        points.append(CurvePoint(k, len(times), float(times.mean()), interval))
    done = sum(r.completed for r in runs)
    return LapCurve(
        condition=next(iter(conditions)),
        points=tuple(points),
        n_runs=len(runs),
        n_completed=done,
        completion=wilson_interval(done, len(runs), level),
        dnf=tuple((r.run_id, r.dnf_cause) for r in runs if not r.completed),
    )
