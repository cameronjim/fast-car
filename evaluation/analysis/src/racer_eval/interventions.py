"""The intervention counting rule of pre-registration section 4.2, exactly.

Count `/safety/events` records with `phase == PHASE_ENGAGE` stamped between the first non-zero
`/drive_raw` command of a run and its last `/pnp/planner/status` message (for a driver without a
status topic: its last non-zero `/drive_raw`). Both edges are included. The watchdog engagement
that follows after the planner stopped publishing is the stop mechanism and falls after the
window, so it is not counted; one that lands before the last status is. Sources are reported
separately.

Plus, per start: whether any `watchdog` or `rate_limit` ENGAGE record lies in the first 2.0 s
after the first non-zero command (clipped to the window).
"""

from __future__ import annotations

from collections import Counter
from collections.abc import Sequence
from dataclasses import dataclass

import numpy as np
from numpy.typing import ArrayLike

from racer_eval.runs import Run

PHASE_ENGAGE = 0

#: Section 4.2: "ttc and obstacle (primary)". `ttc` is the forward gate, `ttc_reverse` the rear
#: one (racer_msgs/SafetyEvent source list); the reverse gate is an obstacle brake too.
OBSTACLE_SOURCES = ("ttc", "ttc_reverse")
#: The two kinds the start rule and the planner acceptance (4.2) look at.
START_SOURCES = ("watchdog", "rate_limit")
#: Section 4.2: "in the first 2.0 s after the first non-zero command".
START_WINDOW_S = 2.0
_SOURCES_REPORTED = ("watchdog", "rate_limit", *OBSTACLE_SOURCES)


@dataclass(frozen=True)
class RunInterventions:
    run_id: str
    duration_s: float
    #: ENGAGE records in the window by source, every source that occurred.
    counts: dict[str, int]
    #: Any watchdog or rate_limit ENGAGE in the first 2.0 s after the first non-zero command.
    start_event: bool
    #: False when the run never had a non-zero command (no counting window).
    commanded: bool = True

    @property
    def per_minute(self) -> dict[str, float]:
        minutes = self.duration_s / 60.0
        return {k: (v / minutes if minutes > 0 else float("nan")) for k, v in self.counts.items()}

    def count(self, *sources: str) -> int:
        return sum(self.counts.get(s, 0) for s in sources)

    @property
    def obstacle(self) -> int:
        return self.count(*OBSTACLE_SOURCES)


def count_interventions(
    run: Run, ev_t: ArrayLike, ev_phase: ArrayLike, ev_source: Sequence[str]
) -> RunInterventions:
    """Apply the counting rule to one run. Event arrays are the `/safety/events` records of the
    bag (arrival times in bag seconds)."""
    t = np.asarray(ev_t, dtype=np.float64)
    phase = np.asarray(ev_phase)
    window = run.counting_window
    if window is None:
        return RunInterventions(run.run_id, 0.0, {}, False, commanded=False)
    lo, hi = window
    in_window = (phase == PHASE_ENGAGE) & (t >= lo) & (t <= hi)
    sources = [str(s) for s in np.asarray(list(ev_source), dtype=object)[in_window]]
    counts = Counter(sources)
    for name in _SOURCES_REPORTED:
        counts.setdefault(name, 0)
    start_hi = min(hi, lo + START_WINDOW_S)
    start_event = any(
        (src in START_SOURCES) and lo <= ti <= start_hi
        for ti, src in zip(t[in_window], sources, strict=True)
    )
    return RunInterventions(run.run_id, run.duration_s, dict(counts), start_event)


def total(rows: Sequence[RunInterventions]) -> RunInterventions:
    """All runs pooled: summed durations and counts (start_event is not pooled, see
    starts_with_event)."""
    counts: Counter[str] = Counter()
    for r in rows:
        counts.update(r.counts)
    return RunInterventions("total", sum(r.duration_s for r in rows), dict(counts), False)


def starts_with_event(rows: Sequence[RunInterventions]) -> tuple[int, int]:
    """(starts with a watchdog or rate_limit in their first 2.0 s, starts) over runs that
    commanded at all."""
    commanded = [r for r in rows if r.commanded]
    return sum(r.start_event for r in commanded), len(commanded)
