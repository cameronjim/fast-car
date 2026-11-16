#!/usr/bin/env python3
"""TEST ONLY entry point of lap_memory_node that COUNTS the crossing candidates its innovation gate
rejects, per lap (brief 40, battery B5 report item on the round donut O1, where every place looks
like every other). Reported without a threshold.

racer_laps is not changed: the counter wraps `StartScanGate._consider` from outside and decides
"rejected by the innovation gate" by the gate's own condition, after the same checks that come
before it in `_consider` (window and free space violation, heading, minimum lap length), and
only when the original did not keep the candidate. The counts are written to the JSON file named
by the environment variable LAP_INNOVATION_OUT after every rejection and every lap closure, so a
killed run loses nothing:

    {"per_lap": [rejected during lap 1, lap 2, ...], "total": n, "accepted_total": m}

The runner (`--count-innovation`) starts the lap node through this file in place of the installed
executable and reads the JSON back into the run's result. The ROS node is imported only in main().
"""

from __future__ import annotations

import json
import os
import sys
from pathlib import Path

OUT_ENV = "LAP_INNOVATION_OUT"


class Counter:
    """Per lap counts of rejected and accepted candidates."""

    def __init__(self, out: Path | None = None) -> None:
        self.rejected: dict[int, int] = {}
        self.accepted: dict[int, int] = {}
        self._out = out

    def note(self, lap_in_progress: int, rejected: bool, accepted: bool) -> None:
        if rejected:
            self.rejected[lap_in_progress] = self.rejected.get(lap_in_progress, 0) + 1
        if accepted:
            self.accepted[lap_in_progress] = self.accepted.get(lap_in_progress, 0) + 1
        if rejected or accepted:
            self.write()

    def as_dict(self) -> dict:
        laps = max([*self.rejected, *self.accepted, 0])
        return {
            "per_lap": [self.rejected.get(k, 0) for k in range(1, laps + 1)],
            "accepted_per_lap": [self.accepted.get(k, 0) for k in range(1, laps + 1)],
            "total": sum(self.rejected.values()),
            "accepted_total": sum(self.accepted.values()),
        }

    def write(self) -> None:
        if self._out is not None:
            tmp = self._out.with_suffix(".tmp")
            tmp.write_text(json.dumps(self.as_dict()))
            tmp.replace(self._out)


def passes_checks_before_innovation(gate, res, now, two_pi: float) -> tuple[bool, float]:
    """True when `_consider` gets as far as the innovation test for this match, and the line
    position it computed. These are the conditions that precede it in racer_laps/gate.py."""
    c = gate.cfg
    if abs(res.x_m) > c.consensus_window_m or res.freespace_violation > c.strong_max_violation:
        return False, 0.0
    if abs(now.heading - gate._heading_origin) < two_pi - c.closure_heading_slack_rad:
        return False, 0.0
    line = now.total_m - res.x_m
    if line - gate._origin_m < c.min_lap_length_m:
        return False, line
    return True, line


def rejected_by_innovation(gate, res, now, two_pi: float) -> bool:
    """The gate's innovation condition: a trusted length and a line position further from the last
    line plus the estimated length than the band."""
    ok, line = passes_checks_before_innovation(gate, res, now, two_pi)
    if not ok or not gate._est.trusted:
        return False
    return abs(line - (gate._origin_m + gate._est.value_m)) > gate._est.band_m()


def install(gate_class, counter: Counter, two_pi: float) -> None:
    """Wrap `gate_class._consider` so that every call is classified and counted."""
    original = gate_class._consider

    def counting(self, t_eff_s, res, source, now):
        before = len(self._cands)
        would_reject = rejected_by_innovation(self, res, now, two_pi)
        original(self, t_eff_s, res, source, now)
        kept = len(self._cands) > before
        # a candidate the original dropped for the innovation reason; the original also drops it
        # for the support test, which is checked after, so only count when it was not kept and
        # the innovation condition holds (the innovation test comes first in the original)
        counter.note(self.laps_completed + 1, would_reject and not kept, kept)

    gate_class._consider = counting


def main() -> int:
    from racer_laps import gate
    from racer_laps.lap_memory_node import main as node_main

    out = os.environ.get(OUT_ENV)
    counter = Counter(Path(out) if out else None)
    install(gate.StartScanGate, counter, gate.TWO_PI)
    counter.write()
    return node_main(sys.argv)


if __name__ == "__main__":
    sys.exit(main())
