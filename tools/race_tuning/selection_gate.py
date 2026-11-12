"""The selection rule for a race profile that must not lean on safety_node (2026-11-12 retune).

STATED AND COMMITTED BEFORE THE RETUNE WAS RUN (docs/notes/safety-race-retune-2026-11-12.md quotes
it; changing it after seeing results would make the held-out numbers meaningless). The first rule
(selection.py) asked a faster profile to brake no more than the baseline plus 0.5 per lap. The
2026-11-12 race bag showed what that allows: 18 hard brakes in 190 s, 11 of them at the end of the
oval, so now the brake count itself is the requirement.

A candidate is run on every TUNING track of the WIDE family (lanes 1.3 to 1.5 m) in both
directions. It is ADMISSIBLE only if, over all its tuning runs:

  1. every run completed its laps (no stall, no timeout, no crash of a node);
  2. there was no wall contact;
  3. the minimum body clearance over all runs is at least MIN_CLEARANCE_M;
  4. safety_node's hard brakes per lap, over all runs, are at most MAX_BRAKES_PER_LAP;
  5. reverse escapes per lap are at most MAX_ESCAPES_PER_LAP.

Among admissible candidates the best is the one with the lowest mean ratio of its lap time to the
baseline's on the same track and direction; candidates within TIE_BAND of the best ratio tie, and a
tie goes to fewer brakes per lap, then fewer escapes per lap, then the lower top speed. If NO
candidate is admissible the profile is NOT changed and the note says so.

The winner is then run on the HELD-OUT wide tracks, three repeats pooled (brake counts are not
repeatable run to run, selection.py's amendment), and the same five clauses are applied to the
pooled runs; the note reports the numbers whether it passes or not.
"""

from __future__ import annotations

import math
from typing import Any

from race_tuning.selection import Aggregate

MIN_CLEARANCE_M = 0.04
MAX_BRAKES_PER_LAP = 0.25
MAX_ESCAPES_PER_LAP = 0.25
TIE_BAND = 0.02


def admissible(agg: Aggregate) -> tuple[bool, list[str]]:
    """(True, []) or (False, why), per the rule above."""
    why = []
    if agg.completed < agg.runs or agg.runs == 0:
        why.append(f"completed {agg.completed} of {agg.runs} runs")
    if agg.contacts > 0:
        why.append(f"{agg.contacts} wall contacts")
    if not agg.min_clearance_m >= MIN_CLEARANCE_M:
        why.append(f"min clearance {agg.min_clearance_m:.3f} m < {MIN_CLEARANCE_M} m")
    if agg.brakes_per_lap > MAX_BRAKES_PER_LAP:
        why.append(f"{agg.brakes_per_lap:.2f} brakes per lap > {MAX_BRAKES_PER_LAP}")
    if agg.escapes_per_lap > MAX_ESCAPES_PER_LAP:
        why.append(f"{agg.escapes_per_lap:.2f} escapes per lap > {MAX_ESCAPES_PER_LAP}")
    if math.isnan(agg.mean_ratio):
        why.append("no comparable runs against the baseline")
    return (not why, why)


def rank(
    candidates: dict[str, tuple[dict[str, Any], Aggregate]],
) -> list[tuple[str, dict[str, Any], Aggregate]]:
    """The admissible candidates, best first (ties within TIE_BAND of the best score of their
    group are ordered by brakes per lap, escapes per lap, then top speed)."""
    ok = [(cid, p, agg) for cid, (p, agg) in candidates.items() if admissible(agg)[0]]
    ok.sort(key=lambda c: c[2].mean_ratio)
    ranked: list[tuple[str, dict[str, Any], Aggregate]] = []
    while ok:
        best = ok[0][2].mean_ratio
        group = [c for c in ok if c[2].mean_ratio <= best + TIE_BAND]
        group.sort(
            key=lambda c: (
                c[2].brakes_per_lap,
                c[2].escapes_per_lap,
                c[1].get("max_speed_mps", 0.0),
                c[2].mean_ratio,
            )
        )
        ranked.append(group[0])
        ok = [c for c in ok if c is not group[0]]
    return ranked
