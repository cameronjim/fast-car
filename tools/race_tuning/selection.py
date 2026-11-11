"""The selection rule for the race profile's defaults.

STATED BEFORE THE SEARCH WAS RUN (the rule below is the one written into
docs/notes/gap-follow-race-2026-11-12.md when the harness was committed; changing it after seeing
results would make the held-out numbers meaningless).

A candidate is run on every TUNING track in both directions and compared with the baseline (the
recommended profile before this work) run-for-run on the same track and direction. It is
ADMISSIBLE only if, over all its tuning runs:

  1. every run completed its laps (no stall, no timeout, no crash of a node);
  2. there was no wall contact;
  3. the minimum body clearance over all runs is at least MIN_CLEARANCE_M;
  4. its safety_node brakes per lap, over all runs, are at most the baseline's plus
     BRAKE_ALLOWANCE_PER_LAP (a faster car may not be a more intervened-on car);
  5. escapes per lap are at most the baseline's plus ESCAPE_ALLOWANCE_PER_LAP.

Among admissible candidates the score is the mean over runs of (candidate lap time / baseline lap
time), lower is better. Candidates whose scores are within TIE_BAND of each other tie, and a tie
goes to fewer brakes per lap, then fewer escapes per lap, then the lower top speed. The winner's
parameters are the race profile's defaults. If NO candidate is admissible the harness reports
that and the profile keeps the baseline's parameters with the physics law on at the baseline's top
speed (the safe fallback is decided by hand, not by this function).

The held-out tracks are then run once with the winner and the baseline, and the same admissibility
tests are applied to report whether the result generalises. If the winner is inadmissible there,
the next-ranked candidate is tried, and the note says that the held-out set was consumed by it.
"""

from __future__ import annotations

import math
from dataclasses import dataclass
from typing import Any

MIN_CLEARANCE_M = 0.04
BRAKE_ALLOWANCE_PER_LAP = 0.5
ESCAPE_ALLOWANCE_PER_LAP = 0.5
TIE_BAND = 0.01


@dataclass(frozen=True)
class RunRecord:
    """One run, reduced to what the rule reads."""

    track: str
    direction: str
    laps_wanted: int
    laps: int
    lap_time_s: float
    brakes: int
    escapes: int
    contacts: int
    min_clearance_m: float
    completed: bool  # laps reached and no node died


def run_record(result: dict[str, Any], track: str, direction: str, laps_wanted: int) -> RunRecord:
    """Reduce a track_family_runner result JSON."""
    if "error" in result:
        return RunRecord(track, direction, laps_wanted, 0, math.nan, 0, 0, 0, math.nan, False)
    laps = int(result.get("laps") or 0)
    return RunRecord(
        track=track,
        direction=direction,
        laps_wanted=laps_wanted,
        laps=laps,
        lap_time_s=float(result.get("lap_time_s") or math.nan),
        brakes=int(result.get("obstacle_brakes", 0)),
        escapes=int(result.get("escapes", 0)),
        contacts=int(result.get("wall_contacts", 0)),
        min_clearance_m=float(result.get("min_body_clearance_m", math.nan)),
        completed=laps >= laps_wanted and result.get("stopped_because") == "laps",
    )


@dataclass(frozen=True)
class Aggregate:
    runs: int
    completed: int
    contacts: int
    min_clearance_m: float
    brakes_per_lap: float
    escapes_per_lap: float
    mean_ratio: float  # candidate lap / baseline lap, nan without a baseline


def _per_lap(records: list[RunRecord], field: str) -> float:
    laps = sum(max(r.laps, 0) for r in records)
    return sum(getattr(r, field) for r in records) / laps if laps else math.inf


def aggregate(records: list[RunRecord], baseline: list[RunRecord] | None = None) -> Aggregate:
    base = {(r.track, r.direction): r for r in baseline or []}
    ratios = []
    for r in records:
        b = base.get((r.track, r.direction))
        if b and b.completed and r.completed and b.lap_time_s > 0 and r.lap_time_s > 0:
            ratios.append(r.lap_time_s / b.lap_time_s)
    clearances = [r.min_clearance_m for r in records if not math.isnan(r.min_clearance_m)]
    return Aggregate(
        runs=len(records),
        completed=sum(r.completed for r in records),
        contacts=sum(r.contacts for r in records),
        min_clearance_m=min(clearances) if clearances else math.nan,
        brakes_per_lap=_per_lap(records, "brakes"),
        escapes_per_lap=_per_lap(records, "escapes"),
        mean_ratio=sum(ratios) / len(ratios) if ratios else math.nan,
    )


def admissible(agg: Aggregate, baseline: Aggregate) -> tuple[bool, list[str]]:
    """(True, []) or (False, why), per the rule above."""
    why = []
    if agg.completed < agg.runs or agg.runs == 0:
        why.append(f"completed {agg.completed} of {agg.runs} runs")
    if agg.contacts > 0:
        why.append(f"{agg.contacts} wall contacts")
    if not agg.min_clearance_m >= MIN_CLEARANCE_M:
        why.append(f"min clearance {agg.min_clearance_m:.3f} m < {MIN_CLEARANCE_M} m")
    if agg.brakes_per_lap > baseline.brakes_per_lap + BRAKE_ALLOWANCE_PER_LAP:
        why.append(
            f"{agg.brakes_per_lap:.2f} brakes per lap > baseline {baseline.brakes_per_lap:.2f} "
            f"+ {BRAKE_ALLOWANCE_PER_LAP}"
        )
    if agg.escapes_per_lap > baseline.escapes_per_lap + ESCAPE_ALLOWANCE_PER_LAP:
        why.append(
            f"{agg.escapes_per_lap:.2f} escapes per lap > baseline "
            f"{baseline.escapes_per_lap:.2f} + {ESCAPE_ALLOWANCE_PER_LAP}"
        )
    if math.isnan(agg.mean_ratio):
        why.append("no comparable runs against the baseline")
    return (not why, why)


def rank(
    candidates: dict[str, tuple[dict[str, Any], Aggregate]], baseline: Aggregate
) -> list[tuple[str, dict[str, Any], Aggregate]]:
    """The admissible candidates, best first: ties within TIE_BAND of the best score of their
    group are ordered by brakes per lap, escapes per lap, then top speed."""
    ok = [
        (cid, params, agg)
        for cid, (params, agg) in candidates.items()
        if admissible(agg, baseline)[0]
    ]
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
