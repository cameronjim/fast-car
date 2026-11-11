"""Markdown tables for the harness: one row per variant (and direction) with intervals.

Intervals: completion is a Wilson interval over runs; lap time and the lap ratio against the
baseline are percentile bootstrap intervals of the mean over runs (the runs are tracks x
directions, so the spread is the spread over tracks: the simulator is deterministic enough that
repeating one track gives the same lap to 0.1 s, which is why the harness varies the TRACK rather
than the seed). Brakes and escapes are totals per lap over the runs. Everything is computed from
RunRecords, so it is the same code whether the records came from a search or an evaluation.

REPEATS. The simulator's lap times repeat to 0.1 s but the safety_node brake counts do not: the
same configuration on the same tracks gave 1.06 and 1.29 brakes per lap in two runs of the
held-out set (process timing decides whether a TTC latch engages once or twice). A variant run
several times is named `name`, `name_r2`, `name_r3`, ...; `pool` merges them into one variant of
all their runs, and the lap ratio is then taken against the MEAN of the baseline's repeats on the
same track and direction.
"""

from __future__ import annotations

import math
import re

from race_tuning.selection import RunRecord, baseline_times
from race_tuning.stats import mean_ci, paired_ratio_ci, wilson_interval


def _fmt_ci(mean: float, low: float, high: float, digits: int = 2) -> str:
    if math.isnan(mean):
        return "n/a"
    return f"{mean:.{digits}f} [{low:.{digits}f}, {high:.{digits}f}]"


def pool(variants: dict[str, list[RunRecord]]) -> dict[str, list[RunRecord]]:
    """Merge `name_r2`, `name_r3`, ... into `name` (order of first appearance kept)."""
    merged: dict[str, list[RunRecord]] = {}
    for name, records in variants.items():
        base = re.sub(r"_r\d+$", "", name)
        merged.setdefault(base, []).extend(records)
    return merged


def summarise_variant(records: list[RunRecord], baseline: list[RunRecord]) -> dict[str, object]:
    base = baseline_times(baseline)
    done = [r for r in records if r.completed]
    laps = sum(max(r.laps, 0) for r in records)
    paired = [
        (r.lap_time_s, base[(r.track, r.direction)]) for r in done if (r.track, r.direction) in base
    ]
    clearances = [r.min_clearance_m for r in records if not math.isnan(r.min_clearance_m)]
    low, high = wilson_interval(len(done), len(records))
    return {
        "runs": len(records),
        "completed": len(done),
        "completion_ci": (low, high),
        "contacts": sum(r.contacts for r in records),
        "min_clearance_m": min(clearances) if clearances else math.nan,
        "brakes_per_lap": sum(r.brakes for r in records) / laps if laps else math.nan,
        "brakes_ci": mean_ci([r.brakes / r.laps for r in records if r.laps > 0], seed=1),
        "escapes_per_lap": sum(r.escapes for r in records) / laps if laps else math.nan,
        "lap_time": mean_ci([r.lap_time_s for r in done]),
        "ratio": paired_ratio_ci([a for a, _ in paired], [b for _, b in paired]),
    }


def comparison_table(variants: dict[str, list[RunRecord]], baseline_name: str) -> str:
    """Rows: each variant overall and per direction. Columns as in the module docstring."""
    baseline = variants[baseline_name]
    lines = [
        (
            "| Variant | Dir | Runs | Completed (95% Wilson) | Wall contacts "
            "| Min body clearance (m) | Brakes per lap (95% CI) | Escapes per lap "
            "| Lap time (s, 95% CI) "
            "| Lap time vs baseline (ratio, 95% CI) |"
        ),
        "|---|---|---|---|---|---|---|---|---|---|",
    ]
    for name, records in variants.items():
        for direction in ("both", "ccw", "cw"):
            subset = (
                records if direction == "both" else [r for r in records if r.direction == direction]
            )
            base = (
                baseline
                if direction == "both"
                else [r for r in baseline if r.direction == direction]
            )
            if not subset:
                continue
            s = summarise_variant(subset, base)
            low, high = s["completion_ci"]  # type: ignore[misc]
            lines.append(
                f"| {name} | {direction} | {s['runs']} | {s['completed']} "
                f"({low * 100:.0f}% to {high * 100:.0f}%) | {s['contacts']} | "
                f"{s['min_clearance_m']:.3f} | {_fmt_ci(*s['brakes_ci'])} | "  # type: ignore[misc]
                f"{s['escapes_per_lap']:.2f} | {_fmt_ci(*s['lap_time'])} | "  # type: ignore[misc]
                f"{_fmt_ci(*s['ratio'], digits=3)} |"  # type: ignore[misc]
            )
    return "\n".join(lines)


def per_track_table(variants: dict[str, list[RunRecord]]) -> str:
    """Rows: track and direction; one cell per variant (laps, lap time, brakes, escapes,
    contacts, minimum clearance)."""
    keys = sorted({(r.track, r.direction) for rs in variants.values() for r in rs})
    names = list(variants)
    index = {n: {(r.track, r.direction): r for r in rs} for n, rs in variants.items()}
    lines = ["| Track | Dir | " + " | ".join(names) + " |", "|---|---|" + "---|" * len(names)]
    for track, direction in keys:
        cells = []
        for n in names:
            r = index[n].get((track, direction))
            if r is None:
                cells.append("-")
            elif math.isnan(r.lap_time_s):
                cells.append(f"{r.laps} laps, DNF, {r.brakes}b {r.escapes}e {r.contacts}c")
            else:
                cells.append(
                    f"{r.laps} laps {r.lap_time_s:.1f}s, {r.brakes}b {r.escapes}e "
                    f"{r.contacts}c, clr {r.min_clearance_m:.3f}"
                )
        lines.append(f"| {track} | {direction} | " + " | ".join(cells) + " |")
    return "\n".join(lines)
