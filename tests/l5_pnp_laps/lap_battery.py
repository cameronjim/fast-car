"""The lap learning sim battery of the pre-registration (section 5.1): run summaries and the three
decisions D-S0, D-S1 and D-S2 as pure functions.

Every threshold is a module constant that quotes the pre-registration
(docs/notes/preregistration-pnp-v0.2.md, section 5.1). Nothing here may be changed after the
battery has been seen: a threshold that cannot be applied as written is reported, not edited.

Definitions, as pre-registered:

* Per run: T1 is the time of lap 1, T_late the median time of laps 8 to 10, r = T_late / T1.
  Lap times are GROUND TRUTH (the sim knows where the car is); the system's own lap times are
  used only by D-S0, which asks whether they agree with the truth.
* A run COMPLETES when it finishes 10 laps (ground truth).
* D-S0 (lap bookkeeping), over all runs of condition (a), D off at the lap-1 cap: the system's lap
  count equals the ground-truth count and every system lap time is within 0.3 s of the truth.
* D-S1 (D v0 may go on the car), condition (c): 0 wall contacts in the runs; at least 22 of 24
  runs complete 10 laps with r at most 0.80.
* D-S2 (D v1 may go on the car, and "D v1 beats D v0 in sim"), condition (d) against (c): 0 wall
  contacts; at least 22 of 24 complete 10 laps; the median over runs of T_late(d) / T_late(c),
  paired by case, direction and seed, at most 0.95; mean TTC or obstacle brakes per lap over laps 8
  to 10 at most 0.2. If the safety items hold and the 0.95 does not, D v1 is "no better than the
  schedule in sim" and D v0 is what runs on the car.
* If S4 (the saved real map) cannot be loaded the battery is 18 runs per condition and the
  thresholds scale: 22 of 24 becomes 16 of 18 (`required_completions`).
"""

from __future__ import annotations

import json
import math
import statistics
from dataclasses import dataclass, field
from pathlib import Path
from typing import Any

#: Pre-registration 5.1.
D_S0_LAP_TIME_TOLERANCE_S = 0.3
D_S1_MAX_CONTACTS = 0
D_S1_MAX_R = 0.80
D_S2_MAX_CONTACTS = 0
D_S2_MAX_MEDIAN_RATIO = 0.95
D_S2_MAX_BRAKES_PER_LAP = 0.2
FULL_BATTERY_RUNS = 24
FULL_BATTERY_REQUIRED = 22
LAPS_TO_COMPLETE = 10
LATE_LAPS = (8, 9, 10)  # 1-based lap indices of T_late
OBSTACLE_SOURCES = ("ttc", "ttc_reverse")

CONDITIONS = ("a", "b", "c", "d")


def required_completions(n_runs: int) -> int:
    """22 of 24, scaled for fewer runs per condition (16 of 18): floor(n * 22 / 24)."""
    return math.floor(n_runs * FULL_BATTERY_REQUIRED / FULL_BATTERY_RUNS + 1e-9)


@dataclass(frozen=True)
class RunSummary:
    condition: str
    case: str
    direction: str  # "ccw" or "cw"
    seed: int
    gt_lap_times_s: tuple[float, ...]
    system_lap_times_s: tuple[float, ...]
    contacts: int
    brakes_per_lap: tuple[int, ...]  # obstacle (ttc, ttc_reverse) BRAKE engagements per GT lap
    stopped_because: str = ""
    error: str = ""
    extras: dict[str, Any] = field(default_factory=dict, compare=False)

    @property
    def key(self) -> tuple[str, str, int]:
        return (self.case, self.direction, self.seed)

    @property
    def laps(self) -> int:
        return len(self.gt_lap_times_s)

    @property
    def completed(self) -> bool:
        return not self.error and self.laps >= LAPS_TO_COMPLETE

    @property
    def t1_s(self) -> float:
        return self.gt_lap_times_s[0] if self.gt_lap_times_s else math.nan

    @property
    def t_late_s(self) -> float:
        """Median time of laps 8 to 10 (NaN when the run has fewer than 10 laps)."""
        if self.laps < max(LATE_LAPS):
            return math.nan
        return statistics.median(self.gt_lap_times_s[i - 1] for i in LATE_LAPS)

    @property
    def r(self) -> float:
        return self.t_late_s / self.t1_s if self.t1_s > 0.0 else math.nan

    @property
    def late_brakes_per_lap(self) -> float:
        """TTC or obstacle brakes per lap over laps 8 to 10 (NaN when those laps were not driven)."""
        if len(self.brakes_per_lap) < max(LATE_LAPS):
            return math.nan
        return sum(self.brakes_per_lap[i - 1] for i in LATE_LAPS) / len(LATE_LAPS)


def summarise_run(result: dict, condition: str, case: str, direction: str, seed: int) -> RunSummary:
    """A RunSummary from the JSON `track_family_runner.py` wrote."""
    gt = tuple(float(t) for t in result.get("gt_lap_times_s", result.get("lap_times_s", [])))
    system = tuple(float(e["lap_time_s"]) for e in result.get("system_lap_events", []))
    brakes = tuple(
        sum(per.get(src, 0) for src in OBSTACLE_SOURCES) for per in result.get("brake_per_lap", [])
    )
    return RunSummary(
        condition=condition,
        case=case,
        direction=direction,
        seed=seed,
        gt_lap_times_s=gt,
        system_lap_times_s=system,
        contacts=int(result.get("wall_contacts", 0)),
        brakes_per_lap=brakes,
        stopped_because=str(result.get("stopped_because", "")),
        error=str(result.get("error", "")),
        extras={
            "real_time_factor": result.get("real_time_factor"),
            "min_body_clearance_m": result.get("min_body_clearance_m"),
            "safety_brakes": result.get("safety_brakes"),
            "rate_limit_events": result.get("rate_limit_events"),
            "engage_per_lap": result.get("engage_per_lap"),
        },
    )


def load_summaries(results_dir: Path) -> list[RunSummary]:
    """Every `<condition>_<case>_<direction>_<seed>.json` of a battery directory."""
    out = []
    for path in sorted(results_dir.glob("*.json")):
        parts = path.stem.split("_")
        if len(parts) != 4 or parts[0] not in CONDITIONS or parts[3] == "":
            continue
        condition, case, direction, seed = parts
        out.append(
            summarise_run(json.loads(path.read_text()), condition, case, direction, int(seed))
        )
    return out


@dataclass(frozen=True)
class Decision:
    name: str
    passed: bool | None  # None: not decided (pending)
    items: dict[str, tuple[bool | None, str]]  # item -> (ok, the numbers)
    note: str = ""

    def __str__(self) -> str:
        verdict = {True: "PASS", False: "FAIL", None: "PENDING"}[self.passed]
        lines = [f"{self.name}: {verdict}" + (f" ({self.note})" if self.note else "")]
        for item, (ok, numbers) in self.items.items():
            mark = {True: "ok", False: "FAIL", None: "pending"}[ok]
            lines.append(f"  {mark}: {item}: {numbers}")
        return "\n".join(lines)


def decide_d_s0(runs_a: list[RunSummary]) -> Decision:
    """D-S0 over every run of condition (a)."""
    if not runs_a:
        return Decision("D-S0", None, {}, "no condition (a) runs")
    count_bad = [
        f"{r.case}/{r.direction}/{r.seed}: system {len(r.system_lap_times_s)} vs truth {r.laps}"
        for r in runs_a
        if len(r.system_lap_times_s) != r.laps
    ]
    time_bad = []
    for r in runs_a:
        for k, (sys_t, gt_t) in enumerate(zip(r.system_lap_times_s, r.gt_lap_times_s), start=1):
            if abs(sys_t - gt_t) > D_S0_LAP_TIME_TOLERANCE_S:
                time_bad.append(
                    f"{r.case}/{r.direction}/{r.seed} lap {k}: system {sys_t:.2f} vs truth {gt_t:.2f}"
                )
    errors = [f"{r.case}/{r.direction}/{r.seed}: {r.error[:80]}" for r in runs_a if r.error]
    items = {
        "system lap count equals ground truth in every run": (
            not count_bad,
            f"{len(runs_a) - len(count_bad)} of {len(runs_a)} runs agree"
            + (f"; {count_bad[:6]}" if count_bad else ""),
        ),
        f"every system lap time within {D_S0_LAP_TIME_TOLERANCE_S} s of ground truth": (
            not time_bad,
            f"{len(time_bad)} laps outside" + (f"; {time_bad[:8]}" if time_bad else ""),
        ),
        "no run failed to execute": (not errors, f"{len(errors)} errors {errors[:4]}"),
    }
    return Decision("D-S0", all(ok for ok, _ in items.values()), items)


def _completion_items(
    runs: list[RunSummary], contacts_max: int, with_r: bool
) -> dict[str, tuple[bool | None, str]]:
    n = len(runs)
    need = required_completions(n)
    contacts = sum(r.contacts for r in runs)
    done = [r for r in runs if r.completed]
    items: dict[str, tuple[bool | None, str]] = {
        f"{contacts_max} wall contacts in {n} runs": (
            contacts <= contacts_max,
            (
                f"{contacts} contacts, runs with a contact: "
                f"{[f'{r.case}/{r.direction}/{r.seed}' for r in runs if r.contacts][:8]}"
            ),
        )
    }
    if with_r:
        good = [r for r in done if r.r <= D_S1_MAX_R]
        items[f"at least {need} of {n} runs complete 10 laps with r <= {D_S1_MAX_R}"] = (
            len(good) >= need,
            (
                f"{len(done)} complete, {len(good)} of them with r <= {D_S1_MAX_R}; r of the "
                f"completing runs: {sorted(round(r.r, 2) for r in done)}"
            ),
        )
    else:
        items[f"at least {need} of {n} runs complete 10 laps"] = (
            len(done) >= need,
            f"{len(done)} complete",
        )
    return items


def decide_d_s1(runs_c: list[RunSummary]) -> Decision:
    """D-S1 over condition (c), D v0."""
    if not runs_c:
        return Decision("D-S1", None, {}, "no condition (c) runs")
    items = _completion_items(runs_c, D_S1_MAX_CONTACTS, with_r=True)
    return Decision("D-S1", all(ok for ok, _ in items.values()), items)


def paired_ratios(runs_d: list[RunSummary], runs_c: list[RunSummary]) -> list[float]:
    """T_late(d) / T_late(c) for every (case, direction, seed) that completed 10 laps in both."""
    c_by_key = {r.key: r for r in runs_c}
    out = []
    for d in runs_d:
        c = c_by_key.get(d.key)
        if c is not None and d.completed and c.completed and c.t_late_s > 0.0:
            out.append(d.t_late_s / c.t_late_s)
    return out


def decide_d_s2(runs_d: list[RunSummary], runs_c: list[RunSummary]) -> Decision:
    """D-S2 over condition (d) against (c). `passed` is the safety items and the 0.95 together;
    `note` says "no better than the schedule in sim" when only the 0.95 fails."""
    if not runs_d:
        return Decision("D-S2", None, {}, "pending: no condition (d) runs")
    items = _completion_items(runs_d, D_S2_MAX_CONTACTS, with_r=False)
    ratios = paired_ratios(runs_d, runs_c)
    median = statistics.median(ratios) if ratios else math.nan
    ratio_ok = bool(ratios) and median <= D_S2_MAX_MEDIAN_RATIO
    items[f"median T_late(d) / T_late(c) <= {D_S2_MAX_MEDIAN_RATIO}"] = (
        ratio_ok,
        f"median {median:.3f} over {len(ratios)} pairs",
    )
    late = [r.late_brakes_per_lap for r in runs_d if not math.isnan(r.late_brakes_per_lap)]
    mean_brakes = statistics.fmean(late) if late else math.nan
    items[f"mean TTC or obstacle brakes per lap, laps 8 to 10, <= {D_S2_MAX_BRAKES_PER_LAP}"] = (
        bool(late) and mean_brakes <= D_S2_MAX_BRAKES_PER_LAP,
        f"{mean_brakes:.3f} over {len(late)} runs",
    )
    safety_ok = all(ok for name, (ok, _) in items.items() if "median" not in name)
    passed = safety_ok and ratio_ok
    note = ""
    if safety_ok and not ratio_ok:
        note = "no better than the schedule in sim: D v0 is what runs on the car"
    return Decision("D-S2", passed, items, note)


def condition_table(runs: list[RunSummary]) -> str:
    """One markdown row per run: T1, T_late, r, contacts, brakes per lap, completion."""
    lines = [
        (
            "| condition | case | dir | seed | laps | T1 s | T_late s | r | contacts | "
            "obstacle brakes per lap | complete |"
        ),
        "|---|---|---|---|---|---|---|---|---|---|---|",
    ]
    for r in sorted(runs, key=lambda x: (x.condition, x.case, x.direction, x.seed)):
        lines.append(
            f"| {r.condition} | {r.case} | {r.direction} | {r.seed} | {r.laps} | "
            f"{r.t1_s:.2f} | {r.t_late_s:.2f} | {r.r:.2f} | {r.contacts} | "
            f"{list(r.brakes_per_lap)} | {'yes' if r.completed else 'NO ' + r.error[:40]} |"
        )
    return "\n".join(lines)
