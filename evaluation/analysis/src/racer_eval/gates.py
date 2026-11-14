"""The decisions of the pre-registration, as pure functions from tables to a result object.

Each decision lists EVERY item of its section with the measured value, the threshold and pass or
fail, so a reviewer can map each constant to the sentence it implements. Thresholds are the module
constants below, each with the section it comes from. The decision functions take them as
keyword-only arguments defaulting to those constants only so the tests can probe the edges; the
result records whether every threshold used equals the pre-registered one (`preregistered`), and
the CLI never passes any.
"""

from __future__ import annotations

import math
import operator
from collections.abc import Callable, Mapping, Sequence
from dataclasses import asdict, dataclass, field, replace
from typing import Any

import numpy as np

from racer_eval.curves import (
    LATE_FIRST_LAP,
    LATE_LAST_LAP,
    N_LAPS,
    RunLaps,
    run_metrics,
)
from racer_eval.interventions import RunInterventions, starts_with_event
from racer_eval.laps import GP3_LAPS, GP3_WHEEL_DISTANCE_FACTOR, LAP_SOURCE_TOLERANCE_S
from racer_eval.stats import Interval, t_interval, wilson_interval

# --- Section 4.2: planner accepted on the car (C) ---------------------------------------------
#: "over at least 300 s of PnP driving"
ACC_MIN_DRIVING_S = 300.0
#: "at least 8 standing starts at caps 0.5 and 1.0"
ACC_MIN_STARTS = 8
ACC_START_CAPS_MPS = (0.5, 1.0)
#: "`watchdog` at most 0.2 per minute"
ACC_WATCHDOG_MAX_PER_MIN = 0.2
#: "`rate_limit` at most 0.2 per minute"
ACC_RATE_LIMIT_MAX_PER_MIN = 0.2
#: "no event of either kind in the first 2.0 s of at least 7 of 8 starts" (scaled to n starts by
#: ceil(n * 7 / 8) when there are more than 8).
ACC_CLEAN_START_NUM = 7
ACC_CLEAN_START_DEN = 8
#: "TTC and obstacle brakes still 0 at caps up to 1.0"
ACC_MAX_OBSTACLE_BRAKES = 0
ACC_OBSTACLE_CAP_MAX_MPS = 1.0
#: "and no wall contact"
ACC_MAX_WALL_CONTACTS = 0

# --- Section 4.3: GP3 ---------------------------------------------------------------------------
#: "20 consecutive laps" (laps.GP3_LAPS), "wheel distance at least 0.9 times 20 lap lengths"
#: (laps.GP3_WHEEL_DISTANCE_FACTOR).
#: "Fixed before the attempt: cap 1.0 m/s ... direction ccw first."
GP3_CAP_MPS = 1.0
GP3_FIRST_DIRECTION = "ccw"
#: "no wall contact"
GP3_MAX_CONTACTS = 0
#: "A BLOCKED hold longer than 5 s fails it."
GP3_BLOCKED_HOLD_MAX_S = 5.0
#: "nobody touches the car"
GP3_MAX_HAND_TOUCHES = 0

# --- Section 5.1: sim battery -------------------------------------------------------------------
#: Runs per condition and the "22 of 24" count with its scaling ("22 of 24 becomes 16 of 18").
BATTERY_REQUIRED_COMPLETE = {24: 22, 18: 16}
#: D-S0: "every system lap time is within 0.3 s of ground truth" (laps.LAP_SOURCE_TOLERANCE_S).
DS0_LAP_TOLERANCE_S = LAP_SOURCE_TOLERANCE_S
#: D-S1: "0 wall contacts in 24 runs" and "r at most 0.80".
DS_MAX_CONTACTS = 0
DS1_MAX_R = 0.80
#: D-S2: "median over runs of T_late(d) / T_late(c) ... at most 0.95".
DS2_MAX_MEDIAN_RATIO = 0.95
#: D-S2: "mean TTC or obstacle brakes per lap over laps 8 to 10 at most 0.2".
DS2_MAX_BRAKES_PER_LAP = 0.2

# --- Section 5.2: GP4 -----------------------------------------------------------------------------
#: "At least 5 runs with D on".
GP4_MIN_RUNS = 5
#: "the median r is at most 0.80".
GP4_MAX_MEDIAN_R = 0.80
#: "a 95 percent t interval".
GP4_LEVEL = 0.95

_OPS: dict[str, Callable[[Any, Any], bool]] = {
    "<=": operator.le,
    ">=": operator.ge,
    "<": operator.lt,
    ">": operator.gt,
    "==": operator.eq,
}


@dataclass(frozen=True)
class GateItem:
    item_id: str
    #: Pre-registration section (and the numbered item in it) this implements.
    section: str
    description: str
    #: The measured value; None when it could not be computed (the item then fails).
    measured: float | str | None
    comparator: str
    threshold: float | str | None
    passed: bool
    note: str = ""


@dataclass(frozen=True)
class GateResult:
    name: str
    section: str
    items: tuple[GateItem, ...]
    #: True when every threshold used equals the pre-registered module constant.
    preregistered: bool
    #: Named outcome of the section where it defines one.
    outcome: str = ""
    #: Numbers the section asks to report that are not pass or fail items.
    reported: dict[str, Any] = field(default_factory=dict)

    @property
    def passed(self) -> bool:
        return all(i.passed for i in self.items)

    def item(self, item_id: str) -> GateItem:
        for i in self.items:
            if i.item_id == item_id:
                return i
        raise KeyError(item_id)

    def to_dict(self) -> dict[str, Any]:
        return {
            "name": self.name,
            "section": self.section,
            "passed": self.passed,
            "outcome": self.outcome,
            "preregistered": self.preregistered,
            "items": [asdict(i) for i in self.items],
            "reported": self.reported,
        }


def _item(
    item_id: str,
    section: str,
    description: str,
    measured: float | str | None,
    comparator: str,
    threshold: float | str | None,
    note: str = "",
) -> GateItem:
    if measured is None or threshold is None:
        passed = False
    else:
        value = float(measured) if isinstance(measured, bool | int | float) else measured
        passed = bool(_OPS[comparator](value, threshold))
    return GateItem(item_id, section, description, measured, comparator, threshold, passed, note)


def _same(used: Mapping[str, Any], constants: Mapping[str, Any]) -> bool:
    return all(used[k] == constants[k] for k in constants)


def _interval_dict(i: Interval | None) -> dict[str, float] | None:
    return None if i is None else {"estimate": i.estimate, "lo": i.lo, "hi": i.hi, "level": i.level}


# =================================================================================== planner (4.2)


def planner_car_acceptance(
    rows: Sequence[RunInterventions],
    caps_mps: Sequence[float],
    wall_contacts: int,
    *,
    min_driving_s: float = ACC_MIN_DRIVING_S,
    min_starts: int = ACC_MIN_STARTS,
    watchdog_max_per_min: float = ACC_WATCHDOG_MAX_PER_MIN,
    rate_limit_max_per_min: float = ACC_RATE_LIMIT_MAX_PER_MIN,
    clean_start_num: int = ACC_CLEAN_START_NUM,
    clean_start_den: int = ACC_CLEAN_START_DEN,
    max_obstacle_brakes: int = ACC_MAX_OBSTACLE_BRAKES,
    max_wall_contacts: int = ACC_MAX_WALL_CONTACTS,
) -> GateResult:
    """Section 4.2 "C is accepted on the car". `rows` are the per-run counts by the counting rule,
    `caps_mps` the cap of each run, every run a standing start. Miss: the counts are reported and
    the next step goes to the owner (outcome "miss")."""
    if len(rows) != len(caps_mps):
        raise ValueError("one cap per run")
    sec = "4.2"
    driving_s = sum(r.duration_s for r in rows)
    minutes = driving_s / 60.0
    watchdog = sum(r.count("watchdog") for r in rows)
    rate_limit = sum(r.count("rate_limit") for r in rows)
    start_caps = [
        (r, c)
        for r, c in zip(rows, caps_mps, strict=True)
        if r.commanded and any(math.isclose(c, k) for k in ACC_START_CAPS_MPS)
    ]
    starts = [r for r, _ in start_caps]
    with_event, n_starts = starts_with_event(starts)
    clean = n_starts - with_event
    # 7 of 8 at 8 starts; for more starts the same share, rounded up.
    need_clean = -(-n_starts * clean_start_num // clean_start_den) if n_starts else 0
    caps_seen = {c for _, c in start_caps}
    obstacle = sum(
        r.obstacle
        for r, c in zip(rows, caps_mps, strict=True)
        if c <= ACC_OBSTACLE_CAP_MAX_MPS + 1e-9
    )

    def rate(count: int) -> float | None:
        return count / minutes if minutes > 0 else None

    items = (
        _item("4.2-a", sec, "at least 300 s of PnP driving", driving_s, ">=", min_driving_s),
        _item(
            "4.2-b",
            sec,
            "at least 8 standing starts at caps 0.5 and 1.0",
            n_starts,
            ">=",
            min_starts,
        ),
        _item(
            "4.2-c",
            sec,
            "starts at both caps 0.5 and 1.0",
            len(caps_seen & set(ACC_START_CAPS_MPS)),
            ">=",
            len(ACC_START_CAPS_MPS),
            note=f"caps seen {sorted(caps_seen)}",
        ),
        _item(
            "4.2-d",
            sec,
            "watchdog events per minute",
            rate(watchdog),
            "<=",
            watchdog_max_per_min,
            note=f"{watchdog} events",
        ),
        _item(
            "4.2-e",
            sec,
            "rate_limit events per minute",
            rate(rate_limit),
            "<=",
            rate_limit_max_per_min,
            note=f"{rate_limit} events",
        ),
        _item(
            "4.2-f",
            sec,
            "starts with no watchdog or rate_limit event in the first 2.0 s",
            clean,
            ">=",
            need_clean,
            note=f"{clean} of {n_starts} starts",
        ),
        _item(
            "4.2-g",
            sec,
            "TTC and obstacle brakes at caps up to 1.0",
            obstacle,
            "<=",
            max_obstacle_brakes,
        ),
        _item("4.2-h", sec, "wall contacts", wall_contacts, "<=", max_wall_contacts),
    )
    result = GateResult(
        name="planner_car_acceptance",
        section=sec,
        items=items,
        preregistered=_same(
            {
                "min_driving_s": min_driving_s,
                "min_starts": min_starts,
                "watchdog_max_per_min": watchdog_max_per_min,
                "rate_limit_max_per_min": rate_limit_max_per_min,
                "clean_start_num": clean_start_num,
                "clean_start_den": clean_start_den,
                "max_obstacle_brakes": max_obstacle_brakes,
                "max_wall_contacts": max_wall_contacts,
            },
            {
                "min_driving_s": ACC_MIN_DRIVING_S,
                "min_starts": ACC_MIN_STARTS,
                "watchdog_max_per_min": ACC_WATCHDOG_MAX_PER_MIN,
                "rate_limit_max_per_min": ACC_RATE_LIMIT_MAX_PER_MIN,
                "clean_start_num": ACC_CLEAN_START_NUM,
                "clean_start_den": ACC_CLEAN_START_DEN,
                "max_obstacle_brakes": ACC_MAX_OBSTACLE_BRAKES,
                "max_wall_contacts": ACC_MAX_WALL_CONTACTS,
            },
        ),
        reported={
            "driving_s": driving_s,
            "watchdog": watchdog,
            "rate_limit": rate_limit,
            "starts": n_starts,
            "starts_with_event": with_event,
        },
    )
    return replace(result, outcome="accepted" if result.passed else "miss")


# ========================================================================================= GP3 (4.3)


def gp3_decision(
    *,
    laps_counted: int,
    wheel_distance_m: float,
    lap_length_m: float,
    contacts: int,
    longest_blocked_hold_s: float,
    recoveries: int,
    cap_mps: float,
    direction: str,
    hand_touches: int = 0,
    first_attempt: bool = True,
    n_laps: int = GP3_LAPS,
    wheel_factor: float = GP3_WHEEL_DISTANCE_FACTOR,
    max_contacts: int = GP3_MAX_CONTACTS,
    blocked_hold_max_s: float = GP3_BLOCKED_HOLD_MAX_S,
    max_hand_touches: int = GP3_MAX_HAND_TOUCHES,
) -> GateResult:
    """Section 4.3 "One continuous run": 20 consecutive laps, one launch, nobody touches the car,
    no wall contact. `laps_counted` is the camera lap counter's count (`/lap/event`),
    `contacts` the touches seen by either person or on the video (the video decides a
    disagreement), `recoveries` the reverse maneuvers (counted, never a failure). The FIRST
    attempt at a configuration is the GP3 result for it; a later one is a repeat (reported)."""
    sec = "4.3"
    items = [
        _item("4.3-a", sec, "laps counted by the camera lap counter", laps_counted, ">=", n_laps),
        _item(
            "4.3-b",
            sec,
            "wheel distance at least 0.9 times 20 lap lengths",
            wheel_distance_m,
            ">=",
            wheel_factor * n_laps * lap_length_m,
            note=f"lap length {lap_length_m} m",
        ),
        _item("4.3-c", sec, "wall contacts (people or video)", contacts, "<=", max_contacts),
        _item(
            "4.3-d",
            sec,
            "longest BLOCKED hold in seconds",
            longest_blocked_hold_s,
            "<=",
            blocked_hold_max_s,
        ),
        _item(
            "4.3-e", sec, "speed cap fixed before the attempt in m/s", cap_mps, "==", GP3_CAP_MPS
        ),
        _item(
            "4.3-f",
            sec,
            "nobody touches the car (hand touches)",
            hand_touches,
            "<=",
            max_hand_touches,
        ),
    ]
    if first_attempt:
        items.append(
            _item(
                "4.3-g", sec, "direction of the first attempt", direction, "==", GP3_FIRST_DIRECTION
            )
        )
    result = GateResult(
        name="gp3",
        section=sec,
        items=tuple(items),
        preregistered=_same(
            {
                "n_laps": n_laps,
                "wheel_factor": wheel_factor,
                "max_contacts": max_contacts,
                "blocked_hold_max_s": blocked_hold_max_s,
                "max_hand_touches": max_hand_touches,
            },
            {
                "n_laps": GP3_LAPS,
                "wheel_factor": GP3_WHEEL_DISTANCE_FACTOR,
                "max_contacts": GP3_MAX_CONTACTS,
                "blocked_hold_max_s": GP3_BLOCKED_HOLD_MAX_S,
                "max_hand_touches": GP3_MAX_HAND_TOUCHES,
            },
        ),
        reported={
            "recoveries": recoveries,
            "first_attempt": first_attempt,
            "attempt_kind": "gp3_result" if first_attempt else "repeat",
        },
    )
    return replace(result, outcome="pass" if result.passed else "fail")


# ========================================================================================= GP4 (5.2)


def _contact_rate(runs: Sequence[RunLaps]) -> dict[str, Any]:
    laps = sum(len(r.usable) for r in runs)
    contacts = sum(r.contacts for r in runs)
    if laps == 0 or contacts > laps:
        return {"contacts": contacts, "laps": laps, "wilson": None}
    return {
        "contacts": contacts,
        "laps": laps,
        "wilson": _interval_dict(wilson_interval(contacts, laps)),
    }


def gp4_decision(
    on_runs: Sequence[RunLaps],
    off_runs: Sequence[RunLaps],
    *,
    min_runs: int = GP4_MIN_RUNS,
    n_laps: int = N_LAPS,
    max_median_r: float = GP4_MAX_MEDIAN_R,
    level: float = GP4_LEVEL,
) -> GateResult:
    """Section 5.2 "GP4 passes if", over the D-on runs, with `off_runs` the D-off runs at the
    lap-1 cap (condition a, same number of runs, interleaved) for item 3."""
    sec = "5.2"
    metrics = [run_metrics(r) for r in on_runs]
    done = [m for m in metrics if m is not None]
    n_dnf = sum(not r.completed for r in on_runs)
    no_positive = sum(m is None or m.delta <= 0.0 for m in metrics)
    mean_delta = t_interval([m.delta for m in done], level) if len(done) >= 2 else None
    median_r = float(np.median([m.ratio for m in done])) if done else None
    contacts_on = sum(r.contacts for r in on_runs)
    contacts_off = sum(r.contacts for r in off_runs)
    items = (
        _item("5.2-0a", sec, "at least 5 D-on runs", len(on_runs), ">=", min_runs),
        _item(
            "5.2-0b",
            sec,
            "same number of D-off runs (condition a)",
            len(off_runs),
            "==",
            len(on_runs),
        ),
        _item(
            "5.2-1",
            sec,
            "D-on runs that did not complete 10 laps (a DNF fails)",
            n_dnf,
            "==",
            0,
            note="; ".join(f"{r.run_id}: {r.dnf_cause}" for r in on_runs if not r.completed),
        ),
        _item(
            "5.2-2a",
            sec,
            "D-on runs whose Delta = T1 - T_late is not positive",
            no_positive,
            "==",
            0,
        ),
        _item(
            "5.2-2b",
            sec,
            "lower end of the 95 percent t interval of the mean Delta",
            None if mean_delta is None else mean_delta.lo,
            ">",
            0.0,
            note="needs at least 2 completed runs" if mean_delta is None else "",
        ),
        _item("5.2-2c", sec, "median r = T_late / T1", median_r, "<=", max_median_r),
        _item(
            "5.2-3",
            sec,
            "wall contacts D-on against D-off at the lap-1 cap",
            contacts_on,
            "<=",
            contacts_off,
        ),
    )
    result = GateResult(
        name="gp4",
        section=sec,
        items=items,
        preregistered=_same(
            {"min_runs": min_runs, "n_laps": n_laps, "max_median_r": max_median_r, "level": level},
            {
                "min_runs": GP4_MIN_RUNS,
                "n_laps": N_LAPS,
                "max_median_r": GP4_MAX_MEDIAN_R,
                "level": GP4_LEVEL,
            },
        ),
        reported={
            "mean_delta_s": _interval_dict(mean_delta),
            "median_r": median_r,
            "contacts_on": _contact_rate(on_runs),
            "contacts_off": _contact_rate(off_runs),
            "dnf": [(r.run_id, r.dnf_cause) for r in on_runs if not r.completed],
        },
    )
    return replace(result, outcome="pass" if result.passed else "fail")


# =============================================================================== sim battery (5.1)


@dataclass(frozen=True)
class SimRun:
    """One run of the sim battery. Lap times of the system (D-S0) and of ground truth (scoring)."""

    condition: str  # "a", "b", "c" or "d"
    case: str
    direction: str
    seed: int
    system_lap_times: tuple[float, ...]
    gt_lap_times: tuple[float, ...]
    wall_contacts: int
    #: TTC or obstacle brakes counted in each lap (index 0 is lap 1).
    brakes_per_lap: tuple[int, ...] = ()

    @property
    def key(self) -> tuple[str, str, int]:
        return (self.case, self.direction, self.seed)

    def scored(self) -> RunLaps:
        return RunLaps(
            run_id=f"{self.condition}/{self.case}/{self.direction}/{self.seed}",
            condition=self.condition,
            lap_times=self.gt_lap_times,
            contacts=self.wall_contacts,
        )


@dataclass(frozen=True)
class SimBatteryResult:
    ds0: GateResult
    ds1: GateResult
    ds2: GateResult
    #: Condition (b), the "just a slow start" control: reported, no threshold.
    control_b: dict[str, Any]

    def to_dict(self) -> dict[str, Any]:
        return {
            "D-S0": self.ds0.to_dict(),
            "D-S1": self.ds1.to_dict(),
            "D-S2": self.ds2.to_dict(),
            "control_b": self.control_b,
        }


def _by_condition(runs: Sequence[SimRun], condition: str) -> list[SimRun]:
    return [r for r in runs if r.condition == condition]


def _late(run: SimRun) -> float | None:
    m = run_metrics(run.scored())
    return None if m is None else m.t_late


def _completed_with(runs: Sequence[SimRun], max_r: float | None) -> int:
    n = 0
    for r in runs:
        m = run_metrics(r.scored())
        if m is not None and (max_r is None or m.ratio <= max_r):
            n += 1
    return n


def sim_battery_decision(
    runs: Sequence[SimRun],
    *,
    lap_tolerance_s: float = DS0_LAP_TOLERANCE_S,
    max_contacts: int = DS_MAX_CONTACTS,
    ds1_max_r: float = DS1_MAX_R,
    ds2_max_median_ratio: float = DS2_MAX_MEDIAN_RATIO,
    ds2_max_brakes_per_lap: float = DS2_MAX_BRAKES_PER_LAP,
) -> SimBatteryResult:
    """D-S0, D-S1 and D-S2 of section 5.1 from the battery runs of conditions a, c and d (and b
    for the reported control). Every condition used has 24 runs, or 18 when S4 could not load;
    the required count scales (22 of 24 becomes 16 of 18)."""
    sec = "5.1"
    cond = {c: _by_condition(runs, c) for c in "abcd"}
    for c in "acd":
        if not cond[c]:
            raise ValueError(f"no runs of condition {c}")
    sizes = {len(v) for v in cond.values() if v}
    if len(sizes) != 1 or next(iter(sizes)) not in BATTERY_REQUIRED_COMPLETE:
        raise ValueError(
            f"every condition needs the same 24 runs (18 without S4); got "
            f"{ {c: len(v) for c, v in cond.items()} }"
        )
    n = next(iter(sizes))
    need = BATTERY_REQUIRED_COMPLETE[n]
    same = _same(
        {
            "lap_tolerance_s": lap_tolerance_s,
            "max_contacts": max_contacts,
            "ds1_max_r": ds1_max_r,
            "ds2_max_median_ratio": ds2_max_median_ratio,
            "ds2_max_brakes_per_lap": ds2_max_brakes_per_lap,
        },
        {
            "lap_tolerance_s": DS0_LAP_TOLERANCE_S,
            "max_contacts": DS_MAX_CONTACTS,
            "ds1_max_r": DS1_MAX_R,
            "ds2_max_median_ratio": DS2_MAX_MEDIAN_RATIO,
            "ds2_max_brakes_per_lap": DS2_MAX_BRAKES_PER_LAP,
        },
    )

    # D-S0: lap bookkeeping over the runs of (a).
    a = cond["a"]
    count_mismatch = sum(len(r.system_lap_times) != len(r.gt_lap_times) for r in a)
    diffs = [
        abs(s - g)
        for r in a
        if len(r.system_lap_times) == len(r.gt_lap_times)
        for s, g in zip(r.system_lap_times, r.gt_lap_times, strict=True)
    ]
    max_diff = max(diffs) if diffs else None
    ds0_items = (
        _item(
            "D-S0-1",
            sec,
            "runs of (a) whose lap count differs from ground truth",
            count_mismatch,
            "==",
            0,
        ),
        _item(
            "D-S0-2",
            sec,
            "largest system lap time error against ground truth in s",
            max_diff,
            "<=",
            lap_tolerance_s,
            note="compared on runs with equal lap counts",
        ),
    )
    ds0 = GateResult(
        "D-S0",
        sec,
        ds0_items,
        same,
        outcome="lap_source_ok" if all(i.passed for i in ds0_items) else "fail",
    )

    # D-S1: D v0 (condition c).
    c = cond["c"]
    contacts_c = sum(r.wall_contacts for r in c)
    ds1_items = (
        _item("D-S1-1", sec, f"wall contacts in {n} runs", contacts_c, "<=", max_contacts),
        _item(
            "D-S1-2",
            sec,
            f"runs completing 10 laps with r at most {ds1_max_r}",
            _completed_with(c, ds1_max_r),
            ">=",
            need,
            note=f"{need} of {n}",
        ),
    )
    ds1 = GateResult(
        "D-S1",
        sec,
        ds1_items,
        same,
        outcome="d_v0_on_car" if all(i.passed for i in ds1_items) else "fail",
    )

    # D-S2: D v1 (condition d) against D v0, paired by case, direction and seed.
    d = cond["d"]
    late_c = {r.key: _late(r) for r in c}
    pairs = [(_late(r), late_c.get(r.key)) for r in d]
    ratios = [x / y for x, y in pairs if x is not None and y is not None]
    median_ratio = float(np.median(ratios)) if ratios else None
    late_laps = [
        b
        for r in d
        if len(r.gt_lap_times) >= LATE_LAST_LAP and len(r.brakes_per_lap) >= LATE_LAST_LAP
        for b in r.brakes_per_lap[LATE_FIRST_LAP - 1 : LATE_LAST_LAP]
    ]
    brakes = float(np.mean(late_laps)) if late_laps else None
    safety = (
        _item(
            "D-S2-1",
            sec,
            f"wall contacts in {n} runs",
            sum(r.wall_contacts for r in d),
            "<=",
            max_contacts,
        ),
        _item(
            "D-S2-2",
            sec,
            "runs completing 10 laps",
            _completed_with(d, None),
            ">=",
            need,
            note=f"{need} of {n}",
        ),
        _item(
            "D-S2-4",
            sec,
            "mean TTC or obstacle brakes per lap over laps 8 to 10",
            brakes,
            "<=",
            ds2_max_brakes_per_lap,
        ),
    )
    ratio_item = _item(
        "D-S2-3",
        sec,
        "median T_late(d) / T_late(c), paired by case, direction and seed",
        median_ratio,
        "<=",
        ds2_max_median_ratio,
        note=f"{len(ratios)} pairs of {n}",
    )
    ds2_items = (safety[0], safety[1], ratio_item, safety[2])
    if all(i.passed for i in ds2_items):
        outcome = "d_v1_on_car"
    elif all(i.passed for i in safety):
        outcome = "no_better_than_schedule"
    else:
        outcome = "fail"
    ds2 = GateResult("D-S2", sec, ds2_items, same, outcome=outcome)

    b = cond["b"]
    late_b = [x for x in (_late(r) for r in b) if x is not None]
    control = {
        "runs": len(b),
        "wall_contacts": sum(r.wall_contacts for r in b),
        "completed_10_laps": _completed_with(b, None),
        "median_t_late_s": float(np.median(late_b)) if late_b else None,
        "median_t_late_c_s": _median_late(c),
        "median_t_late_d_s": _median_late(d),
    }
    return SimBatteryResult(ds0, ds1, ds2, control)


def _median_late(runs: Sequence[SimRun]) -> float | None:
    vals = [x for x in (_late(r) for r in runs) if x is not None]
    return float(np.median(vals)) if vals else None
