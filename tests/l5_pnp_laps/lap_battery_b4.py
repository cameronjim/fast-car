"""Battery B4 of the v0.3 pre-registration (docs/notes/preregistration-pnp-v0.3.md, section 1):
run loading and the decisions D3-S0, D3-S1, D3-S2 (variants d and e), the v0.2 D-S1 rule for the
record and the owner table, as pure functions.

Every threshold is read from `evaluation/prereg/pnp-v0.3.yaml` (block `lap_battery`), whose SHA-256
is checked against the value written in the pre-registration. Nothing is copied into this file.
A threshold that cannot be applied as written is reported, never edited.

Definitions, as pre-registered (1.3 and 1.4):

* Lap times are GROUND TRUTH BY CROSSING: lap k is the time between ground-truth crossing k-1 and
  k of the start line, crossing 0 being the stamp of the first /scan after the planner first
  reported DRIVING (`gt_lap_times_by_crossing_s` of the runner's result). A run JSON that lacks
  that field is refused. The system's own lap times (`system_lap_events`) are compared to it by
  D3-S0 only.
* Per run: T1 is lap 1, T_late the median of laps 8 to 10, r = T_late / T1. A run COMPLETES when
  it has at least 10 ground-truth laps and no error.
* Paired runs share (case, direction, seed). A missing pair is an error, not a skip.
* 22 of 24 becomes 16 of 18 when N4 was dropped (the yaml's two numbers). Any other run count
  per condition is refused: no run is ever dropped silently.
"""

from __future__ import annotations

import hashlib
import json
import math
import re
import statistics
from dataclasses import dataclass, field
from pathlib import Path
from typing import Any

import yaml
from lap_battery import Decision, RunSummary

REPO_ROOT = Path(__file__).resolve().parents[2]
PREREG_YAML = REPO_ROOT / "evaluation" / "prereg" / "pnp-v0.3.yaml"
PREREG_NOTE = REPO_ROOT / "docs" / "notes" / "preregistration-pnp-v0.3.md"

LATE_LAPS = (8, 9, 10)  # 1-based lap indices of T_late (pre-registration 1.3)
LAPS_TO_COMPLETE = 10
OBSTACLE_SOURCES = ("ttc", "ttc_reverse")
CONDITIONS = ("a", "b", "c", "d", "e")
OWNER_ANSWERS = ("confirms", "rejects", "undecided")


class PreregError(ValueError):
    """The pre-registration cannot be applied as written (hash mismatch, unknown case, ...)."""


class MissingPairError(PreregError):
    """A run has no paired run it must be compared with."""


class RunFileError(PreregError):
    """A run JSON that must not be scored (no by-crossing truth, wrong checkout, ...)."""


# -- the constants file ----------------------------------------------------------------------------


def sha256_file(path: Path) -> str:
    return hashlib.sha256(path.read_bytes()).hexdigest()


def registered_sha(note_path: Path = PREREG_NOTE) -> str:
    """The SHA-256 of the constants file as written in the pre-registration."""
    text = note_path.read_text()
    m = re.search(r"Constants file: `[^`]*pnp-v0\.3\.yaml`,\s*SHA-256\s*`([0-9a-f]{64})`", text)
    if m is None:
        raise PreregError(f"no constants file SHA-256 found in {note_path}")
    return m.group(1)


@dataclass(frozen=True)
class Constants:
    """The `lap_battery` block of the yaml and the SHA-256 it was read under."""

    yaml_sha256: str
    raw: dict[str, Any]

    @property
    def battery_cases(self) -> dict[str, dict]:
        return dict(self.raw["cases"])

    @property
    def development_cases(self) -> dict[str, dict]:
        return dict(self.raw["development"]["cases"])

    @property
    def battery_seeds(self) -> tuple[int, ...]:
        return tuple(int(s) for s in self.raw["seeds"])

    @property
    def development_seeds(self) -> tuple[int, ...]:
        return tuple(int(s) for s in self.raw["development"]["seeds"])

    @property
    def directions(self) -> tuple[str, ...]:
        return tuple(self.raw["directions"])

    @property
    def laps(self) -> int:
        return int(self.raw["laps"])

    @property
    def lap1_cap_mps(self) -> float:
        return float(self.raw["lap1_cap_mps"])

    @property
    def conditions(self) -> dict[str, dict]:
        return dict(self.raw["conditions"])

    @property
    def th(self) -> dict[str, float]:
        return dict(self.raw["thresholds"])

    def required_complete(self, n_runs: int) -> int:
        """22 of 24, or 16 of 18 with N4 dropped. Nothing else is registered."""
        if n_runs == 24:
            return int(self.th["required_complete_of_24"])
        if n_runs == 18:
            return int(self.th["required_complete_of_18"])
        raise PreregError(f"{n_runs} runs in a condition: the pre-registration has 24 or 18")


def load_constants(
    yaml_path: Path = PREREG_YAML, note_path: Path = PREREG_NOTE, expected_sha: str | None = None
) -> Constants:
    """The yaml's `lap_battery` block, refused when its SHA-256 is not the registered one."""
    expected = expected_sha if expected_sha is not None else registered_sha(note_path)
    actual = sha256_file(yaml_path)
    if actual != expected:
        raise PreregError(
            f"{yaml_path} has SHA-256 {actual}, the pre-registration says {expected}: refusing"
        )
    block = yaml.safe_load(yaml_path.read_text())["lap_battery"]
    return Constants(actual, block)


# -- runs -------------------------------------------------------------------------------------------


def summarise_b4_run(
    result: dict, condition: str, case: str, direction: str, seed: int
) -> RunSummary:
    """A RunSummary on the by-crossing ground truth. Refuses a result that lacks it."""
    if "gt_lap_times_by_crossing_s" not in result and "gt_lap_times_registered_s" not in result:
        raise RunFileError(
            f"{condition}_{case}_{direction}_{seed}: no gt_lap_times_by_crossing_s "
            "(the runner of brief 29 is required)"
        )
    # Brief 48: a run of the registered-truth runner carries `gt_lap_times_registered_s`, which
    # is read first; the stored B4 files have only the older field, read as before.
    gt_field = (
        "gt_lap_times_registered_s"
        if "gt_lap_times_registered_s" in result
        else "gt_lap_times_by_crossing_s"
    )
    gt = tuple(float(t) for t in result[gt_field])
    error = str(result.get("error", ""))
    if result.get("truth_valid") is False:
        # the truth recording does not cover crossing 0: not a result, a run that did not execute
        reason = f"truth invalid: {result.get('truth_invalid_reason', 'unknown')}"
        error = f"{error}; {reason}" if error else reason
    system = tuple(float(e["lap_time_s"]) for e in result.get("system_lap_events", []))
    brakes = tuple(
        sum(per.get(src, 0) for src in OBSTACLE_SOURCES) for per in result.get("brake_per_lap", [])
    )
    status = result.get("lap_status") or {}
    floor = result.get("guidance_floor") or {}
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
        error=error,
        extras={
            "floor_messages": int(floor.get("messages_with_stations", 0)),
            "floor_active_messages": int(floor.get("messages_floor_active", 0)),
            "zones_created": status.get("zones_created"),
            "caution_zones": status.get("caution_zones"),
            "max_gap_s": (result.get("drive_raw") or {}).get("max_gap_s"),
        },
    )


def load_b4_runs(results_dir: Path, mode: str = "battery") -> list[RunSummary]:
    """Every `<condition>_<case>_<direction>_<seed>.json` of a B4 directory (condition a to e).

    Every file must carry the `battery_b4` block the runner wrapper writes, with the mode asked
    for, and the by-crossing ground truth."""
    out = []
    for path in sorted(results_dir.glob("*.json")):
        parts = path.stem.split("_")
        if len(parts) != 4 or parts[0] not in CONDITIONS:
            continue
        condition, case, direction, seed = parts
        data = json.loads(path.read_text())
        meta = data.get("battery_b4")
        if not isinstance(meta, dict) or meta.get("mode") != mode:
            raise RunFileError(f"{path.name}: not a B4 {mode} run file")
        out.append(summarise_b4_run(data, condition, case, direction, int(seed)))
    return out


# -- helpers ----------------------------------------------------------------------------------------


def _label(r: RunSummary) -> str:
    return f"{r.case}/{r.direction}/{r.seed}"


def of_condition(runs: list[RunSummary], condition: str) -> list[RunSummary]:
    return [r for r in runs if r.condition == condition]


def _check_unique(runs: list[RunSummary]) -> None:
    keys = [(r.condition, *r.key) for r in runs]
    if len(set(keys)) != len(keys):
        raise PreregError("two runs with the same condition, case, direction and seed")


def _need(runs: list[RunSummary], condition: str, what: str) -> list[RunSummary]:
    rs = of_condition(runs, condition)
    if not rs:
        raise MissingPairError(f"no condition ({condition}) runs: needed for {what}")
    return rs


def paired(
    runs_x: list[RunSummary], runs_ref: list[RunSummary], ref_name: str
) -> list[tuple[RunSummary, RunSummary]]:
    """(x, reference) for every x. A missing reference raises: it is never skipped."""
    ref = {r.key: r for r in runs_ref}
    out = []
    for x in runs_x:
        if x.key not in ref:
            raise MissingPairError(
                f"condition ({x.condition}) run {_label(x)} has no paired condition ({ref_name}) run"
            )
        out.append((x, ref[x.key]))
    return out


def _ok(value: bool) -> bool:
    return bool(value)


def _contacts_item(runs: list[RunSummary], max_contacts: int) -> tuple[bool, str]:
    contacts = sum(r.contacts for r in runs)
    with_contact = [_label(r) for r in runs if r.contacts]
    return contacts <= max_contacts, f"{contacts} contacts, runs with a contact: {with_contact[:8]}"


def _complete_item(runs: list[RunSummary], const: Constants) -> tuple[bool, str]:
    need = const.required_complete(len(runs))
    done = [r for r in runs if r.completed]
    missing = [_label(r) for r in runs if not r.completed]
    return len(done) >= need, f"{len(done)} of {len(runs)} complete, not complete: {missing[:8]}"


def _decision(name: str, items: dict[str, tuple[bool | None, str]], note: str = "") -> Decision:
    return Decision(name, all(ok for ok, _ in items.values()), items, note)


# -- D3-S0 --------------------------------------------------------------------------------------------


def _lap_count_and_time_stats(runs: list[RunSummary], tol: float) -> dict[str, Any]:
    count_bad = [
        f"{_label(r)}: system {len(r.system_lap_times_s)} vs truth {r.laps}"
        for r in runs
        if len(r.system_lap_times_s) != r.laps
    ]
    errs: list[tuple[str, float]] = []
    total_laps = 0
    within = 0
    for r in runs:
        total_laps += r.laps
        for k, (s, g) in enumerate(zip(r.system_lap_times_s, r.gt_lap_times_s), start=1):
            e = abs(s - g)
            errs.append((f"{_label(r)} lap {k}: system {s:.2f} vs truth {g:.2f}", e))
            within += e <= tol
    outside = [(t, e) for t, e in errs if e > tol]
    return {
        "count_bad": count_bad,
        "total": total_laps,
        "within": within,
        "outside": outside,
        "max_error": max((e for _, e in errs), default=0.0),
        "errors": [f"{_label(r)}: {r.error[:80]}" for r in runs if r.error],
    }


def d3_s0(runs: list[RunSummary], const: Constants | None = None) -> Decision:
    """S0a (condition a) and S0b (c, d and e each), pre-registration 1.4."""
    const = const or load_constants()
    _check_unique(runs)
    tol = float(const.th["lap_time_tolerance_s"])
    min_share = float(const.th["s0b_min_share_within_tolerance"])
    max_single = float(const.th["s0b_max_single_error_s"])
    items: dict[str, tuple[bool | None, str]] = {}

    a = _need(runs, "a", "S0a")
    st = _lap_count_and_time_stats(a, tol)
    items["S0a: system lap count equals ground truth in every condition (a) run"] = (
        not st["count_bad"],
        f"{len(a) - len(st['count_bad'])} of {len(a)} runs agree {st['count_bad'][:6]}",
    )
    items[f"S0a: every system lap time within {tol} s of ground truth"] = (
        not st["outside"],
        f"{len(st['outside'])} of {st['total']} laps outside {[t for t, _ in st['outside']][:8]}",
    )
    items["S0a: no condition (a) run failed to execute"] = (
        not st["errors"],
        f"{len(st['errors'])} errors {st['errors'][:4]}",
    )
    for cond in ("c", "d", "e"):
        rs = _need(runs, cond, f"S0b({cond})")
        st = _lap_count_and_time_stats(rs, tol)
        share = st["within"] / st["total"] if st["total"] else 0.0
        items[f"S0b({cond}): lap count equal in every run"] = (
            not st["count_bad"],
            f"{len(rs) - len(st['count_bad'])} of {len(rs)} runs agree {st['count_bad'][:6]}",
        )
        items[f"S0b({cond}): at least {min_share:.0%} of laps within {tol} s"] = (
            share >= min_share,
            (
                f"{st['within']} of {st['total']} laps within ({share:.4f}); outside: "
                f"{[t for t, _ in st['outside']][:8]}"
            ),
        )
        items[f"S0b({cond}): no lap off by more than {max_single} s"] = (
            st["max_error"] <= max_single,
            f"largest error {st['max_error']:.2f} s",
        )
        items[f"S0b({cond}): no run failed to execute"] = (
            not st["errors"],
            f"{len(st['errors'])} errors {st['errors'][:4]}",
        )
    return _decision("D3-S0", items)


# -- D3-S1 --------------------------------------------------------------------------------------------


def d3_s1(runs: list[RunSummary], const: Constants | None = None) -> Decision:
    """Items 1 to 4 of 1.4 on condition c, paired with condition b."""
    const = const or load_constants()
    _check_unique(runs)
    c = _need(runs, "c", "D3-S1")
    b = _need(runs, "b", "D3-S1 (the paired control)")
    pairs = paired(c, b, "b")
    th = const.th
    items: dict[str, tuple[bool | None, str]] = {}
    items[f"1. {int(th['max_contacts'])} wall contacts in the {len(c)} runs of condition c"] = (
        _contacts_item(c, int(th["max_contacts"]))
    )
    items["2. runs of condition c complete 10 laps"] = _complete_item(c, const)
    need = const.required_complete(len(c))
    ratio_max = float(th["s1_reach_max_ratio_to_b"])
    reach = []
    for cr, br in pairs:
        # NaN (a run that did not complete, either side) compares False: not reached
        reach.append((_label(cr), cr.t_late_s / br.t_late_s if br.t_late_s > 0 else math.nan))
    reached = [lab for lab, q in reach if q <= ratio_max]
    items[f"3. reach: T_late(c) <= {ratio_max} x T_late(b) in at least {need} of {len(c)} runs"] = (
        len(reached) >= need,
        f"{len(reached)} of {len(c)} reach; ratios {sorted(round(q, 3) for _, q in reach)}",
    )
    done = [r for r in c if r.completed]
    not_below = [_label(r) for r in done if not r.t_late_s < r.t1_s]
    median_r = statistics.median(r.r for r in done) if done else math.nan
    r_max = float(th["s1_max_median_r"])
    items["4a. T_late below T1 in every completing run"] = (
        bool(done) and not not_below,
        f"{len(not_below)} runs not below {not_below[:8]}",
    )
    items[f"4b. median r <= {r_max}"] = (
        bool(done) and median_r <= r_max,
        f"median r {median_r:.3f} over {len(done)} completing runs",
    )
    return _decision("D3-S1", items)


# -- D3-S2 and the report-only items ------------------------------------------------------------------


def _brakes_mean(runs: list[RunSummary]) -> tuple[float, int]:
    late = [r.late_brakes_per_lap for r in runs if not math.isnan(r.late_brakes_per_lap)]
    return (statistics.fmean(late) if late else math.nan), len(late)


def d3_s2(runs: list[RunSummary], variant: str, const: Constants | None = None) -> Decision:
    """D v1 variant "d" or "e", paired with condition c (1.4)."""
    if variant not in ("d", "e"):
        raise ValueError(f"variant must be 'd' or 'e', got {variant!r}")
    const = const or load_constants()
    _check_unique(runs)
    v = _need(runs, variant, f"D3-S2({variant})")
    c = _need(runs, "c", f"D3-S2({variant}) (the paired schedule)")
    pairs = paired(v, c, "c")
    th = const.th
    items: dict[str, tuple[bool | None, str]] = {}
    items[
        f"{int(th['max_contacts'])} wall contacts in the {len(v)} runs of condition {variant}"
    ] = _contacts_item(v, int(th["max_contacts"]))
    items[f"runs of condition {variant} complete 10 laps"] = _complete_item(v, const)
    ratios = [
        x.t_late_s / ref.t_late_s
        for x, ref in pairs
        if x.completed and ref.completed and ref.t_late_s > 0
    ]
    median = statistics.median(ratios) if ratios else math.nan
    rmax = float(th["s2_max_median_ratio_to_c"])
    items[f"median T_late({variant}) / T_late(c) <= {rmax}"] = (
        bool(ratios) and median <= rmax,
        f"median {median:.3f} over {len(ratios)} of {len(v)} pairs",
    )
    mean_brakes, n_late = _brakes_mean(v)
    bmax = float(th["s2_max_brakes_per_lap_late"])
    items[f"mean TTC or obstacle brakes per lap, laps 8 to 10, <= {bmax}"] = (
        bool(n_late) and mean_brakes <= bmax,
        f"{mean_brakes:.3f} over {n_late} runs",
    )
    safety_ok = all(ok for name, (ok, _) in items.items() if "median" not in name)
    note = ""
    if safety_ok and not items[f"median T_late({variant}) / T_late(c) <= {rmax}"][0]:
        note = "no better than the schedule in sim"
    return _decision(f"D3-S2({variant})", items, note)


def v02_ds1_for_record(runs: list[RunSummary], const: Constants | None = None) -> Decision:
    """The v0.2 D-S1 rule on condition c, for the record. It decides nothing."""
    const = const or load_constants()
    _check_unique(runs)
    c = _need(runs, "c", "the v0.2 D-S1 record")
    th = const.th
    need = const.required_complete(len(c))
    r_max = float(th["v02_ds1_max_r_reported_only"])
    items: dict[str, tuple[bool | None, str]] = {
        f"{int(th['max_contacts'])} wall contacts in {len(c)} runs": _contacts_item(
            c, int(th["max_contacts"])
        )
    }
    done = [r for r in c if r.completed]
    good = [r for r in done if r.r <= r_max]
    items[f"at least {need} of {len(c)} runs complete 10 laps with r <= {r_max}"] = (
        len(good) >= need,
        (
            f"{len(done)} complete, {len(good)} with r <= {r_max}; r: "
            f"{sorted(round(r.r, 2) for r in done)}"
        ),
    )
    d = _decision("v0.2 D-S1 rule (for the record, decides nothing)", items)
    return d


def d_variants_report(runs: list[RunSummary]) -> list[str]:
    """Reported without a threshold: T_late(d) / T_late(e) paired, contacts and brakes of each,
    the share of guidance messages whose active bound was the floor, merged zones in d."""
    d = of_condition(runs, "d")
    e = of_condition(runs, "e")
    if not d or not e:
        return ["no condition d and e runs"]
    lines = []
    ratios = [x.t_late_s / y.t_late_s for x, y in paired(d, e, "e") if x.completed and y.completed]
    lines.append(
        f"T_late(d) / T_late(e), paired: median "
        f"{statistics.median(ratios) if ratios else math.nan:.3f} over {len(ratios)} pairs "
        f"(min {min(ratios, default=math.nan):.3f}, max {max(ratios, default=math.nan):.3f})"
    )
    for name, rs in (("d", d), ("e", e)):
        mean_brakes, n_late = _brakes_mean(rs)
        lines.append(
            f"condition {name}: {sum(r.contacts for r in rs)} contacts, "
            f"{sum(sum(r.brakes_per_lap) for r in rs)} obstacle brakes over all laps, "
            f"{mean_brakes:.3f} per lap over laps 8 to 10 ({n_late} runs)"
        )
    msgs = sum(r.extras.get("floor_messages", 0) for r in d)
    active = sum(r.extras.get("floor_active_messages", 0) for r in d)
    lines.append(
        f"d: floor was the active bound in {active} of {msgs} guidance messages with stations "
        f"({active / msgs if msgs else math.nan:.4f})"
    )
    created = [r.extras.get("zones_created") for r in d]
    kept = [r.extras.get("caution_zones") for r in d]
    if all(isinstance(x, int) for x in created + kept):
        merged = sum(created) - sum(kept)  # type: ignore[arg-type]
        lines.append(
            f"d: zones created {sum(created)}, zones standing at the end {sum(kept)}, "
            f"merged zones (created minus standing) {merged}"  # type: ignore[arg-type]
        )
    else:
        lines.append("d: the lap status did not carry the zone counters")
    return lines


# -- the owner table --------------------------------------------------------------------------------


def car_variant(owner: str, d_pass: bool, e_pass: bool) -> str:
    """The table of pre-registration 1.4, assuming D3-S0 and D3-S1 passed first.

    owner confirms the floor and merge: d if d passes, else e if e passes, else D v0.
    owner rejects: e if e passes, else D v0. No decision yet: D v0 only."""
    if owner not in OWNER_ANSWERS:
        raise ValueError(f"owner must be one of {OWNER_ANSWERS}, got {owner!r}")
    if owner == "undecided":
        return "v0"
    if owner == "confirms" and d_pass:
        return "d"
    return "e" if e_pass else "v0"


# -- the report ---------------------------------------------------------------------------------------


def run_table(runs: list[RunSummary]) -> str:
    lines = [
        (
            "| cond | case | dir | seed | laps | T1 s | T_late s | r | contacts | obstacle brakes "
            "per lap (late) | complete |"
        ),
        "|---|---|---|---|---|---|---|---|---|---|---|",
    ]
    for r in sorted(runs, key=lambda x: (x.condition, x.case, x.direction, x.seed)):
        lines.append(
            f"| {r.condition} | {r.case} | {r.direction} | {r.seed} | {r.laps} | {r.t1_s:.2f} | "
            f"{r.t_late_s:.2f} | {r.r:.3f} | {r.contacts} | {r.late_brakes_per_lap:.2f} | "
            f"{'yes' if r.completed else 'NO ' + r.error[:40]} |"
        )
    return "\n".join(lines)


@dataclass
class B4Report:
    decisions: dict[str, Decision]
    extra: list[str] = field(default_factory=list)
    car: dict[str, str] = field(default_factory=dict)  # owner answer -> what goes on the car


def build_report(runs: list[RunSummary], const: Constants | None = None) -> B4Report:
    const = const or load_constants()
    decisions = {
        "D3-S0": d3_s0(runs, const),
        "D3-S1": d3_s1(runs, const),
        "D3-S2(d)": d3_s2(runs, "d", const),
        "D3-S2(e)": d3_s2(runs, "e", const),
        "v0.2 D-S1 (record)": v02_ds1_for_record(runs, const),
    }
    d_pass = bool(decisions["D3-S2(d)"].passed)
    e_pass = bool(decisions["D3-S2(e)"].passed)
    gates_ok = bool(decisions["D3-S0"].passed) and bool(decisions["D3-S1"].passed)
    car = {}
    for owner in OWNER_ANSWERS:
        v = car_variant(owner, d_pass, e_pass)
        if not gates_ok:
            v = f"no D on the car (D3-S0 or D3-S1 failed); table row would give {v}"
        car[owner] = v
    return B4Report(decisions, d_variants_report(runs), car)
