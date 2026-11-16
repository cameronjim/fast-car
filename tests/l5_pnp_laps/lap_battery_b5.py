"""Battery B5 (obstacle variants in sim) and measurement M-A2 (does a perfect track change lap time)
of the v0.4 pre-registration (docs/notes/preregistration-pnp-v0.4.md sections 3.5 and 5, and
addendum 1 section 3.3): run loading and the decisions D4-S0, D4-S1, D4-S2 and the M-A2 reading, as
pure functions.

Every threshold is read from `evaluation/prereg/pnp-v0.4.yaml` (blocks `lap_battery_b5` and
`oracle_track`), whose SHA-256 is checked against the value written in the pre-registration, or
from `evaluation/prereg/pnp-v0.4-addendum-1.yaml` (`oracle_extensions`). Nothing is copied into
this file. A threshold that cannot be applied as written is reported, never edited.

D4-S0, D4-S1 and D4-S2 are the D3 rules of v0.3 section 1.4 on B5 with the B5 counts: the D3
functions of `lap_battery_b4` are reused for D4-S1 and the shared items, and D4-S0 and D4-S2 differ
only in their condition letters (c and v). Scoring is by ground truth crossing stamps (v0.3 1.3).
"""

from __future__ import annotations

import dataclasses
import hashlib
import json
import math
import re
import statistics
from dataclasses import dataclass, field
from pathlib import Path
from typing import Any

import lap_battery_b4 as b4
import yaml
from lap_battery import Decision, RunSummary

REPO_ROOT = Path(__file__).resolve().parents[2]
PREREG_YAML = REPO_ROOT / "evaluation" / "prereg" / "pnp-v0.4.yaml"
PREREG_NOTE = REPO_ROOT / "docs" / "notes" / "preregistration-pnp-v0.4.md"
ADDENDUM_YAML = REPO_ROOT / "evaluation" / "prereg" / "pnp-v0.4-addendum-1.yaml"
B5_CONDITIONS = ("a", "b", "c", "v")
MA2_CONDITIONS = ("g", "o", "f")
#: v0.4 section 3.5: laps 3 to 6 are scored (1-based).
PreregError = b4.PreregError
MissingPairError = b4.MissingPairError
RunFileError = b4.RunFileError


def registered_sha(note_path: Path = PREREG_NOTE) -> str:
    """The SHA-256 of the v0.4 constants file as written in the pre-registration."""
    text = note_path.read_text()
    m = re.search(r"Constants file: `[^`]*pnp-v0\.4\.yaml`,\s*SHA-256\s*`([0-9a-f]{64})`", text)
    if m is None:
        raise PreregError(f"no constants file SHA-256 found in {note_path}")
    return m.group(1)


@dataclass(frozen=True)
class Constants:
    """The `lap_battery_b5` and `oracle_track` blocks of the v0.4 yaml and the SHA-256 they were
    read under. Has the interface of `lap_battery_b4.Constants` that the D3 functions use."""

    yaml_sha256: str
    raw: dict[str, Any]
    oracle: dict[str, Any]

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
        """15 of 16, or 11 of 12 with O4 dropped. Nothing else is registered."""
        if n_runs == 16:
            return int(self.th["required_complete_of_16"])
        if n_runs == 12:
            return int(self.th["required_complete_of_12"])
        raise PreregError(f"{n_runs} runs in a condition: the pre-registration has 16 or 12")


def load_constants(
    yaml_path: Path = PREREG_YAML, note_path: Path = PREREG_NOTE, expected_sha: str | None = None
) -> Constants:
    """The `lap_battery_b5` and `oracle_track` blocks, refused when the file's SHA-256 is not the
    registered one."""
    expected = expected_sha if expected_sha is not None else registered_sha(note_path)
    actual = hashlib.sha256(yaml_path.read_bytes()).hexdigest()
    if actual != expected:
        raise PreregError(
            f"{yaml_path} has SHA-256 {actual}, the pre-registration says {expected}: refusing"
        )
    doc = yaml.safe_load(yaml_path.read_text())
    return Constants(actual, doc["lap_battery_b5"], doc["oracle_track"])


def load_extensions(path: Path = ADDENDUM_YAML) -> dict[str, Any]:
    """`oracle_extensions` of addendum 1 (M-A2x and M-A2w). The addendum's own SHA-256 is checked
    against the addendum note by the caller of `load_extensions_checked`."""
    return dict(yaml.safe_load(path.read_text())["oracle_extensions"])


def load_extensions_checked(
    path: Path = ADDENDUM_YAML,
    note_path: Path = REPO_ROOT / "docs" / "notes" / "preregistration-pnp-v0.4-addendum-1.md",
) -> dict[str, Any]:
    text = note_path.read_text()
    m = re.search(r"pnp-v0\.4-addendum-1\.yaml`,?\s*SHA-256\s*`([0-9a-f]{64})`", text)
    if m is None:
        raise PreregError(f"no addendum constants SHA-256 found in {note_path}")
    actual = hashlib.sha256(path.read_bytes()).hexdigest()
    if actual != m.group(1):
        raise PreregError(f"{path} has SHA-256 {actual}, the addendum says {m.group(1)}: refusing")
    return load_extensions(path)


# -- run loading -------------------------------------------------------------------------------------


def summarise_run(result: dict, condition: str, case: str, direction: str, seed: int) -> RunSummary:
    """A RunSummary on the by-crossing ground truth (as B4), plus the B5 diagnostics."""
    run = b4.summarise_b4_run(result, condition, case, direction, seed)
    extras = dict(run.extras)
    extras.update(
        {
            "real_time_factor": result.get("real_time_factor"),
            "pnp_ticks": result.get("pnp_ticks"),
            "obstacle_passes": result.get("obstacle_passes"),
            "innovation_rejections": result.get("innovation_rejections"),
            "model_id": (result.get("pnp") or {})
            .get("track_perception_node", {})
            .get("bundle_dir"),
        }
    )
    return dataclasses.replace(run, extras=extras)


def load_runs(
    results_dir: Path, mode: str, conditions: tuple[str, ...], meta_key: str = "battery_b5"
) -> list[RunSummary]:
    """Every `<condition>_<case>_<direction>_<seed>.json` of a directory; each must carry the
    `meta_key` block with the mode asked for and the by-crossing ground truth."""
    out = []
    for path in sorted(results_dir.glob("*.json")):
        parts = path.stem.split("_")
        if len(parts) != 4 or parts[0] not in conditions:
            continue
        condition, case, direction, seed = parts
        data = json.loads(path.read_text())
        meta = data.get(meta_key)
        if not isinstance(meta, dict) or meta.get("mode") != mode:
            raise RunFileError(f"{path.name}: not a {meta_key} {mode} run file")
        out.append(summarise_run(data, condition, case, direction, int(seed)))
    return out


# -- D4-S0, D4-S1, D4-S2 -----------------------------------------------------------------------------


def d4_s0(runs: list[RunSummary], const: Constants | None = None) -> Decision:
    """The rule of D3-S0 on B5: S0a on condition a, S0b on c and v (each present condition)."""
    const = const or load_constants()
    b4._check_unique(runs)
    tol = float(const.th["lap_time_tolerance_s"])
    min_share = float(const.th["s0b_min_share_within_tolerance"])
    max_single = float(const.th["s0b_max_single_error_s"])
    items: dict[str, tuple[bool | None, str]] = {}
    a = b4._need(runs, "a", "S0a")
    st = b4._lap_count_and_time_stats(a, tol)
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
    for cond in ("c", "v"):
        rs = b4.of_condition(runs, cond)
        if not rs:
            if cond == "c":
                raise MissingPairError("no condition (c) runs: needed for S0b(c)")
            continue  # v is omitted when the owner gave no answer (said in the report)
        st = b4._lap_count_and_time_stats(rs, tol)
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
    return b4._decision("D4-S0", items)


def d4_s1(runs: list[RunSummary], const: Constants | None = None) -> Decision:
    """The rule of D3-S1 on B5 (condition c against its paired control b), 15 of 16 or 11 of 12."""
    const = const or load_constants()
    dec = b4.d3_s1(runs, const)  # reads max_contacts, s1_*, and const.required_complete
    return dataclasses.replace(dec, name="D4-S1")


def d4_s2(runs: list[RunSummary], const: Constants | None = None) -> Decision:
    """The rule of D3-S2 for condition v against condition c, same thresholds, 15 of 16."""
    const = const or load_constants()
    b4._check_unique(runs)
    v = b4._need(runs, "v", "D4-S2")
    c = b4._need(runs, "c", "D4-S2 (the paired schedule)")
    pairs = b4.paired(v, c, "c")
    th = const.th
    items: dict[str, tuple[bool | None, str]] = {}
    items[f"{int(th['max_contacts'])} wall contacts in the {len(v)} runs of condition v"] = (
        b4._contacts_item(v, int(th["max_contacts"]))
    )
    items["runs of condition v complete 10 laps"] = b4._complete_item(v, const)
    ratios = [
        x.t_late_s / ref.t_late_s
        for x, ref in pairs
        if x.completed and ref.completed and ref.t_late_s > 0
    ]
    median = statistics.median(ratios) if ratios else math.nan
    rmax = float(th["s2_max_median_ratio_to_c"])
    median_item = f"median T_late(v) / T_late(c) <= {rmax}"
    items[median_item] = (
        bool(ratios) and median <= rmax,
        f"median {median:.3f} over {len(ratios)} of {len(v)} pairs",
    )
    mean_brakes, n_late = b4._brakes_mean(v)
    bmax = float(th["s2_max_brakes_per_lap_late"])
    items[f"mean TTC or obstacle brakes per lap, laps 8 to 10, <= {bmax}"] = (
        bool(n_late) and mean_brakes <= bmax,
        f"{mean_brakes:.3f} over {n_late} runs",
    )
    safety_ok = all(ok for name, (ok, _) in items.items() if name != median_item)
    note = "no better than the schedule in sim" if safety_ok and not items[median_item][0] else ""
    return b4._decision("D4-S2", items, note)


# -- B5 report extras ----------------------------------------------------------------------------------


def per_case_medians(runs: list[RunSummary]) -> list[str]:
    """Per condition and case: runs, completions, contacts, median T1 and T_late over the runs
    that completed. Reported without a threshold."""
    lines = [
        "| cond | case | runs | complete | contacts | median T1 s | median T_late s | median r |"
    ]
    lines.append("|---|---|---|---|---|---|---|---|")
    keys = sorted({(r.condition, r.case) for r in runs})
    for cond, case in keys:
        rs = [r for r in runs if r.condition == cond and r.case == case]
        done = [r for r in rs if r.completed]

        def med(values: list[float]) -> str:
            return f"{statistics.median(values):.2f}" if values else "n/a"

        lines.append(
            f"| {cond} | {case} | {len(rs)} | {len(done)} | {sum(r.contacts for r in rs)} | "
            f"{med([r.t1_s for r in done])} | {med([r.t_late_s for r in done])} | "
            f"{med([r.r for r in done])} |"
        )
    return lines


def innovation_per_lap(runs: list[RunSummary], case: str = "O1") -> list[str]:
    """Without a threshold: on the round donut, how many crossing candidates the innovation gate
    rejected, per lap, for each run that carries the count (the lap node instrumented by
    `lap_node_counting.py`)."""
    lines = []
    for r in sorted(runs, key=lambda x: (x.condition, x.direction, x.seed)):
        if r.case != case:
            continue
        rej = r.extras.get("innovation_rejections")
        if not isinstance(rej, dict) or "per_lap" not in rej:
            lines.append(f"{r.condition}/{r.direction}/{r.seed}: not counted in this run")
            continue
        lines.append(
            f"{r.condition}/{r.direction}/{r.seed}: rejected per lap {rej['per_lap']} "
            f"(total {rej.get('total')}, candidates accepted {rej.get('accepted_total')})"
        )
    return lines or [f"no {case} runs"]


def obstacle_clearance_lines(runs: list[RunSummary]) -> list[str]:
    """Without a threshold: for each obstacle, the minimum planned clearance while passing it, per
    case and condition (the smallest over the runs), and over what number of ticks."""
    best: dict[tuple[str, str, int], tuple[float, int]] = {}
    for r in runs:
        for p in r.extras.get("obstacle_passes") or []:
            key = (r.case, r.condition, int(p["index"]))
            clear = float(p["min_plan_clearance_m"])
            ticks = int(p["ticks"])
            old = best.get(key)
            if old is None or (not math.isnan(clear) and (math.isnan(old[0]) or clear < old[0])):
                best[key] = (clear, ticks + (old[1] if old else 0))
            elif old is not None:
                best[key] = (old[0], old[1] + ticks)
    if not best:
        return ["no obstacle passes recorded"]
    return [
        f"{case} condition {cond} obstacle {idx}: min planned clearance {clear:.3f} m "
        f"over {ticks} passing ticks (all runs of that case and condition)"
        for (case, cond, idx), (clear, ticks) in sorted(best.items())
    ]


def run_table(runs: list[RunSummary]) -> str:
    return b4.run_table(runs)


@dataclass
class B5Report:
    decisions: dict[str, Decision]
    extra: list[str] = field(default_factory=list)


def build_report(runs: list[RunSummary], const: Constants | None = None) -> B5Report:
    const = const or load_constants()
    decisions = {"D4-S0": d4_s0(runs, const), "D4-S1": d4_s1(runs, const)}
    extra: list[str] = []
    if b4.of_condition(runs, "v"):
        decisions["D4-S2"] = d4_s2(runs, const)
    else:
        extra.append("condition v was not run (no owner answer on the D v1 variant): no D4-S2")
    extra += ["", "Per case medians:", *per_case_medians(runs), "", "Innovation gate on O1:"]
    extra += innovation_per_lap(runs)
    extra += ["", "Planned clearance past each obstacle:", *obstacle_clearance_lines(runs)]
    return B5Report(decisions, extra)


# -- M-A2, M-A2x ---------------------------------------------------------------------------------------


def scored_median(run: RunSummary, laps: list[int]) -> float:
    """Median lap time over the given 1-based laps; NaN when the run lacks one of them."""
    if run.error or run.laps < max(laps):
        return math.nan
    return statistics.median(run.gt_lap_times_s[i - 1] for i in laps)


@dataclass(frozen=True)
class Pair:
    case: str
    direction: str
    t_ref: float
    t_x: float
    contacts_ref: int
    contacts_x: int
    done_ref: bool
    done_x: bool

    @property
    def ratio(self) -> float:
        return self.t_x / self.t_ref if self.t_ref > 0 else math.nan

    @property
    def valid(self) -> bool:
        return not math.isnan(self.ratio)


def pairs_against(
    runs: list[RunSummary],
    x_cond: str,
    ref_cond: str,
    const: Constants,
    cases: list[str] | None = None,
) -> list[Pair]:
    """(x over ref) per (case, direction, seed) of `cases` (default: the M-A2 cases of the yaml):
    the median lap time of the scored laps. A missing partner raises."""
    b4._check_unique(runs)
    wanted = list(const.oracle["cases"]) if cases is None else cases
    scored = [int(k) for k in const.oracle["scored_laps"]]
    xs = [r for r in b4.of_condition(runs, x_cond) if r.case in wanted]
    refs = [r for r in b4.of_condition(runs, ref_cond) if r.case in wanted]
    if not xs or not refs:
        raise MissingPairError(f"need runs of conditions {x_cond} and {ref_cond}")
    unpaired = {r.key for r in xs} ^ {r.key for r in refs}
    if unpaired:
        raise MissingPairError(
            f"runs of {x_cond} and {ref_cond} do not pair up: {sorted(unpaired)[:6]}"
        )
    need = int(const.oracle["laps"])
    out = []
    for x, ref in b4.paired(xs, refs, ref_cond):
        out.append(
            Pair(
                x.case,
                x.direction,
                scored_median(ref, scored),
                scored_median(x, scored),
                ref.contacts,
                x.contacts,
                not ref.error and ref.laps >= need,
                not x.error and x.laps >= need,
            )
        )
    return sorted(out, key=lambda p: (p.case, p.direction))


def ma2_reading(pairs: list[Pair], const: Constants) -> tuple[str, str]:
    """(sentence, numbers) of the registered reading, over the valid pairs. Registered (section
    3.5, yaml `oracle_track`): median ratio at least 0.97 and no pair below 0.93 is "no measured
    path"; median at most 0.95 is "matters"; otherwise "undecided"."""
    valid = [p.ratio for p in pairs if p.valid]
    if not valid:
        return "undecided", "no valid pair"
    med = statistics.median(valid)
    lo, hi = min(valid), max(valid)
    no_path = float(const.oracle["no_path_if_median_ratio_at_least"])
    floor = float(const.oracle["and_no_pair_below"])
    matters = float(const.oracle["matters_if_median_ratio_at_most"])
    numbers = f"median ratio {med:.3f}, min {lo:.3f}, max {hi:.3f} over {len(valid)} of {len(pairs)} pairs"
    if med >= no_path and lo >= floor:
        sentence = "no measured path"
    elif med <= matters:
        sentence = "matters"
    else:
        sentence = "undecided"
    return sentence, numbers


def pair_table(pairs: list[Pair], x_name: str, ref_name: str) -> str:
    lines = [
        (
            f"| case | dir | median T3-6 {ref_name} s | median T3-6 {x_name} s | "
            f"ratio {x_name}/{ref_name} | contacts {ref_name} | contacts {x_name} | "
            f"complete {ref_name} | complete {x_name} |"
        ),
        "|---|---|---|---|---|---|---|---|---|",
    ]
    for p in pairs:
        lines.append(
            f"| {p.case} | {p.direction} | {p.t_ref:.2f} | {p.t_x:.2f} | {p.ratio:.3f} | "
            f"{p.contacts_ref} | {p.contacts_x} | {'yes' if p.done_ref else 'NO'} | "
            f"{'yes' if p.done_x else 'NO'} |"
        )
    return "\n".join(lines)


def ma2_report(runs: list[RunSummary], const: Constants | None = None) -> str:
    const = const or load_constants()
    pairs = pairs_against(runs, "o", "g", const)
    sentence, numbers = ma2_reading(pairs, const)
    lines = [pair_table(pairs, "o", "g"), "", f"M-A2: {numbers}", f"Registered reading: {sentence}"]
    bad = [p for p in pairs if not p.valid or not (p.done_ref and p.done_x)]
    if bad:
        lines.append(
            "INCOMPLETE PAIRS (reading computed over the valid pairs only, not decided here): "
            + ", ".join(f"{p.case}/{p.direction}" for p in bad)
        )
    lines.append(
        f"contacts: g {sum(p.contacts_ref for p in pairs)}, o {sum(p.contacts_x for p in pairs)}"
    )
    return "\n".join(lines)


def tick_stats(runs: list[RunSummary], condition: str, cases: list[str]) -> dict[str, float]:
    """Mean over runs of the mean track confidence and the share of ticks held at the speed cap."""
    ticks = [r.extras.get("pnp_ticks") for r in b4.of_condition(runs, condition) if r.case in cases]
    ticks = [t for t in ticks if isinstance(t, dict) and t.get("driving_ticks")]
    if not ticks:
        return {"runs": 0, "mean_track_confidence": math.nan, "share_cap_limited": math.nan}
    return {
        "runs": len(ticks),
        "mean_track_confidence": statistics.fmean(t["mean_track_confidence"] for t in ticks),
        "share_cap_limited": statistics.fmean(t["share_cap_limited"] for t in ticks),
    }


def od_table(runs: list[RunSummary], od_case: str = "OD") -> list[str]:
    """Every run of the obstacle development case, all conditions: lap time (median of laps 3 to
    6), contacts, completion, mean track confidence and the share of ticks held at the cap."""
    lines = [
        (
            "| cond | dir | seed | laps | median T3-6 s | contacts | complete | "
            "mean track conf | share at cap | min planned clearance m |"
        ),
        "|---|---|---|---|---|---|---|---|---|---|",
    ]
    for r in sorted(runs, key=lambda x: (x.case, x.condition, x.direction, x.seed)):
        if r.case != od_case:
            continue
        t = r.extras.get("pnp_ticks") or {}
        lines.append(
            f"| {r.condition} | {r.direction} | {r.seed} | {r.laps} | "
            f"{scored_median(r, [3, 4, 5, 6]):.2f} | {r.contacts} | "
            f"{'yes' if (not r.error and r.laps >= 6) else 'NO'} | "
            f"{t.get('mean_track_confidence', math.nan):.3f} | "
            f"{t.get('share_cap_limited', math.nan):.3f} | "
            f"{t.get('min_plan_clearance_m', math.nan):.3f} |"
        )
    return lines


def ma2x_report(runs: list[RunSummary], f_label: str, const: Constants | None = None) -> str:
    """M-A2x: condition f (the frozen learned candidate, `f_label` says which) next to g and o on
    the M-A2 pairs, and all three on OD. No threshold, nothing decided."""
    const = const or load_constants()
    cases = list(const.oracle["cases"])
    lines = [f"condition f = {f_label}", ""]
    for x, ref in (("f", "g"), ("o", "g"), ("f", "o")):
        pairs = pairs_against(runs, x, ref, const)
        _s, numbers = ma2_reading(pairs, const)
        lines += [f"## {x} over {ref}", "", pair_table(pairs, x, ref), "", numbers, ""]
    lines += ["## Track confidence and speed cap on the M-A2 cases, mean over runs", ""]
    for cond in MA2_CONDITIONS:
        s = tick_stats(runs, cond, cases)
        lines.append(
            f"- {cond}: {s['runs']} runs, mean track confidence {s['mean_track_confidence']:.3f}, "
            f"share of ticks held at the speed cap {s['share_cap_limited']:.3f}"
        )
    lines += ["", "## Obstacle development case OD, all conditions", "", *od_table(runs), ""]
    lines += [
        "Cost terms of the planner's final sequence and the share of ticks per speed limiting",
        "term: NOT AVAILABLE. PlannerStatus carries no cost terms, brief 42's PlannerTiming is not",
        "on main, and planner_replay_cli writes none. Not computed; the planner was not changed.",
    ]
    return "\n".join(lines)
