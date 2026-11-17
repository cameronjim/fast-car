"""Battery B8 of the lap gate v3 pre-registration (docs/notes/preregistration-pnp-v0.5-laps-ring.md
sections 2 to 4): run loading and the decisions D8-S0 and D8-S1 as pure functions.

Every threshold is read from `evaluation/prereg/pnp-v0.5-laps-ring.yaml` (block `lap_battery_b8`),
whose SHA-256 is a constant of this file and is checked on every load. D8-S1 calls D3-S1
(`lap_battery_b4.d3_s1`) with the yaml's counts (33 of 36, 28 of 30, 22 of 24); D3-S1's contacts
threshold (0) comes from `pnp-v0.3.yaml` through `lap_battery_b4.load_constants`, which checks that
file's hash too. A threshold that cannot be applied as written is reported, never edited.

What differs from B7 (`lap_battery_b7.py`), all of it written in the pre-registration:

* two classes of case, fixed by the track: aliased (R1 to R4: no straight longer than the lane is
  wide) and placed (Q1, Q2). The class tolerance of a lap against its truth lap T: placed 0.3 s,
  aliased the larger of 0.3 s and 0.04 T; the single-error limit of condition c: placed 1.0 s,
  aliased the larger of 1.0 s and 0.08 T;
* the count rule: the number of system crossings stamped at or before truth crossing 12 plus half
  of truth lap 12 is exactly 12 (a system lap that does not exist is outside every tolerance);
* 13 ground-truth laps are driven, laps 1 to 12 are judged, and a run whose truth recording ends
  before the count window does is a run that failed to execute;
* the drop rule: a case is dropped if condition a cannot complete 3 laps on it in either direction
  with seed 61, at most two cases, at least two aliased and one placed case must be left, else
  nothing is decided.
"""

from __future__ import annotations

import dataclasses
import hashlib
import json
import math
import statistics
from collections import Counter
from dataclasses import dataclass
from pathlib import Path
from typing import Any

import lap_battery_b4 as b4
import lap_battery_b7 as b7
import yaml
from lap_battery import Decision, RunSummary

REPO_ROOT = Path(__file__).resolve().parents[2]
PREREG_YAML = REPO_ROOT / "evaluation" / "prereg" / "pnp-v0.5-laps-ring.yaml"
PREREG_NOTE = REPO_ROOT / "docs" / "notes" / "preregistration-pnp-v0.5-laps-ring.md"
V05_LAPS_YAML = REPO_ROOT / "evaluation" / "prereg" / "pnp-v0.5-laps.yaml"
V04_YAML = REPO_ROOT / "evaluation" / "prereg" / "pnp-v0.4.yaml"
#: SHA-256 of `pnp-v0.5-laps-ring.yaml` as committed with the pre-registration (commit 1 of brief
#: 50). Checked on every load and written into every run and the report.
REGISTERED_YAML_SHA256 = "74b1dccd0d30910a7e96a45af65a00de62446902380bfb2bae76512d0498dc36"
#: The B7 file the ring file names (`v05_laps_sha256`).
V05_LAPS_SHA256 = b7.REGISTERED_YAML_SHA256
B8_CONDITIONS = ("a", "b", "c")
B8_SEEDS = (61, 62, 63)
B8_DIRECTIONS = ("ccw", "cw")
ALIASED = "aliased"
PLACED = "placed"
#: Development cases outside the ring yaml: P4 of B7 (v0.5 laps yaml, burned as an evaluation case)
#: and OD of B5 (v0.4 yaml, development there).
EXTERNAL_DEVELOPMENT = (("P4", V05_LAPS_YAML, ("lap_battery_b7", "cases")),
                        ("OD", V04_YAML, ("lap_battery_b5", "development", "cases")))  # fmt: skip

PreregError = b4.PreregError
MissingPairError = b4.MissingPairError
RunFileError = b4.RunFileError


def sha256_file(path: Path) -> str:
    return hashlib.sha256(path.read_bytes()).hexdigest()


def class_of_spec(spec: dict[str, Any]) -> str:
    """The class of a track, from the yaml's `class` or, for a development case without one, by the
    rule of the pre-registration: aliased when the centreline has no straight longer than the lane
    is wide."""
    declared = spec.get("class")
    if declared is not None:
        if declared not in (ALIASED, PLACED):
            raise PreregError(f"class {declared!r} is neither {ALIASED} nor {PLACED}")
        return str(declared)
    straight = float(spec.get("straight_length_m", 0.0))
    return PLACED if straight > float(spec["lane_width_m"]) else ALIASED


@dataclass(frozen=True)
class Constants:
    """The `lap_battery_b8` block of the yaml, the SHA-256 it was read under, the whole document
    (the `d8_c0` block and the owner decisions) and the one number D8-S1 takes from D3
    (`max_contacts`). Has the interface of `lap_battery_b4.Constants` that the D3 functions use."""

    yaml_sha256: str
    raw: dict[str, Any]
    doc: dict[str, Any]
    max_contacts: int
    external_cases: dict[str, dict[str, Any]]

    @property
    def battery_cases(self) -> dict[str, dict]:
        return dict(self.raw["cases"])

    @property
    def development_cases(self) -> dict[str, dict]:
        """ND of the ring yaml, P4 and OD: the cases development may run (seeds 11 to 13)."""
        cases = dict(self.raw["development"]["cases"])
        cases.update(self.external_cases)
        return cases

    @property
    def development_seeds(self) -> tuple[int, ...]:
        return tuple(int(s) for s in self.raw["development"]["seeds"])

    @property
    def battery_seeds(self) -> tuple[int, ...]:
        return tuple(int(s) for s in self.raw["seeds"])

    @property
    def directions(self) -> tuple[str, ...]:
        return tuple(self.raw["directions"])

    @property
    def laps_judged(self) -> int:
        return int(self.raw["laps_judged"])

    @property
    def laps_driven(self) -> int:
        return int(self.raw["laps_driven"])

    @property
    def laps(self) -> int:  # the name the D3 functions and the runner command read
        return self.laps_driven

    @property
    def lap1_cap_mps(self) -> float:
        return float(self.raw["lap1_cap_mps"])

    @property
    def conditions(self) -> dict[str, dict]:
        return dict(self.raw["conditions"])

    @property
    def drop_rule(self) -> dict[str, Any]:
        return dict(self.raw["drop_rule"])

    @property
    def validity(self) -> dict[str, Any]:
        return dict(self.raw["validity"])

    @property
    def count_rule(self) -> dict[str, Any]:
        return dict(self.raw["count_rule"])

    @property
    def thresholds(self) -> dict[str, Any]:
        return dict(self.raw["thresholds"])

    @property
    def th(self) -> dict[str, float]:
        """The thresholds under the names the D3 functions read."""
        t = self.raw["thresholds"]
        return {
            "max_contacts": float(self.max_contacts),
            "s1_reach_max_ratio_to_b": float(t["s1_reach_ratio"]),
            "s1_max_median_r": float(t["s1_median_r_max"]),
        }

    def class_of(self, case: str) -> str:
        spec = self.battery_cases.get(case) or self.development_cases.get(case)
        if spec is None:
            raise PreregError(f"case {case!r} is not in the yaml")
        return class_of_spec(spec)

    def required_complete(self, n_runs: int) -> int:
        """33 of 36, 28 of 30 after one dropped case, 22 of 24 after two. Nothing else is
        registered."""
        table = self.raw["thresholds"]["s1_min_complete"]
        for key, n in (("of_36", 36), ("of_30", 30), ("of_24", 24)):
            if n_runs == n:
                return int(table[key])
        raise PreregError(f"{n_runs} runs in a condition: the pre-registration has 36, 30 or 24")

    def tolerance_s(self, klass: str, truth_lap_s: float) -> float:
        """The tolerance of one lap against its truth lap time T."""
        t = self.raw["thresholds"][klass]
        absolute = float(t["tolerance_s"])
        if klass == PLACED:
            return absolute
        return max(absolute, float(t["tolerance_share_of_truth_lap"]) * truth_lap_s)

    def max_single_s(self, klass: str, truth_lap_s: float) -> float:
        """The largest single error allowed in condition c."""
        t = self.raw["thresholds"][klass]
        absolute = float(t["max_single_error_s"])
        if klass == PLACED:
            return absolute
        return max(absolute, float(t["max_single_error_share_of_truth_lap"]) * truth_lap_s)


def _check_consistent(doc: dict[str, Any]) -> None:
    block = doc["lap_battery_b8"]
    t = block["thresholds"]
    if t["s1_reach_min_runs"] != t["s1_min_complete"]:
        raise PreregError("s1_reach_min_runs differs from s1_min_complete: D3-S1 uses one number")
    if int(t["s1_complete_laps"]) != b4.LAPS_TO_COMPLETE:
        raise PreregError(f"s1_complete_laps is not {b4.LAPS_TO_COMPLETE}")
    if tuple(int(k) for k in t["late_laps"]) != b4.LATE_LAPS:
        raise PreregError(f"late_laps is not {list(b4.LATE_LAPS)}")
    if tuple(int(s) for s in block["seeds"]) != B8_SEEDS:
        raise PreregError(f"seeds are not {list(B8_SEEDS)}")
    if tuple(block["directions"]) != B8_DIRECTIONS:
        raise PreregError(f"directions are not {list(B8_DIRECTIONS)}")
    if doc.get("v05_laps_sha256") != V05_LAPS_SHA256:
        raise PreregError(f"v05_laps_sha256 is not {V05_LAPS_SHA256}")
    for name, spec in block["cases"].items():
        class_of_spec(spec)  # the class is stated for every battery case
        if "class" not in spec:
            raise PreregError(f"case {name} has no class")


def load_constants(
    yaml_path: Path = PREREG_YAML,
    expected_sha: str = REGISTERED_YAML_SHA256,
    v05_path: Path = V05_LAPS_YAML,
    v04_path: Path = V04_YAML,
) -> Constants:
    """The yaml's `lap_battery_b8` block, refused when its SHA-256 is not the registered one."""
    actual = sha256_file(yaml_path)
    if actual != expected_sha:
        raise PreregError(
            f"{yaml_path} has SHA-256 {actual}, the registered value is {expected_sha}: refusing"
        )
    doc = yaml.safe_load(yaml_path.read_text())
    _check_consistent(doc)
    d3 = b4.load_constants()  # checks the v0.3 file's hash against its pre-registration
    external: dict[str, dict[str, Any]] = {}
    for name, path, keys in EXTERNAL_DEVELOPMENT:
        use = v05_path if path == V05_LAPS_YAML else v04_path
        if use == v05_path and sha256_file(use) != V05_LAPS_SHA256:
            raise PreregError(f"{use} is not the registered v0.5 laps file")
        node: Any = yaml.safe_load(use.read_text())
        for key in keys:
            node = node[key]
        external[name] = dict(node[name])
    return Constants(actual, doc["lap_battery_b8"], doc, int(d3.th["max_contacts"]), external)


# -- runs ----------------------------------------------------------------------------------------------


def summarise_run(
    result: dict, condition: str, case: str, direction: str, seed: int, const: Constants
) -> RunSummary:
    """A RunSummary on the registered truth, with the truth crossings, the system crossings and the
    line sources in `extras`. A result without the registered truth is refused. A run whose truth
    recording ends before the count window does is a run that failed to execute: its `error` says
    so."""
    name = f"{condition}_{case}_{direction}_{seed}"
    if "gt_lap_times_registered_s" not in result or "truth_valid" not in result:
        raise RunFileError(
            f"{name}: no gt_lap_times_registered_s and truth_valid "
            "(the runner of brief 48 is required)"
        )
    run = b4.summarise_b4_run(result, condition, case, direction, seed)
    truth_crossings = tuple(float(c) for c in result.get("gt_lap_crossings_registered_s", []))
    system = result.get("system_lap_events") or []
    status = result.get("lap_status") or {}
    last = result.get("truth_last_odom_s")
    extras = {
        "truth_crossings_s": truth_crossings,
        "system_crossings_s": tuple(float(e["t_cross_s"]) for e in system),
        "truth_last_odom_s": None if last is None else float(last),
        "line_sources": tuple(status.get("lap_line_sources") or ()),
        "lap_status": dict(status),
    }
    run.extras.update(extras)
    n_window = const.laps_judged
    laps = run.gt_lap_times_s
    if len(truth_crossings) >= n_window and len(laps) >= n_window and not run.error:
        window_end = (
            truth_crossings[n_window - 1] + float(const.count_rule["share"]) * laps[n_window - 1]
        )
        if last is None or float(last) < window_end:
            run = dataclasses.replace(
                run, error=f"recording ends before the count window ({last} < {window_end:.2f})"
            )
            run.extras.update(extras)
    return run


def load_runs(results_dir: Path, const: Constants, mode: str = "battery") -> list[RunSummary]:
    """Every `<condition>_<case>_<direction>_<seed>.json` of a B8 directory (condition a, b, c).
    Each must carry the `battery_b8` block with the mode asked for."""
    out = []
    for path in sorted(results_dir.glob("*.json")):
        parts = path.stem.split("_")
        if len(parts) != 4 or parts[0] not in B8_CONDITIONS:
            continue
        condition, case, direction, seed = parts
        data = json.loads(path.read_text())
        meta = data.get("battery_b8")
        if not isinstance(meta, dict) or meta.get("mode") != mode:
            raise RunFileError(f"{path.name}: not a B8 {mode} run file")
        out.append(summarise_run(data, condition, case, direction, int(seed), const))
    return out


# -- the count rule and the laps -----------------------------------------------------------------------


def count_rule(run: RunSummary, const: Constants) -> tuple[bool, str]:
    """The number of system crossings stamped at or before truth crossing 12 plus half of truth
    lap 12 is exactly 12. A run that has no truth crossing 12 fails."""
    n = const.laps_judged
    crossings = run.extras.get("truth_crossings_s", ())
    system = run.extras.get("system_crossings_s", ())
    if len(crossings) < n or run.laps < n:
        return False, f"{len(crossings)} truth crossings, {n} needed"
    end = float(crossings[n - 1]) + float(const.count_rule["share"]) * run.gt_lap_times_s[n - 1]
    counted = sum(1 for t in system if t <= end)
    required = int(const.count_rule["required"])
    return counted == required, f"{counted} crossings by {end:.2f} s (required {required})"


def judged_errors(run: RunSummary, const: Constants) -> list[tuple[int, float, float, float]]:
    """(lap, error, tolerance, single-error limit) for laps 1 to `laps_judged`. A system lap that
    does not exist, or is NaN, has an infinite error (outside every tolerance)."""
    klass = const.class_of(run.case)
    out = []
    for k in range(1, const.laps_judged + 1):
        if k > len(run.gt_lap_times_s):
            break
        truth = run.gt_lap_times_s[k - 1]
        error = math.inf
        if k <= len(run.system_lap_times_s):
            e = abs(run.system_lap_times_s[k - 1] - truth)
            error = math.inf if math.isnan(e) else e
        out.append((k, error, const.tolerance_s(klass, truth), const.max_single_s(klass, truth)))
    return out


def _label(r: RunSummary) -> str:
    return b4._label(r)


def _lap_stats(runs: list[RunSummary], const: Constants) -> dict[str, Any]:
    count_bad = []
    outside: list[str] = []
    beyond: list[str] = []
    total = within = 0
    max_over_limit = 0.0
    for r in runs:
        ok, detail = count_rule(r, const)
        if not ok:
            count_bad.append(f"{_label(r)}: {detail}")
        for k, error, tol, limit in judged_errors(r, const):
            total += 1
            if error <= tol:
                within += 1
            else:
                outside.append(f"{_label(r)} lap {k}: error {error:.2f} s, tolerance {tol:.2f} s")
            if error > limit:
                beyond.append(f"{_label(r)} lap {k}: error {error:.2f} s, limit {limit:.2f} s")
            max_over_limit = max(max_over_limit, error - limit)
    return {
        "count_bad": count_bad,
        "total": total,
        "within": within,
        "outside": outside,
        "beyond": beyond,
        "errors": [f"{_label(r)}: {r.error[:80]}" for r in runs if r.error],
    }


# -- D8-S0 and D8-S1 ------------------------------------------------------------------------------------


def d8_s0(runs: list[RunSummary], const: Constants | None = None) -> Decision:
    """S0a on condition a, S0b on condition c (pre-registration section 4). Lap 1 is inside both
    items with no allowance; the tolerance is the class tolerance."""
    const = const or load_constants()
    b4._check_unique(runs)
    min_share = float(const.thresholds["s0b_min_share_within_tolerance"])
    items: dict[str, tuple[bool | None, str]] = {}

    a = b4._need(runs, "a", "S0a")
    st = _lap_stats(a, const)
    items["S0a: the count rule holds in every condition (a) run"] = (
        not st["count_bad"],
        f"{len(a) - len(st['count_bad'])} of {len(a)} runs agree {st['count_bad'][:6]}",
    )
    items["S0a: every judged lap within its class tolerance"] = (
        not st["outside"],
        f"{len(st['outside'])} of {st['total']} laps outside {st['outside'][:6]}",
    )
    items["S0a: no condition (a) run failed to execute"] = (
        not st["errors"],
        f"{len(st['errors'])} errors {st['errors'][:4]}",
    )
    c = b4._need(runs, "c", "S0b(c)")
    st = _lap_stats(c, const)
    share = st["within"] / st["total"] if st["total"] else 0.0
    items["S0b(c): the count rule holds in every run"] = (
        not st["count_bad"],
        f"{len(c) - len(st['count_bad'])} of {len(c)} runs agree {st['count_bad'][:6]}",
    )
    items[f"S0b(c): at least {min_share:.0%} of judged laps within their class tolerance"] = (
        share >= min_share,
        f"{st['within']} of {st['total']} laps within ({share:.4f}); outside {st['outside'][:6]}",
    )
    items["S0b(c): no lap beyond its single-error limit"] = (
        not st["beyond"],
        f"{len(st['beyond'])} laps beyond {st['beyond'][:6]}",
    )
    items["S0b(c): no run failed to execute"] = (
        not st["errors"],
        f"{len(st['errors'])} errors {st['errors'][:4]}",
    )
    return b4._decision("D8-S0", items)


def d8_s1(runs: list[RunSummary], const: Constants | None = None) -> Decision:
    """D3-S1's four items on condition c against its paired control b, with the counts of the
    yaml (33 of 36)."""
    const = const or load_constants()
    return dataclasses.replace(b4.d3_s1(runs, const), name="D8-S1")  # type: ignore[arg-type]


# -- the drop rule ---------------------------------------------------------------------------------------


@dataclass(frozen=True)
class DropResult:
    dropped: tuple[str, ...]
    decided: bool
    reason: str
    laps: dict[str, int]

    @property
    def runs_per_condition(self) -> int:
        return 0 if not self.decided else 6 * (6 - len(self.dropped))


def drop_rule(laps_by_run: dict[str, int], const: Constants) -> DropResult:
    """The registered drop rule. `laps_by_run` maps `<case>_<direction>` (condition a, seed 61) to
    the ground-truth laps of its accepted run (0 for a run that failed to execute or has an invalid
    truth). A case is dropped if either direction has fewer than `min_laps`. Nothing is decided if
    more than `max_dropped` cases would go, or fewer than two aliased or no placed case is left."""
    rule = const.drop_rule
    need = int(rule["min_laps"])
    cases = list(const.battery_cases)
    dropped: list[str] = []
    for case in cases:
        counts = [laps_by_run.get(f"{case}_{d}") for d in const.directions]
        if any(n is None or n < need for n in counts):
            dropped.append(case)
    left = [c for c in cases if c not in dropped]
    aliased = sum(1 for c in left if const.class_of(c) == ALIASED)
    placed = sum(1 for c in left if const.class_of(c) == PLACED)
    reason = ""
    if len(dropped) > int(rule["max_dropped"]):
        reason = f"{len(dropped)} cases would be dropped, at most {rule['max_dropped']} may be"
    elif aliased < int(rule["min_aliased_left"]):
        reason = f"{aliased} aliased cases left, at least {rule['min_aliased_left']} are needed"
    elif placed < int(rule["min_placed_left"]):
        reason = f"{placed} placed cases left, at least {rule['min_placed_left']} is needed"
    return DropResult(tuple(dropped), not reason, reason, dict(laps_by_run))


# -- reported without a threshold ---------------------------------------------------------------------------


def case_rows(results: dict[str, dict], const: Constants) -> list[dict[str, Any]]:
    """Per case and condition: the laps by line source, the share of start-scan matches judged
    ambiguous (the lap node's value for its last lap), keyframe fixes per lap, scans skipped for
    stale odometry and the cost of `on_scan`. `results` maps run name to its JSON."""
    groups: dict[tuple[str, str], list[dict]] = {}
    for name, data in results.items():
        condition, case, _direction, _seed = name.split("_")
        groups.setdefault((case, condition), []).append(data)
    rows = []
    for (case, condition), docs in sorted(groups.items()):
        sources: Counter[str] = Counter()
        shares, fixes, stale, mean_ms, p99_ms = [], [], 0, [], []
        for d in docs:
            st = d.get("lap_status") or {}
            sources.update(st.get("lap_line_sources") or [])
            if st.get("start_alias_share_last_lap") is not None:
                shares.append(float(st["start_alias_share_last_lap"]))
            ring = st.get("ring") or {}
            if ring.get("fixes_last_lap") is not None:
                fixes.append(float(ring["fixes_last_lap"]))
            stale += int(st.get("stale_odometry_scans") or 0)
            if st.get("on_scan_ms_mean") is not None:
                mean_ms.append(float(st["on_scan_ms_mean"]))
            if st.get("on_scan_ms_p99") is not None:
                p99_ms.append(float(st["on_scan_ms_p99"]))
        rows.append(
            {
                "case": case,
                "class": const.class_of(case),
                "condition": condition,
                "runs": len(docs),
                "sources": dict(sorted(sources.items())),
                "alias_share_mean": statistics.fmean(shares) if shares else math.nan,
                "fixes_last_lap_mean": statistics.fmean(fixes) if fixes else math.nan,
                "stale_scans": stale,
                "on_scan_ms_mean": statistics.fmean(mean_ms) if mean_ms else math.nan,
                "on_scan_ms_p99_max": max(p99_ms) if p99_ms else math.nan,
            }
        )
    return rows


def case_table(rows: list[dict[str, Any]]) -> str:
    lines = [
        (
            "| case | class | cond | runs | laps by line source | alias share (last lap, mean) | "
            "fixes last lap (mean) | stale scans | on_scan mean ms | on_scan p99 max ms |"
        ),
        "|---|---|---|---|---|---|---|---|---|---|",
    ]
    for r in rows:
        lines.append(
            f"| {r['case']} | {r['class']} | {r['condition']} | {r['runs']} | {r['sources']} "
            f"| {r['alias_share_mean']:.3f} | {r['fixes_last_lap_mean']:.2f} | {r['stale_scans']} "
            f"| {r['on_scan_ms_mean']:.1f} | {r['on_scan_ms_p99_max']:.1f} |"
        )
    return "\n".join(lines)


def crossing12_rows(results: dict[str, dict], const: Constants) -> list[dict[str, Any]]:
    """Per run: system crossing 12 minus truth crossing 12, in seconds and as a share of the
    median truth lap (NaN where either does not exist)."""
    n = const.laps_judged
    rows = []
    for name, data in sorted(results.items()):
        truth = data.get("gt_lap_crossings_registered_s") or []
        laps = data.get("gt_lap_times_registered_s") or []
        system = data.get("system_lap_events") or []
        diff = math.nan
        share = math.nan
        if len(truth) >= n and len(system) >= n and laps:
            diff = float(system[n - 1]["t_cross_s"]) - float(truth[n - 1])
            share = diff / statistics.median(laps)
        rows.append({"run": name, "crossing12_offset_s": diff, "share_of_lap": share})
    return rows


def crossing12_table(rows: list[dict[str, Any]]) -> str:
    lines = ["| run | crossing 12, system minus truth s | share of a lap |", "|---|---|---|"]
    for r in rows:
        lines.append(f"| {r['run']} | {r['crossing12_offset_s']:+.2f} | {r['share_of_lap']:+.3f} |")
    return "\n".join(lines)


def condition_b_laps(runs: list[RunSummary], const: Constants) -> dict[str, Any]:
    """Condition b's judged laps against the same class tolerances (reported, no threshold)."""
    b = [r for r in runs if r.condition == "b"]
    st = _lap_stats(b, const)
    return {
        "runs": len(b),
        "laps": st["total"],
        "within": st["within"],
        "count_bad": len(st["count_bad"]),
    }


def missed_flags_per_condition(flags: dict[str, int]) -> list[str]:
    """`flags` maps run name to its number of laps flagged missed gate."""
    return b7.missed_flags_per_condition(flags)


def lap1_rows(results: dict[str, dict]) -> list[dict[str, Any]]:
    return b7.lap1_rows(results)


def lap1_table(rows: list[dict[str, Any]]) -> str:
    return b7.lap1_table(rows)
