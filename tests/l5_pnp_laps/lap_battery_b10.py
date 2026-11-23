"""Battery B10 of the second camera lap source pre-registration
(docs/notes/preregistration-pnp-v0.5-laps-camera-2.md sections 2 and 3, brief 51c): run loading and
the decisions D10-S0, D10-S1 and D10-S2 as pure functions.

Every threshold is read from `evaluation/prereg/pnp-v0.5-laps-camera-2.yaml` (block
`lap_battery_b10`), whose SHA-256 is a constant of this file and is checked on every load. The
yaml also names the first attempt's yaml (hash checked) and repeats its `source_rule` block, which
must be equal. D10-S1 calls D3-S1 (`lap_battery_b4.d3_s1`) with the yaml's counts (11 of 12, 7 of
8 after one dropped case); D3-S1's contacts threshold (0) comes from `pnp-v0.3.yaml` through
`lap_battery_b4.load_constants`, which checks that file's hash too. A threshold that cannot be
applied as written is reported, never edited.

What B10 is: B9's design (the lap memory takes its laps from a truth oracle, conditions a, b, c, f)
plus the start gate of brief 51c. Every c and f run must have started through the gate (the planner
spawned only after `StartReadyGate` said start, the first DRIVING report after that, the lap node
ready at its start), and condition r (the oracle never ready) must be refused by the starter with
the planner never started. 13 ground-truth laps are driven, laps 1 to 12 are judged. The registered
truth is computed afterwards from the recording and reads neither the oracle nor the system.
"""

from __future__ import annotations

import dataclasses
import hashlib
import itertools
import json
import math
import statistics
from dataclasses import dataclass
from pathlib import Path
from typing import Any

import lap_battery_b4 as b4
import yaml
from lap_battery import Decision, RunSummary

REPO_ROOT = Path(__file__).resolve().parents[2]
PREREG_YAML = REPO_ROOT / "evaluation" / "prereg" / "pnp-v0.5-laps-camera-2.yaml"
D9_YAML = REPO_ROOT / "evaluation" / "prereg" / "pnp-v0.5-laps-camera.yaml"
#: SHA-256 of `pnp-v0.5-laps-camera-2.yaml` as committed with the pre-registration (commit 1 of brief
#: 51c). Checked on every load and written into every run and the report.
REGISTERED_YAML_SHA256 = "823f641c8a40337e1a81aaa440d55976585d19984ae2db55f06f0eee1289bdcd"
D9_YAML_SHA256 = "013cb2f8bec3ce96d2abf456361c3de93d8f85da48c01900c374287d7473807f"
B10_CONDITIONS = ("a", "b", "c", "f", "r")
#: The seeds and the cases of the attempts before this one that are spent: never run again.
SPENT_SEEDS = (71, 72)
SPENT_CASES = ("U1", "U2", "U3")
B10_SEEDS = (81, 82)
B10_DIRECTIONS = ("ccw", "cw")
LAP_JUDGED = 12

PreregError = b4.PreregError
RunFileError = b4.RunFileError
MissingPairError = b4.MissingPairError

#: Tolerance of "at most" and "within" against the float noise of a difference of two times.
EPS = 1e-9


def sha256_file(path: Path) -> str:
    return hashlib.sha256(path.read_bytes()).hexdigest()


@dataclass(frozen=True)
class Constants:
    """The `lap_battery_b10` block of the yaml, the SHA-256 it was read under, the whole document
    and the one number D10-S1 takes from D3 (`max_contacts`). Has the interface of
    `lap_battery_b4.Constants` that the D3 functions use."""

    yaml_sha256: str
    raw: dict[str, Any]
    doc: dict[str, Any]
    max_contacts: int

    @property
    def battery_cases(self) -> dict[str, dict]:
        return dict(self.raw["cases"])

    @property
    def development_seeds(self) -> tuple[int, ...]:
        return tuple(int(s) for s in self.raw["development"]["seeds"])

    @property
    def development_case_names(self) -> tuple[str, ...]:
        return tuple(self.raw["development"]["cases_read_before"])

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
    def thresholds(self) -> dict[str, Any]:
        return dict(self.raw["thresholds"])

    @property
    def fault_plan(self) -> dict[str, Any]:
        return dict(self.raw["fault_plan"])

    @property
    def oracle_ready_after_s(self) -> dict[int, float]:
        """Sim seconds after the lap node's first status that the oracle says ready, by seed."""
        return {int(k): float(v) for k, v in self.raw["oracle_ready_after_s"].items()}

    @property
    def spent(self) -> dict[str, Any]:
        return dict(self.raw["spent_never_run_again"])

    @property
    def th(self) -> dict[str, float]:
        """The thresholds under the names the D3 functions read."""
        t = self.raw["thresholds"]
        return {
            "max_contacts": float(self.max_contacts),
            "s1_reach_max_ratio_to_b": float(t["s1_reach_ratio"]),
            "s1_max_median_r": float(t["s1_median_r_max"]),
        }

    def required_complete(self, n_runs: int) -> int:
        """11 of 12, 7 of 8 after one dropped case. Nothing else is registered."""
        table = self.raw["thresholds"]["s1_min_complete"]
        for key, n in (("of_12", 12), ("of_8", 8)):
            if n_runs == n:
                return int(table[key])
        raise PreregError(f"{n_runs} runs in a condition: the pre-registration has 12 or 8")


def _check_consistent(doc: dict[str, Any]) -> None:
    block = doc["lap_battery_b10"]
    t = block["thresholds"]
    if doc.get("v05_laps_camera_sha256") != D9_YAML_SHA256:
        raise PreregError(f"v05_laps_camera_sha256 is not {D9_YAML_SHA256}")
    if sha256_file(D9_YAML) != D9_YAML_SHA256:
        raise PreregError(f"{D9_YAML} is not the registered first attempt's yaml")
    d9 = yaml.safe_load(D9_YAML.read_text())
    if doc["source_rule"] != d9["source_rule"]:
        raise PreregError("source_rule differs from the first attempt's: it is unchanged by 51c")
    if tuple(int(s) for s in block["spent_never_run_again"]["seeds"]) != SPENT_SEEDS:
        raise PreregError(f"spent seeds are not {list(SPENT_SEEDS)}")
    if tuple(block["spent_never_run_again"]["cases"]) != SPENT_CASES:
        raise PreregError(f"spent cases are not {list(SPENT_CASES)}")
    if {int(k) for k in block["oracle_ready_after_s"]} != set(B10_SEEDS):
        raise PreregError("oracle_ready_after_s does not cover exactly the B10 seeds")
    if t["s1_reach_min_runs"] != t["s1_min_complete"]:
        raise PreregError("s1_reach_min_runs differs from s1_min_complete: D3-S1 uses one number")
    if int(t["s1_complete_laps"]) != b4.LAPS_TO_COMPLETE:
        raise PreregError(f"s1_complete_laps is not {b4.LAPS_TO_COMPLETE}")
    if tuple(int(k) for k in t["late_laps"]) != b4.LATE_LAPS:
        raise PreregError(f"late_laps is not {list(b4.LATE_LAPS)}")
    if tuple(int(s) for s in block["seeds"]) != B10_SEEDS:
        raise PreregError(f"seeds are not {list(B10_SEEDS)}")
    if tuple(block["directions"]) != B10_DIRECTIONS:
        raise PreregError(f"directions are not {list(B10_DIRECTIONS)}")
    if int(block["laps_judged"]) != LAP_JUDGED:
        raise PreregError(f"laps_judged is not {LAP_JUDGED}")


def load_constants(
    yaml_path: Path = PREREG_YAML, expected_sha: str = REGISTERED_YAML_SHA256
) -> Constants:
    """The yaml's `lap_battery_b10` block, refused when its SHA-256 is not the registered one."""
    actual = sha256_file(yaml_path)
    if actual != expected_sha:
        raise PreregError(
            f"{yaml_path} has SHA-256 {actual}, the registered value is {expected_sha}: refusing"
        )
    doc = yaml.safe_load(yaml_path.read_text())
    _check_consistent(doc)
    d3 = b4.load_constants()  # checks the v0.3 file's hash against its pre-registration
    return Constants(actual, doc["lap_battery_b10"], doc, int(d3.th["max_contacts"]))


# -- runs -------------------------------------------------------------------------------------------------


def _times(seq: Any) -> tuple[float, ...]:
    return tuple(float(x) for x in seq)


def _summarise_refusal(result: dict, case: str, direction: str, seed: int) -> RunSummary:
    """A condition r run: no laps are expected, so nothing of the lap truth is read."""
    return RunSummary(
        condition="r",
        case=case,
        direction=direction,
        seed=seed,
        gt_lap_times_s=(),
        system_lap_times_s=(),
        contacts=int(result.get("wall_contacts", 0)),
        brakes_per_lap=(),
        stopped_because=str(result.get("stopped_because", "")),
        error=str(result.get("error", "")),
        extras={},
    )


def summarise_run(
    result: dict, condition: str, case: str, direction: str, seed: int, const: Constants
) -> RunSummary:
    """A RunSummary on the registered truth, with the truth crossings, the system's laps, the
    oracle's events, the lap node's status and the guidance cap series in `extras`. A result
    without the registered truth is refused. A run whose truth recording ends before the end of
    lap 12 is a run that failed to execute: its `error` says so."""
    name = f"{condition}_{case}_{direction}_{seed}"
    if condition != "r" and (
        "gt_lap_times_registered_s" not in result or "truth_valid" not in result
    ):
        raise RunFileError(
            f"{name}: no gt_lap_times_registered_s and truth_valid "
            "(the runner of brief 48 is required)"
        )
    if condition == "r":
        run = _summarise_refusal(result, case, direction, seed)
    else:
        run = b4.summarise_b4_run(result, condition, case, direction, seed)
        if result.get("stopped_because") == "start_refused":
            # a c or f run the starter refused is a run that failed to execute, never repeated
            reason = (result.get("start_refusal") or {}).get("reason", "")
            run = dataclasses.replace(
                run, error=f"{run.error}; start refused: {reason}".lstrip("; ")
            )
    system = result.get("system_lap_events") or []
    status = result.get("lap_status") or {}
    truth_crossings = _times(result.get("gt_lap_crossings_registered_s", []))
    extras = {
        "truth_crossings_s": truth_crossings,
        "truth_crossing0_s": result.get("gt_lap1_start_s"),
        "system_laps": [dict(e) for e in system],
        "oracle_events": [dict(e) for e in result.get("oracle_events") or []],
        "lap_status": dict(status),
        "guidance_cap_series": [
            (float(t), float(c)) for t, c in result.get("guidance_cap_series", [])
        ],
        "truth_last_odom_s": result.get("truth_last_odom_s"),
        "lap_truth_faults": result.get("lap_truth_faults"),
        "start_trace": dict(result.get("start_trace") or {}),
        "start_refusal": dict(result.get("start_refusal") or {}),
        "source_ready": result.get("source_ready"),
        "drive_raw_messages": result.get("drive_raw_messages"),
        "distance_m": result.get("distance_m"),
        "stopped_because": str(result.get("stopped_because", "")),
    }
    run.extras.update(extras)
    n = const.laps_judged
    last = result.get("truth_last_odom_s")
    if (
        condition != "r"
        and len(truth_crossings) >= n
        and not run.error
        and (last is None or float(last) < truth_crossings[n - 1])
    ):
        run = dataclasses.replace(
            run,
            error=f"recording ends before truth crossing {n} ({last} < {truth_crossings[n - 1]:.2f})",
        )
        run.extras.update(extras)
    return run


def load_runs(results_dir: Path, const: Constants, mode: str = "battery") -> list[RunSummary]:
    """Every `<condition>_<case>_<direction>_<seed>.json` of a B10 directory (condition a, b, c, f, r).
    Each must carry the `battery_b10` block with the mode asked for."""
    out = []
    for path in sorted(results_dir.glob("*.json")):
        parts = path.stem.split("_")
        if len(parts) != 4 or parts[0] not in B10_CONDITIONS:
            continue
        condition, case, direction, seed = parts
        data = json.loads(path.read_text())
        meta = data.get("battery_b10")
        if not isinstance(meta, dict) or meta.get("mode") != mode:
            raise RunFileError(f"{path.name}: not a B10 {mode} run file")
        out.append(summarise_run(data, condition, case, direction, int(seed), const))
    return out


# -- helpers ------------------------------------------------------------------------------------------------


def _label(r: RunSummary) -> str:
    return b4._label(r)


def run_name(r: RunSummary) -> str:
    """`<condition>_<case>_<direction>_<seed>`, the name of the run's file."""
    return f"{r.condition}_{r.case}_{r.direction}_{r.seed}"


def within(a: float, b: float, tol: float) -> bool:
    """|a - b| <= tol, False for NaN or inf."""
    d = abs(a - b)
    return math.isfinite(d) and d <= tol + EPS


def _first_n(seq: Any, n: int) -> list[Any]:
    return list(seq)[:n]


def truth_event_times(run: RunSummary) -> dict[int, list[float]]:
    """Published `true` oracle events by the truth crossing they stand for: their crossing times."""
    out: dict[int, list[float]] = {}
    for e in run.extras.get("oracle_events", []):
        if e.get("kind") == "true" and e.get("truth_crossing") is not None:
            out.setdefault(int(e["truth_crossing"]), []).append(float(e["crossing_time_s"]))
    return out


def doubtful_laps(run: RunSummary) -> dict[int, list[str]]:
    """{gate lap index: reasons} of the lap node's last status."""
    return {
        int(d["lap"]): [str(x) for x in d.get("reasons", [])]
        for d in (run.extras.get("lap_status") or {}).get("doubtful_laps", [])
    }


def rejected_total(run: RunSummary) -> int:
    rejected = (run.extras.get("lap_status") or {}).get("rejected_events") or {}
    return int(sum(int(v) for v in rejected.values()))


# -- D10-S0 -----------------------------------------------------------------------------------------------------


def _s0_run(run: RunSummary, const: Constants) -> dict[str, list[str]]:
    """The failures of one condition c run, by item."""
    th = const.thresholds
    tol = float(th["s0_tolerance_s"])
    n = const.laps_judged
    truth = run.extras.get("truth_crossings_s", ())
    system = run.extras.get("system_laps", [])
    bad: dict[str, list[str]] = {"oracle": [], "system": [], "crossings": [], "execute": []}
    if run.error:
        bad["execute"].append(f"{_label(run)}: {run.error[:80]}")
    if len(truth) < n:
        bad["oracle"].append(f"{_label(run)}: {len(truth)} truth crossings, {n} needed")
        return bad
    events = truth_event_times(run)
    others = [e for e in run.extras.get("oracle_events", []) if e.get("kind") != "true"]
    if others:
        bad["oracle"].append(f"{_label(run)}: {len(others)} events that are not truth crossings")
    for k in range(1, n + 1):
        got = events.get(k, [])
        if len(got) != 1:
            bad["oracle"].append(
                f"{_label(run)} crossing {k}: {len(got)} oracle events, one needed"
            )
        elif not within(got[0], truth[k - 1], tol):
            bad["oracle"].append(
                f"{_label(run)} crossing {k}: oracle {got[0]:.2f} s, truth {truth[k - 1]:.2f} s"
            )
    if len(system) < n:
        bad["system"].append(f"{_label(run)}: {len(system)} system laps, {n} needed")
    else:
        late = {k: r for k, r in doubtful_laps(run).items() if k <= n}
        if late:
            bad["system"].append(f"{_label(run)}: doubtful laps {late}")
        if rejected_total(run) > int(th["s0_rejected_allowed"]):
            bad["system"].append(
                f"{_label(run)}: {rejected_total(run)} events rejected "
                f"{(run.extras.get('lap_status') or {}).get('rejected_events')}"
            )
        if int(th["s0_doubtful_allowed"]) != 0:
            raise PreregError("s0_doubtful_allowed is not 0")
        for k in range(1, n + 1):
            t = float(system[k - 1]["t_cross_s"])
            if not within(t, truth[k - 1], tol):
                bad["crossings"].append(
                    f"{_label(run)} lap {k}: system {t:.2f} s, truth {truth[k - 1]:.2f} s"
                )
    return bad


def started_through_gate(run: RunSummary) -> list[str]:
    """What is wrong with the start of a c or f run: the gate said start, the planner was spawned
    after it, the first DRIVING report is later than that, and the lap node started ready. Empty
    when nothing is."""
    tr = run.extras.get("start_trace") or {}
    out: list[str] = []
    if tr.get("gate_verdict") != "start":
        out.append(f"the gate's verdict is {tr.get('gate_verdict')!r}, not 'start'")
    gate, spawn, driving = (
        tr.get("gate_start_s"),
        tr.get("planner_spawn_s"),
        tr.get("driving_first_s"),
    )
    if not _finite(gate):
        out.append("no gate start time")
    else:
        if not _finite(spawn) or spawn < gate - EPS:
            out.append(f"planner spawned at {spawn}, before the gate's start {gate}")
        if not _finite(driving) or driving <= gate:
            out.append(f"first DRIVING report {driving} is not later than the gate's start {gate}")
    if run.extras.get("source_ready") is not True:
        reason = (run.extras.get("lap_status") or {}).get("source_not_ready_reason")
        out.append(f"the lap node did not start ready ({reason})")
    return out


def _finite(x: Any) -> bool:
    return isinstance(x, (int, float)) and not isinstance(x, bool) and math.isfinite(x)


def refusal_failures(run: RunSummary, const: Constants) -> list[str]:
    """What is wrong with a condition r run (S0 item 6): the starter refused within the registered
    time of the lap node's first status, the planner was never started, no message on /drive_raw,
    the ground-truth path within the limit, and the refusal names the source's reason."""
    th = const.thresholds["s0_refusal"]
    tr = run.extras.get("start_trace") or {}
    refusal = run.extras.get("start_refusal") or {}
    out: list[str] = []
    if run.extras.get("stopped_because") != "start_refused":
        out.append(f"stopped because {run.extras.get('stopped_because')!r}, not 'start_refused'")
    first = tr.get("lap_node_first_status_s")
    refused = refusal.get("refused_at_s")
    if not (
        _finite(first)
        and _finite(refused)
        and refused - first <= float(th["within_s_of_lap_node_first_status"]) + EPS
    ):
        out.append(f"refused at {refused} against the lap node's first status {first}")
    if th["planner_started"] is not False:
        raise PreregError("s0_refusal.planner_started is not false")
    if tr.get("planner_spawn_s") is not None:
        out.append(f"the planner was spawned at {tr.get('planner_spawn_s')}")
    n_msgs = run.extras.get("drive_raw_messages")
    if n_msgs != int(th["drive_raw_messages"]):
        out.append(f"{n_msgs} messages on /drive_raw")
    path = run.extras.get("distance_m")
    if not (_finite(path) and path <= float(th["max_truth_path_m"]) + EPS):
        out.append(f"ground-truth path {path} m")
    reason = refusal.get("reason")
    if not (isinstance(reason, str) and reason):
        out.append("the refusal names no reason")
    elif bool(th["reason_named"]) and tr.get("gate_reason") != reason:
        out.append("the refusal's reason is not the gate's")
    if run.error:
        out.append(f"error: {run.error[:80]}")
    return out


def d10_s0(runs: list[RunSummary], const: Constants | None = None) -> Decision:
    """The start, the oracle and the wiring, condition c every run, and the refusal, condition r
    every run (pre-registration section 3)."""
    const = const or load_constants()
    b4._check_unique(runs)
    c = b4._need(runs, "c", "D10-S0")
    rr = [r for r in runs if r.condition == "r"]
    bad: dict[str, list[str]] = {
        "gate": [],
        "oracle": [],
        "system": [],
        "crossings": [],
        "execute": [],
        "refusal": [],
    }
    for r in c:
        for key, lines in _s0_run(r, const).items():
            bad[key].extend(lines)
        bad["gate"].extend(f"{_label(r)}: {x}" for x in started_through_gate(r))
    for r in rr:
        bad["refusal"].extend(f"{_label(r)}: {x}" for x in refusal_failures(r, const))
    n = const.laps_judged
    items: dict[str, tuple[bool | None, str]] = {
        "(1) the planner was started only after the start gate said start, the first DRIVING "
        "report is later than that, and the lap node started ready": (
            not bad["gate"],
            f"{len(bad['gate'])} exceptions in {len(c)} runs {bad['gate'][:6]}",
        ),
        f"the oracle publishes one event per registered truth crossing 1 to {n}, each within "
        f"{const.thresholds['s0_tolerance_s']} s of it": (
            not bad["oracle"],
            f"{len(bad['oracle'])} exceptions in {len(c)} runs {bad['oracle'][:6]}",
        ),
        f"the system closes laps 1 to {n}, all confirmed, none doubtful, no event rejected": (
            not bad["system"],
            f"{len(bad['system'])} exceptions {bad['system'][:6]}",
        ),
        f"every system crossing within {const.thresholds['s0_tolerance_s']} s of the truth "
        "crossing of the same number": (
            not bad["crossings"],
            f"{len(bad['crossings'])} exceptions {bad['crossings'][:6]}",
        ),
        "no run failed to execute": (
            not bad["execute"],
            f"{len(bad['execute'])} errors {bad['execute'][:4]}",
        ),
        "(6) condition r: the starter refused within "
        f"{const.thresholds['s0_refusal']['within_s_of_lap_node_first_status']} s, the planner was "
        "never started, no /drive_raw message, the ground-truth path at most "
        f"{const.thresholds['s0_refusal']['max_truth_path_m']} m, and the refusal names the "
        "source's reason": (
            bool(rr) and not bad["refusal"],
            f"{len(bad['refusal'])} exceptions in {len(rr)} runs {bad['refusal'][:6]}"
            if rr
            else "no condition r runs",
        ),
    }
    return b4._decision("D10-S0", items)


# -- D10-S1 -----------------------------------------------------------------------------------------------------


def d10_s1(runs: list[RunSummary], const: Constants | None = None) -> Decision:
    """D3-S1's four items on condition c against its paired control b, with the counts of the
    yaml (11 of 12, 7 of 8 after one dropped case)."""
    const = const or load_constants()
    laps_runs = [r for r in runs if r.condition != "r"]
    return dataclasses.replace(b4.d3_s1(laps_runs, const), name="D10-S1")  # type: ignore[arg-type]


# -- D10-S2 -----------------------------------------------------------------------------------------------------


def _match_crossing(t: float, truth: tuple[float, ...], tol: float) -> int | None:
    """The 1-based index of the truth crossing within `tol` of `t` (the nearest), or None."""
    best: tuple[float, int] | None = None
    for k, c in enumerate(truth, start=1):
        d = abs(t - c)
        if math.isfinite(d) and d <= tol + EPS and (best is None or d < best[0]):
            best = (d, k)
    return None if best is None else best[1]


def cap_series_min(series: list[tuple[float, float]], t0: float, t1: float) -> float:
    """The lowest guidance cap in [t0, t1] of a series of change points: the cap standing at t0
    and every change inside the window. NaN when no cap was published by t1 or the series is
    empty."""
    if not series:
        return math.nan
    standing = [c for t, c in series if t <= t0]
    inside = [c for t, c in series if t0 < t <= t1]
    values = ([standing[-1]] if standing else []) + inside
    return min(values) if values else math.nan


def system_laps_by_truth(run: RunSummary, const: Constants) -> list[dict[str, Any]]:
    """One row per system lap (gate lap index 1, 2, ...): its closing crossing, the truth crossing
    it matches within the S0 tolerance (None for a false event), its previous crossing and the
    truth crossing that one matches, whether it is doubtful and why, the cap it was driven at."""
    tol = float(const.thresholds["s2_confirmed_lap_tolerance_s"])
    truth = run.extras.get("truth_crossings_s", ())
    start = run.extras.get("lap_status", {}).get("lap1_start_stamp_s")
    c0 = run.extras.get("truth_crossing0_s")
    doubtful = doubtful_laps(run)
    rows = []
    previous_t: float | None = None
    previous_k: int | None = None
    for j, e in enumerate(run.extras.get("system_laps", []), start=1):
        t = float(e["t_cross_s"])
        if j == 1:
            previous_t = float(start) if start is not None else (None if c0 is None else float(c0))
            previous_k = (
                0
                if previous_t is not None and c0 is not None and within(previous_t, float(c0), tol)
                else None
            )
        rows.append(
            {
                "lap": j,
                "t_s": t,
                "truth_k": _match_crossing(t, truth, tol),
                "previous_t_s": previous_t,
                "previous_truth_k": previous_k,
                "doubtful": j in doubtful,
                "reasons": doubtful.get(j, []),
                "cap_mps": float(e["cap_mps"]),
            }
        )
        previous_t, previous_k = t, rows[-1]["truth_k"]
    return rows


def _s2_run(run: RunSummary, const: Constants) -> dict[str, list[str]]:
    th = const.thresholds
    float(th["s2_confirmed_lap_tolerance_s"])
    n = const.laps_judged
    truth = run.extras.get("truth_crossings_s", ())
    plan = const.fault_plan
    rows = system_laps_by_truth(run, const)
    bad: dict[str, list[str]] = {
        k: [] for k in ("contacts", "laps", "confirmed", "doubtful", "cap", "overdue", "dup")
    }
    label = _label(run)
    if run.contacts > int(th["s2_contacts_allowed"]):
        bad["contacts"].append(f"{label}: {run.contacts} contacts")
    if run.error or run.laps < n:
        bad["laps"].append(
            f"{label}: {run.laps} truth laps{', ' + run.error[:60] if run.error else ''}"
        )
    # (3) every confirmed system lap is one truth lap: two consecutive truth crossings
    for r in rows:
        if r["doubtful"]:
            continue
        k, kp = r["truth_k"], r["previous_truth_k"]
        if k is None or kp is None or k != kp + 1:
            bad["confirmed"].append(f"{label} lap {r['lap']}: crossings match truth {kp} to {k}")
    # (4) the laps that close at truth crossings 5 and 8, the false event's lap and the lap after it
    expected = [int(k) for k in th["s2_expected_doubtful_at_truth_crossings"]]
    for k in expected:
        hit = [r for r in rows if r["truth_k"] == k]
        if not hit or not all(r["doubtful"] for r in hit):
            bad["doubtful"].append(
                f"{label}: the lap closing at truth crossing {k} is not doubtful"
            )
    false_events = [e for e in run.extras.get("oracle_events", []) if e.get("kind") == "false"]
    if th["s2_expected_doubtful_false_event_and_next"]:
        if plan.get("false_event") and len(false_events) != 1:
            bad["doubtful"].append(
                f"{label}: {len(false_events)} false events published, one planned"
            )
        for fe in false_events:
            ft = float(fe["crossing_time_s"])
            idx = [r for r in rows if within(r["t_s"], ft, 1e-3)]
            if not idx:
                bad["doubtful"].append(f"{label}: the false event closed no lap")
                continue
            j = idx[0]["lap"]
            for lap in (j, j + 1):
                row = next((r for r in rows if r["lap"] == lap), None)
                if row is None or not row["doubtful"]:
                    bad["doubtful"].append(
                        f"{label}: the lap {'closed by the false event' if lap == j else 'after it'} "
                        f"(lap {lap}) is not doubtful"
                    )
    if th["s2_duplicate_rejected"] and plan.get("duplicate_truth_crossing"):
        rej = (run.extras.get("lap_status") or {}).get("rejected_events") or {}
        n_dup = int(rej.get("not_monotone", 0)) + int(rej.get("duplicate", 0))
        if n_dup < 1:
            bad["dup"].append(f"{label}: the duplicate of crossing 10 was not rejected ({rej})")
    # (5) the cap of the lap after a doubtful lap is at most the cap of that doubtful lap
    for a, b in itertools.pairwise(rows):
        if a["doubtful"] and b["cap_mps"] > a["cap_mps"] + EPS:
            bad["cap"].append(
                f"{label}: lap {b['lap']} driven at {b['cap_mps']:.2f} after doubtful lap "
                f"{a['lap']} at {a['cap_mps']:.2f}"
            )
    # (6) overdue windows: truth crossing d plus half a truth lap to the next truth crossing
    series = run.extras.get("guidance_cap_series", [])
    lap1 = const.lap1_cap_mps
    for d in (int(k) for k in plan.get("drop_truth_crossings", ())):
        if len(truth) < d + 1:
            bad["overdue"].append(f"{label}: no truth crossing {d + 1}")
            continue
        t0 = float(truth[d - 1]) + 0.5 * (float(truth[d]) - float(truth[d - 1]))
        low = cap_series_min(series, t0, float(truth[d]))
        if not (math.isfinite(low) and low <= lap1 + 1e-6):
            bad["overdue"].append(
                f"{label}: lowest guidance cap between crossing {d} plus half a lap and crossing "
                f"{d + 1} is {low:.3f}, the lap-1 cap is {lap1}"
            )
    return bad


def d10_s2(runs: list[RunSummary], const: Constants | None = None) -> Decision:
    """The fallback in closed loop, condition f, every run (pre-registration section 3)."""
    const = const or load_constants()
    b4._check_unique(runs)
    f = b4._need(runs, "f", "D10-S2")
    bad: dict[str, list[str]] = {}
    for r in f:
        for key, lines in _s2_run(r, const).items():
            bad.setdefault(key, []).extend(lines)
        bad.setdefault("gate", []).extend(f"{_label(r)}: {x}" for x in started_through_gate(r))
    th = const.thresholds
    n = const.laps_judged

    def item(key: str) -> tuple[bool, str]:
        lines = bad.get(key, [])
        return not lines, f"{len(lines)} exceptions in {len(f)} runs {lines[:6]}"

    items: dict[str, tuple[bool | None, str]] = {
        f"(1) {th['s2_contacts_allowed']} wall contacts": item("contacts"),
        f"(2) {n} truth laps completed": item("laps"),
        "(3) every confirmed system lap is one truth lap (two consecutive truth crossings within "
        f"{th['s2_confirmed_lap_tolerance_s']} s)": item("confirmed"),
        "(4a) the laps that close at truth crossings 5 and 8, the lap closed by the false event and "
        "the lap after it are doubtful": item("doubtful"),
        "(4b) the duplicate of crossing 10 is rejected": item("dup"),
        "(5) the cap of the lap after a doubtful lap is at most that of the doubtful lap": item(
            "cap"
        ),
        "(6) the lowest guidance cap while overdue is the lap-1 cap": item("overdue"),
        "(7) the run started ready through the start gate (D10-S0 item 1)": item("gate"),
    }
    return b4._decision("D10-S2", items)


# -- the drop rule ------------------------------------------------------------------------------------------------


@dataclass(frozen=True)
class DropResult:
    dropped: tuple[str, ...]
    decided: bool
    reason: str
    laps: dict[str, int]

    @property
    def runs_per_condition(self) -> int:
        return 0 if not self.decided else 4 * (3 - len(self.dropped))


def drop_rule(laps_by_run: dict[str, int], const: Constants) -> DropResult:
    """The registered drop rule. `laps_by_run` maps `<case>_<direction>` (condition a, seed 71) to
    the ground-truth laps of its accepted run (0 for a run that failed to execute or has an invalid
    truth). A case is dropped if either direction has fewer than `min_laps`. Nothing is decided if
    more than `max_dropped` cases would go or fewer than `min_cases_left` are left."""
    rule = const.drop_rule
    need = int(rule["min_laps"])
    cases = list(const.battery_cases)
    dropped: list[str] = []
    for case in cases:
        counts = [laps_by_run.get(f"{case}_{d}") for d in const.directions]
        if any(n is None or n < need for n in counts):
            dropped.append(case)
    left = [c for c in cases if c not in dropped]
    reason = ""
    if len(dropped) > int(rule["max_dropped"]):
        reason = f"{len(dropped)} cases would be dropped, at most {rule['max_dropped']} may be"
    elif len(left) < int(rule["min_cases_left"]):
        reason = f"{len(left)} cases left, at least {rule['min_cases_left']} are needed"
    return DropResult(tuple(dropped), not reason, reason, dict(laps_by_run))


# -- reported without a threshold -----------------------------------------------------------------------------------


def late_lap_rows(runs: list[RunSummary]) -> list[dict[str, Any]]:
    """Per paired f and c run: T_late of condition f against c (reported, no threshold)."""
    by_key = {(r.condition, r.key): r for r in runs}
    rows = []
    for r in runs:
        if r.condition != "f":
            continue
        c = by_key.get(("c", r.key))
        if c is None:
            continue
        ratio = r.t_late_s / c.t_late_s if c.t_late_s and math.isfinite(c.t_late_s) else math.nan
        rows.append(
            {"run": _label(r), "t_late_f_s": r.t_late_s, "t_late_c_s": c.t_late_s, "ratio": ratio}
        )
    return rows


def run_detail_rows(runs: list[RunSummary]) -> list[dict[str, Any]]:
    """Per condition c and f run: the largest event latency, the reference length at the end and
    every doubtful lap with its reasons (reported, no threshold)."""
    rows = []
    for r in runs:
        if r.condition not in ("c", "f"):
            continue
        st = r.extras.get("lap_status") or {}
        rows.append(
            {
                "run": run_name(r),
                "event_latency_s_max": st.get("event_latency_s_max"),
                "l_ref_m": st.get("l_ref_m"),
                "confirmed_laps": st.get("confirmed_laps"),
                "doubtful": doubtful_laps(r),
                "rejected": st.get("rejected_events"),
            }
        )
    return rows


def median_t_late_ratio(runs: list[RunSummary]) -> float:
    rows = [x["ratio"] for x in late_lap_rows(runs) if math.isfinite(x["ratio"])]
    return statistics.median(rows) if rows else math.nan


def start_trace_rows(runs: list[RunSummary]) -> list[dict[str, Any]]:
    """One row per run of every condition with a lap node: the start trace and the source's health
    at the start (reported, no threshold), so a start failure is explained by the report itself."""
    rows = []
    for r in runs:
        if r.condition not in ("c", "f", "r"):
            continue
        tr = r.extras.get("start_trace") or {}
        st = r.extras.get("lap_status") or {}
        rows.append(
            {
                "run": run_name(r),
                **{k: tr.get(k) for k in START_TRACE_KEYS},
                "source_ready": r.extras.get("source_ready"),
                "source_status_age_at_start_s": st.get("source_status_age_at_start_s"),
                "source_statuses_before_start": st.get("source_statuses_before_start"),
                "source_status_gap_max_s": st.get("source_status_gap_max_s"),
                "stopped_because": r.extras.get("stopped_because"),
                "refusal_reason": (r.extras.get("start_refusal") or {}).get("reason"),
            }
        )
    return rows


START_TRACE_KEYS = (
    "lap_node_first_status_s",
    "ready_first_s",
    "gate_start_s",
    "planner_spawn_s",
    "driving_first_s",
    "lap1_start_s",
    "oracle_first_status_s",
    "gate_verdict",
    "gate_reason",
)


def ready_after_s(const: Constants, condition: str, seed: int) -> float:
    """Seconds of sim time after the lap node's first status until the oracle says ready: by seed
    for c and f, never (inf) for r, 0 for the conditions with no oracle."""
    if condition in ("c", "f"):
        return const.oracle_ready_after_s[seed]
    if condition == "r":
        return math.inf
    return 0.0


def g4_source_health(runs: list[RunSummary]) -> dict[str, Any]:
    """Development check G4 of brief 51c on the c and f runs: the largest `source_status_gap_max_s`
    and `source_status_age_at_start_s` (the brief's limits are 2.0 s and 1.5 s)."""
    gaps, ages = [], []
    for r in runs:
        if r.condition not in ("c", "f"):
            continue
        st = r.extras.get("lap_status") or {}
        gap, age = st.get("source_status_gap_max_s"), st.get("source_status_age_at_start_s")
        gaps.append(float("nan") if gap is None else float(gap))
        ages.append(math.inf if age is None else float(age))
    return {
        "runs": len(gaps),
        "gap_max_s": max(gaps) if gaps else math.nan,
        "age_at_start_max_s": max(ages) if ages else math.nan,
        "gap_ok": bool(gaps) and all(math.isfinite(g) and g <= 2.0 + EPS for g in gaps),
        "age_ok": bool(ages) and all(math.isfinite(a) and a <= 1.5 + EPS for a in ages),
    }


def start_summary(results: list[dict]) -> dict[str, Any]:
    """D2 and G5 of brief 51c over start-only run results (raw runner JSON, any condition with a
    lap node): how many started not ready, the distribution of the oracle's first status minus the
    lap node's lap 1 start (positive: the status came after the start, the B9 hole), how many
    started through the gate and had their first DRIVING report after the gate's start."""
    rows = []
    for d in results:
        tr = d.get("start_trace") or {}
        lap1, oracle = tr.get("lap1_start_s"), tr.get("oracle_first_status_s")
        gate, driving = tr.get("gate_start_s"), tr.get("driving_first_s")
        ready = d.get("source_ready")
        rows.append(
            {
                "source_ready": ready,
                "oracle_minus_lap1_s": oracle - lap1 if _finite(lap1) and _finite(oracle) else None,
                "through_gate": tr.get("gate_verdict") == "start",
                "driving_after_gate": _finite(gate) and _finite(driving) and driving > gate,
                "refused": d.get("stopped_because") == "start_refused",
                "error": bool(d.get("error")),
            }
        )
    deltas = sorted(r["oracle_minus_lap1_s"] for r in rows if r["oracle_minus_lap1_s"] is not None)

    def q(f: float) -> float | None:
        return None if not deltas else deltas[min(len(deltas) - 1, int(f * len(deltas)))]

    return {
        "runs": len(rows),
        "started_not_ready": sum(1 for r in rows if r["source_ready"] is False),
        "no_ready_value": sum(1 for r in rows if r["source_ready"] is None),
        "oracle_after_lap1_start": sum(1 for d in deltas if d > 0.0),
        "oracle_minus_lap1_s": {
            "n": len(deltas),
            "min": deltas[0] if deltas else None,
            "median": q(0.5),
            "max": deltas[-1] if deltas else None,
        },
        "through_gate": sum(1 for r in rows if r["through_gate"]),
        "driving_after_gate": sum(1 for r in rows if r["driving_after_gate"]),
        "refused": sum(1 for r in rows if r["refused"]),
        "errors": sum(1 for r in rows if r["error"]),
    }
