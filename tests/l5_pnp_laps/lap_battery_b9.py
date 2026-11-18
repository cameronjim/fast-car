"""Battery B9 of the camera lap source pre-registration
(docs/notes/preregistration-pnp-v0.5-laps-camera.md sections 2 and 3): run loading and the
decisions D9-S0, D9-S1 and D9-S2 as pure functions.

Every threshold is read from `evaluation/prereg/pnp-v0.5-laps-camera.yaml` (block
`lap_battery_b9`), whose SHA-256 is a constant of this file and is checked on every load. D9-S1
calls D3-S1 (`lap_battery_b4.d3_s1`) with the yaml's counts (11 of 12, 7 of 8 after one dropped
case); D3-S1's contacts threshold (0) comes from `pnp-v0.3.yaml` through
`lap_battery_b4.load_constants`, which checks that file's hash too. A threshold that cannot be
applied as written is reported, never edited.

What B9 is, in the pre-registration: the lap memory takes its laps from a truth oracle (the
simulator's ground-truth line crossing, `tests/l5_reactive_bench/truth_lap_source.py`), so the
battery tests the learning through the new path (condition c) and the fallback when the source is
wrong (condition f), and says nothing about lap counting. 13 ground-truth laps are driven, laps 1
to 12 are judged. The registered truth is computed afterwards from the recording and reads neither
the oracle nor the system.
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
PREREG_YAML = REPO_ROOT / "evaluation" / "prereg" / "pnp-v0.5-laps-camera.yaml"
V05_LAPS_YAML = REPO_ROOT / "evaluation" / "prereg" / "pnp-v0.5-laps.yaml"
V05_RING_YAML = REPO_ROOT / "evaluation" / "prereg" / "pnp-v0.5-laps-ring.yaml"
#: SHA-256 of `pnp-v0.5-laps-camera.yaml` as committed with the pre-registration (commit 1 of brief
#: 51). Checked on every load and written into every run and the report.
REGISTERED_YAML_SHA256 = "013cb2f8bec3ce96d2abf456361c3de93d8f85da48c01900c374287d7473807f"
V05_LAPS_SHA256 = "35949c7d86dbdd3c5a3dde11eb0551465ea893b5f26e78dc7de0f1a0890428de"
V05_RING_SHA256 = "74b1dccd0d30910a7e96a45af65a00de62446902380bfb2bae76512d0498dc36"
B9_CONDITIONS = ("a", "b", "c", "f")
B9_SEEDS = (71, 72)
B9_DIRECTIONS = ("ccw", "cw")
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
    """The `lap_battery_b9` block of the yaml, the SHA-256 it was read under, the whole document
    and the one number D9-S1 takes from D3 (`max_contacts`). Has the interface of
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
    block = doc["lap_battery_b9"]
    t = block["thresholds"]
    if t["s1_reach_min_runs"] != t["s1_min_complete"]:
        raise PreregError("s1_reach_min_runs differs from s1_min_complete: D3-S1 uses one number")
    if int(t["s1_complete_laps"]) != b4.LAPS_TO_COMPLETE:
        raise PreregError(f"s1_complete_laps is not {b4.LAPS_TO_COMPLETE}")
    if tuple(int(k) for k in t["late_laps"]) != b4.LATE_LAPS:
        raise PreregError(f"late_laps is not {list(b4.LATE_LAPS)}")
    if tuple(int(s) for s in block["seeds"]) != B9_SEEDS:
        raise PreregError(f"seeds are not {list(B9_SEEDS)}")
    if tuple(block["directions"]) != B9_DIRECTIONS:
        raise PreregError(f"directions are not {list(B9_DIRECTIONS)}")
    if int(block["laps_judged"]) != LAP_JUDGED:
        raise PreregError(f"laps_judged is not {LAP_JUDGED}")
    if doc.get("v05_laps_sha256") != V05_LAPS_SHA256:
        raise PreregError(f"v05_laps_sha256 is not {V05_LAPS_SHA256}")
    if doc.get("v05_laps_ring_sha256") != V05_RING_SHA256:
        raise PreregError(f"v05_laps_ring_sha256 is not {V05_RING_SHA256}")


def load_constants(
    yaml_path: Path = PREREG_YAML, expected_sha: str = REGISTERED_YAML_SHA256
) -> Constants:
    """The yaml's `lap_battery_b9` block, refused when its SHA-256 is not the registered one."""
    actual = sha256_file(yaml_path)
    if actual != expected_sha:
        raise PreregError(
            f"{yaml_path} has SHA-256 {actual}, the registered value is {expected_sha}: refusing"
        )
    doc = yaml.safe_load(yaml_path.read_text())
    _check_consistent(doc)
    d3 = b4.load_constants()  # checks the v0.3 file's hash against its pre-registration
    return Constants(actual, doc["lap_battery_b9"], doc, int(d3.th["max_contacts"]))


# -- runs -------------------------------------------------------------------------------------------------


def _times(seq: Any) -> tuple[float, ...]:
    return tuple(float(x) for x in seq)


def summarise_run(
    result: dict, condition: str, case: str, direction: str, seed: int, const: Constants
) -> RunSummary:
    """A RunSummary on the registered truth, with the truth crossings, the system's laps, the
    oracle's events, the lap node's status and the guidance cap series in `extras`. A result
    without the registered truth is refused. A run whose truth recording ends before the end of
    lap 12 is a run that failed to execute: its `error` says so."""
    name = f"{condition}_{case}_{direction}_{seed}"
    if "gt_lap_times_registered_s" not in result or "truth_valid" not in result:
        raise RunFileError(
            f"{name}: no gt_lap_times_registered_s and truth_valid "
            "(the runner of brief 48 is required)"
        )
    run = b4.summarise_b4_run(result, condition, case, direction, seed)
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
    }
    run.extras.update(extras)
    n = const.laps_judged
    last = result.get("truth_last_odom_s")
    if (
        len(truth_crossings) >= n
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
    """Every `<condition>_<case>_<direction>_<seed>.json` of a B9 directory (condition a, b, c, f).
    Each must carry the `battery_b9` block with the mode asked for."""
    out = []
    for path in sorted(results_dir.glob("*.json")):
        parts = path.stem.split("_")
        if len(parts) != 4 or parts[0] not in B9_CONDITIONS:
            continue
        condition, case, direction, seed = parts
        data = json.loads(path.read_text())
        meta = data.get("battery_b9")
        if not isinstance(meta, dict) or meta.get("mode") != mode:
            raise RunFileError(f"{path.name}: not a B9 {mode} run file")
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


# -- D9-S0 -----------------------------------------------------------------------------------------------------


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


def d9_s0(runs: list[RunSummary], const: Constants | None = None) -> Decision:
    """The oracle and the wiring, condition c, every run (pre-registration section 3)."""
    const = const or load_constants()
    b4._check_unique(runs)
    c = b4._need(runs, "c", "D9-S0")
    bad: dict[str, list[str]] = {"oracle": [], "system": [], "crossings": [], "execute": []}
    for r in c:
        for key, lines in _s0_run(r, const).items():
            bad[key].extend(lines)
    n = const.laps_judged
    items: dict[str, tuple[bool | None, str]] = {
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
    }
    return b4._decision("D9-S0", items)


# -- D9-S1 -----------------------------------------------------------------------------------------------------


def d9_s1(runs: list[RunSummary], const: Constants | None = None) -> Decision:
    """D3-S1's four items on condition c against its paired control b, with the counts of the
    yaml (11 of 12, 7 of 8 after one dropped case)."""
    const = const or load_constants()
    return dataclasses.replace(b4.d3_s1(runs, const), name="D9-S1")  # type: ignore[arg-type]


# -- D9-S2 -----------------------------------------------------------------------------------------------------


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


def d9_s2(runs: list[RunSummary], const: Constants | None = None) -> Decision:
    """The fallback in closed loop, condition f, every run (pre-registration section 3)."""
    const = const or load_constants()
    b4._check_unique(runs)
    f = b4._need(runs, "f", "D9-S2")
    bad: dict[str, list[str]] = {}
    for r in f:
        for key, lines in _s2_run(r, const).items():
            bad.setdefault(key, []).extend(lines)
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
    }
    return b4._decision("D9-S2", items)


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
