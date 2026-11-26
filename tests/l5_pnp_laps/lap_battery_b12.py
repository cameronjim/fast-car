"""B12 copy of lap_battery_b11.py (pnp-v0.7-laps-k6-b12.yaml): same decisions, B12 constants.

Battery B11 of the D v1 pre-registration (brief 53): constants, run summaries, the candidate rule
of M-D1b and the decisions D11-S0 to D11-S3 (pre-registration) and D11-S4 (addendum 1), as pure
functions. No ROS.

Constants come from `evaluation/prereg/pnp-v0.6-laps-dv1.yaml` and its addendum 1, each checked
against the SHA-256 below on every load. D11-S0 and D11-S1 call the D10-S0 and D10-S1 functions of
`lap_battery_b10` unchanged (S0 items 1 to 5 with B10's own tolerances, S1 with B11's counts). Item 3
and the brake rate of D11-S2 call the D3-S2 function of `lap_battery_b4` unchanged, on the k runs
under the variant name it knows. Thresholds are read from the yaml, never written here, except the
labels that quote it.
"""

from __future__ import annotations

import dataclasses
import hashlib
import itertools
import json
import math
import statistics
import sys
from dataclasses import dataclass
from pathlib import Path
from typing import Any

import yaml

HERE = Path(__file__).resolve().parent
sys.path.insert(0, str(HERE))
sys.path.insert(0, str(HERE.parents[0] / "l5_reactive_bench"))
import dv1_measures
import lap_battery_b4 as b4
import lap_battery_b10 as b10
from lap_battery import Decision, RunSummary

REPO_ROOT = HERE.parents[1]
PREREG_YAML = REPO_ROOT / "evaluation" / "prereg" / "pnp-v0.7-laps-k6-b12.yaml"
ADDENDUM_YAML = REPO_ROOT / "evaluation" / "prereg" / "pnp-v0.6-laps-dv1-addendum-1.yaml"
#: B12 (pnp-v0.7-laps-k6-b12.yaml); the addendum is B11's addendum 1, unchanged (its D11-S4 is D12-S4).
REGISTERED_YAML_SHA256 = "ce8de8bed33f57e84d848b7a4745ec0e8f0f7e352250e3a1ab925d64e817fd03"
B11_YAML_SHA256 = "a75b088fb15f791d0ba23087708e0e8ebdfbbe759af6e4342e620f1b38d06c4b"
ADDENDUM_SHA256 = "e5cee1c64b9ac9232a1c819f65188eee0947dcffe5e8e30237de4e58952576fa"

B11_CONDITIONS = ("a", "b", "c", "k", "t", "kf")
DEV_CONDITIONS = ("a", "b", "c", "m", "f", "u", "t", "z", "K1", "K2", "K3")
CANDIDATES = ("K6",)
EPS = 1e-9


class PreregError(ValueError):
    pass


def sha256_file(path: Path) -> str:
    return hashlib.sha256(path.read_bytes()).hexdigest()


@dataclass(frozen=True)
class Constants:
    """The yaml (and addendum) as read under their registered hashes, plus B10's constants (the
    D10 functions and the fault plan B11 reuses unchanged)."""

    yaml_sha256: str
    addendum_sha256: str
    doc: dict[str, Any]
    addendum: dict[str, Any]
    b10: b10.Constants

    @property
    def raw(self) -> dict[str, Any]:
        return dict(self.doc["lap_battery_b12"])

    @property
    def common(self) -> dict[str, Any]:
        return dict(self.doc["common"])

    @property
    def development(self) -> dict[str, Any]:
        return dict(self.doc["development"])

    @property
    def battery_cases(self) -> dict[str, dict]:
        return dict(self.raw["cases"])

    @property
    def battery_seeds(self) -> tuple[int, ...]:
        return tuple(int(s) for s in self.raw["seeds"])

    @property
    def development_seeds(self) -> tuple[int, ...]:
        return tuple(int(s) for s in self.development["seeds"])

    @property
    def development_case_names(self) -> tuple[str, ...]:
        return tuple(self.development["cases"])

    @property
    def minimum_cases(self) -> tuple[str, ...]:
        return tuple(self.development["minimum_cases"])

    @property
    def directions(self) -> tuple[str, ...]:
        return tuple(self.raw["directions"])

    @property
    def laps_judged(self) -> int:
        return int(self.common["laps_judged"])

    @property
    def laps_driven(self) -> int:
        return int(self.common["laps_driven"])

    @property
    def laps(self) -> int:
        return self.laps_driven

    @property
    def late_laps(self) -> tuple[int, ...]:
        return tuple(int(k) for k in self.common["late_laps"])

    @property
    def lap1_cap_mps(self) -> float:
        return float(self.common["lap1_cap_mps"])

    @property
    def conditions(self) -> dict[str, dict]:
        return dict(self.raw["conditions"])

    @property
    def drop_rule(self) -> dict[str, Any]:
        return dict(self.raw["drop_rule"])

    @property
    def thresholds(self) -> dict[str, Any]:
        return dict(self.raw["thresholds"])

    @property
    def s4(self) -> dict[str, Any]:
        return dict(self.addendum["lap_battery_b11"]["s4"])

    @property
    def fault_plan(self) -> dict[str, Any]:
        return self.b10.fault_plan

    @property
    def oracle_ready_after_s(self) -> dict[int, float]:
        return {int(k): float(v) for k, v in self.raw["oracle_ready_after_s"].items()}

    @property
    def candidate_rule(self) -> dict[str, Any]:
        return dict(self.development["candidate_rule"])

    @property
    def th(self) -> dict[str, float]:
        """D10-S1 (D3-S1) and D3-S2 under the names those functions read, from the B11 block."""
        s1 = self.thresholds["s1"]
        s2 = self.thresholds["s2"]
        return {
            "max_contacts": float(s1["contacts_allowed"]),
            "s1_reach_max_ratio_to_b": float(s1["reach_ratio_to_b"]),
            "s1_max_median_r": float(s1["median_r_max"]),
            "s2_max_median_ratio_to_c": float(s2["median_paired_t_late_ratio_k_over_c_max"]),
            "s2_max_brakes_per_lap_late": float(s2["mean_ttc_or_obstacle_brakes_per_lap_late_max"]),
        }

    def required_complete(self, n_runs: int) -> int:
        """15 of 16, 11 of 12 after one dropped case. Nothing else is registered."""
        table = self.thresholds["s1"]["min_complete"]
        for key, n in (("of_16", 16), ("of_12", 12)):
            if n_runs == n:
                return int(table[key])
        raise PreregError(f"{n_runs} runs in a condition: the pre-registration has 16 or 12")


def _check_consistent(doc: dict[str, Any], addendum: dict[str, Any]) -> None:
    block = doc["lap_battery_b12"]
    if addendum.get("v06_laps_dv1_sha256") != B11_YAML_SHA256:
        raise PreregError("the addendum is not B11's addendum 1")
    if doc.get("v06_addendum_sha256") != ADDENDUM_SHA256:
        raise PreregError("the B12 file names another addendum")
    if int(block["runs_planned"]) != 88:
        raise PreregError("runs_planned is not 88")
    for key in ("v05_laps_camera_2_sha256",):
        if doc[key] != b10.REGISTERED_YAML_SHA256:
            raise PreregError(f"{key} is not B10's registered hash")
    if doc["development"]["candidate"]["name"] not in CANDIDATES:
        raise PreregError("the B12 candidate is not K6")


def load_constants(
    path: Path = PREREG_YAML,
    expected_sha256: str = REGISTERED_YAML_SHA256,
    addendum_path: Path = ADDENDUM_YAML,
    addendum_sha256: str = ADDENDUM_SHA256,
) -> Constants:
    """The registered constants, or PreregError when either file is not the registered one."""
    got = sha256_file(path)
    if got != expected_sha256:
        raise PreregError(f"{path} has SHA-256 {got}, the registered one is {expected_sha256}")
    got_a = sha256_file(addendum_path)
    if got_a != addendum_sha256:
        raise PreregError(f"{addendum_path} has SHA-256 {got_a}, registered {addendum_sha256}")
    doc = yaml.safe_load(path.read_text())
    addendum = yaml.safe_load(addendum_path.read_text())
    _check_consistent(doc, addendum)
    return Constants(got, got_a, doc, addendum, b10.load_constants())


# -- run summaries -------------------------------------------------------------------------------------


def summarise_run(
    result: dict, condition: str, case: str, direction: str, seed: int, const: Constants
) -> RunSummary:
    """B10's summary (registered truth, system laps, oracle events, start trace) plus what B11
    reads: per lap clearance, speed commands and engagements, the longest stop, the published
    bound per lap, the lap node's resets, the ceiling."""
    run = b10.summarise_run(result, condition, case, direction, seed, const.b10)
    lap_params = result.get("lap_params") or {}
    run.extras.update(
        {
            "engage_per_lap": list(result.get("engage_per_lap") or []),
            "min_body_clearance_by_lap_m": list(result.get("min_body_clearance_by_lap_m") or []),
            "max_cmd_speed_by_lap_mps": list(result.get("max_cmd_speed_by_lap_mps") or []),
            "max_cmd_speed_mps": result.get("max_cmd_speed_mps"),
            "longest_stop_s": result.get("longest_stop_s"),
            "bound_by_lap": dict(result.get("bound_by_lap") or {}),
            "ceiling_mps": lap_params.get("v_cap_ceiling_mps", result.get("v_max_mps")),
            "real_time_factor": result.get("real_time_factor"),
        }
    )
    return run


def load_runs(results_dir: Path, const: Constants, conditions: tuple[str, ...]) -> list[RunSummary]:
    out = []
    for path in sorted(Path(results_dir).glob("*.json")):
        parts = path.stem.split("_")
        if len(parts) != 4 or parts[0] not in conditions:
            continue
        condition, case, direction, seed = parts
        data = json.loads(path.read_text())
        out.append(summarise_run(data, condition, case, direction, int(seed), const))
    return out


def _label(r: RunSummary) -> str:
    return f"{r.condition} {r.case}/{r.direction}/{r.seed}"


def _of(runs: list[RunSummary], cond: str) -> list[RunSummary]:
    return [r for r in runs if r.condition == cond]


def as_condition(runs: list[RunSummary], cond: str) -> list[RunSummary]:
    return [dataclasses.replace(r, condition=cond) for r in runs]


# -- D11-S0 and D11-S1: the D10 functions, unchanged ----------------------------------------------------

S0_ITEMS = 5  # D10-S0 items 1 to 5 (item 6 is condition r, which B11 does not run)


def d11_s0(runs: list[RunSummary], const: Constants) -> Decision:
    """D10-S0 items 1 to 5, unchanged, on every c and every k run."""
    items: dict[str, tuple[bool | None, str]] = {}
    for cond in ("c", "k"):
        rs = _of(runs, cond)
        if not rs:
            raise b4.MissingPairError(f"D11-S0 needs condition {cond} runs")
        d = b10.d10_s0(as_condition(rs, "c"), const.b10)
        for name, (ok, detail) in list(d.items.items())[:S0_ITEMS]:
            items[f"{cond}: {name}"] = (ok, detail)
    return b4._decision("D11-S0", items)


def d11_s1(runs: list[RunSummary], const: Constants) -> Decision:
    """D10-S1 on the c runs against b, with B11's counts (15 of 16, 11 of 12)."""
    rs = [r for r in runs if r.condition in ("b", "c")]
    return dataclasses.replace(b4.d3_s1(rs, const), name="D11-S1")  # type: ignore[arg-type]


# -- D11-S2 ---------------------------------------------------------------------------------------------


def t_late(r: RunSummary, late: tuple[int, ...] = (8, 9, 10)) -> float:
    if r.error or r.laps < max(late):
        return math.nan
    return statistics.median(r.gt_lap_times_s[i - 1] for i in late)


def engagements(r: RunSummary, first: int, last: int) -> int:
    """PHASE_ENGAGE records of /safety/events on truth laps first to last (every source)."""
    per = r.extras.get("engage_per_lap") or []
    return sum(
        sum(int(v) for v in (per[k - 1] or {}).values())
        for k in range(first, last + 1)
        if k <= len(per)
    )


def late_clearance(r: RunSummary, late: tuple[int, ...] = (8, 9, 10)) -> float:
    per = r.extras.get("min_body_clearance_by_lap_m") or []
    vals = [per[k - 1] for k in late if k <= len(per) and per[k - 1] is not None]
    if len(vals) < len(late):
        return math.nan
    return float(min(vals))


def _ratio(x: float, y: float) -> float:
    return x / y if (math.isfinite(x) and math.isfinite(y) and y > 0.0) else math.nan


def d11_s2(runs: list[RunSummary], const: Constants) -> Decision:
    th = const.thresholds["s2"]
    k = _of(runs, "k")
    c = _of(runs, "c")
    if not k or not c:
        raise b4.MissingPairError("D11-S2 needs condition k and c runs")
    pairs = b4.paired(k, c, "c")
    n_req = const.required_complete(len(k))
    # items 3 and 5: the D3-S2 function, the k runs under the variant name it knows
    d3 = b4.d3_s2(as_condition(k, "d") + c, "d", const)  # type: ignore[arg-type]
    d3_items = list(d3.items.items())
    median_item = next(v for n, v in d3_items if n.startswith("median T_late"))
    brake_item = next(v for n, v in d3_items if n.startswith("mean TTC"))
    items: dict[str, tuple[bool | None, str]] = {}
    items[f"1. {th['contacts_allowed_k']} wall contacts in the k runs"] = b4._contacts_item(
        k, int(th["contacts_allowed_k"])
    )
    done_k = [r for r in k if r.completed]
    done_c = [r for r in c if r.completed]
    stalls = [
        f"{_label(r)} {r.extras.get('longest_stop_s')} s"
        for r in k
        if not (
            isinstance(r.extras.get("longest_stop_s"), (int, float))
            and r.extras["longest_stop_s"] <= float(th["stalls_over_s"]) + EPS
        )
    ]
    ok2 = (
        len(done_k) >= n_req
        and len(done_k) >= len(done_c)
        and len(stalls) <= int(th["stalled_k_runs_allowed"])
    )
    items[
        f"2. at least {n_req} of {len(k)} k runs complete 10 laps, not fewer than the c runs, "
        f"no k run stalls more than {th['stalls_over_s']} s"
    ] = (
        ok2,
        (
            f"k {len(done_k)} complete, c {len(done_c)}; not complete "
            f"{[_label(r) for r in k if not r.completed][:6]}; stalls (or no measure) {stalls[:6]}"
        ),
    )
    items[
        f"3. median over pairs of T_late(k) / T_late(c) <= {th['median_paired_t_late_ratio_k_over_c_max']}"
    ] = median_item
    ratios = [(_label(x), _ratio(t_late(x), t_late(y))) for x, y in pairs]
    finite = [(lab, q) for lab, q in ratios if math.isfinite(q)]
    worst = float(th["worst_paired_t_late_ratio_k_over_c_max"])
    above = [(lab, round(q, 3)) for lab, q in finite if q > worst + EPS]
    items[f"4. no pair above {worst}"] = (
        bool(finite) and not above,
        (
            f"{len(above)} above {above[:6]}; max "
            f"{max((q for _, q in finite), default=math.nan):.3f} over {len(finite)} pairs with "
            f"both complete ({len(ratios) - len(finite)} pairs without)"
        ),
    )
    items[
        f"5. mean TTC or obstacle brakes per lap, laps 8 to 10, <= {th['mean_ttc_or_obstacle_brakes_per_lap_late_max']}"
    ] = brake_item
    lo, hi = (int(x) for x in th["safety_engagements"]["laps"])
    eng_k = sum(engagements(x, lo, hi) for x, _ in pairs)
    eng_c = sum(engagements(y, lo, hi) for _, y in pairs)
    items[
        f"6. safety engagements on laps {lo} to {hi}, sum over k at most the sum over paired c"
    ] = (
        eng_k <= eng_c,
        f"k {eng_k}, c {eng_c}",
    )
    cl_k = [late_clearance(r) for r in k]
    cl_c = [late_clearance(r) for r in c]
    cl_k = [x for x in cl_k if math.isfinite(x)]
    cl_c = [x for x in cl_c if math.isfinite(x)]
    med_k = statistics.median(cl_k) if cl_k else math.nan
    med_c = statistics.median(cl_c) if cl_c else math.nan
    margin = 0.03  # the yaml's "median over c runs minus 0.03 m"
    if "minus 0.03 m" not in str(th["clearance"]["median_over_k_runs_at_least"]):
        raise PreregError("the clearance item no longer says 'minus 0.03 m'")
    items["7. median lowest body to wall clearance on laps 8 to 10, k at least c minus 0.03 m"] = (
        math.isfinite(med_k) and math.isfinite(med_c) and med_k >= med_c - margin - EPS,
        f"k {med_k:.4f} m over {len(cl_k)} runs, c {med_c:.4f} m over {len(cl_c)} runs",
    )
    tol = 0.01  # the yaml's "plus 0.01"
    lap1 = const.lap1_cap_mps
    bad8 = []
    for r in k:
        by_lap = r.extras.get("max_cmd_speed_by_lap_mps") or []
        top = r.extras.get("max_cmd_speed_mps")
        ceiling = r.extras.get("ceiling_mps")
        if not by_lap or by_lap[0] is None or top is None or ceiling is None:
            bad8.append(f"{_label(r)}: no speed command measure")
            continue
        if by_lap[0] > lap1 + tol + EPS:
            bad8.append(f"{_label(r)}: lap 1 command {by_lap[0]:.3f} > {lap1} + {tol}")
        if top > float(ceiling) + tol + EPS:
            bad8.append(f"{_label(r)}: command {top:.3f} > ceiling {ceiling} + {tol}")
    items[f"8. speed command on lap 1 at most {lap1} + {tol}, never above the ceiling + {tol}"] = (
        not bad8,
        f"{len(bad8)} exceptions {bad8[:6]}",
    )
    return b4._decision("D11-S2", items)


SAFETY_ITEMS_S2 = ("1.", "2.", "5.", "6.", "7.", "8.")


# -- D11-S3 ---------------------------------------------------------------------------------------------


def bound_failures(run: RunSummary, const: Constants) -> list[str]:
    """Station bounds of the lap after a doubtful lap above that station's bound in the doubtful
    lap (system laps, the published bound per station bin of `dv1_measures.BoundLog`)."""
    rows = b10.system_laps_by_truth(run, const.b10)
    bounds = run.extras.get("bound_by_lap") or {}
    out = []
    for a, b in itertools.pairwise(rows):
        if not a["doubtful"]:
            continue
        # guidance lap indices are the lap being driven: system lap j is guidance lap j
        ea, eb = bounds.get(str(a["lap"])), bounds.get(str(b["lap"]))
        if eb is None:
            continue
        if ea is None:
            out.append(f"{_label(run)}: lap {b['lap']} has bounds, doubtful lap {a['lap']} none")
            continue
        above = dv1_measures.bounds_above_after(ea, eb, tol=1e-4)
        if above:
            k0, v1, v0 = above[0]
            out.append(
                f"{_label(run)}: lap {b['lap']} above doubtful lap {a['lap']} at {len(above)} "
                f"stations, first {k0 * dv1_measures.BIN_M:.1f} m: {v1:.3f} > {v0:.3f}"
            )
    return out


def d11_s3(runs: list[RunSummary], const: Constants) -> Decision:
    th = const.thresholds["s3"]
    kf = _of(runs, "kf")
    if not kf:
        raise b4.MissingPairError("D11-S3 needs condition kf runs")
    n = const.laps_judged
    need = int(th["min_complete_12_truth_laps"]["of_16" if len(kf) == 16 else "of_12"])
    if len(kf) not in (16, 12):
        raise PreregError(f"{len(kf)} kf runs: the pre-registration has 16 or 12")
    items: dict[str, tuple[bool | None, str]] = {}
    items[f"contacts {th['contacts_allowed']}"] = b4._contacts_item(kf, int(th["contacts_allowed"]))
    done = [r for r in kf if not r.error and r.laps >= n]
    items[f"at least {need} of {len(kf)} complete {n} truth laps"] = (
        len(done) >= need,
        f"{len(done)} complete; not: {[_label(r) for r in kf if r not in done][:6]}",
    )
    rates = []
    for r in kf:
        laps = r.brakes_per_lap[:n]
        if laps:
            rates.append(sum(laps) / len(laps))
    mean_rate = statistics.fmean(rates) if rates else math.nan
    bmax = float(th["mean_ttc_or_obstacle_brakes_per_lap_all_laps_max"])
    items[f"mean TTC or obstacle brakes per lap over all laps <= {bmax}"] = (
        bool(rates) and mean_rate <= bmax + EPS,
        f"{mean_rate:.3f} over {len(rates)} runs",
    )
    cap, overdue, bound = [], [], []
    for r in kf:
        bad = b10._s2_run(r, const.b10)
        cap += bad["cap"]
        overdue += bad["overdue"]
        bound += bound_failures(r, const)
    items[
        "D10-S2 item 5: the cap of the lap after a doubtful lap is at most that of the doubtful lap"
    ] = (
        not cap,
        f"{len(cap)} exceptions {cap[:6]}",
    )
    items["D10-S2 item 6: the lowest guidance cap while overdue is the lap-1 cap"] = (
        not overdue,
        f"{len(overdue)} exceptions {overdue[:6]}",
    )
    items[
        "no station bound of the lap after a doubtful lap above its bound in the doubtful lap"
    ] = (
        not bound,
        f"{len(bound)} exceptions {bound[:6]}",
    )
    return b4._decision("D11-S3", items)


# -- D11-S4 (addendum 1) -------------------------------------------------------------------------------


def sum_laps(r: RunSummary, first: int = 2, last: int = 10) -> float:
    if r.error or r.laps < last:
        return math.nan
    return float(sum(r.gt_lap_times_s[first - 1 : last]))


def d11_s4(
    runs: list[RunSummary], const: Constants, s0: Decision, s1: Decision, s2: Decision, s3: Decision
) -> Decision:
    s4 = const.s4
    k, c = _of(runs, "k"), _of(runs, "c")
    pairs = b4.paired(k, c, "c")
    ratios = [(_label(x), _ratio(sum_laps(x), sum_laps(y))) for x, y in pairs]
    worst = float(s4["worst_paired_sum_ratio_k_over_c_max"])
    med_max = float(s4["median_paired_sum_ratio_k_over_c_max"])
    # a pair without all of laps 2 to 10 counts as a pair above the worst bar (addendum)
    eff = [(lab, q if math.isfinite(q) else math.inf) for lab, q in ratios]
    med = statistics.median(q for _, q in eff) if eff else math.nan
    above = [(lab, round(q, 3)) for lab, q in eff if q > worst + EPS]
    items: dict[str, tuple[bool | None, str]] = {
        f"1. median over pairs of sum(laps 2-10, k) / sum(laps 2-10, c) <= {med_max}": (
            bool(eff) and med <= med_max + EPS,
            f"median {med:.3f} over {len(eff)} pairs",
        ),
        f"2. no pair above {worst} (a pair without laps 2 to 10 counts as above)": (
            bool(eff) and not above,
            f"{len(above)} above {above[:6]}",
        ),
    }
    s2_needed = [
        v
        for name, v in s2.items.items()
        if name.startswith(("1.", "2.", "4.", "5.", "6.", "7.", "8."))
    ]
    also = s0.passed and s1.passed and all(ok for ok, _ in s2_needed) and s3.passed
    items["3. D11-S0, D11-S1, D11-S2 items 1, 2, 4 to 8 and D11-S3 pass"] = (
        bool(also),
        f"S0 {s0.passed}, S1 {s1.passed}, S2 items {[bool(ok) for ok, _ in s2_needed]}, S3 {s3.passed}",
    )
    return b4._decision("D11-S4", items)


def outcome(s0: Decision, s1: Decision, s2: Decision, s3: Decision, t_within: bool | None) -> str:
    """The registered outcome sentence."""
    out = ""
    if not s0.passed:
        out = "lap bookkeeping failed, not judged"
    elif not s1.passed:
        out = "no result: the D v0 baseline failed, D v1 is not judged"
    else:
        failed = [name for name, (ok, _) in s2.items.items() if not ok]
        safety_failed = [n for n in failed if n.startswith(SAFETY_ITEMS_S2)]
        if not failed and s3.passed:
            out = "D v1 faster than the schedule in sim at equal safety"
        elif safety_failed or not s3.passed:
            out = "unsafe in sim"
        elif all(n.startswith("3.") for n in failed):
            out = "no better than the schedule in sim"
        else:
            out = (
                "not covered word for word: D11-S2 failed on items "
                f"{[n.split('.')[0] for n in failed]} with no safety item and S3 passing"
            )
    if t_within:
        out += "; gain not shown to need memory"
    return out


def outcome_s4(s4: Decision, s0: Decision, s1: Decision) -> str:
    if not (s0.passed and s1.passed):
        return "not judged"
    items = list(s4.items.values())
    if not items[2][0]:
        return "not judged"
    if items[0][0] and items[1][0]:
        return "D v1 learns faster than the schedule in sim at equal safety"
    return "D v1 does not learn faster than the schedule in sim"


def t_within_2pct(runs: list[RunSummary]) -> tuple[bool | None, str]:
    """`T_late(t)` within 2 percent of `T_late(k)`: the median over pairs of |t / k - 1| <= 0.02."""
    t = {r.key: r for r in _of(runs, "t")}
    qs = [_ratio(t_late(t[r.key]), t_late(r)) for r in _of(runs, "k") if r.key in t]
    qs = [q for q in qs if math.isfinite(q)]
    if not qs:
        return None, "no pairs"
    med = statistics.median(qs)
    return abs(
        med - 1.0
    ) <= 0.02 + EPS, f"median T_late(t) / T_late(k) {med:.3f} over {len(qs)} pairs"


# -- the candidate rule (M-D1b) -------------------------------------------------------------------------


@dataclass(frozen=True)
class CandidateScore:
    name: str
    eligible: bool
    score: float
    n_pairs: int
    why_not: tuple[str, ...]


def score_candidate(name: str, runs: list[RunSummary], const: Constants) -> CandidateScore:
    rule = const.candidate_rule
    el = rule["eligible"]
    kk = _of(runs, name)
    c = {r.key: r for r in _of(runs, "c")}
    why: list[str] = []
    cases = {r.case for r in kk}
    missing = [m for m in const.minimum_cases if m not in cases]
    if missing:
        why.append(f"minimum cases not run: {missing}")
    if sum(r.contacts for r in kk) > int(el["contacts"]):
        why.append(f"{sum(r.contacts for r in kk)} contacts")
    n = const.laps_judged
    short = [_label(r) for r in kk if r.error or r.laps < n]
    if len(short) > int(el["runs_not_completing_12_laps"]):
        why.append(f"not completing {n} laps: {short[:4]}")
    stalls = [
        _label(r)
        for r in kk
        if not (
            isinstance(r.extras.get("longest_stop_s"), (int, float))
            and r.extras["longest_stop_s"] <= float(el["stalls_over_s"]) + EPS
        )
    ]
    if len(stalls) > int(el["stalls_allowed"]):
        why.append(f"stalls over {el['stalls_over_s']} s (or no measure): {stalls[:4]}")
    late = [r.late_brakes_per_lap for r in kk if not math.isnan(r.late_brakes_per_lap)]
    mean_b = statistics.fmean(late) if late else math.nan
    if not (late and mean_b <= float(el["mean_ttc_or_obstacle_brakes_per_lap_late_max"]) + EPS):
        why.append(f"late brakes per lap {mean_b:.3f}")
    qs = []
    for r in kk:
        if r.key not in c:
            why.append(f"{_label(r)} has no paired c run")
            continue
        qs.append(_ratio(t_late(r), t_late(c[r.key])))
    finite = [q for q in qs if math.isfinite(q)]
    score = statistics.median(finite) if finite and len(finite) == len(qs) else math.nan
    if not math.isfinite(score):
        why.append("no score (a pair without T_late)")
    return CandidateScore(name, not why, score, len(finite), tuple(why))


def candidate_rule(
    runs: list[RunSummary], const: Constants
) -> tuple[str | None, list[CandidateScore], str]:
    """(picked candidate or None, the scores, the sentence). Lowest score among the eligible;
    within `tie_within` of the lowest the tie order decides; above `max_score_to_run_b11` (or no
    eligible candidate) B11 is not run."""
    rule = const.candidate_rule
    scores = [score_candidate(n, runs, const) for n in CANDIDATES]
    ok = [s for s in scores if s.eligible]
    if not ok:
        return None, scores, str(rule["else"])
    best = min(s.score for s in ok)
    tied = [s.name for s in ok if s.score <= best + float(rule["tie_within"]) + EPS]
    pick = next(n for n in rule["tie_order"] if n in tied)
    chosen = next(s for s in ok if s.name == pick)
    if chosen.score > float(rule["max_score_to_run_b11"]) + EPS:
        return None, scores, str(rule["else"])
    return (
        pick,
        scores,
        f"candidate {pick} (score {chosen.score:.3f}; tied within {rule['tie_within']}: {tied})",
    )


# -- the drop rule --------------------------------------------------------------------------------------


@dataclass(frozen=True)
class DropResult:
    dropped: tuple[str, ...]
    decided: bool
    reason: str
    laps: dict[str, int]


def drop_rule(laps_by_run: dict[str, int], const: Constants) -> DropResult:
    """B10's rule with B11's cases: a case is dropped if either direction of condition a seed 91
    has fewer than `min_laps`; nothing is decided if more than `max_dropped` would go or fewer than
    `min_cases_left` stay."""
    rule = const.drop_rule
    need = int(rule["min_laps"])
    cases = list(const.battery_cases)
    dropped = [
        case
        for case in cases
        if any((laps_by_run.get(f"{case}_{d}") or 0) < need for d in const.directions)
        or any(laps_by_run.get(f"{case}_{d}") is None for d in const.directions)
    ]
    left = [c for c in cases if c not in dropped]
    reason = ""
    if len(dropped) > int(rule["max_dropped"]):
        reason = f"{len(dropped)} cases would be dropped, at most {rule['max_dropped']} may be"
    elif len(left) < int(rule["min_cases_left"]):
        reason = f"{len(left)} cases left, at least {rule['min_cases_left']} are needed"
    return DropResult(tuple(dropped), not reason, reason, dict(laps_by_run))


# -- reported without a threshold -----------------------------------------------------------------------


def laps_to_reach(r: RunSummary, ref_t_late: float, ratio: float = 1.05) -> int | None:
    if not math.isfinite(ref_t_late):
        return None
    for k, t in enumerate(r.gt_lap_times_s, start=1):
        if t <= ratio * ref_t_late + EPS:
            return k
    return None


def reported_rows(runs: list[RunSummary], k_name: str = "k") -> list[dict[str, Any]]:
    """Per k run: T_late of k, c, b, t and the ratios, the sums of laps 2 to 10, laps to reach
    1.05 x T_late(b), resets."""
    by = {cond: {r.key: r for r in _of(runs, cond)} for cond in ("b", "c", "t")}
    rows = []
    for r in _of(runs, k_name):
        b, c, t = (by[x].get(r.key) for x in ("b", "c", "t"))
        tb = t_late(b) if b else math.nan
        rows.append(
            {
                "run": f"{r.case}/{r.direction}/{r.seed}",
                "T_late_k": t_late(r),
                "T_late_c": t_late(c) if c else math.nan,
                "T_late_b": tb,
                "T_late_t": t_late(t) if t else math.nan,
                "k_over_c": _ratio(t_late(r), t_late(c)) if c else math.nan,
                "k_over_b": _ratio(t_late(r), tb),
                "t_over_k": _ratio(t_late(t), t_late(r)) if t else math.nan,
                "sum2_10_k": sum_laps(r),
                "sum2_10_c": sum_laps(c) if c else math.nan,
                "reach_k": laps_to_reach(r, tb),
                "reach_c": laps_to_reach(c, tb) if c else None,
                "resets": (r.extras.get("lap_status") or {}).get("resets"),
            }
        )
    return rows


def value_profile_reading(runs: list[RunSummary], max_ratio: float = 0.97) -> str:
    """Brief 44 / addendum 1 section 5.3, fixed before M-D1: a learned value profile is part of a
    later D only if u or f has a median late-lap time at most 0.97 times m's on the same pairs with
    no more contacts; otherwise the later D keeps the planner's progress reward and uses memory as a
    speed bound. Development only."""
    m = {r.key: r for r in _of(runs, "m")}
    parts = []
    keep = False
    for cond in ("u", "f"):
        rs = [r for r in _of(runs, cond) if r.key in m]
        if not rs:
            parts.append(f"{cond}: no pairs with m")
            continue
        qs = [_ratio(t_late(r), t_late(m[r.key])) for r in rs]
        qs = [q for q in qs if math.isfinite(q)]
        med = statistics.median(qs) if qs else math.nan
        c_x = sum(r.contacts for r in rs)
        c_m = sum(m[r.key].contacts for r in rs)
        ok = math.isfinite(med) and med <= max_ratio + EPS and c_x <= c_m
        keep = keep or ok
        parts.append(
            f"{cond}: median T_late / m {med:.3f} over {len(qs)} pairs, contacts {c_x} vs m {c_m}"
        )
    verdict = (
        "a learned value profile may be part of a later D"
        if keep
        else "the later D keeps the planner's own progress reward and uses memory as a speed bound"
    )
    return f"{verdict} ({'; '.join(parts)})"
