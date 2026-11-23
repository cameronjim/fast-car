"""L1: the decisions of battery B10 (`lap_battery_b10`, brief 51c), each item at its pass, at the
threshold, epsilon over, with a NaN, an invalid run and a missing lap; the start gate item and the
refusal item (condition r); a refused c run; the drop rule; the yaml hash; and the freeze guard of
`battery_b10`: the V cases and seeds 81 and 82 need the freeze file, the U cases, seeds 71 and 72,
B9's run files and both evaluation bags are refused always.

The runs are synthetic result JSONs in the shape of `track_family_runner.py`: a stadium of 10.6 m
lapped at 1.0 m/s, the oracle publishing every crossing, the lap node confirming them."""

from __future__ import annotations

import copy
import json
import math
from pathlib import Path

import battery_b10 as bat
import lap_battery_b10 as b10
import pytest
from lap_battery import RunSummary

CONST = b10.load_constants()
LAP = 10.6
N = 13
CAP0 = 0.5
EPS = 1e-6


def result(
    faults: dict | None = None,
    laps: int = N,
    contacts: int = 0,
    cap_step: float = 0.1,
    **over,
) -> dict:
    """A run result: `laps` truth laps of LAP seconds, truth crossing k at k * LAP (crossing 0 at
    1.0), the oracle as the plan says, the lap node confirming every published event."""
    faults = faults or {}
    c0 = 1.0
    crossings = [c0 + k * LAP for k in range(1, laps + 1)]
    drop = set(faults.get("drop_truth_crossings", []))
    events: list[dict] = []
    for k, t in enumerate(crossings, start=1):
        if k in drop:
            continue
        events.append({"kind": "true", "truth_crossing": k, "crossing_time_s": t, "lap": 0})
        dup = faults.get("duplicate_truth_crossing")
        if dup and k == dup["crossing"]:
            events.append({"kind": "duplicate", "truth_crossing": k, "crossing_time_s": t})
    false = faults.get("false_event")
    if false:
        base = crossings[false["lap"] - 2]
        events.append(
            {
                "kind": "false",
                "truth_crossing": None,
                "crossing_time_s": base + 0.6 * LAP,
            }
        )
    events.sort(key=lambda e: e["crossing_time_s"])
    published = [e for e in events if e["kind"] != "duplicate"]
    system = []
    doubtful = []
    cap = CAP0
    level = 0
    previous = c0
    clean_streak_needs_two = False
    streak = 0
    for j, e in enumerate(published, start=1):
        t = e["crossing_time_s"]
        bad = _is_doubtful(e, previous, crossings, drop, false)
        system.append(
            {
                "lap_index": j,
                "t_cross_s": t,
                "lap_time_s": t - previous,
                "cap_mps": cap,
                "clean": not bad,
                "frame_id": "truth",
            }
        )
        if bad:
            doubtful.append({"lap": j, "event_lap": j, "reasons": ["length", "heading"]})
            level = max(0, level - 1)
            streak = 0
            clean_streak_needs_two = True
        else:
            streak += 1
            if not clean_streak_needs_two or streak >= 2:
                level += 1
                streak = 0
                clean_streak_needs_two = False
        cap = min(1.3, CAP0 + cap_step * level)
        previous = t
    cap_series = [[0.0, CAP0]]
    # the cap the guidance carried: the lap-1 cap while overdue (one lap length after the last
    # system crossing plus its band), else the schedule
    for j, e in enumerate(published):
        cap_after = system[j + 1]["cap_mps"] if j + 1 < len(system) else cap
        cap_series.append([e["crossing_time_s"], cap_after])
        # overdue: no crossing one lap and a half later
        nxt = published[j + 1]["crossing_time_s"] if j + 1 < len(published) else math.inf
        if nxt - e["crossing_time_s"] > 1.3 * LAP:
            cap_series.append([e["crossing_time_s"] + 1.3 * LAP, CAP0])
    cap_series.sort()
    rej = {"not_monotone": 1} if faults.get("duplicate_truth_crossing") else {}
    out = {
        "gt_lap_times_registered_s": [LAP] * laps,
        "gt_lap_crossings_registered_s": crossings,
        "gt_lap1_start_s": c0,
        "truth_valid": True,
        "truth_last_odom_s": crossings[-1] + 1.0,
        "wall_contacts": contacts,
        "system_lap_events": system,
        "oracle_events": events,
        "lap_status": {
            "lap1_start_stamp_s": c0,
            "confirmed_laps": len(system) - len(doubtful),
            "doubtful_laps": doubtful,
            "rejected_events": rej,
            "lap_source": "truth",
        },
        "guidance_cap_series": cap_series,
        "lap_truth_faults": faults,
        "brake_per_lap": [{} for _ in range(laps)],
        "stopped_because": "laps",
        "start_trace": good_trace(),
        "source_ready": True,
    }
    out["lap_status"]["source_ready"] = True
    out.update(over)
    return out


def good_trace(**changes) -> dict:
    """The start of a run that went through the gate: the lap node's first status at 1.0 s, ready
    from 2.0 s, the gate's start at 3.0 s, the planner spawned at 3.0 s, DRIVING at 4.0 s."""
    out = {
        "lap_node_first_status_s": 1.0,
        "ready_first_s": 2.0,
        "gate_start_s": 3.0,
        "planner_spawn_s": 3.0,
        "driving_first_s": 4.0,
        "lap1_start_s": 4.02,
        "oracle_first_status_s": 0.5,
        "gate_verdict": "start",
        "gate_reason": "",
    }
    out.update(changes)
    return out


def refusal_result(**over) -> dict:
    """A condition r run: the oracle never ready, the starter refused 31 s after the lap node's
    first status, the planner never started."""
    out = {
        "wall_contacts": 0,
        "stopped_because": "start_refused",
        "drive_raw_messages": 0,
        "distance_m": 0.0,
        "start_trace": good_trace(
            gate_start_s=None,
            planner_spawn_s=None,
            driving_first_s=None,
            lap1_start_s=None,
            gate_verdict="refuse",
            gate_reason="source state 'waiting', not 'running'",
        ),
        "start_refusal": {
            "reason": "source state 'waiting', not 'running'",
            "refused_at_s": 32.0,
        },
        "lap_status": {"phase": "waiting_for_driving"},
    }
    out.update(over)
    return out


def _is_doubtful(e, previous, crossings, drop, false) -> bool:
    if e["kind"] == "false":
        return True
    k = e["truth_crossing"]
    # the merged lap, and the lap after the false event
    return k - 1 in drop or k == (false["lap"] if false else -1)


FAULTS = {
    "drop_truth_crossings": [4, 7],
    "false_event": {"lap": 9, "share_of_median_truth_lap_path": 0.6},
    "duplicate_truth_crossing": {"crossing": 10, "delay_s": 0.2},
    "numbering": "consecutive",
}


def run(cond: str, res: dict, case="V1", direction="ccw", seed=81) -> RunSummary:
    return b10.summarise_run(res, cond, case, direction, seed, CONST)


def r_runs(n: int = 3, **over) -> list[RunSummary]:
    return [run("r", refusal_result(**over), c, "ccw", 81) for c in ("V1", "V2", "V3")[:n]]


def runs_c(n_ok: int = 12, **kw) -> list[RunSummary]:
    out = []
    for i in range(12):
        case, direction, seed = (
            ("V1", "V2", "V3")[i % 3],
            ("ccw", "cw")[(i // 3) % 2],
            (81, 82)[i // 6],
        )
        out.append(run("c", result(**kw), case, direction, seed))
    return out


def swap(runs: list[RunSummary], i: int, cond: str, res: dict) -> RunSummary:
    """A run with the case, direction and seed of runs[i]."""
    r = runs[i]
    return run(cond, res, r.case, r.direction, r.seed)


def s0_ok(items) -> list[bool]:
    return [ok for ok, _ in items.values()]


# -- the constants ---------------------------------------------------------------------------------------


def test_the_constants_are_the_registered_ones(tmp_path: Path) -> None:
    assert CONST.yaml_sha256 == b10.REGISTERED_YAML_SHA256
    assert CONST.battery_seeds == (81, 82) and CONST.laps_driven == 13 and CONST.laps_judged == 12
    assert CONST.required_complete(12) == 11 and CONST.required_complete(8) == 7
    with pytest.raises(b10.PreregError):
        CONST.required_complete(10)
    bad = tmp_path / "x.yaml"
    bad.write_text(b10.PREREG_YAML.read_text() + "\n# edited\n")
    with pytest.raises(b10.PreregError, match="SHA-256"):
        b10.load_constants(bad)
    assert set(CONST.battery_cases) == {"V1", "V2", "V3"}


def test_the_registered_oracle_delays_the_spent_data_and_condition_r() -> None:
    assert CONST.oracle_ready_after_s == {81: 0.0, 82: 6.0}
    assert CONST.spent["seeds"] == [71, 72] and CONST.spent["cases"] == ["U1", "U2", "U3"]
    assert b10.ready_after_s(CONST, "c", 81) == 0.0 and b10.ready_after_s(CONST, "f", 82) == 6.0
    assert math.isinf(b10.ready_after_s(CONST, "r", 81))
    assert b10.ready_after_s(CONST, "b", 82) == 0.0
    r = CONST.conditions["r"]
    assert r["seeds"] == [81] and r["directions"] == ["ccw"]


def test_the_source_rule_block_is_the_first_attempts_and_a_change_is_refused() -> None:
    import yaml

    d9 = yaml.safe_load(b10.D9_YAML.read_text())
    assert CONST.doc["source_rule"] == d9["source_rule"]
    doc = copy.deepcopy(CONST.doc)
    doc["source_rule"]["source_stale_s"] = 4.0
    with pytest.raises(b10.PreregError, match="source_rule"):
        b10._check_consistent(doc)
    doc = copy.deepcopy(CONST.doc)
    doc["v05_laps_camera_sha256"] = "0" * 64
    with pytest.raises(b10.PreregError, match="v05_laps_camera_sha256"):
        b10._check_consistent(doc)
    doc = copy.deepcopy(CONST.doc)
    doc["lap_battery_b10"]["spent_never_run_again"]["seeds"] = [71]
    with pytest.raises(b10.PreregError, match="spent seeds"):
        b10._check_consistent(doc)


def test_the_battery_has_45_runs() -> None:
    jobs = bat.make_jobs(CONST, dropped=())
    assert len(jobs) == 6 + 12 + 12 + 12 + 3
    assert sum(1 for j in jobs if j[1] == "a") == 6 and all(j[4] == 81 for j in jobs if j[1] == "a")
    r = [j for j in jobs if j[1] == "r"]
    assert len(r) == 3 and all(j[3] == "ccw" and j[4] == 81 for j in r)
    assert len(bat.make_jobs(CONST, dropped=("V2",))) == 4 + 8 + 8 + 8 + 2


# -- D10-S0 ------------------------------------------------------------------------------------------------


def test_s0_passes_on_a_clean_oracle_run() -> None:
    d = b10.d10_s0(runs_c() + r_runs(), CONST)
    assert d.passed is True, d
    assert len(d.items) == 6


@pytest.mark.parametrize(("shift", "ok"), [(0.3, True), (0.3 + EPS, False), (-0.3, True)])
def test_s0_oracle_event_within_0_3_s_of_the_truth_crossing(shift: float, ok: bool) -> None:
    res = result()
    for e in res["oracle_events"]:
        if e["truth_crossing"] == 5:
            e["crossing_time_s"] += shift
    runs = runs_c()
    runs[0] = swap(runs, 0, "c", res)
    d = b10.d10_s0(runs, CONST)
    key = next(k for k in d.items if k.startswith("the oracle publishes"))
    assert d.items[key][0] is ok


def test_s0_a_missing_or_extra_oracle_event_fails() -> None:
    res = result()
    res["oracle_events"] = [e for e in res["oracle_events"] if e["truth_crossing"] != 3]
    runs = runs_c()
    runs[1] = swap(runs, 1, "c", res)
    d = b10.d10_s0(runs, CONST)
    assert d.passed is False
    res = result()
    res["oracle_events"].append(dict(res["oracle_events"][3], kind="duplicate"))
    runs[1] = swap(runs, 1, "c", res)
    assert b10.d10_s0(runs, CONST).passed is False  # a duplicate in condition c is not an oracle


@pytest.mark.parametrize(("shift", "ok"), [(0.3, True), (0.3 + EPS, False)])
def test_s0_system_crossing_within_0_3_s_of_the_truth_crossing_of_the_same_number(
    shift: float, ok: bool
) -> None:
    res = result()
    res["system_lap_events"][6]["t_cross_s"] += shift
    runs = runs_c()
    runs[2] = swap(runs, 2, "c", res)
    d = b10.d10_s0(runs, CONST)
    key = next(k for k in d.items if k.startswith("every system crossing"))
    assert d.items[key][0] is ok


def test_s0_nan_crossing_fails() -> None:
    res = result()
    res["system_lap_events"][2]["t_cross_s"] = math.nan
    runs = runs_c()
    runs[0] = swap(runs, 0, "c", res)
    assert b10.d10_s0(runs, CONST).passed is False


def test_s0_a_doubtful_lap_or_a_rejected_event_fails() -> None:
    res = result()
    res["lap_status"]["doubtful_laps"] = [{"lap": 5, "event_lap": 5, "reasons": ["length"]}]
    runs = runs_c()
    runs[0] = swap(runs, 0, "c", res)
    assert b10.d10_s0(runs + r_runs(), CONST).passed is False
    # a doubtful lap after the judged laps is not an S0 exception
    res = result()
    res["lap_status"]["doubtful_laps"] = [{"lap": 13, "event_lap": 13, "reasons": ["length"]}]
    runs[0] = swap(runs, 0, "c", res)
    assert b10.d10_s0(runs + r_runs(), CONST).passed is True
    res = result()
    res["lap_status"]["rejected_events"] = {"latency": 1}
    runs[0] = swap(runs, 0, "c", res)
    assert b10.d10_s0(runs + r_runs(), CONST).passed is False


def test_s0_a_missing_system_lap_fails() -> None:
    res = result()
    res["system_lap_events"] = res["system_lap_events"][:11]
    runs = runs_c()
    runs[0] = swap(runs, 0, "c", res)
    assert b10.d10_s0(runs, CONST).passed is False


def test_s0_an_invalid_run_or_a_short_recording_is_a_run_that_failed_to_execute() -> None:
    runs = runs_c()
    runs[0] = swap(runs, 0, "c", result(truth_valid=False, truth_invalid_reason="no_scan_at_start"))
    d = b10.d10_s0(runs, CONST)
    assert d.passed is False
    assert d.items["no run failed to execute"][0] is False
    runs[0] = swap(runs, 0, "c", result(truth_last_odom_s=5.0))
    assert b10.d10_s0(runs, CONST).items["no run failed to execute"][0] is False


def with_trace(res: dict, **changes) -> dict:
    return {**res, "start_trace": good_trace(**changes)}


@pytest.mark.parametrize(
    ("name", "changes", "ok"),
    [
        ("through the gate", {}, True),
        ("driving just after the gate's start", {"driving_first_s": 3.0 + EPS}, True),
        ("driving at the gate's start", {"driving_first_s": 3.0}, False),
        ("driving before the gate's start", {"driving_first_s": 2.5}, False),
        ("no driving report", {"driving_first_s": None}, False),
        ("nan driving report", {"driving_first_s": math.nan}, False),
        ("planner spawned with the start", {"planner_spawn_s": 3.0}, True),
        ("planner spawned before the start", {"planner_spawn_s": 2.9}, False),
        ("no planner spawn time", {"planner_spawn_s": None}, False),
        ("gate verdict wait", {"gate_verdict": "wait"}, False),
        ("gate verdict none (an old run file)", {"gate_verdict": None}, False),
        ("no gate start", {"gate_start_s": None}, False),
        ("nan gate start", {"gate_start_s": math.nan}, False),
    ],
)
def test_s0_item_1_the_start_went_through_the_gate(name: str, changes: dict, ok: bool) -> None:
    runs = runs_c()
    runs[0] = run("c", with_trace(result(), **changes), runs[0].case, runs[0].direction, 81)
    d = b10.d10_s0(runs + r_runs(), CONST)
    first = next(iter(d.items.values()))[0]
    assert first is ok, name
    assert d.passed is ok, name


def test_s0_item_1_a_lap_node_that_started_not_ready_fails() -> None:
    runs = runs_c()
    bad = result(source_ready=False)
    bad["lap_status"]["source_not_ready_reason"] = "no source status yet"
    runs[3] = run("c", bad, runs[3].case, runs[3].direction, 81)
    d = b10.d10_s0(runs + r_runs(), CONST)
    assert next(iter(d.items.values()))[0] is False and d.passed is False
    assert "no source status yet" in next(iter(d.items.values()))[1]


def test_s0_a_refused_c_run_is_a_run_that_failed_to_execute() -> None:
    runs = runs_c()
    refused = result(stopped_because="start_refused")
    refused["start_refusal"] = {"reason": "similarity 0.50 below 0.90", "refused_at_s": 32.0}
    runs[0] = run("c", refused, runs[0].case, runs[0].direction, 81)
    assert "start refused" in runs[0].error and "similarity 0.50" in runs[0].error
    d = b10.d10_s0(runs + r_runs(), CONST)
    assert d.passed is False
    assert d.items["no run failed to execute"][0] is False


def test_s0_the_condition_r_runs_pass_when_the_starter_refused_and_nothing_moved() -> None:
    d = b10.d10_s0(runs_c() + r_runs(), CONST)
    assert d.passed is True, d
    last = list(d.items.values())[-1]
    assert last[0] is True


def test_s0_without_condition_r_runs_item_6_is_not_met() -> None:
    d = b10.d10_s0(runs_c(), CONST)
    assert list(d.items.values())[-1][0] is False and d.passed is False


@pytest.mark.parametrize(
    ("name", "changes", "ok"),
    [
        ("refused within 35 s", {"start_refusal": {"reason": "r", "refused_at_s": 36.0}}, True),
        ("refused at exactly 35 s", {"start_refusal": {"reason": "r", "refused_at_s": 36.0}}, True),
        (
            "refused epsilon over 35 s",
            {"start_refusal": {"reason": "r", "refused_at_s": 36.0 + EPS}},
            False,
        ),
        (
            "refusal time nan",
            {"start_refusal": {"reason": "r", "refused_at_s": math.nan}},
            False,
        ),
        ("never refused", {"stopped_because": "duration"}, False),
        ("the planner was spawned", {"start_trace": good_trace(planner_spawn_s=5.0)}, False),
        ("one message on /drive_raw", {"drive_raw_messages": 1}, False),
        ("no count of /drive_raw messages", {"drive_raw_messages": None}, False),
        ("path at the limit", {"distance_m": 0.05}, True),
        ("path epsilon over", {"distance_m": 0.05 + EPS}, False),
        ("path nan", {"distance_m": math.nan}, False),
        ("no path", {"distance_m": None}, False),
        ("no reason", {"start_refusal": {"reason": "", "refused_at_s": 10.0}}, False),
        ("an error", {"error": "boom"}, False),
    ],
)
def test_s0_item_6_the_refusal_at_its_thresholds(name: str, changes: dict, ok: bool) -> None:
    res = refusal_result(**changes)
    if "start_trace" not in changes:
        res["start_trace"]["gate_reason"] = res["start_refusal"].get("reason", "")
    if name.startswith("refused"):
        res["start_trace"]["lap_node_first_status_s"] = 1.0
    d = b10.d10_s0(runs_c() + [run("r", res)], CONST)
    assert list(d.items.values())[-1][0] is ok, name


def test_s0_the_refusal_must_name_the_gates_reason() -> None:
    res = refusal_result()
    res["start_trace"]["gate_reason"] = "something else"
    d = b10.d10_s0(runs_c() + [run("r", res)], CONST)
    assert list(d.items.values())[-1][0] is False


def test_s1_and_s2_ignore_condition_r_runs() -> None:
    pairs = []
    for i in range(12):
        case, direction, seed = (
            ("V1", "V2", "V3")[i % 3],
            ("ccw", "cw")[(i // 3) % 2],
            (81, 82)[i // 6],
        )
        c, b = result(), result()
        c["gt_lap_times_registered_s"] = [14.0] + [8.0] * 12
        b["gt_lap_times_registered_s"] = [10.0] * 13
        pairs += [run("c", c, case, direction, seed), run("b", b, case, direction, seed)]
    assert b10.d10_s1(pairs + r_runs(), CONST).passed is True


def test_s2_item_7_the_f_runs_started_through_the_gate() -> None:
    good = [run("f", result(FAULTS), "V1", d, s) for d in ("ccw", "cw") for s in (81, 82)]
    d = b10.d10_s2(good, CONST)
    assert list(d.items.values())[-1][0] is True and d.passed is True, d
    bad = result(FAULTS)
    bad["start_trace"]["driving_first_s"] = 2.0
    d = b10.d10_s2([run("f", bad, "V1", "ccw", 81), *good[1:]], CONST)
    assert list(d.items.values())[-1][0] is False and d.passed is False


def test_the_start_trace_rows_and_the_source_health_check() -> None:
    runs = runs_c()[:2] + r_runs(1)
    rows = b10.start_trace_rows(runs)
    assert len(rows) == 3 and set(b10.START_TRACE_KEYS) <= set(rows[0])
    assert rows[2]["refusal_reason"] == "source state 'waiting', not 'running'"
    for r, gap, age in ((runs[0], 2.0, 1.5), (runs[1], 1.0, 0.2)):
        r.extras["lap_status"].update(
            {"source_status_gap_max_s": gap, "source_status_age_at_start_s": age}
        )
    g4 = b10.g4_source_health(runs)
    assert g4["gap_ok"] and g4["age_ok"] and g4["runs"] == 2
    runs[0].extras["lap_status"]["source_status_gap_max_s"] = 2.0 + EPS
    assert b10.g4_source_health(runs)["gap_ok"] is False
    runs[0].extras["lap_status"]["source_status_gap_max_s"] = 2.0
    runs[0].extras["lap_status"]["source_status_age_at_start_s"] = None  # no status at the start
    assert b10.g4_source_health(runs)["age_ok"] is False


def test_the_start_summary_counts_d2_and_g5() -> None:
    ok = {"source_ready": True, "start_trace": good_trace()}
    late = {
        "source_ready": False,
        "start_trace": good_trace(
            gate_start_s=None, gate_verdict=None, oracle_first_status_s=4.5, lap1_start_s=4.0
        ),
    }
    refused = {
        "stopped_because": "start_refused",
        "start_trace": good_trace(gate_verdict="refuse", gate_start_s=None),
    }
    out = b10.start_summary([ok, late, refused])
    assert out["runs"] == 3 and out["started_not_ready"] == 1 and out["refused"] == 1
    assert out["through_gate"] == 1 and out["driving_after_gate"] == 1
    assert out["oracle_after_lap1_start"] == 1
    assert out["oracle_minus_lap1_s"]["max"] == pytest.approx(0.5)
    empty = b10.start_summary([])
    assert empty["runs"] == 0 and empty["oracle_minus_lap1_s"]["median"] is None


def test_s0_without_condition_c_runs_is_refused() -> None:
    with pytest.raises(b10.MissingPairError):
        b10.d10_s0([run("b", result())], CONST)


def test_a_result_without_the_registered_truth_is_refused() -> None:
    res = result()
    del res["gt_lap_times_registered_s"]
    with pytest.raises(b10.RunFileError):
        run("c", res)


# -- D10-S1 ------------------------------------------------------------------------------------------------


def lap_times(first: float, late: float) -> list[float]:
    return [first] + [first - (first - late) * min(1.0, k / 6.0) for k in range(1, 12)] + [late]


def s1_runs(n_c_complete: int = 12, ratio: float = 0.6, c_contacts: int = 0, b_late: float = 10.0):
    runs = []
    for i in range(12):
        case, direction, seed = (
            ("V1", "V2", "V3")[i % 3],
            ("ccw", "cw")[(i // 3) % 2],
            (81, 82)[i // 6],
        )
        for cond in ("b", "c"):
            times = [b_late] * 13 if cond == "b" else lap_times(b_late * 1.4, b_late * ratio)
            res = result()
            res["gt_lap_times_registered_s"] = (
                times if cond == "b" or i < n_c_complete else times[:6]
            )
            res["wall_contacts"] = c_contacts if (cond == "c" and i == 0) else 0
            runs.append(run(cond, res, case, direction, seed))
    return runs


def test_s1_passes_when_the_schedule_learns() -> None:
    d = b10.d10_s1(s1_runs(), CONST)
    assert d.name == "D10-S1" and d.passed is True, d


def test_s1_complete_threshold_is_11_of_12() -> None:
    assert b10.d10_s1(s1_runs(n_c_complete=11), CONST).passed is True
    d = b10.d10_s1(s1_runs(n_c_complete=10), CONST)
    assert d.passed is False


def test_s1_a_contact_fails() -> None:
    assert b10.d10_s1(s1_runs(c_contacts=1), CONST).passed is False


@pytest.mark.parametrize(("ratio", "ok"), [(0.80, True), (0.8001, False)])
def test_s1_median_r_at_most_0_80(ratio: float, ok: bool) -> None:
    runs = s1_runs(ratio=1.0)
    # r = T_late / T1: build c laps with the first lap 1/ratio times the late laps
    out = []
    for r in runs:
        if r.condition == "c":
            times = [10.0 / ratio] + [10.0] * 12
            res = result()
            res["gt_lap_times_registered_s"] = times
            out.append(run("c", res, r.case, r.direction, r.seed))
        else:
            out.append(r)
    d = b10.d10_s1(out, CONST)
    key = next(k for k in d.items if k.startswith("4b"))
    assert d.items[key][0] is ok


def test_s1_after_one_dropped_case_the_threshold_is_7_of_8() -> None:
    runs = [r for r in s1_runs() if r.case != "V2"]
    assert len([r for r in runs if r.condition == "c"]) == 8
    assert b10.d10_s1(runs, CONST).passed is True
    bad = [
        run(
            r.condition,
            {**result(), "gt_lap_times_registered_s": [10.0] * 5},
            r.case,
            r.direction,
            r.seed,
        )
        if r.condition == "c"
        and (r.case, r.direction, r.seed) in (("V1", "ccw", 81), ("V3", "cw", 82))
        else r
        for r in runs
    ]
    assert b10.d10_s1(bad, CONST).passed is False


# -- D10-S2 ------------------------------------------------------------------------------------------------


def runs_f(**kw) -> list[RunSummary]:
    out = []
    for i in range(12):
        case, direction, seed = (
            ("V1", "V2", "V3")[i % 3],
            ("ccw", "cw")[(i // 3) % 2],
            (81, 82)[i // 6],
        )
        out.append(run("f", result(FAULTS, **kw), case, direction, seed))
    return out


def test_s2_passes_when_the_fallback_works() -> None:
    d = b10.d10_s2(runs_f(), CONST)
    assert d.passed is True, d
    assert len(d.items) == 8


def test_s2_a_contact_fails() -> None:
    runs = runs_f()
    runs[0] = swap(runs, 0, "f", result(FAULTS, contacts=1))
    d = b10.d10_s2(runs, CONST)
    assert d.passed is False and d.items["(1) 0 wall contacts"][0] is False


def test_s2_fewer_than_12_truth_laps_fails() -> None:
    runs = runs_f()
    runs[3] = swap(runs, 3, "f", result(FAULTS, laps=11))
    assert b10.d10_s2(runs, CONST).passed is False


def test_s2_a_confirmed_merged_lap_fails() -> None:
    res = result(FAULTS)
    res["lap_status"]["doubtful_laps"] = [
        d for d in res["lap_status"]["doubtful_laps"] if d["lap"] != 4
    ]
    runs = runs_f()
    runs[0] = swap(runs, 0, "f", res)
    d = b10.d10_s2(runs, CONST)
    assert d.passed is False
    assert d.items[next(k for k in d.items if k.startswith("(3)"))][0] is False
    assert d.items[next(k for k in d.items if k.startswith("(4a)"))][0] is False


def test_s2_the_false_event_lap_and_the_next_must_be_doubtful() -> None:
    res = result(FAULTS)
    j = next(i for i, e in enumerate(res["oracle_events"]) if e["kind"] == "false")
    false_t = res["oracle_events"][j]["crossing_time_s"]
    lap = next(
        r["lap_index"] for r in res["system_lap_events"] if abs(r["t_cross_s"] - false_t) < 1e-6
    )
    keep = [d for d in res["lap_status"]["doubtful_laps"] if d["lap"] != lap + 1]
    res["lap_status"]["doubtful_laps"] = keep
    runs = runs_f()
    runs[0] = swap(runs, 0, "f", res)
    d = b10.d10_s2(runs, CONST)
    assert (
        d.passed is False and d.items[next(k for k in d.items if k.startswith("(4a)"))][0] is False
    )


def test_s2_the_duplicate_must_be_rejected() -> None:
    res = result(FAULTS)
    res["lap_status"]["rejected_events"] = {}
    runs = runs_f()
    runs[0] = swap(runs, 0, "f", res)
    d = b10.d10_s2(runs, CONST)
    assert d.items["(4b) the duplicate of crossing 10 is rejected"][0] is False


def test_s2_cap_after_a_doubtful_lap_at_most_that_of_the_doubtful_lap() -> None:
    res = result(FAULTS)
    j = next(d["lap"] for d in res["lap_status"]["doubtful_laps"])  # the first doubtful lap
    res["system_lap_events"][j]["cap_mps"] = res["system_lap_events"][j - 1]["cap_mps"] + 0.1
    runs = runs_f()
    runs[0] = swap(runs, 0, "f", res)
    d = b10.d10_s2(runs, CONST)
    assert (
        d.passed is False and d.items[next(k for k in d.items if k.startswith("(5)"))][0] is False
    )
    res["system_lap_events"][j]["cap_mps"] = res["system_lap_events"][j - 1]["cap_mps"]  # equal: ok
    runs[0] = swap(runs, 0, "f", res)
    assert d_item(b10.d10_s2(runs, CONST), "(5)") is True


def d_item(d, prefix: str) -> bool:
    return d.items[next(k for k in d.items if k.startswith(prefix))][0]


def test_s2_overdue_window_needs_the_lap_one_cap() -> None:
    res = result(FAULTS)
    res["guidance_cap_series"] = [[0.0, 0.5], [1.0, 0.7]]  # never dropped to the lap-1 cap
    runs = runs_f()
    runs[0] = swap(runs, 0, "f", res)
    d = b10.d10_s2(runs, CONST)
    assert d.passed is False and d_item(d, "(6)") is False
    res = result(FAULTS)
    res["guidance_cap_series"] = []
    runs[0] = swap(runs, 0, "f", res)
    assert d_item(b10.d10_s2(runs, CONST), "(6)") is False  # no series: not shown


@pytest.mark.parametrize(("cap", "ok"), [(0.5, True), (0.5 + 1e-7, True), (0.5 + 1e-3, False)])
def test_s2_the_cap_in_the_window_at_the_threshold(cap: float, ok: bool) -> None:
    res = result(FAULTS)
    c = res["gt_lap_crossings_registered_s"]
    for d in (4, 7):
        t0 = c[d - 1] + 0.5 * (c[d] - c[d - 1])
        res["guidance_cap_series"] = [
            s for s in res["guidance_cap_series"] if not (t0 - 2 < s[0] < c[d])
        ]
        res["guidance_cap_series"] += [[t0 - 1.0, 0.9], [t0, cap]]
    res["guidance_cap_series"].sort()
    runs = runs_f()
    runs[0] = swap(runs, 0, "f", res)
    assert d_item(b10.d10_s2(runs, CONST), "(6)") is ok


def test_s2_nan_in_the_series_is_not_a_pass() -> None:
    assert math.isnan(b10.cap_series_min([], 0.0, 1.0))
    assert math.isnan(b10.cap_series_min([(5.0, 0.5)], 0.0, 1.0))
    assert b10.cap_series_min([(0.0, 0.9), (0.5, 0.5), (2.0, 0.3)], 0.2, 1.0) == 0.5
    assert b10.cap_series_min([(0.0, 0.9), (0.5, 0.5)], 0.7, 1.0) == 0.5
    assert b10.cap_series_min([(0.0, 0.9), (0.5, 0.6)], 0.2, 0.4) == 0.9


def test_s2_without_condition_f_is_refused() -> None:
    with pytest.raises(b10.MissingPairError):
        b10.d10_s2([run("c", result())], CONST)


def test_s2_a_system_lap_that_matches_no_truth_crossing_is_not_confirmed() -> None:
    res = result(FAULTS)
    res["system_lap_events"][1]["t_cross_s"] += 2.0
    runs = runs_f()
    runs[0] = swap(runs, 0, "f", res)
    d = b10.d10_s2(runs, CONST)
    assert d_item(d, "(3)") is False


# -- the drop rule ------------------------------------------------------------------------------------------


def all_laps(n: int = 3) -> dict[str, int]:
    return {f"{c}_{d}": n for c in ("V1", "V2", "V3") for d in ("ccw", "cw")}


def test_the_drop_rule() -> None:
    d = b10.drop_rule(all_laps(), CONST)
    assert d.decided and d.dropped == () and d.runs_per_condition == 12
    laps = all_laps()
    laps["V2_cw"] = 2  # one direction under 3 laps drops the case
    d = b10.drop_rule(laps, CONST)
    assert d.decided and d.dropped == ("V2",) and d.runs_per_condition == 8
    laps = all_laps()
    laps["V2_cw"] = 2
    laps["V3_ccw"] = 0
    d = b10.drop_rule(laps, CONST)
    assert not d.decided and "2 cases would be dropped" in d.reason and d.runs_per_condition == 0
    laps = all_laps()
    del laps["V1_ccw"]  # a missing run is a run that failed to execute: not 3 laps
    assert b10.drop_rule(laps, CONST).dropped == ("V1",)
    assert b10.drop_rule(all_laps(3), CONST).decided
    assert b10.drop_rule({**all_laps(), "V1_ccw": 3}, CONST).dropped == ()  # exactly 3 laps stays


def test_with_fewer_than_two_cases_left_nothing_is_decided() -> None:
    import dataclasses

    raw = dict(CONST.raw)
    raw["drop_rule"] = {**CONST.raw["drop_rule"], "max_dropped": 2}
    const2 = dataclasses.replace(CONST, raw=raw)
    laps = all_laps()
    laps["V2_cw"] = 0
    laps["V3_cw"] = 0
    d = b10.drop_rule(laps, const2)
    assert not d.decided and "cases left" in d.reason


# -- the reported items -------------------------------------------------------------------------------------


def test_late_lap_rows_and_run_details_are_reported() -> None:
    runs = [run("c", result(), "V1", "ccw", 81), run("f", result(FAULTS), "V1", "ccw", 81)]
    rows = b10.late_lap_rows(runs)
    assert len(rows) == 1 and rows[0]["ratio"] == pytest.approx(1.0)
    assert math.isfinite(b10.median_t_late_ratio(runs))
    details = b10.run_detail_rows(runs)
    assert {d["run"] for d in details} == {"c_V1_ccw_81", "f_V1_ccw_81"}
    assert sorted(next(d for d in details if d["run"].startswith("f_"))["doubtful"]) == sorted(
        b10.doubtful_laps(runs[1])
    )
    assert math.isnan(b10.median_t_late_ratio([runs[0]]))


def test_load_runs_reads_the_meta_block(tmp_path: Path) -> None:
    res = result()
    (tmp_path / "c_V1_ccw_81.json").write_text(
        json.dumps({**res, "battery_b10": {"mode": "battery"}})
    )
    (tmp_path / "notes.json").write_text("{}")
    loaded = b10.load_runs(tmp_path, CONST)
    assert [r.condition for r in loaded] == ["c"]
    (tmp_path / "b_V1_ccw_81.json").write_text(json.dumps(res))
    with pytest.raises(b10.RunFileError, match="not a B10 battery run file"):
        b10.load_runs(tmp_path, CONST)


# -- the freeze guard and the commands -------------------------------------------------------------------------


def write_freeze(path: Path, sha: str = b10.REGISTERED_YAML_SHA256, commit: str = "a" * 40) -> None:
    path.write_text(json.dumps({"freeze_commit": commit, "yaml_sha256": sha}))


def test_no_v_case_and_no_seed_81_or_82_without_the_freeze_file(tmp_path: Path) -> None:
    freeze = tmp_path / "b10.json"
    cases = bat.dev_cases(CONST)
    for case, seed in (("V1", 81), ("V2", 82), ("V3", 81), ("P1", 81), ("V1", 11), ("P1", 82)):
        with pytest.raises(bat.BatteryError):
            bat.command_for(
                CONST,
                "c",
                case,
                "ccw",
                seed,
                80,
                tmp_path,
                tmp_path / "o.json",
                Path("config/pnp/fixtures/dyn-car-2026-11-12"),
                tmp_path,
                freeze_path=freeze,
            )
    assert "P1" in cases and "V1" not in cases
    write_freeze(freeze, sha="0" * 64)
    with pytest.raises(bat.BatteryError, match="names yaml hash"):
        bat.read_freeze(freeze)
    freeze.write_text("{")
    with pytest.raises(bat.BatteryError, match="not valid JSON"):
        bat.read_freeze(freeze)
    freeze.write_text(
        json.dumps({"freeze_commit": "xyz", "yaml_sha256": b10.REGISTERED_YAML_SHA256})
    )
    with pytest.raises(bat.BatteryError, match="40 digit"):
        bat.read_freeze(freeze)
    write_freeze(freeze)
    assert bat.read_freeze(freeze)["freeze_commit"] == "a" * 40


@pytest.mark.parametrize(
    ("case", "seed"),
    [("U1", 71), ("U2", 72), ("U3", 11), ("P1", 71), ("P1", 72), ("V1", 71), ("U1", 81)],
)
def test_the_spent_cases_and_seeds_are_refused_even_with_the_freeze_file(
    tmp_path: Path, case: str, seed: int
) -> None:
    freeze = tmp_path / "b10.json"
    write_freeze(freeze)
    with pytest.raises(bat.BatteryError, match="spent"):
        bat.command_for(
            CONST,
            "c",
            case,
            "ccw",
            seed,
            80,
            tmp_path,
            tmp_path / "o.json",
            Path("config/pnp/fixtures/dyn-car-2026-11-12"),
            tmp_path,
            freeze_path=freeze,
            verify_git=False,
        )
    with pytest.raises(bat.BatteryError, match="spent"):
        bat.needs_freeze(CONST, case, seed)


def test_the_two_evaluation_bags_are_refused_always(tmp_path: Path) -> None:
    for name in bat.EVALUATION_BAGS:
        with pytest.raises(bat.BatteryError, match="never opened again"):
            bat.guard_bag(name, tmp_path / "b10.json")
    write_freeze(tmp_path / "b10.json")  # a freeze file changes nothing
    for name in bat.EVALUATION_BAGS:
        with pytest.raises(bat.BatteryError, match="never opened again"):
            bat.guard_bag(name, tmp_path / "b10.json", verify_git=False)
    bat.guard_bag("2026-11-16T22-38-47_car_teleop", tmp_path / "nothing.json")  # development


def test_b9_run_files_are_never_opened(tmp_path: Path) -> None:
    with pytest.raises(bat.BatteryError, match="B9"):
        bat.guard_run_dir(bat.SPENT_RUNS)
    with pytest.raises(bat.BatteryError, match="B9"):
        bat.guard_run_dir(bat.SPENT_RUNS / "attempts")
    with pytest.raises(bat.BatteryError, match="B9"):
        bat.report_cmd(__import__("argparse").Namespace(out=str(bat.SPENT_RUNS)))
    with pytest.raises(bat.BatteryError, match="B9"):
        bat.start_report_cmd(__import__("argparse").Namespace(dirs=[str(bat.SPENT_RUNS)]))
    bat.guard_run_dir(tmp_path)  # anything else is fine


def test_development_runs_are_the_development_cases_on_the_development_seeds_only(
    tmp_path: Path,
) -> None:
    assert bat.needs_freeze(CONST, "V1", 81) is True
    assert bat.needs_freeze(CONST, "P1", 11) is False
    for case, seed in (("P1", 21), ("P1", 31), ("P1", 61), ("P1", 51), ("P1", 41), ("Z9", 11)):
        with pytest.raises(bat.BatteryError):
            bat.needs_freeze(CONST, case, seed)


def test_the_command_has_the_registered_flags(tmp_path: Path) -> None:
    freeze = tmp_path / "b10.json"
    write_freeze(freeze)
    bundle = tmp_path / "dyn"
    bundle.mkdir()
    (bundle / "manifest.yaml").write_text(
        "io:\n  validity: {v_min_mps: 0.0, v_max_mps: 1.3}\n"
        "  nominal: {speed_deadband_mps: 0.22, steer_tau_s: 0.03, command_delay_s: 0.0}\n"
    )
    cmd = bat.command_for(
        CONST,
        "f",
        "V1",
        "cw",
        82,
        80,
        tmp_path / "w",
        tmp_path / "o.json",
        bundle,
        tmp_path,
        freeze_path=freeze,
        verify_git=False,
    )
    flat = " ".join(cmd)
    assert "--stop-after-laps 13" in flat and "--reverse" in cmd
    assert "--lap-source truth" in flat and "--sim-time" in cmd and "--seed 82" in flat
    assert "--lap-memory v0" in flat and "--v-lap1-mps 0.5" in flat and "--v-max-mps 1.3" in flat
    assert "--lap-truth-ready-after-s 6.0" in flat  # seed 82: six seconds
    faults = json.loads(cmd[cmd.index("--lap-truth-faults-json") + 1])
    assert faults == CONST.fault_plan
    a = bat.command_for(
        CONST,
        "a",
        "V1",
        "ccw",
        81,
        80,
        tmp_path / "w",
        tmp_path / "o.json",
        bundle,
        tmp_path,
        freeze_path=freeze,
        verify_git=False,
    )
    assert "--lap-memory off" in " ".join(a) and "--v-max-mps 0.5" in " ".join(a)
    assert "--lap-truth-faults-json" not in a
    b = bat.command_for(
        CONST,
        "b",
        "V2",
        "ccw",
        82,
        80,
        tmp_path / "w",
        tmp_path / "o.json",
        bundle,
        tmp_path,
        freeze_path=freeze,
        verify_git=False,
    )
    assert "--v-max-mps 1.3" in " ".join(b) and "--lap-source" not in b
    c = bat.command_for(
        CONST,
        "c",
        "V3",
        "ccw",
        82,
        80,
        tmp_path / "w",
        tmp_path / "o.json",
        bundle,
        tmp_path,
        freeze_path=freeze,
        verify_git=False,
    )
    assert "--lap-truth-faults-json" not in c and "--lap-source truth" in " ".join(c)
    assert "--lap-truth-ready-after-s 6.0" in " ".join(c)
    c81 = bat.command_for(
        CONST,
        "c",
        "V3",
        "cw",
        81,
        80,
        tmp_path / "w",
        tmp_path / "o.json",
        bundle,
        tmp_path,
        freeze_path=freeze,
        verify_git=False,
    )
    assert "--lap-truth-ready-after-s 0.0" in " ".join(c81)
    r = bat.command_for(
        CONST,
        "r",
        "V1",
        "ccw",
        81,
        80,
        tmp_path / "w",
        tmp_path / "o.json",
        bundle,
        tmp_path,
        freeze_path=freeze,
        verify_git=False,
    )
    flat_r = " ".join(r)
    assert "--lap-truth-ready-after-s inf" in flat_r and "--lap-source truth" in flat_r
    assert "--lap-truth-faults-json" not in r
    assert "--lap-truth-ready-after-s" not in " ".join(a)  # no oracle in condition a
    assert "--lap-truth-ready-after-s" not in " ".join(b)
    dev = bat.command_for(
        CONST,
        "c",
        "P1",
        "ccw",
        12,
        80,
        tmp_path / "w",
        tmp_path / "o.json",
        bundle,
        tmp_path,
        freeze_path=tmp_path / "nothing.json",
        verify_git=False,
    )
    assert "--lap-truth-ready-after-s 6.0" in " ".join(dev)  # development seed 12: six seconds
    dev11 = bat.command_for(
        CONST, "c", "P1", "ccw", 11, 80, tmp_path / "w", tmp_path / "o.json", bundle, tmp_path,
        freeze_path=tmp_path / "nothing.json", verify_git=False, ready_after_s=6.0,
    )  # fmt: skip
    assert "--lap-truth-ready-after-s 6.0" in " ".join(dev11)
    with pytest.raises(bat.BatteryError):
        bat.command_for(
            CONST,
            "c",
            "V3",
            "sideways",
            82,
            80,
            tmp_path / "w",
            tmp_path / "o.json",
            bundle,
            tmp_path,
            freeze_path=freeze,
            verify_git=False,
        )
    with pytest.raises(bat.BatteryError):
        bat.command_for(
            CONST,
            "z",
            "V3",
            "cw",
            82,
            80,
            tmp_path / "w",
            tmp_path / "o.json",
            bundle,
            tmp_path,
            freeze_path=freeze,
            verify_git=False,
        )


def test_starved_invalid_and_short_runs_are_rerun() -> None:
    ok = result()
    assert bat.needs_rerun(ok) is False
    assert bat.needs_rerun({**ok, "truth_valid": False}) is True
    assert bat.needs_rerun({**ok, "drive_raw": {"max_gap_s": 0.05}}) is True
    assert bat.needs_rerun({**ok, "real_time_factor": 0.79}) is True
    assert bat.needs_rerun({**ok, "truth_last_odom_s": 3.0}) is True
    assert bat.needs_rerun({"error": "boom"}) is True


def test_a_refused_run_is_never_repeated_and_a_condition_r_run_has_no_truth_to_be_invalid() -> None:
    refused = result(stopped_because="start_refused", truth_valid=False)
    assert bat.needs_rerun(refused, "c") is False and bat.needs_rerun(refused, "f") is False
    r_ok = refusal_result()
    assert bat.needs_rerun(r_ok, "r") is False
    assert bat.needs_rerun({"error": "runner died"}, "r") is True  # no result at all: rerun
    assert bat.needs_rerun({**r_ok, "drive_raw": {"max_gap_s": 0.1}, "stopped_because": ""}, "r")


def test_a_registered_file_cannot_be_edited_and_the_hash_constant_is_the_files() -> None:
    assert b10.sha256_file(b10.PREREG_YAML) == b10.REGISTERED_YAML_SHA256
    doc = copy.deepcopy(CONST.doc)
    doc["lap_battery_b10"]["seeds"] = [81, 73]
    with pytest.raises(b10.PreregError):
        b10._check_consistent(doc)


# -- the run machinery and the reports, on synthetic result files ---------------------------------------------------


def write_runs(out: Path, mode: str = "battery", dev: bool = False) -> None:
    cases = ("P1", "P2", "P4") if dev else ("V1", "V2", "V3")
    seeds = (11,) if dev else (81, 82)
    for cond in ("b", "c", "f"):
        if dev and cond == "f":
            names = [(c, d, s) for c in ("P1", "P4") for d in ("ccw", "cw") for s in seeds]
        else:
            names = [(c, d, s) for c in cases for d in ("ccw", "cw") for s in seeds]
        for case, direction, seed in names:
            if cond == "b":
                res = result()
                res["gt_lap_times_registered_s"] = [10.0] * 13
            elif cond == "c":
                res = result()
                res["gt_lap_times_registered_s"] = [14.0] + [8.0] * 12
            else:
                res = result(FAULTS)
            res["battery_b10"] = {"mode": mode}
            (out / f"{cond}_{case}_{direction}_{seed}.json").write_text(json.dumps(res))
    r_cases = ("P1", "P4") if dev else cases
    for case in r_cases:
        res = refusal_result()
        res["battery_b10"] = {"mode": mode}
        (out / f"r_{case}_ccw_{seeds[0]}.json").write_text(json.dumps(res))


def test_the_report_decides_three_gates_from_the_run_files(tmp_path: Path) -> None:
    write_runs(tmp_path)
    (tmp_path / "drops.json").write_text(json.dumps({"dropped": [], "decided": True}))
    text = bat.report_text(tmp_path)
    assert "### D10-S0: PASS" in text and "### D10-S1: PASS" in text and "### D10-S2: PASS" in text
    assert "Late lap time" in text and "reference length" in text
    assert "The start of every run" in text and "refuse" in text
    assert "Source health over" in text
    assert b10.REGISTERED_YAML_SHA256 in text


def test_the_report_says_insufficient_when_a_condition_has_no_runs(tmp_path: Path) -> None:
    write_runs(tmp_path)
    for p in tmp_path.glob("f_*.json"):
        p.unlink()
    text = bat.report_text(tmp_path)
    assert "### D10-S2: INSUFFICIENT" in text and "### D10-S0: PASS" in text


def test_the_development_report_has_e4_and_e5(tmp_path: Path) -> None:
    write_runs(tmp_path, mode="development", dev=True)
    text = bat.dev_report_text(tmp_path)
    assert "E4" in text and "E5" in text and "### D10-S1: PASS" in text
    assert "every crossing within 0.10 s: True" in text
    rows = bat.oracle_rows(b10.load_runs(tmp_path, CONST, mode="development"))
    assert all(r["worst_s"] == 0.0 for r in rows if r["run"].startswith("c_"))


def test_the_drop_decision_reads_condition_a_seed_81(tmp_path: Path) -> None:
    for case in ("V1", "V2", "V3"):
        for d in ("ccw", "cw"):
            res = result(laps=3 if not (case == "V2" and d == "cw") else 2)
            (tmp_path / f"a_{case}_{d}_81.json").write_text(json.dumps(res))
    drop = bat.drop_decision(tmp_path, CONST)
    assert drop.decided and drop.dropped == ("V2",)
    assert bat.drop_to_json(drop)["dropped"] == ["V2"]
    (tmp_path / "a_V1_ccw_81.json").unlink()
    with pytest.raises(bat.BatteryError, match="has not run"):
        bat.drop_decision(tmp_path, CONST)


def test_pending_work_writes_each_accepted_result_once(tmp_path: Path) -> None:
    job = ("c_P1_ccw_11", "c", "P1", "ccw", 11)
    assert bat.pending_work(tmp_path, [job]) == [(job, 1)]
    good = {**result(), bat.META: {"starved": False, "invalid_truth": False}}
    bat.b4t.write_once(bat.b4t.attempt_path(tmp_path, job[0], 1), json.dumps(good))
    assert bat.pending_work(tmp_path, [job]) == []
    accepted = json.loads((tmp_path / "c_P1_ccw_11.json").read_text())
    assert accepted[bat.META]["accepted_attempt"] == 1
    # a starved first attempt is rerun, at most three attempts in all
    job2 = ("c_P1_cw_11", "c", "P1", "cw", 11)
    starved = {
        **result(),
        "drive_raw": {"max_gap_s": 0.1},
        bat.META: {"starved": True, "invalid_truth": False},
    }
    bat.b4t.write_once(bat.b4t.attempt_path(tmp_path, job2[0], 1), json.dumps(starved))
    assert bat.pending_work(tmp_path, [job2]) == [(job2, 2)]
    for k in (2, 3):
        bat.b4t.write_once(bat.b4t.attempt_path(tmp_path, job2[0], k), json.dumps(starved))
    assert bat.pending_work(tmp_path, [job2]) == []
    final = json.loads((tmp_path / "c_P1_cw_11.json").read_text())
    assert final[bat.META]["accepted_while_starved"] is True
    assert len(final[bat.META]["attempts"]) == 3


def test_a_dev_run_of_a_battery_case_is_refused(tmp_path: Path) -> None:
    import argparse

    args = argparse.Namespace(
        out=str(tmp_path),
        workers=1,
        cases="V1",
        seeds="11",
        conditions="c",
        domain_base=120,
        bundle="x",
        data_root="y",
        freeze_path=str(tmp_path / "b10.json"),
        laps=None,
    )
    with pytest.raises(bat.BatteryError, match="B10 case runs"):
        bat.dev_run_cmd(args)
    args.cases, args.seeds = "P1", "81"
    with pytest.raises(bat.BatteryError):
        bat.dev_run_cmd(args)


def test_the_main_refuses_before_the_freeze(tmp_path: Path, capsys) -> None:
    code = bat.main(["check-freeze", "--freeze-path", str(tmp_path / "b10.json")])
    assert code == 2 and "REFUSED" in capsys.readouterr().err
    code = bat.main(
        ["run", "--out", str(tmp_path / "o"), "--freeze-path", str(tmp_path / "b10.json")]
    )
    assert code == 2
