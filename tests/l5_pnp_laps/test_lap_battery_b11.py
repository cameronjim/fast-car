"""L1: battery B11 (brief 53). The constants under their hashes; D11-S0 and D11-S1 equal D10-S0 and
D10-S1 on the same inputs; every D11-S2, D11-S3 and D11-S4 item at its pass, at the threshold and
over; the outcome sentences; the candidate rule (eligible, the tie order, the 0.97 stop); the drop
rule; and the guard of `battery_b11`: no W case and no seed 91 or 92 without the freeze file, the
seeds of other attempts refused, the conditions' switches.

The runs reuse the synthetic results of `test_lap_battery_b10` (a stadium lapped in 10.6 s, the
oracle publishing every crossing) with the B11 fields added."""

from __future__ import annotations

import copy
import dataclasses
import json
import math
from pathlib import Path

import battery_b11 as bat
import lap_battery_b10 as b10
import lap_battery_b11 as b11
import pytest
import test_lap_battery_b10 as t10

CONST = b11.load_constants()
CASES = ("W1", "W2", "W3", "W4")
KEYS = [(c, d, s) for c in CASES for d in ("ccw", "cw") for s in (91, 92)]


def res(lap_times: list[float] | None = None, **over) -> dict:
    """A t10 result with the B11 fields: per lap clearance 0.2 m, speed commands within the caps,
    no stop, no engagement, a bound per lap."""
    out = t10.result()
    n = len(out["gt_lap_times_registered_s"])
    if lap_times is not None:
        out["gt_lap_times_registered_s"] = list(lap_times)
    out.update(
        {
            "engage_per_lap": [{} for _ in range(n)],
            "min_body_clearance_by_lap_m": [0.2] * n,
            "max_cmd_speed_by_lap_mps": [0.5] + [1.0] * (n - 1),
            "max_cmd_speed_mps": 1.0,
            "longest_stop_s": 0.0,
            "bound_by_lap": {},
            "lap_params": {"v_cap_ceiling_mps": 1.3},
        }
    )
    out.update(over)
    return out


def run(cond: str, r: dict, key) -> b11.RunSummary:
    return b11.summarise_run(r, cond, key[0], key[1], key[2], CONST)


def times(late: float, early: float = 12.0) -> list[float]:
    """13 lap times: lap 1 20 s, laps 2 to 7 `early`, laps 8 to 13 `late`."""
    return [20.0] + [early] * 6 + [late] * 6


def battery(k_late: float = 10.0, c_late: float = 10.0, k_early: float = 12.0, **k_over) -> list:
    runs = []
    for key in KEYS:
        runs.append(run("b", res(times(10.0, 10.0)), key))
        runs.append(run("c", res(times(c_late)), key))
        runs.append(run("k", res(times(k_late, k_early), **k_over), key))
    return runs


def item(d, prefix: str) -> bool:
    return next(ok for name, (ok, _) in d.items.items() if name.startswith(prefix))


# -- constants ---------------------------------------------------------------------------------------


def test_the_constants_are_the_registered_ones(tmp_path: Path) -> None:
    assert CONST.yaml_sha256 == b11.REGISTERED_YAML_SHA256 == b11.sha256_file(b11.PREREG_YAML)
    assert CONST.addendum_sha256 == b11.ADDENDUM_SHA256 == b11.sha256_file(b11.ADDENDUM_YAML)
    assert CONST.battery_seeds == (91, 92) and set(CONST.battery_cases) == set(CASES)
    assert CONST.laps_judged == 12 and CONST.laps_driven == 13 and CONST.late_laps == (8, 9, 10)
    assert CONST.required_complete(16) == 15 and CONST.required_complete(12) == 11
    with pytest.raises(b11.PreregError):
        CONST.required_complete(8)
    assert CONST.th["s2_max_median_ratio_to_c"] == 0.95
    assert CONST.th["s2_max_brakes_per_lap_late"] == 0.2
    assert CONST.s4["median_paired_sum_ratio_k_over_c_max"] == 0.95
    assert CONST.s4["worst_paired_sum_ratio_k_over_c_max"] == 1.03
    assert CONST.fault_plan == b10.load_constants().fault_plan
    bad = tmp_path / "x.yaml"
    bad.write_text(b11.PREREG_YAML.read_text() + "\n# edited\n")
    with pytest.raises(b11.PreregError, match="SHA-256"):
        b11.load_constants(bad)
    bad_a = tmp_path / "a.yaml"
    bad_a.write_text(b11.ADDENDUM_YAML.read_text().replace("0.95", "0.96"))
    with pytest.raises(b11.PreregError, match="SHA-256"):
        b11.load_constants(addendum_path=bad_a)


def test_the_yaml_blocks_are_checked() -> None:
    doc = copy.deepcopy(CONST.doc)
    doc["development"]["candidate_rule"]["tie_order"] = ["K2", "K1", "K3"]
    with pytest.raises(b11.PreregError, match="tie_order"):
        b11._check_consistent(doc, CONST.addendum)
    add = copy.deepcopy(CONST.addendum)
    add["v06_laps_dv1_sha256"] = "0" * 64
    with pytest.raises(b11.PreregError):
        b11._check_consistent(CONST.doc, add)


def test_the_battery_has_88_runs_and_condition_a_runs_seed_91_only() -> None:
    jobs = bat.b11_jobs(CONST)
    assert len(jobs) == 88
    assert {j[4] for j in jobs if j[1] == "a"} == {91}
    assert sum(1 for j in jobs if j[1] == "kf") == 16
    assert len(bat.b11_jobs(CONST, ("W2",))) == 66


# -- S0 and S1 equal the D10 functions --------------------------------------------------------------


def test_s0_equals_d10_s0_items_1_to_5_on_the_same_inputs() -> None:
    runs = battery()
    s0 = b11.d11_s0(runs, CONST)
    for cond in ("c", "k"):
        rs = b11.as_condition([r for r in runs if r.condition == cond], "c")
        d10 = b10.d10_s0(rs, CONST.b10)
        want = list(d10.items.values())[:5]
        got = [v for name, v in s0.items.items() if name.startswith(f"{cond}: ")]
        assert got == want
    assert s0.passed


def test_s0_fails_when_a_k_run_has_a_doubtful_lap() -> None:
    runs = battery()
    i = next(j for j, r in enumerate(runs) if r.condition == "k")
    bad = res(times(10.0))
    bad["lap_status"]["doubtful_laps"] = [{"lap": 3, "reasons": ["length"]}]
    runs[i] = run("k", bad, runs[i].key)
    assert not b11.d11_s0(runs, CONST).passed


def test_s1_equals_d10_s1_with_b11s_counts() -> None:
    runs = battery()
    s1 = b11.d11_s1(runs, CONST)
    assert s1.passed
    import lap_battery_b4 as b4

    d3 = b4.d3_s1([r for r in runs if r.condition in ("b", "c")], CONST)
    assert s1.items == d3.items
    # 14 of 16 complete fails, 15 passes
    for n_short, ok in ((1, True), (2, False)):
        rs = battery()
        idx = [j for j, r in enumerate(rs) if r.condition == "c"][:n_short]
        for j in idx:
            rs[j] = run("c", res(times(10.0)[:9]), rs[j].key)
        assert item(b11.d11_s1(rs, CONST), "2.") is ok


# -- S2 -------------------------------------------------------------------------------------------------


def test_s2_passes_when_k_is_faster_and_as_safe() -> None:
    d = b11.d11_s2(battery(k_late=9.0), CONST)
    assert d.passed, d


@pytest.mark.parametrize(("k_late", "ok"), [(9.5, True), (9.5001, False), (9.0, True)])
def test_s2_item_3_median_ratio_at_most_0_95(k_late: float, ok: bool) -> None:
    assert item(b11.d11_s2(battery(k_late=k_late), CONST), "3.") is ok


@pytest.mark.parametrize(("q", "ok"), [(1.03, True), (1.0301, False)])
def test_s2_item_4_no_pair_above_1_03(q: float, ok: bool) -> None:
    runs = battery(k_late=9.0)
    i = next(j for j, r in enumerate(runs) if r.condition == "k")
    runs[i] = run("k", res(times(10.0 * q)), runs[i].key)
    assert item(b11.d11_s2(runs, CONST), "4.") is ok


def test_s2_item_1_a_contact_fails() -> None:
    runs = battery(k_late=9.0)
    i = next(j for j, r in enumerate(runs) if r.condition == "k")
    runs[i] = run("k", res(times(9.0), wall_contacts=1), runs[i].key)
    d = b11.d11_s2(runs, CONST)
    assert not item(d, "1.")
    assert b11.outcome(b11.d11_s0(runs, CONST), b11.d11_s1(runs, CONST), d, _s3_pass(), None) == (
        "unsafe in sim"
    )


def _short(runs, cond: str, n: int) -> list:
    idx = [j for j, r in enumerate(runs) if r.condition == cond][:n]
    for j in idx:
        runs[j] = run(cond, res(times(9.0)[:9]), runs[j].key)
    return runs


def test_s2_item_2_completion_not_fewer_than_c_and_stalls() -> None:
    # 15 of 16 k complete, every c complete: fewer than the c runs
    assert not item(b11.d11_s2(_short(battery(k_late=9.0), "k", 1), CONST), "2.")
    # 15 of 16 each: passes
    rs = _short(_short(battery(k_late=9.0), "k", 1), "c", 1)
    assert item(b11.d11_s2(rs, CONST), "2.")
    # 14 of 16 k: fails even with 14 c
    rs = _short(_short(battery(k_late=9.0), "k", 2), "c", 2)
    assert not item(b11.d11_s2(rs, CONST), "2.")
    # a stall over 5 s fails, exactly 5 s passes, no measure fails
    for stop, ok in ((5.0, True), (5.01, False), (None, False)):
        rs = battery(k_late=9.0, longest_stop_s=stop)
        assert item(b11.d11_s2(rs, CONST), "2.") is ok


@pytest.mark.parametrize(("n_runs", "ok"), [(9, True), (10, False)])
def test_s2_item_5_brakes_at_most_0_2(n_runs: int, ok: bool) -> None:
    """One TTC brake on lap 8 in `n_runs` of the 16 k runs: mean n / 16 / 3 (0.1875, 0.208)."""
    rs = []
    for j, key in enumerate(KEYS):
        rs.append(run("b", res(times(10.0, 10.0)), key))
        rs.append(run("c", res(times(10.0)), key))
        bl = [{} for _ in range(13)]
        if j < n_runs:
            bl[7] = {"ttc": 1}
        rs.append(run("k", res(times(9.0), brake_per_lap=bl), key))
    assert item(b11.d11_s2(rs, CONST), "5.") is ok


def test_s2_item_6_engagements_k_at_most_c() -> None:
    eng = [{} for _ in range(13)]
    eng_one = copy.deepcopy(eng)
    eng_one[1] = {"ttc": 1}
    eng_lap1 = copy.deepcopy(eng)
    eng_lap1[0] = {"watchdog": 3}  # lap 1 does not count
    for k_eng, c_eng, ok in (
        (eng_one, eng_one, True),
        (eng_one, eng, False),
        (eng_lap1, eng, True),
    ):
        rs = []
        for key in KEYS:
            rs.append(run("b", res(times(10.0, 10.0)), key))
            rs.append(run("c", res(times(10.0), engage_per_lap=c_eng), key))
            rs.append(run("k", res(times(9.0), engage_per_lap=k_eng), key))
        assert item(b11.d11_s2(rs, CONST), "6.") is ok


@pytest.mark.parametrize(("k_cl", "ok"), [(0.17, True), (0.1699, False), (0.25, True)])
def test_s2_item_7_clearance_k_at_least_c_minus_0_03(k_cl: float, ok: bool) -> None:
    rs = []
    for key in KEYS:
        rs.append(run("b", res(times(10.0, 10.0)), key))
        rs.append(run("c", res(times(10.0), min_body_clearance_by_lap_m=[0.2] * 13), key))
        cl = [0.3] * 13
        cl[8] = k_cl  # lap 9
        rs.append(run("k", res(times(9.0), min_body_clearance_by_lap_m=cl), key))
    assert item(b11.d11_s2(rs, CONST), "7.") is ok


@pytest.mark.parametrize(
    ("lap1", "top", "ok"),
    [(0.51, 1.31, True), (0.5101, 1.0, False), (0.5, 1.3101, False), (0.5, 1.3, True)],
)
def test_s2_item_8_speed_commands(lap1: float, top: float, ok: bool) -> None:
    rs = battery(
        k_late=9.0,
        max_cmd_speed_by_lap_mps=[lap1] + [1.0] * 12,
        max_cmd_speed_mps=top,
    )
    assert item(b11.d11_s2(rs, CONST), "8.") is ok


def test_s2_item_8_needs_the_measure() -> None:
    rs = battery(k_late=9.0, max_cmd_speed_mps=None)
    assert not item(b11.d11_s2(rs, CONST), "8.")


# -- outcomes -------------------------------------------------------------------------------------------


def _s3_pass() -> b11.Decision:
    return b11.Decision("D11-S3", True, {"x": (True, "")})


def test_outcome_sentences() -> None:
    runs = battery(k_late=9.0)
    s0, s1 = b11.d11_s0(runs, CONST), b11.d11_s1(runs, CONST)
    s2 = b11.d11_s2(runs, CONST)
    assert b11.outcome(s0, s1, s2, _s3_pass(), False) == (
        "D v1 faster than the schedule in sim at equal safety"
    )
    assert b11.outcome(s0, s1, s2, _s3_pass(), True).endswith("gain not shown to need memory")
    slow = battery(k_late=9.9)
    s2s = b11.d11_s2(slow, CONST)
    assert b11.outcome(s0, s1, s2s, _s3_pass(), False) == "no better than the schedule in sim"
    s3f = b11.Decision("D11-S3", False, {"x": (False, "")})
    assert b11.outcome(s0, s1, s2, s3f, False) == "unsafe in sim"
    s1f = dataclasses.replace(s1, passed=False)
    assert b11.outcome(s0, s1f, s2, _s3_pass(), False).startswith("no result")
    s0f = dataclasses.replace(s0, passed=False)
    assert b11.outcome(s0f, s1, s2, _s3_pass(), False).startswith("lap bookkeeping failed")


def test_t_within_2_percent() -> None:
    runs = battery(k_late=9.0)
    for key in KEYS:
        runs.append(run("t", res(times(9.18)), key))
    ok, _ = b11.t_within_2pct(runs)
    assert ok
    runs = [r for r in runs if r.condition != "t"]
    for key in KEYS:
        runs.append(run("t", res(times(9.2)), key))
    assert b11.t_within_2pct(runs)[0] is False


# -- S3 -------------------------------------------------------------------------------------------------


def kf_runs(**over) -> list:
    return [
        run("kf", res(**{"lap_truth_faults": t10.FAULTS, **over}) | _faulted(), key) for key in KEYS
    ]


def _faulted() -> dict:
    return {k: v for k, v in t10.result(t10.FAULTS).items() if k not in ("start_trace",)}


def test_s3_passes_on_the_fault_plan_and_reuses_d10_s2_items_5_and_6() -> None:
    d = b11.d11_s3(kf_runs(), CONST)
    assert d.passed, d
    names = list(d.items)
    assert any("item 5" in n for n in names) and any("item 6" in n for n in names)


def test_s3_a_contact_or_a_short_run() -> None:
    rs = kf_runs()
    rs[0] = dataclasses.replace(rs[0], contacts=1)
    assert not b11.d11_s3(rs, CONST).passed
    rs = kf_runs()
    for j in (0, 1):
        rs[j] = dataclasses.replace(rs[j], gt_lap_times_s=rs[j].gt_lap_times_s[:11])
    assert not item(b11.d11_s3(rs, CONST), "at least 15")
    rs = kf_runs()
    rs[0] = dataclasses.replace(rs[0], gt_lap_times_s=rs[0].gt_lap_times_s[:11])
    assert item(b11.d11_s3(rs, CONST), "at least 15")


def test_s3_brakes_over_all_laps() -> None:
    rs = kf_runs()
    rs = [dataclasses.replace(r, brakes_per_lap=(0,) * 10 + (2, 1, 0)) for r in rs]  # 3/12 = 0.25
    assert not item(b11.d11_s3(rs, CONST), "mean TTC")
    rs = [
        dataclasses.replace(r, brakes_per_lap=(0,) * 10 + (1, 1, 5)) for r in rs
    ]  # lap 13 not judged
    assert item(b11.d11_s3(rs, CONST), "mean TTC")


def _bound_run(after_bins: dict, doubtful_bins: dict, cap_only=None) -> b11.RunSummary:
    r = kf_runs()[0]
    rows = b10.system_laps_by_truth(r, CONST.b10)
    j = next(x["lap"] for x in rows if x["doubtful"])
    bounds = {
        str(j): {"cap_only_max": cap_only, "bins": doubtful_bins},
        str(j + 1): {"cap_only_max": None, "bins": after_bins},
    }
    r.extras["bound_by_lap"] = bounds
    return r


def test_s3_station_bound_after_a_doubtful_lap() -> None:
    ok = _bound_run({"10": 0.8, "11": 0.8}, {"10": 0.8, "11": 0.9})
    assert b11.bound_failures(ok, CONST) == []
    bad = _bound_run({"10": 0.81, "11": 0.8}, {"10": 0.8, "11": 0.9})
    assert len(b11.bound_failures(bad, CONST)) == 1
    # the doubtful lap went overdue: the cap alone bounds every station
    capped = _bound_run({"10": 0.6}, {"10": 0.9}, cap_only=0.5)
    assert b11.bound_failures(capped, CONST) == []
    over_cap = _bound_run({"40": 0.6}, {}, cap_only=0.5)
    assert len(b11.bound_failures(over_cap, CONST)) == 1


# -- S4 (addendum 1) ------------------------------------------------------------------------------------


def _decided(runs):
    s0, s1, s2 = b11.d11_s0(runs, CONST), b11.d11_s1(runs, CONST), b11.d11_s2(runs, CONST)
    return s0, s1, s2, _s3_pass()


def test_s4_median_of_the_sum_of_laps_2_to_10() -> None:
    # c: laps 2-7 at 12 s, 8-10 at 10 s: sum 102. k: laps 2-7 at e, 8-10 at 10 s: 6 e + 30
    at = (0.95 * 102 - 30) / 6
    for e, want in ((12.0, False), (at, True), (at + 0.01, False)):
        runs = battery(k_late=10.0, k_early=e)
        d = b11.d11_s4(runs, CONST, *_decided(runs))
        assert item(d, "1.") is want, (e, d)


def test_s4_worst_pair_and_a_pair_without_laps() -> None:
    runs = battery(k_late=10.0, k_early=10.0)  # sums 90 against 102: 0.88
    assert b11.d11_s4(runs, CONST, *_decided(runs)).passed
    i = next(j for j, r in enumerate(runs) if r.condition == "k")
    for q, ok in ((1.03, True), (1.035, False)):
        e = (q * 102 - 30) / 6
        rs = list(runs)
        rs[i] = run("k", res(times(10.0, e)), runs[i].key)
        assert item(b11.d11_s4(rs, CONST, *_decided(rs)), "2.") is ok
    rs = list(runs)
    rs[i] = run("k", res(times(10.0)[:8]), runs[i].key)
    assert not item(b11.d11_s4(rs, CONST, *_decided(rs)), "2.")


def test_s4_needs_the_other_gates_and_the_outcome() -> None:
    runs = battery(k_late=10.0, k_early=10.0)
    s0, s1, s2, s3 = _decided(runs)
    d = b11.d11_s4(runs, CONST, s0, s1, s2, s3)
    assert (
        b11.outcome_s4(d, s0, s1) == "D v1 learns faster than the schedule in sim at equal safety"
    )
    s3f = b11.Decision("D11-S3", False, {"x": (False, "")})
    d = b11.d11_s4(runs, CONST, s0, s1, s2, s3f)
    assert b11.outcome_s4(d, s0, s1) == "not judged"
    slow = battery(k_late=10.0, k_early=12.0)
    d = b11.d11_s4(slow, CONST, *_decided(slow))
    assert b11.outcome_s4(d, s0, s1) == "D v1 does not learn faster than the schedule in sim"
    # S2 item 3 (late ratio) is not needed for S4
    assert not item(b11.d11_s2(runs, CONST), "3.")
    assert item(b11.d11_s4(runs, CONST, s0, s1, s2, s3), "3.")


# -- the candidate rule ---------------------------------------------------------------------------------


DEV_KEYS = [(c, d, s) for c in CONST.minimum_cases for d in ("ccw", "cw") for s in (11, 12, 13)]


def dev_runs(scores: dict[str, float], **over_by_cand) -> list:
    rs = []
    for key in DEV_KEYS:
        rs.append(run("c", res(times(10.0)), key))
        for name, q in scores.items():
            rs.append(run(name, res(times(10.0 * q), **over_by_cand.get(name, {})), key))
    return rs


def test_candidate_rule_lowest_eligible_score() -> None:
    pick, scores, _ = b11.candidate_rule(dev_runs({"K1": 0.96, "K2": 0.90, "K3": 0.93}), CONST)
    assert pick == "K2"
    assert all(s.eligible for s in scores)


def test_candidate_rule_tie_order_within_0_01() -> None:
    pick, _, _ = b11.candidate_rule(dev_runs({"K1": 0.91, "K2": 0.90, "K3": 0.905}), CONST)
    assert pick == "K1"
    pick, _, _ = b11.candidate_rule(dev_runs({"K1": 0.92, "K2": 0.90, "K3": 0.905}), CONST)
    assert pick == "K3"
    pick, _, _ = b11.candidate_rule(dev_runs({"K1": 0.92, "K2": 0.90, "K3": 0.92}), CONST)
    assert pick == "K2"


def test_candidate_rule_the_0_97_stop() -> None:
    pick, _, why = b11.candidate_rule(dev_runs({"K1": 0.97, "K2": 0.99, "K3": 1.0}), CONST)
    assert pick == "K1"
    pick, _, why = b11.candidate_rule(dev_runs({"K1": 0.9701, "K2": 0.99, "K3": 1.0}), CONST)
    assert pick is None and why == "B11 not run: no development benefit"


def test_candidate_rule_eligibility() -> None:
    contact = {"K2": {"wall_contacts": 1}}
    pick, scores, _ = b11.candidate_rule(
        dev_runs({"K1": 0.97, "K2": 0.90, "K3": 0.95}, **contact), CONST
    )
    assert pick == "K3" and not next(s for s in scores if s.name == "K2").eligible
    stall = {"K3": {"longest_stop_s": 5.5}}
    pick, _, _ = b11.candidate_rule(dev_runs({"K1": 0.96, "K2": 0.99, "K3": 0.90}, **stall), CONST)
    assert pick == "K1"
    short = {"K1": {"gt_lap_times_registered_s": times(9.0)[:11]}}
    pick, scores, _ = b11.candidate_rule(
        dev_runs({"K1": 0.90, "K2": 0.99, "K3": 0.93}, **short), CONST
    )
    assert pick == "K3"
    none = {n: {"wall_contacts": 1} for n in ("K1", "K2", "K3")}
    pick, _, why = b11.candidate_rule(dev_runs({"K1": 0.9, "K2": 0.9, "K3": 0.9}, **none), CONST)
    assert pick is None and "not run" in why
    # the minimum cases must all be there
    rs = [
        r
        for r in dev_runs({"K1": 0.9, "K2": 0.95, "K3": 0.95})
        if not (r.condition == "K1" and r.case == "T")
    ]
    pick, _, _ = b11.candidate_rule(rs, CONST)
    assert pick == "K3"


# -- the drop rule --------------------------------------------------------------------------------------


def test_the_drop_rule() -> None:
    laps = {f"{c}_{d}": 13 for c in CASES for d in ("ccw", "cw")}
    assert b11.drop_rule(laps, CONST).dropped == ()
    laps["W2_cw"] = 2
    d = b11.drop_rule(laps, CONST)
    assert d.dropped == ("W2",) and d.decided
    laps["W3_ccw"] = 1
    d = b11.drop_rule(laps, CONST)
    assert not d.decided


# -- the guard and the commands -------------------------------------------------------------------------


def freeze_doc(**over) -> dict:
    doc = {
        "commit_sha": "a" * 40,
        "main_sha_contained": "b" * 40,
        "yaml_sha256": b11.REGISTERED_YAML_SHA256,
        "addendum_sha256": b11.ADDENDUM_SHA256,
        "candidate": "K1",
        "tuned_values": dict(bat.TUNED),
    }
    doc.update(over)
    return doc


def _cmd(case: str, seed: int, cond: str = "c", freeze: Path | None = None, **kw) -> list[str]:
    return bat.command_for(
        CONST, cond, case, "ccw", seed, 150, Path("/tmp/w"), Path("/tmp/o"),
        bat.b4t.DEFAULT_BUNDLE, bat.b4t.DEFAULT_DATA_ROOT,
        freeze_path=freeze or Path("/nonexistent/b11.json"), **kw,
    )  # fmt: skip


def test_no_w_case_and_no_seed_91_or_92_without_the_freeze_file(tmp_path: Path) -> None:
    for case, seed in (("W1", 91), ("W4", 92)):
        with pytest.raises(bat.BatteryError, match="does not exist"):
            _cmd(case, seed)
    for case, seed in (("W1", 11), ("T", 91)):
        with pytest.raises(bat.BatteryError, match="W case"):
            _cmd(case, seed)
    f = tmp_path / "b11.json"
    f.write_text(json.dumps(freeze_doc()))
    cmd = _cmd("W1", 91, "k", freeze=f, verify_git=False)
    i = cmd.index("--lap-params-json")
    assert json.loads(cmd[i + 1])["zone_floor_mode"] == "cap_wins"
    f.write_text(json.dumps(freeze_doc(yaml_sha256="0" * 64)))
    with pytest.raises(bat.BatteryError, match="yaml hash"):
        _cmd("W1", 91, "k", freeze=f, verify_git=False)
    f.write_text(json.dumps(freeze_doc(candidate="K9")))
    with pytest.raises(bat.BatteryError, match="candidate"):
        _cmd("W1", 91, "k", freeze=f, verify_git=False)
    f.write_text(json.dumps(freeze_doc(commit_sha="abc")))
    with pytest.raises(bat.BatteryError, match="commit_sha"):
        _cmd("W1", 91, "k", freeze=f, verify_git=False)
    f.write_text(json.dumps(freeze_doc()))
    with pytest.raises(bat.BatteryError, match="not a B11 condition"):
        _cmd("W1", 91, "K1", freeze=f, verify_git=False)


@pytest.mark.parametrize("seed", [21, 31, 41, 61, 71, 81, 82])
def test_other_attempts_seeds_are_refused(seed: int) -> None:
    with pytest.raises(bat.BatteryError, match="another registered attempt"):
        _cmd("T", seed)


def test_development_runs_are_the_development_cases_on_seeds_11_to_13() -> None:
    cmd = _cmd("T", 11, "K2")
    assert "--lap-memory" in cmd and cmd[cmd.index("--lap-memory") + 1] == "v1"
    assert cmd[cmd.index("--lap-source") + 1] == "truth"
    with pytest.raises(bat.BatteryError):
        _cmd("U1", 11)
    with pytest.raises(bat.BatteryError, match="not a development condition"):
        _cmd("T", 11, "q")
    assert "V1" in bat.dev_cases(CONST) and "OD" not in bat.dev_cases(CONST)


def test_run_dirs_of_earlier_batteries_are_refused() -> None:
    for p in bat.SPENT_RUNS:
        with pytest.raises(bat.BatteryError):
            bat.guard_run_dir(p / "x")


def test_the_conditions_switches() -> None:
    def flags(cond: str, cand: str | None = None) -> tuple[dict, dict]:
        f = bat.condition_flags(CONST, cond, 1.3, 0.0, cand)
        lap = json.loads(f[f.index("--lap-params-json") + 1]) if "--lap-params-json" in f else {}
        pl = (
            json.loads(f[f.index("--planner-params-json") + 1])
            if "--planner-params-json" in f
            else {}
        )
        return lap, pl

    assert flags("u") == ({}, {})
    assert flags("c") == ({}, {})
    assert flags("m") == ({}, {"lap_reward_mode": "keep_progress"})
    assert flags("f")[0] == {"value_profile_mode": "freeze_after_first_clean_lap"}
    assert flags("z")[0] == {"zone_floor_mode": "cap_wins"}
    assert flags("t") == ({}, {"terminal_curvature_bound": True, "curvature_bound_a_lat_mps2": 2.0})
    for k in ("K1", "K2", "K3"):
        lap, pl = flags(k)
        assert lap["zone_floor_mode"] == "cap_wins" and lap["station_uncertainty"] is True
        assert lap["reset_on_change"] is True and pl["lap_station_mode"] == "track"
    assert flags("K1")[1]["lap_reward_mode"] == "keep_progress"
    assert flags("K3")[0]["value_profile_mode"] == "freeze_after_first_clean_lap"
    assert flags("K2")[1]["lap_reward_mode"] == "replace"
    assert flags("k", "K3") == flags("K3")
    f = bat.condition_flags(CONST, "kf", 1.3, 6.0, "K1")
    assert json.loads(f[f.index("--lap-truth-faults-json") + 1]) == CONST.fault_plan
    assert bat.condition_flags(CONST, "a", 1.3, 0.0) == [
        "--lap-memory",
        "off",
        "--v-max-mps",
        "0.5",
    ]
    with pytest.raises(bat.BatteryError):
        bat.condition_flags(CONST, "k", 1.3, 0.0, None)


def test_reported_rows() -> None:
    runs = battery(k_late=9.0)
    rows = b11.reported_rows(runs)
    assert len(rows) == 16
    assert rows[0]["k_over_c"] == pytest.approx(0.9)
    assert rows[0]["reach_k"] == 8 and rows[0]["reach_c"] == 8
    assert math.isnan(rows[0]["T_late_t"])
