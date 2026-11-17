"""Battery B8 (lap gate v3 pre-registration sections 2 to 4): the constants file check, the count
rule, the class tolerance (the 0.3 s floor against 4 percent of the lap, both sides of the switch at
7.5 s), the single-error cap, the decisions D8-S0 and D8-S1 table-driven per item (pass, at the
threshold, epsilon over, NaN, an invalid run, a missing system lap), the drop rule, the freeze-file
guard that keeps every R or Q case and every seed 61 to 63 from running before the freeze commit,
and the run machinery with a fake runner. No ROS and no simulator here.

Every boundary of every threshold is tested from both sides, because the thresholds are the
pre-registration and a decision that flips at the wrong number cannot be found after the battery has
run."""

from __future__ import annotations

import argparse
import json
import math
import subprocess
import sys
from pathlib import Path

import battery_b8 as bb
import lap_battery_b8 as b8
import pytest
from lap_battery import RunSummary

HERE = Path(__file__).resolve().parent
sys.path.insert(0, str(HERE.parent / "l5_reactive_bench"))

CONST = b8.load_constants()
CASES = ("R1", "R2", "R3", "R4", "Q1", "Q2")
DIRS = ("ccw", "cw")
SEEDS = (61, 62, 63)
FROZEN = "f" * 40
LAP_S = 10.0
LAPS = 13


def truth_crossings(lap_s: float = LAP_S, first: float = LAP_S, n: int = LAPS) -> tuple[float, ...]:
    return tuple(100.0 + first + k * lap_s for k in range(n))


def mk(
    condition: str,
    case: str = "R1",
    direction: str = "ccw",
    seed: int = 61,
    laps: int = LAPS,
    lap_s: float = LAP_S,
    system: list[float] | None = None,
    system_crossings: list[float] | None = None,
    contacts: int = 0,
    error: str = "",
    late: float | None = None,
    last_odom: float | None = None,
) -> RunSummary:
    """A run: truth laps of `lap_s` (lap 1 included), the system's the same unless given."""
    times = [lap_s] * laps
    if late is not None:
        times = [lap_s] + [late] * (laps - 1)
    crossings = tuple(100.0 + sum(times[: k + 1]) for k in range(laps))
    sys_times = times if system is None else system
    sys_cross = list(crossings if system_crossings is None else system_crossings)
    return RunSummary(
        condition=condition,
        case=case,
        direction=direction,
        seed=seed,
        gt_lap_times_s=tuple(times),
        system_lap_times_s=tuple(sys_times),
        contacts=contacts,
        brakes_per_lap=tuple([0] * laps),
        error=error,
        extras={
            "truth_crossings_s": crossings,
            "system_crossings_s": tuple(sys_cross),
            "truth_last_odom_s": last_odom if last_odom is not None else crossings[-1] + 1.0,
        },
    )


def keys(cases=CASES):
    return [(c, d, s) for c in cases for d in DIRS for s in SEEDS]


#: Late laps (8 to 10) of conditions b and c are 30 percent shorter than lap 1: D3-S1 item 4.
DEFAULT_LATE = {"b": {"late": 0.7 * LAP_S}, "c": {"late": 0.7 * LAP_S}}


def full(cases=CASES, **override) -> list[RunSummary]:
    runs = []
    for cond in b8.B8_CONDITIONS:
        for case, d, s in keys(cases):
            kw = {**DEFAULT_LATE.get(cond, {}), **override.get(cond, {})}
            runs.append(mk(cond, case, d, s, **kw))
    return runs


def replace(runs, condition, key, **kw):
    out = []
    for r in runs:
        if r.condition == condition and r.key == key:
            r = mk(condition, *key, **kw)
        out.append(r)
    return out


def failed(decision, prefix: str) -> list[str]:
    return [k for k, (ok, _) in decision.items.items() if k.startswith(prefix) and not ok]


# -- the constants file ----------------------------------------------------------------------------


def test_the_yaml_hash_is_the_registered_constant_and_the_committed_file() -> None:
    assert b8.REGISTERED_YAML_SHA256 == b8.sha256_file(b8.PREREG_YAML)
    assert CONST.yaml_sha256 == b8.REGISTERED_YAML_SHA256
    assert b8.V05_LAPS_SHA256 == b8.sha256_file(b8.V05_LAPS_YAML)


def test_thresholds_are_the_preregistered_numbers() -> None:
    t = CONST.thresholds
    assert t["placed"] == {"tolerance_s": 0.3, "max_single_error_s": 1.0}
    assert t["aliased"]["tolerance_share_of_truth_lap"] == 0.04
    assert t["aliased"]["max_single_error_share_of_truth_lap"] == 0.08
    assert t["s0b_min_share_within_tolerance"] == 0.98
    assert (CONST.required_complete(36), CONST.required_complete(30)) == (33, 28)
    assert CONST.required_complete(24) == 22
    assert CONST.th["s1_reach_max_ratio_to_b"] == 1.05 and CONST.th["s1_max_median_r"] == 0.80
    assert CONST.th["max_contacts"] == 0
    assert CONST.battery_seeds == (61, 62, 63) and CONST.laps_judged == 12
    assert CONST.laps_driven == 13 and CONST.laps == 13 and CONST.lap1_cap_mps == 0.5
    assert set(CONST.battery_cases) == set(CASES) and set(CONST.conditions) == {"a", "b", "c"}
    assert CONST.development_seeds == (11, 12, 13)
    assert set(CONST.development_cases) == {"ND", "P4", "OD", "RD"}
    assert CONST.count_rule == {"window_end": "truth_crossing_12_plus_share_of_truth_lap_12",
                                "share": 0.5, "required": 12}  # fmt: skip
    assert CONST.validity["recording_must_cover_count_window"] is True
    assert CONST.doc["d8_c0"]["on_scan_p99_max_ms"] == 80
    with pytest.raises(b8.PreregError):
        CONST.required_complete(18)


def test_the_classes_are_fixed_by_the_track() -> None:
    assert {c: CONST.class_of(c) for c in CASES} == {
        "R1": "aliased", "R2": "aliased", "R3": "aliased", "R4": "aliased",
        "Q1": "placed", "Q2": "placed",
    }  # fmt: skip
    # development cases by the stated rule: no straight longer than the lane is wide
    assert CONST.class_of("P4") == CONST.class_of("OD") == CONST.class_of("ND") == "aliased"
    assert b8.class_of_spec({"lane_width_m": 1.0, "straight_length_m": 1.0}) == "aliased"
    assert b8.class_of_spec({"lane_width_m": 1.0, "straight_length_m": 1.01}) == "placed"
    with pytest.raises(b8.PreregError):
        b8.class_of_spec({"class": "round", "lane_width_m": 1.0})
    with pytest.raises(b8.PreregError):
        CONST.class_of("P1")


def test_a_changed_yaml_is_refused(tmp_path: Path) -> None:
    changed = tmp_path / "ring.yaml"
    changed.write_text(b8.PREREG_YAML.read_text().replace("laps_judged: 12", "laps_judged: 11"))
    with pytest.raises(b8.PreregError, match="SHA-256"):
        b8.load_constants(changed)
    ok = b8.load_constants(changed, expected_sha=b8.sha256_file(changed))
    assert ok.laps_judged == 11


@pytest.mark.parametrize(
    ("old", "new"),
    [
        ("s1_reach_min_runs: {of_36: 33", "s1_reach_min_runs: {of_36: 32"),
        ("s1_complete_laps: 10", "s1_complete_laps: 9"),
        ("late_laps: [8, 9, 10]", "late_laps: [7, 8, 9]"),
        ("seeds: [61, 62, 63]", "seeds: [61, 62, 64]"),
        ("v05_laps_sha256: 35949c7d", "v05_laps_sha256: 35949c7e"),
    ],
)
def test_a_yaml_the_d3_functions_cannot_apply_as_written_is_refused(
    tmp_path: Path, old: str, new: str
) -> None:
    text = b8.PREREG_YAML.read_text()
    assert old in text
    changed = tmp_path / "x.yaml"
    changed.write_text(text.replace(old, new, 1))
    with pytest.raises(b8.PreregError):
        b8.load_constants(changed, expected_sha=b8.sha256_file(changed))


# -- the class tolerance and the single-error cap ----------------------------------------------------


@pytest.mark.parametrize(
    ("klass", "lap_s", "tolerance", "single"),
    [
        ("placed", 5.0, 0.3, 1.0),
        ("placed", 20.0, 0.3, 1.0),  # the placed class has no share
        ("aliased", 5.0, 0.3, 1.0),  # 0.04 T = 0.2 below the floor
        ("aliased", 7.5, 0.3, 1.0),  # the switch: 0.04 T is exactly 0.3
        ("aliased", 7.6, 0.304, 1.0),
        ("aliased", 12.0, 0.48, 1.0),
        ("aliased", 12.5, 0.5, 1.0),  # 0.08 T is exactly 1.0
        ("aliased", 12.6, 0.504, 1.008),
        ("aliased", 20.0, 0.8, 1.6),
    ],
)
def test_the_class_tolerance_and_the_single_error_limit(
    klass: str, lap_s: float, tolerance: float, single: float
) -> None:
    assert CONST.tolerance_s(klass, lap_s) == pytest.approx(tolerance)
    assert CONST.max_single_s(klass, lap_s) == pytest.approx(single)


def test_the_floor_wins_below_7_5_s_and_the_share_above() -> None:
    below = CONST.tolerance_s("aliased", 7.49)
    above = CONST.tolerance_s("aliased", 7.51)
    assert below == 0.3 and above > 0.3
    assert CONST.tolerance_s("aliased", 7.5) == pytest.approx(0.3)


# -- the count rule -------------------------------------------------------------------------------------


def window_end(r: RunSummary) -> float:
    return r.extras["truth_crossings_s"][11] + 0.5 * r.gt_lap_times_s[11]


@pytest.mark.parametrize(
    ("n_counted", "ok"),
    [(11, False), (12, True), (13, False)],
)
def test_the_count_rule_wants_exactly_12_crossings_by_the_window_end(
    n_counted: int, ok: bool
) -> None:
    base = mk("a")
    end = window_end(base)
    crossings = [100.0 + 8.0 * (k + 1) for k in range(n_counted)]  # all inside the window
    crossings += [end + 1.0 + k for k in range(13 - n_counted)]  # the rest outside it
    r = mk("a", system_crossings=crossings)
    assert b8.count_rule(r, CONST)[0] is ok


def test_a_crossing_exactly_at_the_window_end_counts_and_epsilon_after_does_not() -> None:
    base = mk("a")
    end = window_end(base)
    inside = [100.0 + LAP_S * (k + 1) for k in range(11)] + [end]
    assert b8.count_rule(mk("a", system_crossings=inside), CONST)[0] is True
    after = [100.0 + LAP_S * (k + 1) for k in range(11)] + [end + 1e-6]
    assert b8.count_rule(mk("a", system_crossings=after), CONST)[0] is False


def test_the_count_rule_is_about_phase_not_precision() -> None:
    """Every system crossing a third of a lap late: still 12 by truth crossing 12 plus half a lap
    (a winding lap keeps no phase to a scan's precision), but a lap early the count is 13."""
    base = mk("a")
    late = [t + LAP_S / 3.0 for t in base.extras["truth_crossings_s"]]
    assert b8.count_rule(mk("a", system_crossings=late), CONST)[0] is True
    early = [t - LAP_S * 0.6 for t in base.extras["truth_crossings_s"]]
    assert b8.count_rule(mk("a", system_crossings=early), CONST)[0] is False


def test_a_run_without_truth_crossing_12_fails_the_count_rule() -> None:
    r = mk("a", laps=11)
    ok, detail = b8.count_rule(r, CONST)
    assert ok is False and "11 truth crossings" in detail


# -- D8-S0 -------------------------------------------------------------------------------------------------


def test_everything_perfect_passes_both_decisions() -> None:
    runs = full()
    assert b8.d8_s0(runs, CONST).passed
    assert b8.d8_s1(runs, CONST).passed
    assert b8.d8_s1(runs, CONST).name == "D8-S1" and b8.d8_s0(runs, CONST).name == "D8-S0"


@pytest.mark.parametrize(
    ("case", "error_s", "ok"),
    [
        ("R1", 0.30, True),  # aliased, lap 10 s: tolerance max(0.3, 0.4) = 0.4
        ("R1", 0.4 - 1e-9, True),
        ("R1", 0.4001, False),
        ("Q1", 0.3 - 1e-9, True),  # placed: flat 0.3 s
        ("Q1", 0.3001, False),
        ("Q1", 0.40, False),
    ],
)
def test_s0a_every_judged_lap_is_within_its_class_tolerance(
    case: str, error_s: float, ok: bool
) -> None:
    runs = full()
    for k, (c, d, s) in enumerate(keys()):
        if c == case and d == "ccw" and s == 61:
            base = [LAP_S] * LAPS
            base[4] = LAP_S + error_s
            runs = replace(runs, "a", (c, d, s), system=base)
    s0 = b8.d8_s0(runs, CONST)
    assert (failed(s0, "S0a: every judged lap") == []) is ok
    assert s0.passed is ok


def test_lap_1_is_inside_the_tolerance_with_no_allowance() -> None:
    runs = full()
    system = [LAP_S + 0.5] + [LAP_S] * (LAPS - 1)  # a lap 1 that is half a second long
    runs = replace(runs, "a", ("Q1", "ccw", 61), system=system)
    assert failed(b8.d8_s0(runs, CONST), "S0a: every judged lap")


def test_the_thirteenth_lap_is_not_judged() -> None:
    runs = full()
    system = [LAP_S] * 12 + [LAP_S + 5.0]
    runs = replace(runs, "a", ("Q1", "ccw", 61), system=system)
    assert b8.d8_s0(runs, CONST).passed


def test_a_missing_system_lap_is_outside_every_tolerance_and_a_nan_too() -> None:
    runs = replace(full(), "a", ("R1", "ccw", 61), system=[LAP_S] * 10)
    s0 = b8.d8_s0(runs, CONST)
    assert failed(s0, "S0a: every judged lap")
    nan = replace(full(), "a", ("R1", "ccw", 61), system=[math.nan] + [LAP_S] * 12)
    assert failed(b8.d8_s0(nan, CONST), "S0a: every judged lap")
    errs = b8.judged_errors(nan[0] if nan[0].case == "R1" else mk("a"), CONST)
    assert errs[0][1] in (0.0, math.inf)


def test_s0a_the_count_rule_in_every_run() -> None:
    wrong = [100.0 + LAP_S * (k + 1) for k in range(11)]  # 11 crossings: a lap lost
    runs = replace(full(), "a", ("R2", "cw", 62), system_crossings=wrong)
    assert failed(b8.d8_s0(runs, CONST), "S0a: the count rule")


def test_s0b_98_percent_of_judged_laps_within_tolerance() -> None:
    """Condition c has 36 runs of 12 judged laps: 432 laps. 98 percent is 423.36: eight bad laps
    pass (424 within), nine fail (423 within)."""

    def with_bad(n: int) -> list[RunSummary]:
        out = list(full())
        left = n
        for i, r in enumerate(out):
            if r.condition == "c" and left > 0:
                system = [LAP_S + 0.45] + [0.7 * LAP_S] * (LAPS - 1)  # lap 1 is 0.05 over
                out[i] = mk("c", *r.key, system=system, late=0.7 * LAP_S)
                left -= 1
        return out

    assert not failed(b8.d8_s0(with_bad(8), CONST), "S0b(c): at least")  # 424 of 432
    assert failed(b8.d8_s0(with_bad(9), CONST), "S0b(c): at least")  # 423 of 432 < 98 percent


def test_s0b_no_lap_beyond_the_single_error_limit() -> None:
    # aliased lap of 10 s: limit max(1.0, 0.8) = 1.0
    ok = replace(full(), "c", ("R1", "ccw", 61), system=[LAP_S + 1.0] + [LAP_S] * 12)
    assert not failed(b8.d8_s0(ok, CONST), "S0b(c): no lap beyond")
    over = replace(full(), "c", ("R1", "ccw", 61), system=[LAP_S + 1.0001] + [LAP_S] * 12)
    assert failed(b8.d8_s0(over, CONST), "S0b(c): no lap beyond")
    # placed: flat 1.0
    placed = replace(full(), "c", ("Q1", "ccw", 61), system=[LAP_S + 1.0001] + [LAP_S] * 12)
    assert failed(b8.d8_s0(placed, CONST), "S0b(c): no lap beyond")
    # aliased lap of 20 s: the share applies, limit 1.6
    long = mk("c", "R1", "ccw", 61, lap_s=20.0, system=[21.5] + [20.0] * 12)
    assert b8.judged_errors(long, CONST)[0][3] == pytest.approx(1.6)
    runs = [r for r in full() if not (r.condition == "c" and r.key == ("R1", "ccw", 61))]
    assert not failed(b8.d8_s0([*runs, long], CONST), "S0b(c): no lap beyond")


def test_an_invalid_run_is_a_run_that_failed_to_execute() -> None:
    runs = replace(full(), "a", ("R3", "cw", 63), error="truth invalid: no_scan_at_start")
    assert failed(b8.d8_s0(runs, CONST), "S0a: no condition (a) run failed")
    runs = replace(full(), "c", ("R3", "cw", 63), error="starved")
    assert failed(b8.d8_s0(runs, CONST), "S0b(c): no run failed")


def test_a_missing_condition_is_refused() -> None:
    runs = [r for r in full() if r.condition != "c"]
    with pytest.raises(b8.MissingPairError):
        b8.d8_s0(runs, CONST)


# -- D8-S1 ------------------------------------------------------------------------------------------------


def test_d8_s1_is_d3_s1_with_the_counts_of_the_yaml() -> None:
    """33 of 36: three runs may fail to reach, a fourth may not."""

    def with_unreached(n: int) -> list[RunSummary]:
        runs = full(c={"late": 9.0}, b={"late": 9.0})
        out = []
        left = n
        for r in runs:
            if r.condition == "c" and left > 0:
                left -= 1
                r = mk("c", *r.key, late=9.0 * 1.06)  # T_late(c) 6 percent above b: not reached
            out.append(r)
        return out

    def base_runs(n: int) -> list[RunSummary]:
        runs = []
        for r in with_unreached(n):
            if r.condition == "b":
                r = mk("b", *r.key, late=9.0)
            runs.append(r)
        return runs

    assert failed(b8.d8_s1(base_runs(0), CONST), "3.") == []
    assert failed(b8.d8_s1(base_runs(3), CONST), "3.") == []  # 33 of 36 reach
    assert failed(b8.d8_s1(base_runs(4), CONST), "3.")  # 32 of 36 do not


def test_d8_s1_completion_item_needs_33_of_36() -> None:
    runs = full()
    for k, r in enumerate([r for r in runs if r.condition == "c"][:3]):
        runs = replace(runs, "c", r.key, laps=9)
    assert not failed(b8.d8_s1(runs, CONST), "2.")  # 33 of 36 complete
    runs = replace(runs, "c", [r for r in runs if r.condition == "c"][3].key, laps=9)
    assert failed(b8.d8_s1(runs, CONST), "2.")


def test_d8_s1_contacts_and_dropped_case_counts() -> None:
    runs = replace(full(), "c", ("Q1", "ccw", 62), contacts=1)
    assert failed(b8.d8_s1(runs, CONST), "1.")
    # one case dropped: 30 runs per condition, 28 of 30; two dropped: 22 of 24
    assert not failed(b8.d8_s1(full(CASES[:5]), CONST), "2.")
    assert not failed(b8.d8_s1(full(CASES[:4]), CONST), "2.")
    with pytest.raises(b8.PreregError):
        b8.d8_s1(full(CASES[:3]), CONST)  # 18 runs: nothing is registered


# -- the drop rule ----------------------------------------------------------------------------------------


def laps_all(n: int = 13, **override: int) -> dict[str, int]:
    out = {f"{c}_{d}": n for c in CASES for d in DIRS}
    out.update(override)
    return out


def test_the_drop_rule_with_nothing_dropped_is_36_runs() -> None:
    d = b8.drop_rule(laps_all(), CONST)
    assert d.decided and d.dropped == () and d.runs_per_condition == 36


@pytest.mark.parametrize(
    ("override", "dropped", "runs"),
    [
        ({"R1_cw": 2}, ("R1",), 30),  # either direction drops the case
        ({"R1_ccw": 2}, ("R1",), 30),
        ({"R1_cw": 3}, (), 36),  # 3 laps is enough
        ({"R1_cw": 2, "R2_ccw": 0}, ("R1", "R2"), 24),
    ],
)
def test_the_drop_rule_drops_a_case_when_either_direction_has_under_3_laps(
    override: dict, dropped: tuple, runs: int
) -> None:
    d = b8.drop_rule(laps_all(**override), CONST)
    assert d.decided and d.dropped == dropped and d.runs_per_condition == runs


@pytest.mark.parametrize(
    "override",
    [
        {"R1_cw": 0, "R2_cw": 0, "R3_cw": 0},  # three dropped: more than the two allowed
        {"R1_cw": 0, "R2_cw": 0, "R3_cw": 5},  # two dropped, leaving R3, R4: fine, see below
    ],
)
def test_not_decided_with_more_than_two_dropped(override: dict) -> None:
    d = b8.drop_rule(laps_all(**override), CONST)
    if len([k for k, v in override.items() if v < 3]) > 2:
        assert not d.decided and "at most 2" in d.reason
    else:
        assert d.decided


def test_not_decided_when_fewer_than_two_aliased_cases_are_left() -> None:
    d = b8.drop_rule(laps_all(R1_cw=0, R2_cw=0), CONST)
    assert d.decided  # R3 and R4 are left
    d = b8.drop_rule(laps_all(R1_cw=0, R2_cw=0, R3_cw=0), CONST)
    assert not d.decided
    # two aliased left but the placed ones gone
    d = b8.drop_rule(laps_all(Q1_cw=0, Q2_cw=0), CONST)
    assert d.decided is False and "placed" in d.reason
    d = b8.drop_rule(laps_all(R1_cw=0, Q1_ccw=0), CONST)
    assert d.decided and d.runs_per_condition == 24


def test_a_run_missing_from_the_drop_input_drops_its_case() -> None:
    laps = laps_all()
    del laps["Q2_cw"]
    assert b8.drop_rule(laps, CONST).dropped == ("Q2",)


# -- the freeze guard -------------------------------------------------------------------------------------


def write_freeze(path: Path, **kw) -> Path:
    doc = {"freeze_commit": FROZEN, "yaml_sha256": b8.REGISTERED_YAML_SHA256} | kw
    path.write_text(json.dumps(doc))
    return path


def build(freeze: Path, condition="c", case="R1", direction="ccw", seed=61, tmp=Path("/tmp")):
    return bb.command_for(
        CONST,
        condition,
        case,
        direction,
        seed,
        120,
        tmp,
        tmp / "r.json",
        bb.b4t.DEFAULT_BUNDLE,
        Path("/data"),
        freeze_path=freeze,
    )


@pytest.mark.parametrize("case", CASES)
@pytest.mark.parametrize("seed", SEEDS)
def test_no_b8_case_can_run_without_the_freeze_file(
    tmp_path: Path, case: str, seed: int, monkeypatch: pytest.MonkeyPatch
) -> None:
    monkeypatch.setenv(bb.GIT_SHA_ENV, "a" * 40)  # no git here: the guard's file checks only
    with pytest.raises(bb.BatteryError, match="before the freeze"):
        build(tmp_path / "nope.json", case=case, seed=seed)
    bad = tmp_path / "bad.json"
    bad.write_text(json.dumps({"freeze_commit": FROZEN, "yaml_sha256": "0" * 64}))
    with pytest.raises(bb.BatteryError, match="yaml hash"):
        build(bad, case=case, seed=seed)
    bad.write_text(json.dumps({"freeze_commit": "abc", "yaml_sha256": b8.REGISTERED_YAML_SHA256}))
    with pytest.raises(bb.BatteryError, match="40 digit"):
        build(bad, case=case, seed=seed)
    bad.write_text("{")
    with pytest.raises(bb.BatteryError, match="not valid JSON"):
        build(bad, case=case, seed=seed)
    ok = write_freeze(tmp_path / "b8.json")
    assert build(ok, case=case, seed=seed)[0]


@pytest.mark.parametrize("case", ["R1", "Q1"])
def test_a_b8_seed_on_a_development_case_and_a_b8_case_on_a_development_seed_are_refused(
    tmp_path: Path, case: str, monkeypatch: pytest.MonkeyPatch
) -> None:
    monkeypatch.setenv(bb.GIT_SHA_ENV, "a" * 40)
    ok = write_freeze(tmp_path / "b8.json")
    with pytest.raises(bb.BatteryError, match="B8"):
        build(ok, case=case, seed=11)
    with pytest.raises(bb.BatteryError, match="B8"):
        build(ok, case="P4", seed=61)


@pytest.mark.parametrize("seed", [31, 32, 41, 42, 51, 52, 53])
@pytest.mark.parametrize("case", ["P4", "OD", "ND", "R1"])
def test_the_seeds_of_the_other_registered_attempts_never_run(
    tmp_path: Path, seed: int, case: str, monkeypatch: pytest.MonkeyPatch
) -> None:
    monkeypatch.setenv(bb.GIT_SHA_ENV, "a" * 40)
    ok = write_freeze(tmp_path / "b8.json")
    with pytest.raises(bb.BatteryError):
        build(ok, case=case, seed=seed)


@pytest.mark.parametrize("case", ["P4", "OD", "ND", "RD"])
@pytest.mark.parametrize("seed", [11, 12, 13])
def test_a_development_case_on_a_development_seed_needs_no_freeze_file(
    tmp_path: Path, case: str, seed: int
) -> None:
    command = build(tmp_path / "nope.json", case=case, seed=seed)
    assert command[1].endswith("track_family_runner.py")
    assert not bb.needs_freeze(CONST, case, seed)


def test_an_unknown_case_is_refused() -> None:
    with pytest.raises(bb.BatteryError):
        bb.needs_freeze(CONST, "P1", 11)
    with pytest.raises(bb.BatteryError):
        bb.needs_freeze(CONST, "P4", 14)


def test_the_conditions_map_to_the_registered_runner_flags(
    tmp_path: Path, monkeypatch: pytest.MonkeyPatch
) -> None:
    monkeypatch.setenv(bb.GIT_SHA_ENV, "a" * 40)
    freeze = write_freeze(tmp_path / "b8.json")
    ceiling = bb.b4t.ceiling_mps(bb.b4t.DEFAULT_BUNDLE)
    a = parse(build(freeze, "a", tmp=tmp_path))
    assert (a.lap_memory, a.v_max_mps) == ("off", 0.5)
    b = parse(build(freeze, "b", tmp=tmp_path))
    assert (b.lap_memory, b.v_max_mps) == ("off", ceiling)
    c = parse(build(freeze, "c", tmp=tmp_path))
    assert (c.lap_memory, c.v_max_mps, c.v_lap1_mps) == ("v0", ceiling, 0.5)
    assert c.stop_after_laps == 13 and c.driver == "pnp" and c.lap_source == "start_scan"
    cw = parse(build(freeze, "c", direction="cw", tmp=tmp_path))
    assert cw.reverse and not c.reverse
    with pytest.raises(bb.BatteryError):
        build(freeze, "d", tmp=tmp_path)
    with pytest.raises(bb.BatteryError):
        build(freeze, direction="sideways", tmp=tmp_path)


def test_every_case_builds_its_track_without_the_class_key(
    tmp_path: Path, monkeypatch: pytest.MonkeyPatch
) -> None:
    monkeypatch.setenv(bb.GIT_SHA_ENV, "a" * 40)
    freeze = write_freeze(tmp_path / "b8.json")
    for case in CASES:
        args = parse(build(freeze, case=case, tmp=tmp_path))
        params = json.loads(args.track_params_json)
        assert "class" not in params and args.track in ("stadium", "rounded_rect")
    r3 = json.loads(parse(build(freeze, case="R3", tmp=tmp_path)).track_params_json)
    assert r3["obstacles"][0]["protrusion_m"] == 0.2


def parse(command: list[str]):
    import track_family_runner as runner

    return runner.build_parser().parse_args(command[2:])


# -- the run machinery with a fake runner ---------------------------------------------------------------


class FakeRunner:
    """Stands in for subprocess.run: parses the argv with the runner's own parser and writes a
    result shaped like the runner's, with the registered truth fields."""

    def __init__(self, gaps=None, invalid=None, die=None, laps=None, short=None):
        self.calls: list[list[str]] = []
        self.gaps = gaps or {}
        self.invalid = invalid or {}
        self.die = die or set()
        self.laps = laps or {}
        self.short = short or {}  # name -> per attempt: recording ends before the window
        self.attempts: dict[str, int] = {}

    def __call__(self, command, stdout=None, stderr=None, env=None, check=False):
        args = parse(command)
        self.calls.append(command)
        name = Path(args.work_dir).name.split(".attempt")[0]
        k = self.attempts[name] = self.attempts.get(name, 0) + 1
        if name in self.die:
            return subprocess.CompletedProcess(command, 3)
        gaps = self.gaps.get(name, [0.02])
        gap = gaps[min(k, len(gaps)) - 1]
        reasons = self.invalid.get(name, [""])
        reason = reasons[min(k, len(reasons)) - 1]
        n = self.laps.get(name, 13)
        times = ([20.0] + [10.0] * (n - 1)) if n else []
        crossings = [100.0 + sum(times[: i + 1]) for i in range(n)]
        shorts = self.short.get(name, [False])
        last = crossings[-1] + 1.0 if crossings else 0.0
        if shorts[min(k, len(shorts)) - 1] and n >= 12:
            last = crossings[11] + 0.5 * times[11] - 1.0
        result = {
            "gt_lap_times_by_crossing_s": times,
            "gt_lap_times_registered_s": times,
            "gt_lap_crossings_registered_s": crossings,
            "truth_last_odom_s": last,
            "truth_valid": reason == "",
            "truth_invalid_reason": reason,
            "system_lap_events": [
                {"lap_time_s": t, "t_cross_s": c} for t, c in zip(times, crossings, strict=True)
            ],
            "wall_contacts": 0,
            "brake_per_lap": [{} for _ in times],
            "drive_raw": {"max_gap_s": gap},
            "real_time_factor": 1.0,
            "lap_status": {
                "launch_dwell_s": 0.3,
                "lap_line_sources": ["winding"] * n,
                "start_alias_share_last_lap": 0.6,
                "ring": {"fixes_last_lap": 0},
                "stale_odometry_scans": 1,
                "on_scan_ms_mean": 3.0,
                "on_scan_ms_p99": 11.0,
            },
            "stopped_because": "laps",
        }
        Path(args.out).write_text(json.dumps(result))
        log = Path(args.work_dir) / "lap_memory_node.log"
        log.parent.mkdir(parents=True, exist_ok=True)
        log.write_text("lap 1: 20.00 s, clean\nlap 2: 10.0 s, UNCLEAN (missed gate, not a lap)\n")
        return subprocess.CompletedProcess(command, 0)


@pytest.fixture
def harness(tmp_path: Path, monkeypatch):
    monkeypatch.setattr(bb.b4t, "check_clean", lambda: None)
    monkeypatch.setattr(bb.b4t, "git_sha", lambda: "a" * 40)
    monkeypatch.setenv(bb.GIT_SHA_ENV, "a" * 40)
    freeze = write_freeze(tmp_path / "b8.json")

    def go(fake, *argv, freeze_path=freeze):
        monkeypatch.setattr(bb.subprocess, "run", fake)
        return bb.main([*argv, "--freeze-path", str(freeze_path), "--data-root", "/data"])

    return go, tmp_path / "out"


def test_no_runner_is_called_without_the_freeze_file(harness, tmp_path: Path) -> None:
    go, out = harness
    fake = FakeRunner()
    assert go(fake, "run", "--out", str(out), freeze_path=tmp_path / "nope.json") == 2
    assert fake.calls == [] and not out.exists() or not list(out.glob("*.json"))


def test_the_seed_61_condition_a_runs_come_first_and_the_battery_runs_to_completion(
    harness,
) -> None:
    go, out = harness
    fake = FakeRunner()
    assert go(fake, "run", "--out", str(out), "--workers", "1") == 0
    assert len(fake.calls) == 108  # 3 conditions x 6 cases x 2 directions x 3 seeds
    first = [Path(c[c.index("--work-dir") + 1]).name for c in fake.calls[:12]]
    assert all(n.startswith("a_") and "_61.attempt" in n for n in first)
    drops = json.loads((out / "drops.json").read_text())
    assert drops["dropped"] == [] and drops["decided"] and len(drops["laps"]) == 12
    meta = json.loads((out / "c_Q2_cw_62.json").read_text())["battery_b8"]
    assert meta["mode"] == "battery" and meta["git_sha"] == "a" * 40
    assert meta["freeze_commit"] == FROZEN and meta["prereg_yaml_sha256"] == CONST.yaml_sha256
    assert meta["accepted_attempt"] == 1 and meta["attempts"][0]["starved"] is False
    report = bb.report_text(out)
    assert "D8-S0: PASS" in report and "D8-S1: PASS" in report
    assert "pre-registration" in report and "launch dwell s" in report
    assert "crossing 12" in report and "laps by line source" in report
    assert "condition c: 36 laps flagged missed gate in 36 of 36 runs" in report
    again = FakeRunner()
    assert go(again, "run", "--out", str(out), "--workers", "1") == 0
    assert again.calls == []  # resumed: finished names are skipped


def test_a_case_dropped_by_the_rule_is_never_run_again(harness) -> None:
    go, out = harness
    fake = FakeRunner(laps={"a_R4_cw_61": 2})
    assert go(fake, "run", "--out", str(out), "--workers", "1") == 0
    drops = json.loads((out / "drops.json").read_text())
    assert drops["dropped"] == ["R4"] and drops["decided"]
    names = {Path(c[c.index("--work-dir") + 1]).name.split(".")[0] for c in fake.calls}
    assert sorted(n for n in names if "_R4_" in n) == ["a_R4_ccw_61", "a_R4_cw_61"]
    assert len(names) == 3 * 30 + 2
    report = bb.report_text(out)
    assert "INCOMPLETE" not in report and "D8-S0: PASS" in report and "D8-S1: PASS" in report


def test_nothing_is_decided_when_the_drop_rule_leaves_too_little(harness) -> None:
    go, out = harness
    fake = FakeRunner(laps={"a_R1_cw_61": 0, "a_R2_cw_61": 0, "a_R3_cw_61": 0})
    assert go(fake, "run", "--out", str(out), "--workers", "1") == 2
    assert len(fake.calls) == 12  # the seed 61 condition a runs only
    assert "NOTHING IS DECIDED" in bb.report_text(out)


def test_a_starved_invalid_or_short_run_is_rerun_alone_and_every_attempt_is_reported(
    harness,
) -> None:
    go, out = harness
    fake = FakeRunner(
        gaps={"a_R1_ccw_61": [0.05, 0.02]},
        invalid={"a_R1_cw_61": ["truth_starts_after_crossing_0", ""]},
        short={"a_R2_ccw_61": [True, False], "a_Q1_ccw_61": [True, True, True]},
    )
    assert go(fake, "run", "--out", str(out), "--workers", "1") == 0
    assert fake.attempts["a_R1_ccw_61"] == 2 and fake.attempts["a_R1_cw_61"] == 2
    assert fake.attempts["a_R2_ccw_61"] == 2 and fake.attempts["a_Q1_ccw_61"] == 3
    ok = json.loads((out / "a_R2_ccw_61.json").read_text())["battery_b8"]
    assert ok["accepted_attempt"] == 2 and ok["accepted_while_short"] is False
    assert [a["recording_too_short"] for a in ok["attempts"]] == [True, False]
    stuck = json.loads((out / "a_Q1_ccw_61.json").read_text())["battery_b8"]
    assert stuck["accepted_while_short"] is True and len(stuck["attempts"]) == 3
    assert len(list((out / "attempts").glob("a_Q1_ccw_61.attempt*.json"))) == 3
    report = bb.report_text(out)
    assert "## Attempts" in report and "a_Q1_ccw_61" in report
    # still short after 3 attempts: a run that failed to execute
    runs = b8.load_runs(out, CONST)
    q1 = next(
        r for r in runs if (r.condition, r.case, r.direction, r.seed) == ("a", "Q1", "ccw", 61)
    )
    assert "recording ends before the count window" in q1.error


def test_a_recording_that_ends_before_the_window_is_not_a_result() -> None:
    crossings = [100.0 + 10.0 * (k + 1) for k in range(13)]
    laps = [10.0] * 13
    base = {"gt_lap_crossings_registered_s": crossings, "gt_lap_times_registered_s": laps}
    end = crossings[11] + 5.0
    assert bb.recording_too_short({**base, "truth_last_odom_s": end - 0.01})
    assert not bb.recording_too_short({**base, "truth_last_odom_s": end})
    assert bb.recording_too_short({**base})  # no field at all
    short_run = {
        "gt_lap_crossings_registered_s": crossings[:5],
        "gt_lap_times_registered_s": laps[:5],
    }
    assert not bb.recording_too_short(short_run)  # fewer laps than the window: a short run
    assert bb.needs_rerun({**base, "truth_last_odom_s": end - 1.0, "truth_valid": True})


def test_workers_are_capped_at_three(harness) -> None:
    go, out = harness
    fake = FakeRunner()
    assert go(fake, "run", "--out", str(out), "--workers", "4") == 2
    assert fake.calls == []


def test_a_worker_that_dies_stops_the_battery_instead_of_looping(harness) -> None:
    go, out = harness

    def boom(command, **kw):
        raise OSError("disk full")

    assert go(boom, "run", "--out", str(out), "--workers", "1") == 2
    assert not list(out.glob("*_*_*_*.json"))


def test_missed_flags_are_counted_from_the_accepted_attempts_log(tmp_path: Path) -> None:
    log = tmp_path / "work" / "c_R1_ccw_61.attempt2" / "lap_memory_node.log"
    log.parent.mkdir(parents=True)
    log.write_text("lap 1: ok\nlap 2: UNCLEAN (missed gate, not a lap)\n")
    assert bb.missed_flags(tmp_path, "c_R1_ccw_61", 2) == 1
    assert bb.missed_flags(tmp_path, "c_R1_ccw_61", 1) == 0


# -- development runs ---------------------------------------------------------------------------------------


def test_dev_runs_take_development_cases_on_development_seeds_without_a_freeze_file(
    harness,
) -> None:
    go, out = harness
    fake = FakeRunner()
    assert (
        go(fake, "dev-run", "--out", str(out), "--cases", "P4", "--seeds", "11", "--workers", "1")
        == 0
    )
    assert len(fake.calls) == 4  # conditions a and c, two directions
    meta = json.loads((out / "a_P4_ccw_11.json").read_text())["battery_b8"]
    assert meta["mode"] == "development" and meta["freeze_commit"] is None
    assert "A5: PASS" in bb.dev_report_text(out) or "A5: FAIL" in bb.dev_report_text(out)
    bb.dev_report_cmd(argparse.Namespace(out=str(out)))
    assert (out / "dev_report.md").exists()


def test_dev_runs_can_stop_after_fewer_laps_and_battery_runs_cannot(harness) -> None:
    go, out = harness
    fake = FakeRunner(laps={"a_P4_ccw_11": 4, "a_P4_cw_11": 4})
    argv = ["dev-run", "--out", str(out / "d"), "--cases", "P4", "--seeds", "11"]
    argv += ["--conditions", "a", "--laps", "4", "--workers", "1"]
    assert go(fake, *argv) == 0
    assert [parse(c).stop_after_laps for c in fake.calls] == [4, 4]
    battery = FakeRunner()
    go(battery, "run", "--out", str(out / "b"), "--workers", "1")
    assert {parse(c).stop_after_laps for c in battery.calls} == {13}


@pytest.mark.parametrize(
    ("cases", "seeds"),
    [("R1", "11"), ("P4", "61"), ("Q1", "62"), ("P4", "51"), ("OD", "31"), ("P1", "11")],
)
def test_dev_runs_refuse_a_b8_run_or_another_attempts_seed(harness, cases: str, seeds: str) -> None:
    go, out = harness
    fake = FakeRunner()
    assert go(fake, "dev-run", "--out", str(out), "--cases", cases, "--seeds", seeds) == 2
    assert fake.calls == []


def test_the_dev_report_judges_the_count_rule_and_the_class_tolerance(harness) -> None:
    go, out = harness
    fake = FakeRunner()
    go(fake, "dev-run", "--out", str(out), "--cases", "P4,ND", "--seeds", "11", "--workers", "1")
    text = bb.dev_report_text(out)
    assert "A5: PASS" in text and "count rule broken in 0" in text
    # a run that lost a lap fails A5
    path = out / "a_P4_ccw_11.json"
    doc = json.loads(path.read_text())
    doc["system_lap_events"] = doc["system_lap_events"][:11]
    path.write_text(json.dumps(doc))
    assert "A5: FAIL" in bb.dev_report_text(out)


# -- reported without a threshold -----------------------------------------------------------------------------


def test_the_case_table_sums_line_sources_and_averages_the_rest() -> None:
    def doc(sources: list[str], share: float, fixes: float, stale: int) -> dict:
        return {
            "lap_status": {
                "lap_line_sources": sources,
                "start_alias_share_last_lap": share,
                "ring": {"fixes_last_lap": fixes},
                "stale_odometry_scans": stale,
                "on_scan_ms_mean": 4.0,
                "on_scan_ms_p99": 9.0 + stale,
            }
        }

    results = {
        "c_R3_ccw_61": doc(["winding", "anchor", "anchor"], 0.5, 2, 1),
        "c_R3_cw_61": doc(["winding", "anchor"], 0.7, 4, 2),
        "a_Q1_ccw_61": doc(["start_scan"] * 3, 0.0, 5, 0),
    }
    rows = {(r["case"], r["condition"]): r for r in b8.case_rows(results, CONST)}
    r3 = rows[("R3", "c")]
    assert r3["sources"] == {"anchor": 3, "winding": 2} and r3["runs"] == 2
    assert r3["alias_share_mean"] == pytest.approx(0.6) and r3["fixes_last_lap_mean"] == 3.0
    assert r3["stale_scans"] == 3 and r3["on_scan_ms_p99_max"] == 11.0
    assert rows[("Q1", "a")]["class"] == "placed"
    assert "R3" in b8.case_table(list(rows.values()))


def test_crossing_12_offset_as_a_share_of_a_lap() -> None:
    crossings = [100.0 + 10.0 * (k + 1) for k in range(13)]
    events = [{"t_cross_s": c + 0.5, "lap_time_s": 10.0} for c in crossings]
    rows = b8.crossing12_rows(
        {
            "c_R1_ccw_61": {
                "gt_lap_crossings_registered_s": crossings,
                "gt_lap_times_registered_s": [10.0] * 13,
                "system_lap_events": events,
            },
            "c_R1_cw_61": {},
        },
        CONST,
    )
    assert rows[0]["crossing12_offset_s"] == pytest.approx(0.5)
    assert rows[0]["share_of_lap"] == pytest.approx(0.05)
    assert math.isnan(rows[1]["crossing12_offset_s"])
    assert "+0.50" in b8.crossing12_table(rows)


def test_condition_b_laps_are_judged_against_the_same_tolerances() -> None:
    runs = full()
    runs = replace(runs, "b", ("Q1", "ccw", 61), system=[LAP_S + 0.5] + [LAP_S] * 12)
    info = b8.condition_b_laps(runs, CONST)
    assert info["runs"] == 36 and info["laps"] == 36 * 12 and info["within"] == 36 * 12 - 1


def test_the_stall_probe_rd_is_a_ring_with_one_box_close_to_r3_and_not_r3() -> None:
    rd, r3 = CONST.development_cases["RD"], CONST.battery_cases["R3"]
    assert rd != r3 and CONST.class_of("RD") == "aliased"
    assert rd["lane_width_m"] == r3["lane_width_m"] == 1.2
    assert rd["obstacles"][0]["at"] == 0.35 and rd["obstacles"][0]["protrusion_m"] == 0.20
    assert "RD" not in CONST.battery_cases


def test_a5_applies_the_drop_rule_to_development_cases_and_the_probe_is_not_judged(
    harness,
) -> None:
    """Owner decision 14: a development case whose condition a run of the first seed cannot
    complete 3 laps in either direction is dropped and A5 judges the rest; RD is only reported."""
    go, out = harness
    fake = FakeRunner(laps={"a_OD_cw_11": 1, "a_RD_ccw_11": 2, "c_RD_cw_11": 1})
    argv = ["--cases", "P4,OD,RD", "--seeds", "11", "--workers", "1"]
    assert go(fake, "dev-run", "--out", str(out), *argv) == 0
    text = bb.dev_report_text(out)
    assert "dropped {'OD': " in text and "a_OD_cw_11: 1 laps" in text
    assert "A5: PASS" in text  # the dropped OD runs and the stalled RD runs do not count
    assert "Stall probe RD" in text and "RD runs 4, stalled or short 2" in text
    assert "Dropped by the rule" in text


def test_a_stalled_judged_case_that_is_not_dropped_still_fails_a5(harness) -> None:
    go, out = harness
    fake = FakeRunner(laps={"c_P4_cw_11": 5})  # condition c stalls: not dropped by the rule
    assert go(fake, "dev-run", "--out", str(out), "--cases", "P4", "--seeds", "11") == 0
    assert "A5: FAIL" in bb.dev_report_text(out)
