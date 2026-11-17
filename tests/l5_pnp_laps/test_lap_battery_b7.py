"""Battery B7 (v0.5 lap pre-registration sections 2 and 3): the constants file check, the decisions
D7-S0 and D7-S1 table-driven per item (pass, at threshold, epsilon over, NaN, an invalid run, a
dropped case turning 22 of 24 into 16 of 18), the freeze-file guard that keeps every case with seed
51, 52 or 53 from running before the freeze commit, and the run machinery with a fake runner.

Every boundary of every threshold is tested from both sides, because the thresholds are the
pre-registration and a decision that flips at the wrong number cannot be found after the battery has
run. No ROS and no simulator here."""

from __future__ import annotations

import json
import math
import subprocess
import sys
from pathlib import Path

import battery_b7 as bb
import lap_battery_b7 as b7
import pytest
from lap_battery import RunSummary

HERE = Path(__file__).resolve().parent
sys.path.insert(0, str(HERE.parent / "l5_reactive_bench"))

CASES_24 = ("P1", "P2", "P3", "P4")
DIRS = ("ccw", "cw")
SEEDS = (51, 52, 53)
CONST = b7.load_constants()
FROZEN = "f" * 40
LATE = {"a": 27.0, "b": 12.0, "c": 12.0}


def mk(
    condition: str,
    case: str = "P1",
    direction: str = "ccw",
    seed: int = 51,
    laps: int = 12,
    t1: float = 27.0,
    late: float = 12.0,
    contacts: int = 0,
    system: list[float] | None = None,
    error: str = "",
) -> RunSummary:
    times = [t1] + [late] * (laps - 1)
    return RunSummary(
        condition=condition,
        case=case,
        direction=direction,
        seed=seed,
        gt_lap_times_s=tuple(times),
        system_lap_times_s=tuple(times if system is None else system),
        contacts=contacts,
        brakes_per_lap=tuple([0] * laps),
        error=error,
    )


def keys(cases=CASES_24):
    return [(c, d, s) for c in cases for d in DIRS for s in SEEDS]


def full(cases=CASES_24, **override) -> list[RunSummary]:
    runs = []
    for cond in b7.B7_CONDITIONS:
        for case, d, s in keys(cases):
            kw = {"t1": 27.0, "late": LATE[cond]}
            o = override.get(cond, {})
            kw.update(o(case, d, s) if callable(o) else o)
            runs.append(mk(cond, case, d, s, **kw))
    return runs


def replace(runs, condition, key, **kw):
    out = []
    for r in runs:
        if r.condition == condition and r.key == key:
            base = {"t1": r.gt_lap_times_s[0], "late": r.gt_lap_times_s[-1], "laps": r.laps}
            base.update(kw)
            r = mk(condition, *key, **base)
        out.append(r)
    return out


def failed(decision, prefix: str) -> list[str]:
    return [k for k, (ok, _) in decision.items.items() if k.startswith(prefix) and not ok]


# -- the constants file ----------------------------------------------------------------------------


def test_the_yaml_hash_is_the_registered_constant_and_the_committed_file() -> None:
    assert b7.REGISTERED_YAML_SHA256 == b7.sha256_file(b7.PREREG_YAML)
    assert CONST.yaml_sha256 == b7.REGISTERED_YAML_SHA256


def test_thresholds_are_the_preregistered_numbers() -> None:
    th = CONST.th
    assert th["lap_time_tolerance_s"] == 0.3 and th["s0b_min_share_within_tolerance"] == 0.98
    assert th["s0b_max_single_error_s"] == 1.0 and th["max_contacts"] == 0
    assert (th["required_complete_of_24"], th["required_complete_of_18"]) == (22, 16)
    assert th["s1_reach_max_ratio_to_b"] == 1.05 and th["s1_max_median_r"] == 0.80
    assert CONST.battery_seeds == (51, 52, 53) and CONST.laps == 12
    assert CONST.lap1_cap_mps == 0.5 and CONST.directions == ("ccw", "cw")
    assert set(CONST.battery_cases) == set(CASES_24)
    assert set(CONST.conditions) == {"a", "b", "c"}
    assert CONST.drop_rule == {
        "condition": "a",
        "seed": 51,
        "min_laps": 3,
        "either_direction": True,
    }
    assert CONST.validity["max_scan_wait_s"] == 0.25 and CONST.validity["max_attempts"] == 3


def test_a_changed_yaml_is_refused(tmp_path: Path) -> None:
    changed = tmp_path / "pnp-v0.5-laps.yaml"
    changed.write_text(b7.PREREG_YAML.read_text().replace("laps: 12", "laps: 13"))
    with pytest.raises(b7.PreregError, match="SHA-256"):
        b7.load_constants(changed)
    ok = b7.load_constants(changed, expected_sha=b7.sha256_file(changed))
    assert ok.laps == 13


@pytest.mark.parametrize(
    ("old", "new"),
    [
        ("s1_reach_min_runs: {of_24: 22, of_18: 16}", "s1_reach_min_runs: {of_24: 21, of_18: 16}"),
        ("s1_complete_laps: 10", "s1_complete_laps: 9"),
        ("late_laps: [8, 9, 10]", "late_laps: [7, 8, 9]"),
        ("seeds: [51, 52, 53]", "seeds: [51, 52, 54]"),
    ],
)
def test_a_yaml_the_d3_functions_cannot_apply_as_written_is_refused(
    tmp_path: Path, old: str, new: str
) -> None:
    text = b7.PREREG_YAML.read_text()
    assert old in text
    changed = tmp_path / "x.yaml"
    changed.write_text(text.replace(old, new))
    with pytest.raises(b7.PreregError):
        b7.load_constants(changed, expected_sha=b7.sha256_file(changed))


def test_required_complete_has_only_the_two_registered_sizes() -> None:
    assert CONST.required_complete(24) == 22 and CONST.required_complete(18) == 16
    for n in (0, 12, 21, 23, 25):
        with pytest.raises(b7.PreregError):
            CONST.required_complete(n)


# -- run files -------------------------------------------------------------------------------------


def result_json(**kw) -> dict:
    base = {
        "gt_lap_times_registered_s": [31.9, 10.0],
        "gt_lap_times_by_crossing_s": [31.9, 10.0],
        "truth_valid": True,
        "system_lap_events": [{"lap_time_s": 32.0}, {"lap_time_s": 10.1}],
        "wall_contacts": 0,
    }
    base.update(kw)
    return base


def test_a_result_without_the_registered_truth_or_its_validity_is_refused() -> None:
    for drop in ("gt_lap_times_registered_s", "truth_valid"):
        body = result_json()
        del body[drop]
        with pytest.raises(b7.RunFileError, match="brief 48"):
            b7.summarise_run(body, "c", "P1", "ccw", 51)
    s = b7.summarise_run(result_json(), "c", "P1", "ccw", 51)
    assert s.gt_lap_times_s == (31.9, 10.0) and s.error == ""


def test_an_invalid_truth_is_a_run_that_failed_to_execute() -> None:
    body = result_json(truth_valid=False, truth_invalid_reason="reset_after_driving")
    s = b7.summarise_run(body, "a", "P2", "cw", 52)
    assert "reset_after_driving" in s.error


def test_load_runs_demands_the_mode_block(tmp_path: Path) -> None:
    (tmp_path / "c_P1_ccw_51.json").write_text(
        json.dumps(result_json() | {"battery_b7": {"mode": "battery"}})
    )
    (tmp_path / "drops.json").write_text("{}")
    assert [(r.condition, r.case) for r in b7.load_runs(tmp_path)] == [("c", "P1")]
    (tmp_path / "a_P1_ccw_51.json").write_text(json.dumps(result_json()))
    with pytest.raises(b7.RunFileError):
        b7.load_runs(tmp_path)


# -- D7-S0 -----------------------------------------------------------------------------------------


def test_all_pass_and_the_decisions_are_named_d7() -> None:
    runs = full()
    s0, s1 = b7.d7_s0(runs, CONST), b7.d7_s1(runs, CONST)
    assert s0.passed and s1.passed
    assert s0.name == "D7-S0" and s1.name == "D7-S1"
    assert [k.split(" ")[0] for k in s1.items] == ["1.", "2.", "3.", "4a.", "4b."]


def test_d7_s0_has_no_d_or_e_items() -> None:
    d = b7.d7_s0(full(), CONST)
    assert not [k for k in d.items if "(d)" in k or "(e)" in k]
    assert [k.split(":")[0] for k in d.items].count("S0b(c)") == 4


def test_s0a_a_lap_0_31_s_off_fails_0_30_and_0_29_pass() -> None:
    # lap 1 truth 2.0 s: 2.3 - 2.0 is 0.2999999999999998 in floating point, the threshold itself
    for off, ok in ((0.31, False), (0.3 + 1e-6, False), (0.30, True), (0.29, True)):
        runs = replace(full(), "a", ("P2", "cw", 52), t1=2.0, system=[2.0 + off] + [27.0] * 11)
        d = b7.d7_s0(runs, CONST)
        assert d.passed == ok, (off, str(d))
        assert bool(failed(d, "S0a")) == (not ok)


def test_s0a_a_wrong_lap_count_and_a_nan_lap_fail() -> None:
    bad = replace(full(), "a", ("P1", "ccw", 51), system=[27.0] * 11)
    d = b7.d7_s0(bad, CONST)
    assert failed(d, "S0a: system lap count") and not failed(d, "S0a: every system lap")
    nan = replace(full(), "a", ("P1", "ccw", 51), system=[math.nan] + [27.0] * 11)
    d = b7.d7_s0(nan, CONST)
    assert failed(d, "S0a: every system lap time") and not failed(d, "S0a: system lap count")


def test_s0b_share_boundary_283_of_288_passes_282_fails() -> None:
    def runs(outside: int):
        keyed = keys()[:outside]
        return full(
            c=lambda case, d, s: (
                {"system": [27.0 + 0.4] + [12.0] * 11} if (case, d, s) in keyed else {}
            )
        )

    assert b7.d7_s0(runs(5), CONST).passed  # 283 of 288 = 0.9826
    d = b7.d7_s0(runs(6), CONST)  # 282 of 288 = 0.9792
    assert not d.passed and failed(d, "S0b(c): at least")


def test_s0b_lap_1_is_inside_the_item_with_no_allowance() -> None:
    """The B4 failure: a lap 1 that is 2.3 s off fails the 1.0 s item and counts in the share."""
    bad = replace(full(), "c", ("P3", "cw", 53), system=[27.0 + 2.3] + [12.0] * 11)
    d = b7.d7_s0(bad, CONST)
    assert not d.passed and failed(d, "S0b(c): no lap off by more than")
    assert not failed(d, "S0b(c): at least")  # 287 of 288 is within the share


def test_s0b_one_lap_1_0_off_passes_1_01_fails() -> None:
    edge = replace(full(), "c", ("P3", "cw", 53), system=[27.0 + 1.0] + [12.0] * 11)
    assert not failed(b7.d7_s0(edge, CONST), "S0b(c): no lap off")
    over = replace(full(), "c", ("P3", "cw", 53), system=[27.0 + 1.01] + [12.0] * 11)
    assert failed(b7.d7_s0(over, CONST), "S0b(c): no lap off")


def test_s0b_a_lap_count_difference_fails() -> None:
    bad = replace(full(), "c", ("P1", "cw", 52), system=[27.0] + [12.0] * 10)
    assert failed(b7.d7_s0(bad, CONST), "S0b(c): lap count")


def test_an_invalid_or_failed_run_is_an_item_not_a_skip() -> None:
    bad = replace(full(), "c", ("P1", "ccw", 51), error="truth invalid: no_scan_at_start")
    assert failed(b7.d7_s0(bad, CONST), "S0b(c): no run failed")
    bad_a = replace(full(), "a", ("P4", "cw", 53), error="node died")
    assert failed(b7.d7_s0(bad_a, CONST), "S0a: no condition (a) run failed")


def test_s0_needs_conditions_a_and_c() -> None:
    with pytest.raises(b7.MissingPairError):
        b7.d7_s0([r for r in full() if r.condition != "a"], CONST)
    with pytest.raises(b7.MissingPairError):
        b7.d7_s0([r for r in full() if r.condition != "c"], CONST)


# -- D7-S1 -----------------------------------------------------------------------------------------


def test_s1_contact_fails_item_1() -> None:
    d = b7.d7_s1(replace(full(), "c", ("P2", "ccw", 52), contacts=1), CONST)
    assert not d.passed and failed(d, "1.")
    assert b7.d7_s1(replace(full(), "b", ("P2", "ccw", 52), contacts=1), CONST).passed


def test_s1_21_of_24_complete_fails_22_passes() -> None:
    def short(n):
        ks = keys()[:n]
        return full(c=lambda case, d, s: {"laps": 9} if (case, d, s) in ks else {})

    assert failed(b7.d7_s1(short(3), CONST), "2.")
    d2 = b7.d7_s1(short(2), CONST)
    assert not failed(d2, "2.") and not failed(d2, "3.")


def test_s1_reach_1_05_passes_1_05_plus_epsilon_fails() -> None:
    edge = full(c={"late": 12.0 * 1.05 - 1e-9})
    assert not failed(b7.d7_s1(edge, CONST), "3.")
    over = full(c={"late": 12.0 * 1.05 + 1e-6})
    assert failed(b7.d7_s1(over, CONST), "3.")

    def slow(n):
        ks = keys()[:n]
        return full(c=lambda case, d, s: {"late": 12.0 * 1.06} if (case, d, s) in ks else {})

    assert failed(b7.d7_s1(slow(3), CONST), "3.")
    assert not failed(b7.d7_s1(slow(2), CONST), "3.")


def test_s1_a_nan_t_late_is_not_reached_and_not_complete() -> None:
    runs = replace(full(), "c", ("P1", "ccw", 51), laps=9)
    d = b7.d7_s1(runs, CONST)
    assert not failed(d, "2.") and not failed(d, "3.")  # 23 of 24
    runs = replace(runs, "c", ("P1", "ccw", 52), laps=9)
    runs = replace(runs, "c", ("P1", "ccw", 53), laps=9)
    assert failed(b7.d7_s1(runs, CONST), "2.")


def test_s1_median_r_0_81_fails_0_80_passes() -> None:
    d81 = b7.d7_s1(full(b={"late": 90.0}, c={"t1": 100.0, "late": 81.0}), CONST)
    assert failed(d81, "4b.")
    d80 = b7.d7_s1(full(b={"late": 90.0}, c={"t1": 100.0, "late": 80.0}), CONST)
    assert not failed(d80, "4b.") and d80.passed


def test_s1_a_t_late_not_below_t1_fails_4a() -> None:
    bad = replace(full(b={"late": 30.0}), "c", ("P1", "ccw", 51), t1=11.0, late=12.0)
    assert failed(b7.d7_s1(bad, CONST), "4a.")


def test_s1_an_invalid_run_does_not_complete() -> None:
    bad = replace(full(), "c", ("P1", "ccw", 51), error="truth invalid: reset_after_driving")
    assert b7.d7_s1(bad, CONST).passed  # 23 of 24 still reach the 22 needed
    worse = bad
    for key in (("P1", "ccw", 52), ("P1", "ccw", 53)):
        worse = replace(worse, "c", key, error="truth invalid: no_scan_at_start")
    assert failed(b7.d7_s1(worse, CONST), "2.")


def test_s1_a_missing_paired_b_run_raises() -> None:
    runs = [r for r in full() if not (r.condition == "b" and r.key == ("P3", "cw", 53))]
    with pytest.raises(b7.MissingPairError):
        b7.d7_s1(runs, CONST)


# -- a dropped case ----------------------------------------------------------------------------------


def test_a_dropped_case_turns_22_of_24_into_16_of_18() -> None:
    cases = ("P1", "P2", "P4")  # P3 dropped
    assert b7.d7_s0(full(cases), CONST).passed and b7.d7_s1(full(cases), CONST).passed

    def short(n):
        ks = keys(cases)[:n]
        return full(cases, c=lambda case, d, s: {"laps": 9} if (case, d, s) in ks else {})

    assert not failed(b7.d7_s1(short(2), CONST), "2.")  # 16 of 18
    assert failed(b7.d7_s1(short(3), CONST), "2.")  # 15 of 18
    # and 22 of 24 does not stand in for it: 19 of 24 is not 16 of 18, 21 of 24 is not 22 of 24
    assert failed(b7.d7_s1(replace_many_short(24, 3), CONST), "2.")


def replace_many_short(_n: int, k: int) -> list[RunSummary]:
    ks = keys()[:k]
    return full(c=lambda case, d, s: {"laps": 9} if (case, d, s) in ks else {})


def test_two_dropped_cases_have_no_registered_numbers() -> None:
    with pytest.raises(b7.PreregError):
        b7.d7_s1(full(("P1", "P2")), CONST)


# -- reported without a threshold --------------------------------------------------------------------


def test_the_lap_1_table_and_the_missed_flag_counts() -> None:
    results = {
        "c_P1_ccw_51": result_json(lap_status={"launch_dwell_s": 0.31}),
        "a_P1_ccw_51": result_json(truth_valid=True) | {"lap_status": {}},
    }
    rows = b7.lap1_rows(results)
    assert rows[0]["run"] == "a_P1_ccw_51" and math.isnan(rows[0]["launch_dwell_s"])
    assert rows[1]["system_lap1_s"] == 32.0 and rows[1]["truth_lap1_s"] == 31.9
    assert rows[1]["difference_s"] == pytest.approx(0.1) and rows[1]["launch_dwell_s"] == 0.31
    table = b7.lap1_table(rows)
    assert "| c_P1_ccw_51 | 32.00 | 31.90 | +0.10 | 0.31 | True |" in table
    flags = b7.missed_flags_per_condition({"a_P1_ccw_51": 0, "c_P1_ccw_51": 2, "c_P1_cw_51": 0})
    assert flags == [
        "condition a: 0 laps flagged missed gate in 0 of 1 runs",
        "condition b: 0 laps flagged missed gate in 0 of 0 runs",
        "condition c: 2 laps flagged missed gate in 1 of 2 runs",
    ]


# -- the freeze guard --------------------------------------------------------------------------------


def write_freeze(path: Path, **kw) -> Path:
    doc = {"freeze_commit": FROZEN, "yaml_sha256": b7.REGISTERED_YAML_SHA256} | kw
    path.write_text(json.dumps(doc))
    return path


def build(freeze: Path, condition="c", case="P1", direction="ccw", seed=51, tmp=Path("/tmp")):
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


@pytest.mark.parametrize("case", CASES_24)
@pytest.mark.parametrize("seed", SEEDS)
@pytest.mark.parametrize("condition", ["a", "b", "c"])
def test_no_b7_case_can_run_without_the_freeze_file(
    tmp_path: Path, case: str, seed: int, condition: str, monkeypatch: pytest.MonkeyPatch
) -> None:
    monkeypatch.setenv(bb.GIT_SHA_ENV, "a" * 40)  # no git here: the guard's file checks only
    missing = tmp_path / "freeze" / "b7.json"
    for direction in DIRS:
        with pytest.raises(bb.BatteryError, match="freeze"):
            build(missing, condition, case, direction, seed, tmp_path)
    ok = write_freeze(tmp_path / "b7.json")
    for direction in DIRS:
        command = build(ok, condition, case, direction, seed, tmp_path)
        assert ("--reverse" in command) == (direction == "cw")


def test_the_freeze_file_must_name_a_commit_and_the_registered_yaml_hash(
    tmp_path: Path, monkeypatch: pytest.MonkeyPatch
) -> None:
    monkeypatch.setenv(bb.GIT_SHA_ENV, "a" * 40)
    bad_docs = [
        {"freeze_commit": FROZEN, "yaml_sha256": "0" * 64},
        {"freeze_commit": FROZEN},
        {"yaml_sha256": b7.REGISTERED_YAML_SHA256},
        {"freeze_commit": "abc", "yaml_sha256": b7.REGISTERED_YAML_SHA256},
        {"freeze_commit": "G" * 40, "yaml_sha256": b7.REGISTERED_YAML_SHA256},
    ]
    for doc in bad_docs:
        path = tmp_path / "b7.json"
        path.write_text(json.dumps(doc))
        with pytest.raises(bb.BatteryError):
            build(path, tmp=tmp_path)
    path = tmp_path / "b7.json"
    path.write_text("{not json")
    with pytest.raises(bb.BatteryError, match="not valid JSON"):
        build(path, tmp=tmp_path)


def test_the_freeze_commit_must_be_an_ancestor_and_the_code_unchanged(
    tmp_path: Path, monkeypatch: pytest.MonkeyPatch
) -> None:
    monkeypatch.delenv(bb.GIT_SHA_ENV, raising=False)
    freeze = write_freeze(tmp_path / "b7.json")
    answers = {"merge-base": 1, "diff": 0}
    diff_out = {"text": ""}

    def fake_git(*args: str):
        key = "merge-base" if args[0] == "merge-base" else "diff"
        return subprocess.CompletedProcess(
            args, answers[key], stdout=diff_out["text"] if key == "diff" else ""
        )

    monkeypatch.setattr(bb, "_git", fake_git)
    with pytest.raises(bb.BatteryError, match="not an ancestor"):
        bb.require_freeze(freeze)
    answers["merge-base"] = 0
    assert bb.require_freeze(freeze)["freeze_commit"] == FROZEN
    diff_out["text"] = "ros_ws/src/racer_laps/src/racer_laps/gate.py\n"
    with pytest.raises(bb.BatteryError, match="changed since the freeze"):
        bb.require_freeze(freeze)
    # a caller that states its SHA (the container has no git) is trusted for the git part
    monkeypatch.setenv(bb.GIT_SHA_ENV, "a" * 40)
    assert bb.require_freeze(freeze)["freeze_commit"] == FROZEN


def test_only_the_yamls_cases_seeds_directions_and_conditions_can_be_built(tmp_path: Path) -> None:
    freeze = write_freeze(tmp_path / "b7.json")
    for kwargs in (
        {"case": "N1"},  # a B4 case
        {"case": "P9"},
        {"seed": 21},  # a B4 seed
        {"seed": 54},
        {"direction": "up"},
        {"condition": "d"},
    ):
        import os

        os.environ[bb.GIT_SHA_ENV] = "a" * 40
        try:
            with pytest.raises(bb.BatteryError):
                build(freeze, tmp=tmp_path, **kwargs)
        finally:
            del os.environ[bb.GIT_SHA_ENV]


# -- the commands against the runner's own parser -----------------------------------------------------


def parse(command: list[str]):
    import track_family_runner as runner

    return runner.build_parser().parse_args(command[2:])


def test_the_conditions_map_to_the_registered_runner_flags(
    tmp_path: Path, monkeypatch: pytest.MonkeyPatch
) -> None:
    monkeypatch.setenv(bb.GIT_SHA_ENV, "a" * 40)
    freeze = write_freeze(tmp_path / "b7.json")
    ceiling = bb.b4t.ceiling_mps(bb.b4t.DEFAULT_BUNDLE)
    a = parse(build(freeze, "a", tmp=tmp_path))
    assert (a.lap_memory, a.v_max_mps) == ("off", 0.5)
    b = parse(build(freeze, "b", tmp=tmp_path))
    assert (b.lap_memory, b.v_max_mps) == ("off", ceiling)
    c = parse(build(freeze, "c", tmp=tmp_path))
    assert (c.lap_memory, c.v_max_mps, c.v_lap1_mps) == ("v0", ceiling, 0.5)
    assert c.stop_after_laps == 12 and c.driver == "pnp" and c.lap_source == "start_scan"
    short = parse(
        bb.command_for(
            CONST,
            "a",
            "P1",
            "ccw",
            51,
            120,
            tmp_path,
            tmp_path / "r.json",
            bb.b4t.DEFAULT_BUNDLE,
            Path("/data"),
            laps=3,
            freeze_path=freeze,
        )
    )
    assert short.stop_after_laps == 3
    p1 = json.loads(c.track_params_json)
    assert (
        p1
        == {
            "lane_width_m": 1.2,
            "straight_length_m": 3.5,
            "island_radius_m": 0.5,
        }
        and c.track == "stadium"
    )
    p2 = parse(build(freeze, "c", "P2", tmp=tmp_path))
    assert p2.track == "rounded_rect" and json.loads(p2.track_params_json)["island_width_m"] == 1.8


# -- the run machinery with a fake runner ---------------------------------------------------------------


class FakeRunner:
    """Stands in for subprocess.run: parses the argv with the runner's own parser and writes a
    result shaped like the runner's, with the registered truth fields."""

    def __init__(self, gaps=None, invalid=None, die=None, laps=None):
        self.calls: list[list[str]] = []
        self.gaps = gaps or {}  # name -> max /drive_raw gap per attempt
        self.invalid = invalid or {}  # name -> list of reasons ("" = valid) per attempt
        self.die = die or set()
        self.laps = laps or {}  # name -> truth laps
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
        n = self.laps.get(name, 12)
        times = [20.0] + [10.0] * (n - 1)
        result = {
            "gt_lap_times_by_crossing_s": times,
            "gt_lap_times_registered_s": times,
            "truth_valid": reason == "",
            "truth_invalid_reason": reason,
            "system_lap_events": [{"lap_time_s": t} for t in times],
            "wall_contacts": 0,
            "brake_per_lap": [{} for _ in times],
            "drive_raw": {"max_gap_s": gap},
            "real_time_factor": 1.0,
            "lap_status": {"launch_dwell_s": 0.3},
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
    freeze = write_freeze(tmp_path / "b7.json")

    def go(fake, *argv, freeze_path=freeze):
        monkeypatch.setattr(bb.subprocess, "run", fake)
        return bb.main([*argv, "--freeze-path", str(freeze_path), "--data-root", "/data"])

    return go, tmp_path / "out"


def test_no_runner_is_called_without_the_freeze_file(harness, tmp_path: Path) -> None:
    go, out = harness
    fake = FakeRunner()
    assert go(fake, "run", "--out", str(out), freeze_path=tmp_path / "nope.json") == 2
    assert fake.calls == [] and not out.exists() or not list(out.glob("*.json"))


def test_the_seed_51_condition_a_runs_come_first_and_the_battery_runs_to_completion(
    harness,
) -> None:
    go, out = harness
    fake = FakeRunner()
    assert go(fake, "run", "--out", str(out), "--workers", "1") == 0
    assert len(fake.calls) == 72  # 3 conditions x 4 cases x 2 directions x 3 seeds
    first = [Path(c[c.index("--work-dir") + 1]).name for c in fake.calls[:8]]
    assert all(n.startswith("a_") and "_51.attempt" in n for n in first)
    drops = json.loads((out / "drops.json").read_text())
    assert drops["dropped"] == [] and len(drops["laps"]) == 8
    meta = json.loads((out / "c_P2_cw_52.json").read_text())["battery_b7"]
    assert meta["mode"] == "battery" and meta["git_sha"] == "a" * 40
    assert meta["freeze_commit"] == FROZEN and meta["prereg_yaml_sha256"] == CONST.yaml_sha256
    assert meta["accepted_attempt"] == 1 and meta["attempts"][0]["starved"] is False
    report = bb.report_text(out)
    assert "D7-S0: PASS" in report and "D7-S1: PASS" in report
    assert "D v0 may drive" in report and "launch dwell s" in report
    assert "condition c: 24 laps flagged missed gate in 24 of 24 runs" in report
    again = FakeRunner()
    assert go(again, "run", "--out", str(out), "--workers", "1") == 0
    assert again.calls == []  # resumed: finished names are skipped


def test_a_case_whose_seed_51_condition_a_run_has_under_3_laps_is_dropped_and_never_run(
    harness,
) -> None:
    go, out = harness
    fake = FakeRunner(laps={"a_P3_cw_51": 2})
    assert go(fake, "run", "--out", str(out), "--workers", "1") == 0
    drops = json.loads((out / "drops.json").read_text())
    assert drops["dropped"] == ["P3"] and drops["laps"]["a_P3_cw_51"] == 2
    names = {Path(c[c.index("--work-dir") + 1]).name.split(".")[0] for c in fake.calls}
    p3 = sorted(n for n in names if "_P3_" in n)
    assert p3 == ["a_P3_ccw_51", "a_P3_cw_51"]  # only its two seed 51 condition a runs
    assert len(names) == 3 * 18 + 2  # 18 runs per condition, plus the dropped case's 2 on disk
    # 18 runs per condition: 16 of 18 applies, and the report counts it
    report = bb.report_text(out)
    assert "dropped ['P3']" in report and "INCOMPLETE" not in report
    assert "D7-S0: PASS" in report and "D7-S1: PASS" in report
    assert len(b7.load_runs(out)) == 18 * 3 + 2  # the dropped case's two runs are on disk too


def test_two_dropped_cases_stop_the_battery(harness) -> None:
    go, out = harness
    fake = FakeRunner(laps={"a_P3_cw_51": 2, "a_P1_ccw_51": 0})
    assert go(fake, "run", "--out", str(out), "--workers", "1") == 2
    assert len(fake.calls) == 8  # the seed 51 condition a runs only


def test_a_starved_or_invalid_run_is_rerun_alone_and_every_attempt_is_reported(harness) -> None:
    go, out = harness
    fake = FakeRunner(
        gaps={"a_P1_ccw_51": [0.05, 0.02]},
        invalid={
            "a_P1_cw_51": ["truth_starts_after_crossing_0", ""],
            "a_P2_ccw_51": ["reset_after_driving", "no_scan_at_start", "no_scan_at_start"],
        },
    )
    assert go(fake, "run", "--out", str(out), "--workers", "1") == 0
    assert fake.attempts["a_P1_ccw_51"] == 2 and fake.attempts["a_P1_cw_51"] == 2
    assert fake.attempts["a_P2_ccw_51"] == 3
    ok = json.loads((out / "a_P1_cw_51.json").read_text())["battery_b7"]
    assert ok["accepted_attempt"] == 2 and ok["accepted_while_invalid"] is False
    assert [a["invalid_truth"] for a in ok["attempts"]] == [True, False]
    stuck = json.loads((out / "a_P2_ccw_51.json").read_text())["battery_b7"]
    assert stuck["accepted_while_invalid"] is True and len(stuck["attempts"]) == 3
    assert len(list((out / "attempts").glob("a_P2_ccw_51.attempt*.json"))) == 3
    # still invalid after 3 attempts: a run that failed to execute, so D7-S0 fails and says why
    # (a_P2_ccw_51 has no valid laps either, which is also the drop rule: P2 is dropped)
    drops = json.loads((out / "drops.json").read_text())
    assert drops["dropped"] == ["P2"]
    report = bb.report_text(out)
    assert "## Attempts" in report and "a_P2_ccw_51" in report and "a_P1_cw_51" in report


def test_workers_are_capped_at_three(harness) -> None:
    go, out = harness
    fake = FakeRunner()
    assert go(fake, "run", "--out", str(out), "--workers", "4") == 2
    assert fake.calls == []


def test_missed_flags_are_counted_from_the_accepted_attempts_log(tmp_path: Path) -> None:
    log = tmp_path / "work" / "c_P1_ccw_51.attempt2" / "lap_memory_node.log"
    log.parent.mkdir(parents=True)
    log.write_text(
        "lap 1: ok\nlap 2: UNCLEAN (missed gate, not a lap)\nlap 3: UNCLEAN (missed gate, not a lap)\n"
    )
    assert bb.missed_flags(tmp_path, "c_P1_ccw_51", 2) == 2
    assert bb.missed_flags(tmp_path, "c_P1_ccw_51", 1) == 0


def test_a_worker_that_dies_stops_the_battery_instead_of_looping(harness) -> None:
    """A runner call that raises (a renamed flag, a full disk) used to leave the job pending
    forever: the loop ran again and again."""
    go, out = harness

    def boom(command, **kw):
        raise OSError("disk full")

    assert go(boom, "run", "--out", str(out), "--workers", "1") == 2
    assert not list(out.glob("*_*_*_*.json"))
