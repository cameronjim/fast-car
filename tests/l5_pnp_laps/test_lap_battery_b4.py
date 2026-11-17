"""Battery B4 (v0.3 pre-registration section 1): decision functions, the constants file check, the
command builder against the runner's own parser, and the run/resume machinery with a fake runner.

Every boundary of every threshold is tested from both sides, because the thresholds are the
pre-registration and a decision that flips at the wrong number is a bug that cannot be found
after the battery has run. No ROS and no simulator here."""

from __future__ import annotations

import json
import math
import shutil
import subprocess
import sys
from pathlib import Path

import battery_b4 as bb
import lap_battery_b4 as b4
import pytest
from lap_battery import RunSummary

HERE = Path(__file__).resolve().parent
sys.path.insert(0, str(HERE.parent / "l5_reactive_bench"))

CASES_24 = ("N1", "N2", "N3", "N4")
DIRS = ("ccw", "cw")
SEEDS = (21, 22, 23)

CONST = b4.load_constants()


def mk(
    condition: str,
    case: str = "N1",
    direction: str = "ccw",
    seed: int = 21,
    laps: int = 12,
    t1: float = 27.0,
    late: float = 12.0,
    contacts: int = 0,
    system: list[float] | None = None,
    brakes: list[int] | None = None,
    error: str = "",
    **extras: object,
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
        brakes_per_lap=tuple([0] * laps if brakes is None else brakes),
        error=error,
        extras={
            "zones_created": 4,
            "caution_zones": 3,
            "floor_messages": 10,
            "floor_active_messages": 1,
            **extras,
        },
    )


LATE = {"a": 27.0, "b": 12.0, "c": 12.0, "d": 11.0, "e": 11.0}


def keys(cases=CASES_24):
    return [(c, d, s) for c in cases for d in DIRS for s in SEEDS]


def full(cases=CASES_24, **override) -> list[RunSummary]:
    """A passing battery: every condition, every key. `override` maps a condition to kwargs for
    mk, or to a callable (key) -> kwargs."""
    runs = []
    for cond in b4.CONDITIONS:
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
            base = {
                "t1": r.gt_lap_times_s[0],
                "late": r.gt_lap_times_s[-1],
                "laps": r.laps,
            }
            base.update(kw)
            r = mk(condition, *key, **base)
        out.append(r)
    return out


def failed(decision, prefix: str) -> list[str]:
    return [k for k, (ok, _) in decision.items.items() if k.startswith(prefix) and not ok]


# -- the constants file ----------------------------------------------------------------------------


def test_the_yaml_hash_is_the_registered_one() -> None:
    assert CONST.yaml_sha256 == "4415b3719a29cfb08d5f5242ac6cdc5b35fee87e132c6d2939dfa0758808d7a1"
    assert b4.registered_sha() == CONST.yaml_sha256


def test_thresholds_are_the_preregistered_numbers() -> None:
    th = CONST.th
    assert th["lap_time_tolerance_s"] == 0.3 and th["s0b_min_share_within_tolerance"] == 0.98
    assert th["s0b_max_single_error_s"] == 1.0 and th["max_contacts"] == 0
    assert (th["required_complete_of_24"], th["required_complete_of_18"]) == (22, 16)
    assert th["s1_reach_max_ratio_to_b"] == 1.05 and th["s1_max_median_r"] == 0.80
    assert th["s2_max_median_ratio_to_c"] == 0.95 and th["s2_max_brakes_per_lap_late"] == 0.2
    assert th["v02_ds1_max_r_reported_only"] == 0.80
    assert CONST.battery_seeds == (21, 22, 23) and CONST.laps == 12
    assert CONST.lap1_cap_mps == 0.5 and CONST.directions == ("ccw", "cw")
    assert set(CONST.battery_cases) == {"N1", "N2", "N3", "N4"}
    assert set(CONST.development_cases) == {"T", "T2", "S1", "S2", "S3", "S4"}
    assert CONST.development_seeds == (11, 12, 13)


def test_a_changed_yaml_is_refused(tmp_path: Path) -> None:
    changed = tmp_path / "pnp-v0.3.yaml"
    changed.write_text(b4.PREREG_YAML.read_text().replace("laps: 12", "laps: 13"))
    with pytest.raises(b4.PreregError, match="SHA-256"):
        b4.load_constants(changed)
    # the same text with a note that registers its hash is accepted: the check is the hash
    ok = b4.load_constants(changed, expected_sha=b4.sha256_file(changed))
    assert ok.laps == 13


def test_a_note_without_a_hash_is_refused(tmp_path: Path) -> None:
    note = tmp_path / "note.md"
    note.write_text("no constants here")
    with pytest.raises(b4.PreregError, match="no constants file"):
        b4.load_constants(b4.PREREG_YAML, note)


def test_required_complete_has_only_the_two_registered_sizes() -> None:
    assert CONST.required_complete(24) == 22 and CONST.required_complete(18) == 16
    for n in (0, 12, 21, 23, 25):
        with pytest.raises(b4.PreregError):
            CONST.required_complete(n)


# -- run files -------------------------------------------------------------------------------------


def test_a_run_json_without_the_crossing_truth_is_refused() -> None:
    with pytest.raises(b4.RunFileError, match="gt_lap_times_by_crossing_s"):
        b4.summarise_b4_run({"gt_lap_times_s": [10.0] * 12}, "a", "N1", "ccw", 21)


def test_summarise_reads_the_crossing_truth_and_the_counters() -> None:
    result = {
        "gt_lap_times_by_crossing_s": [20.0, 10.0],
        "gt_lap_times_s": [99.0, 99.0],  # the v0.2 field is not used
        "system_lap_events": [{"lap_time_s": 19.9}, {"lap_time_s": 10.1}],
        "wall_contacts": 1,
        "brake_per_lap": [{"ttc": 1, "ttc_reverse": 1, "watchdog": 5}, {}],
        "lap_status": {"zones_created": 5, "caution_zones": 2},
        "guidance_floor": {"messages_with_stations": 40, "messages_floor_active": 8},
        "drive_raw": {"max_gap_s": 0.03},
    }
    s = b4.summarise_b4_run(result, "d", "N3", "cw", 22)
    assert s.gt_lap_times_s == (20.0, 10.0) and s.system_lap_times_s == (19.9, 10.1)
    assert s.contacts == 1 and s.brakes_per_lap == (2, 0)
    assert s.extras["zones_created"] == 5 and s.extras["floor_active_messages"] == 8


def test_load_b4_runs_demands_the_mode_block(tmp_path: Path) -> None:
    body = {"gt_lap_times_by_crossing_s": [9.0] * 11, "battery_b4": {"mode": "battery"}}
    (tmp_path / "e_N2_cw_22.json").write_text(json.dumps(body))
    (tmp_path / "n4_check.json").write_text("{}")
    runs = b4.load_b4_runs(tmp_path)
    assert [(r.condition, r.case, r.direction, r.seed) for r in runs] == [("e", "N2", "cw", 22)]
    with pytest.raises(b4.RunFileError):
        b4.load_b4_runs(tmp_path, "development")
    (tmp_path / "a_N1_cw_21.json").write_text(json.dumps({"gt_lap_times_by_crossing_s": []}))
    with pytest.raises(b4.RunFileError):
        b4.load_b4_runs(tmp_path)


# -- D3-S0 --------------------------------------------------------------------------------------------


def test_all_pass() -> None:
    rep = b4.build_report(full(), CONST)
    for name, dec in rep.decisions.items():
        assert dec.passed, (name, str(dec))
    assert rep.car == {"confirms": "d", "rejects": "e", "undecided": "v0"}


def test_s0a_a_lap_0_31_s_off_fails_and_0_29_passes() -> None:
    bad = replace(full(), "a", ("N2", "cw", 22), system=[27.0 + 0.31] + [27.0] * 11)
    assert not b4.d3_s0(bad, CONST).passed
    assert failed(b4.d3_s0(bad, CONST), "S0a")
    ok = replace(full(), "a", ("N2", "cw", 22), system=[27.0 + 0.29] + [27.0] * 11)
    assert b4.d3_s0(ok, CONST).passed


def test_s0a_a_wrong_lap_count_fails() -> None:
    bad = replace(full(), "a", ("N1", "ccw", 21), system=[27.0] * 11)
    d = b4.d3_s0(bad, CONST)
    assert not d.passed and failed(d, "S0a: system lap count")


def test_s0b_share_boundary_283_of_288_passes_282_fails() -> None:
    def runs(outside: int):
        # one lap 0.4 s off in each of `outside` different condition c runs
        keyed = keys()[:outside]
        return full(
            c=lambda case, d, s: (
                {"system": [27.0 + 0.4] + [12.0] * 11} if (case, d, s) in keyed else {}
            )
        )

    assert b4.d3_s0(runs(5), CONST).passed  # 283 of 288 = 0.9826
    d = b4.d3_s0(runs(6), CONST)  # 282 of 288 = 0.9792
    assert not d.passed and failed(d, "S0b(c): at least")
    # the d and e conditions are judged on their own
    only_d = full(
        d=lambda case, dd, s: (
            {"system": [27.0 + 0.4] + [12.0] * 11} if (case, dd, s) in keys()[:6] else {}
        )
    )
    assert failed(b4.d3_s0(only_d, CONST), "S0b(d): at least")
    assert not failed(b4.d3_s0(only_d, CONST), "S0b(c)")


def test_s0b_97_percent_fails() -> None:
    keyed = keys()[:8]  # 280 of 288 = 0.972
    runs = full(
        c=lambda case, d, s: {"system": [27.4] + [12.0] * 11} if (case, d, s) in keyed else {}
    )
    assert failed(b4.d3_s0(runs, CONST), "S0b(c): at least")


def test_s0b_one_lap_1_1_s_off_fails_one_lap_1_0_off_passes() -> None:
    bad = replace(full(), "e", ("N3", "ccw", 23), system=[27.0 + 1.1] + [11.0] * 11)
    d = b4.d3_s0(bad, CONST)
    assert not d.passed and failed(d, "S0b(e): no lap off by more than")
    assert not failed(d, "S0b(e): at least")  # 287 of 288 is within the share
    edge = replace(full(), "e", ("N3", "ccw", 23), system=[27.0 + 1.0] + [11.0] * 11)
    assert not failed(b4.d3_s0(edge, CONST), "S0b(e): no lap off")


def test_s0b_a_lap_count_difference_fails_in_each_condition() -> None:
    for cond in ("c", "d", "e"):
        bad = replace(full(), cond, ("N1", "cw", 22), system=[27.0] + [12.0] * 10)
        assert failed(b4.d3_s0(bad, CONST), f"S0b({cond}): lap count")


def test_a_run_error_is_an_item_not_a_skip() -> None:
    bad = replace(full(), "c", ("N1", "ccw", 21), error="node died")
    assert failed(b4.d3_s0(bad, CONST), "S0b(c): no run failed")


# -- D3-S1 --------------------------------------------------------------------------------------------


def test_s1_contact_fails_item_1() -> None:
    d = b4.d3_s1(replace(full(), "c", ("N2", "ccw", 22), contacts=1), CONST)
    assert not d.passed and failed(d, "1.")
    # a contact in b, d or e is not an S1 item
    assert b4.d3_s1(replace(full(), "b", ("N2", "ccw", 22), contacts=1), CONST).passed


def test_s1_21_of_24_complete_fails_22_passes() -> None:
    def short(n):
        ks = keys()[:n]
        return full(c=lambda case, d, s: {"laps": 9} if (case, d, s) in ks else {})

    assert failed(b4.d3_s1(short(3), CONST), "2.")
    d2 = b4.d3_s1(short(2), CONST)
    assert not failed(d2, "2.")  # 22 of 24 complete
    assert not failed(d2, "3.")  # 22 of 24 reach; a run that did not complete cannot have reached


def test_s1_reach_1_06_in_three_runs_fails_in_two_passes() -> None:
    def slow(n):
        ks = keys()[:n]
        return full(c=lambda case, d, s: {"late": 12.0 * 1.06} if (case, d, s) in ks else {})

    assert failed(b4.d3_s1(slow(3), CONST), "3.")
    assert not failed(b4.d3_s1(slow(2), CONST), "3.")
    edge = full(c={"late": 12.0 * 1.05 - 1e-9})
    assert not failed(b4.d3_s1(edge, CONST), "3.")
    over = full(c={"late": 12.0 * 1.05 + 1e-6})
    assert failed(b4.d3_s1(over, CONST), "3.")


def test_s1_median_r_0_81_fails_0_80_passes() -> None:
    # b is slower than c here so reach holds; r = late / t1
    d81 = b4.d3_s1(full(b={"late": 90.0}, c={"t1": 100.0, "late": 81.0}), CONST)
    assert failed(d81, "4b.")
    d80 = b4.d3_s1(full(b={"late": 90.0}, c={"t1": 100.0, "late": 80.0}), CONST)
    assert not failed(d80, "4b.") and d80.passed


def test_s1_a_run_whose_t_late_is_not_below_t1_fails_4a() -> None:
    bad = replace(full(b={"late": 30.0}), "c", ("N1", "ccw", 21), t1=11.0, late=12.0)
    assert failed(b4.d3_s1(bad, CONST), "4a.")


def test_s1_a_missing_paired_b_run_raises() -> None:
    runs = [r for r in full() if not (r.condition == "b" and r.key == ("N3", "cw", 23))]
    with pytest.raises(b4.MissingPairError):
        b4.d3_s1(runs, CONST)
    with pytest.raises(b4.MissingPairError):
        b4.d3_s1([r for r in full() if r.condition != "b"], CONST)


def test_s1_an_unfinished_b_run_cannot_be_reached_and_is_reported_not_skipped() -> None:
    runs = replace(full(), "b", ("N1", "ccw", 21), laps=9)
    d = b4.d3_s1(runs, CONST)
    assert not failed(d, "3.")  # 23 of 24 reach, the 22 needed
    assert "23 of 24" in d.items[next(k for k in d.items if k.startswith("3."))][1]


def test_the_18_run_scaling_is_16_of_18() -> None:
    cases = ("N1", "N2", "N3")
    assert b4.d3_s1(full(cases), CONST).passed
    ks = keys(cases)[:2]
    ok = full(cases, c=lambda case, d, s: {"laps": 9} if (case, d, s) in ks else {})
    assert not failed(b4.d3_s1(ok, CONST), "2.")  # 16 of 18
    ks3 = keys(cases)[:3]
    bad = full(cases, c=lambda case, d, s: {"laps": 9} if (case, d, s) in ks3 else {})
    assert failed(b4.d3_s1(bad, CONST), "2.")  # 15 of 18
    with pytest.raises(b4.PreregError):
        b4.d3_s2(full(cases)[:-1], "e", CONST)  # 17 runs of e: a size nobody registered


# -- D3-S2 --------------------------------------------------------------------------------------------


def test_s2_each_variant_is_judged_alone() -> None:
    runs = replace(full(), "d", ("N1", "ccw", 21), contacts=1)
    assert not b4.d3_s2(runs, "d", CONST).passed
    assert b4.d3_s2(runs, "e", CONST).passed
    assert b4.car_variant("confirms", False, True) == "e"


def test_s2_median_ratio_boundary_0_95() -> None:
    ok = b4.d3_s2(full(d={"late": 12.0 * 0.95}), "d", CONST)
    assert ok.passed
    bad = b4.d3_s2(full(d={"late": 12.0 * 0.951}), "d", CONST)
    assert not bad.passed and "no better than the schedule" in bad.note


def test_s2_completion_and_brakes_boundaries() -> None:
    def short(n):
        ks = keys()[:n]
        return full(e=lambda case, d, s: {"laps": 9} if (case, d, s) in ks else {})

    assert failed(b4.d3_s2(short(3), "e", CONST), "runs of condition e complete")
    assert not failed(b4.d3_s2(short(2), "e", CONST), "runs of condition e complete")
    # brakes: 0.2 per lap on average passes, above fails
    brakes = [0] * 7 + [1, 0, 0] + [0, 0]  # 1 brake in laps 8 to 10: 0.333 per lap
    one = full(
        d=lambda case, dd, s: {"brakes": brakes} if (case, dd, s) == ("N1", "ccw", 21) else {}
    )
    assert not failed(b4.d3_s2(one, "d", CONST), "mean TTC")  # mean over 24 runs: 0.0139
    allbrake = full(d={"brakes": [0] * 7 + [1, 0, 0] + [0, 0]})
    assert failed(b4.d3_s2(allbrake, "d", CONST), "mean TTC")  # 0.333
    edge = full(d={"brakes": [0] * 7 + [0, 0, 0] + [0, 0]})
    assert not failed(b4.d3_s2(edge, "d", CONST), "mean TTC")


def test_s2_a_missing_paired_c_run_raises() -> None:
    runs = [r for r in full() if not (r.condition == "c" and r.key == ("N4", "ccw", 22))]
    with pytest.raises(b4.MissingPairError):
        b4.d3_s2(runs, "d", CONST)
    with pytest.raises(ValueError):
        b4.d3_s2(full(), "c", CONST)


def test_v02_rule_is_labelled_and_independent() -> None:
    d = b4.v02_ds1_for_record(full(b={"late": 90.0}, c={"t1": 100.0, "late": 81.0}), CONST)
    assert "record" in d.name and d.passed is False
    ok = b4.v02_ds1_for_record(full(b={"late": 90.0}, c={"t1": 100.0, "late": 80.0}), CONST)
    assert ok.passed


def test_the_report_only_lines() -> None:
    lines = b4.d_variants_report(full())
    text = "\n".join(lines)
    assert "T_late(d) / T_late(e)" in text and "1.000" in text
    assert "10 of" not in text and "24 of 240" in text  # floor active in 24 runs x 1 of 10 each
    assert "merged zones (created minus standing) 24" in text  # 4 - 3 = 1 per run
    assert b4.d_variants_report([r for r in full() if r.condition != "e"]) == [
        "no condition d and e runs"
    ]
    no_counters = full(d={"zones_created": None})
    assert "did not carry" in "\n".join(b4.d_variants_report(no_counters))


# -- the owner table -------------------------------------------------------------------------------


@pytest.mark.parametrize(
    ("owner", "d_pass", "e_pass", "expected"),
    [
        ("confirms", True, True, "d"),
        ("confirms", True, False, "d"),
        ("confirms", False, True, "e"),
        ("confirms", False, False, "v0"),
        ("rejects", True, True, "e"),
        ("rejects", True, False, "v0"),
        ("rejects", False, True, "e"),
        ("rejects", False, False, "v0"),
        ("undecided", True, True, "v0"),
        ("undecided", True, False, "v0"),
        ("undecided", False, True, "v0"),
        ("undecided", False, False, "v0"),
    ],
)
def test_car_variant_table(owner: str, d_pass: bool, e_pass: bool, expected: str) -> None:
    assert b4.car_variant(owner, d_pass, e_pass) == expected


def test_car_variant_refuses_an_unknown_owner_answer() -> None:
    with pytest.raises(ValueError):
        b4.car_variant("maybe", True, True)


def test_the_report_withholds_d_when_s0_or_s1_failed() -> None:
    rep = b4.build_report(replace(full(), "c", ("N1", "ccw", 21), contacts=1), CONST)
    assert all(v.startswith("no D on the car") for v in rep.car.values())


# -- the battery tool -------------------------------------------------------------------------------


def make_data_root(tmp_path: Path) -> Path:
    for spec in (CONST.battery_cases["N4"], CONST.development_cases["S4"]):
        d = tmp_path / spec["map_dir"]
        d.mkdir(parents=True, exist_ok=True)
        prefix = "oval" if "oval" in d.name else d.name
        (d / f"{prefix}.yaml").write_text("image: x.pgm\n")
        (d / f"{prefix}_v3.yaml").write_text("image: y.pgm\n")
        (d / f"{prefix}_centreline.csv").write_text("0,0\n")
    return tmp_path


def test_map_files_pick_the_yaml_of_the_centreline(tmp_path: Path) -> None:
    root = make_data_root(tmp_path)
    yml, csv = bb.map_files(root / CONST.battery_cases["N4"]["map_dir"])
    assert yml.name == "oval.yaml" and csv.name == "oval_centreline.csv"
    with pytest.raises(bb.BatteryError):
        bb.map_files(tmp_path)


def test_the_ceiling_is_the_bundle_validity_named_in_the_yaml() -> None:
    assert "config/pnp/fixtures/dyn-car-2026-11-12" in CONST.raw["ceiling"]
    assert bb.ceiling_mps(bb.DEFAULT_BUNDLE) == 1.3


def argv_for(condition, case, tmp_path, development=False, direction="cw", seed=None):
    root = make_data_root(tmp_path)
    seed = seed or (11 if development else 21)
    return bb.command_for(
        CONST,
        condition,
        case,
        direction,
        seed,
        120,
        tmp_path,
        tmp_path / "r.json",
        bb.DEFAULT_BUNDLE,
        root,
        development=development,
    )[2:]  # without the interpreter and the script


def test_every_command_parses_with_the_runners_parser(tmp_path: Path) -> None:
    import track_family_runner as runner

    parser = runner.build_parser()
    expect = {
        "a": ("off", 0.5, None, None),
        "b": ("off", 1.3, None, None),
        "c": ("v0", 1.3, 0.5, None),
        "d": ("v1", 1.3, 0.5, (0.3, True)),
        "e": ("v1", 1.3, 0.5, (0.0, False)),
    }
    for cond, (mode, vmax, lap1, v1) in expect.items():
        for case in CONST.battery_cases:
            for direction in DIRS:
                args = parser.parse_args(argv_for(cond, case, tmp_path / cond, False, direction))
                assert args.lap_memory == mode and args.v_max_mps == vmax
                assert args.v_lap1_mps == lap1 and args.reverse == (direction == "cw")
                assert args.seed == 21 and args.stop_after_laps == 12 and args.driver == "pnp"
                if v1:
                    assert (args.lap_bound_floor_mps, args.lap_zone_merge) == v1
                else:
                    assert args.lap_bound_floor_mps is None and args.lap_zone_merge is True
    for case in CONST.development_cases:
        args = parser.parse_args(argv_for("c", case, tmp_path / "dev", True))
        assert args.seed == 11 and args.lap_memory == "v0"
    n4 = parser.parse_args(argv_for("a", "N4", tmp_path / "n4"))
    assert n4.track == "map" and "oval" in n4.track_params_json


def test_a_case_or_seed_outside_the_yaml_refuses(tmp_path: Path) -> None:
    root = make_data_root(tmp_path)

    def build(case="N1", seed=21, direction="ccw", development=False, condition="a"):
        return bb.command_for(
            CONST,
            condition,
            case,
            direction,
            seed,
            120,
            tmp_path,
            tmp_path / "r.json",
            bb.DEFAULT_BUNDLE,
            root,
            development=development,
        )

    for kwargs in (
        {"case": "S1"},  # a development case is not a battery case
        {"case": "N9"},
        {"seed": 11},  # a development seed
        {"seed": 24},
        {"direction": "up"},
        {"condition": "f"},
        {"case": "N1", "development": True},
        {"case": "S1", "development": True, "seed": 21},
    ):
        with pytest.raises(bb.BatteryError):
            build(**kwargs)


def test_development_cannot_touch_the_n_cases_and_the_battery_cannot_touch_development(
    tmp_path: Path, monkeypatch
) -> None:
    monkeypatch.setattr(bb, "check_clean", lambda: None)
    monkeypatch.setattr(bb, "git_sha", lambda: "x" * 40)
    out = tmp_path / "o"
    base = ["--out", str(out), "--data-root", str(make_data_root(tmp_path / "root"))]
    assert bb.main(["run", "--development", "--cases", "S1,N1", *base]) == 2
    assert bb.main(["run", "--development", "--cases", "N4", *base]) == 2
    assert bb.main(["run", "--cases", "T2", *base]) == 2
    assert bb.main(["run", *base]) == 2  # no n4-check yet
    assert bb.main(["run", "--cases", "N1", "--workers", "6", *base]) == 2
    assert bb.main(["run", "--cases", "N1", "--conditions", "z", *base]) == 2
    assert not list(out.glob("*.json"))


# -- the run machinery with a fake runner ---------------------------------------------------------------


class FakeRunner:
    """Stands in for subprocess.run: parses the argv with the runner's own parser (so a renamed
    flag fails here) and writes a result file shaped like the runner's. The run name is read from
    the work directory the tool gave it."""

    def __init__(self, gaps=None, die=None, laps=12):
        self.calls: list[list[str]] = []
        self.gaps = gaps or {}  # name -> max /drive_raw gap per attempt
        self.die = die or set()  # names that never write a result
        self.laps = laps
        self.attempts: dict[str, int] = {}

    def __call__(self, command, stdout=None, stderr=None, env=None, check=False):
        import track_family_runner as runner

        args = runner.build_parser().parse_args(command[2:])
        self.calls.append(command)
        name = Path(args.work_dir).name.split(".attempt")[0]
        k = self.attempts[name] = self.attempts.get(name, 0) + 1
        if name in self.die:
            return subprocess.CompletedProcess(command, 3)
        gaps = self.gaps.get(name, [0.02])
        gap = gaps[min(k, len(gaps)) - 1]
        times = [20.0] + [10.0] * (self.laps - 1)
        result = {
            "gt_lap_times_by_crossing_s": times,
            "system_lap_events": [{"lap_time_s": t} for t in times],
            "wall_contacts": 0,
            "brake_per_lap": [{} for _ in times],
            "drive_raw": {"max_gap_s": gap},
            "lap_status": {"zones_created": 2, "caution_zones": 1},
            "guidance_floor": {"messages_with_stations": 10, "messages_floor_active": 1},
            "stopped_because": "laps",
        }
        Path(args.out).write_text(json.dumps(result))
        return subprocess.CompletedProcess(command, 0)


@pytest.fixture
def harness(tmp_path: Path, monkeypatch):
    monkeypatch.setattr(bb, "check_clean", lambda: None)
    monkeypatch.setattr(bb, "git_sha", lambda: "a" * 40)
    root = make_data_root(tmp_path / "root")

    def go(fake, *argv):
        monkeypatch.setattr(bb.subprocess, "run", fake)
        return bb.main([*argv, "--data-root", str(root)])

    return go, tmp_path / "out"


def test_n4_check_passes_and_runs_only_once(harness) -> None:
    go, out = harness
    fake = FakeRunner(laps=3)
    assert go(fake, "n4-check", "--out", str(out)) == 0
    doc = json.loads((out / "n4_check.json").read_text())
    assert doc["dropped"] is False and doc["laps_by_direction"] == {"ccw": 3, "cw": 3}
    assert len(fake.calls) == 2 and all("--stop-after-laps" in c for c in fake.calls)
    assert go(FakeRunner(laps=3), "n4-check", "--out", str(out)) == 2  # a second check refuses
    assert len(fake.calls) == 2


def test_n4_is_dropped_when_a_direction_does_not_finish_three_laps(harness) -> None:
    go, out = harness
    assert go(FakeRunner(laps=2), "n4-check", "--out", str(out)) == 0
    assert json.loads((out / "n4_check.json").read_text())["dropped"] is True
    assert go(FakeRunner(), "run", "--out", str(out), "--cases", "N4") == 2


def test_n4_is_dropped_when_the_map_cannot_be_loaded(harness, tmp_path: Path) -> None:
    go, out = harness
    fake = FakeRunner(die={"a_N4_cw_21", "a_N4_ccw_21"})
    assert go(fake, "n4-check", "--out", str(out)) == 0
    doc = json.loads((out / "n4_check.json").read_text())
    assert doc["dropped"] is True and any("no result file" in n for n in doc["notes"])


def test_the_battery_runs_resumes_and_never_overwrites(harness) -> None:
    go, out = harness
    assert go(FakeRunner(laps=3), "n4-check", "--out", str(out)) == 0
    fake = FakeRunner()
    assert go(fake, "run", "--out", str(out), "--conditions", "ab", "--cases", "N1,N2") == 0
    assert len(fake.calls) == 24  # 2 conditions x 2 cases x 2 directions x 3 seeds
    assert len(list(out.glob("*_*_*_*.json"))) == 24
    meta = json.loads((out / "a_N1_ccw_21.json").read_text())["battery_b4"]
    assert meta["mode"] == "battery" and meta["git_sha"] == "a" * 40
    assert meta["prereg_yaml_sha256"] == CONST.yaml_sha256
    assert meta["accepted_attempt"] == 1 and meta["attempts"][0]["starved"] is False
    before = (out / "a_N1_ccw_21.json").read_text()
    again = FakeRunner()
    assert go(again, "run", "--out", str(out), "--conditions", "ab", "--cases", "N1,N2") == 0
    assert again.calls == [] and (out / "a_N1_ccw_21.json").read_text() == before
    with pytest.raises(FileExistsError):
        bb.write_once(out / "a_N1_ccw_21.json", "x")


def test_a_starved_run_is_rerun_and_every_attempt_is_reported(harness) -> None:
    go, out = harness
    assert go(FakeRunner(laps=3), "n4-check", "--out", str(out)) == 0
    fake = FakeRunner(gaps={"a_N1_ccw_21": [0.05, 0.045, 0.02], "a_N1_cw_22": [0.05]})
    assert go(fake, "run", "--out", str(out), "--conditions", "a", "--cases", "N1") == 0
    assert fake.attempts["a_N1_ccw_21"] == 3 and fake.attempts["a_N1_cw_22"] == 3
    assert fake.attempts["a_N1_ccw_22"] == 1
    ok = json.loads((out / "a_N1_ccw_21.json").read_text())["battery_b4"]
    assert ok["accepted_attempt"] == 3 and ok["accepted_while_starved"] is False
    assert [a["starved"] for a in ok["attempts"]] == [True, True, False]
    stuck = json.loads((out / "a_N1_cw_22.json").read_text())["battery_b4"]
    assert stuck["accepted_while_starved"] is True and len(stuck["attempts"]) == 3
    assert len(list((out / "attempts").glob("a_N1_ccw_21.attempt*.json"))) == 3
    text = bb.report_text(out)
    assert "a_N1_ccw_21" in text and "a_N1_cw_22" in text
    # the reruns went one at a time, after every first attempt
    names = [Path(c[c.index("--work-dir") + 1]).name for c in fake.calls]
    assert all(n.endswith("attempt1") for n in names[:6]) and names[6].endswith("attempt2")


def test_starvation_rule_and_attempt_cap(tmp_path: Path) -> None:
    assert bb.starved({"drive_raw": {"max_gap_s": 0.0401}})
    assert not bb.starved({"drive_raw": {"max_gap_s": 0.040}})
    assert not bb.starved({})
    assert bb.needs_rerun({"error": "no result file"})
    assert not bb.needs_rerun({"error": "x", "gt_lap_times_by_crossing_s": [1.0]})
    out = tmp_path
    job = ("a_N1_ccw_21", "a", "N1", "ccw", 21)

    def put(k: int, gap: float) -> None:
        body = {
            "gt_lap_times_by_crossing_s": [10.0],
            "drive_raw": {"max_gap_s": gap},
            "battery_b4": {"starved": gap > 0.04},
        }
        bb.write_once(bb.attempt_path(out, job[0], k), json.dumps(body))

    assert bb.pending_work(out, [job]) == [(job, 1)]
    put(1, 0.06)
    assert bb.pending_work(out, [job]) == [(job, 2)]
    put(2, 0.05)
    assert bb.pending_work(out, [job]) == [(job, 3)]
    put(3, 0.07)  # three attempts: accepted as it is, flagged
    assert bb.pending_work(out, [job]) == []
    final = json.loads((out / f"{job[0]}.json").read_text())["battery_b4"]
    assert final["accepted_attempt"] == 3 and final["accepted_while_starved"] is True
    assert [a["starved"] for a in final["attempts"]] == [True, True, True]
    assert bb.pending_work(out, [job]) == []


def test_a_clean_second_attempt_is_accepted(tmp_path: Path) -> None:
    job = ("c_N2_cw_22", "c", "N2", "cw", 22)
    for k, gap in ((1, 0.09), (2, 0.03)):
        body = {
            "gt_lap_times_by_crossing_s": [10.0],
            "drive_raw": {"max_gap_s": gap},
            "battery_b4": {"starved": gap > 0.04},
        }
        bb.write_once(bb.attempt_path(tmp_path, job[0], k), json.dumps(body))
    assert bb.pending_work(tmp_path, [job]) == []
    final = json.loads((tmp_path / f"{job[0]}.json").read_text())["battery_b4"]
    assert final["accepted_attempt"] == 2 and final["accepted_while_starved"] is False


def test_a_directory_never_mixes_development_and_battery_runs(harness) -> None:
    go, out = harness
    assert (
        go(
            FakeRunner(),
            "run",
            "--development",
            "--out",
            str(out),
            "--conditions",
            "a",
            "--cases",
            "S1",
        )
        == 0
    )
    assert bb.main(["run", "--out", str(out), "--cases", "N1", "--data-root", "/x"]) == 2
    assert (out / "a_S1_ccw_11.json").exists()


def test_the_dirty_tree_check_names_the_guarded_paths(monkeypatch) -> None:
    assert bb.GUARDED_PATHS == (
        "ros_ws/src/racer_laps",
        "ros_ws/src/racer_planner",
        "tests/l5_reactive_bench",
    )
    monkeypatch.setattr(bb, "_git", lambda *a: " M ros_ws/src/racer_laps/x.py\n")
    with pytest.raises(bb.BatteryError, match="dirty"):
        bb.check_clean()
    monkeypatch.setattr(bb, "_git", lambda *a: "")
    bb.check_clean()


def test_a_full_fake_battery_reports_five_decisions(harness) -> None:
    go, out = harness
    assert go(FakeRunner(laps=3), "n4-check", "--out", str(out)) == 0
    assert go(FakeRunner(), "run", "--out", str(out), "--workers", "5") == 0
    assert len(list(out.glob("*_*_*_*.json"))) == 120
    text = bb.report_text(out)
    assert "D3-S0" in text and "D3-S2(d)" in text and "D3-S2(e)" in text
    assert "v0.2 D-S1 rule" in text and "What goes on the car" in text
    # the fake plays the same lap times in every condition: no D gate passes the 0.95 ratio
    assert "D3-S2(d): FAIL" in text


def test_a_partial_battery_makes_no_decision(harness) -> None:
    go, out = harness
    assert go(FakeRunner(laps=3), "n4-check", "--out", str(out)) == 0
    assert go(FakeRunner(), "run", "--out", str(out), "--conditions", "ab", "--cases", "N1") == 0
    assert "INCOMPLETE" in bb.report_text(out)


def test_the_runner_script_is_where_the_builder_says() -> None:
    assert bb.RUNNER.exists() and bb.DEFAULT_BUNDLE.exists()
    assert shutil.which(sys.executable) is not None
    assert not math.isnan(bb.STARVED_GAP_S)


def test_a_stated_git_sha_replaces_git_in_the_container(monkeypatch) -> None:
    monkeypatch.setenv(bb.GIT_SHA_ENV, "b" * 40)
    monkeypatch.setattr(bb, "_git", lambda *a: (_ for _ in ()).throw(AssertionError("no git")))
    assert bb.git_sha() == "b" * 40
    bb.check_clean()
    monkeypatch.setenv(bb.GIT_SHA_ENV, "xyz")
    with pytest.raises(bb.BatteryError):
        bb.git_sha()


# -- brief 48: the registered truth fields and invalid runs ---------------------------------------


def test_summarise_reads_the_registered_truth_field_first() -> None:
    result = {
        "gt_lap_times_registered_s": [31.9, 10.0],
        "gt_lap_times_by_crossing_s": [29.6, 10.0],  # the pre-brief-48 field of the same run
        "system_lap_events": [{"lap_time_s": 32.0}, {"lap_time_s": 10.1}],
        "truth_valid": True,
    }
    s = b4.summarise_b4_run(result, "c", "N3", "cw", 23)
    assert s.gt_lap_times_s == (31.9, 10.0) and s.error == ""


def test_an_invalid_truth_is_a_run_that_failed_to_execute() -> None:
    result = {
        "gt_lap_times_registered_s": [29.6] + [10.0] * 11,
        "system_lap_events": [{"lap_time_s": 32.0}] + [{"lap_time_s": 10.0}] * 11,
        "truth_valid": False,
        "truth_invalid_reason": "truth_starts_after_crossing_0",
    }
    s = b4.summarise_b4_run(result, "c", "N3", "cw", 23)
    assert "truth_starts_after_crossing_0" in s.error and not s.completed


def test_a_result_without_validity_is_a_stored_b4_run_and_is_read_as_before() -> None:
    result = {"gt_lap_times_by_crossing_s": [20.0, 10.0], "system_lap_events": []}
    s = b4.summarise_b4_run(result, "a", "N1", "ccw", 21)
    assert s.error == "" and s.gt_lap_times_s == (20.0, 10.0)


def test_an_invalid_truth_is_rerun_like_a_starved_run() -> None:
    ok = {"gt_lap_times_by_crossing_s": [10.0], "truth_valid": True}
    assert not bb.needs_rerun(ok)
    bad = {**ok, "truth_valid": False, "truth_invalid_reason": "reset_after_driving"}
    assert bb.needs_rerun(bad)
    assert not bb.needs_rerun({"gt_lap_times_by_crossing_s": [10.0]})  # a stored B4 file


def test_a_nan_system_lap_time_is_never_within_tolerance() -> None:
    """NaN compares False both ways: it used to be neither within nor outside, so S0a passed with a
    NaN lap and S0b counted it nowhere (brief 48, found writing the B7 decision tests)."""
    nan_a = replace(full(), "a", ("N1", "ccw", 21), system=[math.nan] + [27.0] * 11)
    d = b4.d3_s0(nan_a, CONST)
    assert not d.passed and failed(d, "S0a: every system lap time")
    nan_c = replace(full(), "c", ("N1", "ccw", 21), system=[math.nan] + [12.0] * 11)
    d = b4.d3_s0(nan_c, CONST)
    # 287 of 288 laps within is above the 98 percent, but a NaN lap is off by more than any limit
    assert not d.passed and failed(d, "S0b(c): no lap off by more than")
    assert not failed(d, "S0b(c): at least")
