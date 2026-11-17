"""Battery B5 and measurement M-A2 (v0.4 pre-registration sections 3.5 and 5, addendum 1 section
3.3): the decisions D4-S0, D4-S1, D4-S2, the M-A2 reading, the constants file check, the command
builders against the runner's own parser, the B4 gate and the freeze check, with fakes. No ROS and
no simulator here.

Every boundary of every threshold is tested from both sides: the thresholds are the
pre-registration and a decision that flips at the wrong number cannot be found after the battery
has run."""

from __future__ import annotations

import json
import math
import sys
from pathlib import Path

import battery_b5 as bb
import lap_battery_b4 as b4
import lap_battery_b5 as b5
import pytest
from lap_battery import RunSummary

HERE = Path(__file__).resolve().parent
sys.path.insert(0, str(HERE.parent / "l5_reactive_bench"))
REPO = HERE.parents[1]

CONST = b5.load_constants()
CASES_16 = ("O1", "O2", "O3", "O4")
CASES_12 = ("O1", "O2", "O3")
DIRS = ("ccw", "cw")
SEEDS = (31, 32)


def mk(
    condition: str,
    case: str = "O1",
    direction: str = "ccw",
    seed: int = 31,
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
        extras=dict(extras),
    )


LATE = {"a": 27.0, "b": 12.0, "c": 12.0, "v": 11.0}


def keys(cases):
    return [(c, d, s) for c in cases for d in DIRS for s in SEEDS]


def full(cases=CASES_16, conditions=("a", "b", "c", "v"), **override) -> list[RunSummary]:
    runs = []
    for cond in conditions:
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


def test_the_yaml_hash_is_the_registered_one() -> None:
    assert CONST.yaml_sha256 == "78510c1423278f80c42511e6caab095818598af6db6a284dd5479f74a03a1d63"
    assert b5.registered_sha() == CONST.yaml_sha256


def test_thresholds_are_the_preregistered_numbers() -> None:
    th = CONST.th
    assert th["lap_time_tolerance_s"] == 0.3 and th["s0b_min_share_within_tolerance"] == 0.98
    assert th["s0b_max_single_error_s"] == 1.0 and th["max_contacts"] == 0
    assert (th["required_complete_of_16"], th["required_complete_of_12"]) == (15, 11)
    assert th["s1_reach_max_ratio_to_b"] == 1.05 and th["s1_max_median_r"] == 0.80
    assert th["s2_max_median_ratio_to_c"] == 0.95 and th["s2_max_brakes_per_lap_late"] == 0.2
    assert CONST.battery_seeds == (31, 32) and CONST.development_seeds == (11, 12, 13)
    assert CONST.directions == ("ccw", "cw") and CONST.laps == 12 and CONST.lap1_cap_mps == 0.5
    assert set(CONST.battery_cases) == set(CASES_16) and set(CONST.development_cases) == {"OD"}
    assert set(CONST.conditions) == {"a", "b", "c", "v"}


def test_oracle_constants_are_the_preregistered_numbers() -> None:
    o = CONST.oracle
    assert o["cases"] == ["T", "S1", "S2", "S3", "S4"] and o["seed"] == 11 and o["laps"] == 6
    assert o["scored_laps"] == [3, 4, 5, 6] and o["cap"] == "ceiling"
    assert o["no_path_if_median_ratio_at_least"] == 0.97 and o["and_no_pair_below"] == 0.93
    assert o["matters_if_median_ratio_at_most"] == 0.95


def test_a_yaml_with_another_hash_is_refused(tmp_path: Path) -> None:
    copy = tmp_path / "pnp-v0.4.yaml"
    copy.write_text((REPO / "evaluation" / "prereg" / "pnp-v0.4.yaml").read_text() + "\n# edit\n")
    with pytest.raises(b5.PreregError, match="SHA-256"):
        b5.load_constants(copy)


def test_the_addendum_constants_are_checked_and_read() -> None:
    ext = b5.load_extensions_checked()
    assert ext["extra_case"] == "OD"
    assert ext["weight_check"]["factors"] == [0.5, 2.0]
    assert ext["weight_check"]["headroom_if_median_ratio_at_most"] == 0.95


def test_required_completions_only_for_the_registered_counts() -> None:
    assert CONST.required_complete(16) == 15 and CONST.required_complete(12) == 11
    for n in (24, 18, 15, 8):
        with pytest.raises(b5.PreregError):
            CONST.required_complete(n)


# -- D4-S0 -------------------------------------------------------------------------------------------


def test_a_clean_battery_passes_all_three() -> None:
    runs = full()
    for dec in (b5.d4_s0(runs, CONST), b5.d4_s1(runs, CONST), b5.d4_s2(runs, CONST)):
        assert dec.passed, str(dec)
    assert b5.d4_s0(runs, CONST).name == "D4-S0"
    assert b5.d4_s1(runs, CONST).name == "D4-S1"
    assert b5.d4_s2(runs, CONST).name == "D4-S2"


def test_d4_s0a_count_and_every_lap_time() -> None:
    runs = full()
    key = ("O2", "cw", 32)
    bad = replace(runs, "a", key, system=[27.0] * 11)  # 11 system laps against 12
    assert failed(b5.d4_s0(bad, CONST), "S0a: system lap count")
    t = [27.0] + [27.0] * 11
    t[4] += 0.299  # just inside the tolerance: passes
    edge = replace(runs, "a", key, system=t)
    assert not failed(b5.d4_s0(edge, CONST), "S0a")
    t[4] += 0.002  # 0.301 over
    over = replace(runs, "a", key, system=t)
    assert failed(b5.d4_s0(over, CONST), "S0a: every system lap time")


def test_d4_s0a_a_failed_run_fails() -> None:
    bad = replace(full(), "a", ("O1", "ccw", 31), error="runner died")
    assert failed(b5.d4_s0(bad, CONST), "S0a: no condition (a) run failed")


@pytest.mark.parametrize("cond", ["c", "v"])
def test_d4_s0b_share_and_single_error(cond: str) -> None:
    runs = full()

    # 16 runs x 12 laps = 192 laps; 98 percent means at most 3 laps outside (189 / 192 = 0.984)
    def with_outside(n_runs_laps: int, err: float):
        out = runs
        for key in keys(CASES_16)[:n_runs_laps]:
            r = next(x for x in out if x.condition == cond and x.key == key)
            system = list(r.gt_lap_times_s)
            system[2] += err
            out = replace(out, cond, key, system=system)
        return out

    ok = with_outside(3, 0.5)  # 189 of 192 within = 0.984375 >= 0.98
    assert not failed(b5.d4_s0(ok, CONST), f"S0b({cond}): at least")
    bad = with_outside(4, 0.5)  # 188 of 192 = 0.9792 < 0.98
    assert failed(b5.d4_s0(bad, CONST), f"S0b({cond}): at least")
    edge = with_outside(1, 1.0)  # a lap off by exactly 1.0 s passes the single error item
    assert not failed(b5.d4_s0(edge, CONST), f"S0b({cond}): no lap off")
    over = with_outside(1, 1.01)
    assert failed(b5.d4_s0(over, CONST), f"S0b({cond}): no lap off")


def test_d4_s0b_lap_count_must_agree_in_every_run() -> None:
    bad = replace(full(), "c", ("O3", "ccw", 31), system=[27.0] * 13)
    assert failed(b5.d4_s0(bad, CONST), "S0b(c): lap count")


def test_d4_s0_without_condition_v_is_decided_on_a_and_c() -> None:
    runs = full(conditions=("a", "b", "c"))
    dec = b5.d4_s0(runs, CONST)
    assert dec.passed
    assert not any("S0b(v)" in k for k in dec.items)


def test_d4_s0_needs_condition_a_and_c() -> None:
    with pytest.raises(b4.MissingPairError):
        b5.d4_s0(full(conditions=("b", "c")), CONST)
    with pytest.raises(b4.MissingPairError):
        b5.d4_s0(full(conditions=("a", "b")), CONST)


# -- D4-S1 -------------------------------------------------------------------------------------------


def test_d4_s1_contacts_need_to_be_zero() -> None:
    bad = replace(full(), "c", ("O2", "ccw", 31), contacts=1)
    assert failed(b5.d4_s1(bad, CONST), "1.")


def test_d4_s1_completion_15_of_16_and_11_of_12() -> None:
    runs = full()
    one = replace(runs, "c", ("O1", "ccw", 31), laps=9)  # 15 of 16 complete
    assert not failed(b5.d4_s1(one, CONST), "2.")
    two = replace(one, "c", ("O1", "cw", 31), laps=9)  # 14 of 16
    assert failed(b5.d4_s1(two, CONST), "2.")
    runs12 = full(CASES_12)
    a = replace(runs12, "c", ("O1", "ccw", 31), laps=9)  # 11 of 12
    assert not failed(b5.d4_s1(a, CONST), "2.")
    b = replace(a, "c", ("O1", "cw", 31), laps=9)  # 10 of 12
    assert failed(b5.d4_s1(b, CONST), "2.")


def test_d4_s1_reach_at_most_1_05_of_the_paired_control() -> None:
    # 15 of 16 pairs must reach: 15 just inside 1.05, one over
    runs = full()
    for i, key in enumerate(keys(CASES_16)):
        late = 12.0 * 1.049 if i < 15 else 12.0 * 1.06
        runs = replace(runs, "c", key, late=late)
    dec = b5.d4_s1(runs, CONST)
    assert not failed(dec, "3."), str(dec)
    runs = replace(runs, "c", keys(CASES_16)[14], late=12.0 * 1.06)  # 14 reach only
    assert failed(b5.d4_s1(runs, CONST), "3.")


def test_d4_s1_t_late_below_t1_and_median_r() -> None:
    runs = full()
    not_below = replace(runs, "c", ("O3", "cw", 32), t1=12.0, late=12.0)
    assert failed(b5.d4_s1(not_below, CONST), "4a.")
    # median r exactly 0.80 passes, above fails (every run the same r)
    edge = full(c={"t1": 15.0, "late": 12.0})  # r = 0.8
    assert not failed(b5.d4_s1(edge, CONST), "4b.")
    over = full(c={"t1": 14.99, "late": 12.0})  # r = 0.8005
    assert failed(b5.d4_s1(over, CONST), "4b.")


def test_d4_s1_a_missing_pair_is_an_error() -> None:
    runs = [r for r in full() if not (r.condition == "b" and r.key == ("O1", "ccw", 31))]
    with pytest.raises(b4.MissingPairError):
        b5.d4_s1(runs, CONST)


# -- D4-S2 -------------------------------------------------------------------------------------------


def test_d4_s2_median_ratio_to_c() -> None:
    edge = full(v={"late": 12.0 * 0.949})
    assert not failed(b5.d4_s2(edge, CONST), "median"), str(b5.d4_s2(edge, CONST))
    over = full(v={"late": 12.0 * 0.951})
    dec = b5.d4_s2(over, CONST)
    assert failed(dec, "median")
    assert dec.note == "no better than the schedule in sim"


def test_d4_s2_contacts_completion_and_brakes() -> None:
    runs = full()
    assert failed(b5.d4_s2(replace(runs, "v", ("O1", "ccw", 31), contacts=1), CONST), "0 wall")
    two = replace(replace(runs, "v", ("O1", "ccw", 31), laps=9), "v", ("O1", "cw", 31), laps=9)
    assert failed(b5.d4_s2(two, CONST), "runs of condition v complete")
    # mean brakes per lap over laps 8 to 10: 0.2 passes, above fails (all 16 runs the same)
    edge = full(v={"brakes": [0] * 7 + [1, 0, 0, 0, 0]})  # (1 / 3) > 0.2 -> fails
    assert failed(b5.d4_s2(edge, CONST), "mean TTC")
    ok = full(v={"brakes": [0] * 12})
    assert not failed(b5.d4_s2(ok, CONST), "mean TTC")


def test_d4_s2_brakes_at_the_threshold() -> None:
    # 0.2 exactly: of 16 runs, 3 have one brake in laps 8 to 10 -> mean (3 / 3) / 16 = 0.0625 ok;
    # 10 of 16 have one -> (1 / 3) * 10 / 16 = 0.2083 over; 9 of 16 -> 0.1875 ok
    def brakes_for(n_bad: int):
        def f(case, d, s):
            idx = keys(CASES_16).index((case, d, s))
            return {"brakes": [0] * 7 + ([1] if idx < n_bad else [0]) + [0] * 4}

        return f

    assert not failed(b5.d4_s2(full(v=brakes_for(9)), CONST), "mean TTC")
    assert failed(b5.d4_s2(full(v=brakes_for(10)), CONST), "mean TTC")


def test_d4_s2_needs_condition_c_and_v() -> None:
    with pytest.raises(b4.MissingPairError):
        b5.d4_s2(full(conditions=("a", "b", "c")), CONST)


# -- the report --------------------------------------------------------------------------------------


def test_report_without_condition_v_says_so_and_has_no_d4_s2() -> None:
    rep = b5.build_report(full(conditions=("a", "b", "c")), CONST)
    assert set(rep.decisions) == {"D4-S0", "D4-S1"}
    assert any("condition v was not run" in x for x in rep.extra)


def test_report_extras_have_medians_innovation_and_clearances() -> None:
    runs = [
        mk(
            c,
            "O1",
            d,
            s,
            innovation_rejections={"per_lap": [2, 0, 1], "total": 3, "accepted_total": 14},
            obstacle_passes=[{"index": 0, "ticks": 20, "min_plan_clearance_m": 0.12}],
        )
        for c in ("a", "b", "c")
        for d in DIRS
        for s in SEEDS
    ]
    text = "\n".join(
        [
            *b5.per_case_medians(runs),
            *b5.innovation_per_lap(runs),
            *b5.obstacle_clearance_lines(runs),
        ]
    )
    assert "| c | O1 | 4 | 4 |" in text
    assert "rejected per lap [2, 0, 1]" in text
    assert "min planned clearance 0.120 m over 80 passing ticks" in text
    assert "not counted" in "\n".join(b5.innovation_per_lap([mk("a", "O1")]))


# -- M-A2 ---------------------------------------------------------------------------------------------


def ma2_runs(ratio, cases=("T", "S1", "S2", "S3", "S4"), laps=6):
    runs = []
    for case in cases:
        for d in DIRS:
            r = ratio(case, d) if callable(ratio) else ratio
            runs.append(mk("g", case, d, 11, laps=laps, t1=20.0, late=10.0))
            runs.append(mk("o", case, d, 11, laps=laps, t1=20.0, late=10.0 * r))
    return runs


def test_scored_laps_are_3_to_6_and_only_those() -> None:
    run = RunSummary("g", "T", "ccw", 11, (50.0, 40.0, 10.0, 11.0, 12.0, 13.0), (), 0, ())
    assert b5.scored_median(run, [3, 4, 5, 6]) == pytest.approx(11.5)
    short = RunSummary("g", "T", "ccw", 11, (50.0, 40.0, 10.0, 11.0), (), 0, ())
    assert math.isnan(b5.scored_median(short, [3, 4, 5, 6]))


@pytest.mark.parametrize(
    "ratio, reading",
    [
        (1.00, "no measured path"),
        (0.97, "no measured path"),  # at the threshold
        (0.969, "undecided"),
        (0.96, "undecided"),
        (0.951, "undecided"),
        (0.95, "matters"),  # at the threshold
        (0.80, "matters"),
    ],
)
def test_ma2_reading_boundaries(ratio: float, reading: str) -> None:
    pairs = b5.pairs_against(ma2_runs(ratio), "o", "g", CONST)
    assert b5.ma2_reading(pairs, CONST)[0] == reading


def test_ma2_one_pair_below_0_93_blocks_no_path() -> None:
    def ratio(case, d):
        return 0.929 if (case, d) == ("S2", "cw") else 1.0

    pairs = b5.pairs_against(ma2_runs(ratio), "o", "g", CONST)
    sentence, numbers = b5.ma2_reading(pairs, CONST)
    assert sentence == "undecided" and "min 0.929" in numbers
    edge = b5.pairs_against(
        ma2_runs(lambda c, d: 0.93 if (c, d) == ("S2", "cw") else 1.0), "o", "g", CONST
    )
    assert b5.ma2_reading(edge, CONST)[0] == "no measured path"


def test_ma2_median_over_the_ten_pairs() -> None:
    ratios = {("T", "ccw"): 0.9, ("T", "cw"): 0.9, ("S1", "ccw"): 0.9, ("S1", "cw"): 0.9}
    pairs = b5.pairs_against(ma2_runs(lambda c, d: ratios.get((c, d), 1.0)), "o", "g", CONST)
    assert len(pairs) == 10
    assert b5.ma2_reading(pairs, CONST)[1].startswith("median ratio 1.000, min 0.900, max 1.000")


def test_ma2_incomplete_pairs_are_reported_not_decided_silently() -> None:
    runs = ma2_runs(1.0)
    runs = [r for r in runs if not (r.condition == "o" and r.case == "S3" and r.direction == "cw")]
    runs.append(mk("o", "S3", "cw", 11, laps=4, t1=20.0, late=10.0, contacts=1))
    text = b5.ma2_report(runs, CONST)
    assert "INCOMPLETE PAIRS" in text and "S3/cw" in text
    assert "over 9 of 10 pairs" in text


def test_ma2_missing_partner_raises() -> None:
    for missing in ("o", "g"):  # a partner missing on either side is an error, never skipped
        runs = [r for r in ma2_runs(1.0) if not (r.condition == missing and r.case == "T")]
        with pytest.raises(b4.MissingPairError):
            b5.pairs_against(runs, "o", "g", CONST)


def test_ma2_ignores_od_runs_in_the_registered_pairs() -> None:
    runs = ma2_runs(1.0) + [mk("g", "OD", "ccw", 11, laps=6)]  # no o partner for OD
    assert len(b5.pairs_against(runs, "o", "g", CONST)) == 10


def test_ma2x_report_has_three_pairings_od_and_the_missing_terms_statement() -> None:
    runs = ma2_runs(0.98)
    for case in ("T", "S1", "S2", "S3", "S4"):
        for d in DIRS:
            runs.append(
                mk(
                    "f",
                    case,
                    d,
                    11,
                    laps=6,
                    t1=20.0,
                    late=10.5,
                    pnp_ticks={
                        "driving_ticks": 100,
                        "mean_track_confidence": 0.9,
                        "share_cap_limited": 0.5,
                        "min_plan_clearance_m": 0.1,
                    },
                )
            )
    for c, late in (("g", 10.0), ("o", 9.9), ("f", 10.4)):
        for d in DIRS:
            runs.append(mk(c, "OD", d, 11, laps=6, t1=20.0, late=late))
    text = b5.ma2x_report(runs, "trackperc-v2-2026-11-14 refined", CONST)
    assert "condition f = trackperc-v2-2026-11-14 refined" in text
    for heading in ("## f over g", "## o over g", "## f over o", "## Obstacle development case OD"):
        assert heading in text
    assert "NOT AVAILABLE" in text and "PlannerTiming" in text
    assert "| f | ccw | 11 |" in text


# -- the command builders ------------------------------------------------------------------------------


def argv_of(command: list[str]) -> list[str]:
    return command[2:]  # after python and the runner path


def parse_with_runner(command: list[str]):
    import track_family_runner as runner

    parser_args = argv_of(command)
    old = sys.argv
    try:
        sys.argv = ["runner", *parser_args]
        # the runner's parser is built inside main(); reach it by parse via a dry call
        return runner, parser_args
    finally:
        sys.argv = old


def flag(command: list[str], name: str) -> str:
    return command[command.index(name) + 1]


BUNDLE = bb.DEFAULT_BUNDLE


def test_b5_command_for_a_battery_run() -> None:
    job = bb.Job.make("c", "O3", "cw", 32)
    cmd = bb.b5_command(
        CONST, job, "b5", 120, Path("/w"), Path("/o.json"), BUNDLE, Path("/data"), ""
    )
    assert flag(cmd, "--track") == "stadium"
    params = json.loads(flag(cmd, "--track-params-json"))
    assert params["straight_length_m"] == 1.0 and len(params["obstacles"]) == 3
    assert flag(cmd, "--driver") == "pnp" and flag(cmd, "--backend") == "geometric"
    assert flag(cmd, "--lap-memory") == "v0" and flag(cmd, "--seed") == "32"
    assert flag(cmd, "--stop-after-laps") == "12" and "--reverse" in cmd
    assert "--count-innovation" in cmd and "--sim-time" not in cmd


def test_b5_conditions_flags() -> None:
    ceiling = b4_ceiling = 1.3
    del b4_ceiling
    a = bb.b5_condition_flags(CONST, "a", ceiling, "")
    assert a == ["--lap-memory", "off", "--v-max-mps", "0.5"]
    b = bb.b5_condition_flags(CONST, "b", ceiling, "")
    assert b == ["--lap-memory", "off", "--v-max-mps", "1.3"]
    c = bb.b5_condition_flags(CONST, "c", ceiling, "")
    assert c[:2] == ["--lap-memory", "v0"]
    d = bb.b5_condition_flags(CONST, "v", ceiling, "d")
    assert d[:2] == ["--lap-memory", "v1"]
    assert "--lap-zone-merge" in d and flag(d, "--lap-bound-floor-mps") == "0.3"
    e = bb.b5_condition_flags(CONST, "v", ceiling, "e")
    assert "--no-lap-zone-merge" in e and flag(e, "--lap-bound-floor-mps") == "0.0"
    with pytest.raises(bb.BatteryError, match="variant"):
        bb.b5_condition_flags(CONST, "v", ceiling, "")


def test_no_o_case_with_a_seed_outside_its_mode() -> None:
    job_dev = bb.Job.make("a", "OD", "ccw", 31)
    with pytest.raises(bb.BatteryError, match="seed"):
        bb.b5_command(
            CONST, job_dev, "b5-development", 1, Path("w"), Path("o"), BUNDLE, Path("/d"), ""
        )
    job_bat = bb.Job.make("a", "O1", "ccw", 11)
    with pytest.raises(bb.BatteryError, match="seed"):
        bb.b5_command(CONST, job_bat, "b5", 1, Path("w"), Path("o"), BUNDLE, Path("/d"), "")
    with pytest.raises(bb.BatteryError, match="case"):
        bb.b5_command(
            CONST,
            bb.Job.make("a", "O1", "ccw", 11),
            "b5-development",
            1,
            Path("w"),
            Path("o"),
            BUNDLE,
            Path("/d"),
            "",
        )
    with pytest.raises(bb.BatteryError, match="case"):
        bb.b5_command(
            CONST,
            bb.Job.make("a", "OD", "ccw", 31),
            "b5",
            1,
            Path("w"),
            Path("o"),
            BUNDLE,
            Path("/d"),
            "",
        )


def test_the_development_case_od_command() -> None:
    cmd = bb.b5_command(
        CONST,
        bb.Job.make("a", "OD", "ccw", 11),
        "b5-development",
        1,
        Path("w"),
        Path("o"),
        BUNDLE,
        Path("/d"),
        "",
    )
    p = json.loads(flag(cmd, "--track-params-json"))
    assert p["straight_length_m"] == 0.0 and p["island_radius_m"] == 0.5
    assert p["obstacles"] == [{"at": 0.30, "side": "outer", "protrusion_m": 0.25, "length_m": 0.30}]


def test_o4_command_uses_the_map_files(tmp_path: Path) -> None:
    mapdir = tmp_path / "car-data" / "maps" / "2026-11-13-pnp-lap1"
    mapdir.mkdir(parents=True)
    (mapdir / "2026-11-13-pnp-lap1.yaml").write_text("image: x.pgm\n")
    (mapdir / "2026-11-13-pnp-lap1_centreline.csv").write_text("x")
    cmd = bb.b5_command(
        CONST,
        bb.Job.make("a", "O4", "ccw", 31),
        "b5",
        1,
        Path("w"),
        Path("o"),
        BUNDLE,
        tmp_path,
        "",
    )
    assert flag(cmd, "--track") == "map"
    p = json.loads(flag(cmd, "--track-params-json"))
    assert p["map_yaml"].endswith("2026-11-13-pnp-lap1.yaml") and len(p["obstacles"]) == 2


def test_ma2_commands_for_g_o_f_and_the_weight(tmp_path: Path) -> None:
    perception = Path("/models/trackperc-v2-2026-11-14")
    g = bb.ma2_command(
        CONST,
        bb.Job.make("g", "T", "ccw", 11),
        "ma2",
        1,
        Path("w"),
        Path("o"),
        BUNDLE,
        Path("/d"),
        None,
        None,
    )
    assert flag(g, "--backend") == "geometric" and flag(g, "--stop-after-laps") == "6"
    assert flag(g, "--lap-memory") == "off" and flag(g, "--seed") == "11"
    assert float(flag(g, "--v-max-mps")) == bb.b4t.ceiling_mps(BUNDLE)
    o = bb.ma2_command(
        CONST,
        bb.Job.make("o", "S1", "cw", 11),
        "ma2",
        1,
        Path("w"),
        Path("o"),
        BUNDLE,
        Path("/d"),
        None,
        None,
    )
    assert flag(o, "--backend") == "oracle" and "--reverse" in o
    f = bb.ma2_command(
        CONST,
        bb.Job.make("f", "OD", "ccw", 11),
        "ma2x",
        1,
        Path("w"),
        Path("o"),
        BUNDLE,
        Path("/d"),
        perception,
        None,
    )
    assert flag(f, "--backend") == "net" and flag(f, "--perception-bundle") == str(perception)
    assert json.loads(flag(f, "--track-params-json"))["obstacles"]
    w = bb.ma2_command(
        CONST,
        bb.Job.make("g", "T", "ccw", 11),
        "ma2w",
        1,
        Path("w"),
        Path("o"),
        BUNDLE,
        Path("/d"),
        None,
        2.0,
    )
    assert json.loads(flag(w, "--planner-params-json")) == {
        "w_bound": 2.0 * bb.default_boundary_weight()
    }


def test_ma2_refuses_wrong_seed_condition_case_and_stage() -> None:
    args = (Path("w"), Path("o"), BUNDLE, Path("/d"), None, None)
    with pytest.raises(bb.BatteryError, match="seed"):
        bb.ma2_command(CONST, bb.Job.make("g", "T", "ccw", 12), "ma2", 1, *args)
    with pytest.raises(bb.BatteryError, match="not g, o or f"):
        bb.ma2_command(CONST, bb.Job.make("x", "T", "ccw", 11), "ma2", 1, *args)
    with pytest.raises(bb.BatteryError, match="M-A2 case"):
        bb.ma2_command(CONST, bb.Job.make("g", "OD", "ccw", 11), "ma2", 1, *args)
    with pytest.raises(bb.BatteryError, match="perception-bundle"):
        bb.ma2_command(CONST, bb.Job.make("f", "T", "ccw", 11), "ma2x", 1, *args)
    with pytest.raises(bb.BatteryError, match="M-A2 case"):
        bb.ma2_command(CONST, bb.Job.make("g", "O1", "ccw", 11), "ma2x", 1, *args)


def test_default_boundary_weight_is_read_from_the_planner_source() -> None:
    assert bb.default_boundary_weight() == 200.0
    text = (REPO / "ros_ws/src/racer_planner/include/racer_planner/mppi.hpp").read_text()
    assert "double w_bound = 200.0;" in text


def test_the_runner_parser_accepts_the_commands(tmp_path: Path) -> None:
    """Every flag the builders emit is one the runner's own parser knows."""
    import argparse

    import track_family_runner as runner

    captured: dict = {}

    def fake_run(args: argparse.Namespace):
        captured["args"] = args
        return {}

    for command in (
        bb.b5_command(
            CONST,
            bb.Job.make("v", "O2", "cw", 31),
            "b5",
            1,
            tmp_path,
            tmp_path / "o.json",
            BUNDLE,
            Path("/d"),
            "d",
        ),
        bb.ma2_command(
            CONST,
            bb.Job.make("o", "T", "ccw", 11),
            "ma2",
            1,
            tmp_path,
            tmp_path / "o.json",
            BUNDLE,
            Path("/d"),
            None,
            None,
        ),
        bb.ma2_command(
            CONST,
            bb.Job.make("g", "OD", "cw", 11),
            "ma2x",
            1,
            tmp_path,
            tmp_path / "o.json",
            BUNDLE,
            Path("/d"),
            None,
            0.5,
        ),
    ):
        old_run = runner.run_benchmark
        runner.run_benchmark = fake_run
        try:
            assert runner.main(command[2:]) == 0
        finally:
            runner.run_benchmark = old_run
        args = captured["args"]
        assert args.driver == "pnp"
    assert captured["args"].planner_params_json


# -- the B4 gate and the freeze --------------------------------------------------------------------------


def write_b4_dir(path: Path, passing: bool, complete: bool = True, shas=("1" * 40,)) -> None:
    path.mkdir(parents=True)
    const3 = b4.load_constants()
    for cond in b4.CONDITIONS:
        for case in ("N1", "N2", "N3", "N4"):
            for d in ("ccw", "cw"):
                for s in (21, 22, 23):
                    if not complete and (cond, case, d, s) == ("e", "N4", "cw", 23):
                        continue
                    late = {"a": 27.0, "b": 12.0, "c": 12.0, "d": 11.0, "e": 11.0}[cond]
                    gt = [27.0] + [late] * 11
                    if not passing and cond == "c":
                        gt = [27.0] + [26.0] * 11  # no improvement: D3-S1 fails
                    result = {
                        "gt_lap_times_by_crossing_s": gt,
                        "system_lap_events": [{"lap_time_s": t} for t in gt],
                        "wall_contacts": 0,
                        "brake_per_lap": [{} for _ in gt],
                        "battery_b4": {
                            "mode": "battery",
                            "git_sha": shas[(len(cond) + s) % len(shas)],
                            "prereg_yaml_sha256": const3.yaml_sha256,
                        },
                    }
                    (path / f"{cond}_{case}_{d}_{s}.json").write_text(json.dumps(result))
    (path / "n4_check.json").write_text(json.dumps({"dropped": False}))


def test_b4_gate_passes_only_when_d3_s0_and_d3_s1_pass(tmp_path: Path) -> None:
    write_b4_dir(tmp_path / "good", True)
    ok, sha, text = bb.b4_gate(tmp_path / "good")
    assert ok and sha == "1" * 40 and "D3-S0: PASS" in text and "D3-S1: PASS" in text
    write_b4_dir(tmp_path / "bad", False)
    ok, _sha, text = bb.b4_gate(tmp_path / "bad")
    assert not ok and "D3-S1: FAIL" in text


def test_b4_gate_refuses_an_incomplete_or_mixed_b4_directory(tmp_path: Path) -> None:
    write_b4_dir(tmp_path / "short", True, complete=False)
    with pytest.raises(bb.BatteryError, match="not a complete B4"):
        bb.b4_gate(tmp_path / "short")
    write_b4_dir(tmp_path / "mixed", True, shas=("1" * 40, "2" * 40))
    with pytest.raises(bb.BatteryError, match="git SHAs"):
        bb.b4_gate(tmp_path / "mixed")


def test_b5_battery_is_refused_without_a_passing_b4(tmp_path: Path, monkeypatch) -> None:
    write_b4_dir(tmp_path / "bad", False)
    started = []
    monkeypatch.setattr(bb, "run_jobs", lambda *a, **k: started.append(1))
    out = tmp_path / "b5"
    assert bb.main(["b5", "--out", str(out), "--b4-dir", str(tmp_path / "bad")]) == 2
    assert started == []
    assert bb.main(["b5", "--out", str(out)]) == 2  # no --b4-dir at all
    assert started == []
    assert bb.main(["o4-check", "--out", str(out), "--b4-dir", str(tmp_path / "bad")]) == 2


def test_freeze_problems_names_changed_trees_and_functions() -> None:
    src = (REPO / "tests" / "l5_reactive_bench" / "track_family_runner.py").read_text()
    same = bb.freeze_problems("old", "new", show=lambda rev, path: src, diff=lambda a, b, p: "")
    assert same == []
    tree = bb.freeze_problems(
        "old",
        "new",
        show=lambda rev, path: src,
        diff=lambda a, b, p: "ros_ws/src/racer_planner/src/mppi.cpp\n",
    )
    assert tree == ["changed since B4: ros_ws/src/racer_planner/src/mppi.cpp"]
    edited = src.replace("def first_scan_at_or_after(", "def first_scan_at_or_after(  # edited\n")
    fn = bb.freeze_problems(
        "old",
        "new",
        show=lambda rev, path: edited if rev == "new" else src,
        diff=lambda a, b, p: "",
    )
    assert fn == ["lap scoring function changed since B4: first_scan_at_or_after"]


def test_freeze_ignores_what_b5_adds_outside_the_lap_scoring() -> None:
    src = (REPO / "tests" / "l5_reactive_bench" / "track_family_runner.py").read_text()
    added = src + "\n\ndef something_new():\n    return 1\n"
    assert (
        bb.freeze_problems(
            "old",
            "new",
            show=lambda rev, path: added if rev == "new" else src,
            diff=lambda a, b, p: "",
        )
        == []
    )


def test_function_sources_refuses_a_missing_function() -> None:
    with pytest.raises(bb.BatteryError):
        bb.function_sources("def a():\n    pass\n", ("b",))


# -- the run machinery with a fake runner ------------------------------------------------------------------


def fake_result(rtf: float = 1.0, gap: float = 0.02, laps: int = 12) -> dict:
    gt = [27.0] + [12.0] * (laps - 1)
    return {
        "gt_lap_times_by_crossing_s": gt,
        "system_lap_events": [{"lap_time_s": t} for t in gt],
        "wall_contacts": 0,
        "real_time_factor": rtf,
        "drive_raw": {"max_gap_s": gap},
    }


def test_starved_rule_is_the_gap_or_the_real_time_factor() -> None:
    assert not bb.starved(fake_result())
    assert bb.starved(fake_result(gap=0.0401))
    assert not bb.starved(fake_result(gap=0.04))
    assert bb.starved(fake_result(rtf=0.79))
    assert not bb.starved(fake_result(rtf=0.8))
    assert not bb.starved({"gt_lap_times_by_crossing_s": [1.0]})  # no information is not starved
    assert bb.needs_rerun({"error": "x"})


def test_run_jobs_resumes_reruns_starved_alone_and_never_overwrites(tmp_path: Path) -> None:
    out = tmp_path / "out"
    jobs = [bb.Job.make("a", "O1", "ccw", 31), bb.Job.make("a", "O1", "cw", 31)]
    calls: list[str] = []
    starve_first = {"a_O1_cw_31": 1}

    def make_command(job, domain, work, result_path):
        calls.append(job.name)
        rtf = 0.5 if starve_first.get(job.name, 0) > 0 and calls.count(job.name) == 1 else 1.0
        script = (
            "import json,sys;"
            f"open({str(result_path)!r},'w').write(json.dumps({fake_result(rtf)!r}))"
        )
        return [sys.executable, "-c", script]

    base = {"id": "B5", "mode": "b5", "prereg_yaml_sha256": "x", "git_sha": "s"}
    bb.run_jobs(jobs, make_command, base, out, workers=2, domain_base=100)
    assert calls.count("a_O1_ccw_31") == 1 and calls.count("a_O1_cw_31") == 2
    final = json.loads((out / "a_O1_cw_31.json").read_text())
    assert final["battery_b5"]["accepted_attempt"] == 2
    assert [a["starved"] for a in final["battery_b5"]["attempts"]] == [True, False]
    assert (out / "attempts" / "a_O1_cw_31.attempt1.json").exists()
    # a second call does nothing: finished runs are never rerun
    n = len(calls)
    bb.run_jobs(jobs, make_command, base, out, workers=2, domain_base=100)
    assert len(calls) == n
    # attempt files are never overwritten
    with pytest.raises(FileExistsError):
        bb.b4t.write_once(out / "attempts" / "a_O1_cw_31.attempt1.json", "x")


def test_a_run_starved_three_times_is_accepted_and_reported_as_such(tmp_path: Path) -> None:
    out = tmp_path / "out"
    jobs = [bb.Job.make("a", "O1", "ccw", 31)]

    def make_command(job, domain, work, result_path):
        script = (
            f"import json;open({str(result_path)!r},'w').write(json.dumps({fake_result(0.4)!r}))"
        )
        return [sys.executable, "-c", script]

    bb.run_jobs(
        jobs,
        make_command,
        {"id": "B5", "mode": "b5", "prereg_yaml_sha256": "x", "git_sha": "s"},
        out,
        1,
        100,
    )
    final = json.loads((out / "a_O1_ccw_31.json").read_text())
    assert (
        final["battery_b5"]["accepted_attempt"] == 3
        and final["battery_b5"]["accepted_while_starved"]
    )


def test_loading_refuses_a_file_of_another_mode_or_without_the_block(tmp_path: Path) -> None:
    r = fake_result()
    (tmp_path / "a_O1_ccw_31.json").write_text(json.dumps({**r, "battery_b5": {"mode": "ma2"}}))
    with pytest.raises(b5.RunFileError):
        b5.load_runs(tmp_path, "b5", b5.B5_CONDITIONS)
    (tmp_path / "a_O1_ccw_31.json").write_text(json.dumps(r))
    with pytest.raises(b5.RunFileError):
        b5.load_runs(tmp_path, "b5", b5.B5_CONDITIONS)


def test_a_directory_never_holds_two_modes(tmp_path: Path) -> None:
    (tmp_path / "a_OD_ccw_11.json").write_text(
        json.dumps({"battery_b5": {"mode": "b5-development"}})
    )
    with pytest.raises(bb.BatteryError, match="separate directory"):
        bb.check_mode_of_directory(tmp_path, "b5")
    bb.check_mode_of_directory(tmp_path, "b5-development")


# -- the registered cases build ------------------------------------------------------------------------------


def test_registered_generated_cases_build_and_meet_the_start_rule() -> None:
    lines = bb.check_obstacle_tracks(CONST, Path("/nonexistent"))
    by_case = {ln.split(":")[0]: ln for ln in lines}
    for case in ("development OD", "battery O1", "battery O2", "battery O3"):
        assert by_case[case].split(":")[1].strip().startswith("ok"), by_case[case]
    # O4 needs the map under car-data: without it the check reports a failure, it does not crash
    assert "FAILED" in by_case["battery O4"]


# -- reruns forced by the host -----------------------------------------------------------------------------


def write_run(
    path: Path, condition: str, case: str, direction: str, laps: int, late: float
) -> None:
    r = fake_result(laps=laps)
    r["gt_lap_times_by_crossing_s"] = [20.0] + [late] * (laps - 1)
    r["battery_b5"] = {"mode": "ma2"}
    (path / f"{condition}_{case}_{direction}_11.json").write_text(json.dumps(r))


def test_a_rerun_directory_replaces_the_first_run_and_says_so(tmp_path: Path) -> None:
    first, rerun = tmp_path / "first", tmp_path / "rerun"
    first.mkdir()
    rerun.mkdir()
    for case in ("T", "S1", "S2", "S3", "S4"):
        for d in DIRS:
            write_run(first, "g", case, d, 6, 10.0)
            write_run(first, "o", case, d, 6, 10.0)
    write_run(first, "o", "S3", "ccw", 2, 10.0)  # the run the host put to sleep
    write_run(rerun, "o", "S3", "ccw", 6, 9.9)
    runs = b5.load_runs(first, "ma2", ("g", "o"))
    merged, notes = bb.merge_reruns(runs, rerun, "ma2", ("g", "o"))
    assert len(merged) == len(runs)
    assert len(notes) == 1 and "o/S3/ccw/11" in notes[0] and "first run stays" in notes[0]
    pairs = b5.pairs_against(merged, "o", "g", CONST)
    assert all(p.valid for p in pairs) and len(pairs) == 10
    assert bb.merge_reruns(runs, None, "ma2", ("g", "o")) == (runs, [])


def test_ma2_only_names_must_be_jobs_of_the_stage(tmp_path: Path, monkeypatch) -> None:
    ran: list = []
    monkeypatch.setattr(bb, "run_jobs", lambda jobs, *a, **k: ran.append([j.name for j in jobs]))
    monkeypatch.setattr(bb.b4t, "check_clean", lambda: None)
    monkeypatch.setattr(bb.b4t, "git_sha", lambda: "0" * 40)
    assert bb.main(["ma2", "--out", str(tmp_path / "o"), "--only", "o_S3_ccw_11"]) == 0
    assert ran == [["o_S3_ccw_11"]]
    assert bb.main(["ma2", "--out", str(tmp_path / "p"), "--only", "o_ZZ_ccw_11"]) == 2


def test_an_invalid_truth_is_rerun_like_a_starved_run_in_b5() -> None:
    ok = {"gt_lap_times_by_crossing_s": [10.0], "truth_valid": True}
    assert not bb.needs_rerun(ok)
    assert bb.needs_rerun({**ok, "truth_valid": False, "truth_invalid_reason": "no_scan_at_start"})
