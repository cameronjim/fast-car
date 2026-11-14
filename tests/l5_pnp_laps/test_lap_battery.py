"""The decision functions of the lap learning battery (pre-registration 5.1), without ROS.

Every boundary of every threshold is tested from both sides, because the thresholds are the
pre-registration and a decision that flips at the wrong number is a bug that cannot be found
after the battery has run."""

from __future__ import annotations

import json
import math
from pathlib import Path

import lap_battery as lb
import pytest


def run(
    condition: str = "c",
    case: str = "S1",
    direction: str = "ccw",
    seed: int = 11,
    laps: int = 12,
    t1: float = 20.0,
    late: float = 10.0,
    contacts: int = 0,
    system: list[float] | None = None,
    brakes: list[int] | None = None,
    error: str = "",
) -> lb.RunSummary:
    times = [t1] + [late] * (laps - 1)
    return lb.RunSummary(
        condition=condition,
        case=case,
        direction=direction,
        seed=seed,
        gt_lap_times_s=tuple(times),
        system_lap_times_s=tuple(times if system is None else system),
        contacts=contacts,
        brakes_per_lap=tuple([0] * laps if brakes is None else brakes),
        error=error,
    )


def battery(n: int, **kw: object) -> list[lb.RunSummary]:
    """n runs with distinct keys."""
    cases = [
        (c, d, s) for c in ("S1", "S2", "S3", "S4") for d in ("ccw", "cw") for s in (11, 12, 13)
    ]
    return [run(case=c, direction=d, seed=s, **kw) for c, d, s in cases[:n]]  # type: ignore[arg-type]


def test_thresholds_are_the_preregistered_numbers() -> None:
    assert lb.D_S0_LAP_TIME_TOLERANCE_S == 0.3
    assert lb.D_S1_MAX_CONTACTS == 0 and lb.D_S1_MAX_R == 0.80
    assert lb.D_S2_MAX_CONTACTS == 0 and lb.D_S2_MAX_MEDIAN_RATIO == 0.95
    assert lb.D_S2_MAX_BRAKES_PER_LAP == 0.2
    assert (lb.FULL_BATTERY_RUNS, lb.FULL_BATTERY_REQUIRED, lb.LAPS_TO_COMPLETE) == (24, 22, 10)
    assert lb.LATE_LAPS == (8, 9, 10)


def test_required_completions_scales_22_of_24_to_16_of_18() -> None:
    assert lb.required_completions(24) == 22
    assert lb.required_completions(18) == 16
    assert lb.required_completions(12) == 11
    assert lb.required_completions(0) == 0


# -- run summaries -------------------------------------------------------------------------------


def test_t1_t_late_and_r() -> None:
    times = tuple(float(x) for x in (20, 18, 16, 14, 12, 11, 10.5, 10.0, 9.0, 11.0, 9.5, 9.5))
    r = lb.RunSummary("c", "S1", "ccw", 11, times, times, 0, tuple([0] * 12))
    assert r.t1_s == 20.0
    assert r.t_late_s == 10.0  # median of laps 8, 9, 10: 10.0, 9.0, 11.0
    assert r.r == pytest.approx(0.5)
    assert r.laps == 12 and r.completed


def test_a_short_run_has_no_t_late_and_does_not_complete() -> None:
    short = run(laps=9)
    assert math.isnan(short.t_late_s) and math.isnan(short.r) and not short.completed
    assert run(laps=10).completed and not math.isnan(run(laps=10).t_late_s)
    assert not run(error="node died").completed
    empty = lb.RunSummary("c", "S1", "ccw", 1, (), (), 0, ())
    assert math.isnan(empty.t1_s) and math.isnan(empty.r)


def test_late_brakes_per_lap_is_over_laps_8_to_10() -> None:
    brakes = [5] * 7 + [1, 0, 2] + [7, 7]  # laps 8 to 10 are indices 7, 8, 9
    assert run(brakes=brakes).late_brakes_per_lap == pytest.approx(1.0)
    assert math.isnan(run(brakes=[0] * 8).late_brakes_per_lap)


def test_summarise_run_reads_the_runner_json() -> None:
    result = {
        "gt_lap_times_s": [20.0, 10.0],
        "system_lap_events": [{"lap_time_s": 19.9}, {"lap_time_s": 10.1}],
        "wall_contacts": 2,
        "brake_per_lap": [{"ttc": 1, "ttc_reverse": 1, "watchdog": 5}, {"rate_limit": 2}],
        "stopped_because": "laps",
        "real_time_factor": 1.0,
    }
    s = lb.summarise_run(result, "d", "S3", "cw", 12)
    assert (s.condition, s.case, s.direction, s.seed) == ("d", "S3", "cw", 12)
    assert s.gt_lap_times_s == (20.0, 10.0) and s.system_lap_times_s == (19.9, 10.1)
    assert s.contacts == 2 and s.brakes_per_lap == (2, 0)  # obstacle sources only
    assert s.key == ("S3", "cw", 12)
    fallback = lb.summarise_run({"lap_times_s": [5.0]}, "a", "S1", "ccw", 11)
    assert fallback.gt_lap_times_s == (5.0,)


def test_load_summaries_reads_a_battery_directory(tmp_path: Path) -> None:
    (tmp_path / "c_S2_cw_12.json").write_text(json.dumps({"gt_lap_times_s": [9.0] * 11}))
    (tmp_path / "notes.json").write_text("{}")
    (tmp_path / "x_S2_cw_12.json").write_text("{}")
    runs = lb.load_summaries(tmp_path)
    assert [(r.condition, r.case, r.direction, r.seed) for r in runs] == [("c", "S2", "cw", 12)]


# -- D-S0 ----------------------------------------------------------------------------------------


def test_d_s0_passes_when_counts_and_lap_times_agree() -> None:
    d = lb.decide_d_s0(battery(24, condition="a"))
    assert d.passed is True
    assert "D-S0: PASS" in str(d)


def test_d_s0_boundary_of_the_lap_time_tolerance() -> None:
    times = [20.0] + [10.0] * 11
    ok = [t + 0.29 for t in times]
    bad = [t + 0.31 for t in times]
    assert lb.decide_d_s0([run(condition="a", system=ok)]).passed is True
    assert lb.decide_d_s0([run(condition="a", system=bad)]).passed is False
    one_bad = list(times)
    one_bad[7] += 0.31
    d = lb.decide_d_s0([run(condition="a", system=one_bad)])
    assert d.passed is False and "lap 8" in str(d)


def test_d_s0_fails_on_a_missing_or_extra_lap_or_an_error() -> None:
    times = [20.0] + [10.0] * 11
    assert lb.decide_d_s0([run(condition="a", system=times[:-1])]).passed is False
    assert lb.decide_d_s0([run(condition="a", system=[*times, 10.0])]).passed is False
    assert lb.decide_d_s0([run(condition="a", error="died")]).passed is False


def test_d_s0_without_runs_is_pending() -> None:
    assert lb.decide_d_s0([]).passed is None


# -- D-S1 ----------------------------------------------------------------------------------------


def test_d_s1_passes_with_22_of_24_complete_and_r_within_0_8() -> None:
    runs = battery(24, t1=20.0, late=16.0)  # r = 0.8: at the limit
    assert lb.decide_d_s1(runs).passed is True
    two_slow = [*battery(22, t1=20.0, late=16.0)]
    two_slow += [run(case="S4", direction="cw", seed=s, t1=20.0, late=16.2) for s in (12, 13)]
    d = lb.decide_d_s1(two_slow)
    assert d.passed is True  # 22 of 24 have r <= 0.8


def test_d_s1_fails_with_a_third_run_over_r_0_8_or_incomplete_or_a_contact() -> None:
    runs = battery(21, t1=20.0, late=16.0) + [
        run(case="S4", direction=dd, seed=s, t1=20.0, late=16.2)
        for dd, s in (("cw", 11), ("cw", 12), ("cw", 13))
    ]
    assert lb.decide_d_s1(runs).passed is False
    incomplete = battery(21, t1=20.0, late=10.0) + [
        run(case="S4", direction=dd, seed=s, laps=9)
        for dd, s in (("cw", 11), ("cw", 12), ("cw", 13))
    ]
    assert lb.decide_d_s1(incomplete).passed is False
    contact = battery(24, t1=20.0, late=10.0)
    contact[3] = run(
        case=contact[3].case, direction=contact[3].direction, seed=contact[3].seed, contacts=1
    )
    assert lb.decide_d_s1(contact).passed is False


def test_d_s1_scales_to_16_of_18() -> None:
    ok = battery(16, t1=20.0, late=10.0) + [
        run(case="S4", direction="ccw", seed=s, laps=5) for s in (11, 12)
    ]
    assert len(ok) == 18
    assert lb.decide_d_s1(ok).passed is True
    worse = battery(15, t1=20.0, late=10.0) + [
        run(case="S4", direction="ccw", seed=s, laps=5) for s in (11, 12, 13)
    ]
    assert lb.decide_d_s1(worse).passed is False


def test_d_s1_without_runs_is_pending() -> None:
    assert lb.decide_d_s1([]).passed is None


# -- D-S2 ----------------------------------------------------------------------------------------


def paired(
    ratio: float, n: int = 24, **kw: object
) -> tuple[list[lb.RunSummary], list[lb.RunSummary]]:
    c = battery(n, condition="c", t1=20.0, late=10.0)
    d = battery(n, condition="d", t1=20.0, late=10.0 * ratio, **kw)
    return d, c


def test_d_s2_passes_at_a_median_ratio_of_exactly_0_95() -> None:
    d, c = paired(0.95)
    out = lb.decide_d_s2(d, c)
    assert out.passed is True and out.note == ""


def test_d_s2_reports_no_better_than_the_schedule_when_only_the_ratio_fails() -> None:
    d, c = paired(0.951)
    out = lb.decide_d_s2(d, c)
    assert out.passed is False
    assert "no better than the schedule in sim" in out.note and "D v0" in out.note


def test_d_s2_median_not_mean_over_pairs() -> None:
    c = battery(24, condition="c", t1=20.0, late=10.0)
    d = battery(24, condition="d", t1=20.0, late=9.0)  # ratio 0.9 for 24 pairs
    d[0] = run(condition="d", case=d[0].case, direction=d[0].direction, seed=d[0].seed, late=30.0)
    assert lb.decide_d_s2(d, c).passed is True  # one terrible pair does not move the median


def test_d_s2_safety_items() -> None:
    d, c = paired(0.9, brakes=[0] * 7 + [1, 0, 0] + [0, 0])
    # one brake per run over laps 8 to 10 = 1/3 per lap: above 0.2
    assert lb.decide_d_s2(d, c).passed is False
    note = lb.decide_d_s2(d, c).note
    assert note == ""  # a safety failure is not "no better than the schedule"
    d2, c2 = paired(0.9)
    assert lb.decide_d_s2(d2, c2).passed is True
    # mean exactly 0.2: 12 runs with one brake in laps 8 to 10 out of 24 = mean 1/6 per lap here
    brakes_runs = [
        run(
            condition="d",
            case=c2[i].case,
            direction=c2[i].direction,
            seed=c2[i].seed,
            late=9.0,
            brakes=[0] * 7 + ([3, 0, 0] if i < 4 else [0, 0, 0]) + [0, 0],
        )
        for i in range(24)
    ]
    # 4 runs with 3 brakes in laps 8 to 10: per run 1.0 per lap; mean over 24 runs = 1/6 < 0.2
    assert lb.decide_d_s2(brakes_runs, c2).passed is True
    more = [
        run(
            condition="d",
            case=c2[i].case,
            direction=c2[i].direction,
            seed=c2[i].seed,
            late=9.0,
            brakes=[0] * 7 + ([3, 0, 0] if i < 5 else [0, 0, 0]) + [0, 0],
        )
        for i in range(24)
    ]
    # 5 of 24: mean 5/24 = 0.208 > 0.2
    assert lb.decide_d_s2(more, c2).passed is False


def test_d_s2_contacts_and_completion() -> None:
    d, c = paired(0.9)
    d[5] = run(
        condition="d",
        case=d[5].case,
        direction=d[5].direction,
        seed=d[5].seed,
        late=9.0,
        contacts=1,
    )
    assert lb.decide_d_s2(d, c).passed is False
    d2, c2 = paired(0.9)
    for i in (0, 1, 2):
        d2[i] = run(
            condition="d", case=d2[i].case, direction=d2[i].direction, seed=d2[i].seed, laps=9
        )
    assert lb.decide_d_s2(d2, c2).passed is False  # 21 of 24 complete


def test_d_s2_pairs_by_case_direction_and_seed_and_skips_incomplete_pairs() -> None:
    c = [
        run(condition="c", case="S1", direction="ccw", seed=11, late=10.0),
        run(condition="c", case="S1", direction="cw", seed=11, late=20.0),
        run(condition="c", case="S2", direction="ccw", seed=11, laps=5),
    ]
    d = [
        run(condition="d", case="S1", direction="cw", seed=11, late=10.0),  # 0.5
        run(condition="d", case="S1", direction="ccw", seed=11, late=9.0),  # 0.9
        run(condition="d", case="S2", direction="ccw", seed=11, late=1.0),  # c incomplete: skipped
        run(condition="d", case="S3", direction="ccw", seed=11, late=1.0),  # no c: skipped
    ]
    assert sorted(lb.paired_ratios(d, c)) == pytest.approx([0.5, 0.9])


def test_d_s2_is_pending_without_condition_d_runs() -> None:
    out = lb.decide_d_s2([], battery(24, condition="c"))
    assert out.passed is None and "pending" in out.note


def test_condition_table_and_decision_text() -> None:
    runs = [run(condition="c"), run(condition="d", error="stalled and so on and so forth", laps=3)]
    table = lb.condition_table(runs)
    assert table.count("\n") == 3 and "| c | S1 | ccw | 11 | 12 |" in table
    assert "NO stalled" in table
    text = str(lb.decide_d_s1(battery(24, t1=20.0, late=16.4)))
    assert "FAIL" in text and "D-S1: FAIL" in text
