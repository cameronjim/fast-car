"""L1 tests of the decision functions: every item at, just inside and just outside its threshold."""

from __future__ import annotations

from dataclasses import replace

import pytest
from racer_eval.curves import RunLaps
from racer_eval.gates import (
    GateResult,
    SimRun,
    gp3_decision,
    gp4_decision,
    planner_car_acceptance,
    sim_battery_decision,
)
from racer_eval.interventions import RunInterventions

# --- GP4 (5.2) ----------------------------------------------------------------------------------


def laps(t1: float = 10.0, mid: float = 9.0, late: float = 8.0, n: int = 10) -> tuple[float, ...]:
    """Lap 1 = t1, laps 2 to 7 = mid, laps 8 to 10 = late."""
    return tuple([t1] + [mid] * 6 + [late] * (n - 7))


def on_runs(late: list[float], **kw: object) -> list[RunLaps]:
    return [RunLaps(f"on{i}", "d_on", laps(late=x), **kw) for i, x in enumerate(late)]  # type: ignore[arg-type]


def off_runs(n: int = 5, contacts: int = 0) -> list[RunLaps]:
    return [RunLaps(f"off{i}", "d_off", laps(), contacts=contacts // n) for i in range(n)]


def test_gp4_passes_with_the_edge_value_r_exactly_080() -> None:
    res = gp4_decision(on_runs([8.0] * 5), off_runs())  # r = 8.0 / 10.0 = 0.80 exactly
    assert res.passed and res.outcome == "pass" and res.preregistered
    assert res.item("5.2-2c").measured == 0.8
    assert {i.item_id for i in res.items} == {
        "5.2-0a", "5.2-0b", "5.2-1", "5.2-2a", "5.2-2b", "5.2-2c", "5.2-3",
    }  # fmt: skip


def test_gp4_median_r_just_outside() -> None:
    res = gp4_decision(on_runs([8.01] * 5), off_runs())
    assert not res.item("5.2-2c").passed and not res.passed and res.outcome == "fail"
    assert res.item("5.2-2a").passed and res.item("5.2-2b").passed


def test_gp4_median_not_mean_decides_item_2c() -> None:
    """Two runs with r = 0.95 and three with 0.7: the median is 0.7 and passes."""
    res = gp4_decision(on_runs([9.5, 9.5, 7.0, 7.0, 7.0]), off_runs())
    assert res.item("5.2-2c").passed
    res = gp4_decision(on_runs([9.5, 9.5, 9.5, 7.0, 7.0]), off_runs())
    assert not res.item("5.2-2c").passed


def test_gp4_delta_must_be_positive_in_every_run() -> None:
    inside = gp4_decision(on_runs([8.0, 8.0, 8.0, 8.0, 9.9999]), off_runs())
    outside = gp4_decision(on_runs([8.0, 8.0, 8.0, 8.0, 10.0]), off_runs())  # Delta = 0
    assert inside.item("5.2-2a").passed
    assert not outside.item("5.2-2a").passed and not outside.passed


def test_gp4_interval_of_mean_delta_must_exclude_zero() -> None:
    """Deltas 0.5, 1.5, 0.5, 1.5, 1.0: mean 1, sd 0.5, half width 2.77645 * 0.5 / sqrt(5) = 0.621,
    lower end 0.379 > 0. Deltas 0.1, 0.2, 0.1, 0.2, 2.0: mean 0.52, sd 0.8, interval contains 0
    although every Delta is positive."""
    ok = gp4_decision(on_runs([9.5, 8.5, 9.5, 8.5, 9.0]), off_runs())
    assert ok.item("5.2-2b").measured == pytest.approx(0.379, abs=1e-3)
    assert ok.item("5.2-2b").passed
    noisy = gp4_decision(on_runs([9.9, 9.8, 9.9, 9.8, 8.0]), off_runs())
    assert noisy.item("5.2-2a").passed
    assert not noisy.item("5.2-2b").passed and noisy.item("5.2-2b").measured < 0.0


def test_gp4_dnf_fails_items_1_and_2a_and_is_listed() -> None:
    runs = on_runs([8.0] * 4) + [
        RunLaps("on4", "d_on", (10.0, 9.0, 9.0, 9.0), dnf_after=4, dnf_cause="wall contact")
    ]
    res = gp4_decision(runs, off_runs())
    assert not res.item("5.2-1").passed and res.item("5.2-1").measured == 1
    assert "wall contact" in res.item("5.2-1").note
    assert not res.item("5.2-2a").passed
    assert res.reported["dnf"] == [("on4", "wall contact")]


def test_gp4_needs_five_runs_and_equal_off_runs() -> None:
    four = gp4_decision(on_runs([8.0] * 4), off_runs(4))
    assert not four.item("5.2-0a").passed and not four.passed
    five = gp4_decision(on_runs([8.0] * 5), off_runs(5))
    assert five.item("5.2-0a").passed  # exactly 5 is enough
    assert not gp4_decision(on_runs([8.0] * 5), off_runs(4)).item("5.2-0b").passed


def test_gp4_contacts_are_no_more_on_than_off() -> None:
    def runs_with(contacts: list[int]) -> list[RunLaps]:
        return [RunLaps(f"x{i}", "d", laps(), contacts=c) for i, c in enumerate(contacts)]

    def on(c: list[int]) -> GateResult:
        runs = [replace(r, contacts=c[i]) for i, r in enumerate(on_runs([8.0] * 5))]
        return gp4_decision(runs, runs_with([1] * 5))

    assert on([1, 0, 0, 0, 0]).item("5.2-3").passed
    assert on([1, 1, 1, 1, 1]).item("5.2-3").passed  # 5 against 5, equal is allowed
    assert not on([1, 1, 1, 1, 2]).item("5.2-3").passed  # 6 against 5
    res = on([1, 1, 1, 1, 1])
    assert res.reported["contacts_on"]["contacts"] == 5
    assert res.reported["contacts_on"]["laps"] == 50
    wilson = res.reported["contacts_on"]["wilson"]
    assert wilson["lo"] < 0.1 < wilson["hi"]


def test_gp4_too_few_completed_runs_for_an_interval() -> None:
    res = gp4_decision(on_runs([8.0]), off_runs(1))
    assert res.item("5.2-2b").measured is None and not res.item("5.2-2b").passed
    empty = gp4_decision([RunLaps("x", "d_on", (10.0,), dnf_after=1)], off_runs(1))
    assert empty.item("5.2-2c").measured is None and not empty.item("5.2-2c").passed
    assert empty.reported["contacts_on"]["wilson"] is not None


def test_gp4_contact_rate_without_laps_has_no_wilson_interval() -> None:
    res = gp4_decision(on_runs([8.0] * 5), [RunLaps("o", "d_off", ())])
    assert res.reported["contacts_off"] == {"contacts": 0, "laps": 0, "wilson": None}


def test_gp4_results_serialise_and_flag_changed_thresholds() -> None:
    res = gp4_decision(on_runs([8.0] * 5), off_runs(), max_median_r=0.9)
    assert not res.preregistered
    d = res.to_dict()
    assert d["name"] == "gp4" and d["passed"] and len(d["items"]) == 7
    with pytest.raises(KeyError):
        res.item("nope")


# --- GP3 (4.3) ----------------------------------------------------------------------------------

GP3 = {
    "laps_counted": 20, "wheel_distance_m": 18.0, "lap_length_m": 1.0, "contacts": 0,
    "longest_blocked_hold_s": 5.0, "recoveries": 2, "cap_mps": 1.0, "direction": "ccw",
}  # fmt: skip


def test_gp3_passes_at_every_edge() -> None:
    res = gp3_decision(**GP3)  # 20 laps, 0.9 * 20 * 1.0 = 18.0 m, a 5.0 s hold, all at the edge
    assert res.passed and res.outcome == "pass" and res.preregistered
    assert res.reported["recoveries"] == 2 and res.reported["attempt_kind"] == "gp3_result"
    assert len(res.items) == 7


@pytest.mark.parametrize(
    "item,change",
    [
        ("4.3-a", {"laps_counted": 19}),
        ("4.3-b", {"wheel_distance_m": 17.99}),
        ("4.3-c", {"contacts": 1}),
        ("4.3-d", {"longest_blocked_hold_s": 5.01}),
        ("4.3-e", {"cap_mps": 0.9}),
        ("4.3-f", {"hand_touches": 1}),
        ("4.3-g", {"direction": "cw"}),
    ],
)
def test_gp3_each_item_just_outside_fails(item: str, change: dict[str, object]) -> None:
    res = gp3_decision(**{**GP3, **change})  # type: ignore[arg-type]
    assert not res.item(item).passed and not res.passed and res.outcome == "fail"
    assert sum(not i.passed for i in res.items) == 1


def test_gp3_repeat_attempts_may_run_cw_and_are_labelled() -> None:
    res = gp3_decision(**{**GP3, "direction": "cw"}, first_attempt=False)  # type: ignore[arg-type]
    assert res.passed and res.reported["attempt_kind"] == "repeat"
    assert len(res.items) == 6


def test_gp3_recoveries_never_fail_and_changed_thresholds_are_flagged() -> None:
    assert gp3_decision(**{**GP3, "recoveries": 50}).passed  # type: ignore[arg-type]
    assert not gp3_decision(**GP3, n_laps=19).preregistered  # type: ignore[arg-type]


# --- planner accepted on the car (4.2) ----------------------------------------------------------


def iv(
    run_id: str = "R", duration: float = 40.0, start_event: bool = False, **counts: int
) -> RunInterventions:
    base = {"watchdog": 0, "rate_limit": 0, "ttc": 0, "ttc_reverse": 0}
    return RunInterventions(run_id, duration, {**base, **counts}, start_event)


def eight(duration: float = 40.0) -> tuple[list[RunInterventions], list[float]]:
    rows = [iv(f"R{i}", duration) for i in range(8)]
    return rows, [0.5, 1.0] * 4


def test_acceptance_passes_at_the_edges() -> None:
    """8 starts of 37.5 s is exactly 300 s; 7 of 8 starts clean; the first start has one event."""
    rows, caps = eight(37.5)
    rows[0] = iv("R0", 37.5, start_event=True)
    res = planner_car_acceptance(rows, caps, 0)
    assert res.passed and res.outcome == "accepted" and res.preregistered
    assert res.item("4.2-f").measured == 7
    assert len(res.items) == 8


def test_acceptance_duration_just_outside() -> None:
    rows, caps = eight(37.49)
    res = planner_car_acceptance(rows, caps, 0)
    assert not res.item("4.2-a").passed and res.outcome == "miss"


def test_acceptance_rate_limits_per_minute() -> None:
    """10 runs of 90 s = 15 min: 3 events is exactly 0.2 per minute, 4 is 0.27."""

    def run(w: int, r: int) -> GateResult:
        rows = [iv(f"R{i}", 90.0) for i in range(10)]
        rows[1] = iv("R1", 90.0, watchdog=w, rate_limit=r)
        return planner_car_acceptance(rows, [0.5, 1.0] * 5, 0)

    assert run(3, 3).item("4.2-d").passed and run(3, 3).item("4.2-e").passed
    assert not run(4, 0).item("4.2-d").passed
    assert not run(0, 4).item("4.2-e").passed
    assert run(0, 4).item("4.2-d").passed


def test_acceptance_clean_starts_need_seven_of_eight_and_scale() -> None:
    rows, caps = eight()
    rows[0], rows[1] = iv("R0", start_event=True), iv("R1", start_event=True)
    assert not planner_car_acceptance(rows, caps, 0).item("4.2-f").passed  # 6 of 8
    ten = [iv(f"R{i}", 40.0) for i in range(10)]
    ten[0] = iv("R0", 40.0, start_event=True)
    assert planner_car_acceptance(ten, [0.5, 1.0] * 5, 0).item("4.2-f").passed  # 9 >= ceil(8.75)
    ten[1] = iv("R1", 40.0, start_event=True)
    assert not planner_car_acceptance(ten, [0.5, 1.0] * 5, 0).item("4.2-f").passed  # 8 < 9


def test_acceptance_starts_caps_obstacles_and_contacts() -> None:
    rows, caps = eight()
    assert not planner_car_acceptance(rows[:7], caps[:7], 0).item("4.2-b").passed  # 7 starts
    assert not planner_car_acceptance(rows, [0.5] * 8, 0).item("4.2-c").passed  # one cap only
    assert not planner_car_acceptance(rows, [0.7] * 8, 0).item("4.2-b").passed  # not start caps
    rows[3] = iv("R3", ttc=1)
    assert not planner_car_acceptance(rows, caps, 0).item("4.2-g").passed
    rows[3] = iv("R3", ttc_reverse=1)
    assert not planner_car_acceptance(rows, caps, 0).item("4.2-g").passed
    # a brake in a run capped above 1.0 is outside "caps up to 1.0" (and not a start cap)
    assert (
        planner_car_acceptance(rows, [0.5, 1.0, 0.5, 1.5, 0.5, 1.0, 0.5, 1.0], 0)
        .item("4.2-g")
        .passed
    )
    rows[3] = iv("R3")
    assert planner_car_acceptance(rows, caps, 0).item("4.2-h").passed
    assert not planner_car_acceptance(rows, caps, 1).item("4.2-h").passed


def test_acceptance_ignores_runs_that_never_commanded_and_checks_lengths() -> None:
    rows, caps = eight()
    rows.append(RunInterventions("N", 0.0, {}, False, commanded=False))
    caps.append(0.5)
    assert planner_car_acceptance(rows, caps, 0).item("4.2-b").measured == 8
    with pytest.raises(ValueError):
        planner_car_acceptance(rows, caps[:-1], 0)


def test_acceptance_with_no_driving_has_no_rates() -> None:
    res = planner_car_acceptance([], [], 0)
    assert res.item("4.2-d").measured is None and not res.passed
    assert not planner_car_acceptance(*eight(), 0, min_starts=7).preregistered


# --- sim battery (5.1) --------------------------------------------------------------------------

KEYS = [(c, d, s) for c in ("S1", "S2", "S3", "S4") for d in ("ccw", "cw") for s in (11, 12, 13)]


def gt(t1: float, late: float, n: int = 12) -> tuple[float, ...]:
    return tuple([t1] + [9.0] * 6 + [late] * (n - 7))


def battery(
    cond: str,
    late: float = 8.0,
    keys: list[tuple[str, str, int]] = KEYS,
    **kw: object,
) -> list[SimRun]:
    return [
        SimRun(cond, c, d, s, gt(10.0, late), gt(10.0, late), 0, (0,) * 12, **kw)  # type: ignore[arg-type]
        for c, d, s in keys
    ]


def full(**over: list[SimRun]) -> list[SimRun]:
    parts = {"a": battery("a", 10.0), "b": battery("b", 9.9), "c": battery("c"),
             "d": battery("d", 7.6)}  # fmt: skip
    parts.update(over)
    return [r for p in parts.values() for r in p]


def test_battery_all_pass_at_the_edges() -> None:
    """(c) r = 8.0 / 10.0 = 0.80 exactly; (d) ratio 7.6 / 8.0 = 0.95 exactly."""
    res = sim_battery_decision(full())
    assert res.ds0.passed and res.ds1.passed and res.ds2.passed
    assert (res.ds0.outcome, res.ds1.outcome, res.ds2.outcome) == (
        "lap_source_ok", "d_v0_on_car", "d_v1_on_car",
    )  # fmt: skip
    assert res.ds1.item("D-S1-2").measured == 24 and res.ds1.item("D-S1-2").threshold == 22
    assert res.ds2.item("D-S2-3").measured == 0.95
    assert res.control_b["runs"] == 24 and res.control_b["median_t_late_s"] == 9.9
    assert res.control_b["median_t_late_c_s"] == 8.0 and res.control_b["median_t_late_d_s"] == 7.6
    assert res.ds0.preregistered and res.to_dict()["D-S2"]["outcome"] == "d_v1_on_car"


def test_ds0_lap_count_and_lap_time_tolerance() -> None:
    a = battery("a", 10.0)
    short = replace(a[0], system_lap_times=a[0].system_lap_times[:-1])
    res = sim_battery_decision(full(a=[short, *a[1:]]))
    assert not res.ds0.item("D-S0-1").passed and res.ds0.item("D-S0-1").measured == 1
    assert res.ds0.outcome == "fail"
    off = replace(a[0], system_lap_times=(10.5,) + a[0].gt_lap_times[1:])
    assert not sim_battery_decision(full(a=[off, *a[1:]])).ds0.item("D-S0-2").passed
    near = replace(a[0], system_lap_times=(10.25,) + a[0].gt_lap_times[1:])  # 0.25 s off
    ok = sim_battery_decision(full(a=[near, *a[1:]]))
    assert ok.ds0.item("D-S0-2").passed and ok.ds0.item("D-S0-2").measured == 0.25
    at = sim_battery_decision(full(a=[near, *a[1:]]), lap_tolerance_s=0.25)  # at the threshold
    assert at.ds0.item("D-S0-2").passed and not at.ds0.preregistered


def test_ds1_contacts_and_completion_with_r() -> None:
    c = battery("c")
    # 22 runs with r = 0.80 pass, 2 with r = 0.81 do not: 22 of 24 is exactly enough
    bad = [replace(r, gt_lap_times=gt(10.0, 8.1), system_lap_times=gt(10.0, 8.1)) for r in c[:2]]
    assert sim_battery_decision(full(c=bad + c[2:])).ds1.passed
    # a third one over 0.80: 21 of 24
    bad3 = [replace(r, gt_lap_times=gt(10.0, 8.1)) for r in c[:3]]
    res = sim_battery_decision(full(c=bad3 + c[3:]))
    assert not res.ds1.item("D-S1-2").passed and res.ds1.item("D-S1-2").measured == 21
    assert res.ds1.outcome == "fail"
    # a run with 9 laps has not completed 10
    nine = replace(c[0], gt_lap_times=gt(10.0, 8.0, 9))
    assert sim_battery_decision(full(c=[nine, *c[1:]])).ds1.item("D-S1-2").measured == 23
    # one wall contact
    hit = replace(c[0], wall_contacts=1)
    assert not sim_battery_decision(full(c=[hit, *c[1:]])).ds1.item("D-S1-1").passed


def test_battery_of_18_runs_scales_to_16() -> None:
    keys = KEYS[:18]
    parts = {
        k: battery(k, late, keys) for k, late in zip("abcd", (10.0, 9.9, 8.0, 7.6), strict=True)
    }
    runs = [r for p in parts.values() for r in p]
    assert sim_battery_decision(runs).ds1.item("D-S1-2").threshold == 16
    c = parts["c"]
    bad = [replace(r, gt_lap_times=gt(10.0, 8.1)) for r in c[:3]]  # 15 of 18
    runs = [r for k in "abd" for r in parts[k]] + bad + c[3:]
    assert not sim_battery_decision(runs).ds1.passed


def test_ds2_ratio_is_paired_and_the_outcome_names_the_failure() -> None:
    d_slow = battery("d", 7.61)  # ratio 0.95125, safety all fine
    res = sim_battery_decision(full(d=d_slow))
    assert not res.ds2.item("D-S2-3").passed
    assert res.ds2.outcome == "no_better_than_schedule"
    # pairing: (c) is slower (9.0) in 11 of 24 cases; those pairs have ratio 0.844, the other 13
    # have 0.95, so the median over pairs is 0.95 (both middle values)
    c = battery("c")
    c2 = [replace(r, gt_lap_times=gt(10.0, 9.0)) if i < 11 else r for i, r in enumerate(c)]
    res = sim_battery_decision(full(c=c2, d=battery("d", 7.6)))
    assert res.ds2.item("D-S2-3").measured == pytest.approx(0.95)


def test_ds2_safety_items_and_unpaired_runs() -> None:
    d = battery("d", 7.6)
    assert (
        sim_battery_decision(full(d=[replace(d[0], wall_contacts=1), *d[1:]])).ds2.outcome == "fail"
    )

    # 14 brakes in laps 8 to 10 over 24 runs: 14 / 72 = 0.194 passes, 15 / 72 = 0.208 fails
    def with_brakes(n: int) -> list[SimRun]:
        out = []
        for i, r in enumerate(d):
            brakes = [0] * 12
            if i < n:
                brakes[8] = 1
            out.append(replace(r, brakes_per_lap=tuple(brakes)))
        return out

    assert sim_battery_decision(full(d=with_brakes(14))).ds2.item("D-S2-4").passed
    res = sim_battery_decision(full(d=with_brakes(15)))
    assert not res.ds2.item("D-S2-4").passed and res.ds2.outcome == "fail"
    # a (d) run that did not finish 10 laps: not paired, and one fewer completed (23 of 24 >= 22)
    dnf = replace(d[0], gt_lap_times=gt(10.0, 7.6, 8))
    res = sim_battery_decision(full(d=[dnf, *d[1:]]))
    assert res.ds2.item("D-S2-2").measured == 23 and res.ds2.item("D-S2-3").note == "23 pairs of 24"
    two = [replace(r, gt_lap_times=gt(10.0, 7.6, 8)) for r in d[:3]]
    assert not sim_battery_decision(full(d=two + d[3:])).ds2.item("D-S2-2").passed


def test_ds2_without_any_pair_or_late_laps_fails_cleanly() -> None:
    d = [replace(r, gt_lap_times=gt(10.0, 7.6, 8), brakes_per_lap=()) for r in battery("d")]
    res = sim_battery_decision(full(d=d))
    assert res.ds2.item("D-S2-3").measured is None and res.ds2.item("D-S2-4").measured is None
    assert res.ds2.outcome == "fail"


def test_battery_refusals() -> None:
    with pytest.raises(ValueError, match="no runs of condition c"):
        sim_battery_decision(battery("a") + battery("d"))
    with pytest.raises(ValueError, match="24 runs"):
        sim_battery_decision(full(c=battery("c")[:20]))
    with pytest.raises(ValueError, match="24 runs"):
        sim_battery_decision(full(b=battery("b")[:18]))


def test_battery_control_b_may_be_absent() -> None:
    res = sim_battery_decision(full(b=[]))
    assert res.control_b["runs"] == 0 and res.control_b["median_t_late_s"] is None
