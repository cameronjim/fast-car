"""L1 tests of lap tables, lap-index curves and the interleaved run order."""

from __future__ import annotations

from itertools import pairwise

import numpy as np
import pytest
from racer_eval import interleave
from racer_eval.curves import RunLaps, lap_curve, run_metrics
from racer_eval.interleave import run_order
from racer_eval.laps import (
    in_run_mask,
    lap_table,
    lap_times_of,
    run_wheel_distance,
    wheel_distance,
)

# --- laps ---------------------------------------------------------------------------------------


def test_wheel_distance_integrates_the_absolute_speed() -> None:
    """v = 1 m/s over [2, 6] is 4 m. A ramp |v| = t over [2, 6] is (2 + 6) / 2 * 4 = 16 m,
    with the ends interpolated between the samples at 0 and 10. A reverse run counts as path."""
    t = np.array([0.0, 10.0])
    assert wheel_distance(np.linspace(0, 10, 11), np.ones(11), 2.0, 6.0) == pytest.approx(4.0)
    assert wheel_distance(t, [0.0, 10.0], 2.0, 6.0) == pytest.approx(16.0)
    assert wheel_distance(t, [0.0, -10.0], 2.0, 6.0) == pytest.approx(16.0)
    assert wheel_distance(t, [1.0, 1.0], -1.0, 6.0) is None  # starts before the odometry
    assert wheel_distance(t, [1.0, 1.0], 2.0, 11.0) is None  # ends after it
    assert wheel_distance(t, [1.0, 1.0], 6.0, 2.0) is None
    assert wheel_distance([0.0], [1.0], 0.0, 0.0) is None
    with pytest.raises(ValueError):
        run_wheel_distance(t, [1.0, 1.0], 2.0, 11.0)
    assert run_wheel_distance(t, [1.0, 1.0], 2.0, 6.0) == pytest.approx(4.0)


def test_in_run_mask_edges_and_margin() -> None:
    mask = in_run_mask([9.9, 10.0, 20.0, 23.0, 23.1], 10.0, 20.0)
    assert mask.tolist() == [False, True, True, True, False]


ODOM_T = np.linspace(0.0, 40.0, 401)
ODOM_V = np.full(401, 0.5)


def test_lap_table_rows_and_flags() -> None:
    """Camera laps 10.5, 10.75, 10.4 s; pnp laps 10.25 (differs by 0.25, not flagged) and 10.15
    (differs by 0.60, flagged); lap 3 has no pnp event. Wheel distance is 0.5 m/s times the lap
    time of the camera event."""
    rows = lap_table(
        "r1",
        camera_event_t=[10.5, 21.25, 31.65],
        camera_lap_time_s=[10.5, 10.75, 10.4],
        camera_distance_m=[5.0, 5.0, 5.0],
        pnp_event_t=[10.6, 21.4],
        pnp_lap_time_s=[10.25, 10.15],
        odom_t=ODOM_T,
        odom_v=ODOM_V,
    )
    assert [r.lap for r in rows] == [1, 2, 3]
    assert rows[0].diff_s == pytest.approx(0.25) and not rows[0].flag_differs
    assert rows[1].diff_s == pytest.approx(0.60) and rows[1].flag_differs
    assert rows[2].diff_s is None and not rows[2].flag_differs
    assert rows[2].flag_missing_pnp and not rows[2].flag_missing_camera
    assert not rows[0].flag_missing_pnp
    assert rows[0].wheel_distance_m == pytest.approx(5.25)
    assert rows[2].wheel_distance_m == pytest.approx(5.2)
    assert rows[0].camera_distance_m == 5.0
    assert rows[0].to_dict()["run_id"] == "r1"
    assert lap_times_of(rows) == (10.5, 10.75, 10.4)


def test_lap_table_with_no_camera_uses_the_pnp_event_and_flags_it() -> None:
    rows = lap_table("r2", [], [], [], [10.5], [10.0], ODOM_T, ODOM_V)
    assert len(rows) == 1
    assert rows[0].flag_missing_camera and not rows[0].flag_missing_pnp
    assert rows[0].camera_time_s is None and rows[0].diff_s is None
    assert rows[0].wheel_distance_m == pytest.approx(5.0)
    assert lap_times_of(rows) == ()


def test_lap_table_wheel_distance_none_when_odometry_does_not_cover() -> None:
    rows = lap_table("r3", [100.0], [10.0], [5.0], [], [], ODOM_T, ODOM_V)
    assert rows[0].wheel_distance_m is None and rows[0].flag_missing_pnp


def test_lap_table_refuses_ragged_sources() -> None:
    with pytest.raises(ValueError):
        lap_table("r", [1.0, 2.0], [1.0], [1.0, 1.0], [], [], ODOM_T, ODOM_V)
    with pytest.raises(ValueError):
        lap_table("r", [], [], [], [1.0], [], ODOM_T, ODOM_V)


# --- curves -------------------------------------------------------------------------------------

LAPS = (10.0, 9.5, 9.0, 8.8, 8.7, 8.6, 8.5, 8.4, 8.2, 8.0)


def test_run_metrics() -> None:
    """T1 = 10.0, T_late = median of laps 8 to 10 = median(8.4, 8.2, 8.0) = 8.2, Delta = 1.8,
    r = 0.82."""
    m = run_metrics(RunLaps("a", "c", LAPS))
    assert m is not None
    assert (m.t1, m.t_late) == (10.0, 8.2)
    assert m.delta == pytest.approx(1.8) and m.ratio == pytest.approx(0.82)
    assert run_metrics(RunLaps("a", "c", LAPS[:9])) is None
    assert run_metrics(RunLaps("a", "c", LAPS, dnf_after=9)) is None  # lap 10 is beyond the DNF


def test_usable_laps_stop_at_target_and_at_a_dnf() -> None:
    long = tuple(float(x) for x in range(1, 16))
    assert len(RunLaps("a", "c", long).usable) == 10
    assert len(RunLaps("a", "c", long, dnf_after=4).usable) == 4
    assert len(RunLaps("a", "c", long, dnf_after=-1).usable) == 0
    assert len(RunLaps("a", "c", long, n_target=12).usable) == 12
    assert RunLaps("a", "c", long).completed
    assert not RunLaps("a", "c", long, dnf_after=9).completed
    assert not RunLaps("a", "c", long[:9]).completed


def test_lap_curve_means_intervals_and_dnf() -> None:
    """Three runs. Lap 1: 10, 12, 8 (mean 10, sd 2, t(0.975, 2) = 4.30265, half 4.97). The third
    run stops after 1 lap (DNF) and is never imputed, so lap 2 has n = 2."""
    a = RunLaps("a", "c", (10.0, 9.0), dnf_after=None, n_target=2)
    b = RunLaps("b", "c", (12.0, 11.0), n_target=2)
    c = RunLaps("c", "c", (8.0,), dnf_after=1, dnf_cause="stall", n_target=2)
    curve = lap_curve([a, b, c])
    p1, p2 = curve.points
    assert (p1.lap, p1.n, p1.mean) == (1, 3, 10.0)
    assert p1.interval is not None
    assert p1.interval.lo == pytest.approx(10.0 - 4.30265 * 2.0 / np.sqrt(3), abs=1e-3)
    assert (p2.lap, p2.n, p2.mean) == (2, 2, 10.0)
    assert curve.n_runs == 3 and curve.n_completed == 2
    assert curve.dnf == (("c", "stall"),)
    assert curve.completion.estimate == pytest.approx(2 / 3)
    assert curve.completion.lo < 2 / 3 < curve.completion.hi


def test_lap_curve_single_run_has_no_interval_and_skips_lapless_indices() -> None:
    curve = lap_curve([RunLaps("a", "c", (9.0, 8.0))])
    assert [p.lap for p in curve.points] == [1, 2]
    assert all(p.interval is None for p in curve.points)


def test_lap_curve_refusals() -> None:
    with pytest.raises(ValueError):
        lap_curve([])
    with pytest.raises(ValueError):
        lap_curve([RunLaps("a", "x", LAPS), RunLaps("b", "y", LAPS)])


# --- interleave ---------------------------------------------------------------------------------


def _max_in_a_row(order: list[str]) -> int:
    best = run = 1
    for a, b in pairwise(order):
        run = run + 1 if a == b else 1
        best = max(best, run)
    return best


def test_run_order_is_balanced_seeded_and_limited_to_two_in_a_row() -> None:
    conds = ["on", "off_a", "off_b", "race"]
    order = run_order(conds, 5, seed=2026)
    assert len(order) == 20
    assert all(order.count(c) == 5 for c in conds)
    assert _max_in_a_row(order) <= 2
    assert order == run_order(conds, 5, seed=2026)
    assert order != run_order(conds, 5, seed=2027)


def test_run_order_tight_cases() -> None:
    assert run_order(["a"], 2, 0) == ["a", "a"]
    order = run_order(["a", "b"], {"a": 6, "b": 2}, 5)  # aabaabaa: the only shape that fits
    assert order.count("a") == 6 and _max_in_a_row(order) <= 2
    uneven = run_order(["a", "b", "c"], {"a": 1, "b": 0, "c": 3}, 1)  # c c a c or c a c c
    assert sorted(uneven) == ["a", "c", "c", "c"] and _max_in_a_row(uneven) <= 2


def test_run_order_search_backtracks_and_gives_up_when_the_pruning_is_off(
    monkeypatch: pytest.MonkeyPatch,
) -> None:
    """With the feasibility pruning disabled the search itself must still never return an order
    that breaks the rule: it exhausts every order and raises."""
    monkeypatch.setattr(interleave, "_feasible", lambda *_a: True)
    with pytest.raises(ValueError, match="in a row"):
        run_order(["a", "b"], {"a": 5, "b": 1}, 0)
    order = run_order(["a", "b"], {"a": 4, "b": 2}, 3)
    assert order.count("a") == 4 and _max_in_a_row(order) <= 2


def test_run_order_impossible_cases_raise() -> None:
    with pytest.raises(ValueError, match="in a row"):
        run_order(["a"], 3, 0)
    with pytest.raises(ValueError, match="in a row"):
        run_order(["a", "b"], {"a": 7, "b": 2}, 0)
    with pytest.raises(ValueError):
        run_order([], 3, 0)
    with pytest.raises(ValueError):
        run_order(["a", "a"], 3, 0)
    with pytest.raises(ValueError):
        run_order(["a", "b"], {"a": 1}, 0)
    with pytest.raises(ValueError):
        run_order(["a", "b"], 0, 0)
    with pytest.raises(ValueError):
        run_order(["a", "b"], {"a": -1, "b": 3}, 0)
