"""L1 tests of pnp_tick_summary (brief 40): confidence, cap limiting, blocked ticks and the planned
clearance past an obstacle, from PlannerStatus ticks. No ROS."""

from __future__ import annotations

import math

import pnp_tick_summary as pts
import pytest


def tick(t, state=1, cmd=0.5, cap=0.5, conf=1.0, clear=0.3, block=0):
    return pts.Tick(t, state, cmd, cap, conf, clear, block)


def test_summary_counts_only_driving_ticks() -> None:
    ticks = [tick(0.0, state=0, conf=0.0), tick(0.02), tick(0.04, conf=0.5), tick(0.06, state=5)]
    s = pts.tick_summary(ticks)
    assert s["driving_ticks"] == 2
    assert s["mean_track_confidence"] == pytest.approx(0.75)


def test_cap_limited_share_uses_the_documented_margin() -> None:
    inside = 0.5 - pts.CAP_LIMITED_WITHIN_MPS + 1e-6
    outside = 0.5 - pts.CAP_LIMITED_WITHIN_MPS - 1e-3
    s = pts.tick_summary([tick(0.0, cmd=inside), tick(0.02, cmd=outside), tick(0.04, cmd=0.5)])
    assert s["share_cap_limited"] == pytest.approx(2 / 3)
    assert s["share_below_cap"] == pytest.approx(1 / 3)


def test_a_zero_cap_is_never_cap_limited() -> None:
    assert pts.tick_summary([tick(0.0, cmd=0.0, cap=0.0)])["share_cap_limited"] == 0.0


def test_min_clearance_ignores_nan_and_cost_terms_are_declared_missing() -> None:
    s = pts.tick_summary([tick(0.0, clear=0.4), tick(0.02, clear=math.nan), tick(0.04, clear=0.1)])
    assert s["min_plan_clearance_m"] == pytest.approx(0.1)
    assert "not available" in s["cost_terms"]


def test_blocked_ticks_are_counted_by_reason_over_all_states() -> None:
    ticks = [
        tick(0.0, state=5, block=2),
        tick(0.02, state=5, block=2),
        tick(0.04, state=7, block=1),
    ]
    ticks.append(tick(0.06))
    assert pts.tick_summary(ticks)["blocked_ticks_by_reason"] == {"2": 2, "1": 1}


def test_no_driving_ticks() -> None:
    assert pts.tick_summary([tick(0.0, state=0)]) == {"driving_ticks": 0}


def test_obstacle_passes_use_the_ticks_while_the_car_is_near_the_box() -> None:
    # the car drives along y = 0 from x = 0 to 5 at 1 m/s; ticks every 0.1 s; box centre (2.5, 0.3)
    odom = [(0.1 * k, 0.1 * k, 0.0, 0.0, 1.0) for k in range(51)]
    ticks = [
        tick(0.1 * k, clear=1.0 if abs(0.1 * k - 2.5) > 1.0 else 0.2 + 0.1 * abs(0.1 * k - 2.5))
        for k in range(51)
    ]
    (p,) = pts.obstacle_passes(ticks, odom, [(2.5, 0.3, 0.3)])
    assert p["ticks"] > 10
    assert p["min_plan_clearance_m"] == pytest.approx(0.2, abs=1e-6)
    far = pts.obstacle_passes(ticks, odom, [(2.5, 5.0, 0.3)])
    assert far[0]["ticks"] == 0 and math.isnan(far[0]["min_plan_clearance_m"])


def test_obstacle_passes_two_boxes_and_no_odometry() -> None:
    odom = [(0.1 * k, 0.1 * k, 0.0, 0.0, 1.0) for k in range(51)]
    ticks = [tick(0.1 * k, clear=0.5 + (0.3 if k < 25 else 0.0)) for k in range(51)]
    a, b = pts.obstacle_passes(ticks, odom, [(1.0, 0.2, 0.3), (4.0, 0.2, 0.3)])
    assert a["min_plan_clearance_m"] == pytest.approx(0.8)
    assert b["min_plan_clearance_m"] == pytest.approx(0.5)
    none = pts.obstacle_passes(ticks, [], [(1.0, 0.0, 0.3)])
    assert none == [
        {"index": 0, "ticks": 0, "min_plan_clearance_m": none[0]["min_plan_clearance_m"]}
    ]
    assert math.isnan(none[0]["min_plan_clearance_m"])


def test_a_non_driving_tick_near_the_box_is_not_counted() -> None:
    odom = [(0.1 * k, 2.5, 0.0, 0.0, 0.0) for k in range(10)]
    ticks = [tick(0.1 * k, state=5, clear=0.01) for k in range(10)]
    assert pts.obstacle_passes(ticks, odom, [(2.5, 0.3, 0.3)])[0]["ticks"] == 0
