# pure pursuit geometry on a synthetic raceline, plus one real lap of spielberg

import math

import numpy as np
import pytest

from f1rl.planners import PurePursuitConfig, PurePursuitPlanner
from f1rl.track import RacelineIndex

CIRCLE_RADIUS_M = 10.0
CIRCLE_POINTS = 720
WHEELBASE_M = 0.3302


def circle_line(speeds=None) -> RacelineIndex:
    """counter-clockwise circular raceline of radius 10 m starting at (10, 0)."""
    angles = np.linspace(0.0, 2.0 * np.pi, CIRCLE_POINTS, endpoint=False)
    xs = CIRCLE_RADIUS_M * np.cos(angles)
    ys = CIRCLE_RADIUS_M * np.sin(angles)
    if speeds is None:
        speeds = np.full(CIRCLE_POINTS, 3.0)
    return RacelineIndex(xs, ys, speeds=speeds)


def circle_planner(**overrides) -> PurePursuitPlanner:
    """planner on the synthetic circle with the pose taken at the rear axle."""
    settings = {"cog_to_rear_axle_m": 0.0, "wheelbase_m": WHEELBASE_M}
    settings.update(overrides)
    return PurePursuitPlanner(circle_line(), PurePursuitConfig(**settings))


def test_arclength_matches_the_circumference():
    line = circle_line()
    assert line.length == pytest.approx(2.0 * math.pi * CIRCLE_RADIUS_M, rel=1e-4)
    assert line.has_speed_profile


def test_projection_returns_arclength_and_signed_offset():
    line = circle_line()
    # a quarter turn counter-clockwise, one metre outside the circle, so to the car's right
    s, lateral = line.project(0.0, CIRCLE_RADIUS_M + 1.0)
    assert s == pytest.approx(0.25 * line.length, abs=0.05)
    assert lateral == pytest.approx(-1.0, abs=0.01)
    _, inside = line.project(0.0, CIRCLE_RADIUS_M - 1.0)
    assert inside == pytest.approx(1.0, abs=0.01)


def test_lookahead_point_is_that_far_along_the_line():
    line = circle_line()
    s, _ = line.project(CIRCLE_RADIUS_M, 0.0)
    goal_x, goal_y = line.point_at(s + 2.0)
    expected = 2.0 / CIRCLE_RADIUS_M
    assert math.hypot(goal_x, goal_y) == pytest.approx(CIRCLE_RADIUS_M, rel=1e-3)
    assert math.atan2(goal_y, goal_x) == pytest.approx(expected, abs=1e-3)


def test_lookahead_wraps_around_the_start():
    line = circle_line()
    before = line.point_at(-0.5)
    same = line.point_at(line.length - 0.5)
    assert before == pytest.approx(same, abs=1e-9)
    after = line.point_at(line.length + 0.5)
    assert after == pytest.approx(line.point_at(0.5), abs=1e-9)


def test_projection_just_before_the_start_stays_near_the_lap_end():
    line = circle_line()
    step_back = 0.4
    angle = -step_back / CIRCLE_RADIUS_M
    s, _ = line.project(CIRCLE_RADIUS_M * math.cos(angle), CIRCLE_RADIUS_M * math.sin(angle))
    assert s == pytest.approx(line.length - step_back, abs=0.05)


def test_lookahead_scales_with_speed_and_clamps():
    planner = circle_planner(lookahead_gain_sec=0.5, lookahead_min_m=1.0, lookahead_max_m=3.0)
    assert planner.lookahead_m(0.0) == pytest.approx(1.0)
    assert planner.lookahead_m(2.0) == pytest.approx(2.0)
    assert planner.lookahead_m(20.0) == pytest.approx(3.0)
    assert planner.lookahead_m(-5.0) == pytest.approx(1.0)


def test_on_the_line_steers_towards_the_circle_centre():
    planner = circle_planner(lookahead_gain_sec=0.0, lookahead_min_m=1.0)
    # at (10, 0) heading +y, the line curves left, so the wheel goes left
    steering, _ = planner.plan(CIRCLE_RADIUS_M, 0.0, math.pi / 2.0, 3.0)
    assert steering > 0.0
    assert steering == pytest.approx(math.atan(WHEELBASE_M / CIRCLE_RADIUS_M), abs=5e-3)


def test_offset_to_the_left_steers_back_right():
    planner = circle_planner(lookahead_gain_sec=0.0, lookahead_min_m=2.0)
    on_line, _ = planner.plan(CIRCLE_RADIUS_M, 0.0, math.pi / 2.0, 3.0)
    # running counter-clockwise, a car inside the circle sits to the left of the line
    offset, _ = planner.plan(CIRCLE_RADIUS_M - 1.0, 0.0, math.pi / 2.0, 3.0)
    assert offset < on_line
    mirrored, _ = planner.plan(CIRCLE_RADIUS_M + 1.0, 0.0, math.pi / 2.0, 3.0)
    assert mirrored > on_line


def test_steering_is_clipped_at_the_vehicle_limit():
    planner = circle_planner(steer_max_rad=0.4189)
    # a goal 0.2 m off the nose asks for a curvature no front wheel can hold
    assert planner.steering_to(0.0, 0.0, 0.0, 0.0, 0.2) == pytest.approx(0.4189)
    assert planner.steering_to(0.0, 0.0, 0.0, 0.0, -0.2) == pytest.approx(-0.4189)


def test_goal_on_top_of_the_car_holds_the_wheel_straight():
    planner = circle_planner()
    assert planner.steering_to(1.0, 2.0, 0.3, 1.0, 2.0) == 0.0


def test_speed_command_follows_the_raceline_profile():
    angles = np.linspace(0.0, 2.0 * np.pi, CIRCLE_POINTS, endpoint=False)
    speeds = 5.0 + 2.0 * np.cos(angles)
    line = RacelineIndex(
        CIRCLE_RADIUS_M * np.cos(angles), CIRCLE_RADIUS_M * np.sin(angles), speeds=speeds
    )
    planner = PurePursuitPlanner(line, PurePursuitConfig(speed_scale=0.5, cog_to_rear_axle_m=0.0))
    assert planner.speed_at(0.0) == pytest.approx(3.5, abs=1e-3)
    assert planner.speed_at(0.5 * line.length) == pytest.approx(1.5, abs=1e-3)
    assert planner.speed_at(line.length) == pytest.approx(3.5, abs=1e-3)


def test_speed_falls_back_when_the_line_has_no_profile():
    angles = np.linspace(0.0, 2.0 * np.pi, CIRCLE_POINTS, endpoint=False)
    line = RacelineIndex(CIRCLE_RADIUS_M * np.cos(angles), CIRCLE_RADIUS_M * np.sin(angles))
    planner = PurePursuitPlanner(line, PurePursuitConfig(fallback_speed_mps=2.5))
    assert not planner.has_speed_profile
    assert planner.speed_at(12.0) == pytest.approx(2.5)


def test_config_rejects_unknown_keys_and_bad_bands():
    with pytest.raises(ValueError, match="unknown pure pursuit config keys"):
        PurePursuitConfig.from_dict({"lookahead_gain": 0.3})
    with pytest.raises(ValueError):
        PurePursuitConfig(lookahead_min_m=3.0, lookahead_max_m=1.0)
    with pytest.raises(ValueError):
        PurePursuitConfig(speed_scale=0.0)


def test_duplicated_closing_waypoint_is_dropped():
    angles = np.linspace(0.0, 2.0 * np.pi, 61)
    line = RacelineIndex(CIRCLE_RADIUS_M * np.cos(angles), CIRCLE_RADIUS_M * np.sin(angles))
    assert line.n == 60
    assert line.length == pytest.approx(2.0 * math.pi * CIRCLE_RADIUS_M, rel=1e-2)


@pytest.mark.slow
def test_pure_pursuit_laps_spielberg_without_crashing():
    from f1rl.run_planner import build_planner_env, run_laps

    env = build_planner_env("Spielberg", seed=0)
    planner = PurePursuitPlanner(env.unwrapped.track, PurePursuitConfig())
    assert planner.has_speed_profile
    result = run_laps(env, planner, laps=1, seed=0, max_steps=20000)
    env.close()
    assert result["laps"] >= 1
    assert not result["collided"]
    assert result["lap_times_sec"][0] < 120.0
