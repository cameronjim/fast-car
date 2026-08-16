"""L1 unit tests for racer_train.raceline.PurePursuitController -- the same hand-computed,
sign-convention cases as ros_ws/src/racer_control/test/test_pure_pursuit.cpp (verbatim from
claude-docs/06-vehicle-params.md: steering angle is the road-wheel angle in radians, LEFT
positive; REP-103 frames), mirrored here so the Python port has its own independent L1
coverage in addition to the cross-language divergence test in
test_base_controller_divergence.py.
"""

from __future__ import annotations

import math

import pytest
from racer_train.raceline import (
    PurePursuitConfig,
    PurePursuitController,
    Raceline,
    RacelineLoadError,
    RacelinePoint,
)


def default_config() -> PurePursuitConfig:
    return PurePursuitConfig(
        wheelbase_m=0.3302,  # config/vehicle_params.yaml chassis.wheelbase_m
        lookahead_min_m=0.4,
        lookahead_max_m=1.5,
        lookahead_curvature_ref_1pm=0.4,
        max_steering_angle_rad=0.4189,  # config/vehicle_params.yaml steering.max_angle_rad
    )


def straight_raceline() -> Raceline:
    points = tuple(
        RacelinePoint(
            s_m=float(i),
            x_m=float(i),
            y_m=0.0,
            heading_rad=0.0,
            curvature_1pm=0.0,
            target_speed_mps=4.0,
        )
        for i in range(21)
    )
    return Raceline(points)


# -- Sign-convention cases --------------------------------------------------------------


def test_target_to_the_left_produces_positive_steering():
    points = (
        RacelinePoint(0.0, 0.0, 0.0, 0.0, 0.0, 3.0),
        RacelinePoint(1.0, 1.0, 0.5, 0.0, 0.0, 3.0),
        RacelinePoint(2.0, 2.0, 1.0, 0.0, 0.0, 3.0),
    )
    raceline = Raceline(points)
    controller = PurePursuitController(default_config())

    cmd = controller.compute_command(raceline, 0.0, 0.0, 0.0)
    assert cmd.steering_angle_rad > 0.0


def test_target_to_the_right_produces_negative_steering():
    points = (
        RacelinePoint(0.0, 0.0, 0.0, 0.0, 0.0, 3.0),
        RacelinePoint(1.0, 1.0, -0.5, 0.0, 0.0, 3.0),
        RacelinePoint(2.0, 2.0, -1.0, 0.0, 0.0, 3.0),
    )
    raceline = Raceline(points)
    controller = PurePursuitController(default_config())

    cmd = controller.compute_command(raceline, 0.0, 0.0, 0.0)
    assert cmd.steering_angle_rad < 0.0


def test_target_dead_ahead_produces_zero_steering():
    raceline = straight_raceline()
    controller = PurePursuitController(default_config())

    cmd = controller.compute_command(raceline, 0.0, 0.0, 0.0)
    assert cmd.steering_angle_rad == pytest.approx(0.0, abs=1e-9)


def test_yaw_rotation_is_honored_for_steering_sign():
    half_pi = math.pi / 2.0
    points = (
        RacelinePoint(0.0, 0.0, 0.0, half_pi, 0.0, 3.0),
        RacelinePoint(1.0, 0.0, 1.0, half_pi, 0.0, 3.0),
        RacelinePoint(2.0, 0.0, 2.0, half_pi, 0.0, 3.0),
    )
    raceline = Raceline(points)
    controller = PurePursuitController(default_config())

    cmd = controller.compute_command(raceline, 0.0, 0.0, half_pi)
    assert cmd.steering_angle_rad == pytest.approx(0.0, abs=1e-9)


# -- Curvature-adaptive lookahead ---------------------------------------------------------


def test_lookahead_is_max_on_straights():
    controller = PurePursuitController(default_config())
    assert controller.lookahead_distance_m(0.0) == pytest.approx(1.5)


def test_lookahead_is_min_at_or_beyond_curvature_ref():
    controller = PurePursuitController(default_config())
    assert controller.lookahead_distance_m(0.4) == pytest.approx(0.4)
    assert controller.lookahead_distance_m(2.0) == pytest.approx(0.4)
    assert controller.lookahead_distance_m(-2.0) == pytest.approx(0.4)


def test_lookahead_interpolates_linearly_between_min_and_max():
    controller = PurePursuitController(default_config())
    assert controller.lookahead_distance_m(0.2) == pytest.approx(1.5 - 0.5 * (1.5 - 0.4))


# -- Steering saturation --------------------------------------------------------------


def test_steering_saturates_at_max_angle():
    points = (
        RacelinePoint(0.0, 0.0, 0.0, 0.0, 0.0, 3.0),
        RacelinePoint(1.0, 0.05, 0.5, 0.0, 0.0, 3.0),
    )
    raceline = Raceline(points)
    config = PurePursuitConfig(
        wheelbase_m=default_config().wheelbase_m,
        lookahead_min_m=0.05,
        lookahead_max_m=0.05,
        lookahead_curvature_ref_1pm=default_config().lookahead_curvature_ref_1pm,
        max_steering_angle_rad=default_config().max_steering_angle_rad,
    )
    controller = PurePursuitController(config)

    cmd = controller.compute_command(raceline, 0.0, 0.0, 0.0)
    assert abs(cmd.steering_angle_rad) == pytest.approx(config.max_steering_angle_rad, abs=1e-9)
    assert cmd.steering_angle_rad > 0.0


# -- Speed command ----------------------------------------------------------------------


def test_speed_command_matches_nearest_point_target_speed():
    points = (
        RacelinePoint(0.0, 0.0, 0.0, 0.0, 0.0, 3.0),
        RacelinePoint(1.0, 1.0, 0.0, 0.0, 0.0, 7.5),
        RacelinePoint(2.0, 2.0, 0.0, 0.0, 0.0, 3.0),
    )
    raceline = Raceline(points)
    controller = PurePursuitController(default_config())

    cmd = controller.compute_command(raceline, 1.0, 0.0, 0.0)
    assert cmd.speed_mps == pytest.approx(7.5)


# -- Raceline loading ---------------------------------------------------------------------


def test_raceline_refuses_zero_points():
    with pytest.raises(RacelineLoadError):
        Raceline(())


def test_load_from_csv_round_trips_the_committed_gym_oval_raceline(real_vehicle_params):
    from generate_divergence_fixture import RACELINE_PATH

    raceline = Raceline.load_from_csv(RACELINE_PATH)
    assert len(raceline) > 0
    first = raceline.at(0)
    assert math.isfinite(first.x_m)
    assert math.isfinite(first.target_speed_mps)


def test_load_from_csv_missing_file_raises():
    with pytest.raises(RacelineLoadError):
        Raceline.load_from_csv("/nonexistent/path/raceline.csv")


def test_load_from_csv_wrong_header_raises(tmp_path):
    bad = tmp_path / "bad.csv"
    bad.write_text("a,b,c\n1,2,3\n", encoding="utf-8")
    with pytest.raises(RacelineLoadError):
        Raceline.load_from_csv(bad)


def test_advance_to_lookahead_wraps_around():
    raceline = straight_raceline()
    # From the last index, walking forward wraps back to index 0.
    idx = raceline.advance_to_lookahead(len(raceline) - 1, 20.0, 0.0, 5.0)
    assert idx == 0
