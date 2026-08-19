"""L1 tests for racer_tools.raceline_path_builder (claude-docs/12-testing.md), pure/ROS-free
-- no geometry_msgs/nav_msgs import anywhere in this module or the one under test."""

from __future__ import annotations

import math

import pytest
from racer_tools.raceline_loader import RacelinePoint
from racer_tools.raceline_path_builder import (
    PathPointFields,
    raceline_path_fields,
    yaw_to_quaternion_zw,
)


@pytest.mark.parametrize(
    "yaw_rad,expected_qz,expected_qw",
    [
        (0.0, 0.0, 1.0),
        (math.pi, 1.0, 0.0),
        (-math.pi / 2.0, -math.sin(math.pi / 4.0), math.cos(math.pi / 4.0)),
    ],
)
def test_yaw_to_quaternion_zw_known_angles(yaw_rad, expected_qz, expected_qw):
    qz, qw = yaw_to_quaternion_zw(yaw_rad)
    assert qz == pytest.approx(expected_qz, abs=1e-9)
    assert qw == pytest.approx(expected_qw, abs=1e-9)


def _point(s, x, y, heading, speed=3.0):
    return RacelinePoint(
        s_m=s, x_m=x, y_m=y, heading_rad=heading, curvature_1pm=0.0, target_speed_mps=speed
    )


def test_raceline_path_fields_returns_one_entry_per_point_plus_closing_point():
    points = [_point(0.0, 0.0, 0.0, 0.0), _point(1.0, 1.0, 0.0, 0.0), _point(2.0, 1.0, 1.0, 0.0)]
    fields = raceline_path_fields(points)
    assert len(fields) == len(points) + 1


def test_raceline_path_fields_closing_point_repeats_the_first_point():
    points = [_point(0.0, 5.0, -2.0, 0.3), _point(1.0, 6.0, -2.0, 0.3)]
    fields = raceline_path_fields(points)
    assert fields[-1].x_m == pytest.approx(5.0)
    assert fields[-1].y_m == pytest.approx(-2.0)
    assert fields[-1] == fields[0]


def test_raceline_path_fields_preserves_xy_and_encodes_heading():
    points = [_point(0.0, 1.5, -3.5, math.pi)]
    fields = raceline_path_fields(points)
    qz, qw = yaw_to_quaternion_zw(math.pi)
    assert fields[0] == PathPointFields(1.5, -3.5, qz, qw)


def test_raceline_path_fields_empty_input_returns_empty_output():
    assert raceline_path_fields([]) == []
