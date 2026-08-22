"""L1 unit tests for racer_gym_bridge.conversions (claude-docs/12-testing.md).

Pure logic only -- no ROS, no f1tenth_gym. Runs under plain `uv run pytest`
(see the package's pyproject.toml) with no ROS or gym installation, and
also runs as part of `colcon test` in CI's ros-dev-based job.
"""

from __future__ import annotations

import math

import numpy as np
import pytest
from racer_gym_bridge.conversions import (
    build_odom_fields,
    build_scan_fields,
    drive_cmd_to_action,
    quaternion_to_yaw,
    yaw_to_quaternion,
)


class TestDriveCmdToAction:
    def test_shape_and_dtype(self):
        action = drive_cmd_to_action(0.1, 2.0)
        assert action.shape == (1, 2)
        assert action.dtype == np.float64

    def test_column_order_is_steer_then_speed(self):
        # See the docstring in conversions.py: f1tenth_gym's per-agent
        # action row is [steering_angle_rad, speed_mps], verified
        # empirically against the pinned gym commit.
        action = drive_cmd_to_action(steering_angle=0.35, speed=2.5)
        assert action[0, 0] == pytest.approx(0.35)
        assert action[0, 1] == pytest.approx(2.5)

    def test_zero_command(self):
        action = drive_cmd_to_action(0.0, 0.0)
        assert action[0, 0] == 0.0
        assert action[0, 1] == 0.0

    def test_negative_values_pass_through(self):
        # Left-positive steering convention (claude-docs/04-architecture.md)
        # means a right turn is a negative steering angle; reverse speed is
        # negative. The pure conversion does not clamp or reinterpret sign
        # -- that is the safety/envelope layer's job, not the bridge's.
        action = drive_cmd_to_action(-0.4189, -1.0)
        assert action[0, 0] == pytest.approx(-0.4189)
        assert action[0, 1] == pytest.approx(-1.0)


class TestYawQuaternionRoundTrip:
    @pytest.mark.parametrize(
        "yaw",
        [0.0, 0.1, math.pi / 2, math.pi, -math.pi / 2, -math.pi, 3.0, -3.0],
    )
    def test_round_trip(self, yaw):
        x, y, z, w = yaw_to_quaternion(yaw)
        assert x == 0.0
        assert y == 0.0
        recovered = quaternion_to_yaw(x, y, z, w)
        # atan2 wraps to (-pi, pi]; compare via the wrapped difference.
        diff = (recovered - yaw + math.pi) % (2 * math.pi) - math.pi
        assert diff == pytest.approx(0.0, abs=1e-9)

    def test_zero_yaw_is_identity_quaternion(self):
        assert yaw_to_quaternion(0.0) == (0.0, 0.0, 0.0, 1.0)

    def test_half_turn(self):
        x, y, z, w = yaw_to_quaternion(math.pi)
        assert x == pytest.approx(0.0)
        assert y == pytest.approx(0.0)
        assert z == pytest.approx(1.0)
        assert w == pytest.approx(0.0, abs=1e-9)


class TestBuildScanFields:
    def test_known_geometry(self):
        # f1tenth_gym pinned-commit defaults: 1080 beams, 4.7 rad fov.
        ranges = [1.0] * 1080
        fields = build_scan_fields(ranges, fov_rad=4.7, range_min=0.0, range_max=30.0)
        assert fields.angle_min == pytest.approx(-2.35)
        assert fields.angle_max == pytest.approx(2.35)
        assert fields.angle_increment == pytest.approx(4.7 / 1079)
        assert fields.range_min == 0.0
        assert fields.range_max == 30.0
        assert len(fields.ranges) == 1080
        assert fields.ranges[0] == 1.0

    def test_ranges_cast_to_float(self):
        ranges = np.array([1.5, 2.5, 3.5], dtype=np.float32)
        fields = build_scan_fields(ranges, fov_rad=1.0, range_min=0.1, range_max=10.0)
        assert fields.ranges == [pytest.approx(1.5), pytest.approx(2.5), pytest.approx(3.5)]
        assert all(isinstance(r, float) for r in fields.ranges)

    def test_too_few_beams_raises(self):
        with pytest.raises(ValueError):
            build_scan_fields([1.0], fov_rad=4.7, range_min=0.0, range_max=30.0)

    def test_empty_beams_raises(self):
        with pytest.raises(ValueError):
            build_scan_fields([], fov_rad=4.7, range_min=0.0, range_max=30.0)


class TestBuildOdomFields:
    def test_values_pass_through(self):
        fields = build_odom_fields(
            pose_x=1.0, pose_y=2.0, yaw_rad=0.0, vx=3.0, vy=0.5, yaw_rate=0.2
        )
        assert fields.position == (1.0, 2.0, 0.0)
        assert fields.orientation == (0.0, 0.0, 0.0, 1.0)
        assert fields.linear == (3.0, 0.5, 0.0)
        assert fields.angular == (0.0, 0.0, 0.2)

    def test_orientation_matches_yaw_to_quaternion(self):
        fields = build_odom_fields(
            pose_x=0.0, pose_y=0.0, yaw_rad=1.2, vx=0.0, vy=0.0, yaw_rate=0.0
        )
        assert fields.orientation == yaw_to_quaternion(1.2)
