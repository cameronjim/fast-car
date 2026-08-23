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
    build_occupancy_grid_fields,
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


class TestBuildOccupancyGridFields:
    def test_all_free_map_is_a_synthetic_track_like_uniform_255(self):
        # f1tenth_gym's Track.from_refline (this bridge's default synthetic track, and its
        # closed-loop raceline tracks) builds a uniform all-free occupancy_map of 255.0.
        occupancy_map = np.full((4, 6), 255.0, dtype=np.float32)
        fields = build_occupancy_grid_fields(
            occupancy_map=occupancy_map,
            resolution=0.05,
            origin=(1.0, -2.0, 0.0),
            negate=False,
            occupied_thresh=0.65,
            free_thresh=0.196,
        )
        assert fields.height == 4
        assert fields.width == 6
        assert fields.resolution == pytest.approx(0.05)
        assert fields.origin_position == (1.0, -2.0, 0.0)
        assert fields.origin_orientation == yaw_to_quaternion(0.0)
        assert len(fields.data) == 4 * 6
        assert all(v == 0 for v in fields.data), "an all-255 (free) map must decode to all-0"

    def test_black_pixels_are_occupied_white_pixels_are_free(self):
        # Same binarization Track.from_track_name itself applies (pixel > 128 -> 255 free,
        # <= 128 -> 0 occupied) -- 0 must map to ROS "occupied" (100), 255 to "free" (0).
        occupancy_map = np.array([[0.0, 255.0], [255.0, 0.0]], dtype=np.float32)
        fields = build_occupancy_grid_fields(
            occupancy_map=occupancy_map,
            resolution=0.05,
            origin=(0.0, 0.0, 0.0),
            negate=False,
            occupied_thresh=0.65,
            free_thresh=0.196,
        )
        assert fields.data == [100, 0, 0, 100]

    def test_negate_flips_the_interpretation(self):
        occupancy_map = np.array([[0.0, 255.0]], dtype=np.float32)
        fields = build_occupancy_grid_fields(
            occupancy_map=occupancy_map,
            resolution=0.05,
            origin=(0.0, 0.0, 0.0),
            negate=True,
            occupied_thresh=0.65,
            free_thresh=0.196,
        )
        # negate=True: raw pixel/255 IS the occupied probability, so 0 -> free, 255 -> occupied.
        assert fields.data == [0, 100]

    def test_mid_gray_between_thresholds_is_unknown(self):
        # 128/255 ~= 0.502 occupied-probability (negate=False -> (255-128)/255 = 0.498),
        # strictly between free_thresh=0.196 and occupied_thresh=0.65 -> unknown (-1).
        occupancy_map = np.array([[128.0]], dtype=np.float32)
        fields = build_occupancy_grid_fields(
            occupancy_map=occupancy_map,
            resolution=0.05,
            origin=(0.0, 0.0, 0.0),
            negate=False,
            occupied_thresh=0.65,
            free_thresh=0.196,
        )
        assert fields.data == [-1]

    def test_data_is_row_major_matching_occupancy_grid_convention(self):
        # Row 0 (y=origin) must come first, each row left-to-right (x increasing) -- this is
        # nav_msgs/OccupancyGrid's required data[row * width + col] layout.
        occupancy_map = np.array([[0.0, 255.0, 255.0], [255.0, 255.0, 0.0]], dtype=np.float32)
        fields = build_occupancy_grid_fields(
            occupancy_map=occupancy_map,
            resolution=0.05,
            origin=(0.0, 0.0, 0.0),
            negate=False,
            occupied_thresh=0.65,
            free_thresh=0.196,
        )
        assert fields.width == 3
        assert fields.height == 2
        assert fields.data == [100, 0, 0, 0, 0, 100]

    def test_origin_orientation_matches_yaw_to_quaternion(self):
        occupancy_map = np.full((2, 2), 255.0, dtype=np.float32)
        fields = build_occupancy_grid_fields(
            occupancy_map=occupancy_map,
            resolution=0.05,
            origin=(0.0, 0.0, 1.2),
            negate=False,
            occupied_thresh=0.65,
            free_thresh=0.196,
        )
        assert fields.origin_orientation == yaw_to_quaternion(1.2)


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
