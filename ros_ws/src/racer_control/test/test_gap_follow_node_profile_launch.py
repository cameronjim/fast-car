"""L3 test of gap_follow.launch.py's floor-2026-10-06 profile and the speed smoothing parameters.

Launched through the real launch/gap_follow.launch.py with `profile:=floor-2026-10-06`, the
test params file (control rate, watchdog, synthetic-scan yaw) and one explicit override
(`target_range_median_scans:=3`), so this covers the profile and an explicit argument
reaching the RUNNING node, not just the launch description (test_gap_follow_launch_profile.py
covers that).

Checks:
  * the node's own parameter service reports every profile value, with the override winning;
  * the published speed never exceeds the profile's 0.9 m/s;
  * speed smoothing is live: a step DOWN in the target range (the speed rate limiter never
    limits deceleration, so without the low-pass the speed would drop in one cycle) is spread
    over many control cycles.
"""

from __future__ import annotations

import os

# Own domain per launch test file, see test_tracker_node_launch.py.
os.environ.setdefault("ROS_DOMAIN_ID", "87")

import itertools
import math
import signal
import time
import unittest
from pathlib import Path

import launch
import launch_testing
import launch_testing.actions
import launch_testing.asserts
import pytest
import rclpy
from ackermann_msgs.msg import AckermannDriveStamped
from ament_index_python.packages import get_package_share_directory
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from rcl_interfaces.msg import ParameterType
from rcl_interfaces.srv import GetParameters
from rclpy.qos import HistoryPolicy, QoSProfile, ReliabilityPolicy
from sensor_msgs.msg import LaserScan

_PARAMS_FILE = str(Path(__file__).resolve().parent / "fixtures" / "gap_follow_test_params.yaml")
_PROFILE_MAX_SPEED_MPS = 0.9
_PROFILE_MIN_SPEED_MPS = 0.5
_FOV_RAD = 4.7
_NUM_RAYS = 1080

# What the running node must report: the floor-2026-10-06 profile, with the explicit
# target_range_median_scans:=3 launch argument replacing the profile's 5.
_EXPECTED = {
    "min_speed_mps": 0.5,
    "max_speed_mps": 0.9,
    "k_steer": 0.4,
    "free_space_threshold_m": 0.6,
    "steering_gain": 1.6,
    "steering_time_constant_s": 0.15,
    "disparity_threshold_m": 0.3,
    "cone_half_angle_rad": 1.2,
    "forward_preference": 0.0,
    "gap_switch_margin": 0.0,
    "swept_path_clamp": True,
    "swept_path_lookahead_m": 0.6,
    "corner_sector_inner_rad": 0.4,
    "corner_sector_outer_rad": 1.6,
    "corner_min_clearance_m": 0.35,
    "speed_rate_limit_margin_fraction": 0.1,
    "target_deepest_ray": False,
    "speed_time_constant_s": 0.5,
    "target_range_median_scans": 3,
}


def _reliable_qos() -> QoSProfile:
    return QoSProfile(
        reliability=ReliabilityPolicy.RELIABLE, history=HistoryPolicy.KEEP_LAST, depth=10
    )


def _best_effort_qos() -> QoSProfile:
    return QoSProfile(
        reliability=ReliabilityPolicy.BEST_EFFORT, history=HistoryPolicy.KEEP_LAST, depth=10
    )


@pytest.mark.launch_test
def generate_test_description():
    launch_file = Path(get_package_share_directory("racer_control")) / "launch"
    gap_follow = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(str(launch_file / "gap_follow.launch.py")),
        launch_arguments={
            "profile": "floor-2026-10-06",
            "params_file": _PARAMS_FILE,
            "target_range_median_scans": "3",
        }.items(),
    )
    return launch.LaunchDescription([gap_follow, launch_testing.actions.ReadyToTest()])


def _uniform_scan(range_m: float) -> LaserScan:
    """Every ray at range_m: with range_m above the profile's 0.6 m free-space threshold the
    whole cone is one gap and the target is straight ahead; below it no gap is found and the
    node aims straight ahead. Either way the steering is 0 and the target range is range_m
    (clipped at the node's clip_max_range_m, 5 m by default)."""
    msg = LaserScan()
    msg.header.frame_id = "laser"
    msg.angle_min = -_FOV_RAD / 2.0
    msg.angle_max = _FOV_RAD / 2.0
    msg.angle_increment = _FOV_RAD / (_NUM_RAYS - 1)
    msg.range_min = 0.0
    msg.range_max = 30.0
    msg.ranges = [range_m] * _NUM_RAYS
    return msg


def _value(parameter_value):
    if parameter_value.type == ParameterType.PARAMETER_DOUBLE:
        return parameter_value.double_value
    if parameter_value.type == ParameterType.PARAMETER_INTEGER:
        return parameter_value.integer_value
    if parameter_value.type == ParameterType.PARAMETER_BOOL:
        return parameter_value.bool_value
    raise AssertionError(f"unexpected parameter type {parameter_value.type}")


class TestGapFollowNodeFloorProfile(unittest.TestCase):
    @classmethod
    def setUpClass(cls):
        rclpy.init()

    @classmethod
    def tearDownClass(cls):
        rclpy.shutdown()

    def setUp(self):
        self.node = rclpy.create_node("test_gap_follow_node_profile_client")
        self.received: list[AckermannDriveStamped] = []
        self.node.create_subscription(
            AckermannDriveStamped, "/drive_raw", self.received.append, _reliable_qos()
        )
        self.scan_pub = self.node.create_publisher(LaserScan, "/scan", _best_effort_qos())
        self._spin_for(0.5)  # discovery

    def tearDown(self):
        self.node.destroy_node()

    def _spin_for(self, seconds: float) -> None:
        end = time.time() + seconds
        while time.time() < end:
            rclpy.spin_once(self.node, timeout_sec=0.02)

    def _feed(self, scan: LaserScan, seconds: float) -> None:
        # About 10 Hz, like the RPLIDAR C1, so the median window spans real scans.
        end = time.time() + seconds
        next_scan = 0.0
        while time.time() < end:
            if time.time() >= next_scan:
                self.scan_pub.publish(scan)
                next_scan = time.time() + 0.1
            rclpy.spin_once(self.node, timeout_sec=0.01)

    def test_a_running_node_reports_the_profile_and_the_override(self):
        client = self.node.create_client(GetParameters, "/gap_follow_node/get_parameters")
        self.assertTrue(client.wait_for_service(timeout_sec=30.0), "no parameter service")
        request = GetParameters.Request()
        request.names = list(_EXPECTED)
        future = client.call_async(request)
        rclpy.spin_until_future_complete(self.node, future, timeout_sec=10.0)
        self.assertTrue(future.done(), "get_parameters timed out")
        got = {name: _value(v) for name, v in zip(request.names, future.result().values)}
        for name, expected in _EXPECTED.items():
            if isinstance(expected, float):
                self.assertAlmostEqual(got[name], expected, places=9, msg=name)
            else:
                self.assertEqual(got[name], expected, name)

    def test_b_speed_steps_down_smoothly_and_stays_under_the_profile_cap(self):
        # Open room: target range 5 m, raw speed at the 0.9 m/s cap. The ramp up is bounded by
        # the rate limiter (0.1 x vehicle_params max acceleration) plus the low-pass lag.
        self._feed(_uniform_scan(5.0), 5.0)
        self.assertGreater(len(self.received), 0, "no /drive_raw on a nominal scan")
        settled = self.received[-1].drive.speed
        self.assertGreater(settled, 0.8, "speed did not settle near the 0.9 m/s cap")

        # Walls at 0.55 m all round: raw speed drops to the 0.5 m/s floor (0.55 m x 1/s,
        # clamped to [0.5, 0.9]) in one scan. Unfiltered, that is one 0.35 m/s step.
        self.received.clear()
        self._feed(_uniform_scan(0.55), 3.0)
        self.assertGreater(len(self.received), 20)
        speeds = [m.drive.speed for m in self.received]
        for speed in speeds:
            self.assertTrue(math.isfinite(speed))
            self.assertGreaterEqual(speed, 0.0)
            self.assertLessEqual(speed, _PROFILE_MAX_SPEED_MPS + 1e-3)
        drops = [a - b for a, b in itertools.pairwise(speeds)]
        # tau 0.5 s at the test's 20 Hz control rate moves at most 0.05 / 0.55 = 9 percent of
        # the remaining 0.35 m/s per cycle, about 0.032 m/s.
        self.assertLess(max(drops), 0.1, "speed dropped in one step: low-pass not active")
        self.assertLess(speeds[-1], _PROFILE_MIN_SPEED_MPS + 0.05, "speed never came down")


@launch_testing.post_shutdown_test()
class TestGapFollowNodeFloorProfileShutdown(unittest.TestCase):
    def test_clean_exit(self, proc_info):
        launch_testing.asserts.assertExitCodes(proc_info, allowable_exit_codes=[0, -signal.SIGINT])
