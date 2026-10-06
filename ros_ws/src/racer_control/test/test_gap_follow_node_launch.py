"""L3 node tests for gap_follow_node (claude-docs/12-testing.md, GitHub issue 26).

Launched through the real launch/gap_follow.launch.py (max_speed_mps, forward_preference and
gap_switch_margin launch arguments plus the test/fixtures/gap_follow_test_params.yaml params
file), so the launch file is exercised too, following test_tracker_node_launch.py's
structure. Out-of-range values for the two gap selection parameters are covered by
test_gap_follow_node_params_launch.py.

Checklist covered:
  * nominal input: a synthetic scan in racer_gym_bridge's ray ordering (1080 rays over
    4.7 rad, angle_min = -fov/2, counter-clockwise) with a gap on the LEFT produces a positive
    (left) steering command; a gap on the RIGHT produces a negative one. Every command stays
    inside +/- steering.max_angle_rad and [0, max_speed_mps], and is finite.
  * silence: once /scan stops, /drive_raw stops (the watchdog path); a scan with no valid
    returns counts as silence.
  * QoS: the mock /scan publisher is best_effort, so a response proves the node's /scan
    subscription is best_effort (a reliable subscription rejects a best_effort publisher);
    this test's /drive_raw subscriber is reliable, so receiving anything proves /drive_raw
    is published reliable.
  * /drive is never published.
  * clean shutdown.
"""

from __future__ import annotations

import os

# Pinned before rclpy is imported, see test_tracker_node_launch.py for why each launch test
# file gets its own domain.
os.environ.setdefault("ROS_DOMAIN_ID", "84")

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
from rclpy.qos import HistoryPolicy, QoSProfile, ReliabilityPolicy
from sensor_msgs.msg import LaserScan

_PARAMS_FILE = str(Path(__file__).resolve().parent / "fixtures" / "gap_follow_test_params.yaml")
_SCAN_TIMEOUT_S = 0.2  # matches the params file
_MAX_SPEED_MPS = 1.0
# vehicle_params steering.max_angle_rad (0.4189) is read by the node from the generated
# binding; this test only needs an upper bound to check against, with float32 slack.
_MAX_STEERING_RAD = 0.4189 + 1e-3

_FOV_RAD = 4.7
_NUM_RAYS = 1080


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
            "max_speed_mps": str(_MAX_SPEED_MPS),
            "params_file": _PARAMS_FILE,
            # Integer-looking strings on purpose: the launch file must hand them to the node
            # as floats (a double parameter rejects an integer). 0 keeps today's behaviour,
            # which the steering assertions below rely on.
            "forward_preference": "0",
            "gap_switch_margin": "0",
        }.items(),
    )
    return launch.LaunchDescription([gap_follow, launch_testing.actions.ReadyToTest()])


def _make_scan(gap_lo_rad: float | None, gap_hi_rad: float | None, fill: float = 1.0) -> LaserScan:
    msg = LaserScan()
    msg.header.frame_id = "laser"
    msg.angle_min = -_FOV_RAD / 2.0
    msg.angle_max = _FOV_RAD / 2.0
    msg.angle_increment = _FOV_RAD / (_NUM_RAYS - 1)
    msg.range_min = 0.0
    msg.range_max = 30.0
    ranges = []
    for i in range(_NUM_RAYS):
        bearing = msg.angle_min + i * msg.angle_increment
        free = gap_lo_rad is not None and gap_lo_rad <= bearing <= gap_hi_rad
        ranges.append(5.0 if free else fill)
    msg.ranges = ranges
    return msg


class TestGapFollowNode(unittest.TestCase):
    @classmethod
    def setUpClass(cls):
        rclpy.init()

    @classmethod
    def tearDownClass(cls):
        rclpy.shutdown()

    def setUp(self):
        self.node = rclpy.create_node("test_gap_follow_node_client")
        self.received: list[AckermannDriveStamped] = []
        self.drive_received: list[AckermannDriveStamped] = []
        self.node.create_subscription(
            AckermannDriveStamped, "/drive_raw", self.received.append, _reliable_qos()
        )
        self.node.create_subscription(
            AckermannDriveStamped, "/drive", self.drive_received.append, _best_effort_qos()
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
        end = time.time() + seconds
        while time.time() < end:
            self.scan_pub.publish(scan)
            rclpy.spin_once(self.node, timeout_sec=0.05)

    def _assert_in_bounds(self, msgs: list[AckermannDriveStamped]) -> None:
        for msg in msgs:
            self.assertTrue(math.isfinite(msg.drive.steering_angle))
            self.assertTrue(math.isfinite(msg.drive.speed))
            self.assertLessEqual(abs(msg.drive.steering_angle), _MAX_STEERING_RAD)
            self.assertGreaterEqual(msg.drive.speed, 0.0)
            self.assertLessEqual(msg.drive.speed, _MAX_SPEED_MPS + 1e-3)

    def test_a_left_gap_steers_left_within_bounds(self):
        """The gap is 50 degrees wide: at a 1 m edge the disparity bubble for this car
        (chassis.width_m / 2 + 0.1 m margin) eats about 14 degrees from each side, so a
        narrower gap is correctly rejected as too tight and the node aims straight ahead."""
        self._feed(_make_scan(math.radians(20.0), math.radians(70.0)), 2.0)
        self.assertGreater(len(self.received), 0, "no /drive_raw on a nominal scan")
        self._assert_in_bounds(self.received)
        self.assertGreater(
            self.received[-1].drive.steering_angle,
            0.05,
            "a gap on the left (high ray indices, positive bearing) must steer left (positive)",
        )
        self.assertGreater(self.received[-1].drive.speed, 0.0)
        self.assertEqual(len(self.drive_received), 0, "gap_follow_node published /drive")

    def test_b_right_gap_steers_right_within_bounds(self):
        self._feed(_make_scan(math.radians(-70.0), math.radians(-20.0)), 2.0)
        self.assertGreater(len(self.received), 0)
        self._assert_in_bounds(self.received)
        self.assertLess(self.received[-1].drive.steering_angle, -0.05)

    def test_c_steering_is_smoothed(self):
        """A left-to-right gap flip must not jump the steering in one cycle (low-pass)."""
        self._feed(_make_scan(math.radians(20.0), math.radians(70.0)), 1.5)
        self.received.clear()
        self._feed(_make_scan(math.radians(-70.0), math.radians(-20.0)), 1.5)
        self.assertGreater(len(self.received), 3)
        steps = [
            abs(b.drive.steering_angle - a.drive.steering_angle)
            for a, b in zip(self.received, self.received[1:])
        ]
        # Full lock to full lock is 2 * 0.4189 rad. With tau = 0.1 s at 20 Hz each cycle moves
        # at most dt / (tau + dt) = 1/3 of the remaining distance.
        self.assertLess(max(steps), 0.5 * 2 * 0.4189)

    def test_d_stops_publishing_on_scan_silence(self):
        self._feed(_make_scan(math.radians(-25.0), math.radians(25.0)), 1.0)
        self.assertGreater(len(self.received), 0, "no /drive_raw before the silence test")
        self._spin_for(_SCAN_TIMEOUT_S * 2.0)
        self.received.clear()
        self._spin_for(_SCAN_TIMEOUT_S * 5.0)
        self.assertEqual(len(self.received), 0, "kept publishing /drive_raw after /scan silence")

    def test_e_scan_with_no_valid_returns_counts_as_silence(self):
        self._feed(_make_scan(math.radians(-25.0), math.radians(25.0)), 1.0)
        self.assertGreater(len(self.received), 0)
        garbage = _make_scan(None, None, fill=float("nan"))
        self._feed(garbage, _SCAN_TIMEOUT_S * 2.0)
        self.received.clear()
        self._feed(garbage, _SCAN_TIMEOUT_S * 5.0)
        self.assertEqual(len(self.received), 0, "drove on a scan with no valid returns")


@launch_testing.post_shutdown_test()
class TestGapFollowNodeShutdown(unittest.TestCase):
    def test_clean_exit(self, proc_info):
        launch_testing.asserts.assertExitCodes(proc_info, allowable_exit_codes=[0, -signal.SIGINT])
