"""L3 node tests for wall_follow_node (claude-docs/12-testing.md, GitHub issue 26).

Launched through the real launch/wall_follow.launch.py with
test/fixtures/wall_follow_test_params.yaml, following test_tracker_node_launch.py.

Checklist covered:
  * nominal input, sign convention (LEFT positive, claude-docs/06-vehicle-params.md): with
    the default right-wall rays, a car too far from the wall steers right (negative), a car
    too close steers left (positive). Every command is finite and inside the steering and
    speed bounds.
  * no wall in view (the wall rays read inf): the node keeps publishing with zero steering.
  * silence: once /scan stops, /drive_raw stops.
  * QoS: best_effort /scan mock and reliable /drive_raw subscriber, as in
    test_gap_follow_node_launch.py. /drive is never published.
  * clean shutdown.
"""

from __future__ import annotations

import os

os.environ.setdefault("ROS_DOMAIN_ID", "85")

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

_PARAMS_FILE = str(Path(__file__).resolve().parent / "fixtures" / "wall_follow_test_params.yaml")
_SCAN_TIMEOUT_S = 0.2
_MAX_SPEED_MPS = 1.0
_MAX_STEERING_RAD = 0.4189 + 1e-3
_TARGET_DISTANCE_M = 0.6  # the node's default target_distance_m

_FOV_RAD = 4.7
_NUM_RAYS = 1080
_RANGE_MAX = 30.0


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
    wall_follow = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(str(launch_file / "wall_follow.launch.py")),
        launch_arguments={
            "max_speed_mps": str(_MAX_SPEED_MPS),
            "params_file": _PARAMS_FILE,
        }.items(),
    )
    return launch.LaunchDescription([wall_follow, launch_testing.actions.ReadyToTest()])


def _right_wall_scan(distance_m: float | None) -> LaserScan:
    """Scan of a straight wall parallel to the car on its right, distance_m away (None: no
    wall, every ray reads inf)."""
    msg = LaserScan()
    msg.header.frame_id = "laser"
    msg.angle_min = -_FOV_RAD / 2.0
    msg.angle_max = _FOV_RAD / 2.0
    msg.angle_increment = _FOV_RAD / (_NUM_RAYS - 1)
    msg.range_min = 0.05
    msg.range_max = _RANGE_MAX
    ranges = []
    for i in range(_NUM_RAYS):
        bearing = msg.angle_min + i * msg.angle_increment
        if distance_m is None or bearing >= 0.0:
            ranges.append(float("inf"))
            continue
        r = distance_m / math.sin(-bearing)
        ranges.append(r if r <= _RANGE_MAX else float("inf"))
    msg.ranges = ranges
    return msg


class TestWallFollowNode(unittest.TestCase):
    @classmethod
    def setUpClass(cls):
        rclpy.init()

    @classmethod
    def tearDownClass(cls):
        rclpy.shutdown()

    def setUp(self):
        self.node = rclpy.create_node("test_wall_follow_node_client")
        self.received: list[AckermannDriveStamped] = []
        self.drive_received: list[AckermannDriveStamped] = []
        self.node.create_subscription(
            AckermannDriveStamped, "/drive_raw", self.received.append, _reliable_qos()
        )
        self.node.create_subscription(
            AckermannDriveStamped, "/drive", self.drive_received.append, _best_effort_qos()
        )
        self.scan_pub = self.node.create_publisher(LaserScan, "/scan", _best_effort_qos())
        self._spin_for(0.5)

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

    def test_a_too_far_from_right_wall_steers_right(self):
        self._feed(_right_wall_scan(_TARGET_DISTANCE_M + 0.6), 2.0)
        self.assertGreater(len(self.received), 0, "no /drive_raw on a nominal scan")
        self._assert_in_bounds(self.received)
        self.assertLess(self.received[-1].drive.steering_angle, -0.05)
        self.assertGreater(self.received[-1].drive.speed, 0.0)
        self.assertEqual(len(self.drive_received), 0, "wall_follow_node published /drive")

    def test_b_too_close_to_right_wall_steers_left(self):
        self._feed(_right_wall_scan(_TARGET_DISTANCE_M - 0.3), 2.0)
        self.assertGreater(len(self.received), 0)
        self._assert_in_bounds(self.received)
        self.assertGreater(self.received[-1].drive.steering_angle, 0.05)

    def test_c_no_wall_in_view_drives_straight(self):
        self._feed(_right_wall_scan(None), 1.5)
        self.assertGreater(len(self.received), 0, "stopped publishing with no wall in view")
        self._assert_in_bounds(self.received)
        self.assertAlmostEqual(self.received[-1].drive.steering_angle, 0.0, places=6)

    def test_d_stops_publishing_on_scan_silence(self):
        self._feed(_right_wall_scan(_TARGET_DISTANCE_M), 1.0)
        self.assertGreater(len(self.received), 0)
        self._spin_for(_SCAN_TIMEOUT_S * 2.0)
        self.received.clear()
        self._spin_for(_SCAN_TIMEOUT_S * 5.0)
        self.assertEqual(len(self.received), 0, "kept publishing /drive_raw after /scan silence")


@launch_testing.post_shutdown_test()
class TestWallFollowNodeShutdown(unittest.TestCase):
    def test_clean_exit(self, proc_info):
        launch_testing.asserts.assertExitCodes(proc_info, allowable_exit_codes=[0, -signal.SIGINT])
