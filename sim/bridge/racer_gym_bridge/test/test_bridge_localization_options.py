"""L3 test of bridge_node's localisation options (2026-11-03): with them set,

  * no ground-truth map -> base_link on /tf (publish_ground_truth_tf false);
  * the static base_link -> laser sits at lidar_offset_x_m - base_link_offset_x_m;
  * /scan comes at about scan_rate_hz with range_max = scan_range_max_m and no finite return
    beyond it;
  * /odom/wheel comes at about wheel_odom_rate_hz and reads wheel_speed_scale times the true
    longitudinal speed.

The defaults (all of this off) are covered by test_bridge_node.py, unchanged. Same importorskip
guard as that file: this only runs where rclpy, launch_testing and f1tenth_gym exist.
"""

from __future__ import annotations

import math
import os
import signal
import time
import unittest

import pytest

pytest.importorskip("rclpy")
pytest.importorskip("launch_testing")
pytest.importorskip("f1tenth_gym")
pytest.importorskip("tf2_msgs")

os.environ.setdefault("ROS_DOMAIN_ID", "94")

import launch
import launch_testing
import launch_testing.actions
import launch_testing.asserts
import rclpy
from ackermann_msgs.msg import AckermannDriveStamped
from launch_ros.actions import Node as LaunchNode
from nav_msgs.msg import Odometry
from rclpy.qos import DurabilityPolicy, HistoryPolicy, QoSProfile, ReliabilityPolicy
from sensor_msgs.msg import LaserScan
from tf2_msgs.msg import TFMessage

_LIDAR_OFFSET_X_M = 0.11
_BASE_LINK_OFFSET_X_M = -0.17
_SCAN_RATE_HZ = 10.0
_WHEEL_RATE_HZ = 50.0
_RANGE_MAX_M = 2.0
_SPEED_SCALE = 1.25


@pytest.mark.launch_test
def generate_test_description():
    bridge_node = LaunchNode(
        package="racer_gym_bridge",
        executable="bridge_node",
        name="bridge_node",
        parameters=[
            {
                "seed": 5,
                # Walls both near (the lane, 0.55 m) and far (the end of the straight, about 2.8 m
                # ahead of the seed-5 start), so the range cut has returns on both sides of it.
                "synthetic_track": "lane_round",
                "publish_ground_truth_tf": False,
                "base_link_offset_x_m": _BASE_LINK_OFFSET_X_M,
                "lidar_offset_x_m": _LIDAR_OFFSET_X_M,
                "scan_rate_hz": _SCAN_RATE_HZ,
                "scan_range_max_m": _RANGE_MAX_M,
                "scan_noise_std_m": 0.01,
                "publish_wheel_odom": True,
                "wheel_odom_rate_hz": _WHEEL_RATE_HZ,
                "wheel_speed_scale": _SPEED_SCALE,
            }
        ],
    )
    return launch.LaunchDescription([bridge_node, launch_testing.actions.ReadyToTest()])


def _qos(reliable: bool = True, latched: bool = False) -> QoSProfile:
    return QoSProfile(
        reliability=ReliabilityPolicy.RELIABLE if reliable else ReliabilityPolicy.BEST_EFFORT,
        durability=DurabilityPolicy.TRANSIENT_LOCAL if latched else DurabilityPolicy.VOLATILE,
        history=HistoryPolicy.KEEP_LAST,
        depth=50,
    )


class TestBridgeLocalizationOptions(unittest.TestCase):
    @classmethod
    def setUpClass(cls):
        rclpy.init()
        cls.node = rclpy.create_node("test_bridge_localization_options")
        cls.scans: list[LaserScan] = []
        cls.wheel: list[tuple[float, float]] = []
        cls.truth: list[Odometry] = []
        cls.tf: list[TFMessage] = []
        cls.tf_static: list[TFMessage] = []
        n = cls.node
        n.create_subscription(LaserScan, "/scan", cls.scans.append, _qos(reliable=False))
        n.create_subscription(
            Odometry,
            "/odom/wheel",
            lambda m: cls.wheel.append((time.monotonic(), m.twist.twist.linear.x)),
            _qos(),
        )
        n.create_subscription(Odometry, "/sim/ground_truth_odom", cls.truth.append, _qos())
        n.create_subscription(TFMessage, "/tf", cls.tf.append, _qos())
        n.create_subscription(TFMessage, "/tf_static", cls.tf_static.append, _qos(latched=True))
        cls.drive_pub = n.create_publisher(AckermannDriveStamped, "/drive", _qos())
        # Warm-up: the gym JITs on its first step (see test_bridge_node.py).
        deadline = time.monotonic() + 60.0
        while not cls.scans and time.monotonic() < deadline:
            rclpy.spin_once(n, timeout_sec=0.1)

    @classmethod
    def tearDownClass(cls):
        cls.node.destroy_node()
        rclpy.shutdown()

    def _spin(self, seconds: float, speed: float | None = None) -> None:
        end = time.monotonic() + seconds
        while time.monotonic() < end:
            if speed is not None:
                msg = AckermannDriveStamped()
                msg.drive.speed = speed
                self.drive_pub.publish(msg)
            rclpy.spin_once(self.node, timeout_sec=0.02)

    def test_no_ground_truth_tf_and_moved_laser_mount(self):
        self.tf.clear()
        self._spin(2.0)
        pairs = {(t.header.frame_id, t.child_frame_id) for m in self.tf for t in m.transforms}
        self.assertNotIn(("map", "base_link"), pairs)
        laser = [t for m in self.tf_static for t in m.transforms if t.child_frame_id == "laser"]
        self.assertEqual(len(laser), 1)
        self.assertAlmostEqual(
            laser[0].transform.translation.x, _LIDAR_OFFSET_X_M - _BASE_LINK_OFFSET_X_M, places=6
        )

    def test_scan_rate_and_range_max(self):
        self.scans.clear()
        self._spin(3.0)
        rate = len(self.scans) / 3.0
        self.assertGreater(rate, 0.6 * _SCAN_RATE_HZ)
        self.assertLess(rate, 1.4 * _SCAN_RATE_HZ)
        scan = self.scans[-1]
        self.assertAlmostEqual(scan.range_max, _RANGE_MAX_M, places=5)
        finite = [r for r in scan.ranges if math.isfinite(r)]
        self.assertTrue(finite, "every return was cut")
        self.assertLessEqual(max(finite), _RANGE_MAX_M)
        self.assertLess(len(finite), len(scan.ranges), "nothing beyond range_max was cut")

    def test_wheel_rate_and_scale(self):
        # About 1.5 m of driving in all: the lane's first corner is about 2 m ahead of the start.
        self._spin(0.8, speed=1.0)
        self.wheel.clear()
        self.truth.clear()
        self._spin(0.8, speed=1.0)
        rate = len(self.wheel) / 0.8
        self.assertGreater(rate, 0.6 * _WHEEL_RATE_HZ)
        self.assertLess(rate, 1.4 * _WHEEL_RATE_HZ)
        measured = self.wheel[-1][1]
        true = self.truth[-1].twist.twist.linear.x
        self.assertGreater(true, 0.5, "the car did not get going")
        self.assertAlmostEqual(measured / true, _SPEED_SCALE, delta=0.05)


@launch_testing.post_shutdown_test()
class TestBridgeLocalizationOptionsShutdown(unittest.TestCase):
    def test_clean_exit(self, proc_info):
        launch_testing.asserts.assertExitCodes(proc_info, allowable_exit_codes=[0, -signal.SIGINT])
