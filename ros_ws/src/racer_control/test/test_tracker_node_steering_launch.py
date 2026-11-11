"""L3 test of tracker_node's steering parameters (2026-11-11 track session): the per-side limits
on the TRUE road-wheel angle (max_steering_left_rad, max_steering_right_rad) and the command
needed per radian of true angle on each side (steering_command_per_rad_left / _right).

The fixture raceline's first straight runs along y = 0. With the car at x = 0.3 facing +x, the
lookahead (1.5 m on a straight) lands on the fixture point (2, 0), so a car 0.2 m off the line has
a pure pursuit curvature of 2 * 0.2 / (1.7^2 + 0.2^2) and a true angle atan(kappa * wheelbase):

  * the car 0.2 m RIGHT of the line steers LEFT by that angle times steering_command_per_rad_left
    (here 0.8), and 0.2 m LEFT steers right by it times steering_command_per_rad_right (0.5);
  * a car 0.5 m right of the line wants about 0.105 rad, more than the left limit (0.08 here), so
    the true angle saturates at that limit before the command scale is applied: 0.08 x 0.8, and
    likewise to the right against the 0.06 limit.

The test methods share one tracker_node process and run in name order.
"""

from __future__ import annotations

import os

os.environ.setdefault("ROS_DOMAIN_ID", "94")

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
from launch_ros.actions import Node as LaunchNode
from nav_msgs.msg import Odometry
from rclpy.qos import HistoryPolicy, QoSProfile, ReliabilityPolicy

_FIXTURE_RACELINE = str(Path(__file__).resolve().parent / "fixtures" / "tiny_raceline.csv")
_WHEELBASE_M = 0.3302  # config/vehicle_params.yaml chassis.wheelbase_m
_LEFT_SCALE = 0.8
_RIGHT_SCALE = 0.5
_LEFT_LIMIT_RAD = 0.08
_RIGHT_LIMIT_RAD = 0.06


def _qos() -> QoSProfile:
    return QoSProfile(
        reliability=ReliabilityPolicy.RELIABLE, history=HistoryPolicy.KEEP_LAST, depth=10
    )


@pytest.mark.launch_test
def generate_test_description():
    tracker_node = LaunchNode(
        package="racer_control",
        executable="tracker_node",
        name="tracker_node",
        parameters=[
            {
                "raceline_path": _FIXTURE_RACELINE,
                "steering_command_per_rad_left": _LEFT_SCALE,
                "steering_command_per_rad_right": _RIGHT_SCALE,
                "max_steering_left_rad": _LEFT_LIMIT_RAD,
                "max_steering_right_rad": _RIGHT_LIMIT_RAD,
            }
        ],
    )
    return launch.LaunchDescription([tracker_node, launch_testing.actions.ReadyToTest()])


def _pursuit_true_angle(offset_m: float) -> float:
    """True angle for a car at (0.3, offset) facing +x chasing the fixture point (2, 0)."""
    lx, ly = 2.0 - 0.3, -offset_m
    curvature = 2.0 * ly / (lx * lx + ly * ly)
    return math.atan(curvature * _WHEELBASE_M)


class TestTrackerSteering(unittest.TestCase):
    @classmethod
    def setUpClass(cls):
        rclpy.init()
        cls.node = rclpy.create_node("test_tracker_steering_client")
        cls.received: list[AckermannDriveStamped] = []
        cls.node.create_subscription(
            AckermannDriveStamped, "/drive_raw", cls.received.append, _qos()
        )
        cls.odom_pub = cls.node.create_publisher(Odometry, "/odom", _qos())

    @classmethod
    def tearDownClass(cls):
        cls.node.destroy_node()
        rclpy.shutdown()

    def _steering_at(self, y_m: float) -> float:
        self.received.clear()
        end = time.monotonic() + 1.0
        while time.monotonic() < end:
            odom = Odometry()
            odom.header.stamp = self.node.get_clock().now().to_msg()
            odom.header.frame_id = "map"
            odom.pose.pose.position.x = 0.3
            odom.pose.pose.position.y = y_m
            odom.pose.pose.orientation.w = 1.0
            self.odom_pub.publish(odom)
            rclpy.spin_once(self.node, timeout_sec=0.02)
        self.assertGreater(len(self.received), 5, "no /drive_raw")
        return self.received[-1].drive.steering_angle

    def test_1_left_command_is_the_true_angle_times_the_left_scale(self):
        steer = self._steering_at(-0.2)  # right of the line: steer left
        self.assertAlmostEqual(steer, _pursuit_true_angle(-0.2) * _LEFT_SCALE, delta=5e-4)
        self.assertGreater(steer, 0.0)

    def test_2_right_command_is_the_true_angle_times_the_right_scale(self):
        steer = self._steering_at(0.2)  # left of the line: steer right
        self.assertAlmostEqual(steer, _pursuit_true_angle(0.2) * _RIGHT_SCALE, delta=5e-4)
        self.assertLess(steer, 0.0)

    def test_3_true_angle_saturates_at_the_per_side_limit_before_the_scale(self):
        self.assertGreater(abs(_pursuit_true_angle(-0.5)), _LEFT_LIMIT_RAD)  # really past the limit
        steer = self._steering_at(-0.5)
        self.assertAlmostEqual(steer, _LEFT_LIMIT_RAD * _LEFT_SCALE, delta=5e-4)

    def test_4_right_saturation_uses_the_right_limit(self):
        self.assertGreater(abs(_pursuit_true_angle(0.5)), _RIGHT_LIMIT_RAD)
        steer = self._steering_at(0.5)
        self.assertAlmostEqual(steer, -_RIGHT_LIMIT_RAD * _RIGHT_SCALE, delta=5e-4)


@launch_testing.post_shutdown_test()
class TestTrackerSteeringShutdown(unittest.TestCase):
    def test_clean_exit(self, proc_info):
        launch_testing.asserts.assertExitCodes(proc_info, allowable_exit_codes=[0, -signal.SIGINT])
