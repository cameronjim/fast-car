"""L3 test of tracker_node's odom_topic and pose_frame parameters (2026-11-03 localisation).

tracker_node follows /odom_under_test (odom_topic) with pose_frame:=map, so its pose is the TF
map -> odom composed with the Odometry's odom-frame pose (tracker_node.cpp POSE SOURCE):

  * with no map -> odom on /tf it publishes nothing (no localisation, no driving);
  * with map -> odom = (0, +0.5 m, 0) and the car at odom (0.3, 0) facing +x, the composed pose is
    0.5 m LEFT of the fixture raceline's first straight (y = 0), so pure pursuit steers RIGHT
    (negative); the same Odometry with no composition would steer about straight;
  * when map -> odom stops arriving while /odom keeps coming, it stops publishing again.

The test methods share one tracker_node process and run in name order.
"""

from __future__ import annotations

import os

os.environ.setdefault("ROS_DOMAIN_ID", "93")

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
from geometry_msgs.msg import TransformStamped
from launch_ros.actions import Node as LaunchNode
from nav_msgs.msg import Odometry
from rclpy.qos import HistoryPolicy, QoSProfile, ReliabilityPolicy
from tf2_msgs.msg import TFMessage

_FIXTURE_RACELINE = str(Path(__file__).resolve().parent / "fixtures" / "tiny_raceline.csv")
_POSE_FRAME_TIMEOUT_S = 0.3


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
                "odom_topic": "/odom_under_test",
                "pose_frame": "map",
                "pose_frame_timeout_s": _POSE_FRAME_TIMEOUT_S,
            }
        ],
    )
    return launch.LaunchDescription([tracker_node, launch_testing.actions.ReadyToTest()])


class TestTrackerPoseFrame(unittest.TestCase):
    @classmethod
    def setUpClass(cls):
        rclpy.init()
        cls.node = rclpy.create_node("test_tracker_pose_frame_client")
        cls.received: list[AckermannDriveStamped] = []
        cls.node.create_subscription(
            AckermannDriveStamped, "/drive_raw", cls.received.append, _qos()
        )
        cls.odom_pub = cls.node.create_publisher(Odometry, "/odom_under_test", _qos())
        cls.tf_pub = cls.node.create_publisher(TFMessage, "/tf", _qos())

    @classmethod
    def tearDownClass(cls):
        cls.node.destroy_node()
        rclpy.shutdown()

    def _publish(self, with_tf: bool) -> None:
        stamp = self.node.get_clock().now().to_msg()
        odom = Odometry()
        odom.header.stamp = stamp
        odom.header.frame_id = "odom"
        odom.child_frame_id = "base_link"
        odom.pose.pose.position.x = 0.3
        odom.pose.pose.orientation.w = 1.0
        self.odom_pub.publish(odom)
        if with_tf:
            tf = TransformStamped()
            tf.header.stamp = stamp
            tf.header.frame_id = "map"
            tf.child_frame_id = "odom"
            tf.transform.translation.y = 0.5
            tf.transform.rotation.w = 1.0
            self.tf_pub.publish(TFMessage(transforms=[tf]))

    def _run(self, seconds: float, with_tf: bool) -> None:
        end = time.monotonic() + seconds
        while time.monotonic() < end:
            self._publish(with_tf)
            rclpy.spin_once(self.node, timeout_sec=0.02)

    def test_1_no_map_to_odom_publishes_nothing(self):
        self._run(0.5, False)
        self.received.clear()
        self._run(1.0, False)
        self.assertEqual(len(self.received), 0, "drove with no map -> odom transform")

    def test_2_composed_pose_is_used(self):
        self._run(1.5, True)
        self.assertGreater(len(self.received), 10, "no /drive_raw with map -> odom present")
        steer = self.received[-1].drive.steering_angle
        self.assertTrue(math.isfinite(steer))
        self.assertLess(
            steer,
            -0.05,
            "the car is 0.5 m left of the raceline in map; pure pursuit should steer right",
        )

    def test_3_stale_map_to_odom_stops_publishing(self):
        self._run(0.5, True)
        self._run(_POSE_FRAME_TIMEOUT_S * 2.0, False)  # let it go stale, drain
        self.received.clear()
        self._run(1.0, False)
        self.assertEqual(len(self.received), 0, "kept driving on a stale map -> odom transform")


@launch_testing.post_shutdown_test()
class TestTrackerPoseFrameShutdown(unittest.TestCase):
    def test_clean_exit(self, proc_info):
        launch_testing.asserts.assertExitCodes(proc_info, allowable_exit_codes=[0, -signal.SIGINT])
