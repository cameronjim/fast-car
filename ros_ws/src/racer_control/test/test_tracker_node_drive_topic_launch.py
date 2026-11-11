"""L3 node test: tracker_node's `drive_topic` (the handoff to racer_policy's policy_node).

By default the tracker publishes /drive_raw, unchanged. With drive_topic:=/tracker/command it
publishes there INSTEAD and nothing reaches /drive_raw: the policy deployment's single-writer
rule (policy_node is then the only publisher of /drive_raw) depends on exactly this. Two
tracker nodes are launched in one description, one in each mode, on different odometry topics so
their outputs are attributable; both follow the same committed fixture raceline.
"""

from __future__ import annotations

import os

# Distinct from every other launch test's domain in this workspace (77-97 are taken).
os.environ.setdefault("ROS_DOMAIN_ID", "99")

import pathlib
import time
import unittest

import launch
import launch_testing
import launch_testing.actions
import pytest
import rclpy
from ackermann_msgs.msg import AckermannDriveStamped
from launch_ros.actions import Node as LaunchNode
from nav_msgs.msg import Odometry
from rclpy.qos import HistoryPolicy, QoSProfile, ReliabilityPolicy

_RACELINE_CSV = str((pathlib.Path(__file__).parent / "fixtures" / "tiny_raceline.csv").resolve())


def _reliable_qos() -> QoSProfile:
    return QoSProfile(
        reliability=ReliabilityPolicy.RELIABLE, history=HistoryPolicy.KEEP_LAST, depth=10
    )


@pytest.mark.launch_test
def generate_test_description():
    handoff = LaunchNode(
        package="racer_control",
        executable="tracker_node",
        name="tracker_handoff",
        parameters=[
            {
                "raceline_path": _RACELINE_CSV,
                "odom_topic": "/odom_handoff",
                "drive_topic": "/tracker/command",
                "control_rate_hz": 20.0,
            }
        ],
    )
    default = LaunchNode(
        package="racer_control",
        executable="tracker_node",
        name="tracker_default",
        parameters=[
            {"raceline_path": _RACELINE_CSV, "odom_topic": "/odom_default", "control_rate_hz": 20.0}
        ],
    )
    return launch.LaunchDescription([handoff, default, launch_testing.actions.ReadyToTest()])


class TestTrackerNodeDriveTopic(unittest.TestCase):
    @classmethod
    def setUpClass(cls):
        rclpy.init()

    @classmethod
    def tearDownClass(cls):
        rclpy.shutdown()

    def setUp(self):
        self.node = rclpy.create_node("test_tracker_drive_topic_client")

    def tearDown(self):
        self.node.destroy_node()

    def _spin_for(self, seconds: float) -> None:
        end = time.monotonic() + seconds
        while time.monotonic() < end:
            rclpy.spin_once(self.node, timeout_sec=0.02)

    def _odom(self) -> Odometry:
        odom = Odometry()
        odom.header.frame_id = "odom"
        odom.child_frame_id = "base_link"
        odom.pose.pose.orientation.w = 1.0
        return odom

    def test_handoff_publishes_on_the_handoff_topic_and_the_default_on_drive_raw(self):
        on_handoff: list = []
        on_drive_raw: list = []
        self.node.create_subscription(
            AckermannDriveStamped, "/tracker/command", on_handoff.append, _reliable_qos()
        )
        self.node.create_subscription(
            AckermannDriveStamped, "/drive_raw", on_drive_raw.append, _reliable_qos()
        )
        handoff_odom = self.node.create_publisher(Odometry, "/odom_handoff", _reliable_qos())
        self._spin_for(0.5)  # discovery

        # Only the handoff tracker gets odometry: if /drive_raw stays silent, the handoff
        # tracker did not publish there.
        for _ in range(30):
            handoff_odom.publish(self._odom())
            self._spin_for(0.03)
        self.assertGreater(
            len(on_handoff), 5, "handoff tracker published nothing on /tracker/command"
        )
        self.assertEqual(len(on_drive_raw), 0, "handoff tracker leaked a command onto /drive_raw")
        self.assertEqual(self.node.count_publishers("/tracker/command"), 1)

        # Now the default tracker gets odometry: its output is on /drive_raw as it always was.
        default_odom = self.node.create_publisher(Odometry, "/odom_default", _reliable_qos())
        self._spin_for(0.5)
        for _ in range(30):
            default_odom.publish(self._odom())
            self._spin_for(0.03)
        self.assertGreater(len(on_drive_raw), 5, "default tracker no longer publishes /drive_raw")
        self.assertEqual(self.node.count_publishers("/drive_raw"), 1)
