"""L3 node tests for tracker_node (claude-docs/12-testing.md).

Checklist covered: correct behavior on nominal input; correct behavior on silence (the
watchdog path); correct QoS (a genuinely `reliable` subscription rejects a `best_effort`
publisher); clean shutdown. Uses the `test/fixtures/tiny_raceline.csv` fixture already used
by test_raceline.cpp's C++ gtest -- same file, two languages independently parsing it.
"""

from __future__ import annotations

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
_TEST_ODOM_TIMEOUT_S = 0.2


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
    tracker_node = LaunchNode(
        package="racer_control",
        executable="tracker_node",
        name="tracker_node",
        parameters=[
            {
                "raceline_path": _FIXTURE_RACELINE,
                "control_rate_hz": 20.0,
                "odom_timeout_s": _TEST_ODOM_TIMEOUT_S,
            }
        ],
    )
    return launch.LaunchDescription(
        [
            tracker_node,
            launch_testing.actions.ReadyToTest(),
        ]
    )


def _make_odom(x: float, y: float, yaw: float = 0.0, speed: float = 0.0) -> Odometry:
    msg = Odometry()
    msg.header.frame_id = "map"
    msg.child_frame_id = "base_link"
    msg.pose.pose.position.x = x
    msg.pose.pose.position.y = y
    msg.pose.pose.orientation.z = math.sin(yaw / 2.0)
    msg.pose.pose.orientation.w = math.cos(yaw / 2.0)
    msg.twist.twist.linear.x = speed
    return msg


class TestTrackerNode(unittest.TestCase):
    @classmethod
    def setUpClass(cls):
        rclpy.init()

    @classmethod
    def tearDownClass(cls):
        rclpy.shutdown()

    def setUp(self):
        self.node = rclpy.create_node("test_tracker_node_client")

    def tearDown(self):
        self.node.destroy_node()

    def _spin_for(self, seconds: float) -> None:
        end = time.time() + seconds
        while time.time() < end:
            rclpy.spin_once(self.node, timeout_sec=0.05)

    def test_drive_raw_is_plausible_on_nominal_odom(self):
        received = []
        self.node.create_subscription(
            AckermannDriveStamped, "/drive_raw", received.append, _reliable_qos()
        )
        odom_pub = self.node.create_publisher(Odometry, "/odom", _reliable_qos())

        self._spin_for(0.5)  # let discovery settle
        odom_msg = _make_odom(x=0.3, y=0.0)
        end = time.time() + 3.0
        while time.time() < end:
            odom_pub.publish(odom_msg)
            rclpy.spin_once(self.node, timeout_sec=0.05)
            if len(received) >= 3:
                break

        self.assertGreater(len(received), 0, "tracker_node published no /drive_raw on nominal odom")
        msg = received[-1]
        self.assertTrue(math.isfinite(msg.drive.steering_angle))
        self.assertTrue(math.isfinite(msg.drive.speed))
        self.assertLessEqual(abs(msg.drive.steering_angle), 0.4189 + 1e-3)
        # The fixture raceline's nearest point to (0.3, 0) is (0, 0), speed 3.0 m/s
        # (see tiny_raceline.csv).
        self.assertAlmostEqual(msg.drive.speed, 3.0, places=3)

    def test_drive_raw_stops_when_odom_goes_silent(self):
        received = []
        self.node.create_subscription(
            AckermannDriveStamped, "/drive_raw", received.append, _reliable_qos()
        )
        odom_pub = self.node.create_publisher(Odometry, "/odom", _reliable_qos())

        self._spin_for(0.5)
        odom_msg = _make_odom(x=0.5, y=0.0)
        end = time.time() + 2.0
        while time.time() < end:
            odom_pub.publish(odom_msg)
            rclpy.spin_once(self.node, timeout_sec=0.05)
        self.assertGreater(len(received), 0, "no /drive_raw published before silence test began")

        # Stop publishing /odom. First drain any /drive_raw already in flight from odom
        # published just before this point (avoids mistaking a late-arriving, legitimately
        # pre-silence message for a watchdog failure), THEN clear and check silence over a
        # fresh window well past the configured watchdog timeout.
        self._spin_for(_TEST_ODOM_TIMEOUT_S * 2.0)
        received.clear()
        self._spin_for(_TEST_ODOM_TIMEOUT_S * 5.0)
        self.assertEqual(
            len(received),
            0,
            "tracker_node kept publishing /drive_raw after /odom went silent past the watchdog timeout",
        )

    def test_odom_subscription_is_reliable_not_best_effort(self):
        """A best_effort /odom publisher must not reach tracker_node's reliable
        subscription -- proving it is genuinely `reliable`, not left at a default profile
        that happens to interoperate anyway (claude-docs/12-testing.md L3 QoS check)."""
        received = []
        self.node.create_subscription(
            AckermannDriveStamped, "/drive_raw", received.append, _reliable_qos()
        )
        best_effort_odom_pub = self.node.create_publisher(Odometry, "/odom", _best_effort_qos())

        self._spin_for(0.5)
        odom_msg = _make_odom(x=0.5, y=0.0)
        end = time.time() + _TEST_ODOM_TIMEOUT_S * 5.0
        while time.time() < end:
            best_effort_odom_pub.publish(odom_msg)
            rclpy.spin_once(self.node, timeout_sec=0.05)

        self.assertEqual(
            len(received),
            0,
            "tracker_node published /drive_raw from a best_effort /odom publisher; "
            "its /odom subscription is not genuinely QoS reliable",
        )


@launch_testing.post_shutdown_test()
class TestTrackerNodeShutdown(unittest.TestCase):
    def test_clean_exit(self, proc_info):
        launch_testing.asserts.assertExitCodes(proc_info, allowable_exit_codes=[0, -signal.SIGINT])
