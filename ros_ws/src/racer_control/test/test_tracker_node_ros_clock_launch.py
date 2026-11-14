"""L3 node test: tracker_node's opt-in `ros_clock_timers` (GitHub issue 156).

Default false, and always false on the car: the control timer is a WALL timer (claude-docs
tracker_node.cpp CLOCK POLICY). With `ros_clock_timers:=true` and `use_sim_time:=true` it ticks on
/clock instead, so the sim canaries' control loop follows simulated time and not CPU scheduling.
This launches the node that way, with this test as the only /clock publisher, and checks:

  * /clock frozen: no /drive_raw, however long it is frozen and however fresh /odom is;
  * /clock advancing one second of sim time in 50 ms steps, in far less than a second of wall
    time: about control_rate_hz /drive_raw messages per sim second, stamped with sim time.

The wall-timer default is covered by the other tracker_node launch tests, which are unchanged.
"""

from __future__ import annotations

import os

# Distinct from the other launch tests' domains in this workspace (tracker 77, safety 78, clock 79,
# safety clock 80); `colcon test` runs them concurrently in one container.
os.environ.setdefault("ROS_DOMAIN_ID", "83")

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
from rclpy.time import Time
from rosgraph_msgs.msg import Clock

_CONTROL_RATE_HZ = 20.0
_RACELINE_CSV = str((pathlib.Path(__file__).parent / "fixtures" / "tiny_raceline.csv").resolve())


def _reliable_qos() -> QoSProfile:
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
                "use_sim_time": True,
                "ros_clock_timers": True,
                "raceline_path": _RACELINE_CSV,
                "control_rate_hz": _CONTROL_RATE_HZ,
            }
        ],
    )
    return launch.LaunchDescription([tracker_node, launch_testing.actions.ReadyToTest()])


class TestTrackerNodeRosClockTimers(unittest.TestCase):
    @classmethod
    def setUpClass(cls):
        rclpy.init()

    @classmethod
    def tearDownClass(cls):
        rclpy.shutdown()

    def setUp(self):
        self.node = rclpy.create_node("test_tracker_node_ros_clock_client")
        self.drive_raw: list[AckermannDriveStamped] = []
        self.node.create_subscription(
            AckermannDriveStamped, "/drive_raw", self.drive_raw.append, _reliable_qos()
        )
        self.odom_pub = self.node.create_publisher(Odometry, "/odom", _reliable_qos())
        clock_qos = QoSProfile(
            reliability=ReliabilityPolicy.BEST_EFFORT, history=HistoryPolicy.KEEP_LAST, depth=1
        )
        self.clock_pub = self.node.create_publisher(Clock, "/clock", clock_qos)
        self.sim_ns = 10 * 10**9

    def tearDown(self):
        self.node.destroy_node()

    def _publish_odom(self) -> None:
        odom = Odometry()
        odom.header.frame_id = "odom"
        odom.child_frame_id = "base_link"
        odom.pose.pose.orientation.w = 1.0
        self.odom_pub.publish(odom)

    def _publish_clock(self) -> None:
        msg = Clock()
        msg.clock = Time(nanoseconds=self.sim_ns).to_msg()
        self.clock_pub.publish(msg)

    def _spin_for(self, seconds: float) -> None:
        end = time.monotonic() + seconds
        while time.monotonic() < end:
            self._publish_odom()
            rclpy.spin_once(self.node, timeout_sec=0.01)

    def test_control_loop_follows_clock_and_not_the_wall(self):
        self._publish_clock()
        self._spin_for(1.0)  # discovery, the first /clock and a fresh /odom
        self.drive_raw.clear()

        # Frozen /clock: a wall timer would have published about 20 messages by now.
        self._spin_for(1.0)
        self.assertEqual(len(self.drive_raw), 0, "published with /clock frozen")

        # One sim second in 50 ms steps, as fast as the wall allows.
        wall_start = time.monotonic()
        for _ in range(20):
            self.sim_ns += 50_000_000
            self._publish_clock()
            self._publish_odom()
            rclpy.spin_once(self.node, timeout_sec=0.02)
        self._spin_for(0.3)
        self.assertLess(time.monotonic() - wall_start, 1.0 + 0.3 + 0.5)
        count = len(self.drive_raw)
        self.assertGreaterEqual(count, 15, f"only {count} /drive_raw for one sim second")
        self.assertLessEqual(count, 25, f"{count} /drive_raw for one sim second")
        stamps = [Time.from_msg(m.header.stamp).nanoseconds for m in self.drive_raw]
        self.assertTrue(all(10 * 10**9 <= s <= self.sim_ns for s in stamps), stamps[:3])


@launch_testing.post_shutdown_test()
class TestTrackerNodeRosClockShutdown(unittest.TestCase):
    def test_exits_cleanly(self, proc_info):
        launch_testing.asserts.assertExitCodes(proc_info)
