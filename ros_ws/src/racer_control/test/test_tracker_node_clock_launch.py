"""L3 node test: tracker_node's /odom staleness watchdog must measure on a clock the rest of
the system cannot stop or step (claude-docs/12-testing.md L3; same root cause as GitHub
issue #22 in racer_safety).

tracker_node's documented degradation behaviour (see tracker_node.cpp's header) is to STOP
publishing /drive_raw when /odom goes stale, and let safety_node's watchdog do the braking.
That only works if tracker_node can actually tell that /odom has gone stale.

It used to measure staleness as `this->now() - <receipt stamp taken with this->now()>`, on
the node's ROS clock. With `use_sim_time:=true` and no /clock publisher that clock is pinned
at zero, so the measured age is permanently 0.0, never exceeds `odom_timeout_s`, and the
watchdog never fires: tracker_node keeps publishing pure-pursuit commands computed from a
single FROZEN pose, at 50 Hz, forever. Downstream, safety_node sees a perfectly fresh
/drive_raw stream and passes it through. The car drives on a pose from minutes ago.

(The `use_sim_time:=false` variant of the same bug is a backwards CLOCK_REALTIME step, which
makes the measured age negative -- also never `> odom_timeout_s`, also "fresh". Both are
removed by measuring on RCL_STEADY_TIME; see tracker_node.cpp's CLOCK POLICY comment.)

This test launches tracker_node with `use_sim_time:=true`, publishes no /clock, sends one
/odom message, and asserts /drive_raw goes silent. Pre-fix it never does.
"""

from __future__ import annotations

import os

# Distinct from every other launch test's domain in this workspace (tracker 77, safety 78,
# safety clock 80) -- `colcon test` runs them concurrently in one container and they all
# touch /drive_raw. setdefault so an operator-set ROS_DOMAIN_ID is never clobbered.
os.environ.setdefault("ROS_DOMAIN_ID", "79")

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

_TEST_CONTROL_RATE_HZ = 20.0
_ODOM_TIMEOUT_S = 0.3

# The same committed fixture racer_control's other tests load (a tiny valid raceline);
# tracker_node refuses to start without a loadable raceline_path.
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
                # The point of this file: nothing publishes /clock, so this node's ROS clock
                # stays pinned at zero for its whole life.
                "use_sim_time": True,
                "raceline_path": _RACELINE_CSV,
                "control_rate_hz": _TEST_CONTROL_RATE_HZ,
                "odom_timeout_s": _ODOM_TIMEOUT_S,
            }
        ],
    )
    return launch.LaunchDescription([tracker_node, launch_testing.actions.ReadyToTest()])


class TestTrackerNodeOdomWatchdogClock(unittest.TestCase):
    @classmethod
    def setUpClass(cls):
        rclpy.init()

    @classmethod
    def tearDownClass(cls):
        rclpy.shutdown()

    def setUp(self):
        self.node = rclpy.create_node("test_tracker_node_clock_client")

    def tearDown(self):
        self.node.destroy_node()

    def _spin_for(self, seconds: float) -> None:
        end = time.monotonic() + seconds
        while time.monotonic() < end:
            rclpy.spin_once(self.node, timeout_sec=0.02)

    def test_odom_watchdog_stops_drive_raw_even_with_a_frozen_ros_clock(self):
        drive_raw = []
        self.node.create_subscription(
            AckermannDriveStamped, "/drive_raw", drive_raw.append, _reliable_qos()
        )
        odom_pub = self.node.create_publisher(Odometry, "/odom", _reliable_qos())
        self._spin_for(0.5)  # let discovery settle

        odom = Odometry()
        odom.header.frame_id = "odom"
        odom.child_frame_id = "base_link"
        odom.pose.pose.orientation.w = 1.0
        odom_pub.publish(odom)

        # Let the node pick it up and publish for a moment, proving it IS publishing when
        # odom is fresh -- otherwise "went silent" below would be vacuously true.
        self._spin_for(0.4)
        self.assertGreater(
            len(drive_raw),
            0,
            "tracker_node published no /drive_raw even while /odom was fresh; this test "
            "cannot distinguish watchdog behaviour from a node that never publishes",
        )

        # Now go silent on /odom for many multiples of odom_timeout_s and check that
        # tracker_node stops. Measured on time.monotonic() here for the same reason the node
        # itself now does: this assertion must not depend on the host wall clock either.
        self._spin_for(_ODOM_TIMEOUT_S * 5)
        drive_raw.clear()
        self._spin_for(_ODOM_TIMEOUT_S * 5)

        self.assertEqual(
            len(drive_raw),
            0,
            "tracker_node kept publishing /drive_raw after /odom went silent, because its "
            "ROS clock is frozen (use_sim_time:=true, no /clock) and its staleness "
            f"measurement never advanced. Got {len(drive_raw)} message(s); last speed "
            f"{drive_raw[-1].drive.speed if drive_raw else None}. Staleness must be measured "
            "on a monotonic clock (tracker_node.cpp CLOCK POLICY).",
        )


@launch_testing.post_shutdown_test()
class TestTrackerNodeCleanShutdown(unittest.TestCase):
    def test_exits_cleanly(self, proc_info):
        launch_testing.asserts.assertExitCodes(proc_info)
