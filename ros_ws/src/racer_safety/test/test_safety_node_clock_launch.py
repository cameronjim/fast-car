"""L3 node test: safety_node's watchdog must measure /drive_raw staleness on a clock the
rest of the system cannot stop or step (claude-docs/12-testing.md L3, GitHub issue #22).

WHAT THIS PINS DOWN

`safety_node` used to compute the /drive_raw age as `this->now() - <receipt stamp taken with
this->now()>`. `this->now()` is the node's ROS clock, and BOTH of its modes can lie about
elapsed time:

  * `use_sim_time:=false`: the ROS clock is CLOCK_REALTIME. NTP, a VM resync, or an operator
    running `date` steps it in either direction. A backwards step makes the measured age
    negative -- exactly the `age=-0.12s` watchdog record reported in issue #22, reproduced
    for this audit by stepping the container host's clock back 5s under a healthy 50 Hz
    /drive_raw stream (`age=-4.858085s`).
  * `use_sim_time:=true` with no /clock publisher: the ROS clock is pinned at zero forever,
    so the measured age is permanently 0.0. The watchdog then considers a /drive_raw that
    has been silent since boot permanently FRESH and never brakes for staleness. That is the
    dangerous direction, and it is what this test exercises, because it is the one a launch
    file can trigger with a single wrong argument and no exotic host-clock conditions at all.

This test launches safety_node with `use_sim_time:=true` and deliberately publishes NO
/clock. It sends one /drive_raw command, stops, and asserts the node brakes and emits a
`watchdog` /safety/events record anyway. Against the pre-fix node this fails: the watchdog
never fires, because its own clock never advances. Against the fixed node it passes, because
the age is measured on RCL_STEADY_TIME (see safety_node.cpp's CLOCK POLICY comment), which
no ROS-level configuration can freeze.

This is deliberately a SEPARATE launch file from test_safety_node_launch.py: that file's
node is launched once for the whole file with the normal (wall-clock) configuration, and
`use_sim_time` is a node-construction-time setting, not something to toggle mid-suite.
"""

from __future__ import annotations

import os

# Pinned BEFORE rclpy is imported, and distinct from every other launch test's domain in
# this workspace (racer_control's tracker test uses 77, racer_safety's main node test 78,
# racer_control's tracker clock test 79): `colcon test` runs these suites as CONCURRENT
# processes in one container, and they all publish /drive_raw. setdefault, not a plain
# assignment, so an operator-set ROS_DOMAIN_ID is never clobbered.
os.environ.setdefault("ROS_DOMAIN_ID", "80")

import time
import unittest

import launch
import launch_testing
import launch_testing.actions
import pytest
import rclpy
from ackermann_msgs.msg import AckermannDriveStamped
from launch_ros.actions import Node as LaunchNode
from racer_msgs.msg import SafetyEvent
from rclpy.qos import HistoryPolicy, QoSProfile, ReliabilityPolicy

# Faster than the real 50 Hz (claude-docs/04-architecture.md) purely so the watchdog timeout
# is short enough for a quick test; the node does not behave differently at another rate.
_TEST_CONTROL_RATE_HZ = 10.0
_TEST_WATCHDOG_MISSED_CYCLES = 3
_WATCHDOG_TIMEOUT_S = _TEST_WATCHDOG_MISSED_CYCLES / _TEST_CONTROL_RATE_HZ  # 0.3s


def _reliable_qos() -> QoSProfile:
    return QoSProfile(
        reliability=ReliabilityPolicy.RELIABLE, history=HistoryPolicy.KEEP_LAST, depth=10
    )


@pytest.mark.launch_test
def generate_test_description():
    safety_node = LaunchNode(
        package="racer_safety",
        executable="safety_node",
        name="safety_node",
        parameters=[
            {
                # The whole point of this file. Nothing publishes /clock, so this node's ROS
                # clock stays pinned at zero for its entire life.
                "use_sim_time": True,
                "control_rate_hz": _TEST_CONTROL_RATE_HZ,
                "watchdog_missed_cycles": _TEST_WATCHDOG_MISSED_CYCLES,
            }
        ],
    )
    return launch.LaunchDescription([safety_node, launch_testing.actions.ReadyToTest()])


class TestSafetyNodeWatchdogClock(unittest.TestCase):
    @classmethod
    def setUpClass(cls):
        rclpy.init()

    @classmethod
    def tearDownClass(cls):
        rclpy.shutdown()

    def setUp(self):
        self.node = rclpy.create_node("test_safety_node_clock_client")

    def tearDown(self):
        self.node.destroy_node()

    def _spin_for(self, seconds: float) -> None:
        end = time.monotonic() + seconds
        while time.monotonic() < end:
            rclpy.spin_once(self.node, timeout_sec=0.02)

    def test_node_publishes_drive_despite_frozen_ros_clock(self):
        """Sanity precondition: the node is alive and its 50 Hz-equivalent loop runs at all.

        The control loop is a wall timer, so a frozen ROS clock must not stop it. If this
        fails, the watchdog assertion below would be meaningless (no cycles, no events).
        """
        drive_out = []
        self.node.create_subscription(
            AckermannDriveStamped, "/drive", drive_out.append, _reliable_qos()
        )
        self._spin_for(2.0)
        self.assertGreater(
            len(drive_out),
            0,
            "safety_node published no /drive at all with use_sim_time:=true and no /clock; "
            "its control loop must be a wall timer, independent of the ROS clock",
        )

    def test_watchdog_brakes_on_silence_even_with_a_frozen_ros_clock(self):
        """The regression test for issue #22's root cause.

        One /drive_raw command, then silence for many multiples of the watchdog timeout. The
        node must brake and say so on /safety/events. A node measuring age on its own
        (frozen) ROS clock measures 0.0s forever and never gets here.
        """
        drive_out = []
        events = []
        self.node.create_subscription(
            AckermannDriveStamped, "/drive", drive_out.append, _reliable_qos()
        )
        self.node.create_subscription(SafetyEvent, "/safety/events", events.append, _reliable_qos())
        drive_raw_pub = self.node.create_publisher(
            AckermannDriveStamped, "/drive_raw", _reliable_qos()
        )
        self._spin_for(0.5)  # let discovery settle

        # A single in-bounds, clearly non-zero command, then nothing ever again.
        cmd = AckermannDriveStamped()
        cmd.drive.steering_angle = 0.1
        cmd.drive.speed = 3.0
        drive_raw_pub.publish(cmd)

        events.clear()
        drive_out.clear()
        self._spin_for(_WATCHDOG_TIMEOUT_S * 10)

        self.assertGreater(len(drive_out), 0, "safety_node stopped publishing /drive")
        self.assertAlmostEqual(
            drive_out[-1].drive.speed,
            0.0,
            places=6,
            msg=(
                "safety_node did not brake on /drive_raw silence when its ROS clock was "
                f"frozen (use_sim_time:=true, no /clock). Last /drive speed was "
                f"{drive_out[-1].drive.speed}. This is issue #22's root cause: staleness "
                "must be measured on a monotonic clock, not the ROS clock."
            ),
        )

        watchdog_events = [e for e in events if e.source == "watchdog"]
        self.assertGreater(
            len(watchdog_events),
            0,
            "safety_node braked but emitted no 'watchdog' /safety/events record "
            "(claude-docs/05-safety.md: an unlogged intervention is a bug). Events seen: "
            f"{sorted({e.source for e in events})}",
        )
        self.assertEqual(
            watchdog_events[-1].severity,
            SafetyEvent.SEVERITY_BRAKE,
            "a watchdog intervention that brakes must be recorded at SEVERITY_BRAKE",
        )

    def test_watchdog_age_is_never_reported_as_negative(self):
        """A negative age must be impossible, not merely handled.

        gate_logic.cpp already treats a negative `drive_raw_age_s` as stale (fail-closed, the
        safe direction), and that stays. But with the age measured on a monotonic clock the
        value should never BE negative in the first place, so no /safety/events detail string
        emitted over this window may contain one. This is the assertion that would have
        turned issue #22's intermittent local flake into a deterministic failure.
        """
        events = []
        self.node.create_subscription(SafetyEvent, "/safety/events", events.append, _reliable_qos())
        self._spin_for(_WATCHDOG_TIMEOUT_S * 10)

        self.assertGreater(len(events), 0, "no /safety/events observed to check")
        negative = [e for e in events if "age=-" in e.detail]
        self.assertEqual(
            negative,
            [],
            "safety_node reported a NEGATIVE /drive_raw age (issue #22). A monotonic clock "
            f"cannot produce one; offending details: {[e.detail for e in negative[:3]]}",
        )


@launch_testing.post_shutdown_test()
class TestSafetyNodeCleanShutdown(unittest.TestCase):
    def test_exits_cleanly(self, proc_info):
        launch_testing.asserts.assertExitCodes(proc_info)
