"""L3 node test for racer_tools' twist_teleop_adapter_node (claude-docs/12-testing.md,
roadmap milestone 5).

Same pytest.importorskip-guarded shape as test_raceline_publisher_node_launch.py: skipped
cleanly under the bare `uv run pytest` L1 run, runs for real under `colcon test` in the
ros-dev image (the l3-and-cpp CI job).

Checklist covered (claude-docs/12-testing.md L3: "correct behavior on nominal input; correct
behavior on silence (watchdog paths); ... clean shutdown"):

  - nominal: a published Twist shows up on /drive_raw converted correctly.
  - silence/timeout: with NO Twist ever published, /drive_raw is still published (never goes
    silent -- see twist_teleop_adapter_node.py's module docstring) and is the zero command.
  - republish-latest: /drive_raw keeps being published at the node's rate even when no new
    Twist arrives in between (more messages received than Twists sent).
  - timeout-after-a-real-Twist: after a nonzero Twist, once `twist_timeout_s` elapses with no
    further Twist, /drive_raw reverts to (and stays at) zero.
"""

from __future__ import annotations

import signal
import time
import unittest

import pytest

pytest.importorskip("rclpy")
pytest.importorskip("launch_testing")

import launch
import launch_testing
import launch_testing.actions
import launch_testing.asserts
import rclpy
from ackermann_msgs.msg import AckermannDriveStamped
from geometry_msgs.msg import Twist
from launch_ros.actions import Node as LaunchNode
from rclpy.qos import HistoryPolicy, QoSProfile, ReliabilityPolicy

_TEST_INPUT_TOPIC = "/test/teleop/cmd_vel"
# Generous relative to a 50 Hz control period so every "assert this arrived before the
# timeout" check below has real margin against CI-runner scheduling/message-delivery latency
# (a much shorter value here was observed flaky on a loaded CI runner: a published Twist
# occasionally had not yet been converted and republished within a 0.15s window).
_TEST_TIMEOUT_S = 1.0


def _reliable_qos() -> QoSProfile:
    return QoSProfile(
        reliability=ReliabilityPolicy.RELIABLE, history=HistoryPolicy.KEEP_LAST, depth=10
    )


@pytest.mark.launch_test
def generate_test_description():
    twist_teleop_adapter_node = LaunchNode(
        package="racer_tools",
        executable="twist_teleop_adapter_node",
        name="twist_teleop_adapter",
        parameters=[
            {
                "input_topic": _TEST_INPUT_TOPIC,
                "control_rate_hz": 50.0,
                "twist_timeout_s": _TEST_TIMEOUT_S,
            }
        ],
    )
    return launch.LaunchDescription(
        [
            twist_teleop_adapter_node,
            launch_testing.actions.ReadyToTest(),
        ]
    )


class TestTwistTeleopAdapterNode(unittest.TestCase):
    @classmethod
    def setUpClass(cls):
        rclpy.init()

    @classmethod
    def tearDownClass(cls):
        rclpy.shutdown()

    def setUp(self):
        self.node = rclpy.create_node("twist_teleop_adapter_test_client")
        self._twist_pub = self.node.create_publisher(Twist, _TEST_INPUT_TOPIC, _reliable_qos())

    def tearDown(self):
        self.node.destroy_node()

    def _spin_for(self, seconds: float) -> None:
        end = time.time() + seconds
        while time.time() < end:
            rclpy.spin_once(self.node, timeout_sec=0.05)

    def _spin_until(self, predicate, timeout_s: float) -> bool:
        """Spin until `predicate()` is true or `timeout_s` elapses; returns whether it was
        satisfied. Used instead of a single fixed `_spin_for` wherever a test needs "this
        specific thing happened" rather than "some fixed amount of wall time passed" -- a
        fixed short sleep proved flaky on a more loaded CI runner (message delivery/scheduling
        latency, not a node bug)."""
        deadline = time.time() + timeout_s
        while time.time() < deadline:
            rclpy.spin_once(self.node, timeout_sec=0.02)
            if predicate():
                return True
        return predicate()

    def _publish_twist(self, linear_x: float, angular_z: float) -> None:
        msg = Twist()
        msg.linear.x = linear_x
        msg.angular.z = angular_z
        self._twist_pub.publish(msg)

    def test_no_twist_ever_published_still_publishes_drive_raw_and_it_is_zero(self):
        received = []
        self.node.create_subscription(
            AckermannDriveStamped, "/drive_raw", received.append, _reliable_qos()
        )
        self._spin_for(1.0)
        self.assertGreater(
            len(received), 0, "expected /drive_raw to be published even with no Twist input"
        )
        last = received[-1]
        self.assertEqual(last.drive.speed, 0.0)
        self.assertEqual(last.drive.steering_angle, 0.0)

    def test_nominal_twist_is_converted_onto_drive_raw(self):
        received = []
        self.node.create_subscription(
            AckermannDriveStamped, "/drive_raw", received.append, _reliable_qos()
        )
        self._spin_for(0.3)  # let the subscription connect
        received.clear()

        self._publish_twist(linear_x=2.0, angular_z=1.0)
        # Poll for the conversion to show up, comfortably under _TEST_TIMEOUT_S -- this test
        # is about nominal conversion, not the timeout boundary (see the dedicated timeout
        # test below for that).
        arrived = self._spin_until(
            lambda: bool(received) and received[-1].drive.speed > 0.0,
            timeout_s=_TEST_TIMEOUT_S / 2,
        )
        self.assertTrue(arrived, "expected the Twist to be converted onto /drive_raw")
        last = received[-1]
        self.assertAlmostEqual(last.drive.speed, 2.0, places=3)
        self.assertGreater(last.drive.steering_angle, 0.0)

    def test_republishes_the_latest_command_continuously_without_a_new_twist(self):
        # This test is specifically about the republish-latest behavior WHILE the command is
        # still fresh, not the timeout-to-zero behavior (see the dedicated timeout test
        # below) -- every wait here stays comfortably under _TEST_TIMEOUT_S.
        received = []
        self.node.create_subscription(
            AckermannDriveStamped, "/drive_raw", received.append, _reliable_qos()
        )
        self._spin_for(0.2)
        self._publish_twist(linear_x=1.5, angular_z=0.0)
        arrived = self._spin_until(
            lambda: bool(received) and received[-1].drive.speed > 0.0,
            timeout_s=_TEST_TIMEOUT_S / 2,
        )
        self.assertTrue(arrived, "expected the Twist to be converted onto /drive_raw")
        count_after_one_twist = len(received)

        # Long enough at 50 Hz to have republished several more times with no further Twist,
        # short enough to stay well under the remainder of _TEST_TIMEOUT_S.
        self._spin_for(_TEST_TIMEOUT_S / 4)
        self.assertGreater(
            len(received),
            count_after_one_twist + 3,
            "expected the node to keep publishing /drive_raw on its own timer, not only when "
            "a new Twist arrives (watchdog-friendly republish-latest semantics)",
        )
        # And still commanding the same nonzero speed (well within twist_timeout_s).
        self.assertAlmostEqual(received[-1].drive.speed, 1.5, places=3)

    def test_after_timeout_with_no_further_twist_drive_raw_reverts_to_zero(self):
        received = []
        self.node.create_subscription(
            AckermannDriveStamped, "/drive_raw", received.append, _reliable_qos()
        )
        self._spin_for(0.2)  # let the subscription connect
        self._publish_twist(linear_x=3.0, angular_z=0.5)

        # Poll (rather than a single fixed sleep) for the command to go nonzero, comfortably
        # under _TEST_TIMEOUT_S so this precondition check itself never races the
        # timeout-to-zero behavior under test below.
        arrived = self._spin_until(
            lambda: bool(received) and received[-1].drive.speed > 0.0,
            timeout_s=_TEST_TIMEOUT_S / 2,
        )
        self.assertTrue(arrived, "precondition: command went nonzero")

        # Wait well past _TEST_TIMEOUT_S with no further Twist.
        self._spin_for(_TEST_TIMEOUT_S + 0.5)
        last = received[-1]
        self.assertEqual(last.drive.speed, 0.0)
        self.assertEqual(last.drive.steering_angle, 0.0)


@launch_testing.post_shutdown_test()
class TestTwistTeleopAdapterNodeShutdown(unittest.TestCase):
    def test_exit_codes(self, proc_info):
        # Either a clean exit(0) or "terminated by SIGINT" is allowable -- same reasoning as
        # test_raceline_publisher_node_launch.py's shutdown test.
        launch_testing.asserts.assertExitCodes(proc_info, allowable_exit_codes=[0, -signal.SIGINT])
