"""L3: the warm start gate on the node.

With no gated /drive (safety_node not running) the node holds speed 0, publishes no /drive_raw
during the grace period (the steering it would start from is not known yet), reports STARTING
and, after start_timeout_s, logs which condition is missing. When a /drive appears with a steering
of 0.4 rad (where safety_node's output still sits from the last run) the first /drive_raw starts
from that steering, speed 0 until the gate passes, then the node drives. One publisher on
/drive_raw, none on /drive.
"""

from __future__ import annotations

import os

os.environ.setdefault("ROS_DOMAIN_ID", "153")

import signal
import sys
import unittest
from pathlib import Path

sys.path.insert(0, str(Path(__file__).resolve().parent))

import pytest

# Skips cleanly under a bare `uv run pytest` (no ROS); runs for real under colcon test.
pytest.importorskip("rclpy")

import launch
import launch_testing
import launch_testing.actions
import launch_testing.asserts
import planner_launch_common as common
import pytest
import rclpy
from ackermann_msgs.msg import AckermannDriveStamped
from launch_ros.actions import Node as LaunchNode
from racer_pnp_msgs.msg import PlannerStatus

_BUNDLE = common.make_bundle()
GATED_STEERING_RAD = 0.40
# one planner step: 0.45 * steering rate * 20 ms, the vehicle numbers come from the live params
STEP_RAD = 0.45 * common.VEHICLE["steering_max_rate_rad_per_s"] * 0.02

_NODE = LaunchNode(
    package="racer_planner",
    executable="mppi_planner_node",
    name="mppi_planner_node",
    output="screen",
    parameters=[
        {
            "dynamics_bundle": str(_BUNDLE),
            "v_max_mps": 0.5,
            "samples": 128,
            "laser_yaw_from_vehicle_params": False,
            "laser_yaw_offset_rad": 0.0,
            "start_timeout_s": 1.0,
            "start_gated_grace_s": 3.0,
        }
    ],
)


@pytest.mark.launch_test
def generate_test_description():
    return launch.LaunchDescription([_NODE, launch_testing.actions.ReadyToTest()])


class TestPlannerNodeStartGate(unittest.TestCase):
    @classmethod
    def setUpClass(cls):
        rclpy.init()
        cls.node = rclpy.create_node("test_planner_start_gate_client")
        cls.drive: list[AckermannDriveStamped] = []
        cls.status: list[PlannerStatus] = []
        cls.node.create_subscription(
            AckermannDriveStamped, "/drive_raw", cls._on_drive_raw, common.reliable(10)
        )
        cls.node.create_subscription(
            PlannerStatus, "/pnp/planner/status", cls.status.append, common.reliable(50)
        )
        cls.gated_pub = cls.node.create_publisher(
            AckermannDriveStamped, "/drive", common.reliable(10)
        )
        cls.feeder = common.Feeder(cls.node, speed_mps=0.3)
        cls.gated_on = False

    @classmethod
    def tearDownClass(cls):
        cls.node.destroy_node()
        rclpy.shutdown()

    @classmethod
    def _on_drive_raw(cls, msg):
        cls.drive.append(msg)

    def _publish_gated(self):
        """A stand-in for safety_node: its output sits at 0.4 rad until the first command arrives,
        then it passes the planner's speed and steering through."""
        if not self.gated_on:
            return
        msg = AckermannDriveStamped()
        msg.header.stamp = self.node.get_clock().now().to_msg()
        if self.drive:
            msg.drive.speed = self.drive[-1].drive.speed
            msg.drive.steering_angle = self.drive[-1].drive.steering_angle
        else:
            msg.drive.speed = 0.0
            msg.drive.steering_angle = GATED_STEERING_RAD
        self.gated_pub.publish(msg)

    def test_a_without_the_gated_drive_it_holds_publishes_nothing_and_says_why(self, proc_output):
        self.feeder.pump(1.6)
        self.assertEqual(
            len(self.drive), 0, "published /drive_raw before the gated /drive was seen"
        )
        self.assertGreater(len(self.status), 20)
        # (NO_ODOM and NO_SCAN while the first inputs are still arriving, then STARTING)
        self.assertEqual({s.state for s in self.status[-20:]}, {PlannerStatus.STATE_STARTING})
        proc_output.assertWaitFor("gated /drive not seen", process=_NODE, timeout=5)

    def test_b_the_first_command_starts_from_the_gated_steering_then_it_drives(self):
        self.gated_on = True
        self.drive.clear()
        self.status.clear()
        self.feeder.pump(1.5, on_spin=self._publish_gated)
        self.assertGreater(len(self.drive), 10, "no /drive_raw after the gated /drive appeared")
        first = self.drive[0].drive
        self.assertEqual(first.speed, 0.0)
        self.assertLessEqual(
            abs(first.steering_angle - GATED_STEERING_RAD),
            STEP_RAD + 0.02,
            f"the first command steered {first.steering_angle}, not from {GATED_STEERING_RAD}",
        )
        # speed 0 for every STARTING status, and it slews toward the straight corridor
        for s in self.status:
            if s.state == PlannerStatus.STATE_STARTING:
                self.assertEqual(s.cmd_speed_mps, 0.0)
        self.feeder.pump(2.5, on_spin=self._publish_gated)
        self.assertEqual(self.status[-1].state, PlannerStatus.STATE_DRIVING)
        self.assertGreater(self.drive[-1].drive.speed, 0.0)

    def test_c_one_publisher_on_drive_raw_and_none_on_drive(self):
        names = [p.node_name for p in self.node.get_publishers_info_by_topic("/drive_raw")]
        self.assertEqual(names, ["mppi_planner_node"])
        self.assertNotIn(
            "mppi_planner_node",
            [p.node_name for p in self.node.get_publishers_info_by_topic("/drive")],
        )


@launch_testing.post_shutdown_test()
class TestPlannerNodeStartGateShutdown(unittest.TestCase):
    def test_clean_exit(self, proc_info):
        launch_testing.asserts.assertExitCodes(proc_info, allowable_exit_codes=[0, -signal.SIGINT])
        common.remove_tree(_BUNDLE.parent)
