"""L3: the planner node with the prototype switch dynamics_adaptation true starts, drives, publishes
its adaptation status on /pnp/dynamics/adaptation (8 numbers, enabled 1, inside the hard limits)
and takes a reset on /pnp/dynamics/adaptation_reset. With the switch off (every other launch test)
the topic does not exist."""

from __future__ import annotations

import os

os.environ.setdefault("ROS_DOMAIN_ID", "155")

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
import rclpy
from launch_ros.actions import Node as LaunchNode
from racer_pnp_msgs.msg import PlannerStatus
from std_msgs.msg import Empty, Float64MultiArray

_BUNDLE = common.make_bundle()


@pytest.mark.launch_test
def generate_test_description():
    node = LaunchNode(
        package="racer_planner",
        executable="mppi_planner_node",
        name="mppi_planner_node",
        output="screen",
        parameters=[
            {
                "dynamics_bundle": str(_BUNDLE),
                "v_max_mps": 0.4,
                "samples": 128,
                "laser_yaw_from_vehicle_params": False,
                "laser_yaw_offset_rad": 0.0,
                "require_gated_for_start": False,
                "dynamics_adaptation": True,
            }
        ],
    )
    return launch.LaunchDescription([node, launch_testing.actions.ReadyToTest()])


class TestAdaptationOn(unittest.TestCase):
    @classmethod
    def setUpClass(cls):
        rclpy.init()
        cls.node = rclpy.create_node("test_planner_adaptation_client")
        cls.status: list[PlannerStatus] = []
        cls.adapt: list[Float64MultiArray] = []
        cls.node.create_subscription(
            PlannerStatus, "/pnp/planner/status", cls.status.append, common.reliable(50)
        )
        cls.node.create_subscription(
            Float64MultiArray, "/pnp/dynamics/adaptation", cls.adapt.append, common.reliable(10)
        )
        cls.reset_pub = cls.node.create_publisher(
            Empty, "/pnp/dynamics/adaptation_reset", common.reliable(10)
        )
        cls.feeder = common.Feeder(cls.node, speed_mps=0.3)

    @classmethod
    def tearDownClass(cls):
        cls.node.destroy_node()
        rclpy.shutdown()

    def test_drives_publishes_status_and_resets(self):
        self.feeder.pump(4.0)
        self.assertGreater(len(self.status), 20, "the node did not start")
        self.assertEqual(self.status[-1].state, PlannerStatus.STATE_DRIVING)
        self.assertGreater(len(self.adapt), 5, "no adaptation status")
        last = list(self.adapt[-1].data)
        self.assertEqual(len(last), 8)
        enabled, bias, gain, _updates, _skipped, resets, _at_limit, _err = last
        self.assertEqual(enabled, 1.0)
        self.assertLessEqual(abs(bias), 0.5 + 1e-12)
        self.assertLessEqual(abs(gain), 0.3 + 1e-12)
        self.reset_pub.publish(Empty())
        self.feeder.pump(1.0)
        self.assertEqual(list(self.adapt[-1].data)[5], resets + 1.0)


@launch_testing.post_shutdown_test()
class TestAdaptationShutdown(unittest.TestCase):
    def test_clean_exit(self, proc_info):
        launch_testing.asserts.assertExitCodes(proc_info, allowable_exit_codes=[0, -signal.SIGINT])
        common.remove_tree(_BUNDLE.parent)
