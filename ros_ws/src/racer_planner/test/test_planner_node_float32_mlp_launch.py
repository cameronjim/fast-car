"""L3: the planner node with dynamics_mlp_float32 true and a bundle whose residual is live loads
the bundle (its golden rollouts checked in the float32 mode), says so in its log and drives. With
the switch off (every other launch test) the node evaluates the double forward pass."""

from __future__ import annotations

import os

os.environ.setdefault("ROS_DOMAIN_ID", "157")

import signal
import sys
import tempfile
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

_BUNDLE = Path(tempfile.mkdtemp(prefix="racer_planner_l3_")) / "residual"
common.make_fixture.make_bundle(
    _BUNDLE,
    common.VEHICLE,
    residual_on=True,
    nominal=common.make_fixture.NOMINAL,
    v_max=common.V_MAX_VALIDITY,
    model_id="dyn-l3-res-v0-2026-11-12",
)


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
                "dynamics_mlp_float32": True,
            }
        ],
    )
    return launch.LaunchDescription([node, launch_testing.actions.ReadyToTest()]), {"node": node}


class TestFloat32MlpOn(unittest.TestCase):
    @classmethod
    def setUpClass(cls):
        rclpy.init()
        cls.node = rclpy.create_node("test_planner_float32_mlp_client")
        cls.status: list[PlannerStatus] = []
        cls.node.create_subscription(
            PlannerStatus, "/pnp/planner/status", cls.status.append, common.reliable(50)
        )
        cls.feeder = common.Feeder(cls.node, speed_mps=0.3)

    @classmethod
    def tearDownClass(cls):
        cls.node.destroy_node()
        rclpy.shutdown()

    def test_loads_in_the_float32_mode_and_drives(self, proc_output, node):
        proc_output.assertWaitFor("residual enabled", process=node, timeout=10.0)
        proc_output.assertWaitFor("mlp exact float32", process=node, timeout=10.0)
        self.feeder.pump(4.0)
        self.assertGreater(len(self.status), 20, "the node did not start")
        self.assertEqual(self.status[-1].state, PlannerStatus.STATE_DRIVING)


@launch_testing.post_shutdown_test()
class TestFloat32MlpShutdown(unittest.TestCase):
    def test_clean_exit(self, proc_info):
        launch_testing.asserts.assertExitCodes(proc_info, allowable_exit_codes=[0, -signal.SIGINT])
        common.remove_tree(_BUNDLE.parent)
