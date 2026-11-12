"""L3: a parameter file that still sets parameters planner v0.1 removed must not stop the node.

The car's data/pnp/planner-2026-11-12.yaml set max_intrusion_fraction (and its siblings) for the
self-check gate that v0.1 replaced with the track confidence. rclcpp stores a file's values for
names the node never declares and ignores them (only allow_undeclared_parameters or
automatically_declare_parameters_from_overrides would make use of them), so the node starts, keeps
the parameters it still declares from the same file, and drives.
"""

from __future__ import annotations

import os

os.environ.setdefault("ROS_DOMAIN_ID", "153")

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
import pytest
import rclpy
from launch_ros.actions import Node as LaunchNode
from racer_pnp_msgs.msg import PlannerStatus

_BUNDLE = common.make_bundle()
_PARAMS_DIR = Path(tempfile.mkdtemp(prefix="racer_planner_params_"))
_PARAMS = _PARAMS_DIR / "planner.yaml"
_PARAMS.write_text(
    f"""mppi_planner_node:
  ros__parameters:
    dynamics_bundle: "{_BUNDLE}"
    v_max_mps: 0.4
    samples: 128
    laser_yaw_from_vehicle_params: false
    laser_yaw_offset_rad: 0.0
    # removed in planner v0.1, still present in the car's file
    max_intrusion_fraction: 0.2
    max_boundary_residual_m: 0.4
    selfcheck_hold_s: 1.0
    min_valid_stations: 4
    track_timeout_s: 0.5
""",
    encoding="utf-8",
)


@pytest.mark.launch_test
def generate_test_description():
    node = LaunchNode(
        package="racer_planner",
        executable="mppi_planner_node",
        name="mppi_planner_node",
        output="screen",
        parameters=[str(_PARAMS)],
    )
    return launch.LaunchDescription([node, launch_testing.actions.ReadyToTest()])


class TestParamsFileWithRemovedNames(unittest.TestCase):
    @classmethod
    def setUpClass(cls):
        rclpy.init()
        cls.node = rclpy.create_node("test_planner_params_file_client")
        cls.status: list[PlannerStatus] = []
        cls.node.create_subscription(
            PlannerStatus, "/pnp/planner/status", cls.status.append, common.reliable(50)
        )
        cls.feeder = common.Feeder(cls.node, speed_mps=0.3)

    @classmethod
    def tearDownClass(cls):
        cls.node.destroy_node()
        rclpy.shutdown()

    def test_node_starts_and_drives_with_the_file_values_it_still_declares(self):
        self.feeder.pump(3.0)
        self.assertGreater(len(self.status), 20, "the node did not start")
        self.assertEqual(self.status[-1].state, PlannerStatus.STATE_DRIVING)
        self.assertLessEqual(self.status[-1].v_cap_mps, 0.4 + 1e-6)  # v_max_mps came from the file
        self.assertGreater(self.status[-1].v_cap_mps, 0.3)


@launch_testing.post_shutdown_test()
class TestParamsFileShutdown(unittest.TestCase):
    def test_clean_exit(self, proc_info):
        launch_testing.asserts.assertExitCodes(proc_info, allowable_exit_codes=[0, -signal.SIGINT])
        common.remove_tree(_BUNDLE.parent)
        common.remove_tree(_PARAMS_DIR)
