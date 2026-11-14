"""L3: a SIGINT ends with one speed 0 command on /drive_raw.

The node drives on nominal inputs, then gets SIGINT. The last /drive_raw message it published must
carry speed 0 (the planner's own polite end; the hardware mux and safety_node's watchdog remain
what stop the car) and nothing may follow it, then the process exits cleanly.
"""

from __future__ import annotations

import os

os.environ.setdefault("ROS_DOMAIN_ID", "154")

import signal
import subprocess
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
from launch_testing.util import KeepAliveProc

_BUNDLE = common.make_bundle()
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
            "require_gated_for_start": False,  # no safety_node in this test
        }
    ],
)


@pytest.mark.launch_test
def generate_test_description():
    # the node under test is stopped by the test itself: keep the launch alive for the checks
    return launch.LaunchDescription([_NODE, KeepAliveProc(), launch_testing.actions.ReadyToTest()])


class TestPlannerNodeSigint(unittest.TestCase):
    @classmethod
    def setUpClass(cls):
        rclpy.init()
        cls.node = rclpy.create_node("test_planner_shutdown_client")
        cls.drive: list[AckermannDriveStamped] = []
        cls.node.create_subscription(
            AckermannDriveStamped, "/drive_raw", cls.drive.append, common.reliable(50)
        )
        cls.feeder = common.Feeder(cls.node, speed_mps=0.3)

    @classmethod
    def tearDownClass(cls):
        cls.node.destroy_node()
        rclpy.shutdown()

    def test_sigint_publishes_speed_zero_last(self, proc_info):
        self.feeder.pump(4.0)
        self.assertGreater(self.drive[-1].drive.speed, 0.1, "not driving before the SIGINT")
        pids = subprocess.run(
            ["pgrep", "-f", "lib/racer_planner/mppi_planner_node"],
            capture_output=True,
            text=True,
            check=False,
        ).stdout.split()
        self.assertTrue(pids, "could not find the node process")
        before = len(self.drive)
        for pid in pids:
            os.kill(int(pid), signal.SIGINT)
        proc_info.assertWaitForShutdown(process=_NODE, timeout=15)
        # let the client drain what arrived
        for _ in range(100):
            rclpy.spin_once(self.node, timeout_sec=0.01)
        after = self.drive[before:]
        self.assertTrue(after, "nothing was published after the SIGINT was sent")
        self.assertEqual(
            after[-1].drive.speed, 0.0, "the last /drive_raw after SIGINT was not speed 0"
        )
        zeros = [m for m in after if m.drive.speed == 0.0]
        self.assertGreaterEqual(len(zeros), 1)
        # once the zero is out, nothing moves again
        first_zero = next(i for i, m in enumerate(after) if m.drive.speed == 0.0)
        self.assertTrue(all(m.drive.speed == 0.0 for m in after[first_zero:]))


@launch_testing.post_shutdown_test()
class TestPlannerNodeSigintShutdown(unittest.TestCase):
    def test_clean_exit(self, proc_info):
        launch_testing.asserts.assertExitCodes(proc_info, allowable_exit_codes=[0, -signal.SIGINT])
        common.remove_tree(_BUNDLE.parent)
