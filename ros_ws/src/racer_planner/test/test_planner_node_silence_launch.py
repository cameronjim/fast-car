"""L3: silence on each input in turn gives speed 0 and the matching state, while /drive_raw keeps
coming at its rate (so safety_node's watchdog only fires when the planner dies)."""

from __future__ import annotations

import os

os.environ.setdefault("ROS_DOMAIN_ID", "151")

import signal
import sys
import unittest
from pathlib import Path

sys.path.insert(0, str(Path(__file__).resolve().parent))

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
                "v_max_mps": 0.5,
                "samples": 128,
                "laser_yaw_from_vehicle_params": False,
                "laser_yaw_offset_rad": 0.0,
            }
        ],
    )
    return launch.LaunchDescription([node, launch_testing.actions.ReadyToTest()])


class TestPlannerNodeSilence(unittest.TestCase):
    @classmethod
    def setUpClass(cls):
        rclpy.init()
        cls.node = rclpy.create_node("test_planner_silence_client")
        cls.drive: list[AckermannDriveStamped] = []
        cls.status: list[PlannerStatus] = []
        cls.node.create_subscription(
            AckermannDriveStamped, "/drive_raw", cls.drive.append, common.reliable(10)
        )
        cls.node.create_subscription(
            PlannerStatus, "/pnp/planner/status", cls.status.append, common.reliable(50)
        )
        cls.feeder = common.Feeder(cls.node, speed_mps=0.3)

    @classmethod
    def tearDownClass(cls):
        cls.node.destroy_node()
        rclpy.shutdown()

    def _check_silence(self, attr: str, expected_state: int, label: str):
        self.feeder.pump(2.0)
        self.assertEqual(
            self.status[-1].state,
            PlannerStatus.STATE_DRIVING,
            f"{label}: not driving before the silence",
        )
        self.assertGreater(
            self.drive[-1].drive.speed, 0.05, f"{label}: not moving before the silence"
        )
        setattr(self.feeder, attr, False)
        self.feeder.pump(0.8)  # longer than every timeout
        self.drive.clear()
        self.status.clear()
        self.feeder.pump(0.5)
        self.assertGreater(len(self.drive), 15, f"{label}: /drive_raw stopped coming")
        self.assertTrue(
            all(m.drive.speed == 0.0 for m in self.drive), f"{label}: nonzero speed during silence"
        )
        self.assertEqual({s.state for s in self.status}, {expected_state}, f"{label}: wrong state")
        setattr(self.feeder, attr, True)
        self.feeder.pump(2.0)
        self.assertEqual(
            self.status[-1].state, PlannerStatus.STATE_DRIVING, f"{label}: did not recover"
        )

    def test_a_odom_silence(self):
        self._check_silence("odom_on", PlannerStatus.STATE_NO_ODOM, "odom")

    def test_b_scan_silence(self):
        # the feeder sends the track with the scan, so silence the scan alone via a track-only feed
        self.feeder.track_on = True
        self._check_silence("scan_on", PlannerStatus.STATE_NO_SCAN, "scan")

    def test_c_track_silence(self):
        self._check_silence("track_on", PlannerStatus.STATE_NO_TRACK, "track")

    def test_d_too_few_valid_stations_is_no_track(self):
        self.feeder.pump(2.0)
        self.feeder.track_valid_count = 3
        self.feeder.pump(0.5)
        self.status.clear()
        self.drive.clear()
        self.feeder.pump(0.4)
        self.assertEqual({s.state for s in self.status}, {PlannerStatus.STATE_NO_TRACK})
        self.assertTrue(all(m.drive.speed == 0.0 for m in self.drive))
        self.feeder.track_valid_count = 16
        self.feeder.pump(1.5)
        self.assertEqual(self.status[-1].state, PlannerStatus.STATE_DRIVING)


@launch_testing.post_shutdown_test()
class TestPlannerNodeSilenceShutdown(unittest.TestCase):
    def test_clean_exit(self, proc_info):
        launch_testing.asserts.assertExitCodes(proc_info, allowable_exit_codes=[0, -signal.SIGINT])
        common.remove_tree(_BUNDLE.parent)
