"""L3: mppi_planner_node on nominal inputs.

/drive_raw at about 50 Hz and STATE_DRIVING, the command inside the speed cap and the steering
limits, a publisher on /drive_raw and none on /drive, QoS of the topics, and a clean SIGINT exit.
"""

from __future__ import annotations

import os

os.environ.setdefault("ROS_DOMAIN_ID", "150")

import math
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
from rclpy.qos import QoSReliabilityPolicy

V_MAX = 0.5
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
                "v_max_mps": V_MAX,
                "samples": 256,
                "laser_yaw_from_vehicle_params": False,
                "laser_yaw_offset_rad": 0.0,
                "require_gated_for_start": False,  # no safety_node in this test
            }
        ],
    )
    return launch.LaunchDescription([node, launch_testing.actions.ReadyToTest()])


class TestPlannerNodeNominal(unittest.TestCase):
    @classmethod
    def setUpClass(cls):
        rclpy.init()
        cls.node = rclpy.create_node("test_planner_client")
        cls.drive: list[tuple[float, AckermannDriveStamped]] = []
        cls.status: list[PlannerStatus] = []
        cls.gated: list[AckermannDriveStamped] = []
        cls.node.create_subscription(
            AckermannDriveStamped,
            "/drive_raw",
            lambda m: cls.drive.append((cls.node.get_clock().now().nanoseconds * 1e-9, m)),
            common.reliable(10),
        )
        cls.node.create_subscription(
            AckermannDriveStamped, "/drive", cls.gated.append, common.reliable(10)
        )
        cls.node.create_subscription(
            PlannerStatus, "/pnp/planner/status", cls.status.append, common.reliable(50)
        )
        cls.feeder = common.Feeder(cls.node, speed_mps=0.3)

    @classmethod
    def tearDownClass(cls):
        cls.node.destroy_node()
        rclpy.shutdown()

    def test_a_nominal_inputs_drive_at_fifty_hertz(self):
        self.feeder.pump(2.0)
        self.drive.clear()
        self.status.clear()
        self.feeder.pump(3.0)
        self.assertGreater(len(self.drive), 120, "fewer than 40 Hz on /drive_raw")
        self.assertLess(len(self.drive), 180, "more than 60 Hz on /drive_raw")
        states = {s.state for s in self.status[-50:]}
        self.assertEqual(states, {PlannerStatus.STATE_DRIVING}, f"states {states}")
        limit = common.VEHICLE["steering_max_rad"]
        for _, m in self.drive:
            self.assertTrue(math.isfinite(m.drive.speed) and math.isfinite(m.drive.steering_angle))
            self.assertGreaterEqual(m.drive.speed, 0.0)
            self.assertLessEqual(m.drive.speed, V_MAX + 1e-6)
            self.assertLessEqual(abs(m.drive.steering_angle), limit)
        self.assertGreater(self.drive[-1][1].drive.speed, 0.1)
        last = self.status[-1]
        self.assertEqual(last.dynamics_model_id, "dyn-l3-v0-2026-11-12")
        self.assertEqual(last.perception_model_id, "geometric-v0")
        self.assertAlmostEqual(last.v_cap_mps, V_MAX, places=5)
        self.assertEqual(last.samples, 256)
        self.assertAlmostEqual(last.est_speed_mps, 0.3, places=5)
        self.assertGreater(last.solve_time_s, 0.0)
        self.assertEqual(len(self.gated), 0, "the planner (or something) published /drive")

    def test_b_publishers_and_qos(self):
        self.feeder.pump(1.0)
        pubs = self.node.get_publishers_info_by_topic("/drive_raw")
        self.assertEqual(
            [p.node_name for p in pubs], ["mppi_planner_node"], "not the only publisher"
        )
        mine = next(p for p in pubs if p.node_name == "mppi_planner_node")
        self.assertEqual(mine.qos_profile.reliability, QoSReliabilityPolicy.RELIABLE)
        drive_pubs = self.node.get_publishers_info_by_topic("/drive")
        self.assertNotIn(
            "mppi_planner_node", [p.node_name for p in drive_pubs], "planner has a /drive publisher"
        )
        status_pub = next(
            p
            for p in self.node.get_publishers_info_by_topic("/pnp/planner/status")
            if p.node_name == "mppi_planner_node"
        )
        self.assertEqual(status_pub.qos_profile.reliability, QoSReliabilityPolicy.RELIABLE)
        plan_pub = next(
            p
            for p in self.node.get_publishers_info_by_topic("/pnp/plan")
            if p.node_name == "mppi_planner_node"
        )
        self.assertEqual(plan_pub.qos_profile.reliability, QoSReliabilityPolicy.RELIABLE)
        scan_sub = next(
            s
            for s in self.node.get_subscriptions_info_by_topic("/scan")
            if s.node_name == "mppi_planner_node"
        )
        self.assertEqual(scan_sub.qos_profile.reliability, QoSReliabilityPolicy.BEST_EFFORT)
        track_sub = next(
            s
            for s in self.node.get_subscriptions_info_by_topic("/pnp/local_track")
            if s.node_name == "mppi_planner_node"
        )
        self.assertEqual(track_sub.qos_profile.reliability, QoSReliabilityPolicy.RELIABLE)

    def test_c_plan_is_published_in_base_link(self):
        from nav_msgs.msg import Path as PathMsg

        paths: list[PathMsg] = []
        sub = self.node.create_subscription(PathMsg, "/pnp/plan", paths.append, common.reliable(5))
        self.feeder.pump(2.0)
        self.node.destroy_subscription(sub)
        self.assertGreater(len(paths), 5)
        nonempty = [p for p in paths if p.poses]
        self.assertGreater(len(nonempty), 3)
        self.assertEqual(nonempty[-1].header.frame_id, "base_link")
        self.assertEqual(len(nonempty[-1].poses), 31)
        self.assertGreater(nonempty[-1].poses[-1].pose.position.x, 0.2)


@launch_testing.post_shutdown_test()
class TestPlannerNodeShutdown(unittest.TestCase):
    def test_clean_exit(self, proc_info):
        launch_testing.asserts.assertExitCodes(proc_info, allowable_exit_codes=[0, -signal.SIGINT])
        common.remove_tree(_BUNDLE.parent)
