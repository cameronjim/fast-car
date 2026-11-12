"""L3 test of bridge_node's car-like plant options (all default off): plant_speed_deadband_mps,
plant_steer_tau_s and plant_command_delay_s.

With a 0.20 m/s dead band a 0.1 m/s request leaves the car standing and a 0.3 m/s one drives it;
with a 0.5 s command delay the car does not move until about half a second after the command. The
steering lag's arithmetic (63 percent after one time constant, within one control period) is
covered without ROS by test_plant_realism.py; here the node only has to accept the parameter.
Same importorskip guard as test_bridge_node.py: this runs where rclpy, launch_testing and
f1tenth_gym exist.
"""

from __future__ import annotations

import os
import signal
import time
import unittest

import pytest

pytest.importorskip("rclpy")
pytest.importorskip("launch_testing")
pytest.importorskip("f1tenth_gym")
pytest.importorskip("tf2_msgs")

os.environ.setdefault("ROS_DOMAIN_ID", "95")

import launch
import launch_testing
import launch_testing.actions
import launch_testing.asserts
import rclpy
from ackermann_msgs.msg import AckermannDriveStamped
from launch_ros.actions import Node as LaunchNode
from nav_msgs.msg import Odometry
from rclpy.qos import DurabilityPolicy, HistoryPolicy, QoSProfile, ReliabilityPolicy

_DEAD_BAND_MPS = 0.20
_DELAY_S = 0.5


@pytest.mark.launch_test
def generate_test_description():
    bridge_node = LaunchNode(
        package="racer_gym_bridge",
        executable="bridge_node",
        name="bridge_node",
        parameters=[
            {
                "seed": 5,
                "synthetic_track": "lane_round",
                "plant_speed_deadband_mps": _DEAD_BAND_MPS,
                "plant_steer_tau_s": 0.1,
                "plant_command_delay_s": _DELAY_S,
            }
        ],
    )
    return launch.LaunchDescription([bridge_node, launch_testing.actions.ReadyToTest()])


def _qos() -> QoSProfile:
    return QoSProfile(
        reliability=ReliabilityPolicy.RELIABLE,
        durability=DurabilityPolicy.VOLATILE,
        history=HistoryPolicy.KEEP_LAST,
        depth=50,
    )


class TestBridgePlantOptions(unittest.TestCase):
    @classmethod
    def setUpClass(cls):
        rclpy.init()
        cls.node = rclpy.create_node("test_bridge_plant_options")
        cls.truth: list[tuple[float, float]] = []  # (monotonic time, longitudinal speed)
        cls.node.create_subscription(
            Odometry,
            "/sim/ground_truth_odom",
            lambda m: cls.truth.append((time.monotonic(), m.twist.twist.linear.x)),
            _qos(),
        )
        cls.drive_pub = cls.node.create_publisher(AckermannDriveStamped, "/drive", _qos())
        # Warm-up: the gym JITs on its first step (see test_bridge_node.py).
        deadline = time.monotonic() + 60.0
        while not cls.truth and time.monotonic() < deadline:
            rclpy.spin_once(cls.node, timeout_sec=0.1)

    @classmethod
    def tearDownClass(cls):
        cls.node.destroy_node()
        rclpy.shutdown()

    def _drive(self, seconds: float, speed: float) -> None:
        end = time.monotonic() + seconds
        while time.monotonic() < end:
            msg = AckermannDriveStamped()
            msg.drive.speed = speed
            self.drive_pub.publish(msg)
            rclpy.spin_once(self.node, timeout_sec=0.02)

    def test_dead_band_and_delay(self):
        # Under the dead band: still standing after well over the delay.
        self._drive(_DELAY_S + 1.5, 0.1)
        self.assertLess(max(abs(v) for _, v in self.truth[-20:]), 0.02, "0.1 m/s moved the car")
        # Over it: the car does not move at once (the delay), and then it drives.
        self.truth.clear()
        t_command = time.monotonic()
        self._drive(_DELAY_S + 1.5, 0.3)
        moving = [t for t, v in self.truth if v > 0.05]
        self.assertTrue(moving, "0.3 m/s did not move the car")
        self.assertGreater(moving[0] - t_command, _DELAY_S - 0.1, "the delay did not hold")
        self.assertLess(moving[0] - t_command, _DELAY_S + 0.6)
        self.assertGreater(self.truth[-1][1], 0.1)


@launch_testing.post_shutdown_test()
class TestBridgePlantOptionsShutdown(unittest.TestCase):
    def test_clean_exit(self, proc_info):
        launch_testing.asserts.assertExitCodes(proc_info, allowable_exit_codes=[0, -signal.SIGINT])
