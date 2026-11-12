"""L3 node tests for racer_state's wheel_twist_node (claude-docs/12-testing.md).

Checklist: nominal input (left steering gives positive yaw rate, right negative, magnitude
g(|delta|) v tan(delta) / L with L and the yaw gain table g from vehicle_params), silence on
/drive (the twist keeps coming with the stale steering variance), silence on /odom/wheel
(nothing published), QoS (the /drive subscription is genuinely reliable: a best_effort publisher
does not reach it), clean shutdown.

The test methods share one node process, so they are named to run in order (unittest sorts
them): test_1 needs a node that has never seen /drive.
"""

from __future__ import annotations

import os

# Own DDS domain: the other launch tests in this container publish /drive and /odom too.
os.environ.setdefault("ROS_DOMAIN_ID", "92")

import math
import signal
import time
import unittest
from pathlib import Path

import launch
import launch_testing
import launch_testing.actions
import launch_testing.asserts
import numpy as np
import pytest
import rclpy
import yaml
from ackermann_msgs.msg import AckermannDriveStamped
from geometry_msgs.msg import TwistWithCovarianceStamped
from launch_ros.actions import Node as LaunchNode
from nav_msgs.msg import Odometry
from rclpy.qos import HistoryPolicy, QoSProfile, ReliabilityPolicy

_REPO_ROOT = Path(__file__).resolve().parents[4]
with (_REPO_ROOT / "config" / "vehicle_params.yaml").open() as _f:
    _VP = yaml.safe_load(_f)
_WHEELBASE_M = float(_VP["chassis"]["wheelbase_m"])
_MAX_STEER_RAD = float(_VP["steering"]["max_angle_rad"])
_GAIN_TABLE = [
    (float(k["abs_angle_rad"]), float(k["gain"])) for k in _VP["steering"]["yaw_rate_gain_table"]
]


def _gain(abs_delta: float) -> float:
    """Piecewise linear in |delta|, held outside the knots (the node's yaw_gain)."""
    xs, ys = zip(*_GAIN_TABLE, strict=True)
    return float(np.interp(abs_delta, xs, ys))


def _gain_slope(abs_delta: float) -> float:
    # Left-hand difference: on a knot the node takes the slope of the segment that ends there.
    eps = 1e-6
    return (_gain(abs_delta) - _gain(abs_delta - eps)) / eps


_DRIVE_TIMEOUT_S = 0.2
_SPEED_MPS = 1.2
# Inside a gain table segment, not on a knot (a float32 command lands on either side of one).
_STEER_RAD = 0.25


def _qos(reliable: bool) -> QoSProfile:
    return QoSProfile(
        reliability=ReliabilityPolicy.RELIABLE if reliable else ReliabilityPolicy.BEST_EFFORT,
        history=HistoryPolicy.KEEP_LAST,
        depth=10,
    )


@pytest.mark.launch_test
def generate_test_description():
    node = LaunchNode(
        package="racer_state",
        executable="wheel_twist_node",
        name="wheel_twist_node",
        parameters=[{"drive_timeout_s": _DRIVE_TIMEOUT_S}],
    )
    return launch.LaunchDescription([node, launch_testing.actions.ReadyToTest()])


class TestWheelTwistNode(unittest.TestCase):
    @classmethod
    def setUpClass(cls):
        rclpy.init()
        cls.node = rclpy.create_node("test_wheel_twist_client")
        cls.twists: list[TwistWithCovarianceStamped] = []
        cls.node.create_subscription(
            TwistWithCovarianceStamped, "/odom/wheel_twist", cls.twists.append, _qos(True)
        )
        cls.wheel_pub = cls.node.create_publisher(Odometry, "/odom/wheel", _qos(True))
        cls.drive_pub = cls.node.create_publisher(AckermannDriveStamped, "/drive", _qos(True))
        cls.best_effort_drive_pub = cls.node.create_publisher(
            AckermannDriveStamped, "/drive", _qos(False)
        )

    @classmethod
    def tearDownClass(cls):
        cls.node.destroy_node()
        rclpy.shutdown()

    def _drive(self, steer: float, reliable: bool = True) -> None:
        msg = AckermannDriveStamped()
        msg.header.stamp = self.node.get_clock().now().to_msg()
        msg.drive.steering_angle = steer
        msg.drive.speed = _SPEED_MPS
        (self.drive_pub if reliable else self.best_effort_drive_pub).publish(msg)

    def _wheel(self, speed: float) -> None:
        msg = Odometry()
        msg.header.stamp = self.node.get_clock().now().to_msg()
        msg.header.frame_id = "odom"
        msg.child_frame_id = "base_link"
        msg.twist.twist.linear.x = speed
        self.wheel_pub.publish(msg)

    def _run(self, seconds: float, speed: float | None, steer: float | None, reliable=True):
        """Publish at 50 Hz for `seconds`: /odom/wheel unless speed is None, /drive unless steer
        is None."""
        end = time.monotonic() + seconds
        while time.monotonic() < end:
            if speed is not None:
                self._wheel(speed)
            if steer is not None:
                self._drive(steer, reliable)
            rclpy.spin_once(self.node, timeout_sec=0.02)

    def _fresh_var_wz(self, speed: float, steer: float) -> float:
        # The node's default noise parameters (wheel_twist.hpp) for a fresh command.
        speed_std = 0.02 + 0.03 * abs(speed)
        c = math.cos(steer)
        g = _gain(abs(steer))
        gp = _gain_slope(abs(steer)) * (1.0 if steer >= 0 else -1.0)
        a = speed * (g / (c * c) + gp * math.tan(steer)) / _WHEELBASE_M
        b = g * math.tan(steer) / _WHEELBASE_M
        return a * a * 0.03**2 + b * b * speed_std**2 + 0.02**2

    def test_1_best_effort_drive_does_not_reach_node(self):
        self._run(0.5, None, None)  # discovery
        self.twists.clear()
        self._run(1.0, _SPEED_MPS, 0.3, reliable=False)
        self.assertGreater(len(self.twists), 10, "no /odom/wheel_twist on nominal wheel input")
        last = self.twists[-1]
        self.assertEqual(
            last.twist.twist.angular.z,
            0.0,
            "a best_effort /drive changed the modelled steering: the subscription is not "
            "genuinely reliable",
        )
        self.assertGreater(last.twist.covariance[35], 10.0 * self._fresh_var_wz(_SPEED_MPS, 0.0))

    def test_2_left_steer_gives_positive_yaw_rate(self):
        self.twists.clear()
        self._run(1.0, _SPEED_MPS, _STEER_RAD)
        last = self.twists[-1]
        self.assertEqual(last.header.frame_id, "base_link")
        self.assertAlmostEqual(last.twist.twist.linear.x, _SPEED_MPS, places=6)
        self.assertEqual(last.twist.twist.linear.y, 0.0)
        expected = _gain(_STEER_RAD) * _SPEED_MPS * math.tan(_STEER_RAD) / _WHEELBASE_M
        self.assertAlmostEqual(last.twist.twist.angular.z, expected, places=6)
        self.assertAlmostEqual(
            last.twist.covariance[35], self._fresh_var_wz(_SPEED_MPS, _STEER_RAD), places=9
        )
        self.assertGreater(last.twist.covariance[7], 0.0, "vy pseudo-measurement needs a variance")

    def test_3_right_steer_and_reverse_signs(self):
        self._run(1.0, _SPEED_MPS, -_STEER_RAD)
        self.assertLess(self.twists[-1].twist.twist.angular.z, 0.0)
        # Reverse with left steering turns clockwise (negative yaw rate).
        self._run(1.0, -0.5, _STEER_RAD)
        last = self.twists[-1]
        self.assertAlmostEqual(
            last.twist.twist.angular.z,
            -0.5 * _gain(_STEER_RAD) * math.tan(_STEER_RAD) / _WHEELBASE_M,
            places=6,
        )
        # Commands beyond the lock are clamped to it.
        self._run(1.0, 1.0, 5.0)
        self.assertAlmostEqual(
            self.twists[-1].twist.twist.angular.z,
            _gain(_MAX_STEER_RAD) * math.tan(_MAX_STEER_RAD) / _WHEELBASE_M,
            places=6,
        )

    def test_4_drive_silence_raises_yaw_variance(self):
        self._run(0.5, _SPEED_MPS, _STEER_RAD)
        fresh = self.twists[-1].twist.covariance[35]
        self._run(_DRIVE_TIMEOUT_S * 4.0, _SPEED_MPS, None)
        last = self.twists[-1]
        self.assertGreater(last.twist.covariance[35], 20.0 * fresh)
        # The last command is still where the servo is modelled to be.
        self.assertAlmostEqual(
            last.twist.twist.angular.z,
            _gain(_STEER_RAD) * _SPEED_MPS * math.tan(_STEER_RAD) / _WHEELBASE_M,
            places=6,
        )

    def test_5_wheel_silence_publishes_nothing(self):
        self._run(0.5, _SPEED_MPS, _STEER_RAD)
        self._run(0.3, None, _STEER_RAD)  # drain
        self.twists.clear()
        self._run(1.0, None, _STEER_RAD)
        self.assertEqual(len(self.twists), 0, "twist published with no /odom/wheel input")


@launch_testing.post_shutdown_test()
class TestWheelTwistNodeShutdown(unittest.TestCase):
    def test_clean_exit(self, proc_info):
        launch_testing.asserts.assertExitCodes(proc_info, allowable_exit_codes=[0, -signal.SIGINT])
