"""L3 node tests for vesc_odometry_node (claude-docs/12-testing.md L3).

The node is fed SYNTHETIC vesc_msgs/VescStateStamped on its input topic, exactly as the f1tenth
vesc_driver would publish them, so neither the driver, a serial port nor a VESC is needed.
Checklist covered: STALE fault before any input; nominal input -> wheel speed, integrated
along-track distance, frame ids and covariance on /odom/wheel; reverse ERPM subtracts distance;
every /telemetry/vesc/* value and the fault name; a gap in the stamps is not bridged; STALE
again on silence; QoS (best_effort subscription, reliable publishers); read-only (the node
publishes nothing but /odom/wheel and /telemetry/vesc/*, in particular no command topic);
clean exit.

The expected speeds are DERIVED from config/vehicle_params.yaml at test time (same reasoning
as test_pwm_output_node_launch.py): a test with its own copy of the gear ratio would stop
testing the real conversion the moment the gear ratio is measured.
"""

from __future__ import annotations

import math
import os

# Pinned BEFORE rclpy is imported: colcon may run launch tests of different packages in one
# DDS domain. Distinct from every domain already claimed in this workspace (77-88).
os.environ.setdefault("ROS_DOMAIN_ID", "89")

import pathlib
import signal
import time
import unittest

import launch
import launch_testing
import launch_testing.actions
import launch_testing.asserts
import pytest
import rclpy
import yaml
from launch_ros.actions import Node as LaunchNode
from nav_msgs.msg import Odometry
from rclpy.qos import HistoryPolicy, QoSProfile, ReliabilityPolicy
from std_msgs.msg import Float32, String
from vesc_msgs.msg import VescStateStamped

_REPO_ROOT = pathlib.Path(__file__).resolve().parents[4]
with open(_REPO_ROOT / "config" / "vehicle_params.yaml", encoding="utf-8") as _handle:
    _PARAMS = yaml.safe_load(_handle)

_POLE_PAIRS = float(_PARAMS["drivetrain"]["pole_pairs"])
_GEAR_RATIO = float(_PARAMS["drivetrain"]["gear_ratio"])
_RADIUS_M = float(_PARAMS["tires"]["nominal_radius_m"])
_MPS_PER_ERPM = 2.0 * math.pi * _RADIUS_M / (60.0 * _POLE_PAIRS * _GEAR_RATIO)

_STATE_TOPIC = "/vesc/sensors/core"
_STALE_TIMEOUT_S = 0.3
_MAX_GAP_S = 0.2
_TWIST_X_VARIANCE = 0.04
_SAMPLE_PERIOD_S = 0.02  # the driver's 50 Hz poll

_TELEMETRY = (
    "erpm",
    "voltage_v",
    "current_motor_a",
    "current_input_a",
    "temp_fet_degc",
    "temp_motor_degc",
)


def _state(stamp_s: float, erpm: float, fault: int = 0) -> VescStateStamped:
    msg = VescStateStamped()
    msg.header.stamp.sec = int(stamp_s)
    msg.header.stamp.nanosec = round((stamp_s - int(stamp_s)) * 1e9)
    msg.state.speed = erpm
    msg.state.voltage_input = 11.7
    msg.state.current_motor = 3.5
    msg.state.current_input = 1.25
    msg.state.temp_fet = 35.5
    msg.state.temp_motor = 41.0
    msg.state.fault_code = fault
    return msg


def _stamp_s(msg) -> float:
    return msg.header.stamp.sec + msg.header.stamp.nanosec * 1e-9


def _reliable(depth: int = 50) -> QoSProfile:
    return QoSProfile(
        reliability=ReliabilityPolicy.RELIABLE, history=HistoryPolicy.KEEP_LAST, depth=depth
    )


@pytest.mark.launch_test
def generate_test_description():
    node = LaunchNode(
        package="racer_drivers",
        executable="vesc_odometry_node",
        name="vesc_odometry_node",
        parameters=[
            {
                "state_topic": _STATE_TOPIC,
                "stale_timeout_s": _STALE_TIMEOUT_S,
                "max_integration_gap_s": _MAX_GAP_S,
                "twist_linear_x_variance": _TWIST_X_VARIANCE,
            }
        ],
        output="screen",
    )
    return launch.LaunchDescription([node, launch_testing.actions.ReadyToTest()])


class TestVescOdometryNode(unittest.TestCase):
    @classmethod
    def setUpClass(cls):
        rclpy.init()
        cls.node = rclpy.create_node("test_vesc_odometry_client")
        cls.odom: list[Odometry] = []
        cls.faults: list[str] = []
        cls.telemetry: dict[str, list[float]] = {leaf: [] for leaf in _TELEMETRY}
        cls.node.create_subscription(Odometry, "/odom/wheel", cls.odom.append, _reliable())
        cls.node.create_subscription(
            String, "/telemetry/vesc/fault", lambda m: cls.faults.append(m.data), _reliable()
        )
        for leaf in _TELEMETRY:
            cls.node.create_subscription(
                Float32,
                f"/telemetry/vesc/{leaf}",
                lambda m, leaf=leaf: cls.telemetry[leaf].append(m.data),
                _reliable(),
            )
        # Reliable publisher: the node's best_effort subscription must accept it (the real
        # driver publishes reliable, rclcpp::QoS{10}).
        cls.pub = cls.node.create_publisher(VescStateStamped, _STATE_TOPIC, _reliable(10))
        deadline = time.time() + 30.0
        while time.time() < deadline and cls.pub.get_subscription_count() == 0:
            rclpy.spin_once(cls.node, timeout_sec=0.1)
        if cls.pub.get_subscription_count() == 0:
            raise RuntimeError("vesc_odometry_node never subscribed to " + _STATE_TOPIC)
        # Let our own subscriptions match the node's publishers too.
        end = time.time() + 1.0
        while time.time() < end:
            rclpy.spin_once(cls.node, timeout_sec=0.05)

    @classmethod
    def tearDownClass(cls):
        cls.node.destroy_node()
        rclpy.shutdown()

    def _spin_for(self, seconds: float) -> None:
        end = time.time() + seconds
        while time.time() < end:
            rclpy.spin_once(self.node, timeout_sec=0.02)

    def _clear(self) -> None:
        self.odom.clear()
        self.faults.clear()
        for values in self.telemetry.values():
            values.clear()

    def _feed(self, start_s: float, erpms: list[float], fault: int = 0) -> None:
        for i, erpm in enumerate(erpms):
            self.pub.publish(_state(start_s + i * _SAMPLE_PERIOD_S, erpm, fault))
            self._spin_for(0.01)
        self._spin_for(0.5)

    def test_a_stale_before_any_input(self):
        """Runs first: nothing has been published yet, so the fault topic must say STALE and
        /odom/wheel must be silent rather than reporting a made-up zero."""
        self._spin_for(_STALE_TIMEOUT_S * 3)
        self.assertIn("STALE_NO_VESC_DATA", self.faults)
        self.assertEqual(self.odom, [])

    def test_b_nominal_speed_distance_frames_and_covariance(self):
        self._clear()
        erpm = 3000.0
        n = 51  # 50 intervals of 0.02 s = 1.0 s
        self._feed(100.0, [erpm] * n)
        # The input subscription is best_effort, so allow for the odd dropped sample on a
        # loaded runner; the distance check below is exact whichever samples arrived, because
        # a constant speed integrates exactly over any sub-gap spacing.
        self.assertGreaterEqual(len(self.odom), n - 3, "about one /odom/wheel per sample")
        first, last = self.odom[0], self.odom[-1]
        expected_speed = erpm * _MPS_PER_ERPM
        self.assertAlmostEqual(last.twist.twist.linear.x, expected_speed, places=9)
        self.assertGreater(last.twist.twist.linear.x, 0.0, "positive ERPM must be forward")
        # The first sample the node ever accepts anchors the distance at zero.
        self.assertEqual(first.pose.pose.position.x, 0.0)
        self.assertAlmostEqual(
            last.pose.pose.position.x, expected_speed * (_stamp_s(last) - _stamp_s(first)), places=6
        )
        self.assertEqual(last.header.frame_id, "odom")
        self.assertEqual(last.child_frame_id, "base_link")
        # Stamps are the driver's stamps, passed through untouched.
        for m in (first, last):
            k = round((_stamp_s(m) - 100.0) / _SAMPLE_PERIOD_S)
            self.assertAlmostEqual(_stamp_s(m), 100.0 + k * _SAMPLE_PERIOD_S, places=6)
        self.assertEqual(last.pose.pose.orientation.w, 1.0)
        self.assertEqual(last.pose.pose.position.y, 0.0)
        # Pose covariance large on every axis; twist.x carries the configured variance.
        for i in range(6):
            self.assertGreaterEqual(last.pose.covariance[i * 6 + i], 1.0e6)
        self.assertAlmostEqual(last.twist.covariance[0], _TWIST_X_VARIANCE)
        for i in range(1, 6):
            self.assertGreaterEqual(last.twist.covariance[i * 6 + i], 1.0e6)

    def test_c_reverse_subtracts_distance(self):
        self._clear()
        # Continue the timeline from test_b (ended at 101.0) so the integrator integrates.
        self._feed(101.02, [-3000.0] * 25)
        self.assertTrue(self.odom)
        speeds = [m.twist.twist.linear.x for m in self.odom]
        self.assertTrue(all(s < 0.0 for s in speeds), "negative ERPM must be reverse")
        distances = [m.pose.pose.position.x for m in self.odom]
        self.assertLess(distances[-1], distances[0])

    def test_d_telemetry_values_and_fault_name(self):
        self._clear()
        self._feed(200.0, [1500.0] * 3, fault=5)
        for leaf, expected in (
            ("erpm", 1500.0),
            ("voltage_v", 11.7),
            ("current_motor_a", 3.5),
            ("current_input_a", 1.25),
            ("temp_fet_degc", 35.5),
            ("temp_motor_degc", 41.0),
        ):
            self.assertTrue(self.telemetry[leaf], f"nothing on /telemetry/vesc/{leaf}")
            self.assertAlmostEqual(self.telemetry[leaf][-1], expected, places=4, msg=leaf)
        self.assertIn("OVER_TEMP_FET", self.faults)
        self.faults.clear()
        self._feed(200.1, [1500.0] * 2, fault=0)
        # _feed spins on past the stale timeout afterwards, so STALE may follow; the fault
        # carried WITH the samples must be NONE and the old fault must be gone.
        self.assertIn("NONE", self.faults)
        self.assertNotIn("OVER_TEMP_FET", self.faults)

    def test_e_gap_is_not_bridged(self):
        self._clear()
        self._feed(300.0, [3000.0] * 2)  # re-anchors after the jump from 200.x (a gap)
        before = self.odom[-1].pose.pose.position.x
        # 5 s later in stamp time: far over max_integration_gap_s, so nothing is added for it.
        self._feed(305.0, [3000.0])
        after_gap = self.odom[-1].pose.pose.position.x
        self.assertAlmostEqual(after_gap, before, places=9)
        self._feed(305.02, [3000.0])
        self.assertAlmostEqual(
            self.odom[-1].pose.pose.position.x, before + 3000.0 * _MPS_PER_ERPM * 0.02, places=9
        )

    def test_f_stale_again_on_silence(self):
        self._clear()
        self._feed(400.0, [0.0] * 3)
        self.faults.clear()
        self._spin_for(_STALE_TIMEOUT_S * 3)
        self.assertIn("STALE_NO_VESC_DATA", self.faults)

    def test_g_qos_best_effort_in_reliable_out(self):
        subs = self.node.get_subscriptions_info_by_topic(_STATE_TOPIC)
        node_subs = [s for s in subs if s.node_name == "vesc_odometry_node"]
        self.assertEqual(len(node_subs), 1)
        self.assertEqual(node_subs[0].qos_profile.reliability, ReliabilityPolicy.BEST_EFFORT)
        for topic in ["/odom/wheel", "/telemetry/vesc/fault"] + [
            f"/telemetry/vesc/{leaf}" for leaf in _TELEMETRY
        ]:
            pubs = [
                p
                for p in self.node.get_publishers_info_by_topic(topic)
                if p.node_name == "vesc_odometry_node"
            ]
            self.assertEqual(len(pubs), 1, topic)
            self.assertEqual(pubs[0].qos_profile.reliability, ReliabilityPolicy.RELIABLE, topic)

    def test_h_read_only_publishes_no_command_topic(self):
        """The node must never be a path to the VESC: everything it publishes is /odom/wheel or
        under /telemetry/vesc/ (plus the two topics every rclcpp node has). Asserted against
        the live graph, not by reading the source."""
        published = {
            name
            for name, _ in self.node.get_publisher_names_and_types_by_node(
                "vesc_odometry_node", "/"
            )
        }
        allowed_infra = {"/rosout", "/parameter_events"}
        for topic in published - allowed_infra:
            self.assertTrue(
                topic == "/odom/wheel" or topic.startswith("/telemetry/vesc/"),
                f"vesc_odometry_node publishes unexpected topic {topic}",
            )
            self.assertNotIn("command", topic)
        subscribed = {
            name
            for name, _ in self.node.get_subscriber_names_and_types_by_node(
                "vesc_odometry_node", "/"
            )
        }
        self.assertEqual(subscribed - allowed_infra, {_STATE_TOPIC})


@launch_testing.post_shutdown_test()
class TestVescOdometryNodeShutdown(unittest.TestCase):
    def test_clean_exit(self, proc_info):
        launch_testing.asserts.assertExitCodes(proc_info, allowable_exit_codes=[0, -signal.SIGINT])
