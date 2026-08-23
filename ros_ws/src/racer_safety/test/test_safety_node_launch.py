"""L3 node tests for safety_node (claude-docs/12-testing.md).

Checklist covered: nominal passthrough on a fresh, in-bounds command; watchdog brake on
/drive_raw silence; TTC brake on a synthetic close-obstacle /scan; fail-closed (and clean
recovery) on an injected internal fault; a bounds_clamp /safety/events record on an
out-of-bounds command; correct QoS (the /drive_raw subscription is genuinely `reliable`, not
accidentally `best_effort`; the TTC test's /scan publisher is deliberately `best_effort`,
proving the /scan subscription accepts it and therefore cannot itself be `reliable`); clean
shutdown.

One safety_node process is launched for the whole file (matching racer_control's
test_tracker_node_launch.py pattern) and its internal state (previous output, watchdog
timing) legitimately carries across test methods -- each test publishes its own steady input
for long enough to converge past any leftover state from a prior test. The one test that
mutates node-wide state outside its own gate inputs (the fault-injection test) resets that
state in a `finally` block so test order never matters.
"""

from __future__ import annotations

import os

# Pinned BEFORE rclpy is imported: racer_control/test/test_tracker_node_launch.py also
# publishes/subscribes `/drive_raw`, and `colcon test` runs different packages' launch_testing
# suites as CONCURRENT processes in the same container/DDS domain -- without per-file domain
# isolation, that test's tracker_node and this one's safety_node cross-contaminate each
# other's `/drive_raw` traffic. A distinct domain from that file's (see its own matching
# comment). setdefault, not a plain assignment, so an operator-set ROS_DOMAIN_ID is never
# clobbered.
os.environ.setdefault("ROS_DOMAIN_ID", "78")

import signal
import time
import unittest

import launch
import launch_testing
import launch_testing.actions
import launch_testing.asserts
import pytest
import rclpy
from ackermann_msgs.msg import AckermannDriveStamped
from launch_ros.actions import Node as LaunchNode
from racer_msgs.msg import SafetyEvent
from rcl_interfaces.msg import Parameter, ParameterType, ParameterValue
from rcl_interfaces.srv import SetParameters
from rclpy.qos import HistoryPolicy, QoSProfile, ReliabilityPolicy
from sensor_msgs.msg import LaserScan

# Faster than the real 50 Hz (claude-docs/04-architecture.md) so the watchdog timeout below
# is short enough for a quick test, not because the node behaves differently at another rate.
_TEST_CONTROL_RATE_HZ = 10.0
_TEST_WATCHDOG_MISSED_CYCLES = 3
_WATCHDOG_TIMEOUT_S = _TEST_WATCHDOG_MISSED_CYCLES / _TEST_CONTROL_RATE_HZ  # 0.3s

# TTC thresholds are unset (null) in the committed config/vehicle_params.yaml pending Phase
# 1/2 tuning (see racer_safety/src/safety_node.cpp's build_limits() comment) -- overridden
# here via the ttc_warning_s/ttc_brake_s launch parameters (which default to whatever
# vehicle_params holds) specifically so this test can exercise the TTC gate itself ahead of
# real tuning data existing.
_TTC_WARNING_S = 1.0
_TTC_BRAKE_S = 0.5

_STEERING_MAX_RAD = 0.4189  # vehicle_params.yaml steering.max_angle_rad


def _reliable_qos() -> QoSProfile:
    return QoSProfile(
        reliability=ReliabilityPolicy.RELIABLE, history=HistoryPolicy.KEEP_LAST, depth=10
    )


def _best_effort_qos() -> QoSProfile:
    return QoSProfile(
        reliability=ReliabilityPolicy.BEST_EFFORT, history=HistoryPolicy.KEEP_LAST, depth=10
    )


def _make_drive(steering: float, speed: float) -> AckermannDriveStamped:
    msg = AckermannDriveStamped()
    msg.drive.steering_angle = steering
    msg.drive.speed = speed
    return msg


def _make_scan(range_m: float, num_beams: int = 100) -> LaserScan:
    msg = LaserScan()
    msg.angle_min = -1.57
    msg.angle_max = 1.57
    msg.angle_increment = 3.14 / max(num_beams - 1, 1)
    msg.range_min = 0.0
    msg.range_max = 30.0
    msg.ranges = [float(range_m)] * num_beams
    return msg


@pytest.mark.launch_test
def generate_test_description():
    safety_node = LaunchNode(
        package="racer_safety",
        executable="safety_node",
        name="safety_node",
        parameters=[
            {
                "control_rate_hz": _TEST_CONTROL_RATE_HZ,
                "watchdog_missed_cycles": _TEST_WATCHDOG_MISSED_CYCLES,
                "ttc_warning_s": _TTC_WARNING_S,
                "ttc_brake_s": _TTC_BRAKE_S,
            }
        ],
    )
    return launch.LaunchDescription(
        [
            safety_node,
            launch_testing.actions.ReadyToTest(),
        ]
    )


class TestSafetyNode(unittest.TestCase):
    @classmethod
    def setUpClass(cls):
        rclpy.init()
        cls._wait_for_safety_node_up()

    @classmethod
    def tearDownClass(cls):
        rclpy.shutdown()

    @staticmethod
    def _wait_for_safety_node_up() -> None:
        """Block until safety_node has published at least one /drive message.

        With no /drive_raw received yet, the very first cycle's watchdog check trips
        immediately (claude-docs/05-safety.md fail-closed default), so the first /drive
        message IS a brake -- this is simply "the node is alive and publishing", not a
        correctness assertion.
        """
        warmup_node = rclpy.create_node("test_safety_node_warmup")
        try:
            received = []
            warmup_node.create_subscription(
                AckermannDriveStamped, "/drive", received.append, _reliable_qos()
            )
            deadline = time.time() + 30.0
            while not received and time.time() < deadline:
                rclpy.spin_once(warmup_node, timeout_sec=0.2)
            if not received:
                raise RuntimeError("safety_node did not publish /drive within 30s of launch")
        finally:
            warmup_node.destroy_node()

    def setUp(self):
        self.node = rclpy.create_node("test_safety_node_client")

    def tearDown(self):
        self.node.destroy_node()

    def _spin_for(self, seconds: float) -> None:
        end = time.time() + seconds
        while time.time() < end:
            rclpy.spin_once(self.node, timeout_sec=0.05)

    def _publish_steadily(
        self, drive_pub, cmd: AckermannDriveStamped, seconds: float, scan_pub=None, scan=None
    ) -> None:
        """Publish `cmd` (and, if given, `scan`) fast enough to stay well inside the watchdog
        timeout for the whole window, spinning `self.node` throughout."""
        end = time.time() + seconds
        while time.time() < end:
            drive_pub.publish(cmd)
            if scan_pub is not None:
                scan_pub.publish(scan)
            rclpy.spin_once(self.node, timeout_sec=0.02)

    def test_nominal_passthrough_converges_to_in_bounds_command(self):
        drive_out = []
        self.node.create_subscription(
            AckermannDriveStamped, "/drive", drive_out.append, _reliable_qos()
        )
        drive_raw_pub = self.node.create_publisher(
            AckermannDriveStamped, "/drive_raw", _reliable_qos()
        )
        self._spin_for(0.3)  # let discovery settle

        cmd = _make_drive(steering=0.1, speed=3.0)
        self._publish_steadily(drive_raw_pub, cmd, seconds=2.0)

        self.assertGreater(len(drive_out), 0, "safety_node published no /drive")
        # Check the tail of recent messages, not just the very last one: a loaded CI runner
        # can occasionally stretch the gap between two /drive_raw deliveries past the
        # watchdog timeout for a single cycle (a real, transient watchdog blip, not a logic
        # bug -- claude-docs/12-testing.md's watchdog behavior is exercised directly and
        # exhaustively elsewhere, in test_watchdog_brakes_on_drive_raw_silence and the L1
        # gtest suite). Requiring the recent window to be MOSTLY converged proves steady-
        # state passthrough without being flaky on that single-cycle noise.
        recent = drive_out[-10:]
        converged = [
            m
            for m in recent
            if abs(m.drive.steering_angle - 0.1) < 5e-3 and abs(m.drive.speed - 3.0) < 0.1
        ]
        self.assertGreaterEqual(
            len(converged),
            int(len(recent) * 0.8),
            f"safety_node did not stay converged near steering=0.1, speed=3.0: last "
            f"{len(recent)} /drive messages were "
            f"{[(round(m.drive.steering_angle, 3), round(m.drive.speed, 3)) for m in recent]}",
        )

    def test_bounds_clamp_event_on_out_of_bounds_steering(self):
        events = []
        self.node.create_subscription(SafetyEvent, "/safety/events", events.append, _reliable_qos())
        drive_out = []
        self.node.create_subscription(
            AckermannDriveStamped, "/drive", drive_out.append, _reliable_qos()
        )
        drive_raw_pub = self.node.create_publisher(
            AckermannDriveStamped, "/drive_raw", _reliable_qos()
        )
        self._spin_for(0.3)

        cmd = _make_drive(steering=1.0, speed=0.0)  # far beyond steering.max_angle_rad
        self._publish_steadily(drive_raw_pub, cmd, seconds=1.0)

        self.assertGreater(len(drive_out), 0)
        self.assertLessEqual(abs(drive_out[-1].drive.steering_angle), _STEERING_MAX_RAD + 1e-3)
        bounds_events = [e for e in events if e.source == "bounds_clamp"]
        self.assertGreater(
            len(bounds_events),
            0,
            "no bounds_clamp /safety/events published for an out-of-bounds command",
        )

    def test_watchdog_brakes_on_drive_raw_silence(self):
        drive_out = []
        self.node.create_subscription(
            AckermannDriveStamped, "/drive", drive_out.append, _reliable_qos()
        )
        events = []
        self.node.create_subscription(SafetyEvent, "/safety/events", events.append, _reliable_qos())
        drive_raw_pub = self.node.create_publisher(
            AckermannDriveStamped, "/drive_raw", _reliable_qos()
        )
        self._spin_for(0.3)

        cmd = _make_drive(steering=0.0, speed=2.0)
        self._publish_steadily(drive_raw_pub, cmd, seconds=1.0)
        self.assertGreater(len(drive_out), 0, "no /drive before silence began")

        # Stop publishing /drive_raw. Drain in-flight messages, then check a fresh window
        # well past the watchdog timeout (generous multipliers: a loaded CI runner can
        # stretch wall-clock scheduling, and this only needs to be "comfortably past
        # timeout", not tight).
        events.clear()
        self._spin_for(_WATCHDOG_TIMEOUT_S * 4.0)
        drive_out.clear()
        self._spin_for(_WATCHDOG_TIMEOUT_S * 10.0)

        self.assertGreater(len(drive_out), 0, "safety_node stopped publishing /drive entirely")
        latest = drive_out[-1]
        self.assertEqual(latest.drive.steering_angle, 0.0)
        self.assertEqual(latest.drive.speed, 0.0)
        watchdog_events = [e for e in events if e.source == "watchdog"]
        self.assertGreater(
            len(watchdog_events), 0, "no watchdog /safety/events published on silence"
        )
        self.assertTrue(all(e.severity == SafetyEvent.SEVERITY_BRAKE for e in watchdog_events))

    def test_ttc_brakes_on_close_obstacle_scan(self):
        """Also the QoS proof that /scan is a genuinely best_effort subscription: this scan
        publisher is deliberately BEST_EFFORT, and the TTC brake below can only fire if
        safety_node actually received it -- a `reliable`-only subscription would be QoS-
        incompatible with a best_effort publisher and would never see these messages."""
        drive_out = []
        self.node.create_subscription(
            AckermannDriveStamped, "/drive", drive_out.append, _reliable_qos()
        )
        events = []
        self.node.create_subscription(SafetyEvent, "/safety/events", events.append, _reliable_qos())
        drive_raw_pub = self.node.create_publisher(
            AckermannDriveStamped, "/drive_raw", _reliable_qos()
        )
        scan_pub = self.node.create_publisher(LaserScan, "/scan", _best_effort_qos())
        self._spin_for(0.3)

        forward_cmd = _make_drive(steering=0.0, speed=5.0)
        far_scan = _make_scan(range_m=100.0)
        self._publish_steadily(
            drive_raw_pub, forward_cmd, seconds=1.0, scan_pub=scan_pub, scan=far_scan
        )
        self.assertGreater(len(drive_out), 0)
        self.assertGreater(
            drive_out[-1].drive.speed,
            1.0,
            "car did not accelerate forward before the TTC test began",
        )

        events.clear()
        close_scan = _make_scan(range_m=0.05)  # ttc = 0.05 / ~5 m/s << 0.5s brake threshold
        self._publish_steadily(
            drive_raw_pub, forward_cmd, seconds=1.0, scan_pub=scan_pub, scan=close_scan
        )

        self.assertEqual(
            drive_out[-1].drive.speed, 0.0, "safety_node did not TTC-brake on a close obstacle"
        )
        ttc_brake_events = [
            e for e in events if e.source == "ttc" and e.severity == SafetyEvent.SEVERITY_BRAKE
        ]
        self.assertGreater(len(ttc_brake_events), 0, "no TTC brake /safety/events published")

    def test_drive_raw_subscription_is_reliable_not_best_effort(self):
        """A best_effort /drive_raw publisher must be QoS-incompatible with safety_node's
        subscription -- proving it is genuinely `reliable`, matching claude-docs/10-
        conventions.md's command-path QoS rule."""
        drive_out = []
        self.node.create_subscription(
            AckermannDriveStamped, "/drive", drive_out.append, _reliable_qos()
        )
        best_effort_pub = self.node.create_publisher(
            AckermannDriveStamped, "/drive_raw", _best_effort_qos()
        )
        self._spin_for(0.3)

        cmd = _make_drive(steering=0.1, speed=3.0)
        self._publish_steadily(best_effort_pub, cmd, seconds=_WATCHDOG_TIMEOUT_S * 10.0)

        self.assertGreater(len(drive_out), 0, "safety_node stopped publishing /drive entirely")
        # A genuinely-incompatible publisher means safety_node never saw a fresh command, so
        # it must still be watchdog-braking throughout.
        self.assertEqual(drive_out[-1].drive.steering_angle, 0.0)
        self.assertEqual(drive_out[-1].drive.speed, 0.0)

    def test_fail_closed_on_injected_fault_then_recovers(self):
        drive_out = []
        self.node.create_subscription(
            AckermannDriveStamped, "/drive", drive_out.append, _reliable_qos()
        )
        events = []
        self.node.create_subscription(SafetyEvent, "/safety/events", events.append, _reliable_qos())
        drive_raw_pub = self.node.create_publisher(
            AckermannDriveStamped, "/drive_raw", _reliable_qos()
        )
        self._spin_for(0.3)

        nominal_cmd = _make_drive(steering=0.0, speed=2.0)
        self._publish_steadily(drive_raw_pub, nominal_cmd, seconds=0.5)

        set_params_client = self.node.create_client(SetParameters, "/safety_node/set_parameters")
        self.assertTrue(
            set_params_client.wait_for_service(timeout_sec=10.0),
            "/safety_node/set_parameters service not available",
        )

        def _set_inject_fault(value: bool) -> None:
            request = SetParameters.Request()
            request.parameters = [
                Parameter(
                    name="inject_fault",
                    value=ParameterValue(type=ParameterType.PARAMETER_BOOL, bool_value=value),
                )
            ]
            future = set_params_client.call_async(request)
            rclpy.spin_until_future_complete(self.node, future, timeout_sec=10.0)
            self.assertIsNotNone(future.result(), "set_parameters call did not complete")

        try:
            _set_inject_fault(True)
            events.clear()
            self._publish_steadily(drive_raw_pub, nominal_cmd, seconds=0.5)

            self.assertGreater(len(drive_out), 0)
            self.assertEqual(drive_out[-1].drive.steering_angle, 0.0)
            self.assertEqual(drive_out[-1].drive.speed, 0.0)
            fault_events = [
                e
                for e in events
                if e.source == "internal_fault" and e.severity == SafetyEvent.SEVERITY_BRAKE
            ]
            self.assertGreater(len(fault_events), 0, "no internal_fault /safety/events published")
        finally:
            _set_inject_fault(False)

        # Recovery: once the fault is cleared, normal operation must resume (proving the
        # fail-closed path does not permanently corrupt node state).
        self._publish_steadily(drive_raw_pub, nominal_cmd, seconds=1.0)
        self.assertAlmostEqual(drive_out[-1].drive.speed, 2.0, places=1)


@launch_testing.post_shutdown_test()
class TestSafetyNodeShutdown(unittest.TestCase):
    def test_clean_exit(self, proc_info):
        launch_testing.asserts.assertExitCodes(proc_info, allowable_exit_codes=[0, -signal.SIGINT])
