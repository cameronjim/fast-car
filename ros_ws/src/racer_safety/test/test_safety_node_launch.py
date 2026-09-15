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

# TTC thresholds are read from the COMMITTED config/vehicle_params.yaml rather than
# overridden here: since 2026-09-13 that file holds PROVISIONAL values (issue #36), so the
# thresholds this test exercises are the ones the car will actually boot with. They are read
# out of the yaml instead of being copied as literals so that a future tuning change cannot
# leave this test proving a number nothing ships with. The node is launched with NO
# ttc_warning_s/ttc_brake_s parameter at all, which is the launch-file-free default path.
_REPO_ROOT = pathlib.Path(__file__).resolve().parents[4]
with open(_REPO_ROOT / "config" / "vehicle_params.yaml", "r", encoding="utf-8") as _handle:
    _VEHICLE_PARAMS = yaml.safe_load(_handle)
_TTC_WARNING_S = _VEHICLE_PARAMS["limits"]["ttc_warning_s"]
_TTC_BRAKE_S = _VEHICLE_PARAMS["limits"]["ttc_brake_s"]
assert _TTC_BRAKE_S is not None and _TTC_BRAKE_S > 0.0, (
    "config/vehicle_params.yaml limits.ttc_brake_s is unset, so the layer-3 TTC gate is inert "
    "on hardware (GitHub issue #36). This test refuses to pass silently in that state."
)
assert _TTC_WARNING_S is not None and _TTC_WARNING_S > 0.0, (
    "config/vehicle_params.yaml limits.ttc_warning_s is unset (GitHub issue #36)."
)

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


def _engages(events, source: str) -> list:
    """PHASE_ENGAGE records for one gate source.

    /safety/events carries gate TRANSITIONS, not a per-cycle status dump (GitHub issue #37):
    one record when a gate engages, one when that engagement releases. An intervention COUNT
    (claude-docs/09-evaluation.md) is a count of PHASE_ENGAGE records, so that is what these
    tests count.
    """
    return [e for e in events if e.source == source and e.phase == SafetyEvent.PHASE_ENGAGE]


def _releases(events, source: str) -> list:
    return [e for e in events if e.source == source and e.phase == SafetyEvent.PHASE_RELEASE]


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
                # Deliberately no ttc_warning_s / ttc_brake_s here: the node must pick up the
                # committed vehicle_params values on its own.
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
        bounds_engages = _engages(events, "bounds_clamp")
        self.assertGreaterEqual(
            len(bounds_engages),
            1,
            "no bounds_clamp PHASE_ENGAGE /safety/events published for an out-of-bounds "
            "command (claude-docs/05-safety.md: an unlogged intervention is a bug)",
        )
        # The command was out of bounds for EVERY cycle of that 1s window (10 cycles at this
        # test's control rate), which the pre-fix node logged as ~10 separate interventions.
        # A small ceiling rather than exactly 1 because a single-cycle watchdog blip on a
        # loaded runner legitimately breaks one engagement into two (the watchdog
        # short-circuits gate evaluation, so bounds_clamp releases and re-engages).
        self.assertLessEqual(
            len(bounds_engages),
            3,
            f"bounds_clamp logged {len(bounds_engages)} interventions for ONE sustained "
            "out-of-bounds command -- /safety/events is counting cycles again, not "
            "interventions (GitHub issue #37)",
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
        # Exactly ONE engagement for one continuous stretch of silence, however long it
        # lasts (GitHub issue #37: this used to be one record per cycle, so the count
        # measured the operator's start-up latency rather than the car's behaviour).
        watchdog_engages = _engages(events, "watchdog")
        self.assertEqual(
            len(watchdog_engages),
            1,
            f"expected exactly one watchdog PHASE_ENGAGE record for one continuous stretch "
            f"of /drive_raw silence, got {len(watchdog_engages)}: "
            f"{[(e.phase, round(e.duration_s, 3), e.detail) for e in events if e.source == 'watchdog']}",
        )
        self.assertTrue(all(e.severity == SafetyEvent.SEVERITY_BRAKE for e in watchdog_engages))
        self.assertEqual(
            _releases(events, "watchdog"),
            [],
            "the watchdog released while /drive_raw was still silent",
        )

    def test_engaged_gate_does_not_re_emit_every_cycle(self):
        """Regression test for GitHub issue #37.

        A gate that stays engaged used to re-publish its /safety/events record on every
        control cycle (measured: 248 watchdog records in 14s at 50 Hz), so counting records
        measured how LONG the gate was engaged rather than how many times it engaged --
        and claude-docs/09-evaluation.md reports that count as a metric. Here the watchdog
        is already engaged before the measurement window opens, so a correct node emits
        NOTHING at all for the whole window.
        """
        events = []
        self.node.create_subscription(SafetyEvent, "/safety/events", events.append, _reliable_qos())
        # No /drive_raw publisher at all in this test: silence long enough that the watchdog
        # is engaged and settled before the measurement window opens.
        self._spin_for(0.3 + _WATCHDOG_TIMEOUT_S * 5.0)

        events.clear()
        measure_s = 3.0
        self._spin_for(measure_s)

        expected_cycles = int(measure_s * _TEST_CONTROL_RATE_HZ)
        self.assertEqual(
            len(events),
            0,
            f"safety_node emitted {len(events)} /safety/events records over {measure_s}s "
            f"({expected_cycles} control cycles) during which NO gate changed state -- it is "
            f"re-emitting per cycle again (GitHub issue #37): "
            f"{[(e.source, e.phase, e.detail) for e in events[:5]]}",
        )

    def test_engage_and_release_bracket_one_intervention(self):
        """One intervention produces one PHASE_ENGAGE record and, when it ends, one
        PHASE_RELEASE record carrying how long it lasted."""
        events = []
        self.node.create_subscription(SafetyEvent, "/safety/events", events.append, _reliable_qos())
        drive_raw_pub = self.node.create_publisher(
            AckermannDriveStamped, "/drive_raw", _reliable_qos()
        )
        self._spin_for(0.3)

        # Fresh commands first, so any watchdog engagement left over from another test is
        # released before the window that matters opens.
        cmd = _make_drive(steering=0.0, speed=1.0)
        self._publish_steadily(drive_raw_pub, cmd, seconds=1.0)

        events.clear()
        silence_s = _WATCHDOG_TIMEOUT_S * 5.0
        self._spin_for(silence_s)  # the intervention: one continuous stretch of silence

        engages = _engages(events, "watchdog")
        self.assertEqual(
            len(engages),
            1,
            f"expected one watchdog PHASE_ENGAGE record, got {len(engages)}",
        )
        self.assertEqual(engages[0].duration_s, 0.0)
        self.assertEqual(_releases(events, "watchdog"), [], "released while still silent")

        # End the intervention.
        self._publish_steadily(drive_raw_pub, cmd, seconds=0.6)

        releases = _releases(events, "watchdog")
        self.assertEqual(
            len(releases),
            1,
            f"expected one watchdog PHASE_RELEASE record once commands resumed, got "
            f"{len(releases)}",
        )
        self.assertEqual(releases[0].severity, SafetyEvent.SEVERITY_BRAKE)
        self.assertGreater(
            releases[0].duration_s,
            _WATCHDOG_TIMEOUT_S,
            "the release record must report how long the engagement actually lasted",
        )
        self.assertLess(
            releases[0].duration_s,
            silence_s + 5.0,
            f"implausible engagement duration {releases[0].duration_s}s reported for a "
            f"~{silence_s}s intervention",
        )

    def test_no_scan_leaves_the_ttc_gate_inert(self):
        """With the TTC thresholds now set in the committed config (issue #36), the gate is
        armed -- but only when a scan exists. With no /scan publisher at all there is no range
        to compute a TTC from, so a fast forward command must pass through untouched and no
        `ttc` event may appear. This is the regression guard on "setting the thresholds did
        not change behaviour on a car with no LiDAR fitted"."""
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

        events.clear()
        self._publish_steadily(drive_raw_pub, _make_drive(steering=0.0, speed=5.0), seconds=1.0)

        self.assertGreater(len(drive_out), 0)
        self.assertGreater(
            drive_out[-1].drive.speed, 1.0, "a forward command was braked with no /scan present"
        )
        self.assertEqual(
            [e for e in events if e.source == "ttc"],
            [],
            "safety_node emitted a ttc event with no /scan publisher",
        )

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
        # ttc = 0.05 m / ~5 m/s = 0.01 s, far below the committed brake threshold.
        self.assertLess(0.05 / 5.0, _TTC_BRAKE_S)
        close_scan = _make_scan(range_m=0.05)
        self._publish_steadily(
            drive_raw_pub, forward_cmd, seconds=1.0, scan_pub=scan_pub, scan=close_scan
        )

        self.assertEqual(
            drive_out[-1].drive.speed, 0.0, "safety_node did not TTC-brake on a close obstacle"
        )
        ttc_brake_engages = [
            e for e in _engages(events, "ttc") if e.severity == SafetyEvent.SEVERITY_BRAKE
        ]
        self.assertGreaterEqual(
            len(ttc_brake_engages), 1, "no TTC brake PHASE_ENGAGE /safety/events published"
        )
        # One continuous close-obstacle condition is one intervention, not one per cycle
        # (GitHub issue #37). Ceiling rather than exactly 1 for the same watchdog-blip reason
        # as the bounds_clamp test above.
        self.assertLessEqual(
            len(ttc_brake_engages),
            3,
            f"TTC logged {len(ttc_brake_engages)} brake interventions for ONE continuous "
            "close-obstacle condition -- /safety/events is counting cycles, not interventions",
        )

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

        self.assertGreater(len(drive_out), 0, "safety_node published no /drive before the test")
        held_steering = drive_out[-1].drive.steering_angle
        before_count = len(drive_out)

        cmd = _make_drive(steering=0.1, speed=3.0)
        self._publish_steadily(best_effort_pub, cmd, seconds=_WATCHDOG_TIMEOUT_S * 10.0)

        self.assertGreater(len(drive_out), 0, "safety_node stopped publishing /drive entirely")
        # A genuinely-incompatible publisher means safety_node never saw a fresh command, so
        # it must still be watchdog-zeroing throughout.
        #
        # Speed is the evidence; steering is asserted as HELD, not as centred. Since the
        # 2026-09-14 command-path review a zero-throttle gate holds the last commanded
        # steering angle rather than snapping the rack to centre (gate_logic.hpp, "STEERING ON
        # A ZERO-THROTTLE GATE"), and this method shares one long-lived safety_node with every
        # other test in this file, so whatever angle a previous test left behind is what
        # SHOULD still be on /drive. Asserting 0.0 here would have been asserting the old
        # centring behaviour under a name that is about QoS.
        self.assertEqual(drive_out[-1].drive.speed, 0.0)
        self.assertEqual(drive_out[-1].drive.steering_angle, held_steering)
        self.assertFalse(
            any(abs(m.drive.steering_angle - 0.1) < 1e-3 for m in drive_out[before_count:]),
            "a steering angle from the best_effort publisher's command reached /drive: the "
            "/drive_raw subscription is not genuinely reliable",
        )

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
            # Cleared BEFORE the parameter is set, not after: the fault engages on the very
            # first cycle after the service call returns, and /safety/events now emits that
            # engagement ONCE (GitHub issue #37) -- a clear() racing that single record would
            # drop the only evidence the fail-closed path ran.
            events.clear()
            _set_inject_fault(True)
            self._publish_steadily(drive_raw_pub, nominal_cmd, seconds=0.5)

            self.assertGreater(len(drive_out), 0)
            self.assertEqual(drive_out[-1].drive.steering_angle, 0.0)
            self.assertEqual(drive_out[-1].drive.speed, 0.0)
            # Fail-closed still logs -- exactly once for one continuous fault, like every
            # other gate. The fault short-circuits gate evaluation entirely, so no watchdog
            # blip can split this engagement: it is deterministically one record.
            fault_engages = _engages(events, "internal_fault")
            self.assertEqual(
                len(fault_engages),
                1,
                "expected exactly one internal_fault PHASE_ENGAGE /safety/events record for "
                f"one continuous injected fault, got {len(fault_engages)}: "
                f"{[(e.source, e.phase) for e in events]}",
            )
            self.assertEqual(fault_engages[0].severity, SafetyEvent.SEVERITY_BRAKE)
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
