"""L3 node tests for safety_node (claude-docs/12-testing.md).

Checklist covered: nominal passthrough on a fresh, in-bounds command; watchdog brake on
/drive_raw silence; TTC brake on a synthetic close-obstacle /scan, its latch and its "ttc brake
released" record; the 2026-10-06 limit-cycle scenario held at zero; the arc corridor (a latch
on a wall releases once the request steers away and its arc is clear); the steering hold while
parked on the obstacle latch (steering stops following /drive_raw after
limits.obstacle_steering_hold_after_s and resumes once the obstacle clears); fail-closed (and clean
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

import math
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

# The 2026-10-06 bench run (bag 2026-10-06T18-51-39_car_teleop): a steady ~0.48 m/s request
# with an obstacle ~0.22 m ahead made the pre-fix gate cycle 0 -> 0.19 -> 0.38 -> 0 m/s
# (docs/notes/ttc-limit-cycle-2026-10-06.md).
_BAG_REQUEST_MPS = 0.48
_BAG_RANGE_M = 0.22
# The scenario must keep violating the CONFIGURED brake threshold as limits.ttc_brake_s is
# tuned down (0.5 -> 0.35 on 2026-10-06): keep the bag's range (above the clearance floor,
# so the TTC half of the gate is what engages) and raise the request until its TTC sits at
# 80 percent of the threshold. At the original 0.5 s this is the bag's own 0.48 m/s.
_MIN_FORWARD_CLEARANCE_M = _VEHICLE_PARAMS["limits"]["min_forward_clearance_m"]
_SCENARIO_REQUEST_MPS = max(_BAG_REQUEST_MPS, _BAG_RANGE_M / (0.8 * _TTC_BRAKE_S))
# Steering hold on the obstacle latch (schema 0.8.0, gate_logic.hpp "STEERING HOLD WHILE PARKED
# ON THE OBSTACLE LATCH"): read from the committed yaml like the thresholds above, so the test
# proves the number the car boots with.
_STEERING_HOLD_AFTER_S = _VEHICLE_PARAMS["limits"]["obstacle_steering_hold_after_s"]
assert _STEERING_HOLD_AFTER_S is not None and _STEERING_HOLD_AFTER_S >= 0.0


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


def _make_wall_scan(distance_m: float, num_beams: int = 100) -> LaserScan:
    """A flat wall across the path, `distance_m` ahead along the vehicle's +x axis.

    Since schema 0.9.0 ("corridor, not wedge", forward_sector.hpp) safety_node brakes on the
    ALONG-TRACK distance x of returns inside a car-width corridor, not on the slant range of
    anything in the sector. A uniform _make_scan(r) is a circle around the head, whose
    in-corridor returns sit at x < r; this wall puts every return ahead at exactly
    x = distance_m, so a scenario can pin the distance the gate sees. Beams that never reach
    the wall (pointing sideways or back) are +inf, an invalid return the node ignores.
    """
    msg = _make_scan(range_m=0.0, num_beams=num_beams)
    ranges = []
    for i in range(num_beams):
        cos_bearing = math.cos(msg.angle_min + i * msg.angle_increment)
        slant_m = distance_m / cos_bearing if cos_bearing > 0.0 else math.inf
        ranges.append(slant_m if slant_m <= msg.range_max else math.inf)
    msg.ranges = ranges
    return msg


def _make_segment_scan(
    x_m: float, y_from_m: float, y_to_m: float, num_beams: int = 100
) -> LaserScan:
    """A wall segment across the path at `x_m` ahead of the head, from `y_from_m` to `y_to_m`
    (left positive), in a vehicle-aligned scan like _make_scan. Rays that miss it are +inf.

    Used by the arc-corridor test (forward_sector.hpp "ARC CORRIDOR", 2026-10-06 late floor
    test): unlike a flat wall across the whole path, a segment can be in the straight corridor
    and clear of a full-lock arc at the same time.
    """
    msg = _make_scan(range_m=0.0, num_beams=num_beams)
    lo_m, hi_m = min(y_from_m, y_to_m), max(y_from_m, y_to_m)
    ranges = []
    for i in range(num_beams):
        bearing = msg.angle_min + i * msg.angle_increment
        cos_bearing = math.cos(bearing)
        hit = math.inf
        if cos_bearing > 0.0:
            y_m = x_m * math.tan(bearing)
            if lo_m <= y_m <= hi_m:
                hit = x_m / cos_bearing
        ranges.append(hit)
    msg.ranges = ranges
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
                #
                # Synthetic scans (_make_scan) are aligned to the VEHICLE (bearing 0 = ahead),
                # not mounted like the real car's LiDAR (sensors.lidar.mount_yaw_rad = pi), so
                # this fixture ignores the binding, the same way racer_control's synthetic-scan
                # fixtures do. The real car always keeps the default (true).
                "laser_yaw_from_vehicle_params": False,
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

        # Obstacle gone: the latch releases (request TTC far above ttc_warning_s), with one
        # PHASE_RELEASE record whose detail says "ttc brake released", and the speed comes
        # back. Also leaves the node's cached scan clear for whatever test runs next.
        self._publish_steadily(
            drive_raw_pub, forward_cmd, seconds=1.5, scan_pub=scan_pub, scan=far_scan
        )
        ttc_brake_releases = [
            e for e in _releases(events, "ttc") if e.severity == SafetyEvent.SEVERITY_BRAKE
        ]
        self.assertGreaterEqual(len(ttc_brake_releases), 1, "the TTC brake latch never released")
        self.assertIn("ttc brake released", ttc_brake_releases[-1].detail)
        self.assertGreater(drive_out[-1].drive.speed, 1.0, "speed did not recover after release")

    def test_ttc_limit_cycle_scenario_is_held_at_zero(self):
        """Regression test for the 2026-10-06 limit cycle (gate_logic.hpp, "THE OBSTACLE GATE
        AND ITS LATCH"). A steady request whose own TTC violates the brake threshold must
        give zero speed on EVERY /drive message, not a ramp that the brake keeps cutting."""
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

        request = _make_drive(steering=0.0, speed=_SCENARIO_REQUEST_MPS)
        self.assertLess(_BAG_RANGE_M / _SCENARIO_REQUEST_MPS, _TTC_BRAKE_S)
        self.assertGreater(_BAG_RANGE_M, _MIN_FORWARD_CLEARANCE_M)
        # A flat wall at the bag's distance, not a uniform circle: with the corridor (schema
        # 0.9.0) a circle of radius 0.22 m reads as x = 0.22 cos(0.6) = 0.18 m at the sector
        # edge, under the clearance floor, so the floor rather than TTC would engage.
        bag_scan = _make_wall_scan(distance_m=_BAG_RANGE_M)
        # Settle: the brake engages within a cycle or two of the scan arriving, and the steering
        # hold (a ttc INFO engage, 2026-10-06 late) follows limits.obstacle_steering_hold_after_s
        # later. Both must be engaged before the window opens, so that "no ttc engage inside the
        # window" below keeps meaning "nothing changed state" rather than racing the hold.
        self._publish_steadily(
            drive_raw_pub,
            request,
            seconds=0.5 + _STEERING_HOLD_AFTER_S + 0.5,
            scan_pub=scan_pub,
            scan=bag_scan,
        )

        drive_out.clear()
        events.clear()
        self._publish_steadily(
            drive_raw_pub, request, seconds=3.0, scan_pub=scan_pub, scan=bag_scan
        )

        self.assertGreater(len(drive_out), 0)
        leaked = [round(m.drive.speed, 3) for m in drive_out if m.drive.speed != 0.0]
        self.assertEqual(
            leaked,
            [],
            "safety_node let throttle through while the request's own TTC violated the brake "
            f"threshold (the 2026-10-06 limit cycle): {leaked}",
        )
        # Sustained and latched: no new engage, no release, and no rate_limit fight.
        self.assertEqual(_engages(events, "ttc"), [], "the TTC brake re-engaged mid-episode")
        self.assertEqual(_releases(events, "ttc"), [], "the TTC brake released mid-episode")
        self.assertEqual(_engages(events, "rate_limit"), [], "rate limiter fought the brake")

        # Clear the obstacle so later tests start from a released latch.
        self._publish_steadily(
            drive_raw_pub, request, seconds=1.0, scan_pub=scan_pub, scan=_make_scan(range_m=100.0)
        )

    def test_steering_hold_while_parked_on_the_obstacle_latch(self):
        """2026-10-06 late (gate_logic.hpp, "STEERING HOLD WHILE PARKED ON THE OBSTACLE LATCH").
        Parked on the obstacle latch, the /drive steering stops following a changing
        /drive_raw steering once limits.obstacle_steering_hold_after_s has passed, stays at the
        angle it had when the hold started, and follows /drive_raw again once the obstacle
        clears. One ttc INFO engage record "steering held while obstacle-latched" with the held
        angle, and one "steering hold released" record on the way out."""
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

        held_angle = 0.2
        far_scan = _make_scan(range_m=100.0)
        # Inside the clearance floor: any forward request latches.
        park_range_m = _MIN_FORWARD_CLEARANCE_M * 0.5
        park_scan = _make_scan(range_m=park_range_m)
        request_mps = 0.5

        # Clear road first (also releases any latch a previous test left behind).
        self._publish_steadily(
            drive_raw_pub,
            _make_drive(steering=held_angle, speed=request_mps),
            seconds=1.0,
            scan_pub=scan_pub,
            scan=far_scan,
        )
        self.assertAlmostEqual(drive_out[-1].drive.steering_angle, held_angle, places=4)

        # Park on the obstacle with the same steering, comfortably past the hold time.
        events.clear()
        self._publish_steadily(
            drive_raw_pub,
            _make_drive(steering=held_angle, speed=request_mps),
            seconds=_STEERING_HOLD_AFTER_S + 1.0,
            scan_pub=scan_pub,
            scan=park_scan,
        )
        self.assertEqual(drive_out[-1].drive.speed, 0.0, "the obstacle latch did not engage")
        hold_engages = [
            e
            for e in _engages(events, "ttc")
            if e.severity == SafetyEvent.SEVERITY_INFO
            and "steering held while obstacle-latched" in e.detail
        ]
        self.assertEqual(
            len(hold_engages),
            1,
            "expected one ttc INFO 'steering held while obstacle-latched' engage record, got "
            f"{[(e.severity, e.phase, e.detail) for e in events if e.source == 'ttc']}",
        )
        self.assertIn("0.2", hold_engages[0].detail, "the held angle is not in the detail")

        # Now wiggle the requested steering every message. The output must not move.
        drive_out.clear()
        wiggle = [-0.3, 0.35, -0.1, 0.05, 0.3, -0.25]
        end = time.time() + 2.0
        i = 0
        while time.time() < end:
            drive_raw_pub.publish(_make_drive(steering=wiggle[i % len(wiggle)], speed=request_mps))
            scan_pub.publish(park_scan)
            rclpy.spin_once(self.node, timeout_sec=0.02)
            i += 1
        self.assertGreater(len(drive_out), 5)
        moved = [
            round(m.drive.steering_angle, 4)
            for m in drive_out
            if abs(m.drive.steering_angle - held_angle) > 1e-6
        ]
        self.assertEqual(
            moved,
            [],
            "the /drive steering followed a changing /drive_raw steering while parked on the "
            f"obstacle latch past the hold time: {moved}",
        )
        self.assertTrue(all(m.drive.speed == 0.0 for m in drive_out))

        # Obstacle clears: the latch releases and the steering follows /drive_raw again.
        resumed_angle = -0.1
        self._publish_steadily(
            drive_raw_pub,
            _make_drive(steering=resumed_angle, speed=request_mps),
            seconds=1.5,
            scan_pub=scan_pub,
            scan=far_scan,
        )
        self.assertAlmostEqual(
            drive_out[-1].drive.steering_angle,
            resumed_angle,
            places=4,
            msg="steering did not follow /drive_raw again after the obstacle cleared",
        )
        self.assertGreater(drive_out[-1].drive.speed, 0.0, "speed did not recover after release")
        hold_releases = [
            e
            for e in _releases(events, "ttc")
            if e.severity == SafetyEvent.SEVERITY_INFO and "steering hold released" in e.detail
        ]
        self.assertEqual(len(hold_releases), 1, "expected one 'steering hold released' record")

    def test_latch_releases_when_the_request_steers_away_and_the_arc_is_clear(self):
        """2026-10-06 late floor test (bag 2026-10-06T22-12-40_car_teleop, forward_sector.hpp
        "ARC CORRIDOR"). The gate braked correctly on a wall about 0.33 m ahead at 1.0 m/s
        (TTC 0.33 s), then stayed latched for 20 to 37 s while gap_follow_node steered at full
        lock away from the wall: the straight corridor still had the wall in it. Now the node
        reduces the last /scan on every cycle along the REQUESTED steering arc, so the same
        wall, with the request at full lock away from it, releases the latch (even though the
        steering hold has frozen the /drive steering) and the car drives off on the arc."""
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

        request_mps = 1.0
        wall_m = 0.33
        # 0.33 m ahead, from 2 cm right of the centreline out to 0.6 m right: in the straight
        # corridor, clear of the full-lock-LEFT arc (its outer edge passes about 3 cm off the
        # wall's inner end with the committed wheelbase, lock, mount and corridor width).
        wall_scan = _make_segment_scan(x_m=wall_m, y_from_m=-0.02, y_to_m=-0.60)
        self.assertLess(wall_m / request_mps, _TTC_BRAKE_S)
        self.assertGreater(wall_m, _MIN_FORWARD_CLEARANCE_M)
        far_scan = _make_scan(range_m=100.0)

        # Clear road first (also releases any latch a previous test left behind).
        self._publish_steadily(
            drive_raw_pub,
            _make_drive(steering=0.0, speed=request_mps),
            seconds=1.0,
            scan_pub=scan_pub,
            scan=far_scan,
        )

        # Straight at the wall: latched, and parked past the steering-hold time.
        events.clear()
        self._publish_steadily(
            drive_raw_pub,
            _make_drive(steering=0.0, speed=request_mps),
            seconds=0.5 + _STEERING_HOLD_AFTER_S + 0.5,
            scan_pub=scan_pub,
            scan=wall_scan,
        )
        self.assertEqual(drive_out[-1].drive.speed, 0.0, "the wall did not latch the brake")
        self.assertGreaterEqual(
            len([e for e in _engages(events, "ttc") if e.severity == SafetyEvent.SEVERITY_BRAKE]),
            1,
            "no ttc brake engage record for the wall",
        )

        # Full lock away from the wall, same scan: the arc is clear, so the latch releases.
        drive_out.clear()
        self._publish_steadily(
            drive_raw_pub,
            _make_drive(steering=_STEERING_MAX_RAD, speed=request_mps),
            seconds=1.5,
            scan_pub=scan_pub,
            scan=wall_scan,
        )
        ttc_brake_releases = [
            e for e in _releases(events, "ttc") if e.severity == SafetyEvent.SEVERITY_BRAKE
        ]
        self.assertGreaterEqual(
            len(ttc_brake_releases),
            1,
            "the latch never released with the request at full lock away from the wall (the "
            "straight-corridor bug from the 2026-10-06 late floor test)",
        )
        self.assertIn("ttc brake released", ttc_brake_releases[-1].detail)
        self.assertGreater(
            drive_out[-1].drive.speed, 0.5, "speed did not come back after the release"
        )
        self.assertAlmostEqual(
            drive_out[-1].drive.steering_angle,
            _STEERING_MAX_RAD,
            places=3,
            msg="the /drive steering did not follow the request after the release",
        )

        # Clear the obstacle so later tests start from a released latch.
        self._publish_steadily(
            drive_raw_pub,
            _make_drive(steering=0.0, speed=request_mps),
            seconds=1.0,
            scan_pub=scan_pub,
            scan=far_scan,
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
