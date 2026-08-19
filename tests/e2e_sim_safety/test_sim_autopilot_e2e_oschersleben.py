"""Milestone 5 end-to-end test, SECOND TRACK: a parallel test case to
test_sim_autopilot_e2e.py (NOT a replacement -- that file stays the `gym_oval` regression
test), proving the classical stack also drives the car through the REAL command path on
`config/tracks/oschersleben/raceline.csv`, the twistier real-map-derived second track
committed for roadmap milestone 5 (see docs/notes/milestone-5-browser-teleop.md):

    tracker_node -> /drive_raw -> safety_node -> /drive -> bridge_node (racer_gym)

Same graph shape, same sim-only `/odom` pose-source remap, and the same "asserting the same
progress-with-no-interventions property" as test_sim_autopilot_e2e.py -- see that file's own
module docstring for the full explanation of the warm-up phase, the residual
rate-limit-jitter tolerance, and why this repo does not chase that residual to a
deterministic zero. Every one of those design decisions is inherited unchanged.

PROGRESS METRIC -- deliberately DIFFERENT from test_sim_autopilot_e2e.py, and why: see
tests/l5_tracker_lap/test_tracker_lap_canary_oschersleben.py's module docstring for the full
finding. In short, the windowed nearest-point-on-raceline (Frenet-style) arc-length tracker
both that file and test_sim_autopilot_e2e.py use is only safe when no two points far apart in
arc length are also close together in XY -- true for gym_oval's simple two-turn stadium, not
safe to assume for this real, 28-corner track. A direct telemetry capture (bridge_node +
tracker_node run standalone, outside any test harness) proved the tracker drives this track
correctly while that Frenet helper's own "progress" reading falsely stayed near zero. This
file instead measures progress as cumulative EUCLIDEAN distance traveled between consecutive
odometry samples -- geometry-agnostic, cannot suffer the same failure mode, and every bit as
honest a "the car did not get stuck" signal for the low bar (`_MIN_PROGRESS_M`, well under one
lap) this specific assertion needs.
"""

from __future__ import annotations

import math
import signal
import time
import unittest
from pathlib import Path

import launch
import launch_testing
import launch_testing.actions
import launch_testing.asserts
import rclpy
from ackermann_msgs.msg import AckermannDriveStamped
from launch_ros.actions import Node as LaunchNode
from nav_msgs.msg import Odometry
from racer_msgs.msg import SafetyEvent
from rclpy.qos import HistoryPolicy, QoSProfile, ReliabilityPolicy

_REPO_ROOT = Path(__file__).resolve().parents[2]
_RACELINE_PATH = _REPO_ROOT / "config" / "tracks" / "oschersleben" / "raceline.csv"

_TEST_SEED = 41

# See test_sim_autopilot_e2e.py's own comment on this same override -- identical reasoning,
# unrelated to which track is loaded.
_SAFETY_WATCHDOG_MISSED_CYCLES = 10

_WARMUP_S = 4.0
_MEASURE_WINDOW_S = 15.0
# "Meaningful progress" -- same bar as test_sim_autopilot_e2e.py's gym_oval test, and easily
# clearable on this much longer (~260m) track too (measured standalone telemetry: ~10.6 m/s
# mean commanded speed, i.e. ~150m in this window).
_MIN_PROGRESS_M = 8.0
_STALL_TIMEOUT_S = 8.0
_STALL_PROGRESS_EPSILON_M = 0.05

# See test_sim_autopilot_e2e.py's module docstring point 2 for the full explanation of this
# tolerance class (cross-process clock jitter between tracker_node's and safety_node's own
# independent rate limiters, never a real disagreement). Kept at the same value as the
# gym_oval e2e test: the mechanism this tolerates is a property of the two-process command
# path, not of which raceline is loaded, so there is no reason to expect a different track to
# need a different number here.
_MAX_BENIGN_RATE_LIMIT_EVENTS = 300


def _reliable_qos() -> QoSProfile:
    return QoSProfile(
        reliability=ReliabilityPolicy.RELIABLE, history=HistoryPolicy.KEEP_LAST, depth=10
    )


def generate_test_description():
    bridge_node = LaunchNode(
        package="racer_gym_bridge",
        executable="bridge_node",
        name="bridge_node",
        parameters=[{"seed": _TEST_SEED, "raceline_path": str(_RACELINE_PATH)}],
    )
    safety_node = LaunchNode(
        package="racer_safety",
        executable="safety_node",
        name="safety_node",
        parameters=[{"watchdog_missed_cycles": _SAFETY_WATCHDOG_MISSED_CYCLES}],
    )
    tracker_node = LaunchNode(
        package="racer_control",
        executable="tracker_node",
        name="tracker_node",
        parameters=[{"raceline_path": str(_RACELINE_PATH)}],
        remappings=[("/odom", "/sim/ground_truth_odom")],
    )
    return launch.LaunchDescription(
        [
            bridge_node,
            safety_node,
            tracker_node,
            launch_testing.actions.ReadyToTest(),
        ]
    )


class TestSimAutopilotE2eOschersleben(unittest.TestCase):
    @classmethod
    def setUpClass(cls):
        rclpy.init()

    @classmethod
    def tearDownClass(cls):
        rclpy.shutdown()

    def setUp(self):
        self.node = rclpy.create_node("e2e_sim_autopilot_test_client_oschersleben")

    def tearDown(self):
        self.node.destroy_node()

    def _spin_for(self, seconds: float) -> None:
        end = time.time() + seconds
        while time.time() < end:
            rclpy.spin_once(self.node, timeout_sec=0.05)

    def _wait_for_first(self, topic_type, topic_name: str, timeout_s: float = 60.0):
        received = []
        sub = self.node.create_subscription(
            topic_type, topic_name, received.append, _reliable_qos()
        )
        deadline = time.time() + timeout_s
        while not received and time.time() < deadline:
            rclpy.spin_once(self.node, timeout_sec=0.2)
        self.node.destroy_subscription(sub)
        if not received:
            raise RuntimeError(f"no message received on {topic_name} within {timeout_s}s")
        return received[0]

    def test_autopilot_makes_progress_with_zero_safety_interventions_once_warmed_up(self):
        self._wait_for_first(Odometry, "/sim/ground_truth_odom")
        self._wait_for_first(AckermannDriveStamped, "/drive")
        self._wait_for_first(AckermannDriveStamped, "/drive_raw")

        odom_positions = []
        self.node.create_subscription(
            Odometry,
            "/sim/ground_truth_odom",
            lambda msg: odom_positions.append(
                (msg.pose.pose.position.x, msg.pose.pose.position.y, time.monotonic())
            ),
            _reliable_qos(),
        )
        drive_raw_msgs = []
        self.node.create_subscription(
            AckermannDriveStamped, "/drive_raw", drive_raw_msgs.append, _reliable_qos()
        )
        drive_msgs = []
        self.node.create_subscription(
            AckermannDriveStamped, "/drive", drive_msgs.append, _reliable_qos()
        )
        safety_events = []
        self.node.create_subscription(
            SafetyEvent, "/safety/events", safety_events.append, _reliable_qos()
        )

        self._spin_for(_WARMUP_S)

        safety_events.clear()
        test_start = time.monotonic()
        deadline = test_start + _MEASURE_WINDOW_S
        last_progress_wall = test_start

        distance_traveled_m = 0.0
        prev_xy = None

        while time.monotonic() < deadline:
            rclpy.spin_once(self.node, timeout_sec=0.1)
            if not odom_positions:
                continue
            x, y, wall_t = odom_positions[-1]
            if prev_xy is None:
                prev_xy = (x, y)
                continue
            step = math.hypot(x - prev_xy[0], y - prev_xy[1])
            if step > _STALL_PROGRESS_EPSILON_M:
                last_progress_wall = wall_t
            distance_traveled_m += step
            prev_xy = (x, y)

            if wall_t - last_progress_wall > _STALL_TIMEOUT_S:
                self.fail(
                    f"raw position stalled for > {_STALL_TIMEOUT_S}s (distance traveled so "
                    f"far {distance_traveled_m:.2f}m) during the measurement phase"
                )

        self.assertGreater(len(drive_raw_msgs), 0, "tracker_node never published /drive_raw")
        self.assertGreater(len(drive_msgs), 0, "safety_node never published /drive")
        self.assertGreater(
            max(m.drive.speed for m in drive_msgs),
            0.5,
            "safety_node never passed a meaningful nonzero forward speed through to /drive",
        )
        self.assertGreaterEqual(
            distance_traveled_m,
            _MIN_PROGRESS_M,
            f"autopilot only traveled {distance_traveled_m:.2f}m in {_MEASURE_WINDOW_S}s "
            f"(wanted >= {_MIN_PROGRESS_M}m)",
        )
        non_benign_events = [
            e
            for e in safety_events
            if e.source != "rate_limit" or e.severity != SafetyEvent.SEVERITY_WARNING
        ]
        self.assertEqual(
            len(non_benign_events),
            0,
            "safety_node emitted a /safety/events record during the nominal (post-warm-up) "
            "autopilot run that is NOT the known-benign rate_limit/WARNING class: "
            f"{[(e.source, e.severity, e.detail) for e in non_benign_events]}",
        )
        self.assertLessEqual(
            len(safety_events),
            _MAX_BENIGN_RATE_LIMIT_EVENTS,
            f"safety_node emitted {len(safety_events)} rate_limit/WARNING /safety/events "
            f"during the nominal (post-warm-up) autopilot run, above the "
            f"{_MAX_BENIGN_RATE_LIMIT_EVENTS}-event tolerance",
        )


@launch_testing.post_shutdown_test()
class TestSimAutopilotE2eOscherslebenShutdown(unittest.TestCase):
    def test_exit_codes(self, proc_info):
        launch_testing.asserts.assertExitCodes(proc_info, allowable_exit_codes=[0, -signal.SIGINT])
