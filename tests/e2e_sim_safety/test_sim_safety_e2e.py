"""L5-flavored end-to-end test (claude-docs/12-testing.md, milestone 1): the whole command
path for real, no test-only shims.

Launches racer_gym_bridge's bridge_node and racer_safety's safety_node together with NO
topic remaps -- /drive_raw -> safety_node -> /drive -> bridge_node is exactly
claude-docs/04-architecture.md's real graph (unlike tests/l5_tracker_lap, which needed
`/drive_raw` -> `/drive` and ground-truth-odom -> `/odom` remaps because no safety_node
existed yet at that point in the roadmap).

Feeds scripted /drive_raw, asserts the sim's ground-truth pose advances, then stops
/drive_raw and asserts the watchdog zeroes /drive -- and that the simulated vehicle actually
slows down in response, proving the gating really propagates through the whole loop rather
than just reading zero on the wire.
"""

from __future__ import annotations

import math
import signal
import time
import unittest

import launch
import launch_testing
import launch_testing.actions
import launch_testing.asserts
import rclpy
from ackermann_msgs.msg import AckermannDriveStamped
from launch_ros.actions import Node as LaunchNode
from nav_msgs.msg import Odometry
from rclpy.qos import HistoryPolicy, QoSProfile, ReliabilityPolicy

_TEST_SEED = 21
_BRIDGE_STEP_RATE_HZ = 50.0
_SAFETY_CONTROL_RATE_HZ = 10.0
_SAFETY_WATCHDOG_MISSED_CYCLES = 3
_WATCHDOG_TIMEOUT_S = _SAFETY_WATCHDOG_MISSED_CYCLES / _SAFETY_CONTROL_RATE_HZ  # 0.3s

_FORWARD_COMMAND_SPEED_MPS = 3.0
_PROGRESS_WINDOW_S = 3.0
_MIN_PROGRESS_M = 0.5  # generous: at ~3 m/s for 3s minus rate-limit ramp-up, expect several meters


def _reliable_qos() -> QoSProfile:
    return QoSProfile(
        reliability=ReliabilityPolicy.RELIABLE, history=HistoryPolicy.KEEP_LAST, depth=10
    )


def generate_test_description():
    bridge_node = LaunchNode(
        package="racer_gym_bridge",
        executable="bridge_node",
        name="bridge_node",
        parameters=[{"seed": _TEST_SEED, "step_rate_hz": _BRIDGE_STEP_RATE_HZ}],
    )
    safety_node = LaunchNode(
        package="racer_safety",
        executable="safety_node",
        name="safety_node",
        parameters=[
            {
                "control_rate_hz": _SAFETY_CONTROL_RATE_HZ,
                "watchdog_missed_cycles": _SAFETY_WATCHDOG_MISSED_CYCLES,
            }
        ],
    )
    return launch.LaunchDescription(
        [
            bridge_node,
            safety_node,
            launch_testing.actions.ReadyToTest(),
        ]
    )


class TestSimSafetyE2e(unittest.TestCase):
    @classmethod
    def setUpClass(cls):
        rclpy.init()

    @classmethod
    def tearDownClass(cls):
        rclpy.shutdown()

    def setUp(self):
        self.node = rclpy.create_node("e2e_sim_safety_test_client")

    def tearDown(self):
        self.node.destroy_node()

    def _spin_for(self, seconds: float) -> None:
        end = time.time() + seconds
        while time.time() < end:
            rclpy.spin_once(self.node, timeout_sec=0.05)

    def _wait_for_first(self, topic_type, topic_name: str, timeout_s: float = 60.0):
        """Block until at least one message arrives on `topic_name` (also the JIT/startup
        warm-up barrier, same reasoning as sim/bridge/racer_gym_bridge/test/test_bridge_node.py's
        `_wait_for_bridge_up`: f1tenth_gym numba-jits on first reset/step)."""
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

    def test_scripted_drive_raw_advances_pose_then_watchdog_brakes_and_stops_the_car(self):
        # Barrier: both nodes publishing before the timed assertions begin.
        self._wait_for_first(Odometry, "/sim/ground_truth_odom")
        self._wait_for_first(AckermannDriveStamped, "/drive")

        odoms = []
        self.node.create_subscription(
            Odometry, "/sim/ground_truth_odom", odoms.append, _reliable_qos()
        )
        drives = []
        self.node.create_subscription(
            AckermannDriveStamped, "/drive", drives.append, _reliable_qos()
        )
        drive_raw_pub = self.node.create_publisher(
            AckermannDriveStamped, "/drive_raw", _reliable_qos()
        )
        self._spin_for(0.3)  # let discovery settle

        # --- Phase 1: scripted forward /drive_raw, published fast enough to stay well
        # inside the watchdog timeout throughout. ---
        cmd = AckermannDriveStamped()
        cmd.drive.steering_angle = 0.0
        cmd.drive.speed = _FORWARD_COMMAND_SPEED_MPS

        start_pos = odoms[-1].pose.pose.position
        end = time.time() + _PROGRESS_WINDOW_S
        while time.time() < end:
            drive_raw_pub.publish(cmd)
            rclpy.spin_once(self.node, timeout_sec=0.02)

        self.assertGreater(len(odoms), 0)
        self.assertGreater(len(drives), 0)
        end_pos = odoms[-1].pose.pose.position
        traveled_m = math.hypot(end_pos.x - start_pos.x, end_pos.y - start_pos.y)
        self.assertGreater(
            traveled_m,
            _MIN_PROGRESS_M,
            f"sim pose advanced only {traveled_m:.3f}m in {_PROGRESS_WINDOW_S}s of scripted "
            "forward /drive_raw -- the command path (drive_raw -> safety_node -> drive -> "
            "bridge_node) does not appear to be moving the car",
        )
        self.assertGreater(
            drives[-1].drive.speed,
            0.5,
            "safety_node was not passing a nonzero forward speed through to /drive during "
            "the scripted-progress phase",
        )

        # --- Phase 2: stop publishing /drive_raw. The watchdog must zero /drive, and the
        # simulated vehicle must actually slow down in response (not just read zero on the
        # wire). ---
        drives.clear()
        self._spin_for(_WATCHDOG_TIMEOUT_S * 4.0)  # drain in-flight messages
        drives.clear()
        self._spin_for(_WATCHDOG_TIMEOUT_S * 10.0)  # fresh window past the watchdog timeout

        self.assertGreater(len(drives), 0, "safety_node stopped publishing /drive entirely")
        self.assertEqual(drives[-1].drive.steering_angle, 0.0)
        self.assertEqual(drives[-1].drive.speed, 0.0)

        # Give the sim a further settling window to actually decelerate in response to the
        # brake command, then confirm ground-truth forward velocity has dropped substantially
        # from its Phase-1 cruising speed.
        self._spin_for(1.0)
        self.assertLess(
            odoms[-1].twist.twist.linear.x,
            _FORWARD_COMMAND_SPEED_MPS * 0.5,
            "simulated vehicle did not slow down after the watchdog zeroed /drive -- the "
            "brake command does not appear to be reaching the sim",
        )


@launch_testing.post_shutdown_test()
class TestSimSafetyE2eShutdown(unittest.TestCase):
    def test_exit_codes(self, proc_info):
        launch_testing.asserts.assertExitCodes(proc_info, allowable_exit_codes=[0, -signal.SIGINT])
