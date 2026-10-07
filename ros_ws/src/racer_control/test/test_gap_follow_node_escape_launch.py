"""L3 test of gap_follow_node's reverse escape (2026-10-06 night floor finding (b)).

The node is launched through the real launch/gap_follow.launch.py with `reverse_escape:=true`,
the test params file (control rate 20 Hz, synthetic-scan yaw) and a second params file that
shortens the escape timings so the test is quick (escape_after_s 0.5 s, escape_retry_after_s
1.0 s). The test plays safety_node: it reads /drive_raw and publishes the gated /drive the node
watches, refusing forward requests (speed 0) and either passing or refusing reverse ones.

Checks, in order:
  * nose-in to a wall across the lane, forward refused: after escape_after_s the node requests
    REVERSE on /drive_raw, never beyond escape_speed_mps, logs "reverse escape 1/3", and after
    escape_distance_m of commanded travel logs "reverse escape complete" and asks to go forward
    again;
  * the same with reverse refused too (the rear corridor): the next escapes are aborted
    ("reverse escape aborted"), each waits escape_retry_after_s after the previous one, and
    after escape_max_attempts without forward progress the node holds and stops reversing;
  * gap_follow_node never publishes /drive (only this test does, standing in for safety_node).
"""

from __future__ import annotations

import os

# Own domain per launch test file, see test_tracker_node_launch.py.
os.environ.setdefault("ROS_DOMAIN_ID", "88")

import itertools
import math
import signal
import tempfile
import time
import unittest
from pathlib import Path

import launch
import launch_testing
import launch_testing.actions
import launch_testing.asserts
import pytest
import rclpy
import yaml
from ackermann_msgs.msg import AckermannDriveStamped
from ament_index_python.packages import get_package_share_directory
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from rclpy.qos import HistoryPolicy, QoSProfile, ReliabilityPolicy
from sensor_msgs.msg import LaserScan

_PARAMS_FILE = str(Path(__file__).resolve().parent / "fixtures" / "gap_follow_test_params.yaml")
_ESCAPE_AFTER_S = 0.5
_RETRY_AFTER_S = 1.0
_ESCAPE_SPEED_MPS = -0.5
_FOV_RAD = 4.7
_NUM_RAYS = 1080

# Shorter escape timings for the test, merged into a copy of the shared params file (the launch
# file takes one params_file; its reverse_escape argument turns the escape on).
_ESCAPE_PARAMS = {
    "gap_follow_node": {
        "ros__parameters": {
            "escape_after_s": _ESCAPE_AFTER_S,
            "escape_retry_after_s": _RETRY_AFTER_S,
            "escape_speed_mps": _ESCAPE_SPEED_MPS,
            "max_speed_mps": 1.0,
        }
    }
}


def _reliable_qos() -> QoSProfile:
    return QoSProfile(
        reliability=ReliabilityPolicy.RELIABLE, history=HistoryPolicy.KEEP_LAST, depth=10
    )


def _best_effort_qos() -> QoSProfile:
    return QoSProfile(
        reliability=ReliabilityPolicy.BEST_EFFORT, history=HistoryPolicy.KEEP_LAST, depth=10
    )


@pytest.mark.launch_test
def generate_test_description():
    launch_file = Path(get_package_share_directory("racer_control")) / "launch"
    # params_file takes one file; merge the shared fixture with the escape timings into one.
    with open(_PARAMS_FILE, encoding="utf-8") as handle:
        merged = yaml.safe_load(handle)
    merged["gap_follow_node"]["ros__parameters"].update(
        _ESCAPE_PARAMS["gap_follow_node"]["ros__parameters"]
    )
    with tempfile.NamedTemporaryFile("w", suffix=".yaml", delete=False) as handle:
        yaml.safe_dump(merged, handle)
    gap_follow = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(str(launch_file / "gap_follow.launch.py")),
        launch_arguments={"params_file": handle.name, "reverse_escape": "true"}.items(),
    )
    return launch.LaunchDescription([gap_follow, launch_testing.actions.ReadyToTest()])


def _trap_scan() -> LaserScan:
    """A vehicle-aligned scan (yaw 0, racer_gym_bridge's ray ordering): a 1.1 m lane with a wall
    across it 0.30 m ahead of the head (about 0.125 m ahead of the body's front, so no steering
    gives the probe's 0.3 m of travel)."""
    walls = [(0.30, -0.55, 0.30, 0.55), (-3.0, 0.55, 0.30, 0.55), (-3.0, -0.55, 0.30, -0.55)]
    msg = LaserScan()
    msg.header.frame_id = "laser"
    msg.angle_min = -_FOV_RAD / 2.0
    msg.angle_max = _FOV_RAD / 2.0
    msg.angle_increment = _FOV_RAD / (_NUM_RAYS - 1)
    msg.range_min = 0.0
    msg.range_max = 30.0
    ranges = []
    for i in range(_NUM_RAYS):
        bearing = msg.angle_min + i * msg.angle_increment
        dx, dy = math.cos(bearing), math.sin(bearing)
        best = math.inf
        for x0, y0, x1, y1 in walls:
            ex, ey = x1 - x0, y1 - y0
            denom = dx * ey - dy * ex
            if abs(denom) < 1e-12:
                continue
            t = (x0 * ey - y0 * ex) / denom
            u = (x0 * dy - y0 * dx) / denom
            if t > 0.0 and 0.0 <= u <= 1.0:
                best = min(best, t)
        ranges.append(best)
    msg.ranges = ranges
    return msg


class TestGapFollowNodeReverseEscape(unittest.TestCase):
    @classmethod
    def setUpClass(cls):
        rclpy.init()

    @classmethod
    def tearDownClass(cls):
        rclpy.shutdown()

    def setUp(self):
        self.node = rclpy.create_node("test_gap_follow_escape_client")
        self.requests: list[AckermannDriveStamped] = []
        self.node.create_subscription(
            AckermannDriveStamped, "/drive_raw", self.requests.append, _reliable_qos()
        )
        self.scan_pub = self.node.create_publisher(LaserScan, "/scan", _best_effort_qos())
        self.gated_pub = self.node.create_publisher(
            AckermannDriveStamped, "/drive", _reliable_qos()
        )
        self.scan = _trap_scan()
        end = time.time() + 1.0
        while time.time() < end:  # discovery
            rclpy.spin_once(self.node, timeout_sec=0.05)

    def tearDown(self):
        self.node.destroy_node()

    def _run(self, seconds: float, pass_reverse: bool, stop=None) -> list[tuple[float, float]]:
        """Feed the trap scan at 10 Hz and play safety_node at 50 Hz: forward refused, reverse
        passed or refused. Returns (receive time, requested speed) for every /drive_raw seen,
        ending early once `stop(seen)` is true."""
        seen: list[tuple[float, float]] = []
        consumed = len(self.requests)
        end = time.time() + seconds
        next_scan = 0.0
        next_gate = 0.0
        while time.time() < end:
            now = time.time()
            if now >= next_scan:
                self.scan_pub.publish(self.scan)
                next_scan = now + 0.1
            if now >= next_gate:
                request = self.requests[-1].drive.speed if self.requests else 0.0
                gated = AckermannDriveStamped()
                gated.drive.speed = request if (pass_reverse and request < 0.0) else 0.0
                self.gated_pub.publish(gated)
                next_gate = now + 0.02
            rclpy.spin_once(self.node, timeout_sec=0.005)
            while consumed < len(self.requests):
                seen.append((time.time(), self.requests[consumed].drive.speed))
                consumed += 1
            if stop is not None and stop(seen):
                break
        return seen

    @staticmethod
    def _reverse_bursts(seen: list[tuple[float, float]]) -> list[tuple[float, float]]:
        """(start, end) times of each run of consecutive reverse requests."""
        bursts: list[tuple[float, float]] = []
        start = None
        last = None
        for t, speed in seen:
            if speed < 0.0:
                if start is None:
                    start = t
                last = t
            elif start is not None:
                bursts.append((start, last))
                start = None
        if start is not None:
            bursts.append((start, last))
        return bursts

    def test_escape_reverses_completes_then_aborts_when_reverse_is_refused(self, proc_output):
        # 1. Forward refused, reverse passed: one escape, then forward again. Stops at the first
        # forward request after the escape, so the next escape belongs to part 2.
        def forward_after_reverse(seen):
            speeds = [s for _, s in seen]
            return any(s < 0.0 for s in speeds) and speeds[-1] > 0.0

        seen = self._run(8.0, pass_reverse=True, stop=forward_after_reverse)
        speeds = [s for _, s in seen]
        self.assertTrue(speeds, "no /drive_raw at all")
        self.assertTrue(all(math.isfinite(s) for s in speeds))
        reverse = [s for s in speeds if s < 0.0]
        self.assertTrue(reverse, "the node never requested reverse while nose-in and refused")
        self.assertGreaterEqual(min(reverse), _ESCAPE_SPEED_MPS - 1e-3)
        first_reverse = next(i for i, s in enumerate(speeds) if s < 0.0)
        self.assertGreater(first_reverse, 0, "reversed before ever asking to go forward")
        self.assertTrue(
            any(s > 0.0 for s in speeds[first_reverse:]), "never asked to go forward again"
        )
        proc_output.assertWaitFor("reverse escape 1/3", timeout=5, stream="stderr")
        proc_output.assertWaitFor("reverse escape complete", timeout=5, stream="stderr")

        # 2. Reverse refused too (the rear corridor): every further escape is aborted, the
        # next one waits at least escape_retry_after_s, and after escape_max_attempts (3)
        # without forward progress the node holds and stops reversing.
        seen = self._run(5.0, pass_reverse=False)
        proc_output.assertWaitFor("reverse escape aborted", timeout=5, stream="stderr")
        proc_output.assertWaitFor("attempts without forward progress", timeout=5, stream="stderr")
        bursts = self._reverse_bursts(seen)
        self.assertGreaterEqual(len(bursts), 2, f"expected attempts 2 and 3, got {bursts}")
        self.assertLessEqual(len(bursts), 2, f"more than escape_max_attempts: {bursts}")
        for (_, end), (start, _) in itertools.pairwise(bursts):
            self.assertGreater(start - end, 0.9 * _RETRY_AFTER_S, "retried before the wait")
        for start, end in bursts:
            self.assertLess(end - start, 0.5, "an aborted escape kept reversing")
        # Holding: nothing but forward requests (refused by the gate) after the last burst.
        tail = [s for t, s in seen if t > bursts[-1][1] + 0.1]
        self.assertTrue(tail)
        self.assertTrue(all(s >= 0.0 for s in tail), "reversed again after holding")

        # 3. gap_follow_node never publishes /drive.
        publishers = self.node.get_publishers_info_by_topic("/drive")
        self.assertEqual(
            [p.node_name for p in publishers if p.node_name == "gap_follow_node"],
            [],
            "gap_follow_node published /drive",
        )


@launch_testing.post_shutdown_test()
class TestGapFollowNodeReverseEscapeShutdown(unittest.TestCase):
    def test_clean_exit(self, proc_info):
        launch_testing.asserts.assertExitCodes(proc_info, allowable_exit_codes=[0, -signal.SIGINT])
