"""L3 node test for gap_follow_node's compute_on_scan (event-driven commands, 2026-11-12).

With `compute_on_scan:=true` the command half also runs the moment a scan has been processed, so
the scan-to-command latency in /gap_follow/status is the processing time instead of waiting for
the next timer tick. The control rate is 20 Hz here (a 50 ms period, the params file's), so the
two modes are far apart: the timer mode's median is about 25 ms, this mode's a few milliseconds.
What this pins:

  * status says event_driven and the median latency is well below half a period;
  * the timer keeps publishing between scans and no gap between /drive_raw messages reaches two
    periods (the watchdog is fed), and the number of commands is at most the timer's plus one per
    scan;
  * a stale scan still stops output, and the commands stay inside the speed and steering bounds;
  * /drive is never published.
"""

from __future__ import annotations

import os

os.environ.setdefault("ROS_DOMAIN_ID", "101")

import itertools
import json
import math
import signal
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
from std_msgs.msg import String

_PARAMS_FILE = str(Path(__file__).resolve().parent / "fixtures" / "gap_follow_test_params.yaml")
_SCAN_TIMEOUT_S = 0.2
_PERIOD_S = 1.0 / 20.0
_MAX_SPEED_MPS = 1.0
_FOV_RAD = 4.7
_NUM_RAYS = 1080

with open(
    Path(__file__).resolve().parents[4] / "config" / "vehicle_params.yaml", encoding="utf-8"
) as _handle:
    _MAX_STEERING_RAD = float(yaml.safe_load(_handle)["steering"]["max_angle_rad"]) + 1e-3


def _qos(reliable: bool) -> QoSProfile:
    return QoSProfile(
        reliability=ReliabilityPolicy.RELIABLE if reliable else ReliabilityPolicy.BEST_EFFORT,
        history=HistoryPolicy.KEEP_LAST,
        depth=50,
    )


@pytest.mark.launch_test
def generate_test_description():
    launch_file = Path(get_package_share_directory("racer_control")) / "launch"
    gap_follow = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(str(launch_file / "gap_follow.launch.py")),
        launch_arguments={
            "max_speed_mps": str(_MAX_SPEED_MPS),
            "params_file": _PARAMS_FILE,
            "compute_on_scan": "true",
            "status_rate_hz": "5",
        }.items(),
    )
    return launch.LaunchDescription([gap_follow, launch_testing.actions.ReadyToTest()])


def _make_scan(gap_lo_rad: float, gap_hi_rad: float) -> LaserScan:
    msg = LaserScan()
    msg.header.frame_id = "laser"
    msg.angle_min = -_FOV_RAD / 2.0
    msg.angle_max = _FOV_RAD / 2.0
    msg.angle_increment = _FOV_RAD / (_NUM_RAYS - 1)
    msg.range_min = 0.0
    msg.range_max = 30.0
    msg.ranges = [
        5.0 if gap_lo_rad <= msg.angle_min + i * msg.angle_increment <= gap_hi_rad else 1.0
        for i in range(_NUM_RAYS)
    ]
    return msg


class TestGapFollowNodeEventDriven(unittest.TestCase):
    @classmethod
    def setUpClass(cls):
        rclpy.init()

    @classmethod
    def tearDownClass(cls):
        rclpy.shutdown()

    def setUp(self):
        self.node = rclpy.create_node("test_gap_follow_node_event_client")
        self.received: list[tuple[float, AckermannDriveStamped]] = []
        self.status: list[dict] = []
        self.drive_received: list[AckermannDriveStamped] = []
        self.node.create_subscription(
            AckermannDriveStamped,
            "/drive_raw",
            lambda m: self.received.append((time.monotonic(), m)),
            _qos(True),
        )
        self.node.create_subscription(
            AckermannDriveStamped, "/drive", self.drive_received.append, _qos(False)
        )
        self.node.create_subscription(
            String,
            "/gap_follow/status",
            lambda m: self.status.append(json.loads(m.data)),
            _qos(True),
        )
        self.scan_pub = self.node.create_publisher(LaserScan, "/scan", _qos(False))
        self._spin_for(0.5)

    def tearDown(self):
        self.node.destroy_node()

    def _spin_for(self, seconds: float) -> None:
        end = time.time() + seconds
        while time.time() < end:
            rclpy.spin_once(self.node, timeout_sec=0.01)

    def _feed(self, scan: LaserScan, seconds: float) -> None:
        # Scans every 100 ms, the LiDAR's rate, so most timer ticks fall between scans.
        end = time.time() + seconds
        next_scan = 0.0
        while time.time() < end:
            if time.time() >= next_scan:
                self.scan_pub.publish(scan)
                next_scan = time.time() + 0.1
            rclpy.spin_once(self.node, timeout_sec=0.005)

    def test_a_latency_is_the_processing_time_not_a_timer_tick(self):
        self._feed(_make_scan(-0.3, 0.3), 4.0)
        self.assertGreater(len(self.status), 5)
        s = self.status[-1]
        self.assertTrue(s["event_driven"])
        lat = s["latency_ms"]
        self.assertGreater(lat["n"], 25)
        # Timer mode would give a median near half a period (25 ms) and a p99 near a period.
        self.assertLess(lat["p50"], 0.3 * 1000.0 * _PERIOD_S, lat)
        self.assertLessEqual(lat["p50"], lat["p99"])

    def test_b_the_timer_still_feeds_the_watchdog_and_extras_are_one_per_scan(self):
        self._feed(_make_scan(-0.3, 0.3), 3.0)
        stamps = [t for t, _ in self.received[5:]]
        gaps = [b - a for a, b in itertools.pairwise(stamps)]
        self.assertGreater(len(gaps), 20)
        self.assertLess(max(gaps), 2.0 * _PERIOD_S, "a gap long enough to trip the watchdog")
        span = stamps[-1] - stamps[0]
        # The timer's 20 per second plus one per 100 ms scan (30 per second), with slack.
        self.assertLess(len(stamps) / span, 20.0 + 10.0 + 6.0)
        self.assertGreater(len(stamps) / span, 15.0)
        for _, m in self.received:
            self.assertTrue(math.isfinite(m.drive.speed) and math.isfinite(m.drive.steering_angle))
            self.assertGreaterEqual(m.drive.speed, 0.0)
            self.assertLessEqual(m.drive.speed, _MAX_SPEED_MPS + 1e-3)
            self.assertLessEqual(abs(m.drive.steering_angle), _MAX_STEERING_RAD)
        self.assertEqual(len(self.drive_received), 0, "gap_follow_node published /drive")

    def test_c_a_stale_scan_still_stops_the_output(self):
        self._feed(_make_scan(-0.3, 0.3), 1.5)
        self.assertGreater(len(self.received), 0)
        self._spin_for(_SCAN_TIMEOUT_S * 2.0)
        self.received.clear()
        self._spin_for(_SCAN_TIMEOUT_S * 4.0)
        self.assertEqual(len(self.received), 0, "kept publishing /drive_raw after /scan silence")


@launch_testing.post_shutdown_test()
class TestGapFollowNodeEventDrivenShutdown(unittest.TestCase):
    def test_clean_exit(self, proc_info):
        launch_testing.asserts.assertExitCodes(proc_info, allowable_exit_codes=[0, -signal.SIGINT])
