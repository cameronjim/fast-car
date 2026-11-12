"""L3 node tests for gap_follow_node's race features, timer mode (2026-11-12).

Launched through the real launch/gap_follow.launch.py with `physics_speed:=true` and the new
launch arguments, so the launch file's forwarding of them is exercised too. Covers:

  * physics speed, through the node: an open scan drives to the top speed, a wall 0.7 m ahead
    holds the speed at or below the speed that stops inside the free path (computed here from the
    committed vehicle_params, the same numbers the node reads), and never below the floor;
  * stale scan (item 4 of the race work; test_gap_follow_node_launch.py has the basic silence
    test): /drive_raw stops within scan_timeout_s plus two control periods of the last scan, and
    when scans resume the speed restarts from rest, not from where it was;
  * /gap_follow/status: a JSON String with the scan count, the scan-to-command latency
    percentiles (in timer mode a scan waits for the next 50 ms tick, so p99 is below one period
    plus slack) and the speed-law terms;
  * /drive is never published; the speed stays inside [0, max_speed_mps].
"""

from __future__ import annotations

import os

os.environ.setdefault("ROS_DOMAIN_ID", "100")

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
_SCAN_TIMEOUT_S = 0.2  # the params file
_CONTROL_PERIOD_S = 1.0 / 20.0  # the params file's 20 Hz
_MAX_SPEED_MPS = 1.0
_MIN_SPEED_MPS = 0.36
_FOV_RAD = 4.7
_NUM_RAYS = 1080

with open(
    Path(__file__).resolve().parents[4] / "config" / "vehicle_params.yaml", encoding="utf-8"
) as _handle:
    _VEHICLE = yaml.safe_load(_handle)


def _reliable_qos() -> QoSProfile:
    return QoSProfile(
        reliability=ReliabilityPolicy.RELIABLE, history=HistoryPolicy.KEEP_LAST, depth=50
    )


def _best_effort_qos() -> QoSProfile:
    return QoSProfile(
        reliability=ReliabilityPolicy.BEST_EFFORT, history=HistoryPolicy.KEEP_LAST, depth=10
    )


def _stopping_speed(free_path_m: float) -> float:
    """The speed that stops inside the free path (race_speed.hpp), from the committed file."""
    a = _VEHICLE["actuation"]["stop_deceleration_mps2"]
    t = _VEHICLE["actuation"]["stop_latency_s"]
    standoff = _VEHICLE["limits"]["min_forward_clearance_m"] + 0.05  # + stopping_margin_m default
    room = free_path_m - standoff
    return 0.0 if room <= 0 else a * (math.sqrt(t * t + 2.0 * room / a) - t)


def _free_path_to_a_wall_all_round(wall_m: float) -> float:
    """Rear-axle travel before the body (inflated by the 0.05 m stopping margin) reaches a wall
    that is `wall_m` from the LiDAR head in every direction: the nearest return inside the body's
    width is the one at the angle where the lateral offset equals the inflated half width."""
    half = _VEHICLE["chassis"]["width_m"] / 2.0 + 0.05
    mount_x = _VEHICLE["sensors"]["lidar"]["mount_x_m"]
    front = _VEHICLE["chassis"]["wheelbase_m"] + _VEHICLE["chassis"]["front_overhang_m"] + 0.05
    x_min = mount_x + wall_m * math.cos(math.asin(half / wall_m))
    return x_min - front


@pytest.mark.launch_test
def generate_test_description():
    launch_file = Path(get_package_share_directory("racer_control")) / "launch"
    gap_follow = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(str(launch_file / "gap_follow.launch.py")),
        launch_arguments={
            "max_speed_mps": str(_MAX_SPEED_MPS),
            "params_file": _PARAMS_FILE,
            "physics_speed": "true",
            "a_lat_max_mps2": "2",
            "stopping_margin_m": "0.05",
            "side_cap_distance_m": "0",
            "apex_offset_m": "0",
            "track_out_offset_m": "0",
            "status_rate_hz": "5",
        }.items(),
    )
    return launch.LaunchDescription([gap_follow, launch_testing.actions.ReadyToTest()])


def _make_scan(fill: float, gap_lo_rad: float | None = None, gap_hi_rad: float | None = None):
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
        free = gap_lo_rad is not None and gap_lo_rad <= bearing <= gap_hi_rad
        ranges.append(5.0 if free else fill)
    msg.ranges = ranges
    return msg


class TestGapFollowNodeRace(unittest.TestCase):
    @classmethod
    def setUpClass(cls):
        rclpy.init()

    @classmethod
    def tearDownClass(cls):
        rclpy.shutdown()

    def setUp(self):
        self.node = rclpy.create_node("test_gap_follow_node_race_client")
        self.received: list[tuple[float, AckermannDriveStamped]] = []
        self.status: list[dict] = []
        self.drive_received: list[AckermannDriveStamped] = []
        self.node.create_subscription(
            AckermannDriveStamped,
            "/drive_raw",
            lambda m: self.received.append((time.monotonic(), m)),
            _reliable_qos(),
        )
        self.node.create_subscription(
            AckermannDriveStamped, "/drive", self.drive_received.append, _best_effort_qos()
        )
        self.node.create_subscription(
            String,
            "/gap_follow/status",
            lambda m: self.status.append(json.loads(m.data)),
            _reliable_qos(),
        )
        self.scan_pub = self.node.create_publisher(LaserScan, "/scan", _best_effort_qos())
        self._spin_for(0.5)

    def tearDown(self):
        self.node.destroy_node()

    def _spin_for(self, seconds: float) -> None:
        end = time.time() + seconds
        while time.time() < end:
            rclpy.spin_once(self.node, timeout_sec=0.02)

    def _feed(self, scan: LaserScan, seconds: float) -> None:
        end = time.time() + seconds
        while time.time() < end:
            self.scan_pub.publish(scan)
            rclpy.spin_once(self.node, timeout_sec=0.05)

    def test_a_open_scan_reaches_the_top_speed_within_bounds(self):
        self._feed(_make_scan(5.0, -0.3, 0.3), 2.5)
        self.assertGreater(len(self.received), 5)
        speeds = [m.drive.speed for _, m in self.received]
        self.assertGreater(speeds[-1], 0.95 * _MAX_SPEED_MPS, speeds[-5:])
        for s in speeds:
            self.assertTrue(math.isfinite(s))
            self.assertGreaterEqual(s, 0.0)
            self.assertLessEqual(s, _MAX_SPEED_MPS + 1e-3)
        self.assertEqual(len(self.drive_received), 0, "gap_follow_node published /drive")

    def test_b_a_wall_ahead_holds_the_speed_at_the_stopping_limit_and_above_the_floor(self):
        wall_m = 0.7
        self._feed(_make_scan(wall_m), 2.5)
        speeds = [m.drive.speed for _, m in self.received[-10:]]
        self.assertGreater(len(speeds), 5)
        bound = max(_MIN_SPEED_MPS, _stopping_speed(_free_path_to_a_wall_all_round(wall_m)))
        self.assertLess(bound, 0.9 * _MAX_SPEED_MPS, "the wall must be the binding limit")
        for s in speeds:
            self.assertLessEqual(s, bound + 0.03, (s, bound))
            self.assertGreaterEqual(s, _MIN_SPEED_MPS - 0.02)
        terms = self.status[-1]["speed_terms_mps"]
        self.assertAlmostEqual(
            terms["stopping"], _stopping_speed(_free_path_to_a_wall_all_round(wall_m)), delta=0.03
        )

    def test_c_stale_scan_stops_output_and_restarts_from_rest(self):
        self._feed(_make_scan(5.0, -0.3, 0.3), 2.0)
        self.assertGreater(self.received[-1][1].drive.speed, 0.8)
        last_scan = time.monotonic()
        self.received.clear()
        self._spin_for(_SCAN_TIMEOUT_S + 6 * _CONTROL_PERIOD_S)
        late = [
            t
            for t, _ in self.received
            if t - last_scan > _SCAN_TIMEOUT_S + 2 * _CONTROL_PERIOD_S + 0.05
        ]
        self.assertEqual(late, [], "kept publishing /drive_raw after the scan timeout")
        self.received.clear()
        self._spin_for(_SCAN_TIMEOUT_S * 3)
        self.assertEqual(len(self.received), 0, "published during the silence")
        self._feed(_make_scan(5.0, -0.3, 0.3), 1.0)
        self.assertGreater(len(self.received), 3)
        first = self.received[0][1].drive.speed
        self.assertLess(first, 0.3, "the speed resumed from where it was, not from rest")
        self.assertGreater(self.received[-1][1].drive.speed, 0.8)

    def test_d_status_reports_latency_cpu_and_the_speed_terms(self):
        self._feed(_make_scan(5.0, -0.3, 0.3), 3.0)
        self.assertGreater(len(self.status), 3)
        s = self.status[-1]
        self.assertFalse(s["event_driven"])
        self.assertTrue(s["physics_speed"])
        self.assertGreater(s["scans"], 20)
        lat = s["latency_ms"]
        self.assertGreater(lat["n"], 20)
        self.assertLessEqual(lat["p50"], lat["p99"])
        self.assertLessEqual(lat["p99"], lat["max"] + 1e-9)
        # A scan waits for the next tick: at most one period (50 ms here) plus processing.
        self.assertLess(lat["p99"], 1000.0 * _CONTROL_PERIOD_S + 40.0)
        self.assertGreaterEqual(lat["p50"], 0.0)
        for key in ("scan_wall_us", "scan_cpu_us"):
            self.assertGreater(s[key]["n"], 20)
            self.assertGreater(s[key]["p50"], 0.0)
            self.assertLess(s[key]["p50"], 20_000.0)  # a scan costs far less than a 20 ms tick
        terms = s["speed_terms_mps"]
        self.assertAlmostEqual(terms["target"], _MAX_SPEED_MPS, delta=0.01)


@launch_testing.post_shutdown_test()
class TestGapFollowNodeRaceShutdown(unittest.TestCase):
    def test_clean_exit(self, proc_info):
        launch_testing.asserts.assertExitCodes(proc_info, allowable_exit_codes=[0, -signal.SIGINT])
