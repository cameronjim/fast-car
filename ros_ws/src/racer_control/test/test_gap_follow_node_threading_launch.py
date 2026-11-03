"""L3 test of gap_follow_node's threading (2026-11-02, the 2026-10-25 stutter).

On the 2026-10-25 floor bag gap_follow_node's /drive_raw had 206 gaps of 63 to 69 ms, each just
over safety_node's 60 ms watchdog, and every one contained a scan arrival: the scan callback
(the perception half, then 20 to 50 ms on the Jetson) ran on the same single thread as the 50 Hz
command timer and held it up. The node now runs the two in separate callback groups on a
two-thread executor (gap_follow_node.cpp THREADING).

The node is launched at the real 50 Hz with the TEST-ONLY scan_processing_delay_s set to 50 ms,
so every scan callback takes at least that long, and fed scans at the RPLIDAR's 10 Hz. Checks:
  * no /drive_raw gap reaches the 60 ms watchdog (3 missed cycles) over 4 s, and the median gap
    stays at the 20 ms control period: the slow scans do not touch the timer;
  * the commands still follow the scans (a left gap steers left).
"""

from __future__ import annotations

import os

# Own domain per launch test file, see test_tracker_node_launch.py.
os.environ.setdefault("ROS_DOMAIN_ID", "89")

import itertools
import signal
import statistics
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
from rclpy.qos import HistoryPolicy, QoSProfile, ReliabilityPolicy
from sensor_msgs.msg import LaserScan

_CONTROL_RATE_HZ = 50.0
_SCAN_DELAY_S = 0.05
_WATCHDOG_S = 3.0 / _CONTROL_RATE_HZ  # safety_node's default: 3 missed cycles
_FOV_RAD = 4.7
_NUM_RAYS = 1080


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
    node = LaunchNode(
        package="racer_control",
        executable="gap_follow_node",
        name="gap_follow_node",
        output="screen",
        parameters=[
            {
                "control_rate_hz": _CONTROL_RATE_HZ,
                "scan_processing_delay_s": _SCAN_DELAY_S,
                # Synthetic, vehicle-aligned scans (yaw 0), like the other synthetic-scan tests.
                "laser_yaw_from_vehicle_params": False,
                "max_speed_mps": 1.0,
            }
        ],
    )
    return launch.LaunchDescription([node, launch_testing.actions.ReadyToTest()])


def _left_gap_scan() -> LaserScan:
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
        ranges.append(5.0 if 0.3 <= bearing <= 1.2 else 1.0)
    msg.ranges = ranges
    return msg


class TestGapFollowNodeThreading(unittest.TestCase):
    @classmethod
    def setUpClass(cls):
        rclpy.init()

    @classmethod
    def tearDownClass(cls):
        rclpy.shutdown()

    def test_slow_scans_never_hold_up_the_command_timer(self):
        node = rclpy.create_node("test_gap_follow_threading_client")
        try:
            arrivals: list[float] = []
            messages: list[AckermannDriveStamped] = []

            def on_drive_raw(msg: AckermannDriveStamped) -> None:
                arrivals.append(time.monotonic())
                messages.append(msg)

            node.create_subscription(
                AckermannDriveStamped, "/drive_raw", on_drive_raw, _reliable_qos()
            )
            scan_pub = node.create_publisher(LaserScan, "/scan", _best_effort_qos())
            scan = _left_gap_scan()
            # Warm up: discovery and the first commands.
            end = time.monotonic() + 3.0
            next_scan = 0.0
            while time.monotonic() < end:
                if time.monotonic() >= next_scan:
                    scan_pub.publish(scan)
                    next_scan = time.monotonic() + 0.1
                rclpy.spin_once(node, timeout_sec=0.005)
            self.assertGreater(len(arrivals), 10, "gap_follow_node published no /drive_raw")
            arrivals.clear()
            messages.clear()
            # The measured window: scans at 10 Hz, each callback taking >= 50 ms in the node.
            end = time.monotonic() + 4.0
            while time.monotonic() < end:
                if time.monotonic() >= next_scan:
                    scan_pub.publish(scan)
                    next_scan = time.monotonic() + 0.1
                rclpy.spin_once(node, timeout_sec=0.005)
            gaps = [b - a for a, b in itertools.pairwise(arrivals)]
            self.assertGreater(len(gaps), 150, f"only {len(gaps) + 1} /drive_raw in 4 s")
            self.assertLess(
                max(gaps),
                _WATCHDOG_S,
                f"a /drive_raw gap of {max(gaps) * 1000:.1f} ms reached safety_node's watchdog "
                "while the scan callback was slow: the timer is not independent of perception",
            )
            self.assertLess(statistics.median(gaps), 0.03)
            self.assertGreater(messages[-1].drive.steering_angle, 0.05, "did not steer left")
        finally:
            node.destroy_node()


@launch_testing.post_shutdown_test()
class TestGapFollowNodeThreadingShutdown(unittest.TestCase):
    def test_clean_exit(self, proc_info):
        launch_testing.asserts.assertExitCodes(
            proc_info, allowable_exit_codes=[0, -signal.SIGINT, -signal.SIGTERM]
        )
