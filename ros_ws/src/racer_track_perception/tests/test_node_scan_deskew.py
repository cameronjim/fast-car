"""L3: scan_deskew_node as a real node (needs rclpy and launch_testing; colcon only).

Test order matters and is by name: the silence test runs first, before any scan is published, then
the cases without odometry, the nominal case, the stale-odometry case last (it stops the odometry).
A reliable and a best effort subscriber both listen to /pnp/scan_deskewed, the two ways the
driver's consumers subscribe to a scan.
"""

from __future__ import annotations

import json
import math
import signal
import time
import unittest

import pytest

pytest.importorskip("rclpy")
pytest.importorskip("launch_testing")
pytest.importorskip("racer_tools.vehicle_params_loader")

import launch
import launch_testing
import launch_testing.actions
import launch_testing.asserts
import rclpy
from ackermann_msgs.msg import AckermannDriveStamped
from launch_ros.actions import Node as LaunchNode
from nav_msgs.msg import Odometry
from rclpy.qos import HistoryPolicy, QoSProfile, ReliabilityPolicy
from sensor_msgs.msg import LaserScan
from std_msgs.msg import String

N = 720
INC = 2.0 * math.pi / 719.0
SCAN_TIME_S = 0.0998
TIME_INCREMENT_S = SCAN_TIME_S / 719.0
EXPECTED_OFFSET_S = TIME_INCREMENT_S * (N - 1)  # the stamp is the start, the reference the end


@pytest.mark.launch_test
def generate_test_description():
    node = LaunchNode(
        package="racer_track_perception",
        executable="scan_deskew_node",
        name="scan_deskew_node",
        parameters=[{"stamp_is": "sweep_start", "beam_order": "index_descending"}],
    )
    return launch.LaunchDescription([node, launch_testing.actions.ReadyToTest()])


class TestScanDeskewNode(unittest.TestCase):
    @classmethod
    def setUpClass(cls):
        rclpy.init()
        cls.node = rclpy.create_node("scan_deskew_test_client")
        cls.reliable_out: list[LaserScan] = []
        cls.best_effort_out: list[LaserScan] = []
        cls.statuses: list[dict] = []
        cls.sent: list[LaserScan] = []
        reliable = QoSProfile(
            reliability=ReliabilityPolicy.RELIABLE, history=HistoryPolicy.KEEP_LAST, depth=50
        )
        best_effort = QoSProfile(
            reliability=ReliabilityPolicy.BEST_EFFORT, history=HistoryPolicy.KEEP_LAST, depth=50
        )
        cls.node.create_subscription(
            LaserScan, "/pnp/scan_deskewed", cls.reliable_out.append, reliable
        )
        cls.node.create_subscription(
            LaserScan, "/pnp/scan_deskewed", cls.best_effort_out.append, best_effort
        )
        cls.node.create_subscription(
            String,
            "/pnp/deskew/status",
            lambda m: cls.statuses.append(json.loads(m.data)),
            reliable,
        )
        cls.scan_pub = cls.node.create_publisher(LaserScan, "/scan", best_effort)
        cls.odom_pub = cls.node.create_publisher(Odometry, "/odom/wheel", best_effort)
        cls.drive_pub = cls.node.create_publisher(AckermannDriveStamped, "/drive", best_effort)
        cls.odometry_on = False

    @classmethod
    def tearDownClass(cls):
        cls.node.destroy_node()
        rclpy.shutdown()

    # -- helpers --------------------------------------------------------------------------------

    @classmethod
    def now_msg(cls):
        return cls.node.get_clock().now().to_msg()

    @classmethod
    def spin(cls, seconds: float) -> None:
        end = time.time() + seconds
        next_odom = 0.0
        while time.time() < end:
            rclpy.spin_once(cls.node, timeout_sec=0.01)
            if cls.odometry_on and time.time() >= next_odom:
                next_odom = time.time() + 0.02
                cls.publish_odometry(1.0, 0.2)

    @classmethod
    def publish_odometry(cls, v: float, delta: float) -> None:
        odom = Odometry()
        odom.header.stamp = cls.now_msg()
        odom.twist.twist.linear.x = v
        cls.odom_pub.publish(odom)
        drive = AckermannDriveStamped()
        drive.header.stamp = odom.header.stamp
        drive.drive.steering_angle = delta
        drive.drive.speed = v
        cls.drive_pub.publish(drive)

    @classmethod
    def publish_scan(cls) -> LaserScan:
        msg = LaserScan()
        now_ns = cls.node.get_clock().now().nanoseconds
        start_ns = now_ns - int(SCAN_TIME_S * 1e9)
        msg.header.stamp.sec = start_ns // 1_000_000_000
        msg.header.stamp.nanosec = start_ns % 1_000_000_000
        msg.header.frame_id = "laser"
        msg.angle_min = -math.pi
        msg.angle_max = math.pi
        msg.angle_increment = INC
        msg.scan_time = SCAN_TIME_S
        msg.time_increment = TIME_INCREMENT_S
        msg.range_min = 0.05
        msg.range_max = 16.0
        msg.ranges = [2.0 + 0.001 * (i % 50) for i in range(N)]
        msg.intensities = [47.0] * N
        cls.sent.append(msg)
        cls.scan_pub.publish(msg)
        return msg

    @classmethod
    def run_scans(cls, n: int, timeout_s: float = 15.0) -> None:
        """Publish scans at 10 Hz until `n` more came out on both subscribers (discovery can lose
        the first few best effort scans) or the timeout."""
        want = len(cls.reliable_out) + n
        end = time.time() + timeout_s
        while (len(cls.reliable_out) < want or len(cls.best_effort_out) < want) and (
            time.time() < end
        ):
            cls.publish_scan()
            cls.spin(0.1)
        cls.spin(0.3)

    @staticmethod
    def stamp_ns(msg: LaserScan) -> int:
        return msg.header.stamp.sec * 1_000_000_000 + msg.header.stamp.nanosec

    def last_status(self) -> dict:
        seen = len(self.statuses)
        end = time.time() + 5.0
        while len(self.statuses) == seen and time.time() < end:
            self.spin(0.1)
        return self.statuses[-1]

    # -- the cases -----------------------------------------------------------------------------------

    def test_a_silence_gives_no_output_and_a_status_that_says_so(self):
        self.spin(3.0)
        self.assertEqual(len(self.reliable_out), 0)
        status = self.statuses[-1]
        self.assertEqual(status["state"], "no_scan")
        self.assertEqual(status["scans_in"], 0)

    def test_b_without_odometry_the_scan_passes_through_unchanged(self):
        self.run_scans(3)
        self.assertGreaterEqual(len(self.reliable_out), 3)
        for out in self.reliable_out:
            sent = next(s for s in self.sent if self.stamp_ns(s) == self.stamp_ns(out))
            self.assertAlmostEqual(out.scan_time, sent.scan_time, places=6)
            self.assertAlmostEqual(out.time_increment, sent.time_increment, places=9)
            self.assertEqual(list(out.ranges), list(sent.ranges))
            self.assertEqual(len(out.intensities), N, "an unchanged scan keeps its intensities")
        status = self.last_status()
        self.assertGreaterEqual(status["passed_through_by_reason"]["no_odom"], 3)
        self.assertEqual(status["scans_deskewed"], 0)
        self.assertEqual(status["scans_in"], status["scans_out"])

    def test_c_nominal_one_out_per_in_with_the_offset_and_zeroed_timing(self):
        type(self).odometry_on = True
        self.spin(1.0)
        before_in = len(self.sent)
        before_out = len(self.reliable_out)
        self.run_scans(5)
        sent = self.sent[before_in:]
        deskewed = self.reliable_out[before_out:]
        self.assertGreaterEqual(len(deskewed), 5)
        by_stamp = {self.stamp_ns(s): s for s in sent}
        for out in deskewed:
            src_ns = self.stamp_ns(out) - round(EXPECTED_OFFSET_S * 1e9)
            self.assertIn(src_ns, by_stamp, "output stamp = input stamp + offset")
            src = by_stamp[src_ns]
            self.assertEqual(out.header.frame_id, src.header.frame_id)
            self.assertAlmostEqual(out.angle_min, src.angle_min, places=6)
            self.assertAlmostEqual(out.angle_max, src.angle_max, places=6)
            self.assertAlmostEqual(out.angle_increment, src.angle_increment, places=6)
            self.assertAlmostEqual(out.range_min, src.range_min, places=6)
            self.assertAlmostEqual(out.range_max, src.range_max, places=6)
            self.assertEqual(len(out.ranges), N)
            self.assertEqual(out.scan_time, 0.0)
            self.assertEqual(out.time_increment, 0.0)
            self.assertEqual(len(out.intensities), 0)
            self.assertTrue(all(r >= 0.05 or math.isinf(r) for r in out.ranges))
        # the deskewed ranges are not the input (the car moves at 1 m/s and turns)
        self.assertNotEqual(
            list(deskewed[-1].ranges),
            list(by_stamp[self.stamp_ns(deskewed[-1]) - round(EXPECTED_OFFSET_S * 1e9)].ranges),
        )
        status = self.last_status()
        self.assertGreaterEqual(status["scans_deskewed"], 5)
        self.assertEqual(status["scans_in"], status["scans_out"])
        self.assertIsNotNone(status["process_ms"]["p99"])
        self.assertIsNotNone(status["twist_age_s"])

    def test_d_a_reliable_and_a_best_effort_subscriber_both_get_every_scan(self):
        before = (len(self.reliable_out), len(self.best_effort_out))
        self.run_scans(4)
        got_reliable = len(self.reliable_out) - before[0]
        got_best_effort = len(self.best_effort_out) - before[1]
        self.assertGreaterEqual(got_reliable, 4)
        self.assertEqual(got_reliable, got_best_effort)

    def test_e_the_node_never_publishes_the_command_topics_or_scan(self):
        node = self.node
        for topic in ("/drive_raw", "/scan"):
            names = [i.node_name for i in node.get_publishers_info_by_topic(topic)]
            self.assertNotIn("scan_deskew_node", names)
        drive = [i.node_name for i in node.get_publishers_info_by_topic("/drive")]
        self.assertNotIn("scan_deskew_node", drive)
        topics = {
            name
            for name, _types in node.get_publisher_names_and_types_by_node("scan_deskew_node", "/")
        }
        self.assertEqual(
            topics - {"/rosout", "/parameter_events"},
            {"/pnp/scan_deskewed", "/pnp/deskew/status"},
        )

    def test_z_silence_on_the_wheel_odometry_passes_the_scans_through(self):
        type(self).odometry_on = False
        self.spin(0.5)  # let the freshness (0.10 s) run out
        before_in = len(self.sent)
        before_out = len(self.reliable_out)
        self.run_scans(3)
        sent = self.sent[before_in:]
        passed = self.reliable_out[before_out:]
        self.assertGreaterEqual(len(passed), 3)
        by_stamp = {self.stamp_ns(s): s for s in sent}
        for out in passed:
            self.assertIn(self.stamp_ns(out), by_stamp, "the stamp is the original one")
            self.assertAlmostEqual(out.scan_time, SCAN_TIME_S, places=6)  # float32 field
        status = self.last_status()
        self.assertGreaterEqual(status["passed_through_by_reason"]["twist_stale"], 3)
        self.assertEqual(status["scans_in"], status["scans_out"])


@launch_testing.post_shutdown_test()
class TestScanDeskewNodeShutdown(unittest.TestCase):
    def test_clean_shutdown(self, proc_info):
        launch_testing.asserts.assertExitCodes(proc_info, allowable_exit_codes=[0, -signal.SIGINT])
