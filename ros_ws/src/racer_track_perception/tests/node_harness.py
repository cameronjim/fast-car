"""Shared plumbing of the L3 node tests (they need rclpy and run under colcon only)."""

from __future__ import annotations

import json
import time

import rclpy
from builtin_interfaces.msg import Time
from racer_pnp_msgs.msg import LocalTrack
from racer_track_perception.vehicle import load_live_vehicle
from rclpy.node import Node
from rclpy.qos import HistoryPolicy, QoSProfile, ReliabilityPolicy
from scan_fixture import SCAN, corridor_ranges
from sensor_msgs.msg import LaserScan
from std_msgs.msg import String

BASE_NS = 1_700_000_000_000_000_000
DT_NS = 100_000_000


class Harness:
    """A test node that publishes /scan and records /pnp/local_track and the status."""

    def __init__(self) -> None:
        self.node: Node = rclpy.create_node("track_perception_test_client")
        self.tracks: list[LocalTrack] = []
        self.statuses: list[dict] = []
        self.published_ns: list[int] = []
        self._ranges = [float(r) for r in corridor_ranges(load_live_vehicle())]
        reliable = QoSProfile(
            reliability=ReliabilityPolicy.RELIABLE, history=HistoryPolicy.KEEP_LAST, depth=50
        )
        best_effort = QoSProfile(
            reliability=ReliabilityPolicy.BEST_EFFORT, history=HistoryPolicy.KEEP_LAST, depth=10
        )
        self.node.create_subscription(LocalTrack, "/pnp/local_track", self.tracks.append, reliable)
        self.node.create_subscription(
            String,
            "/pnp/perception/status",
            lambda m: self.statuses.append(json.loads(m.data)),
            reliable,
        )
        self._scan_pub = self.node.create_publisher(LaserScan, "/scan", best_effort)

    def spin(self, seconds: float) -> None:
        end = time.time() + seconds
        while time.time() < end:
            rclpy.spin_once(self.node, timeout_sec=0.02)

    def publish_scan(self) -> None:
        ns = BASE_NS + len(self.published_ns) * DT_NS
        msg = LaserScan()
        msg.header.stamp = Time(sec=ns // 1_000_000_000, nanosec=ns % 1_000_000_000)
        msg.header.frame_id = "laser"
        msg.angle_min = SCAN["angle_min"]
        msg.angle_max = SCAN["angle_min"] + 719 * SCAN["angle_increment"]
        msg.angle_increment = SCAN["angle_increment"]
        msg.range_min = SCAN["range_min"]
        msg.range_max = SCAN["range_max"]
        msg.ranges = self._ranges
        self.published_ns.append(ns)
        self._scan_pub.publish(msg)

    def run_scans(self, at_least: int, timeout_s: float = 15.0) -> None:
        """Publish scans at 10 Hz until `at_least` tracks came back (discovery can lose the
        first few best-effort scans) or the timeout."""
        end = time.time() + timeout_s
        while len(self.tracks) < at_least and time.time() < end:
            self.publish_scan()
            self.spin(0.1)
        self.spin(0.3)

    def status_while_scanning(self, timeout_s: float = 15.0) -> dict:
        """Keep publishing scans until a status that reports running arrives; return it."""
        end = time.time() + timeout_s
        seen = len(self.statuses)
        while time.time() < end:
            self.publish_scan()
            self.spin(0.1)
            for status in self.statuses[seen:]:
                if status["state"] == "running":
                    return status
        raise AssertionError("no running status within the timeout")

    def destroy(self) -> None:
        self.node.destroy_node()


def stamp_ns(msg: LocalTrack) -> int:
    return msg.header.stamp.sec * 1_000_000_000 + msg.header.stamp.nanosec
