"""Shared harness for park_node's L3 launch tests (test_park_node_launch.py and
test_park_node_fallback_launch.py). Not a test file: each test loads it by path (launch_testing
runs a test file by path, so this directory is not on sys.path).

The harness plays the rest of the graph: it publishes a synthetic, vehicle-aligned (yaw 0) 360
degree /scan at 10 Hz, optionally /odom/wheel at 50 Hz integrating the speed the "car" was
given, and stands in for safety_node by publishing the gated /drive at 50 Hz (passing the latest
/drive_raw request, or holding it at zero).
"""

from __future__ import annotations

import math
import time

import rclpy
from ackermann_msgs.msg import AckermannDriveStamped
from nav_msgs.msg import Odometry
from rclpy.qos import HistoryPolicy, QoSProfile, ReliabilityPolicy
from sensor_msgs.msg import LaserScan
from std_msgs.msg import String
from std_srvs.srv import Trigger

NUM_RAYS = 720

# A continuous row 0.5 m to the right and a wall 1.0 m to the left: no pocket anywhere.
ROW_SCAN_WALLS = [(-6.0, -0.5, 6.0, -0.5), (-6.0, 1.0, 6.0, 1.0)]
# A 1.2 m lane: too narrow for the three-point turn (the planner needs about 1.6 m).
NARROW_LANE_WALLS = [(-6.0, -0.6, 6.0, -0.6), (-6.0, 0.6, 6.0, 0.6)]


def reliable_qos() -> QoSProfile:
    return QoSProfile(
        reliability=ReliabilityPolicy.RELIABLE, history=HistoryPolicy.KEEP_LAST, depth=10
    )


def best_effort_qos() -> QoSProfile:
    return QoSProfile(
        reliability=ReliabilityPolicy.BEST_EFFORT, history=HistoryPolicy.KEEP_LAST, depth=10
    )


def scan_of(walls: list[tuple[float, float, float, float]]) -> LaserScan:
    """A vehicle-aligned 360 degree scan, from the LiDAR head, of segment walls given in the
    head's frame as (x0, y0, x1, y1)."""
    msg = LaserScan()
    msg.header.frame_id = "laser"
    msg.angle_min = -math.pi
    msg.angle_increment = 2.0 * math.pi / NUM_RAYS
    msg.angle_max = msg.angle_min + (NUM_RAYS - 1) * msg.angle_increment
    msg.range_min = 0.05
    msg.range_max = 12.0
    ranges = []
    for i in range(NUM_RAYS):
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
        ranges.append(best if best <= msg.range_max else math.inf)
    msg.ranges = ranges
    return msg


class Harness:
    def __init__(self, node, odometry: bool):
        self.node = node
        self.odometry = odometry
        self.requests: list[AckermannDriveStamped] = []
        self.statuses: list[str] = []
        node.create_subscription(
            AckermannDriveStamped, "/drive_raw", self.requests.append, reliable_qos()
        )
        node.create_subscription(
            String, "/park_node/status", lambda m: self.statuses.append(m.data), reliable_qos()
        )
        self.scan_pub = node.create_publisher(LaserScan, "/scan", best_effort_qos())
        self.gated_pub = node.create_publisher(AckermannDriveStamped, "/drive", reliable_qos())
        self.odom_pub = node.create_publisher(Odometry, "/odom/wheel", reliable_qos())
        self.services = {
            name: node.create_client(Trigger, f"/park_node/{name}")
            for name in ("start", "three_point_turn", "abort")
        }
        self.scan = scan_of(ROW_SCAN_WALLS)
        self.distance = 0.0
        self.gated = 0.0

    def wait_for_services(self, test) -> None:
        for name, client in self.services.items():
            test.assertTrue(client.wait_for_service(timeout_sec=20.0), f"/park_node/{name}")

    def spin(self, seconds: float, gate: str = "pass", until=None) -> list:
        """Run the fake graph for `seconds`: scans at 10 Hz, odometry and the gate at 50 Hz.
        gate "pass" passes the latest request, "hold" holds it at zero. Returns (time, speed,
        steering) for every /drive_raw received; stops early once until(seen) is true."""
        seen: list[tuple[float, float, float]] = []
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
                self.gated = request if gate == "pass" else 0.0
                msg = AckermannDriveStamped()
                msg.drive.speed = self.gated
                self.gated_pub.publish(msg)
                self.distance += self.gated * 0.02
                if self.odometry:
                    odom = Odometry()
                    odom.header.frame_id = "odom"
                    odom.child_frame_id = "base_link"
                    odom.pose.pose.position.x = self.distance
                    odom.pose.pose.orientation.w = 1.0
                    odom.twist.twist.linear.x = self.gated
                    self.odom_pub.publish(odom)
                next_gate = now + 0.02
            rclpy.spin_once(self.node, timeout_sec=0.005)
            while consumed < len(self.requests):
                r = self.requests[consumed].drive
                seen.append((time.time(), r.speed, r.steering_angle))
                consumed += 1
            if until is not None and until(seen):
                break
        return seen

    def call(self, test, name: str, gate: str = "pass"):
        future = self.services[name].call_async(Trigger.Request())
        end = time.time() + 15.0
        while not future.done() and time.time() < end:
            self.spin(0.05, gate=gate)
        test.assertTrue(future.done(), f"/park_node/{name} did not answer")
        return future.result()
