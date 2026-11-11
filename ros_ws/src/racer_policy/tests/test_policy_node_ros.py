"""policy_node on a live ROS graph (L3): the real node process, a test harness playing the
tracker, odometry and LiDAR, and assertions on /drive_raw, /policy/status and /policy/events.

Covers: a zero residual passes the tracker command through unchanged; a hostile network (outputs
saturated at +-1) never moves the command more than the envelope bound; a stale scan falls
back to the plain tracker command and the fallback is announced once on engage and once on
release; stale odometry stops /drive_raw (the safety watchdog's cue); policy_node is the only
publisher of /drive_raw and never publishes /drive; the status record carries the jitter
histograms.

Needs rclpy, torch (the fixture network is TorchScript) and the colcon-built racer_tools;
skipped otherwise, so it runs in the car image and wherever torch is installed.
"""

from __future__ import annotations

import json
import math
import os
import signal
import subprocess
import sys
import time
from pathlib import Path
from typing import Any

import pytest
import rclpy
from ackermann_msgs.msg import AckermannDriveStamped
from geometry_msgs.msg import TransformStamped
from nav_msgs.msg import Odometry
from policy_fixture import build_policy_dir, torch_policy_bytes
from rclpy.qos import HistoryPolicy, QoSProfile, ReliabilityPolicy
from sensor_msgs.msg import LaserScan
from std_msgs.msg import String
from tf2_msgs.msg import TFMessage

os.environ.setdefault("ROS_DOMAIN_ID", "97")

STEERING_BOUND_RAD = 0.025 * 0.898
SPEED_BOUND_MPS = 0.004 * 25.0


def _circle_csv(path: Path, radius: float = 2.0, n: int = 200) -> Path:
    rows = ["s_m,x_m,y_m,heading_rad,curvature_1pm,target_speed_mps"]
    for i in range(n):
        t = 2.0 * math.pi * i / n
        rows.append(
            f"{radius * t},{radius * math.cos(t)},{radius * math.sin(t)},"
            f"{t + math.pi / 2},{1.0 / radius},1.0"
        )
    path.write_text("\n".join(rows) + "\n", encoding="utf-8")
    return path


class Harness:
    def __init__(
        self,
        tmp_path: Path,
        scale: float,
        extra_params: tuple[str, ...] = (),
        tf: tuple[float, float, float] | None = None,
    ) -> None:
        self.tf = tf
        from racer_tools.vehicle_params_loader import load_vehicle_params

        vp = load_vehicle_params()
        self.policy_dir = build_policy_dir(
            tmp_path, vp, policy_bytes=torch_policy_bytes(scale=scale), name=f"p{scale}"
        )
        self.raceline = _circle_csv(tmp_path / "circle.csv")
        self.proc = subprocess.Popen(
            [
                sys.executable,
                "-m",
                "racer_policy.policy_node",
                "--ros-args",
                "-p",
                f"policy_dir:={self.policy_dir}",
                "-p",
                f"raceline_path:={self.raceline}",
                "-p",
                "scan_timeout_s:=0.3",
                "-p",
                "odom_timeout_s:=0.3",
                "-p",
                "status_period_s:=0.2",
                *[x for pair in (("-p", e) for e in extra_params) for x in pair],
            ],
            env=dict(os.environ),
            stdout=subprocess.PIPE,
            stderr=subprocess.STDOUT,
            text=True,
        )
        rclpy.init()
        self.node = rclpy.create_node("policy_test_harness")
        reliable = QoSProfile(
            reliability=ReliabilityPolicy.RELIABLE, history=HistoryPolicy.KEEP_LAST, depth=50
        )
        sensor = QoSProfile(
            reliability=ReliabilityPolicy.BEST_EFFORT, history=HistoryPolicy.KEEP_LAST, depth=5
        )
        self.tf_pub = self.node.create_publisher(TFMessage, "/tf", reliable)
        self.odom_pub = self.node.create_publisher(Odometry, "/odom", reliable)
        self.scan_pub = self.node.create_publisher(LaserScan, "/scan", sensor)
        self.cmd_pub = self.node.create_publisher(
            AckermannDriveStamped, "/tracker/command", reliable
        )
        self.drive_raw: list[tuple[float, float]] = []
        self.statuses: list[dict[str, Any]] = []
        self.events: list[dict[str, Any]] = []
        self.node.create_subscription(
            AckermannDriveStamped,
            "/drive_raw",
            lambda m: self.drive_raw.append((m.drive.steering_angle, m.drive.speed)),
            reliable,
        )
        self.node.create_subscription(
            String, "/policy/status", lambda m: self.statuses.append(json.loads(m.data)), reliable
        )
        self.node.create_subscription(
            String, "/policy/events", lambda m: self.events.append(json.loads(m.data)), reliable
        )

    def close(self) -> str:
        self.node.destroy_node()
        rclpy.shutdown()
        if self.proc.poll() is None:
            self.proc.send_signal(signal.SIGINT)
            try:
                out, _ = self.proc.communicate(timeout=15)
            except subprocess.TimeoutExpired:
                self.proc.kill()
                out, _ = self.proc.communicate()
        else:
            out, _ = self.proc.communicate()
        return str(out)

    # -- inputs ------------------------------------------------------------------------------

    def _odom(self) -> Odometry:
        m = Odometry()
        m.header.stamp = self.node.get_clock().now().to_msg()
        m.header.frame_id = "odom"
        m.pose.pose.position.x = 2.0
        m.pose.pose.position.y = 0.0
        m.pose.pose.orientation.z = math.sin(math.pi / 4.0)
        m.pose.pose.orientation.w = math.cos(math.pi / 4.0)
        m.twist.twist.linear.x = 1.0
        m.twist.twist.angular.z = 0.5
        return m

    def _tf_message(self) -> TFMessage:
        t = TransformStamped()
        t.header.stamp = self.node.get_clock().now().to_msg()
        t.header.frame_id = "map"
        t.child_frame_id = "odom"
        t.transform.translation.x, t.transform.translation.y = self.tf[0], self.tf[1]
        t.transform.rotation.z = math.sin(self.tf[2] / 2.0)
        t.transform.rotation.w = math.cos(self.tf[2] / 2.0)
        return TFMessage(transforms=[t])

    def _scan(self) -> LaserScan:
        n = 500
        m = LaserScan()
        m.header.stamp = self.node.get_clock().now().to_msg()
        m.header.frame_id = "laser"
        m.angle_min = -math.pi
        m.angle_increment = 2.0 * math.pi / n
        m.angle_max = m.angle_min + m.angle_increment * (n - 1)
        m.range_min = 0.05
        m.range_max = 12.0
        m.ranges = [3.0] * n
        return m

    def run(
        self,
        seconds: float,
        *,
        odom: bool = True,
        scan: bool = True,
        command: bool = True,
        steering: float = 0.1,
        speed: float = 1.0,
    ) -> None:
        end = time.monotonic() + seconds
        next_tick = time.monotonic()
        tick = 0
        while time.monotonic() < end:
            now = time.monotonic()
            if now >= next_tick:
                if odom:
                    self.odom_pub.publish(self._odom())
                    if self.tf is not None:
                        self.tf_pub.publish(self._tf_message())
                if scan and tick % 5 == 0:
                    self.scan_pub.publish(self._scan())
                if command:
                    c = AckermannDriveStamped()
                    c.header.stamp = self.node.get_clock().now().to_msg()
                    c.drive.steering_angle = steering
                    c.drive.speed = speed
                    self.cmd_pub.publish(c)
                tick += 1
                next_tick += 0.02
            rclpy.spin_once(self.node, timeout_sec=0.002)

    def wait_for_node(self, timeout_s: float = 40.0) -> None:
        end = time.monotonic() + timeout_s
        while time.monotonic() < end:
            if self.proc.poll() is not None:
                raise AssertionError("policy_node exited early:\n" + str(self.proc.stdout.read()))
            self.run(0.2)
            if self.drive_raw:
                return
        raise AssertionError("policy_node never published /drive_raw")


@pytest.fixture
def make_harness(tmp_path: Path):
    made: list[Harness] = []

    def make(
        scale: float, extra_params: tuple[str, ...] = (), tf=None, wait: bool = True
    ) -> Harness:
        h = Harness(tmp_path, scale, extra_params, tf)
        made.append(h)
        if wait:
            h.wait_for_node()
        return h

    yield make
    for h in made:
        h.close()


def test_zero_residual_passes_the_tracker_command_through(make_harness) -> None:
    h = make_harness(0.0)
    h.drive_raw.clear()
    h.run(1.5)
    assert len(h.drive_raw) >= 50
    for steering, speed in h.drive_raw:
        assert steering == pytest.approx(0.1, abs=1e-6)
        assert speed == pytest.approx(1.0, abs=1e-6)


def test_only_policy_node_publishes_drive_raw_and_nobody_here_publishes_drive(make_harness) -> None:
    h = make_harness(0.0)
    h.run(0.5)
    assert h.node.count_publishers("/drive_raw") == 1
    assert h.node.count_publishers("/drive") == 0


def test_a_saturated_network_stays_inside_the_envelope(make_harness) -> None:
    h = make_harness(50.0)  # large weights: tanh outputs sit at +-1 for most inputs
    h.drive_raw.clear()
    h.run(2.0)
    assert len(h.drive_raw) >= 80
    moved = False
    for steering, speed in h.drive_raw:
        assert abs(steering - 0.1) <= STEERING_BOUND_RAD + 1e-6
        assert abs(speed - 1.0) <= SPEED_BOUND_MPS + 1e-6
        moved = moved or abs(steering - 0.1) > 1e-3
    assert moved, "the residual was never applied"


def test_stale_scan_falls_back_and_is_announced_once_each_way(make_harness) -> None:
    h = make_harness(1.0)
    h.run(0.6)
    h.drive_raw.clear()
    h.events.clear()
    h.run(1.0, scan=False)  # the LiDAR goes quiet; the tracker keeps commanding
    for steering, speed in h.drive_raw[-30:]:
        assert steering == pytest.approx(0.1, abs=1e-6)  # the plain tracker command
        assert speed == pytest.approx(1.0, abs=1e-6)
    engages = [e for e in h.events if e["event"] == "fallback_engage"]
    assert len(engages) == 1 and engages[0]["reason"] == "scan_stale"
    h.run(1.0)  # the scan returns
    releases = [e for e in h.events if e["event"] == "fallback_release"]
    assert len(releases) == 1 and releases[0]["duration_s"] > 0.0
    assert len(h.events) == 2


def test_stale_odometry_stops_drive_raw(make_harness) -> None:
    h = make_harness(0.0)
    h.run(0.5, odom=False)
    h.run(0.1, odom=False)
    n = len(h.drive_raw)
    h.run(1.0, odom=False)
    # Allow the last in-flight messages; after the 0.3 s odometry timeout nothing more is sent.
    assert len(h.drive_raw) - n <= 1


def test_status_carries_counters_and_both_histograms(make_harness) -> None:
    h = make_harness(1.0)
    h.run(1.5)
    status = h.statuses[-1]
    assert status["state"] in ("active", "fallback")
    assert status["counters"]["steps"] > 50
    for key in ("inference_us", "loop_period_ms"):
        hist = status[key]
        assert len(hist["counts"]) == len(hist["edges"]) + 1
        assert hist["n"] > 0 and hist["p99"] >= hist["p50"] and hist["max"] > 0.0


def test_pose_frame_composes_the_localised_pose_like_the_tracker(make_harness) -> None:
    """With pose_frame set the node follows /odom composed with TF pose_frame -> odom. A map ->
    odom of identity keeps the car on the raceline (active); the same odometry with the frame 5 m
    away puts it far off the line, which the envelope's OOD scorer must catch and announce."""
    h = make_harness(1.0, ("pose_frame:=map",), tf=(0.0, 0.0, 0.0))
    h.run(0.6)  # past the first command, which arrives before the first scan ("no_scan")
    h.events.clear()
    h.run(1.0)
    assert h.statuses[-1]["state"] == "active"
    assert not [e for e in h.events if e["event"] == "fallback_engage"]
    h.tf = (5.0, 0.0, 0.0)
    h.events.clear()
    h.run(1.0)
    engages = [e for e in h.events if e["event"] == "fallback_engage"]
    assert len(engages) == 1 and engages[0]["reason"] == "ood", h.events


def test_no_map_to_odom_transform_means_no_command(make_harness) -> None:
    h = make_harness(0.0, ("pose_frame:=map",), tf=None, wait=False)
    h.run(2.0)
    assert h.proc.poll() is None
    assert h.drive_raw == []
