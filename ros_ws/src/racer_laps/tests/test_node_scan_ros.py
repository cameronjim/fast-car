"""lap_memory_node with the start-scan source on a live ROS graph (L3): the real node process, a
harness that plays a synthetic stadium (scans, wheel odometry, gated steering, planner status) and
assertions on /pnp/lap/event, /pnp/lap/guidance and /pnp/lap/status.

Covers: one LapEvent per lap with frame_id start_scan and the right times, guidance at 10 Hz
(empty arrays in D v0, 61 valid stations in D v1), silence on each input and the status saying why
there is no lap, QoS, nobody publishing /drive_raw or /drive, a bad lap_source or lap_mode refused,
clean shutdown.

Needs rclpy and the colcon-built racer_pnp_msgs, racer_msgs and racer_tools; skipped where they
are absent (the bare `uv run pytest` run).
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

import numpy as np
import pytest

rclpy = pytest.importorskip("rclpy")
pytest.importorskip("racer_pnp_msgs.msg")
pytest.importorskip("racer_msgs.msg")
pytest.importorskip("racer_tools")

from ackermann_msgs.msg import AckermannDriveStamped
from nav_msgs.msg import Odometry
from planner_rules import planner_accepts
from racer_laps.engine import Guidance
from racer_laps.scan_match import scan_to_points
from racer_pnp_msgs.msg import LapEvent, LapGuidance, PlannerStatus
from rclpy.qos import HistoryPolicy, QoSProfile, ReliabilityPolicy
from sensor_msgs.msg import LaserScan
from std_msgs.msg import String
from synth import BODY, MOUNT, RANGE_MAX_M, WHEELBASE_M, Stadium, scan_at

os.environ.setdefault("ROS_DOMAIN_ID", "97")

STADIUM = Stadium()
SPEED_MPS = 3.0
LOOP_S = STADIUM.length_m / SPEED_MPS


class Harness:
    def __init__(self, tmp_path: Path, extra: tuple[str, ...] = ()) -> None:
        self.out_dir = tmp_path / "laps"
        self.proc = subprocess.Popen(
            [
                sys.executable,
                "-m",
                "racer_laps.lap_memory_node",
                "--ros-args",
                "-p",
                f"output_dir:={self.out_dir}",
                "-p",
                "yaw_gain_override:=1.0",
                "-p",
                "save_lap1_map:=false",
                *[x for e in extra for x in ("-p", e)],
            ],
            env=dict(os.environ),
            stdout=subprocess.PIPE,
            stderr=subprocess.STDOUT,
            text=True,
        )
        rclpy.init()
        self.node = rclpy.create_node("lap_scan_test_harness")
        rel = QoSProfile(
            reliability=ReliabilityPolicy.RELIABLE, history=HistoryPolicy.KEEP_LAST, depth=50
        )
        best = QoSProfile(
            reliability=ReliabilityPolicy.BEST_EFFORT, history=HistoryPolicy.KEEP_LAST, depth=10
        )
        self.scan_pub = self.node.create_publisher(LaserScan, "/scan", best)
        self.odom_pub = self.node.create_publisher(Odometry, "/odom/wheel", best)
        self.drive_pub = self.node.create_publisher(AckermannDriveStamped, "/drive", best)
        self.planner_pub = self.node.create_publisher(PlannerStatus, "/pnp/planner/status", rel)
        self.events: list[LapEvent] = []
        self.guidance: list[tuple[float, LapGuidance]] = []
        self.statuses: list[dict[str, Any]] = []
        self.node.create_subscription(LapEvent, "/pnp/lap/event", self.events.append, rel)
        self.node.create_subscription(
            LapGuidance,
            "/pnp/lap/guidance",
            lambda m: self.guidance.append((time.monotonic(), m)),
            rel,
        )
        self.node.create_subscription(
            String, "/pnp/lap/status", lambda m: self.statuses.append(json.loads(m.data)), rel
        )
        self.walls = STADIUM.walls()
        self.rng = np.random.default_rng(7)

    def spin(self, seconds: float) -> None:
        end = time.monotonic() + seconds
        while time.monotonic() < end:
            rclpy.spin_once(self.node, timeout_sec=0.005)

    def wait_for_node(self, timeout: float = 25.0) -> None:
        end = time.monotonic() + timeout
        while time.monotonic() < end:
            self.spin(0.1)
            if self.statuses:
                return
        raise AssertionError("lap_memory_node never published a status:\n" + self.close())

    def now_s(self) -> float:
        return self.node.get_clock().now().nanoseconds * 1e-9

    def driving(self) -> None:
        msg = PlannerStatus()
        msg.state = 1
        msg.plan_min_clearance_m = 0.3
        self.planner_pub.publish(msg)

    def publish_odom(self, v: float) -> None:
        odom = Odometry()
        odom.header.stamp = self.node.get_clock().now().to_msg()
        odom.twist.twist.linear.x = v
        self.odom_pub.publish(odom)

    def publish_drive(self, steer: float) -> None:
        d = AckermannDriveStamped()
        d.header.stamp = self.node.get_clock().now().to_msg()
        d.drive.steering_angle = steer
        self.drive_pub.publish(d)

    def publish_scan(self, x: float, y: float, yaw: float) -> None:
        ranges, a0, da = scan_at(self.walls, x, y, yaw, self.rng)
        msg = LaserScan()
        msg.header.stamp = self.node.get_clock().now().to_msg()
        msg.header.frame_id = "laser"
        msg.angle_min = float(a0)
        msg.angle_max = float(a0 + da * (len(ranges) - 1))
        msg.angle_increment = float(da)
        msg.scan_time = 0.1
        msg.range_min = 0.05
        msg.range_max = float(RANGE_MAX_M)
        msg.ranges = [float(r) for r in ranges]
        self.scan_pub.publish(msg)

    def drive_loops(
        self,
        loops: float,
        scans: bool = True,
        odom: bool = True,
        planner: bool = True,
        start_s: float = 0.0,
    ) -> None:
        """Play the stadium in real time: odometry and steering at 50 Hz, scans at 10 Hz."""
        t0 = time.monotonic()
        next_scan = 0.0
        while True:
            t = time.monotonic() - t0
            if t > loops * LOOP_S:
                break
            x, y, yaw, k = STADIUM.pose_at(start_s + SPEED_MPS * t)
            if odom:
                self.publish_odom(SPEED_MPS)
                self.publish_drive(math.atan(WHEELBASE_M * k))
            if planner:
                self.driving()
            if scans and t >= next_scan:
                next_scan += 0.1
                self.publish_scan(x, y, yaw)
            self.spin(0.02)

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
        self.returncode = self.proc.returncode
        return str(out)


@pytest.fixture
def harness(tmp_path: Path):  # type: ignore[no-untyped-def]
    h = Harness(tmp_path)
    h.wait_for_node()
    try:
        yield h
    finally:
        if h.proc.poll() is None:
            h.close()


def test_one_lap_event_per_lap_with_start_scan_frame_and_v0_guidance(harness: Harness) -> None:
    harness.drive_loops(4.2)
    harness.spin(0.3)
    events = list(harness.events)
    assert [e.lap_index for e in events] == [1, 2, 3, 4]
    for e in events:
        assert e.header.frame_id == "start_scan"
        assert abs(e.lap_time_s - LOOP_S) < 0.3
        assert abs(e.lap_length_m - STADIUM.length_m) < 0.6
        assert e.safety_brakes == 0
    assert [e.clean for e in events] == [True] * 4
    assert [round(e.v_cap_mps, 2) for e in events] == [0.5, 0.6, 0.7, 0.8]
    # guidance at 10 Hz, arrays empty in D v0, the cap follows the laps
    times = [t for t, _ in harness.guidance]
    rate = (len(times) - 1) / (times[-1] - times[0])
    assert 8.0 < rate < 12.0, rate
    assert all(len(g.s_m) == 0 for _, g in harness.guidance)
    assert harness.guidance[-1][1].lap_index == 5
    status = harness.statuses[-1]
    assert status["lap_source"] == "start_scan" and status["lap_mode"] == "v0"
    assert status["laps_completed"] >= 3
    files = sorted(harness.out_dir.glob("station_set_*.npz"))
    assert len(files) == 1
    from racer_laps.station_set import StationSet

    assert StationSet.load(files[0]).n_laps == 4


def test_node_is_not_a_command_publisher_and_qos(harness: Harness) -> None:
    harness.spin(1.0)
    for topic in ("/drive_raw", "/drive"):
        assert [
            i
            for i in harness.node.get_publishers_info_by_topic(topic)
            if i.node_name == "lap_memory_node"
        ] == []
    for topic in ("/pnp/lap/event", "/pnp/lap/guidance", "/pnp/lap/status"):
        info = harness.node.get_publishers_info_by_topic(topic)
        assert info and info[0].node_name == "lap_memory_node"
        assert info[0].qos_profile.reliability == ReliabilityPolicy.RELIABLE
    scan_subs = [
        i
        for i in harness.node.get_subscriptions_info_by_topic("/scan")
        if i.node_name == "lap_memory_node"
    ]
    assert scan_subs and scan_subs[0].qos_profile.reliability == ReliabilityPolicy.BEST_EFFORT


@pytest.mark.parametrize(
    ("scans", "odom", "planner", "why"),
    [
        (False, True, True, "waiting_for_scan"),
        (True, False, True, "waiting_for_scan"),  # no odometry: the gate refuses to start
        (True, True, False, "waiting_for_driving"),
    ],
)
def test_silence_on_each_input_makes_no_lap_and_the_status_says_why(
    harness: Harness, scans: bool, odom: bool, planner: bool, why: str
) -> None:
    harness.drive_loops(1.5, scans=scans, odom=odom, planner=planner)
    harness.spin(1.5)
    assert harness.events == []
    status = harness.statuses[-1]
    assert status["laps_completed"] == 0
    assert status["why_no_lap"] == why
    assert status["phase"] != "running"


def test_a_scan_gap_does_not_invent_a_lap(harness: Harness) -> None:
    harness.drive_loops(0.4)
    harness.drive_loops(0.9, scans=False)  # the LiDAR goes silent for most of a lap
    harness.spin(0.5)
    assert harness.events == []
    assert harness.statuses[-1]["laps_completed"] == 0


def test_v1_publishes_valid_guidance_after_clean_laps(tmp_path: Path) -> None:
    h = Harness(tmp_path, extra=("lap_mode:=v1", "explore_dv_mps:=0.2", "v_cap_ceiling_mps:=1.0"))
    h.wait_for_node()
    try:
        h.drive_loops(3.4)
        h.spin(0.3)
        assert len(h.events) >= 2
        with_arrays = [g for _, g in h.guidance if len(g.s_m) > 0]
        assert with_arrays
        for g in with_arrays:
            assert len(g.s_m) == len(g.v_ref_mps) == len(g.cost_to_go_s) == 61
            assert planner_accepts(
                Guidance(
                    g.lap_index,
                    g.v_cap_mps,
                    np.array(g.s_m, dtype=np.float64),
                    np.array(g.v_ref_mps, dtype=np.float64),
                    np.array(g.cost_to_go_s, dtype=np.float64),
                )
            )
            assert g.v_cap_mps <= 1.0 + 1e-6
        # before any lap is stored the arrays are empty and the cap is the lap-1 cap
        first = [g for _, g in h.guidance if g.lap_index == 1]
        assert first and all(len(g.s_m) == 0 and abs(g.v_cap_mps - 0.5) < 1e-6 for g in first)
    finally:
        h.close()


@pytest.mark.parametrize("bad", ["lap_source:=magic", "lap_mode:=v2", "lap_mode:=v1"])
def test_bad_modes_are_refused(tmp_path: Path, bad: str) -> None:
    params = [bad]
    if bad == "lap_mode:=v1":
        params.append("lap_source:=map_pose")  # v1 is indexed by station: start_scan only
    proc = subprocess.run(
        [
            sys.executable,
            "-m",
            "racer_laps.lap_memory_node",
            "--ros-args",
            *[x for p in params for x in ("-p", p)],
            "-p",
            f"output_dir:={tmp_path}",
        ],
        env=dict(os.environ),
        capture_output=True,
        text=True,
        timeout=40,
        check=False,
    )
    assert proc.returncode != 0
    assert "refused" in proc.stderr + proc.stdout or "lap_" in proc.stderr + proc.stdout


def test_clean_shutdown_on_sigint(tmp_path: Path) -> None:
    h = Harness(tmp_path)
    h.wait_for_node()
    h.close()
    assert h.returncode == 0


def test_scan_points_helper_matches_the_node_inputs() -> None:
    # the harness casts with the car's mount: guard the test world against drifting from it
    x, y, yaw, _k = STADIUM.pose_at(0.0)
    ranges, a0, da = scan_at(STADIUM.walls(), x, y, yaw, np.random.default_rng(0))
    pts = scan_to_points(ranges, a0, da, 0.05, RANGE_MAX_M, MOUNT, BODY)
    assert pts.shape[0] > 300
