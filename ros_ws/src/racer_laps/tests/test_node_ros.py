"""lap_memory_node on a live ROS graph (L3): the real node process, a harness that plays TF, the
planner status and the safety events, and assertions on /pnp/lap/event, /pnp/lap/guidance and
/pnp/lap/status.

Covers: one LapEvent per loop of a square with the right index, guidance at 2 Hz, nobody
publishing /drive_raw or /drive, a missing transform reported and no lap invented, a safety
brake making a lap unclean, the safe set npz written, refusal of a bad cap schedule, clean
shutdown on SIGINT.

Needs rclpy and the colcon-built racer_pnp_msgs and racer_msgs; skipped where they are absent
(the bare `uv run pytest` run).
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

rclpy = pytest.importorskip("rclpy")
pytest.importorskip("racer_pnp_msgs.msg")
pytest.importorskip("racer_msgs.msg")

from geometry_msgs.msg import TransformStamped
from nav_msgs.msg import Odometry
from racer_msgs.msg import SafetyEvent
from racer_pnp_msgs.msg import LapEvent, LapGuidance, PlannerStatus
from rclpy.qos import HistoryPolicy, QoSProfile, ReliabilityPolicy
from std_msgs.msg import String
from tf2_msgs.msg import TFMessage

os.environ.setdefault("ROS_DOMAIN_ID", "96")

SIDE_M = 2.0
SPEED_MPS = 4.0  # 2 s per 8 m loop: fast, the node only sees poses
LOOP_S = 4.0 * SIDE_M / SPEED_MPS


def square_pose(t: float) -> tuple[float, float, float]:
    """Pose on a square of side SIDE_M, counter-clockwise, starting mid edge at (1, 0) heading +x
    (a start line at a corner would sit on the path's turn)."""
    d = (SPEED_MPS * t + 0.5 * SIDE_M) % (4.0 * SIDE_M)
    edge, u = divmod(d, SIDE_M)
    if edge == 0:
        return u, 0.0, 0.0
    if edge == 1:
        return SIDE_M, u, math.pi / 2
    if edge == 2:
        return SIDE_M - u, SIDE_M, math.pi
    return 0.0, SIDE_M - u, -math.pi / 2


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
                *[x for e in extra for x in ("-p", e)],
            ],
            env=dict(os.environ),
            stdout=subprocess.PIPE,
            stderr=subprocess.STDOUT,
            text=True,
        )
        rclpy.init()
        self.node = rclpy.create_node("lap_test_harness")
        rel = QoSProfile(
            reliability=ReliabilityPolicy.RELIABLE, history=HistoryPolicy.KEEP_LAST, depth=50
        )
        self.tf_pub = self.node.create_publisher(TFMessage, "/tf", rel)
        self.planner_pub = self.node.create_publisher(PlannerStatus, "/pnp/planner/status", rel)
        self.safety_pub = self.node.create_publisher(SafetyEvent, "/safety/events", rel)
        self.odom_pub = self.node.create_publisher(Odometry, "/odom/wheel", rel)
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

    def spin(self, seconds: float) -> None:
        end = time.monotonic() + seconds
        while time.monotonic() < end:
            rclpy.spin_once(self.node, timeout_sec=0.005)

    def wait_for_node(self, timeout: float = 20.0) -> None:
        end = time.monotonic() + timeout
        while time.monotonic() < end:
            self.spin(0.1)
            if self.statuses:
                return
        raise AssertionError("lap_memory_node never published a status:\n" + self.close())

    def driving(self) -> None:
        msg = PlannerStatus()
        msg.state = 1
        msg.plan_min_clearance_m = 0.3
        self.planner_pub.publish(msg)

    def publish_pose(self, x: float, y: float, yaw: float, stamp_s: float | None = None) -> None:
        now = self.node.get_clock().now().nanoseconds * 1e-9 if stamp_s is None else stamp_s
        tf = TransformStamped()
        tf.header.stamp.sec = int(now)
        tf.header.stamp.nanosec = int((now % 1.0) * 1e9)
        tf.header.frame_id = "map"
        tf.child_frame_id = "base_link"
        tf.transform.translation.x = x
        tf.transform.translation.y = y
        tf.transform.rotation.z = math.sin(yaw / 2.0)
        tf.transform.rotation.w = math.cos(yaw / 2.0)
        self.tf_pub.publish(TFMessage(transforms=[tf]))

    def drive_loops(self, loops: float, on_time: Any = None) -> None:
        """Play the square in real time at 50 Hz TF."""
        t0 = time.monotonic()
        while True:
            t = time.monotonic() - t0
            if t > loops * LOOP_S:
                break
            if on_time is not None:
                on_time(t)
            self.publish_pose(*square_pose(t))
            odom = Odometry()
            odom.twist.twist.linear.x = SPEED_MPS
            self.odom_pub.publish(odom)
            self.driving()
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


def test_one_lap_event_per_loop_and_guidance_rate(harness: Harness) -> None:
    harness.drive_loops(3.4)
    harness.spin(0.3)
    events = list(harness.events)
    assert [e.lap_index for e in events] == [1, 2, 3]
    for e in events:
        assert abs(e.lap_time_s - LOOP_S) < 0.25
        assert abs(e.lap_length_m - 4 * SIDE_M) < 0.5
        assert e.header.frame_id == "map"
        assert e.safety_brakes == 0
    assert [e.clean for e in events] == [True, True, True]
    assert [round(e.v_cap_mps, 3) for e in events] == [0.5, 0.6, 0.7]

    # guidance: 2 Hz, lap index and cap follow the laps, arrays from the second lap on
    times = [t for t, _ in harness.guidance]
    span = times[-1] - times[0]
    assert span > 5.0
    rate = (len(times) - 1) / span
    assert 1.6 < rate < 2.4, rate
    indices = [g.lap_index for _, g in harness.guidance]
    assert indices == sorted(indices) and indices[-1] == 4
    first_lap = [g for _, g in harness.guidance if g.lap_index == 1]
    assert first_lap and all(len(g.s_m) == 0 for g in first_lap)
    later = [g for _, g in harness.guidance if g.lap_index >= 2 and len(g.s_m) > 0]
    assert later
    g = later[-1]
    assert len(g.s_m) == len(g.v_ref_mps) == len(g.cost_to_go_s)
    assert all(v > 0.0 for v in g.v_ref_mps)

    # the safe set is on disk and loads
    files = sorted(harness.out_dir.glob("safe_set_*.npz"))
    assert len(files) == 1
    from racer_laps.safe_set import SafeSet

    assert SafeSet.load(files[0]).n_laps == 3


def test_node_is_not_a_command_publisher(harness: Harness) -> None:
    harness.spin(1.0)
    for topic in ("/drive_raw", "/drive"):
        assert harness.node.get_publishers_info_by_topic(topic) == [], topic
    assert harness.node.get_publishers_info_by_topic("/pnp/lap/event")


def test_missing_tf_is_reported_and_no_lap_is_invented(harness: Harness) -> None:
    for _ in range(100):
        harness.driving()
        harness.spin(0.02)
    harness.spin(1.5)
    assert harness.events == []
    status = harness.statuses[-1]
    assert status["tf_ok"] is False
    assert status["tf_error"] != ""
    assert status["laps_completed"] == 0

    # a stale transform is also reported, and still no lap
    harness.publish_pose(0.0, 0.0, 0.0, stamp_s=time.time() - 100.0)
    harness.spin(1.5)
    status = harness.statuses[-1]
    assert status["tf_ok"] is False
    assert harness.events == []


def test_safety_brake_makes_the_lap_unclean(harness: Harness) -> None:
    sent = {"done": False}

    def brake(t: float) -> None:
        if t > 1.4 * LOOP_S and not sent["done"]:
            sent["done"] = True
            harness.safety_pub.publish(SafetyEvent(severity=2, phase=0, source="ttc"))

    harness.drive_loops(2.4, on_time=brake)
    harness.spin(0.3)
    assert [e.lap_index for e in harness.events] == [1, 2]
    assert [e.clean for e in harness.events] == [True, False]
    assert harness.events[1].safety_brakes == 1


def test_clean_shutdown_on_sigint(tmp_path: Path) -> None:
    h = Harness(tmp_path)
    h.wait_for_node()
    h.close()
    assert h.returncode == 0


def test_bad_cap_schedule_is_refused(tmp_path: Path) -> None:
    proc = subprocess.run(
        [
            sys.executable,
            "-m",
            "racer_laps.lap_memory_node",
            "--ros-args",
            "-p",
            "v_cap_start_mps:=1.0",
            "-p",
            "v_cap_ceiling_mps:=0.5",
            "-p",
            f"output_dir:={tmp_path}",
        ],
        env=dict(os.environ),
        capture_output=True,
        text=True,
        timeout=30,
        check=False,
    )
    assert proc.returncode != 0
    assert "ceiling" in proc.stderr + proc.stdout
