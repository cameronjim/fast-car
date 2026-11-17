"""lap_memory_node with an external lap source on a live ROS graph (L3, brief 51): the real node
process, a harness that plays a car on a loop (wheel odometry, gated steering, planner status) and
the camera's two JSON topics (`/lap/event`, `/lap/status`), assertions on `/pnp/lap/*`.

Covers: LapEvents with frame_id camera stamped at the crossing, a silent status giving
`source_alive` false and the lap-1 cap within 3.5 s, malformed JSON counted and dropped, QoS (a
best-effort publisher is not matched), `truth` without sim time exiting 2, `output_prefix` moving the
three topics, no `/scan` subscription, nobody publishing `/drive_raw` or `/drive`, clean shutdown.

Needs rclpy and the colcon-built racer_pnp_msgs, racer_msgs and racer_tools; skipped where they are
absent (the bare `uv run pytest` run).
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
pytest.importorskip("racer_tools")

from ackermann_msgs.msg import AckermannDriveStamped
from nav_msgs.msg import Odometry
from racer_pnp_msgs.msg import LapEvent, LapGuidance, PlannerStatus
from rclpy.qos import HistoryPolicy, QoSProfile, ReliabilityPolicy
from std_msgs.msg import String
from synth import WHEELBASE_M

os.environ.setdefault("ROS_DOMAIN_ID", "98")

SPEED_MPS = 3.0
LAP_M = 6.0
LAP_S = LAP_M / SPEED_MPS
STEER = math.atan(2.0 * math.pi * WHEELBASE_M / LAP_M)


def _rel(depth: int = 10) -> QoSProfile:
    return QoSProfile(
        reliability=ReliabilityPolicy.RELIABLE, history=HistoryPolicy.KEEP_LAST, depth=depth
    )


def _best_effort(depth: int = 10) -> QoSProfile:
    return QoSProfile(
        reliability=ReliabilityPolicy.BEST_EFFORT, history=HistoryPolicy.KEEP_LAST, depth=depth
    )


def node_args(tmp_path: Path, *extra: str) -> list[str]:
    return [
        sys.executable,
        "-m",
        "racer_laps.lap_memory_node",
        "--ros-args",
        "-p",
        f"output_dir:={tmp_path / 'laps'}",
        "-p",
        "yaw_gain_override:=1.0",
        "-p",
        "guidance_rate_hz:=10.0",
        *[x for e in extra for x in ("-p", e)],
    ]


class Harness:
    def __init__(
        self,
        tmp_path: Path,
        extra: tuple[str, ...] = ("lap_source:=camera",),
        source_qos: QoSProfile | None = None,
    ) -> None:
        self.proc = subprocess.Popen(
            node_args(tmp_path, *extra),
            env=dict(os.environ),
            stdout=subprocess.PIPE,
            stderr=subprocess.STDOUT,
            text=True,
        )
        rclpy.init()
        self.node = rclpy.create_node("lap_ext_test_harness")
        rel = _rel(50)
        self.planner_pub = self.node.create_publisher(PlannerStatus, "/pnp/planner/status", rel)
        self.odom_pub = self.node.create_publisher(Odometry, "/odom/wheel", rel)
        self.drive_pub = self.node.create_publisher(AckermannDriveStamped, "/drive", rel)
        qos = source_qos or _rel(10)
        self.event_pub = self.node.create_publisher(String, "/lap/event", qos)
        self.status_pub = self.node.create_publisher(String, "/lap/status", qos)
        self.events: list[LapEvent] = []
        self.guidance: list[tuple[float, LapGuidance]] = []
        self.statuses: list[dict[str, Any]] = []
        prefix = "/pnp/lap"
        for e in extra:
            if e.startswith("output_prefix:="):
                prefix = e.split(":=", 1)[1].rstrip("/")
        self.prefix = prefix
        self.node.create_subscription(LapEvent, f"{prefix}/event", self.events.append, rel)
        self.node.create_subscription(
            LapGuidance,
            f"{prefix}/guidance",
            lambda m: self.guidance.append((time.monotonic(), m)),
            rel,
        )
        self.node.create_subscription(
            String, f"{prefix}/status", lambda m: self.statuses.append(json.loads(m.data)), rel
        )
        self.t_start = math.nan

    def now(self) -> float:
        return float(self.node.get_clock().now().nanoseconds) * 1e-9

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

    def camera_status(self, state: str = "running", laps: int = 0, sim: float | None = 1.0) -> None:
        self.status_pub.publish(
            String(data=json.dumps({"state": state, "similarity": sim, "laps": laps}))
        )

    def camera_event(self, lap: int, crossing_s: float) -> None:
        self.event_pub.publish(
            String(
                data=json.dumps(
                    {
                        "lap": lap,
                        "lap_time_s": LAP_S,
                        "peak_similarity": 0.93,
                        "crossing_time_s": crossing_s,
                        "distance_m": LAP_M,
                    }
                )
            )
        )

    def planner_driving(self) -> None:
        msg = PlannerStatus()
        msg.state = 1
        msg.plan_min_clearance_m = 0.3
        self.planner_pub.publish(msg)

    def odom(self) -> None:
        msg = Odometry()
        msg.header.stamp = self.node.get_clock().now().to_msg()
        msg.twist.twist.linear.x = SPEED_MPS
        self.odom_pub.publish(msg)
        drive = AckermannDriveStamped()
        drive.drive.steering_angle = STEER
        self.drive_pub.publish(drive)

    def drive(
        self,
        seconds: float,
        status_every_s: float | None = 1.0,
        events: bool = True,
        latency_s: float = 0.15,
        first_lap_no: int = 1,
    ) -> None:
        """Play `seconds` of driving in real time: odometry at 50 Hz, the planner DRIVING, a camera
        status every `status_every_s` (None: silent), one camera event per lap whose crossing time
        is the clock time of the crossing (it is published `latency_s` later)."""
        t0 = time.monotonic()
        next_status = 0.0
        pending: list[tuple[float, int, float]] = []  # (publish at monotonic, lap, crossing)
        if math.isnan(self.t_start):
            self.t_start = self.now()
        lap = first_lap_no
        sent = 0
        while time.monotonic() - t0 < seconds:
            rel = time.monotonic() - t0
            if status_every_s is not None and rel >= next_status:
                self.camera_status()
                next_status = rel + status_every_s
            self.planner_driving()
            self.odom()
            if events and self.now() - self.t_start >= (sent + 1) * LAP_S:
                crossing = self.t_start + (sent + 1) * LAP_S
                pending.append((time.monotonic() + latency_s, lap, crossing))
                lap += 1
                sent += 1
            for item in [p for p in pending if p[0] <= time.monotonic()]:
                pending.remove(item)
                self.camera_event(item[1], item[2])
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


def test_camera_events_become_lap_events_stamped_at_the_crossing(harness: Harness) -> None:
    harness.camera_status()
    harness.spin(1.5)  # the node's status is 1 Hz
    assert harness.statuses[-1]["source_ready_now"] is True
    harness.drive(4.0 * LAP_S + 1.0)
    harness.spin(0.3)
    events = list(harness.events)
    assert [e.lap_index for e in events][:3] == [1, 2, 3]
    for i, e in enumerate(events[:3], start=1):
        assert e.header.frame_id == "camera"
        stamp = e.header.stamp.sec + e.header.stamp.nanosec * 1e-9
        assert stamp == pytest.approx(harness.t_start + i * LAP_S, abs=0.25)
        assert e.lap_length_m == pytest.approx(LAP_M, abs=0.3)
        assert e.lap_time_s == pytest.approx(LAP_S, abs=0.3)
        assert e.clean
    assert [round(e.v_cap_mps, 3) for e in events[:3]] == [0.5, 0.6, 0.7]
    status = harness.statuses[-1]
    assert status["lap_source"] == "camera" and status["line_source"] == "camera"
    assert status["source_ready"] is True and status["source_alive"] is True
    assert status["confirmed_laps"] >= 3 and status["doubtful_laps"] == []
    assert status["rejected_events"]["malformed"] == 0
    assert status["event_latency_s_max"] < 1.0
    # no scan work, no command publishing
    assert not any(
        i.node_name == "lap_memory_node"
        for i in harness.node.get_subscriptions_info_by_topic("/scan")
    )
    for topic in ("/drive_raw",):
        assert harness.node.get_publishers_info_by_topic(topic) == [], topic
    assert all(
        i.node_name != "lap_memory_node"
        for i in harness.node.get_publishers_info_by_topic("/drive")
    )


def test_a_silent_status_gives_the_lap_one_cap_within_3_5_s(harness: Harness) -> None:
    harness.camera_status()
    harness.spin(0.3)
    harness.drive(2.0 * LAP_S + 0.5)
    assert harness.guidance[-1][1].v_cap_mps > 0.55  # the cap rose
    t_silent = time.monotonic()
    harness.drive(4.0, status_every_s=None, events=False)
    cap_times = [t for t, g in harness.guidance if abs(g.v_cap_mps - 0.5) < 1e-9 and t > t_silent]
    assert cap_times, "the lap-1 cap was never handed out"
    assert cap_times[0] - t_silent < 3.5
    harness.spin(0.3)
    status = harness.statuses[-1]
    assert status["source_alive"] is False
    assert status["station_uncertain"] is True


def test_malformed_json_is_counted_and_dropped(harness: Harness) -> None:
    harness.camera_status()
    harness.event_pub.publish(String(data="not json"))
    harness.event_pub.publish(String(data='{"lap": 1}'))
    harness.status_pub.publish(String(data='{"state": "running"}'))
    harness.spin(1.5)
    status = harness.statuses[-1]
    assert status["rejected_events"]["malformed"] == 3
    assert harness.events == []
    assert harness.proc.poll() is None


def test_status_before_the_start_says_ready_or_why_not(harness: Harness) -> None:
    harness.camera_status(state="waiting", sim=None)
    harness.spin(1.5)
    status = harness.statuses[-1]
    assert status["source_ready_now"] is False
    assert "waiting" in status["source_not_ready_reason"]
    harness.camera_status()
    harness.spin(1.5)
    assert harness.statuses[-1]["source_ready_now"] is True


def test_a_best_effort_publisher_is_not_matched(tmp_path: Path) -> None:
    h = Harness(tmp_path, source_qos=_best_effort(10))
    h.wait_for_node()
    try:
        h.drive(2.0 * LAP_S + 0.5)
        h.spin(1.0)
        assert h.events == []
        status = h.statuses[-1]
        assert status["source_alive"] is False
        assert status["source_status_age_s"] is None  # no status ever arrived
        assert status["source_ready_now"] is False
        assert status["confirmed_laps"] == 0
    finally:
        h.close()


def test_truth_without_sim_time_exits_2(tmp_path: Path) -> None:
    proc = subprocess.run(
        node_args(tmp_path, "lap_source:=truth"),
        env=dict(os.environ),
        capture_output=True,
        text=True,
        timeout=30,
        check=False,
    )
    assert proc.returncode == 2
    assert "use_sim_time" in proc.stderr + proc.stdout


def test_truth_with_sim_time_starts(tmp_path: Path) -> None:
    h = Harness(tmp_path, extra=("lap_source:=truth", "use_sim_time:=true"))
    try:
        # sim time with no /clock stays at zero; the node still starts and publishes
        end = time.monotonic() + 15.0
        while time.monotonic() < end and h.proc.poll() is None:
            h.spin(0.2)
        assert h.proc.poll() is None
    finally:
        h.close()
    assert h.returncode == 0


def test_output_prefix_moves_the_three_topics(tmp_path: Path) -> None:
    h = Harness(tmp_path, extra=("lap_source:=camera", "output_prefix:=/pnp/lap_shadow"))
    h.wait_for_node()
    try:
        for suffix in ("event", "guidance", "status"):
            assert h.node.get_publishers_info_by_topic(f"/pnp/lap_shadow/{suffix}"), suffix
            assert h.node.get_publishers_info_by_topic(f"/pnp/lap/{suffix}") == [], suffix
    finally:
        h.close()


def test_observe_has_no_guidance_topic(tmp_path: Path) -> None:
    h = Harness(tmp_path, extra=("lap_source:=camera", "lap_mode:=observe"))
    h.wait_for_node()
    try:
        h.spin(1.0)
        assert h.node.get_publishers_info_by_topic("/pnp/lap/guidance") == []
        assert h.node.get_publishers_info_by_topic("/pnp/lap/event")
    finally:
        h.close()


def test_v1_with_the_camera_is_refused(tmp_path: Path) -> None:
    proc = subprocess.run(
        node_args(tmp_path, "lap_source:=camera", "lap_mode:=v1"),
        env=dict(os.environ),
        capture_output=True,
        text=True,
        timeout=30,
        check=False,
    )
    assert proc.returncode == 2


def test_clean_shutdown_on_sigint(tmp_path: Path) -> None:
    h = Harness(tmp_path)
    h.wait_for_node()
    h.close()
    assert h.returncode == 0
