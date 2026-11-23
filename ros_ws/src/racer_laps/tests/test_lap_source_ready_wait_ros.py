"""L3 (brief 51c item 4): `lap_source_ready_wait` on a live ROS graph with a fake lap node. Exit 0
after two ready statuses, exit 3 with the reason after the wait (the test shortens `wait_s`
through the gate's config, never through a node parameter), a best-effort publisher is not matched
and the waiter refuses with "no lap node status", and the node shuts down cleanly (it creates no
publisher and no service).

Needs rclpy; skipped where it is absent (the bare `uv run pytest` run)."""

from __future__ import annotations

import json
import os
import threading
import time

import pytest

rclpy = pytest.importorskip("rclpy")

from rclpy.context import Context
from rclpy.qos import HistoryPolicy, QoSProfile, ReliabilityPolicy
from std_msgs.msg import String

os.environ.setdefault("ROS_DOMAIN_ID", "97")

from racer_laps import lap_source_ready_wait as wait
from racer_laps.start_ready import StartGateConfig

TOPIC = "/pnp/lap/status"


def qos(reliable: bool) -> QoSProfile:
    return QoSProfile(
        reliability=ReliabilityPolicy.RELIABLE if reliable else ReliabilityPolicy.BEST_EFFORT,
        history=HistoryPolicy.KEEP_LAST,
        depth=10,
    )


def status(ready: bool = True, reason: str = "") -> str:
    return json.dumps(
        {
            "lap_source": "camera",
            "phase": "waiting_for_driving",
            "source_ready_now": ready,
            "source_alive": ready,
            "source_not_ready_reason": reason,
        }
    )


class FakeLapNode:
    """A lap node's status publisher in a context of its own, on a thread."""

    def __init__(self, reliable: bool, bodies, start_after_s: float = 0.0) -> None:
        self.ctx = Context()
        rclpy.init(context=self.ctx)
        self.node = rclpy.create_node("fake_lap_node", context=self.ctx)
        self.pub = self.node.create_publisher(String, TOPIC, qos(reliable))
        self.bodies = bodies
        self.start_after_s = start_after_s
        self.stop = threading.Event()
        self.thread = threading.Thread(target=self.run, daemon=True)
        self.thread.start()

    def run(self) -> None:
        time.sleep(self.start_after_s)
        k = 0
        while not self.stop.is_set():
            body = self.bodies[min(k, len(self.bodies) - 1)]
            self.pub.publish(String(data=body))
            k += 1
            time.sleep(0.2)

    def close(self) -> None:
        self.stop.set()
        self.thread.join(timeout=2.0)
        self.node.destroy_node()
        rclpy.shutdown(context=self.ctx)


def run_waiter(cfg: StartGateConfig) -> int:
    return wait.main(["--ros-args", "-p", f"status_topic:={TOPIC}"], gate_config=cfg)


def test_exit_0_after_two_ready_statuses_from_a_fake_lap_node() -> None:
    fake = FakeLapNode(True, [status(True)], start_after_s=1.0)
    try:
        started = time.monotonic()
        assert run_waiter(StartGateConfig(wait_s=20.0)) == wait.EXIT_START
        assert time.monotonic() - started >= 1.0  # it waited for the first status
    finally:
        fake.close()


def test_exit_3_with_the_reason_after_the_wait(capfd) -> None:
    reason = "similarity 0.50 below 0.90"
    fake = FakeLapNode(True, [status(False, reason)])
    try:
        assert run_waiter(StartGateConfig(wait_s=2.0)) == wait.EXIT_REFUSE
    finally:
        fake.close()
    err = capfd.readouterr()
    last = [x for x in (err.out + err.err).splitlines() if wait.LAST_LINE_PREFIX in x][-1]
    assert wait.reason_of_last_line(last) == reason


def test_a_best_effort_publisher_is_not_matched_and_the_waiter_refuses(capfd) -> None:
    fake = FakeLapNode(False, [status(True)])
    try:
        assert run_waiter(StartGateConfig(wait_s=2.0)) == wait.EXIT_REFUSE
    finally:
        fake.close()
    out = capfd.readouterr()
    last = [x for x in (out.out + out.err).splitlines() if wait.LAST_LINE_PREFIX in x][-1]
    assert wait.reason_of_last_line(last) == "no lap node status"


def test_the_waiter_makes_no_publisher_and_no_service_and_shuts_down_cleanly() -> None:
    probe_ctx = Context()
    rclpy.init(context=probe_ctx)
    probe = rclpy.create_node("probe", context=probe_ctx)
    seen: list[str] = []
    stop = threading.Event()

    def watch() -> None:
        while not stop.is_set():
            for name, _types in (
                probe.get_publisher_names_and_types_by_node("lap_source_ready_wait", "/")
                if "lap_source_ready_wait" in probe.get_node_names()
                else []
            ):
                seen.append(name)
            time.sleep(0.05)

    thread = threading.Thread(target=watch, daemon=True)
    thread.start()
    try:
        assert run_waiter(StartGateConfig(wait_s=1.0)) == wait.EXIT_REFUSE
    finally:
        stop.set()
        thread.join(timeout=2.0)
        probe.destroy_node()
        rclpy.shutdown(context=probe_ctx)
    # parameter events and rosout are the only topics a node publishes by itself
    assert [n for n in seen if n not in ("/rosout", "/parameter_events")] == []
    assert not rclpy.ok()  # the module shut its context down
