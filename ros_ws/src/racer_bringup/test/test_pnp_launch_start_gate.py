"""brief 51c, the start gate on the car (pnp.launch.py).

The planner is not started until the counter is in the graph, the `/lap_counter/reset` call has
exited 0 and the `lap_source_ready_wait` node (the start rule of `racer_laps.start_ready`) has
exited 0. Structure tests of the launch description first, then G6 with real launch machinery and
a fake lap node: ready after 8 s starts the planner after it, never ready refuses within 35 s with
the reason in the log and no planner process ever created, a reset that fails refuses, a lap node
that dies after one ready status refuses."""

from __future__ import annotations

import json
import logging
import sys
import threading
import time
from pathlib import Path

import pytest
from launch.actions import EmitEvent
from launch.events import Shutdown
from test_pnp_launch import (
    _bundle,
    _context,
    _module,
    _with_perception_installed,
)


def _gate_of(actions):
    from launch.actions import OpaqueFunction

    gates = [a for a in actions if isinstance(a, OpaqueFunction)]
    return gates[-1] if gates else None


def _planner_nodes(actions) -> list:
    return [a for a in actions if getattr(a, "node_package", "") == "racer_planner"]


def _processes(actions) -> list:
    return [a for a in actions if type(a).__name__ == "ExecuteProcess"]


def _camera_actions(m, monkeypatch, tmp_path, counter_in_graph=False, **overrides):
    _with_perception_installed(monkeypatch)
    monkeypatch.setattr(m, "recorder_in_graph", lambda *a, **k: True)
    monkeypatch.setattr(m, "lap_counter_in_graph", lambda *a, **k: counter_in_graph)
    args = {
        "lap_memory": "true",
        "lap_source": "camera",
        "lap_counter": "true",
        "dynamics_bundle": _bundle(tmp_path, 0.9),
        "v_max_mps": "0.8",
        "v_lap1_mps": "0.5",
    }
    args.update(overrides)
    return m._make_actions(_context(**args))


def _text(actions) -> str:
    return " ".join(
        "".join(getattr(t, "text", str(t)) for t in a.msg) for a in actions if hasattr(a, "msg")
    )


def _has_shutdown(actions) -> bool:
    return any(isinstance(x, EmitEvent) and isinstance(x.event, Shutdown) for x in actions)


def _cmd_text(process) -> str:
    parts = []
    for c in process.cmd:
        parts.append(
            "".join(getattr(t, "text", str(t)) for t in c) if isinstance(c, list) else str(c)
        )
    return " ".join(parts)


REFUSAL = "refusing to drive: the camera lap source is not ready:"


def test_the_last_line_prefix_is_the_waiters() -> None:
    from racer_laps.lap_source_ready_wait import LAST_LINE_PREFIX

    assert _module().WAITER_LAST_LINE_PREFIX == LAST_LINE_PREFIX


@pytest.mark.parametrize(
    ("lap_memory", "source", "shadow", "topic"),
    [
        (True, "camera", "", "/pnp/lap/status"),
        (True, "start_scan", "camera", "/pnp/lap_shadow/status"),
        (True, "start_scan", "start_scan", None),
        (True, "start_scan", "", None),
        (True, "map_pose", "", None),
        (False, "camera", "", None),
        (False, "start_scan", "camera", None),
    ],
)
def test_the_camera_lap_node_is_found_by_its_topic(lap_memory, source, shadow, topic) -> None:
    assert _module().camera_status_topic(lap_memory, source, shadow) == topic


def test_the_waiter_is_a_ros2_run_of_the_console_script_on_the_nodes_status_topic() -> None:
    text = _cmd_text(_module().lap_source_ready_waiter("/pnp/lap_shadow/status"))
    assert "ros2 run racer_laps lap_source_ready_wait" in text
    assert "status_topic:=/pnp/lap_shadow/status" in text


def test_camera_primary_holds_the_planner_behind_the_chain(monkeypatch, tmp_path) -> None:
    m = _module()
    actions = _camera_actions(m, monkeypatch, tmp_path)
    assert not _planner_nodes(actions)  # not started with the others
    assert not _processes(actions)  # the reset is not called until the counter is in the graph
    gate = _gate_of(actions)
    assert gate is not None and gate.chain.status_topic == "/pnp/lap/status"
    assert gate.chain.reset is not None  # the counter is started here: the reset is part of it


def test_the_chain_runs_counter_then_reset_then_waiter_then_planner(monkeypatch, tmp_path) -> None:
    m = _module()
    chain = _gate_of(_camera_actions(m, monkeypatch, tmp_path)).chain
    step1 = chain.after_counter()  # the counter is in the graph
    assert chain.reset in step1 and chain.waiter not in step1 and chain.planner not in step1
    step2 = chain.reset_exit(0)
    assert chain.waiter in step2 and chain.planner not in step2 and not _has_shutdown(step2)
    step3 = chain.waiter_exit(0)
    assert chain.planner in step3 and not _has_shutdown(step3)


@pytest.mark.parametrize("code", [1, 2, 124, -9])
def test_a_reset_that_failed_or_timed_out_is_a_refusal(monkeypatch, tmp_path, code) -> None:
    m = _module()
    chain = _gate_of(_camera_actions(m, monkeypatch, tmp_path)).chain
    out = chain.reset_exit(code)
    assert _has_shutdown(out) and chain.planner not in out and chain.waiter not in out
    text = _text(out)
    assert REFUSAL in text and f"exit code {code}" in text and "ERROR" in text


@pytest.mark.parametrize("code", [3, 1, -11])
def test_a_waiter_that_did_not_exit_0_refuses_and_the_planner_never_starts(
    monkeypatch, tmp_path, code
) -> None:
    m = _module()
    chain = _gate_of(_camera_actions(m, monkeypatch, tmp_path)).chain
    chain.capture_line(
        f"[lap_source_ready_wait]: {m.WAITER_LAST_LINE_PREFIX}similarity 0.5 below 0.9"
    )
    out = chain.waiter_exit(code)
    assert _has_shutdown(out) and chain.planner not in out
    text = _text(out)
    assert text.count(REFUSAL) == 1
    assert "similarity 0.5 below 0.9" in text  # the lap node's own reason


def test_a_refusal_with_no_captured_line_still_says_why(monkeypatch, tmp_path) -> None:
    m = _module()
    chain = _gate_of(_camera_actions(m, monkeypatch, tmp_path)).chain
    text = _text(chain.waiter_exit(3))
    assert REFUSAL in text and "exit code 3" in text


def test_without_a_reset_the_chain_goes_from_the_counter_to_the_waiter(
    monkeypatch, tmp_path
) -> None:
    m = _module()
    # the counter is run by someone else: this launch makes no reset call, the gate still holds
    actions = _camera_actions(m, monkeypatch, tmp_path, counter_in_graph=True, lap_counter="false")
    gate = _gate_of(actions)
    assert gate is not None and gate.chain.reset is None and not _planner_nodes(actions)
    out = gate.chain.after_counter()
    assert gate.chain.waiter in out and gate.chain.planner not in out


def test_the_counter_gate_hands_over_to_the_chain_when_the_counter_is_there(
    monkeypatch, tmp_path
) -> None:
    from launch.actions import TimerAction
    from launch.launch_context import LaunchContext

    m = _module()
    gate = _gate_of(_camera_actions(m, monkeypatch, tmp_path))
    probes = [False, True]
    monkeypatch.setattr(m, "lap_counter_in_graph", lambda *a, **k: probes.pop(0))
    out = gate.execute(LaunchContext())
    assert len(out) == 1 and isinstance(out[0], TimerAction)
    out = out[0].actions[0].execute(LaunchContext())
    assert gate.chain.reset in out and not _planner_nodes(out)


def test_the_shadow_camera_gates_the_planner_on_the_shadows_topic(monkeypatch, tmp_path) -> None:
    m = _module()
    actions = _camera_actions(
        m, monkeypatch, tmp_path, lap_source="start_scan", lap_shadow="camera"
    )
    assert not _planner_nodes(actions)
    assert _gate_of(actions).chain.status_topic == "/pnp/lap_shadow/status"


def test_start_scan_without_a_camera_shadow_starts_the_planner_as_today_and_has_no_waiter(
    monkeypatch, tmp_path
) -> None:
    m = _module()
    monkeypatch.setattr(m, "recorder_in_graph", lambda *a, **k: True)
    for source, shadow in (
        ("start_scan", ""),
        ("map_pose", ""),
        ("map_pose", "start_scan"),
        ("camera", "start_scan"),
    ):
        if source == "camera":
            continue  # a camera primary has the chain: covered above
        actions = m._make_actions(
            _context(
                lap_memory="true",
                lap_source=source,
                lap_shadow=shadow,
                lap_counter="false",
                dynamics_bundle=_bundle(tmp_path, 0.9),
            )
        )
        assert len(_planner_nodes(actions)) == 1, (source, shadow)
        assert not any(hasattr(g, "chain") for g in actions)
        assert not any("lap_source_ready_wait" in _cmd_text(p) for p in _processes(actions))


def test_lap_memory_false_has_no_chain_whatever_the_camera_says(monkeypatch) -> None:
    m = _module()
    monkeypatch.setattr(m, "recorder_in_graph", lambda *a, **k: True)
    for source in ("camera", "start_scan"):
        actions = m._make_actions(_context(lap_memory="false", lap_source=source))
        assert len(_planner_nodes(actions)) == 1
        assert not any(hasattr(g, "chain") for g in actions)


def test_nothing_in_the_chain_names_drive(monkeypatch, tmp_path) -> None:
    m = _module()
    chain = _gate_of(_camera_actions(m, monkeypatch, tmp_path)).chain
    text = " ".join(_cmd_text(p) for p in (chain.waiter, chain.reset))
    assert "/drive" not in text and "drive_raw" not in text


def test_the_waiter_is_told_the_reset_id_the_counter_acknowledged(monkeypatch, tmp_path) -> None:
    """Outside review finding 2: reset acknowledged, then fresh frames, then ready, then planner. The
    reset call's answer carries `reset_id=N`; the waiter ignores every status of an older reset."""
    m = _module()
    assert "min_reset_id:=4" in _cmd_text(m.lap_source_ready_waiter("/pnp/lap/status", 4))
    assert "min_reset_id" not in _cmd_text(m.lap_source_ready_waiter("/pnp/lap/status"))
    chain = _gate_of(_camera_actions(m, monkeypatch, tmp_path)).chain
    chain.capture_reset_text(
        "response:\nstd_srvs.srv.Trigger_Response(success=True, message='reset: the next frame "
        "becomes the reference, laps back to 0 (reset_id=3)')\n"
    )
    out = chain.reset_exit(0)
    waiter = next(x for x in out if type(x).__name__ == "ExecuteProcess")
    assert "min_reset_id:=3" in _cmd_text(waiter) and chain.planner not in out
    assert chain.waiter is waiter


def test_a_reset_that_names_no_id_still_waits_and_says_so(monkeypatch, tmp_path) -> None:
    m = _module()
    chain = _gate_of(_camera_actions(m, monkeypatch, tmp_path)).chain
    out = chain.reset_exit(0)
    waiter = next(x for x in out if type(x).__name__ == "ExecuteProcess")
    assert "min_reset_id" not in _cmd_text(waiter)
    assert "WARN" in _text(out) and "reset_id" in _text(out)


def test_the_reset_call_output_is_captured_for_its_id(monkeypatch, tmp_path) -> None:
    m = _module()
    chain = _gate_of(_camera_actions(m, monkeypatch, tmp_path)).chain
    out = chain.after_counter()
    assert any(type(x).__name__ == "RegisterEventHandler" for x in out)
    assert sum(type(x).__name__ == "RegisterEventHandler" for x in out) >= 2  # exit and output


# -- G6: the real launch machinery with a fake lap node -------------------------------------------------

rclpy = pytest.importorskip("rclpy")

from launch import LaunchDescription, LaunchService
from launch.actions import ExecuteProcess, OpaqueFunction, RegisterEventHandler
from launch.event_handlers import OnProcessExit
from rclpy.context import Context
from rclpy.qos import HistoryPolicy, QoSProfile, ReliabilityPolicy
from std_msgs.msg import String

TOPIC = "/pnp/lap/status"


def status(ready: bool, reason: str = "not ready") -> str:
    return json.dumps(
        {
            "lap_source": "camera",
            "phase": "waiting_for_driving",
            "source_ready_now": ready,
            "source_alive": ready,
            "source_not_ready_reason": "" if ready else reason,
        }
    )


class FakeLapNode:
    """A lap node's status stream in a context of its own: not ready until `ready_after_s`, ready
    after (never if None), and silent after `die_after_s` when it is given."""

    def __init__(self, ready_after_s: float | None, die_after_s: float | None = None) -> None:
        self.ctx = Context()
        rclpy.init(context=self.ctx)
        self.node = rclpy.create_node("fake_lap_node", context=self.ctx)
        self.pub = self.node.create_publisher(
            String,
            TOPIC,
            QoSProfile(
                reliability=ReliabilityPolicy.RELIABLE, history=HistoryPolicy.KEEP_LAST, depth=10
            ),
        )
        self.ready_after_s = ready_after_s
        self.die_after_s = die_after_s
        self.t0 = time.monotonic()
        self.ready_at: float | None = None
        self.stop = threading.Event()
        self.thread = threading.Thread(target=self.run, daemon=True)
        self.thread.start()

    def run(self) -> None:
        while not self.stop.is_set():
            t = time.monotonic() - self.t0
            if self.die_after_s is not None and t > self.die_after_s:
                return
            ready = self.ready_after_s is not None and t >= self.ready_after_s
            if ready and self.ready_at is None:
                self.ready_at = time.monotonic()
            self.pub.publish(String(data=status(ready, "similarity 0.50 below 0.90")))
            time.sleep(1.0)

    def close(self) -> None:
        self.stop.set()
        self.thread.join(timeout=3.0)
        self.node.destroy_node()
        rclpy.shutdown(context=self.ctx)


def run_chain(m, tmp_path: Path, reset_cmd: list[str], timeout_s: float):
    """The chain on a real LaunchService with the real waiter (python -m, no install needed), a
    reset that is `reset_cmd` and a planner that writes a marker file. Returns (planner marker
    time or None, the launch's output)."""
    marker = tmp_path / "planner.started"
    planner = ExecuteProcess(
        cmd=[sys.executable, "-c", f"open({str(marker)!r}, 'w').write('x')"], output="screen"
    )
    waiter = ExecuteProcess(
        cmd=[
            sys.executable,
            "-m",
            "racer_laps.lap_source_ready_wait",
            "--ros-args",
            "-p",
            f"status_topic:={TOPIC}",
        ],
        output="screen",
    )
    reset = ExecuteProcess(cmd=reset_cmd, output="screen")
    chain = m.CameraStartChain(planner, TOPIC, reset=reset, waiter=waiter)
    description = LaunchDescription(
        [
            RegisterEventHandler(
                OnProcessExit(target_action=planner, on_exit=[EmitEvent(event=Shutdown())])
            ),
            OpaqueFunction(function=lambda context: chain.after_counter()),
        ]
    )
    service = LaunchService(noninteractive=True)
    service.include_launch_description(description)
    records: list[str] = []

    class Collect(logging.Handler):
        def emit(self, record: logging.LogRecord) -> None:
            records.append(record.getMessage())

    collector = Collect()
    user_log = logging.getLogger("launch.user")
    user_log.addHandler(collector)
    # LaunchService runs in the main thread; a timer ends it if the chain does not
    timer = threading.Timer(timeout_s, service.shutdown)
    timer.start()
    started = time.monotonic()
    try:
        service.run()
    finally:
        timer.cancel()
        user_log.removeHandler(collector)
    finished = time.monotonic() - started < timeout_s
    return (marker.stat().st_mtime if marker.exists() else None), finished, "\n".join(records)


def test_g6_ready_after_8_s_starts_the_planner_after_it(tmp_path, capfd) -> None:
    m = _module()
    fake = FakeLapNode(ready_after_s=8.0)
    try:
        started, finished, _log = run_chain(m, tmp_path, ["true"], timeout_s=60.0)
    finally:
        fake.close()
    assert finished and started is not None
    assert fake.ready_at is not None and started > fake.ready_at  # after the source said ready


def test_g6_never_ready_refuses_within_35_s_with_the_reason_and_no_planner(tmp_path, capfd) -> None:
    m = _module()
    fake = FakeLapNode(ready_after_s=None)
    t0 = time.monotonic()
    try:
        started, finished, log = run_chain(m, tmp_path, ["true"], timeout_s=60.0)
    finally:
        fake.close()
    elapsed = time.monotonic() - t0
    text = log
    assert finished and started is None  # the planner process was never created
    assert elapsed <= 35.0 + 3.0  # the 30 s wait and the launch's own start and stop
    assert REFUSAL in text and "similarity 0.50 below 0.90" in text


def test_g6_a_reset_that_fails_refuses(tmp_path, capfd) -> None:
    m = _module()
    fake = FakeLapNode(ready_after_s=0.0)
    try:
        started, finished, log = run_chain(m, tmp_path, ["false"], timeout_s=30.0)
    finally:
        fake.close()
    assert finished and started is None
    assert REFUSAL in log and "reset" in log


def test_g6_a_lap_node_that_dies_after_one_ready_status_refuses(tmp_path, capfd) -> None:
    m = _module()
    fake = FakeLapNode(ready_after_s=0.0, die_after_s=0.5)  # one ready status, then silence
    try:
        started, finished, log = run_chain(m, tmp_path, ["true"], timeout_s=60.0)
    finally:
        fake.close()
    assert finished and started is None
    assert REFUSAL in log
