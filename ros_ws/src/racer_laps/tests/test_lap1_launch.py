"""L1: lap 1 is the launch lap (brief 48 item 2, design 13 section 19.2). Its start is the effective
time of the first scan after the planner's first DRIVING report, with the car standing, and its time
includes the standing dwell and the launch. The status says where it started and when the car first
moved; the replay tool starts it where the node does; one standstill constant serves all of them.
`LapEvent` and `LapGuidance` are unchanged."""

from __future__ import annotations

import hashlib
import inspect
import math
from pathlib import Path
from types import SimpleNamespace as NS

import pytest
from racer_laps import gate, replay
from racer_laps.engine import LapConfig, LapMemory
from racer_laps.gate import STANDSTILL_MPS, GateConfig
from racer_laps.lap_record import STATE_DRIVING
from racer_laps.scan_match import scan_to_points
from synth import WHEELBASE_M, Stadium
from test_gate_v21 import LAP_M, PTS, Car
from test_replay_scan import VEHICLE, messages

KEYS = ("lap1_start_stamp_s", "first_motion_stamp_s", "launch_dwell_s")
MSG_DIR = Path(__file__).resolve().parents[2] / "racer_pnp_msgs" / "msg"


def fresh() -> LapMemory:
    cfg = LapConfig(lap_source="start_scan", gate=GateConfig(wheelbase_m=WHEELBASE_M))
    return LapMemory(cfg)


def test_one_standstill_constant_serves_the_gate_and_the_replay() -> None:
    assert STANDSTILL_MPS == 0.05
    assert gate.STANDSTILL_MPS == STANDSTILL_MPS
    default = inspect.signature(replay.find_start_time).parameters["standstill_mps"].default
    assert default == STANDSTILL_MPS


def test_the_status_reports_the_launch_nominally() -> None:
    mem = fresh()
    mem.on_planner_status(STATE_DRIVING, 0.3)
    mem.on_wheel_odom(0.00, 0.0)
    mem.on_wheel_odom(0.02, 0.0)
    assert mem.on_scan(0.04, PTS) is None  # the start scan: the first one after DRIVING
    for k in range(3, 14):  # standing until 0.26 s
        mem.on_wheel_odom(0.02 * k, 0.0)
    mem.on_wheel_odom(0.28, 0.049)  # just under the threshold: still standing
    mem.on_wheel_odom(0.30, STANDSTILL_MPS)  # at the threshold: moving
    mem.on_wheel_odom(0.32, 0.2)
    status = mem.status()
    assert status["lap1_start_stamp_s"] == pytest.approx(0.04)
    assert status["first_motion_stamp_s"] == pytest.approx(0.30)
    assert status["launch_dwell_s"] == pytest.approx(0.26)


def test_the_launch_keys_are_empty_before_the_start_scan_and_after_silence() -> None:
    mem = fresh()
    assert all(k in mem.status() and math.isnan(mem.status()[k]) for k in KEYS)  # type: ignore[arg-type]
    mem.on_planner_status(STATE_DRIVING, 0.3)
    mem.on_wheel_odom(0.0, 0.0)
    assert all(math.isnan(mem.status()[k]) for k in KEYS)  # type: ignore[arg-type]  # DRIVING only
    mem.on_scan(0.04, PTS)
    status = mem.status()
    assert status["lap1_start_stamp_s"] == pytest.approx(0.04)
    # odometry goes silent after the start scan: the car never seen to move
    assert math.isnan(status["first_motion_stamp_s"]) and math.isnan(status["launch_dwell_s"])  # type: ignore[arg-type]
    # a sample before the start scan's instant is not the motion after it
    mem.on_wheel_odom(0.02, 0.5)
    assert math.isnan(mem.status()["first_motion_stamp_s"])  # type: ignore[arg-type]


def test_the_status_keys_are_json_safe_through_the_node_rule() -> None:
    """The node publishes `math.nan` as null (`_json_safe`); the engine keeps them floats."""
    status = fresh().status()
    for k in KEYS:
        assert isinstance(status[k], float)


def test_lap_1_carries_the_launch_dwell_and_later_laps_zero(
    monkeypatch: pytest.MonkeyPatch,
) -> None:
    car = Car(monkeypatch)
    for _ in range(10):  # 0.2 s standing after the start scan
        car.step(0.0)
    first = car.lap(LAP_M)
    second = car.lap(LAP_M)
    assert first is not None and second is not None
    assert first.launch_dwell_s == pytest.approx(0.22, abs=0.03)
    assert second.launch_dwell_s == 0.0
    # lap 1 contains the dwell: the lap time is the standing plus the drive
    assert first.lap_time_s - second.lap_time_s == pytest.approx(first.launch_dwell_s, abs=0.1)


# -- the replay starts lap 1 where the node does --------------------------------------------------


def node_rule(msgs: list[tuple]) -> tuple[LapMemory, list]:
    """Every message from the start of the segment in log order, as the node receives it."""
    mem = LapMemory(replay.default_start_scan_config(VEHICLE))
    laps = []
    for _t_log, topic, msg in msgs:
        if topic == replay.ODOM_TOPIC:
            mem.on_wheel_odom(
                msg.header.stamp.sec + msg.header.stamp.nanosec * 1e-9,
                float(msg.twist.twist.linear.x),
            )
        elif topic == replay.DRIVE_TOPIC:
            mem.on_drive(float(msg.drive.steering_angle))
        elif topic == replay.PLANNER_TOPIC:
            mem.on_planner_status(int(msg.state), float(msg.plan_min_clearance_m))
        elif topic == replay.SCAN_TOPIC:
            stamp = msg.header.stamp.sec + msg.header.stamp.nanosec * 1e-9
            pts = scan_to_points(
                msg.ranges,
                float(msg.angle_min),
                float(msg.angle_increment),
                float(msg.range_min),
                float(msg.range_max),
                VEHICLE.mount,
                VEHICLE.body,
            )
            lap = mem.on_scan(stamp + 0.5 * float(msg.scan_time), pts)
            if lap is not None:
                laps.append(lap)
    return mem, laps


def driving_before_the_first_motion(lead_s: float) -> list[tuple]:
    """1.0 s of standstill, then the car moves; the planner reports DRIVING `lead_s` before."""
    msgs = messages(2.4, standstill_s=1.0)
    t_motion = 1000.0 + 1.0
    msgs.append(
        (
            t_motion - lead_s,
            "/pnp/planner/status",
            NS(state=STATE_DRIVING, plan_min_clearance_m=0.3),
        )
    )
    msgs.sort(key=lambda m: m[0])
    return msgs


def test_the_replay_starts_lap_1_at_the_nodes_start_scan_and_lap_1_contains_the_dwell() -> None:
    """DRIVING 0.27 s before the first motion. The replay used to drop everything before the last
    standstill odometry sample, so its start scan was later than the node's and its lap 1 shorter
    by the dwell (the GP3-R1 bag: 5.70 s against 5.99 s)."""
    msgs = driving_before_the_first_motion(0.27)
    node_mem, node_laps = node_rule(msgs)
    result = replay.run_replay_start_scan(msgs, VEHICLE)
    node_status = node_mem.status()
    assert not math.isnan(node_status["lap1_start_stamp_s"])  # type: ignore[arg-type]
    assert result.status["lap1_start_stamp_s"] == node_status["lap1_start_stamp_s"]
    assert result.status["first_motion_stamp_s"] == node_status["first_motion_stamp_s"]
    assert [lap.lap_index for lap in result.laps] == [lap.lap_index for lap in node_laps]
    assert result.laps[0].lap_time_s == pytest.approx(node_laps[0].lap_time_s, abs=1e-9)
    # the dwell is in it: the start scan is before the motion, by about the lead
    dwell = result.laps[0].launch_dwell_s
    assert 0.2 < dwell < 0.45
    period = Stadium().length_m
    assert result.laps[0].lap_time_s == pytest.approx(period + dwell, abs=0.3)
    # the log time of the start scan is reported
    assert result.start_t_s is not None and result.start_t_s < 1001.0


def test_a_segment_without_planner_status_keeps_the_standstill_start_rule() -> None:
    msgs = messages(2.4, standstill_s=1.0)
    result = replay.run_replay_start_scan(msgs, VEHICLE)
    assert result.start_t_s == pytest.approx(1000.0 + 0.98, abs=0.03)
    assert result.laps


# -- the messages are unchanged -----------------------------------------------------------------


def test_lap_event_and_lap_guidance_definitions_are_unchanged() -> None:
    """md5 of the .msg files at the time of brief 48 (new fields go in the status JSON)."""
    digests = {
        name: hashlib.md5((MSG_DIR / f"{name}.msg").read_bytes()).hexdigest()
        for name in ("LapEvent", "LapGuidance")
    }
    assert digests == {
        "LapEvent": "ba79fcf3b4720849edd8c16b9c300c7c",
        "LapGuidance": "833d1b7aee456dfc685f2b9750de39b6",
    }
