"""L1: S6, the lap engine says so when the car does not answer at the start. After the start, gated
speed commands of 0.1 m/s or more for over `start_unanswered_s` with no wheel motion mark the run
`start_unanswered` and every lap of it doubtful (as a start that was not ready). A pause shorter
than the window, a car that answers, and a standing command do not."""

from __future__ import annotations

import math

from external_support import DT, EngineRig, steering_for
from racer_laps.engine import LapConfig
from racer_laps.external_gate import DOUBTFUL_REASONS


def run(rig: EngineRig, seconds: float, cmd: float, wheel: float) -> None:
    """`seconds` of 50 Hz odometry at `wheel`, with /drive speed `cmd` and the node clock."""
    end = rig.k + round(seconds / DT)
    while rig.k < end:
        rig.k += 1
        t = rig.k * DT
        rig.mem.on_wheel_odom(t, wheel)
        rig.mem.on_drive(steering_for(rig.lap_m), cmd, t)
        rig.mem.on_clock(t)


def test_the_window_is_a_declared_parameter() -> None:
    assert LapConfig.__dataclass_fields__["start_unanswered_s"].default == 3.0
    assert "start_unanswered" in DOUBTFUL_REASONS


def test_commands_with_no_wheel_motion_over_the_window_mark_the_run() -> None:
    rig = EngineRig(speed=0.0)
    run(rig, 2.9, cmd=0.98, wheel=0.0)
    assert rig.mem.status()["start_unanswered"] is False
    run(rig, 0.3, cmd=0.98, wheel=0.0)
    s = rig.mem.status()
    assert s["start_unanswered"] is True
    assert rig.mem.start_unanswered


def test_every_lap_of_an_unanswered_run_is_doubtful() -> None:
    rig = EngineRig(speed=0.0)
    run(rig, 3.5, cmd=0.98, wheel=0.0)
    assert rig.mem.start_unanswered
    rig.speed = 1.0
    t0 = rig.t
    for i in range(3):
        rig.event(t0 + rig.lap_m * (i + 1), i + 1)
    doubtful = {d["lap"]: d["reasons"] for d in rig.mem.status()["doubtful_laps"]}
    assert sorted(doubtful) == [1, 2, 3]
    assert all("start_unanswered" in r for r in doubtful.values())
    assert not any(r.clean for r in rig.mem.records)


def test_a_reverse_command_is_a_command_too() -> None:
    rig = EngineRig(speed=0.0)
    run(rig, 3.3, cmd=-0.5, wheel=0.0)
    assert rig.mem.start_unanswered


def test_a_normal_start_does_not_mark_the_run() -> None:
    rig = EngineRig(speed=1.0)
    run(rig, 1.0, cmd=1.0, wheel=1.0)
    rig.laps(2)
    run(rig, 3.0, cmd=1.0, wheel=1.0)
    assert rig.mem.status()["start_unanswered"] is False
    for d in rig.mem.status()["doubtful_laps"]:
        assert "start_unanswered" not in d["reasons"]
    assert rig.mem.status()["confirmed_laps"] == 2


def test_a_short_pause_before_the_wheels_move_does_not_mark_the_run() -> None:
    rig = EngineRig(speed=0.0)
    run(rig, 2.5, cmd=0.98, wheel=0.0)  # under 3 s of commands with the car standing
    run(rig, 5.0, cmd=0.98, wheel=0.9)  # then it answers
    run(rig, 5.0, cmd=0.98, wheel=0.0)  # a later stall is not a start problem
    assert rig.mem.status()["start_unanswered"] is False


def test_a_command_pause_restarts_the_window() -> None:
    rig = EngineRig(speed=0.0)
    run(rig, 2.0, cmd=0.98, wheel=0.0)
    run(rig, 2.0, cmd=0.0, wheel=0.0)  # the planner stood still: no command to answer
    run(rig, 2.0, cmd=0.98, wheel=0.0)
    assert rig.mem.status()["start_unanswered"] is False


def test_a_silent_drive_topic_is_not_an_unanswered_start() -> None:
    rig = EngineRig(speed=0.0)
    for _ in range(400):
        rig.k += 1
        rig.mem.on_wheel_odom(rig.k * DT, 0.0)
        rig.mem.on_clock(rig.k * DT)
    assert rig.mem.status()["start_unanswered"] is False


def test_the_window_comes_from_the_config() -> None:
    from external_support import memory

    assert math.isclose(memory("camera").cfg.start_unanswered_s, 3.0)
