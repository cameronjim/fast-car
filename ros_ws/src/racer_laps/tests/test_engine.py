"""L1: the engine (detector + recorder + schedule + safe set) on a synthetic oval."""

from __future__ import annotations

import math

import pytest
from racer_laps.engine import PHASE_ENGAGE, SEVERITY_BRAKE, LapConfig, LapMemory, LapResult
from racer_laps.lap_record import STATE_DRIVING
from tracks import circle_poses

NO_TRACK = 2
TWO_PI = 2.0 * math.pi


def drive(
    mem: LapMemory,
    loops: float,
    *,
    clearance: float = 0.3,
    on_pose: object = None,
    start_driving: bool = True,
) -> list[LapResult]:
    if start_driving:
        mem.on_planner_status(STATE_DRIVING, clearance)
    out: list[LapResult] = []
    for t, x, y, yaw in circle_poses(loops, phase_s=0.011):
        mem.on_wheel_speed(1.0)
        mem.on_drive_raw(1.0, 0.1)
        mem.on_planner_status(STATE_DRIVING, clearance)
        if callable(on_pose):
            on_pose(mem, t)
        r = mem.on_pose(t, x, y, yaw)
        if r is not None:
            out.append(r)
    return out


def cfg() -> LapConfig:
    return LapConfig(v_cap_start_mps=0.5, v_cap_step_mps=0.1, v_cap_ceiling_mps=0.8)


def test_no_start_line_until_the_planner_reports_driving() -> None:
    mem = LapMemory(cfg())
    assert mem.on_pose(0.0, 1.0, 0.0, 0.0) is None
    assert mem.status()["phase"] == "waiting_for_driving"
    assert mem.guidance().lap_index == 0
    mem.on_planner_status(NO_TRACK, 1.0)
    assert mem.status()["phase"] == "waiting_for_driving"
    mem.on_planner_status(STATE_DRIVING, 1.0)
    assert mem.status()["phase"] == "waiting_for_pose"
    mem.on_pose(1.0, 1.0, 0.0, 0.0)
    assert mem.status()["phase"] == "running"
    assert mem.guidance().lap_index == 1


def test_clean_laps_raise_the_cap_and_fill_the_safe_set() -> None:
    mem = LapMemory(cfg())
    results = drive(mem, 4.0)
    assert [r.lap_index for r in results] == [1, 2, 3, 4]
    assert [r.clean for r in results] == [True] * 4
    assert [round(r.cap_mps, 6) for r in results] == [0.5, 0.6, 0.7, 0.8]
    assert results[-1].next_cap_mps == pytest.approx(0.8)  # ceiling
    assert mem.safe_set.n_laps == 4
    assert all(abs(r.lap_time_s - TWO_PI) < 0.02 for r in results)
    assert len(mem.records) == 4


def test_a_brake_makes_the_lap_unclean_and_drops_the_cap() -> None:
    mem = LapMemory(cfg())
    braked = {"done": False}

    def brake_once(m: LapMemory, t: float) -> None:
        if t > 9.0 and not braked["done"]:  # during lap 2
            braked["done"] = True
            m.on_safety_event(PHASE_ENGAGE, SEVERITY_BRAKE)

    results = drive(mem, 3.0, on_pose=brake_once)
    assert [r.clean for r in results] == [True, False, True]
    assert results[1].safety_brakes == 1
    assert [round(r.cap_mps, 6) for r in results] == [0.5, 0.6, 0.5]
    assert mem.safe_set.n_laps == 2
    assert not results[1].stored


def test_release_and_info_events_do_not_count() -> None:
    mem = LapMemory(cfg())
    mem.on_planner_status(STATE_DRIVING, 0.3)
    mem.on_pose(0.0, 1.0, 0.0, math.pi / 2)
    mem.on_safety_event(1, SEVERITY_BRAKE)  # PHASE_RELEASE
    mem.on_safety_event(PHASE_ENGAGE, 0)  # INFO
    mem.on_safety_event(PHASE_ENGAGE, 1)  # WARNING
    assert mem._recorder.brake_count == 0


def test_planner_leaving_driving_or_low_clearance_makes_the_lap_unclean() -> None:
    mem = LapMemory(cfg())

    def blip(m: LapMemory, t: float) -> None:
        if 1.0 < t < 1.1:
            m.on_planner_status(NO_TRACK, 0.3)

    assert [r.clean for r in drive(mem, 1.0, on_pose=blip)] == [False]
    assert drive(LapMemory(cfg()), 1.0, clearance=0.05)[0].clean is False


def test_events_before_the_start_line_do_not_count() -> None:
    mem = LapMemory(cfg())
    mem.on_safety_event(PHASE_ENGAGE, SEVERITY_BRAKE)
    mem.on_planner_status(NO_TRACK, 0.0)
    assert drive(mem, 1.0)[0].clean is True


def test_guidance_has_safe_set_arrays_from_the_second_lap() -> None:
    mem = LapMemory(cfg())
    g = mem.guidance()
    assert (g.lap_index, g.v_cap_mps, g.s_m.shape[0]) == (0, 0.5, 0)
    first: list[LapResult] = []
    seen: dict[int, int] = {}

    def watch(m: LapMemory, t: float) -> None:
        g = m.guidance()
        seen[g.lap_index] = g.s_m.shape[0]

    first = drive(mem, 2.5, on_pose=watch)
    assert len(first) == 2
    assert seen[1] == 0  # first lap: cap only
    assert seen[3] > 0  # third lap has two stored laps behind it
    g = mem.guidance()
    assert g.lap_index == 3
    assert g.s_m.shape == g.v_ref_mps.shape == g.cost_to_go_s.shape
    assert g.s_m[0] >= mem.detector.path_since_crossing_m - 1e-9
    assert g.s_m[-1] - g.s_m[0] <= 3.0 + 1e-9
    assert g.v_cap_mps == pytest.approx(0.7)


def test_missing_pose_holds_state_and_never_invents_a_lap() -> None:
    mem = LapMemory(cfg())
    mem.on_planner_status(STATE_DRIVING, 0.3)
    mem.on_pose(0.0, 1.0, 0.0, math.pi / 2)
    for _ in range(1000):
        mem.on_pose_missing()
    assert mem.status()["pose_ok"] is False
    assert mem.detector.laps_completed == 0
    mem.on_pose(0.05, 1.0, 0.05, math.pi / 2)
    assert mem.status()["pose_ok"] is True


def test_old_poses_are_dropped() -> None:
    mem = LapMemory(cfg())
    mem.on_planner_status(STATE_DRIVING, 0.3)
    mem.on_pose(1.0, 1.0, 0.0, 0.0)
    assert mem.on_pose(1.0, 5.0, 5.0, 0.0) is None
    assert mem.on_pose(0.5, 5.0, 5.0, 0.0) is None
    assert mem._recorder.n_samples == 1


def test_a_lap_that_ends_while_the_planner_is_blocked_starts_the_next_one_unclean() -> None:
    mem = LapMemory(cfg())
    mem.on_planner_status(STATE_DRIVING, 0.3)
    results: list[LapResult] = []
    for t, x, y, yaw in circle_poses(1.5, phase_s=0.011):
        if t > TWO_PI - 0.01:
            mem.on_planner_status(5, 0.3)  # BLOCKED, set just before the crossing sample
        r = mem.on_pose(t, x, y, yaw)
        if r:
            results.append(r)
    assert len(results) == 1
    assert mem._recorder._non_driving is True  # the next lap carries it
