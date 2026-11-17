"""L1: the lap engine on an external lap source (brief 51): the engine's whole public surface
against an `ExternalLapGate`, the cap schedule through the missed-gate path, the lap-1 cap while
overdue, observe mode, and a brake in the latency window."""

from __future__ import annotations

import math

import pytest
from external_support import DT, LAP_M, EngineRig, memory, odom_model, steering_for
from racer_laps.engine import LAP_SOURCES, LapConfig, LapMemory
from racer_laps.external_gate import ExternalConfig, ExternalLapGate
from racer_laps.lap_record import STATE_DRIVING

CAP0 = 0.5
STEP = 0.1


# -- the engine's public surface, with every method and property the node calls ---------------------


def test_lap_sources_gain_camera_and_truth() -> None:
    assert LAP_SOURCES == ("start_scan", "map_pose", "camera", "truth")


@pytest.mark.parametrize("source", ["camera", "truth"])
def test_the_public_surface_runs_against_an_external_gate(source: str) -> None:
    mem = memory(source)
    assert isinstance(mem.gate, ExternalLapGate)
    # everything lap_memory_node calls, before the start
    mem.on_planner_status(0, 0.3)
    mem.on_safety_event(0, 2, "ttc")  # not started: ignored
    mem.on_wheel_speed(0.0)
    mem.on_drive_raw(0.5, 0.1)
    mem.on_drive(0.0)
    mem.on_pose_missing()
    assert mem.on_scan(0.0, __import__("numpy").zeros((5, 2))) is None
    assert mem.on_pose(1.0, 0.0, 0.0, 0.0) is None
    assert mem.on_lap_source_event(1.0, 1, 0.9) is None  # before the start: rejected
    mem.on_lap_source_status(0.5, "running", 0, 1.0)
    mem.on_clock(0.6)
    assert mem.laps_completed == 0
    assert mem.lap_in_progress == 0
    assert math.isnan(mem.station_m)
    assert mem.crosscheck == "off"
    g = mem.guidance()
    assert g.lap_index == 0 and g.v_cap_mps == CAP0 and g.s_m.shape == (0,)
    s = mem.status()
    assert s["phase"] == "waiting_for_driving" and s["lap_source"] == source
    # the start: the planner reports DRIVING, the next odometry sample starts lap 1
    mem.on_planner_status(STATE_DRIVING, 0.3)
    assert s["phase"] == "waiting_for_driving"
    mem.on_wheel_odom(1.0, 0.0)
    assert mem.lap_in_progress == 1
    s = mem.status()
    assert s["phase"] == "running" and s["why_no_lap"] == "gate_not_armed"
    assert not math.isnan(mem.station_m)
    # the gate's side, by name: what the engine reads
    gate = mem.gate
    for name in (
        "started",
        "laps_completed",
        "l_ref_m",
        "l_ref_trusted",
        "suspect_laps",
        "station_m",
        "armed",
        "on_odom",
        "on_steering",
        "poll_uncertain",
        "status",
    ):
        assert hasattr(gate, name), name
    assert gate.suspect_laps == []
    # JSON-able status keys of the new sources
    for key in (
        "source_ready",
        "source_ready_now",
        "source_not_ready_reason",
        "source_alive",
        "source_status_age_s",
        "confirmed_laps",
        "doubtful_laps",
        "rejected_events",
        "event_latency_s_last",
        "event_latency_s_max",
        "line_source",
        "station_m",
        "l_ref_m",
        "l_ref_trusted",
        "station_uncertain",
        "missed_gate_laps",
    ):
        assert key in s, key
    assert s["line_source"] == source


def test_a_camera_config_needs_the_odometry_model() -> None:
    with pytest.raises(ValueError):
        LapConfig(lap_source="camera")


@pytest.mark.parametrize("source", ["camera", "truth"])
def test_lap_mode_v1_is_refused_with_an_external_source(source: str) -> None:
    with pytest.raises(ValueError):
        LapConfig(lap_source=source, lap_mode="v1", gate=odom_model())


def test_external_config_is_used_when_given() -> None:
    mem = memory("camera", ext=ExternalConfig(max_event_latency_s=1.0))
    assert isinstance(mem.gate, ExternalLapGate) and mem.gate.cfg.max_event_latency_s == 1.0
    # and the node's min_lap_length_m flows into the default one
    cfg = LapConfig(lap_source="camera", gate=odom_model(), min_lap_length_m=4.0)
    assert isinstance(LapMemory(cfg).gate, ExternalLapGate)
    assert LapMemory(cfg).gate.cfg.min_lap_length_m == 4.0  # type: ignore[union-attr]


# -- the cap schedule through the missed-gate path ----------------------------------------------------


def test_a_confirmed_clean_lap_raises_the_cap_one_step() -> None:
    rig = EngineRig()
    r = rig.event(LAP_M, 1)
    assert r is not None
    assert (r.clean, r.missed_gate, r.stored) == (True, False, True)
    assert r.source == "camera" and r.line_source == "camera"
    assert r.cap_mps == CAP0
    assert r.next_cap_mps == pytest.approx(CAP0 + STEP)
    assert rig.mem.guidance().v_cap_mps == pytest.approx(CAP0 + STEP)
    assert r.lap_time_s == pytest.approx(LAP_M) and r.length_m == pytest.approx(LAP_M, abs=1e-6)
    assert r.t_cross_s == pytest.approx(LAP_M)
    assert r.heading_rad == pytest.approx(2 * math.pi, abs=1e-6)


def test_a_doubtful_lap_lowers_the_cap_is_not_stored_and_two_clean_laps_must_follow() -> None:
    rig = EngineRig()
    rig.laps(3)  # caps 0.5 -> 0.6 -> 0.7 -> 0.8
    assert rig.mem.guidance().v_cap_mps == pytest.approx(0.8)
    stored_before = rig.mem.station_set.n_laps
    bad = rig.event(4 * LAP_M, 6)  # a skipped event number: doubtful
    assert bad is not None
    assert bad.missed_gate and not bad.clean and not bad.stored
    assert rig.mem.station_set.n_laps == stored_before
    assert bad.cap_mps == pytest.approx(0.8)  # the cap it was driven under
    assert bad.next_cap_mps == pytest.approx(0.7)  # one step down
    nxt = rig.event(5 * LAP_M, 7)  # the next lap, confirmed and clean: no rise yet
    assert nxt is not None and nxt.clean and not nxt.missed_gate
    assert nxt.next_cap_mps == pytest.approx(0.7)
    nxt2 = rig.event(6 * LAP_M, 8)  # the second clean lap raises it
    assert nxt2 is not None and nxt2.next_cap_mps == pytest.approx(0.8)


def test_a_doubtful_lap_never_goes_below_the_lap_one_cap() -> None:
    rig = EngineRig()
    r = rig.event(LAP_M, 5)  # doubtful at the start
    assert r is not None and r.missed_gate and r.next_cap_mps == CAP0
    r = rig.event(2 * LAP_M, 6)
    assert r is not None and r.next_cap_mps == CAP0


def test_a_run_that_started_not_ready_stays_at_the_lap_one_cap() -> None:
    rig = EngineRig(ready=False)
    for lap in range(1, 6):
        r = rig.event(lap * LAP_M, lap)
        assert r is not None and r.missed_gate and not r.clean
        assert r.next_cap_mps == CAP0
    assert set(rig.caps) == {CAP0}
    assert rig.mem.status()["confirmed_laps"] == 0
    assert rig.mem.station_set.n_laps == 0


def test_no_doubtful_lap_is_stored_or_raises_the_cap_with_any_pattern() -> None:
    """A mixed run: caps only rise after confirmed clean laps."""
    rig = EngineRig()
    numbers = [1, 2, 4, 5, 6, 7, 8]  # lap 3 skips an event number
    for i, no in enumerate(numbers, start=1):
        r = rig.event(i * LAP_M, no)
        assert r is not None
        if r.missed_gate:
            assert r.next_cap_mps <= r.cap_mps and not r.stored
        else:
            assert r.clean
    assert [d["lap"] for d in rig.mem.gate.doubtful] == [3]  # type: ignore[union-attr]


# -- overdue and a dead source: the lap-1 cap ---------------------------------------------------------


def test_while_overdue_the_guidance_cap_is_the_lap_one_cap() -> None:
    rig = EngineRig()
    rig.laps(3)
    assert rig.mem.guidance().v_cap_mps == pytest.approx(0.8)
    rig.drive_until(3 * LAP_M + LAP_M + 0.3)  # inside the band: still 0.8
    assert rig.mem.guidance().v_cap_mps == pytest.approx(0.8)
    rig.drive_until(3 * LAP_M + LAP_M + 0.7)  # past reference plus band: overdue
    assert rig.mem.guidance().v_cap_mps == CAP0
    assert rig.mem.status()["station_uncertain"] is True
    # the cap stays down until the next crossing, then the missed lap steps the schedule
    r = rig.event(4 * LAP_M + 1.0, 4)  # a long lap: doubtful by length
    assert r is not None and r.missed_gate
    assert rig.mem.guidance().v_cap_mps == pytest.approx(0.7)


def test_a_dead_source_leaves_the_car_at_the_lap_one_cap() -> None:
    rig = EngineRig(status_period_s=None)
    rig.mem.on_lap_source_status(0.0, "running", 0, 1.0)
    rig.laps(2)  # status silent: these laps are doubtful (alive), caps never rise
    assert set(rig.caps) == {CAP0}
    rig2 = EngineRig()
    rig2.laps(3)
    assert rig2.mem.guidance().v_cap_mps == pytest.approx(0.8)
    rig2.status_period_s = None  # the camera dies
    rig2.drive_until(rig2.t + 3.5)
    assert rig2.mem.guidance().v_cap_mps == CAP0
    rig2.mem.on_lap_source_status(rig2.t, "running", 0, 1.0)  # it returns: still latched
    assert rig2.mem.guidance().v_cap_mps == CAP0


# -- observe ------------------------------------------------------------------------------------------


def test_observe_keeps_the_cap_at_the_start_value() -> None:
    rig = EngineRig(mode="observe")
    for lap in range(1, 4):
        r = rig.event(lap * LAP_M, lap)
        assert r is not None and r.clean and r.next_cap_mps == CAP0
    assert set(rig.caps) == {CAP0}
    assert rig.mem.guidance().v_cap_mps == CAP0


# -- the latency window -------------------------------------------------------------------------------


def test_a_brake_in_the_latency_window_is_counted_in_exactly_one_lap() -> None:
    rig = EngineRig()
    rig.drive_until(LAP_M + 0.1)  # the car is 0.1 s past the line, the event is not here yet
    rig.mem.on_safety_event(0, 2, "ttc")  # an obstacle brake
    r1 = rig.event(LAP_M, 1, latency_s=0.26)
    assert r1 is not None
    r2 = rig.event(2 * LAP_M, 2)
    assert r2 is not None
    assert r1.obstacle_brakes + r2.obstacle_brakes == 1
    assert (r1.obstacle_brakes, r2.obstacle_brakes) == (1, 0)  # the closing lap has it
    assert not r1.clean and r2.clean


def test_a_brake_after_the_arrival_belongs_to_the_next_lap() -> None:
    rig = EngineRig()
    r1 = rig.event(LAP_M, 1, latency_s=0.26)
    rig.mem.on_safety_event(0, 2, "ttc")
    r2 = rig.event(2 * LAP_M, 2)
    assert r1 is not None and r2 is not None
    assert (r1.obstacle_brakes, r2.obstacle_brakes) == (0, 1)


# -- the start ----------------------------------------------------------------------------------------


def test_the_start_is_the_first_odometry_sample_after_the_driving_report() -> None:
    mem = memory()
    mem.on_drive(steering_for(LAP_M))
    mem.on_wheel_odom(0.0, 0.0)  # before DRIVING: no start
    mem.on_wheel_odom(0.02, 0.0)
    assert mem.lap_in_progress == 0
    mem.on_planner_status(STATE_DRIVING, 0.3)
    assert mem.lap_in_progress == 0  # not by the report alone
    mem.on_wheel_odom(0.04, 0.0)
    assert mem.lap_in_progress == 1
    s = mem.status()
    assert s["lap1_start_stamp_s"] == pytest.approx(0.04)


def test_the_launch_dwell_is_filled_the_same_way() -> None:
    mem = memory()
    mem.on_drive(steering_for(LAP_M))
    mem.on_lap_source_status(0.0, "running", 0, 1.0)
    mem.on_planner_status(STATE_DRIVING, 0.3)
    t = 0.0
    for _ in range(100):  # standing for 2 s
        mem.on_wheel_odom(t, 0.0)
        t += DT
    assert math.isnan(mem.launch_dwell_s)
    for i in range(300):
        mem.on_wheel_odom(t, 1.0)
        t += DT
        if i % 50 == 0:
            mem.on_lap_source_status(t, "running", 0, 1.0)
    assert mem.launch_dwell_s == pytest.approx(2.0, abs=0.03)
    r = mem.on_lap_source_event(t, 1, t - 0.1)
    assert r is not None and r.launch_dwell_s == pytest.approx(2.0, abs=0.03)
    assert mem.status()["first_motion_stamp_s"] == pytest.approx(2.0, abs=0.03)


def test_a_status_alone_never_counts_a_lap() -> None:
    rig = EngineRig()
    rig.drive_until(2 * LAP_M)
    rig.mem.on_lap_source_status(rig.t, "running", 7, 1.0)  # "laps 7" counts for nothing
    assert rig.mem.laps_completed == 0
    assert rig.results == []


def test_the_start_scan_source_is_untouched() -> None:
    """Events and statuses of a source are ignored by the start-scan engine."""
    from racer_laps.gate import GateConfig

    mem = LapMemory(LapConfig(lap_source="start_scan", gate=GateConfig(wheelbase_m=0.33)))
    assert mem.on_lap_source_event(1.0, 1, 0.9) is None
    mem.on_lap_source_status(1.0, "running", 0, 1.0)
    mem.on_clock(1.0)
    assert mem.laps_completed == 0


# -- the messages are byte-identical ----------------------------------------------------------------


def test_lap_messages_are_unchanged() -> None:
    """Brief 51 changes no message: new fields go in the JSON of /pnp/lap/status."""
    import hashlib
    from pathlib import Path

    msgs = Path(__file__).resolve().parents[2] / "racer_pnp_msgs" / "msg"
    if not msgs.is_dir():
        pytest.skip("racer_pnp_msgs is outside this checkout")
    got = {
        name: hashlib.md5((msgs / name).read_bytes()).hexdigest()
        for name in ("LapEvent.msg", "LapGuidance.msg")
    }
    assert got == {
        "LapEvent.msg": "ba79fcf3b4720849edd8c16b9c300c7c",
        "LapGuidance.msg": "833d1b7aee456dfc685f2b9750de39b6",
    }
