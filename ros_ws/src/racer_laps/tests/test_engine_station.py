"""L1: the lap memory with the start-scan source: lap bookkeeping, the clean rule for each event
source, the cap schedule, caution zones and the guidance of D v0 and D v1. A scripted gate closes
laps at given times (everything else, odometry and station, is the real gate)."""

from __future__ import annotations

import math

import numpy as np
import pytest
from racer_laps.engine import (
    PHASE_ENGAGE,
    SEVERITY_BRAKE,
    GateConfig,
    LapConfig,
    LapMemory,
    LapResult,
)
from racer_laps.gate import StartScanGate
from racer_laps.lap_record import STATE_DRIVING
from synth import WHEELBASE_M

PTS = np.stack([np.linspace(0, 1, 40), np.zeros(40)], axis=1)
LAP_S = 6.0  # seconds per scripted lap at 1 m/s: a 6 m lap
RELEASE = 1
SEVERITY_INFO = 0
NO_TRACK = 2
BLOCKED = 5
RECOVERING = 7


class ScriptedGate(StartScanGate):
    """The real odometry and station; laps close when the test says so."""

    def __init__(self, cfg: GateConfig) -> None:
        super().__init__(cfg)
        self.close_at: list[float] = []

    def on_scan(self, t_eff_s: float, points):  # type: ignore[no-untyped-def]
        if self.close_at and t_eff_s >= self.close_at[0]:
            t = self.close_at.pop(0)
            return self._finish(t, 0.0, 0.0)
        return None


def make(mode: str = "v0", **kw: object) -> tuple[LapMemory, ScriptedGate]:
    gcfg = GateConfig(wheelbase_m=WHEELBASE_M, min_along_support=0)
    opts: dict = {"guidance_horizon_m": 6.0, **kw}
    cfg = LapConfig(
        lap_source="start_scan",
        lap_mode=mode,
        gate=gcfg,
        v_cap_start_mps=0.5,
        v_cap_step_mps=0.1,
        v_cap_ceiling_mps=1.0,
        **opts,
    )
    mem = LapMemory(cfg)
    gate = ScriptedGate(gcfg)
    mem.gate = gate
    return mem, gate


class Driver:
    """Plays odometry at 50 Hz and a scan every 0.1 s into a LapMemory, at 1 m/s."""

    def __init__(self, mem: LapMemory, gate: ScriptedGate, clearance: float = 0.3) -> None:
        self.mem, self.gate, self.t = mem, gate, 100.0
        self.clearance = clearance
        self.results: list[LapResult] = []
        self.speed = 1.0

    def planner(self, state: int = STATE_DRIVING) -> None:
        self.mem.on_planner_status(state, self.clearance)

    def run(self, seconds: float) -> None:
        for _ in range(int(seconds * 50)):
            self.t += 0.02
            self.mem.on_drive(0.0)
            self.mem.on_wheel_odom(self.t, self.speed)
            if round(self.t * 50) % 5 == 0:
                r = self.mem.on_scan(self.t, PTS)
                if r is not None:
                    self.results.append(r)

    def lap(self, close_in: float = LAP_S) -> LapResult:
        """Run until the next scripted closure; returns that lap."""
        self.gate.close_at.append(self.t + close_in)
        n = len(self.results)
        self.run(close_in + 0.3)
        assert len(self.results) == n + 1
        return self.results[-1]


def started(mode: str = "v0", **kw: object) -> Driver:
    mem, gate = make(mode, **kw)
    d = Driver(mem, gate)
    d.planner()
    d.run(0.2)  # odometry, then the first scan starts the lap
    assert mem.gate is not None and mem.gate.started
    return d


# -- starting ------------------------------------------------------------------------------------


def test_nothing_starts_until_the_planner_drives_and_there_is_odometry_and_a_scan() -> None:
    mem, gate = make()
    assert mem.on_scan(1.0, PTS) is None
    assert mem.status()["phase"] == "waiting_for_driving"
    mem.on_planner_status(NO_TRACK, 0.3)
    assert mem.status()["phase"] == "waiting_for_driving"
    mem.on_planner_status(STATE_DRIVING, 0.3)
    assert mem.status()["phase"] == "waiting_for_scan"
    assert mem.on_scan(1.0, PTS) is None  # no odometry yet: the gate refuses to start
    assert not gate.started
    mem.on_wheel_odom(1.0, 0.0)
    mem.on_scan(1.1, PTS)
    assert gate.started and mem.status()["phase"] == "running"
    assert mem.lap_in_progress == 1
    assert mem.status()["why_no_lap"] == "gate_not_armed"


def test_a_start_scan_config_without_a_gate_config_is_refused() -> None:
    with pytest.raises(ValueError):
        LapConfig(lap_source="start_scan")
    with pytest.raises(ValueError):
        LapConfig(lap_source="map_pose", lap_mode="v1")
    with pytest.raises(ValueError):
        LapConfig(lap_source="bogus")
    with pytest.raises(ValueError):
        LapConfig(lap_mode="v2")
    gate = GateConfig(wheelbase_m=0.33)
    with pytest.raises(ValueError):
        LapConfig(lap_source="start_scan", lap_mode="v1", gate=gate, guidance_horizon_m=13.0)
    with pytest.raises(ValueError):
        LapConfig(lap_source="start_scan", lap_mode="v1", gate=gate, guidance_horizon_m=0.0)


def test_scans_are_ignored_by_a_map_pose_engine() -> None:
    mem = LapMemory(LapConfig())
    assert mem.on_scan(1.0, PTS) is None
    mem.on_wheel_odom(1.0, 1.0)
    mem.on_drive(0.1)
    assert math.isnan(mem.station_m)


# -- clean laps and the cap schedule (D v0) ------------------------------------------------------


def test_clean_laps_raise_the_cap_one_step_and_fill_the_station_set() -> None:
    d = started()
    first = d.lap()
    assert first.lap_index == 1 and first.source == "start_scan"
    assert first.clean and first.stored and first.cap_mps == 0.5 and first.next_cap_mps == 0.6
    assert first.length_m == pytest.approx(LAP_S + 0.2, abs=0.25)
    assert first.lap_time_s == pytest.approx(LAP_S + 0.2, abs=0.25)
    second = d.lap()
    assert second.cap_mps == 0.6 and second.next_cap_mps == 0.7
    assert d.mem.station_set.n_laps == 2
    assert d.mem.guidance().v_cap_mps == pytest.approx(0.7)
    assert d.mem.guidance().s_m.size == 0  # D v0: empty arrays
    assert d.mem.guidance().lap_index == 3
    st = d.mem.status()
    assert st["laps_completed"] == 2 and st["safe_set_laps"] == 2
    assert st["why_no_lap"] == ""


@pytest.mark.parametrize(
    ("source", "unclean"),
    [
        ("ttc", True),
        ("ttc_reverse", True),
        ("watchdog", False),
        ("rate_limit", False),
        ("command_sanity", False),
    ],
)
def test_clean_rule_by_event_source(source: str, unclean: bool) -> None:
    d = started()
    d.lap()
    d.mem.on_safety_event(PHASE_ENGAGE, SEVERITY_BRAKE, source)
    r = d.lap()
    assert r.safety_brakes == 1  # every BRAKE-severity engagement counts in the event
    assert r.obstacle_brakes == (1 if source in ("ttc", "ttc_reverse") else 0)
    assert r.timing_brakes == (0 if source in ("ttc", "ttc_reverse") else 1)
    assert r.clean is (not unclean)
    assert r.stored is (not unclean)
    assert d.mem.status()["timing_events"] == (0 if unclean else 1)


def test_info_severity_and_release_records_are_not_brakes() -> None:
    d = started()
    d.mem.on_safety_event(PHASE_ENGAGE, SEVERITY_INFO, "ttc")
    d.mem.on_safety_event(RELEASE, SEVERITY_BRAKE, "ttc")
    r = d.lap()
    assert r.safety_brakes == 0 and r.clean


def test_a_safety_event_with_no_source_is_taken_as_an_obstacle_gate() -> None:
    d = started()
    d.mem.on_safety_event(PHASE_ENGAGE, SEVERITY_BRAKE)
    assert not d.lap().clean


@pytest.mark.parametrize("state", [NO_TRACK, BLOCKED, RECOVERING])
def test_leaving_driving_makes_the_lap_unclean(state: int) -> None:
    d = started()
    d.planner(state)
    d.run(0.1)
    d.planner(STATE_DRIVING)
    r = d.lap()
    assert not r.clean and not r.stored
    assert r.next_cap_mps == 0.5  # an unclean lap does not raise the cap (and cannot lower lap 1's)


def test_low_plan_clearance_makes_the_lap_unclean() -> None:
    d = started()
    d.clearance = 0.05
    d.planner()
    d.run(0.1)
    d.clearance = 0.3
    d.planner()
    r = d.lap()
    assert not r.clean and r.min_clearance_m == pytest.approx(0.05)


def test_a_missed_gate_lap_is_unclean_and_not_stored() -> None:
    d = started()
    d.lap()  # L_ref is about 6 m
    l_ref = d.gate.l_ref_m
    # drive 1.4 reference laps without a closure, then close one
    d.gate.close_at.append(d.t + 1.4 * l_ref)
    d.run(1.4 * l_ref + 0.3)
    r = d.results[-1]
    assert r.missed_gate and not r.clean and not r.stored
    assert d.gate.l_ref_m == l_ref
    # the guidance while the station is uncertain drops to the lap-1 cap with empty arrays
    d.run(1.6 * l_ref)  # no closure scripted: uncertain again
    g = d.mem.guidance()
    assert g.v_cap_mps == 0.5 and g.s_m.size == 0
    assert d.mem.status()["station_uncertain"] is True


def test_cap_schedule_after_an_unclean_lap_needs_two_clean_laps() -> None:
    d = started()
    assert d.lap().next_cap_mps == pytest.approx(0.6)
    d.mem.on_safety_event(PHASE_ENGAGE, SEVERITY_BRAKE, "ttc")
    assert d.lap().next_cap_mps == pytest.approx(0.5)  # lowered one step
    assert d.lap().next_cap_mps == pytest.approx(0.5)  # first clean lap after: no rise yet
    assert d.lap().next_cap_mps == pytest.approx(0.6)  # second clean lap: rise


# -- caution zones -------------------------------------------------------------------------------


def test_zone_events_in_lap_1_wait_for_the_reference_length_then_apply() -> None:
    d = started()
    d.run(3.0)
    d.mem.on_safety_event(PHASE_ENGAGE, SEVERITY_BRAKE, "ttc")  # at station about 3 m
    assert d.mem.station_set.zones == ()  # no reference length yet
    assert d.mem.status()["zones_created"] == 1
    d.lap(close_in=LAP_S - 3.0)
    zones = d.mem.station_set.zones
    assert len(zones) == 1
    assert zones[0].centre_m == pytest.approx(3.2, abs=0.2)
    assert zones[0].created_lap == 1
    assert zones[0].speed_mps == pytest.approx(1.0, abs=0.05)  # driven at 1 m/s


def test_zones_for_brake_blocked_recovering_and_low_clearance_and_their_expiry() -> None:
    d = started("v1")
    d.lap()  # lap 1 stored, L_ref known
    d.run(2.0)
    d.mem.on_safety_event(PHASE_ENGAGE, SEVERITY_BRAKE, "ttc")
    d.run(0.5)
    d.planner(BLOCKED)
    d.planner(BLOCKED)  # the same state again is not a new zone
    d.run(0.5)
    d.planner(STATE_DRIVING)
    d.planner(RECOVERING)
    d.run(0.5)
    d.planner(STATE_DRIVING)
    d.clearance = 0.01
    d.planner()
    d.planner()  # staying low is one zone, not one per message
    d.clearance = 0.3
    d.planner()
    zones = d.mem.station_set.zones
    assert len(zones) == 4
    assert all(z.created_lap == 2 for z in zones)
    d.lap(close_in=LAP_S - 3.0)  # lap 2 ends: the zones now apply to laps 3 and 4
    assert len(d.mem.station_set.zones) == 4
    d.lap()  # lap 3
    assert len(d.mem.station_set.zones) == 4
    d.lap()  # lap 4: still in its second lap when it closes
    assert d.mem.station_set.zones == ()  # in lap 5 they are gone


def test_a_timing_event_creates_no_zone() -> None:
    d = started("v1")
    d.lap()
    d.mem.on_safety_event(PHASE_ENGAGE, SEVERITY_BRAKE, "watchdog")
    d.mem.on_safety_event(PHASE_ENGAGE, SEVERITY_BRAKE, "rate_limit")
    assert d.mem.station_set.zones == ()


# -- D v1 guidance -------------------------------------------------------------------------------


def test_v1_guidance_is_empty_until_a_clean_lap_is_stored_then_has_61_stations() -> None:
    d = started("v1")
    g = d.mem.guidance()
    assert g.s_m.size == 0 and g.v_cap_mps == 0.5
    d.mem.on_safety_event(PHASE_ENGAGE, SEVERITY_BRAKE, "ttc")
    unclean = d.lap()
    assert not unclean.stored
    assert d.mem.guidance().s_m.size == 0 and d.mem.guidance().v_cap_mps == 0.5
    d.lap()  # clean: stored
    g = d.mem.guidance()
    assert g.s_m.size == g.v_ref_mps.size == g.cost_to_go_s.size == 61
    assert np.allclose(np.diff(g.s_m), 0.1)
    assert g.s_m[0] == pytest.approx(d.mem.station_m)
    assert np.all(np.diff(g.cost_to_go_s) <= 1e-9)
    # v_bound is v_safe + 0.2 and the cap is the largest bound capped at the ceiling
    assert g.v_cap_mps == pytest.approx(min(1.0, float(np.max(g.v_ref_mps))), abs=0.25)
    assert g.v_cap_mps <= 1.0


def test_v1_cap_never_exceeds_the_ceiling_and_the_lap_event_reports_the_driven_cap() -> None:
    d = started("v1", explore_dv_mps=1.0)
    first = d.lap()
    assert first.cap_mps == 0.5
    second = d.lap()
    assert second.cap_mps <= 1.0
    assert d.mem.guidance().v_cap_mps <= 1.0
    assert second.next_cap_mps == pytest.approx(d.mem.guidance().v_cap_mps, abs=0.3)


# -- the lap-1 cross-check -----------------------------------------------------------------------


def test_crosscheck_states() -> None:
    d = started()
    assert d.mem.crosscheck == "no_pose"
    t0 = d.t
    r = LAP_S / (2 * math.pi)  # a pose track that returns to its start every LAP_S seconds
    d.gate.close_at.append(t0 + LAP_S + 0.2)
    while d.t - t0 < LAP_S + 1.0:
        d.run(0.04)
        a = (d.t - t0) / r
        assert d.mem.on_pose(d.t, r * math.cos(a), r * math.sin(a), a + math.pi / 2) is None
    assert d.results, "the scripted gate closed lap 1"
    assert d.mem.crosscheck in ("agree", "disagree")
    assert d.mem.on_pose(d.t - 5.0, 0.0, 0.0, 0.0) is None  # an old stamp is ignored


def test_crosscheck_agree_and_disagree_by_the_closure_times() -> None:
    d = started()
    mem = d.mem
    mem._pose_t1 = 100.0
    mem._gate_t1 = 100.3
    assert mem.crosscheck == "agree"
    mem._gate_t1 = 101.0
    assert mem.crosscheck == "disagree"
    mem._pose_t1 = math.nan
    assert mem.crosscheck == "no_pose"  # the pose detector never started
    mem.detector.set_start(0.0, 0.0, 0.0, 0.0)
    mem._pose_feeding = False
    assert mem.crosscheck == "disagree"  # the gate closed and the pose never did
    mem._pose_feeding = True
    assert mem.crosscheck == "pending"
