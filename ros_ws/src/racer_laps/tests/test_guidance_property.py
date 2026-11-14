"""L2 (hypothesis): for ANY sequence of laps and events, every guidance the lap memory emits passes
the planner's validation rules, the cap never exceeds the ceiling (nor the lap-1 cap before a clean
lap is stored), a caution zone's bound never exceeds the speed driven there minus 0.1, and the lap
count never increases without a crossing (scan gaps neither create nor lose a lap).

The planner's rules (brief 26, section 2.4 of the lap learning brief): three arrays of equal
length between 2 and 128 (or all empty), all finite, stations increasing by 0.10 within 1e-3,
v_ref >= 0, cost_to_go non-increasing within 1e-3, as float32 (the message type)."""

from __future__ import annotations

import math

import numpy as np
import pytest

hypothesis = pytest.importorskip("hypothesis")
from hypothesis import HealthCheck, given, settings
from hypothesis import strategies as st
from planner_rules import planner_accepts
from racer_laps.engine import PHASE_ENGAGE, SEVERITY_BRAKE
from racer_laps.lap_record import STATE_DRIVING
from test_engine_station import PTS, make

CEILING = 1.0
LAP1_CAP = 0.5


EVENTS = ("none", "ttc", "ttc_reverse", "watchdog", "rate_limit", "blocked", "recovering", "low")

lap_plan = st.lists(
    st.tuples(
        st.floats(4.5, 9.0),  # lap time at the lap's speed, seconds
        st.floats(0.25, 1.4),  # speed, m/s
        st.sampled_from(EVENTS),
        st.floats(0.0, 1.0),  # when in the lap the event happens
        st.booleans(),  # drop the scans of this lap's last 0.4 s (a scan gap at the crossing)
    ),
    min_size=2,
    max_size=7,
)


@settings(
    max_examples=40,
    deadline=None,
    suppress_health_check=[HealthCheck.too_slow, HealthCheck.data_too_large],
)
@given(plan=lap_plan, explore=st.floats(0.0, 0.6), horizon=st.sampled_from([0.2, 3.0, 6.0, 12.0]))
def test_every_guidance_is_valid_and_bounded_whatever_happens(
    plan: list[tuple[float, float, str, float, bool]], explore: float, horizon: float
) -> None:
    mem, gate = make("v1", explore_dv_mps=explore, guidance_horizon_m=horizon)
    t = 100.0
    mem.on_planner_status(STATE_DRIVING, 0.3)
    mem.on_wheel_odom(t, 0.0)
    mem.on_scan(t, PTS)
    assert gate.started
    laps_seen = 0
    stored_before_lap = 0
    for lap_s, speed, event, when, drop_last in plan:
        close_t = t + lap_s
        gate.close_at.append(close_t)
        event_t = t + when * lap_s
        sent = False
        steps = int((lap_s + 0.3) * 50)
        for _ in range(steps):
            t += 0.02
            mem.on_drive(0.0)
            mem.on_wheel_odom(t, speed)
            if not sent and t >= event_t:
                sent = True
                if event in ("ttc", "ttc_reverse", "watchdog", "rate_limit"):
                    mem.on_safety_event(PHASE_ENGAGE, SEVERITY_BRAKE, event)
                elif event in ("blocked", "recovering"):
                    mem.on_planner_status(5 if event == "blocked" else 7, 0.3)
                    mem.on_planner_status(STATE_DRIVING, 0.3)
                elif event == "low":
                    mem.on_planner_status(STATE_DRIVING, 0.001)
                    mem.on_planner_status(STATE_DRIVING, 0.3)
            if round(t * 50) % 5 == 0:
                gap = drop_last and close_t - 0.4 <= t <= close_t + 0.25
                before = mem.laps_completed
                r = None if gap else mem.on_scan(t, PTS)
                if r is not None:
                    laps_seen += 1
                assert mem.laps_completed == before + (1 if r is not None else 0)
            if round(t * 50) % 5 == 1:
                g = mem.guidance()
                assert planner_accepts(g), (g.s_m[:3], g.v_ref_mps[:3], g.cost_to_go_s[:3])
                assert g.v_cap_mps <= CEILING + 1e-9
                if mem.station_set.n_laps == 0:
                    assert g.v_cap_mps <= LAP1_CAP + 1e-9
                    assert g.s_m.size == 0
                # the zone bound: never above the speed driven there minus 0.1
                for z in mem.station_set.zones:
                    if mem.station_set.n_laps >= 1 and z.active_on(mem.lap_in_progress):
                        grid = z.centre_m - np.linspace(0.0, z.length_m, 11)
                        b = mem.station_set.v_bound(grid, mem.lap_in_progress, explore)
                        assert np.all(b <= max(0.0, z.speed_mps - 0.1) + 1e-9)
        # a scripted closure whose scan was dropped closes on the next scan: never lost, once
        stored_before_lap = mem.station_set.n_laps
    # every scripted closure produced exactly one lap and nothing else did
    assert mem.laps_completed == laps_seen <= len(plan)
    assert stored_before_lap <= mem.laps_completed
    assert not math.isnan(mem.station_m)


@settings(max_examples=25, deadline=None)
@given(st.lists(st.floats(0.0, 3.0), min_size=0, max_size=30))
def test_the_lap_count_never_increases_without_a_crossing(gaps: list[float]) -> None:
    mem, gate = make("v0")
    t = 100.0
    mem.on_planner_status(STATE_DRIVING, 0.3)
    mem.on_wheel_odom(t, 0.0)
    mem.on_scan(t, PTS)
    for gap in gaps:
        t += gap  # pauses in the scan and odometry streams, however long
        mem.on_wheel_odom(t, 1.0)
        mem.on_drive(0.2)
        assert mem.on_scan(t, PTS) is None
        assert mem.laps_completed == 0
    assert gate.laps_completed == 0
