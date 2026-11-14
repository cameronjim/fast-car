"""L1: the start-scan gate. Arming, acceptance, crossing interpolation and the missed gate with a
scripted matcher (exact numbers), then the whole thing on a synthetic stadium."""

from __future__ import annotations

import math

import numpy as np
import pytest
import racer_laps.gate as gate_module
from racer_laps.engine import LapConfig, LapMemory
from racer_laps.gate import GateConfig, StartScanGate, yaw_gain
from racer_laps.lap_record import STATE_DRIVING
from racer_laps.scan_match import MatchResult
from synth import WHEELBASE_M, Stadium, feed

TWO_PI = 2.0 * math.pi
PTS = np.stack([np.linspace(0, 1, 40), np.zeros(40)], axis=1)  # enough points to start


def make_gate(**kw: object) -> StartScanGate:
    # one ICP call per scan by default (identity seed), so a scripted matcher lines up
    opts: dict = {
        "min_along_support": 0,
        "dead_reckoning_seed": False,
        "seed_search": False,
        **kw,
    }
    return StartScanGate(GateConfig(wheelbase_m=WHEELBASE_M, **opts))


def drive_odom(
    gate: StartScanGate, t0: float, seconds: float, v: float, steer: float, hz: float = 50.0
) -> float:
    """Feed constant speed and steering; returns the end time."""
    gate.on_steering(steer)
    n = int(seconds * hz)
    for i in range(n + 1):
        gate.on_odom(t0 + i / hz, v)
    return t0 + n / hz


def result(x: float, y: float = 0.0, yaw: float = 0.0, rms: float = 0.02, share: float = 0.9):
    return MatchResult(x, y, yaw, rms, share, 200)


class Scripted:
    """Stands in for match_scans: returns the next scripted result, records the seeds."""

    def __init__(self, results: list[MatchResult | None]) -> None:
        self.results = list(results)
        self.seeds: list[tuple[float, float, float]] = []

    def __call__(self, source, target, seed=(0.0, 0.0, 0.0), iterations=30, inlier_m=0.05, **kw):
        self.seeds.append(seed)
        return self.results.pop(0) if self.results else None


def armed_gate(monkeypatch: pytest.MonkeyPatch, **kw: object) -> StartScanGate:
    """A gate that has driven one full turn (arming: distance and heading both satisfied)."""
    g = make_gate(**kw)
    g.on_odom(0.0, 0.0)
    assert g.set_start(0.0, PTS)
    # 1.0 m/s with steering tan(delta) = L / R: R = 0.9 m for a full turn in 2 pi R = 5.65 m
    steer = math.atan(WHEELBASE_M / 0.9)
    drive_odom(g, 0.0, 5.7, 1.0, steer)
    return g


# -- yaw gain ------------------------------------------------------------------------------------


def test_yaw_gain_is_piecewise_linear_and_held_outside_the_knots() -> None:
    table = ((0.1, 1.12), (0.2, 0.96), (0.3, 0.92), (0.42, 0.86))
    assert yaw_gain(table, 0.0) == pytest.approx(1.12)
    assert yaw_gain(table, 0.15) == pytest.approx(1.04)
    assert yaw_gain(table, 0.5) == pytest.approx(0.86)
    assert yaw_gain((), 0.3) == 1.0


# -- config --------------------------------------------------------------------------------------


@pytest.mark.parametrize(
    "kw",
    [
        {"wheelbase_m": math.nan},
        {"wheelbase_m": 0.0},
        {"wheelbase_m": 0.33, "min_lap_length_m": 0.0},
        {"wheelbase_m": 0.33, "arm_fraction": 0.0},
        {"wheelbase_m": 0.33, "arm_fraction": 1.5},
        {"wheelbase_m": 0.33, "missed_factor": 1.0},
    ],
)
def test_bad_configs_are_refused(kw: dict) -> None:
    with pytest.raises(ValueError):
        GateConfig(**kw)


# -- station and heading -------------------------------------------------------------------------


def test_station_is_forward_wheel_distance_and_ignores_reverse() -> None:
    g = make_gate()
    g.on_odom(0.0, 1.0)
    assert g.set_start(0.0, PTS)
    for i in range(1, 51):
        g.on_odom(i * 0.02, 1.0)
    assert g.station_m == pytest.approx(1.0, abs=0.02)
    for i in range(51, 101):
        g.on_odom(i * 0.02, -0.5)  # backing out
    assert g.station_m == pytest.approx(1.0 + 0.01 * 0.5 / 0.5 * 0 + 0.5 * 0.0 + 0.01, abs=0.05)
    assert g.heading_since_closure_rad == pytest.approx(0.0, abs=1e-9)


def test_odometry_ignores_nan_old_and_repeated_stamps_and_caps_dropouts() -> None:
    g = make_gate()
    g.on_odom(0.0, 1.0)
    g.on_odom(0.02, math.nan)
    g.on_odom(math.nan, 1.0)
    g.on_odom(0.02, 1.0)
    g.on_odom(0.02, 1.0)  # same stamp: ignored
    g.on_odom(0.01, 1.0)  # older: ignored
    g.on_odom(10.0, 1.0)  # a 10 s dropout is integrated as 1 s at most
    assert g.total_m == pytest.approx(1.02, abs=0.01)


def test_heading_follows_v_tan_delta_over_l_and_the_gain_table() -> None:
    g = StartScanGate(
        GateConfig(wheelbase_m=0.5, yaw_gain_table=((0.0, 2.0),), min_along_support=0)
    )
    g.on_odom(0.0, 0.0)
    g.set_start(0.0, PTS)
    delta = 0.2
    g.on_steering(delta)
    for i in range(1, 101):
        g.on_odom(i * 0.01, 1.0)
    # 1 s at ~1 m/s (the first step averages 0 and 1): rate = 2.0 * v * tan(delta) / L
    expected = 2.0 * math.tan(delta) / 0.5 * 0.995
    assert g.heading_since_closure_rad == pytest.approx(expected, rel=0.02)
    g.on_steering(-delta)
    h0 = g.heading_since_closure_rad
    for i in range(101, 201):
        g.on_odom(i * 0.01, 1.0)
    assert g.heading_since_closure_rad < h0  # right turn reduces it


# -- arming --------------------------------------------------------------------------------------


def test_not_armed_before_start_or_with_distance_only_or_heading_only() -> None:
    g = make_gate()
    assert not g.armed
    g.on_odom(0.0, 0.0)
    g.set_start(0.0, PTS)
    # straight 5 m: distance enough, heading not
    drive_odom(g, 0.0, 5.0, 1.0, 0.0)
    assert g.station_m > 3.0 and not g.armed
    # a tight turn in place of travel: heading enough, distance not
    h = make_gate()
    h.on_odom(0.0, 0.0)
    h.set_start(0.0, PTS)
    drive_odom(h, 0.0, 1.0, 1.0, 0.449)
    assert h.station_m < 3.0 and not h.armed


def test_armed_needs_both_distance_and_two_pi_minus_the_slack() -> None:
    g = make_gate()
    g.on_odom(0.0, 0.0)
    g.set_start(0.0, PTS)
    steer = math.atan(WHEELBASE_M / 0.9)
    t = 0.0
    armed_at = None
    for i in range(1, 700):
        t = i * 0.01
        g.on_steering(steer)
        g.on_odom(t, 1.0)
        if g.armed and armed_at is None:
            armed_at = g.heading_since_closure_rad
    assert armed_at is not None
    assert armed_at == pytest.approx(TWO_PI - 1.2, abs=0.05)
    # a half lap does not arm: a stadium looks the same half a lap on
    h = make_gate()
    h.on_odom(0.0, 0.0)
    h.set_start(0.0, PTS)
    for i in range(1, 400):
        h.on_steering(steer)
        h.on_odom(i * 0.01, 1.0)
    assert h.station_m > 3.0
    assert abs(h.heading_since_closure_rad) < TWO_PI - 1.2
    assert not h.armed


def test_arming_works_for_a_clockwise_lap_too() -> None:
    g = make_gate()
    g.on_odom(0.0, 0.0)
    g.set_start(0.0, PTS)
    steer = -math.atan(WHEELBASE_M / 0.9)
    for i in range(1, 700):
        g.on_steering(steer)
        g.on_odom(i * 0.01, 1.0)
    assert g.heading_since_closure_rad < -(TWO_PI - 1.3)
    assert g.armed


def test_later_laps_arm_at_0_6_of_the_reference_length(monkeypatch: pytest.MonkeyPatch) -> None:
    g = armed_gate(monkeypatch)
    monkeypatch.setattr(gate_module, "match_scans", Scripted([result(-0.1), result(0.1)]))
    assert g.on_scan(5.69, PTS) is None
    c = g.on_scan(5.74, PTS)
    assert c is not None and c.lap_index == 1
    l_ref = g.l_ref_m
    assert l_ref == pytest.approx(5.7, abs=0.15)
    # the next lap: below 0.6 L_ref of distance the gate is not armed even with the heading
    steer = math.atan(WHEELBASE_M / 0.9)
    t = 5.74
    g.on_steering(steer)
    for _ in range(int(0.55 * l_ref * 100)):
        t += 0.01
        g.on_odom(t, 1.0)
    assert g.station_m < 0.6 * l_ref
    assert not g.armed
    for _ in range(int(0.1 * l_ref * 100) + 400):
        t += 0.01
        g.on_odom(t, 1.0)
        if g.armed:
            break
    assert g.armed
    assert g.station_m >= 0.6 * l_ref


# -- acceptance ----------------------------------------------------------------------------------


@pytest.mark.parametrize(
    ("res", "accepted"),
    [
        (result(-0.1), True),
        (result(-0.1, rms=0.031), False),
        (result(-0.1, share=0.59), False),
        (result(-0.1, y=0.76), False),
        (result(-0.1, y=-0.76), False),
        (result(-0.1, yaw=math.radians(61)), False),
        (result(-0.1, yaw=-math.radians(61)), False),
        (result(-0.1, y=0.74, yaw=math.radians(59), rms=0.029, share=0.61), True),
        (None, False),
    ],
)
def test_acceptance_thresholds(
    monkeypatch: pytest.MonkeyPatch, res: MatchResult | None, accepted: bool
) -> None:
    g = armed_gate(monkeypatch)
    monkeypatch.setattr(gate_module, "match_scans", Scripted([res]))
    g.on_scan(5.69, PTS)
    assert (g._prev is not None) == accepted


# -- crossing ------------------------------------------------------------------------------------


def test_crossing_time_is_interpolated_between_the_two_matches(
    monkeypatch: pytest.MonkeyPatch,
) -> None:
    g = armed_gate(monkeypatch)
    # x goes from -0.1 at t = 5.60 to +0.3 at t = 5.70: zero at a quarter of the way
    monkeypatch.setattr(gate_module, "match_scans", Scripted([result(-0.1, y=0.2, yaw=0.1)]))
    assert g.on_scan(5.60, PTS) is None
    monkeypatch.setattr(gate_module, "match_scans", Scripted([result(0.3, y=0.0, yaw=0.3)]))
    c = g.on_scan(5.70, PTS)
    assert c is not None
    assert c.t_cross_s == pytest.approx(5.625, abs=1e-9)
    assert c.lateral_m == pytest.approx(0.15, abs=1e-9)
    assert c.yaw_rad == pytest.approx(0.15, abs=1e-9)
    assert c.lap_time_s == pytest.approx(5.625, abs=1e-9)  # lap 1 starts at the start scan, t=0
    # wheel distance between the crossings: station at 5.625
    assert c.length_m == pytest.approx(5.625, abs=0.1)
    assert not c.missed
    assert g.l_ref_m == pytest.approx(c.length_m)


def test_two_matches_more_than_half_a_second_apart_are_not_a_crossing(
    monkeypatch: pytest.MonkeyPatch,
) -> None:
    g = armed_gate(monkeypatch)
    monkeypatch.setattr(gate_module, "match_scans", Scripted([result(-0.1)]))
    g.on_scan(5.60, PTS)
    monkeypatch.setattr(gate_module, "match_scans", Scripted([result(0.6)]))
    assert g.on_scan(6.20, PTS) is None  # 0.6 s later and 0.6 m ahead: not a crossing
    assert g.laps_completed == 0


def test_the_first_accepted_match_just_past_the_line_places_the_crossing_back_by_distance(
    monkeypatch: pytest.MonkeyPatch,
) -> None:
    g = armed_gate(monkeypatch)
    total = g.total_m
    t_end = 5.7  # the odometry of armed_gate ran to 5.7 s at 1 m/s
    monkeypatch.setattr(gate_module, "match_scans", Scripted([result(0.3, y=0.1, yaw=0.05)]))
    c = g.on_scan(t_end, PTS)
    assert c is not None
    assert c.t_cross_s == pytest.approx(t_end - 0.3, abs=0.03)  # 0.3 m back at 1 m/s
    assert c.length_m == pytest.approx(total - 0.3, abs=0.05)
    assert c.lateral_m == pytest.approx(0.1)
    assert c.yaw_rad == pytest.approx(0.05)


def test_the_late_match_rule_has_a_limit_and_a_switch(monkeypatch: pytest.MonkeyPatch) -> None:
    g = armed_gate(monkeypatch)
    monkeypatch.setattr(gate_module, "match_scans", Scripted([result(0.41)]))
    assert g.on_scan(5.7, PTS) is None  # 0.41 m ahead: nothing is invented
    h = armed_gate(monkeypatch, late_match_max_m=0.0)
    monkeypatch.setattr(gate_module, "match_scans", Scripted([result(0.1)]))
    assert h.on_scan(5.7, PTS) is None  # switched off
    # a borderline match (share 0.84, or RMS 0.026) of a partial view is not evidence
    for weak in (result(0.1, share=0.84), result(0.1, rms=0.026)):
        w = armed_gate(monkeypatch)
        monkeypatch.setattr(gate_module, "match_scans", Scripted([weak]))
        assert w.on_scan(5.7, PTS) is None
    ok = armed_gate(monkeypatch)
    monkeypatch.setattr(gate_module, "match_scans", Scripted([result(0.1, share=0.85, rms=0.025)]))
    assert ok.on_scan(5.7, PTS) is not None
    # a fresh earlier match that is already ahead of the line does not allow the late rule
    k = armed_gate(monkeypatch)
    monkeypatch.setattr(gate_module, "match_scans", Scripted([result(0.9)]))
    assert k.on_scan(5.6, PTS) is None
    monkeypatch.setattr(gate_module, "match_scans", Scripted([result(0.3)]))
    assert k.on_scan(5.7, PTS) is None


def test_a_failed_scan_between_two_accepted_ones_within_the_window_still_crosses(
    monkeypatch: pytest.MonkeyPatch,
) -> None:
    g = armed_gate(monkeypatch)
    monkeypatch.setattr(
        gate_module, "match_scans", Scripted([result(-0.2), None, None, result(0.2)])
    )
    assert g.on_scan(5.60, PTS) is None
    assert g.on_scan(5.70, PTS) is None  # no match: nothing happens
    c = g.on_scan(5.80, PTS)
    assert c is not None and c.t_cross_s == pytest.approx(5.70)


def test_no_crossing_when_the_offset_stays_negative_or_never_was_negative(
    monkeypatch: pytest.MonkeyPatch,
) -> None:
    g = armed_gate(monkeypatch)
    monkeypatch.setattr(gate_module, "match_scans", Scripted([result(-0.3), result(-0.1)]))
    assert g.on_scan(5.60, PTS) is None
    assert g.on_scan(5.70, PTS) is None
    h = armed_gate(monkeypatch, late_match_max_m=0.0)
    monkeypatch.setattr(gate_module, "match_scans", Scripted([result(0.1), result(0.2)]))
    assert h.on_scan(5.60, PTS) is None
    assert h.on_scan(5.70, PTS) is None  # was never behind the line


def test_scans_do_nothing_before_the_start_or_while_not_armed() -> None:
    g = make_gate()
    assert g.on_scan(1.0, PTS) is None
    assert not g.started
    g.on_odom(0.0, 1.0)
    g.set_start(0.0, PTS)
    assert g.on_scan(1.0, PTS) is None  # not armed
    assert g.n_armed_scans == 0
    assert g.last_arming_state == "waiting"


def test_set_start_refuses_a_tiny_scan_or_no_odometry() -> None:
    g = make_gate()
    assert not g.set_start(0.0, PTS)  # no odometry yet
    g.on_odom(0.0, 0.0)
    assert not g.set_start(0.0, PTS[:5])
    assert not g.started
    assert g.set_start(0.0, PTS)
    assert g.started


# -- the seeds -----------------------------------------------------------------------------------


def test_seeds_are_the_dead_reckoned_pose_then_the_identity_then_the_chain(
    monkeypatch: pytest.MonkeyPatch,
) -> None:
    g = armed_gate(monkeypatch, dead_reckoning_seed=True)
    sc = Scripted([result(-0.5, y=0.1, yaw=0.05), None])
    monkeypatch.setattr(gate_module, "match_scans", sc)
    g.on_scan(5.60, PTS)
    assert len(sc.seeds) == 2
    assert sc.seeds[1] == (0.0, 0.0, 0.0)
    # DR: after a full 5.7 m turn the car is near the start (within the odometry error)
    assert math.hypot(sc.seeds[0][0], sc.seeds[0][1]) < 1.0
    # the next scan 0.1 s later is seeded with the accepted result advanced by the odometry
    t = 5.70
    g.on_odom(t, 1.0)
    g.on_odom(t + 0.1, 1.0)
    sc2 = Scripted([None, None])
    monkeypatch.setattr(gate_module, "match_scans", sc2)
    g.on_scan(5.70, PTS)
    assert sc2.seeds[0][0] > -0.5  # moved forward from -0.5
    # without the dead-reckoning seed only the identity is tried
    h = armed_gate(monkeypatch)
    sc3 = Scripted([None])
    monkeypatch.setattr(gate_module, "match_scans", sc3)
    h.on_scan(5.60, PTS)
    assert sc3.seeds == [(0.0, 0.0, 0.0)]


def test_the_best_of_the_seeds_wins(monkeypatch: pytest.MonkeyPatch) -> None:
    g = armed_gate(monkeypatch, dead_reckoning_seed=True)
    good = result(-0.4, share=0.95)
    bad = result(0.5, share=0.65)
    monkeypatch.setattr(gate_module, "match_scans", Scripted([bad, good]))
    g.on_scan(5.60, PTS)
    assert g._prev is not None and g._prev[1].x_m == -0.4


# -- the along-lane support check ------------------------------------------------------------------


def test_a_crossing_whose_along_lane_offset_is_not_observable_is_refused(
    monkeypatch: pytest.MonkeyPatch,
) -> None:
    g = armed_gate(monkeypatch, min_along_support=8)
    monkeypatch.setattr(gate_module, "match_scans", Scripted([result(-0.1), result(0.1)]))
    monkeypatch.setattr(gate_module, "along_support", lambda *a, **k: 7)
    g.on_scan(5.60, PTS)
    assert g.on_scan(5.70, PTS) is None  # 7 supporting matches: not enough
    assert g.laps_completed == 0


def test_an_observable_crossing_passes_the_support_check(monkeypatch: pytest.MonkeyPatch) -> None:
    g = armed_gate(monkeypatch, min_along_support=8)
    monkeypatch.setattr(gate_module, "match_scans", Scripted([result(-0.1), result(0.1)]))
    monkeypatch.setattr(gate_module, "along_support", lambda *a, **k: 8)
    g.on_scan(5.60, PTS)
    assert g.on_scan(5.70, PTS) is not None


# -- the missed gate -----------------------------------------------------------------------------


def test_missed_gate_never_invents_a_lap_and_the_next_crossing_is_flagged(
    monkeypatch: pytest.MonkeyPatch,
) -> None:
    g = armed_gate(monkeypatch)
    monkeypatch.setattr(gate_module, "match_scans", Scripted([result(-0.1), result(0.1)]))
    g.on_scan(5.69, PTS)
    c1 = g.on_scan(5.74, PTS)
    assert c1 is not None and not c1.missed
    l_ref = g.l_ref_m
    steer = math.atan(WHEELBASE_M / 0.9)
    t = 5.74
    g.on_steering(steer)
    # drive 1.3 reference laps with every scan failing to match
    monkeypatch.setattr(gate_module, "match_scans", Scripted([]))
    while g.station_m < 1.3 * l_ref:
        t += 0.1
        for k in range(5):
            g.on_odom(t - 0.08 + 0.02 * k, 1.0)
        assert g.on_scan(t, PTS) is None
    assert g.station_uncertain
    assert g.poll_uncertain()
    assert g.status()["station_uncertain"] is True
    assert g.laps_completed == 1  # no lap invented
    assert g.armed  # stays armed
    # the next real crossing closes a lap that is flagged and does not move L_ref
    monkeypatch.setattr(gate_module, "match_scans", Scripted([result(-0.1), result(0.1)]))
    t += 0.1
    g.on_odom(t, 1.0)
    g.on_scan(t, PTS)
    c2 = g.on_scan(t + 0.05, PTS)
    assert c2 is not None
    assert c2.missed
    assert c2.lap_index == 2
    assert g.l_ref_m == l_ref
    assert not g.station_uncertain
    assert not g.poll_uncertain()


def test_status_keys() -> None:
    g = make_gate()
    st = g.status()
    for key in (
        "gate_armed",
        "station_m",
        "heading_since_closure_rad",
        "l_ref_m",
        "station_uncertain",
        "armed_scans",
        "matched_scans",
    ):
        assert key in st


# -- the whole thing on a synthetic stadium ------------------------------------------------------


def start_mem() -> LapMemory:
    cfg = LapConfig(
        lap_source="start_scan",
        gate=GateConfig(wheelbase_m=WHEELBASE_M),
        v_cap_start_mps=0.5,
        v_cap_ceiling_mps=1.0,
    )
    mem = LapMemory(cfg)
    mem.on_planner_status(STATE_DRIVING, 0.3)
    return mem


@pytest.mark.parametrize("clockwise", [False, True])
def test_stadium_laps_counted_with_the_right_times(clockwise: bool) -> None:
    stadium = Stadium()
    mem = start_mem()
    laps = feed(mem, stadium, laps=4.2, speed_mps=1.0, clockwise=clockwise, seed=3)
    assert [r.lap_index for r in laps] == [1, 2, 3, 4]  # type: ignore[attr-defined]
    period = stadium.length_m / 1.0
    for r in laps:
        assert r.lap_time_s == pytest.approx(period, abs=0.2)  # type: ignore[attr-defined]
        assert r.length_m == pytest.approx(stadium.length_m, abs=0.4)  # type: ignore[attr-defined]
        assert not r.missed_gate  # type: ignore[attr-defined]


def test_the_gate_does_not_close_at_half_a_lap_on_a_symmetric_stadium() -> None:
    stadium = Stadium()
    mem = start_mem()
    seen: list[float] = []

    def on_tick(m: object, tick: object) -> None:
        seen.append(m.gate.station_m)  # type: ignore[attr-defined]

    laps = feed(mem, stadium, laps=0.8, speed_mps=1.0, seed=4, on_tick=on_tick)
    assert laps == []
    assert max(seen) > 0.75 * stadium.length_m  # it did drive past the half way point
    assert mem.gate is not None and mem.gate.laps_completed == 0
