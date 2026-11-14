"""L1: the start-scan gate. Arming, acceptance, crossing interpolation and the missed gate with a
scripted matcher (exact numbers), then the whole thing on a synthetic stadium."""

from __future__ import annotations

import math

import numpy as np
import pytest
import racer_laps.gate as gate_module
from racer_laps.engine import LapConfig, LapMemory
from racer_laps.gate import GateConfig, GateCrossing, StartScanGate, yaw_gain
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
        "along_search_m": 0.0,
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
        {"wheelbase_m": 0.33, "strong_max_violation": 1.5},
        {"wheelbase_m": 0.33, "strong_max_violation": -0.1},
        {"wheelbase_m": 0.33, "consensus_window_m": 0.0},
        {"wheelbase_m": 0.33, "consensus_past_m": -0.1},
        {"wheelbase_m": 0.33, "consensus_wait_s": -1.0},
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


def test_the_right_turn_gain_scale_applies_to_negative_steering_only() -> None:
    def heading_after(steer: float, scale: float) -> float:
        g = StartScanGate(
            GateConfig(
                wheelbase_m=0.5,
                yaw_gain_table=((0.0, 2.0),),
                yaw_gain_right_scale=scale,
                min_along_support=0,
            )
        )
        g.on_odom(0.0, 0.0)
        g.set_start(0.0, PTS)
        g.on_steering(steer)
        for i in range(1, 101):
            g.on_odom(i * 0.01, 1.0)
        return g.heading_since_closure_rad

    left = heading_after(0.2, 0.5)
    right = heading_after(-0.2, 0.5)
    assert left == pytest.approx(2.0 * math.tan(0.2) / 0.5 * 0.995, rel=0.02)
    assert right == pytest.approx(-0.5 * left, rel=1e-6)  # a right turn, at half the gain


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


def strong(monkeypatch: pytest.MonkeyPatch, violation: float = 0.0) -> None:
    """Every match has this free-space violation share (the scripted points are not a scene)."""
    monkeypatch.setattr(gate_module, "freespace_violation_share", lambda *a, **k: violation)


def advance(g: StartScanGate, t_from: float, seconds: float, v: float = 1.0) -> float:
    steer = math.atan(WHEELBASE_M / 0.9)
    return drive_odom(g, t_from, seconds, v, steer)


def scan(
    g: StartScanGate, mp: pytest.MonkeyPatch, t: float, res: MatchResult | None
) -> GateCrossing | None:
    mp.setattr(gate_module, "match_scans", Scripted([res]))
    return g.on_scan(t, PTS)


def close_lap(g: StartScanGate, mp: pytest.MonkeyPatch, t: float) -> tuple[GateCrossing, float]:
    """Two agreeing STRONG candidates just before the line, then the car drives 0.45 m on and a
    scan with no match closes the lap (0.3 m past the median estimate). Returns (crossing, t)."""
    assert scan(g, mp, t, result(-0.2)) is None
    t = advance(g, t, 0.1)
    assert scan(g, mp, t, result(-0.1)) is None
    t = advance(g, t, 0.5)
    c = scan(g, mp, t, None)
    assert c is not None
    return c, t


def lap_n_gate(mp: pytest.MonkeyPatch, laps: int, **kw: object) -> tuple[StartScanGate, float]:
    """A gate that has closed `laps` laps (through `close_lap`) and driven the next one round:
    armed. Returns (gate, time of the last odometry sample)."""
    strong(mp)
    g = armed_gate(mp, **kw)
    t = 5.7
    for _ in range(laps):
        _c, t = close_lap(g, mp, t)
        t = advance(g, t, 5.5)
    assert g.armed
    return g, t


# -- arming --------------------------------------------------------------------------------------


def test_later_laps_arm_at_0_6_of_the_estimated_length(monkeypatch: pytest.MonkeyPatch) -> None:
    strong(monkeypatch)
    g = armed_gate(monkeypatch)
    c, t = close_lap(g, monkeypatch, 5.7)
    assert c.lap_index == 1
    l_ref = g.l_ref_m
    assert l_ref == pytest.approx(5.8, abs=0.15)
    steer = math.atan(WHEELBASE_M / 0.9)
    g.on_steering(steer)
    while g.station_m < 0.55 * l_ref:
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
    ],
)
def test_acceptance_thresholds(
    monkeypatch: pytest.MonkeyPatch, res: MatchResult, accepted: bool
) -> None:
    g = armed_gate(monkeypatch)
    assert g._accept(res) == accepted


def test_an_accepted_match_keeps_the_tracking_seed_alive(monkeypatch: pytest.MonkeyPatch) -> None:
    strong(monkeypatch)
    g = armed_gate(monkeypatch)
    scan(g, monkeypatch, 5.6, result(-3.0))  # far from the line: no candidate, still the seed
    assert g._last_good is not None and not g._cands
    g2 = armed_gate(monkeypatch)
    scan(g2, monkeypatch, 5.6, result(-0.1, rms=0.031))
    assert g2._last_good is None


# -- consensus crossing --------------------------------------------------------------------------


def test_two_agreeing_strong_matches_close_the_lap_at_the_median_estimate(
    monkeypatch: pytest.MonkeyPatch,
) -> None:
    strong(monkeypatch)
    g = armed_gate(monkeypatch)
    assert scan(g, monkeypatch, 5.60, result(-0.2, y=0.2, yaw=0.1)) is None
    assert scan(g, monkeypatch, 5.70, result(-0.1, y=0.0, yaw=0.3)) is None  # same line: 5.8
    t = advance(g, 5.7, 0.2)
    assert scan(g, monkeypatch, t, None) is None  # 0.1 m past the line: not yet 0.3 m
    t = advance(g, t, 0.3)
    c = scan(g, monkeypatch, t, None)
    assert c is not None and c.lap_index == 1
    assert c.t_cross_s == pytest.approx(5.8, abs=0.03)
    assert c.length_m == pytest.approx(5.8, abs=0.05)
    assert c.n_candidates == 2 and c.spread_m == pytest.approx(0.0, abs=0.03)
    assert not c.missed
    assert g.l_ref_m == pytest.approx(c.length_m)
    # the pose handed on is that of the candidate nearest the line, carried to the line
    assert c.yaw_rad == pytest.approx(0.3)
    assert c.lateral_m == pytest.approx(0.0 - (-0.1) * math.tan(0.3))


def test_two_disagreeing_matches_do_not_close_a_lap(monkeypatch: pytest.MonkeyPatch) -> None:
    strong(monkeypatch)
    g = armed_gate(monkeypatch)
    scan(g, monkeypatch, 5.60, result(-0.2, share=0.65, rms=0.029))  # line 5.8
    scan(g, monkeypatch, 5.70, result(-0.5, share=0.65, rms=0.029))  # line 6.2: 0.4 m apart
    t = advance(g, 5.7, 1.0)
    assert scan(g, monkeypatch, t, None) is None
    assert g.laps_completed == 0


def test_one_match_of_late_quality_closes_but_one_of_normal_quality_does_not(
    monkeypatch: pytest.MonkeyPatch,
) -> None:
    strong(monkeypatch)
    good = armed_gate(monkeypatch)
    scan(good, monkeypatch, 5.7, result(-0.1, share=0.75, rms=0.027))
    t = advance(good, 5.7, 0.5)
    assert scan(good, monkeypatch, t, None) is not None
    plain = armed_gate(monkeypatch)
    scan(plain, monkeypatch, 5.7, result(-0.1, share=0.74, rms=0.029))
    t = advance(plain, 5.7, 0.5)
    assert scan(plain, monkeypatch, t, None) is None


def test_the_wait_rule_closes_half_a_second_after_the_last_candidate_at_the_line(
    monkeypatch: pytest.MonkeyPatch,
) -> None:
    strong(monkeypatch)
    g = armed_gate(monkeypatch)
    scan(g, monkeypatch, 5.65, result(-0.05, share=0.65, rms=0.029))
    scan(g, monkeypatch, 5.70, result(0.05, share=0.65, rms=0.029))  # lines 5.7 and 5.65
    # the car stops on the line: 0.3 m past is never reached, 0.5 s after the last candidate is
    for k in range(1, 61):
        g.on_odom(5.7 + 0.01 * k, 0.0)
    assert scan(g, monkeypatch, 6.1, None) is None  # 0.4 s
    c = scan(g, monkeypatch, 6.25, None)
    assert c is not None and c.length_m == pytest.approx(5.7, abs=0.1)


def test_the_wait_rule_needs_a_candidate_at_or_past_the_line(
    monkeypatch: pytest.MonkeyPatch,
) -> None:
    strong(monkeypatch)
    g = armed_gate(monkeypatch)
    scan(g, monkeypatch, 5.65, result(-0.15, share=0.65, rms=0.029))
    scan(g, monkeypatch, 5.70, result(-0.10, share=0.65, rms=0.029))
    for k in range(1, 61):
        g.on_odom(5.7 + 0.01 * k, 0.0)
    assert scan(g, monkeypatch, 6.25, None) is None  # all candidates before the line, car stopped


def test_a_match_outside_the_window_or_not_strong_is_no_candidate(
    monkeypatch: pytest.MonkeyPatch,
) -> None:
    strong(monkeypatch)
    far = armed_gate(monkeypatch)
    scan(far, monkeypatch, 5.6, result(-0.61, share=0.8, rms=0.02))
    scan(far, monkeypatch, 5.7, result(0.61, share=0.8, rms=0.02))
    assert not far._cands
    strong(monkeypatch, 0.021)
    weak = armed_gate(monkeypatch)
    scan(weak, monkeypatch, 5.7, result(-0.1, share=0.8, rms=0.02))
    assert not weak._cands
    strong(monkeypatch, 0.02)
    edge = armed_gate(monkeypatch)
    scan(edge, monkeypatch, 5.7, result(-0.1, share=0.8, rms=0.02))
    assert len(edge._cands) == 1
    assert edge.last_match is not None and edge.last_match.freespace_violation == 0.02


def test_a_candidate_not_confirmed_in_time_is_dropped(monkeypatch: pytest.MonkeyPatch) -> None:
    strong(monkeypatch)
    g = armed_gate(monkeypatch)
    scan(g, monkeypatch, 5.7, result(-0.1, share=0.65, rms=0.029))
    t = advance(g, 5.7, 2.0)
    scan(g, monkeypatch, t, None)
    assert not g._cands


def test_the_violation_of_the_match_is_measured_both_ways_and_the_larger_kept(
    monkeypatch: pytest.MonkeyPatch,
) -> None:
    shares = iter([0.01, 0.03])
    monkeypatch.setattr(gate_module, "freespace_violation_share", lambda *a, **k: next(shares))
    g = armed_gate(monkeypatch)
    scan(g, monkeypatch, 5.7, result(-0.1))
    assert g.last_match is not None and g.last_match.freespace_violation == 0.03


def test_with_a_trusted_length_an_estimate_outside_the_band_is_ignored(
    monkeypatch: pytest.MonkeyPatch,
) -> None:
    g, t = lap_n_gate(monkeypatch, 2)
    assert g.l_ref_trusted
    # the same in-window match: on the expected line it is a candidate, 2 m early it is not
    g._origin_m -= 2.0  # the expected line is now 2 m behind the match's line
    scan(g, monkeypatch, t, result(-0.1, share=0.8, rms=0.02))
    assert not g._cands
    g._origin_m += 2.0
    scan(g, monkeypatch, t, result(-0.1, share=0.8, rms=0.02))
    assert len(g._cands) == 1


def test_without_a_trusted_length_a_candidate_needs_the_minimum_lap_length(
    monkeypatch: pytest.MonkeyPatch,
) -> None:
    strong(monkeypatch)
    g = armed_gate(monkeypatch)
    g._origin_m += 3.0  # only 2.8 m since the last line
    scan(g, monkeypatch, 5.7, result(-0.1, share=0.8, rms=0.02))
    assert not g._cands


def test_after_a_closure_no_candidate_is_kept_until_the_gate_arms_again(
    monkeypatch: pytest.MonkeyPatch,
) -> None:
    strong(monkeypatch)
    g = armed_gate(monkeypatch)
    _c, t = close_lap(g, monkeypatch, 5.7)
    assert not g._cands and not g.armed
    scan(g, monkeypatch, t, result(-0.1, share=0.9, rms=0.01))
    assert not g._cands and g.laps_completed == 1


def test_a_closure_needs_the_heading_of_a_lap_not_just_an_armed_gate(
    monkeypatch: pytest.MonkeyPatch,
) -> None:
    strong(monkeypatch)
    # armed (heading above 2 pi - 1.2) but short of 2 pi - 0.9: 5.2 rad is 83 percent of a turn
    g, t = gate_at_heading(5.2)
    assert g.armed
    monkeypatch.setattr(gate_module, "match_scans", Scripted([result(-0.1, share=0.8, rms=0.02)]))
    g.on_scan(t, PTS)
    assert not g._cands
    h, t = gate_at_heading(5.6)
    monkeypatch.setattr(gate_module, "match_scans", Scripted([result(-0.1, share=0.8, rms=0.02)]))
    h.on_scan(t, PTS)
    assert len(h._cands) == 1


def gate_at_heading(target: float) -> tuple[StartScanGate, float]:
    g = make_gate()
    g.on_odom(0.0, 0.0)
    g.set_start(0.0, PTS)
    g.on_steering(math.atan(WHEELBASE_M / 0.9))
    t = 0.0
    while g.heading_since_closure_rad < target:
        t += 0.01
        g.on_odom(t, 1.0)
    return g, t


# -- the lap length estimate in the gate -----------------------------------------------------------


def test_a_poisoned_first_lap_is_flagged_suspect_once_the_estimate_is_trusted(
    monkeypatch: pytest.MonkeyPatch,
) -> None:
    g, _t = lap_n_gate(monkeypatch, 3)
    # all three scripted laps are ~5.8 m: nothing suspect
    assert g.l_ref_trusted and g.suspect_laps == []
    g._lengths[1] = 3.9  # pretend lap 1 closed 2 m early
    g._suspect = sorted(i for i, n in g._lengths.items() if g._est.suspect(n))
    assert g.suspect_laps == [1]
    assert g.status()["suspect_laps"] == [1] and g.status()["l_ref_trusted"] is True


# -- the along-lane support check ------------------------------------------------------------------


def test_a_crossing_whose_along_lane_offset_is_not_observable_is_refused(
    monkeypatch: pytest.MonkeyPatch,
) -> None:
    strong(monkeypatch)
    g = armed_gate(monkeypatch, min_along_support=8)
    monkeypatch.setattr(gate_module, "along_support", lambda *a, **k: 7)
    scan(g, monkeypatch, 5.7, result(-0.1, share=0.8, rms=0.02))
    assert not g._cands  # 7 supporting matches: not enough


def test_an_observable_crossing_passes_the_support_check(monkeypatch: pytest.MonkeyPatch) -> None:
    strong(monkeypatch)
    g = armed_gate(monkeypatch, min_along_support=8)
    monkeypatch.setattr(gate_module, "along_support", lambda *a, **k: 8)
    scan(g, monkeypatch, 5.7, result(-0.1, share=0.8, rms=0.02))
    assert len(g._cands) == 1


# -- the missed gate -----------------------------------------------------------------------------


def test_missed_gate_never_invents_a_lap_and_the_next_crossing_is_flagged(
    monkeypatch: pytest.MonkeyPatch,
) -> None:
    strong(monkeypatch)
    g = armed_gate(monkeypatch)
    c1, t = close_lap(g, monkeypatch, 5.7)
    assert not c1.missed
    l_ref = g.l_ref_m
    g.on_steering(math.atan(WHEELBASE_M / 0.9))
    monkeypatch.setattr(gate_module, "match_scans", Scripted([]))
    while g.station_m < 1.3 * l_ref:
        t += 0.1
        for k in range(5):
            g.on_odom(t - 0.08 + 0.02 * k, 1.0)
        assert g.on_scan(t, PTS) is None
    assert g.station_uncertain and g.poll_uncertain()
    assert g.status()["station_uncertain"] is True
    assert g.laps_completed == 1  # no lap invented
    assert g.armed  # stays armed
    c2, t = close_lap(g, monkeypatch, t + 0.1)
    assert c2.missed and c2.lap_index == 2
    assert g.l_ref_m == l_ref  # a missed lap is not a lap length
    assert not g.station_uncertain and not g.poll_uncertain()


def test_a_trusted_length_raises_the_uncertain_flag_at_the_length_plus_the_band(
    monkeypatch: pytest.MonkeyPatch,
) -> None:
    g, _t = lap_n_gate(monkeypatch, 2)
    assert g.l_ref_trusted
    band = g._est.band_m()
    g._origin_m = g.total_m - (g.l_ref_m + band - 0.05)
    assert not g.station_uncertain
    g._origin_m = g.total_m - (g.l_ref_m + band + 0.05)
    assert g.station_uncertain  # earlier than the 1.25 L of an untrusted estimate


def test_an_untrusted_length_raises_it_at_missed_factor_times_the_length(
    monkeypatch: pytest.MonkeyPatch,
) -> None:
    g, _t = lap_n_gate(monkeypatch, 1)
    assert not g.l_ref_trusted
    g._origin_m = g.total_m - 1.2 * g.l_ref_m
    assert not g.station_uncertain
    g._origin_m = g.total_m - 1.3 * g.l_ref_m
    assert g.station_uncertain


def test_status_keys() -> None:
    g = make_gate()
    st = g.status()
    for key in (
        "gate_armed",
        "station_m",
        "heading_since_closure_rad",
        "l_ref_m",
        "l_ref_trusted",
        "suspect_laps",
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
