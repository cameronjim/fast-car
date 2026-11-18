"""L1: the keyframe ring and the alias verdict inside the gate (brief 50). Scripted matchers and
synthetic scans; the ring's bookkeeping alone is in test_ring.py, the ring and stadium drives in
test_gate_v3.py."""

from __future__ import annotations

import math

import numpy as np
import pytest
import racer_laps.gate as gate_module
from racer_laps.gate import AMBIGUOUS, MAX_EXTRAPOLATION_S, UNIQUE, GateConfig, StartScanGate
from racer_laps.ring import Fix
from scripted_car import Car
from synth import WHEELBASE_M, Stadium
from test_gate import PTS, Scripted, result
from v3_support import RING, RING_SPEED, STADIUM, run


def straight_gate(**kw: object) -> StartScanGate:
    """A gate that was started and drives a straight line at 1 m/s; scans are the scripted points
    (no match is made while it is not tracking)."""
    opts: dict = {"min_along_support": 0, "unique_shift_m": 0.0, **kw}
    g = StartScanGate(GateConfig(wheelbase_m=WHEELBASE_M, **opts))
    g.on_odom(0.0, 1.0)
    assert g.set_start(0.0, PTS)
    return g


def roll(
    g: StartScanGate, t0: float, seconds: float, v: float = 1.0, scan_hz: float = 10.0
) -> float:
    """Odometry at 50 Hz and a scan every 1/scan_hz s; returns the end time."""
    t = t0
    n = round(seconds * 50)
    for i in range(n):
        t = t0 + (i + 1) / 50.0
        g.on_odom(t, v)
        if i % round(50 / scan_hz) == 0:
            g.on_scan(t, PTS)
    return t


# -- configuration and small paths -------------------------------------------------------------------


@pytest.mark.parametrize(
    "kw",
    [
        {"unique_shift_m": -0.1},
        {"unique_shift_tol_m": -0.1},
        {"unique_same_m": 0.0},
        {"alias_max_share": 1.1},
        {"alias_max_share": -0.1},
        {"alias_min_scans": 0},
        {"kf_spacing_m": 0.0},
        {"kf_max": 1},
        {"kf_window_m": 0.0},
        {"pair_min_yaw_rad": 0.0},
        {"pair_max_age_s": 0.0},
        {"launch_steer_m": -1.0},
        {"attitude_min_scans": 0},
        {"proof_min_unique": 0},
        {"anchor_wait_m": -0.1},
    ],
)
def test_bad_v3_settings_are_refused(kw: dict) -> None:
    with pytest.raises(ValueError):
        GateConfig(wheelbase_m=WHEELBASE_M, **kw)
    # with the ring off the ring's own numbers are not looked at
    if "kf_max" in kw or "kf_spacing_m" in kw or "kf_window_m" in kw:
        assert GateConfig(wheelbase_m=WHEELBASE_M, ring_enabled=False, **kw)


def test_the_ring_reports_its_size_in_bytes() -> None:
    g = straight_gate()
    roll(g, 0.0, 2.0)
    kf = g.ring.keyframes[-1]
    assert kf.nbytes == g.ring.keyframes[-1].ref.pts.nbytes + kf.ref.normals.nbytes + (
        kf.ref.dense.nbytes + kf.ref.profile.nbytes
    )
    assert g.ring.nbytes >= kf.nbytes > 0


def test_no_pending_estimate_without_the_ring_and_a_winding_line_found_in_one_step() -> None:
    off = straight_gate(ring_enabled=False)
    assert off._pending_estimate_m() is None
    g = straight_gate()
    sample = gate_module._Sample
    last = sample(1.0, 1.0, 0.0, 0.0, 0.0, 0.0, 6.0)
    new = sample(1.02, 1.02, 0.0, 0.0, 0.0, 0.0, 6.4)
    g._note_winding(last, new)
    assert g._w_star_m is not None and 1.0 < g._w_star_m < 1.02
    again = sample(1.04, 1.04, 0.0, 0.0, 0.0, 0.0, 6.5)
    g._note_winding(new, again)  # decided once per lap
    assert g._w_star_m < 1.02
    past = straight_gate()
    past._note_winding(
        sample(1.0, 1.0, 0.0, 0.0, 0.0, 0.0, 7.0), sample(1.02, 1.02, 0.0, 0.0, 0.0, 0.0, 7.2)
    )
    assert past._w_star_m == 1.02  # already past 2 pi at the first sample seen


def test_the_winding_of_lap_1_is_shifted_by_the_attitude_correction_and_later_laps_are_not() -> (
    None
):
    sample = gate_module._Sample
    g = straight_gate()
    g._d_beta = 0.23  # ccw: the line is where psi + 0.23 reaches 2 pi
    g._note_winding(
        sample(1.0, 1.0, 0.0, 0.0, 0.0, 0.0, 6.0), sample(1.02, 1.02, 0.0, 0.0, 0.0, 0.0, 6.1)
    )
    assert g._w_star_m is not None and 1.0 < g._w_star_m < 1.02  # 6.0 + 0.23 < 2 pi < 6.1 + 0.23
    cw = straight_gate()
    cw._d_beta = -0.23
    cw._note_winding(
        sample(1.0, 1.0, 0.0, 0.0, 0.0, 0.0, -6.0), sample(1.02, 1.02, 0.0, 0.0, 0.0, 0.0, -6.1)
    )
    assert cw._w_star_m is not None and 1.0 < cw._w_star_m < 1.02
    later = straight_gate()
    later._d_beta = 0.23
    later._laps = 1  # not lap 1: attitude at two steady crossings is the same
    later._note_winding(
        sample(1.0, 1.0, 0.0, 0.0, 0.0, 0.0, 6.0), sample(1.02, 1.02, 0.0, 0.0, 0.0, 0.0, 6.1)
    )
    assert later._w_star_m is None


def test_an_estimate_before_the_history_or_before_the_lap_start_places_nothing() -> None:
    g = straight_gate()
    roll(g, 0.0, 1.0)
    assert g._close_at(1.0, -5.0, "anchor") is None  # before the odometry history
    assert g._close_at(1.0, 0.0, "anchor") is None  # at the lap start: not after it


# -- teaching --------------------------------------------------------------------------------------


def test_lap_1_teaches_a_keyframe_every_half_metre_with_increasing_stations() -> None:
    g = straight_gate()
    roll(g, 0.0, 3.1)
    stations = [k.station_m for k in g.ring.keyframes]
    assert stations[0] == 0.0 and len(stations) >= 6
    gaps = np.diff(stations)
    assert np.all(gaps > 0.0) and np.all(gaps >= 0.5) and np.all(gaps < 0.65)  # 0.1 m of scans
    assert g.ring.teaching and not g.ring.fixed


def test_no_keyframe_is_taught_while_standing_or_on_stale_odometry() -> None:
    g = straight_gate()
    t = roll(g, 0.0, 1.2)
    n = len(g.ring.keyframes)
    t = roll(g, t, 2.0, v=0.0)  # standing: the wheel total does not move
    assert len(g.ring.keyframes) == n
    t = roll(g, t, 0.4, v=1.0)
    n = len(g.ring.keyframes)
    stale0 = g.stale_odometry_scans
    for k in range(5):  # scans a long time after the last odometry sample
        g.on_scan(t + MAX_EXTRAPOLATION_S + 0.5 + 0.1 * k, PTS)
    assert g.stale_odometry_scans == stale0 + 5
    assert len(g.ring.keyframes) == n


def test_a_slow_car_below_the_standstill_speed_teaches_nothing() -> None:
    g = straight_gate()
    n = len(g.ring.keyframes)
    roll(g, 0.0, 30.0, v=0.04)  # 1.2 m of creeping below the standstill speed
    assert len(g.ring.keyframes) == n


def test_at_kf_max_the_ring_thins_itself_and_doubles_its_spacing() -> None:
    g = straight_gate(kf_max=6)
    roll(g, 0.0, 9.0)
    ring = g.ring
    assert len(ring.keyframes) <= 6
    assert ring.spacing_m >= 1.0
    stations = [k.station_m for k in ring.keyframes]
    assert stations == sorted(stations) and stations[0] == 0.0


def test_set_start_forgets_the_ring_and_keyframe_0_is_the_start_scan() -> None:
    g = straight_gate()
    roll(g, 0.0, 2.0)
    assert len(g.ring.keyframes) > 2
    assert g.set_start(5.0, PTS)
    assert [k.station_m for k in g.ring.keyframes] == [0.0]
    assert g.ring.keyframes[0].ref is g._start
    assert g.ring.teaching and g.ring.generation == 0


def test_with_the_ring_off_nothing_is_taught_and_the_status_has_no_ring() -> None:
    g = straight_gate(ring_enabled=False)
    roll(g, 0.0, 3.0)
    assert g.ring.keyframes == []
    assert "ring" not in g.status()


# -- repeating: which keyframe, which seed ---------------------------------------------------------


def taught_gate(mp: pytest.MonkeyPatch) -> tuple[StartScanGate, Scripted, float]:
    """A straight-line gate with a taught ring (lap 1 'closed' after 3.1 m by hand) whose station
    is re-based to now; matches are scripted. Returns (gate, matcher, time)."""
    g = straight_gate()
    t = roll(g, 0.0, 3.1)
    g.ring.finish_teaching(g.station_m)
    g._origin_m = g.total_m  # lap 2 begins here
    g._laps = 1
    matcher = Scripted([])
    mp.setattr(gate_module, "match_scans", matcher)
    mp.setattr(gate_module, "freespace_violation_share", lambda *a, **k: 0.0)
    return g, matcher, t


def test_a_scan_is_matched_to_one_keyframe_with_the_predicted_pose_as_the_first_seed(
    monkeypatch: pytest.MonkeyPatch,
) -> None:
    g, matcher, t = taught_gate(monkeypatch)
    matcher.results = [result(0.0), result(0.1)]
    t = roll(g, t, 0.12, scan_hz=50.0)  # the first scan of lap 2, station ~0.02
    first = matcher.seeds[0]
    assert first[0] == pytest.approx(g.ring.keyframes[0].station_m + 0.0, abs=0.06)  # kf 0
    assert first[1] == 0.0 and first[2] == 0.0
    n_first = len(matcher.seeds)
    g.on_odom(t + 0.02, 1.0)
    g.on_scan(t + 0.02, PTS)
    assert len(matcher.seeds) == n_first + 1  # one match per scan


def test_the_first_match_of_a_visit_is_seeded_with_the_yaw_the_odometry_predicts(
    monkeypatch: pytest.MonkeyPatch,
) -> None:
    g = straight_gate()
    g.on_steering(0.1)  # a curve: the heading grows with the station
    t = roll(g, 0.0, 3.1)
    g.ring.finish_teaching(g.station_m)
    g._origin_m = g.total_m
    g._heading_origin = g._hist[-1].heading
    g._laps = 1
    matcher = Scripted([])
    monkeypatch.setattr(gate_module, "match_scans", matcher)
    monkeypatch.setattr(gate_module, "freespace_violation_share", lambda *a, **k: 0.0)
    roll(g, t, 0.3, scan_hz=50.0)
    seed = matcher.seeds[0]
    kf = g.ring.keyframes[0]  # the first scans of lap 2 are at station ~0.02, keyframe 0
    yaw_expected = (g._hist[-1].heading - g._heading_origin) - kf.heading_rad
    assert abs(seed[2]) > 0.0 and abs(seed[2]) < 0.1  # the predicted yaw, not zero
    assert abs(yaw_expected) > 0.0


def test_the_second_scan_of_a_visit_is_seeded_by_the_previous_match_advanced_by_odometry(
    monkeypatch: pytest.MonkeyPatch,
) -> None:
    g, matcher, t = taught_gate(monkeypatch)
    prev = result(0.05)
    matcher.results = [prev]
    t = roll(g, t, 0.05, scan_hz=50.0)
    seeds_before = len(matcher.seeds)
    matcher.results = [result(0.15)]
    for _ in range(5):  # 0.1 m of travel
        t += 0.02
        g.on_odom(t, 1.0)
    g.on_scan(t, PTS)
    assert len(matcher.seeds) == seeds_before + 1
    seed = matcher.seeds[-1]
    assert seed[0] == pytest.approx(0.05 + 0.1, abs=0.03)  # the previous match, 0.1 m on


@pytest.mark.parametrize(
    ("offset", "expected_kf"),
    [(0.0, 1), (0.25, 1), (-0.25, 1)],
)
def test_the_keyframe_is_the_live_one_nearest_the_station(
    monkeypatch: pytest.MonkeyPatch, offset: float, expected_kf: int
) -> None:
    g, _matcher, _t = taught_gate(monkeypatch)
    station = g.ring.keyframes[expected_kf].station_m + offset
    kf = g.ring.nearest_live(station)
    assert kf is not None and kf.index == expected_kf


def test_the_window_edge_is_kf_window_exactly_and_epsilon_over_has_no_keyframe(
    monkeypatch: pytest.MonkeyPatch,
) -> None:
    g, _m, _t = taught_gate(monkeypatch)
    last = g.ring.keyframes[-1]
    assert g.ring.nearest_live(last.station_m + g.cfg.kf_window_m - 1e-9) is last
    assert g.ring.nearest_live(last.station_m + g.cfg.kf_window_m + 1e-6) is None


def test_no_keyframe_is_matched_while_the_start_scan_is_tracked(
    monkeypatch: pytest.MonkeyPatch,
) -> None:
    """Once the heading is 62 percent of a turn the start scan is followed and the keyframes rest."""
    g, matcher, t = taught_gate(monkeypatch)
    g._origin_m = g.total_m - 3.5  # 3.5 m into the lap
    g.on_steering(math.atan(WHEELBASE_M / 0.9))
    t = roll(g, t, 4.0, scan_hz=50.0)
    assert g.tracking
    matcher.results = [None] * 3
    n = len(matcher.seeds)
    g.on_odom(t + 0.02, 1.0)
    g.on_scan(t + 0.02, PTS)
    # the start-scan path makes its own matches: seeds there are not a keyframe's predicted pose
    assert g._visit is None
    assert len(matcher.seeds) > n


# -- the alias verdict inside the gate -------------------------------------------------------------


def run_flagged(mp: pytest.MonkeyPatch, flags: list[bool], step: float = 0.05):  # type: ignore[no-untyped-def]
    """One lap of 5.2 m with a match every `step` from 0.6 m before the line, each called unique
    or ambiguous in turn (the uniqueness test is scripted, the ring is off), then the closing scan
    0.41 m after the line. Returns (closure or None, gate)."""
    car = Car(mp, unique_shift_m=0.4)
    it = iter(flags)
    mp.setattr(car.gate, "_classify", lambda *a, **k: UNIQUE if next(it) else AMBIGUOUS)
    length = 5.2
    car.steer_for(length)
    for i, _flag in enumerate(flags):
        x = -0.6 + step * i
        car.move_to(length + x)
        car.scan(result(x))
    car.move_to(length + 0.41)
    return car.scan(None), car.gate


@pytest.mark.parametrize(
    ("n_unique", "n_ambiguous", "verdict", "closes"),
    [
        (3, 0, "unproven", False),  # three unique matches are not proof: the closure is held
        (4, 0, "proven", True),
        (10, 2, "proven", True),
        (9, 3, "proven", True),
        (8, 3, "proven", True),  # brief 50e: the plain majority (50d held this one)
        (4, 4, "unproven", False),  # not a majority unique: held, and the hold does not run out
        (0, 3, "unproven", False),
        (2, 3, "unproven", False),
        (2, 6, "unproven", False),  # 0.75 is not over 0.75
        (0, 6, "aliased", False),  # a clear majority of enough matches
        (2, 7, "aliased", False),
    ],
)
def test_the_verdict_states_decide_whether_a_place_may_place_the_line(
    monkeypatch: pytest.MonkeyPatch, n_unique: int, n_ambiguous: int, verdict: str, closes: bool
) -> None:
    # unique matches first: the verdict latches as soon as it is reached
    flags = [True] * n_unique + [False] * n_ambiguous
    closed, gate = run_flagged(monkeypatch, flags)
    assert gate.start_verdict == verdict
    assert (closed is not None) is closes
    if closes:
        assert closed.line_source == "start_scan"  # type: ignore[attr-defined]
    assert gate.start_aliased is (verdict == "aliased")
    assert gate.start_proven is (verdict == "proven")


def test_an_aliased_start_scan_stays_aliased_between_the_two_shares_and_set_start_clears_it(
    monkeypatch: pytest.MonkeyPatch,
) -> None:
    # 5 unique, 8 ambiguous: share 0.62, between the release (0.5) and the entry (0.75) share
    closed, gate = run_flagged(monkeypatch, [False] * 8 + [True] * 5, step=0.02)
    assert gate.start_aliased and closed is None
    assert gate._life_unique == 5 and gate._cands == []
    assert gate.set_start(gate._hist[-1].t, PTS)
    assert gate.start_verdict == "unproven" and not gate.start_aliased


def test_a_placed_track_is_proven_before_the_line_so_the_crossing_does_not_wait(
    monkeypatch: pytest.MonkeyPatch,
) -> None:
    """12 unique matches before the line (what a stadium gives): proven long before v2.1 would
    close, so the crossing is v2.1's, at the same place."""
    closed, gate = run_flagged(monkeypatch, [True] * 12)
    assert gate.start_verdict == "proven" and closed is not None
    assert closed.length_m == pytest.approx(5.2, abs=0.04)  # type: ignore[attr-defined]


def test_a_held_closure_is_released_as_v21_when_the_ttl_passes_with_the_reference_unproven(
    monkeypatch: pytest.MonkeyPatch,
) -> None:
    """Scarce scans are not evidence of aliasing: two agreeing unique candidates, no more scans,
    `candidate_ttl_s` later the closure is v2.1's."""
    car = Car(monkeypatch, unique_shift_m=0.4)
    monkeypatch.setattr(car.gate, "_classify", lambda *a, **k: UNIQUE)
    length = 5.2
    car.steer_for(length)
    car.move_to(length - 0.2)
    car.scan(result(-0.2))
    car.move_to(length - 0.1)
    car.scan(result(-0.1))
    car.move_to(length + 0.41)
    assert car.scan(None) is None  # held
    assert not car.gate.station_uncertain  # a held closure is a pending candidate
    car.move_to(length + 0.41 + 1.6)  # more than candidate_ttl_s (1.5 s at 1 m/s) later
    closed = car.scan(None)
    assert closed is not None and closed.line_source == "start_scan"
    assert closed.length_m == pytest.approx(length, abs=0.1)


def test_a_held_closure_is_dropped_when_the_reference_turns_out_aliased(
    monkeypatch: pytest.MonkeyPatch,
) -> None:
    closed, gate = run_flagged(monkeypatch, [True, True] + [False] * 7)
    assert closed is None and gate.start_aliased and gate._cands == []


def test_an_aliased_start_scan_never_yields_start_scan_or_anchor_and_matches_no_keyframe(
    monkeypatch: pytest.MonkeyPatch,
) -> None:
    mem, laps = run(RING, 3.3, RING_SPEED)
    assert mem.gate is not None and mem.gate.start_aliased
    assert {r.line_source for r in laps} == {"winding"}
    assert not any(k.visited for k in mem.gate.ring.keyframes)  # no keyframe was matched
    assert mem.status()["ring"]["fixes_last_lap"] == 0  # type: ignore[index]


def test_a_start_scan_that_is_not_aliased_never_yields_winding() -> None:
    mem, laps = run(STADIUM, 3.3, 1.0)
    assert mem.gate is not None and mem.gate.start_proven
    assert {r.line_source for r in laps} == {"start_scan"}
    assert mem.status()["lap_start_verdicts"] == ["proven"] * len(laps)  # type: ignore[comparison-overlap]


def test_the_share_of_ambiguous_matches_is_reported_per_lap(
    monkeypatch: pytest.MonkeyPatch,
) -> None:
    closed, gate = run_flagged(monkeypatch, [True] * 10 + [False] * 2)
    assert closed is not None
    assert gate.alias_share_last_lap == pytest.approx(2 / 12)
    assert gate.status()["start_alias_share_last_lap"] == pytest.approx(2 / 12)


def test_with_no_displaced_scan_the_match_is_not_unique_and_places_nothing(
    monkeypatch: pytest.MonkeyPatch,
) -> None:
    """The scripted car's scans are sparse: no scan 0.4 m back, so nothing is unique and the lap
    is not closed (a lap with no evidence is never invented)."""
    car = Car(monkeypatch, unique_shift_m=0.4)
    car.steer_for(5.2)
    car.move_to(5.0)
    assert car.scan(result(-0.2)) is None
    car.move_to(5.1)
    assert car.scan(result(-0.1)) is None
    car.move_to(5.61)
    assert car.scan(None) is None


# -- keyframe fixes and retiring inside the gate -----------------------------------------------------


def test_a_keyframe_whose_matches_are_ambiguous_is_retired_and_gives_no_fix() -> None:
    """Keyframes of a ring: the stadium's matches are unique, a ring's are not. A latched aliased
    keyframe is retired for good; a fix needs a proven one."""
    g = straight_gate(unique_shift_m=0.4)
    roll(g, 0.0, 3.0)
    g.ring.finish_teaching(g.station_m)
    kf = g.ring.keyframes[2]
    kf.n_unique, kf.n_ambiguous = 2, 0
    g._visit = gate_module.Visit(kf)
    g._visit.estimates.append(gate_module.Estimate(10.0, 0.9, 0.02))
    g._visit.estimates.append(gate_module.Estimate(10.01, 0.9, 0.02))
    g._end_visit()
    assert g.ring.fixes == []  # two unique matches over its life: not proven (4 needed)
    kf.n_unique = 4
    g._visit = gate_module.Visit(kf)
    g._visit.estimates.append(gate_module.Estimate(10.0, 0.9, 0.02))
    g._visit.estimates.append(gate_module.Estimate(10.01, 0.9, 0.02))
    g._end_visit()
    assert len(g.ring.fixes) == 1  # proven and two agreeing estimates: a fix
    aliased = g.ring.keyframes[3]
    aliased.retired = True
    g._visit = gate_module.Visit(aliased)
    g._visit.estimates.append(gate_module.Estimate(10.0, 0.9, 0.02))
    g._visit.estimates.append(gate_module.Estimate(10.01, 0.9, 0.02))
    g._end_visit()
    assert len(g.ring.fixes) == 1  # an aliased keyframe gives none


def test_the_work_of_a_ring_lap_is_the_start_scan_match_and_the_pair_never_a_keyframe_search(
    monkeypatch: pytest.MonkeyPatch,
) -> None:
    """The cost guard, as a work count and not as milliseconds (a wall-clock bound failed on a slow
    shared runner at 72 ms against 60): on a ring the start scan is latched aliased on lap 1, so
    from lap 2 on no keyframe is matched and a scan costs the start-scan match (at most the seeds,
    the grid and the refinement of gate v2.1) plus the one ICP of the pair centre: ICP calls per
    scan, counted in this run, never more than that."""
    calls = {"n": 0}
    real = gate_module.match_scans

    def counting(*args, **kwargs):  # type: ignore[no-untyped-def]
        calls["n"] += 1
        return real(*args, **kwargs)

    per_scan: list[int] = []
    real_on_scan = StartScanGate._on_scan

    def measured(self, t, points):  # type: ignore[no-untyped-def]
        before = calls["n"]
        out = real_on_scan(self, t, points)
        if self.laps_completed >= 2:
            per_scan.append(calls["n"] - before)
        return out

    monkeypatch.setattr(gate_module, "match_scans", counting)
    monkeypatch.setattr(StartScanGate, "_on_scan", measured)
    mem, _ = run(RING, 5.2, RING_SPEED)
    gate = mem.gate
    assert gate is not None and gate.start_aliased
    assert not any(k.visited for k in gate.ring.keyframes)  # no keyframe was ever matched
    assert len(per_scan) > 200
    # v2.1's start-scan path: at most 3 seeds and 2 grid candidates plus 1 refinement, then the
    # pair centre's one ICP; the uniqueness test's ICP only when a candidate is considered
    assert max(per_scan) <= 8, max(per_scan)
    assert sum(per_scan) / len(per_scan) <= 4.0


# -- the order: the start scan wins over the anchor --------------------------------------------------


def test_a_disagreement_above_ring_check_m_is_counted_and_changes_no_crossing() -> None:
    base_mem, base = run(STADIUM, 3.3, 1.0)
    mem, laps = run(STADIUM, 3.3, 1.0, {"ring_check_m": 1e-5})
    assert [r.t_cross_s for r in laps] == [r.t_cross_s for r in base]  # not acted on
    assert all(r.line_source == "start_scan" for r in laps)
    assert base_mem.status()["ring_disagreements"] == 0
    assert mem.status()["ring_disagreements"] >= 1  # fixes differ from the scan by centimetres


def test_a_lap_with_a_unique_start_scan_crossing_is_not_replaced_by_an_anchor() -> None:
    mem, laps = run(STADIUM, 4.3, 1.0)
    assert mem.status()["lap_line_sources"] == ["start_scan"] * len(laps)
    assert mem.status()["ring"]["fixes_last_lap"] >= 1  # type: ignore[index]


# -- the live flag ------------------------------------------------------------------------------------


def test_the_live_flag_stays_down_while_an_anchor_or_winding_estimate_is_pending() -> None:
    """On a ring every lap closes by winding: `station_uncertain` never rises at the line and no
    lap is flagged missed."""
    mem = None
    flags: list[bool] = []

    def on_tick(m: object, _tick: object) -> None:
        flags.append(bool(m.status()["station_uncertain"]))  # type: ignore[attr-defined]

    from racer_laps.engine import LapResult
    from synth import feed
    from v3_support import make_mem

    mem = make_mem()
    out = [
        r
        for r in feed(mem, RING, 3.3, speed_mps=RING_SPEED, t0=0.0, on_tick=on_tick)
        if isinstance(r, LapResult)
    ]
    assert len(out) == 3 and not any(flags)
    assert not any(r.missed_gate for r in out)


def test_missed_stays_by_wheel_length_for_a_winding_lap(monkeypatch: pytest.MonkeyPatch) -> None:
    """A winding or anchor lap of normal length is not missed; one whose wheel length is above the
    estimate plus the band is, as for any lap."""
    g = straight_gate(ring_enabled=True)
    g._est.add(5.2)
    g._est.add(5.2)
    assert g.station_uncertain_at(g._origin_m + 5.2 + 0.4) is False
    assert g.station_uncertain_at(g._origin_m + 5.2 + 0.6) is True


def test_a_pending_anchor_counts_as_a_pending_candidate_for_the_live_flag() -> None:
    g = straight_gate()
    g.ring.finish_teaching(5.2)
    g._origin_m = 0.0
    g._est.add(5.2)
    g._est.add(5.2)
    g.ring.add_fix(Fix(3, 1.5, 1.5))  # E = 1.5 + (5.2 - 1.5) = 5.2
    g.on_steering(0.0)
    t = 0.0
    while g.total_m < 5.6:
        t += 0.02
        g.on_odom(t, 1.0)
    # estimate 5.2, uncertain after 5.2 + 0.5 = 5.7: pending until E + anchor_wait_m = 5.6
    assert not g.station_uncertain
    while g.total_m < 5.75:
        t += 0.02
        g.on_odom(t, 1.0)
    assert g.station_uncertain  # nothing pending any more and the band is used up


def test_the_ring_block_and_keys_of_the_status() -> None:
    mem, _ = run(Stadium(straight_m=2.0, lane_m=1.0, island_r_m=0.4), 1.2, 1.0)
    status = mem.status()
    for key in (
        "line_source",
        "lap_line_sources",
        "phase_observable",
        "missed_gate_laps",
        "ring",
        "start_alias_share_last_lap",
        "ring_disagreements",
        "on_scan_ms_mean",
        "on_scan_ms_p99",
        "stale_odometry_scans",
    ):
        assert key in status, key
    assert set(status["ring"]) == {"keyframes", "live", "retired", "fixes_last_lap", "generation"}  # type: ignore[arg-type]
    assert status["on_scan_ms_p99"] >= status["on_scan_ms_mean"] > 0.0  # type: ignore[operator]
