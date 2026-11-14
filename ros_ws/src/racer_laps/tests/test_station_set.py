"""L1: the station set (D v1): resampling, the MIN filter, Q across the lap end, caution zones,
the guidance arrays and persistence. Numbers are computed by hand."""

from __future__ import annotations

import math
from pathlib import Path

import numpy as np
import pytest
from racer_laps.station_set import StationSet, resample_station_lap


def lap_samples(l_ref: float, speeds: list[float], lap_time: float):
    """Samples at the station grid points of a lap of length l_ref with the given speeds and a
    constant pace in time."""
    n = len(speeds)
    station = np.linspace(0.0, l_ref, n + 1)
    t = np.linspace(0.0, lap_time, n + 1)
    v = np.array(speeds + [speeds[-1]], dtype=float)
    return station, t, v


def make_set(l_ref: float = 0.3, **kw: float) -> StationSet:
    s = StationSet(**kw)  # type: ignore[arg-type]
    s.set_reference_length(l_ref)
    return s


# -- reference length ----------------------------------------------------------------------------


def test_the_reference_length_needs_to_exceed_the_spacing_and_is_final_once_laps_exist() -> None:
    s = StationSet()
    with pytest.raises(ValueError):
        s.set_reference_length(0.05)
    with pytest.raises(ValueError):
        s.set_reference_length(math.nan)
    s.set_reference_length(0.3)
    assert s.l_ref_m == 0.3
    st, t, v = lap_samples(0.3, [0.1, 0.1, 0.1], 3.0)
    assert s.add_lap(1, st, t, v, 3.0, 0.3, 0.5)
    s.set_reference_length(5.0)  # ignored: laps are stored
    assert s.l_ref_m == 0.3


def test_bad_constructor_arguments() -> None:
    with pytest.raises(ValueError):
        StationSet(spacing_m=0.0)
    with pytest.raises(ValueError):
        StationSet(caution_before_m=0.0)
    with pytest.raises(ValueError):
        StationSet(caution_laps=-1)


# -- resampling and Q ----------------------------------------------------------------------------


def test_three_station_lap_cost_to_go_by_hand() -> None:
    s = make_set(0.3)
    st, t, v = lap_samples(0.3, [0.1, 0.2, 0.3], 3.0)  # stations 0, 0.1, 0.2 (and the end)
    assert s.add_lap(1, st, t, v, 3.0, 0.3, 0.5)
    # time to finish at 0, 0.1, 0.2, 0.3 is 3, 2, 1, 0
    q = s.cost_to_go([0.0, 0.1, 0.2, 0.3])
    assert q == pytest.approx([3.0, 2.0, 1.0, 0.0], abs=1e-9)
    # between stations: linear
    assert s.cost_to_go(0.05)[0] == pytest.approx(2.5)
    # past the lap end: Q(L + x) = Q(x) - T_best
    assert s.cost_to_go(0.35)[0] == pytest.approx(2.5 - 3.0)
    assert s.cost_to_go(0.3)[0] == pytest.approx(0.0, abs=1e-9)
    assert s.cost_to_go(0.65)[0] == pytest.approx(2.5 - 6.0)
    # monotone across the end and two laps beyond
    grid = np.linspace(0.0, 1.0, 101)
    assert np.all(np.diff(s.cost_to_go(grid)) <= 1e-9)
    assert s.best_lap_time_s == 3.0


def test_q_is_the_minimum_over_laps_and_stays_non_increasing() -> None:
    s = make_set(0.3)
    st, t, v = lap_samples(0.3, [0.1, 0.1, 0.1], 3.0)
    s.add_lap(1, st, t, v, 3.0, 0.3, 0.5)
    # a faster second lap: 2.4 s, fast start
    st2 = np.array([0.0, 0.1, 0.2, 0.3])
    t2 = np.array([0.0, 0.5, 1.4, 2.4])
    s.add_lap(2, st2, t2, np.array([0.3, 0.3, 0.3, 0.3]), 2.4, 0.3, 0.6)
    q = s.cost_to_go([0.0, 0.1, 0.2, 0.3])
    # min(3, 2.4), min(2, 1.9), min(1, 1.0), 0
    assert q == pytest.approx([2.4, 1.9, 1.0, 0.0], abs=1e-9)
    assert s.best_lap_time_s == 2.4
    assert s.cost_to_go(0.3)[0] == pytest.approx(0.0, abs=1e-9)  # Q(0) - T_best
    assert np.all(np.diff(s.cost_to_go(np.linspace(0, 1.5, 151))) <= 1e-9)


def test_laps_are_rescaled_to_the_reference_length() -> None:
    s = make_set(1.0)
    # a lap of 1.1 m wheel distance: stations are scaled by 1.0 / 1.1
    n = 11
    station = np.linspace(0.0, 1.1, n + 1)
    t = np.linspace(0.0, 5.0, n + 1)
    v = np.full(n + 1, 0.5)
    assert s.add_lap(1, station, t, v, 5.0, 1.1, 0.5)
    # halfway in station is halfway in time
    assert s.cost_to_go(0.5)[0] == pytest.approx(2.5, abs=0.05)
    assert s.v_safe(0.5)[0] == pytest.approx(0.5)


def test_a_lap_with_no_motion_or_bad_numbers_is_not_stored() -> None:
    s = make_set(1.0)
    flat = np.zeros(5)
    assert not s.add_lap(1, flat, np.arange(5.0), np.ones(5), 5.0, 1.0, 0.5)
    st = np.linspace(0, 1, 5)
    assert not s.add_lap(1, st, np.arange(5.0), np.full(5, np.nan), 5.0, 1.0, 0.5)
    assert not s.add_lap(1, st, np.arange(5.0), np.ones(5), 0.0, 1.0, 0.5)
    assert not s.add_lap(1, st, np.arange(5.0), np.ones(5), 5.0, 0.0, 0.5)
    assert not s.add_lap(1, st[:1], np.arange(1.0), np.ones(1), 5.0, 1.0, 0.5)
    assert s.n_laps == 0
    unset = StationSet()
    assert not unset.add_lap(1, st, np.arange(5.0), np.ones(5), 5.0, 1.0, 0.5)
    assert resample_station_lap(st, np.arange(5.0), np.ones(5), 5.0, 1.0, 1.0, 1) is None


def test_nothing_is_defined_before_the_first_lap() -> None:
    s = make_set(1.0)
    assert np.all(np.isnan(s.v_safe(0.2)))
    assert np.all(np.isnan(s.v_bound(0.2, 1, 0.2)))
    assert np.all(np.isnan(s.cost_to_go(0.2)))
    assert math.isnan(s.max_bound(1, 0.2))
    a, b, c = s.arrays(0.0, 6.0, 1, 0.2)
    assert a.size == b.size == c.size == 0


# -- the MIN filter ------------------------------------------------------------------------------


def test_the_min_filter_spreads_a_slow_spot_over_plus_or_minus_0_4_m() -> None:
    s = make_set(1.0)  # n = 10, spacing 0.1: the filter half width is 4 stations
    speeds = [1.0] * 10
    speeds[5] = 0.2
    st, t, v = lap_samples(1.0, speeds, 5.0)
    s.add_lap(1, st, t, v, 5.0, 1.0, 1.0)
    grid = np.arange(10) * 0.1
    v_safe = s.v_safe(grid)
    # indices 1..9 are within 4 stations of index 5, circularly index 0 is 5 away
    assert v_safe[0] == pytest.approx(1.0)
    assert np.all(v_safe[1:10] == pytest.approx(0.2))
    # a second lap that was fast there: v_safe is the max of the filtered laps
    speeds2 = [0.5] * 10
    st, t, v = lap_samples(1.0, speeds2, 4.0)
    s.add_lap(2, st, t, v, 4.0, 1.0, 1.0)
    assert np.all(s.v_safe(grid)[1:10] == pytest.approx(0.5))
    assert s.v_safe(grid)[0] == pytest.approx(1.0)


def test_the_filter_is_periodic() -> None:
    s = make_set(1.0)
    speeds = [1.0] * 10
    speeds[0] = 0.6  # slow, but not a standing start
    st, t, v = lap_samples(1.0, speeds, 5.0)
    s.add_lap(1, st, t, v, 5.0, 1.0, 1.0)
    v_safe = s.v_safe(np.arange(10) * 0.1)
    # the slow start spreads across the lap end: indices 6..9 and 0..4
    assert np.all(v_safe[[6, 7, 8, 9, 0, 1, 2, 3, 4]] == pytest.approx(0.6))
    assert v_safe[5] == pytest.approx(1.0)


def test_v_bound_adds_the_exploration_step_outside_zones() -> None:
    s = make_set(1.0)
    st, t, v = lap_samples(1.0, [0.5] * 10, 5.0)
    s.add_lap(1, st, t, v, 5.0, 1.0, 0.5)
    assert s.v_bound(0.3, 2, 0.2)[0] == pytest.approx(0.7)
    assert s.v_bound(0.3, 2, 0.0)[0] == pytest.approx(0.5)
    assert s.max_bound(2, 0.2) == pytest.approx(0.7)
    # wraps past the lap end
    assert s.v_bound(1.3, 2, 0.2)[0] == pytest.approx(0.7)


# -- caution zones -------------------------------------------------------------------------------


def stored_long_lap() -> StationSet:
    s = make_set(3.0)  # 30 stations
    st, t, v = lap_samples(3.0, [0.8] * 30, 4.0)
    s.add_lap(1, st, t, v, 4.0, 3.0, 0.8)
    return s


def test_zone_creation_bound_and_expiry_after_two_laps() -> None:
    s = stored_long_lap()
    z = s.add_zone(centre_m=2.0, speed_mps=0.6, created_lap=2, reason="brake")
    assert z is not None and z.length_m == 1.0 and z.hold_laps == 2
    # not active on the lap it was created on, active on laps 3 and 4, gone from 5
    inside = np.array([1.0, 1.5, 2.0])  # within the metre before the event at 2.0
    outside = np.array([0.5, 2.1, 2.9])
    for lap, active in ((2, False), (3, True), (4, True), (5, False)):
        b = s.v_bound(inside, lap, 0.2)
        if active:
            assert np.all(b == pytest.approx(0.5)), lap  # min(v_safe 0.8, speed 0.6 - 0.1)
        else:
            assert np.all(b == pytest.approx(1.0)), lap  # v_safe + explore, no zone
        assert np.all(s.v_bound(outside, lap, 0.2) == pytest.approx(1.0))
    s.expire_zones(4)
    assert len(s.zones) == 1
    s.expire_zones(5)
    assert s.zones == ()


def test_in_a_zone_the_bound_never_exceeds_the_speed_driven_there_minus_0_1_and_has_no_step() -> (
    None
):
    s = stored_long_lap()
    s.add_zone(1.5, 0.25, 1, "clearance")
    b = s.v_bound(np.linspace(0.5, 1.5, 11), 2, 0.5)
    assert np.all(b <= 0.25 - 0.1 + 1e-12)
    # a zone speed below 0.1 gives 0, never negative
    s2 = stored_long_lap()
    s2.add_zone(1.5, 0.05, 1, "brake")
    assert np.all(s2.v_bound(np.linspace(0.5, 1.5, 11), 2, 0.5) == 0.0)
    # a zone faster than v_safe is capped by v_safe, not raised
    s3 = stored_long_lap()
    s3.add_zone(1.5, 2.0, 1, "brake")
    assert np.all(s3.v_bound(np.linspace(0.5, 1.5, 11), 2, 0.5) == pytest.approx(0.8))


def test_a_zone_wraps_around_the_lap_start_and_zones_combine() -> None:
    s = stored_long_lap()
    s.add_zone(0.3, 0.5, 1, "blocked")  # covers 2.3..3.0 and 0..0.3
    s.add_zone(2.0, 0.4, 1, "brake")  # covers 1.0..2.0
    b = s.v_bound(np.array([2.5, 0.1, 1.5, 0.6]), 2, 0.2)
    assert b == pytest.approx([0.4, 0.4, 0.3, 1.0])  # 0.5 - 0.1, 0.5 - 0.1, 0.4 - 0.1, free


def test_a_zone_before_the_reference_length_is_not_stored() -> None:
    s = StationSet()
    assert s.add_zone(1.0, 0.5, 1, "brake") is None
    t = make_set(1.0)
    assert t.add_zone(math.nan, 0.5, 1, "brake") is None
    assert t.add_zone(1.0, math.nan, 1, "brake") is None


# -- the guidance arrays -------------------------------------------------------------------------


def test_arrays_are_61_stations_every_0_1_m_unwrapped_past_the_lap_end() -> None:
    s = make_set(3.0)
    st, t, v = lap_samples(3.0, [0.8] * 30, 4.0)
    s.add_lap(1, st, t, v, 4.0, 3.0, 0.8)
    a, b, c = s.arrays(2.5, 6.0, 2, 0.2)
    assert a.shape == b.shape == c.shape == (61,)
    assert a[0] == pytest.approx(2.5) and a[-1] == pytest.approx(8.5)
    assert np.allclose(np.diff(a), 0.1)
    assert np.all(b >= 0.0)
    assert np.all(np.diff(c) <= 1e-9)  # non-increasing across two lap ends
    assert c[0] == pytest.approx(0.667, abs=0.01)  # 4.0 * (3.0 - 2.5) / 3.0
    assert c[-1] < 0.0


def test_arrays_are_empty_for_a_non_finite_station() -> None:
    s = stored_long_lap()
    assert s.arrays(math.nan, 6.0, 2, 0.2)[0].size == 0


# -- persistence ---------------------------------------------------------------------------------


def test_save_and_load_round_trip(tmp_path: Path) -> None:
    s = stored_long_lap()
    s.add_zone(2.0, 0.6, 1, "brake")
    path = tmp_path / "sub" / "set.npz"
    s.save(path)
    assert not list(path.parent.glob("*.tmp"))
    again = StationSet.load(path)
    assert again.n_laps == 1 and again.l_ref_m == 3.0
    grid = np.linspace(0, 3, 31)
    assert np.allclose(again.v_safe(grid), s.v_safe(grid))
    assert np.allclose(again.cost_to_go(grid), s.cost_to_go(grid))
    assert len(again.zones) == 1
    assert np.allclose(again.v_bound(grid, 2, 0.2), s.v_bound(grid, 2, 0.2))
    empty = StationSet()
    empty.save(tmp_path / "empty.npz")
    assert StationSet.load(tmp_path / "empty.npz").n_laps == 0
    bad = tmp_path / "bad.npz"
    np.savez(bad, format_version=np.array([99.0]))
    with pytest.raises(ValueError):
        StationSet.load(bad)


def test_overlapping_zones_take_the_lowest_cap_whatever_their_order() -> None:
    for order in ((0.3, 0.9), (0.9, 0.3)):
        s = stored_long_lap()
        for speed in order:
            s.add_zone(1.5, speed, 1, "brake")
        b = s.v_bound(np.array([1.0, 1.5]), 2, 0.2)
        assert b == pytest.approx([0.2, 0.2])  # 0.3 - 0.1


def test_a_standing_start_does_not_become_the_speed_profile_at_the_start_line() -> None:
    s = make_set(1.0)
    speeds = [0.0, 0.0, 0.1, 0.3, 0.5, 0.7, 0.7, 0.7, 0.7, 0.7]
    st, t, v = lap_samples(1.0, speeds, 5.0)
    s.add_lap(1, st, t, v, 5.0, 1.0, 0.5)
    v_safe = s.v_safe(np.arange(10) * 0.1)
    assert np.all(v_safe >= 0.7 - 1e-9)  # the car crosses the line at 0.7 on every later lap


def test_no_bound_is_below_the_floor_and_a_flickering_state_makes_one_zone() -> None:
    s = StationSet(floor_mps=0.3)
    s.set_reference_length(3.0)
    st, t, v = lap_samples(3.0, [0.8] * 30, 4.0)
    s.add_lap(1, st, t, v, 4.0, 3.0, 0.8)
    s.add_zone(1.5, 0.2, 1, "blocked")
    s.add_zone(1.55, 0.6, 1, "blocked")  # the same spot, same lap: merged, lower speed kept
    s.add_zone(1.45, 0.9, 1, "clearance")
    assert len(s.zones) == 1 and s.zones[0].speed_mps == pytest.approx(0.2)
    b = s.v_bound(np.linspace(0.5, 1.5, 11), 2, 0.2)
    assert np.all(b >= 0.3) and np.all(b[-3:] == pytest.approx(0.3))
    s.add_zone(1.5, 0.2, 2, "blocked")  # another lap: its own zone
    assert len(s.zones) == 2
