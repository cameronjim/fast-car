"""L1 and L2: the lap length estimate, the station set changes that follow it (drop a suspect lap,
rescale, zone merge switch) and the cap schedule's drop."""

from __future__ import annotations

import math

import numpy as np
import pytest
from hypothesis import given, settings
from hypothesis import strategies as st
from racer_laps.cap_schedule import CapSchedule
from racer_laps.gate import LapLengthEstimate
from racer_laps.station_set import StationSet
from test_station_set import lap_samples


def est(*laps: float) -> LapLengthEstimate:
    e = LapLengthEstimate()
    for n in laps:
        e.add(n)
    return e


# -- the estimate, table driven ------------------------------------------------------------------


def test_empty() -> None:
    e = est()
    assert math.isnan(e.value_m) and not e.trusted and e.n_laps == 0
    assert e.band_m() == 0.5
    assert not e.suspect(100.0)


def test_one_lap_is_a_value_but_not_trusted() -> None:
    e = est(9.95)
    assert e.value_m == 9.95 and not e.trusted
    assert not e.suspect(20.0)  # nothing is suspect before the estimate is trusted


def test_two_agreeing_laps_are_trusted() -> None:
    e = est(11.8, 11.99)  # 0.19 m apart, tolerance max(0.3, 0.47)
    assert e.trusted and e.value_m == pytest.approx(11.895)


def test_two_disagreeing_laps_are_not_trusted() -> None:
    e = est(9.95, 11.8)
    assert not e.trusted
    assert est(5.0, 5.31).trusted is False  # 0.31 > max(0.30, 0.20)
    assert est(5.0, 5.29).trusted is True


def test_the_median_is_over_the_last_five() -> None:
    e = est(1.0, 50.0, 10.0, 10.1, 10.0, 10.2, 9.9)
    assert e.value_m == pytest.approx(10.0)  # the last five: 10.0 10.1 10.0 10.2 9.9


def test_a_poisoned_first_lap_heals_by_lap_3_and_is_then_suspect() -> None:
    e = est(9.95)
    assert not e.trusted
    e.add(11.8)
    assert not e.trusted
    e.add(11.99)
    assert e.trusted and e.value_m == pytest.approx(11.8)
    assert e.suspect(9.95) and not e.suspect(11.72) and not e.suspect(11.99)
    e.add(11.72)
    assert e.value_m == pytest.approx(11.76, abs=0.01)


def test_the_band_is_half_a_metre_or_eight_percent() -> None:
    assert est(3.0, 3.0).band_m() == 0.5
    assert est(20.0, 20.0).band_m() == pytest.approx(1.6)
    e = est(10.0, 10.0)
    assert e.suspect(10.81) and not e.suspect(10.79)
    assert e.suspect(9.19) and not e.suspect(9.21)


def test_bad_lengths_are_ignored() -> None:
    e = est(math.nan, -1.0, 0.0, math.inf)
    assert e.n_laps == 1 or e.n_laps == 0
    assert e.n_laps == 0 or math.isfinite(e.value_m)
    e = est(math.nan, -1.0, 0.0)
    assert e.n_laps == 0


def test_clear() -> None:
    e = est(5.0, 5.0)
    e.clear()
    assert e.n_laps == 0 and not e.trusted


@settings(max_examples=60, deadline=None)
@given(
    true_len=st.floats(4.0, 40.0),
    poison=st.floats(0.3, 0.9),
    jitter=st.lists(st.floats(-0.02, 0.02), min_size=6, max_size=6),
)
def test_a_poisoned_first_lap_heals_whatever_the_length(
    true_len: float, poison: float, jitter: list[float]
) -> None:
    e = LapLengthEstimate()
    e.add(true_len * poison)  # closed early (or, with poison > 1 rules, whatever)
    for j in jitter:
        e.add(true_len * (1.0 + j))
    assert e.trusted
    assert e.value_m == pytest.approx(true_len, rel=0.03)
    assert e.suspect(true_len * poison) == (abs(true_len * poison - e.value_m) > e.band_m())


@settings(max_examples=60, deadline=None)
@given(lengths=st.lists(st.floats(1.0, 60.0), min_size=0, max_size=12))
def test_estimate_invariants(lengths: list[float]) -> None:
    e = LapLengthEstimate()
    for n in lengths:
        e.add(n)
    assert e.n_laps == len(lengths)
    if lengths:
        assert min(lengths[-5:]) <= e.value_m <= max(lengths[-5:])
        assert e.band_m() >= 0.5
    else:
        assert math.isnan(e.value_m)
    if len(lengths) < 2:
        assert not e.trusted
        assert not e.suspect(5.0)


# -- the cap schedule forgets a dropped lap ----------------------------------------------------------


def test_dropping_a_lap_replays_the_rest() -> None:
    s = CapSchedule(0.5, 0.1, 1.0)
    for k in (1, 2, 3):
        s.finish_lap(True, k)
    assert s.cap_mps == pytest.approx(0.8)
    assert s.drop_lap(2) == pytest.approx(0.7)
    assert s.drop_lap(2) == pytest.approx(0.7)  # twice is once
    assert s.drop_lap(99) == pytest.approx(0.7)  # unknown lap: nothing happens


def test_dropping_an_unclean_lap_undoes_the_drop() -> None:
    s = CapSchedule(0.5, 0.1, 1.0)
    s.finish_lap(True, 1)
    s.finish_lap(True, 2)
    s.finish_lap(False, 3)
    assert s.cap_mps == pytest.approx(0.6)
    assert s.drop_lap(3) == pytest.approx(0.7)


def test_unindexed_laps_are_not_replayable() -> None:
    s = CapSchedule(0.5, 0.1, 1.0)
    s.finish_lap(True)
    s.finish_lap(True)
    assert s.drop_lap(1) == pytest.approx(0.7)  # nothing indexed: unchanged


# -- the station set: remove, rescale, zone merge --------------------------------------------------


def stored_set(l_ref: float = 3.0, laps: int = 2) -> StationSet:
    s = StationSet()
    s.set_reference_length(l_ref)
    for k in range(laps):
        speeds = [0.5 + 0.1 * k + 0.01 * i for i in range(int(l_ref / 0.1))]
        station, t, v = lap_samples(l_ref, speeds, 6.0 + k)
        assert s.add_lap(k + 1, station, t, v, 6.0 + k, l_ref, 0.5)
    return s


def test_remove_lap() -> None:
    s = stored_set(laps=2)
    assert s.remove_lap(1) and s.n_laps == 1
    assert not s.remove_lap(1)
    assert s.best_lap_time_s == pytest.approx(7.0)
    assert s.remove_lap(2) and s.n_laps == 0
    assert math.isnan(s.best_lap_time_s)
    assert np.isnan(s.v_safe(0.5)).all()
    assert np.isnan(s.cost_to_go(0.5)).all()
    assert s.arrays(0.0, 1.0, 2, 0.2)[0].shape[0] == 0


def test_rescale_keeps_the_shape_on_the_new_grid() -> None:
    s = stored_set(l_ref=3.0, laps=1)
    before = s.v_safe(np.array([0.0, 0.75, 1.5, 2.25]))
    s.rescale(4.5)
    assert s.l_ref_m == 4.5 and s.n_laps == 1
    after = s.v_safe(np.array([0.0, 1.125, 2.25, 3.375]))  # the same fractions of the lap
    assert np.allclose(before, after, atol=0.02)
    assert s.cost_to_go(0.0)[0] == pytest.approx(s.best_lap_time_s)
    assert s.cost_to_go(4.5)[0] == pytest.approx(0.0, abs=1e-9)


def test_rescale_scales_zone_centres() -> None:
    s = stored_set(l_ref=3.0, laps=1)
    s.add_zone(1.5, 0.4, 1, "brake")
    s.rescale(6.0)
    assert s.zones[0].centre_m == pytest.approx(3.0)


def test_rescale_with_no_laps_just_sets_the_length() -> None:
    s = StationSet()
    s.set_reference_length(3.0)
    s.rescale(4.0)
    assert s.l_ref_m == 4.0 and s.n_laps == 0
    t = StationSet()
    t.rescale(5.0)
    assert t.l_ref_m == 5.0
    with pytest.raises(ValueError):
        t.rescale(0.0)


def test_zone_merge_switch() -> None:
    merged = StationSet(zone_merge=True)
    split = StationSet(zone_merge=False)
    for s in (merged, split):
        s.set_reference_length(3.0)
        s.add_zone(1.0, 0.5, 1, "blocked")
        s.add_zone(1.2, 0.3, 1, "blocked")
    assert len(merged.zones) == 1 and merged.zones[0].speed_mps == pytest.approx(0.3)
    assert len(split.zones) == 2
    assert StationSet().zone_merge is True


@settings(max_examples=40, deadline=None)
@given(
    merge=st.booleans(),
    floor=st.floats(0.0, 0.5),
    explore=st.floats(0.0, 0.5),
    zones=st.lists(
        st.tuples(st.floats(0.0, 6.0), st.floats(0.0, 1.5), st.integers(1, 3)), max_size=8
    ),
    lap=st.integers(1, 6),
)
def test_v_bound_stays_between_the_floor_and_v_safe_plus_explore(
    merge: bool, floor: float, explore: float, zones: list[tuple[float, float, int]], lap: int
) -> None:
    s = StationSet(floor_mps=floor, zone_merge=merge)
    s.set_reference_length(3.0)
    speeds = [0.3 + 0.02 * (i % 20) for i in range(30)]
    station, t, v = lap_samples(3.0, speeds, 6.0)
    assert s.add_lap(1, station, t, v, 6.0, 3.0, 0.5)
    for centre, speed, created in zones:
        s.add_zone(centre, speed, created, "brake")
    grid = np.linspace(0.0, 9.0, 91)
    bound = s.v_bound(grid, lap, explore)
    safe = s.v_safe(grid)
    assert np.all(bound >= floor - 1e-12)
    assert np.all(bound <= np.maximum(safe + explore, floor) + 1e-12)
