"""L1: the keyframe ring's bookkeeping (ring.py): teaching, selection, the alias verdict, retiring
and the anchor estimate. No scans: the keyframes carry empty references."""

from __future__ import annotations

import math

import numpy as np
import pytest
from racer_laps.ring import Fix, Ref, Ring, is_aliased, make_ref
from racer_laps.scan_match import thin

EMPTY = Ref(np.zeros((0, 2)), np.zeros((0, 2)), np.zeros((0, 2)), np.zeros(0))


def make_ring(**kw: float) -> Ring:
    opts = {"spacing_m": 0.5, "kf_max": 120, "window_m": 0.3, **kw}
    return Ring(float(opts["spacing_m"]), int(opts["kf_max"]), float(opts["window_m"]))


def taught(n: int = 12, **kw: float) -> Ring:
    """A ring taught over n keyframes at 0.5 m, closed after n * 0.5 m."""
    ring = make_ring(**kw)
    for i in range(n):
        ring.add(0.5 * i, 0.1 * i, 100.0 + 0.5 * i, EMPTY)
    ring.finish_teaching(0.5 * n)
    return ring


@pytest.mark.parametrize(
    ("n_unique", "n_ambiguous", "aliased"),
    [
        (0, 0, False),  # nothing seen
        (10, 2, False),  # two ambiguous: below alias_min_scans
        (0, 2, False),
        (0, 3, True),  # three, share 1.0, no unique match at all
        (9, 3, False),  # share exactly 0.25 is not above 0.25
        (8, 3, True),  # 3 / 11 > 0.25
        (11, 4, True),  # 4 / 15
        (50, 3, False),
        (3, 9, True),  # a place that is mostly ambiguous although some matches are unique
    ],
)
def test_the_alias_verdict_table(n_unique: int, n_ambiguous: int, aliased: bool) -> None:
    assert is_aliased(n_unique, n_ambiguous, 3, 0.25) is aliased


def test_the_alias_share_is_strictly_above_the_limit() -> None:
    assert not is_aliased(9, 3, 3, 0.25)
    assert is_aliased(9, 3, 3, 0.25 - 1e-9)


def test_a_ring_that_was_never_taught_is_not_fixed() -> None:
    ring = make_ring()
    assert ring.teaching and not ring.fixed and not ring.keyframes
    assert math.isnan(ring.anchor_total_m)
    assert ring.nearest_live(0.0) is None


def test_keyframes_are_due_every_spacing_of_station_and_stations_increase() -> None:
    ring = make_ring()
    assert ring.due(0.0)  # the first one is always due
    ring.add(0.0, 0.0, 10.0, EMPTY)
    assert not ring.due(0.49)
    assert ring.due(0.5)
    ring.add(0.5, 0.1, 10.5, EMPTY)
    ring.add(1.0, 0.2, 11.0, EMPTY)
    stations = [k.station_m for k in ring.keyframes]
    assert stations == sorted(stations) and len(set(stations)) == 3


def test_at_kf_max_every_second_keyframe_is_dropped_and_the_spacing_doubles() -> None:
    ring = make_ring(kf_max=6)
    for i in range(6):
        ring.add(0.5 * i, 0.0, 0.5 * i, EMPTY)
    assert len(ring.keyframes) == 6 and ring.spacing_m == 0.5
    ring.add(3.0, 0.0, 3.0, EMPTY)
    assert [k.station_m for k in ring.keyframes] == [0.0, 1.0, 2.0, 3.0]
    assert ring.spacing_m == 1.0
    assert not ring.due(3.9) and ring.due(4.0)
    assert ring.keyframes[0].index == 0  # keyframe 0 stays


def test_finishing_the_teaching_lap_fixes_the_ring_and_drops_the_last_half_spacing() -> None:
    ring = make_ring()
    for i in range(13):  # up to station 6.0
        ring.add(0.5 * i, 0.0, 0.5 * i, EMPTY)
    ring.finish_teaching(5.9)
    assert ring.fixed and ring.length_m == 5.9
    assert max(k.station_m for k in ring.keyframes) == 5.5  # 5.5 <= 5.9 - 0.25, 6.0 is gone
    assert ring.keyframes[0].station_m == 0.0


@pytest.mark.parametrize(
    ("station", "expected"),
    [
        (2.0, 2.0),
        (2.1, 2.0),  # nearest
        (2.26, 2.5),
        (2.3, 2.5),
        (0.0, 0.0),
        (-0.3, 0.0),  # exactly the window below keyframe 0
        (-0.31, None),  # epsilon over
        (5.5 + 0.3, 5.5),  # exactly at the window edge
        (5.5 + 0.31, None),
    ],
)
def test_the_nearest_live_keyframe_within_the_window(
    station: float, expected: float | None
) -> None:
    ring = taught(12)
    kf = ring.nearest_live(station)
    if expected is None:
        assert kf is None
    else:
        assert kf is not None and kf.station_m == pytest.approx(expected)


def test_a_retired_keyframe_is_never_selected_again() -> None:
    ring = taught(12)
    k4 = ring.nearest_live(2.0)
    assert k4 is not None
    k4.retired = True  # aliased: the gate retires it for good
    assert ring.n_retired == 1 and ring.n_live == 11
    assert ring.nearest_live(2.0) is None  # the neighbours are 0.5 m away, outside the window
    nxt = ring.nearest_live(2.2)
    assert nxt is not None and nxt.station_m == pytest.approx(2.5)


def test_the_counts_of_a_keyframe_belong_to_the_keyframe_not_to_a_visit_or_a_lap() -> None:
    ring = taught(12)
    k = ring.keyframes[3]
    assert (k.n_unique, k.n_ambiguous) == (0, 0)
    k.n_unique += 4
    ring.note_visit(k)
    ring.end_lap()  # a lap ends: the place keeps what it learned
    assert (k.n_unique, k.n_ambiguous) == (4, 0) and not k.retired


def test_the_anchor_estimate_uses_the_latest_fix_of_the_lap() -> None:
    ring = taught(12)  # L_ring 6.0
    assert math.isnan(ring.anchor_total_m)
    ring.add_fix(Fix(2, 1.0, 201.0))
    assert ring.anchor_total_m == pytest.approx(201.0 + 5.0)
    ring.add_fix(Fix(5, 2.5, 202.6))  # later: this one wins, not the first or an average
    assert ring.anchor_total_m == pytest.approx(202.6 + 3.5)


def test_a_lap_without_a_fix_twice_with_live_keyframes_teaches_again() -> None:
    ring = taught(12)

    def lap_without_fix() -> None:
        ring.note_visit(ring.keyframes[3])  # visited, no fix
        ring.end_lap()

    lap_without_fix()
    assert ring.fixed and ring.laps_without_fix == 1 and ring.generation == 0
    ring.add_fix(Fix(2, 1.0, 1.0))
    ring.end_lap()  # a fix resets the count
    assert ring.fixed and ring.laps_without_fix == 0
    lap_without_fix()
    lap_without_fix()
    assert ring.teaching and ring.generation == 1 and not ring.keyframes
    assert ring.fixes_last_lap == 0


def test_keyframes_that_were_not_visited_do_not_keep_the_ring_alive() -> None:
    """The last 40 percent of a lap is tracked by the start scan: those keyframes are live but never
    visited, and a ring whose visited keyframes are all retired is not taught again."""
    ring = taught(12)
    for k in ring.keyframes[1:7]:
        k.retired = True  # aliased
        ring.note_visit(k)
    assert all(k.retired for k in ring.keyframes[1:7])
    for _ in range(4):
        ring.end_lap()
    assert ring.fixed and ring.generation == 0 and ring.n_live == 6


def test_no_re_teaching_when_only_keyframe_0_is_left_live() -> None:
    """On a ring every keyframe retires within two laps and the cost falls back to v2.1's: a ring
    with nothing live but keyframe 0 is not taught again every second lap."""
    ring = taught(12)
    for k in ring.keyframes[1:]:
        k.retired = True
    ring.end_lap()
    ring.end_lap()
    ring.end_lap()
    assert ring.fixed and ring.generation == 0


def test_reset_forgets_everything() -> None:
    ring = taught(12, kf_max=12)
    ring.add_fix(Fix(1, 0.5, 1.0))
    ring.reset()
    assert ring.teaching and not ring.keyframes and ring.generation == 0
    assert math.isnan(ring.length_m) and ring.fixes == []


def test_status_counts() -> None:
    ring = taught(12)
    ring.keyframes[3].retired = True
    ring.add_fix(Fix(1, 0.5, 1.0))
    ring.end_lap()
    assert ring.status() == {
        "keyframes": 12,
        "live": 11,
        "retired": 1,
        "fixes_last_lap": 1,
        "generation": 0,
    }


def test_bad_ring_settings_are_refused() -> None:
    for kw in ({"spacing_m": 0.0}, {"kf_max": 1}, {"window_m": 0.0}):
        with pytest.raises(ValueError):
            make_ring(**kw)


def test_a_compact_keyframe_of_640_points_is_small() -> None:
    """120 keyframes of about 640 points and 360 ranges stay under 2 MB."""
    rng = np.random.default_rng(1)
    pts = rng.normal(size=(640, 2))
    thin_pts = thin(pts, 1e-3, 220)
    dense = thin(pts, 1e-3, 420)
    ref = make_ref(pts, thin_pts, dense, (0.0, 0.0), compact=True)
    nbytes = ref.pts.nbytes + ref.normals.nbytes + ref.dense.nbytes + ref.profile.nbytes
    assert ref.pts.dtype == np.float32 and ref.profile.shape == (360,)
    assert 120 * nbytes < 2_000_000
    full = make_ref(pts, thin_pts, dense, (0.0, 0.0))
    assert full.pts.dtype == np.float64
