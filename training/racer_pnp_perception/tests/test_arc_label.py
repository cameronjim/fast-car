"""Arc-length label stations with rounded corners (arc_label.py)."""

from __future__ import annotations

import numpy as np
from racer_pnp_perception import arc_label as al
from racer_track_perception import track as tr


def square_corner(hw: float = 0.4) -> np.ndarray:
    """A raw polyline: 1.4 m straight along x, then a 90 degree left turn, 0.2 m steps."""
    a = [[x, 0.0, hw, hw, 0.05] for x in np.arange(0.0, 1.6, 0.2)]
    b = [[1.4, y, hw, hw, 0.05] for y in np.arange(0.2, 2.0, 0.2)]
    return np.array(a + b)


def gaps(st: np.ndarray, n: int) -> np.ndarray:
    return np.hypot(*np.diff(st[:n, :2], axis=0).T)


def test_stations_are_exactly_the_contract_spacing_apart_through_a_square_corner() -> None:
    dense = al.densify(square_corner())
    assert dense is not None
    for smooth in (0.0, 0.1, 0.2):
        st, n = al.chord_stations(al.smooth_centre(dense, smooth))
        assert n == tr.K_STATIONS
        assert np.allclose(gaps(st, n), tr.STATION_SPACING_M, atol=1e-9)
        assert np.allclose(st[0, :2], 0.0, atol=1e-9)  # the point nearest the car


def test_smoothing_rounds_the_corner_and_keeps_a_straight_lane_unchanged() -> None:
    dense = al.densify(square_corner())
    assert dense is not None

    def max_turn(smooth: float) -> float:
        st, n = al.chord_stations(al.smooth_centre(dense, smooth))
        d = np.diff(st[:n, :2], axis=0)
        h = np.unwrap(np.arctan2(d[:, 1], d[:, 0]))
        return float(np.abs(np.diff(h)).max())

    assert max_turn(0.2) < max_turn(0.1) < max_turn(0.0)
    straight = al.densify(np.array([[x, 0.1, 0.3, 0.5, 0.05] for x in np.arange(0, 3.6, 0.2)]))
    assert straight is not None
    sm = al.smooth_centre(straight, 0.2)
    assert np.allclose(sm[:, :4], straight[:, :4], atol=1e-9)


def test_half_widths_follow_the_centre_so_the_walls_stay_put() -> None:
    dense = al.densify(square_corner(0.4))
    assert dense is not None
    sm = al.smooth_centre(dense, 0.2)
    # Where the centre moved left by d, the left half width shrank by d and the right grew by d.
    assert np.allclose(sm[:, 2] + sm[:, 3], dense[:, 2] + dense[:, 3], atol=1e-9)
    assert (np.abs(sm[:, 2] - dense[:, 2]) > 0.01).any()


def test_a_short_trace_gives_fewer_stations_and_nothing_gives_no_track() -> None:
    dense = al.densify(np.array([[0, 0, 0.4, 0.4, 0.05], [1.0, 0, 0.4, 0.4, 0.05]]))
    assert dense is not None
    _st, n = al.chord_stations(dense)
    assert n == 6
    assert al.arc_track(np.zeros((0, 2)), 0.15, 0.25, 0.2).valid_count == 0
    assert al.densify(np.array([[0, 0, 0.4, 0.4, 0.05]])) is None


def test_arc_track_on_a_corridor_of_points_is_a_valid_label() -> None:
    # Two walls 0.8 m apart along x from -1 to 5 m.
    x = np.arange(-1.0, 5.0, 0.02)
    pts = np.concatenate([np.c_[x, np.full_like(x, 0.4)], np.c_[x, np.full_like(x, -0.4)]])
    t = al.arc_track(pts, 0.15, 0.25, 0.2)
    n = t.valid_count
    assert n == tr.K_STATIONS
    assert np.allclose(np.diff(t.x_m[:n]), tr.STATION_SPACING_M, atol=1e-6)
    assert np.allclose(t.y_m[:n], 0.0, atol=1e-6)
    assert np.allclose(t.half_width_left_m[:n], 0.4, atol=0.02)
