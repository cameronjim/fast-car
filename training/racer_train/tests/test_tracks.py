"""L1 tests for the procedural track family (racer_train.tracks): hand-computed geometry,
determinism, wall/lane consistency, and the reference-line planner's contract."""

from __future__ import annotations

import math

import numpy as np
import pytest
from racer_train import tracks
from racer_train.tracks import (
    TrackFamilyConfig,
    TrackSpec,
    blob_centreline,
    build_pool,
    build_track,
    plan_reference_line,
    rounded_rect_centreline,
    sample_spec,
    stadium_centreline,
)


def _perimeter(x: np.ndarray, y: np.ndarray) -> float:
    return float(np.hypot(np.diff(np.append(x, x[0])), np.diff(np.append(y, y[0]))).sum())


def test_stadium_perimeter_is_two_straights_plus_one_circle():
    straight, radius = 2.0, 0.7
    x, y = stadium_centreline(straight, radius)
    assert _perimeter(x, y) == pytest.approx(2 * straight + 2 * math.pi * radius, rel=5e-3)


def test_rounded_rect_perimeter_hand_computed():
    w, h, r = 3.0, 2.5, 0.6
    x, y = rounded_rect_centreline(w, h, r)
    expected = 2 * (w - 2 * r) + 2 * (h - 2 * r) + 2 * math.pi * r
    assert _perimeter(x, y) == pytest.approx(expected, rel=5e-3)


def test_rounded_rect_with_zero_radius_is_a_plain_rectangle():
    x, y = rounded_rect_centreline(3.0, 2.0, 0.0)
    assert _perimeter(x, y) == pytest.approx(2 * (3.0 + 2.0), rel=1e-3)


def test_rounded_rect_rejects_a_radius_that_does_not_fit():
    with pytest.raises(ValueError):
        rounded_rect_centreline(1.0, 3.0, 0.6)


def test_blob_is_a_closed_curve_with_requested_mean_scale():
    x, y = blob_centreline(2.0, ((0.0, 0.0), (0.0, 0.0), (0.0, 0.0)))
    assert np.allclose(np.hypot(x, y), 2.0)


def test_stadium_ccw_is_counter_clockwise_by_signed_area():
    x, y = stadium_centreline(2.0, 0.8)
    area = 0.5 * float(np.sum(x * np.roll(y, -1) - np.roll(x, -1) * y))
    assert area > 0.0


def _stadium_spec(direction: int, inner: float = 0.4, straight: float = 2.5) -> TrackSpec:
    return TrackSpec(
        shape="stadium",
        direction=direction,
        params=(("straight_m", straight), ("radius_m", inner + 0.5), ("inner_radius_m", inner)),
    )


def test_direction_flips_the_sign_of_curvature():
    fam = TrackFamilyConfig()
    ccw = build_track(_stadium_spec(+1), fam)
    cw = build_track(_stadium_spec(-1), fam)
    k_ccw = [ccw.raceline.at(i).curvature_1pm for i in range(len(ccw.raceline))]
    k_cw = [cw.raceline.at(i).curvature_1pm for i in range(len(cw.raceline))]
    assert max(k_ccw) > 0.5 and min(k_ccw) > -0.2  # left turns, positive (06-vehicle-params)
    assert min(k_cw) < -0.5 and max(k_cw) < 0.2


def test_build_track_is_deterministic():
    fam = TrackFamilyConfig()
    a = build_track(_stadium_spec(+1), fam)
    b = build_track(_stadium_spec(+1), fam)
    assert [p.x_m for p in (a.raceline.at(i) for i in range(len(a.raceline)))] == [
        p.x_m for p in (b.raceline.at(i) for i in range(len(b.raceline)))
    ]
    assert np.array_equal(a.gym_track.occupancy_map, b.gym_track.occupancy_map)


def test_speed_profile_respects_cap_and_slows_for_tight_turns():
    fam = TrackFamilyConfig(v_max_mps=2.0, a_lat_max_mps2=2.0)
    t = build_track(_stadium_spec(+1, inner=0.0), fam)
    speeds = np.array([t.raceline.at(i).target_speed_mps for i in range(len(t.raceline))])
    assert speeds.max() <= 2.0 + 1e-9
    assert speeds.min() < 1.6  # the U-turn is slower than the straight
    assert speeds.max() > 1.9  # the straight reaches the cap


def test_reference_line_stays_inside_the_offset_bound():
    x, y = stadium_centreline(2.0, 0.5)
    rx, ry = plan_reference_line(x, y, max_offset_m=0.25)
    # Distance from every reference point to the centreline polyline is at most the bound.
    cx, cy = stadium_centreline(2.0, 0.5, ds=0.01)
    d = np.hypot(rx[:, None] - cx[None, :], ry[:, None] - cy[None, :]).min(axis=1)
    assert d.max() <= 0.25 + 0.02


def test_reference_line_leaves_gentle_curves_and_straights_alone():
    x, y = stadium_centreline(4.0, 2.5)  # curvature 0.4 1/m, below the widening threshold
    rx, ry = plan_reference_line(x, y, max_offset_m=0.25)
    cx, cy = stadium_centreline(4.0, 2.5, ds=0.01)
    d = np.hypot(rx[:, None] - cx[None, :], ry[:, None] - cy[None, :]).min(axis=1)
    assert d.max() < 0.02


def test_reference_line_widens_a_tight_turn_outward():
    x, y = stadium_centreline(2.0, 0.5)
    rx, _ry = plan_reference_line(x, y, max_offset_m=0.25)
    # Furthest-from-origin reference point is further out than the centreline's own extent
    # in x (half straight + radius = 1.5): the U-turn apex moved outward.
    assert rx.max() > 1.5 + 0.1


def test_walls_corridor_free_on_centreline_and_wall_beyond_lane():
    fam = TrackFamilyConfig(lane_width_m=1.0)
    t = build_track(_stadium_spec(+1), fam)
    occ = t.gym_track.occupancy_map
    ox, oy, _ = t.gym_track.spec.origin
    res = t.gym_track.spec.resolution

    def cell(px: float, py: float) -> float:
        return float(occ[int((py - oy) / res), int((px - ox) / res)])

    # Bottom-straight centreline midpoint is free; 0.6 m beyond it (outside the 0.5 m half
    # lane) is wall.
    radius = 0.9
    assert cell(0.0, -radius) == 255.0
    assert cell(0.0, -radius - 0.6) == 0.0
    # Inside wall (the island) at the oval's centre.
    assert cell(0.0, 0.0) == 0.0


def test_pool_is_deterministic_and_seed_ranges_differ():
    fam = TrackFamilyConfig()
    a = build_pool(fam, 3, 1000)
    b = build_pool(fam, 3, 1000)
    c = build_pool(fam, 3, 5000)
    assert [t.spec for t in a] == [t.spec for t in b]
    assert [t.spec for t in a] != [t.spec for t in c]


def test_sample_spec_respects_family_ranges():
    fam = TrackFamilyConfig(inner_radius_m=(0.0, 0.8), stadium_straight_m=(1.0, 4.0))
    rng = np.random.default_rng(0)
    for i in range(50):
        spec = sample_spec(rng, fam, seed=i)
        assert spec.shape in ("stadium", "rounded_rect")
        assert 0.0 <= spec.param("inner_radius_m") <= 0.8
        assert spec.direction in (-1, 1)
        if spec.shape == "stadium":
            assert 1.0 <= spec.param("straight_m") <= 4.0


def test_family_config_rejects_bad_values():
    with pytest.raises(ValueError):
        TrackFamilyConfig(shapes=("hexagon",), shape_weights=(1.0,))
    with pytest.raises(ValueError):
        TrackFamilyConfig(max_offset_m=0.6)
    with pytest.raises(ValueError):
        TrackFamilyConfig(shapes=("stadium",), shape_weights=(1.0, 2.0))


def test_blob_spec_builds_or_is_rejected_cleanly():
    fam = TrackFamilyConfig(shapes=("blob",), shape_weights=(1.0,))
    pool = build_pool(fam, 2, 20000)
    for t in pool:
        assert t.spec.shape == "blob"
        assert t.perimeter_m > 5.0


def test_start_pose_applies_lateral_offset_along_the_normal():
    fam = TrackFamilyConfig()
    t = build_track(_stadium_spec(+1), fam)
    x0, y0, yaw0 = tracks.start_pose(t, 0)
    x1, y1, yaw1 = tracks.start_pose(t, 0, lateral_m=0.1, heading_rad=0.05)
    assert math.hypot(x1 - x0, y1 - y0) == pytest.approx(0.1, abs=1e-9)
    assert yaw1 - yaw0 == pytest.approx(0.05)
