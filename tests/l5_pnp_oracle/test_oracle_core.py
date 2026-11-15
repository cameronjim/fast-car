"""L1 tests of the oracle track (brief 40): stations on the midline at 0.20 m, half widths equal to
the true distances on a straight, in a corner and beside an obstacle, frame, direction, pose
helpers. No ROS."""

from __future__ import annotations

import math

import numpy as np
import oracle_core as oc
import pytest
from racer_gym_bridge.track_loader import build_synthetic_track
from racer_gym_bridge.track_obstacles import ground_truth_grid

WALL = 0.08
RES = 0.05  # the grid's cell: the oracle's half widths are exact to about this


def truth_for(obstacles=None, lane=1.0, straight=8.0, radius=0.4, reverse=False):
    track = build_synthetic_track(
        "stadium",
        lane_width_m=lane,
        straight_length_m=straight,
        island_radius_m=radius,
        obstacles=obstacles,
    )
    if reverse:
        track = track.reversed()
    grid, origin, res = ground_truth_grid(track)
    return track, oc.GroundTruthTrack(track.centreline_x, track.centreline_y, grid, origin, res)


def car_at_start(track):
    """The car on the first centreline point facing the second (the simulator's start pose)."""
    x0, y0 = track.centreline_x[0], track.centreline_y[0]
    yaw = math.atan2(track.centreline_y[1] - y0, track.centreline_x[1] - x0)
    return x0, y0, yaw


def test_sixteen_stations_every_20_cm_on_the_midline_in_base_link():
    track, truth = truth_for()
    x0, y0, yaw = car_at_start(track)
    out = truth.local_track(x0, y0, yaw)
    assert len(out.x_m) == oc.STATIONS == 16
    assert out.valid_count == 16
    assert out.x_m[0] == pytest.approx(0.0, abs=0.03) and out.y_m[0] == pytest.approx(0.0, abs=0.03)
    # on the bottom straight, facing +x: station k at x = 0.2 k, y = 0 (base_link frame)
    assert np.allclose(out.x_m, 0.2 * np.arange(16), atol=0.03)
    assert np.allclose(out.y_m, 0.0, atol=0.03)
    gaps = np.hypot(np.diff(out.x_m), np.diff(out.y_m))
    assert np.allclose(gaps, 0.2, atol=0.01)


def test_stations_follow_the_midline_round_a_corner():
    track, truth = truth_for(straight=1.0)
    # drive the car to the corner: the first point past the end of the bottom straight
    i = int(np.argmax(track.centreline_x > track.centreline_x[0] + 0.7))
    x, y = track.centreline_x[i], track.centreline_y[i]
    yaw = math.atan2(track.centreline_y[i + 1] - y, track.centreline_x[i + 1] - x)
    out = truth.local_track(x, y, yaw)
    # world positions of the stations lie on the stored midline
    c, s = math.cos(yaw), math.sin(yaw)
    wx = x + c * out.x_m - s * out.y_m
    wy = y + s * out.x_m + c * out.y_m
    near = [
        np.min(np.hypot(track.centreline_x - px, track.centreline_y - py))
        for px, py in zip(wx, wy, strict=True)
    ]
    assert max(near) < 0.03  # vertices are 0.05 m apart: half a step to the nearest one
    # and they turn left: the last station is left of the first heading
    assert out.y_m[-1] > 0.3


def test_half_widths_are_the_true_distances_on_a_straight():
    track, truth = truth_for(lane=1.0)
    out = truth.local_track(*car_at_start(track))
    expect = 0.5 - WALL / 2  # the wall's lane-facing surface
    assert np.allclose(out.half_width_left_m, expect, atol=RES)
    assert np.allclose(out.half_width_right_m, expect, atol=RES)


def test_half_widths_in_a_corner_follow_the_curved_walls():
    lane, radius = 1.0, 0.4
    track, truth = truth_for(lane=lane, straight=1.0, radius=radius)
    i = int(np.argmax(track.centreline_x > track.centreline_x[0] + 0.7))
    x, y = track.centreline_x[i], track.centreline_y[i]
    yaw = math.atan2(track.centreline_y[i + 1] - y, track.centreline_x[i + 1] - x)
    out = truth.local_track(x, y, yaw)
    # in the corner stations the walls are circles round one centre: the inner (left) one of
    # radius `radius`, the outer of radius radius + lane. Half width along the normal = distance
    # from the midline circle (radius + lane / 2) to each wall surface.
    mid_r = radius + lane / 2
    for k in range(6, 16):
        assert out.half_width_left_m[k] == pytest.approx(mid_r - radius - WALL / 2, abs=RES + 0.02)
        assert out.half_width_right_m[k] == pytest.approx(
            radius + lane - mid_r - WALL / 2, abs=RES + 0.02
        )


@pytest.mark.parametrize("side", ["inner", "outer"])
def test_half_widths_beside_an_obstacle_see_the_box_and_the_midline_bends(side):
    base_track, _ = truth_for(straight=4.0)
    length = float(
        np.hypot(
            np.diff(np.r_[base_track.centreline_x, base_track.centreline_x[0]]),
            np.diff(np.r_[base_track.centreline_y, base_track.centreline_y[0]]),
        ).sum()
    )
    at = 1.2 / length
    p = 0.25
    obstacle = {"at": at, "side": side, "protrusion_m": p, "length_m": 0.30}
    track, truth = truth_for([obstacle], straight=4.0)
    plain = 0.5 - WALL / 2
    # a car 0.3 m before the box centre, heading +x along the bottom straight (midline y = -0.5)
    s_target = 1.2 - 0.3
    step = np.hypot(np.diff(track.centreline_x), np.diff(track.centreline_y))
    arc = np.cumsum(np.r_[0.0, step])
    i = int(np.argmin(np.abs(arc - s_target)))
    x, y = track.centreline_x[i], track.centreline_y[i]
    out = truth.local_track(x, y, 0.0)
    k = 2  # 0.4 m ahead of the car: inside the box's 0.30 m (box from 1.05 to 1.35 m)
    # the midline is the middle of the free lane there: both sides see (lane - box) / 2
    assert out.half_width_left_m[k] == pytest.approx(plain - p / 2, abs=RES + 0.01)
    assert out.half_width_right_m[k] == pytest.approx(plain - p / 2, abs=RES + 0.01)
    # and it has moved p / 2 away from the box: south for an inner (left) box, north for outer
    away = -1.0 if side == "inner" else 1.0
    assert y + out.y_m[k] == pytest.approx(-0.5 + away * p / 2, abs=0.03)
    # well before the box the lane is the plain one
    far = truth.local_track(track.centreline_x[0], track.centreline_y[0], 0.0)
    assert far.half_width_left_m[0] == pytest.approx(plain, abs=RES)
    assert far.half_width_right_m[0] == pytest.approx(plain, abs=RES)


def test_confidence_fields_are_at_their_best_and_sigma_is_finite():
    track, truth = truth_for()
    out = truth.local_track(*car_at_start(track))
    assert np.all(out.sigma_m == 0.0) and np.isfinite(out.sigma_m).all()
    assert out.boundary_residual_m == 0.0 and out.intrusion_fraction == 0.0
    assert np.isfinite(out.half_width_left_m).all() and (out.half_width_left_m <= 2.0).all()


def test_half_width_is_capped_at_two_metres():
    big = np.full((200, 200), 255, dtype=np.uint8)
    x = np.linspace(2.0, 8.0, 100)
    truth = oc.GroundTruthTrack(
        np.r_[x, x[::-1]], np.r_[np.full(100, 5.0), np.full(100, 6.0)], big, (0.0, 0.0), 0.05
    )
    out = truth.local_track(3.0, 5.0, 0.0)
    assert np.all(out.half_width_left_m == oc.HALF_WIDTH_CAP_M)


def test_wrong_way_car_gets_the_track_in_its_own_travel_direction():
    track, truth = truth_for(lane=1.0)
    x0, y0, yaw = car_at_start(track)
    out = truth.local_track(x0, y0, yaw + math.pi)  # facing against the stored direction
    assert out.travel_direction == -1
    assert np.allclose(out.x_m, 0.2 * np.arange(16), atol=0.03)  # still ahead of the car
    assert out.x_m[-1] > 2.9


def test_reversed_track_gives_the_same_view_as_a_car_driving_it():
    track, truth = truth_for(lane=1.0, reverse=True)
    out = truth.local_track(*car_at_start(track))
    assert out.travel_direction == 1
    assert np.allclose(out.x_m, 0.2 * np.arange(16), atol=0.03)


def test_left_and_right_swap_when_the_travel_direction_flips():
    track, truth = truth_for(lane=1.0)
    # a car off the midline by 0.1 m to the left of its travel: left narrower than right
    x0, y0, yaw = car_at_start(track)
    out_fwd = truth.local_track(x0, y0 + 0.0, yaw)
    assert abs(out_fwd.half_width_left_m[0] - out_fwd.half_width_right_m[0]) < RES
    # stations are on the midline whatever the car's lateral offset: shifting the car moves y_m
    out_off = truth.local_track(x0, y0 + 0.10, yaw)
    assert out_off.y_m[0] == pytest.approx(-0.10, abs=0.02)


def test_base_pose_is_the_cg_moved_back_along_the_heading():
    bx, by = oc.base_pose_from_cg(1.0, 2.0, math.pi / 2, 0.17145)
    assert (bx, by) == pytest.approx((1.0, 2.0 - 0.17145))


def test_ground_truth_file_round_trips(tmp_path):
    track, truth = truth_for()
    grid, origin, res = ground_truth_grid(track)
    path = tmp_path / "gt.npz"
    oc.write_ground_truth(path, track.centreline_x, track.centreline_y, grid, origin, res)
    loaded = oc.GroundTruthTrack.load(path)
    a = truth.local_track(*car_at_start(track))
    b = loaded.local_track(*car_at_start(track))
    assert np.array_equal(a.x_m, b.x_m) and np.array_equal(a.half_width_left_m, b.half_width_left_m)


def test_short_centreline_is_refused():
    with pytest.raises(ValueError):
        oc.GroundTruthTrack(np.zeros(3), np.zeros(3), np.zeros((4, 4)), (0.0, 0.0), 0.05)


# -- the node's pose at the scan's stamp -----------------------------------------------------------


SAMPLES = [(1.00, 0.0, 0.0, 0.0), (1.01, 0.1, 0.0, 0.1), (1.02, 0.2, 0.0, 0.2)]


def test_pose_is_interpolated_between_the_samples_around_the_stamp():
    x, y, yaw = oc.interpolate_pose(SAMPLES, 1.005)
    assert (x, y, yaw) == pytest.approx((0.05, 0.0, 0.05))
    assert oc.interpolate_pose(SAMPLES, 1.01) == pytest.approx((0.1, 0.0, 0.1))


def test_pose_before_all_samples_is_the_first_and_after_all_is_none():
    assert oc.interpolate_pose(SAMPLES, 0.5) == (0.0, 0.0, 0.0)
    assert oc.interpolate_pose(SAMPLES, 1.021) is None
    assert oc.interpolate_pose([], 1.0) is None


def test_yaw_interpolation_takes_the_short_way_round_the_wrap():
    samples = [(0.0, 0.0, 0.0, math.pi - 0.05), (1.0, 0.0, 0.0, -math.pi + 0.05)]
    _x, _y, yaw = oc.interpolate_pose(samples, 0.5)
    assert abs(abs(yaw) - math.pi) < 1e-9
