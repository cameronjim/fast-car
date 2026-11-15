"""L1: scan matching on synthetic scans with known transforms."""

from __future__ import annotations

import math

import numpy as np
import pytest
from racer_laps.scan_match import (
    BODY_FILTER_MARGIN_M,
    Body,
    Mount,
    along_support,
    freespace_violation_share,
    match_scans,
    polar_profile,
    scan_to_points,
    surface_normals,
    thin,
    wrap,
)
from synth import BODY, MOUNT, N_BEAMS, Stadium, scan_at


def points_at(stadium: Stadium, x: float, y: float, yaw: float, seed: int = 0) -> np.ndarray:
    rng = np.random.default_rng(seed)
    ranges, a0, da = scan_at(stadium.walls(), x, y, yaw, rng, noise_m=0.01)
    return thin(scan_to_points(ranges, a0, da, 0.05, 16.0, MOUNT, BODY))


def test_wrap() -> None:
    assert wrap(3 * math.pi) == pytest.approx(-math.pi)
    assert wrap(0.3) == pytest.approx(0.3)
    assert wrap(-0.3 - 2 * math.pi) == pytest.approx(-0.3)


def test_the_lidar_mount_yaw_of_pi_puts_a_box_ahead_of_the_car_ahead() -> None:
    # One beam at laser bearing pi (the sensor's own rear) looks forward on the car.
    ranges = np.zeros(N_BEAMS)
    ranges[:] = np.nan
    inc = 2 * math.pi / N_BEAMS
    ranges[N_BEAMS // 2] = 1.0  # laser bearing -pi + 360 * inc = 0 ... pick the beam at 0 rad
    # beam i has laser bearing -pi + i * inc; i = N/2 is bearing 0 (laser forward = car backward)
    pts = scan_to_points(ranges, -math.pi, inc, 0.05, 16.0, MOUNT, BODY)
    assert pts.shape == (1, 2)
    # laser forward is the car's rear: x = mount_x - 1.0
    assert pts[0, 0] == pytest.approx(MOUNT.x_m - 1.0, abs=1e-6)
    assert pts[0, 1] == pytest.approx(0.0, abs=1e-6)
    # a beam at laser bearing pi sees ahead of the car
    ranges[:] = np.nan
    ranges[0] = 2.0  # bearing -pi == pi
    pts = scan_to_points(ranges, -math.pi, inc, 0.05, 16.0, MOUNT, BODY)
    assert pts[0, 0] == pytest.approx(MOUNT.x_m + 2.0, abs=1e-6)


def test_body_returns_and_bad_beams_are_dropped() -> None:
    inc = 2 * math.pi / N_BEAMS
    ranges = np.full(N_BEAMS, 1.5)
    ranges[0] = 0.0
    ranges[1] = math.nan
    ranges[2] = math.inf
    ranges[3] = 99.0  # beyond range_max
    ranges[4] = 0.01  # below range_min
    # a return that lands inside the body rectangle (just in front of the head, within the margin)
    ranges[N_BEAMS // 2 + 1] = 0.12
    pts = scan_to_points(ranges, -math.pi, inc, 0.05, 16.0, MOUNT, BODY)
    inside = (
        (pts[:, 0] >= BODY.x_min_m - BODY_FILTER_MARGIN_M)
        & (pts[:, 0] <= BODY.x_max_m + BODY_FILTER_MARGIN_M)
        & (np.abs(pts[:, 1]) <= BODY.half_width_m + BODY_FILTER_MARGIN_M)
    )
    assert not inside.any()
    # zero, NaN, inf, out of range, below range_min and the body return are all gone
    assert len(pts) == N_BEAMS - 6


def test_empty_and_degenerate_scans_do_not_raise() -> None:
    assert scan_to_points([], 0.0, 0.1, 0.0, 10.0, MOUNT, BODY).shape == (0, 2)
    assert scan_to_points([1.0], math.nan, 0.1, 0.0, 10.0, MOUNT, BODY).shape == (0, 2)
    assert scan_to_points([math.nan] * 5, 0.0, 0.1, 0.0, 10.0, MOUNT, BODY).shape == (0, 2)
    assert thin(np.zeros((0, 2))).shape == (0, 2)
    assert match_scans(np.zeros((3, 2)), np.zeros((50, 2))) is None
    assert match_scans(np.zeros((50, 2)), np.zeros((3, 2))) is None


def test_thin_is_deterministic_and_bounded() -> None:
    rng = np.random.default_rng(3)
    pts = rng.uniform(-3, 3, (2000, 2))
    a = thin(pts)
    b = thin(pts)
    assert a.shape[0] <= 220
    assert np.array_equal(a, b)
    # one point per voxel: no two thinned points in the same 4 cm cell
    keys = {tuple(k) for k in np.round(thin(pts, max_points=10_000) / 0.04).astype(int)}
    assert len(keys) == thin(pts, max_points=10_000).shape[0]


@pytest.mark.parametrize(
    ("dx", "dy", "dyaw"),
    [(0.0, 0.0, 0.0), (0.3, 0.0, 0.05), (-0.2, 0.15, -0.1), (0.1, -0.1, 0.2)],
)
def test_match_recovers_a_known_transform(dx: float, dy: float, dyaw: float) -> None:
    stadium = Stadium()
    x0, y0, yaw0 = 1.0, -0.9, 0.0
    target = points_at(stadium, x0, y0, yaw0, seed=1)
    # the car moved by (dx, dy, dyaw) in the start frame
    c, s = math.cos(yaw0), math.sin(yaw0)
    x1 = x0 + c * dx - s * dy
    y1 = y0 + s * dx + c * dy
    source = points_at(stadium, x1, y1, yaw0 + dyaw, seed=2)
    res = match_scans(source, target, seed=(dx, dy, dyaw))
    assert res is not None
    assert res.x_m == pytest.approx(dx, abs=0.04)
    assert res.y_m == pytest.approx(dy, abs=0.04)
    assert res.yaw_rad == pytest.approx(dyaw, abs=0.03)
    assert res.rms_m < 0.03
    assert res.inlier_share > 0.6


def test_a_left_turn_gives_positive_yaw() -> None:
    stadium = Stadium()
    target = points_at(stadium, 1.0, -0.9, 0.0, seed=1)
    source = points_at(stadium, 1.0, -0.9, 0.15, seed=2)  # the car turned left by 0.15 rad
    res = match_scans(source, target, seed=(0.0, 0.0, 0.1))
    assert res is not None
    assert res.yaw_rad > 0.1
    assert res.yaw_rad == pytest.approx(0.15, abs=0.03)


def test_failure_on_a_non_overlapping_pair() -> None:
    stadium = Stadium()
    target = points_at(stadium, 1.0, -0.9, 0.0, seed=1)
    # a scan of nothing like it: a ring of points far away
    a = np.linspace(0, 2 * math.pi, 120, endpoint=False)
    ring = np.stack([8.0 + 3.0 * np.cos(a), 8.0 + 3.0 * np.sin(a)], axis=1)
    res = match_scans(ring, target, seed=(0.0, 0.0, 0.0))
    assert res is None or res.inlier_share < 0.2


def test_inlier_share_drops_when_half_the_points_have_no_counterpart() -> None:
    stadium = Stadium()
    target = points_at(stadium, 1.0, -0.9, 0.0, seed=1)
    source = target.copy()
    source[: len(source) // 2] += np.array([5.0, 5.0])
    res = match_scans(source, target)
    assert res is None or res.inlier_share <= 0.55


def test_a_custom_mount_and_body_are_honoured() -> None:
    mount = Mount(0.0, 0.0, 0.0)
    body = Body(-0.05, 0.05, 0.05)
    inc = 2 * math.pi / 8
    pts = scan_to_points(np.full(8, 1.0), 0.0, inc, 0.0, 10.0, mount, body)
    assert pts.shape == (8, 2)
    assert np.allclose(np.hypot(pts[:, 0], pts[:, 1]), 1.0)


def test_surface_normals_of_a_straight_wall_are_across_it() -> None:
    wall = np.stack([np.linspace(0, 1, 30), np.full(30, 0.5)], axis=1)
    n = surface_normals(wall)
    assert np.allclose(np.abs(n[:, 1]), 1.0, atol=1e-6)
    assert np.allclose(n[:, 0], 0.0, atol=1e-6)
    assert np.all(np.isnan(surface_normals(wall[:2])))
    lonely = np.array([[0.0, 0.0], [5.0, 5.0], [9.0, 0.0], [20.0, 1.0]])
    assert np.all(np.isnan(surface_normals(lonely)))


def test_along_support_is_zero_in_a_straight_lane_and_high_with_end_caps() -> None:
    # a straight corridor: two parallel walls
    xs = np.linspace(-3, 3, 150)
    corridor = np.concatenate(
        [np.stack([xs, np.full(150, 0.5)], axis=1), np.stack([xs, np.full(150, -0.5)], axis=1)]
    )
    n = surface_normals(corridor)
    res = match_scans(corridor, corridor)
    assert res is not None
    assert along_support(corridor, corridor, n, res) == 0
    # the stadium seen from the middle of its straight has island and wall ends in view
    stadium = Stadium()
    target = points_at(stadium, 1.0, -0.9, 0.0, seed=1)
    source = points_at(stadium, 1.2, -0.9, 0.0, seed=2)
    res2 = match_scans(source, target, seed=(0.2, 0.0, 0.0))
    assert res2 is not None
    assert along_support(source, target, surface_normals(target), res2) >= 8


# -- free-space consistency ----------------------------------------------------------------------

ORIGIN = (0.0, 0.0)


def ring(radius: float, n: int = 720, centre: tuple[float, float] = (0.0, 0.0)) -> np.ndarray:
    a = np.linspace(-math.pi, math.pi, n, endpoint=False)
    return np.stack([centre[0] + radius * np.cos(a), centre[1] + radius * np.sin(a)], axis=1)


def test_polar_profile_is_the_nearest_return_per_bin_and_nan_where_unknown() -> None:
    pts = np.array([[2.0, 0.0], [1.5, 0.0], [0.0, 3.0]])
    prof = polar_profile(pts, ORIGIN, dilate_bins=0)
    assert prof.shape == (360,)
    k0 = int((0.0 + math.pi) / (2 * math.pi) * 360)  # the bin of bearing 0
    assert prof[k0] == pytest.approx(1.5)  # the nearer of two returns in one bin
    assert prof[int((math.pi / 2 + math.pi) / (2 * math.pi) * 360)] == pytest.approx(3.0)
    assert np.isnan(prof[0])  # nothing at bearing -pi
    assert np.isfinite(prof).sum() == 2
    assert np.isnan(polar_profile(np.zeros((0, 2)), ORIGIN)).all()


def test_the_profile_is_the_minimum_over_the_dilation_window_and_wraps() -> None:
    bearing = math.radians(0.5)  # bin 180
    pts = np.array([[2.0 * math.cos(bearing), 2.0 * math.sin(bearing)]])
    prof = polar_profile(pts, ORIGIN, dilate_bins=2)
    finite = np.flatnonzero(np.isfinite(prof))
    assert list(finite) == [178, 179, 180, 181, 182]
    assert np.allclose(prof[finite], 2.0)
    # a return at bearing -pi spills over the seam into the last bins
    seam = polar_profile(np.array([[-1.0, 1e-6]]), ORIGIN, dilate_bins=2)
    assert np.isfinite(seam[-1]) and np.isfinite(seam[-2])


def test_the_profile_uses_the_given_origin_not_the_frame_origin() -> None:
    pts = ring(1.0, centre=(0.285, 0.0))
    prof = polar_profile(pts, (0.285, 0.0), dilate_bins=0)
    assert np.nanmax(np.abs(prof - 1.0)) < 1e-6
    off = polar_profile(pts, ORIGIN, dilate_bins=0)
    assert np.nanmax(np.abs(off - 1.0)) > 0.2  # seen from the wrong place the ring is not a ring


def test_a_wall_behind_a_seen_wall_is_not_a_violation_but_one_in_front_is() -> None:
    prof = polar_profile(ring(2.0), ORIGIN)
    assert freespace_violation_share(ring(3.0), prof, ORIGIN) == 0.0  # behind the wall
    assert freespace_violation_share(ring(2.0), prof, ORIGIN) == 0.0  # on it
    assert freespace_violation_share(ring(1.9), prof, ORIGIN) == 0.0  # inside the 0.15 m margin
    assert freespace_violation_share(ring(1.8), prof, ORIGIN) == 1.0  # in the seen free space


def test_unknown_bins_and_points_beyond_the_range_are_not_counted() -> None:
    half = ring(2.0)
    half = half[half[:, 1] > 0]  # the profile only knows the upper half plane
    prof = polar_profile(half, ORIGIN, dilate_bins=0)
    inner = ring(1.0)
    upper_only = inner[inner[:, 1] > 0.05]
    lower_only = inner[inner[:, 1] < -0.05]
    assert freespace_violation_share(lower_only, prof, ORIGIN) == 0.0  # nothing countable
    assert freespace_violation_share(upper_only, prof, ORIGIN) == pytest.approx(1.0)
    # the whole ring: the unknown half is in neither the numerator nor the denominator
    assert freespace_violation_share(inner, prof, ORIGIN) == pytest.approx(1.0)
    far = ring(5.0)
    far_prof = polar_profile(ring(8.0), ORIGIN)
    assert freespace_violation_share(far, far_prof, ORIGIN, max_range_m=4.0) == 0.0
    assert freespace_violation_share(far, far_prof, ORIGIN, max_range_m=6.0) == 1.0
    assert freespace_violation_share(np.zeros((0, 2)), prof, ORIGIN) == 0.0


def test_a_shifted_copy_of_a_scan_violates_where_the_far_wall_moved_inside() -> None:
    stadium = Stadium(straight_m=3.0, lane_m=0.9, island_r_m=0.5)
    a = points_at(stadium, 0.0, -0.95, 0.0)
    mount = (MOUNT.x_m, MOUNT.y_m)
    b = points_at(stadium, 1.0, -0.95, 0.0)  # one metre further along the straight
    prof_a = polar_profile(a, mount)
    # b moved back into a's frame by the true shift: consistent
    assert freespace_violation_share(b + np.array([1.0, 0.0]), prof_a, mount) < 0.04
    # b moved by a wrong shift (the alias): its far end lies inside a's free space
    assert freespace_violation_share(b + np.array([-1.0, 0.0]), prof_a, mount) > 0.04


def test_the_mount_offset_matters() -> None:
    # a ring of radius 2 around the LiDAR, which sits 0.285 m ahead of base_link
    pts = ring(2.0, centre=(0.285, 0.0))
    prof = polar_profile(pts, (0.285, 0.0))
    assert freespace_violation_share(pts, prof, (0.285, 0.0)) == 0.0
    # one margin inside, measured from the right origin, is a violation
    assert freespace_violation_share(ring(1.8, centre=(0.285, 0.0)), prof, (0.285, 0.0)) == 1.0
    # measured from base_link instead, the same inner ring is not a clean violation
    assert freespace_violation_share(ring(1.8, centre=(0.285, 0.0)), prof, ORIGIN) < 1.0


def test_match_result_defaults_the_violation_to_zero() -> None:
    from racer_laps.scan_match import MatchResult

    assert MatchResult(0.0, 0.0, 0.0, 0.01, 0.9, 100).freespace_violation == 0.0
