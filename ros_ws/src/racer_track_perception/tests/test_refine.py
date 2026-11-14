"""L1: `refine_track`, the scan-grounded correction of a net output (design 15.5)."""

from __future__ import annotations

import math

import numpy as np
from racer_track_perception import refine, selfcheck
from racer_track_perception import track as tr

CFG = refine.RefineConfig()


def make_track(
    x: np.ndarray,
    y: np.ndarray,
    hl: float | np.ndarray = 0.5,
    hr: float | np.ndarray = 0.5,
    sigma: float = 0.05,
    n: int = tr.K_STATIONS,
) -> tr.LocalTrackData:
    k = tr.K_STATIONS
    return tr.assemble(
        x,
        y,
        np.broadcast_to(np.asarray(hl, dtype=float), (k,)),
        np.broadcast_to(np.asarray(hr, dtype=float), (k,)),
        np.full(k, sigma),
        0.25,
        valid_cap=n,
    )


def straight_walls(half: float = 0.5, length: float = 4.0, step: float = 0.02) -> np.ndarray:
    xs = np.arange(-0.5, length, step)
    return np.concatenate(
        [np.stack([xs, np.full_like(xs, half)], 1), np.stack([xs, np.full_like(xs, -half)], 1)]
    )


def straight_net(offset: float = 0.0) -> tr.LocalTrackData:
    x = 0.2 * np.arange(tr.K_STATIONS)
    return make_track(x, np.full(tr.K_STATIONS, offset))


def arc_points(
    radius: float, centre: tuple[float, float], t0: float, t1: float, step: float = 0.02
):
    th = np.arange(t0, t1, step / radius)
    return np.stack([centre[0] + radius * np.sin(th), centre[1] - radius * np.cos(th)], 1)


def invariants_hold(t: tr.LocalTrackData) -> None:
    assert len(t.x_m) == tr.K_STATIONS and len(t.sigma_m) == tr.K_STATIONS
    assert t.arrays_ok()
    n = t.valid_count
    for a in (t.x_m, t.y_m, t.half_width_left_m, t.half_width_right_m, t.sigma_m):
        assert np.isfinite(a).all()
    if n >= 2:
        d = np.hypot(np.diff(t.x_m[:n].astype(float)), np.diff(t.y_m[:n].astype(float)))
        assert d.max() <= tr.STATION_SPACING_M + 1e-5  # chord of an arc of 0.20 m


def test_straight_corridor_recentred_within_two_centimetres() -> None:
    net = straight_net(offset=0.2)
    out = refine.refine_track(net, straight_walls(), CFG)
    invariants_hold(out)
    assert out.valid_count == net.valid_count
    n = out.valid_count
    assert np.abs(out.y_m[:n]).max() <= 0.02
    assert np.abs(out.half_width_left_m[:n] - 0.5).max() <= 0.02
    assert np.abs(out.half_width_right_m[:n] - 0.5).max() <= 0.02
    # The input object is not modified.
    assert np.allclose(net.y_m, 0.2)


def test_a_correct_net_output_is_nearly_unchanged() -> None:
    net = straight_net(0.0)
    out = refine.refine_track(net, straight_walls(), CFG)
    n = net.valid_count
    assert out.valid_count == n
    assert np.abs(out.y_m[:n]).max() <= 0.01
    assert np.abs(out.x_m[:n] - net.x_m[:n]).max() <= 0.02


def test_corner_where_the_net_cuts_into_the_outer_wall() -> None:
    # A left turn of centreline radius 1.0 m, half width 0.5 m, centre of curvature (0, 1).
    centre = (0.0, 1.0)
    pts = np.concatenate([arc_points(0.5, centre, -0.4, 3.0), arc_points(1.5, centre, -0.4, 3.0)])
    pts = pts[(pts[:, 0] > -0.4)]
    r_net = 1.2  # the net's centreline is 0.2 m toward the outer (right) wall
    th = (0.2 * np.arange(tr.K_STATIONS)) / r_net
    x = centre[0] + r_net * np.sin(th)
    y = centre[1] - r_net * np.cos(th)
    net = make_track(x, y, 0.5, 0.5)
    before = selfcheck.intrusion_fraction(net, pts)
    assert before > 0.1  # the premise: the corridor of the net runs through the outer wall
    out = refine.refine_track(net, pts, CFG)
    invariants_hold(out)
    after = selfcheck.intrusion_fraction(out, pts)
    assert after == 0.0
    # The refined centreline is nearer to the true radius 1.0 about the centre of curvature.
    n = out.valid_count
    r_out = np.hypot(out.x_m[:n] - centre[0], out.y_m[:n] - centre[1])
    assert np.abs(r_out - 1.0).max() < 0.05


def test_one_wall_only_limits_the_half_width_and_keeps_the_point() -> None:
    xs = np.arange(-0.5, 4.0, 0.02)
    left_wall = np.stack([xs, np.full_like(xs, 0.4)], 1)
    net = make_track(0.2 * np.arange(16), np.zeros(16), hl=0.6, hr=0.5)
    out = refine.refine_track(net, left_wall, CFG)
    invariants_hold(out)
    n = out.valid_count
    assert np.abs(out.y_m[:n]).max() <= 1e-6  # kept
    assert np.allclose(out.half_width_left_m[:n], 0.4, atol=1e-6)  # min(net, measured)
    assert np.allclose(out.half_width_right_m[:n], 0.5, atol=1e-6)  # untouched


def test_one_wall_keeps_the_nets_half_width_when_the_wall_is_further() -> None:
    xs = np.arange(-0.5, 4.0, 0.02)
    right_wall = np.stack([xs, np.full_like(xs, -0.9)], 1)
    net = make_track(0.2 * np.arange(16), np.zeros(16), hl=0.5, hr=0.6)
    out = refine.refine_track(net, right_wall, CFG)
    n = out.valid_count
    assert np.allclose(out.half_width_right_m[:n], 0.6, atol=1e-6)


def test_one_wall_too_close_shifts_the_point_away() -> None:
    xs = np.arange(-0.5, 4.0, 0.02)
    left_wall = np.stack([xs, np.full_like(xs, 0.1)], 1)
    net = make_track(0.2 * np.arange(16), np.zeros(16), hl=0.5, hr=0.5)
    out = refine.refine_track(net, left_wall, CFG)
    invariants_hold(out)
    n = out.valid_count
    shift = CFG.min_half_m - 0.1
    assert np.allclose(out.y_m[:n], -shift, atol=1e-6)
    # The wall is now min_half_m away, and the right boundary stays where the net put it.
    assert np.allclose(out.half_width_left_m[:n], 0.1 + shift, atol=1e-6)
    assert np.allclose(out.half_width_right_m[:n], 0.5 - shift, atol=1e-6)


def test_one_wall_shift_stops_when_the_other_half_width_is_used_up() -> None:
    xs = np.arange(-0.5, 4.0, 0.02)
    left_wall = np.stack([xs, np.full_like(xs, 0.05)], 1)
    net = make_track(0.2 * np.arange(16), np.zeros(16), hl=0.5, hr=0.1)
    out = refine.refine_track(net, left_wall, CFG)
    n = out.valid_count
    assert np.allclose(out.y_m[:n], -0.1, atol=1e-6)
    assert (out.half_width_right_m[:n] >= 0.0).all()
    assert np.allclose(out.half_width_right_m[:n], 0.0, atol=1e-6)


def test_no_scan_points_leaves_the_track_unchanged() -> None:
    net = straight_net(0.2)
    for pts in (np.zeros((0, 2)), np.array([[math.nan, 1.0], [math.inf, 0.0]])):
        out = refine.refine_track(net, pts, CFG)
        assert out.valid_count == net.valid_count
        for a, b in ((out.x_m, net.x_m), (out.y_m, net.y_m)):
            assert np.array_equal(a, b)
        assert np.array_equal(out.half_width_left_m, net.half_width_left_m)


def test_points_out_of_reach_leave_the_track_unchanged() -> None:
    net = straight_net(0.0)
    far = np.array([[1.0, 5.0], [2.0, -5.0]])  # past the feeler
    out = refine.refine_track(net, far, CFG)
    assert np.array_equal(out.y_m, net.y_m)


def test_a_point_never_moves_more_than_max_shift() -> None:
    # A wide lane (walls at +-0.75) and a net centre 0.5 m off: the full correction is 0.5 m.
    net = straight_net(offset=0.5)
    out = refine.refine_track(net, straight_walls(0.75), CFG)
    n = out.valid_count
    assert np.abs(out.y_m[:n] - 0.5).max() <= CFG.max_shift_m + 1e-6
    assert np.allclose(out.y_m[:n], 0.5 - CFG.max_shift_m, atol=0.01)
    # Neither side claims more than the true distance from the moved centre.
    assert (out.half_width_left_m[:n] <= (0.75 - out.y_m[:n]) + 0.011).all()
    assert (out.half_width_right_m[:n] <= (0.75 + out.y_m[:n]) + 0.011).all()


def test_resampling_keeps_spacing_and_station_zero_nearest_the_origin() -> None:
    net = straight_net(0.15)
    out = refine.refine_track(net, straight_walls(), CFG)
    n = out.valid_count
    d = np.hypot(np.diff(out.x_m[:n]), np.diff(out.y_m[:n]))
    assert np.allclose(d, tr.STATION_SPACING_M, atol=1e-5)
    # Station 0 is the polyline point nearest the origin: no later station is nearer to it.
    radii = np.hypot(out.x_m[:n], out.y_m[:n])
    assert radii[0] <= radii.min() + 1e-9
    # A net whose station 0 is 0.4 m ahead of the origin line keeps its station 0 (the polyline
    # starts there: the nearest point of the polyline is its start).
    shifted = make_track(0.4 + 0.2 * np.arange(16), np.zeros(16))
    out2 = refine.refine_track(shifted, straight_walls(), CFG)
    assert abs(out2.x_m[0] - 0.4) <= 1e-6


def test_station_zero_is_found_between_stations() -> None:
    # Stations start 0.3 m behind the origin: the nearest polyline point is mid segment.
    net = make_track(-0.3 + 0.2 * np.arange(16), np.zeros(16))
    out = refine.refine_track(net, straight_walls(), CFG)
    assert abs(out.x_m[0]) <= 1e-6
    assert out.valid_count == 14  # the 0.3 m behind the nearest point are not a station


def test_half_width_never_exceeds_the_measured_wall_distance() -> None:
    rng = np.random.default_rng(3)
    for _ in range(25):
        half = rng.uniform(0.35, 0.75)
        offset = rng.uniform(-0.25, 0.25)
        net = make_track(
            0.2 * np.arange(16), np.full(16, offset), rng.uniform(0.3, 0.9), rng.uniform(0.3, 0.9)
        )
        out = refine.refine_track(net, straight_walls(half), CFG)
        n = out.valid_count
        # Distance from each output station to the nearest wall point on each side.
        assert (out.half_width_left_m[:n] <= (half - out.y_m[:n]) + 0.011).all()
        assert (out.half_width_right_m[:n] <= (half + out.y_m[:n]) + 0.011).all()


def test_sigma_is_carried_and_valid_count_recomputed() -> None:
    sigma = np.full(16, 0.05)
    sigma[12:] = 0.2
    net = tr.assemble(
        0.2 * np.arange(16),
        np.full(16, 0.1),
        np.full(16, 0.5),
        np.full(16, 0.5),
        sigma,
        0.25,
        valid_cap=14,
    )
    out = refine.refine_track(net, straight_walls(), CFG)
    n = out.valid_count
    assert 13 <= n <= 15
    assert (out.sigma_m[:n] >= 0.05 - 1e-9).all() and out.sigma_m[: n - 3].max() <= 0.05 + 1e-9
    assert (out.sigma_m[n:] == tr.INVALID_SIGMA_M).all()


def test_the_clearance_cap_can_be_switched_off_for_the_report_only() -> None:
    # A point just ahead and to the left of a station: with the cap the left half width stops at
    # its distance, without the cap (GP1b's report variant) it does not.
    pts = np.concatenate([straight_walls(0.5), np.array([[0.9, 0.12]])])
    net = straight_net(0.0)
    capped = refine.refine_track(net, pts, CFG)
    free = refine.refine_track(net, pts, refine.RefineConfig(clearance_cap=False))
    assert refine.RefineConfig().clearance_cap is True
    n = min(capped.valid_count, free.valid_count)
    assert (capped.half_width_left_m[:n] <= free.half_width_left_m[:n] + 1e-9).all()
    assert (capped.half_width_left_m[:n] < free.half_width_left_m[:n] - 0.05).any()


def test_short_or_invalid_input_comes_back_as_it_was() -> None:
    for net in (tr.no_track(), make_track(np.zeros(16), np.zeros(16), n=1)):
        out = refine.refine_track(net, straight_walls(), CFG)
        assert out.valid_count == net.valid_count
        assert out is not net
