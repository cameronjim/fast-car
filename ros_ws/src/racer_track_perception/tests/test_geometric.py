from __future__ import annotations

import math

import numpy as np
from racer_track_perception import geometric as geo
from racer_track_perception import selfcheck as sc
from racer_track_perception import track as tr
from racer_track_perception.vehicle import LiveVehicle

SIGMA_MAX = 0.25
SPACING = 0.01  # wall point spacing: a LiDAR at 1 m sees about this


def centreline_points(kind: str, length: float = 6.0) -> np.ndarray:
    """A dense centreline polyline (x, y), starting at the origin heading +x."""
    s = np.arange(0.0, length, SPACING)
    if kind == "straight":
        return np.stack([s, np.zeros_like(s)], axis=1)
    if kind == "bend":  # 1 m straight, a 90 degree left arc of radius 1 m, then straight
        r, lead = 1.0, 1.0
        out = []
        for v in s:
            if v < lead:
                out.append((v, 0.0))
            elif v < lead + r * math.pi / 2:
                a = (v - lead) / r
                out.append((lead + r * math.sin(a), r * (1.0 - math.cos(a))))
            else:
                rest = v - lead - r * math.pi / 2
                out.append((lead + r, r + rest))
        return np.array(out)
    raise ValueError(kind)


def walls(centre: np.ndarray, hl: float, hr: float, left=True, right=True) -> np.ndarray:
    d = np.gradient(centre, axis=0)
    d /= np.hypot(d[:, 0], d[:, 1])[:, None]
    n = np.stack([-d[:, 1], d[:, 0]], axis=1)
    parts = []
    if left:
        parts.append(centre + hl * n)
    if right:
        parts.append(centre - hr * n)
    return np.vstack(parts)


def transform(points: np.ndarray, x: float, y: float, yaw: float) -> np.ndarray:
    """World points seen from a car at (x, y, yaw)."""
    c, s = math.cos(yaw), math.sin(yaw)
    d = points - np.array([x, y])
    return np.stack([c * d[:, 0] + s * d[:, 1], -s * d[:, 0] + c * d[:, 1]], axis=1)


def run(points: np.ndarray, live: LiveVehicle, **kw) -> tr.LocalTrackData:
    return geo.trace(points, live.body.half_width_m, SIGMA_MAX, geo.GeometricConfig(**kw))


def truth_at(centre: np.ndarray, car_xy, k: np.ndarray) -> np.ndarray:
    """Polyline points k * 0.2 m along the centreline from the point nearest the car."""
    i0 = int(np.argmin(np.hypot(centre[:, 0] - car_xy[0], centre[:, 1] - car_xy[1])))
    idx = i0 + np.round(k * tr.STATION_SPACING_M / SPACING).astype(int)
    return centre[np.minimum(idx, len(centre) - 1)]


def test_straight_corridor_centred(live: LiveVehicle) -> None:
    centre = centreline_points("straight")
    pts = walls(centre, 0.5, 0.5)
    t = run(pts, live)
    assert t.valid_count == tr.K_STATIONS and t.arrays_ok()
    k = np.arange(16)
    assert np.abs(t.x_m - 0.2 * k).max() < 0.03
    assert np.abs(t.y_m).max() < 0.03
    assert np.abs(t.half_width_left_m - 0.5).max() < 0.03
    assert np.abs(t.half_width_right_m - 0.5).max() < 0.03
    assert np.allclose(t.sigma_m, geo.SIGMA_TWO_WALLS_M)


def test_straight_corridor_offset_and_asymmetric(live: LiveVehicle) -> None:
    centre = centreline_points("straight")
    pts = walls(centre, 0.6, 0.4)  # lane wider on the left: centre at y = 0, car 0.1 m left of...
    t = run(pts, live)
    assert t.valid_count == 16
    # Walls at y = +0.6 and -0.4: the centre is at y = +0.1, half width 0.5 each side.
    assert np.abs(t.y_m - 0.1).max() < 0.03
    assert np.abs(t.half_width_left_m - 0.5).max() < 0.03


def test_heading_error_is_handled(live: LiveVehicle) -> None:
    centre = centreline_points("straight", 8.0) - np.array([2.0, 0.0])
    world = walls(centre, 0.5, 0.5)
    for yaw in (0.3, -0.3):
        pts = transform(world, 0.0, 0.0, yaw)
        t = run(pts, live)
        assert t.valid_count == 16
        c, s = math.cos(-yaw), math.sin(-yaw)
        truth = np.array([[0.2 * k * c, 0.2 * k * s] for k in range(16)])
        err = np.hypot(t.x_m - truth[:, 0], t.y_m - truth[:, 1])
        assert err.max() < 0.03, (yaw, err.max())
        assert np.abs(t.half_width_left_m - 0.5).max() < 0.03


def test_ninety_degree_bend(live: LiveVehicle) -> None:
    centre = centreline_points("bend", 8.0)
    pts = walls(centre, 0.5, 0.5)
    t = run(pts, live)
    assert t.valid_count == tr.K_STATIONS
    truth = truth_at(centre, (0.0, 0.0), np.arange(16))
    err = np.hypot(t.x_m - truth[:, 0], t.y_m - truth[:, 1])
    assert err.max() < 0.03, err
    assert np.abs(t.half_width_left_m - 0.5).max() < 0.03


def test_right_bend_mirrors(live: LiveVehicle) -> None:
    centre = centreline_points("bend", 8.0) * np.array([1.0, -1.0])
    t = run(walls(centre, 0.5, 0.5), live)
    truth = truth_at(centre, (0.0, 0.0), np.arange(16))
    assert np.hypot(t.x_m - truth[:, 0], t.y_m - truth[:, 1]).max() < 0.03


def test_one_wall_uses_the_default_half_width(live: LiveVehicle) -> None:
    centre = centreline_points("straight")
    pts = walls(centre, 0.5, 0.5, right=False)  # only the left wall exists
    t = run(pts, live)
    assert t.valid_count == 16
    assert np.abs(t.y_m).max() < 0.03
    assert np.allclose(t.sigma_m, geo.SIGMA_ONE_WALL_M)
    assert np.allclose(t.half_width_left_m, 0.5, atol=0.03)
    # And with only the right wall.
    t = run(walls(centre, 0.5, 0.5, left=False), live)
    assert np.abs(t.y_m).max() < 0.03 and t.valid_count == 16


def test_one_wall_keeps_the_last_known_half_width(live: LiveVehicle) -> None:
    centre = centreline_points("straight")
    both = walls(centre, 0.4, 0.4)  # a lane 0.8 m wide, both walls for x < 1.5
    left_only = walls(centre, 0.4, 0.4, right=False)
    pts = np.vstack([both[both[:, 0] < 1.5], left_only[left_only[:, 0] >= 1.5]])
    t = run(pts, live)
    assert t.valid_count == 16
    # Past x = 1.5 the right wall is gone; the 0.4 half width carries on, not the 0.5 default.
    assert np.abs(t.y_m).max() < 0.03
    assert np.allclose(t.half_width_left_m, 0.4, atol=0.03)


def test_an_implausibly_wide_pairing_uses_the_nearer_wall_only(live: LiveVehicle) -> None:
    centre = centreline_points("straight")
    near_right = walls(centre, 0.5, 0.5, left=False)  # a wall at y = -0.5
    far_left = walls(centre, 1.4, 1.4, right=False)  # an unrelated wall at y = +1.4
    pts = np.vstack([near_right, far_left])
    gated = run(pts, live)  # default plausible half width range: the midpoint 0.95 is refused
    assert gated.valid_count == 16
    assert np.abs(gated.y_m).max() < 0.03 and np.allclose(gated.sigma_m, geo.SIGMA_ONE_WALL_M)
    assert np.allclose(gated.half_width_left_m, 0.5, atol=0.03)
    ungated = run(pts, live, hw_max_m=math.inf)
    assert abs(np.median(ungated.y_m) - 0.45) < 0.03  # the plain midpoint of the two walls
    # And the nearer wall on the left works the same way.
    flipped = run(
        np.vstack([walls(centre, 0.5, 0.5, right=False), walls(centre, 1.4, 1.4, left=False)]), live
    )
    assert np.abs(flipped.y_m).max() < 0.03


def test_an_implausibly_narrow_pairing_is_refused_too(live: LiveVehicle) -> None:
    centre = centreline_points("straight")
    slot = walls(centre, 0.25, 0.25)  # a 0.5 m slot: half width 0.25 is under the 0.3 floor
    plain = run(slot, live, hw_min_m=0.0)
    assert plain.valid_count == 16 and np.abs(plain.y_m).max() < 0.03
    gated = run(slot, live)
    # One wall alone puts the centre a default half width (0.5 m) from it, which is through the
    # opposite wall: the body check then ends the trace at once.
    assert gated.valid_count < plain.valid_count and gated.arrays_ok()


def test_no_track_cases(live: LiveVehicle) -> None:
    assert run(np.zeros((0, 2)), live).valid_count == 0
    assert run(np.array([[np.nan, np.nan], [np.inf, 1.0]]), live).valid_count == 0
    # Nothing within the feelers' reach (walls 3 m away).
    far = walls(centreline_points("straight"), 3.0, 3.0)
    assert run(far, live).valid_count == 0
    # A point on the centre line within the body half width stops the trace at once.
    blocked = np.vstack([walls(centreline_points("straight"), 0.5, 0.5), [[0.05, 0.0]]])
    assert run(blocked, live).valid_count == 0
    assert geo.trace("not points", live.body.half_width_m, SIGMA_MAX).valid_count == 0  # type: ignore[arg-type]


def test_blocked_ahead_stops_the_trace_short(live: LiveVehicle) -> None:
    centre = centreline_points("straight")
    pts = np.vstack([walls(centre, 0.5, 0.5), [[1.5, 0.0]]])  # an obstacle on the centre line
    t = run(pts, live)
    assert 0 < t.valid_count < 16
    assert t.x_m[t.valid_count - 1] < 1.5 - live.body.half_width_m + 0.2
    assert np.isfinite(t.x_m).all() and np.all(t.sigma_m[t.valid_count :] == tr.INVALID_SIGMA_M)


def test_trace_is_deterministic(live: LiveVehicle) -> None:
    pts = walls(centreline_points("bend", 8.0), 0.5, 0.5)
    a, b = run(pts, live), run(pts, live)
    assert np.array_equal(a.x_m, b.x_m) and np.array_equal(
        a.half_width_right_m, b.half_width_right_m
    )


def test_single_pass_is_the_plain_procedure(live: LiveVehicle) -> None:
    pts = walls(centreline_points("straight"), 0.5, 0.5)
    t = run(pts, live, two_pass=False)
    assert t.valid_count == 16


def test_stations_are_exactly_a_spacing_apart_along_the_polyline(live: LiveVehicle) -> None:
    pts = walls(centreline_points("bend", 8.0), 0.5, 0.5)
    t = run(pts, live)
    # The chord between stations is at most the spacing and close to it on a gentle arc.
    d = np.hypot(np.diff(t.x_m), np.diff(t.y_m))
    assert d.max() <= tr.STATION_SPACING_M + 1e-6 and d.min() > 0.9 * tr.STATION_SPACING_M


def test_self_check_perfect_prediction(live: LiveVehicle) -> None:
    centre = centreline_points("straight")
    pts = walls(centre, 0.5, 0.5).astype(np.float32)
    t = tr.assemble(
        0.2 * np.arange(16),
        np.zeros(16),
        np.full(16, 0.5),
        np.full(16, 0.5),
        np.full(16, 0.05),
        SIGMA_MAX,
    )
    sc.apply_self_check(t, pts)
    assert t.boundary_residual_m < 0.01
    assert t.intrusion_fraction == 0.0


def test_self_check_intrusion_counts_points_inside(live: LiveVehicle) -> None:
    pts = walls(centreline_points("straight"), 0.5, 0.5)
    base = int(((pts[:, 0] > 0) & (np.hypot(pts[:, 0], pts[:, 1]) <= sc.POINT_RANGE_M)).sum())
    t = tr.assemble(
        0.2 * np.arange(16),
        np.zeros(16),
        np.full(16, 0.5),
        np.full(16, 0.5),
        np.full(16, 0.05),
        SIGMA_MAX,
    )
    inside = np.array([[1.0, 0.0], [1.5, 0.2], [2.0, -0.3]])
    assert abs(sc.intrusion_fraction(t, np.vstack([pts, inside])) - 3 / (base + 3)) < 1e-12
    # Inside by less than 0.05 m of a boundary does not count; just behind the car or past
    # station 15 (x > 3.0) does not count either; behind the car (x < 0) is not evaluated.
    edge = np.array([[1.0, 0.48], [1.0, -0.48], [3.4, 0.0], [-0.5, 0.0]])
    assert sc.intrusion_fraction(t, np.vstack([pts, edge])) == 0.0
    # Points outside the corridor on a narrower prediction are not intrusions either.
    narrow = tr.assemble(
        0.2 * np.arange(16),
        np.zeros(16),
        np.full(16, 0.3),
        np.full(16, 0.3),
        np.full(16, 0.05),
        SIGMA_MAX,
    )
    assert sc.intrusion_fraction(narrow, pts) == 0.0
    # A prediction that is too wide swallows the real walls: every wall point is an intrusion.
    wide = tr.assemble(
        0.2 * np.arange(16),
        np.zeros(16),
        np.full(16, 0.9),
        np.full(16, 0.9),
        np.full(16, 0.05),
        SIGMA_MAX,
    )
    assert sc.intrusion_fraction(wide, pts) > 0.9


def test_self_check_residual_grows_with_a_wrong_width() -> None:
    pts = walls(centreline_points("straight"), 0.5, 0.5)
    wrong = tr.assemble(
        0.2 * np.arange(16),
        np.zeros(16),
        np.full(16, 0.7),
        np.full(16, 0.7),
        np.full(16, 0.05),
        SIGMA_MAX,
    )
    assert abs(sc.boundary_residual_m(wrong, pts) - 0.2) < 0.01


def test_self_check_not_computable_is_nan() -> None:
    t = tr.no_track()
    pts = np.array([[1.0, 0.5]])
    assert math.isnan(sc.boundary_residual_m(t, pts)) and math.isnan(sc.intrusion_fraction(t, pts))
    one = tr.assemble(
        np.zeros(16),
        np.zeros(16),
        np.full(16, 0.5),
        np.full(16, 0.5),
        np.concatenate([[0.05], np.full(15, 0.9)]),
        SIGMA_MAX,
    )
    assert one.valid_count == 1
    assert math.isnan(sc.boundary_residual_m(one, pts))
    full = tr.assemble(
        0.2 * np.arange(16),
        np.zeros(16),
        np.full(16, 0.5),
        np.full(16, 0.5),
        np.full(16, 0.05),
        SIGMA_MAX,
    )
    assert math.isnan(sc.boundary_residual_m(full, np.zeros((0, 2))))
    assert math.isnan(sc.intrusion_fraction(full, np.zeros((0, 2))))
    # Only points behind the car or beyond 2.5 m: nothing to evaluate.
    assert math.isnan(sc.intrusion_fraction(full, np.array([[-1.0, 0.0], [4.0, 0.0]])))
    # Every valid station further than 2.0 m from the car: no residual.
    far = tr.assemble(
        2.5 + 0.2 * np.arange(16),
        np.zeros(16),
        np.full(16, 0.5),
        np.full(16, 0.5),
        np.full(16, 0.05),
        SIGMA_MAX,
    )
    assert math.isnan(sc.boundary_residual_m(far, pts))


def test_self_check_two_station_track() -> None:
    t = tr.assemble(
        0.2 * np.arange(16),
        np.zeros(16),
        np.full(16, 0.5),
        np.full(16, 0.5),
        np.concatenate([[0.05, 0.05], np.full(14, 0.9)]),
        SIGMA_MAX,
    )
    assert t.valid_count == 2
    pts = np.array([[0.0, 0.5], [0.2, 0.5], [0.0, -0.5], [0.2, -0.5], [0.1, 0.0]])
    assert sc.boundary_residual_m(t, pts) < 1e-6
    # Of the three points ahead of the car (x > 0), only (0.1, 0) is inside by more than 0.05 m.
    assert abs(sc.intrusion_fraction(t, pts) - 1 / 3) < 1e-12
