from __future__ import annotations

import math

import numpy as np
from racer_pnp_perception import nuisance as nz
from racer_pnp_perception import sample as sm
from racer_pnp_perception import tracks as tk
from racer_pnp_perception.raycast import ScanGeometry
from racer_track_perception.vehicle import LiveVehicle

LONG = tk.IslandParams(1.0, 6.0, 1.5, 0.4)  # a long bottom straight, lane 1.0 m, centre y = -0.5


def long_world():
    w = tk.island_world(LONG)
    return w, sm.LaneGeometry(w.centreline)


def test_lane_geometry_circle() -> None:
    circle = tk.fourier_centreline(2.0, (0.0,) * 4, (0.0,) * 4)
    lane = sm.LaneGeometry(circle)
    assert abs(lane.length - 2 * math.pi * 2.0) < 0.01
    pos, tan, kappa = lane.at(np.array([0.0, lane.length / 4]))
    assert np.allclose(np.hypot(pos[:, 0], pos[:, 1]), 2.0, atol=0.01)
    assert np.allclose(kappa, 0.5, atol=0.02)  # counter-clockwise: positive
    assert np.allclose(np.hypot(tan[:, 0], tan[:, 1]), 1.0)
    s0 = lane.nearest_s(np.array([0.0, 3.0]))
    assert abs(s0 - lane.length / 4) < 0.02
    # Wraps.
    p_a, _, _ = lane.at(np.array([lane.length + 1.0]))
    p_b, _, _ = lane.at(np.array([1.0]))
    assert np.allclose(p_a, p_b)


def test_label_straight_lane_centreline_and_widths() -> None:
    w, lane = long_world()
    off = 0.2  # the car is 0.2 m to the left of the centreline
    label = sm.label_for_pose(lane, w.walls, (2.0, -0.5 + off, 0.0))
    assert label.shape == (16, 5) and label.dtype == np.float32
    assert np.allclose(label[:, 0], 0.2 * np.arange(16), atol=0.01)
    assert np.allclose(label[:, 1], -off, atol=0.01)  # the centre is to the car's right
    assert np.allclose(label[:, 2] + label[:, 3], 1.0, atol=0.01)
    assert np.allclose(label[:, 2], 0.5, atol=0.01)
    assert np.all(label[:, 4] == 0.0)


def test_label_reverse_travel_swaps_sides() -> None:
    w, lane = long_world()
    off = 0.2
    # Facing -x at world y = -0.5 + 0.2: the centreline (y = -0.5) is now on the car's LEFT.
    label = sm.label_for_pose(lane, w.walls, (4.0, -0.5 + off, math.pi))
    assert np.allclose(label[:, 0], 0.2 * np.arange(16), atol=0.01)
    assert np.allclose(label[:, 1], +off, atol=0.01)
    assert np.allclose(label[:, 2] + label[:, 3], 1.0, atol=0.01)


def test_label_heading_error_rotates_into_the_car_frame() -> None:
    w, lane = long_world()
    psi = 0.3
    label = sm.label_for_pose(lane, w.walls, (2.0, -0.5, psi))
    # The centreline runs along world +x; in the car frame it points at -psi.
    ang = np.arctan2(label[15, 1] - label[0, 1], label[15, 0] - label[0, 0])
    assert abs(ang + psi) < 0.01
    assert np.allclose(label[:, 2] + label[:, 3], 1.0, atol=0.01)


def test_label_caps_the_width() -> None:
    _w, lane = long_world()
    label = sm.label_for_pose(lane, np.zeros((0, 4)), (2.0, -0.5, 0.0))
    assert np.all(label[:, 2:4] == 2.0)


def test_label_wraps_round_the_loop() -> None:
    w, lane = long_world()
    pos, tan, _ = lane.at(np.array([lane.length - 0.5]))
    yaw = math.atan2(tan[0, 1], tan[0, 0])
    label = sm.label_for_pose(lane, w.walls, (pos[0, 0], pos[0, 1], yaw))
    assert np.isfinite(label).all()
    # Stations run on past the start of the loop: station 5 is 1.0 m on, 0.5 m past it.
    p5, _, _ = lane.at(np.array([lane.length - 0.5 + 1.0]))
    c, s = math.cos(yaw), math.sin(yaw)
    dx, dy = p5[0, 0] - pos[0, 0], p5[0, 1] - pos[0, 1]
    assert abs(label[5, 0] - (c * dx + s * dy)) < 0.01


def test_clean_sample_ranges_match_the_geometry(live: LiveVehicle) -> None:
    w, lane = long_world()
    geo = ScanGeometry()
    rng = np.random.default_rng(0)
    pose = (2.0, -0.5, 0.0)
    s = sm.make_sample(w, lane, rng, live.mount, geo, clean=True, pose=pose)
    assert not np.isnan(s.ranges).any()
    # Beam to the left (vehicle +90 degrees): the inner wall at y = 0, from the head at y = -0.5.
    bearings = (geo.laser_bearings() + live.mount.yaw_rad + math.pi) % (2 * math.pi) - math.pi
    left = int(np.argmin(np.abs(bearings - math.pi / 2)))
    err = bearings[left] - math.pi / 2
    assert abs(s.ranges[left] - 0.5 / math.cos(err)) < 5e-3
    assert np.allclose(s.label, sm.label_for_pose(lane, w.walls, pose))


def test_make_sample_with_nuisance_is_seeded(live: LiveVehicle) -> None:
    w, lane = long_world()
    geo = ScanGeometry()
    a = sm.make_sample(w, lane, np.random.default_rng(5), live.mount, geo)
    b = sm.make_sample(w, lane, np.random.default_rng(5), live.mount, geo)
    assert np.array_equal(a.ranges, b.ranges, equal_nan=True)
    assert np.array_equal(a.label, b.label)
    assert np.isfinite(a.label).all() and a.ranges.shape == (720,)


def test_draw_pose_stays_in_the_lane() -> None:
    w, lane = long_world()
    rng = np.random.default_rng(0)
    for _ in range(200):
        x, y, yaw = sm.draw_pose(lane, w.lane_width_m, rng)
        s0 = lane.nearest_s(np.array([x, y]))
        c, _, _ = lane.at(np.array([s0]))
        assert np.hypot(x - c[0, 0], y - c[0, 1]) <= w.lane_width_m / 2 - sm.LATERAL_MARGIN_M + 0.02
        assert math.isfinite(yaw)


def test_nuisance_clean_and_drawn_ranges() -> None:
    rng = np.random.default_rng(0)
    assert nz.draw_nuisance(rng, clean=True) is nz.CLEAN
    n = nz.draw_nuisance(rng)
    assert nz.RANGE_SIGMA_M[0] <= n.range_sigma_m <= nz.RANGE_SIGMA_M[1]
    assert 0.0 <= n.dropout <= nz.DROPOUT_MAX and 0.0 <= n.dead_width_rad <= nz.DEAD_SECTOR_MAX_RAD
    assert abs(n.dead_centre_rad - math.pi) <= nz.DEAD_SECTOR_CENTRE_JITTER_RAD + 1e-9
    assert all(nz.GAP_M[0] <= g <= nz.GAP_M[1] for g in n.gap_widths_m)
    assert abs(nz.motion_for(n, 0.5).yaw_rate_rps - 0.5 * n.speed_mps) < 1e-12


def make_nuisance(**kw) -> nz.Nuisance:
    base = {
        "range_sigma_m": 0.0,
        "dropout": 0.0,
        "dead_centre_rad": math.pi,
        "dead_width_rad": 0.0,
        "n_gaps": 0,
        "gap_widths_m": (),
        "room_margins_m": None,
        "n_clutter": 0,
        "speed_mps": 0.0,
        "sweep_sign": 1,
        "inf_share": 0.0,
    }
    base.update(kw)
    return nz.Nuisance(**base)


def test_perturb_world_gaps_room_and_clutter() -> None:
    w, lane = long_world()
    rng = np.random.default_rng(0)
    out = nz.perturb_world(w.walls, lane.pts, w.lane_width_m, nz.CLEAN, rng)
    assert out.shape == w.walls.shape  # clean: nothing added or removed
    gapped = make_nuisance(n_gaps=3, gap_widths_m=(0.3, 0.3, 0.3))
    out = nz.perturb_world(w.walls, lane.pts, w.lane_width_m, gapped, rng)
    assert len(out) < len(w.walls)
    room = make_nuisance(room_margins_m=(1.0, 2.0, 3.0, 4.0))
    out = nz.perturb_world(w.walls, lane.pts, w.lane_width_m, room, rng)
    assert len(out) == len(w.walls) + 4
    x0, _y0, _x1, y1 = w.bounds()
    box = out[-4:]
    assert abs(box[:, [0, 2]].min() - (x0 - 1.0)) < 1e-9
    assert abs(box[:, [1, 3]].max() - (y1 + 4.0)) < 1e-9
    clutter = make_nuisance(n_clutter=6, room_margins_m=(1.0, 1.0, 1.0, 1.0))
    out = nz.perturb_world(w.walls, lane.pts, w.lane_width_m, clutter, rng)
    added = out[len(w.walls) + 4 :]
    assert len(added) > 0
    for seg in added:
        probe = np.linspace(seg[:2], seg[2:], 7)
        dx = probe[:, None, 0] - lane.pts[None, :, 0]
        dy = probe[:, None, 1] - lane.pts[None, :, 1]
        assert np.hypot(dx, dy).min() > w.lane_width_m / 2


def test_perturb_ranges_noise_dropout_dead_sector() -> None:
    rng = np.random.default_rng(0)
    bearings = -math.pi + np.arange(720) * 0.008739
    ranges = np.full(720, 2.0)
    n = make_nuisance(range_sigma_m=0.02, dead_width_rad=math.radians(40), inf_share=0.5)
    out = nz.perturb_ranges(ranges, bearings, 0.0, n, rng)
    dead = ~np.isfinite(out)
    # Dead sector of 40 degrees centred straight back (vehicle bearing pi): about 80 beams.
    assert 76 <= dead.sum() <= 84
    assert np.isnan(out).any() and np.isinf(out).any()  # both kinds of dropped beam
    kept = out[np.isfinite(out)]
    assert abs(kept.std() - 0.02) < 0.004
    assert np.all(np.abs(bearings[dead]) > math.pi - math.radians(21))
    drop = make_nuisance(dropout=0.1)
    out = nz.perturb_ranges(np.full(20000, 2.0), np.zeros(20000), 0.0, drop, rng)
    assert abs(np.isnan(out).mean() - 0.1) < 0.01
    no_return = nz.perturb_ranges(np.full(5, np.inf), np.zeros(5), 0.0, nz.CLEAN, rng)
    assert np.isinf(no_return).all()
