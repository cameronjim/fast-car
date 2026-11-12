"""L1 tests for tools/mapping_eval (map quality metrics, wheel odometry replica, scan-to-map
registration) and tools/yaw_evidence.py's window statistics. Synthetic maps and scans only."""

from __future__ import annotations

import math

import numpy as np
import pytest
import yaw_evidence
from mapping_eval import odometry, quality, register
from raceline.map_track import DistanceField, MapGrid

RES = 0.05


def box_grid(size: int = 120, margin: int = 10, extra_walls: tuple[int, ...] = ()) -> MapGrid:
    """A square room: one cell thick walls, free inside, unknown outside. extra_walls are
    columns of a second, parallel wall inside the room (a ghost copy)."""
    occ = np.zeros((size, size), bool)
    free = np.zeros((size, size), bool)
    lo, hi = margin, size - margin
    free[lo + 1 : hi, lo + 1 : hi] = True
    occ[lo, lo : hi + 1] = occ[hi, lo : hi + 1] = True
    occ[lo : hi + 1, lo] = occ[lo : hi + 1, hi] = True
    for col in extra_walls:
        occ[lo + 1 : hi, col] = True
        free[lo + 1 : hi, col] = False
    return MapGrid(occ, free, RES, (0.0, 0.0))


def test_clean_room_has_one_layer_and_thin_walls():
    m = quality.map_metrics(box_grid())
    assert m["multi_layer_fraction"] < 0.02
    assert m["thickness_p90_m"] <= 1.5 * RES
    assert m["free_regions"] == 1


def test_ghost_wall_inside_the_room_is_a_second_layer_and_interior_wall():
    clean = quality.map_metrics(box_grid())
    ghost = quality.map_metrics(box_grid(extra_walls=(55,)))
    assert ghost["multi_layer_fraction"] > clean["multi_layer_fraction"] + 0.15
    assert ghost["interior_wall_cells"] > clean["interior_wall_cells"] + 50
    assert ghost["free_regions"] == 2  # the ghost wall splits the floor


def test_smeared_wall_is_thick_but_not_a_second_layer():
    grid = box_grid()
    occ = grid.occupied.copy()
    occ[:, 9:13] |= occ[:, 10:11]  # fatten the left wall to 4 cells
    thick = quality.map_metrics(MapGrid(occ, grid.free & ~occ, RES, (0.0, 0.0)))
    assert thick["thickness_p90_m"] >= 3 * RES
    assert thick["multi_layer_fraction"] < 0.05


def test_empty_map_does_not_crash():
    grid = MapGrid(np.zeros((20, 20), bool), np.zeros((20, 20), bool), RES, (0.0, 0.0))
    m = quality.map_metrics(grid)
    assert m["wall_cells"] == 0
    assert math.isnan(m["thickness_p90_m"])


def test_load_map_strict_keeps_unknown_unknown(tmp_path):
    pgm = tmp_path / "m.pgm"
    pixels = np.array([[0, 205, 254]], dtype=np.uint8)
    pgm.write_bytes(b"P5\n3 1\n255\n" + pixels.tobytes())
    (tmp_path / "m.yaml").write_text(
        "image: m.pgm\nresolution: 0.05\norigin: [1.0, 2.0, 0]\nnegate: 0\n"
        "occupied_thresh: 0.65\nfree_thresh: 0.25\n"
    )
    g = quality.load_map_strict(tmp_path / "m.yaml")
    assert g.occupied.tolist() == [[True, False, False]]
    assert g.free.tolist() == [[False, False, True]]
    assert g.origin_xy == (1.0, 2.0)


def test_servo_rate_limit_and_clamp():
    v = odometry.Vehicle(0.33, -0.4, 0.4, 2.0, (0.0, 0.0, 0.0))
    t = np.array([0.0, 0.1, 0.2, 0.3])
    out = odometry.servo_angle_series(t, np.array([1.0, 1.0, 1.0, 1.0]), v)
    assert out[0] == 0.0
    assert out[1] == pytest.approx(0.2)  # 2 rad/s * 0.1 s
    assert out[-1] == pytest.approx(0.4)  # clamped


def test_wheel_twist_kinematics_and_bag_scale():
    veh = odometry.load_vehicle()
    t = np.arange(0, 3.0, 0.02)
    v = np.full_like(t, 1.401)
    steer = np.full_like(t, 0.3)
    unity = odometry.YawModel(steer_gain=1.0)
    straight = odometry.wheel_twist_series(t, v, t, steer, veh, model=unity)
    assert straight["wz"][-1] == pytest.approx(1.401 * math.tan(0.3) / veh.wheelbase_m)
    # the default takes vehicle_params' measured curvature gain, as wheel_twist_node does
    default = odometry.wheel_twist_series(t, v, t, steer, veh)
    gain = odometry.yaw_gain(veh.yaw_gain_table, np.array([0.3]))[0]
    assert default["wz"][-1] == pytest.approx(gain * straight["wz"][-1])
    assert all(0.5 < g < 1.5 for _, g in veh.yaw_gain_table)
    scaled = odometry.wheel_twist_series(
        t, v, t, steer, veh, odometry.OLD_BAGS, clamp_to_vehicle_limits=False
    )
    assert scaled["v_wheel"][-1] == pytest.approx(1.0)
    assert scaled["delta"][-1] == pytest.approx(0.3 * 1.203)
    right = odometry.wheel_twist_series(
        t, v, t, -steer, veh, odometry.OLD_BAGS, clamp_to_vehicle_limits=False
    )
    assert right["delta"][-1] == pytest.approx(-0.3 * 1.072)  # left positive, per-side scale


def test_yaw_model_options():
    veh = odometry.load_vehicle()
    t = np.arange(0, 2.0, 0.02)
    v, steer = np.full_like(t, 1.0), np.full_like(t, 0.4)
    base = odometry.wheel_twist_series(t, v, t, steer, veh, model=odometry.YawModel(steer_gain=1.0))
    model = odometry.YawModel(
        steer_gain=0.9, ground_ratio_at_zero=1.0, ground_ratio_slope_per_rad=0.25
    )
    out = odometry.wheel_twist_series(t, v, t, steer, veh, model=model)
    ratio = 1.0 - 0.25 * 0.4
    assert out["v"][-1] == pytest.approx(ratio)
    assert out["wz"][-1] == pytest.approx(0.9 * base["wz"][-1] * ratio)


def test_integrate_pose_circle():
    t = np.arange(0, 2 * math.pi, 0.01)
    x, y, yaw = odometry.integrate_pose(t, np.full_like(t, 1.0), np.full_like(t, 1.0))
    assert yaw[-1] == pytest.approx(2 * math.pi, abs=0.02)
    assert math.hypot(x[-1], y[-1]) < 0.05  # back where it started


def _scan_from(grid: MapGrid, pose, mount, n=360):
    """Ray cast the box room from a pose; returns ranges."""
    field = DistanceField(grid)
    ang = np.linspace(-math.pi, math.pi, n, endpoint=False)
    ranges = np.full(n, np.nan)
    c, s = math.cos(pose[2]), math.sin(pose[2])
    for i, a in enumerate(ang):
        for r in np.arange(0.2, 5.0, 0.01):
            bx = mount[0] + r * math.cos(a + mount[2])
            by = mount[1] + r * math.sin(a + mount[2])
            px, py = c * bx - s * by + pose[0], s * bx + c * by + pose[1]
            if field.at(np.array([px]), np.array([py]))[0] < 0.025:
                ranges[i] = r
                break
    return ang[0], ang[1] - ang[0], ranges


def test_registration_recovers_a_known_pose():
    grid = box_grid(size=100, margin=10)
    # an off-centre pose in the 4 m room, asymmetric so the answer is unique
    mount = (0.285, 0.0, math.pi)
    truth = (2.1, 2.4, 0.35)
    amin, ainc, ranges = _scan_from(grid, truth, mount)
    pts = register.scan_points_base(ranges, amin, ainc, mount)
    reg = register.register_scan(DistanceField(grid), pts, (2.15, 2.35, 0.30))
    assert reg.x == pytest.approx(truth[0], abs=0.03)
    assert reg.y == pytest.approx(truth[1], abs=0.03)
    assert reg.yaw == pytest.approx(truth[2], abs=0.02)
    assert reg.inlier_fraction > 0.9


def test_ratio_and_window_stats():
    assert yaw_evidence.ratio(np.array([0.5, 1.0]), np.array([1.0, 2.0])) == pytest.approx(0.5)
    assert math.isnan(yaw_evidence.ratio(np.zeros(2), np.zeros(2)))
    n = 60
    reg_yaw = 0.8 * np.arange(n) * 0.1  # registered turns at 0.8 rad/s
    model_yaw = 1.0 * np.arange(n) * 0.1  # the model says 1.0 rad/s
    xy = np.column_stack([0.9 * 0.1 * np.arange(n), np.zeros(n)])
    path = 1.0 * 0.1 * np.arange(n)
    delta = np.full(n, 0.4)
    w = yaw_evidence.window_stats(
        reg_yaw, xy, model_yaw, path, delta, np.full(n, 1.0), np.ones(n, bool), 10
    )
    rows = yaw_evidence.summarise(w)
    both = [
        r for r in rows if r["side"] == "both" and r["speed_lo"] == 0.0 and r["delta_lo"] == 0.35
    ]
    assert both[0]["yaw_ratio"] == pytest.approx(0.8)
    assert both[0]["speed_ratio"] == pytest.approx(0.9)
    assert both[0]["steer_ratio"] == pytest.approx(0.8 / 0.9)
    assert any(r["side"] == "left" for r in rows) and not any(r["side"] == "right" for r in rows)


def test_window_stats_skips_windows_that_are_not_moving():
    n = 40
    moving = np.ones(n, bool)
    moving[15] = False
    w = yaw_evidence.window_stats(
        np.zeros(n),
        np.zeros((n, 2)),
        np.zeros(n),
        np.zeros(n),
        np.zeros(n),
        np.zeros(n),
        moving,
        10,
    )
    assert len(w["reg_dyaw"]) == (n - 10) - 11  # windows touching scan 15 are gone
