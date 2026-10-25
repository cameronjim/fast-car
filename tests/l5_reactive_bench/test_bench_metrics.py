"""L1 tests for the reactive benchmark's pure metric logic (bench_metrics.py), no ROS.

Hand-computed cases for the geometry, a brute-force cross-check of the clearance maths on random
poses, synthetic recordings for the counters, and the contact signatures of f1tenth_gym's
collision handler.
"""

from __future__ import annotations

import math
import sys
from pathlib import Path

import bench_metrics as bm
import numpy as np
import pytest
import yaml

REPO_ROOT = Path(__file__).resolve().parents[2]
sys.path.insert(0, str(REPO_ROOT / "sim" / "bridge" / "racer_gym_bridge"))

# The real car's box, for hand-computed gaps: front 0.3302 + 0.13 - 0.17145, rear 0.17145 + 0.12.
BOX = bm.BodyBox(front_x_m=0.28875, rear_x_m=0.29145, half_width_m=0.155)
HALF_WALL = bm.WALL_THICKNESS_M / 2.0


def wall(x0, y0, x1, y1) -> np.ndarray:
    return np.array([[x0, y0, x1, y1]], dtype=float)


# -- Body box and the wall thickness ------------------------------------------------------------


def test_body_box_from_the_committed_vehicle_params():
    params = yaml.safe_load((REPO_ROOT / "config" / "vehicle_params.yaml").read_text())
    box = bm.BodyBox.from_vehicle_params(params)
    chassis = params["chassis"]
    assert box.half_width_m == pytest.approx(chassis["width_m"] / 2.0)
    # Front and rear bumpers, wherever the CG is: their sum is the rear-axle-to-bumper length.
    assert box.front_x_m + box.rear_x_m == pytest.approx(
        chassis["wheelbase_m"] + chassis["front_overhang_m"] + chassis["rear_overhang_m"]
    )
    assert box.rear_x_m == pytest.approx(chassis["cg_to_rear_axle_m"] + chassis["rear_overhang_m"])


def test_wall_thickness_matches_the_bridge():
    from racer_gym_bridge.track_loader import WALL_THICKNESS_M

    assert bm.WALL_THICKNESS_M == WALL_THICKNESS_M


# -- body_clearance_m ---------------------------------------------------------------------------


@pytest.mark.parametrize(
    "yaw, near_extent",
    [
        (0.0, BOX.front_x_m),
        (math.pi, BOX.rear_x_m),
        (math.pi / 2.0, BOX.half_width_m),
        (-math.pi / 2.0, BOX.half_width_m),
    ],
)
def test_clearance_to_a_wall_ahead_is_the_gap_to_its_face(yaw, near_extent):
    assert bm.body_clearance_m(0.0, 0.0, yaw, BOX, wall(1.0, -5.0, 1.0, 5.0)) == pytest.approx(
        1.0 - near_extent - HALF_WALL
    )


def test_clearance_diagonal_pose_uses_the_nearest_corner():
    yaw = math.pi / 4.0
    # Furthest forward corner: the front right one, (front, -half width) rotated by 45 degrees.
    x_max = math.cos(yaw) * BOX.front_x_m + math.sin(yaw) * BOX.half_width_m
    assert bm.body_clearance_m(0.0, 0.0, yaw, BOX, wall(1.0, -5.0, 1.0, 5.0)) == pytest.approx(
        1.0 - x_max - HALF_WALL
    )


def test_clearance_to_a_wall_end_is_the_distance_to_the_end_point():
    # A wall segment that stops short of the car's path: its end point at (1, 0.6) is the nearest.
    gap = bm.body_clearance_m(0.0, 0.0, 0.0, BOX, wall(1.0, 0.6, 1.0, 3.0))
    corner = math.hypot(1.0 - BOX.front_x_m, 0.6 - BOX.half_width_m)
    assert gap == pytest.approx(corner - HALF_WALL)


def test_clearance_is_negative_when_the_body_touches_or_overlaps_a_wall():
    # Through the body.
    assert bm.body_clearance_m(0.0, 0.0, 0.0, BOX, wall(0.1, -1.0, 0.1, 1.0)) == pytest.approx(
        -HALF_WALL
    )
    # A wall segment wholly inside the body rectangle.
    assert bm.body_clearance_m(0.0, 0.0, 0.0, BOX, wall(-0.1, 0.0, 0.1, 0.0)) == pytest.approx(
        -HALF_WALL
    )
    # Touching the body's face is a gap of minus half the wall thickness: the wall line is on
    # the body, its face is half a thickness inside it.
    assert bm.body_clearance_m(
        0.0, 0.0, 0.0, BOX, wall(BOX.front_x_m, -1.0, BOX.front_x_m, 1.0)
    ) == (pytest.approx(-HALF_WALL))


def test_clearance_is_zero_gap_to_a_wall_one_half_thickness_from_the_body():
    x = BOX.front_x_m + HALF_WALL
    assert bm.body_clearance_m(0.0, 0.0, 0.0, BOX, wall(x, -1.0, x, 1.0)) == pytest.approx(0.0)


def test_clearance_beside_the_car_and_parallel_to_it():
    gap = bm.body_clearance_m(0.0, 0.0, 0.0, BOX, wall(-3.0, 0.5, 3.0, 0.5))
    assert gap == pytest.approx(0.5 - BOX.half_width_m - HALF_WALL)


def test_clearance_is_capped_and_far_walls_cost_nothing():
    far = wall(100.0, 0.0, 100.0, 1.0)
    assert bm.body_clearance_m(0.0, 0.0, 0.0, BOX, far) == bm.CLEARANCE_CAP_M
    near_but_beyond_cap = wall(3.0, -1.0, 3.0, 1.0)
    assert bm.body_clearance_m(0.0, 0.0, 0.0, BOX, near_but_beyond_cap) == bm.CLEARANCE_CAP_M
    assert bm.body_clearance_m(0.0, 0.0, 0.0, BOX, far, cap_m=0.2) == 0.2


def test_clearance_is_the_minimum_over_all_walls():
    walls = np.array([[2.0, -5.0, 2.0, 5.0], [-5.0, 0.6, 5.0, 0.6], [-1.0, -5.0, -1.0, 5.0]])
    expected = min(2.0 - BOX.front_x_m, 0.6 - BOX.half_width_m, 1.0 - BOX.rear_x_m) - HALF_WALL
    assert bm.body_clearance_m(0.0, 0.0, 0.0, BOX, walls) == pytest.approx(expected)


def _brute_force_gap(x, y, yaw, walls, samples=400):
    """Distance from points sampled along the body outline to the wall lines."""
    corners = np.array(
        [
            (BOX.front_x_m, BOX.half_width_m),
            (BOX.front_x_m, -BOX.half_width_m),
            (-BOX.rear_x_m, -BOX.half_width_m),
            (-BOX.rear_x_m, BOX.half_width_m),
        ]
    )
    pts = []
    for i in range(4):
        a, b = corners[i], corners[(i + 1) % 4]
        for t in np.linspace(0.0, 1.0, samples):
            pts.append(a + t * (b - a))
    pts = np.array(pts)
    c, s = math.cos(yaw), math.sin(yaw)
    px = x + c * pts[:, 0] - s * pts[:, 1]
    py = y + s * pts[:, 0] + c * pts[:, 1]
    best = math.inf
    for ax, ay, bx, by in walls:
        d = bm._point_segment_distance(px, py, ax, ay, bx, by)
        best = min(best, float(d.min()))
    return best


def test_clearance_agrees_with_a_brute_force_outline_distance_on_random_poses():
    rng = np.random.default_rng(5)
    walls = np.array(
        [
            [0.0, 0.0, 4.0, 0.0],
            [4.0, 0.0, 4.0, 3.0],
            [4.0, 3.0, 0.0, 3.0],
            [0.0, 3.0, 0.0, 0.0],
            [1.5, 1.2, 2.5, 1.7],
        ]
    )
    checked = 0
    for _ in range(300):
        x, y = rng.uniform(0.4, 3.6), rng.uniform(0.4, 2.6)
        yaw = rng.uniform(-math.pi, math.pi)
        gap = bm.body_clearance_m(x, y, yaw, BOX, walls, wall_thickness_m=0.0)
        reference = _brute_force_gap(x, y, yaw, walls)
        if gap > 0.01:  # a body overlapping a wall is 0, which a boundary sampling cannot see
            assert gap == pytest.approx(reference, abs=0.004)
            checked += 1
    assert checked > 150


# -- LapTracker ---------------------------------------------------------------------------------

RADIUS = 2.0
CIRCLE_T = np.linspace(0.0, 2.0 * math.pi, 400, endpoint=False)
CIRCLE = (RADIUS * np.cos(CIRCLE_T), RADIUS * np.sin(CIRCLE_T))
CIRCUMFERENCE = 2.0 * math.pi * RADIUS


def _drive_circle(tracker, start_angle, end_angle, t0, speed=1.0, dt=0.05, radius=RADIUS):
    """Feeds positions on a circle from one angle to another at `speed`; returns the end time."""
    n = max(2, int(abs(end_angle - start_angle) * RADIUS / speed / dt))
    t = t0
    for a in np.linspace(start_angle, end_angle, n):
        tracker.update(radius * math.cos(a), radius * math.sin(a), t)
        t += dt
    return t


def test_lap_tracker_loop_length_is_the_closed_perimeter():
    tracker = bm.LapTracker(*CIRCLE)
    # 400-gon inscribed in the circle: just under 2 pi r.
    assert tracker.length_m == pytest.approx(CIRCUMFERENCE, rel=1e-3)


def test_lap_tracker_counts_laps_and_times_them():
    tracker = bm.LapTracker(*CIRCLE)
    _drive_circle(tracker, 0.0, 2.0 * math.pi * 2.5, 100.0)
    assert tracker.laps == 2
    # 1 m/s round a 12.57 m circle.
    assert tracker.lap_times_s == pytest.approx([CIRCUMFERENCE, CIRCUMFERENCE], abs=0.3)
    assert tracker.best_progress_m == pytest.approx(2.5 * CIRCUMFERENCE, rel=0.02)


def test_lap_tracker_does_not_double_count_after_backing_up():
    tracker = bm.LapTracker(*CIRCLE)
    t = _drive_circle(tracker, 0.0, 2.0 * math.pi * 1.2, 0.0)
    assert tracker.laps == 1
    t = _drive_circle(tracker, 2.0 * math.pi * 1.2, 2.0 * math.pi * 0.8, t)  # back over the line
    assert tracker.laps == 1
    assert tracker.progress_m < tracker.best_progress_m
    _drive_circle(tracker, 2.0 * math.pi * 0.8, 2.0 * math.pi * 1.9, t)
    assert tracker.laps == 1  # lap 2 starts at 2.0 turns
    _drive_circle(tracker, 2.0 * math.pi * 1.9, 2.0 * math.pi * 2.1, t + 100.0)
    assert tracker.laps == 2


def test_lap_tracker_counts_the_direction_of_the_loop_only():
    tracker = bm.LapTracker(*CIRCLE)
    _drive_circle(tracker, 0.0, -2.0 * math.pi * 1.5, 0.0)  # the wrong way round
    assert tracker.laps == 0
    assert tracker.progress_m < 0.0


def test_lap_tracker_reset_forgets_the_run_and_restarts_from_the_new_position():
    tracker = bm.LapTracker(*CIRCLE)
    _drive_circle(tracker, 0.0, 2.0 * math.pi * 1.1, 0.0)
    tracker.reset()
    assert tracker.laps == 0 and tracker.best_progress_m == 0.0
    # A car restarted half way round the loop: its first sample is progress 0, not half a lap.
    _drive_circle(tracker, math.pi, math.pi + 2.0 * math.pi * 0.3, 50.0)
    assert tracker.laps == 0
    assert tracker.best_progress_m == pytest.approx(0.3 * CIRCUMFERENCE, rel=0.03)


def test_lap_tracker_reports_the_distance_from_the_centreline():
    tracker = bm.LapTracker(*CIRCLE)
    tracker.update(RADIUS + 0.3, 0.0, 0.0)
    assert tracker.lateral_m == pytest.approx(0.3, abs=0.01)


def test_lap_tracker_distance_is_to_the_centreline_polyline_not_its_points():
    # A straight centreline sampled every 0.5 m: a car on it half way between two points is 0
    # from the line (0.25 m from the nearest point), and one 0.1 m beside it is 0.1 m away.
    xs = np.array([0.0, 0.5, 1.0, 1.5, 1.5, 0.0])
    ys = np.array([0.0, 0.0, 0.0, 0.0, 1.0, 1.0])
    tracker = bm.LapTracker(xs, ys)
    tracker.update(0.75, 0.0, 0.0)
    assert tracker.lateral_m == pytest.approx(0.0, abs=1e-12)
    tracker.update(1.25, -0.1, 0.1)
    assert tracker.lateral_m == pytest.approx(0.1, abs=1e-12)


def test_lap_tracker_centreline_offset_per_lap_and_over_the_run():
    # Two laps 0.1 m outside the centreline, then a part lap 0.3 m outside it.
    tracker = bm.LapTracker(*CIRCLE)
    t = _drive_circle(tracker, 0.0, 2.0 * math.pi * 2.0 + 0.01, 0.0, radius=RADIUS + 0.1)
    _drive_circle(tracker, 2.0 * math.pi * 2.0 + 0.01, 2.0 * math.pi * 2.5, t, radius=RADIUS + 0.3)
    assert tracker.laps == 2
    assert tracker.lap_centreline_offsets_m == pytest.approx([0.1, 0.1], abs=0.002)
    # Over the run: about 4 parts at 0.1 and 1 at 0.3 by distance.
    assert tracker.mean_centreline_offset_m == pytest.approx((4 * 0.1 + 0.3) / 5, abs=0.01)
    tracker.reset()
    assert tracker.lap_centreline_offsets_m == [] and tracker.mean_centreline_offset_m is None


# -- count_reversals ----------------------------------------------------------------------------


def test_count_reversals_basic_shapes():
    assert bm.count_reversals([0.0, 0.1, 0.2, 0.3, 0.4], 0.1) == 0
    assert bm.count_reversals([0.4, 0.2, 0.0, -0.2], 0.1) == 0
    assert bm.count_reversals([0.0, 0.5, 0.0, 0.5], 0.2) == 2  # up, down, up
    assert bm.count_reversals([], 0.1) == 0
    assert bm.count_reversals([1.0], 0.1) == 0


def test_count_reversals_ignores_wiggles_below_the_swing():
    values = [0.0, 0.02, -0.02, 0.03, -0.03, 0.01] * 5
    assert bm.count_reversals(values, 0.1) == 0
    assert bm.count_reversals(values, 0.04) > 0


def test_count_reversals_zigzag():
    # A zig-zag of amplitude 0.3: every peak and trough after the first rise is confirmed by
    # the swing after it, except the last extreme.
    values = [0.0, 0.3] * 4 + [0.0]
    assert bm.count_reversals(values, 0.1) == len(values) - 2


def test_count_reversals_rejects_a_bad_swing():
    with pytest.raises(ValueError):
        bm.count_reversals([0.0, 1.0], 0.0)


# -- contact signatures -------------------------------------------------------------------------


def test_gym_contact_signature_yaw_jump():
    assert bm.gym_contact_signature(1.2, 0.0, 0.9, 0.0) == "gym_yaw_jump"
    # Across the +-pi seam is a small step, not a jump.
    assert bm.gym_contact_signature(3.13, -3.13, 0.9, 0.9) is None


def test_gym_contact_signature_speed_drop_with_heading_already_zero():
    assert bm.gym_contact_signature(0.0, 0.0, 0.9, 0.0) == "gym_speed_drop"


def test_gym_contact_signature_ordinary_driving_and_braking_is_not_a_contact():
    assert bm.gym_contact_signature(0.5, 0.51, 0.9, 0.89) is None
    # Hard but physical braking: 0.1 m/s per step.
    assert bm.gym_contact_signature(0.5, 0.5, 0.9, 0.8) is None


def test_teleport_is_a_big_position_jump():
    assert bm.is_teleport((0.0, 0.0), (0.6, 0.0))
    assert not bm.is_teleport((0.0, 0.0), (0.03, 0.01))


# -- compute_metrics ----------------------------------------------------------------------------


def _recording(seconds=10.0, speed=1.0) -> bm.Recording:
    rec = bm.Recording()
    for i in range(int(seconds * 100)):
        t = 1000.0 + i * 0.01
        rec.odom.append((t, speed * i * 0.01, 0.0, 0.0, speed))
    return rec


def test_compute_metrics_counts_brakes_escapes_and_contacts():
    rec = _recording()
    # Requests: forward, reverse, forward, reverse, reverse: two entries into reverse.
    rec.drive_raw = [(1000.0 + i, s, 0.0) for i, s in enumerate([0.5, -0.5, 0.5, -0.5, -0.5])]
    rec.events = [
        (1001.0, "ttc", 0, 2),  # a forward obstacle brake: counted
        (1002.0, "ttc", 1, 2),  # its release: not
        (1003.0, "ttc_reverse", 0, 2),  # a rear one: counted
        (1004.0, "watchdog", 0, 2),  # another gate: not an obstacle brake
        (1005.0, "ttc", 0, 0),  # an advisory (INFO) engagement: not
    ]
    rec.contacts = [(1003.0, 3.0, 0.0, "gym_yaw_jump")]
    walls = wall(-5.0, 1.0, 20.0, 1.0)
    m = bm.compute_metrics(rec, box=BOX, walls=walls, lap_tracker=None)
    assert m["obstacle_brakes"] == 2
    assert m["escapes"] == 2
    assert m["wall_contacts"] == 1
    assert m["contact_points"] == [{"t_s": 3.0, "x_m": 3.0, "y_m": 0.0, "reason": "gym_yaw_jump"}]
    assert m["laps"] is None and m["lap_time_s"] is None and m["brakes_per_lap"] is None
    assert m["min_body_clearance_m"] == pytest.approx(1.0 - BOX.half_width_m - HALF_WALL, abs=1e-4)
    assert m["duration_s"] == pytest.approx(9.99)
    assert m["distance_m"] == pytest.approx(9.99, abs=0.01)
    assert m["mean_speed_mps"] == pytest.approx(1.0)
    assert m["schema_version"] == bm.SCHEMA_VERSION


def test_compute_metrics_steering_statistics():
    rec = _recording(seconds=60.0)
    # 50 Hz request, +-0.3 rad square wave with a 1 s period: two reversals a second.
    rec.drive_raw = [
        (1000.0 + i * 0.02, 0.9, 0.3 if (i // 25) % 2 == 0 else -0.3) for i in range(3000)
    ]
    m = bm.compute_metrics(rec, box=BOX, walls=wall(0.0, 50.0, 1.0, 50.0), lap_tracker=None)
    assert m["mean_abs_steering_rad"] == pytest.approx(0.3)
    # 60 s of it: 120 changes of sign, all confirmed but the first and the last extreme's.
    assert m["steering_reversals_per_min"] == pytest.approx(120.0, abs=3.0)


def test_compute_metrics_laps_and_brakes_per_lap():
    rec = _recording()
    rec.events = [(1001.0, "ttc", 0, 2), (1002.0, "ttc", 0, 2), (1003.0, "ttc", 0, 2)]
    tracker = bm.LapTracker(*CIRCLE)
    _drive_circle(tracker, 0.0, 2.0 * math.pi * 2.2, 0.0)
    m = bm.compute_metrics(rec, box=BOX, walls=wall(0.0, 50.0, 1.0, 50.0), lap_tracker=tracker)
    assert m["laps"] == 2
    assert m["lap_time_s"] == pytest.approx(CIRCUMFERENCE, abs=0.3)
    assert len(m["lap_times_s"]) == 2
    assert m["brakes_per_lap"] == pytest.approx(1.5)
    assert m["loop_length_m"] == pytest.approx(CIRCUMFERENCE, rel=1e-3)
    # Driven on the centreline: the offsets are the 400-gon's sagitta at most.
    assert m["mean_centreline_offset_m"] == pytest.approx(0.0, abs=1e-3)
    assert m["lap_centreline_offsets_m"] == pytest.approx([0.0, 0.0], abs=1e-3)


def test_compute_metrics_without_a_loop_has_no_centreline_offset():
    m = bm.compute_metrics(
        _recording(), box=BOX, walls=wall(0.0, 50.0, 1.0, 50.0), lap_tracker=None
    )
    assert m["mean_centreline_offset_m"] is None and m["lap_centreline_offsets_m"] == []


def test_compute_metrics_distance_skips_a_reset_teleport():
    rec = bm.Recording()
    for i in range(100):
        rec.odom.append((i * 0.01, i * 0.01, 0.0, 0.0, 1.0))
    for i in range(100):
        rec.odom.append((1.0 + i * 0.01, 5.0 + i * 0.01, 0.0, 0.0, 1.0))  # 4 m jump at the join
    m = bm.compute_metrics(rec, box=BOX, walls=wall(0.0, 50.0, 1.0, 50.0), lap_tracker=None)
    assert m["distance_m"] == pytest.approx(1.98, abs=0.02)


def test_compute_metrics_reports_the_real_time_factor():
    rec = _recording(seconds=10.0)
    m = bm.compute_metrics(rec, box=BOX, walls=wall(0.0, 50.0, 1.0, 50.0), lap_tracker=None)
    assert m["real_time_factor"] == pytest.approx(1.0, abs=0.01)
    # The same 1000 steps stamped over twice the time: the sim ran at half speed.
    slow = bm.Recording()
    slow.odom = [(t * 2.0, x, y, yaw, v) for t, x, y, yaw, v in rec.odom]
    m = bm.compute_metrics(slow, box=BOX, walls=wall(0.0, 50.0, 1.0, 50.0), lap_tracker=None)
    assert m["real_time_factor"] == pytest.approx(0.5, abs=0.01)


def test_compute_metrics_without_requests_has_no_steering_numbers():
    m = bm.compute_metrics(
        _recording(), box=BOX, walls=wall(0.0, 50.0, 1.0, 50.0), lap_tracker=None
    )
    assert m["mean_abs_steering_rad"] is None
    assert m["steering_reversals_per_min"] is None
    assert m["escapes"] == 0 and m["obstacle_brakes"] == 0


def test_compute_metrics_needs_odometry():
    with pytest.raises(ValueError):
        bm.compute_metrics(bm.Recording(), box=BOX, walls=wall(0, 0, 1, 0), lap_tracker=None)


def test_compute_metrics_is_json_serialisable():
    import json

    rec = _recording()
    rec.drive_raw = [(1000.0, 0.5, 0.1), (1000.02, 0.5, -0.1)]
    tracker = bm.LapTracker(*CIRCLE)
    _drive_circle(tracker, 0.0, 2.0 * math.pi * 1.2, 0.0)
    m = bm.compute_metrics(rec, box=BOX, walls=wall(0.0, 50.0, 1.0, 50.0), lap_tracker=tracker)
    assert json.loads(json.dumps(m)) == m


def test_clearance_on_a_synthetic_track_start_pose():
    """End to end with the bridge's own geometry: the car at the start of lane_square sits
    (lane width - car width) / 2 from each wall, less half a wall thickness."""
    from racer_gym_bridge.track_loader import build_synthetic_track

    track = build_synthetic_track("lane_square")
    gap = bm.body_clearance_m(
        float(track.centreline_x[0]), float(track.centreline_y[0]), 0.0, BOX, track.walls
    )
    assert gap == pytest.approx((1.1 - 2.0 * BOX.half_width_m) / 2.0 - HALF_WALL, abs=1e-6)
