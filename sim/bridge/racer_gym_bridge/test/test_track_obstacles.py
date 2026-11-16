"""L1 tests for racer_gym_bridge.track_obstacles and the round donut (brief 40, pre-registration
v0.4 section 5), pure/ROS-free."""

from __future__ import annotations

import math

import numpy as np
import pytest
from racer_gym_bridge.track_loader import build_synthetic_track, map_track
from racer_gym_bridge.track_obstacles import (
    OBSTACLE_RAMP_M,
    OBSTACLE_START_CLEARANCE_M,
    Obstacle,
    ObstacleError,
    apply_obstacles,
    arc_positions,
    ground_truth_grid,
    loop_orientation,
    parse_obstacles,
)

WALL = 0.08  # track_loader.WALL_THICKNESS_M
RES = 0.05


def box(at, side, p=0.25, length=0.30):
    return {"at": at, "side": side, "protrusion_m": p, "length_m": length}


def stadium(obstacles=None, lane=1.0, straight=3.0, radius=0.4):
    return build_synthetic_track(
        "stadium",
        lane_width_m=lane,
        straight_length_m=straight,
        island_radius_m=radius,
        obstacles=obstacles,
    )


def free_distance(track, x, y, dx, dy, cap=2.0):
    """Distance from (x, y) along a unit direction to the first wall cell of the grid the
    simulator ray casts (what the oracle and the LiDAR see)."""
    grid, (ox, oy), res = ground_truth_grid(track)
    for d in np.arange(0.0, cap, 0.005):
        col = math.floor((x + d * dx - ox) / res)
        row = math.floor((y + d * dy - oy) / res)
        if grid[row, col] < 128.0:
            return d
    return cap


def lane_free_width(track, x, y, nx, ny):
    return free_distance(track, x, y, nx, ny) + free_distance(track, x, y, -nx, -ny)


def frame(track, arc):
    s, _length = arc_positions(track.centreline_x, track.centreline_y)
    i = int(np.argmin(np.abs(s - arc)))
    j = (i + 1) % len(s)
    t = np.array(
        [
            track.centreline_x[j] - track.centreline_x[i - 1],
            track.centreline_y[j] - track.centreline_y[i - 1],
        ]
    )
    t /= np.hypot(*t)
    return np.array([track.centreline_x[i], track.centreline_y[i]]), t, np.array([-t[1], t[0]])


# -- the round donut --------------------------------------------------------------------------------


def test_zero_straight_is_a_round_donut_without_duplicate_points():
    track = build_synthetic_track(
        "stadium", lane_width_m=1.0, straight_length_m=0.0, island_radius_m=0.45
    )
    x, y = track.centreline_x, track.centreline_y
    step = np.hypot(np.diff(np.r_[x, x[0]]), np.diff(np.r_[y, y[0]]))
    assert step.min() > 0.04, "no duplicated or zero length step"
    assert step.max() < 0.06
    r = np.hypot(x - 0.45, y - 0.45)
    assert np.allclose(r, 0.45 + 0.5, atol=1e-6), "a circle of radius island + lane / 2"
    assert step.sum() == pytest.approx(2.0 * math.pi * 0.95, rel=1e-3)
    assert (x[0], y[0]) == pytest.approx((0.45, -0.5)), "starts at the bottom, facing +x"
    assert x[1] > x[0]
    assert loop_orientation(x, y) == 1
    # both walls are closed circles: the inner of radius 0.45, the outer of 1.45
    wall_r = np.hypot(track.walls[:, 0] - 0.45, track.walls[:, 1] - 0.45)
    assert set(np.round(wall_r, 2)) == {0.45, 1.45}


def test_zero_straight_reversed_starts_on_the_same_circle():
    track = build_synthetic_track(
        "stadium", lane_width_m=1.0, straight_length_m=0.0, island_radius_m=0.45
    ).reversed()
    assert loop_orientation(track.centreline_x, track.centreline_y) == -1


# -- parsing and refusals --------------------------------------------------------------------------


def test_parse_accepts_the_registered_shape():
    (ob,) = parse_obstacles([box(0.3, "outer")])
    assert ob == Obstacle(0.3, "outer", 0.25, 0.30)
    assert parse_obstacles(None) == () and parse_obstacles([]) == ()


@pytest.mark.parametrize(
    "bad",
    [
        {"at": 0.3, "side": "outer", "protrusion_m": 0.2},  # a key missing
        {**box(0.3, "outer"), "extra": 1},
        box(1.0, "outer"),
        box(-0.1, "outer"),
        box(0.3, "middle"),
        box(0.3, "outer", p=0.0),
        box(0.3, "outer", length=-1.0),
    ],
)
def test_parse_refuses_bad_obstacles(bad):
    with pytest.raises(ObstacleError):
        parse_obstacles([bad])


def test_a_box_that_blocks_the_lane_is_refused():
    with pytest.raises(ObstacleError, match="blocks the lane"):
        stadium([box(0.5, "outer", p=0.95)])


def test_two_boxes_with_touching_ramps_are_refused():
    with pytest.raises(ObstacleError, match="too close"):
        stadium([box(0.50, "outer"), box(0.52, "inner")])


def test_obstacles_need_a_loop_and_only_once():
    open_room = build_synthetic_track("open_room")
    with pytest.raises(ObstacleError, match="loops"):
        apply_obstacles(open_room, [box(0.5, "outer")])
    once = stadium([box(0.5, "outer")])
    with pytest.raises(ObstacleError, match="already"):
        apply_obstacles(once, [box(0.2, "inner")])


def test_no_obstacles_is_the_same_object():
    base = stadium()
    assert apply_obstacles(base, []) is base
    assert build_synthetic_track("stadium", straight_length_m=3.0).obstacles == ()


# -- the start pose rule ---------------------------------------------------------------------------


@pytest.mark.parametrize("at", [0.0, 0.02, 0.97])
def test_a_box_near_the_start_pose_is_refused_not_moved(at):
    with pytest.raises(ObstacleError, match="start pose"):
        stadium([box(at, "outer")])


def test_a_box_just_beyond_the_start_clearance_is_accepted():
    track = stadium()
    _, length = arc_positions(track.centreline_x, track.centreline_y)
    at = (OBSTACLE_START_CLEARANCE_M + 0.15 + 0.02) / length  # the box edge 0.5 m + 2 cm away
    assert stadium([box(at, "outer")]).obstacles


# -- the lane narrows by the protrusion at the box and nowhere else ---------------------------------


@pytest.mark.parametrize("side", ["inner", "outer"])
def test_a_box_narrows_the_lane_by_its_protrusion_at_its_station_only(side):
    # a long straight so the lane is exactly parallel walls at the box
    base = stadium(straight=4.0)
    _, length = arc_positions(base.centreline_x, base.centreline_y)
    p, blen = 0.25, 0.30
    at = 1.0 / length  # 1 m from the start: on the bottom straight (2 m to its end)
    track = stadium([box(at, side, p, blen)], straight=4.0)
    at_arc = at * length
    # the shifted midline at the box station, and its normal, on the straight (normal +y)
    centre, tangent, normal = frame(track, at_arc)
    assert abs(tangent[1]) < 0.01, "the box is on the bottom straight"
    free_plain = lane_free_width(base, centre[0], base.centreline_y[0], 0.0, 1.0)
    assert free_plain == pytest.approx(1.0 - WALL, abs=RES)
    free_box = lane_free_width(track, centre[0], centre[1], normal[0], normal[1])
    assert free_box == pytest.approx(1.0 - WALL - p, abs=RES)
    for offset in (0.4, 0.7, 1.2):  # beyond the box and its ramp: unchanged
        for sign in (-1.0, 1.0):
            c, _t, n = frame(track, at_arc + sign * (blen / 2 + offset))
            assert lane_free_width(track, c[0], c[1], n[0], n[1]) == pytest.approx(
                1.0 - WALL, abs=RES
            )


@pytest.mark.parametrize("side", ["inner", "outer"])
def test_the_midline_is_the_middle_of_the_free_lane_at_the_box(side):
    base = stadium(straight=4.0)
    _, length = arc_positions(base.centreline_x, base.centreline_y)
    at = 1.0 / length
    track = stadium([box(at, side, 0.25, 0.30)], straight=4.0)
    c, _t, n = frame(track, at * length)
    left = free_distance(track, c[0], c[1], n[0], n[1])
    right = free_distance(track, c[0], c[1], -n[0], -n[1])
    assert abs(left - right) < RES
    # away from the wall the box leans on: inner is the left wall of a ccw loop
    narrow, wide = (left, right) if side == "inner" else (right, left)
    assert narrow == pytest.approx(wide, abs=RES)


def test_the_box_is_solid_and_against_the_wall():
    base = stadium(straight=4.0)
    _, length = arc_positions(base.centreline_x, base.centreline_y)
    at = 1.0 / length
    track = stadium([box(at, "outer", 0.25, 0.30)], straight=4.0)
    grid, (ox, oy), res = ground_truth_grid(track)
    c, t, n = frame(base, at * length)  # the unshifted centreline: the lane's own frame
    outer_n = -n  # the outer wall is on the right of a ccw loop
    surface = 0.5 - WALL / 2
    for depth in np.arange(surface - 0.25 + 0.03, surface + 0.07, 0.02):  # box face to its back
        for along in (-0.12, 0.0, 0.12):
            p = c + t * along + outer_n * depth
            assert grid[int((p[1] - oy) / res), int((p[0] - ox) / res)] == 0.0, (depth, along)
    # beyond the box's ends the same depths are free lane
    for along in (-0.25, 0.25):
        p = c + t * along + outer_n * (surface - 0.1)
        assert grid[int((p[1] - oy) / res), int((p[0] - ox) / res)] == 255.0


def test_walls_get_the_box_segments_and_the_scorer_sees_them():
    base = stadium()
    track = stadium([box(0.3, "inner")])
    assert len(track.walls) > len(base.walls)
    new = track.walls[len(base.walls) :]
    assert new.shape[1] == 4 and np.isfinite(new).all()


def test_centreline_start_is_unmoved_and_the_midline_stays_inside_the_free_lane():
    base = stadium(lane=1.0, straight=3.0)
    track = stadium([box(0.15, "outer"), box(0.62, "inner")], straight=3.0)
    assert (track.centreline_x[0], track.centreline_y[0]) == (
        base.centreline_x[0],
        base.centreline_y[0],
    )
    assert len(track.centreline_x) == len(base.centreline_x)
    shift = np.hypot(track.centreline_x - base.centreline_x, track.centreline_y - base.centreline_y)
    assert shift.max() == pytest.approx(0.125, abs=1e-3), "protrusion / 2"
    for x, y in list(zip(track.centreline_x, track.centreline_y, strict=True))[::7]:
        for _ in range(1):
            free = min(
                free_distance(track, x, y, dx, dy) for dx, dy in ((1, 0), (-1, 0), (0, 1), (0, -1))
            )
            assert free > 0.2, "the midline is in free space, away from every wall and box"


def test_reversed_track_keeps_the_obstacles_and_walls():
    track = stadium([box(0.4, "outer")])
    rev = track.reversed()
    assert rev.obstacles == track.obstacles
    assert np.array_equal(rev.walls, track.walls)
    assert loop_orientation(rev.centreline_x, rev.centreline_y) == -1


def test_obstacle_centres_are_filled_in_and_inside_the_lane():
    track = stadium([box(0.4, "inner", 0.25, 0.3)])
    (ob,) = track.obstacles
    assert math.isfinite(ob.centre_x) and math.isfinite(ob.centre_y)
    d = min(np.hypot(track.centreline_x - ob.centre_x, track.centreline_y - ob.centre_y))
    assert d < 0.5


def test_ramp_constant_is_what_the_comment_says():
    assert OBSTACLE_RAMP_M == pytest.approx(0.25)


def test_registered_case_shapes_build_in_both_directions():
    cases = [
        {
            "lane": 1.0,
            "straight": 0.0,
            "radius": 0.5,
            "obstacles": [box(0.30, "outer", 0.25, 0.30)],
        },
        {"lane": 1.0, "straight": 0.0, "radius": 0.45, "obstacles": []},
        {
            "lane": 1.1,
            "straight": 0.0,
            "radius": 0.45,
            "obstacles": [box(0.15, "outer", 0.25, 0.30), box(0.60, "inner", 0.25, 0.30)],
        },
        {
            "lane": 1.2,
            "straight": 1.0,
            "radius": 0.4,
            "obstacles": [
                box(0.10, "inner", 0.30, 0.40),
                box(0.45, "outer", 0.30, 0.40),
                box(0.80, "outer", 0.30, 0.20),
            ],
        },
    ]
    for c in cases:
        track = stadium(c["obstacles"], c["lane"], c["straight"], c["radius"])
        assert len(track.obstacles) == len(c["obstacles"])
        track.reversed()


# -- a map ------------------------------------------------------------------------------------------


def write_ring_map(tmp_path, ccw=True):
    """A 5 x 4 m room with a 2.6 x 1.6 m island: a 1.2 m lane all round, as a pgm + yaml, and its
    centreline csv (rectangle through the lane's middle, starting mid bottom)."""
    cols, rows = 100, 80
    pixels = np.full((rows, cols), 254, dtype=np.uint8)

    def outline(x0, y0, x1, y1):
        c0, c1 = round(x0 / RES), round(x1 / RES)
        r0, r1 = round(y0 / RES), round(y1 / RES)
        pixels[r0 : r1 + 1, [c0, c1 - (1 if c1 == cols else 0)]] = 0
        pixels[[r0, r1 - (1 if r1 == rows else 0)], c0 : c1 + 1] = 0

    outline(0.0, 0.0, 5.0, 4.0)
    outline(1.2, 1.2, 3.8, 2.8)
    flipped = np.flipud(pixels)  # row 0 of the image is the top
    (tmp_path / "ring.pgm").write_bytes(b"P5\n100 80\n255\n" + flipped.tobytes())
    (tmp_path / "ring.yaml").write_text(
        "image: ring.pgm\nresolution: 0.05\norigin: [0.0, 0.0, 0]\nnegate: 0\n"
        "occupied_thresh: 0.65\nfree_thresh: 0.25\n"
    )
    pts = []
    for x in np.arange(2.5, 4.4, 0.05):
        pts.append((x, 0.6))
    for y in np.arange(0.6, 3.4, 0.05):
        pts.append((4.4, y))
    for x in np.arange(4.4, 0.6, -0.05):
        pts.append((x, 3.4))
    for y in np.arange(3.4, 0.6, -0.05):
        pts.append((0.6, y))
    for x in np.arange(0.6, 2.5, 0.05):
        pts.append((x, 0.6))
    if not ccw:
        pts = pts[::-1]
    lines = ["# test ring", "s_m,x_m,y_m,heading_rad,curvature_1pm,target_speed_mps"]
    for k, (x, y) in enumerate(pts):
        lines.append(f"{k * 0.05:.3f},{x:.4f},{y:.4f},0.0,0.0,1.0")
    (tmp_path / "ring_centreline.csv").write_text("\n".join(lines) + "\n")
    return tmp_path / "ring.yaml", tmp_path / "ring_centreline.csv"


@pytest.mark.parametrize("ccw", [True, False])
def test_map_obstacle_is_drawn_into_the_grid_against_the_right_wall(tmp_path, ccw):
    yaml_path, csv_path = write_ring_map(tmp_path, ccw)
    base = map_track(yaml_path, csv_path)
    assert loop_orientation(base.centreline_x, base.centreline_y) == (1 if ccw else -1)
    track = apply_obstacles(base, [box(0.30, "outer", 0.25, 0.30)])
    changed = np.argwhere(track.occupancy != base.occupancy)
    assert len(changed) > 20 and (track.occupancy[tuple(changed.T)] == 0.0).all()
    # outer is the room wall (0 or 5 / 4 m): every new cell is near the room wall, in the lane
    cx = (changed[:, 1] + 0.5) * RES
    cy = (changed[:, 0] + 0.5) * RES
    dist_to_room_wall = np.minimum.reduce([cx, 5.0 - cx, cy, 4.0 - cy])
    assert dist_to_room_wall.max() <= 0.25 + 0.15  # protrusion plus a cell or two
    # the walls array (clearance metric) gained the cells and the obstacle knows its centre
    assert len(track.walls) == len(base.walls) + len(
        np.argwhere((track.occupancy == 0.0) & (base.occupancy == 255.0))
    )
    assert math.isfinite(track.obstacles[0].centre_x)
    # the lane at the box is narrower by the protrusion (grid walls are one cell)
    free_before = lane_free_width(base, *_box_station(base, 0.30))
    free_after = lane_free_width(track, *_box_station(track, 0.30))
    assert free_before - free_after == pytest.approx(0.25, abs=0.07)


def _box_station(track, at):
    _s, length = arc_positions(track.centreline_x, track.centreline_y)
    c, _t, n = frame(track, at * length)
    return c[0], c[1], n[0], n[1]


def test_map_obstacle_without_a_wall_in_reach_is_refused(tmp_path):
    cols = rows = 160
    pixels = np.full((rows, cols), 254, dtype=np.uint8)
    pixels[0, :] = 0  # one wall far from the centreline
    (tmp_path / "open.pgm").write_bytes(b"P5\n160 160\n255\n" + pixels.tobytes())
    (tmp_path / "open.yaml").write_text(
        "image: open.pgm\nresolution: 0.05\norigin: [0.0, 0.0, 0]\nnegate: 0\n"
    )
    t = np.linspace(0.0, 2.0 * math.pi, 120, endpoint=False)
    lines = ["# c", "s_m,x_m,y_m,heading_rad,curvature_1pm,target_speed_mps"]
    for k, a in enumerate(t):
        lines.append(
            f"{k * 0.05:.3f},{4.0 + 1.5 * math.cos(a):.4f},{4.0 + 1.5 * math.sin(a):.4f},0,0,1"
        )
    (tmp_path / "open.csv").write_text("\n".join(lines) + "\n")
    base = map_track(tmp_path / "open.yaml", tmp_path / "open.csv")
    with pytest.raises(ObstacleError, match="no wall within"):
        apply_obstacles(base, [box(0.30, "outer")])
