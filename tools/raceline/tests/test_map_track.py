"""L1 tests for raceline.map_track and tools/track_session/pipeline.py: a map -> centreline, lane
geometry and a racing line that respects the car's turning limits and body, on maps built here
from known geometry (a thin-island stadium), so every expectation is analytic.
"""

from __future__ import annotations

import math
import sys
from pathlib import Path

import numpy as np
import pytest

_REPO = Path(__file__).resolve().parents[3]
sys.path.insert(0, str(_REPO / "tools" / "track_session"))

import pipeline
from raceline import map_track as mt

RES = 0.05


def stadium_map(tmp_path: Path, island_len=2.0, lane=1.0, island_r=0.0) -> Path:
    """A thin-walled stadium lane as a slam_toolbox-style pgm: free in the lane, wall cells on the
    island and the outer wall (0.1 m thick bands), unknown elsewhere. Island: the segment
    (0, 0)-(island_len, 0) grown by island_r; outer wall at island_r + lane."""
    pad = lane + island_r + 0.5
    xs = np.arange(-pad, island_len + pad, RES) + RES / 2
    ys = np.arange(-pad, pad, RES) + RES / 2
    gx, gy = np.meshgrid(xs, ys)
    d = np.hypot(gx - np.clip(gx, 0.0, island_len), gy)  # distance to the island segment
    inner, outer = island_r, island_r + lane
    pixels = np.full(d.shape, 205, dtype=np.uint8)
    pixels[(d > inner + 0.05) & (d < outer - 0.05)] = 254
    pixels[(np.abs(d - inner) <= 0.05)] = 0
    pixels[(np.abs(d - outer) <= 0.05)] = 0
    pixels[d < inner - 0.05] = 205
    image = np.flipud(pixels)
    pgm = tmp_path / "stadium.pgm"
    pgm.write_bytes(f"P5\n{image.shape[1]} {image.shape[0]}\n255\n".encode() + image.tobytes())
    yaml_path = tmp_path / "stadium.yaml"
    yaml_path.write_text(
        f"image: stadium.pgm\nresolution: {RES}\norigin: [{xs[0] - RES / 2}, {ys[0] - RES / 2}, 0]\n"
        "negate: 0\noccupied_thresh: 0.65\nfree_thresh: 0.25\n"
    )
    return yaml_path


@pytest.fixture(scope="module")
def thin_island(tmp_path_factory):
    return stadium_map(tmp_path_factory.mktemp("thin"), island_len=2.0, lane=1.0, island_r=0.0)


def _csv(path):
    rows = [ln.split(",") for ln in Path(path).read_text().splitlines() if ln and ln[0] != "#"]
    return np.array(rows[1:], dtype=float)


@pytest.fixture(scope="module")
def round_island(tmp_path_factory):
    return stadium_map(tmp_path_factory.mktemp("round"), island_len=2.0, lane=1.0, island_r=0.4)


def loop_length(x, y):
    return float(np.hypot(np.diff(x, append=x[0]), np.diff(y, append=y[0])).sum())


def test_load_map_reads_cells_with_row_zero_at_min_y(thin_island):
    grid = mt.load_map(thin_island)
    assert grid.resolution_m == RES
    assert grid.occupied.any() and grid.free.any()
    # The island line (y = 0) sits at the cell row of y = 0.
    row, col = grid.cell_of(1.0, 0.0)
    assert grid.occupied[row, col]


def test_lane_mask_is_the_lane_only(thin_island):
    grid = mt.load_map(thin_island)
    mask = mt.lane_mask(grid, (1.0, -0.5))
    row, col = grid.cell_of(1.0, -0.5)
    assert mask[row, col]
    assert not (mask & grid.occupied).any()
    # Not the unknown floor outside the outer wall.
    row, col = grid.cell_of(1.0, -1.4)
    assert not mask[row, col]


def test_centreline_closes_and_has_the_analytic_length_and_ccw_direction(thin_island):
    grid = mt.load_map(thin_island)
    x, y = mt.trace_centreline(grid, (1.0, -0.5))
    # Two straights of 2 m and two half circles of radius W / 2 = 0.5.
    assert loop_length(x, y) == pytest.approx(4.0 + 2.0 * math.pi * 0.5, rel=0.04)
    area = 0.5 * float(np.sum(x * np.roll(y, -1) - np.roll(x, -1) * y))
    assert area > 0.0  # counter-clockwise
    # Every point is mid lane: 0.5 from the island line or its ends.
    d = np.hypot(x - np.clip(x, 0.0, 2.0), y)
    assert d == pytest.approx(0.5, abs=0.06)


def test_lane_report_finds_the_width_the_island_and_the_ends(thin_island):
    grid = mt.load_map(thin_island)
    x, y = mt.trace_centreline(grid, (1.0, -0.5))
    report = mt.lane_report(grid, x, y)
    assert report.width_median_m == pytest.approx(1.0 - 0.1, abs=0.08)  # free corridor: 10 cm bands
    assert report.surface_width_median_m == pytest.approx(1.0, abs=0.08)
    assert len(report.ends) == 2
    for end in report.ends:
        assert end["centreline_radius_m"] == pytest.approx(0.5, abs=0.1)
        assert end["outer_wall_radius_m"] == pytest.approx(1.0, abs=0.12)
    assert report.island["island_length_m"] == pytest.approx(2.0, abs=0.2)
    assert any(s > 1.5 for s in report.straights_m)


def test_inside_polygon():
    sq_x, sq_y = np.array([0.0, 1.0, 1.0, 0.0]), np.array([0.0, 0.0, 1.0, 1.0])
    inside = mt._inside_polygon(np.array([0.5, 1.5, -0.1]), np.array([0.5, 0.5, 0.5]), sq_x, sq_y)
    assert inside.tolist() == [True, False, False]


def test_fit_circle_recovers_a_known_circle():
    a = np.linspace(0.3, 2.0, 40)
    cx, cy, r = mt.fit_circle(np.column_stack([2.0 + 0.7 * np.cos(a), -1.0 + 0.7 * np.sin(a)]))
    assert (cx, cy, r) == pytest.approx((2.0, -1.0, 0.7), abs=1e-6)


def test_curvature_limits_follow_tan_of_the_lock_over_the_wheelbase():
    left, right = mt.curvature_limits(0.3302, 0.504, 0.449, 0.85)
    assert left == pytest.approx(0.85 * math.tan(0.504) / 0.3302)
    assert right == pytest.approx(0.85 * math.tan(0.449) / 0.3302)
    assert left > right  # the left lock is the tighter turn


def test_chord_curvature_of_a_ccw_circle_is_positive_and_the_radius_is_right():
    t = np.linspace(0.0, 2.0 * math.pi, 200, endpoint=False)
    kappa = pipeline._chord_curvature(0.8 * np.cos(t), 0.8 * np.sin(t), 0.15)
    assert kappa == pytest.approx(1.0 / 0.8, rel=0.01)
    assert pipeline._chord_curvature(0.8 * np.cos(-t), 0.8 * np.sin(-t), 0.15) == pytest.approx(
        -1.0 / 0.8, rel=0.01
    )


def test_racing_line_on_a_round_island_respects_the_steering_the_body_and_runs_wide(round_island):
    grid = mt.load_map(round_island)
    cx, cy = mt.trace_centreline(grid, (1.0, -0.9))
    car = pipeline.car_geometry()
    left_k, right_k = mt.curvature_limits(car.wheelbase_m, *pipeline.steering_limits(), 1.0)
    line = mt.racing_line(
        grid,
        cx,
        cy,
        car,
        max_left_curvature_1pm=left_k,
        max_right_curvature_1pm=right_k,
        body_clearance_m=0.05,
    )
    assert line.curvature_violations == 0 and line.clearance_violations == 0
    assert line.min_body_clearance_m >= 0.04
    kappa = pipeline._chord_curvature(line.x_m, line.y_m, 0.15)
    assert kappa.max() <= left_k * 1.1 and -kappa.min() <= right_k * 1.1
    # The line is longer-radius than the centreline (it runs wide into the ends): its tightest
    # turn is no tighter than the centreline's 0.5 m.
    assert 1.0 / kappa.max() > 0.55
    assert np.abs(line.offsets_m).max() > 0.02  # it really moved off the centreline


def test_pipeline_raceline_from_map_writes_a_loadable_csv_and_a_report(tmp_path, round_island):
    out = tmp_path / "line.csv"
    result = pipeline.raceline_from_map(
        round_island,
        out,
        direction="cw",
        seed_xy=(1.0, -0.9),
        v_max_mps=0.5,
        body_clearance_m=0.05,
        margin=1.0,
    )
    assert pipeline.feasible(result.report)
    assert result.report["direction"] == "cw"
    rows = [
        line.split(",")
        for line in out.read_text().splitlines()
        if line and not line.startswith("#")
    ]
    assert tuple(rows[0]) == (
        "s_m",
        "x_m",
        "y_m",
        "heading_rad",
        "curvature_1pm",
        "target_speed_mps",
    )
    data = np.array(rows[1:], dtype=float)
    assert data[:, 5].max() <= 0.5 + 1e-9
    # Clockwise: negative signed area, and right turns have negative curvature.
    x, y = data[:, 1], data[:, 2]
    assert 0.5 * float(np.sum(x * np.roll(y, -1) - np.roll(x, -1) * y)) < 0.0
    assert out.with_suffix(".json").is_file()


def test_curvature_limits_come_from_vehicle_params_steering():
    left, right = pipeline.steering_limits()
    assert left == pytest.approx(0.449) and right == pytest.approx(0.449)  # schema 0.14.0


def test_write_surface_map_keeps_free_cells_and_only_the_wall_faces(tmp_path):
    occupancy = np.full((10, 10), 255.0, dtype=np.float32)
    occupancy[:, :3] = 0.0  # a solid 3-cell wall on the left
    yaml_path = pipeline.write_surface_map(occupancy, (1.0, 2.0), 0.05, tmp_path / "w")
    grid = mt.load_map(yaml_path)
    assert grid.origin_xy == (1.0, 2.0)
    assert grid.occupied[:, 2].all()  # the face touching free floor
    assert not grid.occupied[:, 0].any()  # the inside of the wall is unknown
    assert not grid.occupied[:, 5:].any()
