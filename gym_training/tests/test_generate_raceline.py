# corridor arithmetic, raceline csv round trip, and one real spielberg generation

import math

import numpy as np
import pytest

from f1rl.track import RacelineIndex, raceline_index_from_csv, read_raceline_csv, write_raceline_csv
from f1rl.track.clearance import ClearanceMap
from f1rl.track.generate_raceline import (
    KAPPA_MAX_RADPM,
    RacelineSpec,
    acceleration_limits,
    cap_track_widths,
    normals_fold,
    shrink_track_widths,
    validate_raceline,
)

CIRCLE_RADIUS_M = 12.0
CIRCLE_POINTS = 480


def circle_raceline(speed_mps: float = 5.0) -> dict:
    """the seven raceline columns of a counter-clockwise circle, left open at the lap join."""
    angles = np.linspace(0.0, 2.0 * np.pi, CIRCLE_POINTS, endpoint=False)
    return {
        "s_m": angles * CIRCLE_RADIUS_M,
        "x_m": CIRCLE_RADIUS_M * np.cos(angles),
        "y_m": CIRCLE_RADIUS_M * np.sin(angles),
        "psi_rad": np.arctan2(np.sin(angles + 0.5 * np.pi), np.cos(angles + 0.5 * np.pi)),
        "kappa_radpm": np.full(CIRCLE_POINTS, 1.0 / CIRCLE_RADIUS_M),
        "vx_mps": np.full(CIRCLE_POINTS, speed_mps),
        "ax_mps2": np.zeros(CIRCLE_POINTS),
    }


def straight_track(half_width_m: float = 1.0, points: int = 20) -> np.ndarray:
    angles = np.linspace(0.0, 2.0 * np.pi, points, endpoint=False)
    return np.column_stack(
        (
            CIRCLE_RADIUS_M * np.cos(angles),
            CIRCLE_RADIUS_M * np.sin(angles),
            np.full(points, half_width_m),
            np.full(points, half_width_m),
        )
    )


def corridor_map(half_width_m: float = 1.0, resolution_m: float = 0.05) -> ClearanceMap:
    """a horizontal free corridor of the given half width through an otherwise walled map."""
    height = int(6.0 / resolution_m)
    occupancy = np.zeros((height, int(30.0 / resolution_m)), dtype=np.float32)
    middle = height // 2
    span = int(half_width_m / resolution_m)
    occupancy[middle - span : middle + span + 1, :] = 255.0
    return ClearanceMap(occupancy, resolution_m, (0.0, -3.0))


def test_shrink_pulls_both_edges_in():
    shrunk = shrink_track_widths(straight_track(half_width_m=1.1), 0.305)
    assert shrunk[:, 2] == pytest.approx(0.795)
    assert shrunk[:, 3] == pytest.approx(0.795)


def test_shrink_floors_instead_of_inverting_the_corridor():
    narrow = straight_track(half_width_m=0.2)
    shrunk = shrink_track_widths(narrow, 0.305)
    assert np.all(shrunk[:, 2:4] == 0.0)
    padded = shrink_track_widths(narrow, 0.305, min_half_width_m=0.05)
    assert np.all(padded[:, 2:4] == 0.05)


def test_shrink_leaves_the_line_itself_alone():
    track = straight_track()
    shrunk = shrink_track_widths(track, 0.3)
    assert shrunk[:, :2] == pytest.approx(track[:, :2])
    assert track[:, 2] == pytest.approx(1.0), "the input must not be edited in place"


def test_keep_out_is_the_car_half_width_plus_the_margin():
    spec = RacelineSpec(half_width_m=0.155, margin_m=0.15)
    assert spec.keep_out_m == pytest.approx(0.305)
    # the finished line only has to hold half the margin, the rest absorbs interpolation
    assert spec.required_clearance_m == pytest.approx(0.23)


def test_spec_rejects_a_negative_margin():
    with pytest.raises(ValueError, match="margin_m"):
        RacelineSpec(margin_m=-0.01)


def test_cap_clips_to_the_measured_corridor_but_never_widens():
    track = straight_track(half_width_m=1.0)
    capped = cap_track_widths(track, np.full(len(track), 0.4), np.full(len(track), 5.0))
    assert capped[:, 2] == pytest.approx(0.4)
    assert capped[:, 3] == pytest.approx(1.0)


def test_cap_keeps_a_negative_bound_that_pushes_the_line_over():
    track = straight_track(half_width_m=1.0)
    capped = cap_track_widths(track, np.full(len(track), -0.2), np.full(len(track), 0.9))
    assert capped[:, 2] == pytest.approx(-0.2)


def test_reference_track_needs_four_columns():
    with pytest.raises(ValueError, match="reference track"):
        shrink_track_widths(np.zeros((10, 2)), 0.1)


def test_acceleration_limits_span_the_speed_range_and_drop_off_at_v_switch():
    spec = RacelineSpec(v_max_mps=12.0, a_max_mps2=9.51, v_switch_mps=6.0, grip_usage=0.5, mu=1.0)
    ggv, ax_max_machines = acceleration_limits(spec)
    assert ggv.shape[1] == 3 and ax_max_machines.shape[1] == 2
    assert ggv[0, 0] == 0.0 and ggv[-1, 0] == pytest.approx(12.0)
    # the motor holds a_max to v_switch, then falls as v_switch / v
    assert np.interp(3.0, ggv[:, 0], ggv[:, 1]) == pytest.approx(9.51)
    assert np.interp(12.0, ggv[:, 0], ggv[:, 1]) == pytest.approx(9.51 * 0.5, rel=1e-6)
    assert np.all(ggv[:, 2] == pytest.approx(0.5 * 9.81))


def test_normals_fold_is_clear_on_a_circle_wider_than_the_corridor():
    track = straight_track(half_width_m=1.0, points=90)
    inward = -track[:, :2] / np.linalg.norm(track[:, :2], axis=1)[:, None]
    assert not normals_fold(track, inward).any()


def test_normals_fold_catches_a_corridor_wider_than_the_corner_radius():
    track = straight_track(half_width_m=CIRCLE_RADIUS_M + 1.0, points=90)
    inward = -track[:, :2] / np.linalg.norm(track[:, :2], axis=1)[:, None]
    # the inward normals all meet at the centre, which is inside a corridor this wide
    assert normals_fold(track, inward).all()


def test_lateral_bounds_measure_the_free_corridor():
    clearance = corridor_map(half_width_m=1.0)
    xs, ys = np.array([15.0]), np.array([0.0])
    up = np.array([[0.0, 1.0]])
    upper, lower = clearance.lateral_bounds(xs, ys, up, 3.0, 0.3)
    assert upper[0] == pytest.approx(0.7, abs=0.1)
    assert lower[0] == pytest.approx(-0.7, abs=0.1)


def test_lateral_bounds_pull_an_off_centre_point_back_into_the_clear_band():
    clearance = corridor_map(half_width_m=1.0)
    # 0.85 m off centre leaves only 0.15 m of wall clearance, under the 0.3 m asked for
    upper, lower = clearance.lateral_bounds(
        np.array([15.0]), np.array([0.85]), np.array([[0.0, 1.0]]), 3.0, 0.3
    )
    assert upper[0] < 0.0, "the whole allowed band must sit back towards the centre"
    assert lower[0] < upper[0]


def test_csv_round_trip_preserves_every_column(tmp_path):
    raceline = circle_raceline()
    path = write_raceline_csv(tmp_path / "Circle_raceline_gen.csv", raceline, comment="unit test")
    reread = read_raceline_csv(path)
    assert sorted(reread) == sorted(raceline)
    for name, column in raceline.items():
        assert reread[name][:CIRCLE_POINTS] == pytest.approx(column, abs=0.0, rel=0.0)


def test_written_csv_repeats_the_start_point_like_the_shipped_files(tmp_path):
    raceline = circle_raceline()
    reread = read_raceline_csv(write_raceline_csv(tmp_path / "Circle_raceline_gen.csv", raceline))
    lap_m = 2.0 * math.pi * CIRCLE_RADIUS_M
    assert reread["x_m"].size == CIRCLE_POINTS + 1
    for name in ("x_m", "y_m", "psi_rad", "kappa_radpm", "vx_mps", "ax_mps2"):
        assert reread[name][-1] == pytest.approx(raceline[name][0])
    # the repeat carries the full lap length, so s spans the whole loop rather than stopping short
    assert reread["s_m"][-1] == pytest.approx(lap_m, rel=1e-4)


def test_written_csv_keeps_the_gym_header_layout(tmp_path):
    path = write_raceline_csv(tmp_path / "Circle_raceline_gen.csv", circle_raceline())
    lines = path.read_text().splitlines()
    assert lines[2] == "# s_m; x_m; y_m; psi_rad; kappa_radpm; vx_mps; ax_mps2"
    assert len(lines) == 3 + CIRCLE_POINTS + 1


def test_writer_rejects_ragged_columns(tmp_path):
    raceline = circle_raceline()
    raceline["vx_mps"] = raceline["vx_mps"][:-1]
    with pytest.raises(ValueError, match="same length"):
        write_raceline_csv(tmp_path / "Circle_raceline_gen.csv", raceline)


def test_reader_rejects_a_file_with_the_wrong_columns(tmp_path):
    path = tmp_path / "Bad_raceline_gen.csv"
    path.write_text("# id\n# when\n# s_m; x_m; y_m\n0.0; 1.0; 2.0\n")
    with pytest.raises(ValueError, match="expected columns"):
        read_raceline_csv(path)


def test_loader_builds_a_closed_index_the_planner_can_drive(tmp_path):
    path = write_raceline_csv(tmp_path / "Circle_raceline_gen.csv", circle_raceline(speed_mps=6.5))
    line = raceline_index_from_csv(path)
    assert isinstance(line, RacelineIndex)
    assert line.n == CIRCLE_POINTS
    assert line.length == pytest.approx(2.0 * math.pi * CIRCLE_RADIUS_M, rel=1e-3)
    assert line.has_speed_profile and line.speed_at(0.0) == pytest.approx(6.5)


def test_exported_columns_wrap_around_the_lap(tmp_path):
    raceline = circle_raceline()
    reread = read_raceline_csv(write_raceline_csv(tmp_path / "Circle_raceline_gen.csv", raceline))
    s = reread["s_m"]
    step_m = 2.0 * math.pi * CIRCLE_RADIUS_M / CIRCLE_POINTS
    assert s[0] == 0.0
    assert np.all(np.diff(s) > 0.0)
    # the closing step is the chord back to the start, so it matches the rest only to the sagitta
    assert np.diff(s) == pytest.approx(step_m, rel=1e-4)
    # heading is wrapped to [-pi, pi] per point but turns exactly one full lap end to end
    psi = reread["psi_rad"]
    assert np.all(np.abs(psi) <= math.pi)
    turned = np.diff(psi)
    assert np.sum((turned + math.pi) % (2.0 * math.pi) - math.pi) == pytest.approx(2.0 * math.pi)
    assert reread["kappa_radpm"][-1] == pytest.approx(reread["kappa_radpm"][0])


def test_validate_reports_the_measured_line(tmp_path):
    spec = RacelineSpec()
    clearance = corridor_map(half_width_m=1.0)
    raceline = circle_raceline(speed_mps=4.0)
    # park the circle outside the corridor map so every point reads as walled off
    raceline["x_m"] = raceline["x_m"] + 100.0
    report = validate_raceline(raceline, clearance, spec, "Circle")
    assert report.min_clearance_m == 0.0 and not report.clears_walls
    assert report.max_kappa_radpm == pytest.approx(1.0 / CIRCLE_RADIUS_M)
    assert report.steerable
    assert report.length_m == pytest.approx(2.0 * math.pi * CIRCLE_RADIUS_M, rel=1e-3)
    assert report.lap_time_sec == pytest.approx(report.length_m / 4.0, rel=1e-6)
    assert report.v_min_mps == 4.0 and report.v_max_mps == 4.0


def test_steering_limit_matches_the_vehicle():
    assert KAPPA_MAX_RADPM == pytest.approx(math.tan(0.4189) / 0.3302)


@pytest.mark.slow
def test_generated_spielberg_line_clears_the_walls_and_the_steering_limit():
    from f1tenth_gym.envs.track import Track

    from f1rl.track.generate_raceline import generate_raceline
    from f1rl.track.raceline_io import centerline_path, read_centerline_csv

    spec = RacelineSpec()
    track = Track.from_track_name("Spielberg")
    clearance = ClearanceMap.from_track(track)
    reftrack = read_centerline_csv(centerline_path(track))
    line, report = generate_raceline(reftrack, clearance, spec, "Spielberg")
    assert report == validate_raceline(line, clearance, spec, "Spielberg")
    assert report.clears_walls, report.summary()
    assert report.steerable, report.summary()
    assert report.length_m == pytest.approx(track.centerline.length, rel=0.05)
