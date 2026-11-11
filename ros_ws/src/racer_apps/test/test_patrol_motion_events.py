"""L1: patrol geometry, command shaping and the event log."""

from __future__ import annotations

import csv
import json
import math
from itertools import pairwise
from pathlib import Path

import pytest
from racer_apps.spiderbot.events import EventLog
from racer_apps.spiderbot.motion import (
    MotionLimits,
    RateLimiter,
    clamp,
    face_command,
    limits_from_vehicle,
)
from racer_apps.spiderbot.patrol import (
    RACELINE_HEADER,
    PatrolError,
    build_raceline,
    load_patrol,
    max_curvature_from_vehicle,
    parse_patrol,
    write_raceline_csv,
)

SQUARE = [[0.0, 0.0], [3.0, 0.0], [3.0, 3.0], [0.0, 3.0]]


def doc(**over):
    base = {"frame": "map", "loop": True, "waypoints": SQUARE}
    base.update(over)
    return base


# -- patrol parsing -------------------------------------------------------------------------


def test_parse_valid_patrol():
    p = parse_patrol(doc(name="hall", speed_mps=0.6))
    assert p.name == "hall" and p.speed_mps == 0.6 and len(p.waypoints) == 4


@pytest.mark.parametrize(
    "bad",
    [
        doc(frame="odom"),
        doc(loop=False),
        doc(waypoints=SQUARE[:2]),
        doc(waypoints="nope"),
        doc(waypoints=[[0, 0], [1, 1], [1]]),
        doc(waypoints=[[0, 0], [1, 1], [math.nan, 2]]),
        doc(waypoints=[[0, 0], [1, 1], [True, 2]]),
        doc(speed_mps=-1.0),
        doc(speed_mps="fast"),
    ],
)
def test_parse_refuses_bad_patrols(bad):
    with pytest.raises(PatrolError):
        parse_patrol(bad)


def test_parse_refuses_a_non_mapping():
    with pytest.raises(PatrolError):
        parse_patrol([1, 2, 3])


def test_load_patrol_from_yaml_and_its_refusals(tmp_path):
    good = tmp_path / "p.yaml"
    good.write_text("frame: map\nloop: true\nwaypoints: [[0,0],[3,0],[3,3],[0,3]]\n")
    assert len(load_patrol(good).waypoints) == 4
    with pytest.raises(PatrolError, match="not found"):
        load_patrol(tmp_path / "missing.yaml")
    bad = tmp_path / "bad.yaml"
    bad.write_text("frame: [unclosed\n")
    with pytest.raises(PatrolError, match="valid YAML"):
        load_patrol(bad)


# -- raceline -------------------------------------------------------------------------------


def test_raceline_is_a_closed_smooth_loop_through_the_waypoints():
    pts = build_raceline([tuple(w) for w in SQUARE], 0.5, max_curvature_1pm=5.0)
    assert len(pts) > 100
    assert all(p.target_speed_mps == 0.5 for p in pts)
    assert pts[0].s_m == 0.0 and all(b.s_m > a.s_m for a, b in pairwise(pts))
    # Every waypoint is passed within a few cm.
    for wx, wy in SQUARE:
        assert (
            min(math.hypot(p.x_m - wx, p.y_m - wy) for p in pts) < 0.06
        )  # half the 0.1 m resampling step
    # Closed: the last point is one step from the first, not a jump.
    step = math.hypot(pts[-1].x_m - pts[0].x_m, pts[-1].y_m - pts[0].y_m)
    assert step < 0.15
    # A counter-clockwise loop turns left (positive curvature) on average.
    assert sum(p.curvature_1pm for p in pts) > 0


def test_raceline_headings_follow_the_path():
    pts = build_raceline([tuple(w) for w in SQUARE], 0.5, 5.0)
    for a, b in pairwise(pts):
        expected = math.atan2(b.y_m - a.y_m, b.x_m - a.x_m)
        diff = math.atan2(math.sin(a.heading_rad - expected), math.cos(a.heading_rad - expected))
        assert abs(diff) < 0.35  # central difference vs forward difference, even at the corners


def test_raceline_refuses_a_corner_tighter_than_the_car_can_turn():
    tight = [(0.0, 0.0), (1.0, 0.0), (1.0, 0.25), (0.0, 0.25)]
    with pytest.raises(PatrolError, match="tighter than the car can"):
        build_raceline(tight, 0.5, max_curvature_1pm=1.5, min_spacing_m=0.2)


def test_raceline_refuses_coincident_waypoints_bad_speed_and_few_points():
    with pytest.raises(PatrolError, match="apart"):
        build_raceline([(0, 0), (0.05, 0), (3, 3), (0, 3)], 0.5, 5.0)
    with pytest.raises(PatrolError, match="speed"):
        build_raceline([tuple(w) for w in SQUARE], 0.0, 5.0)
    with pytest.raises(PatrolError, match="max_curvature"):
        build_raceline([tuple(w) for w in SQUARE], 0.5, 0.0)
    with pytest.raises(PatrolError, match="at least 3"):
        build_raceline([(0, 0), (1, 1)], 0.5, 5.0)


def test_max_curvature_from_vehicle_numbers():
    # tan(0.4) / 0.33 with a 0.75 margin; the numbers here are arguments, not the car's.
    assert max_curvature_from_vehicle(0.33, 0.4) == pytest.approx(0.75 * math.tan(0.4) / 0.33)
    for args in [(0.0, 0.4), (0.33, 0.0), (0.33, 2.0), (0.33, 0.4, 1.5)]:
        with pytest.raises(PatrolError):
            max_curvature_from_vehicle(*args)


def test_csv_has_the_tools_raceline_format(tmp_path):
    pts = build_raceline([tuple(w) for w in SQUARE], 0.5, 5.0)
    out = tmp_path / "r.csv"
    write_raceline_csv(out, pts, "square")
    lines = [line for line in out.read_text().splitlines() if not line.startswith("#")]
    rows = list(csv.reader(lines))
    assert tuple(rows[0]) == RACELINE_HEADER
    assert len(rows) == len(pts) + 1 and all(len(r) == 6 for r in rows)
    assert float(rows[1][5]) == 0.5


def test_the_shipped_example_patrol_is_a_loop_the_car_can_drive():
    example = Path(__file__).resolve().parents[1] / "config" / "patrol_example.yaml"
    patrol = load_patrol(example)
    curvature = max_curvature_from_vehicle(0.33, 0.4)  # numbers here are arguments, not the car's
    points = build_raceline(patrol.waypoints, patrol.speed_mps, curvature)
    assert len(points) > 100 and max(abs(p.curvature_1pm) for p in points) < curvature


# -- motion ---------------------------------------------------------------------------------

LIM = MotionLimits(
    max_steering_rad=0.4, max_speed_mps=1.0, max_speed_step_mps=0.1, max_steering_step_rad=0.05
)


def test_limits_from_vehicle_takes_the_smaller_steering_side_and_a_margin():
    lim = limits_from_vehicle(0.42, -0.40, 3.0, 9.0, 2.9, control_rate_hz=50.0, margin_fraction=0.5)
    assert lim.max_steering_rad == 0.40 and lim.max_speed_mps == 2.9
    assert lim.max_speed_step_mps == pytest.approx(0.5 * 9.0 / 50.0)
    assert lim.max_steering_step_rad == pytest.approx(0.5 * 3.0 / 50.0)
    for kw in ({"margin_fraction": 0.0}, {"margin_fraction": 1.5}, {"control_rate_hz": 0.0}):
        args = {"control_rate_hz": 50.0, "margin_fraction": 0.5, **kw}
        with pytest.raises(ValueError):
            limits_from_vehicle(0.4, -0.4, 3.0, 9.0, 2.9, **args)


def test_limits_reject_non_positive_values():
    with pytest.raises(ValueError):
        MotionLimits(0.0, 1.0, 0.1, 0.05)
    with pytest.raises(ValueError):
        MotionLimits(0.4, 1.0, math.nan, 0.05)


def test_rate_limiter_ramps_both_ways_and_never_overshoots():
    rl = RateLimiter(LIM)
    speeds = [rl.step(0.5, 0.0)[0] for _ in range(8)]
    assert speeds[0] == pytest.approx(0.1) and speeds[-1] == pytest.approx(0.5)
    assert all(b - a <= 0.1 + 1e-12 for a, b in pairwise(speeds))
    down = [rl.step(0.0, 0.0)[0] for _ in range(6)]
    assert down[0] == pytest.approx(0.4) and down[-1] == 0.0


def test_rate_limiter_clamps_targets_and_steering():
    rl = RateLimiter(LIM)
    for _ in range(100):
        speed, steer = rl.step(99.0, -99.0)
    assert speed == 1.0 and steer == -0.4


def test_rate_limiter_treats_non_finite_as_stop():
    rl = RateLimiter(LIM)
    rl.reset(0.5, 0.2)
    speed, steer = rl.step(math.nan, 0.1)
    assert speed == pytest.approx(0.4) and steer == pytest.approx(0.15)
    speed, steer = rl.step(0.3, math.inf)
    assert speed < 0.4


def test_face_command_turns_towards_the_person_with_the_right_sign():
    speed, steer = face_command(0.2, 1.5, 0.3, LIM)  # person on the left -> steer left (+)
    assert speed == 0.3 and steer == pytest.approx(0.3)
    assert face_command(-0.2, 1.5, 0.3, LIM)[1] == pytest.approx(-0.3)
    assert face_command(1.0, 1.5, 0.3, LIM)[1] == 0.4  # saturates
    assert face_command(math.nan, 1.5, 0.3, LIM) == (0.0, 0.0)
    assert face_command(0.1, 1.5, 9.0, LIM)[0] == 1.0  # never above the speed limit
    assert face_command(0.1, 1.5, -1.0, LIM)[0] == 0.0  # never reverses


def test_clamp():
    assert clamp(5, 0, 1) == 1 and clamp(-5, 0, 1) == 0 and clamp(0.5, 0, 1) == 0.5


# -- events ---------------------------------------------------------------------------------


def test_event_log_appends_json_lines_and_cleans_nan(tmp_path):
    log = EventLog(tmp_path / "out", run_stamp="20261111T000000Z")
    a, b = log.new_event_id(), log.new_event_id()
    assert (a, b) == ("20261111T000000Z-0001", "20261111T000000Z-0002")
    log.append("alert", 12.5, event_id=a, range_m=math.nan, pose=(1.0, 2.0, 0.5))
    log.append("resume", 20.0)
    records = log.read_all()
    assert [r["kind"] for r in records] == ["alert", "resume"]
    assert records[0]["range_m"] is None and records[0]["pose"] == [1.0, 2.0, 0.5]
    json.loads(log.path.read_text().splitlines()[0])  # strict JSON, no NaN token


def test_snapshots_are_written_or_reported_missing(tmp_path):
    log = EventLog(tmp_path)
    path = log.write_snapshot("evt", b"\xff\xd8jpeg")
    assert path.endswith("snapshots/evt.jpg")
    assert Path(path).read_bytes() == b"\xff\xd8jpeg"
    assert log.write_snapshot("evt2", None) == ""
    assert log.write_snapshot("evt3", b"") == ""
    assert log.write_snapshot("no/such/dir", b"x") == ""
    assert log.read_all() == []
