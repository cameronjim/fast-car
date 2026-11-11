"""L1/L4 tests for tools/safety_replay and tools/replay_safety.py (the safety gate bag replay).

Synthetic scenes only: a car driving straight at the end wall of a room, braking, and coming to
rest, ray-cast into full-circle scans mounted like the car's LiDAR (0.285 m ahead of the rear
axle, yaw pi). The scan matching must recover the known motion, the analysis the known stopping
travel, end clearance and gain, and the pipeline must run end to end with a fake CLI. The test
against the real C++ binary runs only when it can be found (SAFETY_REPLAY_CLI, a colcon install
tree, or PATH); the CLI's own ctest covers it in CI.
"""

from __future__ import annotations

import dataclasses
import json
import math
import os
import stat
import sys
from pathlib import Path

import numpy as np
import pytest
import replay_safety
from follower_replay.bag import (
    BagData,
    DriveRecord,
    EventRecord,
    OdomRecord,
    ScanRecord,
    read_bag,
)
from safety_replay import analysis, calibration, motion, stream

MOUNT = motion.Mount(0.285, 0.0, math.pi)
BODY = motion.Body(0.3302 + 0.13, 0.12, 0.155)
WALL_X = 2.0  # the room's end wall, world frame
ROOM = (-2.0, WALL_X, -1.0, 1.0)  # x_min, x_max, y_min, y_max
POSTS = ((0.8, 0.6), (1.5, -0.7), (-0.5, 0.5))  # small square posts for ICP's x constraint
N_RAYS = 720
T0 = 1000.0

SPEED = 0.8
T_BRAKE = T0 + 1.55
LATENCY = 0.1
DECEL = 2.0


def car_x(t: float) -> float:
    """Rear axle x (world): 0.8 m/s, then 0.1 s after the brake a 2 m/s^2 stop."""
    rel = t - T0
    t_slow = T_BRAKE - T0 + LATENCY
    if rel <= t_slow:
        return SPEED * rel
    dt = min(rel - t_slow, SPEED / DECEL)
    return SPEED * t_slow + SPEED * dt - 0.5 * DECEL * dt * dt


def rest_x() -> float:
    return car_x(T0 + 10.0)


def segments() -> list[tuple[float, float, float, float]]:
    x0, x1, y0, y1 = ROOM
    segs = [(x0, y0, x1, y0), (x1, y0, x1, y1), (x1, y1, x0, y1), (x0, y1, x0, y0)]
    for px, py in POSTS:
        h = 0.05
        segs += [
            (px - h, py - h, px + h, py - h),
            (px + h, py - h, px + h, py + h),
            (px + h, py + h, px - h, py + h),
            (px - h, py + h, px - h, py - h),
        ]
    return segs


def cast_scan(x: float, stamp: float) -> ScanRecord:
    """A scan from the head at rear axle (x, 0), heading 0, laser yaw pi."""
    hx, hy = x + MOUNT.x_m, MOUNT.y_m
    inc = 2.0 * math.pi / N_RAYS
    ranges = []
    for i in range(N_RAYS):
        bearing = -math.pi + i * inc + MOUNT.yaw_rad
        dx, dy = math.cos(bearing), math.sin(bearing)
        best = math.inf
        for sx0, sy0, sx1, sy1 in segments():
            ex, ey = sx1 - sx0, sy1 - sy0
            denom = dx * ey - dy * ex
            if abs(denom) < 1e-12:
                continue
            t = ((sx0 - hx) * ey - (sy0 - hy) * ex) / denom
            u = ((sx0 - hx) * dy - (sy0 - hy) * dx) / denom
            if t > 0.0 and 0.0 <= u <= 1.0:
                best = min(best, t)
        ranges.append(best)
    return ScanRecord(stamp, -math.pi, inc, 0.05, 12.0, tuple(ranges), stamp)


def scene() -> BagData:
    data = BagData()
    for k in range(40):  # 4 s of scans at 10 Hz
        t = T0 + 0.1 * k
        data.scans.append(cast_scan(car_x(t), t))
    for k in range(200):
        t = T0 + 0.02 * k + 0.001
        data.drive_raw.append(DriveRecord(t, 0.0, SPEED, t))
        data.drive.append(DriveRecord(t, 0.0, SPEED if t < T_BRAKE else 0.0, t))
        data.odom.append(OdomRecord(t, (car_x(t + 0.01) - car_x(t - 0.01)) / 0.02, t))
    data.events = [
        EventRecord(T_BRAKE, 2, 0, 0.0, "ttc", "time-to-collision 0.2s <= brake 0.3s", T_BRAKE),
        EventRecord(T0 + 3.5, 2, 1, 1.95, "ttc", "ttc brake released", T0 + 3.5),
        EventRecord(T0 + 0.5, 2, 0, 0.0, "watchdog", "stale", T0 + 0.5),
    ]
    return data


@pytest.fixture(scope="module")
def bag() -> BagData:
    return scene()


# -- motion -----------------------------------------------------------------------------------


def test_scan_points_apply_the_mount_and_yaw():
    # One return straight ahead of the car at 1 m from the head: laser bearing pi (yaw pi).
    scan = ScanRecord(0.0, math.pi, 0.01, 0.05, 12.0, (1.0, math.nan, 0.0, 20.0, 0.01), 0.0)
    pts = motion.scan_points(scan, MOUNT)
    assert pts.shape == (1, 2)
    assert pts[0] == pytest.approx([1.285, 0.0], abs=1e-9)
    assert len(motion.scan_points(scan, MOUNT, min_range_m=1.5)) == 0


def test_box_clearance():
    pts = np.array([[BODY.front_x_m + 0.1, 0.0], [0.0, BODY.half_width_m + 0.3]])
    assert motion.box_clearance(pts, BODY) == pytest.approx(0.1)
    assert motion.box_clearance(np.array([[0.1, 0.0]]), BODY) == 0.0  # inside
    corner = np.array([[BODY.front_x_m + 0.03, BODY.half_width_m + 0.04]])
    assert motion.box_clearance(corner, BODY) == pytest.approx(0.05)
    assert motion.box_clearance(np.zeros((0, 2)), BODY) == math.inf


def test_icp_recovers_a_rigid_motion():
    rng = np.random.default_rng(3)
    pts = rng.uniform(-2.0, 2.0, size=(300, 2))
    angle = 0.05
    rot = np.array([[math.cos(angle), -math.sin(angle)], [math.sin(angle), math.cos(angle)]])
    target = pts @ rot.T + np.array([0.06, -0.03])
    r, t, rms = motion.icp(pts, target)
    assert t == pytest.approx([0.06, -0.03], abs=1e-6)
    assert math.atan2(r[1, 0], r[0, 0]) == pytest.approx(angle, abs=1e-6)
    assert rms < 1e-6
    with pytest.raises(ValueError):
        motion.icp(pts[:5], target)


def test_car_motion_between_ray_cast_scans():
    a = cast_scan(0.4, 0.0)
    b = cast_scan(0.55, 0.1)
    x, y, heading, rms = motion.car_motion(a, b, MOUNT)
    assert x == pytest.approx(0.15, abs=0.005)
    assert y == pytest.approx(0.0, abs=0.005)
    assert heading == pytest.approx(0.0, abs=0.005)
    assert rms < 0.01


# -- analysis ---------------------------------------------------------------------------------


def trip(t: float, distance: float, speed: float = SPEED, cause: str = "ttc") -> dict:
    return {
        "type": "trip",
        "t": t,
        "gate": "ttc",
        "cause": cause,
        "distance_m": distance,
        "speed_mps": speed,
        "steering_rad": 0.0,
    }


def cycle(t: float, distance: float) -> dict:
    return {"type": "cycle", "t": t, "distance_m": distance}


def test_recorded_brakes_and_engagements(bag):
    brakes, engagements = analysis.recorded_brakes(bag)
    assert brakes == [(T_BRAKE, "ttc")]
    assert engagements == [(T_BRAKE, T0 + 3.5)]
    held = BagData(
        events=[
            EventRecord(1.0, 2, 0, 0.0, "ttc", "ttc brake latch held (...)", 1.0),
            EventRecord(2.0, 2, 0, 0.0, "ttc", "forward clearance 0.04 m < 0.05 m", 2.0),
        ]
    )
    brakes, engagements = analysis.recorded_brakes(held)
    assert brakes == [(2.0, "floor")]
    assert engagements == [(2.0, math.inf)]  # the second engage replaced the first, never closed


def test_measured_travel_end_clearance_and_speed(bag):
    report = analysis.analyse(bag, [trip(T_BRAKE, 0.3)], [], MOUNT, BODY)
    row = report.rows[0]
    # From the scan the gate judged (t 1.5, car at 1.2 m) to rest.
    assert row.travel_m == pytest.approx(rest_x() - car_x(T0 + 1.5), abs=0.01)
    assert row.end_clearance_m == pytest.approx(WALL_X - rest_x() - BODY.front_x_m, abs=0.01)
    assert row.approach_speed_mps == pytest.approx(SPEED, abs=0.03)
    assert row.before is not None
    assert row.after is None and row.basis == "no trip"


def test_an_earlier_trip_predicts_end_plus_gain(bag):
    before = [trip(T_BRAKE, 0.3)]
    after = [trip(T_BRAKE - 0.2, 0.46), cycle(T_BRAKE, 0.3)]
    row = analysis.analyse(bag, before, after, MOUNT, BODY).rows[0]
    assert row.lead_s == pytest.approx(0.2)
    # The scans the two trips judged: t 1.3 and 1.5, 0.16 m apart.
    assert row.gain_m == pytest.approx(0.16, abs=0.01)
    assert row.basis == "earlier"
    assert row.predicted_m == pytest.approx(row.end_clearance_m + row.gain_m)


def test_a_later_trip_subtracts_the_extra_distance(bag):
    after = [cycle(T_BRAKE, 0.30), trip(T_BRAKE + 0.1, 0.22)]
    row = analysis.analyse(bag, [], after, MOUNT, BODY).rows[0]
    assert row.basis == "later"
    assert row.predicted_m == pytest.approx(row.end_clearance_m - 0.08)


def test_no_trip_uses_the_rules_trip_distance(bag):
    after = [cycle(T_BRAKE, 0.30)]
    rule = {"ttc_brake_s": 0.3, "min_forward_clearance_m": 0.05}
    row = analysis.analyse(
        bag, [], after, MOUNT, BODY, trip_distance_m=analysis.trip_distance_for(rule)
    ).rows[0]
    assert row.basis == "no trip"
    assert row.predicted_m == pytest.approx(row.end_clearance_m - (0.30 - 0.24))
    # Nothing in the path of the rule under test at the brake (JSON null): it never brakes.
    row = analysis.analyse(bag, [], [cycle(T_BRAKE, None)], MOUNT, BODY).rows[0]
    assert row.basis == "never" and math.isnan(row.predicted_m)


def test_trip_distance_for():
    f = analysis.trip_distance_for({"ttc_brake_s": 0.34, "min_forward_clearance_m": 0.05})
    assert f(1.0) == pytest.approx(0.34)
    assert f(0.1) == pytest.approx(0.05)
    assert math.isnan(analysis.trip_distance_for({"ttc_brake_s": -1})(1.0))


def test_an_earlier_unreleased_trip_covers_the_brake(bag):
    # The rule under test tripped 1.2 s before the recorded brake and never released: the car
    # would already have been stopped there.
    after = [trip(T_BRAKE - 1.2, 0.9)]
    row = analysis.analyse(bag, [], after, MOUNT, BODY).rows[0]
    assert row.after is not None and row.lead_s == pytest.approx(1.2)
    assert row.basis == "earlier"
    # ... unless it released in between: then that trip is a separate (extra) one.
    released = after + [
        {
            "type": "event",
            "t": T_BRAKE - 1.0,
            "source": "ttc",
            "severity": "brake",
            "phase": "release",
        }
    ]
    report = analysis.analyse(bag, [], released, MOUNT, BODY)
    assert report.rows[0].after is None
    assert len(report.extra_trips) == 1 and report.extra_trips[0]["drove_freely"] is True


def test_contact_correction_and_at_rest():
    row = analysis.BrakeRow(
        0.0, "ttc", 0.8, 0.0, end_clearance_m=0.0, travel_m=0.10, approach_speed_mps=0.5
    )
    row.after = trip(-0.1, 0.3)
    row.lead_s = 0.1
    row.gain_m = 0.05
    analysis.predict(row, 0.2, analysis.trip_distance_for({}))
    # Contact: the wall cut the travel; the 90th percentile says 0.2 s x 0.8 = 0.16 m, 0.06 more.
    assert row.predicted_m == pytest.approx(0.0 - 0.06 + 0.05)
    still = analysis.BrakeRow(
        0.0, "ttc", 0.8, 0.0, end_clearance_m=0.05, travel_m=0.0, approach_speed_mps=0.02
    )
    analysis.predict(still, 0.2, analysis.trip_distance_for({}))
    assert still.basis == "at rest" and math.isnan(still.predicted_m)
    blind = analysis.BrakeRow(0.0, "ttc", 0.8, 0.0)
    analysis.predict(blind, 0.2, analysis.trip_distance_for({}))
    assert blind.basis == "no end clearance"


def test_summary_table_and_watchdog_counts(bag):
    before = [trip(T_BRAKE, 0.3), {"type": "event", "source": "watchdog", "phase": "engage"}]
    after = [trip(T_BRAKE - 0.2, 0.46), cycle(T_BRAKE, 0.3)]
    report = analysis.analyse(bag, before, after, MOUNT, BODY)
    s = analysis.summary(report)
    assert s["recorded_brakes"] == 1 and s["before_matched"] == 1 and s["replayable"] == 1
    assert s["watchdog"] == (1, 1, 0)
    assert s["predicted_below"] == 0
    text = analysis.summary_text(s)
    assert "predicted end clearance" in text and "watchdog engages: recorded 1" in text
    table = analysis.table(report.rows, T0)
    assert table.count("\n") == 2 and "| 1.55 | ttc |" in table


# -- stream and CLI plumbing ------------------------------------------------------------------


def test_stream_lines_are_in_arrival_order(bag):
    lines = stream.stream_lines(bag)
    kinds = [line[0] for line in lines]
    assert kinds.count("S") == 40 and kinds.count("D") == 200 and kinds.count("O") == 200
    stamps = [float(line.split()[1]) for line in lines]
    assert stamps == sorted(stamps)
    first_scan = next(line for line in lines if line.startswith("S")).split()
    assert first_scan[4:7] == ["0.05", "12", str(N_RAYS)]
    assert "inf" in first_scan or all(math.isfinite(float(w)) for w in first_scan[7:])


def test_records_without_log_time_fall_back_to_the_stamp():
    data = BagData(drive_raw=[DriveRecord(5.0, 0.1, 0.2)])
    assert stream.stream_lines(data) == ["D 5.000000 0.1 0.2"]


def test_set_args_and_parse_sets():
    assert stream.set_args({"b": True, "a": 0.25}) == ["--set", "a=0.25", "--set", "b=true"]
    assert replay_safety.parse_sets(["ttc_brake_s=0.3", "distance_from_body=false"]) == {
        "ttc_brake_s": 0.3,
        "distance_from_body": False,
    }
    with pytest.raises(stream.ReplayError):
        replay_safety.parse_sets(["nope"])
    with pytest.raises(stream.ReplayError):
        replay_safety.parse_sets(["x=abc"])


def test_find_cli(tmp_path, monkeypatch):
    fake = tmp_path / "safety_replay_cli"
    fake.write_text("")
    assert stream.find_cli(str(fake)) == fake
    with pytest.raises(stream.ReplayError):
        stream.find_cli(str(tmp_path / "missing"))
    monkeypatch.setenv(stream.CLI_ENV, str(fake))
    assert stream.find_cli() == fake
    monkeypatch.setenv(stream.CLI_ENV, "")
    monkeypatch.setenv("PATH", str(tmp_path))
    fake.chmod(fake.stat().st_mode | stat.S_IEXEC)
    assert stream.find_cli() == fake


FAKE_CLI = f"""#!{sys.executable}
import json, sys
import os
args = sys.argv[1:]
if os.environ.get("FAKE_CLI_LOG") and "--describe" not in args:
    with open(os.environ["FAKE_CLI_LOG"], "a") as log:
        stream = args[args.index("--stream") + 1]
        first_d = next(l for l in open(stream) if l.startswith("D"))
        log.write(json.dumps({{"args": args, "first_d": first_d.split()}}) + "\\n")
if "--describe" in args:
    print(json.dumps({{"lidar_mount_x_m": 0.285, "lidar_mount_y_m": 0.0, "laser_yaw_rad": 3.141593,
        "wheelbase_m": 0.3302, "front_overhang_m": 0.13, "rear_overhang_m": 0.12, "width_m": 0.31,
        "ttc_brake_s": 0.34, "min_forward_clearance_m": 0.05, "max_steering_rad": 0.449}}))
    sys.exit(0)
if "bad=1" in args:
    print("nope", file=sys.stderr)
    sys.exit(2)
legacy = "distance_from_body=false" in args
t = {T_BRAKE!r} if legacy else {T_BRAKE - 0.2!r}
print(json.dumps({{"type": "trip", "t": t, "gate": "ttc", "cause": "ttc", "distance_m": 0.3,
    "speed_mps": 0.8, "steering_rad": 0.0}}))
"""


ODOM_DEF = """std_msgs/Header header
string child_frame_id
geometry_msgs/PoseWithCovariance pose
geometry_msgs/TwistWithCovariance twist
================================================================================
MSG: std_msgs/Header
builtin_interfaces/Time stamp
string frame_id
================================================================================
MSG: builtin_interfaces/Time
int32 sec
uint32 nanosec
================================================================================
MSG: geometry_msgs/PoseWithCovariance
geometry_msgs/Pose pose
float64[36] covariance
================================================================================
MSG: geometry_msgs/Pose
geometry_msgs/Point position
geometry_msgs/Quaternion orientation
================================================================================
MSG: geometry_msgs/Point
float64 x
float64 y
float64 z
================================================================================
MSG: geometry_msgs/Quaternion
float64 x
float64 y
float64 z
float64 w
================================================================================
MSG: geometry_msgs/TwistWithCovariance
geometry_msgs/Twist twist
float64[36] covariance
================================================================================
MSG: geometry_msgs/Twist
geometry_msgs/Vector3 linear
geometry_msgs/Vector3 angular
================================================================================
MSG: geometry_msgs/Vector3
float64 x
float64 y
float64 z
"""


def write_scene_bag(path: Path, data: BagData) -> None:
    """The scene as an mcap with the real message definitions (mcap-ros2-support)."""
    from mcap_ros2.writer import Writer
    from test_replay_follower import ACKERMANN_DEF, EVENT_DEF, LASER_SCAN_DEF, _time

    odom_def = ODOM_DEF
    vec = {"x": 0.0, "y": 0.0, "z": 0.0}
    with path.open("wb") as out:
        writer = Writer(out)
        scan_schema = writer.register_msgdef("sensor_msgs/msg/LaserScan", LASER_SCAN_DEF)
        drive_schema = writer.register_msgdef(
            "ackermann_msgs/msg/AckermannDriveStamped", ACKERMANN_DEF
        )
        event_schema = writer.register_msgdef("racer_msgs/msg/SafetyEvent", EVENT_DEF)
        odom_schema = writer.register_msgdef("nav_msgs/msg/Odometry", odom_def)
        for s in data.scans:
            writer.write_message(
                "/scan",
                scan_schema,
                {
                    "header": {"stamp": _time(s.stamp_s - 0.1), "frame_id": "laser"},
                    "angle_min": s.angle_min,
                    "angle_max": s.angle_min + s.angle_increment * (len(s.ranges) - 1),
                    "angle_increment": s.angle_increment,
                    "time_increment": 0.0,
                    "scan_time": 0.1,
                    "range_min": s.range_min,
                    "range_max": s.range_max,
                    "ranges": list(s.ranges),
                    "intensities": [],
                },
                log_time=round(s.log_time_s * 1e9),
            )
        for topic, records in (("/drive_raw", data.drive_raw), ("/drive", data.drive)):
            for d in records:
                writer.write_message(
                    topic,
                    drive_schema,
                    {
                        "header": {"stamp": _time(d.stamp_s), "frame_id": "base_link"},
                        "drive": {
                            "steering_angle": d.steering_rad,
                            "steering_angle_velocity": 0.0,
                            "speed": d.speed_mps,
                            "acceleration": 0.0,
                            "jerk": 0.0,
                        },
                    },
                    log_time=round(d.log_time_s * 1e9),
                )
        for o in data.odom:
            writer.write_message(
                "/odom/wheel",
                odom_schema,
                {
                    "header": {"stamp": _time(o.stamp_s), "frame_id": "odom"},
                    "child_frame_id": "base_link",
                    "pose": {
                        "pose": {"position": vec, "orientation": {**vec, "w": 1.0}},
                        "covariance": [0.0] * 36,
                    },
                    "twist": {
                        "twist": {"linear": {**vec, "x": o.speed_mps}, "angular": vec},
                        "covariance": [0.0] * 36,
                    },
                },
                log_time=round(o.log_time_s * 1e9),
            )
        for e in data.events:
            writer.write_message(
                "/safety/events",
                event_schema,
                {
                    "stamp": _time(e.stamp_s),
                    "severity": e.severity,
                    "phase": e.phase,
                    "duration_s": e.duration_s,
                    "source": e.source,
                    "detail": e.detail,
                },
                log_time=round(e.log_time_s * 1e9),
            )
        writer.finish()


@pytest.fixture(scope="module")
def scene_bag(tmp_path_factory, bag) -> Path:
    path = tmp_path_factory.mktemp("safety") / "scene.mcap"
    write_scene_bag(path, bag)
    return path


def test_bag_reader_keeps_log_times_and_odometry(scene_bag, bag):
    data = read_bag(scene_bag)
    assert len(data.scans) == 40 and len(data.odom) == 200
    assert data.scans[0].log_time_s == pytest.approx(T0)
    assert data.scans[0].stamp_s == pytest.approx(T0 - 0.1)  # the header stamp, a sweep earlier
    assert data.odom[10].speed_mps == pytest.approx(bag.odom[10].speed_mps)


def test_command_line_end_to_end_with_a_fake_cli(scene_bag, tmp_path, capsys):
    cli = tmp_path / "safety_replay_cli"
    cli.write_text(FAKE_CLI)
    cli.chmod(cli.stat().st_mode | stat.S_IEXEC)
    status = replay_safety.main(
        [str(scene_bag), "--cli", str(cli), "--table", "--recorded-with", "current"]
    )
    out = capsys.readouterr().out
    assert status == 0, out
    assert "before-rule replay reproduces 1 of them" in out
    assert "| 1.55 | ttc |" in out
    base = [str(scene_bag), "--recorded-with", "current"]
    assert replay_safety.main([*base, "--cli", str(cli), "--set", "bad=1"]) == 2
    assert replay_safety.main([*base, "--cli", str(tmp_path / "missing")]) == 2


def _real_cli() -> Path | None:
    try:
        return stream.find_cli()
    except stream.ReplayError:
        return None


@pytest.mark.skipif(_real_cli() is None, reason="safety_replay_cli is not built")
def test_real_cli_brakes_earlier_with_the_body_rule(bag, tmp_path):
    cli = _real_cli()
    path = tmp_path / "stream.txt"
    stream.write_stream(bag, path)
    legacy = [r for r in stream.run_cli(cli, path, stream.LEGACY_RULE) if r["type"] == "trip"]
    current = [r for r in stream.run_cli(cli, path, {}) if r["type"] == "trip"]
    assert legacy and current
    assert current[0]["t"] < legacy[0]["t"]
    described = stream.describe_cli(cli)
    assert described["distance_from_body"] == 1
    assert json.dumps(described)
    assert os.path.basename(str(cli)) == stream.CLI_NAME


# -- calibration: old-unit bags in the committed units ---------------------------------------

OLD = calibration.RECORDED_0_13_0
# The committed map's numbers, typed in on purpose (the conversion test is a golden of the
# 2026-11-10 measurements): left stop 1094 us = 0.504 rad, neutral 1500 us, right stop 1875 us
# = 0.449 rad.
NEW = calibration.Current(
    pwm_left_us=1094.0,
    pwm_neutral_us=1500.0,
    pwm_right_us=1875.0,
    table=((1094.0, 0.504), (1500.0, 0.0), (1875.0, -0.449)),
    planning_limit_rad=0.449,
)


def test_a_recorded_full_lock_becomes_the_measured_real_angle_of_that_stop():
    # The old linear map sent +/- 0.4189 to the two stops, which the table reads as 0.504 / 0.449.
    assert calibration.steering_rad(OLD, NEW, 0.4189) == pytest.approx(0.504)
    assert calibration.steering_rad(OLD, NEW, -0.4189) == pytest.approx(-0.449)
    assert calibration.steering_rad(OLD, NEW, 0.0) == 0.0
    # Halfway to a stop is halfway in angle, on each side (a straight line from neutral).
    assert calibration.steering_rad(OLD, NEW, 0.4189 / 2) == pytest.approx(0.252)
    assert calibration.steering_rad(OLD, NEW, -0.4189 / 2) == pytest.approx(-0.2245)
    # Beyond the old limit the old map clamped at the stop; so does the conversion.
    assert calibration.steering_rad(OLD, NEW, 1.0) == pytest.approx(0.504)
    assert calibration.steering_rad(OLD, NEW, -1.0) == pytest.approx(-0.449)
    assert math.isnan(calibration.steering_rad(OLD, NEW, math.nan))


def test_old_pulse_and_table_angle_round_trip_the_published_numbers():
    assert calibration.old_pulse_us(OLD, NEW, 0.4189) == pytest.approx(1094.0)
    assert calibration.old_pulse_us(OLD, NEW, -0.4189) == pytest.approx(1875.0)
    assert calibration.old_pulse_us(OLD, NEW, math.nan) == 1500.0
    assert calibration.table_angle_rad(NEW, 900.0) == 0.504  # held past the first point
    assert calibration.table_angle_rad(NEW, 2000.0) == -0.449  # and the last
    assert calibration.table_angle_rad(NEW, 1500.0) == 0.0
    assert NEW.steer_limit_rad == 0.504


def test_speeds_are_divided_by_the_push_test_ratio():
    assert OLD.speed_ratio == 1.401
    assert calibration.speed_mps(OLD, 1.401) == pytest.approx(1.0)
    assert calibration.speed_mps(OLD, -0.7005) == pytest.approx(-0.5)


def test_convert_bag_converts_three_topics_and_nothing_else(bag):
    converted = calibration.convert_bag(bag, OLD, NEW)
    assert converted.drive_raw[10].speed_mps == pytest.approx(SPEED / 1.401)
    assert converted.drive[0].speed_mps == pytest.approx(SPEED / 1.401)
    assert converted.odom[10].speed_mps == pytest.approx(bag.odom[10].speed_mps / 1.401)
    assert converted.odom[10].log_time_s == bag.odom[10].log_time_s
    assert converted.scans is bag.scans and converted.events is bag.events
    assert bag.drive_raw[10].speed_mps == SPEED  # the input is not modified
    steered = BagData(drive_raw=[DriveRecord(1.0, 0.4189, 0.8, 1.0)])
    assert calibration.convert_bag(steered, OLD, NEW).drive_raw[0].steering_rad == (
        pytest.approx(0.504)
    )
    kept = calibration.convert_bag(steered, OLD, NEW, steering=False)
    assert kept.drive_raw[0].steering_rad == 0.4189
    assert kept.drive_raw[0].speed_mps == pytest.approx(0.8 / 1.401)


def test_the_committed_file_loads_as_a_calibration():
    current = calibration.load_current()
    assert current.pwm_left_us < current.pwm_neutral_us < current.pwm_right_us
    assert [round(a, 3) for _, a in current.table] == [0.504, 0.0, -0.449]
    assert current.steer_limit_rad >= current.planning_limit_rad
    # The committed table agrees with the one this test types in.
    assert current.table == NEW.table and current.planning_limit_rad == NEW.planning_limit_rad


def test_a_null_table_cannot_convert(tmp_path):
    params = tmp_path / "vehicle_params.yaml"
    params.write_text(
        "steering: {pwm_to_angle_table: null, pwm_left_bound: pwm_min_us, pwm_min_us: 1094.0,"
        " pwm_max_us: 1875.0, pwm_neutral_us: 1500.0, max_angle_rad: 0.4189}\n"
    )
    with pytest.raises(ValueError, match="pwm_to_angle_table is null"):
        calibration.load_current(params)


def test_prepare_writes_a_raw_and_a_converted_stream(bag, tmp_path):
    prep = replay_safety.prepare("scene", tmp_path, OLD, NEW, bag=bag)
    raw = next(l for l in prep.raw_stream.read_text().splitlines() if l.startswith("D")).split()
    new = next(l for l in prep.converted_stream.read_text().splitlines() if l.startswith("D"))
    assert float(raw[3]) == pytest.approx(SPEED)
    assert float(new.split()[3]) == pytest.approx(SPEED / 1.401, rel=1e-6)
    assert prep.lock_rad == 0.504
    same = replay_safety.prepare("scene", tmp_path, None, None, bag=bag)
    assert same.converted is same.raw and same.lock_rad is None


def test_after_rule_runs_on_converted_units_and_the_before_rule_on_raw(
    scene_bag, tmp_path, monkeypatch, capsys
):
    cli = tmp_path / "safety_replay_cli"
    cli.write_text(FAKE_CLI)
    cli.chmod(cli.stat().st_mode | stat.S_IEXEC)
    log = tmp_path / "cli.log"
    monkeypatch.setenv("FAKE_CLI_LOG", str(log))
    status = replay_safety.main([str(scene_bag), "--cli", str(cli), "--recorded-with", "0.13.0"])
    out = capsys.readouterr().out
    assert status == 0, out
    assert "bags converted from 0.13.0 (speed / 1.401" in out
    before, after = (json.loads(line) for line in log.read_text().splitlines())
    assert "--cycles" not in before["args"] and "--cycles" in after["args"]
    assert float(before["first_d"][3]) == pytest.approx(SPEED)
    assert float(after["first_d"][3]) == pytest.approx(SPEED / 1.401, rel=1e-6)
    # The converted left lock is wider than the planning limit, so the gate is given it.
    assert "max_steering_rad=0.504" in after["args"]
    assert "max_steering_rad=0.504" not in before["args"]


def test_current_units_are_replayed_as_recorded(scene_bag, tmp_path, monkeypatch, capsys):
    cli = tmp_path / "safety_replay_cli"
    cli.write_text(FAKE_CLI)
    cli.chmod(cli.stat().st_mode | stat.S_IEXEC)
    log = tmp_path / "cli.log"
    monkeypatch.setenv("FAKE_CLI_LOG", str(log))
    assert (
        replay_safety.main([str(scene_bag), "--cli", str(cli), "--recorded-with", "current"]) == 0
    )
    capsys.readouterr()
    before, after = (json.loads(line) for line in log.read_text().splitlines())
    assert float(before["first_d"][3]) == pytest.approx(float(after["first_d"][3]))
    assert not any(a.startswith("max_steering_rad") for a in after["args"])


def test_the_recorded_with_flag_is_required(scene_bag):
    with pytest.raises(SystemExit) as raised:
        replay_safety.main([str(scene_bag)])
    assert raised.value.code == 2


def test_sweep_prints_one_row_per_value(scene_bag, tmp_path, capsys):
    cli = tmp_path / "safety_replay_cli"
    cli.write_text(FAKE_CLI)
    cli.chmod(cli.stat().st_mode | stat.S_IEXEC)
    status = replay_safety.main(
        [
            str(scene_bag),
            "--cli",
            str(cli),
            "--recorded-with",
            "current",
            "--sweep",
            "ttc_brake_s=0.3,0.4",
        ]
    )
    out = capsys.readouterr().out
    assert status == 0, out
    rows = [line for line in out.splitlines() if line.startswith("| 0.")]
    assert [row.split("|")[1].strip() for row in rows] == ["0.3", "0.4"]
    assert all(row.split("|")[2].strip() == "1" for row in rows)  # the one recorded brake


def test_parse_sweep_and_before_rule_choices():
    assert replay_safety.parse_sweep("ttc_brake_s=0.3, 0.4") == ("ttc_brake_s", [0.3, 0.4])
    for bad in ("x", "x=", "=1", "x=a,b"):
        with pytest.raises(stream.ReplayError):
            replay_safety.parse_sweep(bad)
    assert replay_safety.BEFORE_RULES["0.13.0"]["ttc_brake_s"] == 0.34
    assert replay_safety.BEFORE_RULES["october"] is stream.LEGACY_RULE


# -- the car's own static returns -------------------------------------------------------------


def with_self_post(scan: ScanRecord, range_m: float = 0.185) -> ScanRecord:
    """The scan with one beam straight back at a post 0.10 m ahead of the rear axle (0.185 m
    behind the head): a part of the car, in every scan, as in the 2026-11 bags."""
    ranges = list(scan.ranges)
    ranges[N_RAYS // 2] = range_m  # laser bearing 0, vehicle bearing pi, the beam to the rear
    return dataclasses.replace(scan, ranges=tuple(ranges))


@pytest.fixture(scope="module")
def posted_bag(bag) -> BagData:
    return dataclasses.replace(bag, scans=[with_self_post(s) for s in bag.scans])


def test_a_post_in_every_scan_is_found_as_the_cars_own_part(posted_bag, bag):
    mask = motion.static_self_returns(posted_bag.scans, MOUNT, BODY)
    assert mask and all(abs(x * motion.SELF_CELL_M - 0.10) < 0.03 for x, _ in mask)
    assert motion.static_self_returns(bag.scans, MOUNT, BODY) == frozenset()  # a clean car
    assert motion.static_self_returns([], MOUNT, BODY) == frozenset()


def test_a_return_in_a_few_scans_is_not_the_car(bag):
    few = [with_self_post(s) for s in bag.scans[:3]]
    mixed = few + list(bag.scans[3:])
    assert motion.static_self_returns(mixed, MOUNT, BODY) == frozenset()


def test_without_self_drops_the_cells_and_keeps_the_rest():
    mask = frozenset({(5, 0)})
    points = np.array([[0.10, 0.0], [0.11, 0.01], [0.30, 0.0], [0.10, 0.2]])
    kept = motion.without_self(points, mask)
    assert kept.tolist() == [[0.30, 0.0], [0.10, 0.2]]
    assert motion.without_self(points, None) is points
    assert motion.without_self(points, frozenset()) is points
    assert len(motion.without_self(np.empty((0, 2)), mask)) == 0


def test_the_cars_own_post_does_not_read_as_a_contact_or_anchor_the_matching(posted_bag, bag):
    mask = motion.static_self_returns(posted_bag.scans, MOUNT, BODY)
    clean = analysis.analyse(bag, [trip(T_BRAKE, 0.3)], [], MOUNT, BODY).rows[0]
    blind = analysis.analyse(posted_bag, [trip(T_BRAKE, 0.3)], [], MOUNT, BODY).rows[0]
    aware = analysis.analyse(posted_bag, [trip(T_BRAKE, 0.3)], [], MOUNT, BODY, ignore=mask).rows[0]
    assert blind.end_clearance_m == 0.0 and blind.contact  # the post, inside the footprint
    assert aware.end_clearance_m == pytest.approx(clean.end_clearance_m, abs=0.003)
    assert aware.travel_m == pytest.approx(clean.travel_m, abs=0.01)
    assert not aware.contact
