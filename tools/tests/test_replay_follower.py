"""L1/L4 tests for tools/follower_replay (bag reading, profiles, analysis, plots, pipeline).

The bags are written here with mcap-ros2-support from the repo's own synthetic scan fixture
(ros_ws/src/racer_control/test/replay/scenes.txt), so no ROS and no recorded bag is needed; the
reader is exercised on an indexed file, on an unindexed one (no footer, as after a killed
recorder) and on one cut off inside a record. The tests that need the real C++ replay binary run
only when it can be found (GAP_FOLLOW_REPLAY_CLI, a colcon install tree, or PATH): the CLI's own
ctest covers it in CI.
"""

from __future__ import annotations

import json
import math
import os
import stat
import sys
from pathlib import Path

import pytest
from follower_replay import analysis, bag, plots, replay
from follower_replay.pipeline import PlotOptions, replay_bag

REPO_ROOT = Path(__file__).resolve().parents[2]
REPLAY_FIXTURES = REPO_ROOT / "ros_ws/src/racer_control/test/replay"
T0 = 1000.0  # bag clock start (s); nonzero so header stamps are never mistaken for "unset"

LASER_SCAN_DEF = """std_msgs/Header header
float32 angle_min
float32 angle_max
float32 angle_increment
float32 time_increment
float32 scan_time
float32 range_min
float32 range_max
float32[] ranges
float32[] intensities
================================================================================
MSG: std_msgs/Header
builtin_interfaces/Time stamp
string frame_id
================================================================================
MSG: builtin_interfaces/Time
int32 sec
uint32 nanosec
"""

ACKERMANN_DEF = """std_msgs/Header header
ackermann_msgs/AckermannDrive drive
================================================================================
MSG: ackermann_msgs/AckermannDrive
float32 steering_angle
float32 steering_angle_velocity
float32 speed
float32 acceleration
float32 jerk
================================================================================
MSG: std_msgs/Header
builtin_interfaces/Time stamp
string frame_id
================================================================================
MSG: builtin_interfaces/Time
int32 sec
uint32 nanosec
"""

EVENT_DEF = """builtin_interfaces/Time stamp
uint8 severity
uint8 phase
float64 duration_s
string source
string detail
================================================================================
MSG: builtin_interfaces/Time
int32 sec
uint32 nanosec
"""


def _time(t: float) -> dict[str, int]:
    sec = int(t)
    return {"sec": sec, "nanosec": round((t - sec) * 1e9)}


def fixture_scans() -> list[tuple[float, float, float, list[float]]]:
    """(stamp, angle_min, increment, ranges) for each line of scenes.txt, stamps shifted by T0."""
    out = []
    for line in (REPLAY_FIXTURES / "scenes.txt").read_text().splitlines():
        if not line or line.startswith("#"):
            continue
        words = line.split()
        out.append(
            (T0 + float(words[0]), float(words[1]), float(words[2]), [float(w) for w in words[4:]])
        )
    return out


def write_bag(path: Path, finish: bool = True, chunk_size: int = 1024 * 1024) -> None:
    """The fixture scans at 10 Hz, /drive_raw at 50 Hz (a slow left turn), one TTC latch."""
    from mcap_ros2.writer import Writer

    with path.open("wb") as stream:
        writer = Writer(stream, chunk_size=chunk_size)
        scan_schema = writer.register_msgdef("sensor_msgs/msg/LaserScan", LASER_SCAN_DEF)
        drive_schema = writer.register_msgdef(
            "ackermann_msgs/msg/AckermannDriveStamped", ACKERMANN_DEF
        )
        event_schema = writer.register_msgdef("racer_msgs/msg/SafetyEvent", EVENT_DEF)
        scans = fixture_scans()
        for stamp, angle_min, increment, ranges in scans:
            writer.write_message(
                "/scan",
                scan_schema,
                {
                    "header": {"stamp": _time(stamp), "frame_id": "laser"},
                    "angle_min": angle_min,
                    "angle_max": angle_min + increment * (len(ranges) - 1),
                    "angle_increment": increment,
                    "time_increment": 0.0,
                    "scan_time": 0.1,
                    "range_min": 0.05,
                    "range_max": 12.0,
                    "ranges": ranges,
                    "intensities": [],
                },
                log_time=round(stamp * 1e9),
            )
        for k in range(25):
            stamp = T0 + 0.02 * k
            for topic in ("/drive_raw", "/drive"):
                writer.write_message(
                    topic,
                    drive_schema,
                    {
                        "header": {"stamp": _time(stamp), "frame_id": "base_link"},
                        "drive": {
                            "steering_angle": 0.01 * k,
                            "steering_angle_velocity": 0.0,
                            "speed": 0.5,
                            "acceleration": 0.0,
                            "jerk": 0.0,
                        },
                    },
                    log_time=round(stamp * 1e9),
                )
        for stamp, phase, duration in ((T0 + 0.15, 0, 0.0), (T0 + 0.35, 1, 0.2)):
            writer.write_message(
                "/safety/events",
                event_schema,
                {
                    "stamp": _time(stamp),
                    "severity": 2,
                    "phase": phase,
                    "duration_s": duration,
                    "source": "ttc",
                    "detail": "synthetic",
                },
                log_time=round(stamp * 1e9),
            )
        writer.write_message(
            "/rosout",
            scan_schema,
            {
                "header": {"stamp": _time(T0), "frame_id": ""},
                "angle_min": 0.0,
                "angle_max": 0.0,
                "angle_increment": 1.0,
                "time_increment": 0.0,
                "scan_time": 0.0,
                "range_min": 0.0,
                "range_max": 1.0,
                "ranges": [],
                "intensities": [],
            },
            log_time=round(T0 * 1e9),
        )
        if finish:
            writer.finish()


@pytest.fixture(scope="module")
def indexed_bag(tmp_path_factory) -> Path:
    path = tmp_path_factory.mktemp("bags") / "indexed.mcap"
    write_bag(path)
    return path


# -- bag reading ----------------------------------------------------------------------------


def test_read_indexed_bag(indexed_bag):
    data = bag.read_bag(indexed_bag)
    assert not data.truncated and data.decode_errors == 0
    assert len(data.scans) == 5
    assert len(data.drive_raw) == len(data.drive) == 25
    assert data.topic_counts["/rosout"] == 1
    first = data.scans[0]
    assert first.stamp_s == pytest.approx(T0)
    assert len(first.ranges) == 360 and first.angle_min == pytest.approx(-math.pi, abs=1e-6)
    assert first.range_max == 12.0
    # inf (no return) and nan survive the round trip through the bag.
    assert any(math.isinf(r) for r in first.ranges)
    assert all(math.isnan(r) for r in data.scans[4].ranges)
    assert data.drive_raw[10].steering_rad == pytest.approx(0.10, abs=1e-6)
    assert [(e.phase, e.severity, e.source) for e in data.events] == [(0, 2, "ttc"), (1, 2, "ttc")]


def test_unset_header_stamp_falls_back_to_log_time():
    class Msg:
        sec = 0
        nanosec = 0

    assert bag._stamp_s(Msg, 2_500_000_000) == 2.5


def test_read_unindexed_bag(tmp_path):
    """A recorder killed before finish(): chunks on disk, no summary, no footer."""
    path = tmp_path / "killed.mcap"
    write_bag(path, finish=False, chunk_size=1)
    data = bag.read_bag(path)
    assert data.truncated and "no footer" in data.truncation_note
    assert len(data.scans) == 5 and len(data.drive_raw) == 25 and len(data.events) == 2


def test_read_bag_cut_inside_a_record(indexed_bag, tmp_path):
    raw = indexed_bag.read_bytes()
    path = tmp_path / "cut.mcap"
    path.write_bytes(raw[: int(len(raw) * 0.6)])
    data = bag.read_bag(path)  # must not raise
    assert data.truncated
    assert len(data.scans) <= 5


def test_read_bag_directory_and_errors(indexed_bag, tmp_path):
    folder = tmp_path / "run"
    folder.mkdir()
    (folder / "run_0.mcap").write_bytes(indexed_bag.read_bytes())
    assert len(bag.read_bag(folder).scans) == 5
    empty = tmp_path / "empty"
    empty.mkdir()
    with pytest.raises(FileNotFoundError, match="no .mcap"):
        bag.read_bag(empty)
    with pytest.raises(FileNotFoundError, match="no such bag"):
        bag.read_bag(tmp_path / "missing")


# -- profiles and parameters -------------------------------------------------------------------


def test_profiles_are_read_from_the_launch_file():
    profiles = replay.load_profiles()
    assert profiles["none"] == {}
    floor = profiles["floor-2026-10-06"]
    assert floor["steering_gain"] == 1.6 and floor["swept_path_clamp"] is True
    assert floor["free_space_threshold_m"] == 0.6 and floor["target_deepest_ray"] is False


def test_profile_overrides():
    overrides = replay.parse_overrides(["steering_gain=2", "swept_path_clamp=false", "x=0.5"])
    assert overrides == {"steering_gain": 2, "swept_path_clamp": False, "x": 0.5}
    params = replay.resolve_params("floor-2026-10-06", {"steering_gain": 2.5})
    assert params["steering_gain"] == 2.5 and params["cone_half_angle_rad"] == 1.2
    with pytest.raises(replay.ReplayError, match="unknown profile"):
        replay.resolve_params("nope", {})
    for bad in ("novalue", "=1", "k=abc"):
        with pytest.raises(replay.ReplayError):
            replay.parse_overrides([bad])


def test_load_profiles_resolves_names_and_rejects_no_profiles(tmp_path):
    launch = tmp_path / "x.launch.py"
    launch.write_text(
        'A: dict = {"k": 1.5, "b": True}\nPROFILES = {"none": {}, "a": A}\n', encoding="utf-8"
    )
    assert replay.load_profiles(launch) == {"none": {}, "a": {"k": 1.5, "b": True}}
    launch.write_text("X = 1\n", encoding="utf-8")
    with pytest.raises(replay.ReplayError, match="no PROFILES"):
        replay.load_profiles(launch)
    launch.write_text("PROFILES = {'a': MISSING}\n", encoding="utf-8")
    with pytest.raises(replay.ReplayError):
        replay.load_profiles(launch)


def test_node_parameter_names_in_the_floor_profile_are_known_to_the_cli():
    """Every key of the shipped profile must be one the CLI knows or knowingly ignores."""
    cpp = (REPO_ROOT / "ros_ws/src/racer_control/src/gap_follow_replay_cli.cpp").read_text()
    for key in replay.load_profiles()["floor-2026-10-06"]:
        assert f'"{key}"' in cpp or key.startswith("escape_"), key


def test_cli_defaults_match_the_node():
    """The CLI's parameter table (names, defaults, ranges) is gap_follow_node's, row for row."""
    import re

    root = REPO_ROOT / "ros_ws/src/racer_control/src"
    cpp = (root / "gap_follow_replay_cli.cpp").read_text()
    node = (root / "gap_follow_node.cpp").read_text()

    def num(text: str) -> float:
        return float(
            eval(text.strip().replace("kPi", "math.pi").replace("M_PI", "math.pi"), {"math": math})
        )

    table = {}
    block = cpp[cpp.index("const ParamSpec kParams[]") : cpp.index("// clang-format on")]
    for m in re.finditer(r'\{"(\w+)",\s*([^,]+),\s*([^,]+),\s*([^,]+),\s*(true|false)\}', block):
        table[m.group(1)] = (
            num(m.group(2)),
            num(m.group(3)),
            num(m.group(4)),
            m.group(5) == "true",
        )
    assert len(table) > 20
    flat = re.sub(r"\s+", " ", node)
    seen = set()
    for m in re.finditer(
        r'declare_ranged_double\(\s*\*this, "(\w+)", ([^,]+), ([^,]+), ([^,]+),', flat
    ):
        name = m.group(1)
        if name not in table:
            continue
        seen.add(name)
        assert table[name][:3] == pytest.approx(
            (num(m.group(2)), num(m.group(3)), num(m.group(4)))
        ), name
    for m in re.finditer(r'declare_described_bool\(\s*\*this, "(\w+)", (true|false),', flat):
        name = m.group(1)
        if name not in table:
            continue
        seen.add(name)
        assert table[name][3] and table[name][0] == (1.0 if m.group(2) == "true" else 0.0), name
    cli_only = {"range_min_m", "range_max_m"}
    assert set(table) - seen == cli_only, set(table) - seen - cli_only


def test_scan_line_format():
    scan = bag.ScanRecord(12.5, -3.1415927, 0.0174533, 0.05, 12.0, (1.0, math.inf, math.nan, 0.5))
    words = replay.scan_line(scan).split()
    assert words[0] == "12.500000000" and words[3] == "4" and len(words) == 8
    assert words[4:] == ["1", "inf", "nan", "0.5"]
    assert replay.lidar_range_limits([scan]) == {"range_min_m": 0.05, "range_max_m": 12.0}
    unset = bag.ScanRecord(0.0, 0.0, 0.1, 0.0, math.inf, ())
    assert replay.lidar_range_limits([unset]) == {} and replay.lidar_range_limits([]) == {}


def test_find_cli(tmp_path, monkeypatch):
    fake = tmp_path / "cli"
    fake.write_text("")
    assert replay.find_cli(str(fake)) == fake
    with pytest.raises(replay.ReplayError, match="no such file"):
        replay.find_cli(str(tmp_path / "nope"))
    monkeypatch.setenv(replay.CLI_ENV, str(fake))
    assert replay.find_cli() == fake
    monkeypatch.setenv(replay.CLI_ENV, "")
    monkeypatch.setattr(replay.shutil, "which", lambda _name: None)
    monkeypatch.setattr(replay, "REPO_ROOT", tmp_path)
    with pytest.raises(replay.ReplayError, match="not found"):
        replay.find_cli()


# -- analysis -------------------------------------------------------------------------------


def _row(valid=True, gap=True, steer=0.0, filtered=0.0, clamped=False, corner=False):
    return {
        "valid": valid,
        "gap_found": gap,
        "target_bearing_rad": 0.1,
        "target_range_m": 2.0,
        "wanted_steering_rad": steer,
        "steering_rad": steer,
        "filtered_steering_rad": filtered,
        "clamped": clamped,
        "corner_blocked": corner,
        "centering_rad": 0.0,
    }


def _scan(t):
    return bag.ScanRecord(t, 0.0, 0.1, 0.05, 12.0, (1.0,))


def test_recorded_before_next_picks_the_last_command_of_the_interval():
    recs = [analysis.DriveRecord(t, t * 10, 0.5) for t in (0.0, 0.02, 0.04, 0.10, 0.12)]
    assert analysis.recorded_before_next(recs, 0.0, 0.1) == pytest.approx(0.4)
    assert analysis.recorded_before_next(recs, 0.1, None) == pytest.approx(1.2)
    assert math.isnan(analysis.recorded_before_next(recs, 0.5, 0.6))
    assert math.isnan(analysis.recorded_before_next([], 0.0, None))


def test_table_summary_and_csv(tmp_path):
    scans = [_scan(10.0 + 0.1 * k) for k in range(5)]
    rows = [
        _row(steer=0.3, filtered=0.2, clamped=True),
        _row(steer=-0.3, filtered=-0.2),
        _row(steer=0.0, filtered=0.0, corner=True),
        _row(gap=False),
        _row(valid=False),
    ]
    rows[4].update(
        steering_rad=None,
        filtered_steering_rad=0.0,
        target_bearing_rad=None,
        target_range_m=None,
        wanted_steering_rad=None,
        centering_rad=None,
    )
    raw = [
        analysis.DriveRecord(10.0 + 0.1 * k + 0.05, s, 0.5)
        for k, s in enumerate((0.25, 0.2, 0.0, 0.0, 0.0))
    ]
    table = analysis.build_table(scans, rows, raw, [])
    assert [r["t_s"] for r in table] == pytest.approx([0.0, 0.1, 0.2, 0.3, 0.4])
    assert table[0]["recorded_drive_raw_rad"] == 0.25 and math.isnan(table[0]["recorded_drive_rad"])
    with pytest.raises(ValueError):
        analysis.build_table(scans, rows[:2], raw, [])

    path = tmp_path / "t.csv"
    analysis.write_csv(table, path)
    lines = path.read_text().splitlines()
    assert lines[0].split(",") == list(analysis.CSV_COLUMNS) and len(lines) == 6
    assert lines[5].split(",")[analysis.CSV_COLUMNS.index("steering_rad")] == ""

    events = [
        bag.EventRecord(10.1, 2, 0, 0.0, "ttc", ""),
        bag.EventRecord(10.2, 2, 1, 1.0, "ttc", ""),
        bag.EventRecord(10.3, 2, 0, 0.0, "ttc", ""),
        bag.EventRecord(10.4, 2, 1, 3.0, "ttc", ""),
        bag.EventRecord(10.4, 0, 0, 0.0, "rate_limit", ""),
        bag.EventRecord(10.4, 2, 0, 0.0, "ttc_reverse", ""),
    ]
    data = bag.BagData(
        scans=scans,
        events=events,
        files=["x.mcap"],
        truncated=True,
        truncation_note="x.mcap: no footer",
    )
    summary = analysis.summarise(data, table, "floor-2026-10-06", {"steering_gain": 1.6})
    assert summary["scans"] == 5 and summary["valid_scans"] == 4 and summary["gap_not_found"] == 1
    assert summary["steering_replay"] == {"left": 1, "right": 1, "straight": 2}
    assert summary["clamp_rate"] == 0.25 and summary["corner_override_rate"] == 0.25
    ev = summary["events"]
    assert ev["brakes"] == {"ttc": 2, "ttc_reverse": 1} and ev["brakes_total"] == 3
    assert ev["engages"]["rate_limit"] == 1
    assert ev["latch_durations"]["ttc"] == {
        "count": 2,
        "total_s": 4.0,
        "median_s": 2.0,
        "max_s": 3.0,
    }
    assert ev["still_engaged_at_end"] == {"rate_limit": 1, "ttc_reverse": 1}
    agree = summary["agreement_with_recorded"]
    assert agree["scans_compared"] == 5 and agree["both_turning"] == 2
    assert agree["sign_disagreements_when_both_turning"] == 1  # replay -0.2 vs recorded +0.2
    text = analysis.format_summary(summary)
    assert "truncated/unindexed" in text and "3 brake engagement(s)" in text
    assert "ttc releases: 2, total 4.0 s" in text and chr(0x2014) not in text
    json.dumps(summary)  # serialisable as written to summary.json


def test_summary_of_a_quiet_bag():
    data = bag.BagData(files=["q.mcap"])
    summary = analysis.summarise(data, [], "none", {})
    assert summary["scans"] == 0 and summary["clamp_rate"] == 0.0
    assert summary["agreement_with_recorded"]["mean_abs_error_rad"] is None
    text = analysis.format_summary(summary)
    assert "/safety/events: none recorded" in text


# -- plots ---------------------------------------------------------------------------------


def test_select_scan_indices():
    table = [{"t_s": 0.1 * i, "clamped": i % 4 == 0, "corner_blocked": i == 7} for i in range(20)]
    assert plots.select_scan_indices(table, 3) == [0, 10, 19]
    assert plots.select_scan_indices(table, 1) == [10]
    assert plots.select_scan_indices(table, 99) == list(range(20))
    assert plots.select_scan_indices(table, 0) == []
    assert plots.select_scan_indices(table, 3, at_s=[0.52, 1.49]) == [5, 15]
    assert plots.select_scan_indices(table, 2, flagged_only=True) == [0, 16]
    assert plots.select_scan_indices(table, 9, flagged_only=True) == [0, 4, 7, 8, 12, 16]
    assert plots.select_scan_indices([], 3) == []
    assert (
        plots.select_scan_indices(
            [{"t_s": 0, "clamped": False, "corner_blocked": False}], 2, flagged_only=True
        )
        == []
    )


GEOMETRY = {
    "laser_yaw_rad": math.pi,
    "lidar_mount_x_m": 0.285,
    "lidar_mount_y_m": 0.0,
    "half_width_m": 0.155,
    "wheelbase_m": 0.33,
    "body_front_x_m": 0.46,
    "max_steering_rad": 0.4189,
}


def test_vehicle_points_apply_the_laser_yaw_and_mount():
    # Laser bearing pi with yaw pi is straight ahead of the car; bearing -pi/2 + pi = +pi/2 is left.
    scan = bag.ScanRecord(0.0, math.pi, math.pi / 2, 0.05, 12.0, (2.0, 1.0, math.inf, 0.0))
    xs, ys = plots.vehicle_points(scan, GEOMETRY)
    assert xs[0] == pytest.approx(2.285) and ys[0] == pytest.approx(0.0, abs=1e-9)
    # bearing = pi + pi/2 + pi = 5pi/2 = pi/2: left, x = mount only, y = +1.0
    assert xs[1] == pytest.approx(0.285) and ys[1] == pytest.approx(1.0)
    assert len(xs) == 2  # inf and 0 are not returns


def test_arc_points_follow_the_bicycle_model():
    straight = plots.arc_points(0.0, 0.33)
    assert straight[-1] == (plots.ARC_LENGTH_M, 0.0)
    left = plots.arc_points(0.3, 0.33)
    radius = 0.33 / math.tan(0.3)
    assert left[-1][1] > 0  # steering left curves to +y
    assert math.hypot(left[-1][0], left[-1][1] - radius) == pytest.approx(radius)
    assert plots.arc_points(-0.3, 0.33)[-1][1] < 0


def test_latch_spans():
    events = [
        bag.EventRecord(12.0, 2, 1, 2.0, "ttc", ""),
        bag.EventRecord(13.0, 2, 1, 1.0, "x", ""),
    ]
    assert plots.latch_spans(events, 10.0) == [(0.0, 2.0, "ttc")]


def test_plots_write_pngs(tmp_path):
    pytest.importorskip("matplotlib")
    scans = [
        bag.ScanRecord(T0 + 0.1 * k, a, inc, 0.05, 12.0, tuple(r))
        for k, (_, a, inc, r) in enumerate(fixture_scans())
    ]
    rows = [_row(steer=0.1 * k, filtered=0.05 * k, clamped=k == 3) for k in range(5)]
    table = analysis.build_table(
        scans, rows, [analysis.DriveRecord(T0 + 0.05 * k, 0.1, 0.5) for k in range(10)], []
    )
    events = [bag.EventRecord(T0 + 0.3, 2, 1, 0.2, "ttc", "")]
    plots.plot_scans(scans, table, [0, 1, 3, 4], GEOMETRY, tmp_path / "s.png")
    plots.plot_steering_timeline(table, events, tmp_path / "t.png")
    for name in ("s.png", "t.png"):
        assert (tmp_path / name).read_bytes()[:8] == b"\x89PNG\r\n\x1a\n"


# -- pipeline ------------------------------------------------------------------------------

FAKE_CLI = f"""#!{sys.executable}
import json, sys
args = sys.argv[1:]
if "--describe" in args:
    print(json.dumps({json.dumps(GEOMETRY)}))
    sys.exit(0)
path = args[args.index("--scans") + 1]
params = json.load(open(args[args.index("--params") + 1]))
assert params["steering_gain"] == 1.6 and params["range_max_m"] == 12.0, params
for line in open(path):
    if line.startswith("#"):
        continue
    w = line.split()
    s = 0.2 if float(w[0]) < {T0 + 0.25} else -0.2
    print(json.dumps({{"stamp_s": float(w[0]), "valid": True, "gap_found": True,
        "target_bearing_rad": 0.1, "target_range_m": 2.0, "wanted_steering_rad": s,
        "steering_rad": s, "filtered_steering_rad": s, "clamped": s > 0, "corner_blocked": False,
        "centering_rad": 0.0}}))
"""


def test_pipeline_end_to_end_with_a_fake_cli(indexed_bag, tmp_path):
    cli = tmp_path / "fake_cli"
    cli.write_text(FAKE_CLI)
    cli.chmod(cli.stat().st_mode | stat.S_IXUSR)
    out = tmp_path / "out"
    result = replay_bag(indexed_bag, out, "floor-2026-10-06", {}, cli, PlotOptions(enabled=False))
    assert {p.name for p in out.iterdir()} == {
        "scans.txt",
        "params.json",
        "per_scan.csv",
        "summary.json",
    }
    assert result.summary["scans"] == 5 and result.summary["clamped_scans"] == 3
    assert json.loads((out / "summary.json").read_text())["profile"] == "floor-2026-10-06"
    assert (out / "per_scan.csv").read_text().count("\n") == 6

    pytest.importorskip("matplotlib")
    result = replay_bag(
        indexed_bag, tmp_path / "out2", "none", {"steering_gain": 1.6}, cli, PlotOptions(True, 2)
    )
    assert result.files["plot_scans"].exists() and result.files["plot_steering"].exists()


def test_pipeline_refuses_a_bag_without_scans(tmp_path):
    from mcap_ros2.writer import Writer

    path = tmp_path / "noscan.mcap"
    with path.open("wb") as stream:
        writer = Writer(stream)
        schema = writer.register_msgdef("racer_msgs/msg/SafetyEvent", EVENT_DEF)
        writer.write_message(
            "/safety/events",
            schema,
            {
                "stamp": _time(T0),
                "severity": 0,
                "phase": 0,
                "duration_s": 0.0,
                "source": "ttc",
                "detail": "",
            },
            log_time=round(T0 * 1e9),
        )
        writer.finish()
    with pytest.raises(replay.ReplayError, match="no /scan messages"):
        replay_bag(path, tmp_path / "o", "none", {}, Path("unused"))


def test_command_line(indexed_bag, tmp_path, capsys):
    import replay_follower

    cli = tmp_path / "fake_cli"
    cli.write_text(FAKE_CLI)
    cli.chmod(cli.stat().st_mode | stat.S_IXUSR)
    out = tmp_path / "cli_out"
    code = replay_follower.main(
        [str(indexed_bag), "--cli", str(cli), "--out-dir", str(out), "--param", "steering_gain=1.6"]
    )
    assert code == 0 and "scans: 5" in capsys.readouterr().out
    assert replay_follower.main([str(indexed_bag), "--cli", str(cli), "--profile", "nope"]) == 2
    assert "unknown profile" in capsys.readouterr().err
    assert replay_follower.main([str(tmp_path / "missing"), "--cli", str(cli)]) == 2


# -- against the real C++ binary (skipped when it is not built) -----------------------------


def _real_cli() -> Path | None:
    try:
        return replay.find_cli()
    except replay.ReplayError:
        return None


@pytest.mark.skipif(_real_cli() is None, reason="gap_follow_replay_cli is not built")
def test_real_cli_reproduces_the_committed_golden(indexed_bag, tmp_path):
    """bag -> text -> C++ -> table gives the same numbers the CLI's own ctest pins."""
    cli = _real_cli()
    assert cli is not None
    profile = json.loads((REPLAY_FIXTURES / "params.json").read_text())
    result = replay_bag(indexed_bag, tmp_path / "real", "none", profile, cli)
    expected = [
        json.loads(line) for line in (REPLAY_FIXTURES / "expected.jsonl").read_text().splitlines()
    ]
    assert len(result.table) == len(expected)
    for got, want in zip(result.table, expected, strict=True):
        for key in (
            "steering_rad",
            "wanted_steering_rad",
            "filtered_steering_rad",
            "target_bearing_rad",
        ):
            if want[key] is None:
                assert got[key] is None
            else:
                assert got[key] == pytest.approx(want[key], abs=1e-6), key
        assert got["clamped"] == want["clamped"] and got["valid"] == want["valid"]
    assert result.summary["clamped_scans"] == 1
    assert os.path.exists(result.files["csv"])
