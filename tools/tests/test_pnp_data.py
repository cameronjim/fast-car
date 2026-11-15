"""L1 tests of tools/pnp_data: the bag reader (a bag written here), the runs table, the split of a
new session by the registered rule (with a byte-stable golden), the guard and the confirm wrapper."""

from __future__ import annotations

import json
import subprocess
import sys
from pathlib import Path

import numpy as np
import pytest

pytest.importorskip("racer_eval")
pytest.importorskip("mcap_ros2")

sys.path.insert(0, str(Path(__file__).resolve().parents[1]))

from lapref.bag import read_signals
from pnp_data import cli, guard, mcapread, runs, session_split
from pnp_data import confirm as confirm_mod
from pnp_data.runs import RunRow
from pnp_data.session_split import SessionRun, SplitRefused

REPO = Path(__file__).resolve().parents[2]
FIX = Path(__file__).resolve().parent / "fixtures"
RULE_YAML = REPO / "evaluation" / "prereg" / "pnp-v0.4.yaml"

# -- a small bag written with mcap_ros2 -----------------------------------------------------------

HEADER_NESTED = (
    "================================================================================\n"
    "MSG: std_msgs/Header\nbuiltin_interfaces/Time stamp\nstring frame_id\n"
    "================================================================================\n"
    "MSG: builtin_interfaces/Time\nint32 sec\nuint32 nanosec\n"
)
DRIVE_DEF = (
    "std_msgs/Header header\nackermann_msgs/AckermannDrive drive\n"
    "================================================================================\n"
    "MSG: ackermann_msgs/AckermannDrive\n"
    "float32 steering_angle\nfloat32 steering_angle_velocity\nfloat32 speed\n"
    "float32 acceleration\nfloat32 jerk\n"
    "================================================================================\n"
    "MSG: std_msgs/Header\nbuiltin_interfaces/Time stamp\nstring frame_id\n"
    "================================================================================\n"
    "MSG: builtin_interfaces/Time\nint32 sec\nuint32 nanosec\n"
)
ODOM_DEF = (
    "geometry_msgs/TwistWithCovariance twist\n"
    "================================================================================\n"
    "MSG: geometry_msgs/TwistWithCovariance\ngeometry_msgs/Twist twist\n"
    "================================================================================\n"
    "MSG: geometry_msgs/Twist\ngeometry_msgs/Vector3 linear\ngeometry_msgs/Vector3 angular\n"
    "================================================================================\n"
    "MSG: geometry_msgs/Vector3\nfloat64 x\nfloat64 y\nfloat64 z\n"
)
STATUS_DEF = "std_msgs/Header header\nuint8 state\n" + HEADER_NESTED
SCAN_DEF = "int32 n\n"
NS = 1_000_000_000
T0_NS = 1_700_000_000 * NS


def write_bag(path: Path, drive_raw, odom, status, drive=(), scan_at=0.0, scan=True) -> None:
    """drive_raw / drive: (t, speed, steer); odom: (t, v); status: t. Times in bag seconds."""
    from mcap_ros2.writer import Writer

    def ns(t: float) -> int:
        return T0_NS + round(t * NS)

    with path.open("wb") as stream:
        w = Writer(stream, chunk_size=8192)
        s_scan = w.register_msgdef("sensor_msgs/msg/LaserScan", SCAN_DEF)
        s_drive = w.register_msgdef("ackermann_msgs/msg/AckermannDriveStamped", DRIVE_DEF)
        s_odom = w.register_msgdef("nav_msgs/msg/Odometry", ODOM_DEF)
        s_status = w.register_msgdef("racer_pnp_msgs/msg/PlannerStatus", STATUS_DEF)
        events: list[tuple[int, str, object, dict]] = []
        if scan:
            events.append((ns(scan_at), "/scan", s_scan, {"n": 1}))
            events.append((ns(scan_at + 5.0), "/scan", s_scan, {"n": 2}))
        stamp = {"stamp": {"sec": 0, "nanosec": 0}, "frame_id": ""}
        for topic, rows in (("/drive_raw", drive_raw), ("/drive", drive)):
            for t, v, steer in rows:
                msg = {
                    "header": stamp,
                    "drive": {
                        "steering_angle": steer,
                        "steering_angle_velocity": 0.0,
                        "speed": v,
                        "acceleration": 0.0,
                        "jerk": 0.0,
                    },
                }
                events.append((ns(t), topic, s_drive, msg))
        for t, v in odom:
            msg = {
                "twist": {
                    "twist": {
                        "linear": {"x": v, "y": 0.0, "z": 0.0},
                        "angular": {"x": 0.0, "y": 0.0, "z": 0.0},
                    }
                }
            }
            events.append((ns(t), "/odom/wheel", s_odom, msg))
        for t in status:
            events.append((ns(t), "/pnp/planner/status", s_status, {"header": stamp, "state": 1}))
        for t_ns, topic, schema, msg in sorted(events, key=lambda e: e[0]):
            w.write_message(topic, schema, msg, log_time=t_ns, publish_time=t_ns)
        w.finish()


def synthetic_bag_arrays():
    """Two PnP runs: R0 10 to 40 s ccw at cap 1.0, R1 50 to 80 s cw at cap 0.5."""
    draw, odom, status = [], [], []
    for lo, hi, cap, steer in ((10.0, 40.0, 1.0, 0.2), (50.0, 80.0, 0.5, -0.2)):
        for t in np.arange(lo, hi, 0.1):
            draw.append((t, cap, steer))
            status.append(t)
    for t in np.arange(0.0, 90.0, 0.1):
        in_run = 10.0 <= t < 40.0 or 50.0 <= t < 80.0
        odom.append((t, (1.0 if t < 40.0 else 0.5) if in_run else 0.0))
    return draw, odom, status


@pytest.fixture(scope="module")
def small_bag(tmp_path_factory) -> Path:
    path = tmp_path_factory.mktemp("bag") / "2026-11-20T10-00-00_car_teleop.mcap"
    draw, odom, status = synthetic_bag_arrays()
    write_bag(path, draw, odom, status, drive=[(t, v, s) for t, v, s in draw])
    return path


def test_read_columns_puts_times_on_the_scan_axis(small_bag: Path) -> None:
    t0, cols = mcapread.read_columns(
        small_bag, {"/odom/wheel": lambda m: (m.twist.twist.linear.x,)}
    )
    assert t0 == pytest.approx(1_700_000_000.0)
    odom = cols["/odom/wheel"]
    assert odom[0, 0] == pytest.approx(0.0) and odom[1, 0] == pytest.approx(0.1)
    assert odom.shape == (900, 2)


def test_scan_is_never_decoded_and_only_wanted_topics_come_back(small_bag: Path) -> None:
    seen = list(mcapread.iter_messages(small_bag, ["/drive"]))
    topics = {topic for _, topic, _ in seen}
    assert topics == {"/scan", "/drive"}
    scans = [m for m in seen if m[1] == "/scan"]
    assert len(scans) == 1 and scans[0][2] is None


def test_a_bag_without_scan_has_no_axis(tmp_path: Path) -> None:
    path = tmp_path / "noscan.mcap"
    draw, odom, status = synthetic_bag_arrays()
    write_bag(path, draw, odom, status, scan=False)
    with pytest.raises(ValueError, match="no /scan"):
        mcapread.read_columns(path, {})


def test_a_directory_bag_and_a_missing_bag(small_bag: Path, tmp_path: Path) -> None:
    assert mcapread.bag_files(small_bag.parent) == [small_bag]
    with pytest.raises(FileNotFoundError):
        mcapread.bag_files(tmp_path)


def test_a_truncated_file_is_read_as_far_as_it_goes(small_bag: Path, tmp_path: Path) -> None:
    cut = tmp_path / "cut.mcap"
    data = small_bag.read_bytes()
    cut.write_bytes(data[: int(len(data) * 0.6)])
    _t0, cols = mcapread.read_columns(cut, {"/odom/wheel": lambda m: (m.twist.twist.linear.x,)})
    assert 0 < len(cols["/odom/wheel"]) < 900


def test_lapref_reads_steering_and_wheels(small_bag: Path) -> None:
    sig = read_signals(small_bag)
    assert len(sig.steer_t) == 600 and len(sig.odom_t) == 900
    assert sig.steer[0] == pytest.approx(0.2)
    assert sig.odom_v.max() == pytest.approx(1.0)


def test_run_signals_roundtrip_and_cache(small_bag: Path, tmp_path: Path) -> None:
    sig = runs.read_run_signals(small_bag)
    assert len(sig.status_t) == 600
    sig.save(tmp_path / "s.npz")
    back = runs.RunSignals.load(tmp_path / "s.npz")
    assert np.array_equal(back.draw_t, sig.draw_t)
    assert back.first_scan_log_s == pytest.approx(1_700_000_000.0)


# -- runs ------------------------------------------------------------------------------------------


def test_run_rows_of_the_synthetic_bag(small_bag: Path) -> None:
    rows = runs.rows_of_signals(runs.read_run_signals(small_bag))
    assert [r.kind for r in rows] == ["pnp", "pnp"]
    a, b = rows
    assert (a.start_s, a.end_s) == (pytest.approx(10.0), pytest.approx(39.9))
    assert (a.direction, a.cap_mps, b.direction, b.cap_mps) == ("ccw", 1.0, "cw", 0.5)
    assert a.wheel_distance_m == pytest.approx(29.9, abs=0.3)
    assert a.max_wheel_mps == pytest.approx(1.0)
    assert a.duration_s == pytest.approx(29.9)
    d = a.to_dict()
    assert d["cap_mps"] == 1.0 and d["duration_s"] == pytest.approx(29.9)
    text = runs.format_rows(rows)
    assert "R0" in text and "ccw" in text and "cw" in text


def test_a_bag_without_status_gets_command_runs() -> None:
    t = np.arange(0.0, 20.0, 0.1)
    rows = runs.run_rows(t, np.ones(len(t)), np.full(len(t), 0.1), t, np.ones(len(t)), [])
    assert [(r.kind, r.start_s) for r in rows] == [("drive_raw", 0.0)]


def test_a_status_stretch_without_commands_and_zero_distance() -> None:
    st = np.arange(5.0, 25.0, 0.1)
    rows = runs.run_rows([], [], [], [0.0, 30.0], [0.0, 0.0], st)
    assert rows[0].start_s == pytest.approx(5.0) and rows[0].p99_cmd_mps == 0.0
    assert rows[0].wheel_distance_m == 0.0 and rows[0].max_wheel_mps == 0.0


def test_distance_ignores_gaps_and_short_input() -> None:
    assert runs._distance(np.array([0.0]), np.array([1.0])) == 0.0
    assert runs._distance(np.array([0.0, 5.0]), np.array([1.0, 1.0])) == 0.0


CUT_RUNS = FIX / "pnp_data_l0_cut.npz"
L0 = (
    124.9,
    239.9,
)  # pnp-v0.4.yaml split_2026_11_14, bag 2026-11-14T21-49-21_car_teleop, segment L0


@pytest.mark.skipif(not CUT_RUNS.is_file(), reason="the L0 cut fixture is not committed yet")
def test_runs_reproduce_the_yaml_boundaries_of_l0() -> None:
    sig = runs.RunSignals.load(CUT_RUNS)
    rows = runs.rows_of_signals(sig)
    assert len(rows) == 1
    r = rows[0]
    assert r.start_s == pytest.approx(L0[0], abs=0.1)
    assert r.end_s == pytest.approx(L0[1], abs=0.1)
    assert (r.direction, r.cap_mps) == ("ccw", 1.0)
    assert 100.0 < r.wheel_distance_m < 130.0


# -- the split of a new session --------------------------------------------------------------------


def row(
    rid: str, start: float, end: float, cap: float, direction: str, dist: float = 50.0
) -> RunRow:
    return RunRow(rid, "pnp", start, end, cap, direction, dist, cap + 0.1)


def sr(bag: str, *args, **kw) -> SessionRun:
    return SessionRun(bag, row(*args, **kw))


BAG_A = "2026-11-20T10-00-00_car_teleop"
BAG_B = "2026-11-20T10-30-00_car_teleop"


def table() -> list[SessionRun]:
    """Three speeds by two directions: cells with one run, with two runs and with three; and a
    short run that the rule drops."""
    return [
        sr(BAG_A, "R0", 10.0, 80.0, 1.0, "ccw"),  # the only run of (1.0, ccw)
        sr(BAG_A, "R1", 100.0, 110.0, 1.0, "cw"),  # short
        sr(BAG_A, "R2", 130.0, 200.0, 0.5, "cw"),  # first of two
        sr(BAG_A, "R3", 220.0, 290.0, 0.5, "cw"),  # last of two: held out
        sr(BAG_B, "R0", 5.0, 45.0, 0.7, "ccw"),  # first of three
        sr(BAG_B, "R1", 60.0, 100.0, 0.7, "ccw"),  # second of three
        sr(BAG_B, "R2", 120.0, 160.0, 0.7, "ccw"),  # last of three
        sr(BAG_B, "R3", 180.0, 240.0, 1.3, "cw"),  # the only run of (1.3, cw)
        sr(BAG_B, "R4", 260.0, 330.0, 0.7, "cw"),  # the only run of (0.7, cw)
    ]


def test_roles_by_the_cut_rule() -> None:
    segs = {s.name: s for s in session_split.split_runs(table())}
    # one-run cell: first 55 percent, a 2.0 s guard, the rest
    a, b = segs["S1a"], segs["S1b"]
    assert (a.t_from, a.t_to, a.role) == (10.0, pytest.approx(10.0 + 0.55 * 70.0), "train")
    assert (b.t_from, b.t_to, b.role) == (pytest.approx(10.0 + 0.55 * 70.0 + 2.0), 80.0, "heldout")
    assert segs["S2"].role == "unused" and "shorter" in segs["S2"].why
    assert (segs["S3"].role, segs["S4"].role) == ("train", "heldout")
    assert [segs[n].role for n in ("S5", "S6", "S7")] == ["train", "train", "heldout"]
    assert (segs["S8a"].role, segs["S8b"].role) == ("train", "heldout")
    assert (segs["S9a"].role, segs["S9b"].role) == ("train", "heldout")
    assert "S1" not in segs and "S8" not in segs  # cut runs are named a and b


def test_the_rule_constants_are_the_registered_ones() -> None:
    import yaml

    if not RULE_YAML.is_file():
        pytest.skip("pnp-v0.4.yaml is not in this checkout yet")
    doc = yaml.safe_load(RULE_YAML.read_text())["future_sessions"]
    assert doc["cut_rule"]["single_run_train_share"] == session_split.SINGLE_RUN_TRAIN_SHARE
    assert doc["cut_rule"]["guard_s"] == session_split.GUARD_S
    assert doc["min_run_s"] == session_split.MIN_RUN_S
    assert doc["cut_rule"]["multi_run_cell_heldout"] == "last_run_of_the_cell"


def test_speed_cell_is_the_cap_rounded_to_a_tenth() -> None:
    runs_ = [
        sr(BAG_A, "R0", 0.0, 60.0, 0.96, "ccw"),
        sr(BAG_A, "R1", 70.0, 130.0, 1.04, "ccw"),
    ]
    segs = session_split.split_runs(runs_)
    assert [s.role for s in segs] == ["train", "heldout"]  # both are the (1.0, ccw) cell
    assert segs[0].cap_mps == 1.0


def test_an_undriven_run_is_refused_unless_named() -> None:
    runs_ = [
        sr(BAG_A, "R0", 0.0, 60.0, 1.0, "ccw", dist=0.0),
        sr(BAG_A, "R1", 70.0, 130.0, 1.0, "ccw"),
    ]
    with pytest.raises(SplitRefused, match="wheels did not turn"):
        session_split.split_runs(runs_)
    segs = session_split.split_runs(runs_, [f"{BAG_A}:R0"])
    assert segs[0].role == "unused" and "owner" in segs[0].why
    assert [s.name for s in segs[1:]] == ["S2a", "S2b"]  # R1 is now the only run of its cell


def test_the_session_file_is_byte_stable() -> None:
    segs = session_split.split_runs(table())
    text = session_split.render("2026-11-20", segs, "ab" * 32, {BAG_A: 1.5, BAG_B: 2.25})
    golden = FIX / "pnp_data_session_split_golden.yaml"
    assert text == golden.read_text()
    assert session_split.render("2026-11-20", segs, "ab" * 32, {BAG_A: 1.5, BAG_B: 2.25}) == text


def test_the_golden_parses_to_the_same_roles() -> None:
    import yaml

    doc = yaml.safe_load((FIX / "pnp_data_session_split_golden.yaml").read_text())
    roles = {s["name"]: s["dynamics"] for bag in doc["bags"].values() for s in bag["segments"]}
    assert roles["S1a"] == "train" and roles["S1b"] == "heldout" and roles["S2"] == "unused"
    assert all(
        s["dynamics"] == s["perception"] for b in doc["bags"].values() for s in b["segments"]
    )


def test_write_split_refuses_old_bags_overwrite_and_bad_sessions(tmp_path: Path) -> None:
    ok = [sr(BAG_A, "R0", 10.0, 80.0, 1.0, "ccw")]
    path, sha = session_split.write_split("2026-11-20", ok, "cd" * 32, {BAG_A: 1.0}, tmp_path)
    assert path == tmp_path / "2026-11-20-split.yaml" and len(sha) == 64
    assert sha == session_split.sha256_text(path.read_text())
    with pytest.raises(SplitRefused, match="never overwritten"):
        session_split.write_split("2026-11-20", ok, "cd" * 32, {BAG_A: 1.0}, tmp_path)
    old = [sr("2026-11-14T22-07-50_car_teleop", "R0", 10.0, 80.0, 1.0, "ccw")]
    with pytest.raises(SplitRefused, match="split by hand"):
        session_split.write_split("2026-11-21", old, "cd" * 32, {}, tmp_path)
    with pytest.raises(SplitRefused, match="not after"):
        session_split.write_split("2026-11-14", ok, "cd" * 32, {}, tmp_path)
    with pytest.raises(SplitRefused, match="not a date"):
        session_split.write_split("tomorrow", ok, "cd" * 32, {}, tmp_path)
    with pytest.raises(SplitRefused, match="start with a date"):
        session_split.bag_date("no_date_bag")
    assert len(list(tmp_path.iterdir())) == 1  # nothing written by the refusals


# -- the guard -------------------------------------------------------------------------------------


def git(cwd: Path, *args: str) -> None:
    subprocess.run(
        ["git", "-c", "user.name=t", "-c", "user.email=t@example.com", *args],
        cwd=cwd,
        check=True,
        capture_output=True,
    )


def session_file(dir_: Path, bags: list[str], date: str = "2026-11-20") -> Path:
    dir_.mkdir(parents=True, exist_ok=True)
    path = dir_ / f"{date}-split.yaml"
    body = "".join(f"  {b}:\n    segments: []\n" for b in bags)
    path.write_text(f"session: {date}\nbags:\n{body}")
    return path


def test_guard_passes_old_bags_without_a_file(tmp_path: Path) -> None:
    for name in ("2026-11-14T22-07-50_car_teleop", "2026-11-13T22-00-40_car_teleop"):
        assert guard.require_split(name, tmp_path) is None
    assert guard.require_split(f"/data/bags/{'2026-11-14T23-59-59_car_teleop'}", tmp_path) is None


def test_guard_refuses_a_new_bag_without_a_split(tmp_path: Path) -> None:
    with pytest.raises(guard.NoCommittedSplit, match="no split file lists it"):
        guard.require_split(BAG_A, tmp_path / "sessions")
    session_file(tmp_path / "sessions", [BAG_B])
    with pytest.raises(guard.NoCommittedSplit, match="no split file lists it"):
        guard.require_split(BAG_A, tmp_path / "sessions", require_committed=False)


def test_guard_needs_the_file_to_be_committed(tmp_path: Path) -> None:
    repo = tmp_path / "repo"
    sessions = repo / "evaluation" / "prereg" / "sessions"
    path = session_file(sessions, [BAG_A])
    git(repo, "init", "-q")
    with pytest.raises(guard.NoCommittedSplit, match="not committed"):
        guard.require_split(BAG_A, sessions)
    assert guard.require_split(BAG_A, sessions, require_committed=False) == path
    git(repo, "add", ".")
    with pytest.raises(guard.NoCommittedSplit, match="not committed"):  # staged is not committed
        guard.require_split(BAG_A, sessions)
    git(repo, "commit", "-q", "-m", "split")
    assert guard.require_split(BAG_A, sessions) == path
    assert guard.require_split(f"/x/{BAG_A}", sessions) == path
    with pytest.raises(guard.NoCommittedSplit):
        guard.require_split(BAG_B, sessions)


def test_guard_outside_a_repo_is_not_committed(tmp_path: Path) -> None:
    session_file(tmp_path / "s", [BAG_A])
    with pytest.raises(guard.NoCommittedSplit, match="not committed"):
        guard.require_split(BAG_A, tmp_path / "s")


def test_the_committed_sessions_dir_has_no_stray_file() -> None:
    # every committed split file is listed by the guard and parses
    for f in guard.split_files():
        import yaml

        assert "bags" in yaml.safe_load(f.read_text())


# -- confirm ---------------------------------------------------------------------------------------


def test_confirm_without_a_candidate_is_skipped(capsys, tmp_path: Path) -> None:
    called = []
    assert confirm_mod.confirm(tmp_path, None, None, runner=lambda c: called.append(c) or 0) == 0
    assert "confirm skipped" in capsys.readouterr().out and called == []
    assert confirm_mod.confirm(tmp_path, tmp_path, None, runner=lambda c: 5) == 0


def test_confirm_builds_the_dynamics_command(tmp_path: Path) -> None:
    bag, cand, dep = tmp_path / "bag", tmp_path / "c", tmp_path / "d"
    bag.mkdir()
    got = []
    assert confirm_mod.confirm(bag, cand, dep, runner=lambda c: got.append(list(c)) or 7) == 7
    cmd = got[0]
    assert cmd[:4] == ["uv", "run", "--project", str(confirm_mod.PROJECT)]
    assert cmd[4:9] == ["python", "-m", "racer_pnp_dynamics.cli", "confirm", "--bag"]
    assert cmd[-4:] == ["--candidate", str(cand), "--deployed", str(dep)]
    assert confirm_mod.PROJECT.name == "racer_pnp_dynamics" and confirm_mod.PROJECT.is_dir()


def test_confirm_reports_a_missing_bag(capsys, tmp_path: Path) -> None:
    assert confirm_mod.confirm(tmp_path / "nope", tmp_path, tmp_path, runner=lambda c: 0) == 2
    assert "no such bag" in capsys.readouterr().out


# -- command line ----------------------------------------------------------------------------------


def test_cli_runs(small_bag: Path, capsys, tmp_path: Path) -> None:
    assert cli.main(["runs", str(small_bag), "--cache", str(tmp_path)]) == 0
    assert "ccw" in capsys.readouterr().out
    assert (tmp_path / f"{small_bag.name}.pnp-data-runs.npz").is_file()
    assert cli.main(["runs", str(small_bag), "--json", "--cache", str(tmp_path)]) == 0  # cached
    assert json.loads(capsys.readouterr().out)[1]["direction"] == "cw"


@pytest.mark.skipif(not RULE_YAML.is_file(), reason="pnp-v0.4.yaml is not in this checkout yet")
def test_cli_split_writes_once_and_prints_the_sha(small_bag: Path, capsys, tmp_path: Path) -> None:
    out = tmp_path / "sessions"
    argv = ["split", str(small_bag), "--session", "2026-11-20", "--out-dir", str(out)]
    assert cli.main(argv) == 0
    printed = capsys.readouterr().out
    written = out / "2026-11-20-split.yaml"
    assert "SHA-256 " + session_split.sha256_text(written.read_text()) in printed
    assert cli.main(argv) == 2  # never overwritten
    assert "refused" in capsys.readouterr().err


def test_cli_split_refuses_a_changed_rule_file(small_bag: Path, capsys, tmp_path: Path) -> None:
    fake = tmp_path / "rule.yaml"
    fake.write_text("prereg_id: other\n")
    argv = [
        "split",
        str(small_bag),
        "--session",
        "2026-11-20",
        "--rule-file",
        str(fake),
        "--out-dir",
        str(tmp_path / "o"),
    ]
    assert cli.main(argv) == 2
    assert "not the registered" in capsys.readouterr().err
    assert not (tmp_path / "o").exists()


def test_cli_guard_exit_codes(tmp_path: Path, capsys) -> None:
    assert (
        cli.main(["guard", "2026-11-13T22-00-40_car_teleop", "--sessions-dir", str(tmp_path)]) == 0
    )
    assert "hand split" in capsys.readouterr().out
    assert cli.main(["guard", BAG_A, "--sessions-dir", str(tmp_path)]) == 1
    assert "REFUSED" in capsys.readouterr().out


def test_cli_confirm_skips(capsys, tmp_path: Path) -> None:
    assert cli.main(["confirm", "--bag", str(tmp_path)]) == 0
    assert "skipped" in capsys.readouterr().out


def test_entry_script_runs() -> None:
    script = REPO / "tools" / "pnp_data.py"
    out = subprocess.run(
        [sys.executable, str(script), "--help"], capture_output=True, text=True, check=False
    )
    assert out.returncode == 0 and "split" in out.stdout


def test_cli_split_in_the_dev_loop_uses_session_runs(small_bag: Path, tmp_path: Path) -> None:
    # the two runs of the synthetic bag are different cells, so each is cut
    rows = runs.rows_of_signals(runs.read_run_signals(small_bag))
    segs = session_split.split_runs([SessionRun(small_bag.name, r) for r in rows])
    assert [s.name for s in segs] == ["S1a", "S1b", "S2a", "S2b"]
