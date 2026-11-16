"""L1 tests of the v0.5 session tools in tools/pnp_data: the run finder for bags that mix PnP and
other drivers, the dynamics split of the 2026-11-15 session (one rule for every run, a golden
file, byte-stable output, thresholds read from the registered yaml) and the block map."""

from __future__ import annotations

import json
import sys
from pathlib import Path

import numpy as np
import pytest

pytest.importorskip("racer_eval")
pytest.importorskip("mcap_ros2")

sys.path.insert(0, str(Path(__file__).resolve().parents[1]))
sys.path.insert(0, str(Path(__file__).resolve().parent))

import yaml
from pnp_data import cli, runs, session_map_v05, session_split_v05
from pnp_data.runs import RunRow
from pnp_data.session_split import SplitRefused
from pnp_data.session_split_v05 import Part, SessionRun
from test_pnp_data import write_bag

REPO = Path(__file__).resolve().parents[2]
FIX = Path(__file__).resolve().parent / "fixtures"
RULE_YAML = REPO / session_split_v05.RULE_RELATIVE

BAG_A = "2026-11-16T22-45-39_car_teleop"
BAG_B = "2026-11-16T23-04-09_car_teleop"


def rule() -> session_split_v05.Rule:
    return session_split_v05.load_rule(RULE_YAML)


# -- the registered rule ---------------------------------------------------------------------------


def test_the_rule_is_read_from_the_registered_yaml() -> None:
    r = rule()
    assert (r.min_run_s, r.min_wheel_distance_m, r.train_share, r.guard_s) == (15.0, 1.0, 0.55, 2.0)
    assert r.sha256 == session_split_v05.RULE_SHA256
    doc = yaml.safe_load(RULE_YAML.read_text())["session_2026_11_15"]
    assert doc["heldout"] == "the rest of the run"
    assert doc["file"] == f"evaluation/prereg/sessions/{session_split_v05.FILE_NAME}"


def test_another_hash_is_refused(tmp_path: Path) -> None:
    path = tmp_path / "v05.yaml"
    path.write_text(RULE_YAML.read_text() + "# edited\n")
    with pytest.raises(SplitRefused, match="not the registered"):
        session_split_v05.load_rule(path)


# -- runs of a bag that mixes drivers --------------------------------------------------------------


def mixed_arrays():
    """PnP status 10 to 60 s (commands from 12 s), a gap follow stretch 100 to 130 s with no
    status, a 1 s stop and a second gap follow stretch 131.5 to 150 s (gap under 2 s: one run), a
    stray command inside the PnP stretch."""
    draw_t, draw_v, draw_s = [], [], []
    for t in np.arange(12.0, 60.0, 0.1):
        draw_t.append(t), draw_v.append(1.0), draw_s.append(0.2)
    for lo, hi in ((100.0, 130.0), (131.5, 150.0)):
        for t in np.arange(lo, hi, 0.1):
            draw_t.append(t), draw_v.append(0.6), draw_s.append(-0.2)
    status = np.arange(10.0, 60.0, 0.1)
    odom_t = np.arange(0.0, 160.0, 0.1)
    odom_v = np.where(
        ((odom_t >= 12) & (odom_t < 60)) | ((odom_t >= 100) & (odom_t < 150)), 0.8, 0.0
    )
    return draw_t, draw_v, draw_s, odom_t, odom_v, status


def test_run_rows_all_finds_the_other_drivers_the_status_does_not_cover() -> None:
    rows = runs.run_rows_all(*mixed_arrays())
    assert [(r.run_id, r.kind) for r in rows] == [
        ("R0", "pnp"),
        ("R1", "drive_raw"),
    ]
    pnp, gap = rows
    assert (pnp.start_s, pnp.direction) == (pytest.approx(12.0), "ccw")
    assert (gap.start_s, gap.end_s, gap.direction) == (
        pytest.approx(100.0),
        pytest.approx(149.9),
        "cw",
    )
    assert gap.wheel_distance_m == pytest.approx(0.8 * 49.9, abs=1.0)
    # the v0.4 finder sees the PnP run only: that is why the v0.5 tool has its own
    assert [r.kind for r in runs.run_rows(*mixed_arrays())] == ["pnp"]


def test_run_rows_all_is_run_rows_without_status_or_without_other_drivers() -> None:
    t = np.arange(0.0, 20.0, 0.1)
    args = (t, np.ones(len(t)), np.full(len(t), 0.1), t, np.ones(len(t)), [])
    assert runs.run_rows_all(*args) == runs.run_rows(*args)
    st = np.arange(0.0, 20.0, 0.1)
    with_status = (t, np.ones(len(t)), np.full(len(t), 0.1), t, np.ones(len(t)), st)
    assert runs.run_rows_all(*with_status) == runs.run_rows(*with_status)  # every command covered


def test_a_bag_with_no_drive_raw_has_no_runs_and_does_not_crash(tmp_path: Path) -> None:
    path = tmp_path / "2026-11-16T02-29-27_car_teleop.mcap"
    write_bag(path, [], [(t, 0.0) for t in np.arange(0.0, 5.0, 0.1)], [])
    sig = runs.read_run_signals(path)
    assert len(sig.draw_t) == 0 and len(sig.odom_t) == 50 and len(sig.status_t) == 0
    assert runs.rows_of_signals_all(sig) == []
    write_bag(path, [], [], [])
    assert len(runs.read_run_signals(path).odom_t) == 0


# -- the split ---------------------------------------------------------------------------------------


def run(
    rid: str, start: float, end: float, dist: float = 20.0, kind: str = "pnp", d: str = "ccw"
) -> RunRow:
    return RunRow(rid, kind, start, end, 1.0, d, dist, 1.1)


def sr(bag: str, *args, **kw) -> SessionRun:
    return SessionRun(bag, run(*args, **kw))


def table() -> list[SessionRun]:
    return [
        sr(BAG_A, "R0", 190.0, 193.7, dist=1.1),  # 3.7 s: shorter than 15 s
        sr(BAG_A, "R1", 262.0, 364.0),  # a PnP run
        sr(BAG_A, "R2", 400.0, 415.0, dist=2.0, kind="drive_raw"),  # exactly 15 s: used
        sr(BAG_A, "R3", 440.0, 480.0, dist=0.99, kind="drive_raw"),  # 0.99 m: unused
        sr(BAG_B, "R0", 70.0, 110.0, dist=1.0, d="cw"),  # exactly 1.0 m: used
        sr(BAG_B, "R1", 150.0, 164.9, dist=30.0, d="cw"),  # 14.9 s: shorter than 15 s
    ]


def test_every_run_by_the_one_rule_whatever_drove_it() -> None:
    parts = {p.name: p for p in session_split_v05.split_runs_v05(table(), rule())}
    assert parts["S1"].role == "unused" and "shorter than 15 s" in parts["S1"].why
    a, b = parts["S2a"], parts["S2b"]
    assert (a.t_from, a.t_to, a.role) == (262.0, pytest.approx(262.0 + 0.55 * 102.0), "train")
    assert (b.t_from, b.t_to, b.role) == (
        pytest.approx(262.0 + 0.55 * 102.0 + 2.0),
        364.0,
        "heldout",
    )
    # a run that is not a PnP run is cut the same way
    assert (parts["S3a"].role, parts["S3b"].role) == ("train", "heldout")
    assert parts["S3a"].t_to == pytest.approx(400.0 + 0.55 * 15.0)
    assert parts["S4"].role == "unused" and "under 1 m of wheel distance" in parts["S4"].why
    assert (parts["S5a"].role, parts["S5b"].role) == ("train", "heldout")
    assert parts["S6"].role == "unused"
    # no run is lost: every run is in the table once or twice
    assert {p.run for p in parts.values()} == {"R0", "R1", "R2", "R3"}


def test_the_boundaries_and_the_guard_follow_the_rule_numbers() -> None:
    r = session_split_v05.Rule(15.0, 1.0, 0.25, 4.0, "x")  # other numbers, same code
    parts = session_split_v05.split_runs_v05([sr(BAG_A, "R0", 100.0, 140.0)], r)
    assert parts[0].t_to == pytest.approx(110.0) and parts[1].t_from == pytest.approx(114.0)


def test_the_session_file_is_byte_stable_and_matches_the_golden() -> None:
    parts = session_split_v05.split_runs_v05(table(), rule())
    first = {BAG_A: 1794869142.979, BAG_B: 1794870253.533}
    text = session_split_v05.render_v05(parts, rule(), first)
    assert text == session_split_v05.render_v05(parts, rule(), first)
    golden = FIX / "pnp_data_session_v05_golden.yaml"
    assert text == golden.read_text()
    doc = yaml.safe_load(text)
    assert doc["rule_file_sha256"] == session_split_v05.RULE_SHA256
    assert doc["session"] == "2026-11-15"
    seg = doc["bags"][BAG_A]["segments"][1]
    assert seg["direction"] == "by_heading" and seg["command_direction"] == "ccw"
    assert seg["dynamics"] == "train" and seg["kind"] == "pnp"


def test_the_file_parses_with_the_dynamics_split_reader() -> None:
    """The v0.2 split parser the dynamics package uses must read the file as it is written."""
    sys.path.insert(0, str(REPO / "training" / "racer_pnp_dynamics" / "src"))
    parts = session_split_v05.split_runs_v05(table(), rule())
    text = session_split_v05.render_v05(parts, rule(), {BAG_A: 1.0, BAG_B: 2.0})
    doc = yaml.safe_load(text)
    roles = [s["dynamics"] for e in doc["bags"].values() for s in e["segments"]]
    assert set(roles) == {"train", "heldout", "unused"}
    for entry in doc["bags"].values():
        for seg in entry["segments"]:
            assert seg["from"] < seg["to"] and {"name", "dynamics"} <= set(seg)


def test_write_refuses_old_bags_and_never_overwrites(tmp_path: Path) -> None:
    old = [sr("2026-11-14T22-07-50_car_teleop", "R0", 0.0, 60.0)]
    with pytest.raises(SplitRefused, match="split by hand and frozen"):
        session_split_v05.write_split_v05(old, rule(), {}, out_dir=tmp_path)
    path, sha = session_split_v05.write_split_v05(
        table(), rule(), {BAG_A: 1.0, BAG_B: 2.0}, out_dir=tmp_path
    )
    assert path.name == "2026-11-15-dynamics-v05.yaml"
    assert session_split_v05.sha256_text(path.read_text()) == sha
    with pytest.raises(SplitRefused, match="never overwritten"):
        session_split_v05.write_split_v05(table(), rule(), {}, out_dir=tmp_path)


def test_bag_names_are_checked() -> None:
    session_split_v05.check_bag_names([BAG_A])
    with pytest.raises(SplitRefused, match="not a bag name"):
        session_split_v05.check_bag_names(["latest"])


# -- the block map ---------------------------------------------------------------------------------


def test_launch_interval_reads_ros_time_and_ignores_the_rest() -> None:
    text = (
        "[INFO] [launch]: no time here\n"
        "[WARN] [1794869400.356165825] [rcl]: x\n"
        "[mppi_planner_node-2] [INFO] [1794869403.505598475] [mppi_planner_node]: y\n"
        "pid [5202] and [1234.5]\n"
    )
    assert session_map_v05.launch_interval(text) == (1794869400.356165825, 1794869403.505598475)
    assert session_map_v05.launch_interval("nothing\n") is None


def test_blocks_are_read_from_a_directory_and_runs_mapped_to_the_best_overlap(
    tmp_path: Path,
) -> None:
    (tmp_path / "P_ccw.launch.log").write_text(
        "[INFO] [100.5]\n[INFO] [1000000100.0]\n[1000000200.0]"
    )
    (tmp_path / "G_ccw_1.0.launch.log").write_text("[1000000250.0]\n[1000000300.0]\n")
    (tmp_path / "empty.launch.log").write_text("no ros time\n")
    (tmp_path / "other.txt").write_text("[1000000250.0]")
    blocks = session_map_v05.read_blocks(tmp_path)
    assert sorted(blocks) == ["G_ccw_1.0", "P_ccw"]
    rows = [
        run("R0", 5.0, 8.0),  # an aborted launch: no log
        run("R1", 110.0, 190.0),
        run("R2", 260.0, 290.0, kind="drive_raw"),
    ]
    mapped = session_map_v05.map_runs(BAG_A, rows, 1000000000.0, blocks)
    assert [m.block for m in mapped] == ["unlogged", "P_ccw", "G_ccw_1.0"]
    assert mapped[1].epoch_start == pytest.approx(1000000110.0)
    text = session_map_v05.render_table(mapped)
    assert text.count("\n|") == 4 and "| P_ccw |" in text
    assert session_map_v05.overlap_s((0.0, 1.0), (2.0, 3.0)) == 0.0


# -- the command line --------------------------------------------------------------------------------


@pytest.fixture
def bags(tmp_path_factory) -> list[Path]:
    root = tmp_path_factory.mktemp("session")
    out = []
    for name, lo in (
        ("2026-11-16T22-45-39_car_teleop", 10.0),
        ("2026-11-16T23-04-09_car_teleop", 20.0),
    ):
        bag = root / name
        bag.mkdir()
        t = np.arange(lo, lo + 60.0, 0.1)
        write_bag(
            bag / f"{name}_0.mcap",
            [(x, 1.0, 0.2) for x in t],
            [(x, 0.9) for x in np.arange(0.0, lo + 70.0, 0.1)],
            [],
        )
        out.append(bag)
    return out


def test_split_v05_and_runs_v05_and_map_v05_on_small_bags(
    bags: list[Path], tmp_path: Path, capsys: pytest.CaptureFixture[str]
) -> None:
    out_dir = tmp_path / "sessions"
    assert cli.main(["split-v05", *map(str, bags), "--out-dir", str(out_dir)]) == 0
    printed = capsys.readouterr().out
    assert "SHA-256" in printed
    doc = yaml.safe_load((out_dir / "2026-11-15-dynamics-v05.yaml").read_text())
    assert set(doc["bags"]) == {b.name for b in bags}
    roles = [s["dynamics"] for e in doc["bags"].values() for s in e["segments"]]
    assert roles == ["train", "heldout", "train", "heldout"]
    # a second write is refused with exit code 2
    assert cli.main(["split-v05", *map(str, bags), "--out-dir", str(out_dir)]) == 2
    assert cli.main(["runs-v05", str(bags[0])]) == 0
    assert "drive_raw" in capsys.readouterr().out
    assert cli.main(["runs-v05", str(bags[0]), "--json"]) == 0
    doc = json.loads(capsys.readouterr().out)
    assert doc["first_scan_log_time_s"] == pytest.approx(1_700_000_000.0)
    assert doc["runs"][0]["kind"] == "drive_raw" and doc["runs"][0]["duration_s"] > 50
    logs = tmp_path / "logs"
    logs.mkdir()
    (logs / "P_ccw.launch.log").write_text("[1700000010.0]\n[1700000070.0]\n")
    assert cli.main(["map-v05", *map(str, bags), "--logs", str(logs)]) == 0
    assert "P_ccw" in capsys.readouterr().out


def test_split_v05_refuses_a_bag_with_another_name(tmp_path: Path) -> None:
    bag = tmp_path / "latest"
    bag.mkdir()
    write_bag(bag / "x_0.mcap", [(1.0, 1.0, 0.1)], [(1.0, 1.0)], [])
    assert cli.main(["split-v05", str(bag), "--out-dir", str(tmp_path / "s")]) == 2


def test_a_part_is_a_plain_record() -> None:
    p = Part("S1a", BAG_A, "R0", "pnp", 1.0, 2.0, "ccw", "train", "why")
    assert p.role == "train" and p.kind == "pnp"
