"""L4: the M-R comparison file (gate v1's laps on the 2026-11-13 bags, recorded at f45fd09) and the
decision logic of the M-R table (pre-registration pnp-v0.3 section 1.5). No bag is read here; the
table itself is produced by `python -m racer_laps.replay --m-r` and goes in the PR comment."""

from __future__ import annotations

import json
from pathlib import Path

import pytest

yaml = pytest.importorskip("yaml")
from racer_laps.replay import (
    M_R_POSE_SEGMENTS,
    MRRow,
    driven_segments,
    format_m_r,
)

HERE = Path(__file__).resolve().parent
V1_FILE = HERE / "fixtures" / "gate_v1_laps_2026-11-13.json"
SPLIT = HERE.parents[3] / "evaluation" / "prereg" / "pnp-v0.2-split.yaml"


def row(
    name: str = "X",
    v1: list[float] | None = None,
    v2: list[float] | None = None,
    pose: list[float] | None = None,
) -> MRRow:
    v1, v2 = v1 or [], v2 or []
    pose_times = pose or []
    return MRRow(
        segment=name,
        v1_laps=len(v1),
        v2_laps=len(v2),
        pose_laps=None if pose is None else len(pose),
        v1_times=v1,
        v2_times=v2,
        pose_times=pose_times,
        v2_minus_pose=[a - b for a, b in zip(v2, pose_times, strict=False)],
        v1_minus_pose=[a - b for a, b in zip(v1, pose_times, strict=False)],
    )


def test_the_v1_file_covers_every_driven_segment_and_is_consistent() -> None:
    data = json.loads(V1_FILE.read_text())
    assert data["commit"] == "f45fd09"
    split = yaml.safe_load(SPLIT.read_text())
    wanted = [name for _bag, name in driven_segments(split)]
    assert set(wanted) <= set(data["segments"])
    for name, seg in data["segments"].items():
        assert seg["laps"] == len(seg["lap_times_s"]) == len(seg["crossings_bag_s"]), name
        assert len(seg["lengths_m"]) == seg["laps"]
        assert all(t > 0.0 for t in seg["lap_times_s"])
    # a few numbers worth pinning: v1 closed 26 laps on G1a and 13 on T0
    assert data["segments"]["G1a"]["laps"] == 26
    assert data["segments"]["T0"]["laps"] == 13


def test_driven_segments_are_the_pnp_gap_follow_and_tracker_ones_plus_the_joined() -> None:
    split = yaml.safe_load(SPLIT.read_text())
    names = [name for _bag, name in driven_segments(split)]
    assert "G1a" in names and "R6" in names and "T3b" in names
    assert names[-1] == "T2a+T2b"
    assert len(names) == 4 + 2 + 11 + 6 + 1  # gap follow, race, pnp, tracker, joined
    assert set(M_R_POSE_SEGMENTS) <= set(names)


def test_counts_pass_when_v2_closes_as_many_and_fail_on_a_lost_lap() -> None:
    ok = row("A", v1=[10.0, 10.0], v2=[10.0, 10.0, 10.0])
    bad = row("B", v1=[10.0, 10.0, 10.0], v2=[10.0, 10.0])
    assert ok.qualifies and ok.lap_count_ok
    assert bad.qualifies and not bad.lap_count_ok
    text = format_m_r([ok, bad])
    assert "lap counts" in text and "FAIL B" in text


def test_a_segment_with_under_two_laps_is_not_judged() -> None:
    short = row("S", v1=[10.0], v2=[])
    assert not short.qualifies
    assert "PASS" in format_m_r([short]).splitlines()[-2]


def test_lap_times_are_judged_against_the_pose_detector_only_where_there_is_a_pose() -> None:
    good = row("G1a", v1=[16.3, 16.3], v2=[16.3, 16.31], pose=[16.4, 16.2])
    assert good.lap_times_ok is True
    off = row("T0", v1=[17.8, 17.1], v2=[17.8, 17.1], pose=[16.4, 17.1])
    assert off.lap_times_ok is False
    lost = row("T1", v1=[8.3, 8.3], v2=[8.3], pose=[8.3, 8.3])
    assert lost.lap_times_ok is False  # the pose detector has a lap v2 lacks
    none = row("R1", v1=[10.0, 10.0], v2=[10.0, 10.0])
    assert none.lap_times_ok is None
    text = format_m_r([good, off, none])
    assert text.splitlines()[-1].endswith("FAIL T0")
    exact = row("E", v1=[1.0, 1.0], v2=[10.29, 10.0], pose=[10.0, 10.0])
    assert exact.lap_times_ok is True  # 0.29 s is inside the tolerance


def test_the_table_has_one_row_per_segment() -> None:
    rows = [row("A", v1=[1.0, 1.0], v2=[1.0, 1.0]), row("B", v1=[1.0], v2=[1.0])]
    lines = format_m_r(rows).splitlines()
    assert lines[0].startswith("| segment")
    assert lines[2].startswith("| A ") and lines[3].startswith("| B ")
    assert "n/a (under 2 laps)" in lines[3]


# -- run_m_r and the CLI on a synthetic recording ------------------------------------------------


def _fake_split(tmp_path: Path) -> Path:
    split = {
        "bags": {
            "bagA": {
                "segments": [
                    {"name": "R1", "from": 0.0, "to": 70.0, "driver": "pnp"},
                    {"name": "G1a", "from": 0.0, "to": 70.0, "driver": "gap_follow"},
                    {"name": "U1", "from": 0.0, "to": 70.0, "driver": "teleop"},
                ]
            },
            "2026-11-13T23-27-23_car_teleop": {
                "segments": [
                    {"name": "T2a", "from": 0.0, "to": 40.0, "driver": "tracker"},
                    {"name": "T2b", "from": 40.0, "to": 70.0, "driver": "tracker"},
                ]
            },
        }
    }
    path = tmp_path / "split.yaml"
    path.write_text(yaml.safe_dump(split))
    return path


def _fake_v1(tmp_path: Path) -> Path:
    path = tmp_path / "v1.json"
    path.write_text(
        json.dumps(
            {
                "segments": {
                    "R1": {"lap_times_s": [9.7, 9.7, 9.7]},
                    "G1a": {"lap_times_s": [9.7, 9.7]},
                    "T2a": {"lap_times_s": [9.7]},
                    "T2b": {"lap_times_s": []},
                    "T2a+T2b": {"lap_times_s": [9.7, 9.7]},
                }
            }
        )
    )
    return path


def test_run_m_r_runs_gate_v2_on_every_driven_segment(
    tmp_path: Path, monkeypatch: pytest.MonkeyPatch
) -> None:
    from racer_laps import replay
    from test_replay_scan import VEHICLE, messages

    msgs = messages(3.3, standstill_s=0.5)
    monkeypatch.setattr(replay, "read_messages", lambda path: iter(msgs))
    rows = replay.run_m_r(_fake_split(tmp_path), tmp_path, _fake_v1(tmp_path), VEHICLE)
    by = {r.segment: r for r in rows}
    assert set(by) == {"R1", "G1a", "T2a", "T2b", "T2a+T2b"}  # the teleop segment is skipped
    assert by["R1"].v1_laps == 3 and by["R1"].v2_laps == 3 and by["R1"].pose_laps is None
    assert by["R1"].v2_times == pytest.approx([9.65] * 3, abs=0.3)
    assert by["R1"].lap_count_ok and by["R1"].lap_times_ok is None
    # G1a is a map-pose segment: the synthetic recording has no /tf, so the pose detector closes
    # nothing and the lap-time rule fails (the pose detector has laps v2 lacks, or none to compare)
    assert by["G1a"].pose_laps == 0
    assert by["G1a"].v2_start_bag_s is not None
    only = replay.run_m_r(_fake_split(tmp_path), tmp_path, _fake_v1(tmp_path), VEHICLE, ["R1"])
    assert [r.segment for r in only] == ["R1"]
    tail = replay.run_m_r(
        _fake_split(tmp_path), tmp_path, _fake_v1(tmp_path), VEHICLE, ["R1"], tail_s=5.0
    )
    assert tail[0].v2_laps == 3
    assert "M-R lap counts" in format_m_r(rows)


def test_the_m_r_cli(
    tmp_path: Path, monkeypatch: pytest.MonkeyPatch, capsys: pytest.CaptureFixture[str]
) -> None:
    from racer_laps import replay

    seen: dict = {}

    def fake(split, bags, v1, only=None, tail_s=0.0):  # type: ignore[no-untyped-def]
        seen.update(only=only, tail=tail_s)
        return [row("A", v1=[1.0, 1.0], v2=[1.0, 1.0])]

    monkeypatch.setattr(replay, "run_m_r", fake)
    split, v1 = _fake_split(tmp_path), _fake_v1(tmp_path)
    argv = ["--m-r", "--split", str(split), "--bags-root", str(tmp_path), "--v1-laps", str(v1)]
    assert replay.main([*argv, "--only", "A,B", "--tail-s", "1.5"]) == 0
    assert seen == {"only": ["A", "B"], "tail": 1.5}
    assert "| A |" in capsys.readouterr().out
    with pytest.raises(SystemExit):
        replay.main(["--m-r", "--split", str(split)])
    with pytest.raises(SystemExit):
        replay.main(["--lap-source", "start_scan"])  # no --bag
