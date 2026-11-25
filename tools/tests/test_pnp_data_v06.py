"""L1 tests of `split-v06` (brief 54): the v0.5 rule on the run list of the 2026-11-22 bag, numbers
read from the registered v0.6 yaml (and checked against the v0.5 rule they copy), the registered
bag and run list only, never an overwrite, the SHA-256 printed."""

from __future__ import annotations

import sys
from pathlib import Path
from types import SimpleNamespace

import pytest

pytest.importorskip("racer_eval")
pytest.importorskip("mcap_ros2")

sys.path.insert(0, str(Path(__file__).resolve().parents[1]))
sys.path.insert(0, str(Path(__file__).resolve().parent))

import yaml
from pnp_data import cli, session_split_v05, session_split_v06
from pnp_data.runs import RunRow
from pnp_data.session_split import SplitRefused
from pnp_data.session_split_v05 import SessionRun

REPO = Path(__file__).resolve().parents[2]
RULE_YAML = REPO / session_split_v06.RULE_RELATIVE
BAG = "2026-11-23T03-50-32_car_teleop"


def rule() -> session_split_v06.RuleV06:
    return session_split_v06.load_rule_v06(RULE_YAML)


def built_runs(r: session_split_v06.RuleV06, extra: tuple = ()) -> list[SessionRun]:
    """The registered run list as the run finder would give it (rounded times)."""
    out = []
    for rid, w in r.run_list.items():
        start, end = float(w["start_s"]) + 0.02, float(w["end_s"]) - 0.03
        row = RunRow(rid, str(w["kind"]), start, end, 1.0, str(w["direction"]), 30.0, 1.1)
        out.append(SessionRun(BAG, row))
    return out + list(extra)


def test_the_rule_is_the_v05_rule_read_from_the_registered_yaml() -> None:
    r = rule()
    v05 = session_split_v05.load_rule(REPO / session_split_v05.RULE_RELATIVE)
    assert r.rule.sha256 == session_split_v06.RULE_SHA256
    for name in ("min_run_s", "min_wheel_distance_m", "train_share", "guard_s"):
        assert getattr(r.rule, name) == getattr(v05, name)
    assert r.bag == BAG
    assert r.file == f"evaluation/prereg/sessions/{session_split_v06.FILE_NAME}"
    assert sorted(r.run_list) == ["R0", "R1", "R2", "R3", "R4"]


def test_another_hash_is_refused(tmp_path: Path) -> None:
    path = tmp_path / "x.yaml"
    path.write_bytes(RULE_YAML.read_bytes() + b"\n")
    with pytest.raises(SplitRefused, match="must not change"):
        session_split_v06.load_rule_v06(path)


def test_numbers_that_differ_from_the_v05_rule_are_refused(tmp_path: Path) -> None:
    text = RULE_YAML.read_text().replace("train_share: 0.55", "train_share: 0.60")
    path = tmp_path / "x.yaml"
    path.write_text(text)
    sha = session_split_v05.sha256_text(text)
    with pytest.raises(SplitRefused, match="not the v0.5 rule"):
        session_split_v06.load_rule_v06(
            path, REPO / session_split_v05.RULE_RELATIVE, expected_sha256=sha
        )


def test_the_rule_on_the_built_run_list() -> None:
    r = rule()
    parts = session_split_v05.split_runs_v05(built_runs(r), r.rule)
    assert [p.role for p in parts] == ["train", "heldout"] * 5
    r4a, r4b = parts[8], parts[9]
    assert r4a.t_to - r4a.t_from == pytest.approx(0.55 * (r4b.t_to - r4a.t_from), rel=1e-12)
    assert r4b.t_from - r4a.t_to == pytest.approx(2.0)
    text = session_split_v06.render_v06(parts, r.rule, {BAG: 1795405836.425764})
    assert text == session_split_v06.render_v06(parts, r.rule, {BAG: 1795405836.425764})
    doc = yaml.safe_load(text)
    assert doc["prereg_id"] == "pnp-v0.6-dynamics-yaw"
    assert doc["rule_file_sha256"] == session_split_v06.RULE_SHA256
    seg = doc["bags"][BAG]["segments"]
    assert [s["dynamics"] for s in seg] == ["train", "heldout"] * 5


def test_a_short_or_other_run_list_is_refused() -> None:
    r = rule()
    with pytest.raises(SplitRefused, match="registered run list"):
        session_split_v06.check_run_list(built_runs(r)[:2], r)
    late = built_runs(r)
    row = late[0].row
    late[0] = SessionRun(BAG, RunRow("R0", "pnp", row.start_s + 1.0, row.end_s, 1.0, "ccw", 3, 1))
    with pytest.raises(SplitRefused, match="start_s"):
        session_split_v06.check_run_list(late, r)
    flipped = built_runs(r)
    row = flipped[3].row
    flipped[3] = SessionRun(BAG, RunRow("R3", "pnp", row.start_s, row.end_s, 1.0, "ccw", 3, 1))
    with pytest.raises(SplitRefused, match="registered run"):
        session_split_v06.check_run_list(flipped, r)


def test_write_never_overwrites_and_refuses_another_bag(tmp_path: Path) -> None:
    r = rule()
    path, sha = session_split_v06.write_split_v06(built_runs(r), r, {BAG: 1.0}, out_dir=tmp_path)
    assert path.name == session_split_v06.FILE_NAME
    assert session_split_v05.sha256_text(path.read_text()) == sha
    with pytest.raises(SplitRefused, match="never overwritten"):
        session_split_v06.write_split_v06(built_runs(r), r, {BAG: 1.0}, out_dir=tmp_path)
    other = [SessionRun("2026-11-24T04-36-54_car_teleop", x.row) for x in built_runs(r)]
    with pytest.raises(SplitRefused, match="not the registered bag"):
        session_split_v06.write_split_v06(other, r, {}, out_dir=tmp_path / "o")


def test_check_bag_refuses_another_name_and_the_short_copy(tmp_path: Path) -> None:
    r = rule()
    session_split_v06.check_bag(tmp_path / BAG, r)
    with pytest.raises(SplitRefused, match="not the registered bag"):
        session_split_v06.check_bag(tmp_path / "2026-11-24T04-36-54_car_teleop", r)
    with pytest.raises(SplitRefused, match="short copy"):
        session_split_v06.check_bag(Path(r.not_this_copy), r)


def test_split_v06_command_prints_the_sha_and_refuses_a_second_write(
    tmp_path: Path, monkeypatch: pytest.MonkeyPatch, capsys: pytest.CaptureFixture[str]
) -> None:
    r = rule()
    rows = [x.row for x in built_runs(r)]
    monkeypatch.setattr(
        cli, "load_signals", lambda bag, cache: SimpleNamespace(first_scan_log_s=7.0)
    )
    monkeypatch.setattr(cli.runs, "rows_of_signals_all", lambda sig: rows)
    out = tmp_path / "sessions"
    assert cli.main(["split-v06", str(tmp_path / BAG), "--out-dir", str(out)]) == 0
    printed = capsys.readouterr().out
    assert "SHA-256" in printed
    sha = printed.split("SHA-256 ")[1].strip()
    assert sha == session_split_v05.sha256_text((out / session_split_v06.FILE_NAME).read_text())
    assert cli.main(["split-v06", str(tmp_path / BAG), "--out-dir", str(out)]) == 2
    assert cli.main(["split-v06", str(tmp_path / "latest"), "--out-dir", str(out)]) == 2
