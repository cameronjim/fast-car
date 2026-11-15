"""L1 for the data roles of v0.4: the frozen constants file, segment roles, the guards, and the
held-out truth cache that stage 1 cannot open."""

from __future__ import annotations

import hashlib
from pathlib import Path

import numpy as np
import pytest
import yaml
from racer_pnp_dynamics import split_v04 as s4
from racer_pnp_dynamics import truth_v04 as t4
from racer_pnp_dynamics.icp_motion import MotionSeries
from racer_pnp_dynamics.split import Split, SplitInvalid
from racer_pnp_dynamics.vehicle import repo_root

DOC = {
    "prereg_id": "pnp-v0.4-test",
    "split_2026_11_14": {
        "bags": {
            "2026-11-14T21-36-40_car_teleop": {
                "whole_bag": {"dynamics": "unused", "perception": "unused", "lap": "unused"}
            },
            "2026-11-14T22-07-50_car_teleop": {
                "first_scan_log_time_s": 1000.0,
                "segments": [
                    {
                        "name": "X1",
                        "from": 10.0,
                        "to": 60.0,
                        "direction": "cw",
                        "cap_mps": 1.0,
                        "dynamics": "train",
                    },
                    {
                        "name": "X2b",
                        "from": 62.0,
                        "to": 90.0,
                        "direction": "cw",
                        "cap_mps": 0.5,
                        "dynamics": "heldout",
                    },
                ],
            },
        }
    },
}


@pytest.fixture
def yaml_file(tmp_path):
    path = tmp_path / "pnp-v0.4.yaml"
    path.write_text(yaml.safe_dump(DOC), encoding="utf-8")
    return path, hashlib.sha256(path.read_bytes()).hexdigest()


def test_the_real_constants_file_hashes_to_the_pre_registered_value():
    path = repo_root() / s4.V04_RELATIVE
    if not path.is_file():
        pytest.skip("evaluation/prereg/pnp-v0.4.yaml is not in this checkout (PR 186)")
    split = s4.load_split_v04(path)
    assert split.sha256 == s4.V04_SHA256
    names = {s.name for e in split.bags.values() for s in e.segments}
    assert {"K1", "K2", "L0", "X0", "X1", "X2a", "X2b", "X7b"} <= names
    assert s4.heldout_segment_names(split) == {"K1", "K2", "X2b", "X3b", "X4b", "X5b", "X6b", "X7b"}
    assert not s4.heldout_segment_names(split) & s4.train_segment_names(split)


def test_a_file_with_another_hash_is_refused(yaml_file):
    path, _ = yaml_file
    with pytest.raises(SplitInvalid, match="not the pre-registered"):
        s4.load_split_v04(path)  # the default expected hash is the real one


def test_roles_are_read_per_segment(yaml_file):
    path, digest = yaml_file
    split = s4.load_split_v04(path, digest)
    bag = "2026-11-14T22-07-50_car_teleop"
    assert s4.role_or_raise(split, bag, 20.0, 40.0) == "train"
    assert s4.role_or_raise(split, bag, 70.0, 80.0) == "heldout"
    assert s4.heldout_segment_names(split) == {"X2b"} and s4.train_segment_names(split) == {"X1"}
    assert split.role_of("2026-11-14T21-36-40_car_teleop", 0.0, 5.0) == "unused"


@pytest.mark.parametrize(
    ("t0", "t1"),
    [(0.0, 5.0), (55.0, 70.0), (60.5, 61.5), (85.0, 95.0), (200.0, 210.0)],
    ids=["before", "straddles train end", "the guard between", "straddles heldout end", "after"],
)
def test_a_time_outside_every_segment_is_refused(yaml_file, t0, t1):
    path, digest = yaml_file
    split = s4.load_split_v04(path, digest)
    with pytest.raises(s4.UnsplitData, match="outside every segment"):
        s4.role_or_raise(split, "2026-11-14T22-07-50_car_teleop", t0, t1)


def test_a_bag_the_files_do_not_name_is_refused(yaml_file):
    path, digest = yaml_file
    split = s4.load_split_v04(path, digest)
    with pytest.raises(s4.UnsplitData):
        s4.role_or_raise(split, "2026-11-14T23-59-59_car_teleop", 0.0, 1.0)


def test_overlapping_segments_are_refused(tmp_path):
    doc = yaml.safe_load(yaml.safe_dump(DOC))
    doc["split_2026_11_14"]["bags"]["2026-11-14T22-07-50_car_teleop"]["segments"][1]["from"] = 50.0
    path = tmp_path / "x.yaml"
    path.write_text(yaml.safe_dump(doc), encoding="utf-8")
    with pytest.raises(SplitInvalid, match="overlap"):
        s4.load_split_v04(path, hashlib.sha256(path.read_bytes()).hexdigest())


def test_combined_split_holds_both_files_and_refuses_a_shared_bag(yaml_file):
    path, digest = yaml_file
    v04 = s4.load_split_v04(path, digest)
    v02 = Split("v02", "a" * 64, {"2026-11-12T02-57-50_car_teleop": next(iter(v04.bags.values()))})
    both = s4.combined_split(v02, v04)
    assert set(both.bags) == set(v02.bags) | set(v04.bags)
    assert both.sha256 == f"{'a' * 64}+{digest}"
    with pytest.raises(SplitInvalid, match="both split files"):
        s4.combined_split(v04, v04)


def test_require_split_refuses_a_bag_newer_than_2026_11_14_without_a_committed_split(tmp_path):
    s4.require_split("2026-11-14T22-07-50_car_teleop", tmp_path)  # covered by the v0.4 file
    s4.require_split("2026-11-12T02-57-50_car_teleop", tmp_path)
    with pytest.raises(s4.UnsplitData, match="not committed"):
        s4.require_split("2026-11-15T10-00-00_car_teleop", tmp_path)
    sessions = tmp_path / s4.SESSIONS_RELATIVE
    sessions.mkdir(parents=True)
    (sessions / "2026-11-15-split.yaml").write_text("x: 1\n", encoding="utf-8")
    s4.require_split("2026-11-15T10-00-00_car_teleop", tmp_path)
    with pytest.raises(s4.UnsplitData):
        s4.require_split("2026-11-16T10-00-00_car_teleop", tmp_path)
    with pytest.raises(ValueError, match="no leading date"):
        s4.bag_date("teleop_bag")


# ---- truth_v04: the held-out cache --------------------------------------------------------


def _motion(n: int, t0: float = 0.0, ok: bool = True) -> MotionSeries:
    t = t0 + 0.1 * np.arange(n)
    return MotionSeries(
        t_s=t, dt_s=np.full(n, 0.1), dx_m=np.full(n, 0.05), dy_m=np.zeros(n),
        dyaw_rad=np.full(n, 0.01), rms_m=np.full(n, 0.01), inlier_share=np.ones(n),
        ok=np.full(n, ok), seeded=np.ones(n, dtype=bool),
    )  # fmt: skip


def test_stage_one_cannot_load_a_heldout_segment(tmp_path):
    t4.save_slice(t4.heldout_path(tmp_path, "bag", "X2b"), t4.Slice(0, _motion(5)))
    t4.save_slice(t4.train_path(tmp_path, "bag", "X1"), t4.Slice(0, _motion(5)))
    assert len(t4.load_stage1_motion(tmp_path, "bag", "X1", "train").motion.t_s) == 5
    with pytest.raises(t4.HeldoutLocked, match="train segments only"):
        t4.load_stage1_motion(tmp_path, "bag", "X2b", "heldout")
    with pytest.raises(t4.HeldoutLocked):
        t4.load_stage1_motion(tmp_path, "bag", "X2b", "unused")


def test_the_heldout_cache_is_a_sibling_directory_no_train_path_reaches(tmp_path):
    h = t4.heldout_path(tmp_path, "bag", "X2b")
    tr = t4.train_path(tmp_path, "bag", "X2b")
    assert t4.TRAIN_DIR not in h.parts and t4.HELDOUT_DIR in h.parts
    assert tr.parent != h.parent and tr.parent not in h.parents and h.parent not in tr.parents
    assert not str(h).startswith(str(tr.parent))
    # a stage-1 style listing of the train directory never shows a held-out file
    t4.save_slice(h, t4.Slice(0, _motion(3)))
    t4.save_slice(tr, t4.Slice(0, _motion(3)))
    listed = {p.name for p in (tmp_path / t4.TRAIN_DIR).glob("*")}
    assert listed == {tr.name}
    assert (tmp_path / t4.HELDOUT_DIR).is_dir()


def test_the_heldout_reader_needs_a_freeze_commit_and_the_heldout_role(tmp_path):
    t4.save_slice(t4.heldout_path(tmp_path, "bag", "X2b"), t4.Slice(2, _motion(4)))
    with pytest.raises(t4.HeldoutLocked, match="FreezeToken"):
        t4.load_heldout_motion(tmp_path, "bag", "X2b", "heldout", "abc")  # type: ignore[arg-type]
    with pytest.raises(t4.HeldoutLocked, match="git SHA"):
        t4.freeze("main")
    token = t4.freeze("0123456789abcdef0123456789abcdef01234567")
    with pytest.raises(t4.HeldoutLocked, match="not heldout"):
        t4.load_heldout_motion(tmp_path, "bag", "X2b", "train", token)
    sl = t4.load_heldout_motion(tmp_path, "bag", "X2b", "heldout", token)
    assert sl.first == 2 and len(sl.motion.t_s) == 4 and sl.motion.seeded is not None


def test_segment_intervals_need_both_scans_inside():
    stamps = 100.0 + 0.1 * np.arange(101)
    i0, i1 = t4.segment_intervals(stamps, 102.0, 105.0)
    assert (i0, i1) == (20, 50)  # interval i is scans i and i + 1: the last one starts at 104.9
    assert t4.segment_intervals(stamps[:1], 0.0, 1.0) == (0, 0)
    i0, i1 = t4.segment_intervals(stamps, 500.0, 600.0)
    assert i1 - i0 == 0


def test_assemble_puts_slices_in_the_run_and_marks_the_rest_not_ok():
    stamps = 10.0 + 0.1 * np.arange(41)
    a = t4.Slice(5, _motion(10, 10.5))
    b = t4.Slice(20, _motion(5, 12.0, ok=False))
    m = t4.assemble(stamps, [a, b])
    assert len(m.t_s) == 40 and m.ok[5:15].all() and not m.ok[:5].any() and not m.ok[15:].any()
    assert not m.ok[20:25].any() and np.isnan(m.dx_m[0]) and m.dx_m[7] == pytest.approx(0.05)
    assert m.seeded[5:15].all() and not m.seeded[:5].any()
    with pytest.raises(ValueError, match="same intervals"):
        t4.assemble(stamps, [a, t4.Slice(10, _motion(2, 11.0))])
    with pytest.raises(ValueError, match="outside the run"):
        t4.assemble(stamps, [t4.Slice(35, _motion(10))])


def test_a_stage_one_module_never_names_the_heldout_cache():
    """No stage 1 module may import the held-out reader or spell its directory."""
    src = Path(__file__).resolve().parents[1] / "src" / "racer_pnp_dynamics"
    for name in ("budget.py", "refit_v04.py"):
        path = src / name
        if path.is_file():
            text = path.read_text(encoding="utf-8")
            assert "load_heldout_motion" not in text and "HELDOUT_DIR" not in text
