"""L1 tests of racer_eval.split: every refusal, role_of on inside, straddling and outside
intervals, and the committed pre-registered split file."""

from __future__ import annotations

from pathlib import Path

import pytest
from racer_eval.split import SplitError, file_sha256, load_split, role_of

SPLIT_FILE = Path(__file__).resolve().parents[2] / "prereg" / "pnp-v0.2-split.yaml"
#: The SHA-256 the pre-registration quotes for the split file (brief 21, section 1).
SPLIT_SHA256 = "bc5185c912ece67a6177581f1b37adaaa9101b34ee1efe9881af7723681aef83"
DAY = (
    "2026-11-13T22-00-40_car_teleop",
    "2026-11-13T22-20-18_car_teleop",
    "2026-11-13T23-19-51_car_teleop",
    "2026-11-13T23-27-23_car_teleop",
)

SEG = "{{name: {name}, from: {t0}, to: {t1}, driver: pnp, direction: ccw, cap_mps: 0.5, {extra}}}"


def seg(
    name: str, t0: float, t1: float, extra: str = "perception: train, dynamics: heldout"
) -> str:
    return SEG.format(name=name, t0=t0, t1=t1, extra=extra)


def write(tmp_path: Path, body: str) -> Path:
    p = tmp_path / "split.yaml"
    p.write_text(f"split_id: t\nbags:\n{body}")
    return p


def two_segments(tmp_path: Path) -> Path:
    return write(
        tmp_path,
        "  bagA:\n    segments:\n"
        f"      - {seg('A', 10.0, 20.0)}\n"
        f"      - {seg('B', 22.0, 30.0, 'perception: heldout, dynamics: train')}\n"
        "  bagW:\n    whole_bag: {perception: heldout_layout, dynamics: legacy_70_30}\n",
    )


def test_role_of_inside_straddling_and_outside(tmp_path: Path) -> None:
    s = load_split(two_segments(tmp_path))
    assert s.role_of("bagA", 12.0, 18.0, "perception") == "train"
    assert s.role_of("bagA", 12.0, 18.0, "dynamics") == "heldout"
    assert s.role_of("bagA", 23.0, 29.0, "perception") == "heldout"
    # exactly the edges of a segment are inside it
    assert s.role_of("bagA", 10.0, 20.0, "perception") == "train"
    # straddles the edge of A, straddles the gap between A and B, lies in the gap, before, after
    assert s.role_of("bagA", 19.0, 20.5, "perception") is None
    assert s.role_of("bagA", 19.0, 23.0, "perception") is None
    assert s.role_of("bagA", 20.5, 21.5, "perception") is None
    assert s.role_of("bagA", 1.0, 5.0, "perception") is None
    assert s.role_of("bagA", 31.0, 32.0, "perception") is None
    # a bag that is not in the split has no role
    assert s.role_of("nobag", 1.0, 2.0, "perception") is None


def test_role_of_whole_bag_and_path_names(tmp_path: Path) -> None:
    s = load_split(two_segments(tmp_path))
    assert s.role_of("bagW", 0.0, 1e6, "perception") == "heldout_layout"
    assert s.role_of("/some/dir/bagW", 5.0, 6.0, "dynamics") == "legacy_70_30"
    assert role_of(s, "/x/bagA", 12.0, 13.0, "perception") == "train"


def test_role_of_refusals(tmp_path: Path) -> None:
    s = load_split(two_segments(tmp_path))
    with pytest.raises(ValueError):
        s.role_of("bagA", 5.0, 4.0, "perception")
    with pytest.raises(ValueError):
        s.role_of("bagA", 1.0, 2.0, "nonsense")
    with pytest.raises(ValueError):
        s.duration_s("nonsense", "train")


def test_touching_segments_are_allowed(tmp_path: Path) -> None:
    p = write(
        tmp_path,
        f"  b:\n    segments:\n      - {seg('A', 1.0, 2.0)}\n      - {seg('B', 2.0, 3.0)}\n",
    )
    assert len(load_split(p).segments) == 2


def test_refuses_sha_mismatch(tmp_path: Path) -> None:
    p = two_segments(tmp_path)
    with pytest.raises(SplitError, match="SHA-256"):
        load_split(p, expected_sha256="0" * 64)
    assert load_split(p, expected_sha256=file_sha256(p)).sha256 == file_sha256(p)


def test_refuses_overlap(tmp_path: Path) -> None:
    p = write(
        tmp_path,
        f"  b:\n    segments:\n      - {seg('A', 1.0, 5.0)}\n      - {seg('B', 4.0, 9.0)}\n",
    )
    with pytest.raises(SplitError, match="overlap"):
        load_split(p)


@pytest.mark.parametrize("t0,t1", [(5.0, 5.0), (5.0, 4.0)])
def test_refuses_to_not_after_from(tmp_path: Path, t0: float, t1: float) -> None:
    p = write(tmp_path, f"  b:\n    segments:\n      - {seg('A', t0, t1)}\n")
    with pytest.raises(SplitError, match="not after"):
        load_split(p)


def test_refuses_unknown_role(tmp_path: Path) -> None:
    bad = seg("A", 1.0, 2.0, "perception: gold, dynamics: train")
    with pytest.raises(SplitError, match="unknown role"):
        load_split(write(tmp_path, f"  b:\n    segments:\n      - {bad}\n"))
    with pytest.raises(SplitError, match="unknown role"):
        load_split(write(tmp_path, "  b:\n    whole_bag: {perception: gold, dynamics: train}\n"))


@pytest.mark.parametrize(
    "missing", ["name", "from", "to", "driver", "direction", "cap_mps", "perception", "dynamics"]
)
def test_refuses_missing_segment_field(tmp_path: Path, missing: str) -> None:
    fields = {
        "name": "A", "from": 1.0, "to": 2.0, "driver": "pnp", "direction": "ccw",
        "cap_mps": 0.5, "perception": "train", "dynamics": "train",
    }  # fmt: skip
    del fields[missing]
    body = "{" + ", ".join(f"{k}: {v}" for k, v in fields.items()) + "}"
    with pytest.raises(SplitError, match="missing field"):
        load_split(write(tmp_path, f"  b:\n    segments:\n      - {body}\n"))


def test_refuses_structural_errors(tmp_path: Path) -> None:
    with pytest.raises(SplitError, match="missing field"):
        load_split(write(tmp_path, "  b:\n    whole_bag: {perception: train}\n"))
    with pytest.raises(SplitError, match="exactly one"):
        load_split(write(tmp_path, "  b:\n    units: current\n"))
    with pytest.raises(SplitError, match="exactly one"):
        load_split(write(tmp_path, "  b:\n    segments: []\n    whole_bag: {}\n"))
    with pytest.raises(SplitError, match="not a mapping"):
        load_split(write(tmp_path, "  b: 3\n"))
    with pytest.raises(SplitError, match="not a mapping"):
        load_split(write(tmp_path, "  b:\n    segments:\n      - 3\n"))
    with pytest.raises(SplitError, match="numbers"):
        load_split(write(tmp_path, f"  b:\n    segments:\n      - {seg('A', 'x', 2.0)}\n"))
    p = tmp_path / "s.yaml"
    p.write_text("- 1\n")
    with pytest.raises(SplitError, match="not a mapping"):
        load_split(p)
    p.write_text("split_id: x\n")
    with pytest.raises(SplitError, match="missing field"):
        load_split(p)
    p.write_text("split_id: x\nbags: [1]\n")
    with pytest.raises(SplitError, match="bags is not a mapping"):
        load_split(p)


def test_committed_split_file_is_the_preregistered_one() -> None:
    assert file_sha256(SPLIT_FILE) == SPLIT_SHA256


def test_committed_split_loads_with_23_segments_and_the_stated_durations() -> None:
    s = load_split(SPLIT_FILE, expected_sha256=SPLIT_SHA256)
    day = [seg_ for seg_ in s.segments if seg_.bag in DAY]
    assert len(day) == 23
    assert {seg_.bag for seg_ in day} == set(DAY)
    for workstream in ("perception", "dynamics"):
        assert sum(x.duration_s for x in day if x.role(workstream) == "heldout") == pytest.approx(
            441.0, abs=1.0
        )
        assert sum(x.duration_s for x in day if x.role(workstream) == "train") == pytest.approx(
            1107.0, abs=1.0
        )
    assert s.duration_s("dynamics", "heldout") == pytest.approx(441.0, abs=1.0)


def test_committed_split_roles() -> None:
    s = load_split(SPLIT_FILE)
    bag = "2026-11-13T22-20-18_car_teleop"
    assert s.role_of(bag, 2730.0, 2780.0, "dynamics") == "heldout"  # inside R3
    assert s.role_of(bag, 2780.0, 2790.0, "dynamics") is None  # leaves R3
    assert s.role_of(bag, 2980.0, 2985.0, "perception") is None  # the guard between R7a and R7b
    assert s.role_of("2026-11-12T02-57-50_car_teleop", 3.0, 99.0, "perception") == "heldout_layout"
    assert s.role_of("2026-11-13T23-18-49_car_teleop", 3.0, 9.0, "dynamics") == "unused"
    assert len(s.segments_of("/data/" + bag)) == 11
