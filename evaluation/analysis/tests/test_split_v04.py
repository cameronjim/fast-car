"""L1 tests of racer_eval.split_v04: the SHA-256 refusal, the role of every segment of the
2026-11-14 split, the sealed perception reserve, the joined lap runs and `role_of`."""

from __future__ import annotations

from pathlib import Path

import pytest
import yaml
from racer_eval.split import SplitError
from racer_eval.split_v04 import (
    PREREG_V04_SHA256,
    ReserveIsSealed,
    UnknownBag,
    load_v04,
    parse_v04,
)

YAML = Path(__file__).resolve().parents[2] / "prereg" / "pnp-v0.4.yaml"
BAG = "2026-11-14T22-07-50_car_teleop"

#: (dynamics, perception, lap) of every segment, pre-registration v0.4 section 1.
EXPECTED = {
    "K1": ("heldout", "heldout", "test"),
    "K2": ("heldout", "heldout", "test"),
    "L0": ("train", "reserve", "dev"),
    "X0": ("train", "reserve", "dev"),
    "X1": ("train", "reserve", "dev"),
    "X2a": ("train", "reserve", "test"),
    "X2b": ("heldout", "heldout", "test"),
    "X3a": ("train", "reserve", "dev"),
    "X3b": ("heldout", "heldout", "dev"),
    "X4a": ("train", "reserve", "test"),
    "X4b": ("heldout", "heldout", "test"),
    "X5a": ("train", "reserve", "dev"),
    "X5b": ("heldout", "heldout", "dev"),
    "X6a": ("train", "reserve", "dev"),
    "X6b": ("heldout", "heldout", "dev"),
    "X7a": ("train", "reserve", "test"),
    "X7b": ("heldout", "heldout", "test"),
}


@pytest.fixture(scope="module")
def split():
    return load_v04(YAML)


def test_the_committed_file_has_the_registered_sha() -> None:
    assert load_v04(YAML).sha256 == PREREG_V04_SHA256


def test_sha_mismatch_refuses(tmp_path: Path) -> None:
    copy = tmp_path / "pnp-v0.4.yaml"
    copy.write_text(YAML.read_text() + "\n# one byte more\n")
    with pytest.raises(SplitError, match="SHA-256"):
        load_v04(copy)
    # the escape hatch exists only for synthetic files
    assert load_v04(copy, expected_sha256=None).sha256 != PREREG_V04_SHA256


def test_every_segment_has_its_role(split) -> None:
    got = {s.name: (s.dynamics, s.perception, s.lap) for s in split.segments_all}
    assert got == EXPECTED


def test_dynamics_and_perception_lists(split) -> None:
    heldout = {s.name for s in split.segments("dynamics", "heldout")}
    assert heldout == {"K1", "K2", "X2b", "X3b", "X4b", "X5b", "X6b", "X7b"}
    assert {s.name for s in split.segments("perception", "heldout")} == heldout
    train = {s.name for s in split.segments("dynamics", "train")}
    assert train == {"L0", "X0", "X1", "X2a", "X3a", "X4a", "X5a", "X6a", "X7a"}
    seg = split.segments("dynamics", "heldout")[0]
    assert (seg.bag, seg.name, seg.t_from, seg.t_to) == (
        "2026-11-14T21-37-23_car_teleop",
        "K1",
        235.8,
        306.9,
    )
    assert (seg.direction, seg.cap_mps) == ("ccw", 1.0)
    assert seg.duration_s == pytest.approx(71.1)


def test_reserve_is_sealed(split) -> None:
    with pytest.raises(ReserveIsSealed):
        split.segments("perception", "reserve")
    sealed = split.segments("perception", "reserve", i_am_the_split_tool=True)
    assert len(sealed) == 9


def test_unknown_role_or_workstream(split) -> None:
    with pytest.raises(ValueError, match="no role"):
        split.segments("dynamics", "reserve")
    with pytest.raises(ValueError, match="no role"):
        split.segments("perception", "train")
    with pytest.raises(ValueError, match="workstream"):
        split.segments("lap", "dev")
    with pytest.raises(ValueError, match="lap role"):
        split.lap_runs("train")


def test_lap_runs_join_their_parts(split) -> None:
    assert [r.name for r in split.lap_runs("dev")] == ["L0", "X0", "X1", "X3", "X5", "X6"]
    assert [r.name for r in split.lap_runs("test")] == ["K1", "K2", "X2", "X4", "X7"]
    x2 = split.lap_run("X2")
    assert (x2.parts, x2.t_from, x2.t_to) == (("X2a", "X2b"), 446.7, 517.1)
    assert (x2.bag, x2.direction, x2.cap_mps, x2.role) == (BAG, "cw", 0.5, "test")
    assert x2.duration_s == pytest.approx(70.4)
    assert split.lap_run("L0").parts == ("L0",)
    assert split.lap_run("X7").t_to == 1466.0
    with pytest.raises(KeyError):
        split.lap_run("nope")


def test_role_of(split) -> None:
    assert split.role_of(BAG, 250.0, 300.0, "dynamics") == "train"  # X0
    assert split.role_of(BAG, 500.0, 510.0, "dynamics") == "heldout"  # X2b
    assert split.role_of(BAG, 500.0, 510.0, "perception") == "heldout"
    assert split.role_of(BAG, 450.0, 460.0, "perception") == "reserve"  # X2a
    # straddles the edge between X2a and X2b, which are 2.0 s apart
    assert split.role_of(BAG, 480.0, 490.0, "dynamics") == "unused"
    # in the gap, before the first segment, after the last
    assert split.role_of(BAG, 486.0, 486.5, "dynamics") == "unused"
    assert split.role_of(BAG, 0.0, 10.0, "dynamics") == "unused"
    assert split.role_of(BAG, 2000.0, 2001.0, "perception") == "unused"
    # the edges are closed
    assert split.role_of(BAG, 246.5, 348.3, "dynamics") == "train"
    # whole bag of no PnP run
    assert split.role_of("2026-11-14T21-36-40_car_teleop", 1.0, 2.0, "dynamics") == "unused"
    # a full path works as a bag name
    assert split.role_of(f"/x/y/{BAG}", 250.0, 300.0, "dynamics") == "train"
    with pytest.raises(UnknownBag):
        split.role_of("2026-11-20T10-00-00_car_teleop", 0.0, 1.0, "dynamics")
    with pytest.raises(ValueError, match="before it starts"):
        split.role_of(BAG, 5.0, 4.0, "dynamics")
    with pytest.raises(ValueError, match="workstream"):
        split.role_of(BAG, 1.0, 2.0, "lap")


# -- validation of a synthetic file ---------------------------------------------------------------

SEG = {
    "name": "A",
    "from": 10.0,
    "to": 20.0,
    "direction": "ccw",
    "cap_mps": 1.0,
    "dynamics": "train",
    "perception": "reserve",
    "lap": "dev",
}


def doc(segments, lap_runs=None, whole=None):
    bags = {"bagA": {"segments": segments}} if segments is not None else {}
    if whole is not None:
        bags["bagW"] = {"whole_bag": whole}
    return {
        "split_2026_11_14": {
            "bags": bags,
            "lap_runs": lap_runs if lap_runs is not None else {"dev": ["A"], "test": []},
        }
    }


def seg(**kw):
    return {**SEG, **kw}


def test_synthetic_valid() -> None:
    s = parse_v04(doc([seg()]), "x")
    assert s.role_of("bagA", 12.0, 15.0, "dynamics") == "train"


@pytest.mark.parametrize(
    ("segments", "lap_runs", "message"),
    [
        ([seg(dynamics="reserve")], None, "unknown role"),
        ([seg(perception="train")], None, "unknown role"),
        ([seg(lap="maybe")], None, "unknown role"),
        ([seg(to=5.0)], None, "not after"),
        ([seg(to="x")], None, "must be numbers"),
        ([{k: v for k, v in SEG.items() if k != "lap"}], None, "missing field"),
        (["nope"], None, "not a mapping"),
        ([seg(), seg(name="B", **{"from": 15.0, "to": 30.0})], None, "overlap"),
        ([seg(), seg(**{"from": 30.0, "to": 40.0})], None, "not unique"),
        ([seg()], {"dev": ["Z"], "test": []}, "no segment"),
        ([seg(lap="test")], {"dev": ["A"], "test": []}, "do not carry"),
        ([seg(), seg(name="B", **{"from": 30.0, "to": 40.0})], None, "in no lap run"),
        ([seg(lap="unused")], None, "do not carry"),
    ],
)
def test_synthetic_refusals(segments, lap_runs, message) -> None:
    with pytest.raises(SplitError, match=message):
        parse_v04(doc(segments, lap_runs), "x")


def test_lap_run_part_rules() -> None:
    unused_listed = doc([seg(lap="unused")], {"dev": [], "test": []})
    assert parse_v04(unused_listed, "x").lap_runs("dev") == []
    parts = [
        seg(name="Ba", **{"from": 10.0, "to": 20.0}),
        seg(name="Bb", **{"from": 22.0, "to": 30.0}),
    ]
    ok = parse_v04(doc(parts, {"dev": ["B"], "test": []}), "x")
    assert ok.lap_run("B").parts == ("Ba", "Bb")
    far = [parts[0], seg(name="Bb", **{"from": 40.0, "to": 50.0})]
    with pytest.raises(SplitError, match="not consecutive"):
        parse_v04(doc(far, {"dev": ["B"], "test": []}), "x")
    turn = [parts[0], seg(name="Bb", direction="cw", **{"from": 22.0, "to": 30.0})]
    with pytest.raises(SplitError, match="direction"):
        parse_v04(doc(turn, {"dev": ["B"], "test": []}), "x")


def test_a_segment_with_role_unused_cannot_be_in_a_run() -> None:
    d = doc([seg(lap="unused")], {"dev": ["A"], "test": []})
    with pytest.raises(SplitError, match="do not carry"):
        parse_v04(d, "x")


def test_structure_refusals() -> None:
    with pytest.raises(SplitError, match="bags"):
        parse_v04({}, "x")
    with pytest.raises(SplitError, match="lap_runs"):
        parse_v04({"split_2026_11_14": {"bags": {"b": {"segments": []}}}}, "x")
    with pytest.raises(SplitError, match="not a mapping"):
        parse_v04({"split_2026_11_14": {"bags": {"b": 3}}}, "x")
    with pytest.raises(SplitError, match="exactly one"):
        parse_v04({"split_2026_11_14": {"bags": {"b": {}}}}, "x")
    with pytest.raises(SplitError, match="missing field"):
        parse_v04(doc(None, whole={"dynamics": "unused"}), "x")
    with pytest.raises(SplitError, match="unknown role"):
        parse_v04(
            doc(None, whole={"dynamics": "reserve", "perception": "unused", "lap": "unused"}), "x"
        )
    two_bags = doc([seg()])
    two_bags["split_2026_11_14"]["bags"]["bagB"] = {"segments": [seg(name="A")]}
    with pytest.raises(SplitError, match="not unique"):
        parse_v04(two_bags, "x")


def test_not_a_mapping_file(tmp_path: Path) -> None:
    p = tmp_path / "x.yaml"
    p.write_text("- 1\n- 2\n")
    with pytest.raises(SplitError, match="not a mapping"):
        load_v04(p, expected_sha256=None)


def test_a_multi_bag_run_is_refused() -> None:
    d = doc([seg(name="Ca")])
    d["split_2026_11_14"]["bags"]["bagB"] = {"segments": [seg(name="Cb", **{"from": 12.0})]}
    d["split_2026_11_14"]["lap_runs"] = {"dev": ["C"], "test": []}
    with pytest.raises(SplitError, match="several bags"):
        parse_v04(d, "x")


def test_yaml_id() -> None:
    assert yaml.safe_load(YAML.read_text())["prereg_id"] == "pnp-v0.4"
