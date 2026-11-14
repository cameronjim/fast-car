"""The split loader: roles by time interval, the SHA check, and malformed files."""

from __future__ import annotations

import hashlib
from pathlib import Path

import pytest
from hypothesis import given
from hypothesis import strategies as st
from racer_pnp_perception import split as sp

YAML = """
split_id: test
bags:
  bagA:
    first_scan_log_time_s: 100.0
    segments:
      - {name: A1, from: 0.0, to: 10.0, direction: ccw, cap_mps: 0.5, perception: train, dynamics: train}
      - {name: A2, from: 12.0, to: 20.0, direction: cw, perception: heldout, dynamics: heldout}
  bagW:
    whole_bag: {perception: heldout_layout, dynamics: legacy_70_30}
  bagN:
    whole_bag: {perception: unused, dynamics: unused}
  bagT:
    whole_bag: {perception: train, dynamics: legacy_70_30}
"""


@pytest.fixture(scope="module")
def split() -> sp.Split:
    return sp.parse_split(YAML)


def test_inside_a_segment(split: sp.Split) -> None:
    assert split.role_of("bagA", 1.0, 9.0) == "train"
    assert split.role_of("bagA", 12.0, 20.0) == "heldout"  # the edges belong to the segment
    assert split.role_of("bagA", 5.0, 5.0) == "train"
    seg = split.locate("bagA", 13.0, 14.0)
    assert seg is not None and seg.name == "A2" and seg.direction == "cw"


def test_straddling_two_segments_or_an_edge_is_unused(split: sp.Split) -> None:
    assert split.role_of("bagA", 9.0, 13.0) == "unused"  # train into heldout across the guard
    assert split.role_of("bagA", 9.9, 10.1) == "unused"  # past the end of A1
    assert split.role_of("bagA", -0.1, 1.0) == "unused"  # before the start of A1
    assert split.role_of("bagA", 0.0, 20.0) == "unused"


def test_outside_every_segment_is_unused(split: sp.Split) -> None:
    assert split.role_of("bagA", 10.5, 11.5) == "unused"  # the guard
    assert split.role_of("bagA", 25.0, 26.0) == "unused"
    assert split.role_of("bagA", 5.0, 4.0) == "unused"  # not an interval


def test_whole_bag_has_one_role_everywhere(split: sp.Split) -> None:
    assert split.role_of("bagW", 0.0, 1e6) == "heldout_layout"
    assert split.role_of("bagT", -5.0, 5.0) == "train"
    assert split.role_of("bagN", 0.0, 1.0) == "unused"
    seg = split.locate("bagW", 3.0, 4.0)
    assert seg is not None and seg.name == sp.WHOLE_BAG


def test_unknown_bag_is_unused(split: sp.Split) -> None:
    assert split.role_of("nope", 0.0, 1.0) == "unused"
    assert split.locate("nope", 0.0, 1.0) is None


def test_bags_and_segments_by_role(split: sp.Split) -> None:
    assert split.bag_names_with_role("train") == ["bagA", "bagT"]
    assert split.bag_names_with_role("heldout") == ["bagA"]
    assert split.bag_names_with_role("heldout_layout") == ["bagW"]
    assert [s.name for _b, s in split.segments_with_role("train")] == ["A1"]


def test_load_split_checks_the_sha256(tmp_path: Path) -> None:
    path = tmp_path / "split.yaml"
    path.write_text(YAML)
    digest = hashlib.sha256(YAML.encode()).hexdigest()
    loaded = sp.load_split(path, expected_sha256=digest)
    assert loaded.sha256 == digest and loaded.role_of("bagA", 1.0, 2.0) == "train"
    with pytest.raises(sp.SplitError, match="pre-registered"):
        sp.load_split(path)  # the default digest is the pre-registered file's
    path.write_text(YAML + "\n# edited\n")
    with pytest.raises(sp.SplitError):
        sp.load_split(path, expected_sha256=digest)


def test_the_constant_is_the_one_in_rule_10() -> None:
    assert sp.SPLIT_SHA256 == "bc5185c912ece67a6177581f1b37adaaa9101b34ee1efe9881af7723681aef83"


def test_the_real_split_file_has_the_pre_registered_digest_and_loads() -> None:
    if not sp.DEFAULT_SPLIT_PATH.is_file():
        pytest.skip("the split file is not in this checkout")
    split = sp.load_split()  # raises unless the digest matches
    names = [s.name for _b, s in split.segments_with_role("heldout")]
    assert {"G1b", "G3b", "P0", "R3", "R4", "R5", "R6", "R7b", "R8", "T2b", "T3b"} <= set(names)
    assert split.bag_names_with_role("heldout_layout") == [
        "2026-11-12T02-57-50_car_teleop",
        "2026-11-12T03-04-56_car_teleop",
    ]


@pytest.mark.parametrize(
    "text",
    [
        "split_id: x\n",
        "bags:\n  b: {segments: [{name: s, from: 5, to: 5, perception: train}]}\n",
        "bags:\n  b: {segments: [{name: s, from: 0, to: 5, perception: bogus}]}\n",
        "bags:\n  b: {}\n",
        "bags:\n  b:\n    whole_bag: {perception: train}\n    segments: [{name: s, from: 0, to: 5, perception: train}]\n",
        "bags:\n  b:\n    segments:\n      - {name: s, from: 0, to: 5, perception: train}\n      - {name: t, from: 4, to: 8, perception: train}\n",
    ],
)
def test_malformed_files_are_refused(text: str) -> None:
    with pytest.raises(sp.SplitError):
        sp.parse_split(text)


@st.composite
def disjoint_segments(draw: st.DrawFn) -> list[tuple[float, float]]:
    cuts = sorted(
        draw(st.lists(st.integers(0, 1000), min_size=2, max_size=12, unique=True)),
    )
    # Pair consecutive cuts into segments, leaving a gap between pairs.
    return [(cuts[i] / 10.0, cuts[i + 1] / 10.0) for i in range(0, len(cuts) - 1, 2)]


@given(
    segs=disjoint_segments(),
    a=st.floats(-5.0, 105.0, allow_nan=False),
    b=st.floats(-5.0, 105.0, allow_nan=False),
)
def test_role_of_never_gives_a_role_to_an_interval_touching_two_segments(
    segs: list[tuple[float, float]], a: float, b: float
) -> None:
    lines = "\n".join(
        f"      - {{name: S{k}, from: {lo}, to: {hi}, perception: train}}"
        for k, (lo, hi) in enumerate(segs)
    )
    split = sp.parse_split(f"bags:\n  b:\n    segments:\n{lines}\n")
    lo_t, hi_t = min(a, b), max(a, b)
    touched = [k for k, (s, e) in enumerate(segs) if not (hi_t < s or lo_t > e)]
    inside = [k for k, (s, e) in enumerate(segs) if s <= lo_t and hi_t <= e]
    role = split.role_of("b", lo_t, hi_t)
    if len(touched) >= 2:
        assert role == "unused"
    assert (role == "train") == bool(inside)
