"""L1/L2 for split.py: the pre-registered split file loader and `role_of`."""

from __future__ import annotations

import hashlib
from pathlib import Path

import pytest
from hypothesis import given, settings
from hypothesis import strategies as st
from racer_pnp_dynamics import split as split_mod
from racer_pnp_dynamics.split import SplitInvalid, load_split

YAML = """\
split_id: test-split
bags:
  bag-a:
    first_scan_log_time_s: 100.0
    units: current
    segments:
      - {name: A1, from: 10.0, to: 50.0, driver: pnp, direction: cw, cap_mps: 0.5, dynamics: train}
      - {name: A2, from: 52.0, to: 80.0, driver: pnp, direction: ccw, cap_mps: 0.5, dynamics: heldout}
      - {name: A3, from: 90.0, to: 95.0, driver: pnp, direction: ccw, dynamics: unused}
  bag-b:
    units: current
    whole_bag: {perception: unused, dynamics: legacy_70_30}
  bag-c:
    units: current
    whole_bag: {perception: unused, dynamics: unused}
"""


def _write(tmp_path: Path, text: str = YAML) -> tuple[Path, str]:
    path = tmp_path / "split.yaml"
    path.write_text(text, encoding="utf-8")
    return path, hashlib.sha256(text.encode()).hexdigest()


def test_the_pre_registered_hash_constant_is_the_one_of_the_preregistration():
    assert split_mod.SPLIT_SHA256 == (
        "bc5185c912ece67a6177581f1b37adaaa9101b34ee1efe9881af7723681aef83"
    )


def test_load_refuses_a_file_that_does_not_hash_to_the_constant(tmp_path):
    path, digest = _write(tmp_path)
    with pytest.raises(SplitInvalid, match="pre-registered"):
        load_split(path)  # the default constant is the real file's hash
    assert load_split(path, expected_sha256=digest).split_id == "test-split"
    path.write_text(YAML + "# edited\n", encoding="utf-8")
    with pytest.raises(SplitInvalid):
        load_split(path, expected_sha256=digest)


def test_role_of_needs_the_whole_interval_inside_one_segment(tmp_path):
    path, digest = _write(tmp_path)
    s = load_split(path, expected_sha256=digest)
    assert s.role_of("bag-a", 10.0, 50.0) == "train"  # the segment itself
    assert s.role_of("bag-a", 20.0, 30.0) == "train"
    assert s.role_of("bag-a", 60.0, 61.0) == "heldout"
    assert s.role_of("bag-a", 49.0, 53.0) is None  # straddles the guard between A1 and A2
    assert s.role_of("bag-a", 5.0, 15.0) is None  # starts before the first segment
    assert s.role_of("bag-a", 51.0, 51.5) is None  # inside the guard
    assert s.role_of("bag-a", 90.0, 91.0) == "unused"
    assert s.role_of("bag-b", 0.0, 1e6) == "legacy_70_30"
    assert s.role_of("bag-c", 0.0, 1.0) == "unused"
    assert s.role_of("no-such-bag", 0.0, 1.0) is None
    with pytest.raises(ValueError):
        s.role_of("bag-a", 5.0, 4.0)


def test_segment_metadata_and_whole_bag_entries(tmp_path):
    path, digest = _write(tmp_path)
    s = load_split(path, expected_sha256=digest)
    seg = s.segment_of("bag-a", 11.0, 12.0)
    assert (seg.name, seg.direction, seg.cap_mps, seg.driver) == ("A1", "cw", 0.5, "pnp")
    assert s.entry("bag-a").first_scan_log_time_s == 100.0
    assert s.entry("bag-b").segments == () and s.entry("bag-b").first_scan_log_time_s is None
    assert s.segment_of("bag-b", 0.0, 1.0) is None


@pytest.mark.parametrize(
    "text, why",
    [
        (
            "split_id: x\nbags:\n  b:\n    segments:\n      - {name: a, from: 5, to: 5, dynamics: train}\n",
            "empty",
        ),
        (
            "split_id: x\nbags:\n  b:\n    segments:\n      - {name: a, from: 1, to: 5, dynamics: bogus}\n",
            "unknown",
        ),
        ("split_id: x\nbags:\n  b:\n    whole_bag: {dynamics: bogus}\n", "unknown"),
        ("split_id: x\nbags:\n  b:\n    units: current\n", "neither"),
        (
            (
                "split_id: x\nbags:\n  b:\n    segments:\n"
                "      - {name: a, from: 1, to: 5, dynamics: train}\n"
                "      - {name: b, from: 4, to: 9, dynamics: train}\n"
            ),
            "overlap",
        ),
    ],
)
def test_malformed_files_are_refused(tmp_path, text, why):
    path, digest = _write(tmp_path, text)
    with pytest.raises(SplitInvalid, match=why):
        load_split(path, expected_sha256=digest)


REAL = Path("/Users/cameronjim/code/car/evaluation/prereg/pnp-v0.2-split.yaml")


@pytest.mark.skipif(
    not REAL.is_file(), reason="the pre-registered split file is not on this machine"
)
def test_the_real_split_file_loads_with_its_hash_and_has_the_listed_segments():
    s = load_split(REAL)
    assert s.sha256 == split_mod.SPLIT_SHA256
    runs = s.entry("2026-11-13T22-20-18_car_teleop")
    names = {x.name: x.role for x in runs.segments}
    assert names["R3"] == "heldout" and names["R7a"] == "train" and names["R7b"] == "heldout"
    assert s.role_of("2026-11-13T22-20-18_car_teleop", 2730.0, 2780.0) == "heldout"
    assert s.role_of("2026-11-13T22-20-18_car_teleop", 2980.0, 2983.0) is None  # R7 guard
    assert s.role_of("2026-11-13T23-18-49_car_teleop", 0.0, 10.0) == "unused"
    assert s.role_of("2026-11-12T03-04-56_car_teleop", 0.0, 10.0) == "legacy_70_30"


@settings(max_examples=200, deadline=None)
@given(
    a=st.floats(0.0, 120.0, allow_nan=False),
    b=st.floats(0.0, 120.0, allow_nan=False),
)
def test_role_of_never_answers_for_an_interval_that_straddles_segments(a, b, tmp_path_factory):
    path, digest = _write(tmp_path_factory.mktemp("split"))
    s = load_split(path, expected_sha256=digest)
    lo, hi = min(a, b), max(a, b)
    role = s.role_of("bag-a", lo, hi)
    inside = [x for x in s.entry("bag-a").segments if x.t_from_s <= lo and hi <= x.t_to_s]
    if role is not None:
        assert len(inside) == 1 and inside[0].role == role
    else:
        assert inside == []
    # an interval that touches two segments has no role
    for first, second in zip(
        s.entry("bag-a").segments, s.entry("bag-a").segments[1:], strict=False
    ):
        if lo <= first.t_to_s <= hi and lo < second.t_from_s <= hi and lo < first.t_to_s:
            assert role is None
