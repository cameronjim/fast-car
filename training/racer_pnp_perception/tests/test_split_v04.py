"""L1: the perception roles of the 2026-11-14 bags. `reserve` is never loadable, and the heldout-only
split puts every reserve stretch outside every segment (so no scoring path admits it)."""

from __future__ import annotations

from pathlib import Path

import pytest
import yaml
from racer_pnp_perception import split_v04
from racer_pnp_perception.split import ROLE_HELDOUT, ROLE_UNUSED, SplitError

DOC_TEXT = """
prereg_id: pnp-v0.4
split_2026_11_14:
  bags:
    bagA:
      whole_bag: {dynamics: unused, perception: unused, lap: unused}
    bagB:
      first_scan_log_time_s: 100.0
      segments:
        - {name: K1, from: 10.0, to: 20.0, direction: ccw, cap_mps: 1.0, dynamics: heldout, perception: heldout, lap: test}
        - {name: K2, from: 30.0, to: 40.0, direction: cw, cap_mps: 1.0, dynamics: heldout, perception: heldout, lap: test}
    bagC:
      segments:
        - {name: X0, from: 0.0, to: 50.0, direction: ccw, cap_mps: 1.0, dynamics: train, perception: reserve, lap: dev}
        - {name: X2b, from: 52.0, to: 80.0, direction: cw, cap_mps: 0.5, dynamics: heldout, perception: heldout, lap: test}
"""


@pytest.fixture
def doc() -> dict:
    return yaml.safe_load(DOC_TEXT)


def test_only_heldout_segments_are_loaded(doc) -> None:
    s = split_v04.perception_split_from_doc(doc, "x")
    assert set(s.bags) == {"bagB", "bagC"}
    assert [seg.name for seg in s.bags["bagB"].segments] == ["K1", "K2"]
    assert [seg.name for seg in s.bags["bagC"].segments] == ["X2b"]
    assert s.bags["bagB"].segments[0].direction == "ccw"
    assert s.bags["bagB"].first_scan_log_time_s == 100.0


def test_a_reserve_stretch_is_outside_every_segment(doc) -> None:
    s = split_v04.perception_split_from_doc(doc)
    assert s.role_of("bagC", 10.0, 20.0) == ROLE_UNUSED  # inside X0, which is reserve
    assert s.locate("bagC", 10.0, 20.0) is None
    assert s.role_of("bagC", 60.0, 70.0) == ROLE_HELDOUT
    assert s.role_of("bagA", 0.0, 1.0) == ROLE_UNUSED  # unused whole bag is absent
    assert s.segments_with_role("reserve") == []


def test_asking_the_loader_for_reserve_is_refused(doc, tmp_path: Path) -> None:
    with pytest.raises(split_v04.ReserveSegmentRefused):
        split_v04.perception_split_from_doc(doc, roles=("reserve",))
    with pytest.raises(split_v04.ReserveSegmentRefused):
        split_v04.perception_split_from_doc(doc, roles=(ROLE_HELDOUT, "reserve"))
    with pytest.raises(split_v04.ReserveSegmentRefused):
        split_v04.perception_split(tmp_path / "missing.yaml", roles=("reserve",))
    with pytest.raises(SplitError):
        split_v04.perception_split_from_doc(doc, roles=("train",))


def test_the_guard_refuses_an_interval_touching_a_reserve_segment(doc) -> None:
    assert split_v04.reserve_intervals(doc) == [("bagC", 0.0, 50.0)]
    split_v04.assert_not_reserve(doc, "bagC", 60.0, 70.0)
    split_v04.assert_not_reserve(doc, "bagB", 10.0, 20.0)  # another bag's times
    with pytest.raises(split_v04.ReserveSegmentRefused):
        split_v04.assert_not_reserve(doc, "bagC", 49.0, 53.0)  # straddles the edge
    with pytest.raises(split_v04.ReserveSegmentRefused):
        split_v04.assert_not_reserve(doc, "bagC", 5.0, 5.0)


def test_a_file_with_another_digest_is_refused(tmp_path: Path) -> None:
    p = tmp_path / "pnp-v0.4.yaml"
    p.write_text(DOC_TEXT)
    with pytest.raises(SplitError, match="SHA-256"):
        split_v04.read_yaml(p)
    with pytest.raises(SplitError, match="SHA-256"):
        split_v04.perception_split(p)


def test_the_committed_file_loads_when_present() -> None:
    path = split_v04.default_path()
    if not path.is_file():
        pytest.skip("evaluation/prereg/pnp-v0.4.yaml is not on this branch yet (PR 186)")
    split = split_v04.perception_split()
    names = {seg.name for _b, seg in split.segments_with_role(ROLE_HELDOUT)}
    assert names == {"K1", "K2", "X2b", "X3b", "X4b", "X5b", "X6b", "X7b"}
    doc, _d = split_v04.read_yaml()
    for bag, lo, hi in split_v04.reserve_intervals(doc):
        assert split.role_of(bag, lo, hi) == ROLE_UNUSED
