"""Load and validate the pre-registered data split (evaluation/prereg/pnp-v0.2-split.yaml).

Reference loader for the split. `role_of` implements rule 4 of pre-registration section 1: a
sample belongs to a role only if every message it was built from lies inside ONE segment, so an
interval that straddles a segment edge, or lies outside every segment, has no role (None).

Segment intervals are closed: [t_from, t_to]. Two segments of a bag may touch at one instant but
not overlap. Times are bag seconds, the axis of the split file (seconds after the first /scan).
"""

from __future__ import annotations

import hashlib
from dataclasses import dataclass
from itertools import pairwise
from pathlib import Path
from typing import Any

import yaml

#: The roles a split entry may carry (header of the split file).
ROLES = ("train", "heldout", "heldout_layout", "legacy_70_30", "unused")
WORKSTREAMS = ("perception", "dynamics")
_SEGMENT_FIELDS = ("name", "from", "to", "driver", "direction", "cap_mps", *WORKSTREAMS)


class SplitError(ValueError):
    """The split file is not valid, or is not the file that was expected."""


@dataclass(frozen=True)
class Segment:
    bag: str
    name: str
    t_from: float
    t_to: float
    driver: str
    direction: str
    cap_mps: float
    perception: str
    dynamics: str

    @property
    def duration_s(self) -> float:
        return self.t_to - self.t_from

    def role(self, workstream: str) -> str:
        return self.perception if workstream == "perception" else self.dynamics


@dataclass(frozen=True)
class WholeBag:
    """A bag that has one role for its whole length (the older bags)."""

    bag: str
    perception: str
    dynamics: str

    def role(self, workstream: str) -> str:
        return self.perception if workstream == "perception" else self.dynamics


@dataclass(frozen=True)
class Split:
    split_id: str
    sha256: str
    segments: tuple[Segment, ...]
    whole_bags: tuple[WholeBag, ...]

    def segments_of(self, bag: str) -> tuple[Segment, ...]:
        key = _bag_key(bag)
        return tuple(s for s in self.segments if s.bag == key)

    def duration_s(self, workstream: str, role: str) -> float:
        """Seconds of segment time carrying `role` for `workstream` (segmented bags only)."""
        _check_workstream(workstream)
        return sum(s.duration_s for s in self.segments if s.role(workstream) == role)

    def role_of(self, bag: str, t_from: float, t_to: float, workstream: str) -> str | None:
        """The role of the interval [t_from, t_to] of `bag`, or None when it does not lie wholly
        inside one segment (straddles an edge, falls in a gap, or the bag is not in the split).
        A whole-bag entry gives its role to every interval of the bag."""
        _check_workstream(workstream)
        if t_to < t_from:
            raise ValueError(f"interval ends before it starts: {t_from} to {t_to}")
        key = _bag_key(bag)
        for whole in self.whole_bags:
            if whole.bag == key:
                return whole.role(workstream)
        for seg in self.segments:
            if seg.bag == key and seg.t_from <= t_from and t_to <= seg.t_to:
                return seg.role(workstream)
        return None


def _bag_key(bag: str | Path) -> str:
    return Path(str(bag)).name


def _check_workstream(workstream: str) -> None:
    if workstream not in WORKSTREAMS:
        raise ValueError(f"unknown workstream {workstream!r}; one of {WORKSTREAMS}")


def file_sha256(path: Path) -> str:
    return hashlib.sha256(Path(path).read_bytes()).hexdigest()


def _role(value: Any, where: str) -> str:
    if value not in ROLES:
        raise SplitError(f"{where}: unknown role {value!r}; one of {ROLES}")
    return str(value)


def _segment(bag: str, raw: Any) -> Segment:
    if not isinstance(raw, dict):
        raise SplitError(f"{bag}: a segment is not a mapping: {raw!r}")
    name = raw.get("name", "?")
    where = f"{bag} segment {name}"
    for field in _SEGMENT_FIELDS:
        if field not in raw:
            raise SplitError(f"{where}: missing field {field!r}")
    try:
        t_from, t_to, cap = float(raw["from"]), float(raw["to"]), float(raw["cap_mps"])
    except (TypeError, ValueError) as error:
        raise SplitError(f"{where}: from, to and cap_mps must be numbers") from error
    if t_to <= t_from:
        raise SplitError(f"{where}: to ({t_to}) is not after from ({t_from})")
    return Segment(
        bag=bag,
        name=str(name),
        t_from=t_from,
        t_to=t_to,
        driver=str(raw["driver"]),
        direction=str(raw["direction"]),
        cap_mps=cap,
        perception=_role(raw["perception"], where),
        dynamics=_role(raw["dynamics"], where),
    )


def _check_overlaps(bag: str, segments: list[Segment]) -> None:
    ordered = sorted(segments, key=lambda s: s.t_from)
    for a, b in pairwise(ordered):
        if b.t_from < a.t_to:
            raise SplitError(f"{bag}: segments {a.name} and {b.name} overlap")


def load_split(path: str | Path, expected_sha256: str | None = None) -> Split:
    """Parse and validate the split file. Raises SplitError on a SHA-256 mismatch (when one is
    given), an overlap inside a bag, `to <= from`, an unknown role or a missing field."""
    path = Path(path)
    digest = file_sha256(path)
    if expected_sha256 is not None and digest != expected_sha256:
        raise SplitError(f"{path}: SHA-256 {digest} is not the expected {expected_sha256}")
    doc = yaml.safe_load(path.read_text())
    if not isinstance(doc, dict):
        raise SplitError(f"{path}: not a mapping")
    for field in ("split_id", "bags"):
        if field not in doc:
            raise SplitError(f"{path}: missing field {field!r}")
    if not isinstance(doc["bags"], dict):
        raise SplitError(f"{path}: bags is not a mapping")
    segments: list[Segment] = []
    wholes: list[WholeBag] = []
    for bag, entry in doc["bags"].items():
        if not isinstance(entry, dict):
            raise SplitError(f"{bag}: not a mapping")
        has_segments, has_whole = "segments" in entry, "whole_bag" in entry
        if has_segments == has_whole:
            raise SplitError(f"{bag}: needs exactly one of segments and whole_bag")
        if has_segments:
            parsed = [_segment(str(bag), raw) for raw in entry["segments"]]
            _check_overlaps(str(bag), parsed)
            segments.extend(parsed)
        else:
            whole = entry["whole_bag"]
            for field in WORKSTREAMS:
                if not isinstance(whole, dict) or field not in whole:
                    raise SplitError(f"{bag} whole_bag: missing field {field!r}")
            wholes.append(
                WholeBag(
                    str(bag),
                    _role(whole["perception"], f"{bag} whole_bag"),
                    _role(whole["dynamics"], f"{bag} whole_bag"),
                )
            )
    return Split(str(doc["split_id"]), digest, tuple(segments), tuple(wholes))


def role_of(split: Split, bag: str, t_from: float, t_to: float, workstream: str) -> str | None:
    """Module-level form of `Split.role_of`."""
    return split.role_of(bag, t_from, t_to, workstream)
