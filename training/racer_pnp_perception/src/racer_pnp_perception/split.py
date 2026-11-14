"""Loader for the pre-registered data split (evaluation/prereg/pnp-v0.2-split.yaml).

The split says, per bag, which stretches of bag time belong to which role for perception:

    train           may be trained on (and cut into an internal validation part)
    heldout         in-layout held-out, decides GP1 items 1 to 3, 5 and 6
    heldout_layout  a layout the net never trains on, decides the cross-layout part of GP1
    unused          anything else (outside every segment, an unknown bag, a bag marked unused)

Time is seconds after the first /scan message of the bag by recorder log time (the axis of
tools/pnp_replay/bag.py). A sample belongs to a segment only when EVERY message it was built from
lies inside [from, to] of that one segment: `role_of` takes the time of the first and the last
message and gives a role only then. A bag with a `whole_bag` entry has one role everywhere.

The file is read-only to this package. `load_split` refuses a file whose SHA-256 is not the one
written into the pre-registration, so a changed split cannot be used by accident.
"""

from __future__ import annotations

import hashlib
import itertools
from dataclasses import dataclass
from pathlib import Path
from typing import Any

import yaml

# Rule 10 of the v0.2 briefs: no result may change the split file with this digest.
SPLIT_SHA256 = "bc5185c912ece67a6177581f1b37adaaa9101b34ee1efe9881af7723681aef83"
DEFAULT_SPLIT_PATH = Path("/Users/cameronjim/code/car/evaluation/prereg/pnp-v0.2-split.yaml")

ROLE_TRAIN = "train"
ROLE_HELDOUT = "heldout"
ROLE_HELDOUT_LAYOUT = "heldout_layout"
ROLE_UNUSED = "unused"
ROLES = (ROLE_TRAIN, ROLE_HELDOUT, ROLE_HELDOUT_LAYOUT, "legacy_70_30", ROLE_UNUSED)
WHOLE_BAG = "whole_bag"
WORKSTREAM = "perception"


class SplitError(ValueError):
    """The split file is not the pre-registered one, or is malformed."""


@dataclass(frozen=True)
class Segment:
    name: str
    t_from_s: float
    t_to_s: float
    role: str
    direction: str | None = None
    cap_mps: float | None = None
    driver: str | None = None

    def contains(self, t_first_s: float, t_last_s: float) -> bool:
        return self.t_from_s <= t_first_s and t_last_s <= self.t_to_s and t_first_s <= t_last_s


@dataclass(frozen=True)
class BagSplit:
    name: str
    segments: tuple[Segment, ...]
    whole_bag_role: str | None
    first_scan_log_time_s: float | None = None


@dataclass(frozen=True)
class Split:
    split_id: str
    bags: dict[str, BagSplit]
    sha256: str
    workstream: str = WORKSTREAM

    def locate(self, bag: str, t_first_s: float, t_last_s: float) -> Segment | None:
        """The segment (or a whole-bag pseudo segment) that holds the whole interval, else None.
        An unknown bag has none."""
        entry = self.bags.get(bag)
        if entry is None:
            return None
        if entry.whole_bag_role is not None:
            return Segment(WHOLE_BAG, float("-inf"), float("inf"), entry.whole_bag_role)
        for seg in entry.segments:
            if seg.contains(t_first_s, t_last_s):
                return seg
        return None

    def role_of(self, bag: str, t_first_s: float, t_last_s: float) -> str:
        """The role of the interval, `unused` unless it lies inside one segment."""
        seg = self.locate(bag, t_first_s, t_last_s)
        return seg.role if seg is not None else ROLE_UNUSED

    def segments_with_role(self, role: str) -> list[tuple[str, Segment]]:
        out: list[tuple[str, Segment]] = []
        for bag in self.bags.values():
            out += [(bag.name, s) for s in bag.segments if s.role == role]
        return out

    def whole_bags_with_role(self, role: str) -> list[str]:
        return [b.name for b in self.bags.values() if b.whole_bag_role == role]

    def bag_names_with_role(self, role: str) -> list[str]:
        names = {b for b, _ in self.segments_with_role(role)} | set(self.whole_bags_with_role(role))
        return sorted(names)


def _role(entry: dict[str, Any], workstream: str, where: str) -> str:
    role = entry.get(workstream)
    if role not in ROLES:
        raise SplitError(f"{where}: role {role!r} is not one of {ROLES}")
    return str(role)


def parse_split(text: str, workstream: str = WORKSTREAM, sha256: str = "") -> Split:
    """Parse split YAML text. Segment times are validated (from < to, no overlap in a bag)."""
    doc = yaml.safe_load(text)
    if not isinstance(doc, dict) or "bags" not in doc:
        raise SplitError("split file has no `bags` mapping")
    bags: dict[str, BagSplit] = {}
    for name, entry in doc["bags"].items():
        whole = entry.get("whole_bag")
        segments: list[Segment] = []
        for raw in entry.get("segments", []) or []:
            seg = Segment(
                name=str(raw["name"]),
                t_from_s=float(raw["from"]),
                t_to_s=float(raw["to"]),
                role=_role(raw, workstream, f"{name}/{raw.get('name')}"),
                direction=raw.get("direction"),
                cap_mps=float(raw["cap_mps"]) if "cap_mps" in raw else None,
                driver=raw.get("driver"),
            )
            if not seg.t_from_s < seg.t_to_s:
                raise SplitError(f"{name}/{seg.name}: from must be before to")
            segments.append(seg)
        segments.sort(key=lambda s: s.t_from_s)
        for a, b in itertools.pairwise(segments):
            if b.t_from_s <= a.t_to_s:
                raise SplitError(f"{name}: segments {a.name} and {b.name} overlap")
        if whole is not None and segments:
            raise SplitError(f"{name}: both whole_bag and segments")
        if whole is None and not segments:
            raise SplitError(f"{name}: neither whole_bag nor segments")
        first = entry.get("first_scan_log_time_s")
        bags[str(name)] = BagSplit(
            name=str(name),
            segments=tuple(segments),
            whole_bag_role=_role(whole, workstream, f"{name}/whole_bag") if whole else None,
            first_scan_log_time_s=float(first) if first is not None else None,
        )
    return Split(str(doc.get("split_id", "")), bags, sha256, workstream)


def load_split(
    path: Path = DEFAULT_SPLIT_PATH,
    workstream: str = WORKSTREAM,
    expected_sha256: str = SPLIT_SHA256,
) -> Split:
    """Load the split file. Raises SplitError when its SHA-256 is not `expected_sha256` (the
    pre-registered digest by default)."""
    data = Path(path).read_bytes()
    digest = hashlib.sha256(data).hexdigest()
    if digest != expected_sha256:
        raise SplitError(
            f"{path}: SHA-256 {digest} is not the pre-registered {expected_sha256}; "
            "the split must not change"
        )
    return parse_split(data.decode("utf-8"), workstream, digest)
