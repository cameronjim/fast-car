"""The pre-registered data split of Plug-and-Play v0.2 (evaluation/prereg/pnp-v0.2-split.yaml).

The file lists, per bag, time segments (bag seconds after the first /scan) with a role for each
workstream, or a role for a whole bag. Only the `dynamics` role is read here. Rule 10 of the v0.2
briefs: the file is never edited after the pre-registration, so `load_split` refuses a file whose
SHA-256 is not the constant below. `Split.role_of` answers "which role does this time interval
have" and answers only when the WHOLE interval lies inside one segment of one bag, so a fit
window or an evaluation window that straddles a segment edge has no role and is not used.
"""

from __future__ import annotations

import hashlib
from dataclasses import dataclass
from pathlib import Path

import yaml

# SHA-256 of evaluation/prereg/pnp-v0.2-split.yaml as pre-registered (docs/notes/
# preregistration-pnp-v0.2.md, "Split file SHA-256 at the time of writing").
SPLIT_SHA256 = "bc5185c912ece67a6177581f1b37adaaa9101b34ee1efe9881af7723681aef83"
SPLIT_RELATIVE = Path("evaluation") / "prereg" / "pnp-v0.2-split.yaml"
WORKSTREAM = "dynamics"
ROLES = ("train", "heldout", "heldout_layout", "legacy_70_30", "unused")


class SplitInvalid(ValueError):
    """The split file does not hash to the pre-registered constant, or is malformed."""


@dataclass(frozen=True)
class SplitSegment:
    """One time segment of a bag, with its role for the dynamics workstream."""

    bag: str
    name: str
    t_from_s: float  # bag seconds after the first /scan
    t_to_s: float
    role: str
    driver: str = ""
    direction: str = ""
    cap_mps: float | None = None

    def contains(self, t_from_s: float, t_to_s: float) -> bool:
        return self.t_from_s <= t_from_s and t_to_s <= self.t_to_s


@dataclass(frozen=True)
class BagEntry:
    name: str
    units: str
    first_scan_log_time_s: float | None
    segments: tuple[SplitSegment, ...] = ()
    whole_bag_role: str | None = None  # role of the dynamics workstream for a whole bag


@dataclass(frozen=True)
class Split:
    split_id: str
    sha256: str
    bags: dict[str, BagEntry]

    def entry(self, bag: str) -> BagEntry | None:
        return self.bags.get(bag)

    def segment_of(self, bag: str, t_from_s: float, t_to_s: float) -> SplitSegment | None:
        """The segment of `bag` that contains [t_from_s, t_to_s] whole, else None."""
        entry = self.bags.get(bag)
        if entry is None:
            return None
        for seg in entry.segments:
            if seg.contains(t_from_s, t_to_s):
                return seg
        return None

    def role_of(self, bag: str, t_from_s: float, t_to_s: float) -> str | None:
        """Role of the interval (bag seconds), or None when it is not inside one segment (or the
        bag is not in the split file). A whole-bag entry gives its role for any interval."""
        if t_to_s < t_from_s:
            raise ValueError("t_to_s is before t_from_s")
        entry = self.bags.get(bag)
        if entry is None:
            return None
        if entry.whole_bag_role is not None:
            return entry.whole_bag_role
        seg = self.segment_of(bag, t_from_s, t_to_s)
        return None if seg is None else seg.role


def _entry_of(name: str, raw: dict) -> BagEntry:
    segments = []
    for item in raw.get("segments", []):
        role = item.get(WORKSTREAM)
        if role not in ROLES:
            raise SplitInvalid(f"{name}/{item.get('name')}: {WORKSTREAM} role {role!r} unknown")
        if not item["from"] < item["to"]:
            raise SplitInvalid(f"{name}/{item.get('name')}: empty segment")
        segments.append(
            SplitSegment(
                bag=name,
                name=str(item["name"]),
                t_from_s=float(item["from"]),
                t_to_s=float(item["to"]),
                role=role,
                driver=str(item.get("driver", "")),
                direction=str(item.get("direction", "")),
                cap_mps=None if item.get("cap_mps") is None else float(item["cap_mps"]),
            )
        )
    whole = raw.get("whole_bag")
    role = None
    if whole is not None:
        role = whole.get(WORKSTREAM)
        if role not in ROLES:
            raise SplitInvalid(f"{name}: whole_bag {WORKSTREAM} role {role!r} unknown")
    if not segments and role is None:
        raise SplitInvalid(f"{name}: neither segments nor whole_bag")
    first = raw.get("first_scan_log_time_s")
    return BagEntry(
        name=name,
        units=str(raw.get("units", "")),
        first_scan_log_time_s=None if first is None else float(first),
        segments=tuple(sorted(segments, key=lambda s: s.t_from_s)),
        whole_bag_role=role,
    )


def load_split(path: Path, expected_sha256: str = SPLIT_SHA256) -> Split:
    """Parse the split file after checking its SHA-256. Raises SplitInvalid on a mismatch."""
    blob = Path(path).read_bytes()
    digest = hashlib.sha256(blob).hexdigest()
    if digest != expected_sha256:
        raise SplitInvalid(
            f"{path}: SHA-256 {digest} is not the pre-registered {expected_sha256}; the split "
            "file must not change (rule 10)"
        )
    raw = yaml.safe_load(blob.decode("utf-8"))
    bags = {name: _entry_of(name, body) for name, body in raw["bags"].items()}
    for entry in bags.values():
        for a, b in zip(entry.segments, entry.segments[1:], strict=False):
            if b.t_from_s < a.t_to_s:
                raise SplitInvalid(f"{entry.name}: segments {a.name} and {b.name} overlap")
    return Split(split_id=str(raw["split_id"]), sha256=digest, bags=bags)


def default_split_path(root: Path) -> Path:
    """The split file of a checkout. It is committed by the analysis PR; until that lands it sits
    untracked in the main checkout, which the CLI's --split option points at."""
    return root / SPLIT_RELATIVE
