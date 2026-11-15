"""The dynamics roles of the 2026-11-14 bags (evaluation/prereg/pnp-v0.4.yaml, block
`split_2026_11_14`) and the data-role guards of brief 37.

The file is FROZEN: `load_split_v04` refuses a file whose SHA-256 is not the pre-registered
constant, on every call. The v0.4 blocks use the same shape as the v0.2 split file, so the parsing
is the v0.2 parser (`split._entry_of`), and `combined_split` gives one `Split` that holds the v0.2
bags and the 2026-11-14 bags, which is what `dataset.make_segments` reads.

Guards, each one a function with a test:

* `role_or_raise`: a time of a 2026-11-14 bag that lies outside every segment is refused (it has no
  role, so no code may read it);
* `require_split`: a bag recorded after 2026-11-14 is refused unless a committed split file for
  its session exists (pre-registration v0.4 section 1, `future_sessions`). This is the local form
  of brief 36's guard of the same name; switch to that one when it is on main;
* the held-out truth cache is only reachable through `truth_v04.HeldoutKey`, see that module.
"""

from __future__ import annotations

import hashlib
import re
from pathlib import Path

import yaml

from .split import BagEntry, Split, SplitInvalid, _entry_of

# SHA-256 of evaluation/prereg/pnp-v0.4.yaml as pre-registered (docs/notes/
# preregistration-pnp-v0.4.md, header).
V04_SHA256 = "78510c1423278f80c42511e6caab095818598af6db6a284dd5479f74a03a1d63"
V04_RELATIVE = Path("evaluation") / "prereg" / "pnp-v0.4.yaml"
SESSIONS_RELATIVE = Path("evaluation") / "prereg" / "sessions"
LAST_COVERED_DATE = "2026-11-14"  # bags of this date and earlier are covered by v0.2 / v0.4

TRAIN = "train"
HELDOUT = "heldout"

_DATE = re.compile(r"^(\d{4}-\d{2}-\d{2})T")


class UnsplitData(ValueError):
    """Data with no role was asked for."""


def load_split_v04(path: Path, expected_sha256: str = V04_SHA256) -> Split:
    """The 2026-11-14 bags with their dynamics roles. Raises SplitInvalid if the file is not the
    pre-registered one."""
    blob = Path(path).read_bytes()
    digest = hashlib.sha256(blob).hexdigest()
    if digest != expected_sha256:
        raise SplitInvalid(
            f"{path}: SHA-256 {digest} is not the pre-registered {expected_sha256}; the v0.4 "
            "constants file must not change"
        )
    raw = yaml.safe_load(blob.decode("utf-8"))
    block = raw["split_2026_11_14"]["bags"]
    bags: dict[str, BagEntry] = {}
    for name, body in block.items():
        entry = _entry_of(name, body)
        for a, b in zip(entry.segments, entry.segments[1:], strict=False):
            if b.t_from_s < a.t_to_s:
                raise SplitInvalid(f"{name}: segments {a.name} and {b.name} overlap")
        bags[name] = entry
    return Split(split_id=str(raw["prereg_id"]), sha256=digest, bags=bags)


def combined_split(v02: Split, v04: Split) -> Split:
    """One Split with the bags of both files. A bag in both would be a bug in the files."""
    both = set(v02.bags) & set(v04.bags)
    if both:
        raise SplitInvalid(f"bags in both split files: {sorted(both)}")
    return Split(
        split_id=f"{v02.split_id}+{v04.split_id}",
        sha256=f"{v02.sha256}+{v04.sha256}",
        bags={**v02.bags, **v04.bags},
    )


def role_or_raise(split: Split, bag: str, t_from_s: float, t_to_s: float) -> str:
    """The role of [t_from_s, t_to_s] (bag seconds after the first /scan), or UnsplitData when the
    interval is not inside one segment of the split (so it is unused data)."""
    role = split.role_of(bag, t_from_s, t_to_s)
    if role is None:
        raise UnsplitData(
            f"{bag} [{t_from_s:.1f}, {t_to_s:.1f}] s is outside every segment of the split: "
            "it is unused data and no stage may read it"
        )
    return role


def bag_date(bag_name: str) -> str:
    m = _DATE.match(bag_name)
    if m is None:
        raise ValueError(f"{bag_name}: no leading date, cannot tell the session")
    return m.group(1)


def require_split(bag_name: str, root: Path) -> None:
    """Training code calls this for every bag it reads. A bag recorded after 2026-11-14 needs the
    committed split of its session (`evaluation/prereg/sessions/<date>-split.yaml`) written by the
    tool of pre-registration v0.4 section 1; without it, a fit that reads the bag is a bug."""
    date = bag_date(bag_name)
    if date <= LAST_COVERED_DATE:
        return
    path = Path(root) / SESSIONS_RELATIVE / f"{date}-split.yaml"
    if not path.is_file():
        raise UnsplitData(
            f"{bag_name}: recorded after {LAST_COVERED_DATE} and {SESSIONS_RELATIVE}/{date}-split."
            "yaml is not committed; confirm the frozen candidates first, then write the split"
        )


def heldout_segment_names(split: Split) -> frozenset[str]:
    return frozenset(s.name for e in split.bags.values() for s in e.segments if s.role == HELDOUT)


def train_segment_names(split: Split) -> frozenset[str]:
    return frozenset(s.name for e in split.bags.values() for s in e.segments if s.role == TRAIN)
