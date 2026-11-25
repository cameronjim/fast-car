"""The yaw-only residual attempt R6 (brief 54): the registered constants, the 2026-11-22 session
split and the data roles of `evaluation/prereg/pnp-v0.6-dynamics-yaw.yaml`.

* `load_constants_v06` reads the yaml after checking its SHA-256 (on every run).
* `load_session_v06` reads `evaluation/prereg/sessions/2026-11-22-dynamics-v06.yaml` (written by
  `tools/pnp_data.py split-v06`) after checking that it names the v0.6 file and, by default, that
  it is committed (the commit order is the evidence).
* `Roles` is the role loader. Train and selection are open to the fit stages; the held-out parts of
  the 2026-11-22 runs need a `FreezeToken` (`truth_v04.freeze`, the git SHA of the freeze commit);
  a bag recorded after 2026-11-22 is confirmation only and is refused for every role.
* `train_path` / `heldout_path` keep the 2026-11-22 truth in two sibling caches, `load_train_slice`
  refuses a held-out part and `load_heldout_slice` needs the token.
"""

from __future__ import annotations

import hashlib
import subprocess
from collections.abc import Mapping
from dataclasses import dataclass
from pathlib import Path
from typing import Any

import yaml

from .split import Split, SplitSegment
from .truth_v04 import T2, VARIANTS, FreezeToken, HeldoutLocked, Slice, read_slice

V06_SHA256 = "db62eaac18ba655bf00e4ff7c8e8b287cd7e042b6ea671014ebb404032d00e0c"
V06_RELATIVE = Path("evaluation") / "prereg" / "pnp-v0.6-dynamics-yaw.yaml"
SESSION_RELATIVE = Path("evaluation") / "prereg" / "sessions" / "2026-11-22-dynamics-v06.yaml"
SESSION_BAG = "2026-11-23T03-50-32_car_teleop"  # the 2026-11-22 session (UTC date in the name)
TRAIN_DIR = "pnp-v06-t2-train"
HELDOUT_DIR = "pnp-v06-t2-heldout"

TRAIN = "train"
SELECTION = "selection"
HELDOUT = "heldout"
CONFIRMATION = "confirmation_only"


class PreregInvalid(ValueError):
    """The v0.6 constants or the session file are not the registered ones."""


class ConfirmationOnly(PermissionError):
    """A bag recorded after 2026-11-22 was asked for: it is confirmation only (GP2g-C)."""


@dataclass(frozen=True)
class ConstantsV06:
    sha256: str
    doc: Mapping[str, Any]

    @property
    def gate(self) -> Mapping[str, Any]:
        return self.doc["gate_residual"]

    @property
    def metrics(self) -> Mapping[str, Any]:
        return self.doc["metrics"]

    @property
    def validity_v_max_mps(self) -> float:
        return float(self.doc["candidate"]["validity_v_max_mps"])

    @property
    def seeds(self) -> tuple[int, ...]:
        return tuple(int(s) for s in self.doc["candidate"]["seeds"])


def load_constants_v06(path: Path, expected_sha256: str = V06_SHA256) -> ConstantsV06:
    blob = Path(path).read_bytes()
    digest = hashlib.sha256(blob).hexdigest()
    if digest != expected_sha256:
        raise PreregInvalid(
            f"{path}: SHA-256 {digest} is not the pre-registered {expected_sha256}; the v0.6 "
            "constants must not change"
        )
    return ConstantsV06(digest, yaml.safe_load(blob.decode("utf-8")))


def _committed(path: Path) -> bool:
    try:
        out = subprocess.run(
            ["git", "-C", str(path.parent), "ls-tree", "--name-only", "HEAD", "--", path.name],
            capture_output=True,
            text=True,
            check=True,
        )
    except (OSError, subprocess.CalledProcessError):
        return False
    return path.name in out.stdout.split()


def load_session_v06(path: Path, require_committed: bool = True) -> Split:
    """The 2026-11-22 parts as a `Split`. Refuses a file that names another rule file, one with
    another bag, or (with `require_committed`) one that is not in the HEAD commit."""
    from .session_v05 import load_session_split

    path = Path(path)
    if not path.is_file():
        raise PreregInvalid(f"{path} is not there: split-v06 comes before any fit")
    if require_committed and not _committed(path):
        raise PreregInvalid(f"{path} is not committed: commit it, then fit")
    split = load_session_split(path, expected_rule_sha256=V06_SHA256)
    if set(split.bags) != {SESSION_BAG}:
        raise PreregInvalid(f"{path}: bags {sorted(split.bags)} are not [{SESSION_BAG}]")
    return split


def is_confirmation_bag(bag: str) -> bool:
    """A bag recorded after the 2026-11-22 session (its name sorts after the session bag's: the
    names start with the UTC time)."""
    return Path(bag).name > SESSION_BAG


@dataclass(frozen=True)
class Roles:
    """The data roles of this attempt. `split` is the committed 2026-11-22 session file."""

    split: Split

    def check_bag(self, bag: str) -> None:
        if is_confirmation_bag(bag):
            raise ConfirmationOnly(
                f"{bag} was recorded after 2026-11-22: confirmation only, never train, selection "
                "or heldout for this attempt"
            )

    def train_parts(self) -> list[SplitSegment]:
        return [s for e in self.split.bags.values() for s in e.segments if s.role == TRAIN]

    def heldout_parts(self, token: FreezeToken) -> list[SplitSegment]:
        if not isinstance(token, FreezeToken):
            raise HeldoutLocked("the 2026-11-22 held-out parts need the freeze token of R6")
        return [s for e in self.split.bags.values() for s in e.segments if s.role == HELDOUT]

    def role_of(self, bag: str, part: str) -> str:
        self.check_bag(bag)
        entry = self.split.bags.get(Path(bag).name)
        if entry is None:
            raise KeyError(f"{bag} is not in the 2026-11-22 session file")
        for s in entry.segments:
            if s.name == part:
                return s.role
        raise KeyError(f"{bag}/{part} is not a part of the 2026-11-22 session file")


# ---- the truth caches of the 2026-11-22 parts ------------------------------------------------


def _file(bag: str, part: str, variant: str = T2) -> str:
    if variant not in VARIANTS:
        raise ValueError(f"unknown truth variant {variant!r}")
    tag = "" if variant == T2 else f"-{variant}"
    return f"{bag}.{part}.motion-wheel{tag}.npz"


def train_path(cache_dir: Path, bag: str, part: str, variant: str = T2) -> Path:
    return Path(cache_dir) / TRAIN_DIR / _file(bag, part, variant)


def heldout_path(cache_dir: Path, bag: str, part: str, variant: str = T2) -> Path:
    return Path(cache_dir) / HELDOUT_DIR / _file(bag, part, variant)


def load_train_slice(
    roles: Roles, cache_dir: Path, bag: str, part: str, variant: str = T2
) -> Slice:
    """What the fit stages read. Refuses any part that is not train."""
    role = roles.role_of(bag, part)
    if role != TRAIN:
        raise HeldoutLocked(f"{bag}/{part} has role {role!r}: the fit stages read train parts only")
    return read_slice(train_path(cache_dir, bag, part, variant))


def load_heldout_slice(
    roles: Roles, cache_dir: Path, bag: str, part: str, token: FreezeToken, variant: str = T2
) -> Slice:
    """The one reader of the 2026-11-22 held-out cache: the frozen pass, with its token."""
    if not isinstance(token, FreezeToken):
        raise HeldoutLocked("the held-out truth needs a FreezeToken (truth_v04.freeze)")
    role = roles.role_of(bag, part)
    if role != HELDOUT:
        raise HeldoutLocked(f"{bag}/{part} has role {role!r}, not heldout")
    return read_slice(heldout_path(cache_dir, bag, part, variant))
