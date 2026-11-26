"""Data roles of the dynamics iteration on dyn-v0.7p's line (versions dyn-v0.8 and later,
unregistered development models, owner decision 29; each version is scored once against its own
rule, written and committed before its pass).

Roles, fixed here before any fit of the iteration:

* train and validation: the older train roles (the v0.2 split train and validation segments, the
  legacy bags, the 2026-11-14 train segments: `cli_v04.Stage1.train`) plus the train and validation
  parts of dyn-v0.7p's development split (`training/pnp_models/dyn-v0.7p-2026-11-24/dev-split.yaml`)
  of the 2026-11-15 session bags and the 2026-11-22 bag (whose train and validation parts are the
  first 55 percent of each run, the GP2g-R train role);
* H1 (decides a version): the GP2g-R held-out parts of the 2026-11-22 runs (brief 54), read only
  through `prereg_v06.Roles.heldout_parts` with a freeze token;
* H2 (reported only): the development parts of dyn-v0.7p's split of the 2026-11-15 session bags
  (dyn-v0.7p's own development held-out without the 2026-11-23 part);
* confirmation only, never read: every bag of the 2026-11-23 session (session 2, bag names
  starting `2026-11-24T`), as the brief 54 prereg says for bags after 2026-11-22. dyn-v0.7p trained
  on that session; no version of this iteration does.

Validation (config and seed choice, early stopping, every design decision) never touches H1 or H2.
"""

from __future__ import annotations

from collections.abc import Iterable, Sequence

from . import proto

SESSION2_PREFIX = "2026-11-24T"  # the 2026-11-23 session (UTC date in the bag name)
GP2G_BAG = "2026-11-23T03-50-32_car_teleop"  # the 2026-11-22 session
H2_SESSION_PREFIX = "2026-11-16T"  # the 2026-11-15 session (UTC date in the bag name)

FIT_ROLES = (proto.TRAIN, proto.VAL)


class RoleRefused(PermissionError):
    """A part was asked for in a role this iteration does not give it."""


def is_session2(bag: str) -> bool:
    return bag.startswith(SESSION2_PREFIX)


def check_bag(bag: str) -> None:
    if is_session2(bag):
        raise RoleRefused(f"{bag}: session 2 (2026-11-23) is confirmation only for this iteration")


def fit_parts(parts: Iterable[proto.RunPart]) -> list[proto.RunPart]:
    """The train and validation parts of the dyn-v0.7p split that this iteration fits on: every
    one outside session 2. A development part is never returned."""
    out = [p for p in parts if p.role in FIT_ROLES and not is_session2(p.bag)]
    for p in out:
        check_bag(p.bag)
    return out


def h2_parts(parts: Iterable[proto.RunPart]) -> list[proto.RunPart]:
    """H2: the development parts of the 2026-11-15 session bags (reported only)."""
    return [p for p in parts if p.role == proto.DEV and p.bag.startswith(H2_SESSION_PREFIX)]


def check_fit_segments(names: Sequence[str], splits: Sequence[str], bags: Sequence[str]) -> None:
    """Refuse a fit set that holds a session 2 bag, a development part or a held-out segment."""
    for name, split, bag in zip(names, splits, bags, strict=True):
        check_bag(bag)
        if split not in FIT_ROLES:
            raise RoleRefused(f"{name}: role {split!r} is not train or validation")
