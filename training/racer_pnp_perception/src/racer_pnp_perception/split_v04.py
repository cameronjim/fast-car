"""Loader for the perception roles of the 2026-11-14 bags (evaluation/prereg/pnp-v0.4.yaml, block
`split_2026_11_14`, pre-registration v0.4 section 1).

Perception roles there are `heldout` (decides GP1c), `reserve` (read by nobody in v0.4: not trained
on, not scored, not used for a setting) and `unused`. `perception_split` returns the usual `Split`
holding the `heldout` segments only: every other stretch of every bag, `reserve` included, is
simply outside the split, so `Split.role_of` says `unused` there and no self-label, no moving-scan
selection and no scoring path of this package ever admits a scan of it. Asking the loader for the
`reserve` role is refused with `ReserveSegmentRefused`.

The file is read-only to this package. The loader refuses a file whose SHA-256 is not the one
written into the pre-registration, so a changed split cannot be used by accident.
"""

from __future__ import annotations

import hashlib
from pathlib import Path
from typing import Any

import yaml
from racer_track_perception.vehicle import find_repo_root

from racer_pnp_perception.split import (
    ROLE_HELDOUT,
    ROLE_UNUSED,
    Split,
    SplitError,
    parse_split,
)

# docs/notes/preregistration-pnp-v0.4.md: "Constants file: evaluation/prereg/pnp-v0.4.yaml, SHA-256".
V04_SHA256 = "78510c1423278f80c42511e6caab095818598af6db6a284dd5479f74a03a1d63"
RELATIVE_PATH = Path("evaluation") / "prereg" / "pnp-v0.4.yaml"
ROLE_RESERVE = "reserve"
WORKSTREAM = "perception"


class ReserveSegmentRefused(SplitError):
    """Something asked to read a perception `reserve` segment, which nobody reads in v0.4."""


def default_path() -> Path:
    return find_repo_root() / RELATIVE_PATH


def read_yaml(
    path: Path | None = None, expected_sha256: str = V04_SHA256
) -> tuple[dict[str, Any], str]:
    """The parsed constants file and its digest. Raises SplitError when the digest is not the
    pre-registered one."""
    p = Path(path) if path is not None else default_path()
    data = p.read_bytes()
    digest = hashlib.sha256(data).hexdigest()
    if digest != expected_sha256:
        raise SplitError(
            f"{p}: SHA-256 {digest} is not the pre-registered {expected_sha256}; the constants "
            "file must not change"
        )
    doc = yaml.safe_load(data.decode("utf-8"))
    if not isinstance(doc, dict) or "split_2026_11_14" not in doc:
        raise SplitError(f"{p}: no split_2026_11_14 block")
    return doc, digest


def _heldout_only(doc: dict[str, Any], roles: tuple[str, ...]) -> dict[str, Any]:
    """The `bags` mapping of the split with the perception role as the only role key and every
    stretch that does not have a role in `roles` removed."""
    bags: dict[str, Any] = {}
    for name, entry in doc["split_2026_11_14"]["bags"].items():
        out: dict[str, Any] = {}
        if "first_scan_log_time_s" in entry:
            out["first_scan_log_time_s"] = entry["first_scan_log_time_s"]
        whole = entry.get("whole_bag")
        if whole is not None:
            if whole.get(WORKSTREAM) in roles:
                out["whole_bag"] = {WORKSTREAM: whole[WORKSTREAM]}
        else:
            segs = [
                {**{k: v for k, v in s.items() if k not in ("dynamics", "lap")}}
                for s in entry.get("segments", []) or []
                if s.get(WORKSTREAM) in roles
            ]
            if segs:
                out["segments"] = segs
        if "whole_bag" in out or "segments" in out:
            bags[str(name)] = out
    return {"split_id": doc.get("prereg_id", ""), "bags": bags}


def perception_split_from_doc(
    doc: dict[str, Any], sha256: str = "", roles: tuple[str, ...] = (ROLE_HELDOUT,)
) -> Split:
    """The `Split` of the given perception roles of an already parsed constants file."""
    if ROLE_RESERVE in roles:
        raise ReserveSegmentRefused(
            "perception `reserve` segments are read by nobody in v0.4 (pre-registration 3.1)"
        )
    unknown = set(roles) - {ROLE_HELDOUT}
    if unknown:
        raise SplitError(f"perception roles {sorted(unknown)} cannot be loaded: only heldout")
    plain = _heldout_only(doc, roles)
    return parse_split(yaml.safe_dump(plain), WORKSTREAM, sha256)


def perception_split(path: Path | None = None, roles: tuple[str, ...] = (ROLE_HELDOUT,)) -> Split:
    """The perception `heldout` segments of 2026-11-14 as a `Split`, from the verified file."""
    if ROLE_RESERVE in roles:
        raise ReserveSegmentRefused(
            "perception `reserve` segments are read by nobody in v0.4 (pre-registration 3.1)"
        )
    doc, digest = read_yaml(path)
    return perception_split_from_doc(doc, digest, roles)


def reserve_intervals(doc: dict[str, Any]) -> list[tuple[str, float, float]]:
    """(bag, from_s, to_s) of every perception `reserve` segment: for a guard that asserts no
    scan the evaluation touched lies in one."""
    out: list[tuple[str, float, float]] = []
    for name, entry in doc["split_2026_11_14"]["bags"].items():
        for s in entry.get("segments", []) or []:
            if s.get(WORKSTREAM) == ROLE_RESERVE:
                out.append((str(name), float(s["from"]), float(s["to"])))
    return out


def assert_not_reserve(doc: dict[str, Any], bag: str, t_first_s: float, t_last_s: float) -> None:
    """Raise `ReserveSegmentRefused` if [t_first_s, t_last_s] of `bag` touches a reserve segment."""
    for b, lo, hi in reserve_intervals(doc):
        if b == bag and t_first_s <= hi and t_last_s >= lo:
            raise ReserveSegmentRefused(
                f"{bag} [{t_first_s:.1f}, {t_last_s:.1f}] s touches a perception reserve "
                f"segment [{lo}, {hi}] s"
            )


__all__ = [
    "ROLE_RESERVE",
    "ROLE_UNUSED",
    "V04_SHA256",
    "ReserveSegmentRefused",
    "assert_not_reserve",
    "perception_split",
    "perception_split_from_doc",
    "read_yaml",
    "reserve_intervals",
]
