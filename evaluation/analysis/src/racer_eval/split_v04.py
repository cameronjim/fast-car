"""Load and validate the split of the 2026-11-14 bags (`split_2026_11_14` of pnp-v0.4.yaml).

Pre-registration v0.4 section 1. The loader checks the SHA-256 of the whole yaml on every call
and refuses on a mismatch. It reads three kinds of role:

* dynamics   train | heldout | unused
* perception heldout | reserve | unused (`reserve` is sealed: no perception code reads it)
* lap        dev | test | unused, given per whole run (the a and b parts of a cut run are joined)

The loader of the v0.2 split (racer_eval.split) is untouched. Times are bag seconds, the axis of
the split file (seconds after the first /scan). Segment intervals are closed: [t_from, t_to].
`role_of` keeps the rule of v0.2 section 1: an interval has the role of a segment only if it lies
wholly inside that one segment, otherwise it is `unused`.
"""

from __future__ import annotations

import re
from dataclasses import dataclass
from pathlib import Path
from typing import Any

import yaml

from racer_eval.split import SplitError, file_sha256

#: SHA-256 of evaluation/prereg/pnp-v0.4.yaml (pre-registration v0.4, frozen).
PREREG_V04_SHA256 = "78510c1423278f80c42511e6caab095818598af6db6a284dd5479f74a03a1d63"

WORKSTREAMS = ("dynamics", "perception")
#: The roles each workstream's column may carry in the yaml.
WORKSTREAM_ROLES = {
    "dynamics": ("train", "heldout", "unused"),
    "perception": ("heldout", "reserve", "unused"),
}
LAP_ROLES = ("dev", "test", "unused")
UNUSED = "unused"
#: Largest gap between the a and b parts of one lap run (they are 2.0 s apart in the yaml).
_MAX_PART_GAP_S = 2.5


class ReserveIsSealed(PermissionError):
    """A perception `reserve` segment was asked for. Nobody reads them in v0.4."""


class UnknownBag(KeyError):
    """The bag is not in the split of 2026-11-14 (a later session has its own committed split)."""


def default_path() -> Path:
    return Path(__file__).resolve().parents[4] / "prereg" / "pnp-v0.4.yaml"


@dataclass(frozen=True)
class Segment:
    bag: str
    name: str
    t_from: float
    t_to: float
    direction: str
    cap_mps: float
    dynamics: str
    perception: str
    lap: str

    @property
    def duration_s(self) -> float:
        return self.t_to - self.t_from

    def role(self, workstream: str) -> str:
        _check_workstream(workstream)
        return self.dynamics if workstream == "dynamics" else self.perception


@dataclass(frozen=True)
class LapRun:
    """A whole run for the lap role: its parts joined, from the first part's start to the last
    part's end."""

    name: str
    bag: str
    t_from: float
    t_to: float
    direction: str
    cap_mps: float
    role: str
    parts: tuple[str, ...]

    @property
    def duration_s(self) -> float:
        return self.t_to - self.t_from


@dataclass(frozen=True)
class SplitV04:
    sha256: str
    segments_all: tuple[Segment, ...]
    whole_bags: dict[str, dict[str, str]]
    lap_runs_all: tuple[LapRun, ...]

    def segments(
        self, workstream: str, role: str, *, i_am_the_split_tool: bool = False
    ) -> list[Segment]:
        """Segments with `role` for `workstream`. Perception `reserve` is sealed."""
        _check_workstream(workstream)
        if role not in WORKSTREAM_ROLES[workstream]:
            raise ValueError(
                f"{workstream} has no role {role!r}; one of {WORKSTREAM_ROLES[workstream]}"
            )
        if workstream == "perception" and role == "reserve" and not i_am_the_split_tool:
            raise ReserveIsSealed(
                "perception reserve segments are read by nobody in v0.4 "
                "(pre-registration v0.4 section 1)"
            )
        return [s for s in self.segments_all if s.role(workstream) == role]

    def lap_runs(self, role: str) -> list[LapRun]:
        """Whole runs with the lap role `role` (dev or test)."""
        if role not in LAP_ROLES:
            raise ValueError(f"no lap role {role!r}; one of {LAP_ROLES}")
        return [r for r in self.lap_runs_all if r.role == role]

    def lap_run(self, name: str) -> LapRun:
        for run in self.lap_runs_all:
            if run.name == name:
                return run
        raise KeyError(f"no lap run {name!r}; known: {[r.name for r in self.lap_runs_all]}")

    def role_of(self, bag: str, t0: float, t1: float, workstream: str) -> str:
        """The role of the interval [t0, t1] of `bag` for `workstream`: that of the one segment it
        lies wholly inside, `unused` if it straddles an edge or lies outside every segment. A bag
        with one role for its whole length gives that role. A bag that is not in the split raises
        UnknownBag (a session after 2026-11-14 has a split of its own)."""
        _check_workstream(workstream)
        if t1 < t0:
            raise ValueError(f"interval ends before it starts: {t0} to {t1}")
        key = Path(str(bag)).name
        if key in self.whole_bags:
            return self.whole_bags[key][workstream]
        if key not in {s.bag for s in self.segments_all}:
            raise UnknownBag(key)
        for seg in self.segments_all:
            if seg.bag == key and seg.t_from <= t0 and t1 <= seg.t_to:
                return seg.role(workstream)
        return UNUSED


def _check_workstream(workstream: str) -> None:
    if workstream not in WORKSTREAMS:
        raise ValueError(f"unknown workstream {workstream!r}; one of {WORKSTREAMS}")


def _check_role(value: Any, allowed: tuple[str, ...], where: str) -> str:
    if value not in allowed:
        raise SplitError(f"{where}: unknown role {value!r}; one of {allowed}")
    return str(value)


def _segment(bag: str, raw: Any) -> Segment:
    if not isinstance(raw, dict):
        raise SplitError(f"{bag}: a segment is not a mapping: {raw!r}")
    name = raw.get("name", "?")
    where = f"{bag} segment {name}"
    for field in ("name", "from", "to", "direction", "cap_mps", *WORKSTREAMS, "lap"):
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
        direction=str(raw["direction"]),
        cap_mps=cap,
        dynamics=_check_role(raw["dynamics"], WORKSTREAM_ROLES["dynamics"], where),
        perception=_check_role(raw["perception"], WORKSTREAM_ROLES["perception"], where),
        lap=_check_role(raw["lap"], LAP_ROLES, where),
    )


def _join_lap_run(name: str, role: str, segments: list[Segment]) -> LapRun:
    """The lap run `name`: the segment of that name, or its a and b parts joined."""
    own = [s for s in segments if s.name == name]
    parts = own or sorted(
        (s for s in segments if re.fullmatch(re.escape(name) + r"[ab]", s.name)),
        key=lambda s: s.t_from,
    )
    if not parts:
        raise SplitError(f"lap run {name}: no segment {name} or {name}a and {name}b")
    bags = {s.bag for s in parts}
    if len(bags) != 1:
        raise SplitError(f"lap run {name}: its parts are in several bags {sorted(bags)}")
    for a, b in zip(parts, parts[1:], strict=False):
        if b.t_from - a.t_to > _MAX_PART_GAP_S:
            raise SplitError(f"lap run {name}: parts {a.name} and {b.name} are not consecutive")
    wrong = [s.name for s in parts if s.lap != role]
    if wrong:
        raise SplitError(f"lap run {name}: parts {wrong} do not carry the lap role {role}")
    if len({s.direction for s in parts}) != 1:
        raise SplitError(f"lap run {name}: its parts differ in direction")
    return LapRun(
        name=name,
        bag=parts[0].bag,
        t_from=parts[0].t_from,
        t_to=parts[-1].t_to,
        direction=parts[0].direction,
        cap_mps=parts[0].cap_mps,
        role=role,
        parts=tuple(s.name for s in parts),
    )


def parse_v04(doc: dict[str, Any], digest: str) -> SplitV04:
    """Validate and build the split from the parsed yaml (the block `split_2026_11_14`)."""
    block = doc.get("split_2026_11_14")
    if not isinstance(block, dict) or not isinstance(block.get("bags"), dict):
        raise SplitError("split_2026_11_14 with a `bags` mapping is missing")
    segments: list[Segment] = []
    whole_bags: dict[str, dict[str, str]] = {}
    for bag, entry in block["bags"].items():
        if not isinstance(entry, dict):
            raise SplitError(f"{bag}: not a mapping")
        has_segments, has_whole = "segments" in entry, "whole_bag" in entry
        if has_segments == has_whole:
            raise SplitError(f"{bag}: needs exactly one of segments and whole_bag")
        if has_whole:
            whole = entry["whole_bag"]
            for field in (*WORKSTREAMS, "lap"):
                if not isinstance(whole, dict) or field not in whole:
                    raise SplitError(f"{bag} whole_bag: missing field {field!r}")
            for ws in WORKSTREAMS:
                _check_role(whole[ws], WORKSTREAM_ROLES[ws], f"{bag} whole_bag")
            _check_role(whole["lap"], LAP_ROLES, f"{bag} whole_bag")
            whole_bags[str(bag)] = {ws: str(whole[ws]) for ws in (*WORKSTREAMS, "lap")}
            continue
        parsed = [_segment(str(bag), raw) for raw in entry["segments"]]
        ordered = sorted(parsed, key=lambda s: s.t_from)
        for a, b in zip(ordered, ordered[1:], strict=False):
            if b.t_from < a.t_to:
                raise SplitError(f"{bag}: segments {a.name} and {b.name} overlap")
        segments.extend(parsed)
    names = [s.name for s in segments]
    if len(set(names)) != len(names):
        raise SplitError("segment names are not unique")
    lap_block = block.get("lap_runs")
    if not isinstance(lap_block, dict):
        raise SplitError("split_2026_11_14.lap_runs is missing")
    lap_runs: list[LapRun] = []
    for role in ("dev", "test"):
        for name in lap_block.get(role, []):
            lap_runs.append(_join_lap_run(str(name), role, segments))
    listed = {n for r in lap_runs for n in r.parts}
    for seg in segments:
        if seg.lap != "unused" and seg.name not in listed:
            raise SplitError(f"segment {seg.name} has lap role {seg.lap} but is in no lap run")
        if seg.lap == "unused" and seg.name in listed:
            raise SplitError(f"segment {seg.name} is in a lap run but has lap role unused")
    return SplitV04(digest, tuple(segments), whole_bags, tuple(lap_runs))


def load_v04(
    path: str | Path | None = None, expected_sha256: str | None = PREREG_V04_SHA256
) -> SplitV04:
    """Parse pnp-v0.4.yaml. Raises SplitError when its SHA-256 is not `expected_sha256` (pass None
    to skip the check; only the tests of this loader do)."""
    p = Path(path) if path is not None else default_path()
    digest = file_sha256(p)
    if expected_sha256 is not None and digest != expected_sha256:
        raise SplitError(f"{p}: SHA-256 {digest} is not the expected {expected_sha256}")
    doc = yaml.safe_load(p.read_text())
    if not isinstance(doc, dict):
        raise SplitError(f"{p}: not a mapping")
    return parse_v04(doc, digest)
