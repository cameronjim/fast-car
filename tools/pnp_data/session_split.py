"""The split of a session recorded after 2026-11-14, by the registered rule (pnp-v0.4.yaml,
`future_sessions`):

* runs shorter than `min_run_s` (15 s) are unused;
* a speed and direction cell with two or more runs gives its LAST run to held-out, the others
  train; a cell with one run is cut: the first 55 percent train, 2.0 s unused, the rest held-out.
  The speed cell is the commanded cap rounded to 0.1 m/s.

Both workstreams (dynamics, perception) get the same role by this rule, as in the v0.2 split.
The rule has no word on a run in which the wheels did not turn, and none on a lap role: this
module applies nothing of its own for either. A run with a wheel distance under
`UNDRIVEN_BELOW_M` makes `split_session` refuse unless the caller names it as unused (the owner's
decision, written into the file). Laps are not assigned.

The file is written by a fixed formatter, not a YAML dumper, so it is byte-stable.
"""

from __future__ import annotations

import hashlib
import re
from collections.abc import Sequence
from dataclasses import dataclass
from pathlib import Path

import yaml

from pnp_data.runs import RunRow

#: pnp-v0.4.yaml future_sessions.cut_rule and min_run_s (checked against the yaml by a test).
SINGLE_RUN_TRAIN_SHARE = 0.55
GUARD_S = 2.0
MIN_RUN_S = 15.0
#: Under this wheel distance a run did not drive; the split refuses rather than guess its role.
UNDRIVEN_BELOW_M = 1.0
#: The newest bag date whose split is by hand (frozen in pnp-v0.4.yaml).
LAST_HAND_SPLIT_DATE = "2026-11-14"
SESSIONS_DIR = Path(__file__).resolve().parents[2] / "evaluation" / "prereg" / "sessions"

_BAG_DATE = re.compile(r"^(\d{4}-\d{2}-\d{2})T")


class SplitRefused(ValueError):
    """The rule cannot be applied as written; nothing was written."""


def bag_date(bag_name: str) -> str:
    m = _BAG_DATE.match(Path(bag_name).name)
    if m is None:
        raise SplitRefused(f"{bag_name}: the bag name does not start with a date (YYYY-MM-DDT)")
    return m.group(1)


@dataclass(frozen=True)
class SessionRun:
    """A run of one bag with the numbers the rule needs."""

    bag: str
    row: RunRow


@dataclass(frozen=True)
class SplitSegment:
    name: str
    bag: str
    run: str
    t_from: float
    t_to: float
    direction: str
    cap_mps: float
    role: str  # train | heldout | unused
    why: str


def _cell(run: SessionRun) -> tuple[float, str]:
    return (run.row.cap_mps, run.row.direction)


def split_runs(
    runs: Sequence[SessionRun], undriven_unused: Sequence[str] = ()
) -> list[SplitSegment]:
    """Segments of every run by the cut rule. Runs are named S1, S2, ... in the order given
    (bag order, then time); a cut run gives S<n>a and S<n>b. `undriven_unused` names runs (as
    `<bag>:<run id>`) the owner declared unused although the rule does not."""
    undriven = [
        f"{r.bag}:{r.row.run_id}"
        for r in runs
        if r.row.wheel_distance_m < UNDRIVEN_BELOW_M and r.row.duration_s >= MIN_RUN_S
    ]
    unnamed = [u for u in undriven if u not in undriven_unused]
    if unnamed:
        raise SplitRefused(
            "the rule says nothing about a run in which the wheels did not turn "
            f"(wheel distance under {UNDRIVEN_BELOW_M:g} m): {', '.join(unnamed)}. "
            "Name each as unused with --undriven-unused <bag>:<run> if that is the decision."
        )
    cells: dict[tuple[float, str], list[int]] = {}
    for i, r in enumerate(runs):
        if r.row.duration_s >= MIN_RUN_S and f"{r.bag}:{r.row.run_id}" not in undriven:
            cells.setdefault(_cell(r), []).append(i)
    segments: list[SplitSegment] = []
    for i, r in enumerate(runs):
        row = r.row

        def seg(
            suffix: str,
            t_from: float,
            t_to: float,
            role: str,
            why: str,
            name: str = f"S{i + 1}",
            r: SessionRun = r,
        ) -> SplitSegment:
            return SplitSegment(
                name + suffix,
                r.bag,
                r.row.run_id,
                t_from,
                t_to,
                r.row.direction,
                r.row.cap_mps,
                role,
                why,
            )

        if row.duration_s < MIN_RUN_S:
            segments.append(
                seg("", row.start_s, row.end_s, "unused", f"shorter than {MIN_RUN_S:g} s")
            )
        elif f"{r.bag}:{row.run_id}" in undriven:
            segments.append(
                seg(
                    "",
                    row.start_s,
                    row.end_s,
                    "unused",
                    "declared unused by the owner (wheels did not turn)",
                )
            )
        elif len(cells[_cell(r)]) >= 2:
            members = cells[_cell(r)]
            last = members[-1] == i
            why = (
                f"last of {len(members)} runs of its cell"
                if last
                else f"one of {len(members)} runs of its cell, not the last"
            )
            segments.append(seg("", row.start_s, row.end_s, "heldout" if last else "train", why))
        else:
            cut = row.start_s + SINGLE_RUN_TRAIN_SHARE * row.duration_s
            segments.append(
                seg("a", row.start_s, cut, "train", "the only run of its cell: first 55 percent")
            )
            segments.append(
                seg(
                    "b",
                    cut + GUARD_S,
                    row.end_s,
                    "heldout",
                    "the only run of its cell: after a 2.0 s guard",
                )
            )
    return segments


def _num(x: float) -> str:
    return f"{x:.3f}".rstrip("0").rstrip(".") if abs(x) < 1e9 else f"{x}"


def render(
    session: str,
    segments: Sequence[SplitSegment],
    rule_sha256: str,
    first_scan_log_s: dict[str, float],
    undriven_unused: Sequence[str] = (),
) -> str:
    """The yaml text of a session split (fixed layout, byte-stable)."""
    lines = [
        f"# Split of the session {session} by the registered rule (pnp-v0.4.yaml, future_sessions).",
        "# Written by tools/pnp_data.py split, before anything is fitted on the train part.",
        "# Do not edit after the first commit. Times are bag seconds (after the first /scan).",
        f"session: {session}",
        f"rule_file_sha256: {rule_sha256}",
        "rule:",
        f"  single_run_train_share: {_num(SINGLE_RUN_TRAIN_SHARE)}",
        f"  guard_s: {_num(GUARD_S)}",
        f"  min_run_s: {_num(MIN_RUN_S)}",
        "  multi_run_cell_heldout: last_run_of_the_cell",
        "  roles: the same for dynamics and perception; laps are not assigned",
    ]
    if undriven_unused:
        lines.append("owner_declared_unused:")
        lines.extend(f"  - {u}" for u in sorted(undriven_unused))
    lines.append("bags:")
    for bag in dict.fromkeys(s.bag for s in segments):
        lines.append(f"  {bag}:")
        lines.append(f"    first_scan_log_time_s: {first_scan_log_s.get(bag, 0.0):.3f}")
        lines.append("    segments:")
        for s in segments:
            if s.bag != bag:
                continue
            lines.append(
                f"      - {{name: {s.name}, run: {s.run}, from: {_num(s.t_from)}, "
                f"to: {_num(s.t_to)}, direction: {s.direction}, cap_mps: {_num(s.cap_mps)}, "
                f"dynamics: {s.role}, perception: {s.role}}}  # {s.why}"
            )
    return "\n".join(lines) + "\n"


def sha256_text(text: str) -> str:
    return hashlib.sha256(text.encode()).hexdigest()


def check_bags(bags: Sequence[str]) -> None:
    """Refuse a bag dated 2026-11-14 or earlier: those are split by hand and frozen."""
    for bag in bags:
        if bag_date(bag) <= LAST_HAND_SPLIT_DATE:
            raise SplitRefused(
                f"{Path(bag).name} is dated {bag_date(bag)}: bags of {LAST_HAND_SPLIT_DATE} and "
                "earlier are split by hand and frozen (pnp-v0.4.yaml, pnp-v0.2-split.yaml)"
            )


def write_split(
    session: str,
    runs: Sequence[SessionRun],
    rule_sha256: str,
    first_scan_log_s: dict[str, float],
    out_dir: Path = SESSIONS_DIR,
    undriven_unused: Sequence[str] = (),
) -> tuple[Path, str]:
    """Write `<out_dir>/<session>-split.yaml`; refuses to overwrite. Returns the path and the
    SHA-256 of the file."""
    if not re.fullmatch(r"\d{4}-\d{2}-\d{2}", session):
        raise SplitRefused(f"--session {session!r} is not a date YYYY-MM-DD")
    if session <= LAST_HAND_SPLIT_DATE:
        raise SplitRefused(f"session {session} is not after {LAST_HAND_SPLIT_DATE}")
    check_bags(sorted({r.bag for r in runs}))
    path = out_dir / f"{session}-split.yaml"
    if path.exists():
        raise SplitRefused(f"{path} exists; a split is never overwritten")
    text = render(
        session, split_runs(runs, undriven_unused), rule_sha256, first_scan_log_s, undriven_unused
    )
    yaml.safe_load(text)  # the file must parse
    out_dir.mkdir(parents=True, exist_ok=True)
    path.write_text(text)
    return path, sha256_text(text)
