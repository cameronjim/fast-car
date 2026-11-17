"""The dynamics roles of the 2026-11-15 session by the rule of pnp-v0.5-dynamics.yaml,
block `session_2026_11_15`. One rule for EVERY run of EVERY bag of the session, whatever drove it:

* a run shorter than `min_run_s` (15 s) is unused;
* a run with less than `min_wheel_distance_m` (1.0 m) of wheel distance is unused (the wheels did
  not turn);
* otherwise the first `train_share` (55 percent) of its duration is train, the next `guard_s`
  (2.0 s) is unused, the rest is held out.

There are no cells and no labels, so nothing is chosen after looking. The thresholds are READ from
the yaml (whose SHA-256 must be the registered one); nothing is typed in here. A run is as
`pnp_data.runs.run_rows_all` finds it: a stretch of /pnp/planner/status with no gap above 2 s, or
a stretch of non-zero /drive_raw commands that no status stretch covers (in a bag without status
that is the yaml's own second clause; in a bag that mixes both it is the same clause applied to the
stretches the status does not cover). The direction of a run is NOT decided here: segments carry
`direction: by_heading` and every window is labelled later from the measured T2 heading; the
commanded direction is recorded as `command_direction` for information.

The file is written by a fixed formatter, not a YAML dumper, so it is byte-stable. It carries the
SHA-256 of the rule file.
"""

from __future__ import annotations

import hashlib
import re
from collections.abc import Sequence
from dataclasses import dataclass
from pathlib import Path

import yaml

from pnp_data.runs import RunRow
from pnp_data.session_split import SplitRefused, bag_date

#: SHA-256 of evaluation/prereg/pnp-v0.5-dynamics.yaml as pre-registered (PR 196).
RULE_SHA256 = "2767f297d951c11683b40834b864b832e8cbd0b01343e2765681e11b22e86a82"
RULE_RELATIVE = Path("evaluation") / "prereg" / "pnp-v0.5-dynamics.yaml"
SESSIONS_DIR = Path(__file__).resolve().parents[2] / "evaluation" / "prereg" / "sessions"
#: The yaml names the session by the planned date; the bags carry the UTC date of the car clock.
SESSION = "2026-11-15"
FILE_NAME = "2026-11-15-dynamics-v05.yaml"
DIRECTION = "by_heading"


@dataclass(frozen=True)
class Rule:
    min_run_s: float
    min_wheel_distance_m: float
    train_share: float
    guard_s: float
    sha256: str


def load_rule(path: Path, expected_sha256: str = RULE_SHA256) -> Rule:
    """The numbers of `session_2026_11_15`, after checking the SHA-256 of the file."""
    blob = Path(path).read_bytes()
    digest = hashlib.sha256(blob).hexdigest()
    if digest != expected_sha256:
        raise SplitRefused(
            f"{path}: SHA-256 {digest} is not the registered {expected_sha256}; "
            "the v0.5 constants file must not change"
        )
    block = yaml.safe_load(blob.decode("utf-8"))["session_2026_11_15"]
    return Rule(
        min_run_s=float(block["min_run_s"]),
        min_wheel_distance_m=float(block["min_wheel_distance_m"]),
        train_share=float(block["train_share"]),
        guard_s=float(block["guard_s"]),
        sha256=digest,
    )


@dataclass(frozen=True)
class SessionRun:
    bag: str
    row: RunRow


@dataclass(frozen=True)
class Part:
    name: str
    bag: str
    run: str
    kind: str
    t_from: float
    t_to: float
    command_direction: str
    role: str  # train | heldout | unused
    why: str


def split_runs_v05(runs: Sequence[SessionRun], rule: Rule) -> list[Part]:
    """Parts of every run by the rule. Runs are named S1, S2, ... in the order given (bag order,
    then time); a run with a train and a held-out part gives S<n>a and S<n>b."""
    parts: list[Part] = []
    for i, r in enumerate(runs):
        row, name = r.row, f"S{i + 1}"

        def part(
            suffix: str,
            a: float,
            b: float,
            role: str,
            why: str,
            r: SessionRun = r,
            name: str = name,
        ) -> Part:
            return Part(
                name + suffix, r.bag, r.row.run_id, r.row.kind, a, b, r.row.direction, role, why
            )

        if row.duration_s < rule.min_run_s:
            parts.append(
                part("", row.start_s, row.end_s, "unused", f"shorter than {rule.min_run_s:g} s")
            )
        elif row.wheel_distance_m < rule.min_wheel_distance_m:
            parts.append(
                part(
                    "",
                    row.start_s,
                    row.end_s,
                    "unused",
                    f"under {rule.min_wheel_distance_m:g} m of wheel distance",
                )
            )
        else:
            cut = row.start_s + rule.train_share * row.duration_s
            parts.append(
                part(
                    "a",
                    row.start_s,
                    cut,
                    "train",
                    f"the first {rule.train_share * 100:g} percent of the run",
                )
            )
            parts.append(
                part(
                    "b",
                    cut + rule.guard_s,
                    row.end_s,
                    "heldout",
                    f"after a {rule.guard_s:g} s guard",
                )
            )
    return parts


def _num(x: float) -> str:
    return f"{x:.3f}".rstrip("0").rstrip(".") if abs(x) < 1e9 else f"{x}"


def render_v05(parts: Sequence[Part], rule: Rule, first_scan_log_s: dict[str, float]) -> str:
    """The yaml text of the session file (fixed layout, byte-stable)."""
    lines = [
        f"# Dynamics roles of the session {SESSION} by the registered rule (pnp-v0.5-dynamics.yaml,",
        "# block session_2026_11_15): every run of every bag, whatever drove it.",
        "# Written by tools/pnp_data.py split-v05 before anything is fitted on a train part.",
        "# Do not edit after the first commit. Times are bag seconds (after the first /scan).",
        "# The label 2026-11-15 is the planned date of the session; the bags carry UTC dates.",
        f'session: "{SESSION}"',
        "prereg_id: pnp-v0.5-dynamics",
        f"rule_file_sha256: {rule.sha256}",
        "rule:",
        f"  min_run_s: {_num(rule.min_run_s)}",
        f"  min_wheel_distance_m: {_num(rule.min_wheel_distance_m)}",
        f"  train_share: {_num(rule.train_share)}",
        f"  guard_s: {_num(rule.guard_s)}",
        "  heldout: the rest of the run",
        "  roles: dynamics only; no cells and no labels; the direction of a window is its measured heading",
        "bags:",
    ]
    for bag in dict.fromkeys(p.bag for p in parts):
        lines.append(f"  {bag}:")
        lines.append(f"    first_scan_log_time_s: {first_scan_log_s.get(bag, 0.0):.3f}")
        lines.append("    segments:")
        for p in parts:
            if p.bag != bag:
                continue
            lines.append(
                f"      - {{name: {p.name}, run: {p.run}, kind: {p.kind}, from: {_num(p.t_from)}, "
                f"to: {_num(p.t_to)}, direction: {DIRECTION}, command_direction: "
                f"{p.command_direction}, dynamics: {p.role}}}  # {p.why}"
            )
    return "\n".join(lines) + "\n"


def sha256_text(text: str) -> str:
    return hashlib.sha256(text.encode()).hexdigest()


def write_split_v05(
    runs: Sequence[SessionRun],
    rule: Rule,
    first_scan_log_s: dict[str, float],
    out_dir: Path = SESSIONS_DIR,
) -> tuple[Path, str]:
    """Write `<out_dir>/2026-11-15-dynamics-v05.yaml`; never overwrites. Returns the path and the
    SHA-256 of the file. Every bag must carry a date after 2026-11-14 (the older ones are split by
    hand and frozen)."""
    for bag in sorted({r.bag for r in runs}):
        if bag_date(bag) <= "2026-11-14":
            raise SplitRefused(f"{bag} is dated 2026-11-14 or earlier: split by hand and frozen")
    path = out_dir / FILE_NAME
    if path.exists():
        raise SplitRefused(f"{path} exists; a split is never overwritten")
    text = render_v05(split_runs_v05(runs, rule), rule, first_scan_log_s)
    yaml.safe_load(text)  # the file must parse
    out_dir.mkdir(parents=True, exist_ok=True)
    path.write_text(text)
    return path, sha256_text(text)


_BAG_NAME = re.compile(r"^\d{4}-\d{2}-\d{2}T\d{2}-\d{2}-\d{2}_")


def check_bag_names(bags: Sequence[str]) -> None:
    for b in bags:
        if _BAG_NAME.match(Path(b).name) is None:
            raise SplitRefused(f"{b}: not a bag name of the form YYYY-MM-DDTHH-MM-SS_...")
