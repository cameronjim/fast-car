"""The dynamics roles of the 2026-11-22 session for the yaw-only residual attempt R6 (brief 54,
pre-registration `evaluation/prereg/pnp-v0.6-dynamics-yaw.yaml`, block `session_2026_11_22`).

The rule is the v0.5 rule of `session_2026_11_15`, unchanged (`session_split_v05.split_runs_v05`):
a run shorter than 15 s or with under 1.0 m of wheel distance is unused, otherwise the first 55
percent of its duration is train, the next 2.0 s is unused and the rest is held out. The numbers
are READ from the v0.6 yaml (whose SHA-256 must be the registered one) and must equal those of the
v0.5 yaml; nothing is typed in here.

Two more refusals, because the bag has a short copy under the same name (155 MB, cut short, two
runs only, `not_this_copy` in the yaml): the bag name must be the registered one, and the runs the
tool finds must be the registered run list (`run_list_2026_11_22`: same ids, kinds and directions,
start and end within `RUN_LIST_TOLERANCE_S`). The file is written once and never overwritten.
"""

from __future__ import annotations

import hashlib
from collections.abc import Mapping, Sequence
from dataclasses import dataclass
from pathlib import Path

import yaml

from pnp_data import session_split_v05
from pnp_data.session_split import SplitRefused
from pnp_data.session_split_v05 import Part, Rule, SessionRun, _num, split_runs_v05

#: SHA-256 of evaluation/prereg/pnp-v0.6-dynamics-yaw.yaml as pre-registered (brief 54).
RULE_SHA256 = "db62eaac18ba655bf00e4ff7c8e8b287cd7e042b6ea671014ebb404032d00e0c"
RULE_RELATIVE = Path("evaluation") / "prereg" / "pnp-v0.6-dynamics-yaw.yaml"
SESSIONS_DIR = session_split_v05.SESSIONS_DIR
SESSION = "2026-11-22"
FILE_NAME = "2026-11-22-dynamics-v06.yaml"
BLOCK = "session_2026_11_22"
#: How far a run the tool finds may lie from the registered run list (the yaml gives 0.1 s).
RUN_LIST_TOLERANCE_S = 0.5


@dataclass(frozen=True)
class RuleV06:
    rule: Rule
    bag: str  # the registered bag name
    not_this_copy: str  # the short copy's path, refused
    run_list: Mapping[str, Mapping[str, object]]
    file: str


def load_rule_v06(
    path: Path,
    v05_path: Path | None = None,
    expected_sha256: str = RULE_SHA256,
) -> RuleV06:
    """The numbers of `session_2026_11_22`, after checking the SHA-256 of the v0.6 file and that
    the numbers equal the v0.5 rule they say they copy."""
    blob = Path(path).read_bytes()
    digest = hashlib.sha256(blob).hexdigest()
    if digest != expected_sha256:
        raise SplitRefused(
            f"{path}: SHA-256 {digest} is not the registered {expected_sha256}; "
            "the v0.6 constants file must not change"
        )
    doc = yaml.safe_load(blob.decode("utf-8"))
    block = doc[BLOCK]
    rule = Rule(
        min_run_s=float(block["min_run_s"]),
        min_wheel_distance_m=float(block["min_wheel_distance_m"]),
        train_share=float(block["train_share"]),
        guard_s=float(block["guard_s"]),
        sha256=digest,
    )
    if v05_path is None:
        v05_path = Path(path).resolve().parents[2] / session_split_v05.RULE_RELATIVE
    v05 = session_split_v05.load_rule(v05_path)
    for name in ("min_run_s", "min_wheel_distance_m", "train_share", "guard_s"):
        if getattr(rule, name) != getattr(v05, name):
            raise SplitRefused(
                f"{BLOCK}.{name} {getattr(rule, name)} is not the v0.5 rule's {getattr(v05, name)}"
            )
    return RuleV06(
        rule=rule,
        bag=Path(str(block["bag"])).name,
        not_this_copy=str(block["not_this_copy"]),
        run_list=doc["run_list_2026_11_22"],
        file=str(block["file"]),
    )


def check_bag(bag: Path, r: RuleV06) -> None:
    """The bag must carry the registered name and must not be the short copy."""
    if bag.name != r.bag:
        raise SplitRefused(f"{bag.name} is not the registered bag {r.bag}")
    if str(Path(bag).resolve()) == str(Path(r.not_this_copy).resolve()):
        raise SplitRefused(f"{bag} is the short copy (not_this_copy); use the 417 MB file")


def check_run_list(runs: Sequence[SessionRun], r: RuleV06) -> None:
    """The runs found must be the registered run list (ids, kind, direction, times)."""
    found = {x.row.run_id: x.row for x in runs}
    if set(found) != set(r.run_list):
        raise SplitRefused(
            f"runs {sorted(found)} are not the registered run list {sorted(r.run_list)} "
            "(the short copy has two runs only)"
        )
    for rid, want in r.run_list.items():
        row = found[rid]
        if row.kind != str(want["kind"]) or row.direction != str(want["direction"]):
            raise SplitRefused(f"{rid}: {row.kind} {row.direction} is not the registered run")
        for got, key in ((row.start_s, "start_s"), (row.end_s, "end_s")):
            if abs(got - float(want[key])) > RUN_LIST_TOLERANCE_S:
                raise SplitRefused(f"{rid}: {key} {got:.3f} is not the registered {want[key]}")


def render_v06(parts: Sequence[Part], rule: Rule, first_scan_log_s: dict[str, float]) -> str:
    """The yaml text of the session file (fixed layout, byte-stable)."""
    lines = [
        f"# Dynamics roles of the session {SESSION} for the yaw-only residual attempt R6 (brief 54,",
        "# pnp-v0.6-dynamics-yaw.yaml, block session_2026_11_22: the v0.5 rule, unchanged).",
        "# Written by tools/pnp_data.py split-v06 before anything is fitted on a train part.",
        "# Do not edit after the first commit. Times are bag seconds (after the first /scan).",
        "# The label 2026-11-22 is the date of the session; the bag carries the UTC date.",
        f'session: "{SESSION}"',
        "prereg_id: pnp-v0.6-dynamics-yaw",
        f"rule_file_sha256: {rule.sha256}",
        "rule:",
        f"  min_run_s: {_num(rule.min_run_s)}",
        f"  min_wheel_distance_m: {_num(rule.min_wheel_distance_m)}",
        f"  train_share: {_num(rule.train_share)}",
        f"  guard_s: {_num(rule.guard_s)}",
        "  heldout: the rest of the run",
        "  roles: dynamics only; train is fitted on, heldout decides GP2g-R once after the freeze",
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
                f"to: {_num(p.t_to)}, direction: {session_split_v05.DIRECTION}, "
                f"command_direction: {p.command_direction}, dynamics: {p.role}}}  # {p.why}"
            )
    return "\n".join(lines) + "\n"


def write_split_v06(
    runs: Sequence[SessionRun],
    r: RuleV06,
    first_scan_log_s: dict[str, float],
    out_dir: Path = SESSIONS_DIR,
) -> tuple[Path, str]:
    """Write `<out_dir>/2026-11-22-dynamics-v06.yaml`; never overwrites. Returns the path and the
    SHA-256 of the file."""
    for bag in {x.bag for x in runs}:
        if bag != r.bag:
            raise SplitRefused(f"{bag} is not the registered bag {r.bag}")
    check_run_list(runs, r)
    path = out_dir / FILE_NAME
    if path.exists():
        raise SplitRefused(f"{path} exists; a split is never overwritten")
    text = render_v06(split_runs_v05(runs, r.rule), r.rule, first_scan_log_s)
    yaml.safe_load(text)  # the file must parse
    out_dir.mkdir(parents=True, exist_ok=True)
    path.write_text(text)
    return path, session_split_v05.sha256_text(text)
