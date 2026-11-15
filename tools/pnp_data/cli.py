"""Command line of tools/pnp_data.py.

    uv run --project tools --extra session python tools/pnp_data.py runs <bag>
    uv run --project tools --extra session python tools/pnp_data.py split <bag...> --session <date>
    uv run --project tools --extra session python tools/pnp_data.py guard <bag name...>
    uv run --project tools --extra session python tools/pnp_data.py confirm --bag <bag> \\
        --candidate <bundle> --deployed <bundle>

`runs` prints one row per run. `split` writes evaluation/prereg/sessions/<date>-split.yaml by the
registered cut rule and prints its SHA-256; it never overwrites and refuses a bag of 2026-11-14 or
earlier. `guard` exits 1 if a bag has no committed split. `confirm` wraps the dynamics confirm.
"""

from __future__ import annotations

import argparse
import json
import sys
from collections.abc import Sequence
from pathlib import Path

from racer_eval.split import file_sha256
from racer_eval.split_v04 import PREREG_V04_SHA256, default_path

from pnp_data import confirm as confirm_mod
from pnp_data import guard, runs, session_split

DEFAULT_CACHE = Path("/Users/cameronjim/code/car-data/pnp/cache")


def load_signals(bag: Path, cache: Path | None) -> runs.RunSignals:
    path = cache / f"{bag.name}.pnp-data-runs.npz" if cache is not None else None
    if path is not None and path.is_file():
        return runs.RunSignals.load(path)
    sig = runs.read_run_signals(bag)
    if path is not None:
        path.parent.mkdir(parents=True, exist_ok=True)
        sig.save(path)
    return sig


def cmd_runs(args: argparse.Namespace) -> int:
    sig = load_signals(args.bag, args.cache)
    rows = runs.rows_of_signals(sig)
    if args.json:
        sys.stdout.write(json.dumps([r.to_dict() for r in rows], indent=2) + "\n")
    else:
        sys.stdout.write(runs.format_rows(rows) + "\n")
    return 0


def cmd_split(args: argparse.Namespace) -> int:
    rule_file = args.rule_file or default_path()
    digest = file_sha256(rule_file)
    if digest != PREREG_V04_SHA256:
        sys.stderr.write(
            f"{rule_file}: SHA-256 {digest} is not the registered {PREREG_V04_SHA256}\n"
        )
        return 2
    session_runs: list[session_split.SessionRun] = []
    first_scan: dict[str, float] = {}
    for bag in sorted(args.bags, key=lambda b: b.name):
        sig = load_signals(bag, args.cache)
        first_scan[bag.name] = sig.first_scan_log_s
        session_runs.extend(
            session_split.SessionRun(bag.name, row) for row in runs.rows_of_signals(sig)
        )
    path, sha = session_split.write_split(
        args.session,
        session_runs,
        digest,
        first_scan,
        out_dir=args.out_dir,
        undriven_unused=args.undriven_unused,
    )
    sys.stdout.write(f"wrote {path}\nSHA-256 {sha}\n")
    return 0


def cmd_guard(args: argparse.Namespace) -> int:
    status = 0
    for name in args.bags:
        try:
            found = guard.require_split(name, args.sessions_dir)
        except guard.NoCommittedSplit as error:
            sys.stdout.write(f"REFUSED {error}\n")
            status = 1
        else:
            sys.stdout.write(
                f"ok {name}: {'hand split (2026-11-14 or earlier)' if found is None else found}\n"
            )
    return status


def cmd_confirm(args: argparse.Namespace) -> int:
    return confirm_mod.confirm(args.bag, args.candidate, args.deployed)


def build_parser() -> argparse.ArgumentParser:
    parser = argparse.ArgumentParser(prog="pnp_data", description=__doc__.split("\n\n")[0])
    sub = parser.add_subparsers(dest="command", required=True)

    p = sub.add_parser("runs", help="one row per run of a bag")
    p.add_argument("bag", type=Path)
    p.add_argument("--json", action="store_true")
    p.add_argument("--cache", type=Path, default=None)
    p.set_defaults(fn=cmd_runs)

    p = sub.add_parser("split", help="write the split of a new session by the registered rule")
    p.add_argument("bags", type=Path, nargs="+")
    p.add_argument("--session", required=True, help="the session date, YYYY-MM-DD")
    p.add_argument("--rule-file", type=Path, default=None, help=argparse.SUPPRESS)
    p.add_argument(
        "--out-dir", type=Path, default=session_split.SESSIONS_DIR, help=argparse.SUPPRESS
    )
    p.add_argument("--cache", type=Path, default=None)
    p.add_argument(
        "--undriven-unused",
        action="append",
        default=[],
        metavar="BAG:RUN",
        help="declare a run in which the wheels did not turn unused (the rule is silent on it)",
    )
    p.set_defaults(fn=cmd_split)

    p = sub.add_parser("guard", help="exit 1 if a bag has no committed split")
    p.add_argument("bags", nargs="+")
    p.add_argument("--sessions-dir", type=Path, default=session_split.SESSIONS_DIR)
    p.set_defaults(fn=cmd_guard)

    p = sub.add_parser("confirm", help="run the dynamics confirm on the PnP runs of a bag")
    p.add_argument("--bag", type=Path, required=True)
    p.add_argument("--candidate", type=Path, default=None)
    p.add_argument("--deployed", type=Path, default=None)
    p.set_defaults(fn=cmd_confirm)
    return parser


def main(argv: Sequence[str] | None = None) -> int:
    args = build_parser().parse_args(argv)
    try:
        return int(args.fn(args))
    except session_split.SplitRefused as error:
        sys.stderr.write(f"refused: {error}\n")
        return 2
