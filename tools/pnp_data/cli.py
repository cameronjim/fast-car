"""Command line of tools/pnp_data.py.

    uv run --project tools --extra session python tools/pnp_data.py runs <bag>
    uv run --project tools --extra session python tools/pnp_data.py split <bag...> --session <date>
    uv run --project tools --extra session python tools/pnp_data.py split-v05 <bag...>
    uv run --project tools --extra session python tools/pnp_data.py guard <bag name...>
    uv run --project tools --extra session python tools/pnp_data.py confirm --bag <bag> \\
        --candidate <bundle> --deployed <bundle>

`runs` prints one row per run. `split-v05` writes
evaluation/prereg/sessions/2026-11-15-dynamics-v05.yaml by the v0.5 dynamics rule (every run, 55
percent train, 2.0 s guard, the rest held out; pnp-v0.5-dynamics.yaml) and prints its SHA-256.
`split` writes evaluation/prereg/sessions/<date>-split.yaml by the
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
from pnp_data import guard, runs, session_map_v05, session_split, session_split_v05

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


def cmd_runs_all(args: argparse.Namespace) -> int:
    sig = load_signals(args.bag, args.cache)
    rows = runs.rows_of_signals_all(sig)
    if args.json:
        doc = {"first_scan_log_time_s": sig.first_scan_log_s, "runs": [r.to_dict() for r in rows]}
        sys.stdout.write(json.dumps(doc, indent=2) + "\n")
    else:
        sys.stdout.write(runs.format_rows(rows) + "\n")
    return 0


def cmd_split_v05(args: argparse.Namespace) -> int:
    rule_file = args.rule_file or session_split_v05.SESSIONS_DIR.parents[2] / (
        session_split_v05.RULE_RELATIVE
    )
    rule = session_split_v05.load_rule(rule_file)
    session_split_v05.check_bag_names([b.name for b in args.bags])
    session_runs: list[session_split_v05.SessionRun] = []
    first_scan: dict[str, float] = {}
    for bag in sorted(args.bags, key=lambda b: b.name):
        sig = load_signals(bag, args.cache)
        first_scan[bag.name] = sig.first_scan_log_s
        session_runs.extend(
            session_split_v05.SessionRun(bag.name, row) for row in runs.rows_of_signals_all(sig)
        )
    path, sha = session_split_v05.write_split_v05(
        session_runs, rule, first_scan, out_dir=args.out_dir
    )
    sys.stdout.write(f"wrote {path}\nSHA-256 {sha}\n")
    return 0


def cmd_map_v05(args: argparse.Namespace) -> int:
    blocks = session_map_v05.read_blocks(args.logs)
    rows: list[session_map_v05.MapRow] = []
    for bag in sorted(args.bags, key=lambda b: b.name):
        sig = load_signals(bag, args.cache)
        rows.extend(
            session_map_v05.map_runs(
                bag.name, runs.rows_of_signals_all(sig), sig.first_scan_log_s, blocks
            )
        )
    sys.stdout.write(session_map_v05.render_table(rows))
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
    return confirm_mod.confirm(args.bag, args.candidate, args.deployed, v05=args.v05, runs=args.run)


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

    p = sub.add_parser("runs-v05", help="one row per run of a bag, v0.5 rule (every driver)")
    p.add_argument("bag", type=Path)
    p.add_argument("--json", action="store_true")
    p.add_argument("--cache", type=Path, default=None)
    p.set_defaults(fn=cmd_runs_all)

    p = sub.add_parser("split-v05", help="write the dynamics roles of the 2026-11-15 session")
    p.add_argument("bags", type=Path, nargs="+")
    p.add_argument("--rule-file", type=Path, default=None, help=argparse.SUPPRESS)
    p.add_argument(
        "--out-dir", type=Path, default=session_split_v05.SESSIONS_DIR, help=argparse.SUPPRESS
    )
    p.add_argument("--cache", type=Path, default=None)
    p.set_defaults(fn=cmd_split_v05)

    p = sub.add_parser("map-v05", help="runs of the session against the block launch logs")
    p.add_argument("bags", type=Path, nargs="+")
    p.add_argument("--logs", type=Path, required=True, help="directory of <block>.launch.log")
    p.add_argument("--cache", type=Path, default=None)
    p.set_defaults(fn=cmd_map_v05)

    p = sub.add_parser("guard", help="exit 1 if a bag has no committed split")
    p.add_argument("bags", nargs="+")
    p.add_argument("--sessions-dir", type=Path, default=session_split.SESSIONS_DIR)
    p.set_defaults(fn=cmd_guard)

    p = sub.add_parser("confirm", help="run the dynamics confirm on the PnP runs of a bag")
    p.add_argument("--bag", type=Path, required=True)
    p.add_argument("--candidate", type=Path, default=None)
    p.add_argument("--deployed", type=Path, default=None)
    p.add_argument(
        "--v05",
        action="store_true",
        help="the v0.5 form (GP2f-C item 2): contract 2.1 candidate, per direction, supported cells",
    )
    p.add_argument("--run", action="append", default=[], help="a named PnP run (with --v05)")
    p.set_defaults(fn=cmd_confirm)
    return parser


def main(argv: Sequence[str] | None = None) -> int:
    args = build_parser().parse_args(argv)
    try:
        return int(args.fn(args))
    except session_split.SplitRefused as error:
        sys.stderr.write(f"refused: {error}\n")
        return 2
