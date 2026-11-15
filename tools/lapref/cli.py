"""Command line of tools/lapref.py.

    uv run --project tools --extra session python tools/lapref.py <bag> --run <start> <end>
    uv run --project tools --extra session python tools/lapref.py <bag> --split-run <name>

`<start>` and `<end>` are bag seconds (after the first /scan). `--split-run` takes the whole run of
the 2026-11-14 split (pnp-v0.4.yaml: a cut run's a and b parts and the 2.0 s between them). A lap
`test` run is refused unless `--scoring-test` is given: the test runs are scored once, after the
method is frozen (pre-registration v0.4 section 4.3).
"""

from __future__ import annotations

import argparse
import json
import subprocess
import sys
from collections.abc import Sequence
from pathlib import Path

from lapref.bag import Signals, read_signals
from lapref.method import LaprefResult, lapref


class TestRunRefused(PermissionError):
    __test__ = False


def laps_of_run(signals: Signals, t_from: float, t_to: float) -> LaprefResult:
    if t_to <= t_from:
        raise ValueError(f"the run ends ({t_to}) before it starts ({t_from})")
    w = signals.window(t_from, t_to)
    return lapref(w.odom_t, w.odom_v, w.steer_t, w.steer)


def _git_head() -> str:
    try:
        out = subprocess.run(
            ["git", "rev-parse", "HEAD"], capture_output=True, text=True, check=True
        )
        return out.stdout.strip()
    except (OSError, subprocess.CalledProcessError):
        return "unknown"


def resolve_split_run(name: str, bag: Path, scoring_test: bool) -> tuple[float, float, str]:
    """(start, end, role) of a whole run of the v0.4 split, checked against the bag's name."""
    from racer_eval.split_v04 import load_v04

    run = load_v04().lap_run(name)
    if Path(bag).name != run.bag:
        raise ValueError(f"run {name} is in bag {run.bag}, not {Path(bag).name}")
    if run.role == "test" and not scoring_test:
        raise TestRunRefused(
            f"{name} is a lap test run: it is scored once, after the freeze commit. "
            "Pass --scoring-test to do that."
        )
    return run.t_from, run.t_to, run.role


def main(argv: Sequence[str] | None = None) -> int:
    parser = argparse.ArgumentParser(prog="lapref", description=__doc__.split("\n\n")[0])
    parser.add_argument("bag", type=Path, help="bag directory or .mcap file")
    where = parser.add_mutually_exclusive_group(required=True)
    where.add_argument("--run", nargs=2, type=float, metavar=("START", "END"))
    where.add_argument("--split-run", metavar="NAME")
    parser.add_argument("--scoring-test", action="store_true", help="allow a lap test run")
    parser.add_argument("--out", type=Path, help="also write the JSON here")
    parser.add_argument("--cache", type=Path, help="directory for the decoded signals (npz)")
    args = parser.parse_args(argv)
    role = None
    if args.split_run:
        t_from, t_to, role = resolve_split_run(args.split_run, args.bag, args.scoring_test)
    else:
        t_from, t_to = args.run
    cache = args.cache / f"{args.bag.name}.lapref-signals.npz" if args.cache else None
    if cache is not None and cache.is_file():
        signals = Signals.load(cache)
    else:
        signals = read_signals(args.bag)
        if cache is not None:
            cache.parent.mkdir(parents=True, exist_ok=True)
            signals.save(cache)
    result = laps_of_run(signals, t_from, t_to).to_dict()
    result.update(
        {
            "bag": args.bag.name,
            "run": [t_from, t_to],
            "split_run": args.split_run,
            "role": role,
            "git_head": _git_head() if role == "test" else None,
        }
    )
    text = json.dumps(result, indent=2)
    sys.stdout.write(text + "\n")
    if args.out:
        args.out.write_text(text + "\n")
    return 0 if result["n_laps"] else 1
