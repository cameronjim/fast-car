#!/usr/bin/env python3
"""Replay a recorded bag through the gap follower: what would it have steered, and why.

    uv run --project tools python tools/replay_follower.py data/bags/<run> \\
        --profile floor-2026-10-06 --cli <path to gap_follow_replay_cli> --plot

Reads /scan, /drive_raw, /drive and /safety/events from an mcap bag (a file or a bag directory;
a bag whose recorder was killed, with no footer, works), runs every scan through the same
GapFollower code gap_follow_node runs, prints a summary and writes per_scan.csv, summary.json,
and with --plot scans.png and steering.png into --out-dir. See docs/notes/replay-follower.md.
"""

from __future__ import annotations

import argparse
import sys
from pathlib import Path

from follower_replay import analysis, replay
from follower_replay.pipeline import PlotOptions, replay_bag

DEFAULT_PROFILE = "floor-2026-10-06"


def build_parser() -> argparse.ArgumentParser:
    parser = argparse.ArgumentParser(description=__doc__.split("\n\n")[0])
    parser.add_argument("bag", type=Path, help="an .mcap file, or a bag directory of .mcap files")
    parser.add_argument(
        "--profile",
        default=DEFAULT_PROFILE,
        help="a profile of gap_follow.launch.py (default %(default)s; `none` = node defaults)",
    )
    parser.add_argument(
        "--param",
        action="append",
        default=[],
        metavar="NAME=VALUE",
        help="override a gap_follow_node parameter on top of the profile (repeatable)",
    )
    parser.add_argument("--cli", help=f"path to {replay.CLI_NAME} (default: see docs)")
    parser.add_argument("--out-dir", type=Path, help="default: data/replay/<bag>-<profile>")
    parser.add_argument(
        "--straight-threshold-rad",
        type=float,
        default=analysis.DEFAULT_STRAIGHT_THRESHOLD_RAD,
        help="|steering| below this counts as straight (default %(default)s)",
    )
    parser.add_argument("--plot", action="store_true", help="write scans.png and steering.png")
    parser.add_argument(
        "--plot-scans",
        type=int,
        default=6,
        metavar="N",
        help="scans to draw, evenly spaced (default %(default)s)",
    )
    parser.add_argument(
        "--plot-at",
        default="",
        metavar="T,T,...",
        help="draw the scans nearest these times (s from the first scan)",
    )
    parser.add_argument(
        "--plot-flagged",
        action="store_true",
        help="pick the evenly spaced scans among clamped / corner-override ones",
    )
    return parser


def main(argv: list[str] | None = None) -> int:
    args = build_parser().parse_args(argv)
    try:
        cli = replay.find_cli(args.cli)
        overrides = replay.parse_overrides(args.param)
        at_s = [float(t) for t in args.plot_at.split(",") if t.strip()]
        out_dir = args.out_dir or (
            replay.REPO_ROOT
            / "data/replay"
            / f"{args.bag.name.removesuffix('.mcap')}-{args.profile}"
        )
        result = replay_bag(
            args.bag,
            out_dir,
            args.profile,
            overrides,
            cli,
            PlotOptions(args.plot, args.plot_scans, at_s, args.plot_flagged),
            args.straight_threshold_rad,
        )
    except (replay.ReplayError, FileNotFoundError, ValueError) as error:
        print(f"replay_follower: {error}", file=sys.stderr)
        return 2
    print(analysis.format_summary(result.summary))
    print("wrote " + ", ".join(str(p) for p in result.files.values()))
    return 0


if __name__ == "__main__":
    sys.exit(main())
