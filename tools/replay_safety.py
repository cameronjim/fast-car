#!/usr/bin/env python3
"""Replay a recorded run through racer_safety's obstacle gate, before and after a rule change.

    uv run --project tools python tools/replay_safety.py data/bags/2026-10-25T19-27-08_car_teleop

Feeds the bag's /scan, /drive_raw (and /odom/wheel, when recorded) through safety_replay_cli, the
real C++ gate, twice: with the rule the October floor bags were driven with (`--before-set` to
change it) and with the committed vehicle_params rule (`--set` to try other values). Every
recorded brake is printed with what the car actually did after it (LiDAR scan matching: travel,
end clearance) and with the predicted end clearance under the rule after. Method:
tools/safety_replay/analysis.py. Nothing here talks to a car.
"""

from __future__ import annotations

import argparse
import sys
import tempfile
from pathlib import Path

sys.path.insert(0, str(Path(__file__).resolve().parent))

from follower_replay.bag import read_bag
from safety_replay import analysis
from safety_replay.motion import Body, Mount
from safety_replay.stream import (
    LEGACY_RULE,
    ReplayError,
    describe_cli,
    find_cli,
    run_cli,
    write_stream,
)


def parse_sets(items: list[str]) -> dict[str, float | bool]:
    params: dict[str, float | bool] = {}
    for item in items:
        key, sep, value = item.partition("=")
        if not sep or not key.strip():
            raise ReplayError(f"--set wants key=value, got {item!r}")
        text = value.strip().lower()
        if text in ("true", "false"):
            params[key.strip()] = text == "true"
        else:
            try:
                params[key.strip()] = float(text)
            except ValueError:
                raise ReplayError(f"--set {item!r}: not a number or true/false") from None
    return params


def geometry(described: dict) -> tuple[Mount, Body]:
    mount = Mount(
        described["lidar_mount_x_m"], described["lidar_mount_y_m"], described["laser_yaw_rad"]
    )
    body = Body(
        described["wheelbase_m"] + described["front_overhang_m"],
        described["rear_overhang_m"],
        described["width_m"] / 2.0,
    )
    return mount, body


def main(argv: list[str] | None = None) -> int:
    parser = argparse.ArgumentParser(description=__doc__.split("\n\n")[0])
    parser.add_argument("bags", nargs="+", help="bag directories or .mcap files")
    parser.add_argument("--cli", help="safety_replay_cli binary (else $SAFETY_REPLAY_CLI, ...)")
    parser.add_argument(
        "--set", action="append", default=[], help="rule AFTER: key=value on the committed rule"
    )
    parser.add_argument(
        "--before-set",
        action="append",
        default=[],
        help="rule BEFORE: key=value on top of the October floor rule",
    )
    parser.add_argument("--min-clearance", type=float, default=0.04, help="pass line (m)")
    parser.add_argument("--table", action="store_true", help="print the per-brake table")
    args = parser.parse_args(argv)
    try:
        cli = find_cli(args.cli)
        after_rule = parse_sets(args.set)
        before_rule = {**LEGACY_RULE, **parse_sets(args.before_set)}
        described = describe_cli(cli, after_rule)
        mount, body = geometry(described)
        failed = False
        for bag_path in args.bags:
            bag = read_bag(bag_path)
            if not bag.scans:
                raise ReplayError(f"{bag_path}: no /scan in the bag")
            with tempfile.TemporaryDirectory() as tmp:
                stream = Path(tmp) / "stream.txt"
                write_stream(bag, stream)
                before = run_cli(cli, stream, before_rule)
                after = run_cli(cli, stream, after_rule, ["--cycles"])
            report = analysis.analyse(
                bag,
                before,
                after,
                mount,
                body,
                trip_distance_m=analysis.trip_distance_for(described),
            )
            s = analysis.summary(report, args.min_clearance)
            print(f"bag: {bag_path}{'  (NOTE truncated/unindexed)' if bag.truncated else ''}")
            print(f"before: {before_rule}")
            print(f"after:  committed vehicle_params rule {after_rule or ''}")
            print(analysis.summary_text(s, args.min_clearance))
            t0 = analysis.arrival(min(bag.scans, key=analysis.arrival))
            for e in report.extra_trips:
                print(
                    f"  after-rule trip on no recorded approach at {e['t'] - t0:.2f} s ({e['cause']}, "
                    f"{e['distance_m']:.3f} m, request {e['speed_mps']:.2f} m/s "
                    f"{e['steering_rad']:+.3f} rad): car "
                    f"{'drove on freely' if e['drove_freely'] else 'stopped anyway'}"
                )
            if args.table:
                print(analysis.table(report.rows, t0))
            print()
            failed |= s["predicted_below"] > 0
        return 1 if failed else 0
    except ReplayError as error:
        print(f"replay_safety: {error}", file=sys.stderr)
        return 2


if __name__ == "__main__":
    sys.exit(main())
