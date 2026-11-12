#!/usr/bin/env python3
"""Replay a recorded run through racer_safety's obstacle gate, before and after a rule change.

    uv run --project tools python tools/replay_safety.py --recorded-with 0.13.0 \\
        data/bags/2026-10-25T19-27-08_car_teleop

Feeds the bag's /scan, /drive_raw (and /odom/wheel, when recorded) through safety_replay_cli, the
real C++ gate, twice: with the rule the bag was driven with (`--before-rule`, `--before-set` to
change it) on the commands exactly as recorded, and with the committed vehicle_params rule
(`--set` to try other values) on the commands in the committed units. Every recorded brake is
printed with what the car actually did after it (LiDAR scan matching: travel, end clearance) and
with the predicted end clearance under the rule after. Method: tools/safety_replay/analysis.py.

UNITS. `--recorded-with` is required because a bag does not say which calibration it was driven
with. `0.13.0` (every bag up to 2026-11-10) converts the bag's speeds (divide by 1.401) and
steering angles (old linear pulse map, then the measured per-side table) into the committed
units before the after-rule replay; `current` replays the bag as it is.
tools/safety_replay/calibration.py has the method. The before-rule replay always runs on the raw
bag, because that is what the car's gate saw.

`--sweep key=v1,v2,...` runs the after rule once per value and prints one summary row each over
all the bags (the re-tune trade-off table). Nothing here talks to a car.
"""

from __future__ import annotations

import argparse
import sys
import tempfile
from dataclasses import dataclass
from pathlib import Path
from typing import Any

sys.path.insert(0, str(Path(__file__).resolve().parent))

from follower_replay.bag import BagData, read_bag
from safety_replay import analysis, calibration, motion
from safety_replay.motion import Body, Mount, static_self_returns
from safety_replay.stream import (
    LEGACY_RULE,
    RULE_0_13_0,
    ReplayError,
    describe_cli,
    find_cli,
    run_cli,
    write_stream,
)

BEFORE_RULES = {"october": LEGACY_RULE, "0.13.0": RULE_0_13_0}
RECORDED = {"0.13.0": calibration.RECORDED_0_13_0, "current": None}


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


def parse_sweep(text: str) -> tuple[str, list[float]]:
    key, sep, values = text.partition("=")
    if not sep or not key.strip() or not values.strip():
        raise ReplayError(f"--sweep wants key=v1,v2,..., got {text!r}")
    try:
        return key.strip(), [float(v) for v in values.split(",")]
    except ValueError:
        raise ReplayError(f"--sweep {text!r}: not numbers") from None


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


@dataclass
class Prepared:
    """One bag, read once: the raw stream (before rule), the converted stream (after rule) and
    the bag-only measurements that do not depend on any rule."""

    path: str
    raw: BagData
    converted: BagData
    raw_stream: Path
    converted_stream: Path
    lock_rad: float | None  # steering lock the after rule must allow, None: the committed limit
    measurements: dict[float, tuple[float, float, float]]
    before: list[dict[str, Any]] | None = None
    # The car's own parts the LiDAR sees in (nearly) every scan, found on the first replay.
    ignore: frozenset[tuple[int, int]] | None = None


def prepare(
    path: str,
    workdir: Path,
    recorded: calibration.Recorded | None,
    current: calibration.Current | None,
    keep_recorded_steering: bool = False,
    bag: BagData | None = None,
) -> Prepared:
    raw = bag if bag is not None else read_bag(path)
    if not raw.scans:
        raise ReplayError(f"{path}: no /scan in the bag")
    if recorded is None or current is None:
        converted, lock = raw, None
    else:
        converted = calibration.convert_bag(
            raw, recorded, current, steering=not keep_recorded_steering
        )
        lock = None if keep_recorded_steering else current.steer_limit_rad
    index = len(list(workdir.glob("raw_*.txt")))
    raw_stream = workdir / f"raw_{index}.txt"
    converted_stream = workdir / f"converted_{index}.txt"
    write_stream(raw, raw_stream)
    write_stream(converted, converted_stream)
    return Prepared(path, raw, converted, raw_stream, converted_stream, lock, {})


def replay(
    prep: Prepared,
    cli: Path,
    before_rule: dict[str, float | bool],
    after_rule: dict[str, float | bool],
    mount: Mount,
    body: Body,
    described: dict[str, Any],
) -> analysis.Report:
    """The before rule on the raw stream (cached on `prep`), the after rule on the converted one."""
    if prep.before is None:
        prep.before = run_cli(cli, prep.raw_stream, before_rule)
    if prep.ignore is None:
        prep.ignore = static_self_returns(prep.raw.scans, mount, body)
    rule = dict(after_rule)
    if prep.lock_rad is not None:
        rule.setdefault(
            "max_steering_rad", max(prep.lock_rad, described.get("max_steering_rad", 0.0))
        )
    after = run_cli(cli, prep.converted_stream, rule, ["--cycles"])
    return analysis.analyse(
        prep.converted,
        prep.before,
        after,
        mount,
        body,
        measurements=prep.measurements,
        trip_distance_m=analysis.trip_distance_for(described),
        ignore=prep.ignore,
    )


def _sweep_row(value: float, summaries: list[dict[str, Any]]) -> str:
    finite = [
        s["predicted_min_m"] for s in summaries if s["predicted_min_m"] == s["predicted_min_m"]
    ]
    lowest = f"{min(finite) * 100.0:.1f}" if finite else "-"

    def total(key: str) -> int:
        return sum(s[key] for s in summaries)

    return (
        f"| {value:g} | {total('recorded_brakes')} | {total('after_later')} | "
        f"{total('predicted_below')} | {lowest} | "
        f"{total('extra_trips')} | {total('extra_trips_free')} |"
    )


SWEEP_HEAD = (
    "| value | recorded brakes | later than recorded | predicted under {limit:.0f} cm | "
    "min predicted (cm) | new brakes | new brakes where the car drove on |\n"
    "|---|---|---|---|---|---|---|"
)


def main(argv: list[str] | None = None) -> int:
    parser = argparse.ArgumentParser(description=__doc__.split("\n\n")[0])
    parser.add_argument("bags", nargs="+", help="bag directories or .mcap files")
    parser.add_argument(
        "--recorded-with",
        required=True,
        choices=sorted(RECORDED),
        help="the vehicle_params calibration the bag was driven with: 0.13.0 (every bag up to "
        "2026-11-10, converted to the committed units) or current (replayed as is)",
    )
    parser.add_argument("--cli", help="safety_replay_cli binary (else $SAFETY_REPLAY_CLI, ...)")
    parser.add_argument(
        "--set", action="append", default=[], help="rule AFTER: key=value on the committed rule"
    )
    parser.add_argument(
        "--before-rule",
        choices=sorted(BEFORE_RULES),
        default="october",
        help="the rule the bag was driven with: october (0.12.0, the 2026-10 bags) or 0.13.0",
    )
    parser.add_argument(
        "--before-set",
        action="append",
        default=[],
        help="rule BEFORE: key=value on top of --before-rule",
    )
    parser.add_argument(
        "--sweep", help="key=v1,v2,...: the after rule once per value, one summary row each"
    )
    parser.add_argument(
        "--keep-recorded-steering",
        action="store_true",
        help="diagnostic: convert speeds but keep the recorded steering angles, i.e. the old "
        "(wrong) turning radius",
    )
    parser.add_argument(
        "--window",
        nargs=2,
        type=float,
        metavar=("START", "END"),
        help="read only this stretch of every bag, in seconds from its first message (an indexed "
        "bag opens in a second; an unindexed one is read whole and the window ignored)",
    )
    parser.add_argument("--min-clearance", type=float, default=0.04, help="pass line (m)")
    parser.add_argument("--table", action="store_true", help="print the per-brake table")
    args = parser.parse_args(argv)
    try:
        cli = find_cli(args.cli)
        after_rule = parse_sets(args.set)
        before_rule = {**BEFORE_RULES[args.before_rule], **parse_sets(args.before_set)}
        recorded = RECORDED[args.recorded_with]
        current = calibration.load_current() if recorded is not None else None
        sweep = parse_sweep(args.sweep) if args.sweep else None
        failed = False
        with tempfile.TemporaryDirectory() as tmp:
            described = describe_cli(cli, after_rule)
            mount, body = geometry(described)
            prepared = [
                prepare(
                    path,
                    Path(tmp),
                    recorded,
                    current,
                    args.keep_recorded_steering,
                    bag=read_bag(path, window_s=tuple(args.window)) if args.window else None,
                )
                for path in args.bags
            ]
            if recorded is not None:
                print(
                    f"units: bags converted from {recorded.label} (speed / {recorded.speed_ratio}, "
                    "steering through the committed pwm_to_angle_table"
                    f"{', KEPT as recorded' if args.keep_recorded_steering else ''})"
                )
            if sweep is not None:
                key, values = sweep
                print(SWEEP_HEAD.format(limit=args.min_clearance * 100.0))
                for value in values:
                    summaries = []
                    for prep in prepared:
                        rule = {**after_rule, key: value}
                        report = replay(
                            prep, cli, before_rule, rule, mount, body, describe_cli(cli, rule)
                        )
                        summaries.append(analysis.summary(report, args.min_clearance))
                    print(_sweep_row(value, summaries), flush=True)
                return 0
            for prep in prepared:
                report = replay(prep, cli, before_rule, after_rule, mount, body, described)
                s = analysis.summary(report, args.min_clearance)
                bag = prep.raw
                print(f"bag: {prep.path}{'  (NOTE truncated/unindexed)' if bag.truncated else ''}")
                print(f"before: {args.before_rule} {before_rule}")
                print(f"after:  committed vehicle_params rule {after_rule or ''}")
                if prep.ignore:
                    print(
                        f"car's own static returns ignored in every measurement: "
                        f"{len(prep.ignore)} cell(s) of {motion.SELF_CELL_M * 100:.0f} cm "
                        "inside the body in at least half of the scans"
                    )
                print(analysis.summary_text(s, args.min_clearance))
                t0 = analysis.arrival(min(bag.scans, key=analysis.arrival))
                for e in report.extra_trips:
                    print(
                        f"  after-rule trip on no recorded approach at {e['t'] - t0:.2f} s "
                        f"({e['cause']}, {e['distance_m']:.3f} m, request {e['speed_mps']:.2f} "
                        f"m/s {e['steering_rad']:+.3f} rad): car "
                        f"{'drove on freely' if e['drove_freely'] else 'stopped anyway'}"
                    )
                if args.table:
                    print(analysis.table(report.rows, t0))
                print()
                failed |= s["predicted_below"] > 0
        return 1 if failed else 0
    except (ReplayError, FileNotFoundError, ValueError) as error:
        print(f"replay_safety: {error}", file=sys.stderr)
        return 2


if __name__ == "__main__":
    sys.exit(main())
