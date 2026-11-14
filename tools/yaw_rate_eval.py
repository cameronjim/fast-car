#!/usr/bin/env python3
"""Yaw rate from scan matching against the map pose, offline (roadmap P2.2, prereg section 3.4).

    uv run --project tools --extra plots python tools/yaw_rate_eval.py \\
        --split evaluation/prereg/pnp-v0.2-split.yaml \\
        --bags-root /Users/cameronjim/code/car-data/bags --out data/yaw_rate

For the segments that have /pose (G1a, G1b, T0, T1, T2a, T2b, T3a, T3b of the pre-registered
split; --extra-segments adds G2, G3a, G3b as a clearly labelled supplement) it measures the
scan-to-scan ICP yaw (tools/safety_replay/motion.py with the dynamics package's quality flags)
and the wheel model's yaw rate (/odom/wheel_twist) against the yaw change between consecutive
/pose messages: interval-mean bias and RMS, 10 Hz closure noise, delay, rejection share, CPU
cost. Writes summary.json, tables.md and one PNG per segment. Nothing runs on the car; the split
file is read, never written. docs/notes/yaw-rate-scan-matching-2026-11-13.md.
"""

from __future__ import annotations

import argparse
import logging
import sys
from pathlib import Path
from typing import Any

import yaml

sys.path.insert(0, str(Path(__file__).resolve().parent))

from yaw_rate import bagio, pipeline, report, run, vehicle

log = logging.getLogger("yaw_rate_eval")
CACHE_DIR = Path("/Users/cameronjim/code/car-data/pnp/cache")
PREREG_SEGMENTS = ("G1a", "G1b", "T0", "T1", "T2a", "T2b", "T3a", "T3b")
EXTRA_SEGMENTS = ("G2", "G3a", "G3b")


def main(argv: list[str] | None = None) -> int:
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawTextHelpFormatter)
    ap.add_argument("--split", type=Path, required=True)
    ap.add_argument("--bags-root", type=Path, required=True)
    ap.add_argument("--out", type=Path, required=True)
    ap.add_argument("--cache", type=Path, default=CACHE_DIR)
    ap.add_argument("--workers", type=int, default=1, help="processes for the ICP matching")
    ap.add_argument("--extra-segments", action="store_true", help="also G2, G3a, G3b (supplement)")
    ap.add_argument(
        "--min-interval-s",
        type=float,
        default=0.0,
        help="merge consecutive /pose messages up to this interval (0: consecutive, as pre-registered)",
    )
    ap.add_argument("--segments", nargs="*", help="override the segment list")
    ap.add_argument("--no-plots", action="store_true")
    ap.add_argument("--cpu-bench", action="store_true", help="time the matcher per scan")
    args = ap.parse_args(argv)
    logging.basicConfig(level=logging.INFO, format="%(message)s")

    split: dict[str, Any] = yaml.safe_load(args.split.read_text())
    names = tuple(args.segments) if args.segments else PREREG_SEGMENTS
    if args.extra_segments and not args.segments:
        names = names + EXTRA_SEGMENTS
    specs = run.split_segments(split, names)
    veh = vehicle.load_vehicle()
    log.info("vehicle_params schema %s, wheelbase %.3f m", veh.schema_version, veh.wheelbase_m)

    recs: dict[str, bagio.Recording] = {}
    results: list[pipeline.SegmentResult] = []
    cpu: dict[str, float] | None = None
    for spec in specs:
        if spec.bag not in recs:
            recs[spec.bag] = bagio.load_recording(
                args.bags_root / spec.bag, args.cache / f"yaw_rate_{spec.bag}.npz"
            )
        rec = recs[spec.bag]
        log.info(
            "%s (%s, %s, %.1f to %.1f s)",
            spec.name,
            spec.bag,
            spec.direction,
            spec.t_from,
            spec.t_to,
        )
        inp = run.build_segment_input(
            rec,
            spec,
            veh,
            workers=args.workers,
            icp_cache=args.cache / f"yaw_rate_icp_{spec.bag}_{spec.name}.npz",
        )
        res = pipeline.evaluate_segment(inp, args.min_interval_s)
        results.append(res)
        if args.cpu_bench and cpu is None:
            cpu = run.cpu_cost_per_scan_s(rec, run.segment_scans(rec, spec), veh)
            log.info("  cpu per scan pair: %s", cpu)
        if not args.no_plots:
            report.plot_segment(res, args.out / "plots" / f"{spec.name}.png")

    summary = pipeline.summarise(results)
    summary["cpu_per_scan_pair"] = cpu
    summary["min_interval_s"] = args.min_interval_s
    summary["segments"] = list(names)
    report.write_json(summary, args.out / "summary.json")
    tables = report.markdown_tables(summary)
    (args.out / "tables.md").write_text(tables)
    print(tables)
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
