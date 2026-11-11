#!/usr/bin/env python3
"""Driver comparison on the track family: runs tests/l5_reactive_bench/track_family_runner.py over
a list of (track, direction, driver) runs side by side and prints markdown tables.

Needs the ROS workspace sourced (see tools/track_bench_docker.sh, which runs this inside the
ros-dev image with no network). Every run gets its own ROS_DOMAIN_ID; results are one JSON file
each under --out, and a run whose JSON exists is skipped, so a killed sweep resumes.

    tools/track_bench.py run --suite family --driver tracker --out /data/runs/family --jobs 6
    tools/track_bench.py report /data/runs/family
    tools/track_bench.py list --suite family

A suite is a set of tracks (`TRACK_SETS`); the driver variants are given on the command line as
`--variant name=JSON` (default: the three stock ones). See docs/notes/track-session-2026-11-11.md
for the sweeps this produced.
"""

from __future__ import annotations

import argparse
import json
import queue
import subprocess
import sys
import time
from collections import defaultdict
from concurrent.futures import ThreadPoolExecutor, as_completed
from pathlib import Path

REPO_ROOT = Path(__file__).resolve().parents[1]
RUNNER = REPO_ROOT / "tests" / "l5_reactive_bench" / "track_family_runner.py"
OVAL_MAP = "/data/maps/2026-11-10-oval/oval.yaml"

# name -> (track, track params). Lane width W, island (end) radius r, straights S in metres.
TRACK_SETS: dict[str, dict[str, tuple[str, dict]]] = {
    "family": {
        "st_w1.0_s2_r0.0": (
            "stadium",
            {"lane_width_m": 1.0, "straight_length_m": 2.0, "island_radius_m": 0.0},
        ),
        "st_w1.0_s2_r0.2": (
            "stadium",
            {"lane_width_m": 1.0, "straight_length_m": 2.0, "island_radius_m": 0.2},
        ),
        "st_w1.0_s2_r0.4": (
            "stadium",
            {"lane_width_m": 1.0, "straight_length_m": 2.0, "island_radius_m": 0.4},
        ),
        "st_w1.0_s2_r0.8": (
            "stadium",
            {"lane_width_m": 1.0, "straight_length_m": 2.0, "island_radius_m": 0.8},
        ),
        "st_w1.0_s1_r0.0": (
            "stadium",
            {"lane_width_m": 1.0, "straight_length_m": 1.0, "island_radius_m": 0.0},
        ),
        "st_w1.0_s4_r0.0": (
            "stadium",
            {"lane_width_m": 1.0, "straight_length_m": 4.0, "island_radius_m": 0.0},
        ),
        "st_w1.2_s2_r0.0": (
            "stadium",
            {"lane_width_m": 1.2, "straight_length_m": 2.0, "island_radius_m": 0.0},
        ),
        "st_w1.2_s2_r0.4": (
            "stadium",
            {"lane_width_m": 1.2, "straight_length_m": 2.0, "island_radius_m": 0.4},
        ),
        "st_w0.8_s1_r0.2": (
            "stadium",
            {"lane_width_m": 0.8, "straight_length_m": 1.0, "island_radius_m": 0.2},
        ),
        "rr_w1.0_2x1.5_r0.0": (
            "rounded_rect",
            {
                "lane_width_m": 1.0,
                "straight_length_m": 2.0,
                "island_width_m": 1.5,
                "island_radius_m": 0.0,
            },
        ),
        "rr_w1.0_3x2_r0.4": (
            "rounded_rect",
            {
                "lane_width_m": 1.0,
                "straight_length_m": 3.0,
                "island_width_m": 2.0,
                "island_radius_m": 0.4,
            },
        ),
        "rr_w1.2_2x1.6_r0.8": (
            "rounded_rect",
            {
                "lane_width_m": 1.2,
                "straight_length_m": 2.0,
                "island_width_m": 1.6,
                "island_radius_m": 0.8,
            },
        ),
    },
    "oval": {"oval_2026-11-10": ("map", {"map_yaml": OVAL_MAP})},
    "lanes": {
        "lane_square": ("lane_square", {}),
        "lane_round": ("lane_round", {}),
    },
}
TRACK_SETS["all"] = {**TRACK_SETS["family"], **TRACK_SETS["oval"]}

DEFAULT_VARIANTS = {
    "gap_follow": {"driver": "gap_follow"},
    "tracker": {"driver": "tracker"},
    "tracker_loc": {"driver": "tracker_loc"},
}


def plan(suite: str, variants: dict[str, dict], directions: list[str]) -> list[dict]:
    runs = []
    for track_name, (track, params) in TRACK_SETS[suite].items():
        for direction in directions:
            for variant, spec in variants.items():
                runs.append(
                    {
                        "id": f"{track_name}__{direction}__{variant}",
                        "track_name": track_name,
                        "track": track,
                        "track_params": params,
                        "direction": direction,
                        "variant": variant,
                        "spec": spec,
                    }
                )
    return runs


def command(run: dict, domain_id: int, out: Path, extra: list[str]) -> list[str]:
    spec = dict(run["spec"])
    cmd = [
        sys.executable,
        str(RUNNER),
        "--track",
        run["track"],
        "--track-params-json",
        json.dumps(run["track_params"]),
        "--driver",
        spec.pop("driver", "gap_follow"),
        "--domain-id",
        str(domain_id),
        "--work-dir",
        str(out / "work" / run["id"]),
        "--out",
        str(out / f"{run['id']}.json"),
    ]
    if run["direction"] == "cw":
        cmd.append("--reverse")
    flag_for = {
        "profile": "--profile",
        "params": "--params-json",
        "tracker": "--tracker-params-json",
        "raceline": "--raceline-json",
        "car": "--car",
        "duration": "--duration",
        "laps": "--stop-after-laps",
        "stall_s": "--stall-s",
        "lidar": "--lidar",
    }
    for key, value in spec.items():
        flag = flag_for[key]
        cmd += [flag, value if isinstance(value, str) else json.dumps(value)]
    return cmd + extra


def run_one(run: dict, domain_id: int, out: Path, extra: list[str]) -> dict:
    path = out / f"{run['id']}.json"
    if path.exists():
        return json.loads(path.read_text())
    (out / "work" / run["id"]).mkdir(parents=True, exist_ok=True)
    start = time.time()
    proc = subprocess.run(
        command(run, domain_id, out, extra),
        capture_output=True,
        text=True,
        check=False,
        timeout=900,
    )
    if not path.exists():
        path.write_text(
            json.dumps({"error": f"runner exited {proc.returncode}: {proc.stderr[-800:]}"}) + "\n"
        )
    result = json.loads(path.read_text())
    result["wall_s"] = round(time.time() - start, 1)
    path.write_text(json.dumps(result, indent=2, sort_keys=True) + "\n")
    return result


def cell(result: dict) -> str:
    if "error" in result:
        return "ERR"
    laps = result.get("laps")
    t = result.get("lap_time_s")
    return (
        f"{laps} laps"
        + (f" {t:.1f}s" if t else "")
        + f", {result['obstacle_brakes']}b {result['escapes']}e {result['wall_contacts']}c"
        + f", clr {result['min_body_clearance_m']:.3f}"
    )


def report(out: Path) -> str:
    """Markdown tables: rows = track + direction, columns = variant."""
    rows: dict[tuple[str, str], dict[str, dict]] = defaultdict(dict)
    variants: list[str] = []
    for path in sorted(out.glob("*__*__*.json")):
        track_name, direction, variant = path.stem.split("__")
        if variant not in variants:
            variants.append(variant)
        rows[(track_name, direction)][variant] = json.loads(path.read_text())
    lines = [
        "| Track | Dir | " + " | ".join(variants) + " |",
        "|---|---|" + "---|" * len(variants),
    ]
    for (track_name, direction), cells in rows.items():
        lines.append(
            f"| {track_name} | {direction} | "
            + " | ".join(cell(cells[v]) if v in cells else "-" for v in variants)
            + " |"
        )
    lines.append("")
    lines.append(
        "laps (mean lap time), b = safety brakes, e = reverse escapes, c = wall contacts, "
        "clr = minimum body clearance (m)"
    )
    return "\n".join(lines)


def main(argv: list[str] | None = None) -> int:
    parser = argparse.ArgumentParser(
        description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter
    )
    sub = parser.add_subparsers(dest="cmd", required=True)
    p_run = sub.add_parser("run")
    p_run.add_argument("--suite", default="family", choices=sorted(TRACK_SETS))
    p_run.add_argument("--out", type=Path, required=True)
    p_run.add_argument("--jobs", type=int, default=4)
    p_run.add_argument("--domain-base", type=int, default=100)
    p_run.add_argument("--directions", default="ccw,cw")
    p_run.add_argument("--only", default="", help="substring filter on run ids")
    p_run.add_argument(
        "--variant",
        action="append",
        default=[],
        help="name=JSON spec (driver, profile, params, tracker, raceline, car, ...)",
    )
    p_run.add_argument("--extra", default="", help="extra runner arguments, space separated")
    p_list = sub.add_parser("list")
    p_list.add_argument("--suite", default="family", choices=sorted(TRACK_SETS))
    p_rep = sub.add_parser("report")
    p_rep.add_argument("out", type=Path)
    args = parser.parse_args(argv)
    if args.cmd == "report":
        print(report(args.out))
        return 0
    if args.cmd == "list":
        for run in plan(args.suite, DEFAULT_VARIANTS, ["ccw", "cw"]):
            print(run["id"])
        return 0
    variants = {}
    for text in args.variant:
        name, _, spec = text.partition("=")
        variants[name] = json.loads(spec)
    variants = variants or DEFAULT_VARIANTS
    runs = [
        r for r in plan(args.suite, variants, args.directions.split(",")) if args.only in r["id"]
    ]
    args.out.mkdir(parents=True, exist_ok=True)
    extra = args.extra.split()
    domains: queue.Queue[int] = queue.Queue()
    for i in range(args.jobs):
        domains.put(args.domain_base + i + 1)

    def worker(run: dict) -> dict:
        domain_id = domains.get()  # one runner per ROS domain at a time
        try:
            return run_one(run, domain_id, args.out, extra)
        finally:
            domains.put(domain_id)

    with ThreadPoolExecutor(max_workers=args.jobs) as pool:
        futures = {pool.submit(worker, run): run for run in runs}
        for future in as_completed(futures):
            run = futures[future]
            print(f"{run['id']}: {cell(future.result())}", flush=True)
    print(report(args.out))
    return 0


if __name__ == "__main__":
    sys.exit(main())
