#!/usr/bin/env python3
"""Run the lap learning sim battery of the pre-registration (section 5.1) and report D-S0 to D-S2.

    python3 tests/l5_pnp_laps/battery.py run --out DIR --conditions abc --workers 4
    python3 tests/l5_pnp_laps/battery.py report --out DIR

Needs the ROS workspace sourced (see .github/scripts/l5_pnp_laps_test.sh) and f1tenth_gym.

Cases (track_family_runner.py, driver pnp, stepped in real time; no node here honours sim time):
  S1  stadium, lane 1.3 m, straight 2.0 m, island radius 0.4 m
  S2  rounded rectangle, lane 1.0 m, straight 3.0 m, island 2.0 m wide, radius 0.4 m
  S3  stadium, lane 0.9 m, straight 3.0 m, island radius 0.5 m
  S4  the saved real map (`--map-dir`) as the world (`--track map`)
  T   the tuning stadium (lane 1.0 m, straight 2.0 m, island radius 0.4 m): the ONLY track D may
      be tuned on; it is never part of the battery
each ccw and cw, seeds 11, 12 and 13, 12 laps from a standing start, lap-1 cap 0.5 m/s, car-like
plant (the fitted bundle's actuation, dead zone, steering lag and command delay), scan noise on.
Conditions: (a) D off at the lap-1 cap; (b) D off at the ceiling; (c) D v0; (d) D v1. The ceiling is
the planner bundle's validity (`--bundle`). The lap position inside the system is the start-scan
gate in every run: ground truth is used only to score.

The battery is run ONCE with frozen parameters. Results go to `<out>/<condition>_<case>_<dir>_
<seed>.json`; an existing file is skipped, so an interrupted run resumes (a bug fix rerun uses a new
directory and the PR reports both).
"""

from __future__ import annotations

import argparse
import json
import os
import queue
import subprocess
import sys
import threading
import time
from pathlib import Path

HERE = Path(__file__).resolve().parent
sys.path.insert(0, str(HERE))
import lap_battery as lb

REPO_ROOT = HERE.parents[1]
RUNNER = REPO_ROOT / "tests" / "l5_reactive_bench" / "track_family_runner.py"
DEFAULT_BUNDLE = REPO_ROOT / "config" / "pnp" / "fixtures" / "dyn-car-2026-11-12"
LAP1_CAP_MPS = 0.5
LAPS = 12
SEEDS = (11, 12, 13)
DIRECTIONS = ("ccw", "cw")

CASES: dict[str, tuple[str, dict]] = {
    "S1": ("stadium", {"lane_width_m": 1.3, "straight_length_m": 2.0, "island_radius_m": 0.4}),
    "S2": (
        "rounded_rect",
        {
            "lane_width_m": 1.0,
            "straight_length_m": 3.0,
            "island_width_m": 2.0,
            "island_radius_m": 0.4,
        },
    ),
    "S3": ("stadium", {"lane_width_m": 0.9, "straight_length_m": 3.0, "island_radius_m": 0.5}),
    "T": ("stadium", {"lane_width_m": 1.0, "straight_length_m": 2.0, "island_radius_m": 0.4}),
}
BATTERY_CASES = ("S1", "S2", "S3", "S4")


def manifest(bundle: Path) -> dict:
    import yaml

    return yaml.safe_load((bundle / "manifest.yaml").read_text())["io"]


def condition_flags(condition: str, ceiling: float) -> list[str]:
    """Runner flags of a condition: lap memory mode and the speeds."""
    if condition == "a":  # D off, cap fixed at the lap-1 cap
        return ["--lap-memory", "off", "--v-max-mps", str(LAP1_CAP_MPS)]
    if condition == "b":  # D off, cap fixed at the ceiling
        return ["--lap-memory", "off", "--v-max-mps", str(ceiling)]
    if condition in ("c", "d"):
        mode = "v0" if condition == "c" else "v1"
        return [
            "--lap-memory",
            mode,
            "--v-max-mps",
            str(ceiling),
            "--v-lap1-mps",
            str(LAP1_CAP_MPS),
        ]
    raise ValueError(f"unknown condition {condition!r}")


def command_for(
    condition: str,
    case: str,
    direction: str,
    seed: int,
    domain: int,
    work: Path,
    out: Path,
    bundle: Path,
    map_dir: Path | None,
    lap_params: str,
    laps: int = LAPS,
) -> list[str]:
    io = manifest(bundle)
    ceiling = float(io["validity"]["v_max_mps"])
    nominal = io["nominal"]
    if case == "S4":
        if map_dir is None:
            raise ValueError("case S4 needs --map-dir")
        track = "map"
        params = {
            "map_yaml": str(map_dir / f"{map_dir.name}.yaml"),
            "map_centreline_csv": str(map_dir / f"{map_dir.name}_centreline.csv"),
        }
    else:
        track, params = CASES[case]
    command = [
        sys.executable,
        str(RUNNER),
        "--track",
        track,
        "--track-params-json",
        json.dumps(params),
        "--driver",
        "pnp",
        "--backend",
        "geometric",
        "--dynamics-bundle",
        str(bundle),
        "--plant-bundle",
        str(bundle),
        "--plant-speed-deadband",
        str(nominal["speed_deadband_mps"]),
        "--plant-steer-tau",
        str(nominal["steer_tau_s"]),
        "--plant-command-delay",
        str(nominal["command_delay_s"]),
        *condition_flags(condition, ceiling),
        "--car",
        "params",
        "--seed",
        str(seed),
        "--stop-after-laps",
        str(laps),
        "--duration",
        "900",
        "--stall-s",
        "30",
        "--domain-id",
        str(domain),
        "--work-dir",
        str(work),
        "--out",
        str(out),
    ]
    if lap_params and lap_params != "{}":
        command += ["--lap-params-json", lap_params]
    if direction == "cw":
        command.append("--reverse")
    return command


def run_battery(args: argparse.Namespace) -> None:
    out_dir = Path(args.out)
    out_dir.mkdir(parents=True, exist_ok=True)
    jobs = []
    for condition in args.conditions:
        for case in args.cases:
            for direction in DIRECTIONS if not args.ccw_only else ("ccw",):
                for seed in args.seeds:
                    name = f"{condition}_{case}_{direction}_{seed}"
                    if (out_dir / f"{name}.json").exists():
                        continue
                    jobs.append((name, condition, case, direction, seed))
    print(f"{len(jobs)} runs, {args.workers} at a time -> {out_dir}", flush=True)
    domains: queue.Queue[int] = queue.Queue()
    for k in range(args.workers):
        domains.put(args.domain_base + k)
    pending: queue.Queue[tuple] = queue.Queue()
    for job in jobs:
        pending.put(job)
    lock = threading.Lock()

    def worker() -> None:
        while True:
            try:
                name, condition, case, direction, seed = pending.get_nowait()
            except queue.Empty:
                return
            domain = domains.get()
            try:
                work = out_dir / "work" / name
                work.mkdir(parents=True, exist_ok=True)
                result_path = work / "result.json"
                command = command_for(
                    condition,
                    case,
                    direction,
                    seed,
                    domain,
                    work,
                    result_path,
                    Path(args.bundle),
                    Path(args.map_dir) if args.map_dir else None,
                    args.lap_params_json,
                    args.laps,
                )
                started = time.monotonic()
                with (work / "runner.log").open("w") as log:
                    proc = subprocess.run(
                        command,
                        stdout=log,
                        stderr=subprocess.STDOUT,
                        env=dict(os.environ),
                        check=False,
                    )
                if result_path.exists():
                    (out_dir / f"{name}.json").write_text(result_path.read_text())
                with lock:
                    print(
                        f"{name}: exit {proc.returncode} in {time.monotonic() - started:.0f} s",
                        flush=True,
                    )
            finally:
                domains.put(domain)

    threads = [threading.Thread(target=worker) for _ in range(args.workers)]
    for t in threads:
        t.start()
    for t in threads:
        t.join()


def report(args: argparse.Namespace) -> None:
    runs = [r for r in lb.load_summaries(Path(args.out)) if r.case in BATTERY_CASES]
    by = {c: [r for r in runs if r.condition == c] for c in lb.CONDITIONS}
    print(lb.condition_table(runs))
    print()
    for condition, label in (
        ("a", "D off at 0.5"),
        ("b", "D off at the ceiling"),
        ("c", "D v0"),
        ("d", "D v1"),
    ):
        rs = by[condition]
        done = [r for r in rs if r.completed]
        print(
            f"condition ({condition}) {label}: {len(rs)} runs, {len(done)} complete 10 laps, "
            f"{sum(r.contacts for r in rs)} wall contacts"
        )
    print()
    print(lb.decide_d_s0(by["a"]))
    print(lb.decide_d_s1(by["c"]))
    print(lb.decide_d_s2(by["d"], by["c"]))


def main(argv: list[str] | None = None) -> int:
    parser = argparse.ArgumentParser(
        description=__doc__, formatter_class=argparse.RawTextHelpFormatter
    )
    sub = parser.add_subparsers(dest="command", required=True)
    run = sub.add_parser("run")
    run.add_argument("--out", required=True)
    run.add_argument("--conditions", default="abcd")
    run.add_argument("--cases", default=",".join(BATTERY_CASES))
    run.add_argument("--seeds", default=",".join(str(s) for s in SEEDS))
    run.add_argument("--workers", type=int, default=4)
    run.add_argument("--domain-base", type=int, default=120)
    run.add_argument("--bundle", default=str(DEFAULT_BUNDLE))
    run.add_argument("--map-dir", default="")
    run.add_argument("--laps", type=int, default=LAPS)
    run.add_argument("--ccw-only", action="store_true")
    run.add_argument("--lap-params-json", default="{}", help="tuning: lap_memory_node parameters")
    rep = sub.add_parser("report")
    rep.add_argument("--out", required=True)
    args = parser.parse_args(argv)
    if args.command == "run":
        args.conditions = list(args.conditions)
        args.cases = args.cases.split(",")
        args.seeds = [int(s) for s in args.seeds.split(",")]
        run_battery(args)
    else:
        report(args)
    return 0


if __name__ == "__main__":
    sys.exit(main())
