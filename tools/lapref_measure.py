#!/usr/bin/env python3
"""M-LR and M-L1 of pre-registration PnP v0.4 sections 4.2 and 4.3 (brief 36). Real-data glue.

    uv run --project tools --extra session python tools/lapref_measure.py m-lr
    uv run --project tools --extra session python tools/lapref_measure.py m-l1 --frozen-sha <sha>

`m-lr` (development): lapref against the pose detector of racer_laps.replay on the 2026-11-13
map-pose segments, lapref on L0 (must find 20 laps), then lapref on the other lap `dev` runs.
`m-l1` (scored once): lapref and lap gate v2 (racer_laps.replay, not changed) on the five lap
`test` runs, whole runs, with a 1.0 s tail for the gate if the bag has it. It refuses to run unless
`--frozen-sha` is the current git HEAD and the tools and racer_laps are clean, and it refuses to
run a second time (a rerun forced by a bug needs `--rerun-reason`, and the first result stays on
disk next to it).

The pure parts (pairing, verdict, tables) are tools/lapref/compare.py and are unit tested; this
file only reads bags and calls racer_laps.replay as it is.
"""

from __future__ import annotations

import argparse
import json
import subprocess
import sys
import time
from pathlib import Path
from typing import Any

REPO = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(Path(__file__).resolve().parent))
sys.path.insert(0, str(REPO / "ros_ws" / "src" / "racer_laps" / "src"))

from lapref import compare
from lapref.bag import Signals, read_signals
from lapref.cli import laps_of_run
from racer_eval.split_v04 import load_v04

BAGS = Path("/Users/cameronjim/code/car-data/bags")
CACHE = Path("/Users/cameronjim/code/car-data/pnp/cache")
RESULTS = Path("/Users/cameronjim/code/car-data/pnp/results/lapref")
V02_SPLIT = REPO / "evaluation" / "prereg" / "pnp-v0.2-split.yaml"
V04_YAML = REPO / "evaluation" / "prereg" / "pnp-v0.4.yaml"
#: Brief 36, M-L1: the gate replay gets this tail after the run's end if the bag has it.
GATE_TAIL_S = 1.0
L0_LAPS = 20


def git(*args: str) -> str:
    return subprocess.run(
        ["git", *args], cwd=REPO, capture_output=True, text=True, check=True
    ).stdout.strip()


def cached_signals(bag: Path) -> Signals:
    path = CACHE / f"{bag.name}.lapref-signals.npz"
    if path.is_file():
        return Signals.load(path)
    sig = read_signals(bag)
    CACHE.mkdir(parents=True, exist_ok=True)
    sig.save(path)
    return sig


def read_windows(
    bag: Path, windows: dict[str, tuple[float, float]], topics: set[str]
) -> tuple[dict[str, list[tuple[float, str, Any]]], float, float]:
    """One pass over a bag with racer_laps.replay's reader: the messages of `topics` inside each
    window (bag seconds), the log time of the first /scan and the bag time of the last message."""
    from racer_laps import replay

    saved = replay.WANTED
    replay.WANTED = tuple(sorted(topics | {replay.SCAN_TOPIC}))
    out: dict[str, list[tuple[float, str, Any]]] = {name: [] for name in windows}
    first_scan: float | None = None
    last = 0.0
    try:
        for t, topic, msg in replay.read_messages(bag):
            if topic == replay.SCAN_TOPIC and first_scan is None:
                first_scan = t
            if first_scan is None:
                continue
            bag_t = t - first_scan
            last = max(last, bag_t)
            for name, (lo, hi) in windows.items():
                if lo <= bag_t <= hi and (topic in topics):
                    out[name].append((t, topic, msg))
    finally:
        replay.WANTED = saved
    if first_scan is None:
        raise ValueError(f"{bag}: no /scan")
    return out, first_scan, last


def pose_laps(bag: Path, windows: dict[str, tuple[float, float]]) -> dict[str, list[float]]:
    """Lap times of the map pose detector on each window, started at the instant the start-scan
    gate would start (as in racer_laps.replay.run_m_r), no scans decoded."""
    from racer_laps import replay

    topics = {
        *replay.TF_TOPICS,
        replay.PLANNER_TOPIC,
        replay.EVENTS_TOPIC,
        replay.ODOM_TOPIC,
        replay.DRIVE_RAW_TOPIC,
    }
    msgs, _first, _last = read_windows(bag, windows, topics)
    out: dict[str, list[float]] = {}
    for name, sub in msgs.items():
        odom = [(t, float(m.twist.twist.linear.x)) for t, topic, m in sub if topic == "/odom/wheel"]
        start = replay.find_start_time(odom)
        out[name] = [lap.lap_time_s for lap in replay.run_replay(sub, start_s=start).laps]
    return out


def m_lr(args: argparse.Namespace) -> int:
    import yaml
    from racer_laps import replay

    timing = compare.load_lap_timing(V04_YAML)
    v02 = yaml.safe_load(V02_SPLIT.read_text())
    where: dict[str, str] = {}
    for bag, entry in v02["bags"].items():
        for seg in entry.get("segments", []):
            where[str(seg["name"])] = bag
    where["T2a+T2b"] = replay.M_R_JOINED["T2a+T2b"]
    rows: list[compare.RunComparison] = []
    detail: dict[str, Any] = {}
    for name in replay.M_R_POSE_SEGMENTS:
        bag = where[name]
        lo, hi = replay.segment_ranges(V02_SPLIT, bag, name)
        pose = pose_laps(args.bags_root / bag, {name: (lo, hi)})[name]
        sig = cached_signals(args.bags_root / bag)
        result = laps_of_run(sig, lo, hi)
        rows.append(
            compare.compare_run(
                name, "pose", pose, "lapref", result.lap_times_s, timing.tolerance_s
            )
        )
        detail[name] = {"bag": bag, "window": [lo, hi], "pose": pose, "lapref": result.to_dict()}
        print(f"{name}: pose {len(pose)} laps, lapref {result.n_laps} laps", flush=True)
    out = [compare.format_m_lr(rows), ""]
    split = load_v04(V04_YAML)
    dev_rows = []
    for run in split.lap_runs("dev"):
        sig = cached_signals(args.bags_root / run.bag)
        result = laps_of_run(sig, run.t_from, run.t_to)
        detail[f"dev:{run.name}"] = {"window": [run.t_from, run.t_to], "lapref": result.to_dict()}
        dev_rows.append((run, result))
    out.append("| dev run | direction | cap | laps | mean lap (s) | peak | L (m) | lap times (s) |")
    out.append("|---|---|---|---|---|---|---|---|")
    for run, r in dev_rows:
        mean = f"{sum(r.lap_times_s) / r.n_laps:.2f}" if r.n_laps else "-"
        peak = "-" if r.autocorr_peak is None else f"{r.autocorr_peak:.3f}"
        length = "-" if r.lap_length_m is None else f"{r.lap_length_m:.3f}"
        times = " ".join(f"{t:.2f}" for t in r.lap_times_s) or r.reason
        out.append(
            f"| {run.name} | {run.direction} | {run.cap_mps:g} | {r.n_laps} | {mean} | {peak} | "
            f"{length} | {times} |"
        )
    l0 = next(r for run, r in dev_rows if run.name == "L0")
    out.append("")
    out.append(
        f"L0 must find {L0_LAPS} laps (owner observed): lapref found {l0.n_laps}: "
        f"{'OK' if l0.n_laps == L0_LAPS else 'NOT MET'}"
    )
    text = "\n".join(out)
    print(text)
    if args.out:
        args.out.parent.mkdir(parents=True, exist_ok=True)
        args.out.write_text(json.dumps({"detail": detail, "table": text}, indent=2))
    return 0 if l0.n_laps == L0_LAPS else 1


def check_frozen(frozen_sha: str) -> None:
    head = git("rev-parse", "HEAD")
    if head != frozen_sha:
        raise SystemExit(f"--frozen-sha {frozen_sha} is not HEAD ({head}): not scoring")
    dirty = git("status", "--porcelain", "--", "tools", "ros_ws/src/racer_laps")
    if dirty:
        raise SystemExit(f"the tools or racer_laps have uncommitted changes, not scoring:\n{dirty}")


def m_l1(args: argparse.Namespace) -> int:
    from racer_laps import replay
    from racer_laps.vehicle import load_live_vehicle

    check_frozen(args.frozen_sha)
    result_path: Path = args.out
    if result_path.exists():
        if not args.rerun_reason:
            raise SystemExit(
                f"{result_path} exists: the test runs are scored once. A rerun forced by a bug "
                "needs --rerun-reason, and its result goes next to the first one."
            )
        result_path = result_path.with_name(result_path.stem + f".rerun-{int(time.time())}.json")
    timing = compare.load_lap_timing(V04_YAML)
    split = load_v04(V04_YAML)
    vehicle = load_live_vehicle()
    runs = split.lap_runs("test")
    by_bag: dict[str, list[Any]] = {}
    for run in runs:
        by_bag.setdefault(run.bag, []).append(run)
    record: dict[str, Any] = {
        "frozen_sha": args.frozen_sha,
        "gate_tree": git("rev-parse", "HEAD:ros_ws/src/racer_laps"),
        "gate_last_commit": git("log", "-1", "--format=%H %s", "--", "ros_ws/src/racer_laps"),
        "rerun_reason": args.rerun_reason,
        "tail_s": GATE_TAIL_S,
        "runs": {},
    }
    comparisons: list[compare.RunComparison] = []
    for bag, bag_runs in by_bag.items():
        windows = {r.name: (r.t_from, r.t_to + GATE_TAIL_S) for r in bag_runs}
        msgs, first_scan, last_bag_t = read_windows(
            args.bags_root / bag,
            windows,
            {
                replay.ODOM_TOPIC,
                replay.DRIVE_TOPIC,
                replay.DRIVE_RAW_TOPIC,
                replay.PLANNER_TOPIC,
                replay.EVENTS_TOPIC,
                replay.SCAN_TOPIC,
            },
        )
        sig = cached_signals(args.bags_root / bag)
        for run in bag_runs:
            tail_ok = last_bag_t >= run.t_to + GATE_TAIL_S
            sub = msgs[run.name]
            if not tail_ok:
                sub = [m for m in sub if m[0] - first_scan <= run.t_to]
            gate = replay.run_replay_start_scan(sub, vehicle)
            gate_times = [lap.lap_time_s for lap in gate.laps]
            tail_laps = [
                k + 1 for k, lap in enumerate(gate.laps) if lap.t_cross_s - first_scan > run.t_to
            ]
            ref = laps_of_run(sig, run.t_from, run.t_to)
            comparisons.append(
                compare.compare_run(
                    run.name, "lapref", ref.lap_times_s, "gate", gate_times, timing.tolerance_s
                )
            )
            record["runs"][run.name] = {
                "bag": bag,
                "window": [run.t_from, run.t_to],
                "tail_available": tail_ok,
                "gate_laps_closed_in_the_tail": tail_laps,
                "gate": gate_times,
                "lapref": ref.to_dict(),
            }
            result_path.parent.mkdir(parents=True, exist_ok=True)
            result_path.write_text(json.dumps(record, indent=2))  # after every run
            print(
                f"{run.name}: lapref {ref.n_laps} laps, gate {len(gate_times)} laps "
                f"(tail laps {tail_laps}, tail available {tail_ok})",
                flush=True,
            )
    text = compare.format_m_l1(comparisons, timing)
    record["table"] = text
    result_path.write_text(json.dumps(record, indent=2))
    print(text)
    return 0


def main(argv: list[str] | None = None) -> int:
    parser = argparse.ArgumentParser(description=__doc__.split("\n\n")[0])
    sub = parser.add_subparsers(dest="cmd", required=True)
    p = sub.add_parser("m-lr")
    p.add_argument("--bags-root", type=Path, default=BAGS)
    p.add_argument("--out", type=Path, default=RESULTS / "m-lr.json")
    p.set_defaults(fn=m_lr)
    p = sub.add_parser("m-l1")
    p.add_argument("--bags-root", type=Path, default=BAGS)
    p.add_argument("--frozen-sha", required=True)
    p.add_argument("--out", type=Path, default=RESULTS / "m-l1.json")
    p.add_argument("--rerun-reason", default=None)
    p.set_defaults(fn=m_l1)
    args = parser.parse_args(argv)
    return int(args.fn(args))


if __name__ == "__main__":
    sys.exit(main())
