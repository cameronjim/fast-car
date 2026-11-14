"""Record the laps the start-scan gate closes on every driven segment of the pre-registered split.

    PYTHONPATH=<checkout>/ros_ws/src/racer_laps/src RACER_REPO_ROOT=<checkout> \
        python record_gate_laps.py --split evaluation/prereg/pnp-v0.2-split.yaml \
        --bags-root /Users/cameronjim/code/car-data/bags --out gate_v1_laps_2026-11-13.json

Brief 29 keeps gate v1 out of the code. Its laps were recorded once by running this script from a
checkout of commit f45fd09 (the last commit with gate v1); gate v2 is then compared with the
committed JSON (`racer_laps.replay --m-r`). It only uses the replay API that both gates share.
Segments: every PnP, gap follow and tracker segment of the split, plus `T2a+T2b` joined.
"""

from __future__ import annotations

import argparse
import json
import subprocess
import sys
from pathlib import Path

import yaml
from racer_laps.replay import read_messages, run_replay_start_scan, segment_ranges
from racer_laps.vehicle import load_live_vehicle

DRIVEN = ("pnp", "gap_follow", "gap_follow_race", "tracker")
JOINED = {"T2a+T2b": ("2026-11-13T23-27-23_car_teleop", "T2a+T2b")}


def driven_segments(split: dict) -> list[tuple[str, str]]:
    """(bag name, segment name) of every driven segment, joined ones appended."""
    out = []
    for bag, entry in split["bags"].items():
        for seg in entry.get("segments", []):
            if seg.get("driver") in DRIVEN:
                out.append((bag, seg["name"]))
    out.extend(JOINED.values())
    return out


def record(split_path: Path, bags_root: Path, commit: str) -> dict:
    split = yaml.safe_load(split_path.read_text(encoding="utf-8"))
    vehicle = load_live_vehicle()
    segments: dict[str, dict] = {}
    by_bag: dict[str, list[str]] = {}
    for bag, name in driven_segments(split):
        by_bag.setdefault(bag, []).append(name)
    for bag, names in by_bag.items():
        messages = list(read_messages(bags_root / bag))
        for name in names:
            window = segment_ranges(split_path, bag, name)
            try:
                result = run_replay_start_scan(messages, vehicle, window_s=window)
            except ValueError as error:  # no /scan in the window
                segments[name] = {"bag": bag, "error": str(error), "laps": 0, "lap_times_s": []}
                continue
            segments[name] = {
                "bag": bag,
                "window_bag_s": list(window),
                "laps": len(result.laps),
                "lap_times_s": [round(lap.lap_time_s, 3) for lap in result.laps],
                "crossings_bag_s": [
                    round(result.bag_time(lap.t_cross_s), 3) for lap in result.laps
                ],
                "lengths_m": [round(lap.length_m, 3) for lap in result.laps],
                "missed": [bool(lap.missed_gate) for lap in result.laps],
                "wheel_distance_m": round(result.wheel_distance_m, 2),
            }
            print(name, segments[name]["laps"], segments[name]["lap_times_s"], flush=True)
        del messages
    return {
        "gate": "start-scan gate",
        "commit": commit,
        "split": str(split_path),
        "segments": segments,
    }


def main(argv: list[str] | None = None) -> int:
    parser = argparse.ArgumentParser(description=__doc__.split("\n\n")[0])
    parser.add_argument("--split", type=Path, required=True)
    parser.add_argument("--bags-root", type=Path, required=True)
    parser.add_argument("--out", type=Path, required=True)
    parser.add_argument("--commit", default="")
    args = parser.parse_args(argv)
    commit = args.commit
    if not commit:
        try:
            commit = subprocess.run(
                ["git", "rev-parse", "--short", "HEAD"], capture_output=True, text=True, check=True
            ).stdout.strip()
        except (OSError, subprocess.CalledProcessError):
            commit = "unknown"
    data = record(args.split, args.bags_root, commit)
    args.out.write_text(json.dumps(data, indent=1) + "\n", encoding="utf-8")
    return 0


if __name__ == "__main__":
    sys.exit(main())
