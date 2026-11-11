"""Replay mode: would the race profile have commanded anything crazy on the real bags?

Every bag with /scan messages is replayed through gap_follow_replay_cli (the code gap_follow_node
runs, with --speed so the speed half runs too) twice: the BASELINE (lane-2026-10-25 with the
recommended lookahead 0.8 and outside entry 0.25, the car's top speed 0.64 m/s) and the RACE
profile. The scans are the real car's, so this is the one place the race profile meets the real
LiDAR, real walls and real noise. What it CANNOT say is how the car would have driven: the bag's
scans came from where the lane profile took the car, not from where the race profile would have.
So this checks what the controller asks for on real scans, not what happens next.

HARD CHECKS (any failure makes `replay` exit 1), per bag, on the race profile:
  1. every steering and speed is finite and inside the limits (|steering| <= the steering limit,
     0 <= speed <= max_speed_mps);
  2. the published speed never rises faster than the acceleration limiter's bound, per interval;
  3. the stopping law holds on the scans: wherever physics_speed is on, the target speed is at
     most max(min_speed, the speed that stops inside the free path less the standoff);
  4. steering reversals per minute are at most REVERSAL_FACTOR times the baseline's plus
     REVERSAL_SLACK_PER_MIN (the race profile may not be twitchier than the baseline);
  5. no scan-to-scan jump of the filtered steering beyond MAX_STEERING_STEP_RAD (a jump that big
     is a different branch of the lane, not a steering change).
INFORMATION, not gated: speed distribution, the race speed minus the recorded speed in the second
before each recorded safety brake, how many scans were clamped, how far the race steering departs
from the baseline's.

Vehicle numbers (steering limit, stopping deceleration, latency, standoff, acceleration bound) come
from the CLI's --describe, i.e. from the vehicle_params binding, never from this file.
"""

from __future__ import annotations

import argparse
import json
import math
import tempfile
from bisect import bisect_left
from collections.abc import Sequence
from pathlib import Path
from typing import Any

import numpy as np
from follower_replay import bag, replay
from safety_replay.calibration import RECORDED_0_13_0

# Bags up to and including this date were driven with vehicle_params 0.13.0 (the 2026-11-10
# calibration landed the day the last of them was recorded): their recorded speeds are
# SPEED_RATIO times the true ones. The scans are real either way.
OLD_UNITS_THROUGH = "2026-11-11"

REVERSAL_FACTOR = 1.5
REVERSAL_SLACK_PER_MIN = 3.0
MAX_STEERING_STEP_RAD = 0.35
REVERSAL_SWING_RAD = 0.1
SPEED_TOLERANCE_MPS = 1e-6

# The recommended profile before the race work, as parameter overrides of lane-2026-10-25.
BASELINE_OVERRIDES: dict[str, replay.ParamValue] = {
    "lane_lookahead_m": 0.8,
    "outside_entry_offset_m": 0.25,
}


def reversals(values: Sequence[float], swing_rad: float = REVERSAL_SWING_RAD) -> int:
    """Zig-zag count of a signal with a hysteresis: the number of times it turns round after
    travelling at least `swing_rad` one way (the same idea as bench_metrics.count_reversals)."""
    count = 0
    direction = 0
    extreme: float | None = None
    for v in values:
        if extreme is None:
            extreme = v
        elif direction == 0:
            if abs(v - extreme) >= swing_rad:
                direction = 1 if v > extreme else -1
                extreme = v
        elif direction == 1:
            if v > extreme:
                extreme = v
            elif extreme - v >= swing_rad:
                count += 1
                direction = -1
                extreme = v
        else:
            if v < extreme:
                extreme = v
            elif v - extreme >= swing_rad:
                count += 1
                direction = 1
                extreme = v
    return count


def stopping_limit(free_path_m: float, standoff_m: float, decel: float, latency_s: float) -> float:
    """The speed that stops inside free_path - standoff (race_speed.hpp stopping_limit_speed,
    written out again so the check is independent of the C++ it checks)."""
    room = free_path_m - standoff_m
    if not room > 0.0:
        return 0.0
    return decel * (math.sqrt(latency_s * latency_s + 2.0 * room / decel) - latency_s)


def analyse(
    rows: Sequence[dict[str, Any]],
    stamps: Sequence[float],
    geometry: dict[str, float],
    params: dict[str, Any],
) -> dict[str, Any]:
    """One variant's numbers on one bag, from the CLI's rows (run with --speed)."""
    valid = [r for r in rows if r["valid"]]
    steer = [r["filtered_steering_rad"] for r in rows]
    speed = [r["speed_mps"] for r in rows]
    finite = all(math.isfinite(v) for v in steer + speed)
    max_speed = float(params.get("max_speed_mps", 1.43))
    min_speed = float(params.get("min_speed_mps", 0.36))
    steer_limit = geometry["max_steering_rad"]
    accel_bound = geometry["max_acceleration_mps2"] * float(
        params.get("speed_rate_limit_margin_fraction", 0.5)
    )
    in_bounds = (
        finite
        and all(abs(v) <= steer_limit + 1e-9 for v in steer)
        and all(-SPEED_TOLERANCE_MPS <= v <= max_speed + SPEED_TOLERANCE_MPS for v in speed)
    )
    dts = np.diff(np.asarray(stamps, dtype=float))
    dv = np.diff(np.asarray(speed, dtype=float))
    with np.errstate(divide="ignore", invalid="ignore"):
        accel = np.where(dts > 0, dv / dts, 0.0)
    max_accel = float(accel.max()) if accel.size else 0.0
    steps = np.abs(np.diff(np.asarray(steer, dtype=float))) if len(steer) > 1 else np.zeros(1)
    standoff = geometry["min_forward_clearance_m"] + float(params.get("stopping_margin_m", 0.05))
    stop_violations = 0
    stop_checked = 0
    if params.get("physics_speed"):
        for r in rows:
            target = r.get("target_speed_mps")
            free = r.get("free_path_m")
            if target is None or free is None:
                continue
            stop_checked += 1
            allowed = max(
                min_speed,
                stopping_limit(
                    free, standoff, geometry["stop_deceleration_mps2"], geometry["stop_latency_s"]
                ),
            )
            if target > allowed + 1e-6:
                stop_violations += 1
    minutes = max((stamps[-1] - stamps[0]) / 60.0, 1e-9) if len(stamps) > 1 else 1e-9
    return {
        "scans": len(rows),
        "valid_scans": len(valid),
        "finite": finite,
        "in_bounds": in_bounds,
        "speed_mean_mps": float(np.mean(speed)) if speed else math.nan,
        "speed_max_mps": float(max(speed)) if speed else math.nan,
        "speed_at_floor_fraction": (
            float(np.mean([v <= min_speed + 1e-6 for v in speed])) if speed else math.nan
        ),
        "max_accel_mps2": max_accel,
        "accel_bound_mps2": accel_bound,
        "max_steering_step_rad": float(steps.max()),
        "reversals_per_min": reversals(steer) / minutes,
        "clamped_scans": sum(bool(r["clamped"]) for r in rows),
        "stop_checked": stop_checked,
        "stop_violations": stop_violations,
    }


def hard_checks(race: dict[str, Any], baseline: dict[str, Any]) -> dict[str, bool]:
    return {
        "finite_and_within_limits": bool(race["finite"] and race["in_bounds"]),
        "acceleration_within_limiter": race["max_accel_mps2"]
        <= race["accel_bound_mps2"] * 1.02 + 1e-6,
        "stopping_law_holds": race["stop_violations"] == 0,
        "not_twitchier_than_baseline": race["reversals_per_min"]
        <= REVERSAL_FACTOR * baseline["reversals_per_min"] + REVERSAL_SLACK_PER_MIN,
        "no_steering_jumps": race["max_steering_step_rad"] <= MAX_STEERING_STEP_RAD,
    }


def pre_brake_speeds(
    rows: Sequence[dict[str, Any]],
    stamps: Sequence[float],
    events: Sequence[bag.EventRecord],
    recorded: Sequence[bag.DriveRecord],
    window_s: float = 1.0,
) -> dict[str, float] | None:
    """For each recorded safety BRAKE engagement, the mean replayed speed and the mean recorded
    /drive speed over the second before it; the medians over the events. None without events."""
    stamps_arr = list(stamps)
    rec_stamps = [r.stamp_s for r in recorded]
    replayed, rec = [], []
    for e in events:
        if e.severity != bag.SEVERITY_BRAKE or e.phase != bag.PHASE_ENGAGE:
            continue
        lo = bisect_left(stamps_arr, e.stamp_s - window_s)
        hi = bisect_left(stamps_arr, e.stamp_s)
        if hi <= lo:
            continue
        replayed.append(float(np.mean([rows[i]["speed_mps"] for i in range(lo, hi)])))
        rlo = bisect_left(rec_stamps, e.stamp_s - window_s)
        rhi = bisect_left(rec_stamps, e.stamp_s)
        if rhi > rlo:
            rec.append(float(np.mean([recorded[i].speed_mps for i in range(rlo, rhi)])))
    if not replayed:
        return None
    return {
        "events": float(len(replayed)),
        "replayed_median_mps": float(np.median(replayed)),
        "recorded_median_mps": float(np.median(rec)) if rec else math.nan,
    }


def steering_departure(
    race_rows: Sequence[dict[str, Any]], base_rows: Sequence[dict[str, Any]]
) -> dict[str, float]:
    """|race - baseline| of the filtered steering per scan: median, p99, max (rad)."""
    d = np.abs(
        np.asarray([r["filtered_steering_rad"] for r in race_rows], dtype=float)
        - np.asarray([r["filtered_steering_rad"] for r in base_rows], dtype=float)
    )
    if d.size == 0:
        return {"median_rad": math.nan, "p99_rad": math.nan, "max_rad": math.nan}
    return {
        "median_rad": float(np.median(d)),
        "p99_rad": float(np.percentile(d, 99)),
        "max_rad": float(d.max()),
    }


def bag_paths(root: Path) -> list[Path]:
    """The bags under `root`: the file itself, a directory that holds .mcap files directly (one
    bag), or, when it holds bag directories, those directories plus any loose .mcap files."""
    if root.is_file():
        return [root]
    bag_dirs = sorted(p for p in root.iterdir() if p.is_dir() and any(p.glob("*.mcap")))
    loose = sorted(root.glob("*.mcap"))
    if bag_dirs:
        return [*bag_dirs, *loose]
    return [root] if loose else []


def replay_bag(
    path: Path,
    cli: Path,
    race_params: dict[str, Any],
    base_params: dict[str, Any],
    work: Path,
) -> dict[str, Any] | None:
    data = bag.read_bag(path)
    if not data.scans:
        return None
    scans_file = work / "scans.txt"
    replay.write_scan_file(data.scans, scans_file)
    limits = replay.lidar_range_limits(data.scans)
    results: dict[str, Any] = {}
    rows_by_name: dict[str, list[dict[str, Any]]] = {}
    geometry: dict[str, float] = {}
    for name, params in (("baseline", base_params), ("race", race_params)):
        params_file = work / f"{name}.json"
        replay.write_params_file({**limits, **params}, params_file)
        rows = replay.run_cli(cli, scans_file, params_file, extra=["--speed"])
        geometry = replay.describe_cli(cli, params_file)
        rows_by_name[name] = rows
        results[name] = analyse(rows, [s.stamp_s for s in data.scans], geometry, params)
    stamps = [s.stamp_s for s in data.scans]
    scale = RECORDED_0_13_0.speed_ratio if path.name[:10] <= OLD_UNITS_THROUGH else 1.0
    recorded = [
        bag.DriveRecord(r.stamp_s, r.steering_rad, r.speed_mps / scale, r.log_time_s)
        for r in data.drive
    ]
    checks = hard_checks(results["race"], results["baseline"])
    return {
        "bag": path.name,
        "duration_s": stamps[-1] - stamps[0] if len(stamps) > 1 else 0.0,
        "truncated": data.truncated,
        "baseline": results["baseline"],
        "race": results["race"],
        "steering_departure": steering_departure(rows_by_name["race"], rows_by_name["baseline"]),
        "pre_brake": {
            name: pre_brake_speeds(rows_by_name[name], stamps, data.events, recorded)
            for name in ("baseline", "race")
        },
        "checks": checks,
        "passed": all(checks.values()),
    }


def format_report(results: Sequence[dict[str, Any]]) -> str:
    lines = [
        (
            "| Bag | Duration (s) | Scans | Race speed mean / max (m/s) | Baseline speed mean / max "
            "| Race reversals/min | Baseline reversals/min | Race max steering step (rad) "
            "| Steering departure p99 (rad) | Stopping-law violations | Hard checks |"
        ),
        "|---|---|---|---|---|---|---|---|---|---|---|",
    ]
    for r in results:
        race, base = r["race"], r["baseline"]
        lines.append(
            f"| {r['bag']} | {r['duration_s']:.0f} | {race['scans']} | "
            f"{race['speed_mean_mps']:.2f} / {race['speed_max_mps']:.2f} | "
            f"{base['speed_mean_mps']:.2f} / {base['speed_max_mps']:.2f} | "
            f"{race['reversals_per_min']:.1f} | {base['reversals_per_min']:.1f} | "
            f"{race['max_steering_step_rad']:.3f} | {r['steering_departure']['p99_rad']:.3f} | "
            f"{race['stop_violations']} of {race['stop_checked']} | "
            f"{'pass' if r['passed'] else 'FAIL: ' + ', '.join(k for k, v in r['checks'].items() if not v)} |"
        )
    return "\n".join(lines)


def main(args: argparse.Namespace) -> int:
    cli = replay.find_cli(args.cli)
    overrides = replay.parse_overrides(args.param)
    race_params = replay.resolve_params(args.profile, overrides)
    base_params = replay.resolve_params(args.baseline_profile, BASELINE_OVERRIDES)
    results = []
    for path in bag_paths(Path(args.bags)):
        with tempfile.TemporaryDirectory() as tmp:
            try:
                result = replay_bag(path, cli, race_params, base_params, Path(tmp))
            except (replay.ReplayError, FileNotFoundError) as error:
                print(f"{path.name}: skipped ({error})")
                continue
        if result is None:
            print(f"{path.name}: no /scan messages, skipped")
            continue
        results.append(result)
    text = format_report(results)
    print(text)
    if args.out:
        args.out.mkdir(parents=True, exist_ok=True)
        (args.out / "replay_check.json").write_text(json.dumps(results, indent=2) + "\n")
        (args.out / "replay_check.md").write_text(text + "\n")
    failed = [r["bag"] for r in results if not r["passed"]]
    if failed:
        print(f"HARD CHECKS FAILED on: {', '.join(failed)}")
        return 1
    return 0
