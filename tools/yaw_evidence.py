"""How far the wheel odometry's yaw rate and speed are from where the car really went.

    uv run --project tools python tools/yaw_evidence.py NPZ MAP.yaml [--old-bag] [--window 10]
        [--stride 1] [--limit N] [--json out.json]

NPZ is tools/track_session/extract_bag.py's output, MAP.yaml a finished slam_toolbox map of the
SAME run in the frame of its first scan (a map from tools/track_session/replay_sync.py or
map_from_bag.sh). Every scan is registered against the map (mapping_eval/register.py), which
gives the car's true heading and position without frame-to-frame drift. Over windows of
`--window` scans the wheel odometry's heading change, and its path length, are compared with
the registered ones, in bins of the mean steering angle:

  yaw ratio   sum |registered heading change| / sum |modelled heading change| (the slope
              through 0 is also kept, as yaw_ratio_slope in the json: it reads low when the
              modelled turn is a scan or two out of step with the registered one)
  speed ratio registered path length / path length of the (calibrated) wheel speed
  steer ratio yaw ratio / speed ratio: curvature per modelled curvature

`--old-bag` puts a 2026-11-10 bag on the calibrated scale (speed / 1.401, steering x 1.203 left,
x 1.072 right), see mapping_eval/odometry.py. The map must be good enough to register against:
a doubled map biases the answer.
"""

from __future__ import annotations

import argparse
import json
import math
import sys
from itertools import pairwise
from pathlib import Path

import numpy as np
from mapping_eval.odometry import (
    OLD_BAGS,
    BagScale,
    YawModel,
    integrate_pose,
    load_vehicle,
    wheel_twist_series,
)
from mapping_eval.quality import load_map_strict
from mapping_eval.register import scan_points_base, track_scans

BINS = (0.0, 0.05, 0.15, 0.25, 0.35, 0.60)
SPEED_BINS = (0.0, 0.3, 0.45, 0.6, 0.8, 1.2, 3.0)


def window_stats(
    reg_yaw: np.ndarray,
    reg_xy: np.ndarray,
    model_yaw: np.ndarray,
    model_path: np.ndarray,
    delta: np.ndarray,
    speed: np.ndarray,
    moving: np.ndarray,
    window: int,
) -> dict[str, np.ndarray]:
    """Per-window (start i, end i + window) registered and modelled heading change and path length,
    mean |steering|, keeping only windows where the car moves forward throughout."""
    n = len(reg_yaw)
    i0 = np.arange(0, n - window)
    i1 = i0 + window
    ok = np.array([moving[a : b + 1].all() for a, b in zip(i0, i1, strict=True)])
    seg = np.hypot(np.diff(reg_xy[:, 0]), np.diff(reg_xy[:, 1]))
    cum = np.concatenate([[0.0], np.cumsum(seg)])
    cum_delta = np.concatenate([[0.0], np.cumsum(np.abs(delta)[1:])])
    cum_signed = np.concatenate([[0.0], np.cumsum(delta[1:])])
    cum_speed = np.concatenate([[0.0], np.cumsum(speed[1:])])
    return {
        "reg_dyaw": (reg_yaw[i1] - reg_yaw[i0])[ok],
        "model_dyaw": (model_yaw[i1] - model_yaw[i0])[ok],
        "reg_path": (cum[i1] - cum[i0])[ok],
        "model_path": (model_path[i1] - model_path[i0])[ok],
        "mean_abs_delta": ((cum_delta[i1] - cum_delta[i0]) / window)[ok],
        "mean_delta": ((cum_signed[i1] - cum_signed[i0]) / window)[ok],
        "mean_speed": ((cum_speed[i1] - cum_speed[i0]) / window)[ok],
    }


def heading_drift(
    reg_yaw: np.ndarray, model_yaw: np.ndarray, contiguous: np.ndarray, min_stretch: int = 100
) -> dict:
    """Heading error of the dead-reckoned yaw against the registered yaw inside each stretch of
    consecutive scans (at least ``min_stretch`` scans, 10 s): the running error after the stretch
    start, as a fraction of the heading turned (|net registered turn| is not used: laps cancel it;
    the sum of absolute registered steps is), and its worst value in rad."""
    starts = np.nonzero(~contiguous)[0]
    bounds = list(starts) + [len(reg_yaw)]
    final, worst, turned = [], 0.0, []
    for a, b in pairwise(bounds):
        if b - a < min_stretch:
            continue
        err = (model_yaw[a:b] - model_yaw[a]) - (reg_yaw[a:b] - reg_yaw[a])
        final.append(abs(err[-1]))
        worst = max(worst, float(np.abs(err).max()))
        turned.append(float(np.abs(np.diff(reg_yaw[a:b])).sum()))
    if not final:
        return {"stretches": 0}
    return {
        "stretches": len(final),
        "turned_rad": float(sum(turned)),
        "final_error_rad_median": float(np.median(final)),
        "final_error_rad_max": float(max(final)),
        "worst_running_error_rad": worst,
        "error_per_rad_turned": float(sum(final) / sum(turned)),
    }


def ratio(y: np.ndarray, x: np.ndarray) -> float:
    """Slope of y on x through the origin."""
    d = float((x * x).sum())
    return float((x * y).sum() / d) if d > 0.0 else math.nan


def ratio_of_sums(y: np.ndarray, x: np.ndarray) -> float:
    """sum(y) / sum(x). Unlike the regression slope this is not pulled towards zero by timing
    error in x (a modelled turn that leads or lags the registered one by a scan or two)."""
    d = float(x.sum())
    return float(y.sum() / d) if d != 0.0 else math.nan


def _row(w, m, **extra) -> dict | None:
    if m.sum() < 5:
        return None
    # The headline ratios are ratios of sums (regression dilution: see ratio_of_sums); the
    # through-origin slopes are kept as *_slope.
    yaw = ratio_of_sums(np.abs(w["reg_dyaw"][m]), np.abs(w["model_dyaw"][m]))
    spd = ratio_of_sums(w["reg_path"][m], w["model_path"][m])
    return {
        **extra,
        "n": int(m.sum()),
        "yaw_ratio": yaw,
        "speed_ratio": spd,
        "steer_ratio": yaw / spd if spd else math.nan,
        "yaw_ratio_slope": ratio(w["reg_dyaw"][m], w["model_dyaw"][m]),
    }


def summarise(w: dict[str, np.ndarray], bins=BINS, speed_bins=SPEED_BINS) -> list[dict]:
    """Rows by |steering| bin and side (left, right, both), then at full lock (|delta| >= 0.35)
    by speed bin and side."""
    rows = []
    sides = {
        "both": np.ones_like(w["mean_delta"], bool),
        "left": w["mean_delta"] > 0,
        "right": w["mean_delta"] < 0,
    }
    for side, sm in sides.items():
        for lo, hi in pairwise(bins):
            m = sm & (w["mean_abs_delta"] >= lo) & (w["mean_abs_delta"] < hi)
            r = _row(w, m, side=side, delta_lo=lo, delta_hi=hi, speed_lo=0.0, speed_hi=9.0)
            if r:
                rows.append(r)
        for lo, hi in pairwise(speed_bins):
            m = (
                sm
                & (w["mean_abs_delta"] >= 0.35)
                & (w["mean_speed"] >= lo)
                & (w["mean_speed"] < hi)
            )
            r = _row(w, m, side=side, delta_lo=0.35, delta_hi=0.60, speed_lo=lo, speed_hi=hi)
            if r:
                rows.append(r)
    return rows


def main(argv: list[str] | None = None) -> int:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("npz", type=Path)
    parser.add_argument("map", type=Path)
    parser.add_argument("--old-bag", action="store_true")
    parser.add_argument("--window", type=int, default=10)
    parser.add_argument(
        "--steer-gain",
        type=float,
        default=None,
        help="constant yaw gain (default: vehicle_params yaw_rate_gain_table; 1.0 = before 2026-11-12)",
    )
    parser.add_argument("--limit", type=int, default=0, help="only the first N scans")
    parser.add_argument("--json", type=Path)
    args = parser.parse_args(argv)

    data = np.load(args.npz)
    vehicle = load_vehicle()
    scale = OLD_BAGS if args.old_bag else BagScale()
    tw = wheel_twist_series(
        data["odom_t"],
        data["odom_v"],
        data["drive_t"],
        data["drive_steer"],
        vehicle,
        scale,
        YawModel(steer_gain=args.steer_gain),
        clamp_to_vehicle_limits=not args.old_bag,
    )
    ox, oy, oyaw = integrate_pose(tw["t"], tw["v"], tw["wz"])
    path = np.cumsum(np.abs(tw["v"]) * np.clip(np.diff(tw["t"], prepend=tw["t"][0]), 0, 0.1))

    st = data["scan_t"]
    meta = data["scan_meta"]
    v_at = np.interp(st, tw["t"], tw["v"])
    keep = np.where(np.abs(v_at) > 0.05)[0]
    # whole moving stretches only: registering a stationary car adds nothing
    if args.limit:
        keep = keep[: args.limit]
    pts = [
        scan_points_base(
            data["scan_ranges"][i].astype(float), meta[0], meta[1], vehicle.mount_xy_yaw
        )
        for i in keep
    ]
    odom_poses = np.column_stack(
        [
            np.interp(st[keep], tw["t"], ox),
            np.interp(st[keep], tw["t"], oy),
            np.interp(st[keep], tw["t"], oyaw),
        ]
    )
    grid = load_map_strict(args.map)
    cache = args.json.with_suffix(".reg.npz") if args.json else None
    if cache is not None and cache.exists() and len(np.load(cache)["yaw"]) == len(keep):
        c = np.load(cache)
        reg_xy, reg_yaw, rms = c["xy"], c["yaw"], c["rms"]
    else:
        regs = track_scans(grid, pts, odom_poses)
        reg_xy = np.array([[r.x, r.y] for r in regs])
        reg_yaw = np.unwrap([r.yaw for r in regs])
        rms = np.array([r.rms_m for r in regs])
        if cache is not None:
            np.savez(cache, xy=reg_xy, yaw=reg_yaw, rms=rms)
    print(
        f"registered {len(reg_yaw)} scans, rms median {np.nanmedian(rms) * 100:.1f} cm, "
        f"p95 {np.nanpercentile(rms, 95) * 100:.1f} cm",
        file=sys.stderr,
    )

    model_yaw = np.interp(st[keep], tw["t"], oyaw)
    model_path = np.interp(st[keep], tw["t"], path)
    delta = np.interp(st[keep], tw["t"], tw["delta"])
    contiguous = np.concatenate([[True], np.diff(st[keep]) < 0.25])
    moving = contiguous & (np.abs(v_at[keep]) > 0.1) & (v_at[keep] > 0)
    speed = np.interp(st[keep], tw["t"], np.abs(tw["v"]))
    # A window containing the first scan after a gap is not `moving` (see `contiguous`), so no
    # window spans a gap.
    w = window_stats(reg_yaw, reg_xy, model_yaw, model_path, delta, speed, moving, args.window)
    rows = summarise(w)
    print("side  |steer| bin (rad)  speed bin (m/s)     n   yaw ratio  speed ratio  steer ratio")
    for r in rows:
        print(
            f"{r['side']:5s} {r['delta_lo']:.2f}-{r['delta_hi']:.2f}  {r['speed_lo']:.2f}-{min(r['speed_hi'], 9):.2f}"
            f"  {r['n']:6d}   {r['yaw_ratio']:.3f}     {r['speed_ratio']:.3f}      {r['steer_ratio']:.3f}   (slope {r['yaw_ratio_slope']:.3f})"
        )
    drift = heading_drift(reg_yaw, model_yaw, contiguous)
    print("heading drift:", json.dumps(drift))
    if args.json:
        args.json.write_text(
            json.dumps(
                {"rows": rows, "drift": drift, "rms_median_m": float(np.nanmedian(rms))}, indent=1
            )
        )
    return 0


if __name__ == "__main__":
    sys.exit(main())
