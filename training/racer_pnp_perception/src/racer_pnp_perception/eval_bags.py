"""The real-bag gate: run both backends over recorded scans and decide whether the net may drive.

    uv run python -m racer_pnp_perception.eval_bags --bags <bag dir> [<bag dir> ...] \\
        [--bundle <bundle dir>] [--out <dir>]

Each bag is read with tools/follower_replay/bag.py and converted to the units of the current
vehicle_params with tools/safety_replay/calibration.py (every bag predates the 2026-11-10
calibration). The scans used are the ones where the car is moving: wheel speed above 0.2 m/s
(true units, after the conversion) from /odom/wheel, or the /drive speed above that for bags with
no odometry. Both backends run through `TrackPerceiver.process`, the code the node uses. Without
--bundle only the geometric backend is scored.

Per backend and bag it reports the median and 90th percentile of `boundary_residual_m`, the mean
`intrusion_fraction`, the mean `valid_count` and the share of moving scans with `valid_count`
under 6; with both backends, the mean distance between their centrelines over stations 0 to 7.
It also reports the clearance deficit (label-free, offline only): for each valid station within
2.0 m of the car, `min(half_width_left, half_width_right)` minus the Euclidean distance from the
station to the nearest scan point, as p50, p90 and the share over 0.15 m. A correct corridor has a
deficit near 0; a centreline 0.25 m too close to one wall has 0.25.
It writes `metrics.json` and three overlay pictures per bag.

The gate: the net is allowed on the car only if, over the two gate bags together, the median
`boundary_residual_m` is at most 0.10 m, the mean `intrusion_fraction` at most 0.02 and at most 5
percent of moving scans have `valid_count` under 6. The thresholds are not to be tuned.
"""

from __future__ import annotations

import argparse
import json
import logging
import math
import sys
import time
from dataclasses import dataclass
from pathlib import Path
from typing import Any

import numpy as np
from numpy.typing import NDArray
from racer_track_perception.core import (
    BACKEND_GEOMETRIC,
    BACKEND_NET,
    TrackPerceiver,
)
from racer_track_perception.preprocess import scan_to_points
from racer_track_perception.track import LocalTrackData
from racer_track_perception.vehicle import LiveVehicle, find_repo_root, load_live_vehicle

log = logging.getLogger(__name__)

MOVING_SPEED_MPS = 0.2
GATE_BAGS = ("2026-11-11T03-54-11_car_teleop", "2026-11-11T03-54-11_car_teleop_cw")
GATE_MEDIAN_RESIDUAL_M = 0.10
GATE_MEAN_INTRUSION = 0.02
GATE_SHORT_SHARE = 0.05
SHORT_VALID_COUNT = 6
COMPARE_STATIONS = 8
DEFICIT_STATION_RANGE_M = 2.0
DEFICIT_BAD_M = 0.15
N_OVERLAYS = 3
VERDICT_PASS = "net passes"
VERDICT_FAIL = "net fails, use backend:=geometric tonight"


@dataclass(frozen=True)
class ScanRec:
    stamp_s: float
    ranges: Any
    angle_min: float
    angle_increment: float
    range_min: float
    range_max: float


def moving_mask(
    scan_t: NDArray[np.float64],
    speed_t: NDArray[np.float64],
    speed_mps: NDArray[np.float64],
    threshold_mps: float = MOVING_SPEED_MPS,
) -> NDArray[np.bool_]:
    """True for scans whose latest speed sample at or before the scan stamp exceeds the
    threshold (absolute value). All False when there are no speed samples."""
    if len(speed_t) == 0 or len(scan_t) == 0:
        return np.zeros(len(scan_t), dtype=bool)
    order = np.argsort(speed_t)
    st, sv = np.asarray(speed_t)[order], np.asarray(speed_mps)[order]
    idx = np.searchsorted(st, scan_t, side="right") - 1
    valid = idx >= 0
    v = np.where(valid, np.abs(sv[np.clip(idx, 0, None)]), 0.0)
    return valid & (v > threshold_mps)


def centreline_distance(a: LocalTrackData, b: LocalTrackData, stations: int = 8) -> float:
    """Mean distance between two tracks' centrelines over stations 0..stations-1, NaN unless both
    have that many valid stations."""
    if min(a.valid_count, b.valid_count) < stations:
        return math.nan
    d = np.hypot(a.x_m[:stations] - b.x_m[:stations], a.y_m[:stations] - b.y_m[:stations])
    return float(d.mean())


def clearance_deficit(
    track: LocalTrackData,
    points: NDArray[np.floating],
    station_range_m: float = DEFICIT_STATION_RANGE_M,
) -> NDArray[np.float64]:
    """Per valid station within `station_range_m` of the car: the smaller half width minus the
    Euclidean distance to the nearest scan point. Empty when there is no valid station or no
    point."""
    n = track.valid_count
    pts = np.asarray(points, dtype=np.float64).reshape(-1, 2)
    if n < 1 or len(pts) == 0:
        return np.zeros(0)
    x = track.x_m[:n].astype(np.float64)
    y = track.y_m[:n].astype(np.float64)
    near = np.hypot(x, y) <= station_range_m
    if not near.any():
        return np.zeros(0)
    d2 = (x[near, None] - pts[None, :, 0]) ** 2 + (y[near, None] - pts[None, :, 1]) ** 2
    hw = np.minimum(track.half_width_left_m[:n], track.half_width_right_m[:n]).astype(np.float64)
    return hw[near] - np.sqrt(d2.min(axis=1))


def summarise(
    residual: NDArray[np.float64],
    intrusion: NDArray[np.float64],
    valid_count: NDArray[np.float64],
    latency_ms: NDArray[np.float64] | None = None,
    deficit: NDArray[np.float64] | None = None,
) -> dict[str, Any]:
    """The gate numbers for one backend over a set of moving scans. `deficit` is the clearance
    deficit of every evaluated station of every scan, concatenated."""
    n = len(valid_count)
    res = residual[np.isfinite(residual)]
    out: dict[str, Any] = {
        "moving_scans": int(n),
        "residual_scans": len(res),
        "boundary_residual_median_m": float(np.median(res)) if len(res) else math.nan,
        "boundary_residual_p90_m": float(np.percentile(res, 90)) if len(res) else math.nan,
        "intrusion_fraction_mean": float(np.nanmean(intrusion))
        if np.isfinite(intrusion).any()
        else math.nan,
        "valid_count_mean": float(np.mean(valid_count)) if n else math.nan,
        "share_valid_count_under_6": float(np.mean(valid_count < SHORT_VALID_COUNT))
        if n
        else math.nan,
    }
    if deficit is not None:
        out["clearance_deficit_stations"] = len(deficit)
        out["clearance_deficit_p50_m"] = (
            float(np.percentile(deficit, 50)) if len(deficit) else math.nan
        )
        out["clearance_deficit_p90_m"] = (
            float(np.percentile(deficit, 90)) if len(deficit) else math.nan
        )
        out["clearance_deficit_share_over_0_15"] = (
            float(np.mean(deficit > DEFICIT_BAD_M)) if len(deficit) else math.nan
        )
    if latency_ms is not None and len(latency_ms):
        out["latency_ms"] = {
            "p50": float(np.percentile(latency_ms, 50)),
            "p99": float(np.percentile(latency_ms, 99)),
            "max": float(latency_ms.max()),
        }
    return out


def gate_verdict(pooled: dict[str, Any]) -> tuple[str, dict[str, bool]]:
    """The net's verdict from its pooled numbers over the gate bags."""
    med = pooled["boundary_residual_median_m"]
    intr = pooled["intrusion_fraction_mean"]
    short = pooled["share_valid_count_under_6"]
    checks = {
        "median_boundary_residual": bool(med <= GATE_MEDIAN_RESIDUAL_M),  # NaN fails
        "mean_intrusion_fraction": bool(intr <= GATE_MEAN_INTRUSION),
        "share_valid_count_under_6": bool(short <= GATE_SHORT_SHARE),
    }
    return (VERDICT_PASS if all(checks.values()) else VERDICT_FAIL), checks


def score_scans(
    perceivers: dict[str, TrackPerceiver], scans: list[ScanRec]
) -> tuple[dict[str, dict[str, NDArray[np.float64]]], list[dict[str, LocalTrackData]], list[float]]:
    """Run every perceiver over the scans. Returns per-backend arrays (residual, intrusion,
    valid_count, latency_ms, and deficit: one entry per evaluated station, not per scan), the tracks per scan and the centreline distance per scan (NaN
    unless both backends are present)."""
    names = list(perceivers)
    cols: dict[str, dict[str, list[float]]] = {
        n: {"residual": [], "intrusion": [], "valid": [], "latency": []} for n in names
    }
    deficits: dict[str, list[NDArray[np.float64]]] = {n: [] for n in names}
    tracks: list[dict[str, LocalTrackData]] = []
    dist: list[float] = []
    for s in scans:
        per: dict[str, LocalTrackData] = {}
        for n in names:
            t0 = time.perf_counter()
            t = perceivers[n].process(
                s.ranges, s.angle_min, s.angle_increment, s.range_min, s.range_max
            )
            cols[n]["latency"].append((time.perf_counter() - t0) * 1000.0)
            cols[n]["residual"].append(t.boundary_residual_m)
            cols[n]["intrusion"].append(t.intrusion_fraction)
            cols[n]["valid"].append(float(t.valid_count))
            per[n] = t
            veh = perceivers[n].vehicle
            pts = scan_to_points(
                s.ranges,
                s.angle_min,
                s.angle_increment,
                s.range_min,
                s.range_max,
                veh.mount,
                veh.body,
            )
            deficits[n].append(clearance_deficit(t, pts))
        tracks.append(per)
        if BACKEND_NET in per and BACKEND_GEOMETRIC in per:
            dist.append(centreline_distance(per[BACKEND_NET], per[BACKEND_GEOMETRIC]))
        else:
            dist.append(math.nan)
    arrays = {n: {k: np.array(v, dtype=np.float64) for k, v in c.items()} for n, c in cols.items()}
    for n in names:
        arrays[n]["deficit"] = np.concatenate([np.zeros(0), *deficits[n]])
    return arrays, tracks, dist


def plot_overlay(
    path: Path,
    scan: ScanRec,
    tracks: dict[str, LocalTrackData],
    perceiver: TrackPerceiver,
    title: str,
) -> None:
    """Scan points, each backend's centreline and boundaries, in base_link (x forward, y left)."""
    import matplotlib

    matplotlib.use("Agg")
    import matplotlib.pyplot as plt
    from racer_track_perception.preprocess import scan_to_points

    pts = scan_to_points(
        scan.ranges,
        scan.angle_min,
        scan.angle_increment,
        scan.range_min,
        scan.range_max,
        perceiver.vehicle.mount,
        perceiver.vehicle.body,
    )
    fig, ax = plt.subplots(figsize=(7, 7))
    ax.scatter(pts[:, 1], pts[:, 0], s=3, c="0.55", label="scan")
    body = perceiver.vehicle.body
    ax.add_patch(
        plt.Rectangle(
            (-body.half_width_m, body.x_min_m),
            body.width_m,
            body.x_max_m - body.x_min_m,
            fill=False,
            ec="k",
        )
    )
    colours = {BACKEND_NET: "tab:red", BACKEND_GEOMETRIC: "tab:blue"}
    for name, t in tracks.items():
        n = t.valid_count
        if n < 2:
            continue
        x, y = t.x_m[:n].astype(float), t.y_m[:n].astype(float)
        tx, ty = np.gradient(x), np.gradient(y)
        norm = np.maximum(np.hypot(tx, ty), 1e-9)
        nx, ny = -ty / norm, tx / norm
        left = (x + t.half_width_left_m[:n] * nx, y + t.half_width_left_m[:n] * ny)
        right = (x - t.half_width_right_m[:n] * nx, y - t.half_width_right_m[:n] * ny)
        c = colours.get(name, "tab:green")
        ax.plot(y, x, "-o", c=c, ms=3, label=f"{name} centre ({n} valid)")
        ax.plot(left[1], left[0], "--", c=c, lw=1)
        ax.plot(right[1], right[0], "--", c=c, lw=1)
    ax.set_xlim(2.5, -2.5)  # y to the left is plotted to the left
    ax.set_ylim(-1.0, 4.0)
    ax.set_aspect("equal")
    ax.set_xlabel("y, left (m)")
    ax.set_ylabel("x, forward (m)")
    ax.set_title(title, fontsize=9)
    ax.legend(loc="upper right", fontsize=7)
    fig.savefig(path, dpi=110, bbox_inches="tight")
    plt.close(fig)


def load_bag_scans(bag_dir: Path) -> tuple[list[ScanRec], NDArray[np.bool_], dict[str, Any]]:
    """The scans of a bag and the moving mask, in the units of the current vehicle_params."""
    tools = find_repo_root() / "tools"
    if str(tools) not in sys.path:
        sys.path.insert(0, str(tools))
    from follower_replay.bag import read_bag
    from safety_replay.calibration import RECORDED_0_13_0, convert_bag, load_current

    data = convert_bag(read_bag(bag_dir), RECORDED_0_13_0, load_current())
    scans = [
        ScanRec(s.stamp_s, s.ranges, s.angle_min, s.angle_increment, s.range_min, s.range_max)
        for s in data.scans
    ]
    scan_t = np.array([s.stamp_s for s in scans])
    info: dict[str, Any] = {"truncated": bool(getattr(data, "truncated", False))}
    if data.odom:
        speed_t = np.array([o.stamp_s for o in data.odom])
        speed = np.array([o.speed_mps for o in data.odom])
        info["moving_source"] = "/odom/wheel"
    else:
        speed_t = np.array([d.stamp_s for d in data.drive])
        speed = np.array([d.speed_mps for d in data.drive])
        info["moving_source"] = "/drive"
    return scans, moving_mask(scan_t, speed_t, speed), info


def build_perceivers(live: LiveVehicle, bundle_dir: Path | None) -> dict[str, TrackPerceiver]:
    out = {BACKEND_GEOMETRIC: TrackPerceiver(BACKEND_GEOMETRIC, live)}
    if bundle_dir is not None:
        from racer_track_perception.bundle import load_bundle
        from racer_track_perception.runtime import CircConv1dV0

        bundle = load_bundle(bundle_dir, "track_perception", live)
        out[BACKEND_NET] = TrackPerceiver(
            BACKEND_NET,
            live,
            net=CircConv1dV0.from_npz(bundle.payload_path("weights.npz")),
            model_id=bundle.model_id,
            refine_net=bundle.refine,  # the bundle says what runs
        )
    return out


def evaluate_bag(
    bag_dir: Path, perceivers: dict[str, TrackPerceiver], out_dir: Path
) -> tuple[dict[str, Any], dict[str, dict[str, NDArray[np.float64]]]]:
    scans, mask, info = load_bag_scans(bag_dir)
    moving = [s for s, m in zip(scans, mask, strict=True) if m]
    log.info("%s: %d scans, %d moving", bag_dir.name, len(scans), len(moving))
    arrays, tracks, dist = score_scans(perceivers, moving)
    result: dict[str, Any] = {
        "scans": len(scans),
        "moving_scans": len(moving),
        "moving_source": info["moving_source"],
        "truncated": info["truncated"],
        "backends": {
            n: summarise(a["residual"], a["intrusion"], a["valid"], a["latency"], a["deficit"])
            for n, a in arrays.items()
        },
    }
    d = np.array(dist, dtype=np.float64)
    if BACKEND_NET in perceivers and BACKEND_GEOMETRIC in perceivers:
        result["centreline_distance_stations_0_7_mean_m"] = (
            float(np.nanmean(d)) if np.isfinite(d).any() else math.nan
        )
        result["centreline_distance_scans"] = int(np.isfinite(d).sum())
    if moving:
        out_dir.mkdir(parents=True, exist_ok=True)
        picks = sorted({int(q * (len(moving) - 1)) for q in np.linspace(0.15, 0.85, N_OVERLAYS)})
        first = next(iter(perceivers.values()))
        for j, i in enumerate(picks):
            plot_overlay(
                out_dir / f"{bag_dir.name}_overlay{j}.png",
                moving[i],
                tracks[i],
                first,
                f"{bag_dir.name} moving scan {i} of {len(moving)}",
            )
    return result, arrays


def pool(per_bag: list[dict[str, dict[str, NDArray[np.float64]]]], backend: str) -> dict[str, Any]:
    cat = {
        k: np.concatenate([b[backend][k] for b in per_bag])
        for k in ("residual", "intrusion", "valid", "latency", "deficit")
    }
    return summarise(
        cat["residual"], cat["intrusion"], cat["valid"], cat["latency"], cat["deficit"]
    )


def main(argv: list[str] | None = None) -> int:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--bags", nargs="+", type=Path, required=True)
    parser.add_argument("--bundle", type=Path, default=None)
    parser.add_argument("--out", type=Path, default=Path("eval_bags_out"))
    args = parser.parse_args(argv)
    logging.basicConfig(level=logging.INFO, format="%(message)s")
    live = load_live_vehicle()
    perceivers = build_perceivers(live, args.bundle)
    args.out.mkdir(parents=True, exist_ok=True)
    metrics: dict[str, Any] = {"bags": {}, "gate_bags": list(GATE_BAGS)}
    arrays_by_bag: dict[str, dict[str, dict[str, NDArray[np.float64]]]] = {}
    for bag in args.bags:
        result, arrays = evaluate_bag(bag, perceivers, args.out)
        metrics["bags"][bag.name] = result
        arrays_by_bag[bag.name] = arrays
    gate_arrays = [arrays_by_bag[n] for n in GATE_BAGS if n in arrays_by_bag]
    if len(gate_arrays) == len(GATE_BAGS):
        metrics["gate_pooled"] = {n: pool(gate_arrays, n) for n in perceivers}
        for n, pooled in metrics["gate_pooled"].items():
            verdict, checks = gate_verdict(pooled)
            metrics["gate_pooled"][n]["verdict"] = verdict
            metrics["gate_pooled"][n]["checks"] = checks
        if BACKEND_NET in perceivers:
            metrics["verdict"] = metrics["gate_pooled"][BACKEND_NET]["verdict"]
    else:
        metrics["gate_pooled"] = None
        metrics["note"] = "both gate bags were not given, so no pooled verdict"
    (args.out / "metrics.json").write_text(json.dumps(metrics, indent=2, allow_nan=True))
    print(json.dumps(metrics, indent=2, allow_nan=True))
    return 0


if __name__ == "__main__":
    sys.exit(main())
