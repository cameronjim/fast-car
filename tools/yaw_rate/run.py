"""From a bag recording and a split-file segment to the pipeline's `SegmentInput`."""

from __future__ import annotations

import logging
import time
from dataclasses import dataclass
from pathlib import Path
from typing import Any

import numpy as np
from numpy.typing import NDArray

from . import matching, signals
from .bagio import Recording
from .pipeline import IcpInput, SegmentInput
from .vehicle import Vehicle

log = logging.getLogger(__name__)

MIN_MATCH_SPEED_MPS = 0.1  # scan pairs below this wheel speed are not matched (the car is parked)
WHEEL_PAD_S = 1.0  # wheel samples kept beyond the segment, for the lag search
ICP_CACHE_VERSION = 1


@dataclass(frozen=True)
class SegmentSpec:
    bag: str
    name: str
    t_from: float
    t_to: float
    direction: str


def split_segments(split: dict[str, Any], names: tuple[str, ...]) -> list[SegmentSpec]:
    """The named segments of a split file (bags' `segments` lists), in the order of `names`."""
    found: dict[str, SegmentSpec] = {}
    for bag, entry in split["bags"].items():
        for seg in entry.get("segments", []):
            found[seg["name"]] = SegmentSpec(
                bag, seg["name"], float(seg["from"]), float(seg["to"]), str(seg["direction"])
            )
    missing = [n for n in names if n not in found]
    if missing:
        raise KeyError(f"segments not in the split file: {missing}")
    return [found[n] for n in names]


def steering_model_rate(rec: Recording, vehicle: Vehicle) -> tuple[Any, Any]:
    """v_wheel * tan(delta_cmd) / L at every /odom/wheel sample, delta_cmd the gated /drive
    steering command held to the sample time. It leaves out the steering lag, so against a real
    yaw rate it includes that lag."""
    delta = np.interp(rec["odom_t"], rec["drive_t"], rec["drive_steer"])
    return rec["odom_t"], rec["odom_v"] * np.tan(delta) / vehicle.wheelbase_m


def segment_scans(rec: Recording, spec: SegmentSpec) -> NDArray[np.int64]:
    """Indices of the scans whose arrival time is inside the segment."""
    t = rec["scan_t"]
    return np.nonzero((t >= spec.t_from) & (t <= spec.t_to))[0]


def segment_poses(rec: Recording, scans: NDArray[np.int64]) -> NDArray[np.int64]:
    """Indices of the /pose messages computed from one of the segment's scans (their header
    stamp is that scan's stamp)."""
    stamps = rec["scan_stamp"][scans]
    lo, hi = float(stamps[0]) - 1e-3, float(stamps[-1]) + 1e-3
    return np.nonzero((rec["pose_stamp"] >= lo) & (rec["pose_stamp"] <= hi))[0]


def run_icp(
    rec: Recording,
    scans: NDArray[np.int64],
    vehicle: Vehicle,
    speed: signals.RateSeries,
    wheel: signals.RateSeries,
    workers: int,
    cache: Path | None,
    key: str,
    seeded: bool,
) -> IcpInput:
    """Single steps over the segment's consecutive scans and the direct i to i + 2 matches, for
    the pairs where the car is moving. `seeded` starts every match at the wheel twist's motion
    over the interval instead of the identity."""
    stamps = rec["scan_stamp"]
    idx = scans[:-1]
    t0, t1 = stamps[idx], stamps[idx + 1]
    want = np.nan_to_num(speed.mean(t0, t1), nan=0.0) >= MIN_MATCH_SPEED_MPS
    want2 = want[:-1] & want[1:]
    if cache is not None and cache.is_file():
        with np.load(cache) as z:
            if int(z["version"]) == ICP_CACHE_VERSION and str(z["key"]) == key:
                return IcpInput(t0, t1, z["yaw"], z["computed"], z["yaw2"])
    ignore = matching.self_returns(rec, vehicle)
    log.info(
        "  matching %d + %d scan pairs (seeded: %s)", int(want.sum()), int(want2.sum()), seeded
    )

    def seed(sel: NDArray[np.int64], lag: int) -> NDArray[np.float64] | None:
        if not seeded:
            return None
        a, b = stamps[sel], stamps[sel + lag]
        dyaw = np.nan_to_num(wheel.integral(a, b))
        dist = np.nan_to_num(speed.integral(a, b))
        return np.stack([dyaw, dist], axis=1)

    sel1 = idx[want]
    sel2 = idx[:-1][want2]
    s1 = matching.match_steps(rec, sel1, 1, vehicle, ignore, workers, seed(sel1, 1))
    s2 = matching.match_steps(rec, sel2, 2, vehicle, ignore, workers, seed(sel2, 2))
    yaw = np.full(len(idx), np.nan)
    yaw[np.nonzero(want)[0][s1.ok]] = s1.dyaw[s1.ok]
    yaw2 = np.full(max(len(idx) - 1, 0), np.nan)
    yaw2[np.nonzero(want2)[0][s2.ok]] = s2.dyaw[s2.ok]
    if cache is not None:
        cache.parent.mkdir(parents=True, exist_ok=True)
        np.savez(cache, version=ICP_CACHE_VERSION, key=key, yaw=yaw, computed=want, yaw2=yaw2)
    return IcpInput(t0, t1, yaw, want, yaw2)


def build_segment_input(
    rec: Recording,
    spec: SegmentSpec,
    vehicle: Vehicle,
    workers: int = 1,
    icp_cache: Path | None = None,
    variants: tuple[str, ...] = ("icp", "icp_seeded"),
) -> SegmentInput:
    """`variants` are the scan-matching signals to compute: "icp" (identity start) and
    "icp_seeded" (wheel twist start). Their results are cached next to each other."""
    scans = segment_scans(rec, spec)
    if len(scans) < 4:
        raise ValueError(f"{spec.name}: fewer than 4 scans in [{spec.t_from}, {spec.t_to}]")
    poses = segment_poses(rec, scans)
    lo = float(rec["scan_stamp"][scans[0]]) - WHEEL_PAD_S
    hi = float(rec["scan_stamp"][scans[-1]]) + WHEEL_PAD_S
    w = (rec["wt_stamp"] >= lo) & (rec["wt_stamp"] <= hi)
    steer_t, steer_wz = steering_model_rate(rec, vehicle)
    s = (steer_t >= lo) & (steer_t <= hi)
    wheel_speed = signals.RateSeries(rec["wt_stamp"][w], np.abs(rec["wt_vx"][w]))
    wheel_rate = signals.RateSeries(rec["wt_stamp"][w], rec["wt_wz"][w])
    icp: dict[str, IcpInput] = {}
    for name in variants:
        key = (
            f"{spec.bag}|{spec.name}|{spec.t_from}|{spec.t_to}|{name}|"
            f"{matching.RMS_MAX_M}|{matching.INLIER_SHARE_MIN}|{matching.MAX_RANGE_M}"
        )
        cache = None if icp_cache is None else icp_cache.with_name(f"{icp_cache.stem}_{name}.npz")
        icp[name] = run_icp(
            rec, scans, vehicle, wheel_speed, wheel_rate, workers, cache, key, name == "icp_seeded"
        )
    return SegmentInput(
        name=spec.name,
        direction=spec.direction,
        pose_stamp=rec["pose_stamp"][poses],
        pose_x=rec["pose_x"][poses],
        pose_y=rec["pose_y"][poses],
        pose_yaw=rec["pose_yaw"][poses],
        wheel_t=rec["wt_stamp"][w],
        wheel_vx=rec["wt_vx"][w],
        wheel_wz=rec["wt_wz"][w],
        steer_t=steer_t[s],
        steer_wz=steer_wz[s],
        icp=icp,
    )


def cpu_cost_per_scan_s(
    rec: Recording, scans: NDArray[np.int64], vehicle: Vehicle, n: int = 200
) -> dict[str, float]:
    """CPU seconds per scan pair of `matching.scan_motion` (numpy, one thread), median and 90th
    percentile over `n` moving pairs spread over `scans`. A rough indication only."""
    ignore = matching.self_returns(rec, vehicle)
    mount = matching.mount_of(vehicle)
    picks = scans[np.linspace(0, len(scans) - 2, n).astype(int)]
    times = []
    for i in picks:
        a, b = matching.scan_record(rec, int(i)), matching.scan_record(rec, int(i) + 1)
        started = time.process_time()
        try:
            matching.scan_motion(a, b, mount, ignore)
        except ValueError:
            continue
        times.append(time.process_time() - started)
    arr = np.array(times)
    return {
        "median_s": float(np.median(arr)),
        "p90_s": float(np.percentile(arr, 90)),
        "n": len(arr),
    }
