"""Run the node's own decision code (racer_track_perception.deskew_relay.DeskewRelay) over a
recording, in arrival order, the way scan_deskew_node sees the topics: every odometry and drive
sample reaches the relay before the scan that arrives after it, and a scan is deskewed with the
twist the relay pairs with it. Nothing of the algorithm is copied here.

Times: the recording's are seconds after the first scan's log time; the relay is fed absolute
times (`t0_log_s` added back) because it treats a stamp at or below zero as unset.
"""

from __future__ import annotations

import sys
from dataclasses import dataclass
from pathlib import Path
from typing import Any

import numpy as np
from numpy.typing import NDArray

REPO_ROOT = Path(__file__).resolve().parents[2]
PERCEPTION_SRC = REPO_ROOT / "ros_ws" / "src" / "racer_track_perception" / "src"
PRELOAD_S = 2.0  # odometry and drive history fed before a segment's first scan

HYPOTHESES = (
    ("sweep_start", "index_ascending"),
    ("sweep_start", "index_descending"),
    ("sweep_end", "index_ascending"),
    ("sweep_end", "index_descending"),
)


def _modules() -> tuple[Any, Any, Any]:
    if str(PERCEPTION_SRC) not in sys.path:
        sys.path.insert(0, str(PERCEPTION_SRC))
    from racer_track_perception import deskew, deskew_relay, vehicle

    return deskew, deskew_relay, vehicle


@dataclass
class Deskewed:
    """Per scan of a segment: ranges (raw or deskewed), the stamp offset applied, the reason a
    scan passed through ('' when deskewed), the twist used (nan when none)."""

    index: NDArray[np.int64]
    ranges: NDArray[np.float32]  # [n, beams]
    offset_s: NDArray[np.float64]
    reason: list[str]
    v_mps: NDArray[np.float64]
    yaw_rate_radps: NDArray[np.float64]
    process_ms: NDArray[np.float64]


def build_relay(stamp_is: str, beam_order: str, reference: str = "sweep_end") -> Any:
    deskew, deskew_relay, vehicle = _modules()
    vp = vehicle.load_generated_params(REPO_ROOT)
    live = vehicle.from_generated(vp)
    table = tuple((float(p.abs_angle_rad), float(p.gain)) for p in vp.steering.yaw_rate_gain_table)
    cfg = deskew.DeskewConfig(stamp_is=stamp_is, beam_order=beam_order, reference=reference)
    return deskew_relay.DeskewRelay(cfg, live.mount, live.body, table)


def deskew_segment(
    rec: dict[str, NDArray[Any]],
    scan_index: NDArray[np.int64],
    relay: Any,
    scan_ranges: Any,
) -> Deskewed:
    """Feed the relay the odometry and drive of [first scan - PRELOAD_S, last scan] and process
    the scans `scan_index` in arrival order. `scan_ranges(rec, i)` gives float32 ranges."""
    t0 = float(rec["t0_log_s"])
    idx = np.asarray(scan_index, dtype=np.int64)
    n = len(idx)
    beams = len(scan_ranges(rec, int(idx[0]))) if n else 0
    out = Deskewed(
        idx,
        np.zeros((n, beams), np.float32),
        np.zeros(n),
        [""] * n,
        np.full(n, np.nan),
        np.full(n, np.nan),
        np.zeros(n),
    )
    if n == 0:
        return out
    lo = float(rec["scan_t"][idx[0]]) - PRELOAD_S
    hi = float(rec["scan_t"][idx[-1]])
    events: list[tuple[float, int, int]] = []  # (arrival, kind, row)
    for kind, key in ((0, "odom_t"), (1, "drive_t")):
        t = rec[key]
        for r in np.nonzero((t >= lo) & (t <= hi))[0]:
            events.append((float(t[r]), kind, int(r)))
    for k, i in enumerate(idx):
        events.append((float(rec["scan_t"][i]), 2, k))
    events.sort(key=lambda e: (e[0], e[1]))
    for arrival, kind, row in events:
        if kind == 0:
            relay.on_odom(
                arrival + t0, float(rec["odom_stamp"][row]) + t0, float(rec["odom_v"][row])
            )
        elif kind == 1:
            relay.on_drive(arrival + t0, 0.0, float(rec["drive_steer"][row]))
        else:
            i = int(idx[row])
            raw = scan_ranges(rec, i)
            res = relay.process(
                arrival + t0,
                float(rec["scan_stamp"][i]) + t0,
                raw,
                float(rec["scan_angle_min"][i]),
                float(rec["scan_angle_increment"][i]),
                float(rec["scan_range_min"][i]),
                float(rec["scan_range_max"][i]),
                float(rec["scan_time"][i]),
                float(rec["scan_dt"][i]),
            )
            out.process_ms[row] = res.process_ms
            if res.ranges is None:
                out.ranges[row] = raw
                out.reason[row] = res.reason
            else:
                out.ranges[row] = res.ranges
                out.offset_s[row] = res.stamp_offset_s
                out.v_mps[row] = res.twist.v_mps
                out.yaw_rate_radps[row] = res.twist.yaw_rate_radps
    return out


def raw_segment(
    rec: dict[str, NDArray[Any]], scan_index: NDArray[np.int64], scan_ranges: Any
) -> Deskewed:
    idx = np.asarray(scan_index, dtype=np.int64)
    return Deskewed(
        idx,
        np.stack([scan_ranges(rec, int(i)) for i in idx])
        if len(idx)
        else np.zeros((0, 0), np.float32),
        np.zeros(len(idx)),
        ["raw"] * len(idx),
        np.full(len(idx), np.nan),
        np.full(len(idx), np.nan),
        np.zeros(len(idx)),
    )


def wheel_yaw_rate_at_scans(
    rec: dict[str, NDArray[Any]], scan_index: NDArray[np.int64]
) -> tuple[NDArray[np.float64], NDArray[np.float64]]:
    """(wheel speed, wheel-model yaw rate) at each scan's arrival: the newest /odom/wheel and
    /drive sample at or before it. This is what classifies a scan as a fast corner scan."""
    _deskew, deskew_relay, vehicle = _modules()
    vp = vehicle.load_generated_params(REPO_ROOT)
    live = vehicle.from_generated(vp)
    table = tuple((float(p.abs_angle_rad), float(p.gain)) for p in vp.steering.yaw_rate_gain_table)
    t = rec["scan_t"][scan_index]
    oi = np.clip(np.searchsorted(rec["odom_t"], t, side="right") - 1, 0, None)
    di = np.clip(np.searchsorted(rec["drive_t"], t, side="right") - 1, 0, None)
    v = rec["odom_v"][oi]
    delta = rec["drive_steer"][di]
    rate = np.array(
        [
            deskew_relay.wheel_yaw_rate(float(a), float(b), live.body.wheelbase_m, table)
            for a, b in zip(v, delta, strict=True)
        ]
    )
    return v, rate
