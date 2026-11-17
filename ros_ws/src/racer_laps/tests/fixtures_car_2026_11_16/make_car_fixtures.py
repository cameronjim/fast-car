"""How the two `.npz` fixtures of this directory were made (not run in CI). Brief 48.

    uv run --extra replay python tests/fixtures_car_2026_11_16/make_car_fixtures.py \\
        <bags dir> <out dir>

Both are slices of the car runs of 2026-11-16 (bags 2026-11-16T22-38-47_car_teleop, the GP3-R1 run,
and 2026-11-16T02-18-54_car_teleop, the earlier 20 lap attempt) that the lap node's logic as of
PR 191/201 got wrong, cut so that the slice's start line is the full run's start line (the first
scan of a slice is its start scan, so a slice that starts a metre before a crossing has a different
line and does not show the fault):

* `gp3r1_laps_14_to_17.npz`: bag seconds 116.5 to 141.0 of the GP3-R1 run (seconds after the
  first /scan). The full run's start line is crossed at 116.68 s (lap 13 ends) and 122.3 s,
  128.05 s, 133.56 s and 139.34 s end laps 14 to 17. Brief 48 asks for 115 to 141; at 115 s the
  start line is 1.7 s (1.6 m) before the full run's and the old gate flags nothing, so the slice
  starts at 116.5 s. 4 laps of 5.1 to 5.3 m. The gate of PR 201 flags the last one missed
  (decided at station 5.74 m after a crossing at 5.3 m: the live station passed the 5.2 m
  estimate plus the 0.5 m band while the closure was pending).
* `t0218_last_lap_parked.npz`: bag seconds 378.8 to 395.8 of 2026-11-16T02-18-54: two laps of
  5.2 m and then the car stands 0.15 m past the line from 390.8 s with a candidate on every scan:
  12 s of driving (brief 48 says 14) and 5 s parked. The old gate never closes the second lap.

Stored: the scans as uint16 quarter millimetres (lossless: the LiDAR reports in 0.25 mm steps),
the log time of every message used, the wheel odometry (stamp and speed), the gated steering, the
planner status (state and plan clearance) and the safety events (none fall in either slice, but
they are kept: they make a lap unclean).
"""

from __future__ import annotations

import json
import sys
from pathlib import Path

import numpy as np
from racer_laps.replay import read_messages

QUANTUM_M = 0.00025

FIXTURES = {
    "gp3r1_laps_14_to_17.npz": ("2026-11-16T22-38-47_car_teleop", 116.5, 141.0),
    "t0218_last_lap_parked.npz": ("2026-11-16T02-18-54_car_teleop", 378.8, 395.8),
}


def stamp_s(msg: object) -> float:
    return msg.header.stamp.sec + msg.header.stamp.nanosec * 1e-9  # type: ignore[attr-defined]


def cut(bag: Path, lo: float, hi: float) -> dict[str, np.ndarray]:
    messages = list(read_messages(bag))
    first = next(m[0] for m in messages if m[1] == "/scan")
    sel = [m for m in messages if lo <= m[0] - first <= hi]
    scans = [m for m in sel if m[1] == "/scan"]
    meta = scans[0][2]
    return {
        "scan_log": np.array([t for t, _, _ in scans]),
        "scan_stamp": np.array([stamp_s(m) for _, _, m in scans]),
        "scan_q": np.stack(
            [
                np.where(
                    np.isfinite(m.ranges), np.round(np.asarray(m.ranges) / QUANTUM_M), 0
                ).astype(np.uint16)
                for _, _, m in scans
            ]
        ),
        "scan_meta": np.array(
            [meta.angle_min, meta.angle_increment, meta.range_min, meta.range_max, meta.scan_time]
        ),
        "odom": np.array(
            [(t, stamp_s(m), m.twist.twist.linear.x) for t, tp, m in sel if tp == "/odom/wheel"]
        ),
        "drive": np.array([(t, m.drive.steering_angle) for t, tp, m in sel if tp == "/drive"]),
        "planner": np.array(
            [
                (t, m.state, m.plan_min_clearance_m)
                for t, tp, m in sel
                if tp == "/pnp/planner/status"
            ]
        ),
        "events": np.array(
            [(t, m.phase, m.severity) for t, tp, m in sel if tp == "/safety/events"]
        ).reshape(-1, 3),
        "event_source": np.array([str(m.source) for _, tp, m in sel if tp == "/safety/events"]),
        "t0": np.array([first]),
    }


def main(bags: str, out: str) -> None:
    Path(out).mkdir(parents=True, exist_ok=True)
    for name, (bag, lo, hi) in FIXTURES.items():
        arrays = cut(Path(bags) / bag, lo, hi)
        np.savez_compressed(Path(out) / name, **arrays)
        size = (Path(out) / name).stat().st_size
        print(f"{name}: {len(arrays['scan_stamp'])} scans, {size / 1e3:.0f} kB, source {bag}")
    (Path(out) / "SOURCES.json").write_text(
        json.dumps({n: {"bag": b, "from_s": lo, "to_s": hi} for n, (b, lo, hi) in FIXTURES.items()})
        + "\n"
    )


if __name__ == "__main__":
    main(sys.argv[1], sys.argv[2])
