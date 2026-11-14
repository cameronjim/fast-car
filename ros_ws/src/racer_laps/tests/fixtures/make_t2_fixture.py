"""How `t2_scans.npz` and `t2_golden.json` were made (not run in CI).

    uv run --extra replay python tests/fixtures/make_t2_fixture.py <bag dir> <out dir> <split yaml>

The bag is 2026-11-13T23-27-23_car_teleop (the mapped tracker with AMCL, 1.5 m/s line, segments T2a
and T2b of evaluation/prereg/pnp-v0.2-split.yaml). The slice is 18 s from the first moving scan of
the segment: two lap closures. Scans are stored as uint16 quarter millimetres (the LiDAR reports
in 0.25 mm steps, so this is lossless), wheel odometry and the gated steering as they were
recorded. The split is evaluation/prereg/pnp-v0.2-split.yaml. The golden crossing times are the start-scan gate's, in seconds after the first scan of
the slice, together with the map-pose detector's times on the same slice as an independent
reference (`pose_reference_s`); the two differ by less than 0.15 s.
"""

from __future__ import annotations

import json
import sys
from pathlib import Path

import numpy as np
from racer_laps.replay import read_messages, run_replay, segment_ranges

SLICE_S = 18.0
QUANTUM_M = 0.00025


def main(bag: str, out: str, split: str) -> None:
    messages = list(read_messages(bag))
    first = next(m[0] for m in messages if m[1] == "/scan")
    lo, _hi = segment_ranges(split, Path(bag).name, "T2a")
    sel = [m for m in messages if lo <= m[0] - first <= lo + SLICE_S]
    t0 = next(m[0] for m in sel if m[1] == "/scan")
    scans = [m for m in sel if m[1] == "/scan"]
    odom = [m for m in sel if m[1] == "/odom/wheel"]
    drive = [m for m in sel if m[1] == "/drive"]
    first_msg = scans[0][2]
    arrays = {
        "scan_stamp": np.array(
            [m.header.stamp.sec + m.header.stamp.nanosec * 1e-9 for _, _, m in scans]
        ),
        "scan_q": np.stack(
            [
                np.where(
                    np.isfinite(m.ranges), np.round(np.asarray(m.ranges) / QUANTUM_M), 0
                ).astype(np.uint16)
                for _, _, m in scans
            ]
        ),
        "scan_meta": np.array(
            [
                first_msg.angle_min,
                first_msg.angle_increment,
                first_msg.range_min,
                first_msg.range_max,
                first_msg.scan_time,
            ]
        ),
        "odom": np.array(
            [
                (m.header.stamp.sec + m.header.stamp.nanosec * 1e-9, m.twist.twist.linear.x)
                for _, _, m in odom
            ]
        ),
        "drive": np.array([(t, m.drive.steering_angle) for t, _, m in drive]),
        "t0": np.array([t0]),
    }
    Path(out).mkdir(parents=True, exist_ok=True)
    np.savez_compressed(Path(out) / "t2_scans.npz", **arrays)
    # the golden: replay the stored arrays exactly as the test does
    sys.path.insert(0, str(Path(__file__).resolve().parents[1]))
    from test_l4_fixture import replay_fixture

    golden = replay_fixture(Path(out) / "t2_scans.npz")
    pose = run_replay(sel, start_s=odom[0][0])
    ref = [round(lap.t_cross_s - t0, 3) for lap in pose.laps]
    meta = {
        "expected_crossings_s": [round(x, 3) for x in golden["crossings_s"]],
        "pose_reference_s": ref,
        "tolerance_s": 0.1,
        "source": "2026-11-13T23-27-23_car_teleop, T2a, 18 s from the first moving scan",
    }
    (Path(out) / "t2_golden.json").write_text(json.dumps(meta, indent=2) + "\n")
    print(meta)


if __name__ == "__main__":
    main(sys.argv[1], sys.argv[2], sys.argv[3])
