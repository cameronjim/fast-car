"""Cut the ring fixture of brief 50 from a lap node input dump (RUNNER_LAP_DUMP_DIR) of the
simulator: development case P4 (stadium with no straight: lane 1.1 m, island radius 0.40 m, a ring),
counter-clockwise, seed 11, condition a (D off at 0.5 m/s), `a_P4_ccw_11`, 4 truth laps.

    python make_ring_fixture.py <dump.npz> <result.json> <out.npz> [<golden.json>]

The fixture is the lap node's inputs from the start scan to a little over three truth laps: scans
(ranges quantised to 1 mm in uint16, 0 where there was no return), wheel odometry and gated steering,
the start time of the lap and the yaw gains the run's lap node was given. With a fourth argument
the golden of `test_l4_ring_fixture.py` is written: what gate v3 made of the fixture when it was
cut (lap times, line sources), to be compared with a tolerance of 0.02 s and never refreshed to get
a test green.
"""

from __future__ import annotations

import json
import sys
from pathlib import Path

import numpy as np

TAIL_S = 3.0  # kept after truth crossing 3: the system is a second late and decides 0.4 m on
LAPS = 3


def cut(dump: str, result: str, out: str) -> None:
    z = np.load(dump)
    with open(result) as f:
        doc = json.load(f)
    first = doc["system_lap_events"][0]
    start = first["t_cross_s"] - first["lap_time_s"]
    end = float(doc["gt_lap_crossings_registered_s"][LAPS - 1]) + TAIL_S
    scan_t, scan_r = z["scan_t"], z["scan_r"]
    keep = (scan_t >= start - 0.5) & (scan_t <= end)
    ranges = np.nan_to_num(scan_r[keep].astype(np.float64), nan=0.0, posinf=0.0, neginf=0.0)
    mm = np.clip(np.round(ranges * 1000.0), 0, 65535).astype(np.uint16)
    odom, drive = z["odom"], z["drive"]
    ko = (odom[:, 0] >= start - 1.0) & (odom[:, 0] <= end + 1.0)
    kd = (drive[:, 0] >= start - 1.0) & (drive[:, 0] <= end + 1.0)
    params = doc["lap_params"]
    np.savez_compressed(
        out,
        scan_t=scan_t[keep],
        scan_mm=mm,
        odom=odom[ko],
        drive=drive[kd],
        start_t=np.array([start]),
        yaw_gains=np.array(
            [float(params["yaw_gain_override"]), float(params["yaw_gain_override_right"])]
        ),
        truth_lap_s=np.array(doc["gt_lap_times_registered_s"][:LAPS]),
    )


def main(argv: list[str]) -> None:
    dump, result, out = argv[:3]
    cut(dump, result, out)
    if len(argv) > 3:
        sys.path.insert(0, str(Path(__file__).resolve().parents[1]))
        from test_l4_ring_fixture import replay

        laps = replay()
        Path(argv[3]).write_text(
            json.dumps(
                {
                    "tolerance_s": 0.02,
                    "n_laps": len(laps),
                    "lap_times_s": [round(r.lap_time_s, 3) for r in laps],
                    "lengths_m": [round(r.length_m, 3) for r in laps],
                    "line_sources": [r.line_source for r in laps],
                },
                indent=1,
            )
            + "\n"
        )


if __name__ == "__main__":
    main(sys.argv[1:])
