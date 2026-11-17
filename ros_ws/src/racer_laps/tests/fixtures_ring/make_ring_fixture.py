"""Cut a ring fixture (briefs 50 and 50b) from a lap node input dump (RUNNER_LAP_DUMP_DIR) of the
simulator, development data: `a_P4_ccw_11` and `a_P4_cw_11` (stadium with no straight: lane 1.1 m,
island radius 0.40 m, a ring; condition a, D off at 0.5 m/s, 4 truth laps), `c_ND_ccw_11` and
`c_ND_cw_11` (near ring ND: straight 0.3 m, lane 1.0 m, island radius 0.5 m; condition c, D v0, 13
truth laps).

    python make_ring_fixture.py <dump.npz> <result.json> <out.npz> <laps>

The fixture is the lap node's inputs from the start scan to a little over `laps` truth laps: scans
(ranges quantised to 1 mm in uint16, 0 where there was no return), wheel odometry and gated steering,
the start time of the lap, the yaw gains the run's lap node was given and the truth lap times.
"""

from __future__ import annotations

import json
import sys

import numpy as np

TAIL_S = 3.0  # kept after the last truth crossing: the system is late and decides 0.4 m on


def cut(dump: str, result: str, out: str, laps: int) -> None:
    z = np.load(dump)
    with open(result) as f:
        doc = json.load(f)
    first = doc["system_lap_events"][0]
    start = first["t_cross_s"] - first["lap_time_s"]
    end = float(doc["gt_lap_crossings_registered_s"][laps - 1]) + TAIL_S
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
        truth_lap_s=np.array(doc["gt_lap_times_registered_s"][:laps]),
    )


def main(argv: list[str]) -> None:
    dump, result, out, laps = argv[:4]
    cut(dump, result, out, int(laps))


if __name__ == "__main__":
    main(sys.argv[1:])
