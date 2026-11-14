"""Cut the step 0 fixture of brief 29 from a lap node input dump (RUNNER_LAP_DUMP_DIR) of the
simulator: development case T2 (stadium, lane 1.0 m, straight 5.0 m, island radius 0.4 m), clockwise,
seed 12, condition a (D off at 0.5 m/s), the run `a_T2_cw_12` of 2026-11-14.

    python make_alias_fixture.py <dump.npz> <result.json> <out.npz>

In that run gate v1 closed lap 1 at 12.57 m of a 15.65 m loop and gate v2 with the design's first
violation limit (0.04) at 13.61 m: both on a false line about 2 m before the start. The fixture is
the inputs from the start scan to a little over one lap: scans (ranges quantised to 1 mm in uint16,
0 where there was no return), wheel odometry and gated steering, and the start time of the lap.
"""

from __future__ import annotations

import json
import sys

import numpy as np

LAP_S = 40.0  # inputs kept after the start scan (one lap is 34 s)


def main(dump: str, result: str, out: str) -> None:
    z = np.load(dump)
    with open(result) as f:
        first = json.load(f)["system_lap_events"][0]
    start = first["t_cross_s"] - first["lap_time_s"]
    scan_t, scan_r = z["scan_t"], z["scan_r"]
    keep = (scan_t >= start - 0.5) & (scan_t <= start + LAP_S)
    ranges = np.nan_to_num(scan_r[keep].astype(np.float64), nan=0.0, posinf=0.0, neginf=0.0)
    mm = np.clip(np.round(ranges * 1000.0), 0, 65535).astype(np.uint16)
    odom, drive = z["odom"], z["drive"]
    ko = (odom[:, 0] >= start - 1.0) & (odom[:, 0] <= start + LAP_S + 1.0)
    kd = (drive[:, 0] >= start - 1.0) & (drive[:, 0] <= start + LAP_S + 1.0)
    np.savez_compressed(
        out,
        scan_t=scan_t[keep],
        scan_mm=mm,
        odom=odom[ko],
        drive=drive[kd],
        start_t=np.array([start]),
    )


if __name__ == "__main__":
    main(*sys.argv[1:4])
