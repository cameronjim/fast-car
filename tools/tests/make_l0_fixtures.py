"""Cut the committed L4 fixtures of the data loop out of the real L0 bag (2026-11-14T21-49-21).

    uv run --project tools --extra session python tools/tests/make_l0_fixtures.py

L0 has the lap role `dev` in pnp-v0.4.yaml, so reading it is allowed. Writes
fixtures/pnp_data_l0_cut.npz (the arrays `pnp_data runs` reads, 120 to 245 s) and
fixtures/lapref_l0_cut.npz (steering of /drive and speed of /odom/wheel, the run only).
"""

from __future__ import annotations

import sys
from pathlib import Path

import numpy as np

sys.path.insert(0, str(Path(__file__).resolve().parents[1]))

from lapref.bag import read_signals
from pnp_data.runs import RunSignals, read_run_signals

BAG = Path("/Users/cameronjim/code/car-data/bags/2026-11-14T21-49-21_car_teleop")
FIX = Path(__file__).resolve().parent / "fixtures"


def main() -> None:
    sig = read_run_signals(BAG)
    lo, hi = 120.0, 245.0
    d = (sig.draw_t >= lo) & (sig.draw_t <= hi)
    o = (sig.odom_t >= lo) & (sig.odom_t <= hi)
    s = (sig.status_t >= lo) & (sig.status_t <= hi)
    cut = RunSignals(
        sig.draw_t[d],
        sig.draw_speed[d].astype(np.float32),
        sig.draw_steer[d].astype(np.float32),
        sig.odom_t[o],
        sig.odom_v[o].astype(np.float32),
        sig.status_t[s],
        sig.first_scan_log_s,
    )
    cut.save(FIX / "pnp_data_l0_cut.npz")
    ls = read_signals(BAG).window(124.9, 239.9)
    np.savez_compressed(
        FIX / "lapref_l0_cut.npz",
        steer_t=ls.steer_t,
        steer=ls.steer.astype(np.float32),
        odom_t=ls.odom_t,
        odom_v=ls.odom_v.astype(np.float32),
    )


if __name__ == "__main__":
    main()
