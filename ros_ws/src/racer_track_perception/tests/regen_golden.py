"""Regenerate the golden output of the geometric backend on the real-scan fixture.

    uv run python tests/regen_golden.py

Run it only when the geometric tracer was changed on purpose, and say so in the commit: the
golden file is what pins the tracer's behaviour on real scans (test_real_scans.py, 0.02 m).
"""

from __future__ import annotations

from pathlib import Path

import numpy as np
from racer_track_perception import core
from racer_track_perception.vehicle import load_live_vehicle

FIXTURES = Path(__file__).parent / "fixtures"
SCANS = FIXTURES / "real_scans_2026-11-11.npz"
GOLDEN = FIXTURES / "real_scans_2026-11-11_golden.npz"


def run_geometric(live=None) -> dict[str, np.ndarray]:
    live = live or load_live_vehicle()
    perceiver = core.TrackPerceiver(core.BACKEND_GEOMETRIC, live)
    with np.load(SCANS) as d:
        ranges = d["ranges"]
        meta = (
            float(d["angle_min"]),
            float(d["angle_increment"]),
            float(d["range_min"]),
            float(d["range_max"]),
        )
    cols: dict[str, list] = {k: [] for k in ("x", "y", "hl", "hr", "sigma", "valid")}
    for r in ranges:
        t = perceiver.process(r, *meta)
        cols["x"].append(t.x_m)
        cols["y"].append(t.y_m)
        cols["hl"].append(t.half_width_left_m)
        cols["hr"].append(t.half_width_right_m)
        cols["sigma"].append(t.sigma_m)
        cols["valid"].append(t.valid_count)
    return {k: np.array(v) for k, v in cols.items()}


if __name__ == "__main__":
    out = run_geometric()
    np.savez(GOLDEN, **out)
    print(f"wrote {GOLDEN}: {len(out['valid'])} scans, mean valid {out['valid'].mean():.2f}")
