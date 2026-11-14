"""Write the golden output of the REFINED net backend on the two committed real-scan fixtures.

    uv run python tests/make_refined_golden.py

The net is the released v0 weights that are already committed (`test/fixtures/bundle_release`, a
net trained on simulation only), followed by `refine_track`, run through `TrackPerceiver` exactly
as the node runs a bundle with `io.arch.type: circ_conv1d_v0_refine_v1`. It is a NEW golden
(brief 32): it pins the refinement on real scans, and its tolerance is 0.02 m like the goldens of
the unrefined net and of the geometric tracer, which this script does not touch. Run it only when
the refinement was changed on purpose, and say so in the commit.
"""

from __future__ import annotations

from pathlib import Path

import numpy as np
from make_real_scans_2026_11_13 import SCANS as SCANS_1113
from make_real_scans_2026_11_13 import dequantise
from racer_track_perception import core
from racer_track_perception.runtime import CircConv1dV0
from racer_track_perception.vehicle import LiveVehicle, load_live_vehicle
from regen_golden import SCANS as SCANS_1111

FIXTURES = Path(__file__).parent / "fixtures"
GOLDEN = FIXTURES / "real_scans_refined_golden.npz"
RELEASE_WEIGHTS = Path(__file__).parents[1] / "test" / "fixtures" / "bundle_release" / "weights.npz"
KEYS = ("x", "y", "hl", "hr", "sigma", "valid")


def perceiver(live: LiveVehicle) -> core.TrackPerceiver:
    return core.TrackPerceiver(
        core.BACKEND_NET,
        live,
        net=CircConv1dV0.from_npz(RELEASE_WEIGHTS),
        model_id="release-v0-refined-golden",
        refine_net=True,
    )


def run(live: LiveVehicle | None = None) -> dict[str, np.ndarray]:
    live = live or load_live_vehicle()
    p = perceiver(live)
    out: dict[str, np.ndarray] = {}
    for tag, path, quantised in (("1111", SCANS_1111, False), ("1113", SCANS_1113, True)):
        with np.load(path) as d:
            ranges = dequantise(d["ranges_q"]) if quantised else d["ranges"]
            meta = (
                float(d["angle_min"]),
                float(d["angle_increment"]),
                float(d["range_min"]),
                float(d["range_max"]),
            )
        cols: dict[str, list] = {k: [] for k in KEYS}
        for r in ranges:
            t = p.process(r, *meta)
            cols["x"].append(t.x_m)
            cols["y"].append(t.y_m)
            cols["hl"].append(t.half_width_left_m)
            cols["hr"].append(t.half_width_right_m)
            cols["sigma"].append(t.sigma_m)
            cols["valid"].append(t.valid_count)
        for k, v in cols.items():
            out[f"{tag}_{k}"] = np.array(v)
    return out


if __name__ == "__main__":
    result = run()
    np.savez_compressed(GOLDEN, **result)
    print(
        f"wrote {GOLDEN}: {len(result['1111_valid'])} + {len(result['1113_valid'])} scans, "
        f"mean valid {result['1111_valid'].mean():.2f} and {result['1113_valid'].mean():.2f}"
    )
