#!/usr/bin/env python3
"""Derives the contract 2 bundle for the sim plant from the fitted car bundle (brief 43).

The sim plant moves the car by its wheel speed: ground distance over wheel distance is exactly 1.
A step contract 2 bundle whose table is 1 everywhere and whose learned block is off therefore
IS the contract 1 model (the Python reference says so bit for bit: alpha = 1, rho = 0), so it
reuses the 1.x bundle's nominal, validity and golden rollouts unchanged, and the planner running
it must lap exactly as the planner running the 1.x bundle does. This is the L5 canary's contract
2 input, next to the 1.x run; it is a test bundle, not a model of the car, and it is never
deployed.

    derive_contract2_sim.py [--src DIR] [--out DIR]
"""

from __future__ import annotations

import argparse
import hashlib
from pathlib import Path

import yaml

HERE = Path(__file__).resolve().parent
SRC = HERE / "fixtures" / "dyn-car-2026-11-12"
OUT = HERE / "fixtures" / "dyn-car-2026-11-12-contract2-sim"
# The grid of addendum 1 (evaluation/prereg/pnp-v0.4-addendum-1.yaml, block ground_motion).
STEERING_NODES = [-1.0, -0.6667, -0.3333, 0.0, 0.3333, 0.6667, 1.0]
SPEED_NODES = [0.25, 0.6, 0.85, 1.25, 1.5]
INTERPOLATION = (
    "bilinear over steering as a share of each side's limit and wheel speed, "
    "held constant outside the grid"
)
N_WEIGHTS = 4 * 32 + 32 + 32 * 32 + 32 + 2 * 32 + 2


def sha256(path: Path) -> str:
    return hashlib.sha256(path.read_bytes()).hexdigest()


def derive(src: Path, out: Path) -> None:
    manifest = yaml.safe_load((src / "manifest.yaml").read_text(encoding="utf-8"))
    assert manifest["bundle_schema_version"].startswith("1."), "the source must be contract 1"
    assert not manifest["io"]["residual"]["enabled"], "the source must have no residual"
    out.mkdir(parents=True, exist_ok=True)
    (out / "golden_rollouts.csv").write_bytes((src / "golden_rollouts.csv").read_bytes())
    (out / "ground_mlp.f32").write_bytes(b"\x00" * (4 * N_WEIGHTS))
    io = manifest["io"]
    new_io = {
        "state": io["state"],
        "control": io["control"],
        "dt_s": io["dt_s"],
        "nominal": io["nominal"],
        "ground_motion": {
            "alpha_bounds": [0.6, 1.05],
            "table": {
                "steering_nodes_share_of_side_limit": STEERING_NODES,
                "wheel_speed_nodes_mps": SPEED_NODES,
                "interpolation": INTERPOLATION,
                "values": [[1.0] * len(SPEED_NODES) for _ in STEERING_NODES],
            },
            "learned": {
                "enabled": False,
                "inputs": ["v_mps", "delta_rad", "speed_cmd_mps", "steering_cmd_rad"],
                "outputs": ["d_alpha", "rho"],
                "layers": [[4, 32], [32, 32], [32, 2]],
                "activation": "tanh",
                "input_mean": [0.0, 0.0, 0.0, 0.0],
                "input_std": [1.0, 1.0, 1.0, 1.0],
                "d_alpha_max": 0.1,
                "rho_max": 0.25,
                "file": "ground_mlp.f32",
            },
        },
        "validity": io["validity"],
        "golden": io["golden"],
    }
    derived = {
        "bundle_schema_version": "2.0.0",
        "kind": "dynamics",
        "model_id": "dyn-car-2026-11-12-contract2-sim",
        "created_utc": manifest["created_utc"],
        "git_sha": manifest["git_sha"],
        "training_config_sha256": manifest["training_config_sha256"],
        "vehicle_params": manifest["vehicle_params"],
        "payload": [
            {"file": "ground_mlp.f32", "sha256": sha256(out / "ground_mlp.f32")},
            {"file": "golden_rollouts.csv", "sha256": sha256(out / "golden_rollouts.csv")},
        ],
        "io": new_io,
        "data": [
            (
                "step contract 2 form of dyn-car-2026-11-12 for the sim plant: ground table 1, "
                "learned block off, so the model is the contract 1 one; test bundle, never deployed"
            )
        ],
        "metrics": {},
    }
    (out / "manifest.yaml").write_text(
        yaml.safe_dump(derived, sort_keys=False, default_flow_style=None), encoding="utf-8"
    )


def main() -> None:
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawTextHelpFormatter)
    ap.add_argument("--src", type=Path, default=SRC)
    ap.add_argument("--out", type=Path, default=OUT)
    args = ap.parse_args()
    derive(args.src, args.out)


if __name__ == "__main__":
    main()
