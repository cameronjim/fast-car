"""Write and read a contract 2 dynamics bundle (bundle schema 2.0.0, addendum 1 section 2.2).

Same directory layout as `export.py` (the contract 1 writer is untouched and stays bit-identical):
`manifest.yaml`, `ground_mlp.f32` (all zeros when the learned block is off) and
`golden_rollouts.csv`. `io.ground_motion` holds the table (nodes, values, bounds), the learned block
with its normalisation and the two bounds `d_alpha_max` and `rho_max`; there is no `residual` block
and the schema forbids one. The golden rollouts come from `ground_motion.step_v2` with the weights
as they are stored (float32), the same twelve scenarios as contract 1 plus the five at speed.

A planner that does not implement contract 2 refuses this bundle on its schema major version
(`BundleSchemaVersionMismatch`); no planner on main implements it, so every pointer README of a
contract 2 model starts with `FIRST_README_LINE`.
"""

from __future__ import annotations

import hashlib
from dataclasses import dataclass
from pathlib import Path
from typing import Any

import numpy as np
import yaml

from .export import (
    GOLDEN_ATOL,
    GOLDEN_HEADER,
    GOLDEN_RTOL,
    MANIFEST_FILE,
    BundleInvalid,
    _golden_text,
    golden_scenarios,
    read_golden,
    sha256_file,
    validate_manifest,
)
from .ground_motion import (
    BUNDLE_SCHEMA_VERSION,
    GroundConstants,
    GroundMotion,
    GroundTable,
    Learned,
    rollout_v2,
)
from .model import (
    CONTROL_NAMES,
    DT_S,
    RESIDUAL_INPUTS,
    RESIDUAL_LAYERS,
    STATE_NAMES,
    Nominal,
    pack_weights,
    unpack_weights,
    zero_weights,
)
from .vehicle import Vehicle

KIND = "dynamics"
GROUND_FILE = "ground_mlp.f32"
GOLDEN_FILE = "golden_rollouts.csv"
OUTPUTS = ("d_alpha", "rho")
FIRST_README_LINE = "contract 2: no planner on main can load this; candidate only"
INTERPOLATION = "bilinear over steering as a share of each side's limit and wheel speed, held constant outside the grid"
__all__ = ["BUNDLE_SCHEMA_VERSION", "GOLDEN_HEADER"]


@dataclass(frozen=True)
class BundleV2:
    manifest: dict[str, Any]
    nominal: Nominal
    ground: GroundMotion
    v_max_mps: float


def _stored(learned: Learned | None) -> tuple[Learned | None, bytes]:
    """The learned block as a loader will read it (float32 weights) and the weights file."""
    if learned is None:
        return None, np.zeros(pack_weights(zero_weights()).size, dtype="<f4").tobytes()
    flat = pack_weights(learned.weights)
    return (
        Learned(unpack_weights(flat), learned.input_mean, learned.input_std),
        flat.astype("<f4").tobytes(),
    )


def golden_rows_v2(
    nominal: Nominal,
    vehicle: Vehicle,
    ground: GroundMotion,
    v_max_mps: float,
    scenarios: list[dict[str, Any]] | None = None,
) -> list[list[float]]:
    scenarios = scenarios or golden_scenarios(v_max_mps, vehicle.steer_max_rad, True)
    rows: list[list[float]] = []
    for k, sc in enumerate(scenarios):
        controls = np.asarray(sc["controls"], dtype=float)
        states = rollout_v2(
            np.asarray(sc["state0"], dtype=float), controls, nominal, vehicle, ground
        )
        padded = np.vstack([controls, [0.0, 0.0]])
        for i in range(len(states)):
            rows.append([k, i, *padded[i], *states[i]])
    return rows


def golden_matches_v2(
    golden: dict[int, np.ndarray], nominal: Nominal, vehicle: Vehicle, ground: GroundMotion
) -> bool:
    for data in golden.values():
        states = rollout_v2(data[0, 2:7], data[:-1, 0:2], nominal, vehicle, ground)
        if not np.allclose(states, data[:, 2:7], rtol=GOLDEN_RTOL, atol=GOLDEN_ATOL):
            return False
    return True


def build_manifest_v2(
    *,
    model_id: str,
    created_utc: str,
    git_sha: str,
    training_config_sha256: str,
    vehicle: Vehicle,
    nominal: Nominal,
    ground: GroundMotion,
    v_max_mps: float,
    payload: list[dict[str, str]],
    data: list[str],
    metrics: dict[str, Any],
) -> dict[str, Any]:
    c, t, ln = ground.constants, ground.table, ground.learned
    return {
        "bundle_schema_version": BUNDLE_SCHEMA_VERSION,
        "kind": KIND,
        "model_id": model_id,
        "created_utc": created_utc,
        "git_sha": git_sha,
        "training_config_sha256": training_config_sha256,
        "vehicle_params": {
            "schema_version": vehicle.schema_version,
            "sysid_session_id": vehicle.sysid_session_id,
        },
        "payload": payload,
        "io": {
            "state": list(STATE_NAMES),
            "control": list(CONTROL_NAMES),
            "dt_s": DT_S,
            "nominal": nominal.to_dict(),
            "ground_motion": {
                "alpha_bounds": [c.alpha_min, c.alpha_max],
                "table": {
                    "steering_nodes_share_of_side_limit": [float(x) for x in t.steering_nodes],
                    "wheel_speed_nodes_mps": [float(x) for x in t.speed_nodes_mps],
                    "interpolation": INTERPOLATION,
                    "values": [[float(x) for x in row] for row in t.values],
                },
                "learned": {
                    "enabled": ln is not None,
                    "inputs": list(RESIDUAL_INPUTS),
                    "outputs": list(OUTPUTS),
                    "layers": [list(layer) for layer in RESIDUAL_LAYERS],
                    "activation": "tanh",
                    "input_mean": [float(v) for v in (ln.input_mean if ln else np.zeros(4))],
                    "input_std": [float(v) for v in (ln.input_std if ln else np.ones(4))],
                    "d_alpha_max": c.d_alpha_max,
                    "rho_max": c.rho_max,
                    "file": GROUND_FILE,
                },
            },
            "validity": {"v_min_mps": 0.0, "v_max_mps": float(v_max_mps)},
            "golden": GOLDEN_FILE,
        },
        "data": data,
        "metrics": metrics,
    }


def write_bundle_v2(
    out_dir: Path,
    *,
    model_id: str,
    created_utc: str,
    git_sha: str,
    training_config_sha256: str,
    vehicle: Vehicle,
    nominal: Nominal,
    ground: GroundMotion,
    v_max_mps: float,
    data: list[str],
    metrics: dict[str, Any],
    schema: dict[str, Any] | None = None,
) -> dict[str, Any]:
    """Write the bundle directory and return the manifest. Nothing is written on a refusal."""
    if not 0.0 < v_max_mps <= vehicle.throttle_full_scale_mps:
        raise BundleInvalid(
            f"v_max_mps {v_max_mps} must be in (0, {vehicle.throttle_full_scale_mps}] "
            "(actuation.throttle_full_scale_mps)"
        )
    stored_learned, weights_bytes = _stored(ground.learned)
    stored = GroundMotion(ground.constants, ground.table, stored_learned)
    rows = golden_rows_v2(nominal, vehicle, stored, v_max_mps)
    files = {GROUND_FILE: weights_bytes, GOLDEN_FILE: _golden_text(rows).encode("utf-8")}
    payload = [{"file": n, "sha256": hashlib.sha256(b).hexdigest()} for n, b in files.items()]
    manifest = build_manifest_v2(
        model_id=model_id,
        created_utc=created_utc,
        git_sha=git_sha,
        training_config_sha256=training_config_sha256,
        vehicle=vehicle,
        nominal=nominal,
        ground=ground,
        v_max_mps=v_max_mps,
        payload=payload,
        data=data,
        metrics=metrics,
    )
    validate_manifest(manifest, schema)
    out_dir.mkdir(parents=True, exist_ok=True)
    for name, blob in files.items():
        (out_dir / name).write_bytes(blob)
    (out_dir / MANIFEST_FILE).write_text(
        yaml.safe_dump(manifest, sort_keys=False, default_flow_style=None), encoding="utf-8"
    )
    return manifest


def read_bundle_v2(bundle_dir: Path, constants: GroundConstants) -> BundleV2:
    """Load a contract 2 bundle: schema validation, checksums, golden rollouts re-simulated."""
    manifest = yaml.safe_load((bundle_dir / MANIFEST_FILE).read_text(encoding="utf-8"))
    validate_manifest(manifest)
    if not str(manifest["bundle_schema_version"]).startswith("2."):
        raise BundleInvalid("not a contract 2 bundle")
    for item in manifest["payload"]:
        if sha256_file(bundle_dir / item["file"]) != item["sha256"]:
            raise ValueError(f"checksum mismatch for {item['file']}")
    gm = manifest["io"]["ground_motion"]
    t = gm["table"]
    table = GroundTable(
        np.asarray(t["steering_nodes_share_of_side_limit"], dtype=float),
        np.asarray(t["wheel_speed_nodes_mps"], dtype=float),
        np.asarray(t["values"], dtype=float),
    )
    ln = gm["learned"]
    learned = None
    if ln["enabled"]:
        flat = np.frombuffer((bundle_dir / ln["file"]).read_bytes(), dtype="<f4")
        learned = Learned(
            unpack_weights(flat, tuple(tuple(x) for x in ln["layers"])),
            np.asarray(ln["input_mean"], dtype=float),
            np.asarray(ln["input_std"], dtype=float),
        )
    c = GroundConstants(
        **{
            **constants.__dict__,
            "alpha_min": float(gm["alpha_bounds"][0]),
            "alpha_max": float(gm["alpha_bounds"][1]),
            "d_alpha_max": float(ln["d_alpha_max"]),
            "rho_max": float(ln["rho_max"]),
        }
    )
    return BundleV2(
        manifest,
        Nominal.from_dict(manifest["io"]["nominal"]),
        GroundMotion(c, table, learned),
        float(manifest["io"]["validity"]["v_max_mps"]),
    )


def read_golden_v2(bundle_dir: Path) -> dict[int, np.ndarray]:
    return read_golden(bundle_dir / GOLDEN_FILE)
