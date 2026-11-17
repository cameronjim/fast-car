"""Write and read a contract 2.1 dynamics bundle (bundle schema 2.1.0, pre-registration v0.5 section 2).

Same directory layout as `export_v2.py` (untouched, as is `export.py`): `manifest.yaml`,
`ground_mlp.f32` (all zeros when the learned block is off) and `golden_rollouts.csv`.
`io.ground_motion` holds the alpha table (nodes, values, support), the yaw gain table `yaw_table`
(the same axes, values, bounds, support), the learned block with its normalisation and the two
bounds `d_alpha_max` and `rho_max`; there is no `residual` block and the schema forbids one. The
golden rollouts come from `ground_motion_v21.step_v21` with the weights as they are stored
(float32), the same scenarios as contract 2 (twelve and the five at speed).

A planner that does not implement contract 2.1 refuses this bundle on its schema major version
(`BundleSchemaVersionMismatch`, the loader on main loads major 1 only), so every pointer README of a
contract 2.1 model starts with `FIRST_README_LINE`, and every v0.5 bundle is `do-not-deploy` until
the owner approves schema 2.1.0.
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
    GOLDEN_RTOL,
    MANIFEST_FILE,
    BundleInvalid,
    _golden_text,
    golden_scenarios,
    read_golden,
    sha256_file,
    validate_manifest,
)
from .export_v2 import GOLDEN_FILE, GROUND_FILE, INTERPOLATION, OUTPUTS, _stored
from .ground_motion import GroundTable, Learned
from .ground_motion_v21 import (
    BUNDLE_SCHEMA_VERSION_V21,
    ConstantsV05,
    GroundMotionV21,
    check_yaw_table,
    rollout_v21,
)
from .model import (
    CONTROL_NAMES,
    DT_S,
    RESIDUAL_INPUTS,
    RESIDUAL_LAYERS,
    STATE_NAMES,
    Nominal,
    unpack_weights,
)
from .vehicle import Vehicle

KIND = "dynamics"
FIRST_README_LINE = "contract 2.1: no planner on main can load this; candidate only"
DO_NOT_DEPLOY = "do-not-deploy"


@dataclass(frozen=True)
class BundleV21:
    manifest: dict[str, Any]
    nominal: Nominal
    ground: GroundMotionV21
    v_max_mps: float


def readme_first_line() -> str:
    """Every v0.5 bundle is `do-not-deploy` (schema 2.1.0 is the owner's decision), and no planner
    on main can load it."""
    return f"{DO_NOT_DEPLOY}; {FIRST_README_LINE}"


def golden_rows_v21(
    nominal: Nominal,
    vehicle: Vehicle,
    ground: GroundMotionV21,
    v_max_mps: float,
    scenarios: list[dict[str, Any]] | None = None,
) -> list[list[float]]:
    scenarios = scenarios or golden_scenarios(v_max_mps, vehicle.steer_max_rad, True)
    rows: list[list[float]] = []
    for k, sc in enumerate(scenarios):
        controls = np.asarray(sc["controls"], dtype=float)
        states = rollout_v21(
            np.asarray(sc["state0"], dtype=float), controls, nominal, vehicle, ground
        )
        padded = np.vstack([controls, [0.0, 0.0]])
        for i in range(len(states)):
            rows.append([k, i, *padded[i], *states[i]])
    return rows


def golden_matches_v21(
    golden: dict[int, np.ndarray], nominal: Nominal, vehicle: Vehicle, ground: GroundMotionV21
) -> bool:
    for data in golden.values():
        states = rollout_v21(data[0, 2:7], data[:-1, 0:2], nominal, vehicle, ground)
        if not np.allclose(states, data[:, 2:7], rtol=GOLDEN_RTOL, atol=GOLDEN_ATOL):
            return False
    return True


def _support_of(ground: GroundMotionV21) -> list[list[bool]]:
    shape = ground.table.values.shape
    sup = np.ones(shape, dtype=bool) if ground.support is None else np.asarray(ground.support, bool)
    if sup.shape != shape:
        raise BundleInvalid("the support record does not have the grid of the tables")
    return [[bool(x) for x in row] for row in sup]


def build_manifest_v21(
    *,
    model_id: str,
    created_utc: str,
    git_sha: str,
    training_config_sha256: str,
    vehicle: Vehicle,
    nominal: Nominal,
    ground: GroundMotionV21,
    v_max_mps: float,
    payload: list[dict[str, str]],
    data: list[str],
    metrics: dict[str, Any],
) -> dict[str, Any]:
    c, a, y, ln = ground.constants, ground.table, ground.yaw_table, ground.learned
    support = _support_of(ground)
    nodes = {
        "steering_nodes_share_of_side_limit": [float(x) for x in a.steering_nodes],
        "wheel_speed_nodes_mps": [float(x) for x in a.speed_nodes_mps],
        "interpolation": INTERPOLATION,
    }
    return {
        "bundle_schema_version": BUNDLE_SCHEMA_VERSION_V21,
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
                    **nodes,
                    "values": [[float(x) for x in row] for row in a.values],
                    "support": support,
                },
                "yaw_table": {
                    **nodes,
                    "values": [[float(x) for x in row] for row in y.values],
                    "bounds": [c.gamma_min, c.gamma_max],
                    "support": support,
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


def write_bundle_v21(
    out_dir: Path,
    *,
    model_id: str,
    created_utc: str,
    git_sha: str,
    training_config_sha256: str,
    vehicle: Vehicle,
    nominal: Nominal,
    ground: GroundMotionV21,
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
    try:
        check_yaw_table(ground.yaw_table, ground.constants)
    except ValueError as error:
        raise BundleInvalid(str(error)) from error
    stored_learned, weights_bytes = _stored(ground.learned)
    stored = GroundMotionV21(
        ground.constants, ground.table, ground.yaw_table, stored_learned, ground.support
    )
    rows = golden_rows_v21(nominal, vehicle, stored, v_max_mps)
    files = {GROUND_FILE: weights_bytes, GOLDEN_FILE: _golden_text(rows).encode("utf-8")}
    payload = [{"file": n, "sha256": hashlib.sha256(b).hexdigest()} for n, b in files.items()]
    manifest = build_manifest_v21(
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


def read_bundle_v21(bundle_dir: Path, constants: ConstantsV05) -> BundleV21:
    """Load a contract 2.1 bundle: schema validation, checksums, the registered contract numbers
    (a bundle whose bounds differ from `constants` is refused)."""
    manifest = yaml.safe_load((bundle_dir / MANIFEST_FILE).read_text(encoding="utf-8"))
    validate_manifest(manifest)
    if not str(manifest["bundle_schema_version"]).startswith("2.1."):
        raise BundleInvalid("not a contract 2.1 bundle")
    for item in manifest["payload"]:
        if sha256_file(bundle_dir / item["file"]) != item["sha256"]:
            raise ValueError(f"checksum mismatch for {item['file']}")
    gm = manifest["io"]["ground_motion"]
    if [float(x) for x in gm["alpha_bounds"]] != [constants.alpha_min, constants.alpha_max] or [
        float(x) for x in gm["yaw_table"]["bounds"]
    ] != [constants.gamma_min, constants.gamma_max]:
        raise BundleInvalid("the bounds of the bundle are not the registered ones")

    def table_of(block: dict) -> GroundTable:
        return GroundTable(
            np.asarray(block["steering_nodes_share_of_side_limit"], dtype=float),
            np.asarray(block["wheel_speed_nodes_mps"], dtype=float),
            np.asarray(block["values"], dtype=float),
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
    ground = GroundMotionV21(
        constants,
        table_of(gm["table"]),
        table_of(gm["yaw_table"]),
        learned,
        np.asarray(gm["table"]["support"], dtype=bool),
    )
    check_yaw_table(ground.yaw_table, constants)
    return BundleV21(
        manifest,
        Nominal.from_dict(manifest["io"]["nominal"]),
        ground,
        float(manifest["io"]["validity"]["v_max_mps"]),
    )


def read_golden_v21(bundle_dir: Path) -> dict[int, np.ndarray]:
    return read_golden(bundle_dir / GOLDEN_FILE)
