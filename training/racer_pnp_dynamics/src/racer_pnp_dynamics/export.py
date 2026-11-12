"""Write and read a dynamics model bundle (claude-docs/13-plug-and-play.md section 4.4).

A bundle is a directory: `manifest.yaml`, `residual_mlp.f32` (all zeros when the residual is not
enabled) and `golden_rollouts.csv`. The manifest is validated against
config/pnp/bundle.schema.json BEFORE anything is written, and the writer refuses when a
vehicle_params field it needs is null. The golden rollouts are produced by model.py for the
exported parameters, which is what proves the C++ planner implements the model that was fitted.
"""

from __future__ import annotations

import csv
import hashlib
import json
from dataclasses import dataclass
from pathlib import Path
from typing import Any

import numpy as np
import yaml
from jsonschema import Draft202012Validator

from .model import (
    CONTROL_NAMES,
    DT_S,
    RESIDUAL_INPUTS,
    RESIDUAL_LAYERS,
    RESIDUAL_OUTPUTS,
    STATE_NAMES,
    Nominal,
    Residual,
    pack_weights,
    rollout,
    unpack_weights,
)
from .vehicle import Vehicle, repo_root

BUNDLE_SCHEMA_VERSION = "1.1.0"
KIND = "dynamics"
SCHEMA_RELATIVE = Path("config") / "pnp" / "bundle.schema.json"
RESIDUAL_FILE = "residual_mlp.f32"
GOLDEN_FILE = "golden_rollouts.csv"
MANIFEST_FILE = "manifest.yaml"
GOLDEN_HEADER = (
    "rollout",
    "step",
    "speed_cmd_mps",
    "steering_cmd_rad",
    "x_m",
    "y_m",
    "yaw_rad",
    "v_mps",
    "delta_rad",
)
GOLDEN_STEPS = 40
# 9 significant digits (the contract), so a re-simulation matches to this relative tolerance.
GOLDEN_RTOL = 1e-8
GOLDEN_ATOL = 1e-9


class BundleInvalid(ValueError):
    """The manifest does not validate against bundle.schema.json, so nothing was written."""


@dataclass(frozen=True)
class Bundle:
    """What read_bundle returns."""

    manifest: dict[str, Any]
    nominal: Nominal
    residual: Residual
    v_max_mps: float


def sha256_file(path: Path) -> str:
    return hashlib.sha256(path.read_bytes()).hexdigest()


def sha256_text(text: str) -> str:
    return hashlib.sha256(text.encode("utf-8")).hexdigest()


def config_sha256(config: dict[str, Any]) -> str:
    """Checksum of a JSON-able config, key order independent."""
    return sha256_text(json.dumps(config, sort_keys=True, default=float))


def load_schema(root: Path | None = None) -> dict[str, Any]:
    return json.loads(((root or repo_root()) / SCHEMA_RELATIVE).read_text(encoding="utf-8"))


def validate_manifest(manifest: dict[str, Any], schema: dict[str, Any] | None = None) -> None:
    """Raise BundleInvalid listing every schema violation."""
    validator = Draft202012Validator(schema or load_schema())
    errors = sorted(validator.iter_errors(manifest), key=lambda e: list(e.absolute_path))
    if errors:
        lines = [
            f"{'/'.join(str(p) for p in e.absolute_path) or '<root>'}: {e.message}" for e in errors
        ]
        raise BundleInvalid("manifest invalid: " + "; ".join(lines))


def golden_scenarios(v_max_mps: float, steer_limit_rad: float) -> list[dict[str, Any]]:
    """Twelve hand-designed rollouts: left, right, accelerating, braking, saturated both ways, an
    S-curve and a hard steering reversal (which engages the steering rate limit), then three that
    cross the speed dead zone: a command ramp 0 to 0.4 m/s and back, and a constant 0.1 m/s from
    rest and from 0.4 m/s rolling. Initial states and commands are short decimals so they survive
    the CSV exactly."""
    v_hi = round(0.9 * v_max_mps, 3)
    v_mid = round(0.5 * v_max_mps, 3)
    s_mid = round(0.6 * steer_limit_rad, 3)
    n = GOLDEN_STEPS
    half = n // 2

    def const(speed: float, steer: float) -> np.ndarray:
        return np.tile([speed, steer], (n, 1))

    s_curve = np.array([[v_mid, s_mid if (i // 10) % 2 == 0 else -s_mid] for i in range(n)])
    reversal = np.array(
        [[v_mid, 1.2 * steer_limit_rad if i < half else -1.2 * steer_limit_rad] for i in range(n)]
    )
    speed_steps = np.array(
        [[v_hi if i < half else 0.0, s_mid * (i / n)] for i in range(n)], dtype=float
    )
    speed_steps[:, 1] = np.round(speed_steps[:, 1], 4)
    ramp = np.array(
        [[round(0.4 * (i / half if i < half else (n - i) / half), 4), 0.0] for i in range(n)]
    )
    return [
        {"state0": [0.0, 0.0, 0.0, v_mid, 0.0], "controls": const(v_mid, s_mid)},
        {"state0": [0.0, 0.0, 0.0, v_mid, 0.0], "controls": const(v_mid, -s_mid)},
        {"state0": [0.0, 0.0, 0.0, 0.0, 0.0], "controls": const(v_hi, 0.0)},
        {"state0": [0.5, -0.25, 0.3, v_hi, 0.0], "controls": const(0.0, 0.0)},
        {"state0": [0.0, 0.0, 0.0, v_mid, 0.0], "controls": const(v_hi, 1.5 * steer_limit_rad)},
        {"state0": [0.0, 0.0, 0.0, v_mid, 0.0], "controls": const(v_hi, -1.5 * steer_limit_rad)},
        {"state0": [0.1, 0.2, -0.4, v_mid, 0.05], "controls": s_curve},
        {"state0": [0.0, 0.0, 0.0, v_mid, 0.1], "controls": reversal},
        {"state0": [0.0, 0.0, 0.2, v_hi, -0.1], "controls": speed_steps},
        {"state0": [0.0, 0.0, 0.0, 0.0, 0.0], "controls": ramp},
        {"state0": [0.0, 0.0, 0.0, 0.0, 0.0], "controls": const(0.1, 0.0)},
        {"state0": [0.0, 0.0, 0.0, 0.4, 0.0], "controls": const(0.1, 0.0)},
    ]


def golden_rows(
    nominal: Nominal,
    residual: Residual,
    vehicle: Vehicle,
    v_max_mps: float,
    scenarios: list[dict[str, Any]] | None = None,
) -> list[list[float]]:
    """CSV rows (rollout, step, controls, state). The last row of a rollout has controls 0, 0."""
    scenarios = scenarios or golden_scenarios(v_max_mps, vehicle.steer_max_rad)
    rows: list[list[float]] = []
    for k, sc in enumerate(scenarios):
        controls = np.asarray(sc["controls"], dtype=float)
        states = rollout(
            np.asarray(sc["state0"], dtype=float), controls, nominal, vehicle, residual
        )
        padded = np.vstack([controls, [0.0, 0.0]])
        for i in range(len(states)):
            rows.append([k, i, *padded[i], *states[i]])
    return rows


def read_golden(path: Path) -> dict[int, np.ndarray]:
    """rollout index -> array (steps, 7): speed_cmd, steering_cmd, x, y, yaw, v, delta."""
    with path.open(newline="", encoding="utf-8") as fh:
        reader = csv.reader(fh)
        header = tuple(next(reader))
        if header != GOLDEN_HEADER:
            raise ValueError(f"{path}: unexpected golden header {header}")
        out: dict[int, list[list[float]]] = {}
        for row in reader:
            out.setdefault(int(row[0]), []).append([float(v) for v in row[2:]])
    return {k: np.asarray(v) for k, v in out.items()}


def golden_error(
    golden: dict[int, np.ndarray], nominal: Nominal, residual: Residual, vehicle: Vehicle
) -> float:
    """Largest |model - csv| over every rollout, re-simulating from each rollout's row 0 state
    and its CSV controls, as a loader does."""
    worst = 0.0
    for data in golden.values():
        controls = data[:-1, 0:2]
        states = rollout(data[0, 2:7], controls, nominal, vehicle, residual)
        worst = max(worst, float(np.max(np.abs(states - data[:, 2:7]))))
    return worst


def golden_matches(
    golden: dict[int, np.ndarray], nominal: Nominal, residual: Residual, vehicle: Vehicle
) -> bool:
    for data in golden.values():
        states = rollout(data[0, 2:7], data[:-1, 0:2], nominal, vehicle, residual)
        if not np.allclose(states, data[:, 2:7], rtol=GOLDEN_RTOL, atol=GOLDEN_ATOL):
            return False
    return True


def build_manifest(
    *,
    model_id: str,
    created_utc: str,
    git_sha: str,
    training_config_sha256: str,
    vehicle: Vehicle,
    nominal: Nominal,
    residual: Residual,
    v_max_mps: float,
    payload: list[dict[str, str]],
    data: list[str],
    metrics: dict[str, Any],
) -> dict[str, Any]:
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
            "residual": {
                "enabled": bool(residual.enabled),
                "inputs": list(RESIDUAL_INPUTS),
                "outputs": list(RESIDUAL_OUTPUTS),
                "layers": [list(layer) for layer in RESIDUAL_LAYERS],
                "activation": "tanh",
                "input_mean": [float(v) for v in residual.input_mean],
                "input_std": [float(v) for v in residual.input_std],
                "output_scale": [float(v) for v in residual.output_scale],
                "file": RESIDUAL_FILE,
            },
            "validity": {"v_min_mps": 0.0, "v_max_mps": float(v_max_mps)},
            "golden": GOLDEN_FILE,
        },
        "data": data,
        "metrics": metrics,
    }


def write_bundle(
    out_dir: Path,
    *,
    model_id: str,
    created_utc: str,
    git_sha: str,
    training_config_sha256: str,
    vehicle: Vehicle,
    nominal: Nominal,
    residual: Residual,
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
    weights_flat = (
        pack_weights(residual.weights)
        if residual.enabled
        else np.zeros(pack_weights(residual.weights).size, dtype=np.float32)
    )
    weights_bytes = weights_flat.astype("<f4").tobytes()
    rows = golden_rows(nominal, residual, vehicle, v_max_mps)
    payload_files = {RESIDUAL_FILE: weights_bytes}
    # Build the golden text in memory so the manifest can be validated before any write.
    payload_files[GOLDEN_FILE] = _golden_text(rows).encode("utf-8")
    payload = [
        {"file": name, "sha256": hashlib.sha256(blob).hexdigest()}
        for name, blob in payload_files.items()
    ]
    manifest = build_manifest(
        model_id=model_id,
        created_utc=created_utc,
        git_sha=git_sha,
        training_config_sha256=training_config_sha256,
        vehicle=vehicle,
        nominal=nominal,
        residual=residual,
        v_max_mps=v_max_mps,
        payload=payload,
        data=data,
        metrics=metrics,
    )
    validate_manifest(manifest, schema)
    out_dir.mkdir(parents=True, exist_ok=True)
    for name, blob in payload_files.items():
        (out_dir / name).write_bytes(blob)
    (out_dir / MANIFEST_FILE).write_text(
        yaml.safe_dump(manifest, sort_keys=False, default_flow_style=None), encoding="utf-8"
    )
    return manifest


def _golden_text(rows: list[list[float]]) -> str:
    lines = [",".join(GOLDEN_HEADER)]
    for row in rows:
        lines.append(",".join([str(int(row[0])), str(int(row[1])), *(f"{v:.9g}" for v in row[2:])]))
    return "\n".join(lines) + "\n"


def read_bundle(bundle_dir: Path) -> Bundle:
    """Load a bundle written by write_bundle: validates the manifest against the schema (raises
    BundleInvalid) and checks checksums, not the vehicle_params."""
    manifest = yaml.safe_load((bundle_dir / MANIFEST_FILE).read_text(encoding="utf-8"))
    validate_manifest(manifest)  # a bundle without io.nominal.speed_deadband_mps is refused
    for item in manifest["payload"]:
        if sha256_file(bundle_dir / item["file"]) != item["sha256"]:
            raise ValueError(f"checksum mismatch for {item['file']}")
    io = manifest["io"]
    res = io["residual"]
    flat = np.frombuffer((bundle_dir / res["file"]).read_bytes(), dtype="<f4")
    residual = Residual(
        enabled=bool(res["enabled"]),
        weights=unpack_weights(flat, tuple(tuple(layer) for layer in res["layers"])),
        input_mean=np.asarray(res["input_mean"], dtype=float),
        input_std=np.asarray(res["input_std"], dtype=float),
        output_scale=np.asarray(res["output_scale"], dtype=float),
    )
    return Bundle(
        manifest=manifest,
        nominal=Nominal.from_dict(io["nominal"]),
        residual=residual,
        v_max_mps=float(io["validity"]["v_max_mps"]),
    )
