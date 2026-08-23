"""Builds and writes a `contract.yaml` manifest ros_ws/src/racer_policy's `load_contract` can
load (claude-docs/08-learning.md "Deployment contract").

Every field below is read off a real, already-constructed `ResidualRacerEnv` instance (its
`observation_fields`, `lidar_config`, `action_fields`, `residual_limits`,
`actuator_assumptions`, `vehicle_params`) rather than re-derived independently -- the
manifest this module writes is guaranteed to match what the env actually produced during
training, by construction. Torch-free: this module never imports torch or
stable_baselines3; `train.py` computes the policy checksum itself (it already needs torch to
export the model) and passes it in.
"""

from __future__ import annotations

from pathlib import Path
from typing import Any

import yaml

from racer_train.env import ResidualRacerEnv

CONTRACT_VERSION = "1.0.0"  # major must match racer_policy.contract.SUPPORTED_CONTRACT_MAJOR


def build_contract_manifest(
    *,
    env: ResidualRacerEnv,
    policy_filename: str,
    policy_checksum_sha256: str,
    normalization_mean: list[float],
    normalization_std: list[float],
    config_hash: str,
    git_sha: str,
) -> dict[str, Any]:
    """Assemble the contract.yaml dict. Does not write anything to disk -- see
    `write_contract_dir`."""
    lidar = env.lidar_config
    expected_dim = len(env.observation_fields) + lidar.beam_count
    if len(normalization_mean) != expected_dim or len(normalization_std) != expected_dim:
        raise ValueError(
            f"normalization mean/std must have length {expected_dim} "
            f"({len(env.observation_fields)} low-dim fields + {lidar.beam_count} LiDAR "
            f"beams), got mean={len(normalization_mean)}, std={len(normalization_std)}"
        )

    steering_fraction, speed_fraction = env.residual_limits

    return {
        "contract_version": CONTRACT_VERSION,
        "policy": {
            "filename": policy_filename,
            "checksum_sha256": policy_checksum_sha256,
        },
        "observation_schema": {
            "fields": [
                {"name": name, "dtype": dtype, "units": units}
                for name, dtype, units in env.observation_fields
            ],
            "lidar": {
                "beam_count": lidar.beam_count,
                "fov_rad": lidar.fov_rad,
                "downsample_factor": lidar.downsample_factor,
            },
        },
        "normalization": {
            "mean": [float(v) for v in normalization_mean],
            "std": [float(v) for v in normalization_std],
        },
        "action_space": {
            "fields": [
                {"name": name, "low": low, "high": high, "units": units}
                for name, low, high, units in env.action_fields
            ],
            "scaling": "linear",
            "residual_limits": {
                "steering_fraction": steering_fraction,
                "speed_fraction": speed_fraction,
            },
        },
        "actuator_assumptions": dict(env.actuator_assumptions),
        "vehicle_params": {
            "schema_version": env.vehicle_params.meta.schema_version,
            "sysid_session_id": env.vehicle_params.meta.sysid_session_id,
        },
        "training": {
            "config_hash": config_hash,
            "git_sha": git_sha,
        },
    }


def write_contract_dir(directory: Path, manifest: dict[str, Any]) -> Path:
    """Write `manifest` as `contract.yaml` into `directory` (created if needed). The named
    policy artifact must already exist in `directory` -- this function does not write it
    (train.py writes the TorchScript artifact itself, since only it has torch loaded)."""
    directory = Path(directory)
    directory.mkdir(parents=True, exist_ok=True)
    policy_path = directory / manifest["policy"]["filename"]
    if not policy_path.is_file():
        raise FileNotFoundError(
            f"write_contract_dir: manifest names policy artifact {policy_path}, "
            "but it does not exist -- write the policy artifact before the manifest."
        )
    manifest_path = directory / "contract.yaml"
    manifest_path.write_text(yaml.safe_dump(manifest, sort_keys=False), encoding="utf-8")
    return manifest_path
