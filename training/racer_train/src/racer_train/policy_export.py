"""Turn a trained SB3 SAC actor into a deployable policy directory.

The directory is exactly what `racer_policy.load_contract` accepts:

    policy.pt          TorchScript: RAW observation (N x D float32) -> raw action in [-1, 1]^2
    contract.yaml      the deployment contract (schema, limits, provenance, checksum)
    policy_meta.yaml   everything the contract schema has no room for: the v0 label, the
                       normalisation statistics that are baked into policy.pt, the envelope and
                       base-controller settings a deployment must match, randomisation ranges

`contract.yaml` is validated by `additionalProperties: false` at every level, so extra
metadata CANNOT go there (and the schema is deliberately not loosened); `policy_meta.yaml` is
the sidecar. The loader does not read it, so a mismatch in it is NOT refused automatically;
the deploy node (roadmap 5.2) must check the fields marked `must_match_at_deploy`.

The exported module normalises internally (see `racer_train.normalization`), so the contract's
`normalization` block is the identity on purpose.
"""

from __future__ import annotations

import hashlib
import subprocess
from datetime import UTC, datetime
from pathlib import Path
from typing import Any

import yaml

from racer_train.config import ExperimentConfig, config_sha256
from racer_train.contract_export import build_contract_manifest, write_contract_dir
from racer_train.env import ResidualRacerEnv
from racer_train.normalization import CLIP_SIGMA, ObsNormalization

POLICY_FILENAME = "policy.pt"
META_FILENAME = "policy_meta.yaml"
LABEL = (
    "v0, PRE-SYSID. Trained in a simulator that has not been identified against the car "
    "(roadmap Phase 3 not run). This is NOT the thesis result and makes no claim about the "
    "real car; roadmap 5.1 retrains in the calibrated sim and supersedes it."
)


def git_sha(repo_root: Path) -> str:
    result = subprocess.run(
        ["git", "rev-parse", "HEAD"], cwd=repo_root, capture_output=True, text=True, check=True
    )
    return result.stdout.strip()


def git_is_dirty(repo_root: Path) -> bool:
    result = subprocess.run(
        ["git", "status", "--porcelain", "--untracked-files=no"],
        cwd=repo_root,
        capture_output=True,
        text=True,
        check=True,
    )
    return bool(result.stdout.strip())


def build_policy_module(actor: Any, norm: ObsNormalization) -> Any:
    """TorchScript-able deterministic policy: raw obs -> tanh(mu(latent_pi(normalize(obs))))."""
    import torch

    class DeterministicPolicy(torch.nn.Module):
        def __init__(self) -> None:
            super().__init__()
            self.latent_pi = actor.latent_pi
            self.mu = actor.mu
            self.register_buffer("mean", torch.tensor(norm.mean, dtype=torch.float32))
            self.register_buffer("std", torch.tensor(norm.std, dtype=torch.float32))
            self.low_dim_count = norm.low_dim_count
            self.lidar_clip_m = float(norm.lidar_clip_m)

        def forward(self, obs: torch.Tensor) -> torch.Tensor:
            low = obs[:, : self.low_dim_count]
            lidar = torch.clamp(obs[:, self.low_dim_count :], max=self.lidar_clip_m)
            x = torch.cat([low, lidar], dim=1)
            x = torch.clamp((x - self.mean) / self.std, -CLIP_SIGMA, CLIP_SIGMA)
            return torch.tanh(self.mu(self.latent_pi(x)))

    return DeterministicPolicy().eval()


def export_policy_dir(
    *,
    out_dir: Path,
    sac_model: Any,
    env: ResidualRacerEnv,
    config: ExperimentConfig,
    config_path: Path,
    norm: ObsNormalization,
    repo_root: Path,
    meta_extra: dict[str, Any] | None = None,
) -> Path:
    """Write policy.pt + contract.yaml + policy_meta.yaml into `out_dir` and return it."""
    import torch

    out_dir = Path(out_dir)
    out_dir.mkdir(parents=True, exist_ok=True)

    module = build_policy_module(sac_model.policy.actor.cpu(), norm)
    obs_dim = env.observation_space.shape[0]
    example = torch.zeros((1, obs_dim), dtype=torch.float32)
    with torch.no_grad():
        traced = torch.jit.trace(module, example)
    policy_path = out_dir / POLICY_FILENAME
    traced.save(str(policy_path))
    checksum = hashlib.sha256(policy_path.read_bytes()).hexdigest()

    manifest = build_contract_manifest(
        env=env,
        policy_filename=POLICY_FILENAME,
        policy_checksum_sha256=checksum,
        normalization_mean=[0.0] * obs_dim,
        normalization_std=[1.0] * obs_dim,
        config_hash=config_sha256(config_path),
        git_sha=git_sha(repo_root),
    )
    write_contract_dir(out_dir, manifest)

    env_cfg = config.envelope
    meta: dict[str, Any] = {
        "label": LABEL,
        "created": datetime.now(UTC).date().isoformat(),
        "policy_version": "v0",
        "seed": config.seed,
        "config_name": config.name,
        "config_sha256": config_sha256(config_path),
        "git_sha": git_sha(repo_root),
        "git_tree_dirty_at_export": git_is_dirty(repo_root),
        "observation_normalization": {
            "applied_inside_policy_pt": True,
            "contract_normalization_block": "identity by design; do not apply any other normalization",
            "clip_sigma": CLIP_SIGMA,
            "lidar_clip_m": norm.lidar_clip_m,
            "low_dim_count": norm.low_dim_count,
            "mean": list(norm.mean),
            "std": list(norm.std),
        },
        "must_match_at_deploy": {
            "envelope": {
                "residual_fraction_steering": env_cfg.residual_fraction_steering,
                "residual_fraction_speed": env_cfg.residual_fraction_speed,
                "speed_cap_mps": env_cfg.speed_cap_mps,
                "ood_threshold": env_cfg.ood_threshold,
                "ood_reference_state": list(env_cfg.ood_reference_state),
                "ood_reference_scale": list(env_cfg.ood_reference_scale or ()),
                "control_rate_hz": 1.0 / config.env.timestep_s,
            },
            "action_scale": {
                "steering_rad_per_unit": config.env.action.steering_scale_rad,
                "speed_mps_per_unit": config.env.action.speed_scale_mps,
            },
            "base_controller": {
                "lookahead_min_m": config.pure_pursuit.lookahead_min_m,
                "lookahead_max_m": config.pure_pursuit.lookahead_max_m,
                "lookahead_curvature_ref_1pm": config.pure_pursuit.lookahead_curvature_ref_1pm,
                "raceline_a_lat_max_mps2": (
                    config.env.tracks.family.a_lat_max_mps2 if config.env.tracks else None
                ),
                "raceline_v_max_mps": (
                    config.env.tracks.family.v_max_mps if config.env.tracks else None
                ),
            },
            "observation": {
                "profile": config.env.observation_profile,
                "lidar_beam_count": env.lidar_config.beam_count,
                "lidar_fov_rad": env.lidar_config.fov_rad,
                "lidar_downsample_factor": env.lidar_config.downsample_factor,
                "lidar_min_pool": True,
                "lidar_rate_hz_trained_against": [50.0 / 6.0, 50.0 / 4.0],
            },
        },
        "vehicle_overrides_assumed": config.vehicle_overrides,
        "randomization": _randomization_dict(config),
    }
    if meta_extra:
        meta.update(meta_extra)
    (out_dir / META_FILENAME).write_text(yaml.safe_dump(meta, sort_keys=False), encoding="utf-8")
    return out_dir


def _randomization_dict(config: ExperimentConfig) -> dict[str, Any]:
    import dataclasses

    raw = dataclasses.asdict(config.randomization)
    return {k: (list(v) if isinstance(v, tuple) else v) for k, v in raw.items()}
