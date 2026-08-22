"""Experiment config: everything `train.py` needs, loaded from one committed YAML file.

claude-docs/02-repo-layout.md naming rule: "Experiment configs: configs/<phase>/
<name>_<semver>.yaml; never edit a config that has produced a committed result -- copy and
bump." claude-docs/10-conventions.md: "Experiments are reproducible from: config file + git
SHA + seed + vehicle_params version" -- this module's `config_sha256` is the hash recorded
in the deployment contract's `training.config_hash` (ros_ws/src/racer_policy/
contract.schema.json), so a contract can always be traced back to the exact config bytes
that produced it.

Only pure config (torch-free, gym-free): no environment or model objects are constructed
here.
"""

from __future__ import annotations

import hashlib
from dataclasses import dataclass, field
from pathlib import Path
from typing import Any

import yaml

from racer_train.envelope_config import EnvelopeSettings
from racer_train.reward import RewardWeights


@dataclass(frozen=True)
class PurePursuitSettings:
    """Curvature-adaptive pure-pursuit lookahead tuning gains -- NOT physical constants
    (mirrors tracker_node.cpp's declared ROS param defaults: lookahead_min_m=0.4,
    lookahead_max_m=1.5, lookahead_curvature_ref_1pm=0.4)."""

    lookahead_min_m: float = 0.4
    lookahead_max_m: float = 1.5
    lookahead_curvature_ref_1pm: float = 0.4


@dataclass(frozen=True)
class ActionSettings:
    """Bounds of the RAW residual the policy's network outputs (before the layer-4 envelope
    further tightens it via `residual_fraction_*`, claude-docs/08-learning.md). Recorded
    verbatim in the deployment contract's `action_space.fields`."""

    steering_scale_rad: float = 0.4189
    speed_scale_mps: float = 5.0


@dataclass(frozen=True)
class EnvSettings:
    track_csv: str = "config/tracks/gym_oval/raceline.csv"
    lidar_downsample_factor: int = 10
    timestep_s: float = 0.02  # 50 Hz, claude-docs/08-learning.md "Inference target: 50 Hz"
    max_episode_steps: int = 2000
    action: ActionSettings = field(default_factory=ActionSettings)


@dataclass(frozen=True)
class SACSettings:
    total_timesteps: int = 200_000
    learning_rate: float = 3e-4
    buffer_size: int = 100_000
    batch_size: int = 256
    learning_starts: int = 1000
    train_freq: int = 1
    gradient_steps: int = 1
    net_arch: tuple[int, ...] = (64, 64)


@dataclass(frozen=True)
class ExperimentConfig:
    name: str
    seed: int
    env: EnvSettings
    pure_pursuit: PurePursuitSettings
    envelope: EnvelopeSettings
    reward: RewardWeights
    sac: SACSettings


def _pure_pursuit_from_dict(raw: dict[str, Any]) -> PurePursuitSettings:
    return PurePursuitSettings(**raw)


def _action_from_dict(raw: dict[str, Any]) -> ActionSettings:
    return ActionSettings(**raw)


def _env_from_dict(raw: dict[str, Any]) -> EnvSettings:
    raw = dict(raw)
    action_raw = raw.pop("action", {})
    return EnvSettings(action=_action_from_dict(action_raw), **raw)


def _envelope_from_dict(raw: dict[str, Any]) -> EnvelopeSettings:
    raw = dict(raw)
    if "ood_reference_state" in raw:
        raw["ood_reference_state"] = tuple(raw["ood_reference_state"])
    if raw.get("ood_reference_scale") is not None:
        raw["ood_reference_scale"] = tuple(raw["ood_reference_scale"])
    return EnvelopeSettings(**raw)


def _reward_from_dict(raw: dict[str, Any]) -> RewardWeights:
    return RewardWeights(**raw)


def _sac_from_dict(raw: dict[str, Any]) -> SACSettings:
    raw = dict(raw)
    if "net_arch" in raw:
        raw["net_arch"] = tuple(raw["net_arch"])
    return SACSettings(**raw)


def config_from_dict(raw: dict[str, Any]) -> ExperimentConfig:
    return ExperimentConfig(
        name=raw["name"],
        seed=raw["seed"],
        env=_env_from_dict(raw.get("env", {})),
        pure_pursuit=_pure_pursuit_from_dict(raw.get("pure_pursuit", {})),
        envelope=_envelope_from_dict(raw["envelope"]),
        reward=_reward_from_dict(raw.get("reward", {})),
        sac=_sac_from_dict(raw.get("sac", {})),
    )


def load_config(path: Path | str) -> ExperimentConfig:
    path = Path(path)
    with path.open("r", encoding="utf-8") as f:
        raw = yaml.safe_load(f)
    if not isinstance(raw, dict):
        raise TypeError(f"{path} must parse to a YAML mapping at the top level")
    return config_from_dict(raw)


def config_sha256(path: Path | str) -> str:
    """sha256 hex digest of the raw config file bytes -- the exact value recorded in the
    deployment contract's `training.config_hash` (ros_ws/src/racer_policy/
    contract.schema.json's `sha256_hex` pattern)."""
    return hashlib.sha256(Path(path).read_bytes()).hexdigest()
