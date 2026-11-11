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
from racer_train.randomization import RandomizationConfig
from racer_train.reward import RewardWeights
from racer_train.tracks import TrackFamilyConfig


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
class TracksSettings:
    """Procedural track pool for the v0 env (racer_train.tracks). When absent the env falls
    back to the single committed `track_csv` (the S.3 behaviour)."""

    family: TrackFamilyConfig = field(default_factory=TrackFamilyConfig)
    train_pool_size: int = 60
    train_seed_base: int = 1000


@dataclass(frozen=True)
class EpisodeSettings:
    """How an episode starts. `rolling` starts anywhere on the line at a random fraction of
    the base target speed (training: covers the whole track quickly); `standing` starts at
    the first raceline point at rest (evaluation: a real lap from a real start)."""

    start: str = "rolling"
    start_lateral_m: float = 0.08
    start_heading_rad: float = 0.12


@dataclass(frozen=True)
class EnvSettings:
    track_csv: str = "config/tracks/gym_oval/raceline.csv"
    lidar_downsample_factor: int = 10
    timestep_s: float = 0.02  # 50 Hz, claude-docs/08-learning.md "Inference target: 50 Hz"
    max_episode_steps: int = 2000
    action: ActionSettings = field(default_factory=ActionSettings)
    observation_profile: str = "s3"
    tracks: TracksSettings | None = None
    episode: EpisodeSettings = field(default_factory=EpisodeSettings)


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
    gamma: float = 0.99
    tau: float = 0.005
    checkpoint_every_steps: int = 0  # 0 = only the final checkpoint
    eval_every_steps: int = 0  # 0 = no periodic in-training evaluation
    eval_episodes: int = 12
    normalization_steps: int = 3000  # env steps used to fit the frozen observation statistics


@dataclass(frozen=True)
class ExperimentConfig:
    name: str
    seed: int
    env: EnvSettings
    pure_pursuit: PurePursuitSettings
    envelope: EnvelopeSettings
    reward: RewardWeights
    sac: SACSettings
    randomization: RandomizationConfig = field(default_factory=RandomizationConfig)
    # In-memory overrides applied on top of the generated vehicle_params binding (nested
    # field names, e.g. {"steering": {"max_angle_rad": 0.504}}). They exist so a training run
    # can use measurements that are not in config/vehicle_params.yaml YET (the 2026-11-10
    # calibration lands on its own branch); they are part of the hashed config, so a contract
    # always records exactly what was assumed. Never written back to vehicle_params.yaml.
    vehicle_overrides: dict[str, Any] = field(default_factory=dict)
    # Written into policy_meta.yaml. Empty `label` keeps the v0 export label.
    label: str = ""
    policy_version: str = "v0"


def _pure_pursuit_from_dict(raw: dict[str, Any]) -> PurePursuitSettings:
    return PurePursuitSettings(**raw)


def _action_from_dict(raw: dict[str, Any]) -> ActionSettings:
    return ActionSettings(**raw)


def _tupled(raw: dict[str, Any]) -> dict[str, Any]:
    """YAML lists -> tuples (the dataclasses are frozen/hashable and typed with tuples)."""
    return {k: (tuple(v) if isinstance(v, list) else v) for k, v in raw.items()}


def _tracks_from_dict(raw: dict[str, Any]) -> TracksSettings:
    raw = dict(raw)
    family_raw = raw.pop("family", {})
    return TracksSettings(family=TrackFamilyConfig(**_tupled(family_raw)), **raw)


def _env_from_dict(raw: dict[str, Any]) -> EnvSettings:
    raw = dict(raw)
    action_raw = raw.pop("action", {})
    tracks_raw = raw.pop("tracks", None)
    episode_raw = raw.pop("episode", {})
    return EnvSettings(
        action=_action_from_dict(action_raw),
        tracks=_tracks_from_dict(tracks_raw) if tracks_raw is not None else None,
        episode=EpisodeSettings(**episode_raw),
        **raw,
    )


def _randomization_from_dict(raw: dict[str, Any]) -> RandomizationConfig:
    return RandomizationConfig(**_tupled(raw))


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
        randomization=_randomization_from_dict(raw.get("randomization", {})),
        vehicle_overrides=dict(raw.get("vehicle_overrides", {})),
        label=str(raw.get("label", "")),
        policy_version=str(raw.get("policy_version", "v0")),
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
