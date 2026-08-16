"""racer_train: the S.3 SAC residual training pipeline (claude-docs/08-learning.md).

Public API kept deliberately small and torch-free (see this package's pyproject.toml
description): everything needed to construct and step the training environment, without
importing torch or stable_baselines3 anywhere in this module.
"""

from __future__ import annotations

from racer_train.config import (
    ActionSettings,
    EnvSettings,
    ExperimentConfig,
    PurePursuitSettings,
    SACSettings,
    config_sha256,
    load_config,
)
from racer_train.env import ResidualRacerEnv
from racer_train.envelope_config import EnvelopeSettings
from racer_train.raceline import (
    PurePursuitCommand,
    PurePursuitConfig,
    PurePursuitController,
    Raceline,
    RacelineLoadError,
    RacelinePoint,
)
from racer_train.reward import RewardTerms, RewardWeights, compute_reward, progress_reward

__all__ = [
    "ActionSettings",
    "EnvSettings",
    "EnvelopeSettings",
    "ExperimentConfig",
    "PurePursuitCommand",
    "PurePursuitConfig",
    "PurePursuitController",
    "PurePursuitSettings",
    "Raceline",
    "RacelineLoadError",
    "RacelinePoint",
    "ResidualRacerEnv",
    "RewardTerms",
    "RewardWeights",
    "SACSettings",
    "compute_reward",
    "config_sha256",
    "load_config",
    "progress_reward",
]
