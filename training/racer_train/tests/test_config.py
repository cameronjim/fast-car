"""Unit tests for racer_train.config: loading experiment configs and hashing them for the
deployment contract's `training.config_hash` (claude-docs/10-conventions.md reproducibility
rule)."""

from __future__ import annotations

import hashlib

from conftest import SMOKE_CONFIG_PATH
from racer_train.config import ExperimentConfig, config_sha256, load_config


def test_load_config_parses_the_smoke_fixture():
    config = load_config(SMOKE_CONFIG_PATH)
    assert isinstance(config, ExperimentConfig)
    assert config.name == "sac_residual_smoke"
    assert config.seed == 0
    assert config.env.lidar_downsample_factor == 10
    assert config.envelope.residual_fraction_steering == 0.2
    assert config.envelope.ood_reference_state == (3.0, 0.0, 0.0, 0.0, 0.0)
    assert config.sac.net_arch == (16, 16)


def test_load_config_real_experiment_config_parses(real_vehicle_params):
    from conftest import REPO_ROOT

    config_path = REPO_ROOT / "training" / "configs" / "s3" / "sac_residual_0.1.0.yaml"
    config = load_config(config_path)
    assert config.name == "sac_residual_gym_oval"
    assert config.sac.total_timesteps == 200_000


def test_config_sha256_matches_hand_computed_digest():
    expected = hashlib.sha256(SMOKE_CONFIG_PATH.read_bytes()).hexdigest()
    assert config_sha256(SMOKE_CONFIG_PATH) == expected


def test_config_sha256_is_stable_across_calls():
    assert config_sha256(SMOKE_CONFIG_PATH) == config_sha256(SMOKE_CONFIG_PATH)
