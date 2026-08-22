"""Tests for racer_train.contract_export: the manifest it builds must be loadable and
verifiable through the REAL S.5 loader (ros_ws/src/racer_policy), not just internally
self-consistent. This is the "S.5 loader must load it" requirement from the S.3 task."""

from __future__ import annotations

import hashlib

import pytest
from conftest import REPO_ROOT, SMOKE_CONFIG_PATH
from racer_policy import load_contract, verify_against_environment
from racer_policy.contract import ObservationField
from racer_policy.environment import LiveEnvironment, LiveObservationSchema
from racer_train.config import load_config
from racer_train.contract_export import build_contract_manifest, write_contract_dir
from racer_train.env import ResidualRacerEnv

DUMMY_POLICY_BYTES = b"not-a-real-torchscript-model, just a fixture for the checksum path"


@pytest.fixture
def built_env():
    config = load_config(SMOKE_CONFIG_PATH)
    e = ResidualRacerEnv(config, repo_root=REPO_ROOT)
    yield e
    e.close()


def _write_dummy_contract(tmp_path, env: ResidualRacerEnv):
    lidar_dim = env.lidar_config.beam_count
    obs_dim = len(env.observation_fields) + lidar_dim

    contract_dir = tmp_path / "contract"
    contract_dir.mkdir()
    (contract_dir / "policy.pt").write_bytes(DUMMY_POLICY_BYTES)

    manifest = build_contract_manifest(
        env=env,
        policy_filename="policy.pt",
        policy_checksum_sha256=hashlib.sha256(DUMMY_POLICY_BYTES).hexdigest(),
        normalization_mean=[0.0] * obs_dim,
        normalization_std=[1.0] * obs_dim,
        config_hash=hashlib.sha256(b"fixture-config").hexdigest(),
        git_sha="abc1234",
    )
    write_contract_dir(contract_dir, manifest)
    return contract_dir, manifest


def test_build_contract_manifest_rejects_wrong_length_normalization(built_env):
    with pytest.raises(ValueError):
        build_contract_manifest(
            env=built_env,
            policy_filename="policy.pt",
            policy_checksum_sha256="0" * 64,
            normalization_mean=[0.0],  # wrong length
            normalization_std=[1.0],
            config_hash="0" * 64,
            git_sha="abc1234",
        )


def test_emitted_contract_loads_through_racer_policy(tmp_path, built_env):
    contract_dir, _manifest = _write_dummy_contract(tmp_path, built_env)
    contract = load_contract(contract_dir)
    assert contract.contract_version == "1.0.0"
    assert contract.policy.filename == "policy.pt"


def test_emitted_contract_verifies_against_a_matching_live_environment(tmp_path, built_env):
    contract_dir, _manifest = _write_dummy_contract(tmp_path, built_env)
    contract = load_contract(contract_dir)

    live = LiveEnvironment(
        vehicle_params=built_env.vehicle_params,
        observation=LiveObservationSchema(
            fields=tuple(
                ObservationField(name=n, dtype=d, units=u)
                for n, d, u in built_env.observation_fields
            ),
            lidar_beam_count=built_env.lidar_config.beam_count,
            lidar_fov_rad=built_env.lidar_config.fov_rad,
            lidar_downsample_factor=built_env.lidar_config.downsample_factor,
        ),
    )
    # Must not raise -- claude-docs/08-learning.md: this is the "S.5 loader must load it"
    # round trip.
    verify_against_environment(contract, live)


def test_emitted_contract_refuses_on_observation_field_order_mismatch(tmp_path, built_env):
    contract_dir, _manifest = _write_dummy_contract(tmp_path, built_env)
    contract = load_contract(contract_dir)

    reordered_fields = tuple(
        ObservationField(name=n, dtype=d, units=u) for n, d, u in built_env.observation_fields
    )
    reordered_fields = (reordered_fields[1], reordered_fields[0]) + reordered_fields[2:]

    live = LiveEnvironment(
        vehicle_params=built_env.vehicle_params,
        observation=LiveObservationSchema(
            fields=reordered_fields,
            lidar_beam_count=built_env.lidar_config.beam_count,
            lidar_fov_rad=built_env.lidar_config.fov_rad,
            lidar_downsample_factor=built_env.lidar_config.downsample_factor,
        ),
    )
    from racer_policy.errors import ContractError

    with pytest.raises(ContractError):
        verify_against_environment(contract, live)
