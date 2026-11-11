"""L5 (nightly, torch-gated): a tiny v0 run exports a policy directory that

* `racer_policy.load_contract` + `verify_against_environment` + `load_model` accept,
* reproduces the SB3 actor's deterministic action from RAW observations (normalisation is
  baked into policy.pt), and
* still REFUSES every class of mismatch (checksum, contract version, vehicle_params
  version, observation units, observation order). A test proving it refuses is as important
  as one proving it loads (12-testing.md).
"""

from __future__ import annotations

import shutil
from pathlib import Path

import numpy as np
import pytest
import yaml
from conftest import FIXTURES_DIR, REPO_ROOT
from racer_train.config import load_config

torch = pytest.importorskip("torch")
pytest.importorskip("stable_baselines3")

V0_TINY = FIXTURES_DIR / "v0_tiny_config.yaml"


@pytest.fixture(scope="module")
def exported(tmp_path_factory):
    from racer_train.train import train

    out = tmp_path_factory.mktemp("v0run")
    config = load_config(V0_TINY)
    contract_dir = train(config, V0_TINY, out, repo_root=REPO_ROOT)
    return out, contract_dir, config


def _live(env, vehicle_params=None, fields=None):
    from racer_policy.contract import ObservationField
    from racer_policy.environment import LiveEnvironment, LiveObservationSchema

    return LiveEnvironment(
        vehicle_params=vehicle_params or env.committed_vehicle_params,
        observation=LiveObservationSchema(
            fields=fields
            or tuple(
                ObservationField(name=n, dtype=d, units=u) for n, d, u in env.observation_fields
            ),
            lidar_beam_count=env.lidar_config.beam_count,
            lidar_fov_rad=env.lidar_config.fov_rad,
            lidar_downsample_factor=env.lidar_config.downsample_factor,
        ),
    )


@pytest.mark.slow
def test_exported_dir_loads_verifies_and_runs(exported):
    from racer_policy import load_contract, load_model, verify_against_environment
    from racer_train.env import ResidualRacerEnv

    _out, contract_dir, config = exported
    contract = load_contract(contract_dir)
    env = ResidualRacerEnv(config, repo_root=REPO_ROOT)
    try:
        verify_against_environment(contract, _live(env))
        model = load_model(contract)
        obs, _ = env.reset(options={"episode_seed": 1})
        with torch.no_grad():
            action = model(torch.from_numpy(obs[None, :]))
        assert action.shape == (1, 2) and bool((action.abs() <= 1.0).all())
    finally:
        env.close()


@pytest.mark.slow
def test_contract_normalization_is_identity_and_meta_carries_the_v0_label(exported):
    _out, contract_dir, config = exported
    raw = yaml.safe_load((contract_dir / "contract.yaml").read_text())
    assert set(raw["normalization"]["mean"]) == {0.0} and set(raw["normalization"]["std"]) == {1.0}
    meta = yaml.safe_load((contract_dir / "policy_meta.yaml").read_text())
    assert (
        "v0" in meta["label"] and "PRE-SYSID" in meta["label"] and "NOT the thesis" in meta["label"]
    )
    assert meta["policy_version"] == "v0"
    assert (
        meta["must_match_at_deploy"]["envelope"]["speed_cap_mps"] == config.envelope.speed_cap_mps
    )
    assert meta["observation_normalization"]["applied_inside_policy_pt"] is True
    assert meta["config_sha256"] == raw["training"]["config_hash"]


@pytest.mark.slow
def test_torchscript_matches_sb3_actor_on_raw_observations(exported):
    import json

    from racer_policy import load_contract, load_model
    from racer_train.normalization import ObsNormalization
    from stable_baselines3 import SAC

    out, contract_dir, _config = exported
    stats = json.loads((out / "normalization.json").read_text())
    norm = ObsNormalization(
        mean=tuple(stats["mean"]),
        std=tuple(stats["std"]),
        low_dim_count=stats["low_dim_count"],
        lidar_clip_m=stats["lidar_clip_m"],
    )
    sac = SAC.load(out / "sac_checkpoint.zip", device="cpu")
    model = load_model(load_contract(contract_dir))
    rng = np.random.default_rng(0)
    dim = len(stats["mean"])
    raw = rng.uniform(-1.0, 12.0, size=(64, dim)).astype(np.float32)  # includes > clip LiDAR
    with torch.no_grad():
        ts = model(torch.from_numpy(raw)).numpy()
    expected, _ = sac.predict(norm.normalize(raw), deterministic=True)
    np.testing.assert_allclose(ts, expected, atol=1e-5)


def _copy(contract_dir: Path, tmp_path: Path) -> Path:
    dst = tmp_path / "policy"
    shutil.copytree(contract_dir, dst)
    return dst


@pytest.mark.slow
def test_v0_dir_refuses_a_corrupted_policy(exported, tmp_path):
    from racer_policy import ChecksumMismatchError, load_contract

    _out, contract_dir, _config = exported
    d = _copy(contract_dir, tmp_path)
    blob = bytearray((d / "policy.pt").read_bytes())
    blob[len(blob) // 2] ^= 0xFF
    (d / "policy.pt").write_bytes(bytes(blob))
    with pytest.raises(ChecksumMismatchError):
        load_contract(d)


@pytest.mark.slow
def test_v0_dir_refuses_a_future_contract_version(exported, tmp_path):
    from racer_policy import ContractVersionMismatchError, load_contract

    _out, contract_dir, _config = exported
    d = _copy(contract_dir, tmp_path)
    raw = yaml.safe_load((d / "contract.yaml").read_text())
    raw["contract_version"] = "2.0.0"
    (d / "contract.yaml").write_text(yaml.safe_dump(raw, sort_keys=False))
    with pytest.raises(ContractVersionMismatchError):
        load_contract(d)


@pytest.mark.slow
def test_v0_dir_refuses_vehicle_params_and_observation_mismatches(exported):
    import dataclasses

    from racer_policy import (
        ObservationSchemaMismatchError,
        ObservationUnitsMismatchError,
        VehicleParamsVersionMismatchError,
        load_contract,
        verify_against_environment,
    )
    from racer_policy.contract import ObservationField
    from racer_train.env import ResidualRacerEnv

    _out, contract_dir, config = exported
    contract = load_contract(contract_dir)
    env = ResidualRacerEnv(config, repo_root=REPO_ROOT)
    try:
        vp = env.committed_vehicle_params
        other_meta = dataclasses.replace(vp.meta, sysid_session_id="some-other-session")
        with pytest.raises(VehicleParamsVersionMismatchError):
            verify_against_environment(
                contract, _live(env, vehicle_params=dataclasses.replace(vp, meta=other_meta))
            )
        other_schema = dataclasses.replace(vp.meta, schema_version="9.9.9")
        with pytest.raises(VehicleParamsVersionMismatchError):
            verify_against_environment(
                contract, _live(env, vehicle_params=dataclasses.replace(vp, meta=other_schema))
            )

        fields = [ObservationField(name=n, dtype=d, units=u) for n, d, u in env.observation_fields]
        wrong_units = list(fields)
        wrong_units[0] = ObservationField(fields[0].name, fields[0].dtype, "km/h")
        with pytest.raises(ObservationUnitsMismatchError):
            verify_against_environment(contract, _live(env, fields=tuple(wrong_units)))

        swapped = list(fields)
        swapped[0], swapped[1] = swapped[1], swapped[0]
        with pytest.raises(ObservationSchemaMismatchError):
            verify_against_environment(contract, _live(env, fields=tuple(swapped)))
    finally:
        env.close()


@pytest.mark.slow
def test_export_from_checkpoint_reproduces_the_trained_export(exported, tmp_path):
    """Exporting a saved checkpoint later (a run stopped early) must refit the SAME frozen
    normalisation: the re-exported policy gives identical actions."""
    from racer_policy import load_contract, load_model
    from racer_train.train import export_from_checkpoint

    out, contract_dir, config = exported
    redo = export_from_checkpoint(
        config, V0_TINY, out / "sac_checkpoint.zip", tmp_path / "redo", repo_root=REPO_ROOT
    )
    a = load_model(load_contract(contract_dir))
    b = load_model(load_contract(redo))
    raw = torch.rand(16, 47) * 3.0
    with torch.no_grad():
        np.testing.assert_allclose(a(raw).numpy(), b(raw).numpy(), atol=1e-6)


@pytest.mark.slow
def test_latency_tool_reports_a_sane_summary(exported):
    from racer_train.latency import measure

    _out, contract_dir, _config = exported
    result = measure(contract_dir, n_calls=60, warmup=5)
    assert result["n_calls"] == 60 and result["parameters"] > 0
    assert 0.0 < result["p50_us"] <= result["p99_us"] <= result["max_us"]
    assert sum(result["histogram_us"].values()) == 60
    assert result["budget_us_at_50hz"] == 20000.0
