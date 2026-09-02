# observation layout and deploy contract tests

import json

import numpy as np
import pytest

from f1rl.envs.obs import ActionBounds, ObsConfig, deploy_contract

TRACK_LENGTH_M = 338.13


def default_cfg(**changes) -> ObsConfig:
    base = dict(
        features=("scan", "linear_vel_x", "ang_vel_z", "delta", "frenet_pose"),
        num_beams=108,
        track_length_m=TRACK_LENGTH_M,
    )
    base.update(changes)
    return ObsConfig(**base)


def test_flat_layout_is_alphabetical():
    cfg = default_cfg()
    assert cfg.slices["ang_vel_z"] == slice(0, 1)
    assert cfg.slices["delta"] == slice(1, 2)
    assert cfg.slices["frenet_pose"] == slice(2, 5)
    assert cfg.slices["linear_vel_x"] == slice(5, 6)
    assert cfg.slices["scan"] == slice(6, 114)
    assert cfg.raw_dim == 114
    assert cfg.obs_dim == 116


def test_normalize_scales_each_feature():
    cfg = default_cfg()
    raw = np.zeros(cfg.raw_dim, dtype=np.float32)
    raw[cfg.slices["ang_vel_z"]] = 2.5
    raw[cfg.slices["delta"]] = 0.4189
    raw[cfg.slices["frenet_pose"]] = [TRACK_LENGTH_M / 2, 2.5, np.pi]
    raw[cfg.slices["linear_vel_x"]] = 4.0
    raw[cfg.slices["scan"]] = 15.0
    obs = cfg.normalize(raw, prev_action=[0.25, -0.75])

    assert obs.shape == (116,)
    assert obs[0] == pytest.approx(0.5)
    assert obs[1] == pytest.approx(1.0)
    assert obs[2:5] == pytest.approx([0.5, 0.5, 1.0])
    assert obs[5] == pytest.approx(0.5)
    assert obs[6:114] == pytest.approx(np.full(108, 0.5))
    assert obs[114:] == pytest.approx([0.25, -0.75])


def test_normalize_clips_and_handles_non_finite_beams():
    cfg = default_cfg()
    raw = np.zeros(cfg.raw_dim, dtype=np.float32)
    raw[cfg.slices["scan"]][:2] = [np.inf, np.nan]
    raw[cfg.slices["frenet_pose"]] = [0.0, 500.0, 0.0]
    obs = cfg.normalize(raw)
    assert obs[6] == pytest.approx(1.0)
    assert obs[7] == pytest.approx(1.0)
    assert obs[3] == pytest.approx(1.0)
    assert np.all(np.abs(obs) <= 1.0)


def test_normalize_rejects_wrong_shape():
    cfg = default_cfg()
    with pytest.raises(ValueError):
        cfg.normalize(np.zeros(cfg.raw_dim + 1, dtype=np.float32))


def test_frenet_needs_track_length():
    with pytest.raises(ValueError):
        ObsConfig(features=("scan", "frenet_pose"), num_beams=8)


def test_unknown_feature_has_no_normalization():
    with pytest.raises(ValueError):
        ObsConfig(features=("scan", "std_state"), num_beams=8)


def test_contract_json_round_trip():
    cfg = default_cfg()
    blob = json.loads(json.dumps(deploy_contract(cfg, ActionBounds())))
    restored = ObsConfig.from_json(blob)
    assert restored == cfg
    assert restored.slices == cfg.slices
    raw = np.linspace(0.0, 1.0, cfg.raw_dim, dtype=np.float32)
    assert restored.normalize(raw) == pytest.approx(cfg.normalize(raw))


def test_contract_marks_frenet_as_sim_only():
    blob = deploy_contract(default_cfg(), ActionBounds())
    deployable = {f["name"]: f["deployable"] for f in blob["features"]}
    assert deployable["frenet_pose"] is False
    assert deployable["scan"] is True
    assert blob["action"]["speed_cap_mps"] == 3.0
    assert default_cfg().undeployable_features() == ("frenet_pose",)


def test_deploy_feature_set_drops_frenet():
    cfg = ObsConfig(features=("scan", "linear_vel_x", "ang_vel_z", "delta"), num_beams=108)
    assert cfg.raw_dim == 111
    assert cfg.obs_dim == 113
    assert cfg.undeployable_features() == ()


def test_action_bounds_reject_inverted_speed_range():
    with pytest.raises(ValueError):
        ActionBounds(speed_min_mps=4.0, speed_cap_mps=3.0)
