"""Vehicle overrides, normalisation, and the committed v0 experiment config."""

from __future__ import annotations

import dataclasses

import numpy as np
import pytest
from conftest import REPO_ROOT
from racer_train.config import config_sha256, load_config
from racer_train.normalization import CLIP_SIGMA, ObsNormalization, fit_normalization
from racer_train.tune_base import with_base_settings
from racer_train.vehicle_overrides import apply_vehicle_overrides

V0_CONFIG = REPO_ROOT / "training" / "configs" / "v0" / "sac_residual_v0_0.1.0.yaml"


def test_overrides_apply_to_a_copy(real_vehicle_params):
    out = apply_vehicle_overrides(
        real_vehicle_params, {"steering": {"max_angle_rad": 0.504, "min_angle_rad": -0.449}}
    )
    assert out.steering.max_angle_rad == 0.504 and out.steering.min_angle_rad == -0.449
    assert real_vehicle_params.steering.max_angle_rad == 0.4189  # original untouched
    assert out.chassis.wheelbase_m == real_vehicle_params.chassis.wheelbase_m


def test_unknown_override_key_is_an_error(real_vehicle_params):
    with pytest.raises(KeyError):
        apply_vehicle_overrides(real_vehicle_params, {"steering": {"max_angel_rad": 0.5}})
    with pytest.raises(KeyError):
        apply_vehicle_overrides(real_vehicle_params, {"nonexistent": {"x": 1}})


def test_empty_override_returns_the_same_object(real_vehicle_params):
    assert apply_vehicle_overrides(real_vehicle_params, {}) is real_vehicle_params


def test_v0_config_overrides_are_all_valid_fields_and_asymmetric(real_vehicle_params):
    cfg = load_config(V0_CONFIG)
    vp = apply_vehicle_overrides(real_vehicle_params, cfg.vehicle_overrides)
    assert vp.steering.max_angle_rad == 0.504
    assert vp.steering.min_angle_rad == -0.449
    assert vp.chassis.front_overhang_m == 0.11 and vp.chassis.rear_overhang_m == 0.14
    # 2.91 / 1.401 = 2.077: the corrected VESC-cap top speed.
    assert vp.actuation.throttle_full_scale_mps == pytest.approx(2.91 / 1.401, abs=2e-3)
    assert vp.chassis.wheelbase_m == pytest.approx(0.3302)


def test_v0_config_is_v0_mode_and_hash_is_stable():
    cfg = load_config(V0_CONFIG)
    assert cfg.env.observation_profile == "v0"
    assert cfg.env.tracks is not None and cfg.randomization.enabled
    assert cfg.envelope.speed_cap_mps == 2.0
    assert config_sha256(V0_CONFIG) == config_sha256(V0_CONFIG)
    assert len(config_sha256(V0_CONFIG)) == 64


def test_action_scale_does_not_exceed_the_envelope_bound(real_vehicle_params):
    cfg = load_config(V0_CONFIG)
    vp = apply_vehicle_overrides(real_vehicle_params, cfg.vehicle_overrides)
    steer_range = vp.steering.max_angle_rad - vp.steering.min_angle_rad
    speed_range = vp.limits.global_speed_cap_mps - vp.limits.min_velocity_mps
    assert (
        cfg.env.action.steering_scale_rad <= cfg.envelope.residual_fraction_steering * steer_range
    )
    assert cfg.env.action.speed_scale_mps <= cfg.envelope.residual_fraction_speed * speed_range


def test_with_base_settings_only_changes_the_base():
    cfg = load_config(V0_CONFIG)
    out = with_base_settings(cfg, 0.5, 0.9, 0.7, 2.0, 1.7, 5, 3000, 999)
    assert out.pure_pursuit.lookahead_min_m == 0.5 and out.pure_pursuit.lookahead_max_m == 0.9
    assert out.env.tracks.family.a_lat_max_mps2 == 2.0 and out.env.tracks.family.v_max_mps == 1.7
    assert out.env.tracks.train_pool_size == 5 and out.env.max_episode_steps == 999
    assert out.envelope == cfg.envelope and out.randomization == cfg.randomization
    assert dataclasses.replace(out, pure_pursuit=cfg.pure_pursuit).sac == cfg.sac


def test_normalization_fit_and_apply_hand_computed():
    obs = np.array([[1.0, 10.0, 9.0], [3.0, 10.0, 1.0]], dtype=np.float32)
    n = fit_normalization(obs, low_dim_count=2, lidar_clip_m=5.0)
    # LiDAR column clipped to 5 before the statistics: values 5 and 1 -> mean 3, std 2.
    assert n.mean == pytest.approx((2.0, 10.0, 3.0))
    assert n.std[0] == pytest.approx(1.0) and n.std[2] == pytest.approx(2.0)
    assert n.std[1] == pytest.approx(1e-3)  # constant column gets min_std, not 0
    out = n.normalize(np.array([[3.0, 10.0, 99.0]]))
    assert out[0, 0] == pytest.approx(1.0)
    assert out[0, 1] == pytest.approx(0.0)
    assert out[0, 2] == pytest.approx(1.0)  # 99 m clipped to 5 m -> (5-3)/2


def test_normalization_clips_outliers_to_sigma_bound():
    n = ObsNormalization(mean=(0.0, 0.0), std=(1.0, 1.0), low_dim_count=1, lidar_clip_m=5.0)
    assert n.normalize(np.array([1e6, 0.0]))[0] == CLIP_SIGMA
    assert n.normalize(np.array([-1e6, 0.0]))[0] == -CLIP_SIGMA


def test_normalization_validates():
    with pytest.raises(ValueError):
        ObsNormalization(mean=(0.0,), std=(1.0, 1.0), low_dim_count=1, lidar_clip_m=5.0)
    with pytest.raises(ValueError):
        ObsNormalization(mean=(0.0,), std=(0.0,), low_dim_count=1, lidar_clip_m=5.0)
    with pytest.raises(ValueError):
        ObsNormalization(mean=(0.0,), std=(1.0,), low_dim_count=2, lidar_clip_m=5.0)
