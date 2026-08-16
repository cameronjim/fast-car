"""Unit tests for racer_train.env.ResidualRacerEnv: observation schema stability, residual
application, and the L5 "envelope-in-env test" divergence check
(claude-docs/12-testing.md: "run a deliberately hostile policy (max residual, max rate) --
assert the env-side envelope clips identically to the deployment library (same envelope/
import, same numbers; a divergence test, not two implementations)").
"""

from __future__ import annotations

import numpy as np
import pytest
from conftest import REPO_ROOT, SMOKE_CONFIG_PATH
from envelope import Command
from envelope import apply as envelope_apply
from racer_train.config import load_config
from racer_train.env import ResidualRacerEnv
from racer_train.observation import LOW_DIM_FIELDS


@pytest.fixture
def env():
    config = load_config(SMOKE_CONFIG_PATH)
    e = ResidualRacerEnv(config, repo_root=REPO_ROOT)
    yield e
    e.close()


# -- observation schema stability --------------------------------------------------------


def test_observation_space_shape_matches_low_dim_plus_lidar(env):
    expected_dim = len(LOW_DIM_FIELDS) + env.lidar_config.beam_count
    assert env.observation_space.shape == (expected_dim,)


def test_reset_observation_matches_declared_shape_and_dtype(env):
    obs, _info = env.reset(seed=0)
    assert obs.shape == env.observation_space.shape
    assert obs.dtype == np.float32
    assert np.all(np.isfinite(obs))


def test_step_observation_matches_declared_shape_and_dtype(env):
    env.reset(seed=0)
    obs, _reward, _term, _trunc, _info = env.step(np.array([0.0, 0.0]))
    assert obs.shape == env.observation_space.shape
    assert obs.dtype == np.float32
    assert np.all(np.isfinite(obs))


def test_observation_fields_match_low_dim_fields_order():
    # This is the contract-facing schema (racer_train.contract_export reads it verbatim) --
    # it must never silently drift from racer_train.observation.LOW_DIM_FIELDS, the single
    # source of truth claude-docs/02-repo-layout.md requires.
    config = load_config(SMOKE_CONFIG_PATH)
    e = ResidualRacerEnv(config, repo_root=REPO_ROOT)
    try:
        assert e.observation_fields == tuple(
            (name, "float32", units) for name, units in LOW_DIM_FIELDS
        )
    finally:
        e.close()


def test_lidar_beam_count_matches_raw_beam_count_over_downsample_factor(env):
    assert (
        env.lidar_config.beam_count
        == env.lidar_config.raw_beam_count // env.lidar_config.downsample_factor
    )


# -- action space / residual application --------------------------------------------------


def test_action_space_is_bounded_minus_one_to_one(env):
    assert env.action_space.shape == (2,)
    assert np.all(env.action_space.low == -1.0)
    assert np.all(env.action_space.high == 1.0)


def test_action_fields_bounds_match_configured_action_scales(env):
    steering_field, speed_field = env.action_fields
    assert steering_field[1] == pytest.approx(-env.config.env.action.steering_scale_rad)
    assert steering_field[2] == pytest.approx(env.config.env.action.steering_scale_rad)
    assert speed_field[1] == pytest.approx(-env.config.env.action.speed_scale_mps)
    assert speed_field[2] == pytest.approx(env.config.env.action.speed_scale_mps)


def test_zero_action_yields_zero_residual_command(env):
    env.reset(seed=0)
    _obs, _reward, _term, _trunc, info = env.step(np.array([0.0, 0.0]))
    assert info["envelope"]["residual_command"]["steering_rad"] == pytest.approx(0.0)
    assert info["envelope"]["residual_command"]["speed_mps"] == pytest.approx(0.0)


def test_full_scale_action_yields_scaled_residual_command(env):
    env.reset(seed=0)
    _obs, _reward, _term, _trunc, info = env.step(np.array([1.0, -1.0]))
    assert info["envelope"]["residual_command"]["steering_rad"] == pytest.approx(
        env.config.env.action.steering_scale_rad
    )
    assert info["envelope"]["residual_command"]["speed_mps"] == pytest.approx(
        -env.config.env.action.speed_scale_mps
    )


def test_action_is_clipped_before_scaling(env):
    env.reset(seed=0)
    # An out-of-space action must be clipped to [-1, 1] before scaling, never extrapolated.
    _obs, _reward, _term, _trunc, info = env.step(np.array([5.0, -5.0]))
    assert info["envelope"]["residual_command"]["steering_rad"] == pytest.approx(
        env.config.env.action.steering_scale_rad
    )
    assert info["envelope"]["residual_command"]["speed_mps"] == pytest.approx(
        -env.config.env.action.speed_scale_mps
    )


# -- reward wiring --------------------------------------------------------------------------


def test_reward_breakdown_is_exposed_in_info_and_sums_to_reward(env):
    env.reset(seed=0)
    _obs, reward, _term, _trunc, info = env.step(np.array([0.0, 0.0]))
    breakdown = info["reward"]
    expected_total = breakdown["progress"] - breakdown["crash"] - breakdown["envelope_violation"]
    assert reward == pytest.approx(expected_total)


# -- L5 envelope-in-env divergence test (claude-docs/12-testing.md) -----------------------


@pytest.mark.parametrize("action", [(0.0, 0.0), (1.0, 1.0), (-1.0, -1.0), (1.0, -1.0)])
def test_env_envelope_matches_direct_apply_call(env, action):
    """The core divergence check: run `action` through `env.step()`, then independently
    reconstruct the SAME `envelope.apply()` call from public state (the config/state the env
    reports, and the base/residual commands it reports having used) and assert the two
    agree exactly -- same `envelope/` import, same numbers, not two implementations."""
    obs, _info = env.reset(seed=0)
    observed_state = tuple(float(v) for v in obs[: len(LOW_DIM_FIELDS)])
    envelope_state_before = env.envelope_state

    _obs, _reward, _term, _trunc, info = env.step(np.array(action))

    base = info["envelope"]["base_command"]
    residual = info["envelope"]["residual_command"]
    expected = envelope_apply(
        env.envelope_config,
        envelope_state_before,
        Command(steering_rad=base["steering_rad"], speed_mps=base["speed_mps"]),
        Command(steering_rad=residual["steering_rad"], speed_mps=residual["speed_mps"]),
        observed_state=observed_state,
    )

    actual = info["envelope"]["command"]
    assert actual["steering_rad"] == expected.command.steering_rad
    assert actual["speed_mps"] == expected.command.speed_mps
    assert info["envelope"]["residual_clipped"] == expected.residual_clipped
    assert info["envelope"]["rate_limited"] == expected.rate_limited
    assert info["envelope"]["ood_triggered"] == expected.ood_triggered


def test_hostile_action_actually_exercises_residual_clipping(env):
    """Sanity check that the divergence test above is not vacuous: a max-amplitude residual
    at this env's configured action scale (0.4189 rad / 5.0 m/s, both larger than the
    envelope's residual_fraction-derived bounds -- see training/configs/*.yaml's comment)
    must actually get clipped by at least one channel."""
    env.reset(seed=0)
    _obs, _reward, _term, _trunc, info = env.step(np.array([1.0, 1.0]))
    assert info["envelope"]["residual_clipped"] is True
