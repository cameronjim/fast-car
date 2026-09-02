# wrapper tests against the real f1tenth gym env

from pathlib import Path

import numpy as np
import pytest

from f1rl.envs import build_env, load_config

CONFIG_PATH = Path(__file__).resolve().parents[1] / "configs" / "sac_scratch.yaml"

pytestmark = pytest.mark.slow


@pytest.fixture(scope="module")
def env():
    cfg = load_config(CONFIG_PATH)
    cfg["env"]["max_episode_steps"] = 200
    built = build_env(cfg)
    yield built
    built.close()


def test_spaces_match_the_contract(env):
    assert env.observation_space.shape == (116,)
    assert env.action_space.shape == (2,)
    assert np.all(env.action_space.low == -1.0)
    assert np.all(env.action_space.high == 1.0)


def test_reset_obs_is_in_space(env):
    obs, _ = env.reset(seed=0)
    assert obs.dtype == np.float32
    assert env.observation_space.contains(obs)
    # a fresh episode has no previous action yet
    assert obs[114:] == pytest.approx([0.0, 0.0])


def test_action_is_rescaled_and_clipped(env):
    bounds = env.action_bounds
    assert env.rescale_action([1.0, 1.0]) == pytest.approx(
        [bounds.steer_max_rad, bounds.speed_cap_mps]
    )
    assert env.rescale_action([-1.0, -1.0]) == pytest.approx(
        [-bounds.steer_max_rad, bounds.speed_min_mps]
    )
    # step() upstream does not clip, so out-of-range actions must be clamped here
    assert env.rescale_action([5.0, -5.0]) == pytest.approx(
        [bounds.steer_max_rad, bounds.speed_min_mps]
    )


def test_speed_cap_is_mutable(env):
    original = env.action_bounds.speed_cap_mps
    env.set_speed_cap(6.0)
    assert env.rescale_action([0.0, 1.0])[1] == pytest.approx(6.0)
    env.set_speed_cap(original)


def test_driving_slowly_forward_earns_positive_reward(env):
    obs, _ = env.reset(seed=1)
    rewards = []
    for _ in range(100):
        obs, reward, terminated, truncated, info = env.step(np.array([0.0, -0.6], np.float32))
        rewards.append(reward)
        assert env.observation_space.contains(obs)
        assert obs[114:] == pytest.approx([0.0, -0.6], abs=1e-6)
        if terminated or truncated:
            break
    assert sum(rewards) > 0.0
    assert info["progress_m"] > 0.0


def test_collision_terminates_with_the_penalty(env):
    env.reset(seed=2)
    hard_left = np.array([1.0, 1.0], np.float32)
    for _ in range(400):
        _, reward, terminated, truncated, info = env.step(hard_left)
        if terminated:
            break
    assert terminated
    assert info["is_collision"] or info["wrong_way"]
    if info["is_collision"]:
        assert reward < 0.0


def test_sb3_check_env_passes():
    from stable_baselines3.common.env_checker import check_env

    cfg = load_config(CONFIG_PATH)
    cfg["env"]["max_episode_steps"] = 50
    checked = build_env(cfg, seed_offset=3)
    check_env(checked, warn=True, skip_render_check=True)
    checked.close()
