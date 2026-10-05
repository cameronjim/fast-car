# progress reward arithmetic tests

import pytest

from f1rl.envs.reward import ProgressReward

STILL = [0.0, 0.0]


def test_progress_is_weighted_metres():
    reward = ProgressReward(w_prog=2.0, w_rate=0.0)
    assert reward.step_reward(0.03, STILL, STILL) == pytest.approx(0.06)


def test_backwards_progress_is_negative():
    reward = ProgressReward(w_rate=0.0)
    assert reward.step_reward(-0.04, STILL, STILL) == pytest.approx(-0.04)


def test_action_rate_penalty_is_squared_l2():
    reward = ProgressReward(w_prog=0.0, w_rate=0.05)
    assert reward.step_reward(0.0, [0.3, -0.4], STILL) == pytest.approx(-0.05 * 0.25)


def test_holding_the_action_costs_nothing():
    reward = ProgressReward(w_prog=0.0, w_rate=0.5)
    assert reward.step_reward(0.0, [1.0, -1.0], [1.0, -1.0]) == pytest.approx(0.0)


def test_collision_subtracts_the_penalty():
    reward = ProgressReward(w_rate=0.0, collision_penalty=10.0)
    assert reward.step_reward(0.02, STILL, STILL, is_collision=True) == pytest.approx(-9.98)


def test_lap_bonus_scales_with_target_over_actual():
    reward = ProgressReward(w_prog=0.0, w_rate=0.0, lap_bonus=50.0, target_lap_time_sec=120.0)
    assert reward.step_reward(0.0, STILL, STILL, lap_time_sec=60.0) == pytest.approx(100.0)
    assert reward.step_reward(0.0, STILL, STILL, lap_time_sec=240.0) == pytest.approx(25.0)


def test_no_lap_bonus_on_ordinary_steps():
    reward = ProgressReward(w_prog=0.0, w_rate=0.0, lap_bonus=50.0)
    assert reward.step_reward(0.0, STILL, STILL, lap_time_sec=None) == pytest.approx(0.0)


def test_from_config_rejects_typos():
    with pytest.raises(ValueError):
        ProgressReward.from_config({"w_progress": 1.0})


def test_from_config_defaults_when_absent():
    assert ProgressReward.from_config(None) == ProgressReward()
