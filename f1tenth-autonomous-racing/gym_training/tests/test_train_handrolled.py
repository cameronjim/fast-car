# tests for the hand-rolled sac adapter: the cross-package import and the action mapping

import numpy as np
import pytest
import torch

from f1rl.train_handrolled import (
    ACTION_DIM,
    HandrolledPolicy,
    handrolled_action,
    import_handrolled,
    unit_action,
)


def test_the_ros_package_trainer_and_nets_import_by_path():
    SACTrainer, SACActorNet, SACCriticNet = import_handrolled()

    assert SACTrainer.__module__ == "learned_control.sac.train"
    assert SACActorNet.__module__ == "learned_control.sac.model"
    assert SACCriticNet.__module__ == "learned_control.sac.model"


def test_the_actor_range_maps_onto_the_wrapper_action_box():
    assert unit_action([0.0, 0.0]) == pytest.approx([-1.0, -1.0])
    assert unit_action([1.0, 1.0]) == pytest.approx([1.0, 1.0])
    assert unit_action([0.5, 0.5]) == pytest.approx([0.0, 0.0])


def test_the_mapping_round_trips_in_both_directions():
    handrolled = np.linspace(0.0, 1.0, 11, dtype=np.float32)
    for value in handrolled:
        pair = [value, 1.0 - value]
        assert handrolled_action(unit_action(pair)) == pytest.approx(pair, abs=1e-6)

    unit = np.linspace(-1.0, 1.0, 11, dtype=np.float32)
    for value in unit:
        pair = [value, -value]
        assert unit_action(handrolled_action(pair)) == pytest.approx(pair, abs=1e-6)


def test_an_actor_output_outside_its_own_range_is_clipped_into_the_box():
    assert unit_action([5.0, -5.0]) == pytest.approx([1.0, -1.0])
    assert handrolled_action([5.0, -5.0]) == pytest.approx([1.0, 0.0])


def test_a_real_actor_drives_the_policy_shim_inside_the_action_box():
    _, SACActorNet, _ = import_handrolled()
    obs_dim = 116
    policy = HandrolledPolicy(SACActorNet(obs_dim, ACTION_DIM), torch.device("cpu"))

    action, state = policy.predict(np.zeros(obs_dim, dtype=np.float32), deterministic=True)

    assert state is None
    assert action.shape == (ACTION_DIM,)
    assert np.all(np.abs(action) <= 1.0)
