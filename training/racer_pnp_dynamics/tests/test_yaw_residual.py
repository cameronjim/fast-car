"""L1 for the yaw-only residual (candidate R, brief 31): a synthetic plant with a known yaw residual
AND a known speed residual. The yaw-only model recovers the yaw part, and its speed prediction
equals the nominal's exactly (max abs difference 0.0) at every horizon."""

from __future__ import annotations

from dataclasses import replace

import numpy as np
import pytest
from racer_pnp_dynamics import residual_train as rt
from racer_pnp_dynamics.dataset import by_split
from racer_pnp_dynamics.evaluate import eval_set, window_errors
from racer_pnp_dynamics.model import rollout
from racer_pnp_dynamics.residual import build_samples
from test_fit import TRUTH
from test_residual_train import CFG, NOISE, _numpy_trainer, _segments, known_residual

YAW_ONLY = replace(CFG, name="yaw_only_np", yaw_only=True)


@pytest.fixture(scope="module")
def trained(toy_vehicle_module):
    segs = _segments(toy_vehicle_module, known_residual(), **NOISE)
    train, val = by_split(segs, "train"), by_split(segs, "val")
    s_train = build_samples(train, TRUTH, toy_vehicle_module)
    s_val = build_samples(val, TRUTH, toy_vehicle_module)
    val_set = eval_set([s for s in val if s.run.has_odom])
    both = rt.fit_candidate(
        CFG, 0, TRUTH, toy_vehicle_module, s_train, s_val, val_set, None, None, _numpy_trainer
    )
    yaw = rt.fit_candidate(
        YAW_ONLY, 0, TRUTH, toy_vehicle_module, s_train, s_val, val_set, None, None, _numpy_trainer
    )
    return both, yaw, val_set


def test_the_speed_entry_of_the_scale_is_exactly_zero(trained):
    _, yaw, _ = trained
    assert yaw.residual.enabled
    assert yaw.residual.output_scale[0] == 0.0
    assert yaw.residual.output_scale[1] > 0.0


def test_the_unmasked_config_keeps_a_speed_output(trained):
    both, _, _ = trained
    assert both.residual.output_scale[0] > 0.0


def test_speed_prediction_equals_the_nominals_exactly(trained, toy_vehicle_module):
    _, yaw, _ = trained
    rng = np.random.default_rng(3)
    n = 400
    state0 = np.zeros((n, 5))
    state0[:, 3] = rng.uniform(0.0, 1.2, n)
    state0[:, 4] = rng.uniform(-0.4, 0.4, n)
    controls = np.stack([rng.uniform(0.0, 1.2, (n, 30)), rng.uniform(-0.4, 0.4, (n, 30))], axis=-1)
    with_res = rollout(state0, controls, TRUTH, toy_vehicle_module, yaw.residual)
    without = rollout(state0, controls, TRUTH, toy_vehicle_module, None)
    # every horizon, v and delta: not a tolerance, the same numbers
    assert float(np.max(np.abs(with_res[..., 3] - without[..., 3]))) == 0.0
    assert float(np.max(np.abs(with_res[..., 4] - without[..., 4]))) == 0.0
    # and the yaw channel did change something
    assert float(np.max(np.abs(with_res[..., 2] - without[..., 2]))) > 1e-3


def test_the_yaw_part_is_recovered(trained, toy_vehicle_module):
    both, yaw, val_set = trained
    nominal = window_errors(val_set, TRUTH, toy_vehicle_module, None)[1.0]
    fitted = window_errors(val_set, TRUTH, toy_vehicle_module, yaw.residual)[1.0]
    rms = lambda e: float(np.sqrt(np.mean(e**2)))
    assert rms(fitted["yaw"]) < 0.8 * rms(nominal["yaw"])
    # the speed error is the nominal's, to the last bit
    np.testing.assert_array_equal(fitted["speed"], nominal["speed"])
    # the unmasked residual moves the speed channel, which is what v0.3 forbids
    with_speed = window_errors(val_set, TRUTH, toy_vehicle_module, both.residual)[1.0]
    assert not np.array_equal(with_speed["speed"], nominal["speed"])


def test_masked_output_gets_no_gradient_in_the_loss(toy_vehicle_module):
    """A target on the masked output, however wild, changes nothing."""
    rng = np.random.default_rng(0)
    x = rng.normal(size=(300, 4))
    y = np.stack([rng.normal(size=300) * 50.0, np.tanh(x[:, 0])], axis=1)
    y_clean = y.copy()
    y_clean[:, 0] = 0.0
    mask = np.array([0.0, 1.0])
    w1 = rt.train_numpy(x, y, x, y, 0, epochs=5, loss_mask=mask)
    w2 = rt.train_numpy(x, y_clean, x, y_clean, 0, epochs=5, loss_mask=mask)
    for a, b in zip(w1.arrays(), w2.arrays(), strict=True):
        np.testing.assert_array_equal(a, b)


def test_normalisation_and_divisor_for_yaw_only():
    s = rt.ResidualSamples(
        np.random.default_rng(0).normal(size=(50, 4)),
        np.ones((50, 2)) * [2.0, 3.0] + 0.1 * np.random.default_rng(1).normal(size=(50, 2)),
    )
    _, _, scale = rt.normalisation(s, "standardise", yaw_only=True)
    assert scale[0] == 0.0 and scale[1] > 0.0
    np.testing.assert_array_equal(rt.target_scale(scale), [1.0, scale[1]])
    _, _, none_scale = rt.normalisation(s, "none", yaw_only=True)
    np.testing.assert_array_equal(none_scale, [0.0, 1.0])
    assert rt.loss_mask_of(YAW_ONLY).tolist() == [0.0, 1.0] and rt.loss_mask_of(CFG) is None
