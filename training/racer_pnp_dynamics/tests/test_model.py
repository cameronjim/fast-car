"""L1 for model.py: the step against hand-computed numbers, signs, clamps, the residual."""

from __future__ import annotations

import math

import numpy as np
import pytest
from racer_pnp_dynamics.model import (
    MlpWeights,
    Residual,
    mlp_forward,
    pack_weights,
    rollout,
    step,
    unpack_weights,
    zero_weights,
)

A_D = 1.0 - math.exp(-0.05 / 0.10)  # 0.3934693403
A_V = 1.0 - math.exp(-0.05 / 0.25)  # 0.1812692469


def test_straight_command_one_step(toy_vehicle, toy_nominal):
    out = step([0.0, 0.0, 0.0, 0.5, 0.0], [0.5, 0.0], toy_nominal, toy_vehicle)
    np.testing.assert_allclose(out, [0.025, 0.0, 0.0, 0.5, 0.0], atol=1e-15)


def test_left_command_one_step(toy_vehicle, toy_nominal):
    out = step([0.0, 0.0, 0.0, 0.5, 0.1], [0.5, 0.2], toy_nominal, toy_vehicle)
    # delta_next = 0.1 + 0.3934693403 * 0.1, yaw_rate = 0.5 * tan(0.1) / 0.5
    np.testing.assert_allclose(out[4], 0.1393469340, atol=1e-9)
    np.testing.assert_allclose(out[2], 0.05 * 0.1003346721, atol=1e-9)
    np.testing.assert_allclose(out[:2], [0.025, 0.0], atol=1e-15)
    assert out[3] == pytest.approx(0.5)


def test_right_command_one_step(toy_vehicle, toy_nominal):
    out = step([0.0, 0.0, 0.0, 0.5, -0.1], [0.5, -0.2], toy_nominal, toy_vehicle)
    np.testing.assert_allclose(out[4], -0.1393469340, atol=1e-9)
    np.testing.assert_allclose(out[2], -0.05 * 0.1003346721, atol=1e-9)


def test_positive_steering_gives_positive_yaw_and_y_drift(toy_vehicle, toy_nominal):
    controls = np.tile([0.5, 0.2], (40, 1))
    states = rollout([0.0, 0.0, 0.0, 0.5, 0.0], controls, toy_nominal, toy_vehicle)
    assert states[-1, 2] > 0.1 and states[-1, 1] > 0.0 and states[-1, 4] > 0.0
    mirrored = rollout([0.0, 0.0, 0.0, 0.5, 0.0], controls * [1, -1], toy_nominal, toy_vehicle)
    np.testing.assert_allclose(mirrored[:, [1, 2, 4]], -states[:, [1, 2, 4]], atol=1e-12)


def test_left_gain_for_positive_and_right_gain_for_negative(toy_vehicle, toy_nominal):
    p = toy_nominal.replace(steer_gain_left=1.2, steer_gain_right=0.8)
    left = step([0, 0, 0, 0.0, 0.0], [0.0, 0.1], p, toy_vehicle)
    right = step([0, 0, 0, 0.0, 0.0], [0.0, -0.1], p, toy_vehicle)
    assert left[4] == pytest.approx(A_D * 0.12)
    assert right[4] == pytest.approx(-A_D * 0.08)
    zero = step([0, 0, 0, 0.0, 0.0], [0.0, 0.0], p.replace(steer_offset_rad=0.01), toy_vehicle)
    assert zero[4] == pytest.approx(A_D * 0.01)  # 0 counts as a left command


def test_steering_limit_clamps_the_target(toy_vehicle, toy_nominal):
    out = step([0, 0, 0, 0.0, 0.39], [0.0, 1.0], toy_nominal, toy_vehicle)
    assert out[4] == pytest.approx(0.39 + A_D * (0.4 - 0.39))
    out = step([0, 0, 0, 0.0, -0.39], [0.0, -1.0], toy_nominal, toy_vehicle)
    assert out[4] == pytest.approx(-0.39 - A_D * (0.4 - 0.39))


def test_steering_rate_limit(toy_vehicle, toy_nominal):
    out = step([0, 0, 0, 0.0, 0.0], [0.0, 0.4], toy_nominal, toy_vehicle)
    assert A_D * 0.4 > 0.1  # the lag alone would exceed the rate limit
    assert out[4] == pytest.approx(2.0 * 0.05)
    out = step([0, 0, 0, 0.0, 0.0], [0.0, -0.4], toy_nominal, toy_vehicle)
    assert out[4] == pytest.approx(-2.0 * 0.05)


def test_accel_limit_and_decel_limit(toy_vehicle, toy_nominal):
    out = step([0, 0, 0, 0.0, 0.0], [10.0, 0.0], toy_nominal, toy_vehicle)
    assert A_V * 10.0 > 2.0 * 0.05
    assert out[3] == pytest.approx(2.0 * 0.05)
    out = step([0, 0, 0, 1.0, 0.0], [0.0, 0.0], toy_nominal, toy_vehicle)
    assert out[3] == pytest.approx(1.0 - 1.5 * 0.05)


def test_speed_lag_below_the_limits(toy_vehicle, toy_nominal):
    out = step([0, 0, 0, 0.4, 0.0], [0.45, 0.0], toy_nominal, toy_vehicle)
    assert out[3] == pytest.approx(0.4 + A_V * 0.05)  # 0.0091 < accel limit 0.1


def test_speed_never_negative(toy_vehicle, toy_nominal):
    out = step([0, 0, 0, 0.01, 0.0], [-5.0, 0.0], toy_nominal, toy_vehicle)
    assert out[3] == 0.0


def test_speed_gain_and_understeer(toy_vehicle, toy_nominal):
    p = toy_nominal.replace(speed_gain=0.9, understeer_s2_per_m2=1.0)
    out = step([0, 0, 0, 0.5, 0.1], [0.5, 0.1], p, toy_vehicle)
    dv = A_V * (0.9 * 0.5 - 0.5)
    assert out[3] == pytest.approx(0.5 + dv)
    yaw_rate = 0.5 * math.tan(0.1) / 0.5 / (1.0 + 0.25)
    assert out[2] == pytest.approx(0.05 * yaw_rate)


def test_position_uses_the_state_at_the_start_of_the_step(toy_vehicle, toy_nominal):
    out = step([1.0, 2.0, math.pi / 2, 0.4, 0.0], [0.4, 0.0], toy_nominal, toy_vehicle)
    np.testing.assert_allclose(out[:2], [1.0, 2.0 + 0.4 * 0.05], atol=1e-12)


def test_rollout_shapes_and_control_alignment(toy_vehicle, toy_nominal):
    controls = np.array([[0.5, 0.0], [0.5, 0.2], [0.0, 0.0]])
    states = rollout([0, 0, 0, 0.5, 0.0], controls, toy_nominal, toy_vehicle)
    assert states.shape == (4, 5)
    np.testing.assert_allclose(
        states[1], step([0, 0, 0, 0.5, 0.0], controls[0], toy_nominal, toy_vehicle)
    )
    batch = rollout(np.zeros((3, 5)), np.stack([controls] * 3), toy_nominal, toy_vehicle)
    assert batch.shape == (3, 4, 5)


def _weights(w1, b1, w2, b2, w3, b3) -> MlpWeights:
    return MlpWeights(*(np.asarray(a, dtype=float) for a in (w1, b1, w2, b2, w3, b3)))


def test_mlp_forward_hand_case():
    # 4 -> 2 -> 2 -> 2: h1 = tanh(x0, x1), h2 = tanh(2 h1_0, h1_1 + 0.5), out = (h2_0 + h2_1, 3 h2_1)
    w = _weights(
        [[1, 0, 0, 0], [0, 1, 0, 0]],
        [0, 0],
        [[2, 0], [0, 1]],
        [0, 0.5],
        [[1, 1], [0, 3]],
        [0.1, -0.1],
    )
    h1 = [math.tanh(0.5), math.tanh(-0.25)]
    h2 = [math.tanh(2 * h1[0]), math.tanh(h1[1] + 0.5)]
    expected = [h2[0] + h2[1] + 0.1, 3 * h2[1] - 0.1]
    np.testing.assert_allclose(mlp_forward(w, np.array([0.5, -0.25, 9.0, 9.0])), expected)


def test_mlp_forward_zero_weights_returns_the_output_bias():
    w = zero_weights()
    b3 = np.array([0.3, -0.2])
    w = MlpWeights(w.w1, w.b1, w.w2, w.b2, w.w3, b3)
    out = mlp_forward(w, np.random.default_rng(0).normal(size=(5, 4)))
    np.testing.assert_allclose(out, np.tile(b3, (5, 1)))


def _constant_residual(b3, enabled=True) -> Residual:
    w = zero_weights()
    return Residual(
        enabled=enabled,
        weights=MlpWeights(w.w1, w.b1, w.w2, w.b2, w.w3, np.asarray(b3, dtype=float)),
        input_mean=np.zeros(4),
        input_std=np.ones(4),
        output_scale=np.array([1.0, 1.0]),
    )


def test_residual_adds_to_dv_and_yaw_rate(toy_vehicle, toy_nominal):
    res = _constant_residual([2.0, 0.5])
    out = step([0, 0, 0, 0.5, 0.0], [0.5, 0.0], toy_nominal, toy_vehicle, res)
    assert out[3] == pytest.approx(0.5 + 2.0 * 0.05)
    assert out[2] == pytest.approx(0.5 * 0.05)


def test_residual_output_scale_and_standardisation():
    w = _weights([[1, 0, 0, 0]], [0], [[1]], [0], [[1], [2]], [0, 0])
    res = Residual(True, w, np.full(4, 0.5), np.full(4, 0.5), np.array([2.0, 3.0]))
    h2 = math.tanh(math.tanh((1.0 - 0.5) / 0.5))
    np.testing.assert_allclose(res.forward(np.ones(4)), [2.0 * h2, 3.0 * 2.0 * h2])


def test_disabled_residual_ignores_the_weights(toy_vehicle, toy_nominal):
    junk = _constant_residual([50.0, 50.0], enabled=False)
    a = step([0, 0, 0, 0.5, 0.1], [0.5, 0.2], toy_nominal, toy_vehicle, junk)
    b = step([0, 0, 0, 0.5, 0.1], [0.5, 0.2], toy_nominal, toy_vehicle, None)
    np.testing.assert_array_equal(a, b)


def test_pack_unpack_round_trip_and_size():
    rng = np.random.default_rng(1)
    w = zero_weights()
    w = MlpWeights(*(rng.normal(size=a.shape) for a in w.arrays()))
    flat = pack_weights(w)
    assert flat.dtype == np.float32 and flat.size == 4 * 32 + 32 + 32 * 32 + 32 + 32 * 2 + 2
    back = unpack_weights(flat)
    for a, b in zip(w.arrays(), back.arrays(), strict=True):
        np.testing.assert_allclose(a, b, atol=1e-6)
    with pytest.raises(ValueError):
        unpack_weights(flat[:-1])
    assert w.layers() == [[4, 32], [32, 32], [32, 2]]


def test_dead_zone_command_below_it_has_zero_target(toy_vehicle, toy_nominal):
    p = toy_nominal.replace(speed_deadband_mps=0.2, speed_gain=0.9)
    from_rest = step([0, 0, 0, 0.0, 0.0], [0.19, 0.0], p, toy_vehicle)
    assert from_rest[3] == 0.0
    rolling = step([0, 0, 0, 0.4, 0.0], [0.19, 0.0], p, toy_vehicle)
    assert rolling[3] == pytest.approx(0.4 - A_V * 0.4)  # target 0, not 0.9 * 0.19


def test_dead_zone_command_at_the_edge_and_above_it_moves(toy_vehicle, toy_nominal):
    p = toy_nominal.replace(speed_deadband_mps=0.2, speed_gain=0.9)
    at_edge = step([0, 0, 0, 0.0, 0.0], [0.2, 0.0], p, toy_vehicle)
    assert at_edge[3] == pytest.approx(A_V * 0.9 * 0.2)
    above = step([0, 0, 0, 0.0, 0.0], [0.5, 0.0], p, toy_vehicle)
    assert above[3] == pytest.approx(A_V * 0.9 * 0.5)


def test_dead_zone_is_symmetric_in_the_sign_of_the_command(toy_vehicle, toy_nominal):
    p = toy_nominal.replace(speed_deadband_mps=0.2)
    inside = step([0, 0, 0, 0.4, 0.0], [-0.1, 0.0], p, toy_vehicle)
    assert inside[3] == pytest.approx(0.4 - A_V * 0.4)
    assert step([0, 0, 0, 0.05, 0.0], [-0.5, 0.0], p, toy_vehicle)[3] == 0.0


def test_zero_dead_zone_is_the_old_model(toy_vehicle, toy_nominal):
    cmds = np.array([[-0.3, 0.1], [0.0, 0.1], [0.05, 0.1], [0.4, 0.1]])
    state = np.tile([0.0, 0.0, 0.0, 0.2, 0.1], (4, 1))
    a = step(state, cmds, toy_nominal.replace(speed_deadband_mps=0.0), toy_vehicle)
    dv = np.clip(
        A_V * (toy_nominal.speed_gain * cmds[:, 0] - 0.2),
        -toy_nominal.decel_max_mps2 * 0.05,
        toy_nominal.accel_max_mps2 * 0.05,
    )
    np.testing.assert_allclose(a[:, 3], np.maximum(0.0, 0.2 + dv), atol=1e-15)


def test_dead_zone_broadcasts_over_batches_and_keeps_nan(toy_vehicle, toy_nominal):
    p = toy_nominal.replace(speed_deadband_mps=0.2)
    state = np.zeros((3, 5))
    out = step(state, np.array([[0.1, 0.0], [0.3, 0.0], [np.nan, 0.0]]), p, toy_vehicle)
    assert out[0, 3] == 0.0 and out[1, 3] > 0.0 and np.isnan(out[2, 3])


def test_dead_zone_does_not_touch_steering(toy_vehicle, toy_nominal):
    a = step([0, 0, 0, 0.3, 0.05], [0.1, 0.2], toy_nominal, toy_vehicle)
    b = step(
        [0, 0, 0, 0.3, 0.05], [0.1, 0.2], toy_nominal.replace(speed_deadband_mps=0.5), toy_vehicle
    )
    assert a[2] == b[2] and a[4] == b[4]
