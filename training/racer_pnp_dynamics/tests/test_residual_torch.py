"""Torch side of the residual training (dependency group `train`, skipped where torch is not
installed, which is the CI environment): the torch step equals model.step including the residual,
and rollout training lowers the multi step loss and the validation P1 of a known residual."""

from __future__ import annotations

import numpy as np
import pytest

torch = pytest.importorskip("torch")

from racer_pnp_dynamics import residual_train as rt
from racer_pnp_dynamics.dataset import by_split
from racer_pnp_dynamics.evaluate import eval_set
from racer_pnp_dynamics.model import MlpWeights, Residual, step
from racer_pnp_dynamics.residual import build_samples
from test_fit import TRUTH
from test_residual_train import SEGS, _segments, known_residual


def _torch_net(weights: MlpWeights):
    net = torch.nn.Sequential(
        torch.nn.Linear(4, 32), torch.nn.Tanh(),
        torch.nn.Linear(32, 32), torch.nn.Tanh(),
        torch.nn.Linear(32, 2),
    ).double()  # fmt: skip
    for layer, (w, b) in zip((net[0], net[2], net[4]), rt._pairs(weights), strict=True):
        layer.weight.data = torch.as_tensor(w.copy())
        layer.bias.data = torch.as_tensor(b.copy())
    return net


def test_the_torch_step_is_the_numpy_step_with_the_residual(toy_vehicle):
    res = known_residual()
    rng = np.random.default_rng(0)
    state = np.column_stack(
        [rng.normal(size=64), rng.normal(size=64), rng.normal(size=64), rng.uniform(0, 1.4, 64),
         rng.uniform(-0.4, 0.4, 64)]
    )  # fmt: skip
    ctrl = np.column_stack([rng.uniform(-0.2, 1.5, 64), rng.uniform(-0.6, 0.6, 64)])
    want = step(state, ctrl, TRUTH, toy_vehicle, res)
    got = rt._torch_step(
        torch, torch.as_tensor(state), torch.as_tensor(ctrl),
        {k: float(v) for k, v in TRUTH.to_dict().items()}, toy_vehicle, _torch_net(res.weights),
        torch.as_tensor(res.input_mean), torch.as_tensor(res.input_std),
        torch.as_tensor(res.output_scale), 0.05,
    ).detach().numpy()  # fmt: skip
    np.testing.assert_allclose(got, want, atol=1e-12)


@pytest.fixture(scope="module")
def segs(toy_vehicle_module):
    return _segments(toy_vehicle_module, known_residual(), yaw_noise=0.02, speed_noise=0.05)


def test_multi_step_training_recovers_a_known_residual(segs, toy_vehicle_module):
    veh = toy_vehicle_module
    train, val = by_split(segs, "train"), by_split(segs, "val")
    cfg = rt.ResidualConfig("ms", rt.MULTI_STEP, 1e-3, max_epochs=25, patience=6, lr=5e-3)
    s_tr, s_va = build_samples(train, TRUTH, veh), build_samples(val, TRUTH, veh)
    val_set = eval_set(val)
    tr_ms = rt.multi_step_data(eval_set(train, 0.1, 0.1), TRUTH, veh)
    va_ms = rt.multi_step_data(eval_set(val, 0.1, 0.1), TRUTH, veh)
    nominal_p1 = rt.validation_p1(TRUTH, None, veh, val_set)
    cand = rt.fit_candidate(cfg, 0, TRUTH, veh, s_tr, s_va, val_set, tr_ms, va_ms)
    assert isinstance(cand.residual, Residual) and cand.residual.enabled
    assert cand.val_p1_m < 0.8 * nominal_p1
    capped = rt.fit_candidate(
        rt.ResidualConfig("cap", rt.MULTI_STEP, 1e-3, max_epochs=3, patience=2, out_cap=(0.3, 0.1)),
        0, TRUTH, veh, s_tr, s_va, val_set, tr_ms, va_ms,
    )  # fmt: skip
    w = capped.residual.weights
    bound = capped.residual.output_scale * (np.abs(w.w3).sum(axis=1) + np.abs(w.b3))
    assert (bound <= np.array([0.3, 0.1]) + 1e-9).all()
    warm = rt.fit_candidate(
        rt.ResidualConfig("warm", rt.MULTI_STEP_FROM_ONE, 1e-3, max_epochs=3, patience=2),
        1, TRUTH, veh, s_tr, s_va, val_set, tr_ms, va_ms,
    )  # fmt: skip
    assert np.isfinite(warm.val_p1_m)


def test_the_one_step_torch_trainer_is_used_when_torch_is_installed(segs, toy_vehicle_module):
    cfg = rt.ResidualConfig("one", rt.ONE_STEP, 1e-3, max_epochs=3, patience=2)
    trainer = rt.one_step_trainer(cfg, None)
    x = np.random.default_rng(0).normal(size=(300, 4))
    y = np.stack([np.tanh(x[:, 0]), 0.5 * x[:, 1]], axis=1)
    w = trainer(x, y, x[:50], y[:50], 0)
    assert isinstance(w, MlpWeights) and w.w1.shape == (32, 4)
    assert SEGS  # the module under test shares its split with the numpy tests
