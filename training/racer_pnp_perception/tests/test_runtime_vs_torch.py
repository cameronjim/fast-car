"""The numpy runtime equals the torch model within 1e-5 (needs torch: the `train` group)."""

from __future__ import annotations

import numpy as np
import pytest

torch = pytest.importorskip("torch")

from racer_pnp_perception import model
from racer_track_perception.runtime import CircConv1dV0


@pytest.mark.torch
def test_numpy_forward_equals_torch_on_random_inputs() -> None:
    torch.manual_seed(0)
    net = model.build_model()
    # Random, not the structured initial bias, so every layer matters.
    with torch.no_grad():
        for p in net.parameters():
            p.add_(torch.randn_like(p) * 0.05)
    net.eval()
    runtime = CircConv1dV0(model.to_npz_arrays(net))
    rng = np.random.default_rng(0)
    for _ in range(20):
        x = rng.uniform(0.0, 1.0, size=(1, 2, 360)).astype(np.float32)
        x[0, 1] = (x[0, 1] > 0.5).astype(np.float32)
        with torch.no_grad():
            want = net(torch.from_numpy(x))[0].numpy()
        got = runtime.forward(x)
        assert np.abs(got - want).max() < 1e-5


@pytest.mark.torch
def test_model_shape_parameters_and_loss() -> None:
    net = model.build_model()
    assert model.count_parameters(net) < 130_000
    x = torch.rand(3, 2, 360)
    y = net(x)
    assert y.shape == (3, 16, 5)
    target = torch.zeros(3, 16, 5)
    loss = model.loss_fn(y, target)
    assert torch.isfinite(loss)
    loss.backward()
    assert all(p.grad is not None for p in net.parameters())
    # A perfect prediction with small sigma has a lower loss than a wrong one.
    good = torch.zeros(3, 16, 5)
    good[..., 4] = -3.0
    bad = good.clone()
    bad[..., 0] = 1.0
    assert model.loss_fn(good, target) < model.loss_fn(bad, target)


def test_layout_helpers_do_not_need_torch() -> None:
    assert model.conv_out_length(360) == 180
    assert model.flat_features() == 32 * 23
