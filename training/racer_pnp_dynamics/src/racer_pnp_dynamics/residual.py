"""Training of the residual MLP (4-32-32-2, tanh) on what the fitted nominal model gets wrong.

Targets per scan interval of real training data (odometry bags only): the extra dv/dt (m/s^2)
and yaw rate (rad/s) that would have made the nominal model's two steps match the measured wheel
speed change and the ICP yaw change. Inputs `[v, delta, speed_cmd, steering_cmd]` with `delta`
taken from the nominal model's own steering state. Inputs are standardised, outputs scaled.
AdamW weight decay, early stopping on validation, three seeds.

SHIPPING RULE, fixed before any number was looked at (claude brief 03): the residual is enabled
only if it lowers the held-out 1.0 s multi-step error by at least 5 percent relative on BOTH
wheel speed and yaw, for the median seed. Concretely `decide` takes each seed's pair of relative
improvements, takes the median over seeds of each of the two, and enables only if both medians
are at least `MIN_RELATIVE_IMPROVEMENT`; the seed shipped is the one whose worse improvement is the
median of the seeds' worse improvements. Otherwise zeros and `enabled: false`.
"""

from __future__ import annotations

import logging
from collections.abc import Callable, Sequence
from dataclasses import dataclass

import numpy as np

from .dataset import Segment
from .model import (
    DT_S,
    RESIDUAL_LAYERS,
    MlpWeights,
    Nominal,
    Residual,
    rollout,
    zero_weights,
)
from .vehicle import Vehicle
from .windows import INTERVAL_N, WARM_N, warm_state, yaw_data

LOG = logging.getLogger(__name__)

MIN_RELATIVE_IMPROVEMENT = 0.05
SEEDS = (0, 1, 2)


@dataclass(frozen=True)
class ResidualSamples:
    x: np.ndarray  # (N, 4) v, delta, speed_cmd, steering_cmd
    y: np.ndarray  # (N, 2) dv/dt error (m/s^2), yaw rate error (rad/s)

    def __len__(self) -> int:
        return len(self.x)


def build_samples(
    segments: Sequence[Segment], params: Nominal, vehicle: Vehicle
) -> ResidualSamples:
    """Residual samples from the odometry segments' good scan intervals."""
    odom = [s for s in segments if s.run.has_odom]
    data = yaw_data(odom, params.command_delay_s)
    if len(data) == 0:
        return ResidualSamples(np.zeros((0, 4)), np.zeros((0, 2)))
    state = warm_state(params, vehicle, data.cmds, data.v_mps)
    x = np.stack(
        [state[:, 3], state[:, 4], data.cmds[:, WARM_N, 0], data.cmds[:, WARM_N, 1]], axis=1
    )
    nominal = rollout(state, data.cmds[:, WARM_N : WARM_N + INTERVAL_N, :], params, vehicle)[:, -1]
    interval_s = INTERVAL_N * DT_S
    dv_error = ((data.v_end_mps - state[:, 3]) - (nominal[:, 3] - state[:, 3])) / interval_s
    yaw_error = (data.dyaw_rad - nominal[:, 2]) / interval_s
    return ResidualSamples(x, np.stack([dv_error, yaw_error], axis=1))


MIN_VAL_SAMPLES = 200
FALLBACK_VAL_FRACTION = 0.2


def early_stopping_split(
    train: ResidualSamples, val: ResidualSamples
) -> tuple[ResidualSamples, ResidualSamples, str]:
    """Samples to train on and to early stop on. The validation split is used when it is big
    enough; the 2026-11-11 bag has no driving in its validation tenth, so otherwise the last
    FALLBACK_VAL_FRACTION of the training samples (the latest segments in time, a blocked
    split) stand in for it."""
    if len(val) >= MIN_VAL_SAMPLES:
        return train, val, "validation split"
    cut = int(len(train) * (1.0 - FALLBACK_VAL_FRACTION))
    return (
        ResidualSamples(train.x[:cut], train.y[:cut]),
        ResidualSamples(train.x[cut:], train.y[cut:]),
        f"last {FALLBACK_VAL_FRACTION:.0%} of the training samples (validation split too small)",
    )


Trainer = Callable[[np.ndarray, np.ndarray, np.ndarray, np.ndarray, int], MlpWeights]


def train_residual(
    train: ResidualSamples,
    val: ResidualSamples,
    seed: int,
    trainer: Trainer | None = None,
) -> Residual:
    """Standardise inputs, scale outputs, train, and wrap the weights as an enabled Residual."""
    mean = train.x.mean(axis=0)
    std = np.maximum(train.x.std(axis=0), 1e-6)
    scale = np.maximum(train.y.std(axis=0), 1e-6)
    trainer = trainer or train_torch
    weights = trainer(
        (train.x - mean) / std, train.y / scale, (val.x - mean) / std, val.y / scale, seed
    )
    return Residual(True, weights, mean, std, scale)


def decide(improvements: Sequence[tuple[float, float]]) -> tuple[bool, int, tuple[float, float]]:
    """The shipping rule. `improvements[i]` is seed i's (speed, yaw) relative improvement of the
    held-out 1.0 s error over the fitted nominal model (positive is better). Returns
    (enabled, index of the median seed, (median speed improvement, median yaw improvement))."""
    arr = np.asarray(improvements, dtype=float)
    medians = (float(np.median(arr[:, 0])), float(np.median(arr[:, 1])))
    worst = arr.min(axis=1)
    order = np.argsort(worst, kind="stable")
    chosen = int(order[len(order) // 2])
    enabled = all(m >= MIN_RELATIVE_IMPROVEMENT for m in medians)
    return enabled, chosen, medians


def relative_improvement(baseline: float, candidate: float) -> float:
    return (baseline - candidate) / baseline


def shipped(candidate: Residual, enabled: bool) -> Residual:
    """The residual that goes in the bundle: the candidate, or a disabled all zero one."""
    if enabled:
        return candidate
    return Residual(False, zero_weights(RESIDUAL_LAYERS), np.zeros(4), np.ones(4), np.ones(2))


def train_torch(  # pragma: no cover - needs torch (dependency group `train`), real data only
    x: np.ndarray,
    y: np.ndarray,
    x_val: np.ndarray,
    y_val: np.ndarray,
    seed: int,
    epochs: int = 600,
    patience: int = 40,
    lr: float = 3e-3,
    weight_decay: float = 1e-2,
    batch: int = 256,
    loss_mask: np.ndarray | None = None,
) -> MlpWeights:
    import torch

    mask = torch.ones(2, dtype=torch.float64)
    if loss_mask is not None:
        mask = torch.as_tensor(np.asarray(loss_mask, dtype=np.float64))
    torch.manual_seed(seed)
    net = torch.nn.Sequential(
        torch.nn.Linear(4, 32), torch.nn.Tanh(),
        torch.nn.Linear(32, 32), torch.nn.Tanh(),
        torch.nn.Linear(32, 2),
    ).double()  # fmt: skip
    opt = torch.optim.AdamW(net.parameters(), lr=lr, weight_decay=weight_decay)
    xt, yt = torch.as_tensor(x), torch.as_tensor(y)
    xv, yv = torch.as_tensor(x_val), torch.as_tensor(y_val)
    gen = torch.Generator().manual_seed(seed)
    best, best_state, bad = float("inf"), None, 0
    for _ in range(epochs):
        perm = torch.randperm(len(xt), generator=gen)
        for i in range(0, len(xt), batch):
            idx = perm[i : i + batch]
            opt.zero_grad()
            loss = torch.sum(((net(xt[idx]) - yt[idx]) * mask) ** 2) / (len(idx) * mask.sum())
            loss.backward()
            opt.step()
        with torch.no_grad():
            val_loss = float(torch.sum(((net(xv) - yv) * mask) ** 2) / (len(xv) * mask.sum()))
        if val_loss < best - 1e-6:
            best, bad = val_loss, 0
            best_state = {k: v.clone() for k, v in net.state_dict().items()}
        else:
            bad += 1
            if bad >= patience:
                break
    assert best_state is not None
    net.load_state_dict(best_state)
    p = [t.detach().numpy() for t in net.parameters()]
    return MlpWeights(p[0], p[1], p[2], p[3], p[4], p[5])
