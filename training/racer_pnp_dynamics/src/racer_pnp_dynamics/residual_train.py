"""Residual training of dyn-v0.2 (brief 23, section 3.4).

The network is the fixed one of the C++ model: 4-32-32-2, tanh, inputs `[v, delta, speed_cmd,
steering_cmd]`, outputs `[dv/dt, dyaw/dt]`. What may vary, all chosen on the internal validation
part (role `val`, the last 15 percent of each train segment) and never on held-out data:

- the loss: one step targets (what the nominal gets wrong per scan interval, as in v0.1) or the
  multi step rollout error against the wheel speed and the chained ICP pose over 1.5 s windows,
  optionally started from the one step solution;
- input and output normalisation (standardised or none), weight decay, the number of epochs
  (early stopping on the validation loss);
- an output bound. The C++ step has no clamp, so the bound is a property of the weights: each
  output row of the last layer is scaled so that `output_scale * (sum |w| + |b|)` stays under the
  cap (|tanh| <= 1, so the output can never leave it).

Two trainers share the weights format: a small numpy AdamW (`train_numpy`, one step loss, used by
the tests and anywhere torch is missing) and torch (`train_one_step_torch`,
`train_multi_step_torch`, dependency group `train`, imported lazily).
"""

from __future__ import annotations

import logging
from dataclasses import dataclass, replace

import numpy as np

from .evaluate import EvalSet, window_errors
from .model import (
    DT_S,
    RESIDUAL_LAYERS,
    MlpWeights,
    Nominal,
    Residual,
    zero_weights,
)
from .residual import ResidualSamples
from .strata import assign_strata, macro_rmse
from .vehicle import Vehicle
from .windows import warm_state

LOG = logging.getLogger(__name__)

ONE_STEP = "one_step"
MULTI_STEP = "multi_step"
MULTI_STEP_FROM_ONE = "multi_step_from_one_step"
MODES = (ONE_STEP, MULTI_STEP, MULTI_STEP_FROM_ONE)


@dataclass(frozen=True)
class ResidualConfig:
    """One candidate setting. Every field is chosen on validation data."""

    name: str
    mode: str = ONE_STEP
    weight_decay: float = 1e-2
    norm: str = "standardise"  # or "none"
    max_epochs: int = 150
    patience: int = 15
    lr: float = 3e-3
    out_cap: tuple[float, float] | None = None  # bound on |dv/dt| (m/s2) and |dyaw/dt| (rad/s)

    def __post_init__(self) -> None:
        if self.mode not in MODES:
            raise ValueError(f"mode {self.mode!r} not in {MODES}")
        if self.norm not in ("standardise", "none"):
            raise ValueError(f"norm {self.norm!r}")


# The candidate grid. Fixed in code before any held-out number was read; the winner is picked on
# validation P1.
DEFAULT_CANDIDATES = (
    ResidualConfig("one_step_wd1e-2", ONE_STEP, 1e-2),
    ResidualConfig("multi_step_wd1e-3", MULTI_STEP, 1e-3),
    ResidualConfig("multi_step_wd1e-2", MULTI_STEP, 1e-2),
    ResidualConfig("multi_step_from_one_wd1e-2", MULTI_STEP_FROM_ONE, 1e-2),
    ResidualConfig("multi_step_capped_wd1e-2", MULTI_STEP, 1e-2, out_cap=(2.0, 0.5)),
    ResidualConfig("multi_step_nonorm_wd1e-2", MULTI_STEP, 1e-2, norm="none"),
)


@dataclass(frozen=True)
class MultiStepData:
    """Training windows for the rollout loss: start state (yaw and position zero, delta from the
    nominal's steering lag, measured wheel speed), delayed commands, wheel speed and ICP poses."""

    state0: np.ndarray  # (E, 5)
    cmds: np.ndarray  # (E, N, 2)
    v_true: np.ndarray  # (E, N + 1)
    poses: np.ndarray  # (E, J + 1, 3), a pose every scan interval

    def __len__(self) -> int:
        return len(self.state0)


def multi_step_data(es: EvalSet, nominal: Nominal, vehicle: Vehicle) -> MultiStepData:
    data = es.data(nominal.command_delay_s)
    if len(data) == 0:
        n = es.v_true.shape[1] - 1
        return MultiStepData(
            np.zeros((0, 5)), np.zeros((0, n, 2)), np.zeros((0, n + 1)), np.zeros((0, 1, 3))
        )
    state0 = warm_state(nominal, vehicle, data.cmds, data.v_warm)
    state0[:, :3] = 0.0
    n_warm = data.cmds.shape[1] - (data.v_true.shape[1] - 1)
    return MultiStepData(state0, data.cmds[:, n_warm:, :], data.v_true, data.poses)


def normalisation(samples: ResidualSamples, norm: str) -> tuple[np.ndarray, np.ndarray, np.ndarray]:
    """(input mean, input std, output scale). `none`: identity, outputs in physical units."""
    if norm == "none" or len(samples) == 0:
        return np.zeros(4), np.ones(4), np.ones(2)
    return (
        samples.x.mean(axis=0),
        np.maximum(samples.x.std(axis=0), 1e-6),
        np.maximum(samples.y.std(axis=0), 1e-6),
    )


def cap_weights(weights: MlpWeights, cap_normalised: np.ndarray | None) -> MlpWeights:
    """Scale each output row of the last layer so `sum |w| + |b|` is at most the cap of that
    output (normalised units). tanh hidden units are bounded by 1, so the output is too."""
    if cap_normalised is None:
        return weights
    bound = np.abs(weights.w3).sum(axis=1) + np.abs(weights.b3)
    factor = np.minimum(1.0, cap_normalised / np.maximum(bound, 1e-12))
    return replace(weights, w3=weights.w3 * factor[:, None], b3=weights.b3 * factor)


def _init_weights(rng: np.random.Generator, out_scale: float = 1.0) -> MlpWeights:
    arrays: list[np.ndarray] = []
    for k, (n_in, n_out) in enumerate(RESIDUAL_LAYERS):
        bound = np.sqrt(6.0 / (n_in + n_out))
        w = rng.uniform(-bound, bound, (n_out, n_in))
        arrays += [w * (out_scale if k == len(RESIDUAL_LAYERS) - 1 else 1.0), np.zeros(n_out)]
    return MlpWeights(*arrays)


def _forward_cache(w: MlpWeights, x: np.ndarray):
    a1 = np.tanh(x @ w.w1.T + w.b1)
    a2 = np.tanh(a1 @ w.w2.T + w.b2)
    return a1, a2, a2 @ w.w3.T + w.b3


def train_numpy(
    x: np.ndarray,
    y: np.ndarray,
    x_val: np.ndarray,
    y_val: np.ndarray,
    seed: int,
    epochs: int = 300,
    patience: int = 30,
    lr: float = 3e-3,
    weight_decay: float = 1e-2,
    batch: int = 256,
    cap_normalised: np.ndarray | None = None,
) -> MlpWeights:
    """AdamW on the mean squared one step error with early stopping on the validation error.
    Hand written backpropagation of the 4-32-32-2 tanh network, float64."""
    rng = np.random.default_rng(seed)
    w = _init_weights(rng)
    params = list(w.arrays())
    m = [np.zeros_like(p) for p in params]
    v = [np.zeros_like(p) for p in params]
    best, best_params, bad, t = np.inf, [p.copy() for p in params], 0, 0
    beta1, beta2, eps = 0.9, 0.999, 1e-8
    n = len(x)
    for _ in range(epochs):
        perm = rng.permutation(n)
        for i in range(0, n, batch):
            idx = perm[i : i + batch]
            xb, yb = x[idx], y[idx]
            cur = MlpWeights(*params)
            a1, a2, out = _forward_cache(cur, xb)
            d3 = 2.0 * (out - yb) / (len(idx) * yb.shape[1])
            g_w3, g_b3 = d3.T @ a2, d3.sum(axis=0)
            d2 = (d3 @ cur.w3) * (1.0 - a2**2)
            g_w2, g_b2 = d2.T @ a1, d2.sum(axis=0)
            d1 = (d2 @ cur.w2) * (1.0 - a1**2)
            g_w1, g_b1 = d1.T @ xb, d1.sum(axis=0)
            grads = [g_w1, g_b1, g_w2, g_b2, g_w3, g_b3]
            t += 1
            for k, (p, g) in enumerate(zip(params, grads, strict=True)):
                p *= 1.0 - lr * weight_decay
                m[k] = beta1 * m[k] + (1 - beta1) * g
                v[k] = beta2 * v[k] + (1 - beta2) * g * g
                p -= lr * (m[k] / (1 - beta1**t)) / (np.sqrt(v[k] / (1 - beta2**t)) + eps)
            if cap_normalised is not None:
                capped = cap_weights(MlpWeights(*params), cap_normalised)
                params[4][:], params[5][:] = capped.w3, capped.b3
        val_loss = float(np.mean((_forward_cache(MlpWeights(*params), x_val)[2] - y_val) ** 2))
        if val_loss < best - 1e-9:
            best, bad, best_params = val_loss, 0, [p.copy() for p in params]
        else:
            bad += 1
            if bad >= patience:
                break
    return MlpWeights(*best_params)


def validation_p1(
    nominal: Nominal, residual: Residual | None, vehicle: Vehicle, val_set: EvalSet
) -> float:
    """Macro position RMSE at 1.0 s on the validation windows, strata and merges of the
    pre-registration (the quantity the shipping seed and the candidate are chosen by)."""
    if len(val_set) == 0:
        return float("nan")
    strata = assign_strata(val_set.direction, val_set.start_speed)
    err = window_errors(val_set, nominal, vehicle, residual, (1.0,))[1.0]["pos"]
    return macro_rmse(err, strata)


@dataclass(frozen=True)
class Candidate:
    config: ResidualConfig
    seed: int
    residual: Residual
    val_p1_m: float
    epochs_note: str = ""


def one_step_trainer(cfg: ResidualConfig, cap_norm: np.ndarray | None):
    """The one step trainer for `cfg`: torch when it is installed, numpy otherwise."""
    try:
        import torch  # noqa: F401  # pragma: no cover
    except ImportError:
        return _numpy_trainer(cfg, cap_norm)
    return _torch_one_step_trainer(cfg, cap_norm)  # pragma: no cover - needs torch


def _numpy_trainer(cfg: ResidualConfig, cap_norm: np.ndarray | None):
    def trainer(x, y, xv, yv, seed):
        return train_numpy(
            x, y, xv, yv, seed, epochs=cfg.max_epochs * 2, patience=cfg.patience * 2, lr=cfg.lr,
            weight_decay=cfg.weight_decay, cap_normalised=cap_norm,
        )  # fmt: skip

    return trainer


def fit_candidate(
    cfg: ResidualConfig,
    seed: int,
    nominal: Nominal,
    vehicle: Vehicle,
    train: ResidualSamples,
    val: ResidualSamples,
    val_set: EvalSet,
    train_ms: MultiStepData | None = None,
    val_ms: MultiStepData | None = None,
    one_step_trainer_fn=None,
) -> Candidate:
    """Train one candidate (config, seed) on train data only, early stopped on validation data,
    and score it on the validation windows."""
    mean, std, scale = normalisation(train, cfg.norm)
    cap = None if cfg.out_cap is None else np.asarray(cfg.out_cap) / scale
    xs = lambda s: (s.x - mean) / std
    ys = lambda s: s.y / scale
    make = one_step_trainer_fn or one_step_trainer
    weights: MlpWeights | None = None
    if cfg.mode in (ONE_STEP, MULTI_STEP_FROM_ONE):
        weights = make(cfg, cap)(xs(train), ys(train), xs(val), ys(val), seed)
    if cfg.mode in (MULTI_STEP, MULTI_STEP_FROM_ONE):
        if train_ms is None or val_ms is None:
            raise ValueError("multi step training needs MultiStepData")
        weights = train_multi_step_torch(  # pragma: no cover - needs torch
            cfg, seed, nominal, vehicle, train_ms, val_ms, (mean, std, scale), cap, weights
        )
    assert weights is not None
    residual = Residual(True, weights, mean, std, scale)
    return Candidate(cfg, seed, residual, validation_p1(nominal, residual, vehicle, val_set))


def select_candidate(
    results: list[Candidate],
) -> tuple[ResidualConfig, Candidate, dict[str, dict[str, float]]]:
    """Pick the config with the lowest mean validation P1 over its seeds, then the seed with the
    lowest validation P1 within it. Returns (config, shipping candidate, table of validation P1
    per config: mean, min and per seed)."""
    by_cfg: dict[str, list[Candidate]] = {}
    for c in results:
        by_cfg.setdefault(c.config.name, []).append(c)
    table = {
        name: {
            "mean": float(np.mean([c.val_p1_m for c in cs])),
            "min": float(np.min([c.val_p1_m for c in cs])),
            **{f"seed{c.seed}": c.val_p1_m for c in cs},
        }
        for name, cs in by_cfg.items()
    }
    best_name = min(table, key=lambda n: np.inf if np.isnan(table[n]["mean"]) else table[n]["mean"])
    cs = by_cfg[best_name]
    ship = min(cs, key=lambda c: np.inf if np.isnan(c.val_p1_m) else c.val_p1_m)
    return ship.config, ship, table


def disabled_residual() -> Residual:
    return Residual(False, zero_weights(RESIDUAL_LAYERS), np.zeros(4), np.ones(4), np.ones(2))


def _torch_one_step_trainer(cfg: ResidualConfig, cap_norm):  # pragma: no cover - needs torch
    from .residual import train_torch

    def trainer(x, y, xv, yv, seed):
        w = train_torch(
            x, y, xv, yv, seed, epochs=cfg.max_epochs * 4, patience=cfg.patience * 2, lr=cfg.lr,
            weight_decay=cfg.weight_decay,
        )  # fmt: skip
        return cap_weights(w, cap_norm)

    return trainer


def train_multi_step_torch(  # pragma: no cover - needs torch (dependency group `train`)
    cfg: ResidualConfig,
    seed: int,
    nominal: Nominal,
    vehicle: Vehicle,
    train: MultiStepData,
    val: MultiStepData,
    norm: tuple[np.ndarray, np.ndarray, np.ndarray],
    cap_normalised: np.ndarray | None,
    init: MlpWeights | None = None,
    batch: int = 256,
) -> MlpWeights:
    """AdamW on the rollout error of nominal plus residual: wheel speed, ICP yaw and ICP position
    at every scan interval of the 1.5 s windows, each term divided by the nominal's own RMSE
    over the training windows so the three are comparable. Early stopping on the same loss over
    the validation windows."""
    import torch

    torch.manual_seed(seed)
    torch.set_num_threads(2)
    net = torch.nn.Sequential(
        torch.nn.Linear(4, 32), torch.nn.Tanh(),
        torch.nn.Linear(32, 32), torch.nn.Tanh(),
        torch.nn.Linear(32, 2),
    ).double()  # fmt: skip
    if init is not None:
        for layer, (w, b) in zip((net[0], net[2], net[4]), _pairs(init), strict=True):
            layer.weight.data = torch.as_tensor(w.copy())
            layer.bias.data = torch.as_tensor(b.copy())
    else:
        with torch.no_grad():
            net[4].weight.mul_(0.1)
            net[4].bias.zero_()
    mean, std, scale = (torch.as_tensor(a) for a in norm)
    c = {k: float(v) for k, v in nominal.to_dict().items()}

    def rollout(d_state0, d_cmds):
        state = d_state0
        out = [state]
        for k in range(d_cmds.shape[1]):
            state = _torch_step(
                torch, state, d_cmds[:, k, :], c, vehicle, net, mean, std, scale, DT_S
            )
            out.append(state)
        return torch.stack(out, dim=1)

    def to_t(d: MultiStepData):
        return tuple(torch.as_tensor(a) for a in (d.state0, d.cmds, d.v_true, d.poses))

    tr, va = to_t(train), to_t(val)
    with torch.no_grad():
        nom0 = _nominal_sigmas(torch, rollout, tr, net)
    sigmas = nom0

    def loss_of(data, idx=None):
        s0, cm, vt, ps = data
        if idx is not None:
            s0, cm, vt, ps = s0[idx], cm[idx], vt[idx], ps[idx]
        st = rollout(s0, cm)
        return _rollout_loss(torch, st, vt, ps, sigmas)

    opt = torch.optim.AdamW(net.parameters(), lr=cfg.lr, weight_decay=cfg.weight_decay)
    gen = torch.Generator().manual_seed(seed)
    best, best_state, bad = float("inf"), None, 0
    cap_t = None if cap_normalised is None else torch.as_tensor(cap_normalised)
    for _ in range(cfg.max_epochs):
        perm = torch.randperm(len(tr[0]), generator=gen)
        for i in range(0, len(perm), batch):
            opt.zero_grad()
            loss = loss_of(tr, perm[i : i + batch])
            loss.backward()
            opt.step()
            if cap_t is not None:
                with torch.no_grad():
                    bound = net[4].weight.abs().sum(dim=1) + net[4].bias.abs()
                    f = torch.clamp(cap_t / bound.clamp_min(1e-12), max=1.0)
                    net[4].weight.mul_(f[:, None])
                    net[4].bias.mul_(f)
        with torch.no_grad():
            val_loss = float(loss_of(va))
        if val_loss < best - 1e-6:
            best, bad = val_loss, 0
            best_state = {k: v.clone() for k, v in net.state_dict().items()}
        else:
            bad += 1
            if bad >= cfg.patience:
                break
    assert best_state is not None
    net.load_state_dict(best_state)
    p = [t.detach().numpy().copy() for t in net.parameters()]
    return MlpWeights(p[0], p[1], p[2], p[3], p[4], p[5])


def _pairs(w: MlpWeights):  # pragma: no cover - needs torch
    return ((w.w1, w.b1), (w.w2, w.b2), (w.w3, w.b3))


def _torch_step(torch, state, ctrl, c, veh, net, mean, std, scale, dt):  # pragma: no cover
    """model.step in torch, residual included (same equations, float64)."""
    x, y, yaw, v, delta = state.unbind(-1)
    speed_cmd, steer_cmd = ctrl[..., 0], ctrl[..., 1]
    gain = torch.where(steer_cmd >= 0.0, c["steer_gain_left"] * torch.ones_like(steer_cmd),
                       c["steer_gain_right"] * torch.ones_like(steer_cmd))  # fmt: skip
    delta_target = torch.clamp(
        gain * steer_cmd + c["steer_offset_rad"], veh.steer_min_rad, veh.steer_max_rad
    )
    a_d = 1.0 - float(np.exp(-dt / c["steer_tau_s"]))
    rate = veh.steer_rate_max_rad_per_s * dt
    delta_next = delta + torch.clamp(a_d * (delta_target - delta), -rate, rate)
    a_v = 1.0 - float(np.exp(-dt / c["speed_tau_s"]))
    target = torch.where(
        speed_cmd.abs() < c["speed_deadband_mps"],
        torch.zeros_like(speed_cmd),
        c["speed_gain"] * speed_cmd,
    )
    dv = torch.clamp(a_v * (target - v), -c["decel_max_mps2"] * dt, c["accel_max_mps2"] * dt)
    yaw_rate = v * torch.tan(delta) / veh.wheelbase_m / (1.0 + c["understeer_s2_per_m2"] * v * v)
    r = scale * net((torch.stack([v, delta, speed_cmd, steer_cmd], dim=-1) - mean) / std)
    dv = dv + r[..., 0] * dt
    yaw_rate = yaw_rate + r[..., 1]
    return torch.stack(
        [
            x + v * torch.cos(yaw) * dt,
            y + v * torch.sin(yaw) * dt,
            yaw + yaw_rate * dt,
            torch.clamp(v + dv, min=0.0),
            delta_next,
        ],
        dim=-1,
    )


def _rollout_terms(torch, states, v_true, poses):  # pragma: no cover - needs torch
    """Squared errors at every scan interval j = 1..J (step 2j): (speed, yaw, position)."""
    j = poses.shape[1] - 1
    ks = [round(i * 0.1 / DT_S) for i in range(1, j + 1)]
    sp = states[:, ks, 3] - v_true[:, ks]
    yw = states[:, ks, 2] - poses[:, 1:, 2]
    ps = (states[:, ks, 0] - poses[:, 1:, 0]) ** 2 + (states[:, ks, 1] - poses[:, 1:, 1]) ** 2
    return sp**2, yw**2, ps


def _nominal_sigmas(torch, rollout, tr, net):  # pragma: no cover - needs torch
    """Mean squared error of the NOMINAL (the network's output zeroed) over the training windows,
    per term, the normalisers of the loss."""
    saved = [p.detach().clone() for p in net.parameters()]
    for p in net.parameters():
        p.zero_()
    s0, cm, vt, ps = tr
    sp, yw, pp = _rollout_terms(torch, rollout(s0, cm), vt, ps)
    for p, q in zip(net.parameters(), saved, strict=True):
        p.data = q
    return tuple(float(t.mean()) + 1e-12 for t in (sp, yw, pp))


def _rollout_loss(torch, states, v_true, poses, sigmas):  # pragma: no cover - needs torch
    sp, yw, ps = _rollout_terms(torch, states, v_true, poses)
    return sp.mean() / sigmas[0] + yw.mean() / sigmas[1] + ps.mean() / sigmas[2]
