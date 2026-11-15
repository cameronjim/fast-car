"""TL4, the bounded learned correction on top of a frozen T4 table (addendum 1 section 2.3).

The wheel channel is the base nominal's and never sees the network, so the speed state `v_i`, the
steering state `delta_i` and the commands at every step of a window do not depend on the weights:
they are computed once with the contract 1 rollout. The network (4-32-32-2 tanh, inputs
`v, delta, speed command, steering command`) only decides `alpha_i` and `rho_i`:

    alpha_i  = clamp(table_i + d_alpha_max * tanh(m0_i), alpha_min, alpha_max)
    rho_i    = rho_max * tanh(m1_i)
    yaw_i    = sum_{j < i} (1 + rho_j) * r_j * dt          r_j = v tan(delta) / L / (1 + u v^2)
    pos_i    = sum_{j < i} alpha_j * v_j * dt * (cos yaw_j, sin yaw_j)

so a whole window batch is a few cumulative sums in torch. Loss: the Huber loss of
`fit_data_v04` on along, cross and heading at 0.5, 1.0 and 1.5 s, quality weighted. AdamW, early
stopping on the same loss over the validation windows; the config (weight decay) and the shipping
seed are chosen by validation P1. Torch runs on the CPU in float64: the recipe is float64 and the
Apple MPS backend has no float64, so MPS is not used; the device is recorded in the result.
"""

from __future__ import annotations

from collections.abc import Sequence
from dataclasses import dataclass

import numpy as np

from .evaluate import predict
from .fit_data_v04 import HORIZONS_S, HUBER_DELTA, FitWindows
from .ground_motion import GroundConstants, GroundMotion, GroundTable, Learned, table_alpha
from .icp_motion import SCAN_DT_S
from .model import DT_S, MlpWeights, Nominal
from .table_fit import validation_p1
from .vehicle import Vehicle

DEVICE_NOTE = "cpu, float64 (MPS available but has no float64 and the recipe is float64)"


@dataclass(frozen=True)
class GmConfig:
    name: str
    weight_decay: float
    lr: float = 3e-3
    max_epochs: int = 150
    patience: int = 15
    batch: int = 256


# Fixed in code before any held-out number exists; the winner is picked on validation P1.
GM_CONFIGS = (GmConfig("wd1e-3", 1e-3), GmConfig("wd1e-2", 1e-2))


@dataclass(frozen=True)
class Tracks:
    """What does not depend on the network, per window and step (E, K)."""

    v: np.ndarray
    delta: np.ndarray
    speed_cmd: np.ndarray
    steer_cmd: np.ndarray
    rate: np.ndarray  # r = v tan(delta) / L / (1 + u v^2)
    table_alpha: np.ndarray
    truth: np.ndarray  # (E, H, 3) x, y, yaw at the horizons
    quality: np.ndarray


def build_tracks(
    fw: FitWindows,
    nominal: Nominal,
    vehicle: Vehicle,
    table: GroundTable,
    horizons_s: Sequence[float] = HORIZONS_S,
) -> Tracks:
    from .windows import WARM_N

    es = fw.es
    data = es.data(nominal.command_delay_s)
    states = predict(data, nominal, vehicle)  # contract 1: v, delta and yaw rate do not see alpha
    k = round(max(horizons_s) / DT_S)
    v, delta = states[:, :k, 3], states[:, :k, 4]
    cmds = data.cmds[:, WARM_N : WARM_N + k, :]
    rate = v * np.tan(delta) / vehicle.wheelbase_m / (1.0 + nominal.understeer_s2_per_m2 * v * v)
    truth = np.stack([es.poses[:, round(h / SCAN_DT_S), :3] for h in horizons_s], axis=1)
    return Tracks(
        v, delta, cmds[..., 0], cmds[..., 1], rate, table_alpha(table, delta, v, vehicle),
        truth, fw.quality,
    )  # fmt: skip


def inputs_of(tr: Tracks) -> np.ndarray:
    return np.stack([tr.v, tr.delta, tr.speed_cmd, tr.steer_cmd], axis=-1)


def normalisation(tr: Tracks) -> tuple[np.ndarray, np.ndarray]:
    x = inputs_of(tr).reshape(-1, 4)
    return x.mean(axis=0), np.maximum(x.std(axis=0), 1e-6)


def predict_from_tracks(
    tr: Tracks,
    c: GroundConstants,
    learned: Learned | None,
    horizons_s: Sequence[float] = HORIZONS_S,
) -> np.ndarray:
    """(E, H, 3) x, y, yaw at the horizons by the cumulative sums of the module docstring (the
    numpy twin of the torch loss; a test checks it against `ground_motion.predict_v2`)."""
    alpha, rho = tr.table_alpha, np.zeros_like(tr.table_alpha)
    if learned is not None:
        m = learned.forward(inputs_of(tr))
        alpha = np.clip(alpha + c.d_alpha_max * np.tanh(m[..., 0]), c.alpha_min, c.alpha_max)
        rho = c.rho_max * np.tanh(m[..., 1])
    dyaw = (1.0 + rho) * tr.rate * DT_S
    yaw_start = np.cumsum(dyaw, axis=1) - dyaw
    cx = np.cumsum(alpha * tr.v * np.cos(yaw_start) * DT_S, axis=1)
    cy = np.cumsum(alpha * tr.v * np.sin(yaw_start) * DT_S, axis=1)
    cyaw = np.cumsum(dyaw, axis=1)
    steps = [round(h / DT_S) - 1 for h in horizons_s]
    return np.stack([cx[:, steps], cy[:, steps], cyaw[:, steps]], axis=-1)


@dataclass
class GmCandidate:
    config: GmConfig
    seed: int
    learned: Learned
    val_p1_m: float
    epochs: int
    device: str = DEVICE_NOTE


def _pairs(w: MlpWeights):  # pragma: no cover - needs torch
    return ((w.w1, w.b1), (w.w2, w.b2), (w.w3, w.b3))


def train_gm_torch(  # pragma: no cover - needs torch (dependency group `train`)
    cfg: GmConfig,
    seed: int,
    c: GroundConstants,
    train: Tracks,
    val: Tracks,
    scales: dict[str, float],
    horizons_s: Sequence[float] = HORIZONS_S,
) -> tuple[Learned, int]:
    import torch

    torch.manual_seed(seed)
    torch.set_num_threads(2)
    net = torch.nn.Sequential(
        torch.nn.Linear(4, 32), torch.nn.Tanh(),
        torch.nn.Linear(32, 32), torch.nn.Tanh(),
        torch.nn.Linear(32, 2),
    ).double()  # fmt: skip
    with torch.no_grad():
        net[4].weight.mul_(0.1)
        net[4].bias.zero_()
    mean_np, std_np = normalisation(train)
    mean, std = torch.as_tensor(mean_np), torch.as_tensor(std_np)
    steps = [round(h / DT_S) for h in horizons_s]
    sc = torch.as_tensor([scales["along"], scales["cross"], scales["heading"]])

    def tt(a):
        return torch.as_tensor(np.ascontiguousarray(a))

    def pack(tr: Tracks):
        return tuple(
            tt(a) for a in (inputs_of(tr), tr.v, tr.rate, tr.table_alpha, tr.truth, tr.quality)
        )

    def loss_of(data, idx=None):
        x, v, rate, ta, truth, q = data
        if idx is not None:
            x, v, rate, ta, truth, q = x[idx], v[idx], rate[idx], ta[idx], truth[idx], q[idx]
        m = net((x - mean) / std)
        alpha = torch.clamp(ta + c.d_alpha_max * torch.tanh(m[..., 0]), c.alpha_min, c.alpha_max)
        rho = c.rho_max * torch.tanh(m[..., 1])
        dyaw = (1.0 + rho) * rate * DT_S
        yaw_start = torch.cumsum(dyaw, dim=1) - dyaw  # yaw at the start of each step
        dx = alpha * v * torch.cos(yaw_start) * DT_S
        dy = alpha * v * torch.sin(yaw_start) * DT_S
        cx, cy, cyaw = torch.cumsum(dx, 1), torch.cumsum(dy, 1), torch.cumsum(dyaw, 1)
        pred = torch.stack(
            [torch.stack([cx[:, k - 1], cy[:, k - 1], cyaw[:, k - 1]], dim=-1) for k in steps],
            dim=1,
        )
        r = (pred - truth) / sc
        a = r.abs()
        hub = torch.where(a <= HUBER_DELTA, 0.5 * r * r, HUBER_DELTA * (a - 0.5 * HUBER_DELTA))
        per_window = hub.mean(dim=(1, 2)) * 3.0
        return (q * per_window).sum() / q.sum()

    tr_d, va_d = pack(train), pack(val)
    opt = torch.optim.AdamW(net.parameters(), lr=cfg.lr, weight_decay=cfg.weight_decay)
    gen = torch.Generator().manual_seed(seed)
    best, best_state, bad, epochs = float("inf"), None, 0, 0
    for epochs in range(1, cfg.max_epochs + 1):
        perm = torch.randperm(len(tr_d[0]), generator=gen)
        for i in range(0, len(perm), cfg.batch):
            opt.zero_grad()
            loss_of(tr_d, perm[i : i + cfg.batch]).backward()
            opt.step()
        with torch.no_grad():
            val_loss = float(loss_of(va_d))
        if val_loss < best - 1e-7:
            best, bad = val_loss, 0
            best_state = {k: t.clone() for k, t in net.state_dict().items()}
        else:
            bad += 1
            if bad >= cfg.patience:
                break
    assert best_state is not None
    net.load_state_dict(best_state)
    p = [t.detach().numpy().copy() for t in net.parameters()]
    return Learned(MlpWeights(*p), mean_np, std_np), epochs


def fit_tl4(  # pragma: no cover - needs torch (dependency group `train`)
    train: FitWindows,
    val: FitWindows,
    base: Nominal,
    vehicle: Vehicle,
    c: GroundConstants,
    table: GroundTable,
    configs: Sequence[GmConfig] = GM_CONFIGS,
    seeds: Sequence[int] | None = None,
    trainer=None,
) -> tuple[list[GmCandidate], GmCandidate, dict[str, dict[str, float]]]:
    """All (config, seed) candidates on `table`, the shipping one chosen as in the residual
    pipeline: the config with the lowest mean validation P1 over its seeds, then its best seed."""
    trainer = trainer or train_gm_torch
    tr_tracks = build_tracks(train, base, vehicle, table)
    va_tracks = build_tracks(val, base, vehicle, table)
    candidates: list[GmCandidate] = []
    for cfg in configs:
        for seed in seeds if seeds is not None else c.seeds:
            learned, epochs = trainer(cfg, seed, c, tr_tracks, va_tracks, train.scales)
            p1 = validation_p1(val.es, base, vehicle, GroundMotion(c, table, learned))
            candidates.append(GmCandidate(cfg, seed, learned, p1, epochs))
    return (candidates, *select(candidates))


def select(
    candidates: Sequence[GmCandidate],
) -> tuple[GmCandidate, dict[str, dict[str, float]]]:
    """(shipping candidate, table of validation P1 per config: mean, min and per seed)."""
    by_cfg: dict[str, list[GmCandidate]] = {}
    for cand in candidates:
        by_cfg.setdefault(cand.config.name, []).append(cand)
    table = {
        n: {
            "mean": float(np.mean([x.val_p1_m for x in cs])),
            "min": float(np.min([x.val_p1_m for x in cs])),
            **{f"seed{x.seed}": x.val_p1_m for x in cs},
        }
        for n, cs in by_cfg.items()
    }
    best = min(table, key=lambda n: np.inf if np.isnan(table[n]["mean"]) else table[n]["mean"])
    ship = min(by_cfg[best], key=lambda x: np.inf if np.isnan(x.val_p1_m) else x.val_p1_m)
    return ship, table
