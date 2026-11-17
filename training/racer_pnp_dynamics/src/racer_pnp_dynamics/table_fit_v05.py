"""T5, the classical alpha and gamma tables of contract 2.1 (pre-registration v0.5 section 3, brief 47).

Each table is fitted against its OWN measured target, with the MEASURED wheel speed and the base
nominal's steering state as inputs (never the model's predicted wheel speed, the fault of T4):

    window       0.5 s, a start every 0.25 s (the gate's window builder, `evaluate.eval_set` at
                 horizon 0.5 s); a window with any wheel speed sample under 0.30 m/s is not used
    alpha        ground distance of the window (T2, the sum of the ICP step lengths, the
                 quantity M-B4 measures)  =  sum_k alpha(s_k, v_k) * v_k * dt
    gamma        yaw change of the window (T2)  =  sum_k (1 + gamma(s_k, v_k)) * w_nom_k * dt,
                 w_nom_k the nominal's yaw rate at the measured speed
    loss         Huber 1.345 on the residual divided by the train RMS of the base nominal's
                 residual (alpha = 1, gamma = 0), rows weighted by sqrt(quality / sum quality),
                 plus lambda times the second differences of the table over the grid
    bounds       alpha in [alpha_min, alpha_max], gamma in [gamma_min, gamma_max]; gamma's centre
                 column (steering 0) is fixed at 0 and is not a parameter

Both are linear in the node values (the bilinear weights depend on the measured speed and the
nominal's steering state only), so each fit is a bounded linear least squares problem with a
Huber loss, solved exactly by scipy's trust region solver.

Support: a window belongs to the node nearest to its mean steering share and its mean measured
wheel speed (nearest on each axis separately, which is nearest on the grid); a node with at least
`min_windows_per_node` windows is supported. An unsupported node copies the nearest supported node
of its own steering column (nearest in speed, the lower node on a tie) or, if the column has none,
the nearest supported node of its speed row on the same side (the lower index on a tie, the centre
column counts as both sides); it is not a free parameter. A grid with nothing to copy from is an
error (`CannotCopy`), reported, never patched.

Smoothness: lambda from the registered list, the gamma table first by Y1 and then the alpha table
(with that gamma table) by A1, both on full open-loop rollouts of the selection set (macro RMSE at
1.0 s, the registered strata). The selection set is never fitted on: `build_t5_windows` accepts
train and validation-of-train segments only, and `fit_*` accept only windows built that way.

Split half: the same fit (same support, same lambda) on the windows of the odd and of the even
10 s blocks of train; the report gives the difference at every supported node.
"""

from __future__ import annotations

from collections.abc import Sequence
from dataclasses import dataclass, replace

import numpy as np
from scipy.optimize import least_squares

from .dataset import Segment
from .evaluate import EvalSet, eval_set, predict
from .fit_data_v04 import window_quality
from .ground_motion import GroundTable, steering_share
from .ground_motion_v21 import (
    CENTRE_NODE,
    ConstantsV05,
    GroundMotionV21,
    window_errors_v21,
    zero_yaw_table,
)
from .icp_motion import SCAN_DT_S
from .model import DT_S, Nominal
from .strata import assign_strata, macro_rmse
from .table_fit import node_weights, second_difference_rows
from .vehicle import Vehicle

TRAIN_ROLES = ("train", "val")  # `val` is the internal validation part of a train segment
MIN_SCALE = 1e-9


class RoleViolation(ValueError):
    """Data that is not train role was offered to a fit."""


class CannotCopy(ValueError):
    """An unsupported node has no supported node to copy from under the registered rule."""


# --- windows ----------------------------------------------------------------------------------


@dataclass(frozen=True)
class T5Windows:
    """The fit windows of train data: per step (K = window / dt) the measured wheel speed, the
    nominal's steering state and yaw rate at that speed, and the two T2 targets."""

    role: str
    t_s: np.ndarray  # (E,) scan stamp of the window start
    segment: np.ndarray  # (E,) segment name
    direction: np.ndarray  # (E,)
    half: np.ndarray  # (E,) int: parity of the 10 s block of the segment the window starts in
    v: np.ndarray  # (E, K) measured wheel speed at each step start
    delta: np.ndarray  # (E, K) nominal steering state at each step start
    w_nom: np.ndarray  # (E, K) nominal yaw rate at the measured speed
    ground_m: np.ndarray  # (E,) T2 ground distance over the window
    dyaw_rad: np.ndarray  # (E,) T2 yaw change over the window
    quality: np.ndarray  # (E,)

    def __len__(self) -> int:
        return len(self.t_s)

    def subset(self, keep: np.ndarray) -> T5Windows:
        return T5Windows(
            self.role, self.t_s[keep], self.segment[keep], self.direction[keep], self.half[keep],
            self.v[keep], self.delta[keep], self.w_nom[keep], self.ground_m[keep],
            self.dyaw_rad[keep], self.quality[keep],
        )  # fmt: skip

    def require_train(self) -> None:
        if self.role != "train":
            raise RoleViolation(f"windows of role {self.role!r} cannot be fitted on")


def mean_share_speed(win: T5Windows, vehicle: Vehicle) -> tuple[np.ndarray, np.ndarray]:
    """(E,) mean steering share (signed) and mean measured wheel speed of each window."""
    return steering_share(win.delta, vehicle).mean(axis=1), win.v.mean(axis=1)


def _half_of(es: EvalSet, segments: Sequence[Segment], block_s: float) -> np.ndarray:
    """Parity of the `block_s` block (counted from the start of its segment) of each window."""
    out = np.full(len(es), -1, dtype=int)
    for seg in segments:
        sel = np.flatnonzero(
            np.array([r is seg.run for r in es.runs])[es.run_idx]
            & (es.t_s >= seg.t0_s - 1e-9)
            & (es.t_s <= seg.t1_s + 1e-9)
            & (out < 0)
        )
        out[sel] = np.floor((es.t_s[sel] - seg.t0_s) / block_s + 1e-9).astype(int) % 2
    if (out < 0).any():
        raise ValueError("a window lies in no segment")
    return out


def build_t5_windows(
    segments: Sequence[Segment], nominal: Nominal, vehicle: Vehicle, c: ConstantsV05
) -> T5Windows:
    """Fit windows of train-role segments. Any other role raises `RoleViolation`."""
    bad = sorted({s.split for s in segments if s.split not in TRAIN_ROLES})
    if bad:
        raise RoleViolation(f"segments of role {bad} cannot be fitted on (train data only)")
    odom = [s for s in segments if s.run.has_odom and s.motion is not None]
    es = eval_set(odom, horizon_s=c.window_length_s)
    n = len(es)
    k = round(c.window_length_s / DT_S)
    if n == 0:
        z = np.zeros((0, k))
        return T5Windows(
            "train", np.zeros(0), np.array([], dtype=str), np.array([], dtype=str),
            np.zeros(0, int), z, z.copy(), z.copy(), np.zeros(0), np.zeros(0), np.zeros(0),
        )  # fmt: skip
    # the nominal's steering state: it does not depend on the speed, so the contract 1 rollout
    # gives it whatever speed the rollout used
    states = predict(es.data(nominal.command_delay_s), nominal, vehicle)
    delta = states[:, :k, 4]
    v = es.v_true[:, :k]  # the MEASURED wheel speed
    w_nom = v * np.tan(delta) / vehicle.wheelbase_m / (1.0 + nominal.understeer_s2_per_m2 * v * v)
    n_iv = round(c.window_length_s / SCAN_DT_S)
    steps = np.diff(es.poses[:, : n_iv + 1, :2], axis=1)
    ground = np.hypot(steps[..., 0], steps[..., 1]).sum(axis=1)
    dyaw = es.poses[:, n_iv, 2] - es.poses[:, 0, 2]
    quality = window_quality(es, odom, c.window_length_s)
    keep = es.v_true.min(axis=1) >= c.min_wheel_speed_mps  # any sample under 0.30 m/s: not used
    half = _half_of(es, odom, c.split_half_block_s)
    return T5Windows(
        "train", es.t_s, es.segment, es.direction, half, v, delta, w_nom, ground, dyaw, quality
    ).subset(keep)


# --- support and copies -----------------------------------------------------------------------


def cell_of(c: ConstantsV05, share: np.ndarray, speed: np.ndarray) -> tuple[np.ndarray, np.ndarray]:
    """Index of the nearest steering node and of the nearest speed node (ties: the lower node)."""
    s_nodes, v_nodes = np.array(c.steering_nodes), np.array(c.speed_nodes_mps)
    i = np.argmin(np.abs(np.asarray(share)[..., None] - s_nodes), axis=-1)
    j = np.argmin(np.abs(np.asarray(speed)[..., None] - v_nodes), axis=-1)
    return i, j


def support_counts(c: ConstantsV05, win: T5Windows, vehicle: Vehicle) -> np.ndarray:
    """(steering nodes, speed nodes) windows whose cell is the node."""
    share, speed = mean_share_speed(win, vehicle)
    i, j = cell_of(c, share, speed)
    out = np.zeros((len(c.steering_nodes), len(c.speed_nodes_mps)), dtype=int)
    np.add.at(out, (i, j), 1)
    return out


def copy_map(c: ConstantsV05, supported: np.ndarray) -> np.ndarray:
    """(nodes,) the node each node copies: itself when supported, else the nearest supported node
    of its steering column (nearest in speed, lower on a tie), else the nearest supported node of
    its speed row on the same side (the centre column is on both sides). Raises `CannotCopy`."""
    s_nodes, v_nodes = np.array(c.steering_nodes), np.array(c.speed_nodes_mps)
    n_s, n_v = supported.shape
    tied = np.arange(n_s * n_v)
    for i in range(n_s):
        for j in range(n_v):
            if supported[i, j]:
                continue
            col = [jj for jj in range(n_v) if supported[i, jj]]
            if col:
                best = min(col, key=lambda jj: (round(abs(v_nodes[jj] - v_nodes[j]), 9), jj))
                tied[i * n_v + j] = i * n_v + best
                continue
            same = [
                ii
                for ii in range(n_s)
                if supported[ii, j]
                and (
                    s_nodes[i] == CENTRE_NODE
                    or s_nodes[ii] == CENTRE_NODE
                    or np.sign(s_nodes[ii]) == np.sign(s_nodes[i])
                )
            ]
            if not same:
                raise CannotCopy(
                    f"node (steering {s_nodes[i]:+g}, speed {v_nodes[j]:g}) is unsupported, its "
                    "column has no supported node and neither has its row on the same side"
                )
            best_i = min(same, key=lambda ii: (round(abs(s_nodes[ii] - s_nodes[i]), 9), ii))
            tied[i * n_v + j] = best_i * n_v + j
    return tied


# --- the linear problems ----------------------------------------------------------------------


@dataclass(frozen=True)
class Problem:
    """pred = base + a @ theta (theta: node values), one row per window."""

    a: np.ndarray  # (E, nodes)
    base: np.ndarray  # (E,)
    target: np.ndarray  # (E,)
    quality: np.ndarray  # (E,)
    scale: float  # train RMS of the base nominal's residual


def _step_weights(c: ConstantsV05, win: T5Windows, vehicle: Vehicle) -> np.ndarray:
    return node_weights(c, win.delta, win.v, vehicle)  # (E, K, nodes)


def alpha_problem(
    c: ConstantsV05, win: T5Windows, vehicle: Vehicle, scale: float | None = None
) -> Problem:
    w = _step_weights(c, win, vehicle)
    a = np.einsum("ek,ekn->en", win.v * DT_S, w)
    base = np.zeros(len(win))
    nominal_residual = win.ground_m - (win.v * DT_S).sum(axis=1)  # alpha = 1
    return Problem(a, base, win.ground_m, win.quality, _scale(nominal_residual, scale))


def gamma_problem(
    c: ConstantsV05, win: T5Windows, vehicle: Vehicle, scale: float | None = None
) -> Problem:
    w = _step_weights(c, win, vehicle)
    a = np.einsum("ek,ekn->en", win.w_nom * DT_S, w)
    base = (win.w_nom * DT_S).sum(axis=1)
    nominal_residual = win.dyaw_rad - base  # gamma = 0
    return Problem(a, base, win.dyaw_rad, win.quality, _scale(nominal_residual, scale))


def _scale(residual: np.ndarray, given: float | None) -> float:
    if given is not None:
        return given
    return max(float(np.sqrt(np.mean(residual**2))) if len(residual) else 0.0, MIN_SCALE)


def free_map(tied: np.ndarray, fixed: np.ndarray | None = None) -> tuple[np.ndarray, np.ndarray]:
    """(nodes, free) matrix m with theta = m @ free, and the node index of each free parameter.
    A copied node takes the value of its source; a fixed node (value 0), and a node that copies a
    fixed node, has an all zero row and is no parameter."""
    n = len(tied)
    fixed = np.zeros(n, bool) if fixed is None else fixed
    zero = fixed | fixed[tied]
    free_nodes = np.array([f for f in range(n) if tied[f] == f and not zero[f]], dtype=int)
    m = np.zeros((n, len(free_nodes)))
    for col, f in enumerate(free_nodes):
        m[(tied == f) & ~zero, col] = 1.0
    return m, free_nodes


def solve(
    p: Problem,
    c: ConstantsV05,
    lam: float,
    m: np.ndarray,
    bounds: tuple[float, float],
    start: float,
    shape: tuple[int, int],
) -> np.ndarray:
    """Node values (nodes,) for one smoothness weight."""
    if m.shape[1] == 0:
        return np.zeros(m.shape[0])
    q = p.quality / p.quality.sum()
    sw = np.sqrt(q)
    a_eff = p.a @ m
    pen = second_difference_rows(shape) @ m * lam
    a_mat = a_eff / p.scale * sw[:, None]

    def fun(theta: np.ndarray) -> np.ndarray:
        r = (p.base + a_eff @ theta - p.target) / p.scale * sw
        return np.concatenate([r, pen @ theta])

    def jac(_: np.ndarray) -> np.ndarray:
        return np.vstack([a_mat, pen])

    sol = least_squares(
        fun, np.full(m.shape[1], start), jac=jac, bounds=bounds, loss="huber",
        f_scale=c.huber_delta, xtol=1e-10, ftol=1e-10, gtol=1e-10, max_nfev=300,
    )  # fmt: skip
    return m @ sol.x


# --- the fit ----------------------------------------------------------------------------------


@dataclass(frozen=True)
class Scores:
    """Macro RMSE at 1.0 s on a window set (the registered strata and merging)."""

    p1: float
    a1: float
    c1: float
    y1: float
    s1: float


def score(es: EvalSet, nominal: Nominal, vehicle: Vehicle, gm: GroundMotionV21) -> Scores:
    strata = assign_strata(es.direction, es.start_speed)
    e = window_errors_v21(es, nominal, vehicle, gm, (1.0,))[1.0]
    return Scores(
        macro_rmse(e["pos"], strata), macro_rmse(e["along"], strata),
        macro_rmse(e["across"], strata), macro_rmse(e["yaw"], strata),
        macro_rmse(e["speed"], strata),
    )  # fmt: skip


@dataclass
class T5Fit:
    constants: ConstantsV05
    alpha: GroundTable
    gamma: GroundTable
    lam_alpha: float
    lam_gamma: float
    support: np.ndarray  # (S, V) windows per node
    supported: np.ndarray  # (S, V) bool
    tied: np.ndarray  # (nodes,) node each node copies (itself when supported)
    fixed: np.ndarray  # (S, V) bool: gamma nodes fixed at 0 (the centre column)
    on_bound_alpha: list[str]
    on_bound_gamma: list[str]
    selection_y1: dict[float, float]  # lambda_gamma -> Y1 on the selection set
    selection_a1: dict[float, float]  # lambda_alpha -> A1 on the selection set
    split_half: dict[str, np.ndarray] | None = None  # alpha_diff, gamma_diff, odd/even windows
    counts: dict[str, int] | None = None

    def groundmotion(self) -> GroundMotionV21:
        return GroundMotionV21(self.constants, self.alpha, self.gamma, None, self.supported)

    def node_name(self, i: int, j: int) -> str:
        c = self.constants
        return f"s{c.steering_nodes[i]:+.4f}/v{c.speed_nodes_mps[j]:g}"

    def rows(self) -> list[dict[str, object]]:
        c = self.constants
        n_v = len(c.speed_nodes_mps)
        out = []
        for i, s in enumerate(c.steering_nodes):
            for j, v in enumerate(c.speed_nodes_mps):
                n = i * n_v + j
                name = self.node_name(i, j)
                sh = self.split_half
                out.append(
                    {
                        "steering_share": s,
                        "speed_mps": v,
                        "alpha": float(self.alpha.values[i, j]),
                        "gamma": float(self.gamma.values[i, j]),
                        "windows": int(self.support[i, j]),
                        "supported": bool(self.supported[i, j]),
                        "tied_to": None if self.tied[n] == n else int(self.tied[n]),
                        "gamma_fixed": bool(self.fixed[i, j]),
                        "alpha_on_bound": name in self.on_bound_alpha,
                        "gamma_on_bound": name in self.on_bound_gamma,
                        "alpha_split_half": None if sh is None else float(sh["alpha_diff"][i, j]),
                        "gamma_split_half": None if sh is None else float(sh["gamma_diff"][i, j]),
                    }
                )
        return out


def _fixed_nodes(c: ConstantsV05) -> np.ndarray:
    n_v = len(c.speed_nodes_mps)
    fixed = np.zeros((len(c.steering_nodes), n_v), dtype=bool)
    fixed[np.isclose(np.array(c.steering_nodes), CENTRE_NODE)] = True
    return fixed


def _on_bound(
    c: ConstantsV05, values: np.ndarray, bounds: tuple[float, float],
    supported: np.ndarray, fixed: np.ndarray,
) -> list[str]:  # fmt: skip
    out = []
    for i in range(values.shape[0]):
        for j in range(values.shape[1]):
            if not supported[i, j] or fixed[i, j]:
                continue
            if min(abs(values[i, j] - bounds[0]), abs(values[i, j] - bounds[1])) <= c.bound_margin:
                out.append(f"s{c.steering_nodes[i]:+.4f}/v{c.speed_nodes_mps[j]:g}")
    return out


def _tables(
    c: ConstantsV05, alpha: np.ndarray, gamma: np.ndarray
) -> tuple[GroundTable, GroundTable]:
    shape = (len(c.steering_nodes), len(c.speed_nodes_mps))
    s, v = np.array(c.steering_nodes), np.array(c.speed_nodes_mps)
    return GroundTable(s, v, alpha.reshape(shape)), GroundTable(s, v, gamma.reshape(shape))


def fit_alpha_values(
    c: ConstantsV05,
    win: T5Windows,
    vehicle: Vehicle,
    tied: np.ndarray,
    lam: float,
    scale: float | None = None,
) -> np.ndarray:
    """Node values (nodes,) of the alpha table for one smoothness weight."""
    shape = (len(c.steering_nodes), len(c.speed_nodes_mps))
    m, _ = free_map(tied)
    return solve(
        alpha_problem(c, win, vehicle, scale), c, lam, m, (c.alpha_min, c.alpha_max), 1.0, shape
    )


def fit_gamma_values(
    c: ConstantsV05,
    win: T5Windows,
    vehicle: Vehicle,
    tied: np.ndarray,
    fixed: np.ndarray,
    lam: float,
    scale: float | None = None,
) -> np.ndarray:
    """Node values (nodes,) of the gamma table (centre column 0) for one smoothness weight."""
    shape = (len(c.steering_nodes), len(c.speed_nodes_mps))
    m, _ = free_map(tied, fixed.reshape(-1))
    return solve(
        gamma_problem(c, win, vehicle, scale), c, lam, m, (c.gamma_min, c.gamma_max), 0.0, shape
    )


def fit_t5(
    train: T5Windows,
    selection: EvalSet,
    nominal: Nominal,
    vehicle: Vehicle,
    c: ConstantsV05,
    split_half: bool = True,
) -> T5Fit:
    """T5 on `train` (train role windows only), the two smoothness weights by selection Y1 (gamma)
    then A1 (alpha), the split half refit with those weights."""
    train.require_train()
    if len(train) == 0:
        raise ValueError("no train windows")
    support = support_counts(c, train, vehicle)
    supported = support >= c.min_windows_per_node
    tied = copy_map(c, supported)
    fixed = _fixed_nodes(c)
    scales = (alpha_problem(c, train, vehicle).scale, gamma_problem(c, train, vehicle).scale)
    flat_one = np.ones(support.size)
    # gamma first: the yaw channel does not depend on alpha, so Y1 is a function of gamma alone
    y1: dict[float, float] = {}
    gam: dict[float, np.ndarray] = {}
    for lam in c.smoothness_weights:
        gam[lam] = fit_gamma_values(c, train, vehicle, tied, fixed, lam, scales[1])
        a_t, g_t = _tables(c, flat_one, gam[lam])
        y1[lam] = score(selection, nominal, vehicle, GroundMotionV21(c, a_t, g_t)).y1
    lam_g = min(y1, key=lambda k: np.inf if np.isnan(y1[k]) else y1[k])
    # then alpha, with the chosen gamma table in the rollouts
    a1: dict[float, float] = {}
    alp: dict[float, np.ndarray] = {}
    for lam in c.smoothness_weights:
        alp[lam] = fit_alpha_values(c, train, vehicle, tied, lam, scales[0])
        a_t, g_t = _tables(c, alp[lam], gam[lam_g])
        a1[lam] = score(selection, nominal, vehicle, GroundMotionV21(c, a_t, g_t)).a1
    lam_a = min(a1, key=lambda k: np.inf if np.isnan(a1[k]) else a1[k])
    alpha_tab, gamma_tab = _tables(c, alp[lam_a], gam[lam_g])
    fit = T5Fit(
        constants=c,
        alpha=alpha_tab,
        gamma=gamma_tab,
        lam_alpha=lam_a,
        lam_gamma=lam_g,
        support=support,
        supported=supported,
        tied=tied,
        fixed=fixed,
        on_bound_alpha=_on_bound(
            c, alpha_tab.values, (c.alpha_min, c.alpha_max), supported, np.zeros_like(fixed)
        ),
        on_bound_gamma=_on_bound(c, gamma_tab.values, (c.gamma_min, c.gamma_max), supported, fixed),
        selection_y1=y1,
        selection_a1=a1,
        counts={"windows": len(train)},
    )
    if split_half:
        fit.split_half = split_half_differences(fit, train, vehicle, scales)
    return fit


def split_half_differences(
    fit: T5Fit, train: T5Windows, vehicle: Vehicle, scales: tuple[float, float] | None = None
) -> dict[str, np.ndarray]:
    """Refit on the odd and on the even blocks (same support, same weights) and return the
    absolute difference at every node (NaN at nodes that are not supported) and the windows of
    each half per node."""
    train.require_train()
    c = fit.constants
    vals = {}
    wins = {}
    for h in (0, 1):
        sub = train.subset(train.half == h)
        wins[h] = support_counts(c, sub, vehicle) if len(sub) else np.zeros_like(fit.support)
        if len(sub):
            vals[h] = (
                fit_alpha_values(
                    c, sub, vehicle, fit.tied, fit.lam_alpha, None if scales is None else scales[0]
                ),
                fit_gamma_values(
                    c,
                    sub,
                    vehicle,
                    fit.tied,
                    fit.fixed,
                    fit.lam_gamma,
                    None if scales is None else scales[1],
                ),
            )
        else:
            vals[h] = (np.full(fit.support.size, np.nan), np.full(fit.support.size, np.nan))
    shape = fit.support.shape
    out = {}
    for k, name in enumerate(("alpha_diff", "gamma_diff")):
        d = np.abs(vals[0][k] - vals[1][k]).reshape(shape)
        d[~fit.supported] = np.nan
        if name == "gamma_diff":
            d[fit.fixed] = np.nan  # fixed at 0: nothing was fitted
        out[name] = d
    out["windows_even_blocks"] = wins[0]
    out["windows_odd_blocks"] = wins[1]
    return out


def with_windows(fit: T5Fit, **kwargs) -> T5Fit:
    return replace(fit, **kwargs)


def unit_motion(c: ConstantsV05) -> GroundMotionV21:
    """alpha = 1 and gamma = 0 on the grid: contract 2.1 equal to the base nominal."""
    shape = (len(c.steering_nodes), len(c.speed_nodes_mps))
    s, v = np.array(c.steering_nodes), np.array(c.speed_nodes_mps)
    alpha = GroundTable(s, v, np.ones(shape))
    return GroundMotionV21(c, alpha, zero_yaw_table(alpha))
