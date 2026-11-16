"""T4, the classical ground motion table (addendum 1 section 2.3, brief 37b section 5).

The wheel channel and the yaw channel of T4 are those of its base nominal, untouched. Position at a
horizon is the sum over the steps before it of `alpha_i * v_i * dt * (cos yaw_i, sin yaw_i)` with
`v_i`, `yaw_i` and `delta_i` of the CONTRACT 1 rollout (alpha never feeds back into them), so the
position is LINEAR in the table values: bilinear interpolation weights are fixed by `(delta_i, v_i)`.
The fit is therefore a bounded linear least squares problem with a Huber loss, which `scipy` solves
with an exact Jacobian:

    residuals  = (along and cross error at 0.5, 1.0, 1.5 s) / (train RMS under dyn-v0.1)
                 * sqrt(quality / sum quality)             (the heading term does not depend on alpha)
    penalty    = lambda * (second differences of the table along each axis)
    bounds     = [alpha_min, alpha_max] on every node value (so the clamp of the step never acts)

`lambda` is chosen from the pre-registered list by the macro position RMSE at 1.0 s on the
validation windows. A node is supported by a window if the window's 1.5 s trajectory puts at least
`MIN_STEP_WEIGHT` steps of interpolation weight on it; a node with fewer than 30 supporting windows
is tied to its nearest supported node (grid index distance), so unsupported nodes carry no free
parameter. The report holds the table, the support, the ties and the nodes on a bound.
"""

from __future__ import annotations

from dataclasses import dataclass

import numpy as np
from scipy.optimize import least_squares

from .evaluate import EvalSet, predict
from .fit_data_v04 import HORIZONS_S, HUBER_DELTA, FitWindows
from .ground_motion import (
    GroundConstants,
    GroundMotion,
    GroundTable,
    steering_share,
    window_errors_v2,
)
from .icp_motion import SCAN_DT_S
from .model import DT_S, Nominal
from .strata import assign_strata, macro_rmse
from .vehicle import Vehicle

MIN_STEP_WEIGHT = 1.0  # steps of interpolation weight that make a window "support" a node
BOUND_TOL = 1e-6


def _axis_weights(nodes: np.ndarray, x: np.ndarray) -> tuple[np.ndarray, np.ndarray, np.ndarray]:
    xc = np.clip(x, nodes[0], nodes[-1])
    hi = np.clip(np.searchsorted(nodes, xc, side="right"), 1, len(nodes) - 1)
    lo = hi - 1
    return lo, hi, (xc - nodes[lo]) / (nodes[hi] - nodes[lo])


def node_weights(
    c: GroundConstants, delta: np.ndarray, v: np.ndarray, vehicle: Vehicle
) -> np.ndarray:
    """(..., nodes) bilinear weights of the table nodes (steering index major) at (delta, v)."""
    s_nodes, v_nodes = np.array(c.steering_nodes), np.array(c.speed_nodes_mps)
    i0, i1, wi = _axis_weights(s_nodes, steering_share(delta, vehicle))
    j0, j1, wj = _axis_weights(v_nodes, np.asarray(v, dtype=np.float64))
    n_v = len(v_nodes)
    out = np.zeros(delta.shape + (len(s_nodes) * n_v,))
    idx = np.indices(delta.shape)
    for ii, jj, w in (
        (i0, j0, (1 - wi) * (1 - wj)),
        (i0, j1, (1 - wi) * wj),
        (i1, j0, wi * (1 - wj)),
        (i1, j1, wi * wj),
    ):
        np.add.at(out, (*idx, ii * n_v + jj), w)
    return out


@dataclass(frozen=True)
class TableProblem:
    """The linear map from table values to the position at each horizon, for one set of windows."""

    a: np.ndarray  # (E, H, 2, nodes): position (x, y) at horizon h = a @ theta
    truth: np.ndarray  # (E, H, 2)
    support: np.ndarray  # (nodes,) windows supporting each node
    quality: np.ndarray


def build_problem(
    fw: FitWindows,
    nominal: Nominal,
    vehicle: Vehicle,
    c: GroundConstants,
    horizons_s=HORIZONS_S,
) -> TableProblem:
    es = fw.es
    states = predict(es.data(nominal.command_delay_s), nominal, vehicle)  # contract 1 rollout
    k_max = round(max(horizons_s) / DT_S)
    v, yaw, delta = states[:, :k_max, 3], states[:, :k_max, 2], states[:, :k_max, 4]
    w = node_weights(c, delta, v, vehicle)  # (E, steps, nodes)
    u = np.stack([v * np.cos(yaw), v * np.sin(yaw)], axis=-1) * DT_S  # (E, steps, 2)
    a = np.zeros((len(es), len(horizons_s), 2, w.shape[-1]))
    for h, hz in enumerate(horizons_s):
        k = round(hz / DT_S)
        a[:, h] = np.einsum("ekc,ekn->ecn", u[:, :k], w[:, :k])
    truth = np.stack([es.poses[:, round(hz / SCAN_DT_S), :2] for hz in horizons_s], axis=1)
    support = (w.sum(axis=1) >= MIN_STEP_WEIGHT).sum(axis=0)
    return TableProblem(a, truth, support, fw.quality)


def nearest_supported(support: np.ndarray, shape: tuple[int, int], minimum: int) -> np.ndarray:
    """For every node the index of the supported node it is tied to (itself when supported), by
    grid index distance; ties in distance go to the lower index."""
    ok = support >= minimum
    if not ok.any():
        raise ValueError("no table node has enough supporting windows")
    ij = np.array([(i, j) for i in range(shape[0]) for j in range(shape[1])], dtype=float)
    tied = np.arange(len(support))
    for n in np.flatnonzero(~ok):
        d = np.linalg.norm(ij - ij[n], axis=1)
        d[~ok] = np.inf
        tied[n] = int(np.argmin(d))
    return tied


def second_difference_rows(shape: tuple[int, int]) -> np.ndarray:
    """Rows (r, nodes) of the second differences along the steering and the speed axis."""
    n_s, n_v = shape
    rows = []
    for i in range(1, n_s - 1):
        for j in range(n_v):
            r = np.zeros(n_s * n_v)
            r[(i - 1) * n_v + j], r[i * n_v + j], r[(i + 1) * n_v + j] = 1.0, -2.0, 1.0
            rows.append(r)
    for i in range(n_s):
        for j in range(1, n_v - 1):
            r = np.zeros(n_s * n_v)
            r[i * n_v + j - 1], r[i * n_v + j], r[i * n_v + j + 1] = 1.0, -2.0, 1.0
            rows.append(r)
    return np.array(rows)


def solve_table(
    problem: TableProblem,
    scales: dict[str, float],
    c: GroundConstants,
    lam: float,
    tied: np.ndarray,
    start: float = 1.0,
) -> np.ndarray:
    """The table values (nodes,) for one smoothness weight."""
    n = problem.a.shape[-1]
    free = np.unique(tied)
    m = np.zeros((n, len(free)))
    for col, f in enumerate(free):
        m[tied == f, col] = 1.0  # a tied node copies its supported node
    q = problem.quality / problem.quality.sum()
    sw = np.sqrt(q)[:, None]
    a_eff = np.einsum("ehcn,nf->ehcf", problem.a, m)
    scale = np.array([scales["along"], scales["cross"]])
    shape = (len(c.steering_nodes), len(c.speed_nodes_mps))
    pen = second_difference_rows(shape) @ m * lam
    n_data = a_eff.shape[0] * a_eff.shape[1] * 2
    a_mat = (a_eff / scale[None, None, :, None] * sw[:, :, None, None]).reshape(n_data, -1)
    pen_n = pen.shape[0]

    def fun(theta: np.ndarray) -> np.ndarray:
        pred = problem.a @ (m @ theta)
        r = ((pred - problem.truth) / scale * sw[:, :, None]).reshape(-1)
        return np.concatenate([r, pen @ theta])

    def jac(_: np.ndarray) -> np.ndarray:
        return np.vstack([a_mat, pen]) if pen_n else a_mat

    x0 = np.full(len(free), start)
    lo, hi = c.alpha_min, c.alpha_max
    sol = least_squares(
        fun, x0, jac=jac, bounds=(lo, hi), loss="huber", f_scale=HUBER_DELTA, xtol=1e-10,
        ftol=1e-10, gtol=1e-10, max_nfev=200,
    )  # fmt: skip
    return m @ sol.x


def validation_p1(es: EvalSet, nominal: Nominal, vehicle: Vehicle, gm: GroundMotion) -> float:
    """Macro position RMSE at 1.0 s on `es` (the registered strata and merging)."""
    strata = assign_strata(es.direction, es.start_speed)
    err = window_errors_v2(es, nominal, vehicle, gm, (1.0,))[1.0]["pos"]
    return macro_rmse(err, strata)


@dataclass
class T4Fit:
    base: Nominal
    constants: GroundConstants
    table: GroundTable
    smoothness: float
    validation_p1: dict[float, float]  # lambda -> macro P1 on the validation windows
    nominal_validation_p1: float
    support: np.ndarray  # (steering nodes, speed nodes) windows supporting each node
    tied: np.ndarray  # (nodes,) index each node copies
    on_bound: list[str]
    scales: dict[str, float]
    counts: dict[str, int]

    def table_rows(self) -> list[dict[str, object]]:
        n_v = len(self.constants.speed_nodes_mps)
        rows = []
        for i, s in enumerate(self.constants.steering_nodes):
            for j, v in enumerate(self.constants.speed_nodes_mps):
                n = i * n_v + j
                rows.append(
                    {
                        "steering_share": s,
                        "speed_mps": v,
                        "alpha": float(self.table.values[i, j]),
                        "supporting_windows": int(self.support[i, j]),
                        "tied_to": None if self.tied[n] == n else int(self.tied[n]),
                        "on_bound": f"s{s:+.4f}/v{v:g}" in self.on_bound,
                    }
                )
        return rows

    def groundmotion(self) -> GroundMotion:
        return GroundMotion(self.constants, self.table)


def fit_t4(
    train: FitWindows,
    val: FitWindows,
    base: Nominal,
    vehicle: Vehicle,
    c: GroundConstants,
) -> T4Fit:
    """T4 on `base` (dyn-v0.1 or N4): the table by the Huber loss on the train windows, the
    smoothness weight by validation P1."""
    problem = build_problem(train, base, vehicle, c)
    shape = (len(c.steering_nodes), len(c.speed_nodes_mps))
    tied = nearest_supported(problem.support, shape, c.min_windows_per_node)
    proto = GroundTable.constant(c)
    results: dict[float, np.ndarray] = {}
    scores: dict[float, float] = {}
    for lam in c.smoothness_weights:
        theta = solve_table(problem, train.scales, c, lam, tied)
        results[lam] = theta
        table = GroundTable(proto.steering_nodes, proto.speed_nodes_mps, theta.reshape(shape))
        scores[lam] = validation_p1(val.es, base, vehicle, GroundMotion(c, table))
    best = min(scores, key=lambda k: np.inf if np.isnan(scores[k]) else scores[k])
    theta = results[best]
    table = GroundTable(proto.steering_nodes, proto.speed_nodes_mps, theta.reshape(shape))
    supported = problem.support >= c.min_windows_per_node
    on_bound = []
    n_v = shape[1]
    for n in np.flatnonzero(supported):
        if min(abs(theta[n] - c.alpha_min), abs(theta[n] - c.alpha_max)) <= BOUND_TOL:
            on_bound.append(f"s{c.steering_nodes[n // n_v]:+.4f}/v{c.speed_nodes_mps[n % n_v]:g}")
    unit = GroundMotion(c, proto)
    return T4Fit(
        base=base,
        constants=c,
        table=table,
        smoothness=best,
        validation_p1=scores,
        nominal_validation_p1=validation_p1(val.es, base, vehicle, unit),
        support=problem.support.reshape(shape),
        tied=tied,
        on_bound=on_bound,
        scales=train.scales,
        counts={"windows_train": len(train), "windows_val": len(val)},
    )
