"""Step contract 2: the Python reference (addendum 1 section 2.2, bundle schema 2.0.0).

The wheel channel (steering lag, speed lag, limits, dead zone) is `model.step` of contract 1,
unchanged, with no learned term on it. Position and yaw advance by

    alpha    = clamp(table(delta, v), alpha_min, alpha_max)
    rho      = 0
    if learned: alpha = clamp(alpha + d_alpha_max * tanh(m[0]), alpha_min, alpha_max)
                rho   = rho_max * tanh(m[1])
    v_g      = alpha * v
    yaw_rate = (1 + rho) * v * tan(delta) / L / (1 + understeer * v^2)
    x_next   = x + v_g * cos(yaw) * dt ; y_next = y + v_g * sin(yaw) * dt
    yaw_next = yaw + yaw_rate * dt

`v` and `delta` are the state at the START of the step (as in contract 1), `m` the output of the
4-32-32-2 tanh network on (v, delta, speed command, steering command), standardised with the
bundle's mean and std. Both corrections multiply `v`, so at zero wheel speed ground motion and yaw
rate are zero for any weights. The table is bilinear on the grid of the constants file: steering as
a share of EACH SIDE'S limit (limits from the vehicle binding) by wheel speed, held constant
outside the grid. With the table equal to 1 and no learned block the step is the contract 1 step
bit for bit (alpha = 1, rho = 0).

All constants (`alpha` bounds, `d_alpha_max`, `rho_max`, the grid) come from
`evaluation/prereg/pnp-v0.4-addendum-1.yaml`, block `ground_motion`, through `load_constants`
(which refuses a file with another SHA-256); nothing here is typed in.
"""

from __future__ import annotations

import hashlib
from dataclasses import dataclass
from pathlib import Path

import numpy as np
import yaml

from .evaluate import EvalSet, predict
from .icp_motion import SCAN_DT_S
from .model import DT_S, MlpWeights, Nominal, mlp_forward, step
from .vehicle import Vehicle
from .windows import WARM_N, warm_state

# SHA-256 of evaluation/prereg/pnp-v0.4-addendum-1.yaml as pre-registered (addendum 1 header).
ADDENDUM_SHA256 = "ada5a83675e5c843fb8e121a1190e4458f823b1ed2ff32a34a46c5fb4ff76824"
ADDENDUM_RELATIVE = Path("evaluation") / "prereg" / "pnp-v0.4-addendum-1.yaml"
BUNDLE_SCHEMA_VERSION = "2.0.0"


class AddendumInvalid(ValueError):
    """The addendum constants file is not the pre-registered one."""


@dataclass(frozen=True)
class GroundConstants:
    """The `ground_motion` block of the addendum yaml."""

    alpha_min: float
    alpha_max: float
    steering_nodes: tuple[float, ...]  # share of each side's limit
    speed_nodes_mps: tuple[float, ...]
    min_windows_per_node: int
    smoothness_weights: tuple[float, ...]
    d_alpha_max: float
    rho_max: float
    layers: tuple[tuple[int, int], ...]
    seeds: tuple[int, ...]
    horizons_s: tuple[float, ...]
    t2s_scale: float
    sha256: str = ""


def parse_constants(doc: dict, sha256: str = "") -> GroundConstants:
    g = doc["ground_motion"]
    return GroundConstants(
        alpha_min=float(g["alpha_bounds"][0]),
        alpha_max=float(g["alpha_bounds"][1]),
        steering_nodes=tuple(float(x) for x in g["table"]["steering_nodes_share_of_side_limit"]),
        speed_nodes_mps=tuple(float(x) for x in g["table"]["wheel_speed_nodes_mps"]),
        min_windows_per_node=int(g["table"]["min_windows_per_node"]),
        smoothness_weights=tuple(float(x) for x in g["table"]["smoothness_weights"]),
        d_alpha_max=float(g["learned"]["d_alpha_max"]),
        rho_max=float(g["learned"]["rho_max"]),
        layers=tuple((int(a), int(b)) for a, b in g["learned"]["layers"]),
        seeds=tuple(int(x) for x in g["learned"]["seeds"]),
        horizons_s=tuple(float(x) for x in g["training_windows"]["horizons_s"]),
        t2s_scale=float(g["alt_truth"]["icp_seed_translation_scale"]),
        sha256=sha256,
    )


def load_constants(path: Path, expected_sha256: str = ADDENDUM_SHA256) -> GroundConstants:
    blob = Path(path).read_bytes()
    digest = hashlib.sha256(blob).hexdigest()
    if digest != expected_sha256:
        raise AddendumInvalid(
            f"{path}: SHA-256 {digest} is not the pre-registered {expected_sha256}; the addendum "
            "constants must not change"
        )
    return parse_constants(yaml.safe_load(blob.decode("utf-8")), digest)


@dataclass(frozen=True)
class GroundTable:
    """alpha on the grid: values[i, j] at steering node i and wheel speed node j."""

    steering_nodes: np.ndarray
    speed_nodes_mps: np.ndarray
    values: np.ndarray

    def __post_init__(self) -> None:
        if self.values.shape != (len(self.steering_nodes), len(self.speed_nodes_mps)):
            raise ValueError("table values do not match the grid")

    @classmethod
    def constant(cls, c: GroundConstants, value: float = 1.0) -> GroundTable:
        s, v = np.array(c.steering_nodes), np.array(c.speed_nodes_mps)
        return cls(s, v, np.full((len(s), len(v)), value))


@dataclass(frozen=True)
class Learned:
    """The bounded network: 4-32-32-2 tanh, inputs standardised with mean and std."""

    weights: MlpWeights
    input_mean: np.ndarray
    input_std: np.ndarray

    def forward(self, inputs: np.ndarray) -> np.ndarray:
        return mlp_forward(self.weights, (inputs - self.input_mean) / self.input_std)


@dataclass(frozen=True)
class GroundMotion:
    constants: GroundConstants
    table: GroundTable
    learned: Learned | None = None


def _interp_axis(nodes: np.ndarray, x: np.ndarray) -> tuple[np.ndarray, np.ndarray, np.ndarray]:
    """Lower index, upper index and weight of the upper node, constant outside the grid."""
    xc = np.clip(x, nodes[0], nodes[-1])
    hi = np.clip(np.searchsorted(nodes, xc, side="right"), 1, len(nodes) - 1)
    lo = hi - 1
    w = (xc - nodes[lo]) / (nodes[hi] - nodes[lo])
    return lo, hi, w


def steering_share(delta: np.ndarray, vehicle: Vehicle) -> np.ndarray:
    """Steering angle as a share of the limit of its own side (positive left)."""
    delta = np.asarray(delta, dtype=np.float64)
    return np.where(delta >= 0.0, delta / vehicle.steer_max_rad, delta / -vehicle.steer_min_rad)


def table_alpha(
    table: GroundTable, delta: np.ndarray, v_mps: np.ndarray, vehicle: Vehicle
) -> np.ndarray:
    """Bilinear alpha at (steering, wheel speed). A NaN input gives NaN."""
    share = steering_share(delta, vehicle)
    v = np.asarray(v_mps, dtype=np.float64)
    i0, i1, wi = _interp_axis(table.steering_nodes, share)
    j0, j1, wj = _interp_axis(table.speed_nodes_mps, v)
    t = table.values
    return (
        (1 - wi) * (1 - wj) * t[i0, j0]
        + (1 - wi) * wj * t[i0, j1]
        + wi * (1 - wj) * t[i1, j0]
        + wi * wj * t[i1, j1]
    )


def alpha_rho(
    gm: GroundMotion, state: np.ndarray, control: np.ndarray, vehicle: Vehicle
) -> tuple[np.ndarray, np.ndarray]:
    """alpha and rho of a state and control (the first lines of the contract 2 step)."""
    c = gm.constants
    v, delta = state[..., 3], state[..., 4]
    alpha = np.clip(table_alpha(gm.table, delta, v, vehicle), c.alpha_min, c.alpha_max)
    rho = np.zeros_like(alpha)
    if gm.learned is not None:
        m = gm.learned.forward(np.stack([v, delta, control[..., 0], control[..., 1]], axis=-1))
        alpha = np.clip(alpha + c.d_alpha_max * np.tanh(m[..., 0]), c.alpha_min, c.alpha_max)
        rho = c.rho_max * np.tanh(m[..., 1])
    return alpha, rho


def step_v2(
    state: np.ndarray,
    control: np.ndarray,
    params: Nominal,
    vehicle: Vehicle,
    gm: GroundMotion,
    dt: float = DT_S,
) -> np.ndarray:
    """One step of contract 2. State [x, y, yaw, v, delta], control [speed_cmd, steering_cmd]."""
    state = np.asarray(state, dtype=np.float64)
    control = np.asarray(control, dtype=np.float64)
    wheel = step(state, control, params, vehicle, None, dt)  # the wheel channel, contract 1
    x, y, yaw, v, delta = (state[..., i] for i in range(5))
    alpha, rho = alpha_rho(gm, state, control, vehicle)
    v_g = alpha * v
    yaw_rate = (
        (1.0 + rho)
        * v
        * np.tan(delta)
        / vehicle.wheelbase_m
        / (1.0 + params.understeer_s2_per_m2 * v * v)
    )
    return np.stack(
        [
            x + v_g * np.cos(yaw) * dt,
            y + v_g * np.sin(yaw) * dt,
            yaw + yaw_rate * dt,
            wheel[..., 3],
            wheel[..., 4],
        ],
        axis=-1,
    )


def rollout_v2(
    state0: np.ndarray,
    controls: np.ndarray,
    params: Nominal,
    vehicle: Vehicle,
    gm: GroundMotion,
    dt: float = DT_S,
) -> np.ndarray:
    """States (..., N + 1, 5) for controls (..., N, 2)."""
    controls = np.asarray(controls, dtype=np.float64)
    state = np.broadcast_to(np.asarray(state0, dtype=np.float64), controls.shape[:-2] + (5,)).copy()
    out = [state]
    for i in range(controls.shape[-2]):
        state = step_v2(state, controls[..., i, :], params, vehicle, gm, dt)
        out.append(state)
    return np.stack(out, axis=-2)


def predict_v2(es: EvalSet, params: Nominal, vehicle: Vehicle, gm: GroundMotion) -> np.ndarray:
    """States (E, N + 1, 5) of the windows of `es`: the same warm start (the wheel channel is
    contract 1's) and delayed commands as `evaluate.predict`."""
    data = es.data(params.command_delay_s)
    state0 = warm_state(params, vehicle, data.cmds, data.v_warm, None)
    state0[:, :3] = 0.0
    return rollout_v2(state0, data.cmds[:, WARM_N:, :], params, vehicle, gm)


def window_errors_v2(
    es: EvalSet,
    params: Nominal,
    vehicle: Vehicle,
    gm: GroundMotion,
    horizons_s: tuple[float, ...] = (0.5, 1.0, 1.5),
) -> dict[float, dict[str, np.ndarray]]:
    """Per window error at each horizon, as `evaluate.window_errors` (`speed`, `yaw`, `pos`) plus
    `along` and `across`, the components of the position error in the true heading at the start."""
    states = predict_v2(es, params, vehicle, gm)
    out: dict[float, dict[str, np.ndarray]] = {}
    for h in horizons_s:
        k, j = round(h / DT_S), round(h / SCAN_DT_S)
        dx = states[:, k, 0] - es.poses[:, j, 0]
        dy = states[:, k, 1] - es.poses[:, j, 1]
        out[h] = {
            "speed": states[:, k, 3] - es.v_true[:, k],
            "yaw": states[:, k, 2] - es.poses[:, j, 2],
            "pos": np.hypot(dx, dy),
            "along": dx,
            "across": dy,
        }
    return out


def contract1_window_errors(
    es: EvalSet,
    params: Nominal,
    vehicle: Vehicle,
    residual=None,
    horizons_s: tuple[float, ...] = (0.5, 1.0, 1.5),
) -> dict[float, dict[str, np.ndarray]]:
    """The same fields for a contract 1 model (position is what its one speed state integrates)."""
    states = predict(es.data(params.command_delay_s), params, vehicle, residual)
    out: dict[float, dict[str, np.ndarray]] = {}
    for h in horizons_s:
        k, j = round(h / DT_S), round(h / SCAN_DT_S)
        dx = states[:, k, 0] - es.poses[:, j, 0]
        dy = states[:, k, 1] - es.poses[:, j, 1]
        out[h] = {
            "speed": states[:, k, 3] - es.v_true[:, k],
            "yaw": states[:, k, 2] - es.poses[:, j, 2],
            "pos": np.hypot(dx, dy),
            "along": dx,
            "across": dy,
        }
    return out
