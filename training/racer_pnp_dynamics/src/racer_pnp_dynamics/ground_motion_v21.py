"""Step contract 2.1: the Python reference (pre-registration v0.5 section 2, bundle schema 2.1.0).

Contract 2 (`ground_motion.py`, untouched) with one classical term added, a yaw gain table:

    alpha    = clamp(table_a(delta, v), 0.70, 1.05)
    gamma    = clamp(table_y(delta, v), -0.25, 0.25)
    rho      = 0
    if learned: alpha = clamp(alpha + d_alpha_max * tanh(m[0]), alpha_min, alpha_max)
                rho   = rho_max * tanh(m[1])
    v_g      = alpha * v
    yaw_rate = (1 + gamma) * (1 + rho) * v * tan(delta) / L / (1 + understeer * v^2)
    x_next   = x + v_g * cos(yaw) * dt ; y_next = y + v_g * sin(yaw) * dt ; yaw_next = yaw + yaw_rate * dt

`v` and `delta` are the state at the START of the step. The wheel channel (speed lag, steering lag,
limits, dead zone) is `model.step` of contract 1, untouched, and nothing learned acts on it. Every
correction multiplies `v`, so at zero wheel speed ground motion and yaw rate are zero for any
tables and weights. Both tables are bilinear on one grid, steering as a share of EACH SIDE's limit
by wheel speed, held constant outside the grid; `gamma` at steering 0 is fixed at 0 (a table whose
centre column is not 0 is refused by `check_yaw_table`). With every gamma 0 the step is the
contract 2 step bit for bit (`1.0 * x` is `x`).

All constants come from `evaluation/prereg/pnp-v0.5-dynamics.yaml` through `load_constants_v05`,
which refuses a file with another SHA-256; nothing here is typed in except the SHA-256 itself.
"""

from __future__ import annotations

import hashlib
from dataclasses import dataclass, field
from pathlib import Path
from typing import Any

import numpy as np
import yaml

from .evaluate import EvalSet
from .ground_motion import (
    GroundConstants,
    GroundTable,
    Learned,
    alpha_rho,
    table_alpha,
)
from .icp_motion import SCAN_DT_S
from .model import DT_S, Nominal, step
from .vehicle import Vehicle
from .windows import WARM_N, warm_state

# SHA-256 of evaluation/prereg/pnp-v0.5-dynamics.yaml as pre-registered (v0.5 header, PR 196).
V05_SHA256 = "2767f297d951c11683b40834b864b832e8cbd0b01343e2765681e11b22e86a82"
V05_RELATIVE = Path("evaluation") / "prereg" / "pnp-v0.5-dynamics.yaml"
BUNDLE_SCHEMA_VERSION_V21 = "2.1.0"
CENTRE_NODE = 0.0  # the steering node whose gamma is fixed at 0


class PreregInvalid(ValueError):
    """The v0.5 constants file is not the pre-registered one."""


@dataclass(frozen=True, kw_only=True)
class ConstantsV05(GroundConstants):
    """`GroundConstants` (same field names, so the v0.4 helpers that only read the contract
    numbers work on it) plus what v0.5 adds. `doc` is the whole yaml for the gate thresholds."""

    gamma_min: float
    gamma_max: float
    bound_margin: float
    window_length_s: float
    window_start_period_s: float
    min_wheel_speed_mps: float
    huber_delta: float
    split_half_block_s: float
    weight_decays: tuple[float, ...]
    doc: dict[str, Any] = field(default_factory=dict, compare=False, repr=False)


def parse_constants_v05(doc: dict, sha256: str = "") -> ConstantsV05:
    c, f, lf = doc["contract"], doc["table_fit"], doc["learned_fit"]
    return ConstantsV05(
        alpha_min=float(c["alpha_bounds"][0]),
        alpha_max=float(c["alpha_bounds"][1]),
        steering_nodes=tuple(float(x) for x in c["table"]["steering_nodes_share_of_side_limit"]),
        speed_nodes_mps=tuple(float(x) for x in c["table"]["wheel_speed_nodes_mps"]),
        min_windows_per_node=int(f["min_windows_per_node"]),
        smoothness_weights=tuple(float(x) for x in f["smoothness_weights"]),
        d_alpha_max=float(c["learned"]["d_alpha_max"]),
        rho_max=float(c["learned"]["rho_max"]),
        layers=tuple((int(a), int(b)) for a, b in c["learned"]["layers"]),
        seeds=tuple(int(x) for x in lf["seeds"]),
        horizons_s=tuple(float(x) for x in lf["windows"]["horizons_s"]),
        t2s_scale=float(doc["alt_truth"]["icp_seed_translation_scale"]),
        sha256=sha256,
        gamma_min=float(c["yaw_gain_bounds"][0]),
        gamma_max=float(c["yaw_gain_bounds"][1]),
        bound_margin=float(c["bound_margin"]),
        window_length_s=float(f["windows"]["length_s"]),
        window_start_period_s=float(f["windows"]["start_period_s"]),
        min_wheel_speed_mps=float(f["min_measured_wheel_speed_mps"]),
        huber_delta=float(f["loss"]["huber_delta"]),
        split_half_block_s=float(f["split_half"]["block_s"]),
        weight_decays=tuple(float(x["weight_decay"]) for x in lf["configs"]),
        doc=doc,
    )


def load_constants_v05(path: Path, expected_sha256: str = V05_SHA256) -> ConstantsV05:
    blob = Path(path).read_bytes()
    digest = hashlib.sha256(blob).hexdigest()
    if digest != expected_sha256:
        raise PreregInvalid(
            f"{path}: SHA-256 {digest} is not the pre-registered {expected_sha256}; the v0.5 "
            "constants must not change"
        )
    return parse_constants_v05(yaml.safe_load(blob.decode("utf-8")), digest)


@dataclass(frozen=True)
class GroundMotionV21:
    """Contract 2.1: the alpha table, the gamma table and the optional bounded network. `alpha`
    and `gamma` live on the same grid. `support` is the record of which nodes the data supported
    (booleans, same shape as the tables); it does not enter the step."""

    constants: ConstantsV05
    table: GroundTable
    yaw_table: GroundTable
    learned: Learned | None = None
    support: np.ndarray | None = field(default=None, compare=False)

    def __post_init__(self) -> None:
        if self.yaw_table.values.shape != self.table.values.shape:
            raise ValueError("the yaw table must have the grid of the alpha table")


def zero_yaw_table(table: GroundTable) -> GroundTable:
    return GroundTable(table.steering_nodes, table.speed_nodes_mps, np.zeros_like(table.values))


def check_yaw_table(table: GroundTable, c: ConstantsV05) -> None:
    """The centre column (steering 0) must be 0 and every value inside the gamma bounds."""
    centre = np.flatnonzero(np.isclose(table.steering_nodes, CENTRE_NODE))
    if len(centre) != 1:
        raise ValueError("the steering axis has no centre node")
    if np.any(table.values[centre[0]] != 0.0):
        raise ValueError("the centre column of the yaw table (steering 0) is fixed at 0")
    if np.any(table.values < c.gamma_min) or np.any(table.values > c.gamma_max):
        raise ValueError(f"yaw table values outside [{c.gamma_min}, {c.gamma_max}]")


def alpha_gamma_rho(
    gm: GroundMotionV21, state: np.ndarray, control: np.ndarray, vehicle: Vehicle
) -> tuple[np.ndarray, np.ndarray, np.ndarray]:
    """alpha, gamma and rho of a state and control (the first lines of the contract 2.1 step)."""
    c = gm.constants
    alpha, rho = alpha_rho(gm, state, control, vehicle)  # type: ignore[arg-type]
    v, delta = state[..., 3], state[..., 4]
    gamma = np.clip(table_alpha(gm.yaw_table, delta, v, vehicle), c.gamma_min, c.gamma_max)
    return alpha, gamma, rho


def step_v21(
    state: np.ndarray,
    control: np.ndarray,
    params: Nominal,
    vehicle: Vehicle,
    gm: GroundMotionV21,
    dt: float = DT_S,
) -> np.ndarray:
    """One step of contract 2.1. State [x, y, yaw, v, delta], control [speed_cmd, steering_cmd]."""
    state = np.asarray(state, dtype=np.float64)
    control = np.asarray(control, dtype=np.float64)
    wheel = step(state, control, params, vehicle, None, dt)  # the wheel channel, contract 1
    x, y, yaw, v, delta = (state[..., i] for i in range(5))
    alpha, gamma, rho = alpha_gamma_rho(gm, state, control, vehicle)
    v_g = alpha * v
    yaw_rate = (
        (1.0 + gamma)
        * (1.0 + rho)
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


def rollout_v21(
    state0: np.ndarray,
    controls: np.ndarray,
    params: Nominal,
    vehicle: Vehicle,
    gm: GroundMotionV21,
    dt: float = DT_S,
) -> np.ndarray:
    """States (..., N + 1, 5) for controls (..., N, 2)."""
    controls = np.asarray(controls, dtype=np.float64)
    state = np.broadcast_to(np.asarray(state0, dtype=np.float64), controls.shape[:-2] + (5,)).copy()
    out = [state]
    for i in range(controls.shape[-2]):
        state = step_v21(state, controls[..., i, :], params, vehicle, gm, dt)
        out.append(state)
    return np.stack(out, axis=-2)


def predict_v21(es: EvalSet, params: Nominal, vehicle: Vehicle, gm: GroundMotionV21) -> np.ndarray:
    """States (E, N + 1, 5) of the windows of `es`, from the same warm start and delayed commands
    as `evaluate.predict` and `ground_motion.predict_v2`."""
    data = es.data(params.command_delay_s)
    state0 = warm_state(params, vehicle, data.cmds, data.v_warm, None)
    state0[:, :3] = 0.0
    return rollout_v21(state0, data.cmds[:, WARM_N:, :], params, vehicle, gm)


def window_errors_v21(
    es: EvalSet,
    params: Nominal,
    vehicle: Vehicle,
    gm: GroundMotionV21,
    horizons_s: tuple[float, ...] = (0.5, 1.0, 1.5),
) -> dict[float, dict[str, np.ndarray]]:
    """Per window error at each horizon, the fields of `ground_motion.window_errors_v2`."""
    states = predict_v21(es, params, vehicle, gm)
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
