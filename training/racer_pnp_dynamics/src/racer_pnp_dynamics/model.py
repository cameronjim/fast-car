"""The Python reference of the dynamics model step (claude-docs/13-plug-and-play.md section 4.4).

This is THE model: the C++ planner implements the same step and reproduces this module's golden
rollouts to 1e-4. float64 throughout. The step reads the wheelbase, the steering limits and the
steering rate limit from the generated vehicle_params binding (`Vehicle`), nothing physical is
typed here.

State  [x_m, y_m, yaw_rad, v_mps, delta_rad], control [speed_cmd_mps, steering_cmd_rad].
`v` is the WHEEL speed. All right hand sides use the state at the start of the step. The command
delay is not part of the step: whoever rolls the model forward feeds it the delayed commands.
"""

from __future__ import annotations

from dataclasses import dataclass, fields
from typing import Any

import numpy as np

from .vehicle import Vehicle

STATE_NAMES = ("x_m", "y_m", "yaw_rad", "v_mps", "delta_rad")
CONTROL_NAMES = ("speed_cmd_mps", "steering_cmd_rad")
X, Y, YAW, V, DELTA = range(5)
DT_S = 0.05  # io.dt_s of every bundle this package writes

RESIDUAL_INPUTS = ("v_mps", "delta_rad", "speed_cmd_mps", "steering_cmd_rad")
RESIDUAL_OUTPUTS = ("dv_dt_mps2", "dyaw_dt_rad_per_s")
RESIDUAL_LAYERS = ((4, 32), (32, 32), (32, 2))


@dataclass(frozen=True)
class Nominal:
    """`io.nominal` of a dynamics bundle."""

    speed_gain: float
    speed_tau_s: float
    accel_max_mps2: float
    decel_max_mps2: float
    steer_gain_left: float
    steer_gain_right: float
    steer_offset_rad: float
    steer_tau_s: float
    command_delay_s: float
    understeer_s2_per_m2: float

    def to_dict(self) -> dict[str, float]:
        return {f.name: float(getattr(self, f.name)) for f in fields(self)}

    @classmethod
    def from_dict(cls, values: dict[str, Any]) -> Nominal:
        return cls(**{f.name: float(values[f.name]) for f in fields(cls)})

    def replace(self, **changes: float) -> Nominal:
        return Nominal.from_dict({**self.to_dict(), **changes})


# The contract's placeholder values (section 4.4). Used as "nominal defaults" for the fixture and
# as the baseline the fit has to beat.
PLACEHOLDER_NOMINAL = Nominal(
    speed_gain=1.0,
    speed_tau_s=0.25,
    accel_max_mps2=2.0,
    decel_max_mps2=1.5,
    steer_gain_left=1.0,
    steer_gain_right=1.0,
    steer_offset_rad=0.0,
    steer_tau_s=0.10,
    command_delay_s=0.08,
    understeer_s2_per_m2=0.0,
)


@dataclass(frozen=True)
class MlpWeights:
    """4-32-32-2 (or any consistent shapes), W as [out, in] row-major, tanh hidden layers."""

    w1: np.ndarray
    b1: np.ndarray
    w2: np.ndarray
    b2: np.ndarray
    w3: np.ndarray
    b3: np.ndarray

    def arrays(self) -> tuple[np.ndarray, ...]:
        return (self.w1, self.b1, self.w2, self.b2, self.w3, self.b3)

    def layers(self) -> list[list[int]]:
        return [[w.shape[1], w.shape[0]] for w in (self.w1, self.w2, self.w3)]


def zero_weights(layers: tuple[tuple[int, int], ...] = RESIDUAL_LAYERS) -> MlpWeights:
    arrays: list[np.ndarray] = []
    for n_in, n_out in layers:
        arrays += [np.zeros((n_out, n_in)), np.zeros(n_out)]
    return MlpWeights(*arrays)


def pack_weights(weights: MlpWeights) -> np.ndarray:
    """Flat float32, the order of residual_mlp.f32: W1, b1, W2, b2, W3, b3."""
    return np.concatenate([a.astype(np.float32).ravel() for a in weights.arrays()])


def unpack_weights(
    flat: np.ndarray, layers: tuple[tuple[int, int], ...] = RESIDUAL_LAYERS
) -> MlpWeights:
    flat = np.asarray(flat)
    expected = sum(n_in * n_out + n_out for n_in, n_out in layers)
    if flat.size != expected:
        raise ValueError(f"residual file holds {flat.size} floats, expected {expected}")
    arrays: list[np.ndarray] = []
    at = 0
    for n_in, n_out in layers:
        arrays.append(flat[at : at + n_in * n_out].reshape(n_out, n_in).astype(np.float64))
        at += n_in * n_out
        arrays.append(flat[at : at + n_out].astype(np.float64))
        at += n_out
    return MlpWeights(*arrays)


def mlp_forward(weights: MlpWeights, x: np.ndarray) -> np.ndarray:
    """tanh, tanh, linear. x has shape (..., n_in)."""
    h = np.tanh(x @ weights.w1.T + weights.b1)
    h = np.tanh(h @ weights.w2.T + weights.b2)
    return h @ weights.w3.T + weights.b3


@dataclass(frozen=True)
class Residual:
    """`io.residual` plus the weights. With `enabled` False the weights are never read."""

    enabled: bool
    weights: MlpWeights
    input_mean: np.ndarray
    input_std: np.ndarray
    output_scale: np.ndarray

    @classmethod
    def disabled(cls) -> Residual:
        return cls(
            enabled=False,
            weights=zero_weights(),
            input_mean=np.zeros(4),
            input_std=np.ones(4),
            output_scale=np.ones(2),
        )

    def forward(self, inputs: np.ndarray) -> np.ndarray:
        """output_scale * MLP((inputs - input_mean) / input_std), shape (..., 2)."""
        return self.output_scale * mlp_forward(
            self.weights, (inputs - self.input_mean) / self.input_std
        )


def steering_target(steer_cmd: np.ndarray, params: Nominal, vehicle: Vehicle) -> np.ndarray:
    """The steady state road wheel angle of a steering command (the first line of the step)."""
    steer_cmd = np.asarray(steer_cmd, dtype=np.float64)
    gain = np.where(steer_cmd >= 0.0, params.steer_gain_left, params.steer_gain_right)
    return np.clip(
        gain * steer_cmd + params.steer_offset_rad, vehicle.steer_min_rad, vehicle.steer_max_rad
    )


def step(
    state: np.ndarray,
    control: np.ndarray,
    params: Nominal,
    vehicle: Vehicle,
    residual: Residual | None = None,
    dt: float = DT_S,
) -> np.ndarray:
    """One step of the contract. `state` (..., 5) and `control` (..., 2) broadcast together."""
    state = np.asarray(state, dtype=np.float64)
    control = np.asarray(control, dtype=np.float64)
    x, y, yaw, v, delta = (state[..., i] for i in range(5))
    speed_cmd = control[..., 0]
    steer_cmd = control[..., 1]

    delta_target = steering_target(steer_cmd, params, vehicle)
    a_d = 1.0 - np.exp(-dt / params.steer_tau_s)
    rate = vehicle.steer_rate_max_rad_per_s * dt
    delta_next = delta + np.clip(a_d * (delta_target - delta), -rate, rate)
    a_v = 1.0 - np.exp(-dt / params.speed_tau_s)
    dv = np.clip(
        a_v * (params.speed_gain * speed_cmd - v),
        -params.decel_max_mps2 * dt,
        params.accel_max_mps2 * dt,
    )
    yaw_rate = v * np.tan(delta) / vehicle.wheelbase_m / (1.0 + params.understeer_s2_per_m2 * v * v)
    if residual is not None and residual.enabled:
        r = residual.forward(np.stack([v, delta, speed_cmd, steer_cmd], axis=-1))
        dv = dv + r[..., 0] * dt
        yaw_rate = yaw_rate + r[..., 1]
    return np.stack(
        [
            x + v * np.cos(yaw) * dt,
            y + v * np.sin(yaw) * dt,
            yaw + yaw_rate * dt,
            np.maximum(0.0, v + dv),
            delta_next,
        ],
        axis=-1,
    )


def rollout(
    state0: np.ndarray,
    controls: np.ndarray,
    params: Nominal,
    vehicle: Vehicle,
    residual: Residual | None = None,
    dt: float = DT_S,
) -> np.ndarray:
    """States (..., N + 1, 5) for controls (..., N, 2); control i is applied from state i."""
    controls = np.asarray(controls, dtype=np.float64)
    state = np.broadcast_to(np.asarray(state0, dtype=np.float64), controls.shape[:-2] + (5,)).copy()
    out = [state]
    for i in range(controls.shape[-2]):
        state = step(state, controls[..., i, :], params, vehicle, residual, dt)
        out.append(state)
    return np.stack(out, axis=-2)
