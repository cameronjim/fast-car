"""Python reference of the planner's online speed adaptation (racer_planner speed_adaptation.hpp)
and its offline replay on recorded runs (prototype, owner decision 29).

The planner fits, by recursive least squares with forgetting, `a_meas - a_pred = bias + gain *
a_pred` over each odometry interval (a_pred: one model step from the measured wheel speed and the
model's steering under the command acting then) and corrects the speed step of its rollouts
`dv <- dv * (1 + gain) + bias * dt` while the speed target is above 0. Same constants, same
gating, same limits as the C++ class; `replay` runs it causally along a recorded run (it only
ever uses the odometry up to the window start, as the car would) and scores the corrected
rollouts on the same windows as the uncorrected ones.
"""

from __future__ import annotations

import math
from dataclasses import dataclass

import numpy as np

from .model import DT_S, Nominal, Residual, speed_target, step
from .vehicle import Vehicle


@dataclass(frozen=True)
class AdaptationParams:
    enabled: bool = True
    forgetting: float = 0.995
    initial_variance: float = 0.05
    variance_max: float = 1.0
    bias_max_mps2: float = 0.5
    gain_max: float = 0.3
    min_speed_mps: float = 0.3
    min_dt_s: float = 0.01
    max_dt_s: float = 0.2
    innovation_max_mps2: float = 4.0


class SpeedAdaptation:
    """Line for line the C++ SpeedAdaptation (one sample at a time)."""

    def __init__(self, params: AdaptationParams | None = None) -> None:
        params = params or AdaptationParams()
        f = params.forgetting if math.isfinite(params.forgetting) else 0.995
        self.p = AdaptationParams(
            enabled=params.enabled,
            forgetting=min(max(f, 0.9), 1.0),
            initial_variance=max(1e-9, params.initial_variance),
            variance_max=max(params.initial_variance, params.variance_max),
            bias_max_mps2=max(0.0, params.bias_max_mps2)
            if math.isfinite(params.bias_max_mps2)
            else 0.0,
            gain_max=min(max(params.gain_max, 0.0), 0.9) if math.isfinite(params.gain_max) else 0.0,
            min_speed_mps=params.min_speed_mps,
            min_dt_s=params.min_dt_s,
            max_dt_s=params.max_dt_s,
            innovation_max_mps2=params.innovation_max_mps2,
        )
        self.resets = 0
        self.skipped = 0
        self.reset()
        self.resets = 0

    def reset(self) -> None:
        self.theta = np.zeros(2)
        self.P = np.eye(2) * self.p.initial_variance
        self.updates = 0
        self.at_limit = False
        self.resets += 1

    def update(
        self, a_pred: float, a_meas: float, v_start: float, dt: float, forward: bool
    ) -> bool:
        p = self.p
        if not p.enabled:
            return False
        if not all(math.isfinite(x) for x in (a_pred, a_meas, v_start, dt)):
            self.skipped += 1
            self.reset()
            return False
        y = a_meas - a_pred
        if (
            not forward
            or v_start < p.min_speed_mps
            or dt < p.min_dt_s
            or dt > p.max_dt_s
            or abs(y) > p.innovation_max_mps2
        ):
            self.skipped += 1
            return False
        phi = np.array([1.0, a_pred])
        e = y - float(self.theta @ phi)
        pphi = self.P @ phi
        denom = p.forgetting + float(phi @ pphi)
        k = pphi / denom
        self.theta = self.theta + k * e
        self.P = (self.P - np.outer(k, pphi)) / p.forgetting
        self.P = 0.5 * (self.P + self.P.T)
        dmax = max(self.P[0, 0], self.P[1, 1])
        if dmax > p.variance_max:
            self.P *= p.variance_max / dmax
        clamped = np.array(
            [
                min(max(self.theta[0], -p.bias_max_mps2), p.bias_max_mps2),
                min(max(self.theta[1], -p.gain_max), p.gain_max),
            ]
        )
        self.at_limit = bool(np.any(clamped != self.theta))
        self.theta = clamped
        self.updates += 1
        return True

    def correction(self) -> tuple[float, float]:
        if not self.p.enabled:
            return 0.0, 0.0
        return float(self.theta[0]), float(self.theta[1])


def step_corrected(
    state: np.ndarray,
    control: np.ndarray,
    params: Nominal,
    vehicle: Vehicle,
    residual: Residual | None,
    bias: np.ndarray,
    gain: np.ndarray,
    dt: float = DT_S,
) -> np.ndarray:
    """The contract step with the speed correction (bias, gain broadcast with the batch)."""
    out = step(state, control, params, vehicle, residual, dt)
    state = np.asarray(state, dtype=np.float64)
    v, delta = state[..., 3], state[..., 4]
    target = speed_target(control[..., 0], params)
    a_v = 1.0 - np.exp(-dt / params.speed_tau_s)
    dv = np.clip(a_v * (target - v), -params.decel_max_mps2 * dt, params.accel_max_mps2 * dt)
    if residual is not None and residual.enabled:
        r = residual.forward(np.stack([v, delta, control[..., 0], control[..., 1]], axis=-1))
        dv = dv + r[..., 0] * dt
    corrected = np.where(target > 0.0, dv * (1.0 + gain) + bias * dt, dv)
    out[..., 3] = np.maximum(0.0, v + corrected)
    return out


def run_corrections(
    t_s: np.ndarray,
    wheel_mps: np.ndarray,
    cmds: np.ndarray,
    params: Nominal,
    vehicle: Vehicle,
    residual: Residual | None,
    ap: AdaptationParams | None = None,
) -> np.ndarray:
    """(T, 2) bias and gain after each grid sample of a run, causally (row i uses samples up to
    i). The steering estimate is the model's own lag driven by the commands; a gap in the wheel
    speed starts a new interval chain, as on the car."""
    est = SpeedAdaptation(ap or AdaptationParams())
    out = np.zeros((len(t_s), 2))
    delta = 0.0
    prev = None  # (t, v, delta, cmd)
    for i in range(len(t_s)):
        u = cmds[i]
        v = wheel_mps[i]
        if math.isfinite(v) and np.all(np.isfinite(u)):
            if prev is not None:
                t0, v0, d0, u0 = prev
                s = np.array([0.0, 0.0, 0.0, v0, d0])
                a_pred = (float(step(s, u0, params, vehicle, residual)[3]) - v0) / DT_S
                dt = float(t_s[i] - t0)
                floor = params.speed_deadband_mps
                est.update(a_pred, (v - v0) / dt, v0, dt, bool(u0[0] > 0.0 and u0[0] >= floor))
            prev = (float(t_s[i]), float(v), delta, u.copy())
            s = np.array([0.0, 0.0, 0.0, v, delta])
            delta = float(step(s, u, params, vehicle, None, float(t_s[1] - t_s[0]))[4])
        else:
            prev = None
        out[i] = est.correction()
    return out


def predict_corrected(
    data,
    params: Nominal,
    vehicle: Vehicle,
    residual: Residual | None,
    bias: np.ndarray,
    gain: np.ndarray,
) -> np.ndarray:
    """`evaluate.predict` with each window's speed correction (bias, gain of shape (E,)) applied
    to the rollout from the window start (the warm start is the model's, uncorrected)."""
    from .windows import WARM_N, warm_state

    state = warm_state(params, vehicle, data.cmds, data.v_warm, residual)
    state[:, :3] = 0.0
    out = [state]
    for i in range(WARM_N, data.cmds.shape[1]):
        state = step_corrected(state, data.cmds[:, i, :], params, vehicle, residual, bias, gain)
        out.append(state)
    return np.stack(out, axis=1)


def window_corrections(es, params: Nominal, vehicle: Vehicle, residual, ap=None):
    """(E, 2) the correction each window of `es` starts with: the run's causal estimate at the
    last grid sample at or before the window start (commands read with the model's delay)."""
    from .windows import cmd_at

    out = np.zeros((len(es), 2))
    for r, run in enumerate(es.runs):
        sel = np.flatnonzero(es.run_idx == r)
        if len(sel) == 0:
            continue
        cmds = cmd_at(run, run.t_s - params.command_delay_s)
        corr = run_corrections(run.t_s, run.wheel_speed_mps, cmds, params, vehicle, residual, ap)
        idx = np.clip(np.searchsorted(run.t_s, es.t_s[sel], side="right") - 1, 0, len(run.t_s) - 1)
        out[sel] = corr[idx]
    return out


def corrected_window_errors(es, params, vehicle, residual, corr, horizons_s=(0.5, 1.0, 1.5)):
    """The fields of `ground_motion.contract1_window_errors` for the corrected model."""
    from .evaluate import SCAN_DT_S

    states = predict_corrected(
        es.data(params.command_delay_s), params, vehicle, residual, corr[:, 0], corr[:, 1]
    )
    out = {}
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
