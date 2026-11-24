"""Python reference of the planner's online speed adaptation (racer_planner speed_adaptation.hpp)
and its offline replay on recorded runs (prototype, owner decision 29).

The planner corrects the speed step of its model `dv <- dv * (1 + gain) + bias * dt` (only while
the speed target is above 0) and fits (bias, gain) at run time from the wheel speed by a
recursive prediction error method: over each comparison window of `window_s` a shadow of the
CORRECTED model is stepped once per odometry sample from the measured speed at the window start
under the commands that acted, together with the sensitivities of its speed to bias and gain
(S <- dF/dv * S + dF/dtheta); at the window end the error measured - shadow and the sensitivities
are one recursive least squares sample (normalised by `error_sigma_mps`), with forgetting, hard
limits on both parameters and a covariance ceiling. Comparing over a window, not per sample,
keeps the derivative of a noisy and delayed wheel speed out of it; the sensitivities make the
fitted numbers the step's own parameters, not window averages. Same constants, gating and limits
as the C++ class; `run_corrections` replays it causally along a recorded run.
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
    forgetting: float = 0.98  # per window
    initial_variance: float = 0.1  # prior variance of each parameter after a reset
    variance_max: float = 1.0
    error_sigma_mps: float = 0.05  # the window error a sample is normalised by
    bias_max_mps2: float = 0.5
    gain_max: float = 0.3
    min_speed_mps: float = 0.3
    min_dt_s: float = 0.05
    max_dt_s: float = 2.0
    window_s: float = 0.5
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
            window_s=params.window_s,
            error_sigma_mps=params.error_sigma_mps
            if math.isfinite(params.error_sigma_mps) and params.error_sigma_mps > 0
            else 0.05,
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
        self,
        phi_bias: float,
        phi_gain: float,
        error_mps: float,
        v_start: float,
        span_s: float,
        forward: bool,
    ) -> bool:
        """One closed window: the sensitivities of the shadow's end speed to bias and gain, the
        error measured minus shadow at its end, the speed at its start and its length."""
        p = self.p
        if not p.enabled:
            return False
        if not all(math.isfinite(x) for x in (phi_bias, phi_gain, error_mps, v_start, span_s)):
            self.skipped += 1
            self.reset()
            return False
        if (
            not forward
            or v_start < p.min_speed_mps
            or span_s < p.min_dt_s
            or span_s > p.max_dt_s
            or abs(error_mps) > p.innovation_max_mps2 * span_s
        ):
            self.skipped += 1
            return False
        phi = np.array([phi_bias, phi_gain]) / p.error_sigma_mps
        e = error_mps / p.error_sigma_mps
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


SENS_EPS_MPS = 1e-4  # finite difference step of dF/dv


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
    i), as the planner computes them (module docstring). The steering estimate is the model's
    own lag driven by the commands. A gap in the wheel speed drops the open window."""
    ap = ap or AdaptationParams()
    est = SpeedAdaptation(ap)
    out = np.zeros((len(t_s), 2))
    dt = float(t_s[1] - t_s[0]) if len(t_s) > 1 else DT_S
    floor = params.speed_deadband_mps
    delta = 0.0
    win = None
    for i in range(len(t_s)):
        u = cmds[i]
        v = wheel_mps[i]
        if not (math.isfinite(v) and np.all(np.isfinite(u))):
            win = None
            out[i] = est.correction()
            continue
        if win is not None:
            b, g = est.correction()
            shadow, sens, u0 = win["shadow"], win["sens"], win["u"]
            nxt = step_scalar(shadow, u0, params, vehicle, residual, b, g, dt)
            bumped = shadow.copy()
            bumped[3] += SENS_EPS_MPS
            f_v = (step_scalar(bumped, u0, params, vehicle, residual, b, g, dt)[3] - nxt[3]) / (
                SENS_EPS_MPS
            )
            if float(speed_target(u0[0], params)) > 0.0:
                raw = step(shadow, u0, params, vehicle, residual, dt)
                f_theta = np.array([dt, raw[3] - shadow[3]])
            else:
                f_theta = np.zeros(2)
            win["sens"] = f_v * sens + f_theta
            win["shadow"] = nxt
            win["fwd"] = win["fwd"] and bool(u0[0] > 0.0 and u0[0] >= floor)
            win["u"] = u.copy()
            span = float(t_s[i] - win["t0"])
            if span >= ap.window_s - 1e-9:
                est.update(win["sens"][0], win["sens"][1], v - nxt[3], win["v0"], span, win["fwd"])
                win = None
        if win is None:
            win = {
                "t0": float(t_s[i]),
                "v0": float(v),
                "shadow": np.array([0.0, 0.0, 0.0, v, delta]),
                "sens": np.zeros(2),
                "fwd": True,
                "u": u.copy(),
            }
        s = np.array([0.0, 0.0, 0.0, v, delta])
        delta = float(step(s, u, params, vehicle, None, dt)[4])
        out[i] = est.correction()
    return out


def step_scalar(state, control, params, vehicle, residual, bias, gain, dt):
    """`step_corrected` of one state with scalar bias and gain."""
    return step_corrected(
        state, np.asarray(control, float), params, vehicle, residual,
        np.array(bias), np.array(gain), dt,
    )  # fmt: skip


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
