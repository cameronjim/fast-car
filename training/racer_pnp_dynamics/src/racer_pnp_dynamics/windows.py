"""Cutting segments into the windows the fit, the residual and the evaluation work on.

Everything here samples the 50 Hz run at the model's own 0.05 s step: commands by zero order hold
at (time - command delay), wheel speed by linear interpolation. The steering state delta is not
measured; it is reconstructed by running the model's steering lag from the steady state of the
command `WARM_S` before the window, which is long enough (five time constants) to forget the
start.
"""

from __future__ import annotations

from collections.abc import Sequence
from dataclasses import dataclass

import numpy as np

from .bags import GRID_DT_S, Run
from .dataset import Segment
from .icp_motion import SCAN_DT_S
from .model import DT_S, Nominal, Residual, steering_target, step
from .vehicle import Vehicle

WARM_N = 10  # warm up steps of DT_S before a window (0.5 s)
INTERVAL_N = round(SCAN_DT_S / DT_S)  # model steps per scan interval


def cmd_at(run: Run, times_s: np.ndarray) -> np.ndarray:
    """Zero order hold commands at the given times, shape (..., 2) (speed, steering); NaN where
    the time is outside the run or the command there is not valid."""
    idx = np.floor((np.asarray(times_s) - run.t_s[0]) / GRID_DT_S + 1e-9).astype(int)
    ok = (idx >= 0) & (idx < len(run.t_s))
    idx = np.clip(idx, 0, len(run.t_s) - 1)
    out = np.stack([run.speed_cmd_mps[idx], run.steering_cmd_rad[idx]], axis=-1)
    out[~ok] = np.nan
    return out


def wheel_at(run: Run, times_s: np.ndarray, wheel: np.ndarray | None = None) -> np.ndarray:
    """Linearly interpolated wheel speed at the given times (NaN outside or next to NaN)."""
    wheel = run.wheel_speed_mps if wheel is None else wheel
    pos = (np.asarray(times_s) - run.t_s[0]) / GRID_DT_S
    lo = np.floor(pos + 1e-9).astype(int)
    frac = pos - lo
    ok = (lo >= 0) & (lo + 1 < len(wheel))
    lo = np.clip(lo, 0, len(wheel) - 2)
    out = wheel[lo] * (1.0 - frac) + wheel[lo + 1] * frac
    out[~ok] = np.nan
    return out


REVERSE_TOLERANCE_MPS = 0.03


def forward_only(v_mps: np.ndarray, cmds: np.ndarray) -> np.ndarray:
    """Windows where the car is driven forward throughout: the model has v >= 0, so a window with
    a reverse command or a clearly negative wheel speed (the 2026-11-11 bags back out of walls)
    is outside what it describes. NaN compares false and is left to the finite checks."""
    with np.errstate(invalid="ignore"):
        return (v_mps >= -REVERSE_TOLERANCE_MPS).all(axis=1) & (cmds[..., 0] >= 0.0).all(axis=1)


@dataclass(frozen=True)
class SpeedWindows:
    v_mps: np.ndarray  # (W, n + 1) measured wheel speed at 0, dt, ..., n dt
    cmds: np.ndarray  # (W, n, 2) delayed commands applied from each step

    def __len__(self) -> int:
        return len(self.v_mps)


def speed_windows(
    segments: Sequence[Segment],
    delay_s: float,
    horizon_s: float = 1.0,
    stride_s: float = 0.25,
) -> SpeedWindows:
    """Windows of measured wheel speed with the commands that drove them (odometry bags only)."""
    n = round(horizon_s / DT_S)
    offsets = np.arange(n + 1) * DT_S
    vs, cs = [], []
    for seg in segments:
        if not seg.run.has_odom:
            continue
        starts = np.arange(seg.t0_s, seg.t1_s - horizon_s, stride_s)
        if len(starts) == 0:
            continue
        times = starts[:, None] + offsets[None, :]
        v = wheel_at(seg.run, times)
        c = cmd_at(seg.run, times[:, :-1] - delay_s)
        ok = np.isfinite(v).all(axis=1) & np.isfinite(c).all(axis=(1, 2))
        ok &= forward_only(v, c)
        v = np.maximum(v, 0.0)
        vs.append(v[ok])
        cs.append(c[ok])
    if not vs:
        return SpeedWindows(np.zeros((0, n + 1)), np.zeros((0, n, 2)))
    return SpeedWindows(np.concatenate(vs), np.concatenate(cs))


def speed_rollout(
    windows: SpeedWindows, params: Nominal, vehicle: Vehicle, residual: Residual | None = None
) -> np.ndarray:
    """Predicted wheel speed (W, n + 1) from each window's first measured speed."""
    n_w = len(windows)
    state = np.zeros((n_w, 5))
    state[:, 3] = windows.v_mps[:, 0]
    out = [state[:, 3].copy()]
    for k in range(windows.cmds.shape[1]):
        state = step(state, windows.cmds[:, k, :], params, vehicle, residual)
        out.append(state[:, 3].copy())
    return np.stack(out, axis=1)


def reconstruct_wheel_speed(run: Run, params: Nominal, vehicle: Vehicle) -> np.ndarray:
    """Wheel speed on the run's 50 Hz grid from the speed part of the model driven by the delayed
    commands (for bags with no odometry). Starts from rest; a missing command counts as zero."""
    speed_cmd = np.nan_to_num(run.speed_cmd_mps, nan=0.0)
    lag = round(params.command_delay_s / GRID_DT_S)
    out = np.zeros(len(speed_cmd))
    state = np.zeros(5)
    for k in range(len(speed_cmd) - 1):
        out[k] = state[3]
        j = max(k - lag, 0)
        state = step(state, [speed_cmd[j], 0.0], params, vehicle, dt=GRID_DT_S)
    out[-1] = state[3]
    return out


@dataclass(frozen=True)
class YawData:
    """Scan intervals with the delayed commands and the wheel speed over WARM_N + INTERVAL_N
    model steps ending at the interval's end, and the ICP motion of the interval."""

    cmds: np.ndarray  # (I, WARM_N + INTERVAL_N, 2)
    v_mps: np.ndarray  # (I, WARM_N + INTERVAL_N) wheel speed at each step start
    v_end_mps: np.ndarray  # (I,) wheel speed at the interval end
    dyaw_rad: np.ndarray  # (I,) ICP yaw change
    ground_mps: np.ndarray  # (I,) ICP ground speed of the interval
    t_s: np.ndarray  # (I,) scan stamp of the interval start
    scan_index: np.ndarray  # (I,) index of the interval in its motion series
    run_name: np.ndarray  # (I,) str

    def __len__(self) -> int:
        return len(self.dyaw_rad)


def yaw_data(
    segments: Sequence[Segment],
    delay_s: float,
    wheel_by_run: dict[str, np.ndarray] | None = None,
    min_speed_mps: float = 0.05,
) -> YawData:
    """Good ICP intervals inside the segments (with warm up history available) while moving."""
    n = WARM_N + INTERVAL_N
    offsets = (np.arange(n) - WARM_N) * DT_S
    cmds, vs, ve, dy, gs, ts, si, names = [], [], [], [], [], [], [], []
    for seg in segments:
        m = seg.motion
        if m is None:
            continue
        wheel = None if wheel_by_run is None else wheel_by_run.get(seg.run.name)
        lo = seg.t0_s + WARM_N * DT_S + delay_s
        hi = seg.t1_s - SCAN_DT_S - DT_S
        pick = np.flatnonzero(m.ok & (m.t_s >= lo) & (m.t_s <= hi))
        if len(pick) == 0:
            continue
        times = m.t_s[pick][:, None] + offsets[None, :]
        c = cmd_at(seg.run, times - delay_s)
        v = wheel_at(seg.run, times, wheel)
        v_end = wheel_at(seg.run, m.t_s[pick] + SCAN_DT_S, wheel)
        ok = np.isfinite(c).all(axis=(1, 2)) & np.isfinite(v).all(axis=1) & np.isfinite(v_end)
        ok &= v[:, WARM_N] > min_speed_mps
        ok &= forward_only(v, c)
        v = np.maximum(v, 0.0)
        pick = pick[ok]
        cmds.append(c[ok])
        vs.append(v[ok])
        ve.append(v_end[ok])
        dy.append(m.dyaw_rad[pick])
        gs.append(np.hypot(m.dx_m[pick], m.dy_m[pick]) / m.dt_s[pick])
        ts.append(m.t_s[pick])
        si.append(pick)
        names.extend([seg.run.name] * len(pick))
    if not cmds:
        return YawData(
            np.zeros((0, n, 2)), np.zeros((0, n)), np.zeros(0), np.zeros(0), np.zeros(0),
            np.zeros(0), np.zeros(0, dtype=int), np.array([], dtype=str),
        )  # fmt: skip
    return YawData(
        np.concatenate(cmds),
        np.concatenate(vs),
        np.concatenate(ve),
        np.concatenate(dy),
        np.concatenate(gs),
        np.concatenate(ts),
        np.concatenate(si),
        np.array(names),
    )


def warm_state(
    params: Nominal,
    vehicle: Vehicle,
    cmds: np.ndarray,
    v_mps: np.ndarray,
    residual: Residual | None = None,
) -> np.ndarray:
    """State (I, 5) after WARM_N steps: delta from the steering lag started at the steady state
    of the first command, the measured wheel speed substituted at every step, yaw reset to 0."""
    state = np.zeros((len(cmds), 5))
    state[:, 4] = steering_target(cmds[:, 0, 1], params, vehicle)
    for k in range(WARM_N):
        state[:, 3] = v_mps[:, k]
        state = step(state, cmds[:, k, :], params, vehicle, residual)
    state[:, 2] = 0.0
    state[:, 3] = v_mps[:, WARM_N]
    return state


def predict_yaw_change(
    params: Nominal,
    vehicle: Vehicle,
    data: YawData,
    residual: Residual | None = None,
    speed: str = "ground",
) -> np.ndarray:
    """Model yaw change over each interval. `speed` is the speed that drives the interval steps:
    "ground" (the ICP ground speed of the interval, so the steering gain is path geometry) or
    "wheel" (the measured wheel speed at each step, what the planner's model does at run time).
    The warm up always uses the wheel speed."""
    if speed not in ("ground", "wheel"):
        raise ValueError(f"speed must be 'ground' or 'wheel', got {speed!r}")
    state = warm_state(params, vehicle, data.cmds, data.v_mps, residual)
    for k in range(WARM_N, WARM_N + INTERVAL_N):
        state[:, 3] = data.ground_mps if speed == "ground" else data.v_mps[:, k]
        state = step(state, data.cmds[:, k, :], params, vehicle, residual)
    return state[:, 2]
