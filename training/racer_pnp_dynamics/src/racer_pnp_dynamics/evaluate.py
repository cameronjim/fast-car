"""Multi-step open-loop prediction error on the held-out data.

A window starts at a scan stamp. The model starts from the measured wheel speed, a steering state
reconstructed from the command history, zero pose; it is then driven by the (delayed) commands
only, and compared at 0.5, 1.0 and 1.5 s with: the measured wheel speed, the ICP-integrated yaw
and the ICP-integrated path in the frame at the start. Windows need every scan interval inside
them to be a good ICP match, and a car that is moving.
"""

from __future__ import annotations

from collections.abc import Sequence
from dataclasses import dataclass
from itertools import pairwise

import numpy as np

from .dataset import Segment
from .icp_motion import SCAN_DT_S, compose
from .model import DT_S, Nominal, Residual, rollout
from .vehicle import Vehicle
from .windows import WARM_N, cmd_at, forward_only, warm_state, wheel_at

HORIZONS_S = (0.5, 1.0, 1.5)
MAX_DELAY_S = 0.30  # window selection must not depend on the variant's delay
STRIDE_INTERVALS = 3


@dataclass(frozen=True)
class EvalData:
    t_s: np.ndarray  # (E,) scan stamps
    run_name: np.ndarray  # (E,) str
    cmds: np.ndarray  # (E, WARM_N + N, 2) delayed commands, step k applied from t - (WARM_N - k) dt
    v_warm: np.ndarray  # (E, WARM_N + 1) measured wheel speed before the start
    v_true: np.ndarray  # (E, N + 1) measured wheel speed after the start
    poses: np.ndarray  # (E, N / INTERVAL_N + 1, 3) ICP x, y, yaw in the start frame

    def __len__(self) -> int:
        return len(self.t_s)


def eval_data(
    segments: Sequence[Segment],
    delay_s: float,
    horizon_s: float = max(HORIZONS_S),
    min_speed_mps: float = 0.05,
) -> EvalData:
    n = round(horizon_s / DT_S)
    n_iv = round(horizon_s / SCAN_DT_S)
    warm_offsets = (np.arange(WARM_N + 1) - WARM_N) * DT_S
    cmd_offsets = (np.arange(WARM_N + n) - WARM_N) * DT_S
    true_offsets = np.arange(n + 1) * DT_S
    out: dict[str, list] = {k: [] for k in ("t", "name", "cmds", "vw", "vt", "poses")}
    for seg in segments:
        m = seg.motion
        if m is None or not seg.run.has_odom:
            continue
        lo = seg.t0_s + WARM_N * DT_S + MAX_DELAY_S
        hi = seg.t1_s - horizon_s - SCAN_DT_S
        for i in np.flatnonzero((m.t_s >= lo) & (m.t_s <= hi))[::STRIDE_INTERVALS]:
            if not m.ok[i : i + n_iv].all():
                continue
            t = m.t_s[i]
            c = cmd_at(seg.run, t + cmd_offsets - delay_s)
            vw = wheel_at(seg.run, t + warm_offsets)
            vt = wheel_at(seg.run, t + true_offsets)
            if not (np.isfinite(c).all() and np.isfinite(vw).all() and np.isfinite(vt).all()):
                continue
            if vw[-1] <= min_speed_mps:
                continue
            if not (forward_only(vt[None], c[None]) & forward_only(vw[None], c[None]))[0]:
                continue
            vw, vt = np.maximum(vw, 0.0), np.maximum(vt, 0.0)
            out["t"].append(t)
            out["name"].append(seg.run.name)
            out["cmds"].append(c)
            out["vw"].append(vw)
            out["vt"].append(vt)
            out["poses"].append(
                compose(m.dx_m[i : i + n_iv], m.dy_m[i : i + n_iv], m.dyaw_rad[i : i + n_iv])
            )
    if not out["t"]:
        return EvalData(
            np.zeros(0), np.array([], dtype=str), np.zeros((0, WARM_N + n, 2)),
            np.zeros((0, WARM_N + 1)), np.zeros((0, n + 1)), np.zeros((0, n_iv + 1, 3)),
        )  # fmt: skip
    return EvalData(
        np.array(out["t"]),
        np.array(out["name"]),
        np.stack(out["cmds"]),
        np.stack(out["vw"]),
        np.stack(out["vt"]),
        np.stack(out["poses"]),
    )


def predict(
    data: EvalData, params: Nominal, vehicle: Vehicle, residual: Residual | None = None
) -> np.ndarray:
    """Predicted states (E, N + 1, 5) from the start of each window."""
    state0 = warm_state(params, vehicle, data.cmds, data.v_warm, residual)
    state0[:, :3] = 0.0
    return rollout(state0, data.cmds[:, WARM_N:, :], params, vehicle, residual)


def horizon_errors(
    data: EvalData,
    params: Nominal,
    vehicle: Vehicle,
    residual: Residual | None = None,
    horizons_s: Sequence[float] = HORIZONS_S,
) -> dict[float, dict[str, float]]:
    """RMSE at each horizon: wheel speed (m/s), yaw (rad), position (m, Euclidean)."""
    if len(data) == 0:
        return {h: {"speed_rmse_mps": np.nan, "yaw_rmse_rad": np.nan, "pos_rmse_m": np.nan}
                for h in horizons_s}  # fmt: skip
    states = predict(data, params, vehicle, residual)
    out: dict[float, dict[str, float]] = {}
    for h in horizons_s:
        k = round(h / DT_S)
        j = round(h / SCAN_DT_S)
        dv = states[:, k, 3] - data.v_true[:, k]
        dyaw = states[:, k, 2] - data.poses[:, j, 2]
        dpos = np.hypot(
            states[:, k, 0] - data.poses[:, j, 0], states[:, k, 1] - data.poses[:, j, 1]
        )
        out[h] = {
            "speed_rmse_mps": float(np.sqrt(np.mean(dv**2))),
            "yaw_rmse_rad": float(np.sqrt(np.mean(dyaw**2))),
            "pos_rmse_m": float(np.sqrt(np.mean(dpos**2))),
        }
    return out


def zero_yaw_rmse(data: EvalData, horizons_s: Sequence[float] = HORIZONS_S) -> dict[float, float]:
    """Context: the yaw error of a model that predicts no turning at all."""
    return {
        h: float(np.sqrt(np.mean(data.poses[:, round(h / SCAN_DT_S), 2] ** 2))) for h in horizons_s
    }


def compare_variants(
    segments: Sequence[Segment],
    vehicle: Vehicle,
    variants: dict[str, tuple[Nominal, Residual | None]],
) -> dict[str, dict[float, dict[str, float]]]:
    """horizon errors per named variant on the same held-out windows."""
    return {
        name: horizon_errors(eval_data(segments, params.command_delay_s), params, vehicle, res)
        for name, (params, res) in variants.items()
    }


def ground_speed_ratio(
    segments: Sequence[Segment],
    delay_s: float,
    bin_edges: Sequence[float] = (0.0, 0.05, 0.1, 0.2, 0.3, 0.4, 0.55),
    min_wheel_mps: float = 0.15,
) -> list[dict[str, float]]:
    """Median ratio of ICP ground speed to wheel speed against |steering command| (a finding,
    not a model parameter: in tight turns the wheels slip and ground speed is lower)."""
    ratios, steer = [], []
    for seg in segments:
        m = seg.motion
        if m is None or not seg.run.has_odom:
            continue
        sel = np.flatnonzero(
            m.ok & (m.t_s >= seg.t0_s + MAX_DELAY_S) & (m.t_s <= seg.t1_s - SCAN_DT_S)
        )
        if len(sel) == 0:
            continue
        mid = m.t_s[sel] + 0.5 * SCAN_DT_S
        wheel = wheel_at(seg.run, mid)
        cmd = cmd_at(seg.run, mid - delay_s)
        ground = np.hypot(m.dx_m[sel], m.dy_m[sel]) / m.dt_s[sel]
        ok = np.isfinite(wheel) & np.isfinite(cmd).all(axis=1) & (wheel >= min_wheel_mps)
        ratios.append(ground[ok] / wheel[ok])
        steer.append(np.abs(cmd[ok, 1]))
    if not ratios:
        return []
    r, s = np.concatenate(ratios), np.concatenate(steer)
    rows = []
    for lo, hi in pairwise(bin_edges):
        sel = (s >= lo) & (s < hi)
        if sel.sum() >= 5:
            rows.append(
                {"steer_lo_rad": lo, "steer_hi_rad": hi, "n": int(sel.sum()),
                 "median_ratio": float(np.median(r[sel])),
                 "p25": float(np.percentile(r[sel], 25)), "p75": float(np.percentile(r[sel], 75))}
            )  # fmt: skip
    return rows
