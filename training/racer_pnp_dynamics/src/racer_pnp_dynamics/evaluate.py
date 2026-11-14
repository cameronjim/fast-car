"""Multi-step open-loop prediction error on the held-out data.

The first half of this module is the dyn-v0.1 report (pooled windows every 0.3 s, no strata),
kept as the `legacy_` report so v0.1's numbers stay comparable. `EvalSet` and what follows it
is the pre-registered v0.2 evaluation (docs/notes/preregistration-pnp-v0.2.md section 3).

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

from .bags import Run
from .dataset import Segment
from .icp_motion import SCAN_DT_S, compose
from .model import DT_S, Nominal, Residual, rollout
from .vehicle import Vehicle
from .windows import WARM_N, cmd_at, forward_only, unresponsive, warm_state, wheel_at

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
            if unresponsive(vt[None], c[None, WARM_N:])[0]:
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


EVAL_STRIDE_S = 0.25
EVAL_MIN_START_SPEED_MPS = 0.25
BLOCK_S = 5.0  # bootstrap block length (pre-registration 3)


@dataclass(frozen=True)
class EvalSet:
    """Evaluation windows that do not depend on any model: the same set scores every model.

    A window starts at a scan stamp. Its commands are taken by `commands(delay_s)` so each model
    reads them delayed as its own `command_delay_s` says. A window is in the set only if every
    sample it can touch (commands from MAX_DELAY_S plus WARM_N steps before the start, to the
    horizon, wheel speed, every ICP interval) lies inside ONE segment and is valid for every
    delay up to MAX_DELAY_S."""

    runs: tuple[Run, ...]
    run_idx: np.ndarray  # (E,) index into runs
    t_s: np.ndarray  # (E,) scan stamp of the window start
    segment: np.ndarray  # (E,) split segment name
    direction: np.ndarray  # (E,) cw / ccw / legacy
    block: np.ndarray  # (E,) int, one id per (segment, 5 s block)
    start_speed: np.ndarray  # (E,) wheel speed at the start
    v_warm: np.ndarray  # (E, WARM_N + 1)
    v_true: np.ndarray  # (E, N + 1)
    poses: np.ndarray  # (E, N / INTERVAL_N + 1, 3) ICP x, y, yaw in the start frame

    def __len__(self) -> int:
        return len(self.t_s)

    def commands(self, delay_s: float) -> np.ndarray:
        """(E, WARM_N + N, 2) delayed commands, step k applied from t - (WARM_N - k) dt."""
        n = self.v_true.shape[1] - 1
        offsets = (np.arange(WARM_N + n) - WARM_N) * DT_S
        out = np.zeros((len(self), WARM_N + n, 2))
        for r, run in enumerate(self.runs):
            sel = np.flatnonzero(self.run_idx == r)
            if len(sel):
                out[sel] = cmd_at(run, self.t_s[sel][:, None] + offsets[None, :] - delay_s)
        return out

    def data(self, delay_s: float) -> EvalData:
        return EvalData(
            self.t_s,
            np.array([self.runs[i].name for i in self.run_idx], dtype=str),
            self.commands(delay_s),
            self.v_warm,
            self.v_true,
            self.poses,
        )

    def subset(self, keep: np.ndarray) -> EvalSet:
        return EvalSet(
            self.runs, self.run_idx[keep], self.t_s[keep], self.segment[keep],
            self.direction[keep], self.block[keep], self.start_speed[keep], self.v_warm[keep],
            self.v_true[keep], self.poses[keep],
        )  # fmt: skip


def _empty_set(n: int, n_iv: int) -> EvalSet:
    return EvalSet(
        (), np.zeros(0, int), np.zeros(0), np.array([], dtype=str), np.array([], dtype=str),
        np.zeros(0, int), np.zeros(0), np.zeros((0, WARM_N + 1)), np.zeros((0, n + 1)),
        np.zeros((0, n_iv + 1, 3)),
    )  # fmt: skip


def _snap_to_scans(scan_t: np.ndarray, targets: np.ndarray) -> np.ndarray:
    """Indices of the scan stamps nearest to each target time (within half a scan interval)."""
    near = np.clip(np.searchsorted(scan_t, targets), 1, len(scan_t) - 1)
    left = np.abs(scan_t[near - 1] - targets) <= np.abs(scan_t[near] - targets)
    idx = np.where(left, near - 1, near)
    close = np.abs(scan_t[idx] - targets) <= 0.5 * SCAN_DT_S + 1e-9
    return np.unique(idx[close])


def eval_set(
    segments: Sequence[Segment],
    stride_s: float = EVAL_STRIDE_S,
    min_start_speed_mps: float = EVAL_MIN_START_SPEED_MPS,
    horizon_s: float = max(HORIZONS_S),
) -> EvalSet:
    """The pre-registered windows of `segments` (odometry and ICP segments only).

    Start times are t0 + k * stride_s inside the segment, each snapped to the nearest scan stamp
    (scans arrive every 0.1 s; the pose chain starts at a scan). The start wheel speed must be at
    least `min_start_speed_mps`. A window with any failed ICP interval is dropped, and so is one
    with an invalid command anywhere a delay of 0 to MAX_DELAY_S could read, a reverse command
    or wheel speed, or the unresponsive pattern; all of these are model independent. Windows
    need `horizon_s` of data after the start, so the same set serves 0.5, 1.0 and 1.5 s."""
    n = round(horizon_s / DT_S)
    n_iv = round(horizon_s / SCAN_DT_S)
    warm_offsets = (np.arange(WARM_N + 1) - WARM_N) * DT_S
    true_offsets = np.arange(n + 1) * DT_S
    cmd_offsets = (np.arange(WARM_N + n) - WARM_N) * DT_S
    runs: list[Run] = []
    names = ("run", "t", "seg", "dir", "block", "v0", "vw", "vt", "poses")
    cols: dict[str, list] = {k: [] for k in names}
    block_ids: dict[tuple[int, int], int] = {}
    for si, seg in enumerate(segments):
        m = seg.motion
        if m is None or not seg.run.has_odom or len(m.t_s) < 2:
            continue
        ri = next((i for i, r in enumerate(runs) if r is seg.run), len(runs))
        if ri == len(runs):
            runs.append(seg.run)
        lo = seg.t0_s + WARM_N * DT_S + MAX_DELAY_S
        hi = seg.t1_s - horizon_s - SCAN_DT_S
        for i in _snap_to_scans(m.t_s, np.arange(lo, hi, stride_s)):
            t = float(m.t_s[i])
            if not (lo - 1e-9 <= t <= hi + 1e-9) or not m.ok[i : i + n_iv].all():
                continue
            c_slow = cmd_at(seg.run, t + cmd_offsets - MAX_DELAY_S)
            c_fast = cmd_at(seg.run, t + cmd_offsets)
            vw = wheel_at(seg.run, t + warm_offsets)
            vt = wheel_at(seg.run, t + true_offsets)
            finite = all(np.isfinite(a).all() for a in (c_slow, c_fast, vw, vt))
            if not finite or vw[-1] < min_start_speed_mps:
                continue
            ok = True
            for c in (c_slow, c_fast):
                ok &= bool((forward_only(vt[None], c[None]) & forward_only(vw[None], c[None]))[0])
                ok &= not unresponsive(vt[None], c[None, WARM_N:])[0]
            if not ok:
                continue
            block = block_ids.setdefault((si, int((t - seg.t0_s) // BLOCK_S)), len(block_ids))
            cols["run"].append(ri)
            cols["t"].append(t)
            cols["seg"].append(seg.name or f"{seg.run.name}:{seg.i0}")
            cols["dir"].append(seg.direction)
            cols["block"].append(block)
            cols["v0"].append(vw[-1])
            cols["vw"].append(np.maximum(vw, 0.0))
            cols["vt"].append(np.maximum(vt, 0.0))
            cols["poses"].append(
                compose(m.dx_m[i : i + n_iv], m.dy_m[i : i + n_iv], m.dyaw_rad[i : i + n_iv])
            )
    if not cols["t"]:
        return _empty_set(n, n_iv)
    return EvalSet(
        tuple(runs),
        np.array(cols["run"], dtype=int),
        np.array(cols["t"]),
        np.array(cols["seg"], dtype=str),
        np.array(cols["dir"], dtype=str),
        np.array(cols["block"], dtype=int),
        np.array(cols["v0"]),
        np.stack(cols["vw"]),
        np.stack(cols["vt"]),
        np.stack(cols["poses"]),
    )


def window_errors(
    es: EvalSet,
    params: Nominal,
    vehicle: Vehicle,
    residual: Residual | None = None,
    horizons_s: Sequence[float] = HORIZONS_S,
) -> dict[float, dict[str, np.ndarray]]:
    """Per window error at each horizon: `speed` and `yaw` signed (model minus truth), `pos` the
    Euclidean position error. The model reads its own delayed commands of the shared windows."""
    states = predict(es.data(params.command_delay_s), params, vehicle, residual)
    out: dict[float, dict[str, np.ndarray]] = {}
    for h in horizons_s:
        k, j = round(h / DT_S), round(h / SCAN_DT_S)
        out[h] = {
            "speed": states[:, k, 3] - es.v_true[:, k],
            "yaw": states[:, k, 2] - es.poses[:, j, 2],
            "pos": np.hypot(
                states[:, k, 0] - es.poses[:, j, 0], states[:, k, 1] - es.poses[:, j, 1]
            ),
        }
    return out
