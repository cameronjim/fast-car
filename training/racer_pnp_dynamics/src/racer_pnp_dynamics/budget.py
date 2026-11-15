"""M-B3 (the error budget) and M-B2 (direction asymmetry) of pre-registration v0.4 sections 2.2 and
2.3. Measurements on TRAIN data only, committed before any v0.4 fit; they decide nothing.

M-B3, for one model on the train windows at 1.0 s:

* the position error split into the component ALONG the true heading at the window start and the
  component ACROSS it (the windows' poses are in the frame at the start, so along is x and across
  is y); per stratum `RMSE_pos^2 = RMSE_along^2 + RMSE_across^2` exactly, the macro values are the
  means of the per stratum RMSEs (as every v0.2 macro is) and do not add in quadrature;
* oracle "perfect yaw": the rollout with the true yaw substituted at every 0.05 s step (the true
  yaw between two scans is interpolated linearly);
* oracle "perfect speed": the rollout with the true wheel speed substituted at every step;
* P1 of each oracle over P1 of the model, macro and per stratum.
"""

from __future__ import annotations

from collections.abc import Mapping, Sequence
from dataclasses import dataclass

import numpy as np

from .dataset import Segment
from .evaluate import EvalSet
from .icp_motion import SCAN_DT_S
from .model import DT_S, Nominal, Residual, step
from .strata import SPEED_BIN_EDGES_MPS, Strata, assign_strata, macro_rmse, stratum_rmse
from .vehicle import Vehicle
from .windows import INTERVAL_N, WARM_N, warm_state, wheel_at

HORIZON_S = 1.0
UNREACHABLE_IF_PERFECT_YAW_RATIO_ABOVE = 0.95  # yaml dynamics.budget_measurement
# Brief 37b: written as "not expected", never "impossible": the perfect yaw oracle is a diagnostic,
# not a bound.
UNREACHABLE_SENTENCE = "a yaw-only model is not expected to meet item 1 of GP2d-R on this data"


@dataclass(frozen=True)
class Budget:
    horizon_s: float
    windows: int
    strata: Strata
    pos: np.ndarray  # (E,) Euclidean position error of the model
    along: np.ndarray  # (E,) signed, model minus truth
    across: np.ndarray
    pos_perfect_yaw: np.ndarray
    pos_perfect_speed: np.ndarray

    def table(self) -> dict[str, dict[str, float]]:
        """Per stratum and `macro`: windows, RMSE of position, along, across and of both oracles,
        and the two oracle ratios (oracle P1 over model P1)."""
        rows: dict[str, dict[str, float]] = {}
        arrays = {
            "pos": self.pos,
            "along": self.along,
            "across": self.across,
            "pos_perfect_yaw": self.pos_perfect_yaw,
            "pos_perfect_speed": self.pos_perfect_speed,
        }
        per = {k: stratum_rmse(a, self.strata) for k, a in arrays.items()}
        for name in self.strata.names:
            rows[name] = {"windows": float(self.strata.counts[name])}
            rows[name].update({k: per[k][name] for k in arrays})
        rows["macro"] = {"windows": float(self.windows)}
        rows["macro"].update({k: macro_rmse(a, self.strata) for k, a in arrays.items()})
        for row in rows.values():
            row["ratio_perfect_yaw"] = row["pos_perfect_yaw"] / row["pos"]
            row["ratio_perfect_speed"] = row["pos_perfect_speed"] / row["pos"]
        return rows

    def perfect_yaw_ratio(self) -> float:
        return self.table()["macro"]["ratio_perfect_yaw"]


def _true_yaw_steps(es: EvalSet, n: int) -> np.ndarray:
    """(E, n + 1) true yaw at every model step, linear between the scan stamps."""
    j = n // INTERVAL_N
    yaw = es.poses[:, : j + 1, 2]
    out = np.zeros((len(es), n + 1))
    for k in range(n + 1):
        lo = min(k // INTERVAL_N, j)
        hi = min(lo + 1, j)
        frac = (k - lo * INTERVAL_N) / INTERVAL_N
        out[:, k] = (1.0 - frac) * yaw[:, lo] + frac * yaw[:, hi]
    return out


def oracle_positions(
    es: EvalSet,
    nominal: Nominal,
    vehicle: Vehicle,
    residual: Residual | None = None,
    horizon_s: float = HORIZON_S,
) -> dict[str, np.ndarray]:
    """End positions (E, 2) at `horizon_s` in the start frame of: the model, perfect yaw, perfect
    speed. Same warm start and same delayed commands as `evaluate.predict`."""
    data = es.data(nominal.command_delay_s)
    n = round(horizon_s / DT_S)
    controls = data.cmds[:, WARM_N : WARM_N + n, :]
    start = warm_state(nominal, vehicle, data.cmds, data.v_warm, residual)
    start[:, :3] = 0.0
    out: dict[str, np.ndarray] = {}
    true_yaw = _true_yaw_steps(es, n)
    for name in ("model", "perfect_yaw", "perfect_speed"):
        state = start.copy()
        for k in range(n):
            if name == "perfect_yaw":
                state[:, 2] = true_yaw[:, k]
            if name == "perfect_speed":
                state[:, 3] = es.v_true[:, k]
            state = step(state, controls[:, k, :], nominal, vehicle, residual)
        out[name] = state[:, :2].copy()
    return out


def error_budget(
    es: EvalSet,
    nominal: Nominal,
    vehicle: Vehicle,
    residual: Residual | None = None,
    horizon_s: float = HORIZON_S,
) -> Budget:
    """M-B3 for one model on the windows of `es` (train windows when it is used as stage 1)."""
    if len(es) == 0:
        raise ValueError("no windows")
    j = round(horizon_s / SCAN_DT_S)
    truth = es.poses[:, j, :2]
    pos = oracle_positions(es, nominal, vehicle, residual, horizon_s)
    diff = pos["model"] - truth
    return Budget(
        horizon_s=horizon_s,
        windows=len(es),
        strata=assign_strata(es.direction, es.start_speed),
        pos=np.hypot(diff[:, 0], diff[:, 1]),
        along=diff[:, 0],
        across=diff[:, 1],
        pos_perfect_yaw=np.linalg.norm(pos["perfect_yaw"] - truth, axis=1),
        pos_perfect_speed=np.linalg.norm(pos["perfect_speed"] - truth, axis=1),
    )


def render_mb3(budget: Budget, model_name: str, notes: Sequence[str]) -> str:
    t = budget.table()
    ratio = t["macro"]["ratio_perfect_yaw"]
    lines = [
        f"# M-B3: error budget of {model_name} on the train windows at {budget.horizon_s:g} s",
        "",
        (
            "Pre-registration v0.4 section 2.2. A measurement on TRAIN data only (train and the "
            "internal validation part of the train segments, truth T2), produced and committed "
            "before any v0.4 fit. It decides nothing."
        ),
        "",
        *[f"- {n}" for n in notes],
        "",
        (
            "Along and across are the components of the position error in the true heading at the "
            "window start (model minus truth). Per stratum position^2 = along^2 + across^2; the "
            "macro row is the mean of the per stratum RMSEs and does not add in quadrature. "
            "Oracles: perfect yaw puts the true yaw into the rollout at every step, perfect speed "
            "the true wheel speed. Ratio = oracle P1 over the model's P1."
        ),
        "",
        (
            "| stratum | windows | P1 (m) | along (m) | across (m) | perfect yaw P1 | "
            "perfect speed P1 | ratio perfect yaw | ratio perfect speed |"
        ),
        "|---|---|---|---|---|---|---|---|---|",
    ]
    for name, r in t.items():
        lines.append(
            f"| {name} | {int(r['windows'])} | {r['pos']:.4f} | {r['along']:.4f} | "
            f"{r['across']:.4f} | {r['pos_perfect_yaw']:.4f} | {r['pos_perfect_speed']:.4f} | "
            f"{r['ratio_perfect_yaw']:.3f} | {r['ratio_perfect_speed']:.3f} |"
        )
    lines += ["", f"Perfect yaw ratio (macro): {ratio:.3f}.", ""]
    if ratio > UNREACHABLE_IF_PERFECT_YAW_RATIO_ABOVE:
        lines += [
            (
                f"The perfect yaw ratio is above {UNREACHABLE_IF_PERFECT_YAW_RATIO_ABOVE}: "
                f"{UNREACHABLE_SENTENCE}. R4 is still trained and scored, and the gate is still "
                "applied as written."
            ),
            "",
        ]
    else:
        lines += [
            (
                f"The perfect yaw ratio is not above {UNREACHABLE_IF_PERFECT_YAW_RATIO_ABOVE}: "
                "the pre-registered sentence about item 1 of GP2d-R is not written."
            ),
            "",
        ]
    return "\n".join(lines)


# ---- M-B2 -----------------------------------------------------------------------------------

STEER_LIMIT_TOL_RAD = 0.01  # "within 0.01 rad of the limit" (pre-registration 2.3 / brief 37)
MIN_MOVING_MPS = SPEED_BIN_EDGES_MPS[0]


def _bin_label(b: int) -> str:
    e = SPEED_BIN_EDGES_MPS
    return f"{e[b]:g}-{e[b + 1]:g}" if b + 1 < len(e) else f"{e[b]:g}+"


def asymmetry_table(
    segments: Sequence[Segment],
    es: EvalSet,
    nominal: Nominal,
    vehicle: Vehicle,
    residual: Residual | None = None,
    horizon_s: float = HORIZON_S,
) -> list[dict[str, float | str]]:
    """One row per direction and speed bin (start speed bins of the strata, unmerged): mean wheel
    speed and share of time at the steering limit over the 50 Hz samples of the segments that fall
    in the bin (by the sample's own wheel speed), and mean truth yaw rate against the model's on the
    same windows (bias = model minus truth, and RMS of the difference, rad/s)."""
    from .evaluate import window_errors

    rows: list[dict[str, float | str]] = []
    err = window_errors(es, nominal, vehicle, residual, (horizon_s,))[horizon_s]
    j = round(horizon_s / SCAN_DT_S)
    truth_rate = es.poses[:, j, 2] / horizon_s
    model_rate = (err["yaw"] + es.poses[:, j, 2]) / horizon_s
    from .strata import bin_of

    win_bin = bin_of(es.start_speed)
    edges = np.array(SPEED_BIN_EDGES_MPS)
    for direction in sorted({s.direction for s in segments}):
        speed_all, steer_all = [], []
        for seg in segments:
            if seg.direction != direction or not seg.run.has_odom:
                continue
            sl = slice(seg.i0, seg.i1)
            v = seg.run.wheel_speed_mps[sl]
            c = seg.run.steering_cmd_rad[sl]
            ok = np.isfinite(v) & np.isfinite(c)
            speed_all.append(v[ok])
            steer_all.append(c[ok])
        if not speed_all:
            continue
        v, c = np.concatenate(speed_all), np.concatenate(steer_all)
        v_bin = np.clip(np.searchsorted(edges, v, side="right") - 1, -1, len(edges) - 1)
        at_limit = (np.abs(c - vehicle.steer_max_rad) <= STEER_LIMIT_TOL_RAD) | (
            np.abs(c - vehicle.steer_min_rad) <= STEER_LIMIT_TOL_RAD
        )
        for b in range(len(edges)):
            sel = v_bin == b
            w = (es.direction == direction) & (win_bin == b)
            if not sel.any() and not w.any():
                continue
            d = model_rate[w] - truth_rate[w]
            rows.append(
                {
                    "direction": direction,
                    "speed_bin_mps": _bin_label(b),
                    "samples_s": float(sel.sum() * 0.02),
                    "mean_wheel_speed_mps": float(v[sel].mean()) if sel.any() else float("nan"),
                    "steer_at_limit_share": float(at_limit[sel].mean())
                    if sel.any()
                    else float("nan"),
                    "windows": float(w.sum()),
                    "truth_yaw_rate_mean": float(truth_rate[w].mean()) if w.any() else float("nan"),
                    "model_yaw_rate_mean": float(model_rate[w].mean()) if w.any() else float("nan"),
                    "yaw_rate_bias": float(d.mean()) if w.any() else float("nan"),
                    "yaw_rate_rms": float(np.sqrt(np.mean(d**2))) if w.any() else float("nan"),
                }
            )
    return rows


def lap_radius(segment: Segment, min_moving_mps: float = MIN_MOVING_MPS) -> dict[str, float | str]:
    """Lap radius of one segment from wheel distance and the T2 yaw: radius = distance / |yaw|
    over the intervals whose ICP match is ok and the car moving (one lap turns 2 pi, so this is
    lap length over 2 pi). Used when brief 36's lap reference is not on main."""
    m = segment.motion
    out: dict[str, float | str] = {
        "segment": segment.name,
        "direction": segment.direction,
        "cap_mps": float("nan") if segment.cap_mps is None else float(segment.cap_mps),
        "radius_m": float("nan"),
        "wheel_distance_m": float("nan"),
        "yaw_turned_rad": float("nan"),
    }
    if m is None or not segment.run.has_odom:
        return out
    sel = np.flatnonzero((m.t_s >= segment.t0_s) & (m.t_s + m.dt_s <= segment.t1_s) & m.ok)
    if len(sel) == 0:
        return out
    wheel = wheel_at(segment.run, m.t_s[sel] + 0.5 * m.dt_s[sel])
    keep = np.isfinite(wheel) & (wheel >= min_moving_mps)
    dist = float(np.sum(wheel[keep] * m.dt_s[sel][keep]))
    yaw = float(np.sum(m.dyaw_rad[sel][keep]))
    out.update(wheel_distance_m=dist, yaw_turned_rad=yaw)
    if abs(yaw) > 0:
        out["radius_m"] = dist / abs(yaw)
    return out


def render_mb2(
    rows: Sequence[Mapping[str, float | str]],
    radii: Sequence[Mapping[str, float | str]],
    model_name: str,
    notes: Sequence[str],
) -> str:
    lines = [
        f"# M-B2: direction asymmetry, train segments of 2026-11-14, against {model_name}",
        "",
        (
            "Pre-registration v0.4 section 2.3. A measurement on the TRAIN segments of 2026-11-14 "
            "only, produced and committed before any v0.4 fit. No threshold, it decides nothing."
        ),
        "",
        *[f"- {n}" for n in notes],
        "",
        (
            "Mean yaw rate columns are the 1.0 s window yaw change divided by 1.0 s (rad/s); bias "
            "is model minus truth (T2), RMS is of the same difference. Speed bins are the start "
            "speed bins of the strata, unmerged, and bin the 50 Hz samples by their own wheel "
            "speed for the first two columns."
        ),
        "",
        (
            "| direction | speed bin | seconds | mean wheel speed | steering at limit | windows | "
            "truth yaw rate | model yaw rate | bias | RMS |"
        ),
        "|---|---|---|---|---|---|---|---|---|---|",
    ]
    for r in rows:
        lines.append(
            f"| {r['direction']} | {r['speed_bin_mps']} | {r['samples_s']:.1f} | "
            f"{r['mean_wheel_speed_mps']:.3f} | {r['steer_at_limit_share']:.3f} | "
            f"{int(r['windows'])} | {r['truth_yaw_rate_mean']:.3f} | "
            f"{r['model_yaw_rate_mean']:.3f} | {r['yaw_rate_bias']:.3f} | "
            f"{r['yaw_rate_rms']:.3f} |"
        )
    lines += [
        "",
        "Lap radius per segment (wheel distance over the yaw turned, T2 intervals that matched):",
        "",
        "| segment | direction | cap (m/s) | wheel distance (m) | yaw turned (rad) | radius (m) |",
        "|---|---|---|---|---|---|",
    ]
    for r in radii:
        lines.append(
            f"| {r['segment']} | {r['direction']} | {r['cap_mps']:.1f} | "
            f"{r['wheel_distance_m']:.2f} | {r['yaw_turned_rad']:.2f} | {r['radius_m']:.3f} |"
        )
    lines.append("")
    return "\n".join(lines)


__all__ = [
    "Budget",
    "asymmetry_table",
    "error_budget",
    "lap_radius",
    "oracle_positions",
    "render_mb2",
    "render_mb3",
]
