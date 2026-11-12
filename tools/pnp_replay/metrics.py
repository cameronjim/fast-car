"""The numbers of a replay, as pure functions of arrays.

Perception, per scan the car was moving for (wheel speed above 0.05 m/s):
  intrusion          mean, p90 and the share over 0.10 of the track's intrusion_fraction
  boundary residual  p50 and p90 of boundary_residual_m
  valid_count        p10 and the share under 6
  clearance deficit  for each valid station within 2.0 m of the car, min(half_width_left,
                     half_width_right) minus the distance from the station to the nearest scan
                     point (body filtered, base_link): how much room the track claims that the
                     scan says is not there. p90 over all stations and the share over 0.15 m.

Planner, over the ticks that had a scan under 0.35 s old and odometry under 0.10 s old:
  state counts, the share of ticks with a speed command of exactly 0 and with one strictly
  between 0 and the dead zone, the mean collision fraction of the ticks that solved, the
  tick to tick change of the steering command over DRIVING ticks (std, sign flips per second,
  p99 of the absolute value), the solve time (of the machine that ran the replay) and, when the
  segment has a recorded status, the share of ticks in the same state as the recorded one
  nearest in time.
"""

from __future__ import annotations

import math
from typing import Any

import numpy as np
from numpy.typing import NDArray

MOVING_SPEED_MPS = 0.05
STATION_RADIUS_M = 2.0
SHORT_VALID_COUNT = 6
INTRUSION_LIMIT = 0.10
DEFICIT_LIMIT_M = 0.15
SCAN_FRESH_S = 0.35
ODOM_FRESH_S = 0.10
DEFAULT_DEAD_ZONE_MPS = 0.20
AGREEMENT_MAX_GAP_S = 0.02

STATE_NAMES = {
    0: "STARTING",
    1: "DRIVING",
    2: "NO_TRACK",
    3: "NO_SCAN",
    4: "NO_ODOM",
    5: "BLOCKED",
    6: "FAULT",
}
STATE_DRIVING = 1
STATE_BLOCKED = 5


def _nan() -> float:
    return math.nan


def _stat(fn: Any, values: NDArray[np.float64], *args: Any) -> float:
    return float(fn(values, *args)) if len(values) else _nan()


def speed_at(
    times: NDArray[np.float64], odom_t: NDArray[np.float64], odom_v: NDArray[np.float64]
) -> NDArray[np.float64]:
    """Wheel speed interpolated at `times` (0 where there is no odometry at all)."""
    if len(odom_t) == 0:
        return np.zeros(len(times))
    return np.interp(times, odom_t, odom_v)


def moving_mask(speed_mps: NDArray[np.float64], threshold: float = MOVING_SPEED_MPS) -> Any:
    return np.abs(speed_mps) > threshold


def clearance_deficits(
    x: NDArray[np.floating],
    y: NDArray[np.floating],
    half_left: NDArray[np.floating],
    half_right: NDArray[np.floating],
    valid_count: int,
    points: NDArray[np.floating],
    radius_m: float = STATION_RADIUS_M,
) -> NDArray[np.float64]:
    """Clearance deficit of each valid station of one track within radius_m of the car (see the
    module docstring). Empty when there are no points or no such station."""
    n = int(valid_count)
    if n < 1 or len(points) == 0:
        return np.zeros(0)
    sx = np.asarray(x[:n], dtype=np.float64)
    sy = np.asarray(y[:n], dtype=np.float64)
    near = np.hypot(sx, sy) <= radius_m
    if not near.any():
        return np.zeros(0)
    px = np.asarray(points[:, 0], dtype=np.float64)
    py = np.asarray(points[:, 1], dtype=np.float64)
    nearest = np.sqrt(
        ((sx[near, None] - px[None, :]) ** 2 + (sy[near, None] - py[None, :]) ** 2).min(axis=1)
    )
    claimed = np.minimum(
        np.asarray(half_left[:n], dtype=np.float64), np.asarray(half_right[:n], dtype=np.float64)
    )[near]
    return claimed - nearest


def perception_metrics(
    valid_count: NDArray[np.integer],
    intrusion: NDArray[np.float64],
    residual: NDArray[np.float64],
    deficits: NDArray[np.float64],
) -> dict[str, Any]:
    """Perception numbers over a set of scans. Intrusion and residual count only scans with at
    least 2 valid stations (a track with fewer has nothing to check) and ignore NaN; `deficits`
    is every station's deficit of those scans, concatenated."""
    vc = np.asarray(valid_count)
    has_track = vc >= 2
    intr = np.asarray(intrusion, dtype=np.float64)[has_track]
    intr = intr[np.isfinite(intr)]
    res = np.asarray(residual, dtype=np.float64)[has_track]
    res = res[np.isfinite(res)]
    d = np.asarray(deficits, dtype=np.float64)
    d = d[np.isfinite(d)]
    return {
        "scans": len(vc),
        "scans_with_track": int(has_track.sum()),
        "intrusion_mean": _stat(np.mean, intr),
        "intrusion_p90": _stat(np.percentile, intr, 90),
        "intrusion_share_over_0.10": _stat(lambda v: np.mean(v > INTRUSION_LIMIT), intr),
        "boundary_residual_p50_m": _stat(np.percentile, res, 50),
        "boundary_residual_p90_m": _stat(np.percentile, res, 90),
        "valid_count_p10": _stat(np.percentile, vc.astype(np.float64), 10),
        "valid_count_share_under_6": _stat(lambda v: np.mean(v < SHORT_VALID_COUNT), vc),
        "clearance_deficit_p90_m": _stat(np.percentile, d, 90),
        "clearance_deficit_share_over_0.15": _stat(lambda v: np.mean(v > DEFICIT_LIMIT_M), d),
        "clearance_deficit_stations": len(d),
    }


def eligible_ticks(
    now_s: NDArray[np.float64],
    scan_age_s: NDArray[np.float64],
    odom_t: NDArray[np.float64],
) -> NDArray[np.bool_]:
    """Ticks with a scan under 0.35 s old (the CSV writes -1 for no scan) and odometry under
    0.10 s old, the latest odometry arrival at or before the tick."""
    scan_ok = (scan_age_s >= 0.0) & (scan_age_s < SCAN_FRESH_S)
    if len(odom_t) == 0:
        return np.zeros(len(now_s), dtype=bool)
    idx = np.searchsorted(odom_t, now_s, side="right") - 1
    age = np.where(idx >= 0, now_s - odom_t[np.clip(idx, 0, None)], np.inf)
    return scan_ok & (age < ODOM_FRESH_S)


def steering_change_stats(steering: NDArray[np.float64], tick_s: float) -> dict[str, float]:
    """Std, sign flips per second and p99 absolute value of the change of a steering series
    between consecutive samples. A flip is a change whose sign differs from the previous
    change's; zero changes carry the previous sign (a held command is not a reversal)."""
    s = np.asarray(steering, dtype=np.float64)
    if len(s) < 3:
        return {"std_rad": _nan(), "sign_flips_per_s": _nan(), "p99_abs_rad": _nan()}
    ds = np.diff(s)
    nonzero = ds[ds != 0.0]
    flips = int((np.sign(nonzero[1:]) != np.sign(nonzero[:-1])).sum()) if len(nonzero) > 1 else 0
    return {
        "std_rad": float(ds.std()),
        "sign_flips_per_s": flips / (len(ds) * tick_s),
        "p99_abs_rad": float(np.percentile(np.abs(ds), 99)),
    }


def planner_metrics(
    now_s: NDArray[np.float64],
    state: NDArray[np.integer],
    cmd_speed_mps: NDArray[np.float64],
    cmd_steering_rad: NDArray[np.float64],
    collision_fraction: NDArray[np.float64],
    solve_time_s: NDArray[np.float64],
    scan_age_s: NDArray[np.float64],
    odom_t: NDArray[np.float64],
    dead_zone_mps: float = DEFAULT_DEAD_ZONE_MPS,
    recorded_t: NDArray[np.float64] | None = None,
    recorded_state: NDArray[np.integer] | None = None,
) -> dict[str, Any]:
    """Planner numbers over the eligible ticks of a replay (see the module docstring)."""
    ok = eligible_ticks(now_s, scan_age_s, odom_t)
    t, st = now_s[ok], np.asarray(state)[ok]
    v, steer = cmd_speed_mps[ok], cmd_steering_rad[ok]
    n = int(ok.sum())
    counts = {STATE_NAMES.get(int(k), str(int(k))): int((st == k).sum()) for k in np.unique(st)}
    tick_s = float(np.median(np.diff(t))) if n > 1 else 0.02
    solved = (st == STATE_DRIVING) | (st == STATE_BLOCKED)
    driving = st == STATE_DRIVING
    out: dict[str, Any] = {
        "ticks": len(now_s),
        "eligible_ticks": n,
        "state_counts": counts,
        "speed_zero_share": float(np.mean(v == 0.0)) if n else _nan(),
        "speed_below_dead_zone_share": float(np.mean((v > 0.0) & (v < dead_zone_mps)))
        if n
        else _nan(),
        "dead_zone_mps": float(dead_zone_mps),
        "collision_fraction_mean": _stat(np.mean, collision_fraction[ok][solved]),
        "steering_change": steering_change_stats(steer[driving], tick_s),
        "solve_time_ms_this_machine": {
            "p50": _stat(np.percentile, solve_time_s[ok][solved] * 1e3, 50),
            "p99": _stat(np.percentile, solve_time_s[ok][solved] * 1e3, 99),
        },
    }
    if recorded_t is not None and recorded_state is not None and len(recorded_t) and n:
        out["agreement_with_recorded"] = state_agreement(t, st, recorded_t, recorded_state)
    return out


def state_agreement(
    t: NDArray[np.float64],
    state: NDArray[np.integer],
    recorded_t: NDArray[np.float64],
    recorded_state: NDArray[np.integer],
    max_gap_s: float = AGREEMENT_MAX_GAP_S,
) -> dict[str, Any]:
    """Share of ticks in the same state as the recorded status nearest in time (ticks with none
    within max_gap_s are left out), and the confusion as {replayed->recorded: ticks}."""
    order = np.argsort(recorded_t)
    rt, rs = np.asarray(recorded_t)[order], np.asarray(recorded_state)[order]
    idx = np.clip(np.searchsorted(rt, t), 1, len(rt) - 1) if len(rt) > 1 else np.zeros(len(t), int)
    if len(rt) > 1:
        left, right = idx - 1, idx
        idx = np.where(np.abs(t - rt[left]) <= np.abs(t - rt[right]), left, right)
    gap = np.abs(t - rt[idx])
    use = gap <= max_gap_s
    matched = np.asarray(state)[use]
    recorded = rs[idx][use]
    confusion: dict[str, int] = {}
    for a, b in zip(matched, recorded, strict=True):
        key = f"{STATE_NAMES.get(int(a), a)}->{STATE_NAMES.get(int(b), b)}"
        confusion[key] = confusion.get(key, 0) + 1
    return {
        "share": float(np.mean(matched == recorded)) if len(matched) else _nan(),
        "ticks_compared": int(use.sum()),
        "replayed_to_recorded": dict(sorted(confusion.items())),
    }


def recorded_state_counts(
    status_t: NDArray[np.float64],
    status_state: NDArray[np.float64],
    t_from: float,
    t_to: float,
) -> dict[str, int]:
    """State counts of the recorded status messages in [t_from, t_to]."""
    m = (status_t >= t_from) & (status_t <= t_to)
    st = status_state[m].astype(int)
    return {STATE_NAMES.get(int(k), str(int(k))): int((st == k).sum()) for k in np.unique(st)}
