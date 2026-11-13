"""Is a fitted parameter identified by the data? Profiles, flags and an independent steering lag.

A profile is the held-out error as a function of one parameter with the others refit (the callers
build it, see fit.py). `assess` turns a profile and the fitted value into a verdict: a parameter
within 0.5 percent of a bound, or whose profile is flat (within 2 percent of the minimum) over
more than half of its range, is NOT identified, and is listed as such in the metrics rather than
hidden. Nothing here picks a value.

`steering_lag_s` is the cross-check that does not use the fit: the lag of the peak of the
cross-correlation between the steering command and the path curvature (ICP yaw rate over ICP
ground speed).
"""

from __future__ import annotations

from collections.abc import Sequence
from dataclasses import dataclass

import numpy as np

from .dataset import Segment
from .icp_motion import SCAN_DT_S
from .windows import cmd_at

BOUND_TOLERANCE = 0.005  # of the bound range
FLAT_TOLERANCE = 0.02  # a profile point within this of the minimum is flat
FLAT_SHARE = 0.5  # flat over more than this share of the range is "not identified"

# Grids of the profiles. Time constants span two decades, so their grid and their measure of
# "share of the range" are logarithmic; the delay and the dead zone are linear.
TAU_GRID_S = (0.005, 0.01, 0.02, 0.05, 0.1, 0.2, 0.3, 0.5, 0.75, 1.0)


@dataclass(frozen=True)
class Verdict:
    identified: bool
    reasons: tuple[str, ...]

    def text(self) -> str:
        return "; ".join(self.reasons)


def flat_share(grid: Sequence[float], errors: Sequence[float], log_scale: bool = False) -> float:
    """Share of the grid's range (linear or log measure) where the profile is within
    FLAT_TOLERANCE of its minimum. An interval counts when both of its ends do."""
    x = np.asarray(grid, dtype=float)
    e = np.asarray(errors, dtype=float)
    ok = np.isfinite(e)
    if ok.sum() < 2:
        return 0.0
    flat = ok & (e <= (1.0 + FLAT_TOLERANCE) * np.nanmin(e))
    pos = np.log(x) if log_scale else x
    widths = np.diff(pos)
    both = flat[:-1] & flat[1:]
    total = float(widths.sum())
    return float(widths[both].sum() / total) if total > 0 else 0.0


def assess(
    value: float,
    bounds: tuple[float, float],
    grid: Sequence[float],
    errors: Sequence[float],
    log_scale: bool = False,
    ends_are_bounds: bool = True,
) -> Verdict:
    """The two rules. `ends_are_bounds` False skips the bound rule (no physical bound on that
    side, only the extent of a grid)."""
    lo, hi = bounds
    reasons: list[str] = []
    if ends_are_bounds:
        near = BOUND_TOLERANCE * (hi - lo)
        if value - lo <= near:
            reasons.append(f"on its lower bound {lo:g}")
        elif hi - value <= near:
            reasons.append(f"on its upper bound {hi:g}")
    share = flat_share(grid, errors, log_scale)
    if share > FLAT_SHARE:
        reasons.append(
            f"profile flat within {FLAT_TOLERANCE:.0%} of its minimum over {share:.0%} of its "
            f"range ({'log' if log_scale else 'linear'} measure)"
        )
    return Verdict(not reasons, tuple(reasons))


def profile_table(grid: Sequence[float], errors: Sequence[float]) -> list[dict[str, float]]:
    """Rows for the metrics: parameter value, held-out error, error over the minimum."""
    e = np.asarray(errors, dtype=float)
    best = float(np.nanmin(e)) if np.isfinite(e).any() else float("nan")
    return [
        {"value": float(x), "rmse": float(v), "over_min": float(v / best - 1.0)}
        for x, v in zip(grid, e, strict=True)
    ]


# ---- steering lag from the command and the path curvature ------------------------------------

LAG_STEP_S = 0.02
LAG_MAX_S = 0.60
MIN_LAG_SPEED_MPS = 0.4


@dataclass(frozen=True)
class SteeringLag:
    lag_s: float
    peak_correlation: float
    n_intervals: int
    lags_s: tuple[float, ...]
    correlation: tuple[float, ...]


def steering_lag_s(
    segments: Sequence[Segment],
    lag_step_s: float = LAG_STEP_S,
    lag_max_s: float = LAG_MAX_S,
    min_speed_mps: float = MIN_LAG_SPEED_MPS,
) -> SteeringLag | None:
    """Lag of the peak of corr(curvature(t), steering_cmd(t - lag)), pooled over the segments.

    Curvature is the ICP yaw change over the ICP path length of a 0.1 s scan interval (so it does
    not depend on the wheel speed), taken at the interval's middle; only good ICP intervals with
    ICP ground speed above `min_speed_mps`. The peak is refined by a parabola through the three
    lags around the maximum. None when there is no data."""
    lags = np.arange(0.0, lag_max_s + 1e-9, lag_step_s)
    kappa: list[np.ndarray] = []
    cmds: list[np.ndarray] = []
    for seg in segments:
        m = seg.motion
        if m is None:
            continue
        ground = np.hypot(m.dx_m, m.dy_m) / m.dt_s
        pick = np.flatnonzero(
            m.ok
            & (ground >= min_speed_mps)
            & (m.t_s >= seg.t0_s + lag_max_s)
            & (m.t_s <= seg.t1_s - SCAN_DT_S)
        )
        if len(pick) == 0:
            continue
        mid = m.t_s[pick] + 0.5 * SCAN_DT_S
        path = np.hypot(m.dx_m[pick], m.dy_m[pick])
        c = np.stack([cmd_at(seg.run, mid - lag)[:, 1] for lag in lags], axis=1)
        ok = np.isfinite(c).all(axis=1)
        kappa.append((m.dyaw_rad[pick] / path)[ok])
        cmds.append(c[ok])
    if not kappa:
        return None
    k = np.concatenate(kappa)
    c = np.concatenate(cmds)
    if len(k) < 20:
        return None
    k = k - k.mean()
    corr = np.array(
        [
            float(np.corrcoef(k, c[:, j])[0, 1]) if c[:, j].std() > 0 else float("nan")
            for j in range(len(lags))
        ]
    )
    j = int(np.nanargmax(corr))
    lag = float(lags[j])
    if 0 < j < len(lags) - 1:
        y0, y1, y2 = corr[j - 1], corr[j], corr[j + 1]
        denom = y0 - 2.0 * y1 + y2
        if denom < 0:
            lag = float(lags[j] + 0.5 * (y0 - y2) / denom * lag_step_s)
    return SteeringLag(
        lag_s=lag,
        peak_correlation=float(corr[j]),
        n_intervals=len(k),
        lags_s=tuple(float(x) for x in lags),
        correlation=tuple(float(x) for x in corr),
    )
