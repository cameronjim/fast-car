"""Two direct estimates of the speed command dead zone, from the recorded data alone.

(a) Launches from rest: the command at which the wheel speed first exceeds LAUNCH_SPEED_MPS after
    the wheels were still for REST_S. A launch also gives the highest command that did NOT move
    the car during the rest (commands newer than SETTLE_S have not had time to), so the threshold
    lies in (c_rest_max, c_launch]; a launch whose interval is wider than RESOLVED_WIDTH_MPS says
    little (the command jumped over the zone), and one with no interval at all (the car stood
    still under a command that was already that high) is not a dead zone observation.
(b) Steady state: bins of a command that was constant (within STEADY_BAND_MPS) for STEADY_S while
    the wheel speed was constant too. The largest bin whose wheel speed is zero and the smallest
    bin whose wheel speed is above MOVING_MPS bracket the dead zone from the rolling side.

These are findings, not fit inputs: the fitted value is chosen by held-out speed error (fit.py)
and compared with these brackets.
"""

from __future__ import annotations

from collections.abc import Sequence
from dataclasses import dataclass
from itertools import pairwise

import numpy as np

from .bags import GRID_DT_S, Run
from .windows import SpeedWindows

LAUNCH_SPEED_MPS = 0.03
STILL_MPS = 0.01
REST_S = 0.5
SETTLE_S = 0.1  # commands this recent have not had time to move the car (they are not "rest")
RESOLVED_WIDTH_MPS = 0.15
STEADY_S = 0.6
STEADY_BAND_MPS = 0.03
WHEEL_STEADY_MPS = 0.03
ZERO_WHEEL_MPS = 0.02
MOVING_MPS = 0.05
ZERO_SHARE = 0.95
MIN_BIN_SAMPLES = 20
BIN_WIDTH_MPS = 0.01
BIN_MAX_MPS = 0.50


@dataclass(frozen=True)
class Launch:
    run_name: str
    t_s: float
    cmd_launch_mps: float  # command that was applied when the wheels first exceeded the threshold
    cmd_rest_max_mps: float  # highest command applied during the rest before it

    @property
    def width_mps(self) -> float:
        return self.cmd_launch_mps - self.cmd_rest_max_mps


def find_launches(run: Run, delay_s: float = 0.0) -> list[Launch]:
    """Every launch from rest with a positive command. `delay_s` shifts the command back so the
    value paired with the wheel speed is the one the wheels were answering."""
    wheel = run.wheel_speed_mps
    cmd = run.speed_cmd_mps
    if not np.isfinite(wheel).any():
        return []
    n = len(wheel)
    rest_n = round(REST_S / GRID_DT_S)
    settle_n = round(SETTLE_S / GRID_DT_S)
    lag = round(delay_s / GRID_DT_S)
    out: list[Launch] = []
    last = -10 * rest_n
    for k in range(rest_n + settle_n + lag + 1, n):
        if not (wheel[k] > LAUNCH_SPEED_MPS and wheel[k - 1] <= LAUNCH_SPEED_MPS):
            continue
        still = wheel[k - rest_n - settle_n : k - settle_n]  # the rise to 0.03 may take SETTLE_S
        if not (np.isfinite(still).all() and (np.abs(still) < STILL_MPS).all()):
            continue
        if k - last < rest_n:
            continue
        c_launch = cmd[k - lag]
        c_rest = cmd[k - rest_n - lag - 1 : k - lag - settle_n]
        if not (np.isfinite(c_launch) and np.isfinite(c_rest).all()) or c_launch <= 0.0:
            continue
        out.append(Launch(run.name, float(run.t_s[k]), float(c_launch), float(np.max(c_rest))))
        last = k
    return out


def launch_summary(launches: Sequence[Launch]) -> dict[str, object]:
    """Median and 10th to 90th percentile of the launch command, over every launch and over the
    resolved ones (interval no wider than RESOLVED_WIDTH_MPS)."""

    def stats(items: Sequence[Launch]) -> dict[str, float]:
        if not items:
            return {"n": 0}
        c = np.array([x.cmd_launch_mps for x in items])
        lo = np.array([x.cmd_rest_max_mps for x in items])
        return {
            "n": len(items),
            "median_mps": float(np.median(c)),
            "p10_mps": float(np.percentile(c, 10)),
            "p90_mps": float(np.percentile(c, 90)),
            "rest_max_median_mps": float(np.median(lo)),
            "rest_max_p90_mps": float(np.percentile(lo, 90)),
        }

    resolved = [x for x in launches if 0.0 < x.width_mps <= RESOLVED_WIDTH_MPS]
    return {
        "launch_speed_mps": LAUNCH_SPEED_MPS,
        "all": stats(launches),
        "resolved": stats(resolved),
        "resolved_width_mps": RESOLVED_WIDTH_MPS,
    }


def steady_samples(run: Run) -> tuple[np.ndarray, np.ndarray]:
    """(command, wheel speed) at the samples where both have been constant for STEADY_S."""
    cmd, wheel = run.speed_cmd_mps, run.wheel_speed_mps
    n = round(STEADY_S / GRID_DT_S)
    if len(cmd) <= n or not np.isfinite(wheel).any():
        return np.zeros(0), np.zeros(0)
    from numpy.lib.stride_tricks import sliding_window_view

    c_win = sliding_window_view(cmd, n + 1)
    w_win = sliding_window_view(wheel, n + 1)
    with np.errstate(invalid="ignore"):
        ok = np.isfinite(c_win).all(axis=1) & np.isfinite(w_win).all(axis=1)
        ok &= (c_win.max(axis=1) - c_win.min(axis=1)) < STEADY_BAND_MPS
        ok &= (w_win.max(axis=1) - w_win.min(axis=1)) < WHEEL_STEADY_MPS
    return c_win[ok, -1], w_win[ok, -1]


def steady_bins(runs: Sequence[Run]) -> list[dict[str, float]]:
    """Per command bin (BIN_WIDTH_MPS wide, 0 to BIN_MAX_MPS): sample count, median wheel speed
    and the share of samples with the wheels still."""
    cs, ws = [], []
    for run in runs:
        c, w = steady_samples(run)
        cs.append(c)
        ws.append(w)
    c = np.concatenate(cs) if cs else np.zeros(0)
    w = np.concatenate(ws) if ws else np.zeros(0)
    edges = np.arange(0.0, BIN_MAX_MPS + 1e-9, BIN_WIDTH_MPS)
    rows = []
    for lo, hi in pairwise(edges):
        sel = (c >= lo) & (c < hi)
        if sel.sum() == 0:
            continue
        rows.append(
            {
                "cmd_lo_mps": round(float(lo), 4),
                "cmd_hi_mps": round(float(hi), 4),
                "n": int(sel.sum()),
                "median_wheel_mps": float(np.median(w[sel])),
                "share_still": float(np.mean(np.abs(w[sel]) < ZERO_WHEEL_MPS)),
            }
        )
    return rows


def steady_brackets(bins: Sequence[dict[str, float]]) -> dict[str, object]:
    """Largest bin with the wheels still (at least ZERO_SHARE of its samples) and smallest bin
    whose median wheel speed is above MOVING_MPS, each with at least MIN_BIN_SAMPLES samples.
    `consistent` is False when the smallest moving bin lies below the largest still bin (the car
    rolls at a command that elsewhere left it standing: hysteresis)."""
    big = [b for b in bins if b["n"] >= MIN_BIN_SAMPLES]
    still = [b for b in big if b["share_still"] >= ZERO_SHARE and b["cmd_lo_mps"] > 0.0]
    moving = [b for b in big if b["median_wheel_mps"] > MOVING_MPS]
    still_hi = max((b["cmd_hi_mps"] for b in still), default=None)
    moving_lo = min((b["cmd_lo_mps"] for b in moving), default=None)
    return {
        "largest_still_bin_hi_mps": still_hi,
        "smallest_moving_bin_lo_mps": moving_lo,
        "consistent": bool(still_hi is None or moving_lo is None or moving_lo >= still_hi - 1e-9),
        "zero_wheel_mps": ZERO_WHEEL_MPS,
        "moving_wheel_mps": MOVING_MPS,
        "min_bin_samples": MIN_BIN_SAMPLES,
    }


ROLLING_START_MPS = 0.15
MIN_BAND_CMD_MPS = 0.02


def rolling_below_the_dead_zone(
    windows: SpeedWindows, deadband_mps: float, model_v_end: np.ndarray | None = None
) -> dict[str, object]:
    """Does the car keep rolling at commands below the dead zone? Over the 1 s windows that start
    rolling (wheel speed at least ROLLING_START_MPS) and are commanded inside the band
    (MIN_BAND_CMD_MPS, deadband) throughout: how many there are, the median start and end wheel
    speed and the share still turning at the end (above MOVING_MPS). `model_v_end`, the model end
    speed on the same windows, is compared when given."""
    if len(windows) == 0:
        return {"windows": 0}
    c = windows.cmds[..., 0]
    keep = (
        (windows.v_mps[:, 0] >= ROLLING_START_MPS)
        & (c.min(axis=1) > MIN_BAND_CMD_MPS)
        & (c.max(axis=1) < deadband_mps)
    )
    n = int(keep.sum())
    if n == 0:
        return {"windows": 0, "band_mps": [MIN_BAND_CMD_MPS, deadband_mps]}
    v0, v1 = windows.v_mps[keep, 0], windows.v_mps[keep, -1]
    out: dict[str, object] = {
        "windows": n,
        "band_mps": [MIN_BAND_CMD_MPS, deadband_mps],
        "median_start_speed_mps": float(np.median(v0)),
        "median_end_speed_mps": float(np.median(v1)),
        "share_still_turning_at_end": float(np.mean(v1 > MOVING_MPS)),
    }
    if model_v_end is not None:
        out["model_median_end_speed_mps"] = float(np.median(model_v_end[keep]))
    return out
