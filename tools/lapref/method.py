"""The method of `lapref`: lap times of one whole run from the steering command and wheel distance.

Pre-registration v0.4 section 4.2, brief 36. Inputs are two time series and nothing else: the
steering angle of the gated `/drive` and the wheel speed of `/odom/wheel`, both inside one run.

1. Resample the steering onto wheel distance s (GRID_M grid), drop the first DROP_M (the launch).
2. Lap length L: the lag of the highest peak of the normalised autocorrelation of the steering
   over s in [MIN_LAP_M, MAX_LAG_SHARE of the run's distance], refined by a parabola. Under
   MIN_PEAK there is no periodic lap and the result has no laps (never a guess).
3. Template: the mean of the signal folded at L. The phase of each lap is the shift in
   [-SHIFT_SHARE L, +SHIFT_SHARE L] that best matches the template around the predicted crossing;
   the prediction follows the drift lap by lap.
4. Crossing k is at the tracked boundary k, s0 being the first sample after the dropped distance.
   Crossing times are interpolated on the odometry clock. Lap time k = crossing k - crossing k-1.

The start line of lapref is not the gate's start line: only lap times and counts are comparable.
The module is pure numpy and has no bag reading in it (see `bag.py`).
"""

from __future__ import annotations

import math
from dataclasses import asdict, dataclass, field
from itertools import pairwise
from typing import Any

import numpy as np
from numpy.typing import ArrayLike, NDArray

#: Brief 36 section 3 step 1: resampling grid on wheel distance (m).
GRID_M = 0.02
#: Brief 36 step 1: the launch is dropped (m of wheel distance).
DROP_M = 1.0
#: Brief 36 step 2: shortest lap (m) the autocorrelation looks for.
MIN_LAP_M = 2.0
#: Brief 36 step 2: longest lag as a share of the run's distance.
MAX_LAG_SHARE = 0.6
#: Brief 36 step 2: under this autocorrelation peak the run has "no periodic lap found".
MIN_PEAK = 0.5
#: Brief 36 step 3: the phase search covers this share of L on each side of the prediction.
SHIFT_SHARE = 0.15
#: Half width of the matching window around a crossing, as a share of L (set by the author of
#: the tool, not fixed by the brief; development constant of M-LR).
MATCH_LEFT_SHARE = 0.5
MATCH_RIGHT_SHARE = 0.5
#: A crossing is matched only if the run holds at least this share of the window around it.
MIN_WINDOW_COVER = 0.5
#: Gaps in the odometry above this (s) add no distance (a dropout is not travel).
MAX_ODOM_GAP_S = 1.0
#: The first template folds this many laps only: a lap length that drifts smears a fold over
#: many laps, and the passes below rebuild the template from the tracked laps anyway.
FOLD_LAPS = 2
#: Passes that rebuild the template from the laps tracked with the previous one (the first
#: template is the fold at L of the brief; a line that shortens lap by lap smears that fold).
REFINE_PASSES = 2
#: Never return a lap shorter than this (m) (brief 36 tests, L2).
MIN_RETURNED_LAP_M = 2.0


@dataclass
class LaprefResult:
    n_laps: int = 0
    lap_times_s: list[float] = field(default_factory=list)
    lap_lengths_m: list[float] = field(default_factory=list)
    #: Autocorrelation peak value (None when no candidate peak existed).
    autocorr_peak: float | None = None
    #: The lag of that peak: lap length L in metres (None as above).
    lap_length_m: float | None = None
    #: Correlation of the template with the signal at each lap's closing crossing.
    match_scores: list[float] = field(default_factory=list)
    run_distance_m: float = 0.0
    #: Empty when laps were found, else why not.
    reason: str = ""

    def to_dict(self) -> dict[str, Any]:
        out = asdict(self)
        out["mean_lap_time_s"] = float(np.mean(self.lap_times_s)) if self.lap_times_s else None
        return out


def wheel_distance(t: NDArray[np.float64], v: NDArray[np.float64]) -> NDArray[np.float64]:
    """Distance travelled at each odometry sample: the integral of max(0, speed) with the
    trapezoid rule; a gap above MAX_ODOM_GAP_S adds nothing."""
    if len(t) == 0:
        return np.zeros(0)
    speed = np.maximum(v, 0.0)
    dt = np.diff(t)
    step = np.where((dt > 0.0) & (dt <= MAX_ODOM_GAP_S), 0.5 * (speed[1:] + speed[:-1]) * dt, 0.0)
    return np.concatenate([[0.0], np.cumsum(step)])


def resample_on_distance(
    odom_t: NDArray[np.float64],
    odom_v: NDArray[np.float64],
    steer_t: NDArray[np.float64],
    steer: NDArray[np.float64],
) -> tuple[NDArray[np.float64], NDArray[np.float64], NDArray[np.float64], NDArray[np.float64]]:
    """(s_grid, steering on the grid, s per odometry sample, t per odometry sample), distance
    counted from the run's first odometry sample, the first DROP_M dropped. The grid starts at
    the first multiple of GRID_M at or after DROP_M. Where the car stood still the distance does
    not grow, and the steering at the first moment of reaching a distance is used."""
    s = wheel_distance(odom_t, odom_v)
    if len(s) < 2 or s[-1] <= DROP_M + GRID_M:
        return np.zeros(0), np.zeros(0), s, odom_t
    # time of first reaching each distance (the plateaus of a stall collapse to one point)
    s_unique, first = np.unique(s, return_index=True)
    t_of_s = odom_t[first]
    first_grid = math.ceil(DROP_M / GRID_M - 1e-9) * GRID_M
    grid = np.arange(first_grid, s_unique[-1] + 1e-9, GRID_M)
    t_grid = np.interp(grid, s_unique, t_of_s)
    steer_grid = np.interp(t_grid, steer_t, steer)
    return grid, steer_grid, s, odom_t


def autocorrelation(x: NDArray[np.float64], max_lag: int) -> NDArray[np.float64]:
    """Normalised (biased) autocorrelation of the mean-free signal for lags 0..max_lag: the sum of
    x[i] x[i+k] over the N - k overlapping samples, divided by the sum of x squared. It falls with
    the lag for a periodic signal, so the fundamental beats its multiples."""
    n = len(x)
    out = np.zeros(max_lag + 1)
    if n < 2:
        return out
    y = x - np.mean(x)
    energy = float(np.dot(y, y))
    if energy <= 1e-18 or not math.isfinite(energy):
        return out
    size = 1 << (2 * n - 1).bit_length()
    spec = np.fft.rfft(y, size)
    full = np.fft.irfft(spec * np.conj(spec), size)[: min(max_lag + 1, n)]
    out[: len(full)] = full / energy
    return out


def find_lap_length(x: NDArray[np.float64], distance_m: float) -> tuple[float | None, float | None]:
    """(L in metres, autocorrelation peak) of the highest interior local maximum of the
    autocorrelation over lags in [MIN_LAP_M, MAX_LAG_SHARE * distance_m], parabola refined.
    (None, None) if the range holds no interior maximum."""
    lo = math.ceil(MIN_LAP_M / GRID_M - 1e-9)
    hi = min(int(MAX_LAG_SHARE * distance_m / GRID_M), len(x) - 2)
    if hi - lo < 2:
        return None, None
    r = autocorrelation(x, hi + 1)
    mid = r[lo : hi + 1]
    interior = (mid[1:-1] > mid[:-2]) & (mid[1:-1] >= mid[2:])
    idx = np.nonzero(interior)[0] + 1
    if len(idx) == 0:
        return None, None
    k = int(idx[np.argmax(mid[idx])]) + lo
    delta, peak = _parabola(r[k - 1], r[k], r[k + 1])
    return (k + delta) * GRID_M, peak


def _parabola(a: float, b: float, c: float) -> tuple[float, float]:
    """Offset (in samples, within +-0.5) and height of the vertex through three equally spaced
    points a, b, c with b the middle one."""
    denom = a - 2.0 * b + c
    if denom >= -1e-12:
        return 0.0, float(b)
    delta = 0.5 * (a - c) / denom
    delta = float(np.clip(delta, -0.5, 0.5))
    return delta, float(b - 0.25 * (a - c) * delta)


def fold_template(
    grid0: float, x: NDArray[np.float64], period_m: float, n_laps: int
) -> NDArray[np.float64]:
    """Mean of the signal folded at `period_m`: the average of the first n_laps laps starting at
    grid0, each sampled at nbins evenly spaced phases of one period."""
    nbins = max(round(period_m / GRID_M), 4)
    phase = np.arange(nbins) * (period_m / nbins)
    pos = grid0 + np.arange(n_laps)[:, None] * period_m + phase[None, :]
    g = grid0 + np.arange(len(x)) * GRID_M
    return np.asarray(np.mean(np.interp(pos, g, x), axis=0), dtype=np.float64)


def refold_template(
    grid0: float, x: NDArray[np.float64], bounds: list[float], nbins: int
) -> NDArray[np.float64]:
    """Mean over the tracked laps of the signal, each lap resampled to nbins phases of its own
    length (so a line that shortens lap by lap does not smear the template)."""
    g = grid0 + np.arange(len(x)) * GRID_M
    laps = [np.interp(a + (b - a) * np.arange(nbins) / nbins, g, x) for a, b in pairwise(bounds)]
    return np.asarray(np.mean(laps, axis=0), dtype=np.float64)


def _template_at(
    template: NDArray[np.float64], period_m: float, phase_m: NDArray[np.float64]
) -> NDArray[np.float64]:
    """The (periodic) template at offsets `phase_m` (metres, any real values, wrapped) of a lap
    that is `period_m` long."""
    nbins = len(template)
    pos = (phase_m % period_m) / period_m * nbins
    i0 = np.floor(pos).astype(np.int64) % nbins
    frac = pos - np.floor(pos)
    return np.asarray(
        (1.0 - frac) * template[i0] + frac * template[(i0 + 1) % nbins], dtype=np.float64
    )


def pin_template(
    g0: float, x: NDArray[np.float64], template: NDArray[np.float64], lap_len_m: float
) -> NDArray[np.float64]:
    """Rotate the template so that its phase 0 is at g0, the start of the run's first lap. The
    laps rebuilt a template whose phase 0 is the consensus of all laps, and that may differ from
    the signal at g0 by a constant (a gauge): the first lap would then not be a whole lap. The
    rotation is the shift in +-SHIFT_SHARE L that best matches the signal after g0 to the
    template, found as in `match_crossing` with the part before g0 left out (it was dropped)."""
    nbins = len(template)
    offsets = np.arange(0, int(MATCH_RIGHT_SHARE * lap_len_m / GRID_M) + 1) * GRID_M
    max_shift = int(SHIFT_SHARE * lap_len_m / GRID_M)
    shifts = np.arange(-max_shift, max_shift + 1) * GRID_M
    g = g0 + np.arange(len(x)) * GRID_M
    a = np.interp(g0 + offsets, g, x)
    scores = np.array(
        [_pearson(a, _template_at(template, lap_len_m, offsets + shift)) for shift in shifts]
    )
    k = int(np.argmax(scores))
    delta = 0.0
    if 0 < k < len(shifts) - 1:
        delta, _ = _parabola(scores[k - 1], scores[k], scores[k + 1])
    shift = float(shifts[k]) + delta * GRID_M
    phases = np.arange(nbins) * (lap_len_m / nbins) + shift
    return _template_at(template, lap_len_m, phases)


def match_crossing(
    g0: float,
    x: NDArray[np.float64],
    template: NDArray[np.float64],
    period_m: float,
    predicted_m: float,
) -> tuple[float, float] | None:
    """The boundary (distance, in metres) near `predicted_m` where the signal best matches the
    template's start, and the correlation there. The shift is searched in +-SHIFT_SHARE L in
    GRID_M steps and refined by a parabola. None if the run covers too little of the window."""
    offsets = (
        np.arange(
            -int(MATCH_LEFT_SHARE * period_m / GRID_M),
            int(MATCH_RIGHT_SHARE * period_m / GRID_M) + 1,
        )
        * GRID_M
    )
    max_shift = int(SHIFT_SHARE * period_m / GRID_M)
    shifts = np.arange(-max_shift, max_shift + 1) * GRID_M
    g_end = g0 + (len(x) - 1) * GRID_M
    scores = np.full(len(shifts), -np.inf)
    ref = _template_at(template, period_m, offsets)
    for j, shift in enumerate(shifts):
        pos = predicted_m + shift + offsets
        inside = (pos >= g0) & (pos <= g_end)
        if inside.sum() < MIN_WINDOW_COVER * len(offsets) or inside.sum() < 4:
            continue
        a = np.interp(pos[inside], g0 + np.arange(len(x)) * GRID_M, x)
        b = ref[inside]
        scores[j] = _pearson(a, b)
    if not np.isfinite(scores).any():
        return None
    k = int(np.argmax(scores))
    delta = 0.0
    best = float(scores[k])
    if 0 < k < len(shifts) - 1 and np.isfinite(scores[k - 1]) and np.isfinite(scores[k + 1]):
        delta, best = _parabola(scores[k - 1], scores[k], scores[k + 1])
    return predicted_m + float(shifts[k]) + delta * GRID_M, float(best)


def _pearson(a: NDArray[np.float64], b: NDArray[np.float64]) -> float:
    a = a - np.mean(a)
    b = b - np.mean(b)
    denom = math.sqrt(float(np.dot(a, a)) * float(np.dot(b, b)))
    if denom <= 1e-18:
        return 0.0
    return float(np.dot(a, b)) / denom


def track(
    g0: float,
    x: NDArray[np.float64],
    template: NDArray[np.float64],
    first_len_m: float,
) -> tuple[list[float], list[float]]:
    """Boundaries (metres of wheel distance, the first is g0) and the match score of every
    closing crossing. Each boundary is searched around the previous one plus the previous lap's
    length, so the drift of the lap length is followed lap by lap. Stops when the run ends, when
    the window around the prediction is not covered, or when a lap would be under 2 m."""
    g_end = g0 + (len(x) - 1) * GRID_M
    bounds = [g0]
    scores: list[float] = []
    last_len = first_len_m
    while True:
        predicted = bounds[-1] + last_len
        if predicted - SHIFT_SHARE * last_len > g_end:
            break
        matched = match_crossing(g0, x, template, last_len, predicted)
        if matched is None:
            break
        boundary, score = matched
        boundary = max(boundary, bounds[-1] + MIN_RETURNED_LAP_M)
        if boundary > g_end:
            break
        bounds.append(boundary)
        scores.append(score)
        last_len = bounds[-1] - bounds[-2]
    return bounds, scores


def lapref(
    odom_t: ArrayLike, odom_v: ArrayLike, steer_t: ArrayLike, steer: ArrayLike
) -> LaprefResult:
    """The laps of one run. `odom_*`: wheel speed samples (seconds, m/s). `steer_*`: gated
    steering samples (seconds, rad). All inside the run. Never raises on odd input: it returns a
    result with no laps and a `reason`."""
    ot = np.asarray(odom_t, dtype=np.float64)
    ov = np.asarray(odom_v, dtype=np.float64)
    st = np.asarray(steer_t, dtype=np.float64)
    sv = np.asarray(steer, dtype=np.float64)
    ok_o = np.isfinite(ot) & np.isfinite(ov)
    ok_s = np.isfinite(st) & np.isfinite(sv)
    ot, ov, st, sv = ot[ok_o], ov[ok_o], st[ok_s], sv[ok_s]
    order_o, order_s = np.argsort(ot, kind="stable"), np.argsort(st, kind="stable")
    ot, ov, st, sv = ot[order_o], ov[order_o], st[order_s], sv[order_s]
    if len(ot) < 2 or len(st) < 2:
        return LaprefResult(reason="too few odometry or steering samples")
    grid, x, s_odom, t_odom = resample_on_distance(ot, ov, st, sv)
    total = float(s_odom[-1])
    result = LaprefResult(run_distance_m=total)
    if len(grid) < 8:
        result.reason = f"the run is {total:.2f} m, not longer than the {DROP_M:g} m dropped"
        return result
    length, peak = find_lap_length(x, float(grid[-1] - grid[0]))
    result.autocorr_peak, result.lap_length_m = peak, length
    if length is None or peak is None:
        result.reason = "no periodic lap found (no autocorrelation peak in range)"
        return result
    if peak < MIN_PEAK:
        result.reason = f"no periodic lap found (autocorrelation peak {peak:.3f} under {MIN_PEAK})"
        return result
    g0 = float(grid[0])
    g_end = float(grid[-1])
    n_fold = min(max(int((g_end - g0) / length + 1e-9), 1), FOLD_LAPS)
    template = fold_template(g0, x, length, n_fold)
    bounds, scores = track(g0, x, template, length)
    for _ in range(REFINE_PASSES):
        if len(bounds) < 2:
            break
        template = refold_template(g0, x, bounds, len(template))
        template = pin_template(g0, x, template, bounds[1] - bounds[0])
        # the lap length drifts smoothly: the first lap is about as long as the second
        first_len = bounds[2] - bounds[1] if len(bounds) >= 3 else length
        bounds, scores = track(g0, x, template, first_len)
    s_unique, first = np.unique(s_odom, return_index=True)
    t_of_s = t_odom[first]
    if len(bounds) < 2:
        result.reason = "the periodic signal was found but no lap closed inside the run"
        return result
    times = np.interp(np.asarray(bounds), s_unique, t_of_s)
    result.lap_times_s = [float(d) for d in np.diff(times)]
    result.lap_lengths_m = [float(d) for d in np.diff(bounds)]
    result.match_scores = scores
    result.n_laps = len(result.lap_times_s)
    return result
