"""Per segment and pooled: interval-mean errors, closure noise, rejections, delay.

Arrays in, numbers out; nothing here touches a bag or a matcher, so the synthetic tests drive it
directly. `evaluate_segment` takes one segment's pose messages, wheel twist, steering model and
scan-matching steps and returns the per-interval errors. `summarise` pools segments by direction
and speed bin into the table of docs/notes/preregistration-pnp-v0.2.md section 3.4.
"""

from __future__ import annotations

from dataclasses import dataclass, field
from typing import Any

import numpy as np
from numpy.typing import NDArray

from . import delay, reference, signals

Array = NDArray[np.float64]

SPEED_BINS = ((0.25, 0.6), (0.6, 0.85), (0.85, float("inf")))
BIN_LABELS = ("0.25-0.6", "0.6-0.85", "0.85+")
# "icp" is the matcher as the repo has it (identity start); "icp_seeded" the same matcher started
# at the wheel twist's motion (what an online user with a wheel odometer would run).
SIGNAL_NAMES = ("icp", "icp_seeded", "wheel")
RESOLVED_POSE_DT_S = 0.25  # a pose stream with a median interval up to this resolves a 10 Hz yaw
STEERING_WINDOW_S = 0.2
# The pre-registered limits of "usable for adaptation" at 0.85 m/s and above.
LIMIT_RMS_RAD_PER_S = 0.10
LIMIT_CLOSURE_RAD_PER_S = 0.10
LIMIT_DELAY_S = 0.15


@dataclass(frozen=True)
class IcpInput:
    """Scan matching over one segment. `yaw[k]` is the yaw change from scan k to k + 1 over
    [t0[k], t1[k]] (the scans are consecutive), NaN where the match was rejected by the quality
    flag or was not computed; `computed[k]` says which. `yaw2[k]` is the direct match of scan k to
    k + 2 (length len(t0) - 1), NaN likewise."""

    t0: Array
    t1: Array
    yaw: Array
    computed: NDArray[np.bool_]
    yaw2: Array


@dataclass(frozen=True)
class SegmentInput:
    name: str
    direction: str
    pose_stamp: Array
    pose_x: Array
    pose_y: Array
    pose_yaw: Array
    wheel_t: Array
    wheel_vx: Array
    wheel_wz: Array
    steer_t: Array  # the steering model yaw rate v_wheel * tan(delta_cmd) / L
    steer_wz: Array
    icp: dict[str, IcpInput] = field(default_factory=dict)  # by signal name


@dataclass
class StepStats:
    """What the step series of one scan-matching signal needs after the intervals are done."""

    step_ok: NDArray[np.bool_]
    step_speed: Array
    step_t0: Array
    clo_resid: Array
    clo_span: Array
    clo_speed: Array


@dataclass
class SegmentResult:
    name: str
    direction: str
    pose_dt_median_s: float
    pose_resolved: bool
    n_total: int
    n_too_long: int
    n_jumped: int
    iv: reference.Intervals  # kept intervals
    speed: Array  # mean |wheel speed| per kept interval
    err: dict[str, Array]  # interval-mean rate minus reference, NaN where the signal had none
    rejected: dict[str, NDArray[np.bool_]]
    sig: dict[str, Any]  # signal objects with .mean(a, b)
    speed_series: signals.RateSeries
    steer: signals.RateSeries
    steps: dict[str, StepStats] = field(default_factory=dict)


def bin_index(speed: Array) -> NDArray[np.int64]:
    """Speed bin of each speed (m/s): 0, 1, 2, or -1 below the first bin."""
    out = np.full(np.shape(speed), -1, dtype=np.int64)
    for k, (lo, hi) in enumerate(SPEED_BINS):
        out[(speed >= lo) & (speed < hi)] = k
    return out


def evaluate_segment(inp: SegmentInput, min_interval_s: float = 0.0) -> SegmentResult:
    wheel = signals.RateSeries(inp.wheel_t, inp.wheel_wz)
    speed_series = signals.RateSeries(inp.wheel_t, np.abs(inp.wheel_vx))
    steer = signals.RateSeries(inp.steer_t, inp.steer_wz)
    all_iv = reference.pose_intervals(
        inp.pose_stamp, inp.pose_x, inp.pose_y, inp.pose_yaw, min_interval_s
    )
    path = np.nan_to_num(speed_series.integral(all_iv.t0, all_iv.t1), nan=0.0)
    flt = reference.filter_intervals(all_iv, path)
    iv = all_iv.select(flt.keep)
    speed = speed_series.mean(iv.t0, iv.t1)
    sig: dict[str, Any] = {"wheel": wheel}
    err = {"wheel": wheel.mean(iv.t0, iv.t1) - iv.rate}
    rejected = {"wheel": ~np.isfinite(err["wheel"])}
    res = SegmentResult(
        name=inp.name,
        direction=inp.direction,
        pose_dt_median_s=float(np.median(np.diff(inp.pose_stamp)))
        if len(inp.pose_stamp) > 1
        else float("nan"),
        pose_resolved=False,
        n_total=flt.n_total,
        n_too_long=flt.n_too_long,
        n_jumped=flt.n_jumped,
        iv=iv,
        speed=speed,
        err=err,
        rejected=rejected,
        sig=sig,
        speed_series=speed_series,
        steer=steer,
    )
    res.pose_resolved = bool(res.pose_dt_median_s <= RESOLVED_POSE_DT_S)
    for name, icp in inp.icp.items():
        series = signals.StepSeries(icp.t0, icp.t1, icp.yaw)
        sig[name] = series
        err[name] = series.mean(iv.t0, iv.t1) - iv.rate
        rejected[name] = ~np.isfinite(err[name])
        n2 = len(icp.yaw2)
        res.steps[name] = StepStats(
            step_ok=np.isfinite(icp.yaw),
            step_speed=speed_series.mean(icp.t0, icp.t1),
            step_t0=icp.t0,
            clo_resid=signals.closure_residual(icp.yaw[:n2], icp.yaw[1 : n2 + 1], icp.yaw2)
            if n2 > 0
            else np.zeros(0),
            clo_span=icp.t1[1 : n2 + 1] - icp.t0[:n2] if n2 > 0 else np.zeros(0),
            clo_speed=speed_series.mean(icp.t0[:n2], icp.t1[1 : n2 + 1]) if n2 > 0 else np.zeros(0),
        )
    return res


SCAN_MATCHING_SIGNALS = ("icp", "icp_seeded")


def _pool(values: list[Array], speeds: list[Array], k: int) -> Array:
    """The values of the pooled segments whose speed is in bin k."""
    parts = [v[bin_index(sp) == k] for v, sp in zip(values, speeds, strict=True)]
    return np.concatenate(parts) if parts else np.zeros(0)


def _stats(err: Array) -> dict[str, float]:
    e = err[np.isfinite(err)]
    if len(e) == 0:
        return {"n": 0, "bias": float("nan"), "rms": float("nan"), "std": float("nan")}
    return {
        "n": len(e),
        "bias": float(e.mean()),
        "rms": float(np.sqrt(np.mean(e**2))),
        "std": float(e.std()),
    }


def longest_rejection_run_s(
    step_ok: NDArray[np.bool_], in_bin: NDArray[np.bool_], dt_s: float
) -> float:
    """The longest run of consecutive steps that are in the bin and rejected, in seconds (what an
    online user would coast through)."""
    best = run = 0
    for ok, inside in zip(step_ok, in_bin, strict=True):
        if inside and not ok:
            run += 1
            best = max(best, run)
        else:
            run = 0
    return best * dt_s


class _Pooled:
    """Signals of several segments evaluated on the slices of one pooled interval array."""

    def __init__(self, parts: list[tuple[Any, slice]]) -> None:
        self.parts = parts

    def mean(self, a: Array, b: Array) -> Array:
        out = np.full(len(a), np.nan)
        for s, sl in self.parts:
            out[sl] = s.mean(a[sl], b[sl])
        return out


def _lag(results: list[SegmentResult], name: str, bin_k: int, kind: str) -> delay.LagResult:
    """Delay of signal `name` against the pose (`kind` "pose", segments whose pose stream
    resolves 10 Hz) or against the steering model (`kind` "steering", the others)."""
    a_parts, b_parts, r_parts, parts = [], [], [], []
    start = 0
    for r in results:
        if name not in r.sig:
            continue
        if kind == "pose":
            if not r.pose_resolved:
                continue
            a, b, ref = r.iv.t0, r.iv.t1, r.iv.rate
            speed = r.speed
        else:
            if r.pose_resolved or name not in r.steps or len(r.steps[name].step_t0) < 2:
                continue
            a = r.steps[name].step_t0[::2]
            b = a + STEERING_WINDOW_S
            ref = r.steer.mean(a, b)
            speed = r.speed_series.mean(a, b)
        keep = bin_index(speed) == bin_k
        a, b, ref = a[keep], b[keep], ref[keep]
        if len(a) == 0:
            continue
        a_parts.append(a)
        b_parts.append(b)
        r_parts.append(ref)
        parts.append((r.sig[name], slice(start, start + len(a))))
        start += len(a)
    if not parts:
        nan = float("nan")
        return delay.LagResult(nan, nan, nan, 0, np.zeros(0), np.zeros(0))
    return delay.interval_lag(
        np.concatenate(a_parts),
        np.concatenate(b_parts),
        np.concatenate(r_parts),
        _Pooled(parts),
    )


def _lag_dict(lag: delay.LagResult) -> dict[str, float]:
    return {
        "lag_s": lag.lag_s,
        "peak_corr": lag.peak_corr,
        "drop_at_0p1": lag.drop_at_0p1,
        "n": lag.n,
    }


def summarise(
    results: list[SegmentResult], directions: tuple[str, ...] = ("ccw", "cw", "all")
) -> dict[str, Any]:
    """The table: per signal, direction and speed bin."""
    out: dict[str, Any] = {"signals": {}, "drops": {}}
    for r in results:
        out["drops"][r.name] = {
            "pose_intervals": r.n_total,
            "too_long": r.n_too_long,
            "jumped": r.n_jumped,
            "kept": len(r.iv),
            "pose_dt_median_s": r.pose_dt_median_s,
            "pose_resolves_10hz": r.pose_resolved,
        }
    names = [n for n in SIGNAL_NAMES if any(n in r.sig for r in results)]
    for name in names:
        per_dir: dict[str, Any] = {}
        for d in directions:
            sel = [r for r in results if (d == "all" or r.direction == d) and name in r.sig]
            per_bin: dict[str, Any] = {}
            for k, label in enumerate(BIN_LABELS):
                err = _pool([r.err[name] for r in sel], [r.speed for r in sel], k)
                rej = _pool([r.rejected[name] for r in sel], [r.speed for r in sel], k)
                row: dict[str, Any] = _stats(err)
                row["intervals"] = len(rej)
                row["rejected_share"] = float(rej.mean()) if len(rej) else float("nan")
                if name in SCAN_MATCHING_SIGNALS:
                    runs = [
                        longest_rejection_run_s(
                            r.steps[name].step_ok, bin_index(r.steps[name].step_speed) == k, 0.1
                        )
                        for r in sel
                    ]
                    row["longest_rejection_run_s"] = max(runs) if runs else float("nan")
                    resid = _pool(
                        [r.steps[name].clo_resid for r in sel],
                        [r.steps[name].clo_speed for r in sel],
                        k,
                    )
                    span = _pool(
                        [r.steps[name].clo_span for r in sel],
                        [r.steps[name].clo_speed for r in sel],
                        k,
                    )
                    row["closure_n"] = int(np.isfinite(resid).sum())
                    row["closure_rms_rad_per_s"] = signals.closure_rate_rms(resid, span)
                    row["per_step_noise_rad_per_s"] = signals.per_step_noise_rate(
                        row["closure_rms_rad_per_s"]
                    )
                row["delay_vs_pose"] = _lag_dict(_lag(sel, name, k, "pose"))
                row["delay_vs_steering_model"] = _lag_dict(_lag(sel, name, k, "steering"))
                per_bin[label] = row
            per_dir[d] = per_bin
        out["signals"][name] = per_dir
    out["verdicts"] = {
        name: verdict(out["signals"][name]["all"][BIN_LABELS[2]], name) for name in names
    }
    return out


def verdict(row: dict[str, Any], name: str) -> dict[str, Any]:
    """The pre-registered label from the 0.85+ row, pooled over directions."""
    checks: dict[str, bool | None] = {}
    rms = row["rms"]
    checks["rms_error"] = None if not np.isfinite(rms) else bool(rms <= LIMIT_RMS_RAD_PER_S)
    if name in SCAN_MATCHING_SIGNALS:
        clo = row["closure_rms_rad_per_s"]
        checks["closure_noise"] = (
            None if not np.isfinite(clo) else bool(clo <= LIMIT_CLOSURE_RAD_PER_S)
        )
    lag = row["delay_vs_pose"]["lag_s"]
    checks["delay"] = None if not np.isfinite(lag) else bool(abs(lag) <= LIMIT_DELAY_S)
    known = [v for v in checks.values() if v is not None]
    if any(v is None for v in checks.values()):
        label = "not measurable (a criterion has no data)"
    elif all(known):
        label = "usable for adaptation"
    else:
        label = "not usable for adaptation"
    return {"label": label, "checks": checks}
