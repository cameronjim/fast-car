"""Yp, the independent yaw check of GP2c (pre-registration v0.3 section 2.2).

T2, the truth the models are fitted and scored on, is seeded by a wheel model. So the held-out
segments that carry a map pose (`G1b`, `T2b`) get a second reference that no wheel model touched:
the yaw change between consecutive `/pose` messages (slam_toolbox in 22-00-40, AMCL in 23-27-23).
For every pair of consecutive messages at most 1.5 s apart that passes the P2.2 jump filter
(`yaw_rate.reference`, reused as it is) the model is rolled open loop over the interval from the
recorded `/drive` commands. Yp is the RMS of (model yaw change minus pose yaw change), per
segment, and `yp_macro` is the mean over the segments.

The rollout starts at the pose stamp with the wheel speed of the bag, and the steering state of
the model run on the preceding WARM_S seconds of commands (the measured wheel speed substituted at
every step, as `windows.warm_state` does for the GP2 windows). After the start the model's own
speed channel runs, exactly as in a GP2 window. A segment with no usable interval is reported and
left out of the mean; with no usable interval anywhere `yp_macro` is NaN ("cannot be applied") and
nothing replaces it.
"""

from __future__ import annotations

from collections.abc import Mapping, Sequence
from dataclasses import dataclass

import numpy as np

from .bags import Run  # puts tools/ on sys.path, which the next import needs
from .dataset import Segment
from .model import DT_S, Nominal, Residual, steering_target, step
from .vehicle import Vehicle
from .windows import cmd_at, wheel_at

from yaw_rate import reference  # isort: skip

WARM_S = 1.0
WARM_STEPS = round(WARM_S / DT_S)
PATH_SUBSTEPS = 20  # midpoint sum of the wheel speed over an interval, for the jump filter
POSE_CHECK_SEGMENTS = ("G1b", "T2b")  # evaluation/prereg/pnp-v0.3.yaml dynamics.pose_check_segments

Recording = Mapping[str, np.ndarray]


@dataclass(frozen=True)
class YawModel:
    """What the rollout needs of a model: parameters, residual (or None) and the vehicle."""

    nominal: Nominal
    residual: Residual | None
    vehicle: Vehicle


@dataclass(frozen=True)
class PoseSegment:
    """One piece of a held-out segment with its pose intervals and what was dropped."""

    name: str
    segment: Segment
    intervals: reference.Intervals
    counts: dict[str, int]


def _wheel_path_m(run: Run, t0: np.ndarray, t1: np.ndarray) -> np.ndarray:
    h = (t1 - t0) / PATH_SUBSTEPS
    mid = t0[:, None] + (np.arange(PATH_SUBSTEPS)[None, :] + 0.5) * h[:, None]
    with np.errstate(invalid="ignore"):
        return (wheel_at(run, mid) * h[:, None]).sum(axis=1)


def select_pose_intervals(
    recording: Recording, segment: Segment
) -> tuple[reference.Intervals, dict[str, int]]:
    """The usable /pose intervals of `segment` and the counts of what was dropped: pairs inside
    the segment, longer than 1.5 s, flagged as a pose jump by the P2.2 filter, with no wheel
    speed (so no jump test)."""
    stamp = np.asarray(recording["pose_stamp"], dtype=float) + float(recording["t0_log_s"])
    x = np.asarray(recording["pose_x"], dtype=float)
    y = np.asarray(recording["pose_y"], dtype=float)
    yaw = np.asarray(recording["pose_yaw"], dtype=float)
    inside = (stamp >= segment.t0_s) & (stamp <= segment.t1_s)
    iv = reference.pose_intervals(stamp[inside], x[inside], y[inside], yaw[inside])
    path = _wheel_path_m(segment.run, iv.t0, iv.t1)
    result = reference.filter_intervals(iv, path)
    usable = result.keep & np.isfinite(path)
    counts = {
        "pairs": result.n_total,
        "too_long": result.n_too_long,
        "jumped": result.n_jumped,
        "no_wheel_speed": int((result.keep & ~np.isfinite(path)).sum()),
        "kept": int(usable.sum()),
    }
    return iv.select(usable), counts


def pose_yaw_intervals(recording: Recording, segment: Segment) -> reference.Intervals:
    """Consecutive /pose pairs inside `segment`, at most 1.5 s apart, past the P2.2 jump filter."""
    return select_pose_intervals(recording, segment)[0]


def model_yaw_change(model: YawModel, run: Run, t0_s: np.ndarray, dt_s: np.ndarray) -> np.ndarray:
    """Model yaw change over each interval [t0, t0 + dt] (NaN where a command or the wheel speed
    the rollout needs is missing). The commands are the model's own delayed ones."""
    t0 = np.asarray(t0_s, dtype=float)
    dt = np.asarray(dt_s, dtype=float)
    nom, res, veh = model.nominal, model.residual, model.vehicle
    steps_needed = np.ceil(dt / DT_S - 1e-9).astype(int)
    n = int(steps_needed.max()) if len(dt) else 0
    k_all = np.arange(-WARM_STEPS, n)
    times = t0[:, None] + k_all[None, :] * DT_S
    cmds = cmd_at(run, times - nom.command_delay_s)
    v = wheel_at(run, t0[:, None] + np.arange(-WARM_STEPS, 1)[None, :] * DT_S)
    state = np.zeros((len(t0), 5))
    state[:, 4] = steering_target(cmds[:, 0, 1], nom, veh)
    for j in range(WARM_STEPS):
        state[:, 3] = v[:, j]
        state = step(state, cmds[:, j, :], nom, veh, res)
    state[:, 2] = 0.0
    state[:, 3] = v[:, WARM_STEPS]
    yaws = [state[:, 2].copy()]
    for j in range(n):
        state = step(state, cmds[:, WARM_STEPS + j, :], nom, veh, res)
        yaws.append(state[:, 2].copy())
    yaw = np.stack(yaws, axis=1)  # (E, n + 1): yaw at step i is the model yaw at t0 + i * DT_S
    pos = dt / DT_S
    lo = np.clip(np.floor(pos + 1e-9).astype(int), 0, n)
    hi = np.clip(lo + 1, 0, n)
    frac = pos - lo
    rows = np.arange(len(t0))
    out = yaw[rows, lo] * (1.0 - frac) + yaw[rows, hi] * frac
    needed = np.arange(WARM_STEPS + n)[None, :] < (WARM_STEPS + steps_needed)[:, None]
    ok = np.isfinite(cmds).all(axis=2)
    ok = (ok | ~needed).all(axis=1) & np.isfinite(v).all(axis=1)
    return np.where(ok, out, np.nan)


def yp(
    model: YawModel, recording: Recording, segment: Segment, intervals: reference.Intervals
) -> float:
    """RMS over `intervals` of (model yaw change minus pose yaw change), rad. NaN when no
    interval is usable. `recording` is not read again: the intervals came from it."""
    del recording
    if len(intervals) == 0:
        return float("nan")
    err = _interval_errors(model, segment, intervals)
    return float(np.sqrt(np.mean(err**2))) if len(err) else float("nan")


def _interval_errors(
    model: YawModel, segment: Segment, intervals: reference.Intervals
) -> np.ndarray:
    pred = model_yaw_change(model, segment.run, intervals.t0, intervals.dt)
    err = pred - intervals.dyaw
    return err[np.isfinite(err)]


def build_pose_segments(
    segments: Sequence[Segment],
    recordings: Mapping[str, Recording],
    names: Sequence[str] = POSE_CHECK_SEGMENTS,
) -> list[PoseSegment]:
    """The pose intervals of every piece of the named split segments whose bag has a recording
    (keyed by run name)."""
    out: list[PoseSegment] = []
    for seg in segments:
        if seg.name in names and seg.run.name in recordings:
            iv, counts = select_pose_intervals(recordings[seg.run.name], seg)
            out.append(PoseSegment(seg.name, seg, iv, counts))
    return out


def yp_by_segment(
    model: YawModel, split: Sequence[PoseSegment], names: Sequence[str] = POSE_CHECK_SEGMENTS
) -> dict[str, dict[str, float]]:
    """{segment name: {"yp": RMS rad (NaN with no usable interval), "intervals": n}}; the pieces of
    one segment are pooled."""
    out: dict[str, dict[str, float]] = {}
    for name in names:
        errs = [
            _interval_errors(model, p.segment, p.intervals)
            for p in split
            if p.name == name and len(p.intervals)
        ]
        err = np.concatenate(errs) if errs else np.zeros(0)
        out[name] = {
            "yp": float(np.sqrt(np.mean(err**2))) if len(err) else float("nan"),
            "intervals": float(len(err)),
        }
    return out


def yp_macro(
    model: YawModel, split: Sequence[PoseSegment], segments: Sequence[str] = POSE_CHECK_SEGMENTS
) -> float:
    """Yp of the model: the mean over the named segments of their Yp. A segment with no usable
    interval is left out; NaN when every segment is empty (the item cannot be applied)."""
    per = yp_by_segment(model, split, segments)
    values = [v["yp"] for v in per.values() if np.isfinite(v["yp"])]
    return float(np.mean(values)) if values else float("nan")
