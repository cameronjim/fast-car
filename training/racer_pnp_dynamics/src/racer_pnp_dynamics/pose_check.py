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
from .ground_motion import GroundMotion, step_v2
from .ground_motion_v21 import GroundMotionV21, step_v21
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
    ground: GroundMotion | GroundMotionV21 | None = None  # contract 2: step_v2, 2.1: step_v21


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


def _model_states(
    model: YawModel, run: Run, t0_s: np.ndarray, dt_s: np.ndarray
) -> tuple[np.ndarray, np.ndarray, np.ndarray, int]:
    """Rollout of the model from each t0: states (E, n + 1, 5) at t0 + i * DT_S (pose at the start is
    zero), the `ok` mask (every command and wheel speed the rollout needs is present) and the
    number of steps n."""
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
        state = step(state, cmds[:, j, :], nom, veh, res)  # the warm up is the wheel channel
    state[:, :3] = 0.0
    state[:, 3] = v[:, WARM_STEPS]
    states = [state.copy()]
    for j in range(n):
        if model.ground is None:
            state = step(state, cmds[:, WARM_STEPS + j, :], nom, veh, res)
        elif isinstance(model.ground, GroundMotionV21):
            state = step_v21(state, cmds[:, WARM_STEPS + j, :], nom, veh, model.ground)
        else:
            state = step_v2(state, cmds[:, WARM_STEPS + j, :], nom, veh, model.ground)
        states.append(state.copy())
    needed = np.arange(WARM_STEPS + n)[None, :] < (WARM_STEPS + steps_needed)[:, None]
    ok = np.isfinite(cmds).all(axis=2)
    ok = (ok | ~needed).all(axis=1) & np.isfinite(v).all(axis=1)
    return np.stack(states, axis=1), ok, steps_needed, n


def _at_dt(states: np.ndarray, dt: np.ndarray, n: int, column: int) -> np.ndarray:
    pos = dt / DT_S
    lo = np.clip(np.floor(pos + 1e-9).astype(int), 0, n)
    hi = np.clip(lo + 1, 0, n)
    frac = pos - lo
    rows = np.arange(len(dt))
    return states[rows, lo, column] * (1.0 - frac) + states[rows, hi, column] * frac


def model_yaw_change(model: YawModel, run: Run, t0_s: np.ndarray, dt_s: np.ndarray) -> np.ndarray:
    """Model yaw change over each interval [t0, t0 + dt] (NaN where a command or the wheel speed
    the rollout needs is missing). The commands are the model's own delayed ones."""
    dt = np.asarray(dt_s, dtype=float)
    states, ok, _, n = _model_states(model, run, t0_s, dt)
    return np.where(ok, _at_dt(states, dt, n, 2), np.nan)


def model_displacement(model: YawModel, run: Run, t0_s: np.ndarray, dt_s: np.ndarray) -> np.ndarray:
    """(E, 2) model position change over each interval in the frame at its start (NaN where the
    rollout lacks data)."""
    dt = np.asarray(dt_s, dtype=float)
    states, ok, _, n = _model_states(model, run, t0_s, dt)
    out = np.stack([_at_dt(states, dt, n, 0), _at_dt(states, dt, n, 1)], axis=1)
    return np.where(ok[:, None], out, np.nan)


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


# ---- Pp: the position check (addendum 1 section 2.5) ------------------------------------------

PP_HORIZON_S = 1.0


@dataclass(frozen=True)
class PosePairs:
    """Pose pairs about PP_HORIZON_S apart: start stamp, length, and the pose change in the frame at
    the start (x forward, y left), the map pose of the pair's two ends."""

    t0: np.ndarray
    dt: np.ndarray
    dx: np.ndarray
    dy: np.ndarray

    def __len__(self) -> int:
        return len(self.t0)


@dataclass(frozen=True)
class PosePairSegment:
    name: str
    segment: Segment
    pairs: PosePairs
    counts: dict[str, int]


def select_pose_pairs(
    recording: Recording, segment: Segment, horizon_s: float = PP_HORIZON_S
) -> tuple[PosePairs, dict[str, int]]:
    """Consecutive /pose messages of `segment` merged until at least `horizon_s` apart (the same
    merge `reference.pose_intervals` does with `min_interval_s`), then the P2.2 jump filter with the
    wheel path of the pair, as for Yp. A pair is NOT shortened to exactly `horizon_s`: the model is
    rolled for the pair's own length and the report says the mean length."""
    stamp = np.asarray(recording["pose_stamp"], dtype=float) + float(recording["t0_log_s"])
    x = np.asarray(recording["pose_x"], dtype=float)
    y = np.asarray(recording["pose_y"], dtype=float)
    yaw = np.asarray(recording["pose_yaw"], dtype=float)
    inside = (stamp >= segment.t0_s) & (stamp <= segment.t1_s)
    stamp, x, y, yaw = stamp[inside], x[inside], y[inside], yaw[inside]
    starts: list[int] = []
    ends: list[int] = []
    start = 0
    for k in range(1, len(stamp)):
        if stamp[k] - stamp[start] >= horizon_s:
            starts.append(start)
            ends.append(k)
            start = k
    s, e = np.array(starts, dtype=int), np.array(ends, dtype=int)
    if len(s) == 0:
        z = np.zeros(0)
        return PosePairs(z, z, z, z), {"pairs": 0, "too_long": 0, "jumped": 0, "kept": 0}
    wx, wy = x[e] - x[s], y[e] - y[s]
    c, sn = np.cos(yaw[s]), np.sin(yaw[s])
    iv = reference.Intervals(
        t0=stamp[s],
        t1=stamp[e],
        dyaw=reference.wrap(yaw[e] - yaw[s]),
        chord=np.hypot(wx, wy),
    )
    path = _wheel_path_m(segment.run, iv.t0, iv.t1)
    result = reference.filter_intervals(iv, path)
    keep = result.keep & np.isfinite(path)
    counts = {
        "pairs": result.n_total,
        "too_long": result.n_too_long,
        "jumped": result.n_jumped,
        "kept": int(keep.sum()),
    }
    pairs = PosePairs(
        iv.t0[keep], (iv.t1 - iv.t0)[keep], (c * wx + sn * wy)[keep], (-sn * wx + c * wy)[keep]
    )
    return pairs, counts


def build_pose_pair_segments(
    segments: Sequence[Segment],
    recordings: Mapping[str, Recording],
    names: Sequence[str] = POSE_CHECK_SEGMENTS,
) -> list[PosePairSegment]:
    out: list[PosePairSegment] = []
    for seg in segments:
        if seg.name in names and seg.run.name in recordings:
            pairs, counts = select_pose_pairs(recordings[seg.run.name], seg)
            out.append(PosePairSegment(seg.name, seg, pairs, counts))
    return out


def _pair_errors(model: YawModel, p: PosePairSegment) -> np.ndarray:
    disp = model_displacement(model, p.segment.run, p.pairs.t0, p.pairs.dt)
    err = np.hypot(disp[:, 0] - p.pairs.dx, disp[:, 1] - p.pairs.dy)
    return err[np.isfinite(err)]


def pp_by_segment(
    model: YawModel, split: Sequence[PosePairSegment], names: Sequence[str] = POSE_CHECK_SEGMENTS
) -> dict[str, dict[str, float]]:
    """{segment name: {"pp": RMS m (NaN with no usable pair), "pairs": n, "mean_dt_s": s}}."""
    out: dict[str, dict[str, float]] = {}
    for name in names:
        errs, dts = [], []
        for p in split:
            if p.name == name and len(p.pairs):
                errs.append(_pair_errors(model, p))
                dts.append(p.pairs.dt)
        err = np.concatenate(errs) if errs else np.zeros(0)
        out[name] = {
            "pp": float(np.sqrt(np.mean(err**2))) if len(err) else float("nan"),
            "pairs": float(len(err)),
            "mean_dt_s": float(np.concatenate(dts).mean()) if dts else float("nan"),
        }
    return out


def pp_macro(
    model: YawModel,
    split: Sequence[PosePairSegment],
    segments: Sequence[str] = POSE_CHECK_SEGMENTS,
) -> float:
    """Pp of the model: the mean over the named segments of their Pp; NaN when every segment is
    empty (the item cannot be applied)."""
    values = [
        v["pp"] for v in pp_by_segment(model, split, segments).values() if np.isfinite(v["pp"])
    ]
    return float(np.mean(values)) if values else float("nan")
