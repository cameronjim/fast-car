"""Windows, validation split and loss shared by the ground motion fits (brief 37b section 3,
addendum 1 section 2.3).

* Windows: `evaluate.eval_set`, the builder of the gate (wheel speed as the initial speed, the
  recorded gated `/drive` commands read through the model's own delay, open loop, horizons 0.5, 1.0
  and 1.5 s, starts every 0.25 s with a start speed of at least 0.25 m/s). Nothing else builds
  windows for T4, TL4 or the gate.
* Validation: the last 20 percent in time of every train segment, after a 2.0 s guard (the guard is
  taken off the end of the training part). A piece whose validation part would be shorter than
  `MIN_VAL_S` keeps all of its data as training and gives no validation window.
* Loss: Huber on the relative motion split into along, cross and heading (the pose at the horizon
  in the frame at the window start: x, y, yaw), each term divided by its RMS under dyn-v0.1 on the
  train windows (pooled over the horizons), each window weighted by the quality flags of the truth.

Choices the pre-registration leaves open, fixed here and reported: the Huber threshold is 1.345
(in units of the scaled error, the usual 95 percent efficiency constant) and the quality weight of
a window is the minimum over its ICP intervals of `inlier_share * (1 - rms / RMS_MAX_M)`, which is
in (0, 1] for every interval that passed the quality gate.
"""

from __future__ import annotations

from collections.abc import Sequence
from dataclasses import dataclass, replace

import numpy as np

from .dataset import MIN_SEGMENT_S, Segment
from .evaluate import EvalSet, eval_set
from .icp_motion import RMS_MAX_M, SCAN_DT_S
from .model import DT_S, Nominal
from .vehicle import Vehicle

HORIZONS_S = (0.5, 1.0, 1.5)
VAL_SHARE = 0.20
GUARD_S = 2.0
MIN_VAL_S = 2.0
HUBER_DELTA = 1.345
MIN_QUALITY = 1e-3  # a window never gets a weight of exactly zero
TERMS = ("along", "cross", "heading")


def fit_validation_split(segments: Sequence[Segment]) -> tuple[list[Segment], list[Segment]]:
    """(train, validation) pieces of the train-role data. A train segment is a segment of the split
    file (its driving pieces share a run and a name); its last `VAL_SHARE` in time is validation
    and `GUARD_S` before that is dropped from training. A segment whose validation part would hold
    less than `MIN_VAL_S` of driving keeps all its data as training."""
    rate = 50.0
    groups: dict[tuple[int, str], list[Segment]] = {}
    for s in segments:
        if s.split in ("train", "val"):
            groups.setdefault((id(s.run), s.name), []).append(s)
    train: list[Segment] = []
    val: list[Segment] = []
    min_train = int(MIN_SEGMENT_S * rate)
    for pieces in groups.values():
        lo, hi = min(p.i0 for p in pieces), max(p.i1 for p in pieces)
        cut = lo + round((1.0 - VAL_SHARE) * (hi - lo))
        guard_cut = cut - round(GUARD_S * rate)
        val_parts = [replace(p, i0=max(p.i0, cut), split="val") for p in pieces if p.i1 > cut]
        enough = sum(p.i1 - p.i0 for p in val_parts) / rate >= MIN_VAL_S
        for p in sorted(pieces, key=lambda x: x.i0):
            if not enough:
                train.append(replace(p, split="train"))
            elif p.i0 < guard_cut:
                t = replace(p, i1=min(p.i1, guard_cut), split="train")
                if t.i1 - t.i0 >= min_train:
                    train.append(t)
        if enough:
            val.extend(v for v in val_parts if (v.i1 - v.i0) / rate >= MIN_VAL_S)
    return train, val


def window_quality(es: EvalSet, segments: Sequence[Segment], horizon_s: float = max(HORIZONS_S)):
    """(E,) quality weight of each window of `es`, from the ICP quality numbers of the intervals
    inside it (the segments are the ones `es` was built from)."""
    n_iv = round(horizon_s / SCAN_DT_S)
    q = np.ones(len(es))
    if len(es) == 0:
        return q
    for ri, run in enumerate(es.runs):
        for seg in segments:
            m = seg.motion
            if seg.run is not run or m is None:
                continue
            sel = np.flatnonzero((es.run_idx == ri) & (es.t_s >= seg.t0_s) & (es.t_s <= seg.t1_s))
            if len(sel) == 0:
                continue
            start = np.searchsorted(m.t_s, es.t_s[sel] - 1e-6)
            for w, i in zip(sel, start, strict=True):
                share = m.inlier_share[i : i + n_iv]
                rms = m.rms_m[i : i + n_iv]
                q[w] = float(np.min(share * (1.0 - rms / RMS_MAX_M)))
    return np.clip(np.nan_to_num(q, nan=MIN_QUALITY), MIN_QUALITY, 1.0)


def huber(r: np.ndarray, delta: float = HUBER_DELTA) -> np.ndarray:
    """Huber loss of a scaled residual: r^2 / 2 inside delta, delta (|r| - delta / 2) outside."""
    a = np.abs(r)
    return np.where(a <= delta, 0.5 * r * r, delta * (a - 0.5 * delta))


@dataclass(frozen=True)
class FitWindows:
    """Windows of one role with their quality weights and the loss scales."""

    es: EvalSet
    quality: np.ndarray  # (E,)
    scales: dict[str, float]  # RMS of along, cross, heading under dyn-v0.1 on the TRAIN windows

    def __len__(self) -> int:
        return len(self.es)


def relative_motion_errors(
    states: np.ndarray, es: EvalSet, horizons_s: Sequence[float] = HORIZONS_S
) -> dict[str, np.ndarray]:
    """(E, len(horizons)) errors (model minus truth) of the relative motion at each horizon."""
    ks = [round(h / DT_S) for h in horizons_s]
    js = [round(h / SCAN_DT_S) for h in horizons_s]
    return {
        "along": states[:, ks, 0] - es.poses[:, js, 0],
        "cross": states[:, ks, 1] - es.poses[:, js, 1],
        "heading": states[:, ks, 2] - es.poses[:, js, 2],
    }


def loss_scales(
    states_v01: np.ndarray, es: EvalSet, horizons_s: Sequence[float] = HORIZONS_S
) -> dict[str, float]:
    """RMS of each term under dyn-v0.1 (states of its rollout on the TRAIN windows)."""
    err = relative_motion_errors(states_v01, es, horizons_s)
    return {t: max(float(np.sqrt(np.mean(err[t] ** 2))), 1e-9) for t in TERMS}


def window_loss(
    states: np.ndarray,
    fw: FitWindows,
    horizons_s: Sequence[float] = HORIZONS_S,
    terms: Sequence[str] = TERMS,
) -> float:
    """Quality weighted mean over windows and horizons of the Huber loss of the scaled terms."""
    err = relative_motion_errors(states, fw.es, horizons_s)
    total = np.zeros(len(fw.es))
    for t in terms:
        total += huber(err[t] / fw.scales[t]).mean(axis=1)
    return float(np.sum(fw.quality * total) / np.sum(fw.quality))


def build_fit_windows(
    segments: Sequence[Segment],
    nominal: Nominal,
    vehicle: Vehicle,
    scales: dict[str, float] | None = None,
) -> FitWindows:
    """Windows of `segments` with quality weights; `scales` from the train windows (computed from
    `nominal`'s own rollout when None, which is what the train set does with dyn-v0.1)."""
    from .evaluate import predict

    odom = [s for s in segments if s.run.has_odom and s.motion is not None]
    es = eval_set(odom)
    if scales is None:
        scales = loss_scales(predict(es.data(nominal.command_delay_s), nominal, vehicle), es)
    return FitWindows(es, window_quality(es, odom), scales)
