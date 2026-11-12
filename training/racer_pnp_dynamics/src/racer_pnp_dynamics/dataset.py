"""Recordings, segments of continuous driving, and the time split.

Split rule (fixed in the brief): within each bag, by time, the first 70 percent is train, the next
10 percent validation and the last 20 percent held out. The bag `HELD_OUT_BAG` is held out
entirely.

Finding that shapes this module: the bags under car-data/bags are NESTED COPIES of three
recordings, not independent ones. `..._03-54-11_car_teleop` is the first 528 s of
`..._03-54-11_car_teleop_cw` (messages identical), and `..._04-20-19_car_teleop` and `_latest` are
prefixes of `_v3`. Using them all would leak the held-out bag into training, so `drop_nested`
keeps only the longest bag of each group of overlapping time spans.
"""

from __future__ import annotations

from collections.abc import Sequence
from dataclasses import dataclass
from itertools import pairwise

import numpy as np

from .bags import Run
from .icp_motion import MotionSeries

HELD_OUT_BAG = "2026-11-11T04-20-19_car_teleop_v3"
TRAIN_FRACTION = 0.7
VAL_FRACTION = 0.1  # the rest is held out
SPLITS = ("train", "val", "heldout")

DRIVING_MPS = 0.05  # commanded or measured speed above this counts as driving
BRIDGE_S = 1.0  # a stop shorter than this does not split a segment
MIN_SEGMENT_S = 3.0


@dataclass(frozen=True)
class Segment:
    """Grid indices [i0, i1) of a run: continuous driving with valid commands, in one split."""

    run: Run
    motion: MotionSeries | None
    i0: int
    i1: int
    split: str

    @property
    def t0_s(self) -> float:
        return float(self.run.t_s[self.i0])

    @property
    def t1_s(self) -> float:
        return float(self.run.t_s[self.i1 - 1])

    @property
    def duration_s(self) -> float:
        return self.t1_s - self.t0_s


def drop_nested(runs: Sequence[Run], slack_s: float = 0.2) -> list[Run]:
    """Drop every run whose time span lies inside a longer run's span (nested copies)."""
    kept = []
    for r in runs:
        inside = any(
            o is not r
            and o.t_s[0] - slack_s <= r.t_s[0]
            and r.t_s[-1] <= o.t_s[-1] + slack_s
            and (o.duration_s > r.duration_s or (o.duration_s == r.duration_s and o.name < r.name))
            for o in runs
        )
        if not inside:
            kept.append(r)
    return kept


def split_times(t0_s: float, t1_s: float) -> tuple[float, float]:
    """Boundary times: train until the first, validation until the second, then held out."""
    span = t1_s - t0_s
    return t0_s + TRAIN_FRACTION * span, t0_s + (TRAIN_FRACTION + VAL_FRACTION) * span


def split_of_time(run: Run, t_s: float | np.ndarray) -> np.ndarray:
    """Split name index (0 train, 1 val, 2 held out) of grid time(s)."""
    if run.name == HELD_OUT_BAG:
        return np.full(np.shape(t_s), 2)
    b1, b2 = split_times(float(run.t_s[0]), float(run.t_s[-1]))
    t = np.asarray(t_s)
    return np.where(t < b1, 0, np.where(t < b2, 1, 2))


def driving_mask(run: Run) -> np.ndarray:
    """Grid samples where the command is valid (and the odometry too, for bags that have it) and
    the car is commanded or measured moving, with stops shorter than BRIDGE_S bridged."""
    valid = run.command_valid()
    if run.has_odom:
        valid &= np.isfinite(run.wheel_speed_mps)
    moving = np.nan_to_num(run.speed_cmd_mps, nan=0.0) > DRIVING_MPS
    moving |= np.nan_to_num(run.wheel_speed_mps, nan=0.0) > DRIVING_MPS
    moving &= valid
    bridge = round(BRIDGE_S * 50.0)
    out = moving.copy()
    idx = np.flatnonzero(moving)
    for a, b in pairwise(idx):
        if 1 < b - a <= bridge and valid[a:b].all():
            out[a:b] = True
    return out & valid


def _runs_of_true(mask: np.ndarray) -> list[tuple[int, int]]:
    edges = np.diff(np.concatenate([[0], mask.astype(np.int8), [0]]))
    return list(zip(np.flatnonzero(edges == 1), np.flatnonzero(edges == -1), strict=True))


def make_segments(run: Run, motion: MotionSeries | None = None) -> list[Segment]:
    """Segments of continuous driving, cut at the split boundaries, at least MIN_SEGMENT_S long."""
    which = split_of_time(run, run.t_s)
    min_len = int(MIN_SEGMENT_S * 50.0)
    out = []
    mask = driving_mask(run)
    for a, b in _runs_of_true(mask):
        cuts = [a, *np.flatnonzero(np.diff(which[a:b])) + a + 1, b]
        for c0, c1 in pairwise(cuts):
            if c1 - c0 >= min_len:
                out.append(Segment(run, motion, int(c0), int(c1), SPLITS[int(which[c0])]))
    return out


def by_split(segments: Sequence[Segment], split: str, odom_only: bool = False) -> list[Segment]:
    return [s for s in segments if s.split == split and (s.run.has_odom or not odom_only)]


def seconds(segments: Sequence[Segment]) -> float:
    return float(sum(s.duration_s for s in segments))
