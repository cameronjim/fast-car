"""Recordings, segments of continuous driving, and the time split.

Split rule (brief 14): within each bag, by time, the first 70 percent of the bag's MOVING data
(samples of `driving_mask`) is train and the last 30 percent is held out; there is no validation
split (the delay and the dead zone are chosen by blocked cross-validation over the training
segments, and the held-out part scores them). No bag is held out entirely any more.

Finding that shapes this module: the bags under car-data/bags are NESTED COPIES of three
recordings, not independent ones. `..._03-54-11_car_teleop` is the first 528 s of
`..._03-54-11_car_teleop_cw` (messages identical), and `..._04-20-19_car_teleop` and `_latest` are
prefixes of `_v3`. Using them all would leak the held-out part into training, so `drop_nested`
keeps only the longest bag of each group of overlapping time spans.
"""

from __future__ import annotations

from collections.abc import Sequence
from dataclasses import dataclass, replace
from itertools import pairwise

import numpy as np

from .bags import Run
from .icp_motion import MotionSeries
from .split import BagEntry, Split

TRAIN_FRACTION = 0.7  # of a bag's moving data; the rest is held out
VAL_FRACTION = 0.0  # no validation split: see the module docstring
SPLITS = ("train", "val", "heldout", "legacy_heldout")
VAL_SEGMENT_FRACTION = 0.15  # last share in time of each train segment: internal validation
MIN_VAL_SEGMENT_S = 2.0
T_REF_TOLERANCE_S = 0.01  # the split's first_scan_log_time_s against the bag's first /scan

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
    name: str = ""  # the split file's segment name (G1a, R3, ...), "" without a split
    direction: str = ""  # cw / ccw from the split file, "legacy" for legacy_70_30 bags
    cap_mps: float | None = None

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


def split_times(run: Run) -> tuple[float, float]:
    """Boundary times: train until the first, validation until the second (the same time, there
    is no validation split), then held out. The first boundary is the time by which TRAIN_FRACTION
    of the run's moving samples (`driving_mask`) have passed."""
    moving = np.flatnonzero(driving_mask(run))
    if len(moving) == 0:
        return float(run.t_s[0]), float(run.t_s[0])
    k_train = min(int(np.ceil(TRAIN_FRACTION * len(moving))), len(moving) - 1)
    k_val = min(int(np.ceil((TRAIN_FRACTION + VAL_FRACTION) * len(moving))), len(moving) - 1)
    # the boundary sits at the start of the first sample that is not in the part
    return float(run.t_s[moving[k_train]]), float(run.t_s[moving[k_val]])


def split_of_time(run: Run, t_s: float | np.ndarray) -> np.ndarray:
    """Split name index (0 train, 1 val, 2 held out) of grid time(s)."""
    b1, b2 = split_times(run)
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


def _legacy_segments(run: Run, motion: MotionSeries | None) -> list[Segment]:
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


def make_segments(
    run: Run,
    motion: MotionSeries | None = None,
    split: Split | None = None,
    t_ref_s: float | None = None,
) -> list[Segment]:
    """Segments of continuous driving of a run, each in one role.

    Without `split` (the dyn-v0.1 rule): the first 70 percent of the run's moving data is train,
    the rest `heldout`, cut at the boundary, at least MIN_SEGMENT_S long.

    With the pre-registered `split` (bag seconds after the first /scan, `t_ref_s` or the run's
    first scan log time): a segmented bag gives `train` and `heldout` segments exactly as the
    file lists them (the driving mask only cuts them further, never extends them); a
    `legacy_70_30` bag keeps the rule above with the held-out part named `legacy_heldout`; an
    `unused` bag, a bag the file does not name and everything outside a segment give nothing.
    The last VAL_SEGMENT_FRACTION in time of each train segment is role `val`: the internal
    validation part of every model choice, never held-out."""
    if split is None:
        return _legacy_segments(run, motion)
    entry = split.entry(run.name)
    if entry is None:
        return []
    if entry.whole_bag_role is not None:
        return _whole_bag_segments(run, motion, entry)
    t_ref = _t_ref(run, entry, t_ref_s)
    out: list[Segment] = []
    mask = driving_mask(run)
    for seg in entry.segments:
        if seg.role not in ("train", "heldout"):
            continue
        a = int(np.searchsorted(run.t_s, t_ref + seg.t_from_s, side="left"))
        b = int(np.searchsorted(run.t_s, t_ref + seg.t_to_s, side="right"))
        pieces = [(a + p, a + q) for p, q in _runs_of_true(mask[a:b])]
        t_cut = t_ref + seg.t_from_s + (1.0 - VAL_SEGMENT_FRACTION) * (seg.t_to_s - seg.t_from_s)
        cut = int(np.searchsorted(run.t_s, t_cut, side="left"))
        for p, q in pieces:
            for c0, c1, role in _carve(p, q, cut if seg.role == "train" else None, seg.role):
                out.append(Segment(run, motion, c0, c1, role, seg.name, seg.direction, seg.cap_mps))
    return out


def _t_ref(run: Run, entry: BagEntry, t_ref_s: float | None) -> float:
    if t_ref_s is not None:
        return float(t_ref_s)
    if not run.scans:
        raise ValueError(f"{run.name}: no /scan messages, the split's time axis has no origin")
    t_ref = float(run.scans[0].log_time_s)
    if (
        entry.first_scan_log_time_s is not None
        and abs(t_ref - entry.first_scan_log_time_s) > T_REF_TOLERANCE_S
    ):
        raise ValueError(
            f"{run.name}: first /scan log time {t_ref:.3f} is not the split file's "
            f"{entry.first_scan_log_time_s:.3f}"
        )
    return t_ref


def _carve(p: int, q: int, cut: int | None, role: str) -> list[tuple[int, int, str]]:
    """Cut the driving piece [p, q) of a segment of `role` at grid index `cut` (train segments:
    train before it, val from it on), dropping pieces shorter than the minimum for their role."""
    min_train, min_val = int(MIN_SEGMENT_S * 50.0), int(MIN_VAL_SEGMENT_S * 50.0)
    if cut is None:
        return [(p, q, role)] if q - p >= min_train else []
    out = []
    if min(q, cut) - p >= min_train:
        out.append((p, min(q, cut), "train"))
    if q - max(p, cut) >= min_val:
        out.append((max(p, cut), q, "val"))
    return out


def _whole_bag_segments(run: Run, motion: MotionSeries | None, entry: BagEntry) -> list[Segment]:
    role = entry.whole_bag_role
    if role == "legacy_70_30":
        out = []
        for seg in _legacy_segments(run, motion):
            name = "legacy"
            if seg.split == "heldout":
                out.append(replace(seg, split="legacy_heldout", direction=name))
                continue
            # train: the last share of the piece in time is validation
            cut = seg.i0 + int((1.0 - VAL_SEGMENT_FRACTION) * (seg.i1 - seg.i0))
            out.extend(
                replace(seg, i0=c0, i1=c1, split=r, direction=name)
                for c0, c1, r in _carve(seg.i0, seg.i1, cut, "train")
            )
        return out
    if role in ("train", "heldout"):
        mask = driving_mask(run)
        out = []
        min_len = int(MIN_SEGMENT_S * 50.0)
        for a, b in _runs_of_true(mask):
            if b - a >= min_len:
                out.append(Segment(run, motion, int(a), int(b), role, "whole", "whole"))
        return out
    return []  # unused, heldout_layout: nothing for the dynamics workstream


def by_split(segments: Sequence[Segment], split: str, odom_only: bool = False) -> list[Segment]:
    return [s for s in segments if s.split == split and (s.run.has_odom or not odom_only)]


def seconds(segments: Sequence[Segment]) -> float:
    return float(sum(s.duration_s for s in segments))
