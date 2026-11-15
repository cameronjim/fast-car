"""Truth T2 (wheel seeded ICP) of the 2026-11-14 segments, computed ONLY inside segments, cached
per segment, with the held-out cache in a place no stage 1 code path opens (brief 37 section 0).

Layout under the cache directory (`car-data/pnp/cache/`):

    pnp-v04-t2-train/<bag>.<segment>.motion-wheel.npz      train segments (read by every stage)
    pnp-v04-t2-heldout/<bag>.<segment>.motion-wheel.npz    held-out segments (frozen pass only)

The two directories are siblings, so a glob or a prefix check of the train one never reaches the
held-out one. `load_stage1_motion` takes a segment role and refuses `heldout`; the only reader of
the held-out directory is `load_heldout_motion`, and it needs a `FreezeToken`, which is built from
the git SHA of the frozen candidate (`freeze`). A slice holds the intervals of one segment; the
run level `MotionSeries` that `dataset.make_segments` reads is assembled with `assemble`, the
intervals outside the loaded segments are not ok.
"""

from __future__ import annotations

import re
from collections.abc import Sequence
from dataclasses import dataclass
from pathlib import Path

import numpy as np

from .icp_motion import MotionSeries

TRAIN_DIR = "pnp-v04-t2-train"
HELDOUT_DIR = "pnp-v04-t2-heldout"
_SHA = re.compile(r"^[0-9a-f]{40}$")


class HeldoutLocked(PermissionError):
    """The held-out truth was asked for without a freeze token, or by a stage that may not."""


@dataclass(frozen=True)
class FreezeToken:
    """Proof that the candidates are frozen: the 40 hex digit git SHA of the freeze commit."""

    git_sha: str


def freeze(git_sha: str) -> FreezeToken:
    if not _SHA.match(git_sha):
        raise HeldoutLocked(
            f"{git_sha!r} is not a git SHA: the held-out pass needs the freeze commit"
        )
    return FreezeToken(git_sha)


def _file(bag: str, segment: str) -> str:
    return f"{bag}.{segment}.motion-wheel.npz"


def train_path(cache_dir: Path, bag: str, segment: str) -> Path:
    return Path(cache_dir) / TRAIN_DIR / _file(bag, segment)


def heldout_path(cache_dir: Path, bag: str, segment: str) -> Path:
    return Path(cache_dir) / HELDOUT_DIR / _file(bag, segment)


@dataclass(frozen=True)
class Slice:
    """The T2 intervals of one segment: scan intervals [first, first + len(motion))."""

    first: int
    motion: MotionSeries


def save_slice(path: Path, sl: Slice) -> None:
    path.parent.mkdir(parents=True, exist_ok=True)
    arrays = {
        k: getattr(sl.motion, k)
        for k in MotionSeries.__dataclass_fields__
        if getattr(sl.motion, k) is not None
    }
    np.savez_compressed(path, first=np.array(sl.first), **arrays)


def read_slice(path: Path) -> Slice:
    with np.load(path) as z:
        fields = MotionSeries.__dataclass_fields__
        return Slice(int(z["first"]), MotionSeries(**{k: z[k] for k in fields if k in z.files}))


def load_stage1_motion(cache_dir: Path, bag: str, segment: str, role: str) -> Slice:
    """What the budget, the fits and the residual training use. Refuses a held-out segment."""
    if role != "train":
        raise HeldoutLocked(
            f"{bag}/{segment} has role {role!r}: stage 1 (budget, fits, residual training) reads "
            "train segments only"
        )
    return read_slice(train_path(cache_dir, bag, segment))


def load_heldout_motion(
    cache_dir: Path, bag: str, segment: str, role: str, token: FreezeToken
) -> Slice:
    """The one reader of the held-out cache: the frozen pass, with its freeze token."""
    if not isinstance(token, FreezeToken):
        raise HeldoutLocked("the held-out truth needs a FreezeToken (see truth_v04.freeze)")
    if role != "heldout":
        raise HeldoutLocked(f"{bag}/{segment} has role {role!r}, not heldout")
    return read_slice(heldout_path(cache_dir, bag, segment))


def segment_intervals(stamps_s: np.ndarray, t_from_s: float, t_to_s: float) -> tuple[int, int]:
    """Scan intervals [i0, i1) whose two scans both lie inside [t_from_s, t_to_s]."""
    n = len(stamps_s) - 1
    if n <= 0:
        return 0, 0
    i0 = int(np.searchsorted(stamps_s, t_from_s, side="left"))
    i1 = int(np.searchsorted(stamps_s, t_to_s, side="right")) - 1  # last scan inside is i1
    return i0, max(min(i1, n), i0)


def assemble(stamps_s: np.ndarray, slices: Sequence[Slice]) -> MotionSeries:
    """The run level series: every interval of the run, ok only inside the given slices."""
    n = max(len(stamps_s) - 1, 0)
    cols = {k: np.full(n, np.nan) for k in ("dx_m", "dy_m", "dyaw_rad", "rms_m", "inlier_share")}
    ok = np.zeros(n, dtype=bool)
    seeded = np.zeros(n, dtype=bool)
    for sl in slices:
        a, b = sl.first, sl.first + len(sl.motion.t_s)
        if a < 0 or b > n:
            raise ValueError("a slice reaches outside the run")
        if ok[a:b].any() or seeded[a:b].any():
            raise ValueError("two slices cover the same intervals")
        for k, col in cols.items():
            col[a:b] = getattr(sl.motion, k)
        ok[a:b] = sl.motion.ok
        seeded[a:b] = sl.motion.seeded if sl.motion.seeded is not None else False
    return MotionSeries(
        t_s=np.asarray(stamps_s[:n], dtype=float),
        dt_s=np.diff(stamps_s) if n else np.zeros(0),
        ok=ok,
        seeded=seeded,
        **cols,
    )


def compute_segment_slice(  # pragma: no cover - real data (ICP on the bag)
    run, vehicle, ignore, t_from_s: float, t_to_s: float, workers: int = 2
) -> Slice:
    """T2 of the scan intervals inside [t_from_s, t_to_s] (run time) and nothing else."""
    from .icp_motion import SEED_WHEEL, compute_motion, driving_intervals

    stamps = np.array([s.stamp_s for s in run.scans])
    i0, i1 = segment_intervals(stamps, t_from_s, t_to_s)
    n = len(stamps) - 1
    want = np.zeros(n, dtype=bool)
    want[i0:i1] = driving_intervals(run)[i0:i1]
    full = compute_motion(
        run.scans, vehicle, want, workers=workers, ignore=ignore, seed=SEED_WHEEL, run=run
    )
    cut = {
        k: getattr(full, k)[i0:i1]
        for k in MotionSeries.__dataclass_fields__
        if getattr(full, k) is not None
    }
    return Slice(i0, MotionSeries(**cut))
