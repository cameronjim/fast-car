"""Truth T2 of the 2026-11-15 session, cached in three places that stage code opens by role
(pre-registration v0.5 section 5, yaml `data_roles`):

    pnp-v05-t2-train/<bag>.<part>.motion-wheel[-variant].npz     train parts: every fit stage
    pnp-v05-t2-heldout/<bag>.<part>.motion-wheel[-variant].npz   held-out parts: the one frozen pass
    pnp-v05-t2-whole/<bag>.<run>.motion-wheel.npz                whole PnP runs: M-B6 only

The three directories are siblings, so a glob or a prefix check of one never reaches another.
`load_train_motion` refuses any role but `train`; `load_heldout_motion` needs a `FreezeToken` (the
git SHA of the freeze commit, `truth_v04.freeze`) and the role `heldout`; `load_whole_motion` is the
carry-over reader of M-B6 (the whole runs contain the held-out intervals, so no fit stage may open
it) and refuses any purpose but `m-b6`. The 2026-11-14 held-out cache of v0.4 is the SELECTION set
and is not here: it is read by `stages_v05.selection_eval_set` only.
"""

from __future__ import annotations

from pathlib import Path

import numpy as np

from .icp_motion import MotionSeries
from .truth_v04 import T2, VARIANTS, FreezeToken, HeldoutLocked, Slice, read_slice

TRAIN_DIR = "pnp-v05-t2-train"
HELDOUT_DIR = "pnp-v05-t2-heldout"
WHOLE_DIR = "pnp-v05-t2-whole"
CARRY_OVER_PURPOSE = "m-b6"
SLICE_PURPOSE = "session-truth"  # the stage that writes the part caches from the whole runs


def _file(bag: str, part: str, variant: str = T2) -> str:
    if variant not in VARIANTS:
        raise ValueError(f"unknown truth variant {variant!r}")
    tag = "" if variant == T2 else f"-{variant}"
    return f"{bag}.{part}.motion-wheel{tag}.npz"


def train_path(cache_dir: Path, bag: str, part: str, variant: str = T2) -> Path:
    return Path(cache_dir) / TRAIN_DIR / _file(bag, part, variant)


def heldout_path(cache_dir: Path, bag: str, part: str, variant: str = T2) -> Path:
    return Path(cache_dir) / HELDOUT_DIR / _file(bag, part, variant)


def whole_path(cache_dir: Path, bag: str, run: str) -> Path:
    return Path(cache_dir) / WHOLE_DIR / _file(bag, run)


def load_train_motion(cache_dir: Path, bag: str, part: str, role: str, variant: str = T2) -> Slice:
    """What every fit stage reads. Refuses a part that is not train."""
    if role != "train":
        raise HeldoutLocked(
            f"{bag}/{part} has role {role!r}: the fit stages read train parts of the 2026-11-15 "
            "session only"
        )
    return read_slice(train_path(cache_dir, bag, part, variant))


def load_heldout_motion(
    cache_dir: Path, bag: str, part: str, role: str, token: FreezeToken, variant: str = T2
) -> Slice:
    """The one reader of the held-out cache: the frozen pass, with its freeze token."""
    if not isinstance(token, FreezeToken):
        raise HeldoutLocked("the held-out truth needs a FreezeToken (see truth_v05.freeze)")
    if role != "heldout":
        raise HeldoutLocked(f"{bag}/{part} has role {role!r}, not heldout")
    return read_slice(heldout_path(cache_dir, bag, part, variant))


def load_whole_motion(cache_dir: Path, bag: str, run: str, purpose: str) -> Slice:
    """The carry-over reader: whole PnP runs, scored by M-B6 before the split file exists, and cut
    into the train and held-out part caches by the `session-truth` stage."""
    if purpose not in (CARRY_OVER_PURPOSE, SLICE_PURPOSE):
        raise HeldoutLocked(
            f"purpose {purpose!r}: the whole-run truth holds held-out intervals and is read by "
            f"{CARRY_OVER_PURPOSE} and {SLICE_PURPOSE} only"
        )
    return read_slice(whole_path(cache_dir, bag, run))


def cut_slice(whole: Slice, first: int, last: int) -> Slice:
    """The intervals [first, last) (indices of the run) of a slice, as a slice of their own. The
    truth of an interval is computed from its two scans and the wheel and steering at that time
    only, so this is bit-equal to computing the part directly."""
    a, b = first - whole.first, last - whole.first
    if a < 0 or b > len(whole.motion.t_s) or b < a:
        raise ValueError("the part reaches outside the slice it is cut from")
    cut = {
        k: getattr(whole.motion, k)[a:b]
        for k in MotionSeries.__dataclass_fields__
        if getattr(whole.motion, k) is not None
    }
    return Slice(first, MotionSeries(**cut))


def part_intervals(stamps_s: np.ndarray, t_from_s: float, t_to_s: float) -> tuple[int, int]:
    """Scan intervals [first, last) whose two scans both lie inside [t_from_s, t_to_s]."""
    from .truth_v04 import segment_intervals

    return segment_intervals(stamps_s, t_from_s, t_to_s)
