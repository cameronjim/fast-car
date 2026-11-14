"""Real self-labels as training data: the role filter, the internal validation cut, the mirror
and the sensor nuisances that make sense on a real scan.

Only labels whose role is `train` may enter training. `load_real` takes the label files written by
`selflabel --split`, checks every label's recorded role against the split (the file and the split
must agree, and the split file must be the pre-registered one), drops everything that is not
`train`, and returns the segments it used. A label file without roles (written before the split
existed) is refused: nothing proves what it is.

Internal validation: the last `VAL_FRACTION` (in time) of each train segment, minus the labels
whose submap window crosses the cut (they are in neither part). The cut is the span of the
segment in the split; for a whole-bag segment it is the span of that bag's labels. ALL model
selection uses sim validation plus this part only.

Mirror: bin j of the input tensor covers vehicle bearing [-pi + j w, -pi + (j + 1) w), so the
mirror image of a scan (y to -y) is the tensor read backwards; the label keeps x, negates y and
swaps the two half widths. Nuisances: bin dropout and a dead sector, drawn like the simulator's
(`nuisance.py`) but applied to the tensor. No geometric warps: they would invalidate the label.
"""

from __future__ import annotations

import logging
import math
from dataclasses import dataclass
from pathlib import Path
from typing import Any

import numpy as np
from numpy.typing import NDArray

from racer_pnp_perception import nuisance as nz
from racer_pnp_perception.split import ROLE_TRAIN, Split

log = logging.getLogger(__name__)

VAL_FRACTION = 0.15
DROPOUT_MAX = nz.DROPOUT_MAX
DEAD_SECTOR_MAX_RAD = nz.DEAD_SECTOR_MAX_RAD
N_BINS = 360
BIN_WIDTH_RAD = 2.0 * math.pi / N_BINS
REQUIRED_KEYS = (
    "index",
    "path_xy",
    "tensor",
    "label",
    "valid_count",
    "segment",
    "role",
    "window_first_s",
    "window_last_s",
)


class RoleError(ValueError):
    """A label file cannot be used for training."""


@dataclass
class RealSet:
    tensor: NDArray[np.float32]  # [n, 2, 360]
    label: NDArray[np.float32]  # [n, 16, 5]
    valid_count: NDArray[np.int64]  # [n]
    bag: NDArray[np.str_]
    segment: NDArray[np.str_]
    role: NDArray[np.str_]
    window_first_s: NDArray[np.float64]
    window_last_s: NDArray[np.float64]
    scan_index: NDArray[np.int64]  # index into the bag's scans (the scan cache)
    path_xy: NDArray[np.float64]  # [n, 21, 2] the car's own next 2 m, in the scan's frame

    def __len__(self) -> int:
        return len(self.valid_count)

    def take(self, mask: NDArray[np.bool_]) -> RealSet:
        return RealSet(
            self.tensor[mask],
            self.label[mask],
            self.valid_count[mask],
            self.bag[mask],
            self.segment[mask],
            self.role[mask],
            self.window_first_s[mask],
            self.window_last_s[mask],
            self.scan_index[mask],
            self.path_xy[mask],
        )


def read_label_file(path: Path) -> RealSet:
    """One label npz as a RealSet (any role). Refuses a file without the split fields."""
    bag = path.name.removesuffix(".npz")
    with np.load(path) as z:
        missing = [k for k in REQUIRED_KEYS if k not in z.files]
        if missing:
            raise RoleError(f"{path}: no {', '.join(missing)}; relabel with `selflabel --split`")
        n = len(z["valid_count"])
        return RealSet(
            z["tensor"].reshape(n, 2, N_BINS).astype(np.float32),
            z["label"].astype(np.float32),
            z["valid_count"].astype(np.int64),
            np.array([bag] * n, dtype=np.str_),
            z["segment"].astype(np.str_),
            z["role"].astype(np.str_),
            z["window_first_s"].astype(np.float64),
            z["window_last_s"].astype(np.float64),
            z["index"].astype(np.int64),
            z["path_xy"].astype(np.float64),
        )


def concat(sets: list[RealSet]) -> RealSet:
    if not sets:
        raise RoleError("no label files")
    return RealSet(
        np.concatenate([s.tensor for s in sets]),
        np.concatenate([s.label for s in sets]),
        np.concatenate([s.valid_count for s in sets]),
        np.concatenate([s.bag for s in sets]),
        np.concatenate([s.segment for s in sets]),
        np.concatenate([s.role for s in sets]),
        np.concatenate([s.window_first_s for s in sets]),
        np.concatenate([s.window_last_s for s in sets]),
        np.concatenate([s.scan_index for s in sets]),
        np.concatenate([s.path_xy for s in sets]),
    )


def split_role_of(real: RealSet, split: Split) -> NDArray[np.str_]:
    """The role the split gives each label's window (not the one the file claims)."""
    return np.array(
        [
            split.role_of(str(b), float(f), float(t))
            for b, f, t in zip(real.bag, real.window_first_s, real.window_last_s, strict=True)
        ],
        dtype=np.str_,
    )


def only_train(real: RealSet, split: Split) -> RealSet:
    """The labels whose recorded role AND split role are `train`. Raises RoleError when a label's
    recorded role disagrees with the split (a stale or edited label file)."""
    derived = split_role_of(real, split)
    if not np.array_equal(derived, real.role):
        bad = int((derived != real.role).sum())
        raise RoleError(f"{bad} labels have a role that disagrees with the split")
    keep = (real.role == ROLE_TRAIN) & (derived == ROLE_TRAIN)
    out = real.take(keep)
    assert bool((out.role == ROLE_TRAIN).all())
    return out


def load_real(paths: list[Path], split: Split) -> RealSet:
    """The `train` labels of the given label files. Everything else is dropped (and counted in
    the log); a file that cannot be checked against the split is refused."""
    everything = concat([read_label_file(p) for p in paths])
    train = only_train(everything, split)
    log.info(
        "real labels: %d read, %d train kept, %d of other roles dropped",
        len(everything),
        len(train),
        len(everything) - len(train),
    )
    return train


def segment_span(real: RealSet, split: Split, bag: str, segment: str) -> tuple[float, float]:
    """(from, to) of a segment in bag time: from the split, or for a whole bag the span of the
    labels of that bag."""
    entry = split.bags[bag]
    if entry.whole_bag_role is not None:
        m = real.bag == bag
        return float(real.window_first_s[m].min()), float(real.window_last_s[m].max())
    for seg in entry.segments:
        if seg.name == segment:
            return seg.t_from_s, seg.t_to_s
    raise RoleError(f"{bag}: no segment {segment}")


def train_val_split(
    real: RealSet, split: Split, val_fraction: float = VAL_FRACTION
) -> tuple[RealSet, RealSet, dict[str, Any]]:
    """(train part, internal validation part, report): per train segment the cut is at
    `1 - val_fraction` of its span; a label whose window lies before the cut trains, one after it
    validates, one that crosses it is in neither."""
    is_train = np.zeros(len(real), dtype=bool)
    is_val = np.zeros(len(real), dtype=bool)
    report: dict[str, Any] = {}
    for bag in sorted(set(real.bag.tolist())):
        for segment in sorted(set(real.segment[real.bag == bag].tolist())):
            m = (real.bag == bag) & (real.segment == segment)
            t0, t1 = segment_span(real, split, bag, segment)
            cut = t0 + (1.0 - val_fraction) * (t1 - t0)
            tr_m = m & (real.window_last_s <= cut)
            va_m = m & (real.window_first_s >= cut)
            is_train |= tr_m
            is_val |= va_m
            report[f"{bag}/{segment}"] = {
                "from_s": t0,
                "to_s": t1,
                "cut_s": cut,
                "train": int(tr_m.sum()),
                "val": int(va_m.sum()),
                "crossing_dropped": int(m.sum() - tr_m.sum() - va_m.sum()),
            }
    assert not (is_train & is_val).any()
    return real.take(is_train), real.take(is_val), report


def mirror_tensor(tensor: NDArray[np.floating]) -> NDArray[np.floating]:
    """The scan seen in a mirror (left and right swapped): the bins read backwards."""
    return np.ascontiguousarray(tensor[..., ::-1])


def mirror_label(label: NDArray[np.floating]) -> NDArray[np.floating]:
    """The label of the mirrored scan: y negated, half widths swapped, x and sigma unchanged."""
    out = np.array(label, copy=True)
    out[..., 1] = -label[..., 1]
    out[..., 2] = label[..., 3]
    out[..., 3] = label[..., 2]
    return out


def mirror_sample(
    tensor: NDArray[np.floating], label: NDArray[np.floating]
) -> tuple[NDArray[np.floating], NDArray[np.floating]]:
    return mirror_tensor(tensor), mirror_label(label)


def drop_bins(
    tensor: NDArray[np.float32],
    rng: np.random.Generator,
    max_rate: float = DROPOUT_MAX,
    dead_sector_max_rad: float = DEAD_SECTOR_MAX_RAD,
) -> NDArray[np.float32]:
    """Per scan: bins dropped with a rate drawn from [0, max_rate], and with probability 0.7 a dead
    sector (width up to `dead_sector_max_rad`, centred behind the car within the simulator's
    jitter). A dropped bin is what an empty bin is: range 1.0, validity 0. [n, 2, 360] in and out."""
    out = np.array(tensor, copy=True)
    n = len(out)
    rate = rng.uniform(0.0, max_rate, size=n)
    drop = rng.uniform(size=(n, N_BINS)) < rate[:, None]
    has_dead = rng.uniform(size=n) < 0.7
    width = rng.uniform(0.0, dead_sector_max_rad, size=n) * has_dead
    centre = math.pi + rng.uniform(
        -nz.DEAD_SECTOR_CENTRE_JITTER_RAD, nz.DEAD_SECTOR_CENTRE_JITTER_RAD, size=n
    )
    bearing = -math.pi + (np.arange(N_BINS) + 0.5) * BIN_WIDTH_RAD
    delta = np.abs(np.mod(bearing[None, :] - centre[:, None] + math.pi, 2 * math.pi) - math.pi)
    drop |= delta <= width[:, None] / 2.0
    ch0, ch1 = out[:, 0, :], out[:, 1, :]
    ch0[drop] = 1.0
    ch1[drop] = 0.0
    return out


def augment_real(
    tensor: NDArray[np.float32],
    label: NDArray[np.float32],
    rng: np.random.Generator,
    mirror_probability: float = 0.5,
) -> tuple[NDArray[np.float32], NDArray[np.float32]]:
    """Mirror each sample with probability `mirror_probability`, then bin dropout. Returns copies."""
    flip = rng.uniform(size=len(tensor)) < mirror_probability
    x = np.where(flip[:, None, None], mirror_tensor(tensor), tensor)
    y = np.where(flip[:, None, None], mirror_label(label), label)
    return drop_bins(x, rng).astype(np.float32), y.astype(np.float32)
