"""Strata, macro RMSE and the paired block bootstrap of pre-registration section 3.

Strata: direction (from the split file) by start-speed bin [0.25, 0.6), [0.6, 0.85),
[0.85, 1.25), 1.25 and up. A stratum with fewer than 30 windows is merged into the next lower bin
of the same direction and listed as merged. Headline numbers are MACRO averages, the unweighted
mean over strata of the per stratum RMSE.

Bootstrap: windows are grouped into blocks of 5.0 s within a segment (evaluate.EvalSet.block),
whole blocks are drawn with replacement (2000 resamples, seed 0) and every model is scored on the
same resample, so the ratio of two models' macro RMSE is paired. The interval is the 95 percent
percentile interval of the ratio.
"""

from __future__ import annotations

from collections.abc import Sequence
from dataclasses import dataclass, field

import numpy as np

SPEED_BIN_EDGES_MPS = (0.25, 0.6, 0.85, 1.25)  # the last bin is open at the top
MIN_STRATUM_WINDOWS = 30
BOOTSTRAP_RESAMPLES = 2000
BOOTSTRAP_SEED = 0
BOOTSTRAP_LEVEL = 0.95


def bin_of(speed_mps: np.ndarray) -> np.ndarray:
    """Bin index 0..3 of the start speeds (a speed below the first edge goes in bin 0)."""
    return np.clip(np.searchsorted(SPEED_BIN_EDGES_MPS, speed_mps, side="right") - 1, 0, 3)


def bin_label(lo: int, hi: int) -> str:
    """Label of the bins lo..hi (hi inclusive) merged, e.g. `0.25-0.85` or `1.25+`."""
    a = SPEED_BIN_EDGES_MPS[lo]
    if hi == len(SPEED_BIN_EDGES_MPS) - 1:
        return f"{a:g}+"
    return f"{a:g}-{SPEED_BIN_EDGES_MPS[hi + 1]:g}"


@dataclass(frozen=True)
class Strata:
    label: np.ndarray  # (E,) str, "<direction>:<bins>"
    names: tuple[str, ...]  # strata with at least one window, sorted
    counts: dict[str, int]
    merges: tuple[dict, ...] = field(default_factory=tuple)  # what was merged into what
    unmerged_small: tuple[str, ...] = field(default_factory=tuple)  # < min windows, no lower bin


def assign_strata(
    direction: np.ndarray,
    start_speed_mps: np.ndarray,
    min_windows: int = MIN_STRATUM_WINDOWS,
) -> Strata:
    """Stratum of each window, with the merge rule applied per direction from the top bin down:
    a bin with fewer than `min_windows` windows (counting what was merged into it) is merged into
    the next lower bin. The lowest bin has no lower neighbour: if it is still short it stays and
    is listed in `unmerged_small` (the rule has nothing to say about it)."""
    direction = np.asarray(direction, dtype=str)
    bins = bin_of(np.asarray(start_speed_mps, dtype=float))
    n_bins = len(SPEED_BIN_EDGES_MPS)
    label = np.empty(len(bins), dtype=object)
    merges: list[dict] = []
    small: list[str] = []
    for d in sorted(set(direction.tolist())):
        sel = direction == d
        counts = [int(np.sum(sel & (bins == b))) for b in range(n_bins)]
        group = list(range(n_bins))  # lowest bin of the merged group each bin belongs to

        def members(g: int, group: list[int] = group) -> list[int]:
            return [k for k in range(n_bins) if group[k] == g]

        for b in range(n_bins - 1, 0, -1):
            total = sum(counts[k] for k in members(b))
            if 0 < total < min_windows:
                lower = group[b - 1]
                merges.append(
                    {
                        "direction": d,
                        "from": bin_label(b, max(members(b))),
                        "windows": total,
                        "into": bin_label(lower, b - 1),
                    }
                )
                for k in members(b):
                    group[k] = lower
        for b in range(n_bins):
            m = members(group[b])
            label[sel & (bins == b)] = f"{d}:{bin_label(min(m), max(m))}"
        n_low = sum(counts[k] for k in members(0))
        if 0 < n_low < min_windows:
            small.append(f"{d}:{bin_label(0, max(members(0)))}")
    names = tuple(sorted({str(x) for x in label}))
    cnt = {n: int(np.sum(label == n)) for n in names}
    return Strata(label.astype(str), names, cnt, tuple(merges), tuple(small))


def stratum_rmse(err: np.ndarray, strata: Strata) -> dict[str, float]:
    """RMSE of `err` (E,) per stratum."""
    return {
        n: float(np.sqrt(np.mean(err[strata.label == n] ** 2)))
        for n in strata.names
        if strata.counts[n] > 0
    }


def macro_rmse(err: np.ndarray, strata: Strata) -> float:
    """Unweighted mean over strata of the per stratum RMSE (NaN with no windows)."""
    per = stratum_rmse(err, strata)
    return float(np.mean(list(per.values()))) if per else float("nan")


def block_bootstrap_ratio(
    err_a: np.ndarray,
    err_b: np.ndarray,
    strata: Strata,
    block: np.ndarray,
    n_resamples: int = BOOTSTRAP_RESAMPLES,
    seed: int = BOOTSTRAP_SEED,
    level: float = BOOTSTRAP_LEVEL,
) -> dict[str, float]:
    """Macro RMSE of model A over model B (both per window errors on the same windows), with the
    percentile interval from resampling whole blocks. A stratum with no window in a resample is
    left out of that resample's macro mean (`empty_stratum_share` says how often)."""
    err_a, err_b = np.asarray(err_a, float), np.asarray(err_b, float)
    block = np.asarray(block)
    ids = np.unique(block)
    names = [n for n in strata.names if strata.counts[n] > 0]
    pos = np.searchsorted(ids, block)
    s_idx = np.array([names.index(x) for x in strata.label])
    shape = (len(ids), len(names))
    cnt = np.zeros(shape)
    sa = np.zeros(shape)
    sb = np.zeros(shape)
    np.add.at(cnt, (pos, s_idx), 1.0)
    np.add.at(sa, (pos, s_idx), err_a**2)
    np.add.at(sb, (pos, s_idx), err_b**2)
    rng = np.random.default_rng(seed)
    draw = rng.integers(0, len(ids), size=(n_resamples, len(ids)))
    c = cnt[draw].sum(axis=1)
    with np.errstate(invalid="ignore", divide="ignore"):
        ra = np.sqrt(sa[draw].sum(axis=1) / c)
        rb = np.sqrt(sb[draw].sum(axis=1) / c)
    ratios = np.nanmean(ra, axis=1) / np.nanmean(rb, axis=1)
    lo, hi = np.percentile(ratios, [50 * (1 - level), 50 * (1 + level)])
    return {
        "ratio": macro_rmse(err_a, strata) / macro_rmse(err_b, strata),
        "lo": float(lo),
        "hi": float(hi),
        "resamples": n_resamples,
        "blocks": len(ids),
        "seed": seed,
        "empty_stratum_share": float(np.mean(c == 0)),
    }


def per_stratum_table(
    errs: dict[str, np.ndarray], strata: Strata
) -> dict[str, dict[str, float | int]]:
    """{stratum: {"windows": n, <model>: RMSE, ...}} for the named per window error arrays."""
    out: dict[str, dict[str, float | int]] = {}
    for n in strata.names:
        row: dict[str, float | int] = {"windows": strata.counts[n]}
        for model, e in errs.items():
            row[model] = float(np.sqrt(np.mean(e[strata.label == n] ** 2)))
        out[n] = row
    return out


def window_counts(direction: Sequence[str], start_speed_mps: np.ndarray) -> dict[str, int]:
    """Windows per direction and ORIGINAL bin (before any merge), for the report."""
    direction = np.asarray(direction, dtype=str)
    bins = bin_of(np.asarray(start_speed_mps, dtype=float))
    return {
        f"{d}:{bin_label(b, b)}": int(np.sum((direction == d) & (bins == b)))
        for d in sorted(set(direction.tolist()))
        for b in range(len(SPEED_BIN_EDGES_MPS))
    }
