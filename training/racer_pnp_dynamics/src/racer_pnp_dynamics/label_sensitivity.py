"""M-B4, label sensitivity (addendum 1 section 2.4): does the truth lean towards "ground equals
wheel"? A measurement on TRAIN data only, committed before any fit. It has no threshold.

Per stratum (the strata of the T2 windows, after merging): the median of ground distance over wheel
distance in 1.0 s windows under

* T2, the wheel seeded chain;
* T2s, the same chain seeded with 0.80 of the wheel translation;
* T2 with the scan clock shifted by +0.010 s and -0.010 s (the shift moves the stamps used to look
  up the wheel speed and the steering for the seed AND the stamps of the wheel distance of the
  window, so a clock error shows up in both).

Ground distance is the length of the ICP chain over the window (sum of the interval step lengths),
wheel distance is the integral of the wheel speed over the same 1.0 s. A stratum with at least 30
windows in every variant whose T2 and T2s medians differ by more than 0.03 is reported as "label
depends on the seed". Also: the share of candidate windows rejected by the ICP quality flags, by
speed bin and direction. A candidate window is one whose every interval was matched (finite), the
quality flags are then the only reason left to drop it.
"""

from __future__ import annotations

from collections.abc import Mapping, Sequence
from dataclasses import replace

import numpy as np

from .dataset import Segment
from .evaluate import EvalSet, eval_set
from .icp_motion import SCAN_DT_S, MotionSeries
from .model import DT_S
from .strata import SPEED_BIN_EDGES_MPS, assign_strata, bin_of
from .windows import wheel_at

HORIZON_S = 1.0
DEPENDENT_IF_RATIO_DIFFERS_BY = 0.03  # yaml ground_motion.label_sensitivity
MIN_WINDOWS = 30
DEPENDENT_TEXT = "label depends on the seed"
CLOCK_SHIFTS_S = (-0.010, 0.010)


def _key(es: EvalSet) -> list[tuple[str, float]]:
    return [(str(s), round(float(t), 3)) for s, t in zip(es.segment, es.t_s, strict=True)]


def ground_over_wheel(
    es: EvalSet, shift_s: float = 0.0, horizon_s: float = HORIZON_S
) -> np.ndarray:
    """(E,) ground distance over wheel distance of each window. NaN where the wheel speed is
    missing or the wheel distance is not positive."""
    n_iv = round(horizon_s / SCAN_DT_S)
    steps = np.diff(es.poses[:, : n_iv + 1, :2], axis=1)
    ground = np.hypot(steps[..., 0], steps[..., 1]).sum(axis=1)
    k = round(horizon_s / DT_S)
    offsets = np.arange(k + 1) * DT_S
    wheel = np.full(len(es), np.nan)
    for r, run in enumerate(es.runs):
        sel = np.flatnonzero(es.run_idx == r)
        if len(sel):
            v = np.maximum(wheel_at(run, es.t_s[sel][:, None] + shift_s + offsets[None, :]), 0.0)
            wheel[sel] = np.trapezoid(v, dx=DT_S, axis=1)
    with np.errstate(invalid="ignore", divide="ignore"):
        return np.where(wheel > 0.0, ground / wheel, np.nan)


def with_every_computed_interval_ok(segments: Sequence[Segment]) -> list[Segment]:
    """Copies of `segments` whose motion is ok wherever the interval was matched at all, so the
    windows left over are the candidates the quality flags then reject."""
    out = []
    for s in segments:
        m = s.motion
        if m is None:
            out.append(s)
            continue
        finite = np.isfinite(m.dx_m) & np.isfinite(m.dy_m) & np.isfinite(m.dyaw_rad)
        out.append(replace(s, motion=MotionSeries(**{**m.__dict__, "ok": finite})))
    return out


def rejection_table(segments: Sequence[Segment]) -> list[dict[str, float | str]]:
    """Per direction and start speed bin: candidate windows, windows kept by the quality flags,
    rejected share."""
    cand = eval_set(with_every_computed_interval_ok(segments))
    kept = eval_set(segments)
    kept_keys = set(_key(kept))
    is_kept = np.array([k in kept_keys for k in _key(cand)], dtype=bool)
    bins = bin_of(cand.start_speed)
    rows: list[dict[str, float | str]] = []
    for d in sorted(set(cand.direction.tolist())):
        for b in range(len(SPEED_BIN_EDGES_MPS)):
            sel = (cand.direction == d) & (bins == b)
            if not sel.any():
                continue
            rows.append(
                {
                    "direction": d,
                    "speed_bin": _bin_label(b),
                    "candidates": float(sel.sum()),
                    "kept": float(is_kept[sel].sum()),
                    "rejected_share": float(1.0 - is_kept[sel].mean()),
                }
            )
    return rows


def _bin_label(b: int) -> str:
    e = SPEED_BIN_EDGES_MPS
    return f"{e[b]:g}-{e[b + 1]:g}" if b + 1 < len(e) else f"{e[b]:g}+"


def sensitivity_table(
    t2: Sequence[Segment],
    variants: Mapping[str, Sequence[Segment]],
    shifts_s: Mapping[str, float] | None = None,
) -> list[dict[str, object]]:
    """One row per stratum of the T2 windows: windows, median ground over wheel under T2 and under
    every variant of `variants` (name -> segments built on that truth), and the dependence flag
    against the variant named `s0.80` when present. `shifts_s` gives the clock shift of a variant
    (for the wheel distance of its windows)."""
    shifts_s = shifts_s or {}
    es = eval_set(t2)
    if len(es) == 0:
        raise ValueError("no T2 windows")
    strata = assign_strata(es.direction, es.start_speed)
    base = ground_over_wheel(es)
    per_variant: dict[str, dict[tuple[str, float], float]] = {}
    for name, segs in variants.items():
        ev = eval_set(segs)
        ratio = ground_over_wheel(ev, shifts_s.get(name, 0.0))
        per_variant[name] = dict(zip(_key(ev), ratio, strict=True))
    keys = _key(es)
    rows: list[dict[str, object]] = []
    for name in strata.names:
        sel = np.flatnonzero(strata.label == name)
        row: dict[str, object] = {
            "stratum": name,
            "windows": len(sel),
            "median_T2": _median(base[sel]),
        }
        counts = [len(sel)]
        for vname, table in per_variant.items():
            vals = np.array([table.get(keys[i], np.nan) for i in sel])
            row[f"median_{vname}"] = _median(vals)
            row[f"windows_{vname}"] = int(np.isfinite(vals).sum())
            counts.append(int(np.isfinite(vals).sum()))
        seed = row.get("median_s0.80")
        dependent = (
            seed is not None
            and min(counts) >= MIN_WINDOWS
            and abs(float(seed) - float(row["median_T2"])) > DEPENDENT_IF_RATIO_DIFFERS_BY
        )
        row["label_depends_on_the_seed"] = bool(dependent)
        rows.append(row)
    return rows


def _median(x: np.ndarray) -> float:
    x = x[np.isfinite(x)]
    return float(np.median(x)) if len(x) else float("nan")


def render_mb4(
    table: Sequence[Mapping[str, object]],
    rejection: Sequence[Mapping[str, object]],
    variant_names: Sequence[str],
    notes: Sequence[str],
) -> str:
    head = ["stratum", "windows", "T2", *variant_names, "reading"]
    lines = [
        "# M-B4: label sensitivity, train data",
        "",
        (
            "Addendum 1 section 2.4. A measurement on TRAIN data only, produced and committed "
            "before any v0.4 fit. No threshold; the gate items under T2s are the guard."
        ),
        "",
        *[f"- {n}" for n in notes],
        "",
        (
            "Cells: median over windows of ground distance over wheel distance in 1.0 s windows "
            f"(T2: wheel seeded chain; s0.80: seeded with 0.80 of the wheel translation; shift: "
            f"scan clock moved by 10 ms). A stratum with at least {MIN_WINDOWS} windows in every "
            f"variant whose T2 and s0.80 medians differ by more than "
            f'{DEPENDENT_IF_RATIO_DIFFERS_BY} is reported as "{DEPENDENT_TEXT}".'
        ),
        "",
        "| " + " | ".join(head) + " |",
        "|" + "---|" * len(head),
    ]
    for r in table:
        cells = [f"{r['median_T2']:.3f}"] + [f"{r[f'median_{v}']:.3f}" for v in variant_names]
        reading = DEPENDENT_TEXT if r["label_depends_on_the_seed"] else ""
        lines.append(
            f"| {r['stratum']} | {r['windows']} | " + " | ".join(cells) + f" | {reading} |"
        )
    lines += [
        "",
        (
            "Windows rejected by the ICP quality flags (of the candidate windows whose intervals "
            "were all matched), under T2:"
        ),
        "",
        "| direction | speed bin | candidates | kept | rejected share |",
        "|---|---|---|---|---|",
    ]
    for r in rejection:
        lines.append(
            f"| {r['direction']} | {r['speed_bin']} | {int(r['candidates'])} | {int(r['kept'])} | "
            f"{r['rejected_share']:.3f} |"
        )
    lines.append("")
    return "\n".join(lines)
