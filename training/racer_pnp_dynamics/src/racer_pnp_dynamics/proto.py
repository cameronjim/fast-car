"""Prototype dynamics bundle of the overnight run 2026-11-23 (workstream W1, owner decision 29).

An UNREGISTERED development model: a residual (speed and yaw) on the deployed dyn-v0.1 nominal,
trained on every real session bag with odometry since 2026-11-15, to be driven on the car behind
a launch switch even if it is no better than dyn-v0.1. No gate is decided here; the held-out
numbers it prints are development numbers. Pre-registered gates (GP2g-R of brief 54 and every
earlier one) stay separate and are not touched.

Data roles, fixed in code before any fit:

* every run of a bag is a stretch of `tools/pnp_data.py runs-v05` (any driver), kept when it is at
  least `MIN_RUN_S` long with `MIN_WHEEL_M` of wheel distance;
* a run's first `1 - DEV_SHARE` minus `GUARD_S` is train (its last `VAL_SHARE` is the internal
  validation part, used for early stopping and to choose the config and seed), the last
  `DEV_SHARE` is the development held-out part, scored once after the fit;
* the bag of 2026-11-22 (`PROTECTED_BAGS`) is the held-out set of the pre-registered GP2g-R (brief
  54). Only its train role there (the first `PROTECTED_TRAIN_SHARE` of each run, the v0.5 rule) is
  read, nothing of it is scored, so that gate stays unspent.

Truth is T2 (wheel seeded ICP), as for every earlier dynamics fit. The session of 2026-11-15
reuses its cached v0.5 parts (that held-out set was spent by PR 204); the newer bags are computed
here, per part, into `CACHE_DIR`.
"""

from __future__ import annotations

from collections.abc import Mapping, Sequence
from dataclasses import dataclass
from itertools import pairwise

import numpy as np

MODEL_ID = "dyn-v0.7p-2026-11-24"
CACHE_DIR = "pnp-proto-t2"

MIN_RUN_S = 15.0
MIN_WHEEL_M = 1.0
DEV_SHARE = 0.25
GUARD_S = 2.0
VAL_SHARE = 0.15
PROTECTED_TRAIN_SHARE = 0.55
PROTECTED_BAGS = frozenset({"2026-11-23T03-50-32_car_teleop"})

TRAIN, VAL, DEV = "train", "val", "dev"
ROLES = (TRAIN, VAL, DEV)


class RoleRefused(PermissionError):
    """A part of a protected bag was asked for in a role it may not have."""


@dataclass(frozen=True)
class RunPart:
    """A time span [t_from_s, t_to_s] of one run, seconds after the bag's first scan log time."""

    bag: str
    run_id: str
    role: str
    t_from_s: float
    t_to_s: float

    @property
    def name(self) -> str:
        return f"{self.bag[11:19]}:{self.run_id}:{self.role}"

    @property
    def duration_s(self) -> float:
        return self.t_to_s - self.t_from_s


def run_used(row: Mapping[str, float]) -> bool:
    return float(row["duration_s"]) >= MIN_RUN_S and float(row["wheel_distance_m"]) >= MIN_WHEEL_M


def parts_of_run(bag: str, row: Mapping[str, float]) -> list[RunPart]:
    """The train, validation and (unless the bag is protected) development parts of one run."""
    if not run_used(row):
        return []
    start, end = float(row["start_s"]), float(row["end_s"])
    dur = end - start
    run_id = str(row["run_id"])
    if bag in PROTECTED_BAGS:
        fit_end = start + PROTECTED_TRAIN_SHARE * dur
        dev = None
    else:
        dev_start = end - DEV_SHARE * dur
        fit_end = dev_start - GUARD_S
        dev = RunPart(bag, run_id, DEV, dev_start, end)
    if fit_end <= start:
        return []
    val_start = fit_end - VAL_SHARE * (fit_end - start)
    out = [
        RunPart(bag, run_id, TRAIN, start, val_start),
        RunPart(bag, run_id, VAL, val_start, fit_end),
    ]
    if dev is not None:
        out.append(dev)
    return out


def plan_parts(runs_by_bag: Mapping[str, Sequence[Mapping[str, float]]]) -> list[RunPart]:
    """Every part of every bag, in bag and run order."""
    out: list[RunPart] = []
    for bag in sorted(runs_by_bag):
        for row in runs_by_bag[bag]:
            out.extend(parts_of_run(bag, row))
    return out


def check_roles(parts: Sequence[RunPart]) -> None:
    """Refuse a development part of a protected bag, a part of a protected bag past its train
    share, or overlapping parts of one run."""
    by_run: dict[tuple[str, str], list[RunPart]] = {}
    for p in parts:
        if p.role not in ROLES:
            raise ValueError(f"{p.name}: unknown role {p.role!r}")
        if p.bag in PROTECTED_BAGS and p.role == DEV:
            raise RoleRefused(f"{p.name}: the GP2g-R held-out bag has no development part")
        by_run.setdefault((p.bag, p.run_id), []).append(p)
    for key, ps in by_run.items():
        ps = sorted(ps, key=lambda q: q.t_from_s)
        for a, b in pairwise(ps):
            if b.t_from_s < a.t_to_s - 1e-9:
                raise ValueError(f"{key}: parts {a.role} and {b.role} overlap")


def split_document(parts: Sequence[RunPart], first_scan: Mapping[str, float]) -> str:
    """The roles as a byte-stable yaml text (written by hand, not by a dumper)."""
    lines = [
        f"model_id: {MODEL_ID}",
        "kind: development split (unregistered, owner decision 29)",
        (
            f"rule: {{min_run_s: {MIN_RUN_S}, min_wheel_m: {MIN_WHEEL_M}, dev_share: {DEV_SHARE}, "
            f"guard_s: {GUARD_S}, val_share: {VAL_SHARE}, protected_train_share: "
            f"{PROTECTED_TRAIN_SHARE}}}"
        ),
        "protected_bags: [" + ", ".join(sorted(PROTECTED_BAGS)) + "]",
        "bags:",
    ]
    for bag in sorted({p.bag for p in parts}):
        lines.append(f"  {bag}:")
        lines.append(f"    first_scan_log_time_s: {first_scan[bag]:.6f}")
        lines.append("    parts:")
        for p in (q for q in parts if q.bag == bag):
            lines.append(
                f"      - {{run: {p.run_id}, role: {p.role}, t_from_s: {p.t_from_s:.3f}, "
                f"t_to_s: {p.t_to_s:.3f}}}"
            )
    return "\n".join(lines) + "\n"


def ratio(candidate: float, baseline: float) -> float:
    return float(candidate / baseline) if baseline > 0 else float("nan")


def summarise(
    errs: Mapping[str, Mapping[float, Mapping[str, np.ndarray]]],
    labels: np.ndarray,
    groups: Mapping[str, np.ndarray],
    baseline: str,
    horizon_s: float = 1.0,
) -> list[dict[str, object]]:
    """Rows of macro P, Y and S at `horizon_s` per model and group (macro over `labels`, the
    strata), with the ratio to `baseline`. `groups` maps a group name to a window mask."""
    from .strata import Strata, macro_rmse

    rows: list[dict[str, object]] = []
    for gname, mask in groups.items():
        lab = np.asarray(labels)[mask]
        names = sorted(set(lab.tolist()))
        strata = Strata(
            label=lab,
            names=tuple(names),
            counts={n: int(np.sum(lab == n)) for n in names},
            unmerged_small=(),
        )
        base = {
            k: macro_rmse(errs[baseline][horizon_s][f][mask], strata)
            for k, f in (("P", "pos"), ("Y", "yaw"), ("S", "speed"))
        }
        for model, e in errs.items():
            vals = {
                k: macro_rmse(e[horizon_s][f][mask], strata)
                for k, f in (("P", "pos"), ("Y", "yaw"), ("S", "speed"))
            }
            rows.append(
                {
                    "group": gname,
                    "model": model,
                    "windows": int(mask.sum()),
                    **{f"{k}1": round(v, 5) for k, v in vals.items()},
                    **{f"{k}1_ratio": round(ratio(vals[k], base[k]), 4) for k in vals},
                }
            )
    return rows


def render_rows(rows: Sequence[Mapping[str, object]]) -> str:
    head = "| group | model | windows | P1 m | ratio | Y1 rad | ratio | S1 m/s | ratio |"
    lines = [head, "|" + "---|" * 9]
    for r in rows:
        lines.append(
            f"| {r['group']} | {r['model']} | {r['windows']} | {r['P1']} | {r['P1_ratio']} | "
            f"{r['Y1']} | {r['Y1_ratio']} | {r['S1']} | {r['S1_ratio']} |"
        )
    return "\n".join(lines)
