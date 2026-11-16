"""The v0.5 stages (real-bag glue, brief 47). Stage 1 commands fit on TRAIN data only.

    mb5     M-B5: the T5 method on the data that exists before the 2026-11-15 session

The roles are those of `evaluation/prereg/pnp-v0.5-dynamics.yaml` `data_roles`. The 2026-11-14
held-out segments are the SELECTION set: they are read here, only through `selection_eval_set`,
only to choose a smoothness weight and to print P1, A1 and Y1; no fit function accepts them
(`table_fit_v05.RoleViolation`). Nothing in this module opens a 2026-11-15 held-out part.
"""

from __future__ import annotations

import logging
import pickle
from pathlib import Path

from . import ground_motion_v21, table_fit_v05, truth_v04
from .cli_v04 import OUT_DIR as V04_OUT_DIR  # noqa: F401  (documents where the v0.4 numbers are)
from .cli_v04 import Stage1, _find, segments_of_v04_bags
from .evaluate import EvalSet, eval_set

LOG = logging.getLogger(__name__)

# The freeze commit of the v0.4 held-out pass (PR 193). The 2026-11-14 held-out segments were read
# there; their truth cache is read here, as the SELECTION set, with this token.
V04_FREEZE_SHA = "416e68501c424caeb1f6dc80b6a0ee9edc7a288c"


def out_dir(root: Path, date: str) -> Path:  # pragma: no cover - real checkout
    return root / "training" / "pnp_models" / f"dyn-v0.5-{date}"


def constants() -> ground_motion_v21.ConstantsV05:  # pragma: no cover - real checkout
    return ground_motion_v21.load_constants_v05(_find(ground_motion_v21.V05_RELATIVE))


def selection_eval_set(args, ctx: Stage1) -> EvalSet:  # pragma: no cover - real data
    """The selection set: the 2026-11-14 held-out segments (K1, K2, X2b, X3b, X4b, X5b, X6b, X7b)
    under T2 as 1.5 s windows. Never fitted on; the only use is choosing settings and printing."""
    token = truth_v04.freeze(V04_FREEZE_SHA)
    segs = segments_of_v04_bags(args, ctx.v04, ctx.vehicle, "heldout", token)
    return eval_set([s for s in segs if s.run.has_odom])


def mb5(args) -> int:  # pragma: no cover - real data
    """M-B5. Train without 2026-11-15; selection on the 2026-11-14 held-out segments."""
    from .report_v05 import render_mb5

    ctx = Stage1(args)
    c = constants()
    v01 = ctx.v01().nominal
    win = table_fit_v05.build_t5_windows(ctx.train, v01, ctx.fvehicle, c)
    sel = selection_eval_set(args, ctx)
    LOG.info("M-B5: %d train windows, %d selection windows", len(win), len(sel))
    fit = table_fit_v05.fit_t5(win, sel, v01, ctx.fvehicle, c)
    gm = fit.groundmotion()
    base = table_fit_v05.score(sel, v01, ctx.fvehicle, table_fit_v05.unit_motion(c))
    mine = table_fit_v05.score(sel, v01, ctx.fvehicle, gm)
    (args.cache / "dyn-v0.5-mb5.pkl").write_bytes(
        pickle.dumps({"fit": fit, "t5": mine, "v01": base})
    )
    gs = c.doc["gate_support"]
    notes = [
        f"Constants file evaluation/prereg/pnp-v0.5-dynamics.yaml, SHA-256 {c.sha256}.",
        (
            f"Train windows: {len(win)} of 0.5 s (start every 0.25 s, no wheel speed sample under "
            f"{c.min_wheel_speed_mps:g} m/s) from the v0.2 split train segments, the legacy train "
            "part and the 2026-11-14 train segments (train and validation-of-train parts)."
        ),
        f"Selection windows: {len(sel)} (1.5 s windows of the 2026-11-14 held-out segments, T2).",
        (
            "Targets: ground distance = sum of the T2 ICP step lengths (the quantity M-B4 measures), "
            "yaw change = sum of the T2 yaw changes; inputs: the measured wheel speed and the "
            "nominal's steering state; base nominal dyn-v0.1."
        ),
    ]
    text = render_mb5(
        fit, mine, base,
        limit_alpha=float(gs["max_split_half_difference_alpha"]),
        limit_gamma=float(gs["max_split_half_difference_gamma"]),
        notes=notes,
    )  # fmt: skip
    folder = out_dir(ctx.root, args.date)
    folder.mkdir(parents=True, exist_ok=True)
    (folder / "M-B5.md").write_text(text, encoding="utf-8")
    print(text)
    return 0


STAGE_COMMANDS = {"mb5": mb5}
