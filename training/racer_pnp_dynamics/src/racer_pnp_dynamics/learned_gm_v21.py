"""TL5, the bounded learned correction on top of a frozen T5 (pre-registration v0.5 sections 2 and
3): the v0.4 learned fit of `learned_gm` with the yaw gain table of contract 2.1 folded in.

The wheel channel is the base nominal's and never sees the network, so `v`, `delta` and the commands
of every window step are computed once. Contract 2.1 multiplies the yaw rate by `(1 + gamma)` with
`gamma` a function of those same `v` and `delta`, so it does not depend on the network either:
`build_tracks_v21` folds it into the yaw rate of the tracks, and everything else of `learned_gm`
(the numpy twin, the torch loss, the trainer) applies unchanged:

    dyaw_i = (1 + rho_i) * (1 + gamma_i) * r_i * dt          r_i = v tan(delta) / L / (1 + u v^2)

Inputs, bounds and layers are those of contract 2.0 (`d_alpha_max`, `rho_max`, 4-32-32-2 tanh).
Two configurations (weight decay) and seeds 0, 1 and 2 are trained; the configuration and the
shipping seed are chosen by P1 on the SELECTION set (the 2026-11-14 held-out segments), never by a
2026-11-15 number. Early stopping uses the internal validation part of the train segments.
"""

from __future__ import annotations

from collections.abc import Sequence
from dataclasses import replace

import numpy as np

from . import learned_gm
from .evaluate import EvalSet
from .fit_data_v04 import FitWindows
from .ground_motion import GroundTable, table_alpha
from .ground_motion_v21 import ConstantsV05, GroundMotionV21
from .learned_gm import GmCandidate, GmConfig, Tracks
from .model import Nominal
from .table_fit_v05 import T5Fit, score
from .vehicle import Vehicle


def configs_of(c: ConstantsV05) -> tuple[GmConfig, ...]:
    """The two configurations of the yaml (`learned_fit.configs`), named by their weight decay."""
    return tuple(GmConfig(f"wd{wd:.0e}", wd) for wd in c.weight_decays)


def build_tracks_v21(
    fw: FitWindows,
    nominal: Nominal,
    vehicle: Vehicle,
    table: GroundTable,
    yaw_table: GroundTable,
    c: ConstantsV05,
) -> Tracks:
    """The tracks of `learned_gm.build_tracks` with `(1 + gamma)` folded into the yaw rate and the
    table alpha clamped to its bounds first, as the contract's step does."""
    tr = learned_gm.build_tracks(fw, nominal, vehicle, table)
    gamma = np.clip(table_alpha(yaw_table, tr.delta, tr.v, vehicle), c.gamma_min, c.gamma_max)
    return replace(
        tr,
        rate=tr.rate * (1.0 + gamma),
        table_alpha=np.clip(tr.table_alpha, c.alpha_min, c.alpha_max),
    )


def fit_tl5(
    train: FitWindows,
    val: FitWindows,
    selection: EvalSet,
    base: Nominal,
    vehicle: Vehicle,
    c: ConstantsV05,
    t5: T5Fit,
    configs: Sequence[GmConfig] | None = None,
    seeds: Sequence[int] | None = None,
    trainer=None,
) -> tuple[list[GmCandidate], GmCandidate, dict[str, dict[str, float]]]:
    """All (config, seed) candidates on the frozen T5 tables, scored on the selection set. The
    returned candidates carry the selection P1 in `val_p1_m`; the shipping one is the config with
    the lowest mean selection P1 over its seeds, then its best seed (`learned_gm.select`)."""
    trainer = trainer or learned_gm.train_gm_torch
    tr = build_tracks_v21(train, base, vehicle, t5.alpha, t5.gamma, c)
    va = build_tracks_v21(val, base, vehicle, t5.alpha, t5.gamma, c)
    candidates: list[GmCandidate] = []
    for cfg in configs if configs is not None else configs_of(c):
        for seed in seeds if seeds is not None else c.seeds:
            learned, epochs = trainer(cfg, seed, c, tr, va, train.scales)
            gm = GroundMotionV21(c, t5.alpha, t5.gamma, learned, t5.supported)
            p1 = score(selection, base, vehicle, gm).p1
            candidates.append(GmCandidate(cfg, seed, learned, p1, epochs))
    return (candidates, *learned_gm.select(candidates))
