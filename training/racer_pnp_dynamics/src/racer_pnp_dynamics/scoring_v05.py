"""Model specs and window metrics shared by M-B6 and the held-out pass (brief 47).

A `Spec` is one model to score: contract 1 (a nominal and maybe a residual), contract 2 (a ground
motion table) or contract 2.1 (alpha and gamma tables, maybe the learned block). `errors` gives the
per window errors at the registered horizons for any of them on the windows of an `EvalSet`.
Nothing here fits or tunes anything.
"""

from __future__ import annotations

import hashlib
import pickle
from collections.abc import Mapping, Sequence
from dataclasses import dataclass
from pathlib import Path

import numpy as np

from . import pose_check
from .evaluate import HORIZONS_S, EvalSet
from .ground_motion import GroundMotion, contract1_window_errors, window_errors_v2
from .ground_motion_v21 import GroundMotionV21, window_errors_v21
from .model import Nominal, Residual
from .strata import Strata, assign_strata, macro_rmse, stratum_rmse
from .vehicle import Vehicle

V01 = "dyn-v0.1"


@dataclass(frozen=True)
class Spec:
    """One model: contract 1 (`ground` and `ground21` None), contract 2 or contract 2.1."""

    nominal: Nominal
    residual: Residual | None = None
    ground: GroundMotion | None = None
    ground21: GroundMotionV21 | None = None

    def errors(self, es: EvalSet, vehicle: Vehicle) -> dict[float, dict[str, np.ndarray]]:
        if self.ground21 is not None:
            return window_errors_v21(es, self.nominal, vehicle, self.ground21, HORIZONS_S)
        if self.ground is not None:
            return window_errors_v2(es, self.nominal, vehicle, self.ground, HORIZONS_S)
        return contract1_window_errors(es, self.nominal, vehicle, self.residual, HORIZONS_S)

    def pose_model(self, vehicle: Vehicle) -> pose_check.YawModel:
        return pose_check.YawModel(
            self.nominal, self.residual, vehicle, self.ground21 or self.ground
        )


def macro_at(errs: Mapping[str, np.ndarray], strata: Strata) -> dict[str, float]:
    """P1, A1, C1, Y1, S1: the macro RMSE at 1.0 s of position, along, across, yaw, speed."""
    return {
        "P1": macro_rmse(errs["pos"], strata),
        "A1": macro_rmse(errs["along"], strata),
        "C1": macro_rmse(errs["across"], strata),
        "Y1": macro_rmse(errs["yaw"], strata),
        "S1": macro_rmse(errs["speed"], strata),
    }


def direction_macro(
    errs: Mapping[str, np.ndarray], strata: Strata, direction: str
) -> dict[str, float]:
    """P1 and Y1 of one direction by itself: the macro over that direction's strata."""
    names = [n for n in strata.names if n.startswith(direction + ":") and strata.counts[n] > 0]
    out = {}
    for key, field in (("P1", "pos"), ("Y1", "yaw")):
        per = stratum_rmse(errs[field], strata)
        out[key] = float(np.mean([per[n] for n in names])) if names else float("nan")
    return out


def rmse(x: np.ndarray) -> float:
    return float(np.sqrt(np.mean(np.asarray(x) ** 2))) if len(x) else float("nan")


def score_models(
    es: EvalSet, models: Mapping[str, Spec], vehicle: Vehicle
) -> tuple[Strata, dict[str, dict[float, dict[str, np.ndarray]]]]:
    strata = assign_strata(es.direction, es.start_speed)
    return strata, {n: m.errors(es, vehicle) for n, m in models.items()}


def sha256_file(path: Path) -> str:
    return hashlib.sha256(Path(path).read_bytes()).hexdigest()


def frozen_v04_specs(
    cache: Path, v01: Nominal, models: Sequence[str] = ("dyn-v0.1", "N4", "R4", "T4", "TL4")
) -> tuple[dict[str, Spec], dict[str, str]]:  # pragma: no cover - real data (the v0.4 pickles)
    """The frozen v0.4 candidates of the pass of PR 193 (freeze commit 416e685): dyn-v0.1, N4, and
    on dyn-v0.1 (the shipping nominal) R4 at its shipping seed, T4 and TL4 at its shipping seed.
    Returns the specs and the SHA-256 of every pickle read."""
    prefix = "dyn-v0.4-"
    files = {
        "n4": cache / f"{prefix}n4.pkl",
        "r4": cache / f"{prefix}r4-dynv01.pkl",
        "t4": cache / f"{prefix}t4-dynv01.pkl",
        "tl4": cache / f"{prefix}tl4-dynv01.pkl",
    }
    hashes = {k: sha256_file(p) for k, p in files.items()}
    n4 = pickle.loads(files["n4"].read_bytes())
    r4 = pickle.loads(files["r4"].read_bytes())
    t4 = pickle.loads(files["t4"].read_bytes())
    tl4 = pickle.loads(files["tl4"].read_bytes())
    ship_r = next(
        c.residual
        for c in r4.candidates
        if c.config.name == r4.shipping.config.name and c.seed == r4.shipping.seed
    )
    ship = tl4["shipping"]
    ship_gm = GroundMotion(t4.constants, t4.table, ship.learned)
    out = {
        "dyn-v0.1": Spec(v01),
        "N4": Spec(n4.nominal),
        "R4": Spec(v01, ship_r),
        "T4": Spec(v01, None, t4.groundmotion()),
        "TL4": Spec(v01, None, ship_gm),
    }
    return {k: out[k] for k in models}, hashes


def summarise(
    es: EvalSet,
    strata: Strata,
    errs: Mapping[str, Mapping[float, Mapping[str, np.ndarray]]],
    reference: str = V01,
) -> dict:
    """The tables of a measurement printed for every model on identical windows: macro P1, A1, C1,
    Y1, S1 and their ratio to `reference`, P1 and Y1 per direction, the pooled position RMSE at
    1.0 s per run (the window's segment name), the window counts."""
    names = list(errs)
    macro = {n: macro_at(errs[n][1.0], strata) for n in names}
    per_dir = {
        n: {d: direction_macro(errs[n][1.0], strata, d) for d in sorted(set(es.direction.tolist()))}
        for n in names
    }
    runs = sorted(set(es.segment.tolist()))
    per_run = {
        r: {
            "windows": int((es.segment == r).sum()),
            **{n: rmse(errs[n][1.0]["pos"][es.segment == r]) for n in names},
        }
        for r in runs
    }
    ref = macro.get(reference)
    ratio = {
        n: {k: (macro[n][k] / ref[k] if ref and ref[k] > 0 else float("nan")) for k in macro[n]}
        for n in names
    }
    return {
        "windows": len(es),
        "strata": dict(strata.counts),
        "merges": list(strata.merges),
        "directions": {d: int((es.direction == d).sum()) for d in sorted(set(es.direction))},
        "macro": macro,
        "ratio": ratio,
        "per_direction": per_dir,
        "per_run": per_run,
    }
