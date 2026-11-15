"""The ONE held-out pass of v0.4: GP2d and GP2e together (brief 37b section 6).

Everything here takes frozen models and the held-out segments of 2026-11-14 under T2 and under T2s
and prints, on IDENTICAL windows (a window is in the pass only if it exists under both truths),
every metric of every model: P1, A1, C1, Y1, S1 at 1.0 s (P, Y and S at 0.5 and 1.5 s too), per
stratum, per seed, bootstrap ratios, P1 under T2s, Pp and Yp on G1b and T2b, the fast item per
direction. Then the decisions, in the order the pre-registrations give:

1. GP2d-N: N4 against dyn-v0.1. It decides which nominal ships (N4 if it passed, else dyn-v0.1).
2. GP2d-R: R4 on the shipping nominal against that nominal, the heading outcome (a label, no
   deployment flag), and the validity of the bundle.
3. GP2e-T: T4 on the shipping nominal against that nominal; GP2e-L: TL4 against T4, only if GP2e-T
   passed; the outcome name.

The candidates on the other base and the v0.3 candidates are scored in the same pass and decide
nothing. Nothing in this module fits or tunes anything; it cannot, it only receives frozen models.
"""

from __future__ import annotations

from collections.abc import Mapping, Sequence
from dataclasses import dataclass, field
from typing import Any

import numpy as np

from . import gp2, pose_check
from .dataset import Segment
from .evaluate import HORIZONS_S, EvalSet, eval_set
from .ground_motion import (
    GroundMotion,
    contract1_window_errors,
    window_errors_v2,
)
from .ground_sweep import sweep_v2
from .model import Nominal, Residual
from .strata import (
    Strata,
    assign_strata,
    block_bootstrap_ratio,
    macro_rmse,
    stratum_rmse,
)
from .vehicle import Vehicle

V01 = "dyn-v0.1"
N4 = "N4"
BASES = (V01, N4)
SEEDS = (0, 1, 2)


@dataclass(frozen=True)
class ModelSpec:
    """One model to score: contract 1 (`ground` None, an optional residual) or contract 2."""

    nominal: Nominal
    residual: Residual | None = None
    ground: GroundMotion | None = None

    def errors(self, es: EvalSet, vehicle: Vehicle) -> dict[float, dict[str, np.ndarray]]:
        if self.ground is not None:
            return window_errors_v2(es, self.nominal, vehicle, self.ground, HORIZONS_S)
        return contract1_window_errors(es, self.nominal, vehicle, self.residual, HORIZONS_S)

    def pose_model(self, vehicle: Vehicle) -> pose_check.YawModel:
        return pose_check.YawModel(self.nominal, self.residual, vehicle, self.ground)


def window_keys(es: EvalSet) -> list[tuple[str, float]]:
    return [(str(s), round(float(t), 3)) for s, t in zip(es.segment, es.t_s, strict=True)]


def align(es_a: EvalSet, es_b: EvalSet) -> tuple[EvalSet, EvalSet]:
    """The windows both sets hold, in the same order (sorted by segment and start)."""
    ka, kb = window_keys(es_a), window_keys(es_b)
    common = sorted(set(ka) & set(kb))
    pos_a = {k: i for i, k in enumerate(ka)}
    pos_b = {k: i for i, k in enumerate(kb)}
    ia = np.array([pos_a[k] for k in common], dtype=int)
    ib = np.array([pos_b[k] for k in common], dtype=int)
    return es_a.subset(ia), es_b.subset(ib)


@dataclass
class Pass:
    """Everything printed by the pass (JSON-able through `to_dict`)."""

    windows: int
    windows_t2_only: int
    windows_t2s_only: int
    strata: Strata
    block: np.ndarray
    start_speed: np.ndarray
    direction: np.ndarray
    errs: dict[str, dict[float, dict[str, np.ndarray]]]  # model -> horizon -> field -> (E,)
    errs_alt: dict[str, np.ndarray]  # model -> position error at 1.0 s under T2s
    macro: dict[str, dict[float, dict[str, float]]]  # model -> horizon -> P, Y, S (A, C at 1.0)
    stratum_pos: dict[str, dict[str, float]]  # model -> stratum -> RMSE of position at 1.0 s
    p1_alt: dict[str, float]
    pp: dict[str, float]
    yp: dict[str, float]
    pp_segments: dict[str, dict[str, dict[str, float]]]
    yp_segments: dict[str, dict[str, dict[str, float]]]
    fast: dict[str, list[gp2.FastWindows]]  # model -> per direction (vs dyn-v0.1)
    names: list[str] = field(default_factory=list)

    def at1(self, name: str) -> dict[str, float]:
        m = self.macro[name][1.0]
        return {"P1": m["P"], "A1": m["A"], "C1": m["C"], "Y1": m["Y"], "S1": m["S"]}

    def bootstrap(self, a: str, b: str, field_name: str = "pos") -> dict[str, float]:
        """Paired block bootstrap of the macro ratio of `field_name` RMSE at 1.0 s, a over b."""
        return block_bootstrap_ratio(
            self.errs[a][1.0][field_name], self.errs[b][1.0][field_name], self.strata, self.block
        )

    def to_dict(self) -> dict[str, Any]:
        return {
            "windows": self.windows,
            "windows_only_under_T2": self.windows_t2_only,
            "windows_only_under_T2s": self.windows_t2s_only,
            "strata": {"names": list(self.strata.names), "windows": self.strata.counts,
                       "merges": list(self.strata.merges)},
            "macro": {n: {str(h): v for h, v in m.items()} for n, m in self.macro.items()},
            "per_stratum_position_rmse_1s_m": self.stratum_pos,
            "p1_under_T2s": self.p1_alt,
            "pp": self.pp,
            "yp": self.yp,
            "pp_segments": self.pp_segments,
            "yp_segments": self.yp_segments,
            "fast": {
                n: [f.__dict__ for f in fs] for n, fs in self.fast.items()
            },
        }  # fmt: skip


def _macro(errs: Mapping[float, Mapping[str, np.ndarray]], strata: Strata) -> dict:
    return {
        h: {
            "P": macro_rmse(e["pos"], strata),
            "A": macro_rmse(e["along"], strata),
            "C": macro_rmse(e["across"], strata),
            "Y": macro_rmse(e["yaw"], strata),
            "S": macro_rmse(e["speed"], strata),
        }
        for h, e in errs.items()
    }


def _rmse(x: np.ndarray) -> float:
    return float(np.sqrt(np.mean(x**2))) if len(x) else float("nan")


def score_pass(
    held_t2: Sequence[Segment],
    held_t2s: Sequence[Segment],
    vehicle: Vehicle,
    models: Mapping[str, ModelSpec],
    pose_segments: Sequence[pose_check.PoseSegment] = (),
    pair_segments: Sequence[pose_check.PosePairSegment] = (),
    fast_reference: str = V01,
) -> Pass:
    """Score every model of `models` on the aligned windows of the held-out segments."""
    es2_all = eval_set([s for s in held_t2 if s.run.has_odom])
    es2s_all = eval_set([s for s in held_t2s if s.run.has_odom])
    es2, es2s = align(es2_all, es2s_all)
    if len(es2) == 0:
        raise ValueError("no held-out windows in common between T2 and T2s")
    strata = assign_strata(es2.direction, es2.start_speed)
    errs = {n: m.errors(es2, vehicle) for n, m in models.items()}
    alt = {n: m.errors(es2s, vehicle)[1.0]["pos"] for n, m in models.items()}
    macro = {n: _macro(e, strata) for n, e in errs.items()}
    stratum_pos = {n: stratum_rmse(e[1.0]["pos"], strata) for n, e in errs.items()}
    p1_alt = {n: macro_rmse(a, strata) for n, a in alt.items()}
    yp_seg = {
        n: pose_check.yp_by_segment(m.pose_model(vehicle), pose_segments) for n, m in models.items()
    }
    pp_seg = {
        n: pose_check.pp_by_segment(m.pose_model(vehicle), pair_segments) for n, m in models.items()
    }
    yp = {n: pose_check.yp_macro(m.pose_model(vehicle), pose_segments) for n, m in models.items()}
    pp = {n: pose_check.pp_macro(m.pose_model(vehicle), pair_segments) for n, m in models.items()}
    fast: dict[str, list[gp2.FastWindows]] = {}
    for n in models:
        rows = []
        for d in ("ccw", "cw"):
            sel = (es2.direction == d) & (es2.start_speed >= gp2.FAST_MIN_START_SPEED_MPS)
            rows.append(
                gp2.FastWindows(
                    d,
                    int(sel.sum()),
                    _rmse(errs[n][1.0]["pos"][sel]),
                    _rmse(errs[fast_reference][1.0]["pos"][sel])
                    if fast_reference in errs
                    else float("nan"),
                )
            )
        fast[n] = rows
    keys2, keys2s = set(window_keys(es2_all)), set(window_keys(es2s_all))
    return Pass(
        windows=len(es2),
        windows_t2_only=len(keys2 - keys2s),
        windows_t2s_only=len(keys2s - keys2),
        strata=strata,
        block=es2.block,
        start_speed=es2.start_speed,
        direction=es2.direction,
        errs=errs,
        errs_alt=alt,
        macro=macro,
        stratum_pos=stratum_pos,
        p1_alt=p1_alt,
        pp=pp,
        yp=yp,
        pp_segments=pp_seg,
        yp_segments=yp_seg,
        fast=fast,
        names=list(models),
    )


@dataclass
class Frozen:
    """The frozen candidates the pass receives (built by the fit stages, pickled at the freeze)."""

    v01: Nominal
    n4: Nominal
    n4_on_bound: list[str]
    r4: dict[
        str, dict[int, Residual]
    ]  # base -> seed -> residual (all seeds of the shipping config)
    r4_shipping_seed: dict[str, int]
    t4: dict[str, GroundMotion]  # base -> table model
    tl4: dict[str, dict[int, GroundMotion]]  # base -> seed -> table plus learned
    tl4_shipping_seed: dict[str, int]
    v_max_rule_mps: float
    v03: dict[str, tuple[Nominal, Residual | None]] = field(default_factory=dict)
    t4_on_bound: dict[str, list[str]] = field(
        default_factory=dict
    )  # base -> supported nodes on a bound

    def nominal_of(self, base: str) -> Nominal:
        return self.v01 if base == V01 else self.n4

    def models(self) -> dict[str, ModelSpec]:
        out: dict[str, ModelSpec] = {V01: ModelSpec(self.v01), N4: ModelSpec(self.n4)}
        for base in BASES:
            nom = self.nominal_of(base)
            for seed, res in sorted(self.r4.get(base, {}).items()):
                out[f"R4[{base}] seed {seed}"] = ModelSpec(nom, res)
            out[f"T4[{base}]"] = self.t4_spec(base)
            for seed, gm in sorted(self.tl4.get(base, {}).items()):
                out[f"TL4[{base}] seed {seed}"] = ModelSpec(nom, None, gm)
        for name, (nom, res) in self.v03.items():
            out[name] = ModelSpec(nom, res)
        return out

    def t4_spec(self, base: str) -> ModelSpec:
        return ModelSpec(self.nominal_of(base), None, self.t4[base])


def decide(p: Pass, frozen: Frozen, vehicle: Vehicle, v_max_for_sweep: float) -> dict[str, Any]:
    """All decisions of the pass. `v_max_for_sweep` is the validity the sweeps assume."""
    out: dict[str, Any] = {}
    n_dec = gp2.decide_gp2d_nominal(
        p.at1(V01),
        p.at1(N4),
        p.bootstrap(N4, V01)["hi"],
        p.stratum_pos[V01],
        p.stratum_pos[N4],
        frozen.n4_on_bound,
        p.yp[V01],
        p.yp[N4],
    )
    ship = N4 if n_dec.passed else V01
    out["GP2d_N"] = n_dec.to_dict()
    out["shipping_nominal"] = ship
    out["bootstrap_N4_over_v01"] = p.bootstrap(N4, V01)

    # R4 on the shipping nominal
    seed_names = {s: f"R4[{ship}] seed {s}" for s in sorted(frozen.r4.get(ship, {}))}
    ship_r = seed_names[frozen.r4_shipping_seed[ship]]
    r_dec = gp2.decide_gp2d_residual(
        p.at1(ship),
        p.at1(ship_r),
        p.bootstrap(ship_r, ship)["hi"],
        {s: p.at1(n)["P1"] for s, n in seed_names.items()},
        p.stratum_pos[ship],
        p.stratum_pos[ship_r],
        gp2.stability_sweep(
            frozen.nominal_of(ship),
            frozen.r4[ship][frozen.r4_shipping_seed[ship]],
            vehicle,
            v_max_for_sweep,
        ),
        p.yp[ship],
        p.yp[ship_r],
    )
    lat = p.bootstrap(ship_r, ship, "across")
    heading = gp2.heading_outcome(
        p.at1(ship),
        p.at1(ship_r),
        lat["ratio"],
        lat["hi"],
        p.stratum_pos[ship],
        p.stratum_pos[ship_r],
    )
    out.update(
        GP2d_R=r_dec.to_dict(),
        heading_outcome=heading.to_dict(),
        bootstrap_R4_over_nominal=p.bootstrap(ship_r, ship),
        lateral_bootstrap_R4_over_nominal=lat,
        r4_shipping=ship_r,
    )
    # validity of the contract 1 bundle that would ship: N4 if it passed GP2d-N, and the residual
    # on the shipping nominal if it passed GP2d-R (its gate), else the held value
    bundle_model = ship_r if r_dec.passed else ship
    out["validity"] = gp2.decide_validity(
        frozen.v_max_rule_mps, r_dec.passed or ship == N4, p.fast[bundle_model]
    ).to_dict()
    out["validity_bundle_model"] = bundle_model

    # GP2e
    t4 = f"T4[{ship}]"
    t_dec = gp2.decide_gp2e_table(
        p.at1(ship),
        p.at1(t4),
        p.bootstrap(t4, ship)["hi"],
        p.stratum_pos[ship],
        p.stratum_pos[t4],
        frozen.t4_on_bound.get(ship, []),
        p.p1_alt[ship],
        p.p1_alt[t4],
        p.pp[ship],
        p.pp[t4],
    )
    tl_seeds = {s: f"TL4[{ship}] seed {s}" for s in sorted(frozen.tl4.get(ship, {}))}
    ship_tl = tl_seeds[frozen.tl4_shipping_seed[ship]]
    if t_dec.passed:
        l_dec = gp2.decide_gp2e_learned(
            True,
            p.at1(t4),
            p.at1(ship_tl),
            p.bootstrap(ship_tl, t4)["hi"],
            {s: p.at1(n)["P1"] for s, n in tl_seeds.items()},
            p.stratum_pos[t4],
            p.stratum_pos[ship_tl],
            sweep_v2(
                frozen.nominal_of(ship),
                vehicle,
                frozen.tl4[ship][frozen.tl4_shipping_seed[ship]],
                v_max_for_sweep,
            ),
            p.p1_alt[t4],
            p.p1_alt[ship_tl],
            p.pp[t4],
            p.pp[ship_tl],
            p.yp[t4],
            p.yp[ship_tl],
        )
    else:
        l_dec = gp2.decide_gp2e_learned(False)
    out["validity_T4"] = gp2.decide_validity(
        frozen.v_max_rule_mps, t_dec.passed, p.fast[t4]
    ).to_dict()
    out["validity_TL4"] = gp2.decide_validity(
        frozen.v_max_rule_mps, l_dec.passed, p.fast[ship_tl]
    ).to_dict()
    out.update(
        GP2e_T=t_dec.to_dict(),
        GP2e_L=l_dec.to_dict(),
        gp2e_outcome=gp2.gp2e_outcome(t_dec, l_dec),
        bootstrap_T4_over_nominal=p.bootstrap(t4, ship),
        tl4_shipping=ship_tl,
    )
    return out


def comparator_table(p: Pass, names: Sequence[str]) -> list[dict[str, object]]:
    """The comparator table: every model with P1, A1, C1, Y1, S1, P1 under T2s, Pp, Yp."""
    rows = []
    for n in names:
        a = p.at1(n)
        rows.append(
            {
                "model": n,
                **a,
                "P1_T2s": p.p1_alt[n],
                "Pp": p.pp[n],
                "Yp": p.yp[n],
            }
        )
    return rows


def render_comparator(rows: Sequence[Mapping[str, object]]) -> str:
    lines = [
        "| model | P1 (m) | A1 (m) | C1 (m) | Y1 (rad) | S1 (m/s) | P1 under T2s (m) | Pp (m) | Yp (rad) |",
        "|---|---|---|---|---|---|---|---|---|",
    ]
    for r in rows:
        lines.append(
            f"| {r['model']} | {r['P1']:.4f} | {r['A1']:.4f} | {r['C1']:.4f} | {r['Y1']:.4f} | "
            f"{r['S1']:.4f} | {r['P1_T2s']:.4f} | {r['Pp']:.4f} | {r['Yp']:.4f} |"
        )
    return "\n".join(lines) + "\n"
