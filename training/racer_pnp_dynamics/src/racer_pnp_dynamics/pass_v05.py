"""The ONE held-out pass of v0.5: the numbers of GP2f-S, GP2f-T, GP2f-L and GP2f-V (brief 47 step 6).

Everything here takes frozen models and the held-out parts of the 2026-11-15 session under T2 and
under T2s, and prints on IDENTICAL windows (a window is in the pass only if it exists under both
truths) every metric of every model: P1, A1, C1, Y1, S1 at 1.0 s, P1 under T2s, Pp and Yp, per
stratum, per direction (measured heading), the yaw rate bias per direction, near-limit windows per
side, transition windows, the fast windows per direction, and the bootstrap ratios. Then the
decisions in the order the pre-registration gives them. Nothing in this module fits or tunes
anything; it cannot, it only receives frozen models.
"""

from __future__ import annotations

from collections.abc import Mapping, Sequence
from dataclasses import dataclass, field
from typing import Any

import numpy as np

from . import gp2, gp2f, pose_check
from .dataset import Segment
from .evaluate import EvalSet, eval_set, predict
from .ground_motion import steering_share
from .ground_motion_v21 import ConstantsV05, GroundMotionV21, predict_v21
from .ground_sweep_v21 import sweep_v21
from .model import DT_S, Nominal
from .pipeline_v04 import align
from .scoring_v05 import Spec, direction_macro, macro_at, rmse
from .session_v05 import with_heading_direction
from .strata import Strata, assign_strata, block_bootstrap_ratio, macro_rmse, stratum_rmse
from .table_fit_v05 import cell_of
from .vehicle import Vehicle
from .windows import WARM_N

ONE_SECOND_STEPS = round(1.0 / DT_S)  # the 1.0 s horizon in model steps


@dataclass
class Pass05:
    """Everything printed by the pass."""

    es: EvalSet  # the aligned T2 windows with the heading direction
    strata: Strata
    errs: dict[str, dict[float, dict[str, np.ndarray]]]
    errs_alt: dict[str, np.ndarray]  # model -> position error at 1.0 s under T2s
    pp: dict[str, float]
    yp: dict[str, float]
    near_limit: dict[str, np.ndarray]  # side -> (E,) bool
    transition: np.ndarray  # (E,) bool
    windows_t2_only: int
    windows_t2s_only: int
    names: list[str] = field(default_factory=list)

    def metrics(self, name: str, fast_speeds: Sequence[float]) -> gp2f.Metrics:
        e = self.errs[name][1.0]
        dirs = sorted(set(self.es.direction.tolist()))
        pos = e["pos"]
        return gp2f.Metrics(
            macro=macro_at(e, self.strata),
            stratum_pos=stratum_rmse(pos, self.strata),
            direction={d: direction_macro(e, self.strata, d) for d in dirs},
            yaw_bias={
                d: float(np.mean(e["yaw"][self.es.direction == d])) / 1.0
                if (self.es.direction == d).any()
                else float("nan")
                for d in gp2f.DIRECTIONS
            },
            near_limit={s: (int(m.sum()), rmse(pos[m])) for s, m in self.near_limit.items()},
            transition=(int(self.transition.sum()), rmse(pos[self.transition])),
            p1_alt=macro_rmse(self.errs_alt[name], self.strata),
            pp=self.pp[name],
            yp=self.yp[name],
            fast={
                v: {
                    d: (
                        int(((self.es.direction == d) & (self.es.start_speed >= v)).sum()),
                        rmse(pos[(self.es.direction == d) & (self.es.start_speed >= v)]),
                    )
                    for d in gp2f.DIRECTIONS
                }
                for v in fast_speeds
            },
        )

    def bootstrap(self, a: str, b: str, field_name: str = "pos") -> dict[str, float]:
        """Paired block bootstrap of the macro ratio of `field_name` RMSE at 1.0 s, a over b."""
        return block_bootstrap_ratio(
            self.errs[a][1.0][field_name], self.errs[b][1.0][field_name], self.strata, self.es.block
        )


def steering_masks(
    es: EvalSet, nominal: Nominal, vehicle: Vehicle, min_abs_share: float, min_change_mps: float
) -> tuple[dict[str, np.ndarray], np.ndarray]:
    """Near-limit windows (mean absolute steering share over the 1.0 s of the window at least
    `min_abs_share`, per side by the sign of the mean share) and transition windows (the speed
    command changes by at least `min_change_mps` inside the window). The steering state is the
    nominal's, so the sets do not depend on the model scored."""
    data = es.data(nominal.command_delay_s)
    states = predict(data, nominal, vehicle)
    share = steering_share(states[:, :ONE_SECOND_STEPS, 4], vehicle)
    near = np.abs(share).mean(axis=1) >= min_abs_share
    side = share.mean(axis=1)
    speed_cmd = data.cmds[:, WARM_N : WARM_N + ONE_SECOND_STEPS, 0]
    change = speed_cmd.max(axis=1) - speed_cmd.min(axis=1)
    return {"left": near & (side > 0.0), "right": near & (side <= 0.0)}, change >= min_change_mps


def score_pass(
    c: ConstantsV05,
    held_t2: Sequence[Segment],
    held_t2s: Sequence[Segment],
    vehicle: Vehicle,
    models: Mapping[str, Spec],
    nominal: Nominal,
    pose_segments: Sequence[pose_check.PoseSegment] = (),
    pair_segments: Sequence[pose_check.PosePairSegment] = (),
) -> Pass05:
    """Score every model of `models` on the aligned windows of the held-out parts."""
    es2_all = eval_set([s for s in held_t2 if s.run.has_odom])
    es2s_all = eval_set([s for s in held_t2s if s.run.has_odom])
    es2, es2s = align(es2_all, es2s_all)
    if len(es2) == 0:
        raise ValueError("no held-out windows in common between T2 and T2s")
    es2 = with_heading_direction(es2, held_t2)
    strata = assign_strata(es2.direction, es2.start_speed)
    errs = {n: m.errors(es2, vehicle) for n, m in models.items()}
    alt = {n: m.errors(es2s, vehicle)[1.0]["pos"] for n, m in models.items()}
    m = c.doc["metrics"]
    near, trans = steering_masks(
        es2,
        nominal,
        vehicle,
        float(m["near_limit"]["min_mean_abs_steering_share"]),
        float(m["transition"]["min_speed_command_change_mps"]),
    )
    yp = {n: pose_check.yp_macro(s.pose_model(vehicle), pose_segments) for n, s in models.items()}
    pp = {n: pose_check.pp_macro(s.pose_model(vehicle), pair_segments) for n, s in models.items()}
    keys2 = {(str(s), round(float(t), 3)) for s, t in zip(es2_all.segment, es2_all.t_s)}
    keys2s = {(str(s), round(float(t), 3)) for s, t in zip(es2s_all.segment, es2s_all.t_s)}
    return Pass05(
        es=es2,
        strata=strata,
        errs=errs,
        errs_alt=alt,
        pp=pp,
        yp=yp,
        near_limit=near,
        transition=trans,
        windows_t2_only=len(keys2 - keys2s),
        windows_t2s_only=len(keys2s - keys2),
        names=list(models),
    )


# ---- numbers that need a rollout --------------------------------------------------------------


def step_share_in_supported(
    c: ConstantsV05,
    gm: GroundMotionV21,
    es: EvalSet,
    nominal: Nominal,
    vehicle: Vehicle,
    supported: np.ndarray,
) -> float:
    """GP2f-S item 3: the share of the held-out rollout steps (every step of the 1.5 s rollouts)
    whose (steering share, wheel speed) falls in the cell of a supported node."""
    states = predict_v21(es, nominal, vehicle, gm)[:, :-1]
    share = steering_share(states[..., 4], vehicle)
    i, j = cell_of(c, share, states[..., 3])
    return float(np.mean(supported[i, j]))


def saturation_share(
    c: ConstantsV05,
    gm: GroundMotionV21,
    es: EvalSet,
    nominal: Nominal,
    vehicle: Vehicle,
    level: float,
) -> float:
    """GP2f-L item 7: the share of held-out rollout steps where the network's alpha or rho output
    (tanh of the raw output, as a share of its limit) is beyond `level` of its limit."""
    if gm.learned is None:
        return 0.0
    data = es.data(nominal.command_delay_s)
    states = predict_v21(es, nominal, vehicle, gm)[:, :-1]
    k = states.shape[1]
    ctrl = data.cmds[:, WARM_N : WARM_N + k, :]
    inputs = np.stack([states[..., 3], states[..., 4], ctrl[..., 0], ctrl[..., 1]], axis=-1)
    out = np.abs(np.tanh(gm.learned.forward(inputs)))
    return float(np.mean((out > level).any(axis=-1)))


# ---- the decisions -----------------------------------------------------------------------------


def decide_all(
    c: ConstantsV05,
    p: Pass05,
    *,
    t5: str,
    tl5_seeds: Mapping[int, str],
    tl5_shipping: str,
    v01: str,
    on_bound: Sequence[str],
    split_half_alpha_max: float,
    split_half_gamma_max: float,
    t5_motion: GroundMotionV21,
    tl5_motion: GroundMotionV21,
    nominal: Nominal,
    vehicle: Vehicle,
    rule_mps: float,
    supported_alpha: np.ndarray,
    supported_gamma: np.ndarray,
) -> dict[str, Any]:
    """All decisions of the pass, in the order of the pre-registration."""
    speeds = [float(s["min_start_speed_mps"]) for s in c.doc["gate_validity"]["fast_items"]]
    m = {n: p.metrics(n, speeds) for n in p.names}
    out: dict[str, Any] = {}
    share = step_share_in_supported(c, t5_motion, p.es, nominal, vehicle, supported_alpha)
    support = gp2f.decide_gp2f_support(
        c, on_bound, split_half_alpha_max, split_half_gamma_max, share
    )
    table = gp2f.decide_gp2f_table(c, support.passed, m[v01], m[t5], p.bootstrap(t5, v01)["hi"])
    tl5 = tl5_shipping
    if table.passed:
        sweep = sweep_v21(
            nominal,
            vehicle,
            tl5_motion,
            float(c.doc["gate_validity"]["cap_mps"]),
            int(c.doc["gate_learned"]["sweep_samples"]),
        )
        learned = gp2f.decide_gp2f_learned(
            c,
            True,
            m[t5],
            m[tl5],
            p.bootstrap(tl5, t5)["hi"],
            {s: m[n].macro["P1"] for s, n in tl5_seeds.items()},
            sweep.violations,
            sweep.samples,
            saturation_share(
                c,
                tl5_motion,
                p.es,
                nominal,
                vehicle,
                float(c.doc["gate_learned"]["saturation"]["level"]),
            ),
        )
        out["sweep"] = sweep.to_dict()
    else:
        learned = gp2f.decide_gp2f_learned(c, False)
    top = gp2f.top_supported_speed(c, supported_alpha, supported_gamma)

    def fast_of(name: str) -> dict[float, dict[str, tuple[int, float, float]]]:
        return {
            v: {d: (*m[name].fast[v][d], m[v01].fast[v][d][1]) for d in gp2f.DIRECTIONS}
            for v in speeds
        }

    out.update(
        support=support,
        table=table,
        learned=learned,
        outcome=gp2f.gp2f_outcome(support, table, learned),
        validity_T5=gp2f.decide_gp2f_validity(c, table.passed, rule_mps, top, fast_of(t5)),
        validity_TL5=gp2f.decide_gp2f_validity(
            c, table.passed and learned.passed, rule_mps, top, fast_of(tl5)
        ),
        top_supported_speed=top,
        step_share_in_supported=share,
        bootstrap_T5_over_v01=p.bootstrap(t5, v01),
        bootstrap_TL5_over_T5=p.bootstrap(tl5, t5),
        metrics=m,
    )
    return out


def comparator_rows(p: Pass05, names: Sequence[str]) -> list[dict[str, object]]:
    """Every model with P1, A1, C1, Y1, S1, P1 under T2s, Pp, Yp."""
    rows = []
    for n in names:
        a = macro_at(p.errs[n][1.0], p.strata)
        rows.append(
            {
                "model": n,
                **a,
                "P1_T2s": macro_rmse(p.errs_alt[n], p.strata),
                "Pp": p.pp[n],
                "Yp": p.yp[n],
            }
        )
    return rows


def decisions_to_dict(result: Mapping[str, Any]) -> dict[str, Any]:
    """JSON-able decisions (Decision and ValidityDecision objects become dicts)."""
    out: dict[str, Any] = {}
    for k, v in result.items():
        if isinstance(v, (gp2.Decision, gp2.ValidityDecision)):
            out[k] = v.to_dict()
        elif k == "metrics":
            out[k] = {
                n: {f: (dict(x) if isinstance(x, Mapping) else x) for f, x in vars(mm).items()}
                for n, mm in v.items()
            }
        else:
            out[k] = v
    return out
