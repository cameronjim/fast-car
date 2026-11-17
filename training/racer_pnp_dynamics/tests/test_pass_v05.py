"""L1 for the held-out pass machinery (pass_v05.py, ground_sweep_v21.py): the window sets, the
metrics on identical windows, the rollout numbers (supported share, saturation), the sweep and the
decisions wired together, all on synthetic data."""

from __future__ import annotations

import json

import numpy as np
import pytest
from racer_pnp_dynamics import gp2f
from racer_pnp_dynamics import ground_motion as gmo
from racer_pnp_dynamics import ground_motion_v21 as g21
from racer_pnp_dynamics import ground_sweep_v21 as gs
from racer_pnp_dynamics import pass_v05 as pv
from racer_pnp_dynamics.dataset import make_segments
from racer_pnp_dynamics.model import MlpWeights, zero_weights
from racer_pnp_dynamics.scoring_v05 import Spec
from synth import make_run, make_split
from test_fit import TRUTH
from test_step_v2 import random_learned
from test_step_v21 import C5, yaw_table
from test_step_v21 import motion as motion21
from test_step_v21 import table as rand_table


@pytest.fixture(scope="module")
def held(toy_vehicle_module):
    segs = []
    for k, (seed, steer) in enumerate(((3, None), (4, 0.4))):
        run, mot = make_run(
            TRUTH, toy_vehicle_module, seconds=240.0, seed=seed, name=f"h{k}", v_hi=1.0,
            exact_model_dt=True, steer_const=steer,
        )  # fmt: skip
        split = make_split(f"h{k}", [("H", 5.0, 230.0, "heldout", "by_heading")])
        segs += [s for s in make_segments(run, mot, split, t_ref_s=run.t_s[0])]
    return segs


def models():
    return {
        "dyn-v0.1": Spec(TRUTH),
        "T5": Spec(TRUTH, ground21=motion21(5)),
        "TL5 seed 0": Spec(TRUTH, ground21=motion21(5, learned=True, scale=0.3)),
    }


def test_the_pass_scores_every_model_on_identical_windows(held, toy_vehicle_module):
    p = pv.score_pass(C5, held, held, toy_vehicle_module, models(), TRUTH)
    assert len(p.es) > 100 and p.windows_t2_only == 0 and p.windows_t2s_only == 0
    assert set(p.es.direction.tolist()) <= {"ccw", "cw"}
    assert set(p.errs) == set(p.errs_alt) == set(p.pp) == set(p.yp) == set(models())
    assert all(len(e[1.0]["pos"]) == len(p.es) for e in p.errs.values())
    assert np.isnan(p.pp["T5"]) and np.isnan(p.yp["T5"])  # no pose pairs in this data
    m = p.metrics("T5", [1.15, 1.30])
    assert set(m.macro) == {"P1", "A1", "C1", "Y1", "S1"}
    assert set(m.direction) == set(p.es.direction.tolist())
    assert set(m.near_limit) == {"left", "right"} and m.transition[0] == int(p.transition.sum())
    assert set(m.fast) == {1.15, 1.3} and set(m.fast[1.15]) == {"ccw", "cw"}
    assert m.p1_alt == pytest.approx(m.macro["P1"])  # the same windows stand for T2s here
    boot = p.bootstrap("T5", "dyn-v0.1")
    assert boot["lo"] <= boot["ratio"] <= boot["hi"]
    rows = pv.comparator_rows(p, ["dyn-v0.1", "T5"])
    assert [r["model"] for r in rows] == ["dyn-v0.1", "T5"] and rows[0]["P1"] > 0


def test_yaw_bias_is_the_mean_signed_error_per_direction(held, toy_vehicle_module):
    p = pv.score_pass(C5, held, held, toy_vehicle_module, models(), TRUTH)
    m = p.metrics("dyn-v0.1", [])
    for d in ("ccw", "cw"):
        sel = p.es.direction == d
        if sel.any():
            assert m.yaw_bias[d] == pytest.approx(
                float(np.mean(p.errs["dyn-v0.1"][1.0]["yaw"][sel]))
            )
        else:
            assert np.isnan(m.yaw_bias[d])


def test_a_pass_with_no_common_windows_is_an_error(held, toy_vehicle_module):
    with pytest.raises(ValueError, match="no held-out windows"):
        pv.score_pass(C5, [], [], toy_vehicle_module, models(), TRUTH)


def test_near_limit_windows_are_per_side_and_transitions_follow_the_speed_command(
    held, toy_vehicle_module
):
    p = pv.score_pass(C5, held, held, toy_vehicle_module, models(), TRUTH)
    near, trans = pv.steering_masks(p.es, TRUTH, toy_vehicle_module, 0.85, 0.2)
    assert not (near["left"] & near["right"]).any()
    assert near["left"].sum() > 0  # the second run holds the steering at its limit
    assert 0 < trans.sum() < len(p.es)
    # a limit of 0 puts every window near the limit, on the side of its mean steering share
    near0, _ = pv.steering_masks(p.es, TRUTH, toy_vehicle_module, 0.0, 0.2)
    assert (near0["left"] | near0["right"]).all()
    _, none = pv.steering_masks(p.es, TRUTH, toy_vehicle_module, 0.85, 99.0)
    assert not none.any()


def test_step_share_in_supported_cells(held, toy_vehicle_module):
    p = pv.score_pass(C5, held, held, toy_vehicle_module, models(), TRUTH)
    gm = motion21(5)
    every = np.ones((5, 6), dtype=bool)
    assert pv.step_share_in_supported(C5, gm, p.es, TRUTH, toy_vehicle_module, every) == 1.0
    assert pv.step_share_in_supported(C5, gm, p.es, TRUTH, toy_vehicle_module, ~every) == 0.0
    some = every.copy()
    some[:, 2:] = False  # nothing from 0.85 m/s up
    share = pv.step_share_in_supported(C5, gm, p.es, TRUTH, toy_vehicle_module, some)
    assert 0.0 < share < 1.0


def saturated(bias: float) -> g21.GroundMotionV21:
    arrays = list(zero_weights().arrays())
    arrays[5] = np.array([bias, bias])  # b3: tanh(bias) at both outputs
    learned = gmo.Learned(MlpWeights(*arrays), np.zeros(4), np.ones(4))
    return g21.GroundMotionV21(C5, rand_table(1, 0.8, 1.0), yaw_table(2, -0.1, 0.1), learned)


def test_saturation_share_counts_steps_with_an_output_beyond_the_level(held, toy_vehicle_module):
    p = pv.score_pass(C5, held, held, toy_vehicle_module, models(), TRUTH)
    v = toy_vehicle_module
    assert pv.saturation_share(C5, motion21(5), p.es, TRUTH, v, 0.9) == 0.0  # no network
    assert pv.saturation_share(C5, saturated(0.0), p.es, TRUTH, v, 0.9) == 0.0
    assert pv.saturation_share(C5, saturated(5.0), p.es, TRUTH, v, 0.9) == 1.0  # tanh(5) = 0.9999
    assert pv.saturation_share(C5, saturated(0.9), p.es, TRUTH, v, 0.9) == 0.0  # tanh(.9) = 0.716
    assert pv.saturation_share(C5, saturated(1.45), p.es, TRUTH, v, 0.9) == 0.0  # tanh = 0.8957
    assert pv.saturation_share(C5, saturated(1.5), p.es, TRUTH, v, 0.9) == 1.0  # tanh = 0.9051


def test_the_sweep_counts_every_kind_of_violation(toy_vehicle):
    gm = g21.GroundMotionV21(
        C5, rand_table(4, 0.0, 3.0), rand_table(5, -3.0, 3.0), random_learned(4, 4.0)
    )
    res = gs.sweep_v21(TRUTH, toy_vehicle, gm, 1.5, samples=2000, steps=10)
    assert res.passed and res.violations == 0 and res.samples == 2000
    assert res.to_dict()["passed"] is True
    t = rand_table(1, 1.0, 1.0)
    bad = g21.GroundMotionV21(
        C5,
        g21.GroundTable(t.steering_nodes, t.speed_nodes_mps, np.full((5, 6), np.nan)),
        yaw_table(2),
    )
    res = gs.sweep_v21(TRUTH, toy_vehicle, bad, 1.5, samples=500, steps=5)
    assert not res.passed and res.nonfinite > 0 and res.violations >= res.nonfinite


def test_decide_all_wires_the_gates_and_the_learned_branch(held, toy_vehicle_module, monkeypatch):
    v = toy_vehicle_module
    names = models()
    p = pv.score_pass(C5, held, held, v, names, TRUTH)
    supported = np.ones((5, 6), dtype=bool)
    kw = {
        "t5": "T5", "tl5_seeds": {0: "TL5 seed 0"}, "tl5_shipping": "TL5 seed 0", "v01": "dyn-v0.1",
        "on_bound": [], "split_half_alpha_max": 0.01, "split_half_gamma_max": 0.01,
        "t5_motion": motion21(5), "tl5_motion": motion21(5, learned=True, scale=0.3), "nominal": TRUTH,
        "vehicle": v, "rule_mps": 1.42, "supported_alpha": supported, "supported_gamma": supported,
    }  # fmt: skip
    out = pv.decide_all(C5, p, **kw)
    # no pose pairs in this data: Pp and Yp cannot pass, so GP2f-T fails and GP2f-L is not run
    assert out["table"].passed is False and out["learned"].evaluated is False
    assert out["outcome"] == "the one-speed contract stays"
    assert "sweep" not in out and out["top_supported_speed"] == 1.5
    assert out["validity_T5"].v_max_mps == 1.3 and out["validity_TL5"].v_max_mps == 1.3
    assert out["step_share_in_supported"] == 1.0 and out["support"].passed
    # force GP2f-T to pass and shrink the sweep: GP2f-L is evaluated with all its items
    from racer_pnp_dynamics import gp2

    monkeypatch.setattr(
        gp2f,
        "decide_gp2f_table",
        lambda *a, **k: gp2.Decision("T", (gp2.Item("x", 0.0, "<=", 1.0, True),)),
    )
    small = lambda *a, **k: gs.sweep_v21(*a[:4], samples=300, steps=5)
    monkeypatch.setattr(pv, "sweep_v21", small)
    out = pv.decide_all(C5, p, **kw)
    assert out["learned"].evaluated and out["sweep"]["samples"] == 300
    assert out["outcome"] in (gp2f.T5_CANDIDATE, gp2f.TL5_CANDIDATE)
    assert set(out["metrics"]) == set(names)
    d = pv.decisions_to_dict(out)
    json.dumps(d, default=lambda o: o.item() if hasattr(o, "item") else str(o))
    assert d["table"]["gate"] == "T" and "items" in d["validity_T5"]


def test_the_one_second_horizon_is_twenty_steps():
    assert pv.ONE_SECOND_STEPS == 20
