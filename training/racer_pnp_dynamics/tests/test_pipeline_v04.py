"""L1 for the held-out pass (pipeline_v04.py) on a synthetic plant whose ground distance is less than
its wheel distance in turns: windows aligned across T2 and T2s, every metric on identical windows,
the decisions in order, the fast item and the validity."""

from __future__ import annotations

from dataclasses import replace

import numpy as np
import pytest
from racer_pnp_dynamics import fit_data_v04 as fd
from racer_pnp_dynamics import gp2
from racer_pnp_dynamics import ground_motion as gmo
from racer_pnp_dynamics import pipeline_v04 as p4
from racer_pnp_dynamics import table_fit as tf
from racer_pnp_dynamics.dataset import by_split, make_segments
from racer_pnp_dynamics.evaluate import eval_set
from racer_pnp_dynamics.icp_motion import MotionSeries
from racer_pnp_dynamics.model import Residual
from synth import make_run, make_split
from test_fit import TRUTH
from test_step_v2 import C, random_learned

NOMINAL = TRUTH.replace(steer_offset_rad=0.0, steer_gain_right=1.1)
SEGS = [
    ("T1", 5.0, 330.0, "train", "ccw"),
    ("T2", 335.0, 660.0, "train", "cw"),
    ("H1", 665.0, 790.0, "heldout", "ccw"),
    ("H2", 795.0, 920.0, "heldout", "cw"),
]


def _shift(segments, factor):
    out = []
    for s in segments:
        m = s.motion
        out.append(
            replace(
                s,
                motion=MotionSeries(
                    **{**m.__dict__, "dx_m": m.dx_m * factor, "dy_m": m.dy_m * factor}
                ),
            )
        )
    return out


@pytest.fixture(scope="module")
def world(toy_vehicle_module):
    run, motion = make_run(
        NOMINAL, toy_vehicle_module, seconds=930.0, seed=21, name="bag", v_hi=1.4,
        wheel_over_read=0.8, exact_model_dt=True, yaw_noise=0.002, speed_noise=0.005,
    )  # fmt: skip
    segs = make_segments(run, motion, make_split("bag", SEGS), t_ref_s=run.t_s[0])
    train, val = fd.fit_validation_split(segs)
    ftr = fd.build_fit_windows(train, NOMINAL, toy_vehicle_module)
    fva = fd.build_fit_windows(val, NOMINAL, toy_vehicle_module, ftr.scales)
    t4 = tf.fit_t4(ftr, fva, NOMINAL, toy_vehicle_module, C)
    held = by_split(segs, "heldout", odom_only=True)
    return t4, held, _shift(held, 0.97)


def _frozen(t4_fit, learned=True):
    table = t4_fit.groundmotion()
    tl = gmo.GroundMotion(C, table.table, random_learned(1, 0.05))
    res = Residual.disabled()
    return p4.Frozen(
        v01=NOMINAL,
        n4=NOMINAL,
        n4_on_bound=[],
        r4={b: {0: res, 1: res, 2: res} for b in p4.BASES},
        r4_shipping_seed={b: 0 for b in p4.BASES},
        t4={b: table for b in p4.BASES},
        tl4={b: {0: tl, 1: tl, 2: tl} for b in p4.BASES},
        tl4_shipping_seed={b: 0 for b in p4.BASES},
        v_max_rule_mps=1.5,
        t4_on_bound={b: list(t4_fit.on_bound) for b in p4.BASES},
    )


@pytest.fixture(scope="module")
def result(world, toy_vehicle_module):
    t4_fit, held, held_s = world
    frozen = _frozen(t4_fit)
    p = p4.score_pass(held, held_s, toy_vehicle_module, frozen.models())
    return frozen, p


def test_windows_are_identical_across_the_two_truths(world, toy_vehicle_module):
    _, held, held_s = world
    a, b = p4.align(eval_set(held), eval_set(held_s))
    assert len(a) == len(b) > 100
    assert np.array_equal(a.t_s, b.t_s) and np.array_equal(a.segment, b.segment)
    # a window that exists under one truth only is dropped and counted
    broken = []
    for s in held_s:
        m = s.motion
        ok = m.ok.copy()
        a0 = int(np.searchsorted(m.t_s, s.t0_s))
        ok[a0 + 50 : a0 + 90] = False
        broken.append(replace(s, motion=MotionSeries(**{**m.__dict__, "ok": ok})))
    p = p4.score_pass(held, broken, toy_vehicle_module, {p4.V01: p4.ModelSpec(NOMINAL)})
    full = p4.score_pass(held, held_s, toy_vehicle_module, {p4.V01: p4.ModelSpec(NOMINAL)})
    assert p.windows < full.windows and p.windows_t2_only > 0


def test_every_metric_is_printed_for_every_model(result):
    frozen, p = result
    names = set(frozen.models())
    assert set(p.macro) == names and set(p.p1_alt) == names and set(p.stratum_pos) == names
    for n in names:
        assert set(p.at1(n)) == {"P1", "A1", "C1", "Y1", "S1"}
        assert all(np.isfinite(v) for v in p.at1(n).values())
        assert np.isnan(p.pp[n]) and np.isnan(p.yp[n])  # no /pose data in this synthetic world
    d = p.to_dict()
    assert d["windows"] == p.windows and "T4[dyn-v0.1]" in d["macro"]
    rows = p4.comparator_table(p, [p4.V01, "T4[dyn-v0.1]"])
    text = p4.render_comparator(rows)
    assert "| dyn-v0.1 |" in text and "| T4[dyn-v0.1] |" in text


def test_the_table_repairs_position_and_leaves_the_other_channels_alone(result):
    _, p = result
    base, t4 = p.at1(p4.V01), p.at1("T4[dyn-v0.1]")
    assert t4["P1"] < 0.8 * base["P1"]
    assert abs(t4["Y1"] - base["Y1"]) < 1e-9 and abs(t4["S1"] - base["S1"]) < 1e-9
    assert p.p1_alt["T4[dyn-v0.1]"] < p.p1_alt[p4.V01]
    assert p.stratum_pos["T4[dyn-v0.1]"].keys() == p.stratum_pos[p4.V01].keys()
    boot = p.bootstrap("T4[dyn-v0.1]", p4.V01)
    assert boot["hi"] < 1.0 and boot["lo"] <= boot["ratio"] <= boot["hi"]


def test_the_fast_item_counts_windows_per_direction(result):
    _, p = result
    fast = {f.direction: f for f in p.fast["T4[dyn-v0.1]"]}
    assert set(fast) == {"ccw", "cw"}
    assert all(f.windows >= 0 for f in fast.values())
    for f in fast.values():
        if f.windows:
            assert f.model_p1 <= f.v01_p1 + 1e-12


def test_the_decisions_run_in_order_and_gp2e_t_cannot_pass_without_pose_data(
    result, toy_vehicle_module
):
    frozen, p = result
    out = p4.decide(p, frozen, toy_vehicle_module, 1.3)
    # N4 equals dyn-v0.1 here, so P1 ratio is 1: GP2d-N fails item 1a? the ratio is exactly 1.0 (at threshold)
    assert out["shipping_nominal"] in (p4.V01, p4.N4)
    t = {i["name"][:2]: i for i in out["GP2e_T"]["items"]}
    assert t["7 "]["passed"] is False and "cannot be applied" in t["7 "]["note"]
    assert out["GP2e_T"]["passed"] is False
    assert out["GP2e_L"]["passed"] is False and out["GP2e_L"].get("evaluated") is False
    assert out["gp2e_outcome"] == gp2.ONE_SPEED_STAYS
    assert out["heading_outcome"]["label"] in (gp2.HEADING_LABEL, gp2.NO_NAMED_OUTCOME)
    assert "deploy" not in " ".join(out["heading_outcome"])  # a label, never a flag
    assert out["validity"]["v_max_mps"] <= 1.3 + 1e-12


def test_with_pose_data_present_the_table_passes_and_the_learned_gate_is_evaluated(
    result, toy_vehicle_module
):
    frozen, p = result
    q = p4.Pass(**p.__dict__)
    q.pp = {n: (0.30 if "T4" not in n and "TL4" not in n else 0.25) for n in q.pp}
    q.yp = {n: 0.03 for n in q.yp}
    out = p4.decide(q, frozen, toy_vehicle_module, 1.3)
    items = out["GP2e_T"]["items"]
    failed = [i["name"] for i in items if not i["passed"]]
    assert out["GP2e_T"]["passed"], failed
    assert out["GP2e_L"].get("evaluated", True) is True and len(out["GP2e_L"]["items"]) > 10
    assert out["gp2e_outcome"] in (gp2.TABLE_WINS, gp2.TL4_CANDIDATE)
    assert out["tl4_shipping"].startswith("TL4[")


def test_a_pass_with_no_common_windows_is_refused(world, toy_vehicle_module):
    _, held, _ = world
    with pytest.raises(ValueError, match="no held-out windows"):
        p4.score_pass(held, [], toy_vehicle_module, {p4.V01: p4.ModelSpec(NOMINAL)})
