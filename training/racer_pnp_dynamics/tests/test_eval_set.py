"""L1/L2 for evaluate.eval_set: windows are model independent, never straddle a segment, every
horizon is scored on the same set, and the macro numbers prefer the truth over a wrong model."""

from __future__ import annotations

from dataclasses import replace

import numpy as np
from hypothesis import given, settings
from hypothesis import strategies as st
from racer_pnp_dynamics import dataset, evaluate
from racer_pnp_dynamics.strata import assign_strata, macro_rmse
from racer_pnp_dynamics.windows import WARM_N
from synth import make_run, make_split
from test_fit import TRUTH

SEGS = [
    ("A", 5.0, 100.0, "train", "ccw"),
    ("H1", 102.0, 180.0, "heldout", "ccw"),  # a 2 s guard from A and from H2
    ("H2", 182.0, 260.0, "heldout", "cw"),
]


def _held(toy_vehicle, v_hi=1.2, seconds=270.0, seed=4):
    run, motion = make_run(TRUTH, toy_vehicle, seconds=seconds, seed=seed, name="bag", v_hi=v_hi)
    segs = dataset.make_segments(run, motion, make_split("bag", SEGS), t_ref_s=run.t_s[0])
    return run, motion, segs, [s for s in segs if s.split == "heldout"]


def test_every_window_lies_inside_one_segment_with_its_history_and_horizon(toy_vehicle):
    _, _, _, held = _held(toy_vehicle)
    es = evaluate.eval_set(held)
    assert len(es) > 100
    by_name = {}
    for s in held:
        by_name.setdefault(s.name, []).append(s)
    for t, name in zip(es.t_s, es.segment, strict=True):
        pieces = by_name[str(name)]
        # warm up and the largest delay before the start, the horizon and one scan after it
        need0, need1 = t - WARM_N * 0.05 - evaluate.MAX_DELAY_S, t + 1.5 + 0.1
        assert any(p.t0_s <= need0 and need1 <= p.t1_s for p in pieces), (name, t)
    # no window starts in or near the guard between H1 and H2
    t0 = held[0].run.t_s[0]
    gap = (es.t_s - t0 > 180.0 - 1.6 + 1e-6) & (es.t_s - t0 < 182.0 + 0.8 - 1e-6)
    assert not gap.any()


def test_windows_start_every_quarter_second_above_the_minimum_speed(toy_vehicle):
    _, _, _, held = _held(toy_vehicle)
    es = evaluate.eval_set(held)
    assert (es.start_speed >= 0.25 - 1e-12).all()
    for name in set(es.segment):
        t = np.sort(es.t_s[es.segment == name])
        assert (np.diff(t) > 0.15).all()  # one window per quarter second slot, snapped to scans
    assert set(es.direction) == {"ccw", "cw"}
    low = evaluate.eval_set(held, min_start_speed_mps=0.0)
    assert len(low) > len(es)


def test_the_window_set_does_not_depend_on_the_model_and_horizons_share_it(toy_vehicle):
    _, _, _, held = _held(toy_vehicle)
    es = evaluate.eval_set(held)
    slow = TRUTH.replace(command_delay_s=0.30)
    fast = TRUTH.replace(command_delay_s=0.0)
    e_slow = evaluate.window_errors(es, slow, toy_vehicle)
    e_fast = evaluate.window_errors(es, fast, toy_vehicle)
    assert set(e_slow) == {0.5, 1.0, 1.5}
    for h in e_slow:
        assert len(e_slow[h]["pos"]) == len(e_fast[h]["pos"]) == len(es)
    assert not np.allclose(e_slow[1.0]["speed"], e_fast[1.0]["speed"])
    # the commands the models read differ only by the delay shift
    assert es.commands(0.1).shape == (len(es), WARM_N + 30, 2)


def test_a_failed_icp_interval_drops_the_windows_that_contain_it_for_every_model(toy_vehicle):
    _, motion, _, held = _held(toy_vehicle)
    base = evaluate.eval_set(held)
    ok = motion.ok.copy()
    big = max(held, key=lambda s: s.duration_s)
    mid = 0.5 * (big.t0_s + big.t1_s)
    victim = int(np.argmin(np.abs(motion.t_s - mid)))
    ok[victim] = False
    broken = replace(motion, ok=ok)
    held2 = [replace(s, motion=broken) for s in held]
    es = evaluate.eval_set(held2)
    assert len(es) < len(base)
    t_bad = motion.t_s[victim]
    covering = (es.t_s <= t_bad) & (t_bad < es.t_s + 1.5)
    assert not covering.any()
    # and the same dropped windows serve the nominal of any delay
    e1 = evaluate.window_errors(es, TRUTH, toy_vehicle)
    e2 = evaluate.window_errors(es, TRUTH.replace(command_delay_s=0.2), toy_vehicle)
    assert len(e1[1.0]["pos"]) == len(e2[1.0]["pos"]) == len(es)


def test_blocks_are_five_second_pieces_of_one_segment(toy_vehicle):
    _, _, _, held = _held(toy_vehicle)
    es = evaluate.eval_set(held)
    for b in np.unique(es.block):
        sel = es.block == b
        assert len(set(es.segment[sel])) == 1
        assert np.ptp(es.t_s[sel]) < evaluate.BLOCK_S + 1e-9


def test_truth_beats_a_wrong_model_on_every_horizon_in_the_macro_numbers(toy_vehicle):
    _, _, _, held = _held(toy_vehicle, v_hi=1.5)
    es = evaluate.eval_set(held)
    strata = assign_strata(es.direction, es.start_speed)
    wrong = TRUTH.replace(speed_gain=0.7, steer_gain_left=0.7, steer_gain_right=1.3)
    good = evaluate.window_errors(es, TRUTH, toy_vehicle)
    bad = evaluate.window_errors(es, wrong, toy_vehicle)
    for h in good:
        for kind in ("pos", "yaw", "speed"):
            assert macro_rmse(good[h][kind], strata) < macro_rmse(bad[h][kind], strata)
    assert macro_rmse(good[1.0]["speed"], strata) < 0.05
    assert len(strata.names) >= 3  # speeds to 1.5 m/s fill several bins in both directions


def test_an_empty_or_odometry_free_selection_gives_an_empty_set(toy_vehicle):
    es = evaluate.eval_set([])
    assert len(es) == 0 and es.v_true.shape[1] == 31
    run, motion = make_run(TRUTH, toy_vehicle, seconds=80.0, seed=2, odom=False, name="old")
    segs = dataset.make_segments(run, motion)
    assert len(evaluate.eval_set(segs)) == 0
    assert evaluate.window_errors(es, TRUTH, toy_vehicle)[1.0]["pos"].shape == (0,)


@settings(max_examples=15, deadline=None)
@given(seed=st.integers(0, 50), lo=st.floats(0.0, 0.5))
def test_windows_never_straddle_a_segment_whatever_the_data(seed, lo):
    from racer_pnp_dynamics.model import PLACEHOLDER_NOMINAL
    from racer_pnp_dynamics.vehicle import Vehicle

    vehicle = Vehicle(
        0.5, -0.4, 0.4, 2.0, 2.0, "9.9.9", "toy", 0.5, -0.4, 0.6, 0.1, 0.15, 0.3, 0.0, 3.14
    )
    run, motion = make_run(PLACEHOLDER_NOMINAL, vehicle, seconds=120.0, seed=seed, name="b")
    split = make_split(
        "b", [("S1", 5.0, 40.0, "heldout", "cw"), ("S2", 42.0 + lo, 90.0, "heldout", "ccw")]
    )
    segs = dataset.make_segments(run, motion, split, t_ref_s=run.t_s[0])
    es = evaluate.eval_set(segs, min_start_speed_mps=0.0)
    t0 = run.t_s[0]
    for t, name in zip(es.t_s - t0, es.segment, strict=True):
        a, b = (5.0, 40.0) if name == "S1" else (42.0 + lo, 90.0)
        assert a <= t - 0.8 + 1e-6 and t + 1.6 <= b + 1e-6
