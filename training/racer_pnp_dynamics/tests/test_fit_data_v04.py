"""L1 for the shared windows, validation split and loss (fit_data_v04.py)."""

from __future__ import annotations

import numpy as np
import pytest
from racer_pnp_dynamics import fit_data_v04 as fd
from racer_pnp_dynamics.dataset import by_split, make_segments
from racer_pnp_dynamics.evaluate import eval_set, predict
from synth import make_run, make_split
from test_fit import TRUTH

SEGS = [("T1", 5.0, 100.0, "train", "ccw"), ("T2", 105.0, 200.0, "train", "cw")]


@pytest.fixture(scope="module")
def segs(toy_vehicle_module):
    run, motion = make_run(
        TRUTH, toy_vehicle_module, seconds=210.0, seed=7, name="bag", v_hi=1.0, exact_model_dt=True
    )
    return make_segments(run, motion, make_split("bag", SEGS), t_ref_s=run.t_s[0])


def test_validation_is_the_last_fifth_of_every_train_segment_after_a_two_second_guard(segs):
    train, val = fd.fit_validation_split(segs)
    assert train and val
    for name in ("T1", "T2"):
        tr = [s for s in train if s.name == name]
        va = [s for s in val if s.name == name]
        assert tr and va
        raw = [s for s in segs if s.name == name and s.split in ("train", "val")]
        lo = min(s.i0 for s in raw)
        hi = max(s.i1 for s in raw)
        # validation starts at 80 percent of the segment span, training ends a guard before it
        first_val = min(s.i0 for s in va)
        # a stop at the cut can push the first validation piece later, never earlier
        assert (first_val - lo) / (hi - lo) >= 0.8 - 0.01
        assert sum(s.i1 - s.i0 for s in va) <= 0.2 * (hi - lo) + 1
        assert (first_val - max(s.i1 for s in tr)) / 50.0 >= fd.GUARD_S - 0.05
        assert all(s.split == "train" for s in tr) and all(s.split == "val" for s in va)
    # nothing in both
    for t in train:
        for v in val:
            if t.run is v.run and t.name == v.name:
                assert t.i1 <= v.i0 or v.i1 <= t.i0


def test_the_split_is_built_from_train_and_val_roles_and_never_from_heldout(toy_vehicle):
    run, motion = make_run(TRUTH, toy_vehicle, seconds=120.0, seed=1, name="bag")
    split = make_split(
        "bag", [("A", 5.0, 60.0, "train", "ccw"), ("H", 62.0, 110.0, "heldout", "ccw")]
    )
    segs = make_segments(run, motion, split, t_ref_s=run.t_s[0])
    train, val = fd.fit_validation_split(segs)
    assert {s.name for s in train + val} == {"A"}
    assert by_split(segs, "heldout")  # it exists, and is not used


def test_a_short_segment_keeps_all_its_data_as_training(toy_vehicle):
    run, motion = make_run(TRUTH, toy_vehicle, seconds=30.0, seed=1, name="bag")
    segs = make_segments(
        run, motion, make_split("bag", [("A", 2.0, 12.0, "train", "ccw")]), t_ref_s=run.t_s[0]
    )
    train, val = fd.fit_validation_split(segs)
    assert len(train) == 1 and val == []


def test_huber_is_quadratic_inside_and_linear_outside():
    d = fd.HUBER_DELTA
    assert fd.huber(np.array([0.5]))[0] == pytest.approx(0.125)
    assert fd.huber(np.array([d]))[0] == pytest.approx(0.5 * d * d)
    assert fd.huber(np.array([-10.0]))[0] == pytest.approx(d * (10.0 - 0.5 * d))
    big = fd.huber(np.array([100.0]))[0]
    assert big < 0.5 * 100.0**2  # grows linearly


def test_windows_scales_quality_and_loss(segs, toy_vehicle_module):
    train, _ = fd.fit_validation_split(segs)
    fw = fd.build_fit_windows(train, TRUTH, toy_vehicle_module)
    assert len(fw) > 100 and set(fw.scales) == set(fd.TERMS)
    assert ((fw.quality > 0) & (fw.quality <= 1)).all()
    # the plant itself is the model: scaled errors are small, so a wrong model has a bigger loss
    states = predict(fw.es.data(TRUTH.command_delay_s), TRUTH, toy_vehicle_module)
    bad = predict(
        fw.es.data(TRUTH.command_delay_s), TRUTH.replace(steer_gain_left=0.6), toy_vehicle_module
    )
    assert fd.window_loss(states, fw) < fd.window_loss(bad, fw)
    # scales are the RMS of the nominal's own error on the same windows
    err = fd.relative_motion_errors(states, fw.es)
    assert fw.scales["along"] == pytest.approx(np.sqrt(np.mean(err["along"] ** 2)))
    assert fd.window_loss(states, fw, terms=("heading",)) < fd.window_loss(states, fw)


def test_quality_is_lower_for_a_noisier_match(segs, toy_vehicle_module):
    from dataclasses import replace

    from racer_pnp_dynamics.icp_motion import RMS_MAX_M, MotionSeries

    train, _ = fd.fit_validation_split(segs)
    noisy = []
    for s in train:
        m = s.motion
        noisy.append(
            replace(
                s,
                motion=MotionSeries(
                    **{**m.__dict__, "rms_m": np.full(len(m.t_s), 0.5 * RMS_MAX_M)}
                ),
            )
        )
    es = eval_set([s for s in train if s.run.has_odom])
    clean_q = fd.window_quality(es, train)
    noisy_q = fd.window_quality(es, noisy)
    assert noisy_q.mean() < clean_q.mean() and noisy_q.max() <= 0.5 + 1e-9
    assert fd.window_quality(eval_set([]), []).shape == (0,)
