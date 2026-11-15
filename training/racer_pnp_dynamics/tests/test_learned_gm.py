"""L1 for TL4 (learned_gm.py): the cumulative sum form equals the step contract, selection, and (with
torch, which CI does not install) that a known bounded correction is learned."""

from __future__ import annotations

from dataclasses import replace

import numpy as np
import pytest
from racer_pnp_dynamics import fit_data_v04 as fd
from racer_pnp_dynamics import ground_motion as gmo
from racer_pnp_dynamics import learned_gm as lg
from racer_pnp_dynamics.dataset import make_segments
from racer_pnp_dynamics.model import MlpWeights, zero_weights
from synth import make_run, make_split
from test_fit import TRUTH
from test_step_v2 import C, random_learned
from test_table_fit import SEGS, true_table

NOMINAL = TRUTH.replace(steer_offset_rad=0.0, steer_gain_right=1.1)


@pytest.fixture(scope="module")
def windows(toy_vehicle_module):
    run, motion = make_run(
        NOMINAL,
        toy_vehicle_module,
        seconds=670.0,
        seed=9,
        name="bag",
        v_hi=1.1,
        exact_model_dt=True,
    )
    segs = make_segments(run, motion, make_split("bag", SEGS), t_ref_s=run.t_s[0])
    train, val = fd.fit_validation_split(segs)
    ftr = fd.build_fit_windows(train, NOMINAL, toy_vehicle_module)
    return ftr, fd.build_fit_windows(val, NOMINAL, toy_vehicle_module, ftr.scales)


def test_the_cumulative_sum_form_equals_the_step_contract(windows, toy_vehicle_module):
    ftr, _ = windows
    table = true_table()
    learned = random_learned(3, 1.5)
    tr = lg.build_tracks(ftr, NOMINAL, toy_vehicle_module, table)
    got = lg.predict_from_tracks(tr, C, learned)
    states = gmo.predict_v2(
        ftr.es, NOMINAL, toy_vehicle_module, gmo.GroundMotion(C, table, learned)
    )
    for h, hz in enumerate(fd.HORIZONS_S):
        k = round(hz / 0.05)
        np.testing.assert_allclose(got[:, h, :], states[:, k, :3], atol=1e-10)
    none = lg.predict_from_tracks(tr, C, None)
    states0 = gmo.predict_v2(ftr.es, NOMINAL, toy_vehicle_module, gmo.GroundMotion(C, table))
    np.testing.assert_allclose(none[:, 1, :], states0[:, 20, :3], atol=1e-10)


def test_normalisation_and_inputs_have_the_network_shape(windows, toy_vehicle_module):
    ftr, _ = windows
    tr = lg.build_tracks(ftr, NOMINAL, toy_vehicle_module, true_table())
    x = lg.inputs_of(tr)
    assert x.shape == (len(ftr), 30, 4)
    mean, std = lg.normalisation(tr)
    assert mean.shape == (4,) and (std > 0).all()


def _cand(name, seed, p1):
    return lg.GmCandidate(lg.GmConfig(name, 1e-3), seed, random_learned(seed), p1, 10)


def test_the_shipping_candidate_is_the_best_seed_of_the_best_config_by_validation_p1():
    cands = [_cand("a", 0, 1.0), _cand("a", 1, 0.9), _cand("b", 0, 0.5), _cand("b", 1, 1.6)]
    ship, table = lg.select(cands)
    assert table["a"]["mean"] == pytest.approx(0.95) and table["b"]["mean"] == pytest.approx(1.05)
    assert (ship.config.name, ship.seed) == (
        "a",
        1,
    )  # the config with the lower mean, then its best seed
    ship, _ = lg.select([_cand("a", 0, np.nan), _cand("a", 1, 0.7)])
    assert ship.seed == 1  # NaN never wins


def test_the_device_is_recorded_and_the_configs_are_the_pre_declared_ones():
    assert "float64" in lg.DEVICE_NOTE and "MPS" in lg.DEVICE_NOTE
    assert [c.name for c in lg.GM_CONFIGS] == ["wd1e-3", "wd1e-2"]


def test_a_known_bounded_correction_is_learned(windows, toy_vehicle_module):
    pytest.importorskip("torch")
    ftr, fva = windows
    table = true_table()
    # the plant: the table plus a bounded correction that raises alpha at speed and turns a bit more
    arrays = [np.zeros_like(a) for a in zero_weights().arrays()]
    arrays[5] = np.array(
        [0.8, 0.5]
    )  # b3: tanh(0.8) of d_alpha_max and tanh(0.5) of rho_max, constant
    plant = gmo.GroundMotion(C, table, gmo.Learned(MlpWeights(*arrays), np.zeros(4), np.ones(4)))

    def relabel(fw):
        states = gmo.predict_v2(fw.es, NOMINAL, toy_vehicle_module, plant)
        poses = states[:, ::2, :3].copy()[:, : fw.es.poses.shape[1]]
        return replace(fw, es=replace(fw.es, poses=poses))

    tr_w, va_w = relabel(ftr), relabel(fva)
    tr = lg.build_tracks(tr_w, NOMINAL, toy_vehicle_module, table)
    va = lg.build_tracks(va_w, NOMINAL, toy_vehicle_module, table)
    cfg = lg.GmConfig("t", 1e-3, lr=1e-2, max_epochs=60, patience=10)
    learned, epochs = lg.train_gm_torch(cfg, 0, C, tr, va, ftr.scales)
    assert epochs >= 1
    from racer_pnp_dynamics.table_fit import validation_p1

    plain = validation_p1(va_w.es, NOMINAL, toy_vehicle_module, gmo.GroundMotion(C, table))
    got = validation_p1(va_w.es, NOMINAL, toy_vehicle_module, gmo.GroundMotion(C, table, learned))
    assert got < 0.5 * plain
