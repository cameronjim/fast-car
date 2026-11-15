"""L1 for T4 (table_fit.py): data from a known table plus noise is recovered, a sparse node is
tied, a node on a bound is flagged."""

from __future__ import annotations

from dataclasses import replace

import numpy as np
import pytest
from racer_pnp_dynamics import fit_data_v04 as fd
from racer_pnp_dynamics import ground_motion as gmo
from racer_pnp_dynamics import table_fit as tf
from racer_pnp_dynamics.dataset import make_segments
from racer_pnp_dynamics.evaluate import eval_set, predict
from synth import make_run, make_split
from test_fit import TRUTH
from test_step_v2 import C

SEGS = [("T1", 5.0, 330.0, "train", "ccw"), ("T2", 335.0, 660.0, "train", "cw")]
NOMINAL = TRUTH.replace(steer_offset_rad=0.0, steer_gain_right=1.1)


def true_table(low_node: float | None = None) -> gmo.GroundTable:
    t = gmo.GroundTable.constant(C)
    share = np.array(C.steering_nodes)[:, None]
    speed = np.array(C.speed_nodes_mps)[None, :]
    vals = 1.0 - 0.18 * np.abs(share) * np.clip(speed / 1.25, 0.0, 1.0)
    if low_node is not None:
        vals[6, 2] = low_node  # full left lock at 0.85 m/s
    return gmo.GroundTable(t.steering_nodes, t.speed_nodes_mps, vals)


@pytest.fixture(scope="module")
def windows(toy_vehicle_module):
    run, motion = make_run(
        NOMINAL,
        toy_vehicle_module,
        seconds=670.0,
        seed=5,
        name="bag",
        v_hi=1.1,
        exact_model_dt=True,
    )
    segs = make_segments(run, motion, make_split("bag", SEGS), t_ref_s=run.t_s[0])
    train, val = fd.fit_validation_split(segs)
    return train, val


def _relabel(fw: fd.FitWindows, gm: gmo.GroundMotion, vehicle, noise: float, seed: int):
    """The same windows with the truth poses replaced by the rollout of a known ground motion
    model (plus measurement noise): the data of a plant that obeys contract 2 exactly."""
    states = gmo.predict_v2(fw.es, NOMINAL, vehicle, gm)
    poses = states[:, ::2, :3].copy()[:, : fw.es.poses.shape[1]]
    rng = np.random.default_rng(seed)
    poses[:, 1:, :2] += rng.normal(0.0, noise, poses[:, 1:, :2].shape)
    return replace(fw, es=replace(fw.es, poses=poses))


def _fit(windows, vehicle, table, noise=0.003):
    train, val = windows
    gm = gmo.GroundMotion(C, table)
    ftr = fd.build_fit_windows(train, NOMINAL, vehicle)
    fva = fd.build_fit_windows(val, NOMINAL, vehicle, ftr.scales)
    ftr, fva = _relabel(ftr, gm, vehicle, noise, 1), _relabel(fva, gm, vehicle, noise, 2)
    scales = fd.loss_scales(predict(ftr.es.data(NOMINAL.command_delay_s), NOMINAL, vehicle), ftr.es)
    ftr, fva = replace(ftr, scales=scales), replace(fva, scales=scales)
    return tf.fit_t4(ftr, fva, NOMINAL, vehicle, C), ftr


def test_a_known_table_is_recovered_where_the_data_support_it(windows, toy_vehicle_module):
    truth = true_table()
    result, _ = _fit(windows, toy_vehicle_module, truth)
    supported = result.support >= C.min_windows_per_node
    assert supported.sum() >= 10
    err = np.abs(result.table.values - truth.values)[supported]
    assert err.max() < 0.03, err.max()
    assert result.smoothness in C.smoothness_weights
    assert result.on_bound == []
    assert result.validation_p1[result.smoothness] < result.nominal_validation_p1


def test_a_node_with_too_few_windows_is_tied_to_its_nearest_supported_node(
    windows, toy_vehicle_module
):
    result, _ = _fit(windows, toy_vehicle_module, true_table())
    n_v = len(C.speed_nodes_mps)
    sparse = np.flatnonzero(result.support.reshape(-1) < C.min_windows_per_node)
    assert len(sparse) > 0  # the data stop at about 1.1 m/s: the 1.5 m/s column is empty
    for n in sparse:
        t = int(result.tied[n])
        assert t != n and result.support.reshape(-1)[t] >= C.min_windows_per_node
        assert result.table.values[n // n_v, n % n_v] == pytest.approx(
            result.table.values[t // n_v, t % n_v]
        )
    rows = result.table_rows()
    assert len(rows) == 35 and any(r["tied_to"] is not None for r in rows)


def test_a_supported_node_on_a_bound_is_flagged(windows, toy_vehicle_module):
    result, _ = _fit(windows, toy_vehicle_module, true_table(low_node=0.45), noise=0.001)
    if result.support[6, 2] >= C.min_windows_per_node:
        assert result.on_bound
        assert result.table.values[6, 2] == pytest.approx(C.alpha_min, abs=1e-5)
        assert any(r["on_bound"] for r in result.table_rows())
    else:  # the synthetic data never reached that corner: nothing can be flagged there
        pytest.skip("node (full lock, 0.85 m/s) is not supported by this synthetic run")


def test_a_node_off_the_data_is_never_flagged_even_at_a_bound_value():
    support = np.array([100, 0, 100])
    tied = tf.nearest_supported(support, (1, 3), 30)
    assert list(tied) == [0, 0, 2]  # the middle one is equally near: the lower index wins


def test_no_supported_node_is_an_error():
    with pytest.raises(ValueError, match="no table node"):
        tf.nearest_supported(np.zeros(6, dtype=int), (2, 3), 30)


def test_second_difference_rows_kill_a_plane_and_see_a_kink():
    rows = tf.second_difference_rows((7, 5))
    i, j = np.meshgrid(np.arange(7), np.arange(5), indexing="ij")
    plane = (0.3 * i - 0.1 * j + 2.0).reshape(-1)
    assert np.allclose(rows @ plane, 0.0)
    kink = np.zeros((7, 5))
    kink[3, 2] = 1.0
    assert np.abs(rows @ kink.reshape(-1)).max() == 2.0


def test_node_weights_sum_to_one_and_reproduce_the_table_interpolation(toy_vehicle):
    rng = np.random.default_rng(0)
    delta = rng.uniform(-0.45, 0.45, 200)
    v = rng.uniform(0.0, 2.0, 200)
    w = tf.node_weights(C, delta, v, toy_vehicle)
    assert np.allclose(w.sum(axis=-1), 1.0)
    table = gmo.GroundTable.constant(C)
    vals = rng.uniform(0.7, 1.0, table.values.shape)
    table = gmo.GroundTable(table.steering_nodes, table.speed_nodes_mps, vals)
    expect = gmo.table_alpha(table, delta, v, toy_vehicle)
    assert np.allclose(w @ vals.reshape(-1), expect)


def test_eval_set_used_by_the_fit_is_the_gates(windows, toy_vehicle_module):
    train, _ = windows
    ftr = fd.build_fit_windows(train, NOMINAL, toy_vehicle_module)
    gate = eval_set([s for s in train if s.run.has_odom])
    assert np.array_equal(ftr.es.t_s, gate.t_s)
