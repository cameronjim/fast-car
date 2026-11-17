"""L1 for T5 (table_fit_v05.py): known alpha and gamma tables plus noise are recovered, a plant whose
wheel speed model is biased still gives the true alpha (the v0.4 fault), a node with data only
below it is unsupported and copies, a node on a bound is flagged, the split half difference is 0 on
duplicated data, and train-only roles are enforced."""

from __future__ import annotations

from dataclasses import replace

import numpy as np
import pytest
from racer_pnp_dynamics import ground_motion as gmo
from racer_pnp_dynamics import ground_motion_v21 as g21
from racer_pnp_dynamics import table_fit_v05 as tf
from racer_pnp_dynamics.dataset import make_segments
from racer_pnp_dynamics.evaluate import eval_set
from racer_pnp_dynamics.model import DT_S
from synth import make_run, make_split
from test_fit import TRUTH
from test_step_v21 import C5

PLANT = TRUTH.replace(steer_offset_rad=0.0, steer_gain_right=1.1)
SECONDS = 2400.0


def _segments(vehicle, seed: int, v_hi: float, role: str, seconds: float = SECONDS):
    name = f"bag{seed}"
    run, motion = make_run(
        PLANT, vehicle, seconds=seconds, seed=seed, name=name, v_hi=v_hi, exact_model_dt=True
    )
    split = make_split(name, [("S", 5.0, seconds - 10.0, role, "ccw")])
    return make_segments(run, motion, split, t_ref_s=run.t_s[0])


@pytest.fixture(scope="module")
def train_windows(toy_vehicle_module):
    segs = _segments(toy_vehicle_module, 5, 1.6, "train")
    return tf.build_t5_windows(segs, PLANT, toy_vehicle_module, C5)


@pytest.fixture(scope="module")
def slow_windows(toy_vehicle_module):
    segs = _segments(toy_vehicle_module, 6, 0.9, "train")
    return tf.build_t5_windows(segs, PLANT, toy_vehicle_module, C5)


def _truth_tables() -> tuple[gmo.GroundTable, gmo.GroundTable]:
    s, v = np.array(C5.steering_nodes), np.array(C5.speed_nodes_mps)
    alpha = 1.0 - 0.14 * np.abs(s)[:, None] * np.clip(v[None, :] / 1.3, 0.0, 1.0)
    gamma = np.zeros_like(alpha)
    gamma[0] = np.linspace(0.04, -0.03, len(v))  # steering -1 (right)
    gamma[1] = 0.5 * gamma[0]
    gamma[3] = -0.04 * np.ones(len(v))
    gamma[4] = -0.07 * np.ones(len(v))
    return gmo.GroundTable(s, v, alpha), gmo.GroundTable(s, v, gamma)


def _inject(win, vehicle, alpha, gamma, noise=0.002, seed=0):
    """The same windows with the targets of a plant that obeys contract 2.1 exactly."""
    a = gmo.table_alpha(alpha, win.delta, win.v, vehicle)
    g = gmo.table_alpha(gamma, win.delta, win.v, vehicle)
    rng = np.random.default_rng(seed)
    ground = (a * win.v * DT_S).sum(axis=1) + rng.normal(0.0, noise, len(win))
    dyaw = ((1.0 + g) * win.w_nom * DT_S).sum(axis=1) + rng.normal(0.0, noise, len(win))
    return replace(win, ground_m=ground, dyaw_rad=dyaw)


def _selection(vehicle, alpha, gamma, seed=11):
    """A selection set whose truth is the rollout of the known tables (a plant of contract 2.1)."""
    segs = _segments(vehicle, seed, 1.6, "heldout", 400.0)
    es = eval_set([s for s in segs if s.run.has_odom])
    gm = g21.GroundMotionV21(C5, alpha, gamma)
    states = g21.predict_v21(es, PLANT, vehicle, gm)
    poses = states[:, ::2, :3][:, : es.poses.shape[1]].copy()
    return replace(es, poses=poses)


def test_windows_use_the_measured_speed_and_drop_slow_samples(toy_vehicle_module, train_windows):
    win = train_windows
    assert len(win) > 1000 and win.v.shape[1] == 10 and win.role == "train"
    assert np.isfinite(win.ground_m).all() and np.isfinite(win.dyaw_rad).all()
    assert win.half.min() == 0 and win.half.max() == 1
    # every sample of every window is at least 0.30 m/s (the step starts are a subset of them)
    assert win.v.min() >= C5.min_wheel_speed_mps
    segs = _segments(toy_vehicle_module, 5, 1.6, "train")
    es = eval_set([s for s in segs if s.run.has_odom], horizon_s=0.5)
    assert (es.v_true.min(axis=1) < C5.min_wheel_speed_mps).any()  # some were dropped
    assert len(win) < len(es)
    # the speed of a window is the recorded wheel speed, not any model's
    keep = es.v_true.min(axis=1) >= C5.min_wheel_speed_mps
    np.testing.assert_array_equal(win.v, es.v_true[keep][:, :10])


def test_a_known_alpha_and_gamma_table_are_recovered_where_the_data_support_them(
    train_windows, toy_vehicle_module
):
    alpha, gamma = _truth_tables()
    win = _inject(train_windows, toy_vehicle_module, alpha, gamma)
    sel = _selection(toy_vehicle_module, alpha, gamma)
    fit = tf.fit_t5(win, sel, PLANT, toy_vehicle_module, C5)
    assert fit.supported.sum() >= 12
    free = fit.supported & ~fit.fixed
    err_a = np.abs(fit.alpha.values - alpha.values)[fit.supported]
    err_g = np.abs(fit.gamma.values - gamma.values)[free]
    assert err_a.max() < 0.03, err_a.max()
    assert err_g.max() < 0.03, err_g.max()
    assert fit.lam_alpha in C5.smoothness_weights and fit.lam_gamma in C5.smoothness_weights
    assert fit.selection_y1[fit.lam_gamma] == min(fit.selection_y1.values())
    assert fit.selection_a1[fit.lam_alpha] == min(fit.selection_a1.values())
    assert fit.on_bound_alpha == [] and fit.on_bound_gamma == []
    assert (fit.gamma.values[fit.fixed] == 0.0).all()
    g21.check_yaw_table(fit.gamma, C5)
    gm = fit.groundmotion()
    assert gm.support is not None and gm.support.dtype == bool
    assert len(fit.rows()) == 30


def test_a_plant_whose_wheel_speed_model_is_biased_at_low_speed_still_gives_the_true_alpha(
    train_windows, toy_vehicle_module
):
    """The v0.4 fault as a regression test: T4 multiplied the model's PREDICTED wheel speed, so a
    wheel model that reads wrong at low speed (here: a dead band and a low gain) put its own error
    in the table. T5 uses the measured speed, so the nominal's wheel channel cannot matter."""
    alpha, gamma = _truth_tables()
    biased = PLANT.replace(speed_gain=0.7, speed_deadband_mps=0.45, speed_tau_s=0.9)
    segs = _segments(toy_vehicle_module, 5, 1.6, "train")
    win_biased = tf.build_t5_windows(segs, biased, toy_vehicle_module, C5)
    np.testing.assert_array_equal(win_biased.v, train_windows.v)  # measured, not modelled
    np.testing.assert_array_equal(win_biased.delta, train_windows.delta)  # steering unchanged
    win = _inject(win_biased, toy_vehicle_module, alpha, gamma)
    supported = tf.support_counts(C5, win, toy_vehicle_module) >= C5.min_windows_per_node
    tied = tf.copy_map(C5, supported)
    values = tf.fit_alpha_values(C5, win, toy_vehicle_module, tied, 0.01).reshape(supported.shape)
    low = supported[:, 0]  # the lowest speed column, where the biased model is most wrong
    assert low.sum() >= 3, low
    assert np.abs(values[:, 0] - alpha.values[:, 0])[low].max() < 0.03
    assert np.abs(values - alpha.values)[supported].max() < 0.03


def test_a_node_with_data_only_below_it_is_unsupported_and_copies_its_column(
    slow_windows, toy_vehicle_module
):
    alpha, gamma = _truth_tables()
    win = _inject(slow_windows, toy_vehicle_module, alpha, gamma)
    sel = _selection(toy_vehicle_module, alpha, gamma)
    fit = tf.fit_t5(win, sel, PLANT, toy_vehicle_module, C5, split_half=False)
    n_v = len(C5.speed_nodes_mps)
    assert win.v.max() < 1.15  # the data stop below the top three speed nodes
    assert not fit.supported[:, 5].any() and not fit.supported[:, 4].any()
    top = np.flatnonzero(~fit.supported.reshape(-1))
    assert len(top) > 0
    for n in top:
        t = int(fit.tied[n])
        assert t != n and fit.supported.reshape(-1)[t]
        i, j = divmod(n, n_v)
        assert t // n_v == i  # the same steering column
        assert fit.alpha.values[i, j] == fit.alpha.values[t // n_v, t % n_v]
        if not fit.fixed[i, j]:
            assert fit.gamma.values[i, j] == fit.gamma.values[t // n_v, t % n_v]
    rows = fit.rows()
    assert any(r["tied_to"] is not None for r in rows)


def test_a_supported_node_on_a_bound_is_flagged_and_a_copied_one_is_not(
    train_windows, toy_vehicle_module
):
    alpha, gamma = _truth_tables()
    support = tf.support_counts(C5, train_windows, toy_vehicle_module)
    i, j = np.unravel_index(
        np.argmax(np.where(np.arange(5)[:, None] == 2, 0, support)), support.shape
    )
    low = alpha.values.copy()
    low[i, j] = 0.50  # below the lower bound 0.70: the fit can only end on the bound
    win = _inject(
        train_windows,
        toy_vehicle_module,
        gmo.GroundTable(alpha.steering_nodes, alpha.speed_nodes_mps, low),
        gamma,
        noise=0.001,
    )
    supported = support >= C5.min_windows_per_node
    tied = tf.copy_map(C5, supported)
    values = tf.fit_alpha_values(C5, win, toy_vehicle_module, tied, 0.01).reshape(support.shape)
    assert values[i, j] == pytest.approx(C5.alpha_min, abs=1e-6)
    name = f"s{C5.steering_nodes[i]:+.4f}/v{C5.speed_nodes_mps[j]:g}"
    flagged = tf._on_bound(
        C5, values, (C5.alpha_min, C5.alpha_max), supported, np.zeros_like(supported)
    )
    assert name in flagged
    # a node within the margin counts, a node outside it does not
    v = values.copy()
    v[i, j] = C5.alpha_min + C5.bound_margin - 1e-9
    assert name in tf._on_bound(
        C5, v, (C5.alpha_min, C5.alpha_max), supported, np.zeros_like(supported)
    )
    v[i, j] = C5.alpha_min + C5.bound_margin + 1e-3
    assert name not in tf._on_bound(
        C5, v, (C5.alpha_min, C5.alpha_max), supported, np.zeros_like(supported)
    )
    # an unsupported node is never flagged, whatever its value
    unsup = supported.copy()
    unsup[i, j] = False
    assert name not in tf._on_bound(
        C5, values, (C5.alpha_min, C5.alpha_max), unsup, np.zeros_like(supported)
    )


def test_split_half_difference_is_zero_on_duplicated_data(train_windows, toy_vehicle_module):
    alpha, gamma = _truth_tables()
    base = _inject(train_windows, toy_vehicle_module, alpha, gamma)
    one = base.subset(base.half == 0)
    dup = replace(one, half=np.zeros(len(one), dtype=int))
    twin = replace(one, half=np.ones(len(one), dtype=int))
    both = tf.T5Windows(
        "train",
        *(np.concatenate([getattr(dup, f), getattr(twin, f)]) for f in (
            "t_s", "segment", "direction", "half", "v", "delta", "w_nom", "ground_m", "dyaw_rad", "quality")),
    )  # fmt: skip
    sel = _selection(toy_vehicle_module, alpha, gamma)
    fit = tf.fit_t5(both, sel, PLANT, toy_vehicle_module, C5)
    assert fit.split_half is not None
    d = fit.split_half
    assert np.nanmax(d["alpha_diff"]) == pytest.approx(0.0, abs=1e-12)
    assert np.nanmax(d["gamma_diff"]) == pytest.approx(0.0, abs=1e-12)
    assert np.isnan(d["alpha_diff"][~fit.supported]).all()
    assert np.isnan(d["gamma_diff"][fit.fixed]).all()
    assert (d["windows_even_blocks"] == d["windows_odd_blocks"]).all()


def test_split_half_differences_show_a_real_disagreement(train_windows, toy_vehicle_module):
    alpha, gamma = _truth_tables()
    win = _inject(train_windows, toy_vehicle_module, alpha, gamma)
    # the odd blocks come from a plant that is 8 percent shorter in ground distance
    odd = win.half == 1
    ground = np.where(odd, 0.92 * win.ground_m, win.ground_m)
    win = replace(win, ground_m=ground)
    sel = _selection(toy_vehicle_module, alpha, gamma)
    fit = tf.fit_t5(win, sel, PLANT, toy_vehicle_module, C5)
    assert np.nanmax(fit.split_half["alpha_diff"]) > 0.05


def test_copy_map_rules():
    n_v = len(C5.speed_nodes_mps)
    edges = np.zeros((5, n_v), dtype=bool)
    edges[0, :] = edges[4, :] = True  # steering -1 and +1 fully supported
    sup = edges.copy()
    sup[3, 1] = sup[3, 4] = True  # column +0.5 supported at 0.6 and 1.3
    t = tf.copy_map(C5, sup)
    assert t[3 * n_v + 1] == 3 * n_v + 1  # a supported node is itself
    assert t[3 * n_v + 5] == 3 * n_v + 4  # nearest in speed within the column
    assert t[3 * n_v + 0] == 3 * n_v + 1
    assert t[3 * n_v + 3] == 3 * n_v + 4  # 1.1 is nearer 1.3 than 0.6
    # equal distance in speed: the lower speed node
    tie = edges.copy()
    tie[3, 1] = tie[3, 3] = True  # 0.6 and 1.1 are both 0.25 from 0.85
    assert tf.copy_map(C5, tie)[3 * n_v + 2] == 3 * n_v + 1
    # an empty column copies the nearest supported node of its row on the same side
    assert t[1 * n_v + 3] == 0 * n_v + 3  # -0.5 copies -1, never +1
    assert t[2 * n_v + 0] == 0 * n_v + 0  # the centre is on both sides: equal distance, lower
    assert t[2 * n_v + 4] == 3 * n_v + 4  # ... and the nearer node wins otherwise


def test_copy_map_with_nothing_to_copy_from_is_an_error():
    n_v = len(C5.speed_nodes_mps)
    sup = np.zeros((5, n_v), dtype=bool)
    sup[0, 0] = True  # nothing at any other speed on the same side
    with pytest.raises(tf.CannotCopy, match="no supported node"):
        tf.copy_map(C5, sup)


def test_cells_are_the_nearest_node_on_each_axis():
    share = np.array([-0.8, -0.26, 0.24, 0.26, 2.0])
    speed = np.array([0.1, 0.47, 0.48, 1.45, 9.0])
    i, j = tf.cell_of(C5, share, speed)
    assert list(i) == [0, 1, 2, 3, 4]
    assert list(j) == [0, 0, 1, 5, 5]  # 0.47 is nearer 0.35 than 0.6 (0.12 against 0.13)


def test_free_map_copies_and_fixes():
    tied = np.array([0, 0, 2, 2, 4])
    m, free = tf.free_map(tied)
    assert list(free) == [0, 2, 4]
    assert (m @ np.array([1.0, 2.0, 3.0])).tolist() == [1.0, 1.0, 2.0, 2.0, 3.0]
    fixed = np.array([False, False, True, False, False])
    m2, free2 = tf.free_map(tied, fixed)
    assert list(free2) == [0, 4]
    assert (m2 @ np.array([1.0, 3.0])).tolist() == [1.0, 1.0, 0.0, 0.0, 3.0]  # 3 copies the fixed 2


def test_only_train_data_can_be_fitted_on(toy_vehicle_module, train_windows):
    alpha, gamma = _truth_tables()
    sel_segs = _segments(toy_vehicle_module, 21, 1.6, "heldout", 200.0)
    assert {s.split for s in sel_segs} == {"heldout"}
    with pytest.raises(tf.RoleViolation, match="train data only"):
        tf.build_t5_windows(sel_segs, PLANT, toy_vehicle_module, C5)
    foreign = replace(train_windows, role="selection")
    with pytest.raises(tf.RoleViolation, match="cannot be fitted on"):
        tf.fit_t5(
            foreign, _selection(toy_vehicle_module, alpha, gamma), PLANT, toy_vehicle_module, C5
        )
    with pytest.raises(tf.RoleViolation):
        tf.split_half_differences(
            tf.fit_t5(
                _inject(train_windows, toy_vehicle_module, alpha, gamma),
                _selection(toy_vehicle_module, alpha, gamma),
                PLANT,
                toy_vehicle_module,
                C5,
                split_half=False,
            ),
            foreign,
            toy_vehicle_module,
        )


def test_an_empty_window_set_is_an_error_and_builds_as_empty(toy_vehicle_module, train_windows):
    empty = tf.build_t5_windows([], PLANT, toy_vehicle_module, C5)
    assert len(empty) == 0 and empty.v.shape == (0, 10)
    alpha, gamma = _truth_tables()
    with pytest.raises(ValueError, match="no train windows"):
        tf.fit_t5(
            empty, _selection(toy_vehicle_module, alpha, gamma), PLANT, toy_vehicle_module, C5
        )


def test_unit_motion_is_the_base_nominal(toy_vehicle_module, train_windows):
    gm = tf.unit_motion(C5)
    assert (gm.table.values == 1.0).all() and (gm.yaw_table.values == 0.0).all()
    alpha, gamma = _truth_tables()
    es = _selection(toy_vehicle_module, alpha, gamma)
    s = tf.score(es, PLANT, toy_vehicle_module, gm)
    from racer_pnp_dynamics.evaluate import window_errors
    from racer_pnp_dynamics.strata import assign_strata, macro_rmse

    old = window_errors(es, PLANT, toy_vehicle_module, None, (1.0,))[1.0]
    strata = assign_strata(es.direction, es.start_speed)
    assert s.p1 == pytest.approx(macro_rmse(old["pos"], strata), abs=1e-12)
    assert s.y1 == pytest.approx(macro_rmse(old["yaw"], strata), abs=1e-12)
