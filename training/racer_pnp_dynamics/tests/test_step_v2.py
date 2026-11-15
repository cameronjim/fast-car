"""L1 for step contract 2 (ground_motion.py, ground_sweep.py)."""

from __future__ import annotations

import numpy as np
import pytest
import yaml
from hypothesis import given, settings
from hypothesis import strategies as st
from racer_pnp_dynamics import ground_motion as gmo
from racer_pnp_dynamics import ground_sweep
from racer_pnp_dynamics.model import MlpWeights, Nominal, rollout, step, zero_weights
from racer_pnp_dynamics.vehicle import repo_root
from test_fit import TRUTH

DOC = {
    "ground_motion": {
        "alpha_bounds": [0.60, 1.05],
        "table": {
            "steering_nodes_share_of_side_limit": [
                -1.0,
                -0.6667,
                -0.3333,
                0.0,
                0.3333,
                0.6667,
                1.0,
            ],
            "wheel_speed_nodes_mps": [0.25, 0.6, 0.85, 1.25, 1.5],
            "min_windows_per_node": 30,
            "smoothness_weights": [0.0, 0.01, 0.1, 1.0],
        },
        "learned": {
            "layers": [[4, 32], [32, 32], [32, 2]],
            "d_alpha_max": 0.10,
            "rho_max": 0.25,
            "seeds": [0, 1, 2],
        },
        "training_windows": {"horizons_s": [0.5, 1.0, 1.5]},
        "alt_truth": {"icp_seed_translation_scale": 0.80},
    }
}
C = gmo.parse_constants(DOC)


def random_learned(seed: int, scale: float = 1.0) -> gmo.Learned:
    rng = np.random.default_rng(seed)
    arrays = [rng.normal(0.0, scale, a.shape) for a in zero_weights().arrays()]
    return gmo.Learned(
        MlpWeights(*arrays), np.array([0.5, 0.0, 0.5, 0.0]), np.array([0.5, 0.2, 0.5, 0.2])
    )


def random_table(seed: int, lo: float = 0.3, hi: float = 1.4) -> gmo.GroundTable:
    rng = np.random.default_rng(seed)
    t = gmo.GroundTable.constant(C)
    return gmo.GroundTable(t.steering_nodes, t.speed_nodes_mps, rng.uniform(lo, hi, t.values.shape))


def test_constants_are_parsed_from_the_addendum_block():
    assert (C.alpha_min, C.alpha_max, C.d_alpha_max, C.rho_max) == (0.60, 1.05, 0.10, 0.25)
    assert len(C.steering_nodes) == 7 and len(C.speed_nodes_mps) == 5
    assert C.min_windows_per_node == 30 and C.seeds == (0, 1, 2) and C.t2s_scale == 0.80
    assert C.layers == ((4, 32), (32, 32), (32, 2)) and C.smoothness_weights[0] == 0.0


def test_a_constants_file_with_another_hash_is_refused(tmp_path):
    path = tmp_path / "a.yaml"
    path.write_text(yaml.safe_dump(DOC), encoding="utf-8")
    with pytest.raises(gmo.AddendumInvalid, match="not the pre-registered"):
        gmo.load_constants(path)


def test_the_real_addendum_file_parses_to_the_same_constants():
    path = repo_root() / gmo.ADDENDUM_RELATIVE
    if not path.is_file():
        pytest.skip("the addendum yaml is not in this checkout (PR 189)")
    real = gmo.load_constants(path)
    assert real.sha256 == gmo.ADDENDUM_SHA256
    assert {**real.__dict__, "sha256": ""} == C.__dict__


def test_table_reproduces_nodes_interpolates_bilinearly_and_holds_outside(toy_vehicle):
    t = random_table(1)
    s_nodes = np.array(C.steering_nodes)
    v_nodes = np.array(C.speed_nodes_mps)
    for i in (0, 3, 6):
        for j in (0, 2, 4):
            delta = s_nodes[i] * (
                toy_vehicle.steer_max_rad if s_nodes[i] >= 0 else -toy_vehicle.steer_min_rad
            )
            assert gmo.table_alpha(t, delta, v_nodes[j], toy_vehicle) == pytest.approx(
                t.values[i, j]
            )
    mid = gmo.table_alpha(
        t, 0.5 * (0.0 + toy_vehicle.steer_max_rad / 3.0), 0.5 * (0.25 + 0.6), toy_vehicle
    )
    corners = t.values[3:5, 0:2]
    assert mid == pytest.approx(corners.mean(), rel=1e-3)
    assert gmo.table_alpha(t, 0.0, 5.0, toy_vehicle) == pytest.approx(t.values[3, 4])  # fast: held
    assert gmo.table_alpha(t, 0.0, 0.0, toy_vehicle) == pytest.approx(t.values[3, 0])  # slow: held
    assert gmo.table_alpha(t, 9.0, 0.6, toy_vehicle) == pytest.approx(
        t.values[6, 1]
    )  # past the limit
    assert np.isnan(gmo.table_alpha(t, np.nan, 0.6, toy_vehicle))


def test_steering_share_uses_each_sides_own_limit(toy_vehicle):
    from dataclasses import replace

    veh = replace(toy_vehicle, steer_max_rad=0.5, steer_min_rad=-0.25)
    assert gmo.steering_share(0.25, veh) == pytest.approx(0.5)
    assert gmo.steering_share(-0.25, veh) == pytest.approx(-1.0)


@settings(max_examples=60, deadline=None)
@given(
    seed=st.integers(0, 10_000),
    delta=st.floats(-0.4, 0.4),
    cmd_v=st.floats(0.0, 1.5),
    cmd_s=st.floats(-0.4, 0.4),
    x=st.floats(-5, 5),
    yaw=st.floats(-3.1, 3.1),
)
def test_zero_wheel_speed_gives_zero_ground_motion_and_yaw_rate_for_any_weights(
    seed, delta, cmd_v, cmd_s, x, yaw, toy_vehicle_module
):
    gm = gmo.GroundMotion(C, random_table(seed), random_learned(seed, 3.0))
    state = np.array([x, 1.0, yaw, 0.0, delta])
    out = gmo.step_v2(state, np.array([cmd_v, cmd_s]), TRUTH, toy_vehicle_module, gm)
    assert out[0] == x and out[1] == 1.0 and out[2] == yaw


@settings(max_examples=60, deadline=None)
@given(
    seed=st.integers(0, 10_000),
    v=st.floats(0.0, 2.0),
    delta=st.floats(-0.4, 0.4),
    cmd_v=st.floats(0.0, 2.0),
    cmd_s=st.floats(-0.4, 0.4),
)
def test_alpha_and_rho_stay_in_their_bounds_for_any_table_and_weights(
    seed, v, delta, cmd_v, cmd_s, toy_vehicle_module
):
    gm = gmo.GroundMotion(C, random_table(seed, 0.0, 3.0), random_learned(seed, 5.0))
    alpha, rho = gmo.alpha_rho(
        gm, np.array([0.0, 0.0, 0.0, v, delta]), np.array([cmd_v, cmd_s]), toy_vehicle_module
    )
    assert C.alpha_min <= alpha <= C.alpha_max
    assert abs(rho) <= C.rho_max + 1e-15


def test_table_of_one_and_no_learned_block_is_the_contract_1_step(toy_vehicle):
    gm = gmo.GroundMotion(C, gmo.GroundTable.constant(C, 1.0))
    rng = np.random.default_rng(0)
    state = np.column_stack(
        [rng.normal(size=500), rng.normal(size=500), rng.uniform(-3, 3, 500),
         rng.uniform(0, 1.5, 500), rng.uniform(-0.4, 0.4, 500)]
    )  # fmt: skip
    ctrl = np.column_stack([rng.uniform(0, 1.5, 500), rng.uniform(-0.4, 0.4, 500)])
    np.testing.assert_allclose(
        gmo.step_v2(state, ctrl, TRUTH, toy_vehicle, gm), step(state, ctrl, TRUTH, toy_vehicle),
        rtol=0, atol=1e-12,
    )  # fmt: skip
    controls = np.stack([ctrl[:40]] * 30, axis=1)
    np.testing.assert_allclose(
        gmo.rollout_v2(state[:40], controls, TRUTH, toy_vehicle, gm),
        rollout(state[:40], controls, TRUTH, toy_vehicle),
        rtol=0, atol=1e-12,
    )  # fmt: skip


def test_the_wheel_channel_is_untouched_by_the_ground_corrections(toy_vehicle):
    gm = gmo.GroundMotion(C, random_table(3), random_learned(3))
    rng = np.random.default_rng(1)
    state = np.column_stack(
        [
            np.zeros(50),
            np.zeros(50),
            rng.uniform(-3, 3, 50),
            rng.uniform(0, 1.5, 50),
            rng.uniform(-0.4, 0.4, 50),
        ]
    )
    ctrl = np.column_stack([rng.uniform(0, 1.5, 50), rng.uniform(-0.4, 0.4, 50)])
    a = gmo.step_v2(state, ctrl, TRUTH, toy_vehicle, gm)
    b = step(state, ctrl, TRUTH, toy_vehicle)
    np.testing.assert_array_equal(a[:, 3:], b[:, 3:])


def test_ground_distance_is_alpha_times_the_wheel_distance_on_a_straight(toy_vehicle):
    t = gmo.GroundTable.constant(C, 0.8)
    gm = gmo.GroundMotion(C, t)
    state = np.array([0.0, 0.0, 0.0, 1.0, 0.0])
    out = gmo.step_v2(
        state, np.array([1.0, 0.0]), TRUTH.replace(steer_offset_rad=0.0), toy_vehicle, gm
    )
    assert out[0] == pytest.approx(0.8 * 1.0 * 0.05)
    assert out[2] == pytest.approx(0.0)


def test_rho_scales_the_yaw_rate(toy_vehicle):
    # a learned block whose network is zero except a bias that saturates m[1] high
    w = zero_weights()
    arrays = list(w.arrays())
    arrays[5] = np.array([0.0, 50.0])  # b3: tanh(50) = 1
    learned = gmo.Learned(MlpWeights(*arrays), np.zeros(4), np.ones(4))
    gm0 = gmo.GroundMotion(C, gmo.GroundTable.constant(C))
    gm1 = gmo.GroundMotion(C, gmo.GroundTable.constant(C), learned)
    state = np.array([0.0, 0.0, 0.0, 1.0, 0.2])
    a = gmo.step_v2(state, np.array([1.0, 0.2]), TRUTH, toy_vehicle, gm0)
    b = gmo.step_v2(state, np.array([1.0, 0.2]), TRUTH, toy_vehicle, gm1)
    assert b[2] == pytest.approx((1.0 + C.rho_max) * a[2])
    assert b[0] == pytest.approx(a[0])  # alpha unchanged (m[0] = 0)


def test_window_errors_v2_matches_contract_1_for_a_unit_table(toy_vehicle):
    from racer_pnp_dynamics.dataset import by_split, make_segments
    from racer_pnp_dynamics.evaluate import eval_set, window_errors
    from synth import make_run, make_split

    run, motion = make_run(
        TRUTH, toy_vehicle, seconds=200.0, seed=2, name="b", v_hi=1.0, exact_model_dt=True
    )
    segs = make_segments(
        run, motion, make_split("b", [("A", 5.0, 190.0, "train", "ccw")]), t_ref_s=run.t_s[0]
    )
    es = eval_set(by_split(segs, "train", odom_only=True))
    assert len(es) > 50
    gm = gmo.GroundMotion(C, gmo.GroundTable.constant(C))
    new = gmo.window_errors_v2(es, TRUTH, toy_vehicle, gm)
    old = window_errors(es, TRUTH, toy_vehicle)
    helper = gmo.contract1_window_errors(es, TRUTH, toy_vehicle)
    for h in (0.5, 1.0, 1.5):
        for k in ("speed", "yaw", "pos"):
            np.testing.assert_allclose(new[h][k], old[h][k], atol=1e-12)
            np.testing.assert_allclose(helper[h][k], old[h][k], atol=1e-12)
        np.testing.assert_allclose(new[h]["pos"] ** 2, new[h]["along"] ** 2 + new[h]["across"] ** 2)


def test_the_sweep_passes_for_bounded_models_and_catches_a_nan_table(toy_vehicle):
    gm = gmo.GroundMotion(C, random_table(4, 0.0, 3.0), random_learned(4, 4.0))
    res = ground_sweep.sweep_v2(TRUTH, toy_vehicle, gm, 1.5, samples=2000, steps=10)
    assert res.passed and res.samples == 2000 and res.to_dict()["passed"] is True
    t = gmo.GroundTable.constant(C)
    vals = t.values.copy()
    vals[:, :] = np.nan
    bad = gmo.GroundMotion(C, gmo.GroundTable(t.steering_nodes, t.speed_nodes_mps, vals))
    res = ground_sweep.sweep_v2(TRUTH, toy_vehicle, bad, 1.5, samples=500, steps=5)
    assert not res.passed and res.nonfinite > 0


def test_a_table_with_the_wrong_shape_is_refused():
    with pytest.raises(ValueError, match="do not match the grid"):
        gmo.GroundTable(np.zeros(7), np.zeros(5), np.zeros((7, 4)))


def test_nominal_fixture_is_a_nominal():
    assert isinstance(TRUTH, Nominal)
