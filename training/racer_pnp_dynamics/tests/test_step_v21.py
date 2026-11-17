"""L1 for step contract 2.1 (ground_motion_v21.py) and the schema 2.1.0 variant."""

from __future__ import annotations

import copy
import hashlib

import numpy as np
import pytest
import yaml
from hypothesis import given, settings
from hypothesis import strategies as st
from racer_pnp_dynamics import export
from racer_pnp_dynamics import ground_motion as gmo
from racer_pnp_dynamics import ground_motion_v21 as g21
from racer_pnp_dynamics.model import Nominal, rollout, step
from racer_pnp_dynamics.vehicle import repo_root
from test_fit import TRUTH
from test_step_v2 import random_learned

REAL = repo_root() / g21.V05_RELATIVE
C5 = g21.load_constants_v05(REAL)


def table(seed: int, lo: float, hi: float) -> gmo.GroundTable:
    rng = np.random.default_rng(seed)
    s, v = np.array(C5.steering_nodes), np.array(C5.speed_nodes_mps)
    return gmo.GroundTable(s, v, rng.uniform(lo, hi, (len(s), len(v))))


def yaw_table(seed: int, lo: float = -0.6, hi: float = 0.6, centre_zero: bool = True):
    t = table(seed, lo, hi)
    vals = t.values.copy()
    if centre_zero:
        vals[2, :] = 0.0
    return gmo.GroundTable(t.steering_nodes, t.speed_nodes_mps, vals)


def motion(seed: int, learned: bool = False, scale: float = 1.0) -> g21.GroundMotionV21:
    return g21.GroundMotionV21(
        C5,
        table(seed, 0.3, 1.4),
        yaw_table(seed + 1),
        random_learned(seed, scale) if learned else None,
    )


def test_constants_come_from_the_registered_yaml():
    assert C5.sha256 == g21.V05_SHA256
    assert (C5.alpha_min, C5.alpha_max) == (0.70, 1.05)
    assert (C5.gamma_min, C5.gamma_max, C5.bound_margin) == (-0.25, 0.25, 0.005)
    assert C5.steering_nodes == (-1.0, -0.5, 0.0, 0.5, 1.0)
    assert C5.speed_nodes_mps == (0.35, 0.6, 0.85, 1.1, 1.3, 1.5)
    assert C5.min_windows_per_node == 60 and C5.smoothness_weights == (0.01, 0.1, 1.0, 10.0)
    assert (C5.window_length_s, C5.window_start_period_s) == (0.5, 0.25)
    assert (C5.min_wheel_speed_mps, C5.huber_delta, C5.split_half_block_s) == (0.30, 1.345, 10.0)
    assert (C5.d_alpha_max, C5.rho_max, C5.seeds) == (0.10, 0.25, (0, 1, 2))
    assert C5.weight_decays == (1e-3, 1e-2) and C5.horizons_s == (0.5, 1.0, 1.5)
    assert C5.layers == ((4, 32), (32, 32), (32, 2)) and C5.t2s_scale == 0.80
    assert set(C5.doc) >= {"gate_support", "gate_table", "gate_learned", "gate_validity"}


def test_a_constants_file_with_another_hash_is_refused(tmp_path):
    path = tmp_path / "v05.yaml"
    path.write_text(REAL.read_text() + "\n# edited\n", encoding="utf-8")
    with pytest.raises(g21.PreregInvalid, match="not the pre-registered"):
        g21.load_constants_v05(path)
    assert hashlib.sha256(REAL.read_bytes()).hexdigest() == g21.V05_SHA256


@settings(max_examples=60, deadline=None)
@given(
    seed=st.integers(0, 10_000),
    delta=st.floats(-0.4, 0.4),
    cmd_v=st.floats(0.0, 1.5),
    cmd_s=st.floats(-0.4, 0.4),
    x=st.floats(-5, 5),
    yaw=st.floats(-3.1, 3.1),
)
def test_zero_wheel_speed_gives_zero_ground_motion_and_yaw_rate(
    seed, delta, cmd_v, cmd_s, x, yaw, toy_vehicle_module
):
    gm = motion(seed, learned=True, scale=3.0)
    out = g21.step_v21(
        np.array([x, 1.0, yaw, 0.0, delta]),
        np.array([cmd_v, cmd_s]),
        TRUTH,
        toy_vehicle_module,
        gm,
    )
    assert out[0] == x and out[1] == 1.0 and out[2] == yaw


@settings(max_examples=60, deadline=None)
@given(
    seed=st.integers(0, 10_000),
    v=st.floats(0.0, 2.0),
    delta=st.floats(-0.4, 0.4),
    cmd_v=st.floats(0.0, 2.0),
    cmd_s=st.floats(-0.4, 0.4),
)
def test_alpha_gamma_and_rho_stay_in_their_bounds_for_any_tables_and_weights(
    seed, v, delta, cmd_v, cmd_s, toy_vehicle_module
):
    wild_alpha, wild_gamma = table(seed, 0.0, 3.0), table(seed, -3.0, 3.0)
    gm = g21.GroundMotionV21(C5, wild_alpha, wild_gamma, random_learned(seed, 5.0))
    alpha, gamma, rho = g21.alpha_gamma_rho(
        gm, np.array([0.0, 0.0, 0.0, v, delta]), np.array([cmd_v, cmd_s]), toy_vehicle_module
    )
    assert C5.alpha_min <= alpha <= C5.alpha_max
    assert C5.gamma_min <= gamma <= C5.gamma_max
    assert abs(rho) <= C5.rho_max + 1e-15


@pytest.mark.parametrize("learned", [False, True])
def test_gamma_zero_equals_step_v2_to_1e_12(toy_vehicle, learned):
    rng = np.random.default_rng(0)
    alpha = table(3, 0.7, 1.05)
    ln = random_learned(3, 0.7) if learned else None
    new = g21.GroundMotionV21(C5, alpha, g21.zero_yaw_table(alpha), ln)
    old = gmo.GroundMotion(C5, alpha, ln)  # step_v2 reads only the contract numbers of C5
    state = np.column_stack(
        [rng.normal(size=500), rng.normal(size=500), rng.uniform(-3, 3, 500),
         rng.uniform(0, 1.5, 500), rng.uniform(-0.4, 0.4, 500)]
    )  # fmt: skip
    ctrl = np.column_stack([rng.uniform(0, 1.5, 500), rng.uniform(-0.4, 0.4, 500)])
    np.testing.assert_allclose(
        g21.step_v21(state, ctrl, TRUTH, toy_vehicle, new),
        gmo.step_v2(state, ctrl, TRUTH, toy_vehicle, old),
        rtol=0, atol=1e-12,
    )  # fmt: skip
    controls = np.stack([ctrl[:40]] * 30, axis=1)
    np.testing.assert_allclose(
        g21.rollout_v21(state[:40], controls, TRUTH, toy_vehicle, new),
        gmo.rollout_v2(state[:40], controls, TRUTH, toy_vehicle, old),
        rtol=0, atol=1e-12,
    )  # fmt: skip


def test_all_unit_tables_are_the_contract_1_step(toy_vehicle):
    one = gmo.GroundTable(
        np.array(C5.steering_nodes), np.array(C5.speed_nodes_mps), np.ones((5, 6))
    )
    gm = g21.GroundMotionV21(C5, one, g21.zero_yaw_table(one))
    rng = np.random.default_rng(1)
    state = np.column_stack(
        [np.zeros(100), np.zeros(100), rng.uniform(-3, 3, 100), rng.uniform(0, 1.5, 100),
         rng.uniform(-0.4, 0.4, 100)]
    )  # fmt: skip
    ctrl = np.column_stack([rng.uniform(0, 1.5, 100), rng.uniform(-0.4, 0.4, 100)])
    np.testing.assert_allclose(
        g21.step_v21(state, ctrl, TRUTH, toy_vehicle, gm),
        step(state, ctrl, TRUTH, toy_vehicle),
        rtol=0, atol=1e-12,
    )  # fmt: skip
    controls = np.stack([ctrl[:20]] * 30, axis=1)
    np.testing.assert_allclose(
        g21.rollout_v21(state[:20], controls, TRUTH, toy_vehicle, gm),
        rollout(state[:20], controls, TRUTH, toy_vehicle),
        rtol=0, atol=1e-12,
    )  # fmt: skip


def test_gamma_scales_the_yaw_rate_and_leaves_position_and_the_wheel_channel_alone(toy_vehicle):
    alpha = gmo.GroundTable(
        np.array(C5.steering_nodes), np.array(C5.speed_nodes_mps), np.full((5, 6), 0.9)
    )
    flat = np.full((5, 6), 0.1)
    flat[2, :] = 0.0
    g_tab = gmo.GroundTable(alpha.steering_nodes, alpha.speed_nodes_mps, flat)
    state = np.array([0.0, 0.0, 0.3, 1.0, 0.2])
    ctrl = np.array([1.0, 0.2])
    zero = g21.GroundMotionV21(C5, alpha, g21.zero_yaw_table(alpha))
    a = g21.step_v21(state, ctrl, TRUTH, toy_vehicle, zero)
    b = g21.step_v21(state, ctrl, TRUTH, toy_vehicle, g21.GroundMotionV21(C5, alpha, g_tab))
    assert b[2] - 0.3 == pytest.approx(1.1 * (a[2] - 0.3))
    np.testing.assert_array_equal(a[[0, 1, 3, 4]], b[[0, 1, 3, 4]])
    # on the straight (steering node 0) the table is 0: no yaw effect
    straight = g21.step_v21(
        np.array([0.0, 0.0, 0.3, 1.0, 0.0]),
        np.array([1.0, 0.0]),
        TRUTH,
        toy_vehicle,
        g21.GroundMotionV21(C5, alpha, g_tab),
    )
    assert straight[2] == pytest.approx(0.3)


def test_the_yaw_table_has_a_centre_column_of_zero_and_values_in_bounds():
    g21.check_yaw_table(yaw_table(1, -0.2, 0.2), C5)
    with pytest.raises(ValueError, match="centre column"):
        g21.check_yaw_table(yaw_table(1, -0.2, 0.2, centre_zero=False), C5)
    with pytest.raises(ValueError, match="outside"):
        g21.check_yaw_table(yaw_table(2, -0.6, 0.6), C5)
    two = gmo.GroundTable(np.array([-1.0, 1.0]), np.array([0.3, 0.6]), np.zeros((2, 2)))
    with pytest.raises(ValueError, match="no centre node"):
        g21.check_yaw_table(two, C5)


def test_the_yaw_table_must_have_the_grid_of_the_alpha_table():
    other = gmo.GroundTable(np.zeros(2), np.zeros(2), np.zeros((2, 2)))
    with pytest.raises(ValueError, match="grid of the alpha table"):
        g21.GroundMotionV21(C5, table(1, 0.8, 1.0), other)


def test_window_errors_v21_matches_v2_when_gamma_is_zero(toy_vehicle):
    from racer_pnp_dynamics.dataset import by_split, make_segments
    from racer_pnp_dynamics.evaluate import eval_set
    from synth import make_run, make_split

    run, mot = make_run(
        TRUTH, toy_vehicle, seconds=200.0, seed=2, name="b", v_hi=1.0, exact_model_dt=True
    )
    segs = make_segments(
        run, mot, make_split("b", [("A", 5.0, 190.0, "train", "ccw")]), t_ref_s=run.t_s[0]
    )
    es = eval_set(by_split(segs, "train", odom_only=True))
    assert len(es) > 50
    alpha = table(4, 0.7, 1.05)
    zero = g21.GroundMotionV21(C5, alpha, g21.zero_yaw_table(alpha))
    new = g21.window_errors_v21(es, TRUTH, toy_vehicle, zero)
    old = gmo.window_errors_v2(es, TRUTH, toy_vehicle, gmo.GroundMotion(C5, alpha))
    for h in (0.5, 1.0, 1.5):
        for k in ("speed", "yaw", "pos", "along", "across"):
            np.testing.assert_allclose(new[h][k], old[h][k], atol=1e-12)
    with_gamma = g21.window_errors_v21(es, TRUTH, toy_vehicle, motion(5))
    assert not np.allclose(with_gamma[1.0]["yaw"], old[1.0]["yaw"])


# ---- schema 2.1.0 -----------------------------------------------------------------------------


def _manifest_v21() -> dict:
    nodes = {
        "steering_nodes_share_of_side_limit": list(C5.steering_nodes),
        "wheel_speed_nodes_mps": list(C5.speed_nodes_mps),
        "interpolation": "bilinear",
    }
    grid = [[0.9] * 6 for _ in range(5)]
    zeros = [[0.0] * 6 for _ in range(5)]
    sup = [[True] * 6 for _ in range(5)]
    io = {
        "state": ["x", "y", "yaw", "v", "delta"],
        "control": ["speed_cmd", "steering_cmd"],
        "dt_s": 0.05,
        "nominal": TRUTH.to_dict(),
        "ground_motion": {
            "alpha_bounds": [C5.alpha_min, C5.alpha_max],
            "table": {**nodes, "values": grid, "support": sup},
            "yaw_table": {
                **nodes,
                "values": zeros,
                "bounds": [C5.gamma_min, C5.gamma_max],
                "support": sup,
            },
            "learned": {
                "enabled": False,
                "inputs": ["v", "d", "s", "t"],
                "outputs": ["a", "r"],
                "layers": [[4, 32], [32, 32], [32, 2]],
                "activation": "tanh",
                "input_mean": [0.0] * 4,
                "input_std": [1.0] * 4,
                "d_alpha_max": 0.1,
                "rho_max": 0.25,
                "file": "ground_mlp.f32",
            },
        },
        "validity": {"v_min_mps": 0.0, "v_max_mps": 1.3},
        "golden": "golden_rollouts.csv",
    }
    return {
        "bundle_schema_version": g21.BUNDLE_SCHEMA_VERSION_V21,
        "kind": "dynamics",
        "model_id": "dyn-test-v21",
        "created_utc": "2026-11-16T00:00:00Z",
        "git_sha": "0" * 40,
        "training_config_sha256": "1" * 64,
        "vehicle_params": {"schema_version": "9.9.9", "sysid_session_id": "toy"},
        "payload": [{"file": "ground_mlp.f32", "sha256": "2" * 64}],
        "io": io,
        "data": ["synthetic"],
        "metrics": {},
    }


def test_schema_2_1_0_validates():
    m = _manifest_v21()
    export.validate_manifest(m)
    assert m["bundle_schema_version"] == "2.1.0"


@pytest.mark.parametrize(
    "mutate",
    [
        lambda m: m["io"]["ground_motion"].pop("yaw_table"),
        lambda m: m["io"]["ground_motion"]["table"].pop("support"),
        lambda m: m["io"]["ground_motion"]["yaw_table"].pop("bounds"),
        lambda m: m["io"]["ground_motion"]["yaw_table"].pop("support"),
        lambda m: m["io"]["ground_motion"]["table"]["support"][0].__setitem__(0, 1),
        lambda m: m["io"].__setitem__("residual", {"enabled": False}),
        lambda m: m["io"]["ground_motion"]["yaw_table"].__setitem__("extra", 1),
    ],
)
def test_schema_2_1_0_refuses_what_it_does_not_allow(mutate):
    m = _manifest_v21()
    mutate(m)
    with pytest.raises(export.BundleInvalid):
        export.validate_manifest(m)


def test_2_1_0_content_under_other_numbers_and_the_reverse_are_refused():
    m21 = _manifest_v21()
    as_20 = copy.deepcopy(m21)
    as_20["bundle_schema_version"] = "2.0.0"  # the 2.1 blocks do not exist in 2.0.0
    with pytest.raises(export.BundleInvalid):
        export.validate_manifest(as_20)
    as_21 = copy.deepcopy(m21)
    del as_21["io"]["ground_motion"]["yaw_table"]  # a 2.0.0 body under a 2.1.0 number
    del as_21["io"]["ground_motion"]["table"]["support"]
    with pytest.raises(export.BundleInvalid):
        export.validate_manifest(as_21)
    as_1 = copy.deepcopy(m21)
    as_1["bundle_schema_version"] = "1.1.0"
    with pytest.raises(export.BundleInvalid):
        export.validate_manifest(as_1)


def test_2_0_0_and_1_x_bundles_validate_as_before(tmp_path, toy_vehicle):
    from test_export_v2 import _write

    _, manifest = _write(tmp_path / "b", toy_vehicle)
    assert manifest["bundle_schema_version"] == "2.0.0"
    export.validate_manifest(manifest)
    root = repo_root() / "config" / "pnp"
    for path in sorted((root / "fixtures").glob("dyn-*/manifest.yaml")):
        export.validate_manifest(yaml.safe_load(path.read_text(encoding="utf-8")))


def test_the_nominal_of_the_tests_is_a_nominal():
    assert isinstance(TRUTH, Nominal)
