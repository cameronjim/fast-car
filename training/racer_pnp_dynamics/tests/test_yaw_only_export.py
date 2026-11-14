"""Export and L2 for the yaw-only residual (brief 31): a bundle with output_scale [0.0, s] validates
against the unchanged schema, reproduces its golden rollouts, and the refusals are as before; for
any inputs a yaw-only model's speed trajectory is the nominal's."""

from __future__ import annotations

import numpy as np
import pytest
import yaml
from hypothesis import given, settings
from hypothesis import strategies as st
from racer_pnp_dynamics import export
from racer_pnp_dynamics.model import MlpWeights, Residual, rollout, zero_weights
from racer_pnp_dynamics.vehicle import load_vehicle

VEHICLE = load_vehicle()
RNG = np.random.default_rng(11)
WEIGHTS = MlpWeights(*(RNG.normal(scale=0.6, size=a.shape) for a in zero_weights().arrays()))
YAW_ONLY = Residual(
    True,
    WEIGHTS,
    np.array([0.5, 0.0, 0.5, 0.0]),
    np.array([0.4, 0.2, 0.4, 0.2]),
    np.array([0.0, 0.8]),
)


def _bundle(tmp_path, toy_vehicle, toy_nominal, residual=YAW_ONLY):
    export.write_bundle(
        tmp_path / "b",
        model_id="dyn-yaw-only-test",
        created_utc="2026-11-14T00:00:00Z",
        git_sha="a" * 40,
        training_config_sha256="b" * 64,
        vehicle=toy_vehicle,
        nominal=toy_nominal,
        residual=residual,
        v_max_mps=0.9,
        data=["synthetic"],
        metrics={},
        golden_residual_rollouts=True,
    )
    return tmp_path / "b"


def test_a_zero_speed_scale_validates_against_the_unchanged_schema(
    tmp_path, toy_vehicle, toy_nominal
):
    d = _bundle(tmp_path, toy_vehicle, toy_nominal)
    manifest = yaml.safe_load((d / "manifest.yaml").read_text())
    export.validate_manifest(manifest)
    assert manifest["io"]["residual"]["output_scale"] == [0.0, 0.8]
    assert manifest["bundle_schema_version"] == export.BUNDLE_SCHEMA_VERSION == "1.1.0"
    assert manifest["io"]["residual"]["layers"] == [[4, 32], [32, 32], [32, 2]]


def test_golden_rollouts_reproduce_within_1e_4(tmp_path, toy_vehicle, toy_nominal):
    d = _bundle(tmp_path, toy_vehicle, toy_nominal)
    bundle = export.read_bundle(d)
    golden = export.read_golden(d / "golden_rollouts.csv")
    assert golden and bundle.residual.output_scale[0] == 0.0
    assert export.golden_error(golden, bundle.nominal, bundle.residual, toy_vehicle) < 1e-4
    assert export.golden_matches(golden, bundle.nominal, bundle.residual, toy_vehicle)


def test_golden_speed_columns_are_the_nominals_in_a_yaw_only_bundle(
    tmp_path, toy_vehicle, toy_nominal
):
    d = _bundle(tmp_path, toy_vehicle, toy_nominal)
    golden = export.read_golden(d / "golden_rollouts.csv")
    for data in golden.values():
        plain = rollout(data[0, 2:7], data[:-1, 0:2], toy_nominal, toy_vehicle, None)
        np.testing.assert_allclose(data[:, 5], plain[:, 3], rtol=1e-8, atol=1e-9)  # v
        np.testing.assert_allclose(data[:, 6], plain[:, 4], rtol=1e-8, atol=1e-9)  # delta


def test_a_changed_weight_is_still_refused_by_the_checksum(tmp_path, toy_vehicle, toy_nominal):
    d = _bundle(tmp_path, toy_vehicle, toy_nominal)
    blob = bytearray((d / "residual_mlp.f32").read_bytes())
    blob[0] ^= 0xFF
    (d / "residual_mlp.f32").write_bytes(bytes(blob))
    with pytest.raises(ValueError, match="checksum mismatch"):
        export.read_bundle(d)


def test_the_schema_still_refuses_a_malformed_scale(tmp_path, toy_vehicle, toy_nominal):
    d = _bundle(tmp_path, toy_vehicle, toy_nominal)
    manifest = yaml.safe_load((d / "manifest.yaml").read_text())
    manifest["io"]["residual"]["output_scale"] = "zero"
    with pytest.raises(export.BundleInvalid):
        export.validate_manifest(manifest)
    del manifest["io"]["residual"]["output_scale"]
    with pytest.raises(export.BundleInvalid):
        export.validate_manifest(manifest)


finite = {"allow_nan": False, "allow_infinity": False}


@given(
    v=st.floats(0, 1.5, **finite),
    delta=st.floats(VEHICLE.steer_min_rad, VEHICLE.steer_max_rad, **finite),
    speed_cmd=st.floats(-3, 3, **finite),
    steer_cmd=st.floats(-1, 1, **finite),
    scale=st.floats(0.0, 3.0, **finite),
)
@settings(max_examples=200, deadline=None)
def test_yaw_only_rollouts_are_finite_and_have_the_nominals_speed(
    v, delta, speed_cmd, steer_cmd, scale
):
    from racer_pnp_dynamics.model import PLACEHOLDER_NOMINAL

    res = Residual(True, WEIGHTS, np.zeros(4), np.ones(4), np.array([0.0, scale]))
    controls = np.tile([speed_cmd, steer_cmd], (30, 1))
    state0 = np.array([0.0, 0.0, 0.0, v, delta])
    with_res = rollout(state0, controls, PLACEHOLDER_NOMINAL, VEHICLE, res)
    plain = rollout(state0, controls, PLACEHOLDER_NOMINAL, VEHICLE, None)
    assert np.isfinite(with_res).all()
    assert np.array_equal(with_res[:, 3], plain[:, 3])  # v: the nominal's, any input
    assert np.array_equal(with_res[:, 4], plain[:, 4])  # delta too
