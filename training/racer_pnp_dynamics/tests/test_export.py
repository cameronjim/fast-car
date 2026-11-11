"""L1 for export.py and vehicle.py: schema validation, checksums, golden round trip, refusals."""

from __future__ import annotations

import copy
import types
from dataclasses import replace

import numpy as np
import pytest
import yaml
from racer_pnp_dynamics import export
from racer_pnp_dynamics.model import MlpWeights, Residual, zero_weights
from racer_pnp_dynamics.vehicle import Vehicle, VehicleParamsError, load_binding, load_vehicle


def _write(tmp_path, toy_vehicle, toy_nominal, residual=None, v_max=0.9):
    return export.write_bundle(
        tmp_path / "bundle",
        model_id="dyn-test-v0",
        created_utc="2026-11-12T00:00:00Z",
        git_sha="a" * 40,
        training_config_sha256="b" * 64,
        vehicle=toy_vehicle,
        nominal=toy_nominal,
        residual=residual or Residual.disabled(),
        v_max_mps=v_max,
        data=["synthetic"],
        metrics={"x": 1.0},
    )


def test_manifest_validates_and_checksums_match(tmp_path, toy_vehicle, toy_nominal):
    manifest = _write(tmp_path, toy_vehicle, toy_nominal)
    export.validate_manifest(manifest)
    on_disk = yaml.safe_load((tmp_path / "bundle" / "manifest.yaml").read_text())
    assert on_disk == manifest
    assert {p["file"] for p in on_disk["payload"]} == {"residual_mlp.f32", "golden_rollouts.csv"}
    for item in on_disk["payload"]:
        assert export.sha256_file(tmp_path / "bundle" / item["file"]) == item["sha256"]
    assert on_disk["vehicle_params"] == {"schema_version": "9.9.9", "sysid_session_id": "toy"}


def test_disabled_residual_writes_zeros(tmp_path, toy_vehicle, toy_nominal):
    w = zero_weights()
    loud = Residual(
        False, MlpWeights(*(a + 1.0 for a in w.arrays())), np.zeros(4), np.ones(4), np.ones(2)
    )
    _write(tmp_path, toy_vehicle, toy_nominal, loud)
    blob = np.frombuffer((tmp_path / "bundle" / "residual_mlp.f32").read_bytes(), dtype="<f4")
    assert blob.size == 1282 and not blob.any()


def test_enabled_residual_round_trips(tmp_path, toy_vehicle, toy_nominal):
    rng = np.random.default_rng(3)
    w = MlpWeights(*(rng.normal(scale=0.3, size=a.shape) for a in zero_weights().arrays()))
    res = Residual(
        True,
        w,
        np.array([0.1, 0.0, 0.3, 0.0]),
        np.array([1.0, 2.0, 1.0, 2.0]),
        np.array([0.5, 0.25]),
    )
    _write(tmp_path, toy_vehicle, toy_nominal, res)
    bundle = export.read_bundle(tmp_path / "bundle")
    assert bundle.residual.enabled and bundle.nominal == toy_nominal
    np.testing.assert_allclose(bundle.residual.weights.w2, w.w2, atol=1e-6)
    np.testing.assert_allclose(bundle.residual.input_std, res.input_std)
    assert bundle.v_max_mps == 0.9


def test_golden_round_trips_within_tolerance(tmp_path, toy_vehicle, toy_nominal):
    _write(tmp_path, toy_vehicle, toy_nominal)
    golden = export.read_golden(tmp_path / "bundle" / "golden_rollouts.csv")
    assert len(golden) >= 8
    assert all(len(rows) == export.GOLDEN_STEPS + 1 for rows in golden.values())
    for rows in golden.values():
        np.testing.assert_array_equal(rows[-1, 0:2], [0.0, 0.0])  # last row's controls
    res = Residual.disabled()
    assert export.golden_error(golden, toy_nominal, res, toy_vehicle) < 1e-7
    assert export.golden_matches(golden, toy_nominal, res, toy_vehicle)
    other = toy_nominal.replace(steer_tau_s=0.2)
    assert not export.golden_matches(golden, other, res, toy_vehicle)


def test_golden_covers_saturation_and_braking(toy_vehicle, toy_nominal):
    rows = np.asarray(export.golden_rows(toy_nominal, Residual.disabled(), toy_vehicle, 0.9))
    steer = rows[:, 3]
    assert steer.max() > toy_vehicle.steer_max_rad and steer.min() < toy_vehicle.steer_min_rad
    v_by_rollout = {k: rows[rows[:, 0] == k, 7] for k in np.unique(rows[:, 0])}
    assert any(np.diff(v).min() < 0 for v in v_by_rollout.values())
    assert any(np.diff(v).max() > 0 for v in v_by_rollout.values())


def test_golden_text_has_nine_significant_digits(tmp_path, toy_vehicle, toy_nominal):
    _write(tmp_path, toy_vehicle, toy_nominal)
    lines = (tmp_path / "bundle" / "golden_rollouts.csv").read_text().splitlines()
    assert lines[0] == ",".join(export.GOLDEN_HEADER)
    assert any(
        len(f.replace("-", "").replace(".", "").lstrip("0")) == 9 for f in lines[2].split(",")
    )


def test_read_golden_rejects_a_wrong_header(tmp_path):
    path = tmp_path / "g.csv"
    path.write_text("a,b\n1,2\n")
    with pytest.raises(ValueError):
        export.read_golden(path)


def test_invalid_manifest_is_refused_and_nothing_is_written(tmp_path, toy_vehicle, toy_nominal):
    schema = copy.deepcopy(export.load_schema())
    schema["properties"]["data"] = {"type": "array", "maxItems": 0}
    with pytest.raises(export.BundleInvalid):
        export.write_bundle(
            tmp_path / "bundle",
            model_id="dyn-test-v0",
            created_utc="2026-11-12T00:00:00Z",
            git_sha="a" * 40,
            training_config_sha256="b" * 64,
            vehicle=toy_vehicle,
            nominal=toy_nominal,
            residual=Residual.disabled(),
            v_max_mps=0.9,
            data=["not allowed by this schema"],
            metrics={},
            schema=schema,
        )
    assert not (tmp_path / "bundle").exists()


def test_bad_model_id_and_extra_key_fail_validation(tmp_path, toy_vehicle, toy_nominal):
    manifest = _write(tmp_path, toy_vehicle, toy_nominal)
    bad = {**manifest, "model_id": "Dyn_Bad"}
    with pytest.raises(export.BundleInvalid):
        export.validate_manifest(bad)
    bad = {**manifest, "surprise": 1}
    with pytest.raises(export.BundleInvalid):
        export.validate_manifest(bad)
    bad = copy.deepcopy(manifest)
    del bad["io"]["nominal"]["steer_tau_s"]
    with pytest.raises(export.BundleInvalid):
        export.validate_manifest(bad)


def test_v_max_above_full_scale_speed_is_refused(tmp_path, toy_vehicle, toy_nominal):
    with pytest.raises(export.BundleInvalid):
        _write(tmp_path, toy_vehicle, toy_nominal, v_max=toy_vehicle.throttle_full_scale_mps + 0.1)
    assert not (tmp_path / "bundle").exists()


def test_read_bundle_detects_a_changed_payload(tmp_path, toy_vehicle, toy_nominal):
    _write(tmp_path, toy_vehicle, toy_nominal)
    with (tmp_path / "bundle" / "golden_rollouts.csv").open("a") as fh:
        fh.write("\n")
    with pytest.raises(ValueError, match="checksum"):
        export.read_bundle(tmp_path / "bundle")


def test_config_sha256_ignores_key_order():
    assert export.config_sha256({"a": 1, "b": 2}) == export.config_sha256({"b": 2, "a": 1})
    assert export.sha256_text("abc") == (
        "ba7816bf8f01cfea414140de5dae2223b00361a396177a9cb410ff61f20015ad"
    )


def test_export_refuses_when_a_needed_vehicle_param_is_null():
    live = load_binding()
    nulled = types.SimpleNamespace(
        **{name: getattr(live, name) for name in ("chassis", "steering", "actuation", "sensors")},
        meta=live.meta,
    )
    nulled.chassis = replace(live.chassis, wheelbase_m=None)
    with pytest.raises(VehicleParamsError, match="wheelbase_m"):
        Vehicle.from_binding(nulled)
    nulled.chassis = live.chassis
    nulled.actuation = replace(live.actuation, throttle_full_scale_mps=None)
    with pytest.raises(VehicleParamsError, match="throttle_full_scale_mps"):
        Vehicle.from_binding(nulled)


def test_live_vehicle_loads_with_physical_values():
    v = load_vehicle()
    assert v.wheelbase_m > 0 and v.steer_min_rad < 0 < v.steer_max_rad
    assert v.physical_lock_left_rad >= v.steer_max_rad
    assert v.schema_version and v.sysid_session_id
