"""The committed dyn-nominal-v0 fixture must match the live vehicle_params and be reproduced by
model.py. When vehicle_params changes this fails: regenerate deliberately with
`uv run python -m racer_pnp_dynamics.make_nominal_fixture`."""

from __future__ import annotations

import numpy as np
import yaml
from racer_pnp_dynamics import export, make_car_fixture, make_nominal_fixture
from racer_pnp_dynamics.model import PLACEHOLDER_NOMINAL
from racer_pnp_dynamics.vehicle import load_vehicle, repo_root

FIXTURE = repo_root() / make_nominal_fixture.FIXTURE_RELATIVE


def test_fixture_vehicle_params_equal_the_live_file():
    manifest = yaml.safe_load((FIXTURE / "manifest.yaml").read_text())
    live = yaml.safe_load((repo_root() / "config" / "vehicle_params.yaml").read_text())["meta"]
    assert manifest["vehicle_params"] == {
        "schema_version": live["schema_version"],
        "sysid_session_id": live["sysid_session_id"],
    }


def test_fixture_is_a_valid_unfitted_bundle():
    bundle = export.read_bundle(FIXTURE)  # checksums
    export.validate_manifest(bundle.manifest)
    m = bundle.manifest
    assert m["model_id"] == "dyn-nominal-v0" and m["kind"] == "dynamics"
    assert m["data"] == ["not fitted: nominal defaults for simulation and tests"]
    assert bundle.nominal == PLACEHOLDER_NOMINAL
    assert not bundle.residual.enabled and bundle.v_max_mps == 1.0
    assert not bundle.residual.weights.w1.any()


def test_fixture_golden_rollouts_are_reproduced_by_model_py():
    bundle = export.read_bundle(FIXTURE)
    golden = export.read_golden(FIXTURE / "golden_rollouts.csv")
    assert len(golden) >= 8
    assert export.golden_matches(golden, bundle.nominal, bundle.residual, load_vehicle())


def test_regeneration_is_byte_for_byte(tmp_path):
    make_nominal_fixture.make_fixture(tmp_path / "out", load_vehicle())
    for name in ("manifest.yaml", "residual_mlp.f32", "golden_rollouts.csv"):
        assert (tmp_path / "out" / name).read_bytes() == (FIXTURE / name).read_bytes()


def test_cli_writes_the_fixture(tmp_path):
    assert make_nominal_fixture.main(["--out", str(tmp_path / "x")]) == 0
    assert (tmp_path / "x" / "manifest.yaml").is_file()


CAR = repo_root() / make_car_fixture.FIXTURE_RELATIVE


def test_nominal_fixture_has_no_dead_zone_so_existing_canaries_are_unchanged():
    bundle = export.read_bundle(FIXTURE)
    assert bundle.nominal.speed_deadband_mps == 0.0
    assert bundle.manifest["bundle_schema_version"] == "1.1.0"


def test_car_fixture_is_the_fitted_nominal_with_a_dead_zone_and_no_residual():
    bundle = export.read_bundle(CAR)  # checksums and schema
    m = bundle.manifest
    assert m["model_id"] == "dyn-car-2026-11-12" and m["kind"] == "dynamics"
    assert m["metrics"] == {}
    assert not bundle.residual.enabled and not bundle.residual.weights.w1.any()
    assert 0.0 < bundle.nominal.speed_deadband_mps < 0.4
    assert bundle.v_max_mps >= 0.95
    live = yaml.safe_load((repo_root() / "config" / "vehicle_params.yaml").read_text())["meta"]
    assert m["vehicle_params"] == {
        "schema_version": live["schema_version"],
        "sysid_session_id": live["sysid_session_id"],
    }


def test_car_fixture_golden_rollouts_cross_the_dead_zone_and_match_model_py():
    bundle = export.read_bundle(CAR)
    golden = export.read_golden(CAR / "golden_rollouts.csv")
    assert len(golden) == 12
    assert export.golden_matches(golden, bundle.nominal, bundle.residual, load_vehicle())
    ramp, from_rest, rolling = golden[9], golden[10], golden[11]
    assert ramp[:3, 5].max() == 0.0 and ramp[:, 5].max() > 0.25  # still, then moving
    assert from_rest[:, 5].max() == 0.0  # 0.1 m/s from rest never moves the car
    assert rolling[0, 5] == 0.4 and rolling[-1, 5] < 0.05  # 0.1 m/s from 0.4 m/s decays
    assert np.all(ramp[:, 0] <= 0.4 + 1e-12)


def test_car_fixture_regeneration_is_byte_for_byte(tmp_path):
    source = repo_root() / make_car_fixture.SOURCE_RELATIVE
    make_car_fixture.make_fixture(tmp_path / "out", load_vehicle(), source)
    for name in ("manifest.yaml", "residual_mlp.f32", "golden_rollouts.csv"):
        assert (tmp_path / "out" / name).read_bytes() == (CAR / name).read_bytes()
