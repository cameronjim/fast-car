"""L1 export of R6 (brief 54): a bundle of R6's shape (dyn-v0.1 byte for byte, a yaw-only residual,
schema 1.x, validity 1.3) loads in the Python reference and in the planner's loader test fixture
(`ros_ws/src/racer_planner/test/fixtures/make_fixture.py`, the second implementation of the
model written from the contract), and the two roll out within 1e-9 of each other; its golden
rollouts match. The committed pointer manifest, once written, is checked the same way."""

from __future__ import annotations

import importlib.util
import json

import numpy as np
import pytest
import yaml
from racer_pnp_dynamics import export, stages_v06
from racer_pnp_dynamics.model import Nominal, Residual, rollout
from racer_pnp_dynamics.residual_train import _init_weights
from racer_pnp_dynamics.vehicle import load_vehicle, repo_root

ROOT = repo_root()
MODEL_ID = stages_v06.model_id("2026-11-24")
POINTER = ROOT / "training" / "pnp_models" / MODEL_ID
V01 = ROOT / "training" / "pnp_models" / "dyn-v0.1-2026-11-12" / "manifest.yaml"
FIXTURE = ROOT / "ros_ws" / "src" / "racer_planner" / "test" / "fixtures" / "make_fixture.py"


def _manifest(path):
    return yaml.safe_load(path.read_text(encoding="utf-8"))


def _second_implementation():
    spec = importlib.util.spec_from_file_location("make_fixture_r6", FIXTURE)
    mod = importlib.util.module_from_spec(spec)
    assert spec.loader is not None
    spec.loader.exec_module(mod)
    return mod


def _r6_shaped(tmp_path):
    vehicle = load_vehicle()
    nominal = Nominal.from_dict(_manifest(V01)["io"]["nominal"])
    weights = _init_weights(np.random.default_rng(4), 2.0)
    residual = Residual(
        True, weights, np.array([0.7, 0.0, 0.8, 0.0]), np.array([0.3, 0.15, 0.3, 0.15]),
        np.array([0.0, 0.2]),
    )  # fmt: skip
    out = tmp_path / MODEL_ID
    export.write_bundle(
        out, model_id=MODEL_ID, created_utc="2026-11-24T00:00:00Z", git_sha="0" * 40,
        training_config_sha256="0" * 64, vehicle=vehicle, nominal=nominal, residual=residual,
        v_max_mps=1.3, data=["test"], metrics={}, golden_residual_rollouts=True,
    )  # fmt: skip
    return out, vehicle


def _vehicle_dict(vehicle) -> dict:
    return {
        "wheelbase_m": vehicle.wheelbase_m,
        "steering_min_rad": vehicle.steer_min_rad,
        "steering_max_rad": vehicle.steer_max_rad,
        "steering_max_rate_rad_per_s": vehicle.steer_rate_max_rad_per_s,
    }


def _check_against_the_second_implementation(bundle_dir, vehicle) -> float:
    """Largest difference between the Python reference and make_fixture.step on the bundle's own
    golden command sequences, from the bundle's own float32 weights."""
    mk = _second_implementation()
    b = export.read_bundle(bundle_dir)
    w = np.concatenate([a.ravel() for a in b.residual.weights.arrays()]).astype(np.float64)
    res = (w, b.residual.input_mean, b.residual.input_std, b.residual.output_scale)
    nom = b.nominal.to_dict()
    veh = _vehicle_dict(vehicle)
    golden = export.read_golden(bundle_dir / export.GOLDEN_FILE)
    worst = 0.0
    for data in golden.values():
        controls = data[:-1, 0:2]
        ref = rollout(data[0, 2:7], controls, b.nominal, vehicle, b.residual)
        s = tuple(float(x) for x in data[0, 2:7])
        other = [s]
        for u in controls:
            s = mk.step(s, (float(u[0]), float(u[1])), nom, veh, 0.05, res)
            other.append(s)
        worst = max(worst, float(np.max(np.abs(np.asarray(other) - ref))))
    return worst


def test_an_r6_shaped_bundle_loads_in_both_implementations_within_1e_9(tmp_path) -> None:
    out, vehicle = _r6_shaped(tmp_path)
    m = _manifest(out / export.MANIFEST_FILE)
    export.validate_manifest(m)
    assert m["bundle_schema_version"].split(".")[0] == "1"
    assert m["io"]["residual"]["output_scale"][0] == 0.0
    assert m["io"]["nominal"] == _manifest(V01)["io"]["nominal"]
    assert m["io"]["validity"] == {"v_min_mps": 0.0, "v_max_mps": 1.3}
    b = export.read_bundle(out)
    assert export.golden_matches(
        export.read_golden(out / export.GOLDEN_FILE), b.nominal, b.residual, vehicle
    )
    assert _check_against_the_second_implementation(out, vehicle) < 1e-9


def test_the_committed_pointer_manifest_is_r6_on_dyn_v01() -> None:
    path = POINTER / "manifest.yaml"
    if not path.is_file():
        pytest.skip("the pointer manifest is written by the held-out pass")
    m = _manifest(path)
    export.validate_manifest(m)
    assert m["model_id"] == MODEL_ID and m["kind"] == "dynamics"
    assert m["bundle_schema_version"].split(".")[0] == "1"
    assert m["io"]["residual"]["enabled"] is True
    assert m["io"]["residual"]["output_scale"][0] == 0.0
    v01 = _manifest(V01)
    assert m["io"]["nominal"] == v01["io"]["nominal"]
    assert m["io"]["validity"] == v01["io"]["validity"] == {"v_min_mps": 0.0, "v_max_mps": 1.3}
    live = load_vehicle()
    assert m["vehicle_params"] == {
        "schema_version": live.schema_version,
        "sysid_session_id": live.sysid_session_id,
    }
    metrics = json.loads((POINTER / "metrics.json").read_text())
    assert metrics["GP2g-R"] in ("PASS", "FAIL", "INSUFFICIENT")


def test_the_real_bundle_when_present_matches_the_second_implementation() -> None:
    from pathlib import Path

    for root in ("models", "models-candidate"):
        d = Path("/Users/cameronjim/code/car-data/pnp") / root / MODEL_ID
        if (d / export.MANIFEST_FILE).is_file():
            vehicle = load_vehicle()
            b = export.read_bundle(d)
            assert b.residual.enabled and b.residual.output_scale[0] == 0.0
            assert _check_against_the_second_implementation(d, vehicle) < 1e-9
            return
    pytest.skip("the R6 bundle is not on this machine")
