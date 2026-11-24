"""Bundle contract of the prototype dyn-v0.7p (invariant 3): the committed pointer manifest is a
valid schema 1.x dynamics manifest with the residual enabled on the unchanged dyn-v0.1 nominal,
for the live vehicle_params, inside the 1.3 m/s validity; and a bundle of that shape written by
the exporter round trips (golden rollouts) and refuses a changed payload."""

from __future__ import annotations

import numpy as np
import pytest
import yaml
from racer_pnp_dynamics import export, proto
from racer_pnp_dynamics.model import Nominal, Residual
from racer_pnp_dynamics.residual_train import _init_weights, cap_weights
from racer_pnp_dynamics.vehicle import load_vehicle, repo_root

ROOT = repo_root()
POINTER = ROOT / "training" / "pnp_models" / proto.MODEL_ID / "manifest.yaml"
V01 = ROOT / "training" / "pnp_models" / "dyn-v0.1-2026-11-12" / "manifest.yaml"


def _manifest(path):
    return yaml.safe_load(path.read_text(encoding="utf-8"))


def test_pointer_manifest_is_a_valid_enabled_residual_on_dyn_v01() -> None:
    m = _manifest(POINTER)
    export.validate_manifest(m)
    assert m["kind"] == "dynamics" and m["model_id"] == proto.MODEL_ID
    assert m["bundle_schema_version"].split(".")[0] == "1"
    assert m["io"]["residual"]["enabled"] is True
    live = load_vehicle()
    assert m["vehicle_params"] == {
        "schema_version": live.schema_version,
        "sysid_session_id": live.sysid_session_id,
    }
    v01 = _manifest(V01)
    assert m["io"]["nominal"] == v01["io"]["nominal"]
    assert m["io"]["validity"] == v01["io"]["validity"] == {"v_min_mps": 0.0, "v_max_mps": 1.3}
    files = {p["file"] for p in m["payload"]}
    assert files == {m["io"]["residual"]["file"], m["io"]["golden"]}


def test_a_bundle_of_this_shape_round_trips_and_refuses_a_changed_payload(tmp_path) -> None:
    vehicle = load_vehicle()
    nominal = Nominal.from_dict(_manifest(V01)["io"]["nominal"])
    scale = np.array([0.6, 0.25])
    weights = cap_weights(
        _init_weights(np.random.default_rng(0), 3.0), np.array([2.0, 0.5]) / scale
    )
    residual = Residual(True, weights, np.array([0.8, 0.0, 0.9, 0.0]), np.ones(4) * 0.4, scale)
    out = tmp_path / proto.MODEL_ID
    export.write_bundle(
        out, model_id=proto.MODEL_ID, created_utc="2026-11-24T00:00:00Z", git_sha="0" * 40,
        training_config_sha256="0" * 64, vehicle=vehicle, nominal=nominal, residual=residual,
        v_max_mps=1.3, data=["test"], metrics={}, golden_residual_rollouts=True,
    )  # fmt: skip
    b = export.read_bundle(out)
    assert b.residual.enabled and b.v_max_mps == 1.3
    assert export.golden_matches(
        export.read_golden(out / export.GOLDEN_FILE), b.nominal, b.residual, vehicle
    )
    blob = bytearray((out / export.RESIDUAL_FILE).read_bytes())
    blob[0] ^= 0xFF
    (out / export.RESIDUAL_FILE).write_bytes(bytes(blob))
    with pytest.raises(ValueError, match="checksum"):
        export.read_bundle(out)
