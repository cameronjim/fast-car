"""The dyn-v0.2 fixture generator, and the committed dyn-car-2026-11-13* fixtures when they exist
(they are committed only if the refit shipped, and the residual one only if GP2a passed)."""

from __future__ import annotations

import numpy as np
import pytest
import yaml
from racer_pnp_dynamics import export
from racer_pnp_dynamics import make_v02_fixtures as mk
from racer_pnp_dynamics.model import MlpWeights, Residual, zero_weights
from racer_pnp_dynamics.vehicle import load_vehicle, repo_root
from test_fit import TRUTH

FIXTURES = repo_root() / mk.FIXTURES_RELATIVE
COMMITTED = [FIXTURES / mk.NOMINAL_ID, FIXTURES / mk.RESIDUAL_ID]
COMMITTED = [p for p in COMMITTED if p.is_dir()]


def _source_bundle(tmp_path, enabled: bool):
    """A stand-in for an exported dyn-v0.2 bundle (the real one is under car-data)."""
    vehicle = load_vehicle()
    rng = np.random.default_rng(0)
    w = MlpWeights(*(rng.normal(0, 0.3, a.shape) for a in zero_weights().arrays()))
    res = Residual(
        enabled, w, np.array([0.5, 0.0, 0.5, 0.0]), np.ones(4) * 0.3, np.array([0.2, 0.1])
    )
    nominal = TRUTH.replace(speed_deadband_mps=0.2, command_delay_s=0.04)
    export.write_bundle(
        tmp_path / "src", model_id="dyn-v0.2-test", created_utc="2026-11-13T01:00:00Z",
        git_sha="a" * 40, training_config_sha256="b" * 64, vehicle=vehicle, nominal=nominal,
        residual=res, v_max_mps=1.3, data=["x"], metrics={"big": list(range(10))},
        golden_residual_rollouts=enabled,
    )  # fmt: skip
    return tmp_path / "src"


def test_generator_writes_the_nominal_fixture_alone_when_the_residual_is_off(tmp_path):
    out = mk.make_fixtures(tmp_path / "root", load_vehicle(), _source_bundle(tmp_path, False))
    assert [p.name for p in out] == [mk.NOMINAL_ID]
    b = export.read_bundle(out[0])
    assert not b.residual.enabled and not b.residual.weights.w1.any()
    assert b.manifest["metrics"] == {} and b.manifest["git_sha"] == "0" * 40
    assert b.nominal.speed_deadband_mps == 0.2 and b.v_max_mps == 1.3
    assert len(export.read_golden(out[0] / "golden_rollouts.csv")) == 12


def test_generator_writes_a_second_fixture_with_the_residual_enabled(tmp_path):
    out = mk.make_fixtures(tmp_path / "root", load_vehicle(), _source_bundle(tmp_path, True))
    assert [p.name for p in out] == [mk.NOMINAL_ID, mk.RESIDUAL_ID]
    nominal, resid = (export.read_bundle(p) for p in out)
    assert not nominal.residual.enabled and resid.residual.enabled
    assert resid.nominal == nominal.nominal
    golden = export.read_golden(out[1] / "golden_rollouts.csv")
    assert len(golden) == 12 + 5  # the residual one carries the rollouts at speed
    vehicle = load_vehicle()
    assert export.golden_matches(golden, resid.nominal, resid.residual, vehicle)
    assert sum(p.stat().st_size for p in out[1].iterdir()) < 100_000


def test_regeneration_from_the_written_fixture_reproduces_every_payload_byte(tmp_path):
    first = mk.make_fixtures(tmp_path / "a", load_vehicle(), _source_bundle(tmp_path, True))
    again = mk.make_fixtures(tmp_path / "b", load_vehicle(), first[1])  # the residual fixture
    for x, y in zip(first, again, strict=True):
        for name in ("residual_mlp.f32", "golden_rollouts.csv"):
            assert (x / name).read_bytes() == (y / name).read_bytes(), (x.name, name)
        mx, my = (yaml.safe_load((d / "manifest.yaml").read_text()) for d in (x, y))
        assert mx["io"] == my["io"] and mx["payload"] == my["payload"]


def test_cli_writes_into_a_root(tmp_path):
    src = _source_bundle(tmp_path, False)
    assert mk.main(["--bundle", str(src), "--root", str(tmp_path / "r")]) == 0
    assert (tmp_path / "r" / mk.FIXTURES_RELATIVE / mk.NOMINAL_ID / "manifest.yaml").is_file()


@pytest.mark.parametrize("path", COMMITTED, ids=lambda p: p.name)
def test_committed_v02_fixture_is_valid_and_matches_the_live_vehicle_params(path):
    bundle = export.read_bundle(path)
    export.validate_manifest(bundle.manifest)
    live = yaml.safe_load((repo_root() / "config" / "vehicle_params.yaml").read_text())["meta"]
    assert bundle.manifest["vehicle_params"] == {
        "schema_version": live["schema_version"],
        "sysid_session_id": live["sysid_session_id"],
    }
    assert bundle.manifest["vehicle_params"]["schema_version"] == "0.17.0"
    assert bundle.manifest["vehicle_params"]["sysid_session_id"] == "none-preliminary"
    golden = export.read_golden(path / "golden_rollouts.csv")
    assert export.golden_matches(golden, bundle.nominal, bundle.residual, load_vehicle())
    assert sum(p.stat().st_size for p in path.iterdir()) < 100_000
    if path.name == mk.RESIDUAL_ID:
        assert bundle.residual.enabled and len(golden) >= 12 + 4
    else:
        assert not bundle.residual.enabled
    assert bundle.v_max_mps <= 1.5
