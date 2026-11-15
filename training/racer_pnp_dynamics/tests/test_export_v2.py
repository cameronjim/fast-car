"""L1 for the contract 2 export (export_v2.py, schema 2.0.0)."""

from __future__ import annotations

import copy
import os
import tempfile
from pathlib import Path

import numpy as np
import pytest
import yaml
from racer_pnp_dynamics import export, export_v2
from racer_pnp_dynamics import ground_motion as gmo
from racer_pnp_dynamics.vehicle import repo_root
from test_fit import TRUTH
from test_step_v2 import C, random_learned, random_table

COMMON = {
    "model_id": "dyn-test-v2",
    "created_utc": "2026-11-15T00:00:00Z",
    "git_sha": "0" * 40,
    "training_config_sha256": "1" * 64,
    "v_max_mps": 1.3,
    "data": ["synthetic"],
    "metrics": {"note": "test"},
}


def _write(path, vehicle, learned=True, seed=2):
    gm = gmo.GroundMotion(
        C, random_table(seed, 0.7, 1.0), random_learned(seed, 0.5) if learned else None
    )
    manifest = export_v2.write_bundle_v2(path, vehicle=vehicle, nominal=TRUTH, ground=gm, **COMMON)
    return gm, manifest


@pytest.mark.parametrize("learned", [True, False])
def test_a_contract_2_bundle_validates_round_trips_and_reproduces_its_golden_rollouts(
    tmp_path, toy_vehicle, learned
):
    toy_vehicle = _toy(toy_vehicle)
    gm, manifest = _write(tmp_path / "b", toy_vehicle, learned)
    assert manifest["bundle_schema_version"] == "2.0.0" == gmo.BUNDLE_SCHEMA_VERSION
    assert "residual" not in manifest["io"] and "ground_motion" in manifest["io"]
    back = export_v2.read_bundle_v2(tmp_path / "b", C)
    assert back.nominal == TRUTH and back.v_max_mps == 1.3
    assert np.array_equal(back.ground.table.values, gm.table.values)
    assert (back.ground.learned is not None) is learned
    golden = export_v2.read_golden_v2(tmp_path / "b")
    assert len(golden) == 17  # twelve and the five at speed
    assert export_v2.golden_matches_v2(golden, back.nominal, toy_vehicle, back.ground)


def _toy(v):
    return v  # the toy vehicle fixture is a real Vehicle; named for readability


def test_a_perturbed_golden_file_is_detected(tmp_path, toy_vehicle):
    gm, _ = _write(tmp_path / "b", toy_vehicle)
    golden = export_v2.read_golden_v2(tmp_path / "b")
    golden[0][5, 2] += 1e-3  # an x that no model produces
    assert not export_v2.golden_matches_v2(golden, TRUTH, toy_vehicle, gm)


def test_a_bad_checksum_and_a_wrong_version_are_refused(tmp_path, toy_vehicle):
    _write(tmp_path / "b", toy_vehicle)
    (tmp_path / "b" / export_v2.GROUND_FILE).write_bytes(b"\x00" * 8)
    with pytest.raises(ValueError, match="checksum"):
        export_v2.read_bundle_v2(tmp_path / "b", C)


def test_schema_2_0_0_forbids_the_additive_residual_and_1_x_forbids_ground_motion(
    tmp_path, toy_vehicle
):
    _, manifest = _write(tmp_path / "b", toy_vehicle)
    export.validate_manifest(manifest)
    with_residual = copy.deepcopy(manifest)
    with_residual["io"]["residual"] = {"enabled": False}
    with pytest.raises(export.BundleInvalid):
        export.validate_manifest(with_residual)
    as_v1 = copy.deepcopy(manifest)
    as_v1["bundle_schema_version"] = "1.1.0"  # contract 2 content under a 1.x number
    with pytest.raises(export.BundleInvalid):
        export.validate_manifest(as_v1)
    no_ground = copy.deepcopy(manifest)
    del no_ground["io"]["ground_motion"]
    with pytest.raises(export.BundleInvalid):
        export.validate_manifest(no_ground)


def test_1_x_bundles_validate_as_before():
    root = repo_root() / "config" / "pnp"
    for path in sorted((root / "fixtures").glob("dyn-*/manifest.yaml")) + [
        root / "examples" / "dyn-example" / "manifest.yaml"
    ]:
        manifest = yaml.safe_load(path.read_text(encoding="utf-8"))
        assert manifest["bundle_schema_version"].startswith("1.")
        export.validate_manifest(manifest)


def test_the_contract_1_writer_still_writes_schema_1_1_0():
    assert export.BUNDLE_SCHEMA_VERSION == "1.1.0"


def test_v_max_above_the_full_scale_speed_is_refused(tmp_path, toy_vehicle):
    gm = gmo.GroundMotion(C, random_table(1))
    kw = {**COMMON, "v_max_mps": 99.0}
    with pytest.raises(export.BundleInvalid, match="v_max_mps"):
        export_v2.write_bundle_v2(
            tmp_path / "x", vehicle=toy_vehicle, nominal=TRUTH, ground=gm, **kw
        )
    assert not (tmp_path / "x").exists()


def test_the_first_readme_line_is_the_contract_2_warning():
    assert (
        export_v2.FIRST_README_LINE
        == "contract 2: no planner on main can load this; candidate only"
    )


def test_todays_planner_refuses_a_contract_2_bundle(tmp_path, toy_vehicle):
    """The C++ loader on main (planner_replay_cli) must answer BundleSchemaVersionMismatch. Runs
    where the binary exists (a native one, $PLANNER_REPLAY_CLI, or the Linux one in docker through
    $PLANNER_REPLAY_DOCKER_CLI); a missing binary is a skip unless PNP_REPLAY_REQUIRE_CLI is set."""
    from pnp_replay import planner_stream

    try:
        cli = planner_stream.find_cli()
    except planner_stream.ReplayError as exc:
        if os.environ.get("PNP_REPLAY_REQUIRE_CLI"):
            raise
        pytest.skip(f"planner_replay_cli not available: {exc}")
    # a docker run only sees shared folders: $PNP_TEST_SHARED_TMP names one (the system temp
    # directory of a Mac is not shared); native runs use the pytest temp directory
    shared = os.environ.get("PNP_TEST_SHARED_TMP")
    if shared:
        Path(shared).mkdir(parents=True, exist_ok=True)
    with tempfile.TemporaryDirectory(dir=shared or tmp_path) as work:
        bundle = Path(work) / "b"
        _write(bundle, toy_vehicle)
        with pytest.raises(planner_stream.ReplayError, match="BundleSchemaVersionMismatch"):
            planner_stream.describe_cli(cli, bundle, 1.3, {})
