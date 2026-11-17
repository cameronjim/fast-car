"""Step contract 2 (bundle schema 2.0.0) manifests committed for the planner's tests (brief 43)."""

from __future__ import annotations

import copy
import importlib.util
import json
from pathlib import Path

import pytest
import yaml
from jsonschema import Draft202012Validator

PNP_DIR = Path(__file__).resolve().parent.parent
VALIDATOR = Draft202012Validator(json.loads((PNP_DIR / "bundle.schema.json").read_text()))
PLANNER_FIXTURE = (
    PNP_DIR.parents[1] / "ros_ws" / "src" / "racer_planner" / "test" / "fixtures" / "dyn_fixture_v2"
)
CONTRACT2 = [
    PNP_DIR / "fixtures" / "dyn-v0.4t-contract2-test-only",
    PNP_DIR / "fixtures" / "dyn-v0.4tl-contract2-test-only",
    PNP_DIR / "fixtures" / "dyn-car-2026-11-12-contract2-sim",
    PLANNER_FIXTURE,
]


def manifest(path: Path) -> dict:
    return yaml.safe_load((path / "manifest.yaml").read_text(encoding="utf-8"))


@pytest.mark.parametrize("path", CONTRACT2, ids=lambda p: p.name)
def test_contract_2_manifests_validate(path: Path) -> None:
    m = manifest(path)
    assert m["bundle_schema_version"] == "2.0.0"
    VALIDATOR.validate(m)


@pytest.mark.parametrize("path", CONTRACT2[:2], ids=lambda p: p.name)
def test_exported_candidates_are_marked_not_for_deployment(path: Path) -> None:
    readme = (path / "README.md").read_text(encoding="utf-8")
    assert "DO NOT DEPLOY" in readme.splitlines()[0]


def test_a_residual_block_is_not_valid_in_a_contract_2_manifest() -> None:
    m = copy.deepcopy(manifest(CONTRACT2[2]))
    m["io"]["residual"] = {"enabled": False}
    assert not VALIDATOR.is_valid(m)


def test_a_ground_block_is_not_valid_in_a_contract_1_manifest() -> None:
    m = copy.deepcopy(manifest(PNP_DIR / "fixtures" / "dyn-car-2026-11-12"))
    m["io"]["ground_motion"] = manifest(CONTRACT2[2])["io"]["ground_motion"]
    assert not VALIDATOR.is_valid(m)


def test_the_sim_bundle_is_what_the_derivation_script_writes(tmp_path: Path) -> None:
    spec = importlib.util.spec_from_file_location(
        "derive_contract2_sim", PNP_DIR / "derive_contract2_sim.py"
    )
    mod = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(mod)
    mod.derive(mod.SRC, tmp_path)
    for name in ("manifest.yaml", "golden_rollouts.csv", "ground_mlp.f32"):
        assert (tmp_path / name).read_bytes() == (mod.OUT / name).read_bytes(), name
    # table of 1, learned off: the contract 1 model, with the contract 1 golden rollouts
    m = manifest(mod.OUT)
    assert all(v == 1.0 for row in m["io"]["ground_motion"]["table"]["values"] for v in row)
    assert m["io"]["ground_motion"]["learned"]["enabled"] is False
