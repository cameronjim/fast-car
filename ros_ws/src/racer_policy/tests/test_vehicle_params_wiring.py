"""Integration test: `verify_against_environment`'s vehicle_params check against the REAL
generated binding of the committed config/vehicle_params.yaml (regenerated at test time via
tools/gen_params.py, never committed -- see conftest.py's `real_vehicle_params` fixture).
This is the proof that `racer_policy.environment.VehicleParamsLike` is actually satisfied by
what gen_params.py emits for `.meta`, not just by the hand-written `_VehicleParams` stub in
conftest.py's `live_environment_matching`. Mirrors
training/envelope/tests/test_from_vehicle_params_wiring.py.
"""

from __future__ import annotations

from pathlib import Path
from typing import Any

import pytest
from conftest import live_environment_matching, write_contract_dir
from racer_policy.contract import load_contract
from racer_policy.environment import LiveEnvironment
from racer_policy.errors import VehicleParamsVersionMismatchError
from racer_policy.verify import verify_against_environment


def test_accepts_when_contract_matches_the_real_committed_params(
    tmp_path: Path, valid_manifest: dict[str, Any], real_vehicle_params: Any
) -> None:
    # config/vehicle_params.yaml currently has meta.schema_version = "0.1.0" and
    # meta.sysid_session_id = "none-preliminary" -- the same values `_TEMPLATE` in
    # conftest.py records under `vehicle_params`, by construction.
    assert (
        valid_manifest["vehicle_params"]["schema_version"]
        == real_vehicle_params.meta.schema_version
    )
    assert (
        valid_manifest["vehicle_params"]["sysid_session_id"]
        == real_vehicle_params.meta.sysid_session_id
    )

    live = live_environment_matching(valid_manifest)
    live = LiveEnvironment(vehicle_params=real_vehicle_params, observation=live.observation)
    directory = write_contract_dir(tmp_path, valid_manifest)
    contract = load_contract(directory)

    assert verify_against_environment(contract, live) is None


def test_refuses_when_contract_predates_the_real_committed_params(
    tmp_path: Path, valid_manifest: dict[str, Any], real_vehicle_params: Any
) -> None:
    valid_manifest["vehicle_params"]["sysid_session_id"] = "some-earlier-session"
    live = live_environment_matching(valid_manifest)
    live = LiveEnvironment(vehicle_params=real_vehicle_params, observation=live.observation)
    directory = write_contract_dir(tmp_path, valid_manifest)
    contract = load_contract(directory)

    with pytest.raises(VehicleParamsVersionMismatchError, match="sysid_session_id"):
        verify_against_environment(contract, live)
