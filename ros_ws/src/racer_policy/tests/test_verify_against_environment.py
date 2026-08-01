"""Tests for `racer_policy.verify.verify_against_environment` (claude-docs/12-testing.md L1):
one test per live-environment mismatch class, plus the happy path where a loaded contract
matches the live environment exactly. Each mismatch test starts from
`matching_live_environment` (built field-for-field from the same `valid_manifest` the
contract is loaded from) and perturbs exactly one thing, so a failure can only be caused by
the one check under test.
"""

from __future__ import annotations

import dataclasses
from pathlib import Path
from typing import Any

import pytest
from conftest import live_environment_matching, write_contract_dir
from racer_policy.contract import ObservationField, load_contract
from racer_policy.errors import (
    ExtraObservationFieldError,
    MissingObservationFieldError,
    ObservationDtypeMismatchError,
    ObservationSchemaMismatchError,
    ObservationUnitsMismatchError,
    VehicleParamsVersionMismatchError,
)
from racer_policy.verify import verify_against_environment


def test_happy_path_matching_environment_passes(
    valid_contract_dir: Path, matching_live_environment: Any
) -> None:
    contract = load_contract(valid_contract_dir)
    assert verify_against_environment(contract, matching_live_environment) is None


# --- mismatch class: params version -------------------------------------------------------


def test_refuses_on_vehicle_params_schema_version_mismatch(
    tmp_path: Path, valid_manifest: dict[str, Any]
) -> None:
    live = live_environment_matching(valid_manifest)
    directory = write_contract_dir(tmp_path, valid_manifest)
    contract = load_contract(directory)

    live.vehicle_params.meta.schema_version = "9.9.9"

    with pytest.raises(VehicleParamsVersionMismatchError, match="schema_version"):
        verify_against_environment(contract, live)


def test_refuses_on_vehicle_params_sysid_session_mismatch(
    tmp_path: Path, valid_manifest: dict[str, Any]
) -> None:
    live = live_environment_matching(valid_manifest)
    directory = write_contract_dir(tmp_path, valid_manifest)
    contract = load_contract(directory)

    live.vehicle_params.meta.sysid_session_id = "some-other-session"

    with pytest.raises(VehicleParamsVersionMismatchError, match="sysid_session_id"):
        verify_against_environment(contract, live)


# --- mismatch class: missing field ---------------------------------------------------------


def test_refuses_when_live_environment_is_missing_a_contract_field(
    tmp_path: Path, valid_manifest: dict[str, Any]
) -> None:
    live = live_environment_matching(valid_manifest)
    directory = write_contract_dir(tmp_path, valid_manifest)
    contract = load_contract(directory)

    live = dataclasses.replace(
        live,
        observation=dataclasses.replace(live.observation, fields=live.observation.fields[:-1]),
    )

    with pytest.raises(MissingObservationFieldError, match="raceline_heading_error_rad"):
        verify_against_environment(contract, live)


# --- mismatch class: extra field -------------------------------------------------------------


def test_refuses_when_live_environment_publishes_an_undeclared_field(
    tmp_path: Path, valid_manifest: dict[str, Any]
) -> None:
    live = live_environment_matching(valid_manifest)
    directory = write_contract_dir(tmp_path, valid_manifest)
    contract = load_contract(directory)

    extra = ObservationField(name="mystery_field", dtype="float32", units="m")
    live = dataclasses.replace(
        live,
        observation=dataclasses.replace(
            live.observation, fields=live.observation.fields + (extra,)
        ),
    )

    with pytest.raises(ExtraObservationFieldError, match="mystery_field"):
        verify_against_environment(contract, live)


# --- mismatch class: obs schema (field order / LiDAR shape) -----------------------------------


def test_refuses_on_observation_field_order_mismatch(
    tmp_path: Path, valid_manifest: dict[str, Any]
) -> None:
    live = live_environment_matching(valid_manifest)
    directory = write_contract_dir(tmp_path, valid_manifest)
    contract = load_contract(directory)

    reordered = tuple(reversed(live.observation.fields))
    live = dataclasses.replace(
        live, observation=dataclasses.replace(live.observation, fields=reordered)
    )

    with pytest.raises(ObservationSchemaMismatchError, match="order"):
        verify_against_environment(contract, live)


def test_refuses_on_lidar_beam_count_mismatch(
    tmp_path: Path, valid_manifest: dict[str, Any]
) -> None:
    live = live_environment_matching(valid_manifest)
    directory = write_contract_dir(tmp_path, valid_manifest)
    contract = load_contract(directory)

    live = dataclasses.replace(
        live, observation=dataclasses.replace(live.observation, lidar_beam_count=42)
    )

    with pytest.raises(ObservationSchemaMismatchError, match="beam_count"):
        verify_against_environment(contract, live)


def test_refuses_on_lidar_fov_mismatch(tmp_path: Path, valid_manifest: dict[str, Any]) -> None:
    live = live_environment_matching(valid_manifest)
    directory = write_contract_dir(tmp_path, valid_manifest)
    contract = load_contract(directory)

    live = dataclasses.replace(
        live, observation=dataclasses.replace(live.observation, lidar_fov_rad=1.0)
    )

    with pytest.raises(ObservationSchemaMismatchError, match="fov_rad"):
        verify_against_environment(contract, live)


def test_refuses_on_lidar_downsample_factor_mismatch(
    tmp_path: Path, valid_manifest: dict[str, Any]
) -> None:
    live = live_environment_matching(valid_manifest)
    directory = write_contract_dir(tmp_path, valid_manifest)
    contract = load_contract(directory)

    live = dataclasses.replace(
        live, observation=dataclasses.replace(live.observation, lidar_downsample_factor=1)
    )

    with pytest.raises(ObservationSchemaMismatchError, match="downsample_factor"):
        verify_against_environment(contract, live)


# --- mismatch class: wrong dtype ---------------------------------------------------------------


def test_refuses_on_observation_field_dtype_mismatch(
    tmp_path: Path, valid_manifest: dict[str, Any]
) -> None:
    live = live_environment_matching(valid_manifest)
    directory = write_contract_dir(tmp_path, valid_manifest)
    contract = load_contract(directory)

    fields = list(live.observation.fields)
    fields[0] = dataclasses.replace(fields[0], dtype="float64")
    live = dataclasses.replace(
        live, observation=dataclasses.replace(live.observation, fields=tuple(fields))
    )

    with pytest.raises(ObservationDtypeMismatchError, match="velocity_mps"):
        verify_against_environment(contract, live)


# --- mismatch class: wrong units string --------------------------------------------------------


def test_refuses_on_observation_field_units_mismatch(
    tmp_path: Path, valid_manifest: dict[str, Any]
) -> None:
    live = live_environment_matching(valid_manifest)
    directory = write_contract_dir(tmp_path, valid_manifest)
    contract = load_contract(directory)

    fields = list(live.observation.fields)
    fields[0] = dataclasses.replace(fields[0], units="km/h")
    live = dataclasses.replace(
        live, observation=dataclasses.replace(live.observation, fields=tuple(fields))
    )

    with pytest.raises(ObservationUnitsMismatchError, match="velocity_mps"):
        verify_against_environment(contract, live)
