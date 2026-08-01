"""Tests for `racer_policy.contract.load_contract` (claude-docs/12-testing.md L1):

- the happy path (a fully valid fixture contract loads)
- bad checksum (mismatch class: bad checksum)
- contract_version major mismatch (mismatch class: schema version)
- a few "cannot even parse" refusals that ContractManifestError covers generically
"""

from __future__ import annotations

from pathlib import Path
from typing import Any

import pytest
import yaml
from conftest import write_contract_dir
from racer_policy.contract import Contract, load_contract
from racer_policy.errors import (
    ChecksumMismatchError,
    ContractManifestError,
    ContractVersionMismatchError,
)


def test_happy_path_loads_a_fully_valid_contract(valid_contract_dir: Path) -> None:
    contract = load_contract(valid_contract_dir)

    assert isinstance(contract, Contract)
    assert contract.contract_version == "1.0.0"
    assert contract.policy.filename == "policy.pt"
    assert [f.name for f in contract.observation_schema.fields] == [
        "velocity_mps",
        "yaw_rate_rad_s",
        "slip_proxy",
        "raceline_lateral_error_m",
        "raceline_heading_error_rad",
    ]
    assert contract.observation_schema.lidar.beam_count == 108
    assert contract.action_space.scaling == "linear"
    assert contract.vehicle_params.schema_version == "0.1.0"
    assert contract.policy_path == valid_contract_dir / "policy.pt"


# --- mismatch class: bad checksum -------------------------------------------------------


def test_refuses_on_bad_checksum(tmp_path: Path, valid_manifest: dict[str, Any]) -> None:
    directory = write_contract_dir(
        tmp_path,
        valid_manifest,
        checksum_override="f" * 64,  # sha256 hex, but not the real hash of the bytes below
    )

    with pytest.raises(ChecksumMismatchError, match="checksum mismatch"):
        load_contract(directory)


def test_correct_checksum_is_accepted(tmp_path: Path, valid_manifest: dict[str, Any]) -> None:
    directory = write_contract_dir(tmp_path, valid_manifest, policy_bytes=b"some other bytes")
    load_contract(directory)  # must not raise


# --- mismatch class: schema version (contract_version major mismatch) --------------------


def test_refuses_on_contract_version_major_mismatch(
    tmp_path: Path, valid_manifest: dict[str, Any]
) -> None:
    valid_manifest["contract_version"] = "2.0.0"
    directory = write_contract_dir(tmp_path, valid_manifest)

    with pytest.raises(ContractVersionMismatchError, match="major version"):
        load_contract(directory)


def test_matching_major_with_different_minor_patch_is_accepted(
    tmp_path: Path, valid_manifest: dict[str, Any]
) -> None:
    valid_manifest["contract_version"] = "1.9.3"
    directory = write_contract_dir(tmp_path, valid_manifest)

    load_contract(directory)  # must not raise


# --- generic manifest refusals ------------------------------------------------------------


def test_refuses_when_manifest_file_is_missing(tmp_path: Path) -> None:
    directory = tmp_path / "empty_contract"
    directory.mkdir()

    with pytest.raises(ContractManifestError, match="no contract manifest found"):
        load_contract(directory)


def test_refuses_on_unparsable_yaml(tmp_path: Path) -> None:
    directory = tmp_path / "contract"
    directory.mkdir()
    (directory / "contract.yaml").write_text("this: [is not, valid: yaml", encoding="utf-8")

    with pytest.raises(ContractManifestError, match="not valid YAML"):
        load_contract(directory)


def test_refuses_when_manifest_is_not_a_mapping(tmp_path: Path) -> None:
    directory = tmp_path / "contract"
    directory.mkdir()
    (directory / "contract.yaml").write_text("- just\n- a\n- list\n", encoding="utf-8")

    with pytest.raises(ContractManifestError, match="YAML mapping"):
        load_contract(directory)


def test_refuses_when_policy_artifact_file_is_missing(
    tmp_path: Path, valid_manifest: dict[str, Any]
) -> None:
    directory = tmp_path / "contract"
    directory.mkdir()
    valid_manifest["policy"]["checksum_sha256"] = "0" * 64
    (directory / "contract.yaml").write_text(
        yaml.safe_dump(valid_manifest, sort_keys=False), encoding="utf-8"
    )
    # deliberately never write policy.pt

    with pytest.raises(ContractManifestError, match="does not exist"):
        load_contract(directory)
