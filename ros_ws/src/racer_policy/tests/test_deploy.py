"""deploy.load_deployment: the loader accepts a consistent directory and refuses every
mismatch class, including the ones only the sidecar can express. One refusal per test, each
starting from the fully valid fixture and breaking exactly one thing (claude-docs/12-testing.md
L1: one test per mismatch class). Includes the v0-style case: a directory trained against an
older vehicle_params version, which is what the v0 policy now is."""

from __future__ import annotations

from collections.abc import Callable
from pathlib import Path
from typing import Any

import pytest
from policy_fixture import build_policy_dir
from racer_policy import (
    ChecksumMismatchError,
    ContractError,
    ContractManifestError,
    ContractVersionMismatchError,
    ExtraObservationFieldError,
    MissingObservationFieldError,
    ObservationDtypeMismatchError,
    ObservationSchemaMismatchError,
    ObservationUnitsMismatchError,
    VehicleParamsVersionMismatchError,
)
from racer_policy.deploy import (
    PolicyMetaError,
    build_envelope_config,
    load_deployment,
)

Mutator = Callable[[dict[str, Any]], None]


def _ok(tmp_path: Path, vp: Any, **kw: Any) -> Any:
    return load_deployment(build_policy_dir(tmp_path, vp, **kw), vp)


def test_accepts_a_consistent_directory(tmp_path: Path, real_vehicle_params: Any) -> None:
    dep = _ok(tmp_path, real_vehicle_params)
    assert dep.steering_scale_rad == pytest.approx(0.0222)
    assert dep.envelope.speed_cap_mps == 2.0
    assert dep.base_controller.lookahead_max_m == 1.2


def test_envelope_config_is_built_from_the_directory_not_from_parameters(
    tmp_path: Path, real_vehicle_params: Any
) -> None:
    dep = _ok(tmp_path, real_vehicle_params)
    cfg = build_envelope_config(dep, real_vehicle_params)
    assert cfg.residual_fraction_steering == 0.025
    assert cfg.residual_fraction_speed == 0.004
    assert cfg.speed_cap_mps == 2.0
    assert cfg.max_delta_steering_rad == pytest.approx(
        real_vehicle_params.steering.max_rate_rad_per_s / 50.0
    )
    assert cfg.steering_range == (
        real_vehicle_params.steering.min_angle_rad,
        real_vehicle_params.steering.max_angle_rad,
    )


# -- contract-level refusals (the loader's own classes, through the deploy entry point) --------


def test_refuses_a_flipped_checksum(tmp_path: Path, real_vehicle_params: Any) -> None:
    d = build_policy_dir(tmp_path, real_vehicle_params)
    (d / "policy.pt").write_bytes(b"tampered")
    with pytest.raises(ChecksumMismatchError):
        load_deployment(d, real_vehicle_params)


def test_refuses_a_future_contract_version(tmp_path: Path, real_vehicle_params: Any) -> None:
    def m(c: dict[str, Any]) -> None:
        c["contract_version"] = "2.0.0"

    with pytest.raises(ContractVersionMismatchError):
        _ok(tmp_path, real_vehicle_params, mutate_contract=m)


def test_refuses_a_malformed_manifest(tmp_path: Path, real_vehicle_params: Any) -> None:
    def m(c: dict[str, Any]) -> None:
        del c["action_space"]

    with pytest.raises(ContractManifestError):
        _ok(tmp_path, real_vehicle_params, mutate_contract=m)


@pytest.mark.parametrize("version", ["0.13.0", "0.14.1", "9.9.9"])
def test_refuses_another_vehicle_params_version(
    tmp_path: Path, real_vehicle_params: Any, version: str
) -> None:
    def m(c: dict[str, Any]) -> None:
        c["vehicle_params"]["schema_version"] = version

    with pytest.raises(VehicleParamsVersionMismatchError):
        _ok(tmp_path, real_vehicle_params, mutate_contract=m)


def test_refuses_another_sysid_session(tmp_path: Path, real_vehicle_params: Any) -> None:
    def m(c: dict[str, Any]) -> None:
        c["vehicle_params"]["sysid_session_id"] = "2026-12-01-fit"

    with pytest.raises(VehicleParamsVersionMismatchError):
        _ok(tmp_path, real_vehicle_params, mutate_contract=m)


def test_refuses_a_missing_observation_field(tmp_path: Path, real_vehicle_params: Any) -> None:
    def m(c: dict[str, Any]) -> None:
        c["observation_schema"]["fields"][2]["name"] = "something_else"

    with pytest.raises((MissingObservationFieldError, ExtraObservationFieldError)):
        _ok(tmp_path, real_vehicle_params, mutate_contract=m)


def test_refuses_swapped_field_order(tmp_path: Path, real_vehicle_params: Any) -> None:
    def m(c: dict[str, Any]) -> None:
        f = c["observation_schema"]["fields"]
        f[0], f[1] = f[1], f[0]

    with pytest.raises(ObservationSchemaMismatchError):
        _ok(tmp_path, real_vehicle_params, mutate_contract=m)


def test_refuses_wrong_field_units(tmp_path: Path, real_vehicle_params: Any) -> None:
    def m(c: dict[str, Any]) -> None:
        c["observation_schema"]["fields"][0]["units"] = "km/h"

    with pytest.raises(ObservationUnitsMismatchError):
        _ok(tmp_path, real_vehicle_params, mutate_contract=m)


def test_refuses_wrong_field_dtype(tmp_path: Path, real_vehicle_params: Any) -> None:
    def m(c: dict[str, Any]) -> None:
        c["observation_schema"]["fields"][0]["dtype"] = "float64"

    with pytest.raises(ObservationDtypeMismatchError):
        _ok(tmp_path, real_vehicle_params, mutate_contract=m)


@pytest.mark.parametrize(
    ("key", "value"), [("beam_count", 108), ("fov_rad", 4.712), ("downsample_factor", 10)]
)
def test_refuses_a_different_lidar_block(
    tmp_path: Path, real_vehicle_params: Any, key: str, value: float
) -> None:
    def m(c: dict[str, Any]) -> None:
        c["observation_schema"]["lidar"][key] = value
        if key == "beam_count":
            c["normalization"]["mean"] = [0.0] * (11 + value)
            c["normalization"]["std"] = [1.0] * (11 + value)

    with pytest.raises(ObservationSchemaMismatchError):
        _ok(tmp_path, real_vehicle_params, mutate_contract=m)


# -- sidecar refusals (what the contract schema cannot hold) -----------------------------------


def test_refuses_a_missing_sidecar(tmp_path: Path, real_vehicle_params: Any) -> None:
    with pytest.raises(PolicyMetaError, match="missing"):
        _ok(tmp_path, real_vehicle_params, write_meta=False)


def test_refuses_an_unparseable_sidecar(tmp_path: Path, real_vehicle_params: Any) -> None:
    d = build_policy_dir(tmp_path, real_vehicle_params)
    (d / "policy_meta.yaml").write_text("- a\n- list\n", encoding="utf-8")
    with pytest.raises(PolicyMetaError):
        load_deployment(d, real_vehicle_params)


def test_refuses_a_sidecar_missing_the_deploy_section(
    tmp_path: Path, real_vehicle_params: Any
) -> None:
    def m(meta: dict[str, Any]) -> None:
        del meta["must_match_at_deploy"]["envelope"]

    with pytest.raises(PolicyMetaError, match="envelope"):
        _ok(tmp_path, real_vehicle_params, mutate_meta=m)


def test_refuses_when_sidecar_fractions_disagree_with_the_contract(
    tmp_path: Path, real_vehicle_params: Any
) -> None:
    def m(meta: dict[str, Any]) -> None:
        meta["must_match_at_deploy"]["envelope"]["residual_fraction_steering"] = 0.5

    with pytest.raises(PolicyMetaError, match="fractions"):
        _ok(tmp_path, real_vehicle_params, mutate_meta=m)


def test_refuses_when_action_scales_disagree_with_the_contract(
    tmp_path: Path, real_vehicle_params: Any
) -> None:
    def m(meta: dict[str, Any]) -> None:
        meta["must_match_at_deploy"]["action_scale"]["speed_mps_per_unit"] = 1.0

    with pytest.raises(PolicyMetaError, match="action scales"):
        _ok(tmp_path, real_vehicle_params, mutate_meta=m)


def test_refuses_another_control_rate(tmp_path: Path, real_vehicle_params: Any) -> None:
    def m(meta: dict[str, Any]) -> None:
        meta["must_match_at_deploy"]["envelope"]["control_rate_hz"] = 20.0

    with pytest.raises(PolicyMetaError, match="Hz"):
        _ok(tmp_path, real_vehicle_params, mutate_meta=m)


def test_refuses_an_unknown_observation_profile(tmp_path: Path, real_vehicle_params: Any) -> None:
    def m(meta: dict[str, Any]) -> None:
        meta["must_match_at_deploy"]["observation"]["profile"] = "v7"

    with pytest.raises(PolicyMetaError, match="profile"):
        _ok(tmp_path, real_vehicle_params, mutate_meta=m)


def test_refuses_a_sidecar_lidar_block_that_disagrees(
    tmp_path: Path, real_vehicle_params: Any
) -> None:
    def m(meta: dict[str, Any]) -> None:
        meta["must_match_at_deploy"]["observation"]["lidar_min_pool"] = False

    with pytest.raises(PolicyMetaError, match="observation block"):
        _ok(tmp_path, real_vehicle_params, mutate_meta=m)


def test_refuses_a_non_identity_normalization_block(
    tmp_path: Path, real_vehicle_params: Any
) -> None:
    def m(c: dict[str, Any]) -> None:
        c["normalization"]["std"][3] = 2.0

    with pytest.raises(PolicyMetaError, match="normalization"):
        _ok(tmp_path, real_vehicle_params, mutate_contract=m)


def test_refuses_when_normalization_is_not_baked_into_the_policy(
    tmp_path: Path, real_vehicle_params: Any
) -> None:
    def m(meta: dict[str, Any]) -> None:
        meta["observation_normalization"]["applied_inside_policy_pt"] = False

    with pytest.raises(PolicyMetaError, match="applied_inside_policy_pt"):
        _ok(tmp_path, real_vehicle_params, mutate_meta=m)


def test_refuses_a_policy_trained_with_in_memory_overrides(
    tmp_path: Path, real_vehicle_params: Any
) -> None:
    def m(meta: dict[str, Any]) -> None:
        meta["vehicle_overrides_assumed"] = {"steering": {"max_angle_rad": 0.504}}

    with pytest.raises(PolicyMetaError, match="overrides"):
        _ok(tmp_path, real_vehicle_params, mutate_meta=m)


def test_refuses_actuator_assumptions_that_differ_from_the_live_params(
    tmp_path: Path, real_vehicle_params: Any
) -> None:
    def m(c: dict[str, Any]) -> None:
        c["actuator_assumptions"]["rate_limit_speed_mps_s"] = 3.0

    with pytest.raises(PolicyMetaError, match="actuator"):
        _ok(tmp_path, real_vehicle_params, mutate_contract=m)


def test_refuses_a_speed_cap_above_the_vehicle_cap(
    tmp_path: Path, real_vehicle_params: Any
) -> None:
    def m(meta: dict[str, Any]) -> None:
        meta["must_match_at_deploy"]["envelope"]["speed_cap_mps"] = 999.0

    with pytest.raises(PolicyMetaError, match="speed cap"):
        _ok(tmp_path, real_vehicle_params, mutate_contract=None, mutate_meta=m)


def test_every_refusal_is_a_contract_error(tmp_path: Path, real_vehicle_params: Any) -> None:
    """Callers (the node, the app check) catch ContractError once; the sidecar class must be
    part of that family, never a bare ValueError that could be swallowed elsewhere."""
    assert issubclass(PolicyMetaError, ContractError)


def test_the_committed_v0_policy_is_refused_against_the_current_vehicle_params(
    tmp_path: Path, real_vehicle_params: Any
) -> None:
    """The real v0 contract and sidecar text (training/policies/v0-2026-11-11, the binary is not
    committed) with a stand-in policy.pt whose checksum is patched in, so the ONLY thing left
    wrong is what v0 actually has wrong: it was trained against vehicle_params 0.13.0. Since
    vehicle_params 0.14.0 landed the loader must refuse it, and it must say why."""
    import hashlib

    import yaml

    committed = Path(__file__).resolve().parents[4] / "training" / "policies" / "v0-2026-11-11"
    if not (committed / "contract.yaml").is_file():
        pytest.skip("v0 pointer directory not present in this checkout")
    d = tmp_path / "v0"
    d.mkdir()
    (d / "policy.pt").write_bytes(b"stand-in for the uncommitted v0 policy.pt")
    contract = yaml.safe_load((committed / "contract.yaml").read_text(encoding="utf-8"))
    contract["policy"]["checksum_sha256"] = hashlib.sha256(
        (d / "policy.pt").read_bytes()
    ).hexdigest()
    (d / "contract.yaml").write_text(yaml.safe_dump(contract, sort_keys=False), encoding="utf-8")
    (d / "policy_meta.yaml").write_text(
        (committed / "policy_meta.yaml").read_text(encoding="utf-8"), encoding="utf-8"
    )
    assert contract["vehicle_params"]["schema_version"] == "0.13.0"
    with pytest.raises(VehicleParamsVersionMismatchError, match="0.13.0"):
        load_deployment(d, real_vehicle_params)
