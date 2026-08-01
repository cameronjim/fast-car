"""`verify_against_environment`: the dynamic half of the refuse-on-mismatch loader.

`racer_policy.contract.load_contract` proves a contract is internally well-formed
(schema-valid manifest, supported contract_version, correct policy checksum). This module
proves the loaded `Contract` actually matches the system it is about to be deployed onto:
the live `vehicle_params` and the live observation topics' field order/dtype/units/LiDAR
configuration. Every mismatch is a distinct, named exception from `racer_policy.errors`
(CLAUDE.md invariant 3: hard refusal, never a warning, no override).
"""

from __future__ import annotations

from racer_policy.contract import Contract
from racer_policy.environment import LiveEnvironment
from racer_policy.errors import (
    ExtraObservationFieldError,
    MissingObservationFieldError,
    ObservationDtypeMismatchError,
    ObservationSchemaMismatchError,
    ObservationUnitsMismatchError,
    VehicleParamsVersionMismatchError,
)


def _verify_vehicle_params(contract: Contract, live: LiveEnvironment) -> None:
    trained = contract.vehicle_params
    actual = live.vehicle_params.meta
    if trained.schema_version != actual.schema_version:
        raise VehicleParamsVersionMismatchError(
            "vehicle_params schema_version mismatch: this policy was trained against "
            f"{trained.schema_version!r}, but the live vehicle_params reports "
            f"{actual.schema_version!r} (claude-docs/06-vehicle-params.md 'Sysid coupling')."
        )
    if trained.sysid_session_id != actual.sysid_session_id:
        raise VehicleParamsVersionMismatchError(
            "vehicle_params sysid_session_id mismatch: this policy was trained against "
            f"session {trained.sysid_session_id!r}, but the live vehicle_params reports "
            f"session {actual.sysid_session_id!r} (claude-docs/06-vehicle-params.md "
            "'Sysid coupling': a policy trained against one parameter set and deployed "
            "against another is a refuse-on-mismatch case)."
        )


def _verify_observation_fields(contract: Contract, live: LiveEnvironment) -> None:
    contract_fields = contract.observation_schema.fields
    live_fields = live.observation.fields

    contract_names = [f.name for f in contract_fields]
    live_names = [f.name for f in live_fields]
    contract_name_set = set(contract_names)
    live_name_set = set(live_names)

    missing = [n for n in contract_names if n not in live_name_set]
    if missing:
        raise MissingObservationFieldError(
            "contract declares observation field(s) not present in the live environment: "
            f"{missing!r} (contract fields: {contract_names!r}, live fields: {live_names!r})"
        )

    extra = [n for n in live_names if n not in contract_name_set]
    if extra:
        raise ExtraObservationFieldError(
            "live environment publishes observation field(s) the contract does not "
            f"declare: {extra!r} (contract fields: {contract_names!r}, live fields: "
            f"{live_names!r})"
        )

    if contract_names != live_names:
        raise ObservationSchemaMismatchError(
            "observation field order mismatch: contract expects "
            f"{contract_names!r}, live environment provides {live_names!r} in that order."
        )

    live_by_name = {f.name: f for f in live_fields}
    for cf in contract_fields:
        lf = live_by_name[cf.name]
        if cf.dtype != lf.dtype:
            raise ObservationDtypeMismatchError(
                f"observation field {cf.name!r} dtype mismatch: contract expects "
                f"{cf.dtype!r}, live environment provides {lf.dtype!r}."
            )
        if cf.units != lf.units:
            raise ObservationUnitsMismatchError(
                f"observation field {cf.name!r} units mismatch: contract expects "
                f"{cf.units!r}, live environment provides {lf.units!r}."
            )


def _verify_lidar(contract: Contract, live: LiveEnvironment) -> None:
    lidar = contract.observation_schema.lidar
    obs = live.observation
    if lidar.beam_count != obs.lidar_beam_count:
        raise ObservationSchemaMismatchError(
            f"LiDAR beam_count mismatch: contract expects {lidar.beam_count!r}, live "
            f"environment provides {obs.lidar_beam_count!r}."
        )
    if lidar.fov_rad != obs.lidar_fov_rad:
        raise ObservationSchemaMismatchError(
            f"LiDAR fov_rad mismatch: contract expects {lidar.fov_rad!r}, live environment "
            f"provides {obs.lidar_fov_rad!r}."
        )
    if lidar.downsample_factor != obs.lidar_downsample_factor:
        raise ObservationSchemaMismatchError(
            "LiDAR downsample_factor mismatch: contract expects "
            f"{lidar.downsample_factor!r}, live environment provides "
            f"{obs.lidar_downsample_factor!r}."
        )


def verify_against_environment(contract: Contract, live: LiveEnvironment) -> None:
    """Raise a specific `racer_policy.errors.ContractError` subclass if `contract` does not
    match `live`; return `None` (silently) if every check passes.

    Checks, in order (each is independent -- fixing one does not mask another that was
    already going to fire on a later call):

    1. `vehicle_params` schema_version and sysid_session_id (VehicleParamsVersionMismatchError)
    2. observation fields missing from the live environment (MissingObservationFieldError)
    3. live observation fields not declared in the contract (ExtraObservationFieldError)
    4. observation field order (ObservationSchemaMismatchError)
    5. per-field dtype (ObservationDtypeMismatchError) and units (ObservationUnitsMismatchError)
    6. LiDAR beam_count / fov_rad / downsample_factor (ObservationSchemaMismatchError)
    """
    _verify_vehicle_params(contract, live)
    _verify_observation_fields(contract, live)
    _verify_lidar(contract, live)
