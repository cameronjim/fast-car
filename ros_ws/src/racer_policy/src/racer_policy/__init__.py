"""racer_policy: the deployment-contract loader (claude-docs/08-learning.md).

ROS-free by design (claude-docs/10-conventions.md: "Gate/decision logic is always separated
from node plumbing so it is testable without ROS"). The rclpy `policy_node` that wraps
`load_contract` / `verify_against_environment` / `load_model` for 50 Hz on-vehicle inference
is roadmap task 5.2, not this package's job.

Public API:

- `load_contract(directory)` -> `Contract`: parse + schema-validate contract.yaml, check the
  contract-format major version, verify the policy artifact's checksum.
- `verify_against_environment(contract, live)`: compare a loaded `Contract` against a
  `LiveEnvironment` snapshot (vehicle_params, live observation schema); raises on any
  mismatch.
- `load_model(contract)`: deploy-time-only, lazily imports `torch`.

Every failure mode above is a specific exception from `racer_policy.errors`, all deriving
from `ContractError`. CLAUDE.md invariant 3: these are hard refusals, never warnings, and
this package exposes no override.
"""

from __future__ import annotations

from racer_policy.contract import (
    ActionField,
    ActionSpace,
    ActuatorAssumptions,
    Contract,
    LidarConfig,
    Normalization,
    ObservationField,
    ObservationSchema,
    PolicyArtifact,
    ResidualLimits,
    TrainingProvenance,
    VehicleParamsRef,
    load_contract,
)
from racer_policy.environment import (
    LiveEnvironment,
    LiveObservationSchema,
    VehicleParamsLike,
    VehicleParamsMetaLike,
)
from racer_policy.errors import (
    ChecksumMismatchError,
    ContractError,
    ContractManifestError,
    ContractVersionMismatchError,
    ExtraObservationFieldError,
    MissingObservationFieldError,
    MissingTorchDependencyError,
    ObservationDtypeMismatchError,
    ObservationSchemaMismatchError,
    ObservationUnitsMismatchError,
    VehicleParamsVersionMismatchError,
)
from racer_policy.model import load_model
from racer_policy.verify import verify_against_environment

__all__ = [
    "ActionField",
    "ActionSpace",
    "ActuatorAssumptions",
    "ChecksumMismatchError",
    "Contract",
    "ContractError",
    "ContractManifestError",
    "ContractVersionMismatchError",
    "ExtraObservationFieldError",
    "LidarConfig",
    "LiveEnvironment",
    "LiveObservationSchema",
    "MissingObservationFieldError",
    "MissingTorchDependencyError",
    "Normalization",
    "ObservationDtypeMismatchError",
    "ObservationField",
    "ObservationSchema",
    "ObservationSchemaMismatchError",
    "ObservationUnitsMismatchError",
    "PolicyArtifact",
    "ResidualLimits",
    "TrainingProvenance",
    "VehicleParamsLike",
    "VehicleParamsMetaLike",
    "VehicleParamsRef",
    "VehicleParamsVersionMismatchError",
    "load_contract",
    "load_model",
    "verify_against_environment",
]
