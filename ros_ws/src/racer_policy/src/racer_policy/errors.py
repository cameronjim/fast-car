"""Refusal exceptions for the racer_policy deployment contract.

CLAUDE.md hard invariant 3: "Deployment refuses on mismatch ... Never downgrade a refusal
to a warning." Every exception here is a hard failure with a specific message naming the
mismatch; there is no override flag, no warning-only mode, and no way to catch-and-continue
from inside this package. `ContractError` is the common base so calling code (the future
policy_node, roadmap 5.2) can log-and-exit on any of them uniformly, but each mismatch class
listed in claude-docs/12-testing.md L1 gets its own concrete type so a test can assert
exactly which check failed.
"""

from __future__ import annotations


class ContractError(Exception):
    """Base class for every deployment-contract refusal. Never caught-and-ignored."""


class ContractManifestError(ContractError):
    """contract.yaml itself is malformed: fails schema validation, cannot be parsed, or is
    missing entirely. Covers structural corruption of the manifest -- a missing top-level
    section, an unexpected extra one, a field with the wrong JSON type, and so on -- as one
    class distinct from the live-environment comparisons in `verify_against_environment`.
    """


class ChecksumMismatchError(ContractError):
    """The sha256 of the on-disk policy artifact does not match `policy.checksum_sha256` in
    the manifest. Mismatch class: bad checksum (claude-docs/12-testing.md L1)."""


class ContractVersionMismatchError(ContractError):
    """`contract_version`'s major component does not match the major version this loader
    supports. Mismatch class: schema version (claude-docs/12-testing.md L1)."""


class VehicleParamsVersionMismatchError(ContractError):
    """The `vehicle_params` this policy was trained against (schema_version and/or
    sysid_session_id, recorded in the contract) does not match the live vehicle_params the
    deploy environment reports. Mismatch class: params version (claude-docs/12-testing.md
    L1) -- see claude-docs/06-vehicle-params.md 'Sysid coupling'."""


class ObservationSchemaMismatchError(ContractError):
    """The live observation field ORDER, or the live LiDAR beam_count/fov_rad/
    downsample_factor, does not match the contract. Mismatch class: obs schema
    (claude-docs/12-testing.md L1). Per-field name presence and per-field dtype/units are
    their own, more specific, exception types below."""


class MissingObservationFieldError(ContractError):
    """An observation field the contract declares is not present among the live fields.
    Mismatch class: missing field (claude-docs/12-testing.md L1)."""


class ExtraObservationFieldError(ContractError):
    """A live observation field is not declared anywhere in the contract. Mismatch class:
    extra field (claude-docs/12-testing.md L1)."""


class ObservationDtypeMismatchError(ContractError):
    """A field present in both the contract and the live environment has a different dtype
    in each. Mismatch class: wrong dtype (claude-docs/12-testing.md L1)."""


class ObservationUnitsMismatchError(ContractError):
    """A field present in both the contract and the live environment has a different units
    string in each. Mismatch class: wrong units string (claude-docs/12-testing.md L1)."""


class MissingTorchDependencyError(ContractError):
    """`racer_policy.model.load_model` was called but `torch` is not installed. Expected in
    the L1 CI environment (claude-docs/12-testing.md); torch is a deploy-time-only
    dependency (claude-docs/08-learning.md), never required to load or verify a contract."""
