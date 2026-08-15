"""The deployment contract: its data model, its on-disk schema, and `load_contract`.

Per claude-docs/08-learning.md ("Deployment contract"), a deployable policy is a directory
containing a `contract.yaml` manifest (validated against the packaged
`contract.schema.json`) plus the policy weights artifact it names. `load_contract` is the
static half of the refuse-on-mismatch loader: everything it checks (manifest shape, the
contract-format major version, the policy artifact's checksum) can be verified from the
directory alone, with no live ROS/vehicle state. The dynamic half -- comparing the loaded
`Contract` against what the running system actually reports -- is
`racer_policy.verify.verify_against_environment`.

CLAUDE.md hard invariant 3: every failure mode below is a hard exception from
`racer_policy.errors`, never a warning, never bypassable.
"""

from __future__ import annotations

import hashlib
import json
from dataclasses import dataclass
from pathlib import Path
from typing import Any

import jsonschema
import yaml

from racer_policy.errors import (
    ChecksumMismatchError,
    ContractManifestError,
    ContractVersionMismatchError,
)

# The contract-FORMAT major version this loader understands. Bump this, and re-issue
# contracts with a matching `contract_version`, whenever a breaking change is made to
# contract.schema.json (claude-docs/08-learning.md: "contract_version"). A contract whose
# major differs is refused outright -- there is no forward- or backward-compat shim.
SUPPORTED_CONTRACT_MAJOR = 1

_SCHEMA_PATH = Path(__file__).with_name("contract.schema.json")
_DEFAULT_MANIFEST_NAME = "contract.yaml"


@dataclass(frozen=True)
class PolicyArtifact:
    filename: str
    checksum_sha256: str


@dataclass(frozen=True)
class ObservationField:
    name: str
    dtype: str
    units: str


@dataclass(frozen=True)
class LidarConfig:
    beam_count: int
    fov_rad: float
    downsample_factor: int


@dataclass(frozen=True)
class ObservationSchema:
    fields: tuple[ObservationField, ...]
    lidar: LidarConfig


@dataclass(frozen=True)
class Normalization:
    mean: tuple[float, ...]
    std: tuple[float, ...]


@dataclass(frozen=True)
class ActionField:
    name: str
    low: float
    high: float
    units: str


@dataclass(frozen=True)
class ResidualLimits:
    steering_fraction: float
    speed_fraction: float


@dataclass(frozen=True)
class ActionSpace:
    fields: tuple[ActionField, ...]
    scaling: str
    residual_limits: ResidualLimits


@dataclass(frozen=True)
class ActuatorAssumptions:
    rate_limit_steering_rad_s: float
    rate_limit_speed_mps_s: float
    command_delay_s: float


@dataclass(frozen=True)
class VehicleParamsRef:
    schema_version: str
    sysid_session_id: str


@dataclass(frozen=True)
class TrainingProvenance:
    config_hash: str
    git_sha: str


@dataclass(frozen=True)
class Contract:
    """A fully loaded and self-consistency-checked deployment contract.

    Reaching this type means: contract.yaml validated against contract.schema.json,
    `contract_version`'s major matches `SUPPORTED_CONTRACT_MAJOR`, and the policy artifact's
    sha256 matches `policy.checksum_sha256`. It does NOT mean the contract matches the live
    deploy environment -- that is `verify_against_environment`'s job.
    """

    contract_version: str
    policy: PolicyArtifact
    observation_schema: ObservationSchema
    normalization: Normalization
    action_space: ActionSpace
    actuator_assumptions: ActuatorAssumptions
    vehicle_params: VehicleParamsRef
    training: TrainingProvenance
    directory: Path

    @property
    def policy_path(self) -> Path:
        """Absolute path to the policy artifact this contract's directory names."""
        return self.directory / self.policy.filename


def _load_schema() -> dict[str, Any]:
    with open(_SCHEMA_PATH, "r", encoding="utf-8") as f:
        result: dict[str, Any] = json.load(f)
        return result


def _validate_manifest(raw: Any, schema: dict[str, Any]) -> None:
    """Validate `raw` against `schema`, raising ContractManifestError with a specific,
    single-error message on the first (best-match) violation -- mirrors
    tools/gen_params.py's `validate_params` (claude-docs/06-vehicle-params.md consumers all
    validate-or-refuse the same way)."""
    validator_cls = jsonschema.validators.validator_for(schema)
    validator_cls.check_schema(schema)
    validator = validator_cls(schema, format_checker=jsonschema.FormatChecker())
    errors = list(validator.iter_errors(raw))
    if not errors:
        return
    error = jsonschema.exceptions.best_match(errors)
    json_path = "$" + "".join(
        f"[{p!r}]" if isinstance(p, str) else f"[{p}]" for p in error.absolute_path
    )
    if not error.absolute_path:
        json_path = "$ (root)"
    raise ContractManifestError(
        f"contract.yaml failed schema validation at {json_path}: {error.message} "
        f"(failed validator: {error.validator!r})"
    )


def _build_contract(raw: dict[str, Any], directory: Path) -> Contract:
    """Build the typed `Contract` from an already schema-validated manifest dict. No further
    validation happens here -- every field's shape, type, and range was already enforced by
    the jsonschema pass in `_validate_manifest`."""
    policy = PolicyArtifact(**raw["policy"])
    obs_raw = raw["observation_schema"]
    observation_schema = ObservationSchema(
        fields=tuple(ObservationField(**f) for f in obs_raw["fields"]),
        lidar=LidarConfig(**obs_raw["lidar"]),
    )
    normalization = Normalization(
        mean=tuple(raw["normalization"]["mean"]), std=tuple(raw["normalization"]["std"])
    )
    action_raw = raw["action_space"]
    action_space = ActionSpace(
        fields=tuple(ActionField(**f) for f in action_raw["fields"]),
        scaling=action_raw["scaling"],
        residual_limits=ResidualLimits(**action_raw["residual_limits"]),
    )
    actuator_assumptions = ActuatorAssumptions(**raw["actuator_assumptions"])
    vehicle_params = VehicleParamsRef(**raw["vehicle_params"])
    training = TrainingProvenance(**raw["training"])

    return Contract(
        contract_version=raw["contract_version"],
        policy=policy,
        observation_schema=observation_schema,
        normalization=normalization,
        action_space=action_space,
        actuator_assumptions=actuator_assumptions,
        vehicle_params=vehicle_params,
        training=training,
        directory=directory,
    )


def _check_contract_version(contract_version: str) -> None:
    major_str = contract_version.split(".", 1)[0]
    major = int(major_str)  # safe: schema regex already enforced `\d+\.\d+\.\d+`
    if major != SUPPORTED_CONTRACT_MAJOR:
        raise ContractVersionMismatchError(
            f"contract_version {contract_version!r} has major version {major}, but this "
            f"loader only supports major version {SUPPORTED_CONTRACT_MAJOR}. Refusing to "
            "load -- there is no compatibility shim (CLAUDE.md invariant 3)."
        )


def _sha256_of_file(path: Path) -> str:
    digest = hashlib.sha256()
    with open(path, "rb") as f:
        for chunk in iter(lambda: f.read(1 << 20), b""):
            digest.update(chunk)
    return digest.hexdigest()


def _check_checksum(policy: PolicyArtifact, directory: Path) -> None:
    policy_path = directory / policy.filename
    if not policy_path.is_file():
        raise ContractManifestError(
            f"contract.yaml names policy artifact {policy.filename!r}, but "
            f"{policy_path} does not exist."
        )
    actual = _sha256_of_file(policy_path)
    if actual != policy.checksum_sha256:
        raise ChecksumMismatchError(
            f"checksum mismatch for {policy_path}: contract.yaml declares "
            f"{policy.checksum_sha256!r}, computed {actual!r}. Refusing to load a policy "
            "artifact that does not match its declared checksum."
        )


def load_contract(directory: Path, manifest_filename: str = _DEFAULT_MANIFEST_NAME) -> Contract:
    """Load and fully validate the deployment contract in `directory`.

    Raises (all `racer_policy.errors.ContractError` subclasses, never returns a partially
    valid `Contract`):

    - `ContractManifestError`: the manifest file is missing, unparsable, or fails schema
      validation (any missing/extra/wrong-typed field anywhere in it).
    - `ContractVersionMismatchError`: `contract_version`'s major version is not
      `SUPPORTED_CONTRACT_MAJOR`.
    - `ChecksumMismatchError`: the named policy artifact's sha256 does not match
      `policy.checksum_sha256`.
    """
    manifest_path = directory / manifest_filename
    if not manifest_path.is_file():
        raise ContractManifestError(f"no contract manifest found at {manifest_path}")

    try:
        with open(manifest_path, "r", encoding="utf-8") as f:
            raw = yaml.safe_load(f)
    except yaml.YAMLError as e:
        raise ContractManifestError(f"{manifest_path} is not valid YAML: {e}") from e

    if not isinstance(raw, dict):
        raise ContractManifestError(
            f"{manifest_path} must parse to a YAML mapping at the top level, got "
            f"{type(raw).__name__}"
        )

    schema = _load_schema()
    _validate_manifest(raw, schema)

    contract = _build_contract(raw, directory)
    _check_contract_version(contract.contract_version)
    _check_checksum(contract.policy, directory)
    return contract
