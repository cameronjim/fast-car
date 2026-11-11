"""Everything a deployment must check before a policy may move the car, in one place.

`load_deployment` is the single entry point used by BOTH the on-vehicle `policy_node` and the
racer app's "is an ML policy launchable" check (`python -m racer_policy.check`), so "the app
enabled the button" and "the node will start" cannot disagree.

It runs, in order, and raises on the first failure (CLAUDE.md invariant 3: a hard refusal,
never a warning, no override flag anywhere in this module):

1. `load_contract`: manifest schema, contract version, `policy.pt` checksum.
2. `verify_against_environment` against the LIVE `vehicle_params` and the live observation
   schema this package's builder produces (`live_observation`): schema_version and
   sysid_session_id, field names/order/dtype/units, LiDAR block.
3. The checks the contract schema has no room for. The contract is `additionalProperties:
   false`, so everything else a deployment must match lives in the sidecar `policy_meta.yaml`;
   the contract loader does not read it, so THIS module does. A policy directory without a
   readable, consistent `policy_meta.yaml` is refused.

The envelope settings (residual fractions, speed cap, OOD reference) are taken from the policy
directory, never from node parameters, so a deployment cannot quietly run a policy with a
looser envelope than it was trained and validated against.

Torch is not needed here (checksum only); `load_model` is called by the node afterwards.
"""

from __future__ import annotations

import math
import sys
from dataclasses import dataclass
from pathlib import Path
from typing import Any

import yaml

from racer_policy import live_observation as lo
from racer_policy.contract import Contract, ObservationField, load_contract
from racer_policy.environment import LiveEnvironment, LiveObservationSchema
from racer_policy.errors import ContractError
from racer_policy.verify import verify_against_environment

META_FILENAME = "policy_meta.yaml"
# The loop rate the policy was trained at and the node runs at (claude-docs/08-learning.md).
CONTROL_RATE_HZ = 50.0
REL_TOL = 1e-9


class PolicyMetaError(ContractError):
    """`policy_meta.yaml` is missing, malformed, or disagrees with `contract.yaml` or with the
    live system. Refusal class: deployment settings the contract schema cannot hold."""


@dataclass(frozen=True)
class EnvelopeSettings:
    residual_fraction_steering: float
    residual_fraction_speed: float
    speed_cap_mps: float
    ood_threshold: float
    ood_reference_state: tuple[float, ...]
    ood_reference_scale: tuple[float, ...]
    control_rate_hz: float


@dataclass(frozen=True)
class BaseControllerSettings:
    lookahead_min_m: float
    lookahead_max_m: float
    lookahead_curvature_ref_1pm: float
    raceline_a_lat_max_mps2: float | None
    raceline_v_max_mps: float | None


@dataclass(frozen=True)
class Deployment:
    contract: Contract
    envelope: EnvelopeSettings
    steering_scale_rad: float
    speed_scale_mps: float
    base_controller: BaseControllerSettings
    meta: dict[str, Any]
    policy_dir: Path


def live_environment(vehicle_params: Any) -> LiveEnvironment:
    """What this build of the node would feed a policy, for `verify_against_environment`."""
    return LiveEnvironment(
        vehicle_params=vehicle_params,
        observation=LiveObservationSchema(
            fields=tuple(ObservationField(n, d, u) for n, d, u in lo.LIVE_FIELDS_V0),
            lidar_beam_count=lo.LIDAR_BEAM_COUNT,
            lidar_fov_rad=lo.LIDAR_FOV_RAD,
            lidar_downsample_factor=lo.LIDAR_DOWNSAMPLE_FACTOR,
        ),
    )


def _close(a: float, b: float) -> bool:
    return math.isclose(a, b, rel_tol=REL_TOL, abs_tol=REL_TOL)


def _need(mapping: Any, key: str, where: str) -> Any:
    if not isinstance(mapping, dict) or key not in mapping:
        raise PolicyMetaError(f"{META_FILENAME}: missing {where}.{key}")
    return mapping[key]


def _floats(value: Any, where: str) -> tuple[float, ...]:
    try:
        return tuple(float(v) for v in value)
    except (TypeError, ValueError) as exc:
        raise PolicyMetaError(f"{META_FILENAME}: {where} must be a list of numbers") from exc


def _read_meta(policy_dir: Path) -> dict[str, Any]:
    path = policy_dir / META_FILENAME
    if not path.is_file():
        raise PolicyMetaError(
            f"{path} is missing: a policy directory without its deployment settings is refused"
        )
    try:
        raw = yaml.safe_load(path.read_text(encoding="utf-8"))
    except (OSError, yaml.YAMLError) as exc:
        raise PolicyMetaError(f"{path} could not be read: {exc}") from exc
    if not isinstance(raw, dict):
        raise PolicyMetaError(f"{path} must be a YAML mapping")
    return raw


def _check_meta(contract: Contract, meta: dict[str, Any], vehicle_params: Any) -> Deployment:
    must = _need(meta, "must_match_at_deploy", "")
    env_raw = _need(must, "envelope", "must_match_at_deploy")
    scale_raw = _need(must, "action_scale", "must_match_at_deploy")
    base_raw = _need(must, "base_controller", "must_match_at_deploy")
    obs_raw = _need(must, "observation", "must_match_at_deploy")

    envelope = EnvelopeSettings(
        residual_fraction_steering=float(_need(env_raw, "residual_fraction_steering", "envelope")),
        residual_fraction_speed=float(_need(env_raw, "residual_fraction_speed", "envelope")),
        speed_cap_mps=float(_need(env_raw, "speed_cap_mps", "envelope")),
        ood_threshold=float(_need(env_raw, "ood_threshold", "envelope")),
        ood_reference_state=_floats(_need(env_raw, "ood_reference_state", "envelope"), "ood ref"),
        ood_reference_scale=_floats(_need(env_raw, "ood_reference_scale", "envelope"), "ood scale"),
        control_rate_hz=float(_need(env_raw, "control_rate_hz", "envelope")),
    )
    steering_scale = float(_need(scale_raw, "steering_rad_per_unit", "action_scale"))
    speed_scale = float(_need(scale_raw, "speed_mps_per_unit", "action_scale"))

    # The policy's own contract and its sidecar must tell one story.
    limits = contract.action_space.residual_limits
    if not _close(envelope.residual_fraction_steering, limits.steering_fraction) or not _close(
        envelope.residual_fraction_speed, limits.speed_fraction
    ):
        raise PolicyMetaError(
            "residual fractions disagree: contract.yaml action_space.residual_limits is "
            f"({limits.steering_fraction}, {limits.speed_fraction}) but {META_FILENAME} says "
            f"({envelope.residual_fraction_steering}, {envelope.residual_fraction_speed})"
        )
    by_name = {f.name: f for f in contract.action_space.fields}
    steer_f = by_name.get("steering_residual_rad")
    speed_f = by_name.get("speed_residual_mps")
    if steer_f is None or speed_f is None:
        raise PolicyMetaError(
            "contract action_space lacks steering_residual_rad/speed_residual_mps"
        )
    if not _close(steering_scale, steer_f.high) or not _close(speed_scale, speed_f.high):
        raise PolicyMetaError(
            "action scales disagree: contract.yaml action_space bounds are "
            f"({steer_f.high}, {speed_f.high}) but {META_FILENAME} action_scale is "
            f"({steering_scale}, {speed_scale})"
        )
    if (
        len(envelope.ood_reference_state) != lo.OOD_STATE_DIM
        or len(envelope.ood_reference_scale) != lo.OOD_STATE_DIM
    ):
        raise PolicyMetaError(f"OOD reference must have {lo.OOD_STATE_DIM} entries")
    if not _close(envelope.control_rate_hz, CONTROL_RATE_HZ):
        raise PolicyMetaError(
            f"policy was trained at {envelope.control_rate_hz} Hz, this node runs at "
            f"{CONTROL_RATE_HZ} Hz"
        )

    if str(_need(obs_raw, "profile", "observation")) != lo.PROFILE:
        raise PolicyMetaError(
            f"observation profile {obs_raw['profile']!r}: this build only knows {lo.PROFILE!r}"
        )
    if not (
        int(_need(obs_raw, "lidar_beam_count", "observation")) == lo.LIDAR_BEAM_COUNT
        and _close(float(_need(obs_raw, "lidar_fov_rad", "observation")), lo.LIDAR_FOV_RAD)
        and int(_need(obs_raw, "lidar_downsample_factor", "observation"))
        == lo.LIDAR_DOWNSAMPLE_FACTOR
        and bool(_need(obs_raw, "lidar_min_pool", "observation"))
    ):
        raise PolicyMetaError(f"{META_FILENAME} observation block disagrees with the live builder")

    # Normalisation is applied INSIDE policy.pt; a contract that asks for any other would be
    # applied by nobody, so a non-identity block is refused rather than silently ignored.
    norm = contract.normalization
    if any(m != 0.0 for m in norm.mean) or any(s != 1.0 for s in norm.std):
        raise PolicyMetaError(
            "contract.yaml normalization is not the identity, but this node applies none "
            "(the statistics must be baked into policy.pt)"
        )
    nm = meta.get("observation_normalization") or {}
    if nm.get("applied_inside_policy_pt") is not True:
        raise PolicyMetaError(
            f"{META_FILENAME} observation_normalization.applied_inside_policy_pt is not true"
        )

    # A policy trained on numbers the live vehicle_params does not contain is a policy
    # trained for a different car, whatever the version string says.
    overrides = meta.get("vehicle_overrides_assumed")
    if overrides:
        raise PolicyMetaError(
            "policy was trained with in-memory vehicle_params overrides "
            f"{overrides!r} that the live vehicle_params does not contain; re-export it "
            "against committed values"
        )

    # The actuator assumptions recorded at training time must equal what the live params give.
    act = contract.actuator_assumptions
    live_rate = float(vehicle_params.steering.max_rate_rad_per_s)
    live_accel = float(vehicle_params.actuation.max_acceleration_mps2)
    if not _close(act.rate_limit_steering_rad_s, live_rate) or not _close(
        act.rate_limit_speed_mps_s, live_accel
    ):
        raise PolicyMetaError(
            "actuator assumptions disagree with the live vehicle_params: policy trained with "
            f"rate limits ({act.rate_limit_steering_rad_s} rad/s, {act.rate_limit_speed_mps_s} "
            f"m/s2), live values are ({live_rate}, {live_accel})"
        )
    if envelope.speed_cap_mps > float(vehicle_params.limits.global_speed_cap_mps):
        raise PolicyMetaError("policy speed cap exceeds the live global speed cap")

    base = BaseControllerSettings(
        lookahead_min_m=float(_need(base_raw, "lookahead_min_m", "base_controller")),
        lookahead_max_m=float(_need(base_raw, "lookahead_max_m", "base_controller")),
        lookahead_curvature_ref_1pm=float(
            _need(base_raw, "lookahead_curvature_ref_1pm", "base_controller")
        ),
        raceline_a_lat_max_mps2=base_raw.get("raceline_a_lat_max_mps2"),
        raceline_v_max_mps=base_raw.get("raceline_v_max_mps"),
    )
    return Deployment(
        contract=contract,
        envelope=envelope,
        steering_scale_rad=steering_scale,
        speed_scale_mps=speed_scale,
        base_controller=base,
        meta=meta,
        policy_dir=contract.directory,
    )


def load_deployment(policy_dir: Path | str, vehicle_params: Any) -> Deployment:
    """Run every refusal check; return the validated settings or raise a `ContractError`."""
    policy_dir = Path(policy_dir)
    contract = load_contract(policy_dir)
    verify_against_environment(contract, live_environment(vehicle_params))
    return _check_meta(contract, _read_meta(policy_dir), vehicle_params)


def build_envelope_config(deployment: Deployment, vehicle_params: Any) -> Any:
    """The `envelope.EnvelopeConfig` the node enforces: same arguments, same library as
    `racer_train.envelope_config.build_envelope_config` (a parity test compares the two)."""
    from envelope import DistanceOODScorer, EnvelopeConfig

    dt = 1.0 / deployment.envelope.control_rate_hz
    scorer = DistanceOODScorer(
        reference=deployment.envelope.ood_reference_state,
        scale=deployment.envelope.ood_reference_scale,
    )
    return EnvelopeConfig.from_vehicle_params(
        vehicle_params,
        scorer,
        residual_fraction_steering=deployment.envelope.residual_fraction_steering,
        residual_fraction_speed=deployment.envelope.residual_fraction_speed,
        max_delta_steering_rad=vehicle_params.steering.max_rate_rad_per_s * dt,
        max_delta_speed_mps=vehicle_params.actuation.max_acceleration_mps2 * dt,
        speed_cap_mps=deployment.envelope.speed_cap_mps,
        ood_threshold=deployment.envelope.ood_threshold,
    )


def ensure_envelope_importable() -> None:
    """Make `import envelope` work on the car, where it is a sibling directory of the colcon
    workspace rather than an installed package (the same repo-relative walk-up
    `racer_tools.vehicle_params_loader` uses; colcon builds with --symlink-install)."""
    try:
        import envelope  # noqa: F401

        return
    except ImportError:
        pass
    here = Path(__file__).resolve()
    for parent in here.parents:
        candidate = parent / "training" / "envelope" / "src"
        if (candidate / "envelope" / "__init__.py").is_file():
            sys.path.insert(0, str(candidate))
            return
    raise ImportError("envelope library not importable and training/envelope/src not found")
