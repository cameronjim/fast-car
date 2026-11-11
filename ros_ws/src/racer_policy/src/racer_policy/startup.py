"""Startup checks and small pure helpers for policy_node, kept out of the rclpy class so they
are unit-tested without ROS (claude-docs/10-conventions.md). `build_core` is every refusal the
node performs before it subscribes to anything: parameters, the deployment contract and its
sidecar, the raceline, the model load and a dry inference."""

from __future__ import annotations

import math
from typing import Any

import numpy as np

from racer_policy.deploy import ensure_envelope_importable, load_deployment
from racer_policy.errors import ContractError
from racer_policy.live_observation import Array, Raceline, RacelineError
from racer_policy.policy_core import PolicyCore


class StartupRefusal(RuntimeError):
    """The node will not start. The message is the specific reason."""


def clamp_for_float32(value: float, low: float, high: float) -> float:
    """Clamp to [low, high] and round to float32 without landing outside (mirrors
    racer_control::clamp_for_float32_publish: a double exactly on a bound can round a few ULPs
    past it on the float32 wire, which the safety node would count as a violation)."""
    f = np.float32(min(max(value, low), high))
    while float(f) > high:
        f = np.nextafter(f, np.float32(-np.inf))
    while float(f) < low:
        f = np.nextafter(f, np.float32(np.inf))
    return float(f)


def yaw_from_quaternion(w: float, x: float, y: float, z: float) -> float:
    return math.atan2(2.0 * (w * z + x * y), 1.0 - 2.0 * (y * y + z * z))


def load_vehicle_params() -> Any:
    from racer_tools.vehicle_params_loader import load_vehicle_params as _load

    return _load()


def build_core(
    policy_dir: str,
    raceline_path: str,
    vehicle_params: Any,
    scan_timeout_s: float,
) -> tuple[PolicyCore, Any]:
    """Every startup check and the model load. Raises StartupRefusal on any failure."""
    if not policy_dir:
        raise StartupRefusal("parameter policy_dir is required")
    if not raceline_path:
        raise StartupRefusal("parameter raceline_path is required")
    ensure_envelope_importable()
    try:
        deployment = load_deployment(policy_dir, vehicle_params)
    except ContractError as exc:
        raise StartupRefusal(f"{type(exc).__name__}: {exc}") from exc
    except (OSError, ValueError) as exc:
        raise StartupRefusal(f"policy directory unreadable: {exc}") from exc
    try:
        raceline = Raceline.from_csv(raceline_path)
    except RacelineError as exc:
        raise StartupRefusal(str(exc)) from exc
    try:
        from racer_policy.model import load_model

        module = load_model(deployment.contract)
    except ContractError as exc:
        raise StartupRefusal(f"{type(exc).__name__}: {exc}") from exc
    except Exception as exc:
        raise StartupRefusal(f"policy.pt failed to load: {exc}") from exc

    import torch

    torch.set_num_threads(1)
    expected = len(deployment.contract.observation_schema.fields) + (
        deployment.contract.observation_schema.lidar.beam_count
    )

    def model(obs: Array) -> Array:
        with torch.no_grad():
            out = module(torch.from_numpy(np.asarray(obs, dtype=np.float32)[None, :]))
        return np.asarray(out[0].numpy())

    # One dry inference proves the module accepts the observation width and returns two
    # numbers; a model that cannot is refused here, not discovered mid-lap.
    try:
        probe = np.asarray(model(np.zeros(expected, dtype=np.float32)))
    except Exception as exc:
        raise StartupRefusal(f"policy.pt rejected a {expected}-wide observation: {exc}") from exc
    if probe.shape != (2,):
        raise StartupRefusal(f"policy.pt returned shape {probe.shape}, expected (2,)")
    core = PolicyCore(
        deployment=deployment,
        vehicle_params=vehicle_params,
        raceline=raceline,
        model=model,
        scan_timeout_s=scan_timeout_s,
    )
    return core, deployment
