"""Builds a complete, valid policy directory (contract.yaml + policy.pt + policy_meta.yaml) for
the v0 observation profile, so deploy / node / canary tests can start from "everything
agrees" and break exactly one thing.

`policy.pt` is torch-free bytes by default (enough for every contract and sidecar check, which
only hash it); `torch_policy_bytes()` writes a real TorchScript module for the tests that
load it. The module is a tiny fixed network with deterministic small weights, so a canary that
uses it exercises the whole path without being a trained policy.
"""

from __future__ import annotations

import copy
import hashlib
import io
from collections.abc import Callable
from pathlib import Path
from typing import Any

import yaml
from racer_policy import live_observation as lo

OBS_DIM = len(lo.LIVE_FIELDS_V0) + lo.LIDAR_BEAM_COUNT
STEERING_SCALE_RAD = 0.0222
SPEED_SCALE_MPS = 0.099
FRACTION_STEERING = 0.025
FRACTION_SPEED = 0.004
FAKE_BYTES = b"not-a-real-torchscript-model, just fixture bytes for hashing tests"


def manifest(vehicle_params: Any, checksum: str) -> dict[str, Any]:
    return {
        "contract_version": "1.0.0",
        "policy": {"filename": "policy.pt", "checksum_sha256": checksum},
        "observation_schema": {
            "fields": [{"name": n, "dtype": d, "units": u} for n, d, u in lo.LIVE_FIELDS_V0],
            "lidar": {
                "beam_count": lo.LIDAR_BEAM_COUNT,
                "fov_rad": lo.LIDAR_FOV_RAD,
                "downsample_factor": lo.LIDAR_DOWNSAMPLE_FACTOR,
            },
        },
        "normalization": {"mean": [0.0] * OBS_DIM, "std": [1.0] * OBS_DIM},
        "action_space": {
            "fields": [
                {
                    "name": "steering_residual_rad",
                    "low": -STEERING_SCALE_RAD,
                    "high": STEERING_SCALE_RAD,
                    "units": "rad",
                },
                {
                    "name": "speed_residual_mps",
                    "low": -SPEED_SCALE_MPS,
                    "high": SPEED_SCALE_MPS,
                    "units": "m/s",
                },
            ],
            "scaling": "linear",
            "residual_limits": {
                "steering_fraction": FRACTION_STEERING,
                "speed_fraction": FRACTION_SPEED,
            },
        },
        "actuator_assumptions": {
            "rate_limit_steering_rad_s": float(vehicle_params.steering.max_rate_rad_per_s),
            "rate_limit_speed_mps_s": float(vehicle_params.actuation.max_acceleration_mps2),
            "command_delay_s": 0.08,
        },
        "vehicle_params": {
            "schema_version": vehicle_params.meta.schema_version,
            "sysid_session_id": vehicle_params.meta.sysid_session_id,
        },
        "training": {"config_hash": hashlib.sha256(b"fixture").hexdigest(), "git_sha": "abc1234"},
    }


def meta() -> dict[str, Any]:
    return {
        "label": "fixture, not a trained policy",
        "policy_version": "fixture",
        "observation_normalization": {"applied_inside_policy_pt": True},
        "must_match_at_deploy": {
            "envelope": {
                "residual_fraction_steering": FRACTION_STEERING,
                "residual_fraction_speed": FRACTION_SPEED,
                "speed_cap_mps": 2.0,
                "ood_threshold": 3.0,
                "ood_reference_state": [1.8, 0.0, 0.0, 0.0, 0.0],
                "ood_reference_scale": [0.8, 2.5, 0.3, 0.2, 0.4],
                "control_rate_hz": 50.0,
            },
            "action_scale": {
                "steering_rad_per_unit": STEERING_SCALE_RAD,
                "speed_mps_per_unit": SPEED_SCALE_MPS,
            },
            "base_controller": {
                "lookahead_min_m": 1.0,
                "lookahead_max_m": 1.2,
                "lookahead_curvature_ref_1pm": 1.0,
                "raceline_a_lat_max_mps2": 4.0,
                "raceline_v_max_mps": 2.0,
            },
            "observation": {
                "profile": "v0",
                "lidar_beam_count": lo.LIDAR_BEAM_COUNT,
                "lidar_fov_rad": lo.LIDAR_FOV_RAD,
                "lidar_downsample_factor": lo.LIDAR_DOWNSAMPLE_FACTOR,
                "lidar_min_pool": True,
            },
        },
        "vehicle_overrides_assumed": {},
    }


def build_policy_dir(
    root: Path,
    vehicle_params: Any,
    *,
    policy_bytes: bytes = FAKE_BYTES,
    mutate_contract: Callable[[dict[str, Any]], None] | None = None,
    mutate_meta: Callable[[dict[str, Any]], None] | None = None,
    write_meta: bool = True,
    name: str = "policy",
) -> Path:
    directory = root / name
    directory.mkdir(parents=True)
    (directory / "policy.pt").write_bytes(policy_bytes)
    contract = manifest(vehicle_params, hashlib.sha256(policy_bytes).hexdigest())
    if mutate_contract:
        mutate_contract(contract)
    (directory / "contract.yaml").write_text(
        yaml.safe_dump(contract, sort_keys=False), encoding="utf-8"
    )
    if write_meta:
        body = copy.deepcopy(meta())
        if mutate_meta:
            mutate_meta(body)
        (directory / "policy_meta.yaml").write_text(
            yaml.safe_dump(body, sort_keys=False), encoding="utf-8"
        )
    return directory


def torch_policy_bytes(scale: float = 0.0, speed_gain: float = 1.0) -> bytes:
    """A TorchScript module: obs (1, 47) -> tanh(Linear(obs)) in [-1, 1]^2. `scale` multiplies
    the weights; 0.0 gives an all-zero action (the plain tracker), larger values a nonzero
    residual that depends on the observation. `speed_gain` scales the speed output's weights
    on top (0.0: a steering-only residual)."""
    import torch

    torch.manual_seed(0)
    lin = torch.nn.Linear(OBS_DIM, 2)
    with torch.no_grad():
        lin.weight.mul_(scale)
        lin.bias.mul_(scale)
        lin.weight[1].mul_(speed_gain)
        lin.bias[1].mul_(speed_gain)

    class Tiny(torch.nn.Module):
        def __init__(self) -> None:
            super().__init__()
            self.lin = lin

        def forward(self, obs: torch.Tensor) -> torch.Tensor:
            return torch.tanh(self.lin(obs))

    buf = io.BytesIO()
    torch.jit.save(torch.jit.script(Tiny().eval()), buf)
    return buf.getvalue()
