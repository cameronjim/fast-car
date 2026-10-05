"""rebuilds the trained observation from live sensors and maps policy actions to drive commands."""

from __future__ import annotations

import json
from dataclasses import dataclass, replace
from pathlib import Path
from typing import NamedTuple

import numpy as np

from learned_control.preprocessing.scan import downsample_scan

SUPPORTED_CONTRACT_VERSION = 1

# the observation features this node has sensors for, so anything else means a sim-only export
BUILDABLE_FEATURES = frozenset({
    "scan",
    "linear_vel_x",
    "linear_vel_y",
    "linear_vel_magnitude",
    "ang_vel_z",
    "delta",
})

REQUIRED_CONTRACT_KEYS = (
    "version",
    "features",
    "feature_order",
    "num_beams",
    "raw_dim",
    "obs_dim",
    "include_prev_action",
    "clip_abs",
    "control_hz",
    "norm",
    "action",
)
REQUIRED_NORM_KEYS = ("scan_range_max_m",)
REQUIRED_ACTION_KEYS = ("steer_max_rad", "speed_min_mps", "speed_cap_mps")
REQUIRED_FEATURE_KEYS = ("name", "start", "size", "scale")

STEER_COLUMN = 0
SPEED_COLUMN = 1

# centred steering at the slowest commandable speed, the safe reading of a non-finite action
SAFE_UNIT_ACTION = np.array([0.0, -1.0], dtype=np.float64)

# a scan this fraction of a period early still counts, or a lidar running at exactly control_hz
# drops every second frame to timing jitter
RATE_GATE_SLACK = 0.1


@dataclass(frozen=True)
class VehicleState:
    """the odometry the policy sees, plus the steering angle the node last commanded."""

    linear_vel_x_mps: float = 0.0
    linear_vel_y_mps: float = 0.0
    ang_vel_z_rps: float = 0.0
    steering_rad: float = 0.0


class DriveCommand(NamedTuple):
    """one ackermann command, already clamped to the exported bounds."""

    steering_rad: float
    speed_mps: float


def _require_keys(blob: dict, keys, source: str) -> None:
    missing = [key for key in keys if key not in blob]
    if missing:
        raise ValueError(f"{source} is missing required contract keys: {missing}")


class DeployContract:
    """the exported obs_config.json, validated against what a ros node can actually rebuild."""

    def __init__(self, blob: dict, source: str = "<memory>"):
        self.source = source
        _require_keys(blob, REQUIRED_CONTRACT_KEYS, source)
        if blob["version"] != SUPPORTED_CONTRACT_VERSION:
            raise ValueError(
                f"{source} is contract version {blob['version']!r}, this node speaks "
                f"version {SUPPORTED_CONTRACT_VERSION}"
            )
        norm = blob["norm"]
        action = blob["action"]
        _require_keys(norm, REQUIRED_NORM_KEYS, f"{source} 'norm'")
        _require_keys(action, REQUIRED_ACTION_KEYS, f"{source} 'action'")

        features = list(blob["features"])
        for entry in features:
            _require_keys(entry, REQUIRED_FEATURE_KEYS, f"{source} feature entry {entry}")
        self._refuse_undeployable(features)

        self.num_beams = int(blob["num_beams"])
        self.raw_dim = int(blob["raw_dim"])
        self.obs_dim = int(blob["obs_dim"])
        self.include_prev_action = bool(blob["include_prev_action"])
        self.clip_abs = float(blob["clip_abs"])
        self.control_hz = float(blob["control_hz"])
        self.scan_range_max_m = float(norm["scan_range_max_m"])
        self.steer_max_rad = float(action["steer_max_rad"])
        self.speed_min_mps = float(action["speed_min_mps"])
        self.speed_cap_mps = float(action["speed_cap_mps"])

        self.feature_order = tuple(entry["name"] for entry in features)
        if self.feature_order != tuple(blob["feature_order"]):
            raise ValueError(
                f"{source} lists features {self.feature_order} but feature_order says "
                f"{tuple(blob['feature_order'])}"
            )
        self.slices, self.scale = self._layout(features)
        self._check_dimensions()

    def _refuse_undeployable(self, features: list) -> None:
        blocked = [
            entry["name"] for entry in features
            if entry["name"] not in BUILDABLE_FEATURES or not entry.get("deployable", True)
        ]
        if blocked:
            raise ValueError(
                f"{self.source} needs observation features no ros node can build from /scan and "
                f"/odom: {blocked}; retrain and export with a deployable feature set"
            )

    def _layout(self, features: list) -> tuple[dict, np.ndarray]:
        slices: dict[str, slice] = {}
        scale = np.zeros(self.raw_dim, dtype=np.float32)
        cursor = 0
        for entry in features:
            name, start, size = entry["name"], int(entry["start"]), int(entry["size"])
            if start != cursor:
                raise ValueError(
                    f"{self.source} feature {name!r} starts at {start}, expected {cursor}: "
                    f"the flat vector must tile without gaps"
                )
            if size < 1 or start + size > self.raw_dim:
                raise ValueError(
                    f"{self.source} feature {name!r} spans [{start}, {start + size}) which "
                    f"does not fit raw_dim {self.raw_dim}"
                )
            if len(entry["scale"]) != size:
                raise ValueError(
                    f"{self.source} feature {name!r} has {len(entry['scale'])} scale factors "
                    f"for {size} dimensions"
                )
            span = slice(start, start + size)
            slices[name] = span
            scale[span] = np.asarray(entry["scale"], dtype=np.float32)
            cursor = start + size
        if cursor != self.raw_dim:
            raise ValueError(
                f"{self.source} features cover {cursor} dimensions but raw_dim is {self.raw_dim}"
            )
        return slices, scale

    def _check_dimensions(self) -> None:
        scan = self.slices.get("scan")
        if scan is not None and scan.stop - scan.start != self.num_beams:
            raise ValueError(
                f"{self.source} says {self.num_beams} beams but the scan feature spans "
                f"{scan.stop - scan.start} dimensions"
            )
        expected_obs_dim = self.raw_dim + (2 if self.include_prev_action else 0)
        if self.obs_dim != expected_obs_dim:
            raise ValueError(
                f"{self.source} says obs_dim {self.obs_dim}, but raw_dim {self.raw_dim} with "
                f"include_prev_action={self.include_prev_action} gives {expected_obs_dim}"
            )
        if self.clip_abs <= 0.0:
            raise ValueError(f"{self.source} has clip_abs {self.clip_abs}, must be > 0")
        if self.control_hz <= 0.0:
            raise ValueError(f"{self.source} has control_hz {self.control_hz}, must be > 0")
        if self.scan_range_max_m <= 0.0:
            raise ValueError(
                f"{self.source} has scan_range_max_m {self.scan_range_max_m}, must be > 0"
            )
        if self.steer_max_rad <= 0.0:
            raise ValueError(f"{self.source} has steer_max_rad {self.steer_max_rad}, must be > 0")
        if self.speed_cap_mps <= self.speed_min_mps:
            raise ValueError(
                f"{self.source} has speed_cap_mps {self.speed_cap_mps} at or below "
                f"speed_min_mps {self.speed_min_mps}"
            )

    def build_obs(self, ranges, state: VehicleState, prev_action=None) -> np.ndarray:
        """the exact vector the policy trained on, rebuilt from one scan and one odom message."""
        raw = np.zeros(self.raw_dim, dtype=np.float32)
        for name, span in self.slices.items():
            raw[span] = self._feature(name, ranges, state)
        obs = raw * self.scale
        # the scan is already finite here, so this only catches odometry going non-finite
        np.nan_to_num(obs, copy=False, nan=0.0, posinf=self.clip_abs, neginf=-self.clip_abs)
        np.clip(obs, -self.clip_abs, self.clip_abs, out=obs)
        if not self.include_prev_action:
            return obs
        prev = (
            np.zeros(2, dtype=np.float32) if prev_action is None
            else np.asarray(prev_action, dtype=np.float32).reshape(2)
        )
        return np.concatenate((obs, prev))

    def _feature(self, name: str, ranges, state: VehicleState):
        if name == "scan":
            if len(ranges) < self.num_beams:
                raise ValueError(
                    f"scan carries {len(ranges)} rays, fewer than the {self.num_beams} beams "
                    f"the policy expects"
                )
            # the sim swept num_beams over the same 270 deg the ros scan covers, so picking
            # evenly spaced indices across the whole scan reproduces the trained beam angles
            return downsample_scan(ranges, self.num_beams, max_range_m=self.scan_range_max_m)
        if name == "linear_vel_x":
            return state.linear_vel_x_mps
        if name == "linear_vel_y":
            return state.linear_vel_y_mps
        if name == "linear_vel_magnitude":
            return float(np.hypot(state.linear_vel_x_mps, state.linear_vel_y_mps))
        if name == "ang_vel_z":
            return state.ang_vel_z_rps
        if name == "delta":
            return state.steering_rad
        raise ValueError(f"no deploy-side builder for observation feature {name!r}")

    def executed_action(self, unit_action) -> np.ndarray:
        """the clipped unit action, which is what training fed back as prev_action."""
        unit = np.asarray(unit_action, dtype=np.float64).reshape(2)
        unit = np.where(np.isfinite(unit), unit, SAFE_UNIT_ACTION)
        return np.clip(unit, -1.0, 1.0).astype(np.float32)

    def drive_command(self, unit_action) -> DriveCommand:
        """map an action in [-1, 1]^2 onto steering and speed, clamped at both boundaries."""
        unit = self.executed_action(unit_action)
        steering = np.clip(
            unit[STEER_COLUMN] * self.steer_max_rad, -self.steer_max_rad, self.steer_max_rad
        )
        speed_mid = 0.5 * (self.speed_cap_mps + self.speed_min_mps)
        speed_half = 0.5 * (self.speed_cap_mps - self.speed_min_mps)
        speed = np.clip(
            speed_mid + unit[SPEED_COLUMN] * speed_half, self.speed_min_mps, self.speed_cap_mps
        )
        return DriveCommand(float(steering), float(speed))


class RateGate:
    """passes one scan per control period, so a lidar faster than control_hz cannot overdrive."""

    def __init__(self, control_hz: float):
        if control_hz <= 0.0:
            raise ValueError(f"control_hz must be > 0, got {control_hz}")
        self.period_sec = 1.0 / float(control_hz)
        self._next_due_sec = None

    def reset(self) -> None:
        self._next_due_sec = None

    def due(self, now_sec: float) -> bool:
        """true when this scan should drive a control step."""
        now_sec = float(now_sec)
        # a clock that jumped backwards is a sim reset, so resync instead of stalling
        if self._next_due_sec is None or now_sec < self._next_due_sec - self.period_sec:
            self._next_due_sec = now_sec + self.period_sec
            return True
        if now_sec + RATE_GATE_SLACK * self.period_sec < self._next_due_sec:
            return False
        # deadlines advance on a fixed grid, so decimation holds the average rate at control_hz
        self._next_due_sec += self.period_sec
        if self._next_due_sec <= now_sec:
            self._next_due_sec = now_sec + self.period_sec
        return True


def load_contract(path) -> DeployContract:
    """read obs_config.json, failing loudly with the path when it is missing or unusable."""
    path = Path(path)
    if not path.is_file():
        raise FileNotFoundError(f"obs config not found: {path.resolve()}")
    return DeployContract(json.loads(path.read_text()), source=str(path))


def load_policy(path):
    """load the exported torchscript policy on cpu as a callable obs -> unit action."""
    path = Path(path)
    if not path.is_file():
        raise FileNotFoundError(f"policy not found: {path.resolve()}")
    # torch only exists where the policy runs, so importing it here keeps the obs and action
    # logic importable and testable on a box without it
    import torch

    module = torch.jit.load(str(path), map_location="cpu")
    module.eval()

    def predict(obs) -> np.ndarray:
        with torch.no_grad():
            batch = torch.from_numpy(np.asarray(obs, dtype=np.float32).reshape(1, -1))
            return module(batch).numpy().reshape(-1)

    return predict


class RLPolicyRuntime:
    """one control step: gate on the control rate, rebuild the obs, run the policy, clamp."""

    def __init__(self, contract: DeployContract, policy):
        self.contract = contract
        self.policy = policy
        self.gate = RateGate(contract.control_hz)
        self.state = VehicleState()
        self.prev_action = np.zeros(2, dtype=np.float32)

    @classmethod
    def from_files(cls, policy_path, obs_config_path) -> "RLPolicyRuntime":
        contract = load_contract(obs_config_path)
        policy = load_policy(policy_path)
        # a width mismatch between weights and contract has to fail at startup, not mid lap
        probe = np.asarray(policy(np.zeros(contract.obs_dim, dtype=np.float32))).reshape(-1)
        if probe.size != 2:
            raise ValueError(
                f"policy {policy_path} returned {probe.size} values for a "
                f"{contract.obs_dim} dim observation, expected 2"
            )
        return cls(contract, policy)

    def reset(self) -> None:
        """forget the previous action and resync the control clock, as an episode boundary does."""
        self.prev_action = np.zeros(2, dtype=np.float32)
        self.gate.reset()

    def update_odom(self, linear_vel_x_mps, linear_vel_y_mps, ang_vel_z_rps) -> None:
        self.state = VehicleState(
            linear_vel_x_mps=float(linear_vel_x_mps),
            linear_vel_y_mps=float(linear_vel_y_mps),
            ang_vel_z_rps=float(ang_vel_z_rps),
        )

    def step(self, ranges, now_sec: float) -> DriveCommand | None:
        """one control step, or none when this scan falls between control ticks."""
        if not self.gate.due(now_sec):
            return None
        obs = self.contract.build_obs(ranges, self._sensed_state(), self.prev_action)
        self.prev_action = self.contract.executed_action(self.policy(obs))
        return self.contract.drive_command(self.prev_action)

    def _sensed_state(self) -> VehicleState:
        # ros gives no steering feedback, so delta is the angle the last executed action commanded
        steering_rad = float(self.prev_action[STEER_COLUMN]) * self.contract.steer_max_rad
        return replace(self.state, steering_rad=steering_rad)
