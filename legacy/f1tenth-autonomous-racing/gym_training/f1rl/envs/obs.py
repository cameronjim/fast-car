# observation layout, normalization, and the train/deploy contract file

from __future__ import annotations

import json
from dataclasses import dataclass, replace
from pathlib import Path

import numpy as np

CONTRACT_VERSION = 1

SCAN_RANGE_MAX_M = 30.0
SPEED_NORM_MPS = 8.0
YAW_RATE_NORM_RPS = 5.0
STEER_NORM_RAD = 0.4189
EY_NORM_M = 5.0
REF_LATERAL_NORM_M = 2.0
CURVATURE_NORM_RADPM = 0.5
PI = float(np.pi)

DEFAULT_CURVATURE_HORIZONS_M = (5.0, 10.0, 15.0, 20.0, 30.0)

# every feature is scaled to live inside this, so the policy sees a unit box
OBS_CLIP_ABS = 1.0

# features a ROS deploy node can rebuild from /scan and /odom alone
DEPLOYABLE_FEATURES = frozenset(
    {"scan", "linear_vel_x", "linear_vel_y", "linear_vel_magnitude", "ang_vel_z", "delta"}
)

# reference-line context, appended after the env features and before the previous action
SCALAR_CONTEXT_FEATURES = frozenset(
    {"ref_lateral_error", "ref_heading_error", "ref_speed", "ref_steer"}
)


@dataclass(frozen=True)
class ActionBounds:
    """policy action in [-1, 1] mapped onto steering angle and speed command."""

    steer_max_rad: float = STEER_NORM_RAD
    speed_min_mps: float = 0.5
    speed_cap_mps: float = 3.0

    def __post_init__(self) -> None:
        if self.steer_max_rad <= 0.0:
            raise ValueError(f"steer_max_rad must be > 0, got {self.steer_max_rad}")
        if self.speed_cap_mps <= self.speed_min_mps:
            raise ValueError(
                f"speed_cap_mps ({self.speed_cap_mps}) must exceed "
                f"speed_min_mps ({self.speed_min_mps})"
            )

    def to_dict(self) -> dict:
        return {
            "steer_max_rad": self.steer_max_rad,
            "speed_min_mps": self.speed_min_mps,
            "speed_cap_mps": self.speed_cap_mps,
        }

    def with_updates(self, **changes) -> "ActionBounds":
        return replace(self, **changes)


@dataclass(frozen=True)
class ObsConfig:
    """flat observation layout and the constants that normalize it."""

    features: tuple[str, ...] = ("scan", "linear_vel_x", "ang_vel_z", "delta", "frenet_pose")
    num_beams: int = 108
    scan_range_max_m: float = SCAN_RANGE_MAX_M
    speed_norm_mps: float = SPEED_NORM_MPS
    yaw_rate_norm_rps: float = YAW_RATE_NORM_RPS
    steer_norm_rad: float = STEER_NORM_RAD
    ey_norm_m: float = EY_NORM_M
    track_length_m: float | None = None
    include_prev_action: bool = True
    control_hz: float = 25.0
    context_features: tuple[str, ...] = ()
    ref_lateral_norm_m: float = REF_LATERAL_NORM_M
    curvature_norm_radpm: float = CURVATURE_NORM_RADPM
    curvature_horizons_m: tuple[float, ...] = DEFAULT_CURVATURE_HORIZONS_M

    def __post_init__(self) -> None:
        object.__setattr__(self, "features", tuple(self.features))
        object.__setattr__(self, "context_features", tuple(self.context_features))
        object.__setattr__(
            self, "curvature_horizons_m", tuple(float(h) for h in self.curvature_horizons_m)
        )
        if not self.features:
            raise ValueError("ObsConfig needs at least one feature")
        if len(set(self.features)) != len(self.features):
            raise ValueError(f"duplicate observation features: {self.features}")
        if len(set(self.context_features)) != len(self.context_features):
            raise ValueError(f"duplicate context features: {self.context_features}")
        clash = sorted(set(self.context_features) & set(self.features))
        if clash:
            raise ValueError(f"these names are both env and context features: {clash}")
        if self.num_beams < 1:
            raise ValueError(f"num_beams must be >= 1, got {self.num_beams}")
        if self.ref_lateral_norm_m <= 0.0 or self.curvature_norm_radpm <= 0.0:
            raise ValueError("ref_lateral_norm_m and curvature_norm_radpm must both be > 0")
        if "ref_curvature" in self.context_features and not self.curvature_horizons_m:
            raise ValueError("ref_curvature needs at least one entry in curvature_horizons_m")
        if "frenet_pose" in self.features and not self.track_length_m:
            raise ValueError(
                "frenet_pose normalizes s by the track length, so track_length_m must be set"
            )
        # gym.spaces.Dict sorts its keys, so the flat vector is alphabetical, not feature order
        object.__setattr__(self, "_flat_order", tuple(sorted(self.features)))
        slices: dict[str, slice] = {}
        start = 0
        for name in self._flat_order:
            size = self._feature_size(name)
            slices[name] = slice(start, start + size)
            start += size
        object.__setattr__(self, "slices", slices)
        object.__setattr__(self, "raw_dim", start)
        # context keeps the configured order, since it never passes through a sorted gym Dict
        context_slices: dict[str, slice] = {}
        offset = 0
        for name in self.context_features:
            size = self._context_size(name)
            context_slices[name] = slice(offset, offset + size)
            offset += size
        object.__setattr__(self, "context_slices", context_slices)
        object.__setattr__(self, "context_dim", offset)
        object.__setattr__(
            self, "obs_dim", start + offset + (2 if self.include_prev_action else 0)
        )
        scale = np.concatenate([self._feature_scale(name) for name in self._flat_order])
        object.__setattr__(self, "_scale", scale.astype(np.float32))
        context_scale = [self._context_scale(name) for name in self.context_features]
        object.__setattr__(
            self,
            "_context_scale",
            (np.concatenate(context_scale) if context_scale else np.zeros(0)).astype(np.float32),
        )

    def _feature_size(self, name: str) -> int:
        if name == "scan":
            return self.num_beams
        if name == "frenet_pose":
            return 3
        return 1

    def _feature_scale(self, name: str) -> np.ndarray:
        if name == "scan":
            return np.full(self.num_beams, 1.0 / self.scan_range_max_m)
        if name == "frenet_pose":
            return np.array([1.0 / float(self.track_length_m), 1.0 / self.ey_norm_m, 1.0 / PI])
        if name in ("linear_vel_x", "linear_vel_y", "linear_vel_magnitude"):
            return np.array([1.0 / self.speed_norm_mps])
        if name == "ang_vel_z":
            return np.array([1.0 / self.yaw_rate_norm_rps])
        if name == "delta":
            return np.array([1.0 / self.steer_norm_rad])
        if name in ("beta", "pose_theta"):
            return np.array([1.0 / PI])
        raise ValueError(f"no normalization defined for observation feature {name!r}")

    def _context_size(self, name: str) -> int:
        if name == "ref_curvature":
            return len(self.curvature_horizons_m)
        if name in SCALAR_CONTEXT_FEATURES:
            return 1
        raise ValueError(f"no context feature named {name!r}")

    def _context_scale(self, name: str) -> np.ndarray:
        if name == "ref_curvature":
            return np.full(len(self.curvature_horizons_m), 1.0 / self.curvature_norm_radpm)
        if name == "ref_lateral_error":
            return np.array([1.0 / self.ref_lateral_norm_m])
        if name == "ref_heading_error":
            return np.array([1.0 / PI])
        if name == "ref_speed":
            return np.array([1.0 / self.speed_norm_mps])
        if name == "ref_steer":
            return np.array([1.0 / self.steer_norm_rad])
        raise ValueError(f"no normalization defined for context feature {name!r}")

    def normalize(self, flat_raw, prev_action=None, context=None) -> np.ndarray:
        """scale the raw flat observation into the unit box, then the context and previous action."""
        raw = np.asarray(flat_raw, dtype=np.float32)
        if raw.shape != (self.raw_dim,):
            raise ValueError(f"expected raw obs of shape ({self.raw_dim},), got {raw.shape}")
        obs = raw * self._scale
        scan = self.slices.get("scan")
        if scan is not None:
            # a dropped or infinite beam reads as max range, which is the honest interpretation
            np.nan_to_num(obs[scan], copy=False, nan=1.0, posinf=1.0, neginf=0.0)
        np.clip(obs, -OBS_CLIP_ABS, OBS_CLIP_ABS, out=obs)
        parts = [obs]
        if self.context_dim:
            parts.append(self._normalize_context(context))
        elif context is not None:
            raise ValueError("this ObsConfig declares no context features, so context must be None")
        if self.include_prev_action:
            parts.append(
                np.zeros(2, dtype=np.float32)
                if prev_action is None
                else np.asarray(prev_action, dtype=np.float32)
            )
        return obs if len(parts) == 1 else np.concatenate(parts)

    def _normalize_context(self, context) -> np.ndarray:
        if context is None:
            raise ValueError(f"this ObsConfig declares {self.context_dim} context dims, got none")
        raw = np.asarray(context, dtype=np.float32).reshape(-1)
        if raw.shape != (self.context_dim,):
            raise ValueError(f"expected context of shape ({self.context_dim},), got {raw.shape}")
        return np.clip(raw * self._context_scale, -OBS_CLIP_ABS, OBS_CLIP_ABS)

    def to_json(self) -> dict:
        """the deploy contract: everything the ROS node needs to rebuild this vector."""
        features = []
        for name in self._flat_order:
            span = self.slices[name]
            features.append(
                {
                    "name": name,
                    "start": span.start,
                    "size": span.stop - span.start,
                    "scale": [float(v) for v in self._scale[span]],
                    "deployable": name in DEPLOYABLE_FEATURES,
                }
            )
        for name in self.context_features:
            span = self.context_slices[name]
            features.append(
                {
                    "name": name,
                    "start": self.raw_dim + span.start,
                    "size": span.stop - span.start,
                    "scale": [float(v) for v in self._context_scale[span]],
                    # the reference line is a sim asset, so no ros node can rebuild these
                    "deployable": False,
                }
            )
        return {
            "version": CONTRACT_VERSION,
            "features": features,
            "feature_order": list(self._flat_order),
            "configured_features": list(self.features),
            "context_features": list(self.context_features),
            "context_dim": self.context_dim,
            "num_beams": self.num_beams,
            "raw_dim": self.raw_dim,
            "obs_dim": self.obs_dim,
            "include_prev_action": self.include_prev_action,
            "clip_abs": OBS_CLIP_ABS,
            "control_hz": self.control_hz,
            "norm": {
                "scan_range_max_m": self.scan_range_max_m,
                "speed_norm_mps": self.speed_norm_mps,
                "yaw_rate_norm_rps": self.yaw_rate_norm_rps,
                "steer_norm_rad": self.steer_norm_rad,
                "ey_norm_m": self.ey_norm_m,
                "track_length_m": self.track_length_m,
                "ref_lateral_norm_m": self.ref_lateral_norm_m,
                "curvature_norm_radpm": self.curvature_norm_radpm,
                "curvature_horizons_m": list(self.curvature_horizons_m),
            },
        }

    @classmethod
    def from_json(cls, blob: dict) -> "ObsConfig":
        if blob.get("version") != CONTRACT_VERSION:
            raise ValueError(f"obs contract version {blob.get('version')!r} != {CONTRACT_VERSION}")
        norm = blob["norm"]
        # context keys arrived after the first contracts were written, so they stay optional here
        return cls(
            features=tuple(blob["configured_features"]),
            num_beams=blob["num_beams"],
            scan_range_max_m=norm["scan_range_max_m"],
            speed_norm_mps=norm["speed_norm_mps"],
            yaw_rate_norm_rps=norm["yaw_rate_norm_rps"],
            steer_norm_rad=norm["steer_norm_rad"],
            ey_norm_m=norm["ey_norm_m"],
            track_length_m=norm["track_length_m"],
            include_prev_action=blob["include_prev_action"],
            control_hz=blob["control_hz"],
            context_features=tuple(blob.get("context_features", ())),
            ref_lateral_norm_m=norm.get("ref_lateral_norm_m", REF_LATERAL_NORM_M),
            curvature_norm_radpm=norm.get("curvature_norm_radpm", CURVATURE_NORM_RADPM),
            curvature_horizons_m=tuple(
                norm.get("curvature_horizons_m", DEFAULT_CURVATURE_HORIZONS_M)
            ),
        )

    def undeployable_features(self) -> tuple[str, ...]:
        """features no ROS node can build, so exporting for the car needs them dropped."""
        names = self._flat_order + self.context_features
        return tuple(name for name in names if name not in DEPLOYABLE_FEATURES)

    def with_updates(self, **changes) -> "ObsConfig":
        return replace(self, **changes)


def deploy_contract(obs_cfg: ObsConfig, action_bounds: ActionBounds) -> dict:
    """the full obs_config.json payload: observation layout plus action bounds."""
    blob = obs_cfg.to_json()
    blob["action"] = action_bounds.to_dict()
    return blob


def write_deploy_contract(path, obs_cfg: ObsConfig, action_bounds: ActionBounds) -> Path:
    path = Path(path)
    path.write_text(json.dumps(deploy_contract(obs_cfg, action_bounds), indent=2) + "\n")
    return path
