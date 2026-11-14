"""The vehicle limits the replay checks need, from the generated vehicle_params bindings.

Nothing is written here by hand: `load_generated_params` (racer_track_perception.vehicle)
regenerates the Python binding from config/vehicle_params.yaml with tools/gen_params.py, and the
numbers below are read off it (CLAUDE.md invariant 2).
"""

from __future__ import annotations

from dataclasses import dataclass
from functools import lru_cache

from pnp_replay import perception


@dataclass(frozen=True)
class VehicleLimits:
    steering_min_rad: float
    steering_max_rad: float
    steering_max_rate_rad_per_s: float
    max_acceleration_mps2: float
    schema_version: str


@lru_cache(maxsize=1)
def load_vehicle_limits() -> VehicleLimits:
    perception._import_core()  # puts the racer_track_perception sources on sys.path
    from racer_track_perception.vehicle import load_generated_params

    vp = load_generated_params(perception.REPO_ROOT)
    return VehicleLimits(
        steering_min_rad=float(vp.steering.min_angle_rad),
        steering_max_rad=float(vp.steering.max_angle_rad),
        steering_max_rate_rad_per_s=float(vp.steering.max_rate_rad_per_s),
        max_acceleration_mps2=float(vp.actuation.max_acceleration_mps2),
        schema_version=str(vp.meta.schema_version),
    )
