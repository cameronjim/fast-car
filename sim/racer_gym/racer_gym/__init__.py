"""racer_gym: an extension of the pinned f1tenth_gym (roadmap task S.1).

See claude-docs/07-sim-and-sysid.md for the required model upgrades and
claude-docs/00-project-overview.md's regime table for what fidelity is claimed where.
"""

from __future__ import annotations

from .env import RacerGymEnv, build_env
from .params import DynParams, DynParamsResult, build_dyn_params, load_vehicle_params

__all__ = [
    "DynParams",
    "DynParamsResult",
    "RacerGymEnv",
    "build_dyn_params",
    "build_env",
    "load_vehicle_params",
]
