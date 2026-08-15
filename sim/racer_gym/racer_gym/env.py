"""racer_gym gymnasium environment: the pinned f1tenth_gym env with the model, integrator,
and action-type of every agent replaced by the racer_gym upgraded single-track dynamics
(racer_gym/dynamics/model.py) -- load transfer, front/rear Pacejka, first-order actuator
lag, and transport delay (roadmap S.1, claude-docs/07-sim-and-sysid.md).

This is an EXTENSION, not a vendor/fork: no f1tenth_gym source file is copied or modified.
`build_env()` constructs a completely normal `gym.make("f1tenth_gym:f1tenth-v0", ...)` env
and then replaces each agent's `.model` / `.integrator` / `.action_type` attributes -- see
racer_gym/dynamics/model.py's module docstring for why that is a safe, supported-by-
construction seam (those attributes are re-read fresh on every `update_pose()` call, and
`RaceCar.reset()` never touches them). The observation/action space, obs dict shape
(`{"agent_0": {...}}`), collision/scan/lap-counting logic, and rendering are all completely
unmodified upstream f1tenth_gym code.

`vehicle_params` fallback state (which fields were null and got an f1tenth_gym-derived
placeholder instead of a Phase-3 sysid fit -- see racer_gym/params.py) is exposed as
`env.unwrapped.racer_fallback_flags` so callers (training configs, evaluation reports) can
record whether a run used unfitted placeholders, per claude-docs/00-project-overview.md's
honesty rules.
"""

from __future__ import annotations

import logging
from typing import Any

import gymnasium as gym

from .dynamics.model import RacerCarAction, RacerRK4Integrator, RacerSingleTrackModel
from .params import DynParamsResult, build_dyn_params, load_vehicle_params

logger = logging.getLogger(__name__)

DEFAULT_TIMESTEP_S = 0.01  # matches f1tenth_gym's own default (F110Env.default_config())


class RacerGymEnv(gym.Wrapper):
    """Thin wrapper exposing the exact upstream gymnasium interface plus
    `racer_fallback_flags` / `racer_dyn_params_result`."""

    def __init__(self, env: gym.Env, dyn_params_result: DynParamsResult) -> None:
        super().__init__(env)
        self.racer_dyn_params_result = dyn_params_result
        self.racer_fallback_flags = dyn_params_result.fallback_flags
        if dyn_params_result.used_any_placeholder:
            logger.warning(
                "racer_gym: this env was built with unfitted placeholder dynamics "
                "(fallback_flags=%s); do not treat its trajectories as sim-to-real "
                "validated (claude-docs/00-project-overview.md regime table).",
                {k: v for k, v in dyn_params_result.fallback_flags.items() if v},
            )

    def reset(self, *, seed=None, options=None):
        for agent in self.unwrapped.sim.agents:
            action_type = agent.action_type
            if isinstance(action_type, RacerCarAction):
                action_type.reset()
        return self.env.reset(seed=seed, options=options)


def _patch_agents_with_racer_dynamics(env: gym.Env, dyn_params_result: DynParamsResult) -> None:
    dyn_params = dyn_params_result.dyn_params
    timestep_s = env.unwrapped.timestep
    for agent in env.unwrapped.sim.agents:
        agent.model = RacerSingleTrackModel(dyn_params)
        agent.integrator = RacerRK4Integrator()
        agent.action_type = RacerCarAction(dyn_params, timestep_s)
        # Re-seed the state through the new model so it has the right (still-zero) shape;
        # RaceCar.__init__ already set agent.state from the OLD model before we got here.
        agent.state = agent.model.get_initial_state(pose=agent.state[:3])


def build_env(
    config: dict | None = None,
    render_mode: str | None = None,
    vehicle_params: Any | None = None,
) -> RacerGymEnv:
    """Construct a racer_gym env.

    `config` is passed straight through to `gym.make("f1tenth_gym:f1tenth-v0", config=...)`
    (same shape as `F110Env.default_config()` / docker/sim-cpu/smoke_test.py). `vehicle_params`
    is a generated `VEHICLE_PARAMS` instance (see racer_gym.params.load_vehicle_params); when
    None (the default) it is loaded fresh from config/vehicle_params.yaml.
    """
    config = dict(config or {})
    timestep_s = config.get("timestep", DEFAULT_TIMESTEP_S)

    if vehicle_params is None:
        vehicle_params = load_vehicle_params()
    dyn_params_result = build_dyn_params(vehicle_params, timestep_s)

    base_env = gym.make(
        "f1tenth_gym:f1tenth-v0",
        config=config,
        render_mode=render_mode,
    )
    _patch_agents_with_racer_dynamics(base_env, dyn_params_result)

    return RacerGymEnv(base_env, dyn_params_result)
