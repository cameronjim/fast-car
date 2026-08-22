"""The upgraded single-track dynamics function, and the duck-typed "model" / "integrator" /
"action_type" objects that plug it into an otherwise-unmodified f1tenth_gym `RaceCar`.

Why this is an injectable seam (see the PR body / sim/racer_gym/README.md for the full
extension-vs-vendor writeup): `f1tenth_gym.envs.base_classes.RaceCar` and `Simulator` accept
`model`, `integrator`, and `action_type` as plain constructor arguments and re-read
`self.model` / `self.integrator` / `self.action_type` on every `update_pose()` call rather
than caching bound methods at construction time. `F110Env.__init__` only ever passes those
three arguments as fixed strings resolved through `DynamicModel.from_string` /
`IntegratorType.from_string`, so there is no *config-level* way to inject a custom object --
but nothing stops replacing `agent.model` / `agent.integrator` / `agent.action_type` on the
already-constructed `RaceCar` objects afterwards (`racer_gym/env.py` does exactly this, once,
right after `gym.make()`). `RaceCar.reset()` does not touch those three attributes, so the
replacement survives every subsequent `env.reset()`.

The three duck-typed contracts (matching f1tenth_gym.envs.{dynamic_models,integrator,action}
exactly, but implemented from scratch rather than subclassed, since `DynamicModel` and
`IntegratorType` are closed Enums):

  * model:      `.get_initial_state(pose=None) -> np.ndarray` and a `.f_dynamics` property
                returning a callable `f(x, u, params) -> dx/dt`.
  * integrator: `.integrate(f, x, u, dt, params) -> new_x`.
  * action_type: `.act(action, state, params) -> (accel, steer_velocity)`, plus `.space` /
                `.type` (copied from the wrapped upstream action objects so the gym
                action-space bounds -- s_min/s_max/v_min/v_max from vehicle_params -- are
                unaffected).

State vector is the SAME 7-element `[x, y, delta, v, yaw, yaw_rate, slip_angle]` upstream's
`DynamicModel.ST` uses (see `f1tenth_gym.envs.dynamic_models.DynamicModel.get_initial_state`
and the index comments in `f1tenth_gym.envs.observation`), so `observation.py`'s
`kinematic_state`/`dynamic_state` readers and `base_classes.py`'s scan/collision code (which
only ever read indices 0, 1, 2, 3, 4) keep working unmodified.
"""

from __future__ import annotations

import math

import numpy as np
from f1tenth_gym.envs.dynamic_models import (
    DynamicModel,
    accl_constraints,
    pid_accl,
    pid_steer,
    steering_constraint,
)

from ..params import DynParams
from . import load_transfer, tire
from .actuator import FirstOrderActuator
from .delay import TransportDelay

# Numerical-stability floor on forward speed used only to avoid a division-by-zero /
# singular slip-angle computation at V ~= 0; NOT a physical vehicle parameter (see
# claude-docs/06-vehicle-params.md -- it would have no unit-carrying physical meaning to put
# in vehicle_params.yaml). Below this speed slip-angle-based lateral dynamics are not
# meaningful anyway; this keeps the ODE well-defined without a discrete kinematic/dynamic
# mode switch. claude-docs/00-project-overview.md's regime table already excludes claims at
# the friction limit; very-low-speed creep is similarly out of this project's claimed
# fidelity envelope.
_MIN_SPEED_FOR_DYNAMICS_MPS = 0.1


def racer_dynamics_st(x: np.ndarray, u: np.ndarray, dyn_params: DynParams) -> np.ndarray:
    """Upgraded single-track dynamics: same state/input convention as
    `f1tenth_gym.envs.dynamic_models.vehicle_dynamics_st`, but yaw-rate and slip-angle
    derivatives come from front/rear Pacejka tire forces on load-transferred normal loads
    (racer_gym/dynamics/load_transfer.py, racer_gym/dynamics/tire.py) instead of upstream's
    fixed linear-cornering-stiffness closed form.

    Standard bicycle-model force/moment balance (see e.g. Rajamani, "Vehicle Dynamics and
    Control", ch. 2 for the structurally identical small-beta derivation; the difference here
    is only where Fyf/Fyr come from):

        Iz * psi_dot_dot   = lf * Fyf * cos(delta) - lr * Fyr
        m * V * (beta_dot + psi_dot) = Fyf * cos(delta) + Fyr

    Per-axle slip angles use the exact (non-small-angle) axle-point-velocity formula; sign
    convention matches claude-docs/06-vehicle-params.md ("slip angle positive when velocity
    points left of heading"), applied at each axle -- see tire.py's docstring for the
    resulting force sign.
    """
    _x, _y, delta, v, psi, psi_dot, beta = x
    steer_velocity_raw, accel_raw = u

    steer_velocity = steering_constraint(
        delta,
        steer_velocity_raw,
        dyn_params.s_min,
        dyn_params.s_max,
        dyn_params.sv_min,
        dyn_params.sv_max,
    )
    accel = accl_constraints(
        v, accel_raw, dyn_params.v_switch, dyn_params.a_max, dyn_params.v_min, dyn_params.v_max
    )

    v_safe = (
        math.copysign(_MIN_SPEED_FOR_DYNAMICS_MPS, v) if abs(v) < _MIN_SPEED_FOR_DYNAMICS_MPS else v
    )

    ax = accel
    ay = v_safe * psi_dot  # standard centripetal-acceleration approximation

    static = load_transfer.static_axle_loads(
        dyn_params.mass_kg, dyn_params.cg_to_front_axle_m, dyn_params.cg_to_rear_axle_m
    )
    long_loads = load_transfer.longitudinal_load_transfer(
        dyn_params.mass_kg,
        dyn_params.cg_height_m,
        dyn_params.cg_to_front_axle_m,
        dyn_params.cg_to_rear_axle_m,
        ax,
    )
    derate = load_transfer.lateral_grip_derate(
        dyn_params.mass_kg,
        dyn_params.cg_height_m,
        static.front_n,
        static.rear_n,
        ay,
        dyn_params.track_width_m,
    )
    fz_f_eff = long_loads.front_n * derate.front
    fz_r_eff = long_loads.rear_n * derate.rear

    alpha_f = (
        math.atan2(
            v_safe * math.sin(beta) + dyn_params.cg_to_front_axle_m * psi_dot,
            v_safe * math.cos(beta),
        )
        - delta
    )
    alpha_r = math.atan2(
        v_safe * math.sin(beta) - dyn_params.cg_to_rear_axle_m * psi_dot, v_safe * math.cos(beta)
    )

    fyf = tire.lateral_force(alpha_f, fz_f_eff, static.front_n, dyn_params.pacejka_front)
    fyr = tire.lateral_force(alpha_r, fz_r_eff, static.rear_n, dyn_params.pacejka_rear)

    yaw_moment = (
        dyn_params.cg_to_front_axle_m * fyf * math.cos(delta) - dyn_params.cg_to_rear_axle_m * fyr
    )
    psi_dot_dot = yaw_moment / dyn_params.yaw_inertia_kg_m2

    lat_force_total = fyf * math.cos(delta) + fyr
    beta_dot = lat_force_total / (dyn_params.mass_kg * v_safe) - psi_dot

    return np.array(
        [
            v * math.cos(psi + beta),
            v * math.sin(psi + beta),
            steer_velocity,
            accel,
            psi_dot,
            psi_dot_dot,
            beta_dot,
        ]
    )


class RacerSingleTrackModel:
    """Duck-typed replacement for `f1tenth_gym.envs.dynamic_models.DynamicModel.ST`."""

    def __init__(self, dyn_params: DynParams) -> None:
        self.dyn_params = dyn_params

    def get_initial_state(self, pose=None) -> np.ndarray:
        # Same 7-element zero/pose-seeded state as upstream ST -- reused programmatically,
        # not reimplemented.
        return DynamicModel.ST.get_initial_state(pose=pose)

    @property
    def f_dynamics(self):
        dyn_params = self.dyn_params

        def _f(x, u, _params_dict_unused):
            return racer_dynamics_st(x, u, dyn_params)

        return _f


class RacerRK4Integrator:
    """Duck-typed replacement for `f1tenth_gym.envs.integrator.RK4Integrator`.

    Same RK4 update as upstream; the only difference is that `f` is called as `f(x, u,
    params)` with `params` passed straight through (rather than unpacked into upstream's
    fixed 17-argument list), so `model.f_dynamics` above can close over a `DynParams` object
    with the richer field set (Pacejka, load transfer geometry, ...) upstream's fixed
    signature has no room for.
    """

    def __init__(self) -> None:
        self.type = "rk4"

    def integrate(self, f, x, u, dt, params):
        k1 = f(x, u, params)
        k2 = f(x + dt * (k1 / 2), u, params)
        k3 = f(x + dt * (k2 / 2), u, params)
        k4 = f(x + dt * k3, u, params)
        return x + dt * (1.0 / 6.0) * (k1 + 2 * k2 + 2 * k3 + k4)


class RacerCarAction:
    """Duck-typed replacement for `f1tenth_gym.envs.action.CarAction`.

    Wraps the same high-level ("desired speed", "desired steering angle") action convention
    upstream's default `["speed", "steering_angle"]` control_input uses, but routes both
    channels through a transport delay (racer_gym/dynamics/delay.py) and a first-order
    actuator lag (racer_gym/dynamics/actuator.py) before converting the (delayed, lagged)
    target into (accel, steering_velocity) via f1tenth_gym's own `pid_accl`/`pid_steer`
    (reused programmatically -- same conversion upstream's `SpeedAction`/`SteeringAngleAction`
    use, just fed a lagged+delayed target instead of the raw command).

    One instance holds actuator/delay STATE (previous lag output, delay FIFO contents), so
    `racer_gym/env.py` gives every agent its own instance rather than sharing one across
    agents the way upstream's default `CarAction` wiring does.
    """

    def __init__(self, dyn_params: DynParams, dt_s: float) -> None:
        self.dyn_params = dyn_params
        self.dt_s = dt_s
        self.type = ("steering_angle", "speed")

        self._steer_delay = TransportDelay(dyn_params.delay_steps, initial_value=0.0)
        self._throttle_delay = TransportDelay(dyn_params.delay_steps, initial_value=0.0)
        self._steer_actuator = FirstOrderActuator(dyn_params.steer_tau_s, dt_s, initial_output=0.0)
        self._throttle_actuator = FirstOrderActuator(
            dyn_params.throttle_tau_s, dt_s, initial_output=0.0
        )

        import gymnasium as gym

        self.space = gym.spaces.Box(
            low=np.array([dyn_params.s_min, dyn_params.v_min], dtype=np.float32),
            high=np.array([dyn_params.s_max, dyn_params.v_max], dtype=np.float32),
            shape=(2,),
            dtype=np.float32,
        )

    def reset(self) -> None:
        self._steer_delay.reset(0.0)
        self._throttle_delay.reset(0.0)
        self._steer_actuator.reset(0.0)
        self._throttle_actuator.reset(0.0)

    def act(self, action, state, params):
        speed_cmd, steer_cmd = action

        steer_delayed = self._steer_delay.step(float(steer_cmd))
        steer_target = self._steer_actuator.step(steer_delayed)
        sv = pid_steer(steer_target, state[2], self.dyn_params.sv_max)

        speed_delayed = self._throttle_delay.step(float(speed_cmd))
        speed_target = self._throttle_actuator.step(speed_delayed)
        accl = pid_accl(
            speed_target,
            state[3],
            self.dyn_params.a_max,
            self.dyn_params.v_max,
            self.dyn_params.v_min,
        )

        return accl, sv
