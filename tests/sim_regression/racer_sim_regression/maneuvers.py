"""The fixed, seeded battery of maneuvers (roadmap task S.6): throttle step, steering step,
constant-radius circle at a few speeds, and coastdown -- the same maneuver vocabulary
claude-docs/07-sim-and-sysid.md's real-world sysid battery uses (steps, constant-radius
sweeps, coastdown; figure-eights are validation-only there and are not needed here, since
this battery's job is catching a racer_gym CODE regression, not fitting or validating a
model against real telemetry).

Every maneuver function has the signature
``(*, seed: int, vehicle_params) -> (trajectory_records, summary_records, dyn_params_result)``:

- ``trajectory_records``: a decimated, golden-comparable sample of the maneuver's full
  state history (scenarios.sample_trajectory) -- the broad, sensitive regression signal.
- ``summary_records``: a handful of derived sysid-style scalars (metrics.py) -- steady-state
  values and settling times, the compact, physically-interpretable regression signal.
- ``dyn_params_result``: forwarded so callers can build a provenance header
  (provenance.py) without a second env construction.

IMPORTANT CAVEAT (read before touching ``coastdown``): racer_gym's dynamics
(sim/racer_gym/racer_gym/dynamics/model.py) has no independent aerodynamic/rolling-
resistance drag term. Deceleration is entirely actuator/command driven -- see
f1tenth_gym.envs.dynamic_models.pid_accl: a speed command that exactly matches the current
speed produces zero commanded acceleration, not a decaying one. ``coastdown`` below
therefore exercises the accel-limited BRAKING response (a step down to a target speed of
0 mps, following the exact same actuator/delay/accl_constraints code path a throttle step
does) rather than a literal passive coastdown/drag measurement. It is named "coastdown" to
match claude-docs/07-sim-and-sysid.md's real-world battery vocabulary and is still a
legitimate regression canary for that braking code path -- but it is NOT evidence racer_gym
models drag, and if a real drag term is ever added to racer_gym's dynamics, this maneuver's
committed reference must be regenerated with that as the stated reason.
"""

from __future__ import annotations

import numpy as np

from . import metrics
from .scenarios import (
    STATE_SLIP,
    STATE_V,
    STATE_YAW_RATE,
    OpenLoopRun,
    run_open_loop,
    sample_trajectory,
)

# --- throttle step --------------------------------------------------------------------

THROTTLE_STEP_TARGET_SPEED_MPS = 4.0
THROTTLE_STEP_STEPS = 400
THROTTLE_STEP_STRIDE = 10
THROTTLE_STEP_STEADY_WINDOW = 50


def throttle_step(*, seed: int, vehicle_params):
    """From rest, a straight-line step in commanded speed to
    ``THROTTLE_STEP_TARGET_SPEED_MPS``. Exercises the throttle actuator/delay chain and the
    acceleration-limiting branch of ``accl_constraints``."""
    run = run_open_loop(
        seed=seed,
        num_steps=THROTTLE_STEP_STEPS,
        command_fn=lambda _i: (0.0, THROTTLE_STEP_TARGET_SPEED_MPS),
        vehicle_params=vehicle_params,
    )
    times = (np.arange(1, run.states.shape[0] + 1)) * run.dt_s
    speed = run.states[:, STATE_V]

    trajectory = sample_trajectory(run, stride=THROTTLE_STEP_STRIDE)
    steady_state_speed = metrics.steady_state_mean(speed, window=THROTTLE_STEP_STEADY_WINDOW)
    summary = [
        {
            "target_speed_mps": THROTTLE_STEP_TARGET_SPEED_MPS,
            "steady_state_speed_mps": steady_state_speed,
            "settling_time_s": metrics.settling_time_s(
                times, speed, band=0.02 * THROTTLE_STEP_TARGET_SPEED_MPS
            ),
        }
    ]
    return trajectory, summary, run.dyn_params_result


# --- steering step ---------------------------------------------------------------------

STEERING_STEP_CRUISE_SPEED_MPS = 3.0
STEERING_STEP_WARMUP_STEPS = 150
STEERING_STEP_TARGET_RAD = 0.15
STEERING_STEP_RESPONSE_STEPS = 500
STEERING_STEP_STRIDE = 10
STEERING_STEP_STEADY_WINDOW = 50


def steering_step(*, seed: int, vehicle_params):
    """Cruise straight at ``STEERING_STEP_CRUISE_SPEED_MPS`` for a warm-up window (so the
    throttle transient has already settled), then step the commanded steering angle to
    ``STEERING_STEP_TARGET_RAD`` and hold it. Exercises the steering actuator/delay chain
    plus the front/rear Pacejka + load-transfer yaw dynamics (sim/racer_gym/racer_gym/
    dynamics/model.py). Only the post-step response is recorded (the warm-up is run but not
    golden-compared, so the golden file captures the step response cleanly, t=0 at step
    onset)."""

    def command(i: int) -> tuple[float, float]:
        steer = 0.0 if i < STEERING_STEP_WARMUP_STEPS else STEERING_STEP_TARGET_RAD
        return steer, STEERING_STEP_CRUISE_SPEED_MPS

    run = run_open_loop(
        seed=seed,
        num_steps=STEERING_STEP_WARMUP_STEPS + STEERING_STEP_RESPONSE_STEPS,
        command_fn=command,
        vehicle_params=vehicle_params,
    )
    response = OpenLoopRun(
        dt_s=run.dt_s,
        seed=run.seed,
        states=run.states[STEERING_STEP_WARMUP_STEPS:],
        dyn_params_result=run.dyn_params_result,
    )
    times = np.arange(1, response.states.shape[0] + 1) * response.dt_s
    yaw_rate = response.states[:, STATE_YAW_RATE]
    slip = response.states[:, STATE_SLIP]

    trajectory = sample_trajectory(response, stride=STEERING_STEP_STRIDE)
    steady_state_yaw_rate = metrics.steady_state_mean(yaw_rate, window=STEERING_STEP_STEADY_WINDOW)
    steady_state_slip = metrics.steady_state_mean(slip, window=STEERING_STEP_STEADY_WINDOW)
    band = max(0.02 * abs(steady_state_yaw_rate), 1e-3)
    summary = [
        {
            "steer_target_rad": STEERING_STEP_TARGET_RAD,
            "cruise_speed_mps": STEERING_STEP_CRUISE_SPEED_MPS,
            "steady_state_yaw_rate_radps": steady_state_yaw_rate,
            "steady_state_slip_angle_rad": steady_state_slip,
            "yaw_rate_settling_time_s": metrics.settling_time_s(times, yaw_rate, band=band),
            "yaw_rate_overshoot_frac": metrics.peak_overshoot_frac(yaw_rate, steady_state_yaw_rate),
        }
    ]
    return trajectory, summary, run.dyn_params_result


# --- constant-radius circle -------------------------------------------------------------

CIRCLE_STEER_RAD = 0.15
CIRCLE_SPEEDS_MPS = (1.5, 2.5, 3.5)
CIRCLE_RUN_STEPS = 600
CIRCLE_STEADY_WINDOW = 100
CIRCLE_STRIDE = 10


def constant_radius_circle(*, seed: int, vehicle_params):
    """A fixed steering angle at a few target speeds (increasing, per
    claude-docs/07-sim-and-sysid.md's "constant-radius circles at increasing speed"),
    recording the steady-state yaw rate / slip angle / lateral acceleration at each speed --
    the sim analogue of a friction-limit / understeer-gradient sweep. The full sampled
    trajectory golden is recorded only for the MIDDLE speed (kept small; the per-speed
    steady-state summary is this maneuver's primary signal, matching how a real
    constant-radius sweep is actually read)."""
    mid_speed = CIRCLE_SPEEDS_MPS[len(CIRCLE_SPEEDS_MPS) // 2]
    trajectory: list[dict] | None = None
    summary = []
    dyn_params_result = None

    for speed in CIRCLE_SPEEDS_MPS:
        run = run_open_loop(
            seed=seed,
            num_steps=CIRCLE_RUN_STEPS,
            command_fn=lambda _i, _speed=speed: (CIRCLE_STEER_RAD, _speed),
            vehicle_params=vehicle_params,
        )
        dyn_params_result = run.dyn_params_result
        v = run.states[:, STATE_V]
        yaw_rate = run.states[:, STATE_YAW_RATE]
        slip = run.states[:, STATE_SLIP]

        steady_v = metrics.steady_state_mean(v, window=CIRCLE_STEADY_WINDOW)
        steady_yaw_rate = metrics.steady_state_mean(yaw_rate, window=CIRCLE_STEADY_WINDOW)
        steady_slip = metrics.steady_state_mean(slip, window=CIRCLE_STEADY_WINDOW)
        summary.append(
            {
                "speed_target_mps": speed,
                "steady_state_speed_mps": steady_v,
                "steady_state_yaw_rate_radps": steady_yaw_rate,
                "steady_state_slip_angle_rad": steady_slip,
                "steady_state_lateral_accel_mps2": steady_v * steady_yaw_rate,
            }
        )
        if speed == mid_speed:
            trajectory = sample_trajectory(run, stride=CIRCLE_STRIDE)

    assert trajectory is not None  # mid_speed is always a member of CIRCLE_SPEEDS_MPS
    assert dyn_params_result is not None
    return trajectory, summary, dyn_params_result


# --- coastdown (braking step; see module docstring's caveat) ----------------------------

COASTDOWN_CRUISE_SPEED_MPS = 5.0
COASTDOWN_WARMUP_STEPS = 400
COASTDOWN_BRAKE_STEPS = 400
COASTDOWN_STRIDE = 10
COASTDOWN_NEAR_ZERO_THRESHOLD_MPS = 0.1


def coastdown(*, seed: int, vehicle_params):
    """Cruise at ``COASTDOWN_CRUISE_SPEED_MPS`` for a warm-up window, then command a step
    down to a 0 mps target and hold it. See the module docstring's caveat: this exercises
    the accel-limited braking response, not a passive drag/coastdown -- racer_gym's
    dynamics have no independent drag term."""

    def command(i: int) -> tuple[float, float]:
        speed_cmd = COASTDOWN_CRUISE_SPEED_MPS if i < COASTDOWN_WARMUP_STEPS else 0.0
        return 0.0, speed_cmd

    run = run_open_loop(
        seed=seed,
        num_steps=COASTDOWN_WARMUP_STEPS + COASTDOWN_BRAKE_STEPS,
        command_fn=command,
        vehicle_params=vehicle_params,
    )
    response = OpenLoopRun(
        dt_s=run.dt_s,
        seed=run.seed,
        states=run.states[COASTDOWN_WARMUP_STEPS:],
        dyn_params_result=run.dyn_params_result,
    )
    times = np.arange(1, response.states.shape[0] + 1) * response.dt_s
    speed = response.states[:, STATE_V]

    trajectory = sample_trajectory(response, stride=COASTDOWN_STRIDE)
    time_to_near_zero_s = metrics.first_time_below(
        times, np.abs(speed), threshold=COASTDOWN_NEAR_ZERO_THRESHOLD_MPS
    )
    # NOT a step-1 finite difference: the command-to-torque transport delay
    # (sim/racer_gym/racer_gym/dynamics/delay.py) means the first ~delay_steps samples after
    # the brake command switches still reflect the OLD (cruise) command, so a single-step
    # slope right at t=0 measures dead time, not braking rate. The average rate implied by
    # "how long it took to get from cruise speed to near-zero" is well-defined regardless of
    # how much of that time was dead time vs. actual deceleration, and is still directly
    # sensitive to a max_acceleration_mps2 (or delay/lag) change.
    mean_decel_mps2 = COASTDOWN_CRUISE_SPEED_MPS / time_to_near_zero_s
    summary = [
        {
            "cruise_speed_mps": COASTDOWN_CRUISE_SPEED_MPS,
            "brake_target_speed_mps": 0.0,
            "time_to_near_zero_s": time_to_near_zero_s,
            "mean_decel_mps2": mean_decel_mps2,
        }
    ]
    return trajectory, summary, run.dyn_params_result


__all__ = [
    "coastdown",
    "constant_radius_circle",
    "steering_step",
    "throttle_step",
]
