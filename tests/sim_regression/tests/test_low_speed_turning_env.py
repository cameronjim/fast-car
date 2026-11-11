"""The measured low-speed turning radius, through the full env path (actuator lag, transport
delay, rate-limited steering, f1tenth_gym integration), not just the bare dynamics.

Same measurement and tolerances as sim/racer_gym/tests/test_low_speed_turning.py (read its
docstring): rear-axle radius 0.60 m at left lock (0.504 rad effective) and 0.69 m at right lock
(0.449 rad), interquartile ranges 0.56 to 0.64 m and 0.64 to 0.74 m, at about 0.3 m/s. The
steering limit is widened on an in-memory copy of the binding because the calibration that
raises the committed +-0.4189 rad is on another branch.
"""

from __future__ import annotations

import dataclasses
import math

import numpy as np
import pytest
import racer_gym
from racer_sim_regression.scenarios import STATE_X, STATE_Y, STATE_YAW, run_open_loop

SPEED_MPS = 0.3
HOLD_STEPS = 1100  # 11 s; the env ends the episode on a lap checkpoint at about 12.5 s
SKIP_STEPS = 300  # the steering ramp and the yaw transient are long over by then


def _widened_params():
    base = racer_gym.load_vehicle_params()
    wide = dataclasses.replace(base.steering, min_angle_rad=-0.6, max_angle_rad=0.6)
    return dataclasses.replace(base, steering=wide)


def _rear_axle_radius_m(steer_rad: float) -> float:
    vp = _widened_params()
    run = run_open_loop(
        seed=1234,
        num_steps=HOLD_STEPS,
        command_fn=lambda _i: (steer_rad, SPEED_MPS),
        vehicle_params=vp,
    )
    lr = vp.chassis.cg_to_rear_axle_m
    s = run.states[SKIP_STEPS:]
    pts = np.column_stack(
        [s[:, STATE_X] - lr * np.cos(s[:, STATE_YAW]), s[:, STATE_Y] - lr * np.sin(s[:, STATE_YAW])]
    )
    coeffs = np.linalg.lstsq(
        np.column_stack([2 * pts, np.ones(len(pts))]), (pts**2).sum(axis=1), rcond=None
    )[0]
    return math.sqrt(coeffs[2] + coeffs[0] ** 2 + coeffs[1] ** 2)


@pytest.mark.parametrize(
    ("steer_rad", "radius_m", "tol_m"),
    [(0.504, 0.60, 0.04), (-0.449, 0.69, 0.05)],
    ids=["left_lock", "right_lock"],
)
def test_env_low_speed_radius_matches_measurement(steer_rad, radius_m, tol_m):
    r = _rear_axle_radius_m(steer_rad)
    assert abs(r - radius_m) <= tol_m, r
    # and it is the kinematic value (R = L / tan(delta)) to within 5%. Looser than
    # the 2% of the bare-dynamics test because f1tenth_gym's pid_steer is bang-bang at the
    # steering rate limit, so the steer angle chatters by one step (3.2 rad/s * 0.01 s =
    # 0.032 rad, about 6% of 0.5 rad) around its target; observed 2.3% and 3.4%.
    kin = racer_gym.load_vehicle_params().chassis.wheelbase_m / math.tan(abs(steer_rad))
    assert abs(r / kin - 1.0) <= 0.05, (r, kin)
