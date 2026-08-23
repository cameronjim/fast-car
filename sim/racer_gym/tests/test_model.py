"""L1 tests for racer_gym.dynamics.model.racer_dynamics_st: sign conventions per
claude-docs/06-vehicle-params.md (left turn positive, slip angle positive when velocity
points left of heading) and end-to-end load-transfer integration."""

from __future__ import annotations

import numpy as np
from racer_gym.dynamics.model import racer_dynamics_st
from racer_gym.dynamics.tire import PacejkaParams
from racer_gym.params import DynParams

FRONT = PacejkaParams(b_stiffness=8.0, c_shape=1.3, d_peak_n=20.0, e_curvature=0.0)
REAR = PacejkaParams(b_stiffness=10.0, c_shape=1.3, d_peak_n=18.0, e_curvature=0.0)


def make_dyn_params(**overrides) -> DynParams:
    defaults = {
        "mass_kg": 3.74,
        "cg_height_m": 0.074,
        "cg_to_front_axle_m": 0.15875,
        "cg_to_rear_axle_m": 0.17145,
        "yaw_inertia_kg_m2": 0.04712,
        "track_width_m": 0.25,
        "pacejka_front": FRONT,
        "pacejka_rear": REAR,
        "steer_tau_s": 0.0,
        "throttle_tau_s": 0.0,
        "delay_steps": 0,
        "s_min": -0.4189,
        "s_max": 0.4189,
        "sv_min": -3.2,
        "sv_max": 3.2,
        "v_switch": 7.319,
        "a_max": 9.51,
        "v_min": -5.0,
        "v_max": 20.0,
    }
    defaults.update(overrides)
    return DynParams(**defaults)


def make_state(x=0.0, y=0.0, delta=0.0, v=5.0, psi=0.0, psi_dot=0.0, beta=0.0) -> np.ndarray:
    return np.array([x, y, delta, v, psi, psi_dot, beta], dtype=np.float64)


# --------------------------------------------------------------------------------------
# Sign conventions (claude-docs/06-vehicle-params.md)
# --------------------------------------------------------------------------------------


def test_left_steer_produces_left_yaw_acceleration():
    """Steering angle: LEFT positive. From rest-ish straight travel, a positive steer angle
    should produce a positive (CCW / left-turning) yaw acceleration."""
    dyn_params = make_dyn_params()
    state = make_state(delta=0.1)
    dx = racer_dynamics_st(state, np.array([0.0, 0.0]), dyn_params)
    psi_dot_dot = dx[5]
    assert psi_dot_dot > 0.0


def test_right_steer_produces_right_yaw_acceleration():
    dyn_params = make_dyn_params()
    state = make_state(delta=-0.1)
    dx = racer_dynamics_st(state, np.array([0.0, 0.0]), dyn_params)
    psi_dot_dot = dx[5]
    assert psi_dot_dot < 0.0


def test_zero_steer_zero_slip_gives_zero_yaw_acceleration():
    dyn_params = make_dyn_params()
    state = make_state(delta=0.0, beta=0.0, psi_dot=0.0)
    dx = racer_dynamics_st(state, np.array([0.0, 0.0]), dyn_params)
    assert dx[5] == 0.0
    assert dx[6] == 0.0


def test_positive_body_slip_angle_is_self_restoring():
    """Slip angle positive when velocity points left of heading (claude-docs/06-vehicle-
    params.md). With no steering input, a vehicle already slipping "left of heading" (beta >
    0) should generate tire forces that push beta back toward zero (beta_dot < 0) -- the
    classic tire self-centering / restoring behavior."""
    dyn_params = make_dyn_params()
    state = make_state(delta=0.0, beta=0.05, psi_dot=0.0)
    dx = racer_dynamics_st(state, np.array([0.0, 0.0]), dyn_params)
    beta_dot = dx[6]
    assert beta_dot < 0.0


def test_negative_body_slip_angle_is_self_restoring():
    dyn_params = make_dyn_params()
    state = make_state(delta=0.0, beta=-0.05, psi_dot=0.0)
    dx = racer_dynamics_st(state, np.array([0.0, 0.0]), dyn_params)
    beta_dot = dx[6]
    assert beta_dot > 0.0


def test_kinematic_terms_match_standard_bicycle_kinematics():
    dyn_params = make_dyn_params()
    v, psi, beta = 6.0, 0.3, 0.02
    state = make_state(v=v, psi=psi, beta=beta)
    dx = racer_dynamics_st(state, np.array([0.0, 0.0]), dyn_params)
    assert np.isclose(dx[0], v * np.cos(psi + beta))
    assert np.isclose(dx[1], v * np.sin(psi + beta))


# --------------------------------------------------------------------------------------
# Load transfer integration: accelerating reduces front-axle grip, hence reduces the
# magnitude of the yaw moment produced by the SAME steering input, relative to coasting.
# --------------------------------------------------------------------------------------


def test_acceleration_reduces_front_axle_yaw_authority():
    dyn_params = make_dyn_params()
    state = make_state(delta=0.15, v=8.0)

    dx_coast = racer_dynamics_st(state, np.array([0.0, 0.0]), dyn_params)
    dx_accel = racer_dynamics_st(state, np.array([0.0, 6.0]), dyn_params)

    # Hard acceleration transfers load off the front axle, reducing its available lateral
    # force at the same slip angle, hence a smaller-magnitude yaw response from the same
    # steer angle.
    assert abs(dx_accel[5]) < abs(dx_coast[5])


def test_braking_increases_front_axle_yaw_authority():
    dyn_params = make_dyn_params()
    state = make_state(delta=0.15, v=8.0)

    dx_coast = racer_dynamics_st(state, np.array([0.0, 0.0]), dyn_params)
    dx_brake = racer_dynamics_st(state, np.array([0.0, -6.0]), dyn_params)

    assert abs(dx_brake[5]) > abs(dx_coast[5])
