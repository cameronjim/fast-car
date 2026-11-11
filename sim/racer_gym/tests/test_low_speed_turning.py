"""Low-speed turning pinned to the real car, and the tyre-unit regression behind it
(sim tyre fix, docs/notes/sim-tyre-fix-2026-11-11.md).

Measured on the car (scan matching, 2026-11-10, true speed about 0.27 m/s): rear-axle turning
radius 0.60 m at left lock and 0.69 m at right lock, with effective road-wheel angles 0.504 rad
(left) and 0.449 rad (right) on a 0.3302 m wheelbase. Interquartile range of the per-segment
radius estimates: left 0.56 to 0.64 m, right 0.64 to 0.74 m. Those are the tolerances below
(half the IQR: 0.04 m left, 0.05 m right). They are a statement about the measurement spread,
not about the model: the model is within 1% of kinematic geometry, which is what the car does at
that speed (L / tan(delta) = 0.599 m and 0.685 m, inside both bands).

The pre-fix placeholder tyre (f1tenth_gym's dimensionless C_Sf = 4.7 used as N/rad, a factor
mu * Fz = 20 too soft) gave 0.64 / 0.73 m at 0.3 m/s (at the edge of those bands, which is why
the 2% kinematic test below is the one that bites) and 1.76 / 1.85 m at 2 m/s.

The effective steer angles exceed the committed symmetric +-0.4189 rad limit (the calibration
that raises it is on another branch), so these tests widen the limit on an in-memory copy of the
binding; the yaml is not touched.
"""

from __future__ import annotations

import dataclasses
import math

import numpy as np
import pytest
from f1tenth_gym.envs.dynamic_models import vehicle_dynamics_st
from racer_gym.dynamics.load_transfer import static_axle_loads
from racer_gym.dynamics.model import (
    RacerRK4Integrator,
    RacerSingleTrackModel,
    racer_dynamics_st,
    stiffness_rate_per_s,
)
from racer_gym.params import (
    _f1tenth_gym_defaults,
    build_dyn_params,
    gym_cornering_stiffness_n_per_rad,
    load_vehicle_params,
)

# Measurement (see module docstring). Angles are effective road-wheel angles, left positive.
LEFT_ANGLE_RAD = 0.504
RIGHT_ANGLE_RAD = 0.449
LEFT_RADIUS_M = 0.60
RIGHT_RADIUS_M = 0.69
LEFT_TOL_M = 0.04
RIGHT_TOL_M = 0.05
MEASURED_SPEED_MPS = 0.3  # the car was at about 0.27 m/s; 0.3 is the nearest round figure


@pytest.fixture(scope="module")
def dyn():
    base = load_vehicle_params()
    wide = dataclasses.replace(
        base.steering, min_angle_rad=-0.6, max_angle_rad=0.6
    )  # in-memory only
    return build_dyn_params(dataclasses.replace(base, steering=wide), 0.01).dyn_params


def rear_axle_radius_m(dyn, steer_rad: float, speed_mps: float, *, dt_s: float = 0.01) -> float:
    """Steady rear-axle turning radius at a fixed steer angle and speed (no actuator lag or
    delay, steering held at `steer_rad` and the speed held, so only the tyre/chassis model and
    the integrator are under test). Least-squares circle through the rear-axle path over the
    last two thirds of the run, which covers several radians of arc."""
    seconds = 40.0 if speed_mps < 0.2 else 12.0
    model = RacerSingleTrackModel(dyn)
    integrator = RacerRK4Integrator()
    state = np.array([0.0, 0.0, steer_rad, speed_mps, 0.0, 0.0, 0.0])
    hold = np.array([0.0, 0.0])  # no steer velocity, no acceleration
    points = []
    for i in range(round(seconds / dt_s)):
        state = integrator.integrate(model.f_dynamics, state, hold, dt_s, None)
        if i * dt_s > seconds / 3:
            points.append(
                (
                    state[0] - dyn.cg_to_rear_axle_m * math.cos(state[4]),
                    state[1] - dyn.cg_to_rear_axle_m * math.sin(state[4]),
                )
            )
    p = np.array(points)
    coeffs = np.linalg.lstsq(
        np.column_stack([2 * p, np.ones(len(p))]), (p**2).sum(axis=1), rcond=None
    )[0]
    return math.sqrt(coeffs[2] + coeffs[0] ** 2 + coeffs[1] ** 2)


def kinematic_radius_m(dyn, steer_rad: float) -> float:
    return (dyn.cg_to_front_axle_m + dyn.cg_to_rear_axle_m) / math.tan(steer_rad)


# --------------------------------------------------------------------------------------
# Pinned to the measurement
# --------------------------------------------------------------------------------------


def test_left_lock_radius_matches_measurement(dyn):
    r = rear_axle_radius_m(dyn, LEFT_ANGLE_RAD, MEASURED_SPEED_MPS)
    assert abs(r - LEFT_RADIUS_M) <= LEFT_TOL_M, r


def test_right_lock_radius_matches_measurement(dyn):
    r = rear_axle_radius_m(dyn, -RIGHT_ANGLE_RAD, MEASURED_SPEED_MPS)
    assert abs(r - RIGHT_RADIUS_M) <= RIGHT_TOL_M, r


# --------------------------------------------------------------------------------------
# Approaches the kinematic bicycle at low speed
# --------------------------------------------------------------------------------------


@pytest.mark.parametrize("speed_mps", [0.1, 0.3])
@pytest.mark.parametrize("steer_rad", [LEFT_ANGLE_RAD, -RIGHT_ANGLE_RAD, 0.2])
def test_low_speed_radius_is_kinematic_within_two_percent(dyn, speed_mps, steer_rad):
    """The understeer term scales with v^2, so at crawl speed R -> L / tan(delta). 2% is
    far above the numerical error (under 0.5%) and below the 7% the old tyre gave at 0.3 m/s."""
    r = rear_axle_radius_m(dyn, steer_rad, speed_mps)
    r_kin = abs(kinematic_radius_m(dyn, steer_rad))
    assert abs(r / r_kin - 1.0) <= 0.02, (r, r_kin)


def test_understeer_grows_with_speed_toward_the_physical_value(dyn):
    """R/R_kin rises monotonically with speed (understeer gradient is positive for the
    placeholder tyres), and stays physically small at 1 m/s: front axle stiffer per unit
    load than the rear is not assumed, but a correctly sized tyre adds under 10% there. The
    unit bug added 61%."""
    ratios = [
        rear_axle_radius_m(dyn, LEFT_ANGLE_RAD, v) / kinematic_radius_m(dyn, LEFT_ANGLE_RAD)
        for v in (0.1, 0.3, 1.0)
    ]
    assert ratios == sorted(ratios)
    assert ratios[-1] < 1.10


# --------------------------------------------------------------------------------------
# The unit fix itself
# --------------------------------------------------------------------------------------


def test_placeholder_stiffness_matches_stock_gym_force_model():
    """The placeholder's small-slip stiffness must equal what stock f1tenth_gym's own linear
    model implies in newtons per radian: F_y = mu * C_S * Fz * alpha."""
    vp = load_vehicle_params()
    result = build_dyn_params(vp, 0.01).dyn_params
    gym = _f1tenth_gym_defaults()
    static = static_axle_loads(
        vp.chassis.mass_kg, vp.chassis.cg_to_front_axle_m, vp.chassis.cg_to_rear_axle_m
    )
    mu = vp.tires.surface_friction_coefficient
    for pac, c_s, fz in (
        (result.pacejka_front, gym["C_Sf"], static.front_n),
        (result.pacejka_rear, gym["C_Sr"], static.rear_n),
    ):
        slope = pac.b_stiffness * pac.c_shape * pac.d_peak_n
        assert math.isclose(slope, mu * c_s * fz, rel_tol=1e-9)
        # Physical sanity for a 3.7 kg car: tens to hundreds of N/rad, not single digits.
        assert 50.0 < slope < 400.0


def test_conversion_helper_is_mu_times_c_times_load():
    assert gym_cornering_stiffness_n_per_rad(5.0, 1.0, 20.0) == pytest.approx(100.0)
    assert gym_cornering_stiffness_n_per_rad(5.0, 0.5, 20.0) == pytest.approx(50.0)


def test_small_angle_dynamics_agree_with_stock_gym(dyn):
    """At a small slip state (a = 0, 2 m/s, no lateral transfer since track width is null),
    racer_gym's yaw and slip accelerations match f1tenth_gym's own `vehicle_dynamics_st`
    built from the same mu, C_S, geometry and inertia, within 3% (the residual is the
    Pacejka shape curvature and cos(delta)). Before the fix they differed by a factor of
    about 20."""
    vp = load_vehicle_params()
    gym = _f1tenth_gym_defaults()
    x = np.array([0.0, 0.0, 0.02, 2.0, 0.0, 0.05, 0.01])
    u = np.array([0.0, 0.0])
    ours = racer_dynamics_st(x, u, dyn)
    theirs = vehicle_dynamics_st(
        x,
        u,
        gym["mu"],
        gym["C_Sf"],
        gym["C_Sr"],
        gym["lf"],
        gym["lr"],
        gym["h"],
        gym["m"],
        gym["I"],
        gym["s_min"],
        gym["s_max"],
        gym["sv_min"],
        gym["sv_max"],
        gym["v_switch"],
        gym["a_max"],
        gym["v_min"],
        gym["v_max"],
    )
    # The committed vehicle_params were copied from these gym defaults (config header comment).
    assert vp.chassis.mass_kg == gym["m"] and vp.tires.surface_friction_coefficient == gym["mu"]
    # psi_ddot is index 5, beta_dot is index 6.
    assert ours[5] == pytest.approx(theirs[5], rel=0.03)
    assert ours[6] == pytest.approx(theirs[6], rel=0.03)


# --------------------------------------------------------------------------------------
# The integrator: stable at low speed at the 0.01 s step
# --------------------------------------------------------------------------------------


def test_low_speed_result_does_not_depend_on_the_step(dyn):
    """Fixed-step RK4 at dt = 0.01 s is unstable below about 0.4 m/s with physically sized
    tyres (the tyre modes relax at about 1/V). Substepping makes the 0.01 s result agree with
    a 0.001 s reference to 1%; without it the 0.3 m/s full-lock radius was 0.57 m, and 1.07 m
    (instead of 0.60 m) at 0.1 m/s."""
    for speed in (0.1, 0.3):
        coarse = rear_axle_radius_m(dyn, LEFT_ANGLE_RAD, speed, dt_s=0.01)
        fine = rear_axle_radius_m(dyn, LEFT_ANGLE_RAD, speed, dt_s=0.001)
        assert abs(coarse / fine - 1.0) < 0.01, (speed, coarse, fine)


def test_stiff_randomised_tyres_are_also_stable_at_low_speed(dyn):
    """The v0 randomisation reaches 400 N/rad; the integrator must cope with stiffer tyres
    than the placeholder, not just the placeholder."""
    stiff_front = dataclasses.replace(
        dyn.pacejka_front, b_stiffness=dyn.pacejka_front.b_stiffness * 4.0
    )
    stiff_rear = dataclasses.replace(
        dyn.pacejka_rear, b_stiffness=dyn.pacejka_rear.b_stiffness * 4.0
    )
    stiff = dataclasses.replace(dyn, pacejka_front=stiff_front, pacejka_rear=stiff_rear)
    r = rear_axle_radius_m(stiff, LEFT_ANGLE_RAD, 0.3)
    assert abs(r / kinematic_radius_m(stiff, LEFT_ANGLE_RAD) - 1.0) < 0.02


def test_substeps_only_when_stiff_and_bounded(dyn):
    f = RacerSingleTrackModel(dyn).f_dynamics
    sub = RacerRK4Integrator._substeps
    slow = np.array([0, 0, 0, 0.1, 0, 0, 0.0])
    fast = np.array([0, 0, 0, 3.0, 0, 0, 0.0])
    assert sub(f, slow, 0.01) > 1
    assert sub(f, fast, 0.01) == 1  # no extra cost at racing speed
    assert sub(f, np.array([0, 0, 0, 0.0, 0, 0, 0.0]), 0.01) <= 64
    assert stiffness_rate_per_s(slow, dyn) > stiffness_rate_per_s(fast, dyn)
    # A plain callable with no stiffness estimate gets a single step (upstream behaviour).
    assert sub(lambda x, u, p: x * 0.0, slow, 0.01) == 1
