"""L1/L2 tests for racer_gym.dynamics.tire (Pacejka curve shape, sign, front/rear
separation -- claude-docs/12-testing.md, claude-docs/07-sim-and-sysid.md requirement 2)."""

from __future__ import annotations

import math

import pytest
from hypothesis import given
from hypothesis import strategies as st
from racer_gym.dynamics.tire import PacejkaParams, lateral_force, peak_slip_angle_rad

FRONT = PacejkaParams(b_stiffness=8.0, c_shape=1.3, d_peak_n=20.0, e_curvature=0.0)
REAR = PacejkaParams(b_stiffness=10.0, c_shape=1.3, d_peak_n=18.0, e_curvature=0.0)
FZ_NOMINAL_N = 20.0


# --------------------------------------------------------------------------------------
# L1: hand-computed / analytic cases
# --------------------------------------------------------------------------------------


def test_zero_slip_angle_gives_zero_force():
    assert lateral_force(0.0, FZ_NOMINAL_N, FZ_NOMINAL_N, FRONT) == 0.0


def test_sign_convention_positive_alpha_gives_negative_restoring_force():
    """claude-docs/06-vehicle-params.md: slip angle positive when velocity points left of
    heading; the tire resists that slip, so the restoring force is negative (rightward)."""
    fy = lateral_force(0.05, FZ_NOMINAL_N, FZ_NOMINAL_N, FRONT)
    assert fy < 0.0


def test_sign_symmetry():
    alpha = 0.07
    fy_pos = lateral_force(alpha, FZ_NOMINAL_N, FZ_NOMINAL_N, FRONT)
    fy_neg = lateral_force(-alpha, FZ_NOMINAL_N, FZ_NOMINAL_N, FRONT)
    assert math.isclose(fy_pos, -fy_neg, rel_tol=1e-9)


def test_peak_location_matches_closed_form():
    alpha_peak = peak_slip_angle_rad(FRONT)
    fy_at_peak = lateral_force(alpha_peak, FZ_NOMINAL_N, FZ_NOMINAL_N, FRONT)
    fy_before = lateral_force(alpha_peak * 0.9, FZ_NOMINAL_N, FZ_NOMINAL_N, FRONT)
    fy_after = lateral_force(alpha_peak * 1.1, FZ_NOMINAL_N, FZ_NOMINAL_N, FRONT)
    assert abs(fy_at_peak) >= abs(fy_before)
    assert abs(fy_at_peak) >= abs(fy_after)
    assert math.isclose(abs(fy_at_peak), FRONT.d_peak_n, rel_tol=1e-6)


def test_front_rear_curves_are_independent():
    alpha = 0.05
    fy_front = lateral_force(alpha, FZ_NOMINAL_N, FZ_NOMINAL_N, FRONT)
    fy_rear = lateral_force(alpha, FZ_NOMINAL_N, FZ_NOMINAL_N, REAR)
    assert not math.isclose(fy_front, fy_rear, rel_tol=1e-6)


def test_force_scales_linearly_with_normal_load_at_fixed_slip():
    alpha = 0.03
    fy_half = lateral_force(alpha, FZ_NOMINAL_N / 2.0, FZ_NOMINAL_N, FRONT)
    fy_full = lateral_force(alpha, FZ_NOMINAL_N, FZ_NOMINAL_N, FRONT)
    assert math.isclose(fy_half, fy_full / 2.0, rel_tol=1e-9)


def test_zero_or_negative_normal_load_gives_zero_force():
    assert lateral_force(0.1, 0.0, FZ_NOMINAL_N, FRONT) == 0.0
    assert lateral_force(0.1, -5.0, FZ_NOMINAL_N, FRONT) == 0.0


def test_nonpositive_fz_nominal_raises():
    with pytest.raises(ValueError):
        lateral_force(0.1, FZ_NOMINAL_N, 0.0, FRONT)
    with pytest.raises(ValueError):
        lateral_force(0.1, FZ_NOMINAL_N, -1.0, FRONT)


def test_peak_location_requires_zero_curvature():
    with_curvature = PacejkaParams(b_stiffness=8.0, c_shape=1.3, d_peak_n=20.0, e_curvature=0.2)
    with pytest.raises(ValueError):
        peak_slip_angle_rad(with_curvature)


# --------------------------------------------------------------------------------------
# L2: property-based tests (hypothesis)
# --------------------------------------------------------------------------------------


@given(
    alpha=st.floats(min_value=-1.5, max_value=1.5, allow_nan=False),
    fz_n=st.floats(min_value=0.0, max_value=200.0, allow_nan=False),
    mu=st.floats(min_value=0.1, max_value=3.0, allow_nan=False),
    b=st.floats(min_value=1.0, max_value=20.0, allow_nan=False),
    c=st.floats(min_value=1.0, max_value=2.0, allow_nan=False),
)
def test_force_magnitude_bounded_by_mu_times_fz(alpha, fz_n, mu, b, c):
    """claude-docs/12-testing.md L2: 'tire force magnitude bounded by mu*Fz'. d_peak_n is
    defined as the peak force AT the nominal load, i.e. mu*Fz_nominal; scaling by fz_n /
    fz_nominal_n therefore bounds |Fy| by mu*fz_n for any fz_n (the Magic Formula's sin(...)
    term is itself bounded in [-1, 1])."""
    fz_nominal_n = 20.0
    params = PacejkaParams(b_stiffness=b, c_shape=c, d_peak_n=mu * fz_nominal_n, e_curvature=0.0)
    fy = lateral_force(alpha, fz_n, fz_nominal_n, params)
    assert abs(fy) <= mu * fz_n + 1e-9


@given(alpha=st.floats(min_value=-1.5, max_value=1.5, allow_nan=False))
def test_sign_symmetry_property(alpha):
    fy_pos = lateral_force(alpha, FZ_NOMINAL_N, FZ_NOMINAL_N, FRONT)
    fy_neg = lateral_force(-alpha, FZ_NOMINAL_N, FZ_NOMINAL_N, FRONT)
    assert math.isclose(fy_pos, -fy_neg, rel_tol=1e-9, abs_tol=1e-12)
