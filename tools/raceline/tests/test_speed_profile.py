"""L1 tests for raceline.speed_profile: cap respected, a_max respected, hand-computed cases."""

from __future__ import annotations

import math

import numpy as np
import pytest
from raceline.speed_profile import accel_limited_profile_closed, curvature_speed_cap


def test_curvature_speed_cap_straight_is_v_max():
    kappa = np.zeros(10)
    v = curvature_speed_cap(kappa, a_lat_max_mps2=9.51, v_max_mps=20.0)
    np.testing.assert_allclose(v, 20.0)


def test_curvature_speed_cap_matches_hand_computed_value():
    """v = sqrt(a_lat_max / kappa) for a single curvature value, hand-computed."""
    kappa = np.array([1.0 / 3.0])  # R = 3 m turn
    a_lat_max = 9.51
    v = curvature_speed_cap(kappa, a_lat_max_mps2=a_lat_max, v_max_mps=20.0)
    expected = math.sqrt(a_lat_max * 3.0)
    assert v[0] == pytest.approx(expected, rel=1e-9)


def test_curvature_speed_cap_never_exceeds_v_max():
    kappa = np.array([0.0, 1e-6, 0.001, 0.01])
    v = curvature_speed_cap(kappa, a_lat_max_mps2=9.51, v_max_mps=5.0)
    assert np.all(v <= 5.0 + 1e-9)


def test_curvature_speed_cap_rejects_nonpositive_inputs():
    kappa = np.zeros(3)
    with pytest.raises(ValueError):
        curvature_speed_cap(kappa, a_lat_max_mps2=0.0, v_max_mps=10.0)
    with pytest.raises(ValueError):
        curvature_speed_cap(kappa, a_lat_max_mps2=9.51, v_max_mps=0.0)


def test_accel_limited_profile_never_exceeds_cap():
    rng = np.random.default_rng(1)
    n = 200
    v_cap = rng.uniform(1.0, 15.0, size=n)
    seg = np.full(n, 0.1)
    v = accel_limited_profile_closed(v_cap, seg, a_max_mps2=5.0)
    assert np.all(v <= v_cap + 1e-9)


def test_accel_limited_profile_respects_a_max_both_directions():
    rng = np.random.default_rng(2)
    n = 200
    # A deliberately "spiky" cap (alternating high/low) stresses both the forward
    # (acceleration) and backward (deceleration) passes.
    v_cap = np.where(np.arange(n) % 20 < 3, 1.0, 15.0) + rng.uniform(0.0, 0.1, size=n)
    seg = np.full(n, 0.1)
    a_max = 5.0
    v = accel_limited_profile_closed(v_cap, seg, a_max_mps2=a_max, num_passes=4)

    for i in range(n):
        j = (i + 1) % n
        # Implied acceleration going from v[i] to v[j] over seg[i], in EITHER direction of
        # travel around the loop, must not exceed a_max (with a small numerical tolerance).
        accel_forward = (v[j] ** 2 - v[i] ** 2) / (2.0 * seg[i])
        assert accel_forward <= a_max + 1e-6, f"index {i}: implied accel {accel_forward} > a_max"
        decel_backward = (v[i] ** 2 - v[j] ** 2) / (2.0 * seg[i])
        assert decel_backward <= a_max + 1e-6, f"index {i}: implied decel {decel_backward} > a_max"


def test_accel_limited_profile_matches_hand_computed_ramp():
    """A flat, high cap and a single slow point: the profile should ramp away from the
    slow point at exactly a_max (v(s) = sqrt(2*a_max*s)) until it hits the cap."""
    n = 50
    a_max = 4.0
    ds = 0.5
    v_cap = np.full(n, 100.0)
    v_cap[0] = 0.0  # a hard stop point
    seg = np.full(n, ds)
    v = accel_limited_profile_closed(v_cap, seg, a_max_mps2=a_max, num_passes=6)

    assert v[0] == pytest.approx(0.0, abs=1e-9)
    for i in range(1, 10):  # before saturating the (very high) cap
        expected = math.sqrt(2.0 * a_max * i * ds)
        assert v[i] == pytest.approx(expected, rel=1e-6)


def test_accel_limited_profile_rejects_nonpositive_a_max():
    v_cap = np.ones(5)
    seg = np.ones(5)
    with pytest.raises(ValueError):
        accel_limited_profile_closed(v_cap, seg, a_max_mps2=0.0)
