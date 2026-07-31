"""L1 hand-computed cases for raceline.geometry (claude-docs/12-testing.md)."""

from __future__ import annotations

import math

import numpy as np
import pytest
from raceline.geometry import (
    arc_length_closed,
    curvature_from_points_closed,
    heading_from_points_closed,
    max_curvature_discontinuity,
    resample_closed_uniform,
    smooth_closed,
)
from raceline.synthetic_tracks import circle_centerline


def test_arc_length_of_circle_is_circumference():
    radius = 5.0
    x, y = circle_centerline(radius, num_points=1000)
    s, seg = arc_length_closed(x, y)
    total = s[-1] + seg[-1]
    assert total == pytest.approx(2.0 * math.pi * radius, rel=1e-3)


@pytest.mark.parametrize("radius", [1.0, 2.5, 10.0])
def test_ccw_circle_curvature_is_positive_one_over_r(radius):
    """Sign convention (06-vehicle-params.md, REP-103): left turn (CCW) is positive
    curvature, matching left-positive steering."""
    x, y = circle_centerline(radius, num_points=1000, ccw=True)
    kappa = curvature_from_points_closed(x, y)
    np.testing.assert_allclose(kappa, 1.0 / radius, rtol=1e-3)


@pytest.mark.parametrize("radius", [1.0, 2.5, 10.0])
def test_cw_circle_curvature_is_negative_one_over_r(radius):
    """Right turn (CW) is negative curvature -- the sign-convention mirror case."""
    x, y = circle_centerline(radius, num_points=1000, ccw=False)
    kappa = curvature_from_points_closed(x, y)
    np.testing.assert_allclose(kappa, -1.0 / radius, rtol=1e-3)


def test_ccw_circle_curvature_continuity_is_near_zero():
    x, y = circle_centerline(3.0, num_points=1000)
    kappa = curvature_from_points_closed(x, y)
    assert max_curvature_discontinuity(kappa) < 1e-3


def test_heading_of_ccw_circle_matches_analytic_tangent():
    radius = 4.0
    num_points = 720
    theta = np.linspace(0.0, 2.0 * math.pi, num_points, endpoint=False)
    x = radius * np.cos(theta)
    y = radius * np.sin(theta)
    heading = heading_from_points_closed(x, y)
    expected = theta + math.pi / 2.0
    # Compare on the unit circle to sidestep the 2*pi wraparound in angle differences.
    np.testing.assert_allclose(np.cos(heading), np.cos(expected), atol=1e-2)
    np.testing.assert_allclose(np.sin(heading), np.sin(expected), atol=1e-2)


def test_resample_closed_uniform_preserves_circle_shape():
    radius = 6.0
    x, y = circle_centerline(radius, num_points=37)  # deliberately non-uniform-friendly count
    x_u, y_u = resample_closed_uniform(x, y, ds=0.1)
    r = np.hypot(x_u, y_u)
    np.testing.assert_allclose(r, radius, atol=0.05)
    _s, seg = arc_length_closed(x_u, y_u)
    # Uniform spacing: every segment close to the mean segment length.
    assert np.std(seg) < 0.01 * np.mean(seg)


def test_resample_rejects_nonpositive_ds():
    x, y = circle_centerline(1.0, num_points=10)
    with pytest.raises(ValueError):
        resample_closed_uniform(x, y, ds=0.0)


def test_smooth_closed_window_one_is_noop():
    x, y = circle_centerline(2.0, num_points=100)
    x_s, y_s = smooth_closed(x, y, window=1)
    np.testing.assert_array_equal(x_s, x)
    np.testing.assert_array_equal(y_s, y)


def test_smooth_closed_rejects_even_window():
    x, y = circle_centerline(2.0, num_points=100)
    with pytest.raises(ValueError):
        smooth_closed(x, y, window=4)


def test_smooth_closed_reduces_high_frequency_noise():
    rng = np.random.default_rng(0)
    x, y = circle_centerline(5.0, num_points=500)
    noise_scale = 0.02
    x_noisy = x + rng.normal(scale=noise_scale, size=x.shape)
    y_noisy = y + rng.normal(scale=noise_scale, size=y.shape)
    x_s, y_s = smooth_closed(x_noisy, y_noisy, window=9)

    def _roughness(xa, ya):
        return float(
            np.mean(np.hypot(np.diff(np.append(xa, xa[0])), np.diff(np.append(ya, ya[0]))))
        )

    # Smoothing should not increase the average step-to-step jump relative to the noisy input.
    assert _roughness(x_s, y_s) <= _roughness(x_noisy, y_noisy)
