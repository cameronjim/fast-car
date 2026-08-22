"""L1 tests for the analytic synthetic track generators."""

from __future__ import annotations

import math

import numpy as np
import pytest
from raceline.geometry import arc_length_closed, curvature_from_points_closed
from raceline.synthetic_tracks import circle_centerline, stadium_centerline


def test_circle_centerline_rejects_nonpositive_radius():
    with pytest.raises(ValueError):
        circle_centerline(0.0)


def test_stadium_centerline_rejects_nonpositive_inputs():
    with pytest.raises(ValueError):
        stadium_centerline(0.0, 3.0)
    with pytest.raises(ValueError):
        stadium_centerline(8.0, 0.0)


def test_stadium_centerline_total_length_matches_analytic_formula():
    straight_length, turn_radius = 8.0, 3.0
    x, y = stadium_centerline(straight_length, turn_radius, points_per_meter=20.0)
    s, seg = arc_length_closed(x, y)
    total = s[-1] + seg[-1]
    expected = 2.0 * straight_length + 2.0 * math.pi * turn_radius
    assert total == pytest.approx(expected, rel=0.02)


def test_stadium_centerline_curvature_is_zero_or_one_over_r():
    """Every point on a stadium track is either a straight (kappa=0) or a turn
    (kappa=+1/R, both turns are left/CCW by construction) -- a hand-computable case."""
    straight_length, turn_radius = 8.0, 3.0
    x, y = stadium_centerline(straight_length, turn_radius, points_per_meter=20.0)
    kappa = curvature_from_points_closed(x, y)

    near_zero = np.abs(kappa) < 1e-2
    near_one_over_r = np.abs(kappa - 1.0 / turn_radius) < 1e-2
    # A handful of samples right at the straight/arc transition are expected to be
    # inaccurate (finite-difference curvature straddles a curvature discontinuity there);
    # require the overwhelming majority to match one of the two analytic values.
    matches = near_zero | near_one_over_r
    assert np.mean(matches) > 0.97

    assert np.all(kappa > -1e-2), "stadium track should have no right (negative-curvature) turns"


def test_stadium_centerline_is_closed_loop_no_position_jump_at_seam():
    x, y = stadium_centerline(8.0, 3.0, points_per_meter=20.0)
    seam_gap = math.hypot(x[0] - x[-1], y[0] - y[-1])
    mean_step = float(np.mean(np.hypot(np.diff(x), np.diff(y))))
    assert seam_gap < 3.0 * mean_step
