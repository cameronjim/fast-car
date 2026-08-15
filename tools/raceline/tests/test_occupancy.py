"""L1 fixture-map test: raceline stays within track bounds (claude-docs/12-testing.md)."""

from __future__ import annotations

import numpy as np
import pytest
from raceline.occupancy import annulus_track_mask, centerline_from_track_mask


def test_annulus_fixture_extracted_centerline_within_track_bounds():
    inner_r, outer_r = 3.0, 5.0
    grid, resolution, origin = annulus_track_mask(
        resolution_m=0.05, inner_radius_m=inner_r, outer_radius_m=outer_r
    )
    x, y = centerline_from_track_mask(
        grid, resolution, origin, center_xy=(0.0, 0.0), num_angle_samples=180
    )
    r = np.hypot(x, y)
    # "stays within track bounds": every extracted point is strictly between the inner and
    # outer edges of the fixture annulus, never outside the drivable ring.
    assert np.all(r >= inner_r), f"min extracted radius {r.min()} below inner bound {inner_r}"
    assert np.all(r <= outer_r), f"max extracted radius {r.max()} above outer bound {outer_r}"


def test_annulus_fixture_extracted_centerline_matches_hand_computed_midline():
    """A uniform annulus's true centerline is the circle at the mean radius -- a
    hand-computable case, up to grid-resolution discretization error."""
    inner_r, outer_r = 3.0, 5.0
    resolution = 0.05
    grid, resolution, origin = annulus_track_mask(
        resolution_m=resolution, inner_radius_m=inner_r, outer_radius_m=outer_r
    )
    x, y = centerline_from_track_mask(
        grid, resolution, origin, center_xy=(0.0, 0.0), num_angle_samples=360
    )
    r = np.hypot(x, y)
    expected_mid = (inner_r + outer_r) / 2.0
    assert np.mean(r) == pytest.approx(expected_mid, abs=3.0 * resolution)
    assert np.std(r) < 3.0 * resolution


def test_annulus_track_mask_rejects_invalid_radii():
    with pytest.raises(ValueError):
        annulus_track_mask(resolution_m=0.1, inner_radius_m=5.0, outer_radius_m=3.0)
    with pytest.raises(ValueError):
        annulus_track_mask(resolution_m=0.1, inner_radius_m=0.0, outer_radius_m=3.0)


def test_centerline_from_track_mask_raises_when_center_misses_ring():
    grid, resolution, origin = annulus_track_mask(
        resolution_m=0.1, inner_radius_m=3.0, outer_radius_m=5.0, margin_m=6.0
    )
    # A "center" far outside the grid entirely: every ray misses the ring.
    with pytest.raises(ValueError):
        centerline_from_track_mask(grid, resolution, origin, center_xy=(1000.0, 1000.0))
