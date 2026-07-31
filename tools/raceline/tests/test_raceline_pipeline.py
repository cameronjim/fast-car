"""L1 end-to-end pipeline invariants on the stadium reference track (claude-docs/12-testing.md):
curvature continuity, speed profile respects the cap and a_max."""

from __future__ import annotations

import numpy as np
from raceline.geometry import arc_length_closed, max_curvature_discontinuity
from raceline.raceline import build_raceline_from_centerline
from raceline.speed_profile import curvature_speed_cap
from raceline.synthetic_tracks import stadium_centerline

A_MAX = 9.51
V_MAX = 20.0


def _build_stadium_raceline():
    x, y = stadium_centerline(straight_length_m=8.0, turn_radius_m=3.0, points_per_meter=10.0)
    return build_raceline_from_centerline(
        x, y, a_lat_max_mps2=A_MAX, a_max_mps2=A_MAX, v_max_mps=V_MAX
    )


def test_pipeline_curvature_continuity():
    result = _build_stadium_raceline()
    # Resampling + smoothing should leave no large point-to-point curvature jumps -- the
    # "curvature continuity" invariant. 1/turn_radius is the largest curvature present
    # (3 m turn radius here); jumps should be a small fraction of that.
    assert max_curvature_discontinuity(result.curvature_1pm) < 0.5 * (1.0 / 3.0)


def test_pipeline_speed_profile_never_exceeds_v_max():
    result = _build_stadium_raceline()
    assert np.all(result.target_speed_mps <= V_MAX + 1e-6)


def test_pipeline_speed_profile_never_exceeds_curvature_cap():
    result = _build_stadium_raceline()
    cap = curvature_speed_cap(result.curvature_1pm, A_MAX, V_MAX)
    assert np.all(result.target_speed_mps <= cap + 1e-6)


def test_pipeline_speed_profile_respects_a_max_between_adjacent_points():
    result = _build_stadium_raceline()
    v = result.target_speed_mps
    _, seg = arc_length_closed(result.x_m, result.y_m)
    n = len(v)
    for i in range(n):
        j = (i + 1) % n
        if seg[i] <= 0:
            continue
        accel = abs(v[j] ** 2 - v[i] ** 2) / (2.0 * seg[i])
        assert accel <= A_MAX + 1e-3, f"index {i}: implied |accel| {accel} exceeds a_max"


def test_pipeline_output_arrays_are_index_aligned_and_closed_loop_sized():
    result = _build_stadium_raceline()
    n = len(result)
    assert len(result.x_m) == n
    assert len(result.y_m) == n
    assert len(result.heading_rad) == n
    assert len(result.curvature_1pm) == n
    assert len(result.target_speed_mps) == n
    assert n > 50
