"""Analytic, network-free synthetic centerlines.

Same motivation as ``sim/bridge/racer_gym_bridge/racer_gym_bridge/bridge_node.py``'s
``build_synthetic_track``: a named f1tenth_gym map (e.g. Spielberg) fetches from
api.f1tenth.org on first use, which this repo's CI and laptop-offline workflow must not
depend on. These generators produce closed-loop centerlines with EXACTLY known analytic
curvature, which is what makes them usable for hand-computed L1 tests
(claude-docs/12-testing.md) as well as for the committed reference track.
"""

from __future__ import annotations

import math

import numpy as np


def circle_centerline(
    radius_m: float, num_points: int = 720, ccw: bool = True
) -> tuple[np.ndarray, np.ndarray]:
    """A circle of the given radius, traversed CCW (left turn, positive curvature) by
    default, or CW (right turn, negative curvature) when ``ccw=False``.

    Analytic curvature is exactly ``+1/radius_m`` (CCW) or ``-1/radius_m`` (CW) everywhere;
    used verbatim as the hand-computed case in ``tests/test_geometry.py``.
    """
    if radius_m <= 0.0:
        raise ValueError(f"radius_m must be positive, got {radius_m}")
    theta = np.linspace(0.0, 2.0 * math.pi, num_points, endpoint=False)
    x = radius_m * np.cos(theta)
    y = radius_m * np.sin(theta)
    if not ccw:
        # Reverse traversal order (not the shape) to flip the direction of travel, and
        # therefore the sign of curvature, while keeping the same set of points.
        x = x[::-1].copy()
        y = y[::-1].copy()
    return x, y


def stadium_centerline(
    straight_length_m: float, turn_radius_m: float, points_per_meter: float = 10.0
) -> tuple[np.ndarray, np.ndarray]:
    """A closed "stadium" (oval) centerline: two straights joined by two semicircular
    turns of ``turn_radius_m``, traversed counter-clockwise (both turns left/positive
    curvature). This is the reference track used for the committed raceline (S.2) and the
    L5 tracker lap canary.

    Geometry (traversal starts at the bottom straight, heading +x):

      - bottom straight: (-L/2, -R) -> (L/2, -R), heading 0, curvature 0
      - right semicircle: center (L/2, 0), heading 0 -> pi, curvature +1/R
      - top straight: (L/2, R) -> (-L/2, R), heading pi, curvature 0
      - left semicircle: center (-L/2, 0), heading pi -> 2*pi, curvature +1/R

    Total path length is ``2*straight_length_m + 2*pi*turn_radius_m``.
    """
    if straight_length_m <= 0.0:
        raise ValueError(f"straight_length_m must be positive, got {straight_length_m}")
    if turn_radius_m <= 0.0:
        raise ValueError(f"turn_radius_m must be positive, got {turn_radius_m}")
    ds = 1.0 / points_per_meter
    half_l = straight_length_m / 2.0
    n_straight = max(round(straight_length_m / ds), 4)
    n_arc = max(round(math.pi * turn_radius_m / ds), 16)

    x_bottom = np.linspace(-half_l, half_l, n_straight, endpoint=False)
    y_bottom = np.full(n_straight, -turn_radius_m)

    theta_right = np.linspace(-math.pi / 2.0, math.pi / 2.0, n_arc, endpoint=False)
    x_right = half_l + turn_radius_m * np.cos(theta_right)
    y_right = turn_radius_m * np.sin(theta_right)

    x_top = np.linspace(half_l, -half_l, n_straight, endpoint=False)
    y_top = np.full(n_straight, turn_radius_m)

    theta_left = np.linspace(math.pi / 2.0, 3.0 * math.pi / 2.0, n_arc, endpoint=False)
    x_left = -half_l + turn_radius_m * np.cos(theta_left)
    y_left = turn_radius_m * np.sin(theta_left)

    x = np.concatenate([x_bottom, x_right, x_top, x_left])
    y = np.concatenate([y_bottom, y_right, y_top, y_left])
    return x, y
