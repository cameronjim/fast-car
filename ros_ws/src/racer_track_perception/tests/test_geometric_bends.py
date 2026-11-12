"""The v1 tracer on bends of a constant lane width (the case v0 got wrong): the true centreline,
the clearance deficit and the self-check intrusion, over lateral offsets and heading errors."""

from __future__ import annotations

import itertools
import math

import numpy as np
import pytest
from racer_track_perception import selfcheck as sc
from racer_track_perception import track as tr
from racer_track_perception.vehicle import LiveVehicle
from test_geometric import SPACING, run, transform, walls

LANE_M = 1.3
INNER_RADIUS_M = 0.4
CENTRE_RADIUS_M = INNER_RADIUS_M + LANE_M / 2
LEAD_M = 1.5
TAIL_M = 4.0
START_X_M = -1.0  # the car starts 1 m before the bend
OFFSETS_M = (-0.3, -0.15, 0.0, 0.15, 0.3)
YAW_ERRORS_RAD = (-0.25, 0.0, 0.25)
CENTRE_TOL_M = 0.06
DEFICIT_TOL_M = 0.05
INTRUSION_TOL = 0.02


def bend_centreline(angle: float, sign: float) -> np.ndarray:
    """Straight lead-in, an arc of `angle` radians (left for sign +1) and a straight tail."""
    pts = [(v - LEAD_M, 0.0) for v in np.arange(0.0, LEAD_M, SPACING)]
    r = CENTRE_RADIUS_M
    pts += [
        (r * math.sin(v / r), r * (1.0 - math.cos(v / r)))
        for v in np.arange(0.0, r * angle, SPACING)
    ]
    x0, y0 = r * math.sin(angle), r * (1.0 - math.cos(angle))
    pts += [
        (x0 + v * math.cos(angle), y0 + v * math.sin(angle))
        for v in np.arange(0.0, TAIL_M, SPACING)
    ]
    c = np.array(pts)
    c[:, 1] *= sign
    return c


def nearest_distance(a: np.ndarray, b: np.ndarray) -> np.ndarray:
    """For each row of `a`, the Euclidean distance to the nearest row of `b`."""
    return np.sqrt(((a[:, None, :] - b[None, :, :]) ** 2).sum(axis=2)).min(axis=1)


def clearance_deficit(t: tr.LocalTrackData, pts: np.ndarray) -> float:
    """Worst over valid stations within 2 m: min half width minus distance to the nearest point."""
    n = t.valid_count
    xy = np.stack([t.x_m[:n], t.y_m[:n]], axis=1).astype(np.float64)
    near = np.hypot(xy[:, 0], xy[:, 1]) <= 2.0
    hw = np.minimum(t.half_width_left_m[:n], t.half_width_right_m[:n])
    return float((hw - nearest_distance(xy, pts))[near].max())


@pytest.mark.parametrize("angle", [math.pi / 2, math.pi], ids=["90deg", "180deg"])
@pytest.mark.parametrize("sign", [1.0, -1.0], ids=["left", "right"])
def test_constant_width_bend_is_traced_on_the_true_centreline(
    live: LiveVehicle, angle: float, sign: float
) -> None:
    centre = bend_centreline(angle, sign)
    world = walls(centre, LANE_M / 2, LANE_M / 2)
    for offset, yaw in itertools.product(OFFSETS_M, YAW_ERRORS_RAD):
        pose = (START_X_M, offset * sign)
        pts = transform(world, pose[0], pose[1], yaw)
        t = run(pts, live)
        n = t.valid_count
        assert n == tr.K_STATIONS, (offset, yaw, n)
        sc.apply_self_check(t, pts)
        c, s = math.cos(yaw), math.sin(yaw)
        stations = np.stack(
            [
                pose[0] + c * t.x_m[:n] - s * t.y_m[:n],
                pose[1] + s * t.x_m[:n] + c * t.y_m[:n],
            ],
            axis=1,
        )
        near = np.hypot(t.x_m[:n], t.y_m[:n]) <= 2.0
        assert near.sum() >= 8
        err = nearest_distance(stations[near], centre).max()
        assert err <= CENTRE_TOL_M, (offset, yaw, err)
        assert clearance_deficit(t, pts) <= DEFICIT_TOL_M, (offset, yaw)
        assert t.intrusion_fraction <= INTRUSION_TOL, (offset, yaw, t.intrusion_fraction)


@pytest.mark.parametrize("sign", [1.0, -1.0], ids=["left", "right"])
def test_one_wall_corner_keeps_the_last_half_width_from_the_outer_wall(
    live: LiveVehicle, sign: float
) -> None:
    centre = bend_centreline(math.pi / 2, sign)
    d = np.gradient(centre, axis=0)
    d /= np.hypot(d[:, 0], d[:, 1])[:, None]
    normal = np.stack([-d[:, 1], d[:, 0]], axis=1)
    inner = centre + sign * (LANE_M / 2) * normal
    outer = centre - sign * (LANE_M / 2) * normal
    # The inner wall is seen on the lead-in (the half width is learned there) and then removed.
    world = np.vstack([outer, inner[centre[:, 0] < -0.2]])
    for offset, yaw in itertools.product((-0.2, 0.0, 0.2), (-0.2, 0.0, 0.2)):
        pose = (START_X_M, offset * sign)
        t = run(transform(world, pose[0], pose[1], yaw), live)
        n = t.valid_count
        assert n == tr.K_STATIONS, (offset, yaw, n)
        last_hw = float(t.half_width_left_m[n - 1])
        assert abs(last_hw - LANE_M / 2) <= 0.06  # learned on the lead-in, not the 0.5 default
        assert np.allclose(t.sigma_m[n - 3 : n], 0.15)  # one wall: the larger sigma
        outer_seen = transform(outer, pose[0], pose[1], yaw)
        xy = np.stack([t.x_m[:n], t.y_m[:n]], axis=1).astype(np.float64)
        near = np.hypot(xy[:, 0], xy[:, 1]) <= 2.0
        dist = nearest_distance(xy[near], outer_seen)
        assert np.abs(dist[-5:] - last_hw).max() <= 0.05, (offset, yaw, dist[-5:], last_hw)
