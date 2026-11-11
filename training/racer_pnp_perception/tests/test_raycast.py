from __future__ import annotations

import math

import numpy as np
from racer_pnp_perception import raycast as rc
from racer_track_perception.vehicle import LiveVehicle

WALL_X3 = np.array([[3.0, -5.0, 3.0, 5.0]])


def cast(ox, oy, ang, segs, max_range=12.0):
    return rc.cast_rays(np.array([ox]), np.array([oy]), np.array([ang]), np.array(segs), max_range)[
        0
    ]


def test_hits_a_wall_ahead() -> None:
    assert abs(cast(0, 0, 0.0, WALL_X3) - 3.0) < 1e-12


def test_oblique_hit() -> None:
    assert abs(cast(0, 0, math.radians(45), WALL_X3) - 3.0 / math.cos(math.radians(45))) < 1e-9


def test_misses() -> None:
    assert math.isinf(cast(0, 0, math.pi, WALL_X3))  # wall is behind
    assert math.isinf(cast(0, 0, math.pi / 2, WALL_X3))  # parallel
    assert math.isinf(cast(0, 0, math.radians(80), [[3.0, -1.0, 3.0, 1.0]]))  # beyond the end
    assert math.isinf(cast(0, 0, 0.0, WALL_X3, max_range=2.5))  # beyond max range
    assert math.isinf(cast(0, 0, 0.0, np.zeros((0, 4))))


def test_nearest_of_several_and_endpoint_inclusive() -> None:
    segs = [[3.0, -5.0, 3.0, 5.0], [1.5, -1.0, 1.5, 1.0], [2.0, 0.5, 2.0, 1.0]]
    assert abs(cast(0, 0, 0.0, segs) - 1.5) < 1e-12
    # The ray passes exactly through a segment end point.
    assert abs(cast(0, 0, 0.0, [[2.0, 0.0, 2.0, 1.0]]) - 2.0) < 1e-12


def test_origin_inside_a_box_sees_each_side() -> None:
    box = np.array([[-1, -1, 2, -1], [2, -1, 2, 1], [2, 1, -1, 1], [-1, 1, -1, -1]], dtype=float)
    angles = np.array([0.0, math.pi / 2, math.pi, -math.pi / 2])
    r = rc.cast_rays(np.zeros(4), np.zeros(4), angles, box, 12.0)
    assert np.allclose(r, [2.0, 1.0, 1.0, 1.0])


def test_cull_keeps_near_segments() -> None:
    segs = np.array([[0.0, 0.0, 1.0, 0.0], [50.0, 50.0, 51.0, 50.0]])
    near = rc.cull_segments(segs, 0.0, 0.0, 5.0)
    assert near.shape == (1, 4)


def test_lidar_scan_straight_ahead_is_from_the_head(live: LiveVehicle) -> None:
    geo = rc.ScanGeometry()
    r = rc.lidar_scan((0.0, 0.0, 0.0), live.mount, geo, WALL_X3)
    # The beam nearest laser bearing pi looks straight ahead (mount yaw pi).
    i = int(np.argmin(np.abs(geo.laser_bearings() - math.pi)))
    err = geo.laser_bearings()[i] - math.pi
    assert abs(err) < geo.angle_increment_rad
    expected = (3.0 - live.mount.x_m) / math.cos(err)
    assert abs(r[i] - expected) < 1e-6


def test_lidar_scan_pose_rotation(live: LiveVehicle) -> None:
    geo = rc.ScanGeometry()
    # Facing +y with a wall at y = 2: the head is at y = mount_x, so the straight ahead beam
    # sees the wall at 2 - mount_x.
    wall = np.array([[-5.0, 2.0, 5.0, 2.0]])
    r = rc.lidar_scan((0.0, 0.0, math.pi / 2), live.mount, geo, wall)
    i = int(np.argmin(np.abs(geo.laser_bearings() - math.pi)))
    err = geo.laser_bearings()[i] - math.pi
    assert abs(r[i] - (2.0 - live.mount.x_m) / math.cos(err)) < 1e-6


def test_motion_distortion_moves_the_origin(live: LiveVehicle) -> None:
    geo = rc.ScanGeometry()
    still = rc.lidar_scan((0.0, 0.0, 0.0), live.mount, geo, WALL_X3)
    mov = rc.lidar_scan(
        (0.0, 0.0, 0.0),
        live.mount,
        geo,
        WALL_X3,
        rc.Motion(speed_mps=1.0, yaw_rate_rps=0.0, sweep_s=0.1, sweep_sign=1),
    )
    i = int(np.argmin(np.abs(geo.laser_bearings() - math.pi)))
    t = rc.beam_times(geo.n_beams, rc.Motion())[i]
    assert abs((still[i] - mov[i]) - 1.0 * t) < 1e-3
    assert rc.beam_times(geo.n_beams, rc.Motion())[0] == 0.0
    rev = rc.beam_times(geo.n_beams, rc.Motion(sweep_sign=-1))
    assert rev[0] == 0.1 and rev[-1] < 0.001


def test_base_pose_at_constant_twist() -> None:
    m = rc.Motion(speed_mps=1.0, yaw_rate_rps=2.0)
    t = np.array([0.0, 0.5])
    x, y, yaw = rc.base_pose_at((0.0, 0.0, 0.0), m, t)
    # Arc of radius v / w = 0.5 turned through w t = 1 rad.
    assert abs(x[1] - 0.5 * math.sin(1.0)) < 1e-9
    assert abs(y[1] - 0.5 * (1.0 - math.cos(1.0))) < 1e-9
    assert abs(yaw[1] - 1.0) < 1e-12
