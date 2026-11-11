"""Unit tests for the live observation builder (ROS-free). The test that matters most, that the
builder gives the same vector as training's, lives in training/racer_train/tests/
test_live_observation_parity.py because it needs racer_train and the sim; these tests pin the
pieces on their own with hand-computed answers."""

from __future__ import annotations

import math
from pathlib import Path

import numpy as np
import pytest
from racer_policy import live_observation as lo


def _circle_raceline(radius: float = 2.0, n: int = 400) -> lo.Raceline:
    """A counter-clockwise circle about the origin, curvature 1/radius everywhere."""
    t = np.linspace(0.0, 2.0 * math.pi, n, endpoint=False)
    x, y = radius * np.cos(t), radius * np.sin(t)
    heading = t + math.pi / 2.0
    s = radius * t
    return lo.Raceline(x, y, heading, np.full(n, 1.0 / radius), s)


# -- scan pooling ------------------------------------------------------------------------------


def _sim_geometry_scan(value: float = 3.0) -> tuple[np.ndarray, float, float]:
    n = 1080
    angles = np.linspace(-lo.LIDAR_FOV_RAD / 2.0, lo.LIDAR_FOV_RAD / 2.0, n)
    return np.full(n, value), float(angles[0]), float(angles[1] - angles[0])


def test_pool_scan_flat_scan_gives_that_value_in_every_bin() -> None:
    r, a0, inc = _sim_geometry_scan(3.0)
    out = lo.pool_scan(r, a0, inc, 0.0, 0.05)
    assert out.shape == (36,) and out.dtype == np.float32
    assert np.all(out == 3.0)


def test_pool_scan_clips_to_five_metres() -> None:
    r, a0, inc = _sim_geometry_scan(12.0)
    assert np.all(lo.pool_scan(r, a0, inc, 0.0, 0.05) == lo.LIDAR_CLIP_M)


def test_pool_scan_takes_the_minimum_in_a_bin_not_the_mean() -> None:
    r, a0, inc = _sim_geometry_scan(4.0)
    r[540] = 0.4  # the beam at bearing ~0: bin 18 (the first bin right of centre)
    out = lo.pool_scan(r, a0, inc, 0.0, 0.05)
    assert out[18] == pytest.approx(0.4)
    assert np.count_nonzero(out < 4.0) == 1


def test_pool_scan_treats_nan_inf_and_below_minimum_range_as_no_return() -> None:
    r, a0, inc = _sim_geometry_scan(2.0)
    r[100] = float("nan")
    r[101] = float("inf")
    r[102] = 0.0
    r[103] = 0.01  # below the sensor's 0.05 m minimum
    out = lo.pool_scan(r, a0, inc, 0.0, 0.05)
    assert np.all(out == 2.0)  # the bad beams never become "closer than 2"


def test_pool_scan_mount_yaw_pi_puts_a_head_rear_return_in_front() -> None:
    """The car's head is mounted rotated by pi (vehicle_params sensors.lidar.mount_yaw_rad): a
    laser bearing of 177 degrees is straight ahead of the vehicle (that is how the mount was
    measured), and one at -100 degrees is on the driver's left."""
    n = 500
    inc = 2.0 * math.pi / n
    angle_min = -math.pi
    r = np.full(n, 4.5)
    ahead = round((math.radians(177.0) - angle_min) / inc)
    r[ahead] = 0.5
    out = lo.pool_scan(r, angle_min, inc, math.pi, 0.05)
    centre = np.argmin(out)
    assert centre in (17, 18)  # within half a bin of straight ahead
    assert out[centre] == pytest.approx(0.5)
    left = round((math.radians(-100.0) - angle_min) / inc)
    r2 = np.full(n, 4.5)
    r2[left] = 0.7
    out2 = lo.pool_scan(r2, angle_min, inc, math.pi, 0.05)
    # Vehicle bearing = -100 + 180 = +80 degrees: left of centre, inside the 270 degree window.
    expected_bin = math.floor((math.radians(80.0) + lo.LIDAR_FOV_RAD / 2.0) / (4.7 / 36))
    assert np.argmin(out2) == expected_bin


def test_pool_scan_ignores_beams_behind_the_window() -> None:
    n = 720
    inc = 2.0 * math.pi / n
    r = np.full(n, 4.0)
    r[0] = 0.3  # bearing -pi: directly behind the car
    out = lo.pool_scan(r, -math.pi, inc, 0.0, 0.05)
    assert np.all(out == 4.0)


# -- geometry ----------------------------------------------------------------------------------


def test_compose_pose_matches_hand_computed_se2() -> None:
    # Frame is 1 m along x, rotated 90 degrees; the vehicle is 1 m ahead of the odom origin.
    x, y, yaw = lo.compose_pose((1.0, 0.0, math.pi / 2.0), (1.0, 0.0, 0.1))
    assert (x, y) == pytest.approx((1.0, 1.0))
    assert yaw == pytest.approx(math.pi / 2.0 + 0.1)


def test_compose_pose_wraps_yaw() -> None:
    _, _, yaw = lo.compose_pose((0.0, 0.0, 3.0), (0.0, 0.0, 0.5))
    assert -math.pi < yaw <= math.pi
    assert yaw == pytest.approx(3.5 - 2.0 * math.pi)


def test_lateral_error_sign_left_positive() -> None:
    line = _circle_raceline()
    # Vehicle at (2, 0.3) heading +y (tangent): the nearest raceline point is to its RIGHT
    # (towards the origin the line is at x = 2 - ... ) -> use a pose clearly left of the line.
    lateral, heading, _ = lo.raceline_relative_pose(line, 1.7, 0.0, math.pi / 2.0)
    # Inside the circle, facing counter-clockwise: the line (radius 2) is to the right.
    assert lateral < 0.0
    assert lateral == pytest.approx(-0.3, abs=1e-3)
    assert heading == pytest.approx(0.0, abs=0.02)
    lateral_out, _, _ = lo.raceline_relative_pose(line, 2.3, 0.0, math.pi / 2.0)
    assert lateral_out == pytest.approx(0.3, abs=1e-3)


def test_heading_error_wraps_across_the_seam() -> None:
    line = _circle_raceline()
    i = line.nearest_index(2.0, 0.0)
    assert line.heading[i] == pytest.approx(math.pi / 2.0)
    _, heading, _ = lo.raceline_relative_pose(line, 2.0, 0.0, -math.pi + 0.1)
    # raceline heading pi/2, vehicle yaw -pi+0.1: difference pi/2 + pi - 0.1 wraps to negative.
    assert heading == pytest.approx(lo.wrap_to_pi(math.pi / 2.0 - (-math.pi + 0.1)))
    assert -math.pi < heading <= math.pi


def test_slip_proxy_is_zero_when_the_car_turns_as_commanded() -> None:
    wheelbase, v, delta = 0.33, 1.5, 0.2
    yaw_rate = v * math.tan(delta) / wheelbase
    assert lo.slip_proxy(delta, yaw_rate, v, wheelbase) == pytest.approx(0.0, abs=1e-12)


def test_slip_proxy_is_floored_and_clipped() -> None:
    assert lo.slip_proxy(0.0, 100.0, 1.0, 0.33) == -lo.SLIP_PROXY_LIMIT
    # Standstill: the speed floor keeps it finite.
    assert math.isfinite(lo.slip_proxy(0.3, 0.0, 0.0, 0.33))


# -- the builder -------------------------------------------------------------------------------


def test_field_order_and_width() -> None:
    names = [f[0] for f in lo.LIVE_FIELDS_V0]
    assert names[:5] == [
        "velocity_mps",
        "yaw_rate_rad_s",
        "slip_proxy",
        "raceline_lateral_error_m",
        "raceline_heading_error_rad",
    ]
    b = lo.LiveObservationBuilder(_circle_raceline(), 0.33)
    assert b.dim == 11 + 36


def test_builder_assembles_every_field_in_order() -> None:
    line = _circle_raceline()
    b = lo.LiveObservationBuilder(line, 0.33)
    beams = np.linspace(0.5, 3.0, 36, dtype=np.float32)
    obs = b.build(
        lo.LiveInputs(
            velocity_mps=1.4,
            yaw_rate_rad_s=0.7,
            x_m=2.0,
            y_m=0.0,
            yaw_rad=math.pi / 2.0,
            base_steering_rad=0.1,
            base_speed_mps=1.9,
            prev_steering_cmd_rad=0.09,
            prev_speed_cmd_mps=1.85,
            beams=beams,
        )
    )
    assert obs.dtype == np.float32 and obs.shape == (47,)
    assert obs[0] == pytest.approx(1.4) and obs[1] == pytest.approx(0.7)
    assert obs[3] == pytest.approx(0.0, abs=1e-6)  # on the line
    assert obs[5] == pytest.approx(0.1) and obs[6] == pytest.approx(1.9)
    assert obs[7] == pytest.approx(0.5) and obs[8] == pytest.approx(0.5)  # 1/radius
    assert obs[9] == pytest.approx(0.09) and obs[10] == pytest.approx(1.85)
    assert np.array_equal(obs[11:], beams)


def test_builder_rejects_a_wrong_width_scan_and_clips_a_long_one() -> None:
    b = lo.LiveObservationBuilder(_circle_raceline(), 0.33)

    def inputs(beams: np.ndarray) -> lo.LiveInputs:
        return lo.LiveInputs(1.0, 0.0, 2.0, 0.0, math.pi / 2, 0.0, 1.0, 0.0, 1.0, beams)

    with pytest.raises(ValueError):
        b.build(inputs(np.zeros(35, dtype=np.float32)))
    obs = b.build(inputs(np.full(36, 9.0, dtype=np.float32)))
    assert np.all(obs[11:] == lo.LIDAR_CLIP_M)


def test_index_ahead_wraps_around_the_loop() -> None:
    line = _circle_raceline(n=100)
    spacing = line.length_m / 100
    assert line.index_ahead(99, 3 * spacing) == 2


# -- raceline loading --------------------------------------------------------------------------


def test_raceline_csv_round_trip_and_errors(tmp_path: Path) -> None:
    p = tmp_path / "r.csv"
    p.write_text(
        "# provenance\ns_m,x_m,y_m,heading_rad,curvature_1pm,target_speed_mps\n"
        "0,0,0,0,0,1\n1,1,0,0,0,1\n2,1,1,1.5,1,1\n",
        encoding="utf-8",
    )
    line = lo.Raceline.from_csv(p)
    assert len(line) == 3
    assert line.length_m == pytest.approx(2.0 + math.hypot(1.0, 1.0))
    with pytest.raises(lo.RacelineError):
        lo.Raceline.from_csv(tmp_path / "missing.csv")
    p.write_text("a,b\n1,2\n", encoding="utf-8")
    with pytest.raises(lo.RacelineError, match="header"):
        lo.Raceline.from_csv(p)
    p.write_text("s_m,x_m,y_m,heading_rad,curvature_1pm,target_speed_mps\n", encoding="utf-8")
    with pytest.raises(lo.RacelineError, match="no raceline rows"):
        lo.Raceline.from_csv(p)
    p.write_text(
        "s_m,x_m,y_m,heading_rad,curvature_1pm,target_speed_mps\n0,0,0,0,0\n", encoding="utf-8"
    )
    with pytest.raises(lo.RacelineError, match="6 columns"):
        lo.Raceline.from_csv(p)
