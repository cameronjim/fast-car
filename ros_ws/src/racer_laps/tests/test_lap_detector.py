"""L1: the lap detector on synthetic tracks."""

from __future__ import annotations

import math
import random

import pytest
from racer_laps.lap_detector import LapCrossing, LapDetector, wrap_angle
from tracks import Pose, circle_poses

TWO_PI = 2.0 * math.pi


def run(det: LapDetector, poses: list[Pose]) -> list[LapCrossing]:
    det.set_start(*poses[0])
    out = []
    for p in poses[1:]:
        c = det.update(*p)
        if c is not None:
            out.append(c)
    return out


def test_one_lap_per_loop_with_index_time_and_length() -> None:
    laps = run(LapDetector(), list(circle_poses(3.0, phase_s=0.013)))
    assert [lap.lap_index for lap in laps] == [1, 2, 3]
    for lap in laps:
        assert abs(lap.lap_time_s - TWO_PI) < 0.02
        assert abs(lap.length_m - TWO_PI) < 0.02
    assert abs(laps[2].t_cross_s - 3 * TWO_PI) < 0.02


@pytest.mark.parametrize("phase_s", [0.0, 0.007, 0.0233, 0.041, 0.0499])
@pytest.mark.parametrize("omega", [0.6, 1.0, 2.0])
def test_lap_time_interpolation_within_20ms_at_20hz(phase_s: float, omega: float) -> None:
    laps = run(LapDetector(), list(circle_poses(2.0, omega=omega, phase_s=phase_s)))
    assert len(laps) == 2
    for k, lap in enumerate(laps, start=1):
        assert abs(lap.t_cross_s - k * TWO_PI / omega) < 0.02
        assert abs(lap.lap_time_s - TWO_PI / omega) < 0.02


def test_clockwise_start_heading_also_works() -> None:
    laps = run(LapDetector(), list(circle_poses(2.0, clockwise=True, phase_s=0.02)))
    assert [lap.lap_index for lap in laps] == [1, 2]


def test_no_lap_while_parked_on_the_line() -> None:
    rng = random.Random(7)
    det = LapDetector()
    det.set_start(0.0, 1.0, 0.0, math.pi / 2)
    for i in range(1, 2000):
        # 2 cm of noise across the line for 100 s
        x = 1.0 + rng.uniform(-0.02, 0.02)
        y = rng.uniform(-0.02, 0.02)
        assert det.update(i * 0.05, x, y, math.pi / 2 + rng.uniform(-0.05, 0.05)) is None
    assert det.laps_completed == 0


def test_no_lap_when_crossing_backwards() -> None:
    # The start heading says +y (counter-clockwise) but the car drives the loop clockwise: every
    # return to the line is a backward crossing, and the forward crossing on the far side of the
    # circle misses the segment.
    poses = list(circle_poses(3.0, clockwise=True))
    poses[0] = (poses[0][0], poses[0][1], poses[0][2], math.pi / 2)
    assert run(LapDetector(), poses) == []


def test_no_lap_before_min_lap_length_then_counts_later() -> None:
    # A circle of 6.28 m with a 7 m minimum: the first return is too short, the second counts.
    poses = list(circle_poses(2.0, phase_s=0.01))
    laps = run(LapDetector(min_lap_length_m=7.0), poses)
    assert [lap.lap_index for lap in laps] == [1]
    assert laps[0].length_m > 12.0
    assert len(run(LapDetector(min_lap_length_m=6.0), poses)) == 2


def test_no_lap_without_leaving_the_line_even_with_enough_path() -> None:
    # Jitter integrates well over 3 m of path but the car never goes anywhere.
    det = LapDetector()
    det.set_start(0.0, 0.0, 0.0, 0.0)
    t = 0.0
    for i in range(200):
        t += 0.05
        # 0.2 m back and forth across the line, 40 cm per sample
        assert det.update(t, -0.2 if i % 2 == 0 else 0.2, 0.0, 0.0) is None
    assert det.path_since_crossing_m > 3.0
    assert det.laps_completed == 0


def test_excursion_parameter_is_validated() -> None:
    with pytest.raises(ValueError):
        LapDetector(min_excursion_m=-1.0)
    assert LapDetector(min_lap_length_m=4.0).min_excursion_m == 1.0


def test_crossing_outside_the_segment_is_ignored() -> None:
    det = LapDetector()
    det.set_start(0.0, 0.0, 0.0, 0.0)  # line along x = 0, y in [-0.75, 0.75]
    det.update(1.0, -1.0, 2.0, 0.0)
    det.update(2.0, -1.0, 5.0, 0.0)
    det.update(3.0, -1.0, 8.0, 0.0)  # path length now above 3 m
    assert det.update(4.0, 1.0, 8.0, 0.0) is None  # crosses x = 0 at y = 8, far off the line
    assert det.update(5.0, -1.0, 0.0, 0.0) is None
    assert det.update(6.0, 1.0, 0.0, 0.0) is not None  # crosses at y = 0: counts


def test_crossing_with_the_wrong_heading_is_ignored() -> None:
    det = LapDetector()
    det.set_start(0.0, 0.0, 0.0, 0.0)
    det.update(1.0, -2.0, 1.0, 0.0)
    det.update(2.0, -2.0, -1.0, 0.0)
    # Crosses the line forward but the car points 90 degrees away from the start heading.
    assert det.update(3.0, 0.5, 0.0, math.pi / 2) is None
    det.update(4.0, -2.0, 0.0, 0.0)
    # 50 degrees off is still inside the 60 degree limit.
    c = det.update(5.0, 1.0, 0.0, math.radians(50.0))
    assert c is not None and c.lap_index == 1


def test_heading_wraps_across_pi() -> None:
    det = LapDetector()
    det.set_start(0.0, 0.0, 0.0, math.pi - 0.05)  # heading almost -x
    det.update(1.0, 2.0, 1.0, math.pi - 0.05)
    det.update(2.0, 2.0, -1.0, math.pi - 0.05)
    det.update(3.0, 1.0, 0.0, math.pi - 0.05)
    c = det.update(4.0, -1.0, 0.0, -math.pi + 0.05)  # yaw wrapped, same direction
    assert c is not None


@pytest.mark.parametrize("gap", [(5.8, 6.8), (2.0, 3.0), (6.0, 6.6)])
def test_pose_dropout_does_not_create_or_lose_a_lap(gap: tuple[float, float]) -> None:
    poses = [p for p in circle_poses(2.0, phase_s=0.011) if not (gap[0] < p[0] < gap[1])]
    laps = run(LapDetector(), poses)
    assert [lap.lap_index for lap in laps] == [1, 2]
    assert abs(laps[0].lap_time_s - TWO_PI) < 0.1
    assert abs(laps[1].t_cross_s - 2 * TWO_PI) < 0.05
    # path across the gap is a chord: a little short, never long
    assert 0.9 * TWO_PI < laps[0].length_m <= TWO_PI + 0.02


def test_long_dropout_with_no_crossing_makes_no_lap() -> None:
    det = LapDetector()
    det.set_start(0.0, 0.0, 0.0, 0.0)
    det.update(1.0, -1.0, 2.0, 0.0)
    # 100 s of nothing, then a pose that is still behind the line
    assert det.update(101.0, -1.0, -2.0, 0.0) is None
    assert det.laps_completed == 0


def test_stale_or_duplicate_timestamps_are_ignored() -> None:
    det = LapDetector()
    det.set_start(5.0, 0.0, 0.0, 0.0)
    assert det.update(5.0, 1.0, 0.0, 0.0) is None
    assert det.update(4.0, 1.0, 0.0, 0.0) is None
    assert det.path_since_crossing_m == 0.0


def test_update_before_start_raises_and_bad_limits_raise() -> None:
    with pytest.raises(RuntimeError):
        LapDetector().update(0.0, 0.0, 0.0, 0.0)
    with pytest.raises(ValueError):
        LapDetector(min_lap_length_m=0.0)


def test_start_line_geometry() -> None:
    det = LapDetector(line_length_m=1.5)
    det.set_start(0.0, 2.0, 3.0, math.pi / 2)  # heading +y: the line runs along x
    (ax, ay), (bx, by) = det.start_line
    assert (ax, ay) == pytest.approx((1.25, 3.0))
    assert (bx, by) == pytest.approx((2.75, 3.0))
    assert det.started


def test_wrap_angle() -> None:
    assert wrap_angle(3 * math.pi) == pytest.approx(-math.pi)
    assert wrap_angle(0.3) == pytest.approx(0.3)
