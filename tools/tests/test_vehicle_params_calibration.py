"""L1: the 2026-11-10 calibration of config/vehicle_params.yaml is internally consistent.

CLAUDE.md invariant 2 keeps one source of truth for every physical constant. Some committed
fields are DERIVED from others (a hand-written duplicate by nature): the drivetrain ratio from a
push test, the throttle full scale from the VESC cap and the drivetrain, the planning steering
limits from the steering table. These tests recompute each one from its sources, so a later edit
to one field that forgets the others fails here instead of on the floor. The measurement records
below (push-test distances, turning radii) are typed in on purpose: they are the provenance, not
a second copy of the parameter.

docs/notes/calibration-2026-11-10.md has the measurements and the method.
"""

from __future__ import annotations

import math
import xml.etree.ElementTree as ET
from pathlib import Path

import pytest
import yaml
from safety_replay import calibration

REPO = Path(__file__).resolve().parents[2]
PARAMS = yaml.safe_load((REPO / "config" / "vehicle_params.yaml").read_text(encoding="utf-8"))

# Push test, 2026-11-10: pushed by hand along a taped 3.00 m twice, throttle killed, the distance
# /odom/wheel logged (computed with the 0.13.0 values below).
PUSH_TRUE_M = 3.00
PUSH_LOGGED_M = (4.139, 4.266)
OLD_RADIUS_M = 0.1095  # the tyre DIAMETER, entered as a radius
OLD_GEAR_RATIO = 11.82  # the stock Traxxas Slash 4x4 overall ratio
TYRE_DIAMETER_M = 0.11  # measured by tape

# LiDAR scan matching at full lock, 2026-11-10: rear-axle turning radius and the effective
# bicycle steer atan(wheelbase / R) the owner recorded.
LEFT_RADIUS_M, LEFT_STEER_RAD = 0.60, 0.504
RIGHT_RADIUS_M, RIGHT_STEER_RAD = 0.69, 0.449


def push_ratio() -> float:
    return sum(PUSH_LOGGED_M) / len(PUSH_LOGGED_M) / PUSH_TRUE_M


def vesc_l_max_erpm() -> float:
    """The speed cap of the newest committed VESC motor configuration."""
    newest = max((REPO / "config" / "vesc").glob("*-fsesc67-motor.xml"))
    return float(ET.parse(newest).getroot().find(".//l_max_erpm").text)


def test_the_tyre_radius_is_half_the_measured_diameter():
    assert PARAMS["tires"]["nominal_radius_m"] == pytest.approx(TYRE_DIAMETER_M / 2.0)


def test_the_gear_ratio_follows_from_the_push_test_and_the_radius():
    # logged distance scales with radius / gear_ratio, so the old pair over the new pair is the
    # logged / true ratio: solve for the ratio that makes the push test read true.
    radius = PARAMS["tires"]["nominal_radius_m"]
    derived = OLD_GEAR_RATIO * (radius / OLD_RADIUS_M) * push_ratio()
    assert PARAMS["drivetrain"]["gear_ratio"] == pytest.approx(derived, abs=0.005)
    assert push_ratio() == pytest.approx(1.401, abs=0.001)


def test_the_speed_scale_the_replay_converts_with_is_the_one_the_parameters_imply():
    new_scale = PARAMS["tires"]["nominal_radius_m"] / PARAMS["drivetrain"]["gear_ratio"]
    implied = (OLD_RADIUS_M / OLD_GEAR_RATIO) / new_scale
    assert calibration.RECORDED_0_13_0.speed_ratio == pytest.approx(implied, rel=0.002)


def test_the_throttle_full_scale_is_the_vesc_cap_at_the_wheel():
    wheel_rps = (
        vesc_l_max_erpm() / PARAMS["drivetrain"]["pole_pairs"] / PARAMS["drivetrain"]["gear_ratio"]
    ) / 60.0
    derived = 2.0 * math.pi * PARAMS["tires"]["nominal_radius_m"] * wheel_rps
    assert PARAMS["actuation"]["throttle_full_scale_mps"] == pytest.approx(derived, rel=0.005)
    # and it is the old 2.91 m/s divided by the push-test ratio
    assert PARAMS["actuation"]["throttle_full_scale_mps"] == pytest.approx(
        2.91 / push_ratio(), rel=0.005
    )


def test_the_steering_table_is_the_measured_per_side_calibration():
    steering = PARAMS["steering"]
    table = sorted((p["pwm_us"], p["angle_rad"]) for p in steering["pwm_to_angle_table"])
    assert [pwm for pwm, _ in table] == [
        steering["pwm_min_us"],
        steering["pwm_neutral_us"],
        steering["pwm_max_us"],
    ]
    angles = [angle for _, angle in table]
    assert angles == [LEFT_STEER_RAD, 0.0, -RIGHT_STEER_RAD]
    assert steering["pwm_left_bound"] == "pwm_min_us"  # left is the short pulse: the table falls
    assert angles == sorted(angles, reverse=True)


def test_the_effective_angles_match_the_measured_turning_radii():
    wheelbase = PARAMS["chassis"]["wheelbase_m"]
    assert math.atan(wheelbase / LEFT_RADIUS_M) == pytest.approx(LEFT_STEER_RAD, abs=0.005)
    assert math.atan(wheelbase / RIGHT_RADIUS_M) == pytest.approx(RIGHT_STEER_RAD, abs=0.005)


def test_planning_is_symmetric_at_the_weaker_side():
    steering = PARAMS["steering"]
    assert steering["max_angle_rad"] == -steering["min_angle_rad"]
    assert steering["max_angle_rad"] == min(LEFT_STEER_RAD, RIGHT_STEER_RAD)
    # Every planner and the safety arc corridor read max_angle_rad alone: the turn it assumes
    # must be one the car can make to BOTH sides.
    radius_planned = PARAMS["chassis"]["wheelbase_m"] / math.tan(steering["max_angle_rad"])
    assert radius_planned >= LEFT_RADIUS_M
    assert radius_planned == pytest.approx(RIGHT_RADIUS_M, abs=0.01)


def test_the_overhangs_are_the_taped_ones():
    chassis = PARAMS["chassis"]
    assert chassis["front_overhang_m"] == 0.11  # 10 to 11 cm, the upper end (brakes earlier)
    assert chassis["rear_overhang_m"] == 0.14
