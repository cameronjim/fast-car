"""Old-unit bags in the units of the committed vehicle_params (the 2026-11-10 calibration).

Every bag recorded up to and including 2026-11-10 was driven with vehicle_params 0.13.0, whose
speed scale and steering map were wrong (docs/notes/calibration-2026-11-10.md):

  * SPEED. The tyre radius was entered as the tyre diameter and the drivetrain ratio was the stock
    one, so every speed the software believed (/odom/wheel) and every speed it requested
    (/drive_raw, /drive, which the VESC speed loop turns into ERPM with the same scale) was
    `SPEED_RATIO` times the true speed on a straight. True speed = recorded / SPEED_RATIO.
  * STEERING. The pulse map was linear and symmetric: a recorded angle delta went to a pulse of
    neutral -/+ |delta| / 0.4189 of the travel to the stop on that side. What the road wheels did
    for that pulse is the measured per-side table in the committed file
    (steering.pwm_to_angle_table), so the real angle of a recorded command is
    table(old_pulse(delta)): about 1.2 delta to the left and 1.07 delta to the right.

`convert_bag` does that to the three topics that carry those units, so a bag replays through the
gate in the units the car now runs in. Times, scans and safety events are untouched, and so is
everything measured from the LiDAR (end clearance, stopping travel, approach speed): those were
always real. The converted steering can exceed the committed planning limit on the left (the
physical left stop is at 0.504 rad, the planning limit 0.449 rad), which is why
`Current.steer_limit_rad` exists: the gate replay is given that lock, not the planning limit.

The speed scale is a straight-line figure. At full lock the wheels slip and the car is slower than
speed / SPEED_RATIO (measured about 1.5 to 1.9 against the odometry on 2026-11-10), so a converted
cornering speed is an upper bound on the real one. That errs toward earlier brakes in the replay.
"""

from __future__ import annotations

import dataclasses
import itertools
import math
from dataclasses import dataclass
from pathlib import Path

import yaml
from follower_replay.bag import BagData, DriveRecord, OdomRecord

REPO_ROOT = Path(__file__).resolve().parents[2]
PARAMS_PATH = REPO_ROOT / "config" / "vehicle_params.yaml"


@dataclass(frozen=True)
class Recorded:
    """The calibration a bag was recorded under."""

    label: str
    speed_ratio: float  # recorded speed / true speed on a straight
    steering_limit_rad: float  # the angle the old linear map sent to each stop


# vehicle_params 0.13.0 and earlier. The 1.401 is the owner's push test of 2026-11-10 (3.00 m
# taped, /odom/wheel logged 4.139 and 4.266 m); scan matching of the 2026-11-10 driven bags gives
# 1.35 to 1.43 on a straight (docs/notes/calibration-2026-11-10.md). 0.4189 is the gym's s_max the
# old map put at both stops.
RECORDED_0_13_0 = Recorded(label="0.13.0", speed_ratio=1.401, steering_limit_rad=0.4189)


@dataclass(frozen=True)
class Current:
    """The committed steering map: pulse ends, neutral and the per-side table."""

    pwm_left_us: float  # pulse at the left stop
    pwm_neutral_us: float
    pwm_right_us: float  # pulse at the right stop
    table: tuple[tuple[float, float], ...]  # (pwm_us, angle_rad), pwm ascending
    planning_limit_rad: float  # steering.max_angle_rad

    @property
    def steer_limit_rad(self) -> float:
        """The largest |angle| the table can produce: the real lock the gate must allow."""
        return max(abs(angle) for _, angle in self.table)


def load_current(path: Path = PARAMS_PATH) -> Current:
    params = yaml.safe_load(path.read_text(encoding="utf-8"))
    steering = params["steering"]
    table = steering["pwm_to_angle_table"]
    if not table:
        raise ValueError(f"{path}: steering.pwm_to_angle_table is null; nothing to convert with")
    left_is_min = steering["pwm_left_bound"] == "pwm_min_us"
    lo, hi = float(steering["pwm_min_us"]), float(steering["pwm_max_us"])
    return Current(
        pwm_left_us=lo if left_is_min else hi,
        pwm_neutral_us=float(steering["pwm_neutral_us"]),
        pwm_right_us=hi if left_is_min else lo,
        table=tuple(sorted((float(p["pwm_us"]), float(p["angle_rad"])) for p in table)),
        planning_limit_rad=float(steering["max_angle_rad"]),
    )


def old_pulse_us(recorded: Recorded, current: Current, delta_rad: float) -> float:
    """The pulse the old linear map sent for a recorded steering angle (neutral for NaN)."""
    if not math.isfinite(delta_rad):
        return current.pwm_neutral_us
    end = current.pwm_left_us if delta_rad >= 0.0 else current.pwm_right_us
    share = min(abs(delta_rad) / recorded.steering_limit_rad, 1.0)
    return current.pwm_neutral_us + (end - current.pwm_neutral_us) * share


def table_angle_rad(current: Current, pulse_us: float) -> float:
    """Piecewise-linear angle of a pulse through the table, held at the first and last point."""
    points = current.table
    if pulse_us <= points[0][0]:
        return points[0][1]
    if pulse_us >= points[-1][0]:
        return points[-1][1]
    for (p0, a0), (p1, a1) in itertools.pairwise(points):
        if p0 <= pulse_us <= p1:
            return a0 + (a1 - a0) * (pulse_us - p0) / (p1 - p0)
    return points[-1][1]  # unreachable: the bounds above cover every finite pulse


def steering_rad(recorded: Recorded, current: Current, delta_rad: float) -> float:
    """The real road-wheel angle of a recorded steering command. NaN stays NaN."""
    if not math.isfinite(delta_rad):
        return delta_rad
    return table_angle_rad(current, old_pulse_us(recorded, current, delta_rad))


def speed_mps(recorded: Recorded, value_mps: float) -> float:
    return value_mps / recorded.speed_ratio


def _drive(recorded: Recorded, current: Current, d: DriveRecord, steering: bool) -> DriveRecord:
    return dataclasses.replace(
        d,
        steering_rad=steering_rad(recorded, current, d.steering_rad)
        if steering
        else d.steering_rad,
        speed_mps=speed_mps(recorded, d.speed_mps),
    )


def convert_bag(
    bag: BagData, recorded: Recorded, current: Current, steering: bool = True
) -> BagData:
    """A copy of the bag with /drive_raw, /drive and /odom/wheel in the committed units.
    `steering=False` converts the speeds only (a diagnostic: the old, wrong turning radius)."""
    return dataclasses.replace(
        bag,
        drive_raw=[_drive(recorded, current, d, steering) for d in bag.drive_raw],
        drive=[_drive(recorded, current, d, steering) for d in bag.drive],
        odom=[
            OdomRecord(o.stamp_s, speed_mps(recorded, o.speed_mps), o.log_time_s) for o in bag.odom
        ],
    )
