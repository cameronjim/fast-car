"""L5 localisation canary: tracker_node drives gym_oval on the AMCL pose at the car's speed cap.

Scenario: config/tracks/gym_oval (34.9 m stadium, the committed raceline) in a walled corridor
1.6 m wide, raceline speeds capped at actuation.throttle_full_scale_mps (2.91 m/s, the car's
VESC-limited top speed), three laps. Ground-truth TF off: tracker_node follows the EKF's /odom
composed with AMCL's map -> odom (pose_frame map), so if localisation is bad the car leaves the
lane and the canary fails on that before it fails on the error numbers. racer_bringup's
localization.launch.py (EKF, map_server, AMCL, lifecycle manager) runs unmodified on the bridge's
own occupancy map written as a map_server yaml, with the C1-like scan and the sensor noise of
l5_localization_common. The initial pose comes from ground truth on /initialpose.

Above 0.5 m/s f1tenth_gym runs its dynamic single-track model (tyre slip, load transfer), so the
kinematic yaw rate wheel_twist_node feeds the EKF is wrong in transients, as it will be on the car.

L5_MAP_YAML (tuning only) localises on a saved SLAM map instead (l5_localization_common
.localization_setup).

THRESHOLDS are set from measured runs with margin; measured values next to each one and in
docs/notes/localization-2026-11-03.md.
"""

from __future__ import annotations

import importlib.util
import sys
import unittest
from pathlib import Path

import launch
import launch_testing
import launch_testing.actions
import launch_testing.asserts
import rclpy

_spec = importlib.util.spec_from_file_location(
    "l5_localization_common", Path(__file__).resolve().parent / "l5_localization_common.py"
)
common = importlib.util.module_from_spec(_spec)
sys.modules[_spec.name] = common  # dataclasses look their module up there
_spec.loader.exec_module(common)

LABEL = "localization_gym_oval"
SEED = common.env_int("L5_SEED", 31)
SPEED_MPS = common.env_float("L5_SPEED_MPS", common.SPEED_CAP_MPS)
LAPS = common.env_int("L5_LAPS", 3)
HALF_WIDTH_M = 0.8
TRACK = common.gym_oval_track(HALF_WIDTH_M, SPEED_MPS, LABEL)
MAP_YAML, GT_FROM_MAP = common.localization_setup(TRACK, LABEL)
MAX_TEST_WALL_S = 180.0

# Pass thresholds. MEASURED with racer_state's configs, noise on, 2.91 m/s cap, seeds 31/32/33
# plus a rerun of 31, locally in the ros-dev image (docs/notes/localization-2026-11-03.md):
#   cross-track |e| mean  0.040 / 0.040 / 0.041 / 0.041 m
#   position mean         0.151 / 0.158 / 0.167 / 0.182 m  (mostly ALONG track: the 2% wheel
#                          speed error, on walls that say little about along-track position)
#   position p95          0.313 / 0.268 / 0.300 / 0.338 m
#   position max          0.358 / 0.330 / 0.456 / 0.398 m
#   heading p95           0.061 / 0.058 / 0.067 / 0.061 rad
#   heading max           0.093 / 0.100 / 0.114 / 0.108 rad
# Each limit is about 1.5 to 2 times the worst measured value: CI runners are slower and
# noisier than the dev machine, and a real regression (the solid-map bias, nav2's default
# alphas) costs metres, not centimetres. The lane staying inside its walls is checked on every
# ground-truth sample by run_laps, whatever these numbers say.
THRESHOLDS = common.Thresholds(
    cross_abs_mean_m=0.08,
    pos_mean_m=0.30,
    pos_p95_m=0.55,
    pos_max_m=0.80,
    hdg_p95_rad=0.12,
    hdg_max_rad=0.25,
)


def generate_test_description():
    return launch.LaunchDescription(
        [
            *common.launch_actions(TRACK, "localization", MAP_YAML, SEED),
            launch_testing.actions.ReadyToTest(),
        ]
    )


class TestLocalizationCanaryOval(unittest.TestCase):
    @classmethod
    def setUpClass(cls):
        rclpy.init()

    @classmethod
    def tearDownClass(cls):
        rclpy.shutdown()

    def test_laps_on_the_localised_pose(self):
        common.run_localization_canary(
            self,
            LABEL,
            TRACK,
            MAP_YAML,
            GT_FROM_MAP,
            laps=LAPS,
            max_wall_s=MAX_TEST_WALL_S,
            thresholds=THRESHOLDS,
            extra={"speed_cap_mps": SPEED_MPS, "seed": SEED},
        )


@launch_testing.post_shutdown_test()
class TestLocalizationCanaryOvalShutdown(unittest.TestCase):
    def test_exit_codes(self, proc_info):
        common.assert_shutdown_exit_codes(self, proc_info)
