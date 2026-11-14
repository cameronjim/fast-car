"""L5 localisation canary on the living-room lane track: tracker_node laps on the AMCL pose.

Scenario: racer_gym_bridge's lane_round track (5.5 x 4.0 m loop, 1.1 m lane, the owner's floor
track size) with 0.8 m centreline corners, three laps at 1.5 m/s. A narrow lane is the harder
case for the controller (0.4 m of room each side of the car), and a plain rectangle is a hard
case for AMCL (straight walls say little about where along them the car is; the corners and the
far walls have to). Otherwise the same as test_localization_canary_oval.py: ground-truth TF off,
tracker_node on /odom composed with AMCL's map -> odom, racer_bringup's localization.launch.py
on the bridge's own map, C1-like scan, sensor noise on, initial pose from ground truth.

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

LABEL = "localization_lane_round"
SEED = common.env_int("L5_SEED", 41)
SPEED_MPS = common.env_float("L5_SPEED_MPS", 1.5)
LAPS = common.env_int("L5_LAPS", 3)
CORNER_RADIUS_M = 0.8
TRACK = common.lane_track("lane_round", CORNER_RADIUS_M, SPEED_MPS, LABEL)
MAP_YAML, GT_FROM_MAP = common.localization_setup(TRACK, LABEL)
MAX_TEST_WALL_S = 180.0

# Pass thresholds. MEASURED with racer_state's configs, noise on, 1.5 m/s, seeds 41/42/43,
# locally in the ros-dev image (docs/notes/localization-2026-11-03.md):
#   cross-track |e| mean  0.023 / 0.023 / 0.024 m
#   position mean         0.060 / 0.054 / 0.059 m
#   position p95          0.109 / 0.098 / 0.107 m
#   position max          0.120 / 0.107 / 0.117 m
#   heading p95           0.084 / 0.075 / 0.079 rad
#   heading max           0.110 / 0.102 / 0.094 rad
# About 1.7 to 2 times the worst measured value, for the same reasons as the oval canary.
THRESHOLDS = common.Thresholds(
    cross_abs_mean_m=0.05,
    pos_mean_m=0.10,
    pos_p95_m=0.20,
    pos_max_m=0.25,
    hdg_p95_rad=0.15,
    hdg_max_rad=0.22,
)


def generate_test_description():
    return launch.LaunchDescription(
        [
            *common.launch_actions(TRACK, "localization", MAP_YAML, SEED),
            launch_testing.actions.ReadyToTest(),
        ]
    )


class TestLocalizationCanaryLane(unittest.TestCase):
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
            extra={"speed_mps": SPEED_MPS, "seed": SEED},
        )


@launch_testing.post_shutdown_test()
class TestLocalizationCanaryLaneShutdown(unittest.TestCase):
    def test_exit_codes(self, proc_info):
        common.assert_shutdown_exit_codes(self, proc_info)
