"""L5 mapping canary: slam_toolbox maps a living-room lane track; the saved map must match.

Scenario: racer_gym_bridge's lane_round track (the reactive bench's 5.5 x 4.0 m loop, 1.1 m
lane, the size of the owner's floor track) with 0.8 m centreline corners, two laps at 1.0 m/s
with tracker_node on ground truth, racer_bringup's mapping.launch.py (wheel_twist_node, the EKF,
slam_toolbox) unmodified, the C1-like scan and the sensor noise of l5_localization_common. Then
/slam_toolbox/save_map, the service the runbook uses on the car, and the saved pgm + yaml is
compared with the bridge's /sim/map (l5_localization_common.compare_maps).

THRESHOLDS are set from measured runs with margin, never from hope; the measured values are next
to each one and in docs/notes/localization-2026-11-03.md.
"""

from __future__ import annotations

import importlib.util
import json
import os
import signal
import sys
import unittest
from pathlib import Path

import launch
import launch_testing
import launch_testing.actions
import launch_testing.asserts
import rclpy
from nav_msgs.msg import OccupancyGrid

_spec = importlib.util.spec_from_file_location(
    "l5_localization_common", Path(__file__).resolve().parent / "l5_localization_common.py"
)
common = importlib.util.module_from_spec(_spec)
sys.modules[_spec.name] = common  # dataclasses look their module up there
_spec.loader.exec_module(common)

LABEL = "mapping_lane_round"
SEED = common.env_int("L5_SEED", 21)
SPEED_MPS = common.env_float("L5_SPEED_MPS", 1.0)
LAPS = common.env_int("L5_LAPS", 2)
CORNER_RADIUS_M = 0.8
TRACK = common.lane_track("lane_round", CORNER_RADIUS_M, SPEED_MPS, LABEL)
if os.environ.get("L5_MAP_TRACK") == "gym_oval":  # tuning only: map the oval instead
    TRACK = common.gym_oval_track(0.8, SPEED_MPS, LABEL)
MAX_TEST_WALL_S = 150.0
# A SLAM wall cell further than this from a true wall surface counts as spurious, and a true wall
# cell with no SLAM cell this close as not covered: two map cells plus the 2 cm scan noise.
TOLERANCE_M = 0.12

# Pass thresholds. MEASURED with racer_state's slam_toolbox.yaml, noise on, 1.0 m/s, two laps,
# nine runs (seeds 21 to 26 and three more of 21), locally in the ros-dev image
# (docs/notes/localization-2026-11-03.md):
#   wall error mean      0.030 to 0.042 m
#   wall error p95       0.058 to 0.119 m
#   spurious fraction    0.000 to 0.006
#   coverage             1.000 in all nine
#   slam_toolbox pose p95 against ground truth   0.167 to 0.253 m
# Limits about 1.5 times the worst measured value. A broken map is not a few centimetres worse:
# with slam_toolbox's default loop closure settings 3 of 17 runs closed the loop at the wrong
# place (wall error mean 0.14 to 0.23 m, spurious 0.36 to 0.52, coverage 0.77 to 0.83), which
# these limits catch. The pose limit is the loosest because the pose is checked between
# scan-matcher updates, where it is odometry; the map is the deliverable.
MAX_WALL_ERR_MEAN_M = 0.06
MAX_WALL_ERR_P95_M = 0.16
MAX_SPURIOUS_FRACTION = 0.06
MIN_COVERAGE = 0.95
MAX_SLAM_POS_P95_M = 0.40


def generate_test_description():
    return launch.LaunchDescription(
        [
            *common.launch_actions(TRACK, "mapping", None, SEED),
            launch_testing.actions.ReadyToTest(),
        ]
    )


class TestMappingCanary(unittest.TestCase):
    @classmethod
    def setUpClass(cls):
        rclpy.init()

    @classmethod
    def tearDownClass(cls):
        rclpy.shutdown()

    def test_saved_map_matches_the_track(self):
        node = rclpy.create_node("l5_mapping_canary")
        try:
            truth = common.receive_latched(self, node, OccupancyGrid, "/sim/map")
            # tracker_node starts TRACKER_START_DELAY_S after the stack, so the car is still on
            # its start pose: slam_toolbox's map frame is this pose.
            start = common.first_ground_truth_pose(self, node)
            result = common.run_laps(
                self, node, TRACK, target_laps=LAPS, max_wall_s=MAX_TEST_WALL_S
            )
            common.to_ground_truth_frame(result, start)
            stem = Path(os.environ.get("L5_SAVE_MAP", str(common.scratch_dir(LABEL) / "slam_map")))
            slam_yaml = common.save_slam_map(self, node, stem)
            common.start_pose_sidecar(slam_yaml).write_text(
                json.dumps({"gt_from_map": list(start)})
            )
            comparison = common.compare_maps(slam_yaml, truth, start, TOLERANCE_M)
            summary = result.summary()
            common.record_results(
                LABEL,
                {
                    **comparison,
                    "slam_pose": summary,
                    "speed_mps": SPEED_MPS,
                    "noise": common.noise_on(),
                    "seed": SEED,
                    "map": str(slam_yaml),
                },
            )
            self.assertGreaterEqual(result.laps, LAPS)
            self.assertLessEqual(comparison["wall_err_mean_m"], MAX_WALL_ERR_MEAN_M)
            self.assertLessEqual(comparison["wall_err_p95_m"], MAX_WALL_ERR_P95_M)
            self.assertLessEqual(comparison["spurious_fraction"], MAX_SPURIOUS_FRACTION)
            self.assertGreaterEqual(comparison["coverage"], MIN_COVERAGE)
            self.assertLessEqual(summary["pos_p95_m"], MAX_SLAM_POS_P95_M)
        finally:
            node.destroy_node()


@launch_testing.post_shutdown_test()
class TestMappingCanaryShutdown(unittest.TestCase):
    def test_exit_codes(self, proc_info):
        launch_testing.asserts.assertExitCodes(
            proc_info, allowable_exit_codes=[0, -signal.SIGINT, -signal.SIGTERM]
        )
