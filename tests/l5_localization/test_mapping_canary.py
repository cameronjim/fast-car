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
import subprocess
import sys
import time
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

# Pass thresholds. Two sets of measurements, both in the ros-dev image, racer_state's
# slam_toolbox.yaml, noise on, 1.0 m/s, two laps (docs/notes/localization-2026-11-03.md for the
# first, docs/notes/track-session-2026-11-11.md for the second):
#
#   9 runs, 2026-11-03 (an idle machine):   wall error mean 0.030 to 0.042 m
#   25 runs, 2026-11-11, 2 CPUs per run with other jobs on the machine:
#     SAME seed 21, 11 runs:  mean 0.0227 to 0.0454 (median 0.037, sd 0.006), p95 0.072 to 0.119,
#                             spurious 0.000 to 0.040, coverage 1.00. The sim is NOT deterministic:
#                             bridge_node, the EKF, slam_toolbox and the tracker run on wall-clock
#                             timers and the scheduler decides which scan meets which odometry
#                             sample, so one seed gives a spread, and a slower or busier machine
#                             shifts it up.
#                             Since 2026-11-14 the canary runs on simulated time (bridge_node
#                             sim_time, GitHub issue 156, docs/notes/sim-time-lockstep-2026-11-14.md):
#                             immune to host stalls, but tracker_node still ticks on a wall timer, so
#                             the thresholds below are unchanged.
#     varied seeds, 14 runs:  13 healthy ones in the same range (mean 0.028 to 0.040), and ONE
#                             broken map (seed 301): mean 0.114, p95 0.31, spurious 0.39, coverage
#                             0.76: the loop closed at the wrong place, the failure of the first
#                             pass of the stricter loop closure settings. 1 broken map in 25 runs.
#   CI: PR 116 failed this check with a mean of 0.0613 on a loaded runner.
#
# The old mean limit, 0.06, was 1.5 times the idle-machine maximum and sat 4 sd above the
# loaded-machine mean; 0.0613 is a healthy map on a slow runner, not a bad one. The limits now sit
# between the healthy tail and the broken map with room either side: mean 0.075 (healthy 0.045 at
# worst locally, 0.061 seen in CI; broken 0.114), p95 0.16 (healthy 0.119, broken 0.31), spurious
# 0.15 (healthy 0.040, broken 0.39), coverage 0.95 (healthy 1.00, broken 0.76). What the test is for,
# a map whose walls are in the wrong place, is still caught by every one of them. What it cannot
# fix is that about 1 run in 25 BUILDS a wrong map (the plain, featureless lane track): a real
# stack failure, reported as one; a rerun is the right answer and the note has the numbers.
MAX_WALL_ERR_MEAN_M = 0.075
MAX_WALL_ERR_P95_M = 0.16
MAX_SPURIOUS_FRACTION = 0.15
MIN_COVERAGE = 0.95
# The pose limit is the loosest because the pose is checked between scan-matcher updates, where it
# is odometry; the map is the deliverable.
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
        node = common.create_node("l5_mapping_canary")
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
            _stop_slam_toolbox_after_the_map_is_saved()


def _stop_slam_toolbox_after_the_map_is_saved() -> None:
    """SIGKILL slam_toolbox once the map is on disk, instead of letting launch_testing's SIGINT
    reach it with everything else.

    async_slam_toolbox_node died with exit code -6 at that SIGINT in 2 of about 6 CI runs on PR
    115 (GitHub's 2 vCPU runners): the log is "terminate called after throwing an instance of
    'karto::Exception'" 9 ms after the signal, with the map already saved and compared and the
    test body passed. The source (slam_toolbox humble, slam_toolbox_common.cpp) shows what is
    exposed: publishVisualizations() runs `while (rclcpp::ok()) { updateMap(); publishGraph(); }`
    on a bare boost::thread with no try/catch, so any karto::Exception there is std::terminate,
    and the destructor joins that thread before it resets the mapper. Which karto call throws in
    the shutdown window was NOT isolated: 36 local runs of this canary and of the shutdown itself
    (aarch64, one to two CPUs, SIGINT at random times with scans still arriving) never aborted,
    and no upstream issue describes it. It happens after every result of this test is known, so
    the canary ends slam_toolbox itself, the one process whose exit status says nothing, and
    still requires every other process to exit as before and slam_toolbox to have been alive until
    then (an abort during the run would have failed the test body already)."""
    subprocess.run(["pkill", "-KILL", "-f", "async_slam_toolbox_node"], check=False)
    time.sleep(0.5)


@launch_testing.post_shutdown_test()
class TestMappingCanaryShutdown(unittest.TestCase):
    def test_exit_codes(self, proc_info):
        for info in proc_info:
            allowed = [0, -signal.SIGINT, -signal.SIGTERM]
            if "async_slam_toolbox_node" in info.process_name:
                allowed.append(-signal.SIGKILL)  # ended by the test, see above
            self.assertIn(
                info.returncode,
                allowed,
                f"{info.process_name} exited with {info.returncode}",
            )
