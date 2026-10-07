"""L5 sim-in-loop lap canary for gap_follow_node (GitHub issue 26, claude-docs/12-testing.md),
COUNTER-CLOCKWISE: config/tracks/gym_oval in its raceline's own direction.
test_gap_follow_lap_canary_cw.py is the same canary clockwise; the harness for both lives in
l5_reactive_common.py.

Runs racer_gym_bridge's bridge_node and racer_control's gap_follow_node together. The
controller sees ONLY the bridge's /scan (no odometry, no raceline); the test watches the
bridge's ground truth and asserts the car completes two laps within a wall-clock budget,
always moving forward around the loop, and never leaving the corridor.

LANE CENTRING (2026-10-06 night): gap_follow_node runs the node defaults plus the canary's
speed cap and the floor-2026-10-06 profile's lane centring (centering_gain and its sector and
range, read from the launch file), so the canary covers centring at speed in both directions.

WALLS. The tracker canary's track (Track.from_refline) has an occupancy map that is free
everywhere, so a LiDAR controller would see no walls at all and this test could not mean
anything. bridge_node therefore has a `track_half_width_m` parameter (default 0, which keeps
the old wall-free map for every existing caller): here it builds a walled corridor of that half
width around the gym_oval raceline (racer_gym_bridge.track_loader.build_corridor_occupancy).
gym_oval is a stadium with 8 m straights and 3 m turn radius, so a 0.8 m corridor leaves the
inner wall at 2.2 m from the turn centre.

"WITHOUT LEAVING THE TRACK" is checked directly from ground truth: the car's distance from the
raceline (nearest raceline point, which over-estimates the true distance by at most a few
millimetres at this raceline's 0.1 m spacing, so the check is conservative) must stay below the
corridor half width minus half the chassis width (vehicle_params chassis.width_m, read from
config/vehicle_params.yaml, never typed in here). Crossing that line means the chassis has
touched a wall. A progress stall (no new forward progress for 8 s) also fails, which catches a
car wedged against a wall by the gym's collision handling.

SIM-ONLY TOPIC REMAP, same shim and same warning as tests/l5_tracker_lap: gap_follow_node's
/drive_raw is remapped to /drive so the bridge (which subscribes /drive) is driven directly,
with no safety_node in the loop. This is TEST-ONLY and must never be reproduced in a launch
file that could run against hardware; on the car, /drive comes only from safety_node. Ground
truth is read from /sim/ground_truth_odom by the TEST only; the controller under test never
sees it.

Lap counting uses the same windowed nearest-point arc-length tracker as
tests/l5_tracker_lap/test_tracker_lap_canary.py (safe on this simple stadium, see that file;
the oschersleben canary explains why it is not safe on a twisty real track). Timing is
wall-clock, like the tracker canary (the bridge steps at a fixed wall-clock rate); the band is
deliberately wide, for the same CI-runner variance reason that file documents.
"""

from __future__ import annotations

import importlib.util
import signal
import sys
import unittest
from pathlib import Path

import launch
import launch_testing
import launch_testing.actions
import launch_testing.asserts
import rclpy
from launch_ros.actions import Node as LaunchNode


def _load_common():
    spec = importlib.util.spec_from_file_location(
        "l5_reactive_common", Path(__file__).with_name("l5_reactive_common.py")
    )
    module = importlib.util.module_from_spec(spec)
    # Registered before it runs: its dataclasses look their module up in sys.modules.
    sys.modules[spec.name] = module
    spec.loader.exec_module(module)
    return module


common = _load_common()
_REVERSE_DIRECTION = False


def generate_test_description():
    return common.lap_canary_launch(_REVERSE_DIRECTION, launch, launch_testing, LaunchNode)


class TestGapFollowLapCanary(unittest.TestCase):
    @classmethod
    def setUpClass(cls):
        rclpy.init()

    @classmethod
    def tearDownClass(cls):
        rclpy.shutdown()

    def test_completes_laps_inside_the_corridor(self):
        common.run_lap_canary(self, _REVERSE_DIRECTION, "counter-clockwise")


@launch_testing.post_shutdown_test()
class TestGapFollowLapCanaryShutdown(unittest.TestCase):
    def test_exit_codes(self, proc_info):
        launch_testing.asserts.assertExitCodes(proc_info, allowable_exit_codes=[0, -signal.SIGINT])
