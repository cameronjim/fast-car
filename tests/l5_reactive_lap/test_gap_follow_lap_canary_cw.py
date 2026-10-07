"""L5 sim-in-loop lap canary for gap_follow_node, CLOCKWISE (2026-10-06 night).

The same canary as test_gap_follow_lap_canary.py (read its docstring: walls, the sim-only
/drive_raw -> /drive remap, the off-track and stall checks, lane centring on), driven the other
way round config/tracks/gym_oval. The floor test showed the follower misbehaving in BOTH lap
directions, so the canary covers both.

How the direction is set: f1tenth_gym's reset puts the car on the raceline's first waypoint
facing along the raceline, so bridge_node's `reverse_direction:=true` reverses the waypoint
order before the track is built (racer_gym_bridge.bridge_node.build_track_from_raceline). The
corridor walls are the same. The test reverses its own copy of the raceline the same way so
lap progress counts up in the driving direction, and checks the loop really is clockwise.
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
_REVERSE_DIRECTION = True


def generate_test_description():
    return common.lap_canary_launch(_REVERSE_DIRECTION, launch, launch_testing, LaunchNode)


class TestGapFollowLapCanaryClockwise(unittest.TestCase):
    @classmethod
    def setUpClass(cls):
        rclpy.init()

    @classmethod
    def tearDownClass(cls):
        rclpy.shutdown()

    def test_completes_laps_inside_the_corridor_clockwise(self):
        common.run_lap_canary(self, _REVERSE_DIRECTION, "clockwise")


@launch_testing.post_shutdown_test()
class TestGapFollowLapCanaryClockwiseShutdown(unittest.TestCase):
    def test_exit_codes(self, proc_info):
        launch_testing.asserts.assertExitCodes(proc_info, allowable_exit_codes=[0, -signal.SIGINT])
