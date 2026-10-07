"""L5 sim-in-loop canary for the reverse escape, ROUND tight corner (2026-10-06 night).

The same loop, stack and pass criteria as test_gap_follow_escape_canary.py (read its
docstring), with the tight corner ROUND: a 0.25 m centreline radius, well inside the car's
0.742 m full-lock rear-axle radius. Its outside wall is a 0.80 m arc (0.25 m + the 0.55 m half
width).

WHY THIS SCENARIO. With a round corner the follower turns in early, and the wall the car meets
is not head-on but beside its nose: at full lock the car's OUTER FRONT CORNER sweeps the circle
of radius hypot(wheelbase + front overhang, R + half width) about the turn centre, 1.008 m,
while safety_node's arc corridor used to end at R + half width + margin, 0.947 m. In this
scenario the outer corner scraped the outside wall before the brake fired. Since schema 0.11.0
the corridor's outer edge is the outer front corner's sweep (with the margin, 1.053 m;
racer_safety forward_sector.hpp "OUTER BOUNDARY"), so the brake must fire before contact, then
the reverse escape backs the car out and the lap is finished. Since schema 0.12.0 that
outer-corner band is checked only within limits.outer_corner_horizon_m of arc (0.45 m,
forward_sector.hpp "OUTER-CORNER HORIZON"); this canary is the check that the horizon is still
long enough to brake before the corner touches the wall. The floor profile's
escape_probe_distance_m is 0.5 m, not the node default 0.3 m, so the follower backs out
instead of waiting on a latch that releases only at 0.6 m (gap_follow.launch.py).

PASS (l5_reactive_common.run_escape_canary): the escape fires at least once, a reverse command
reaches the gated /drive, and the car completes the lap inside ESCAPE_MAX_LAP_S without leaving
the corridor and WITHOUT TOUCHING A WALL. The wall-contact failure is the point of this canary;
do not loosen it to make the scenario pass.
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

_TIGHT_RADIUS_M = 0.25


def generate_test_description():
    return common.escape_canary_launch(_TIGHT_RADIUS_M, launch, launch_testing, LaunchNode)


class TestGapFollowEscapeCanaryRound(unittest.TestCase):
    @classmethod
    def setUpClass(cls):
        rclpy.init()

    @classmethod
    def tearDownClass(cls):
        rclpy.shutdown()

    def test_backs_out_of_the_round_tight_corner_and_completes_the_lap(self):
        common.run_escape_canary(self, _TIGHT_RADIUS_M, "round corner")


@launch_testing.post_shutdown_test()
class TestGapFollowEscapeCanaryRoundShutdown(unittest.TestCase):
    def test_exit_codes(self, proc_info):
        launch_testing.asserts.assertExitCodes(proc_info, allowable_exit_codes=[0, -signal.SIGINT])
