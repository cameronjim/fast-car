"""L5 sim-in-loop canary for the reverse escape (2026-10-06 night floor finding (b)), SQUARE corner.

On the floor (bags 2026-10-06T23-19-41 and 23-25-38) the car ended nose-in to a corner tighter
than its turning circle (full lock 0.4189 rad on the 0.3302 m wheelbase, about 0.8 m across) in
both lap directions and waited on safety_node's latch for ever. This canary drives a walled
loop with ONE such corner and requires the car to get round it by backing out:

  * TRACK: a counter-clockwise rounded rectangle (l5_reactive_common.rounded_rectangle), a
    1.1 m lane (ESCAPE_TRACK_HALF_WIDTH_M 0.55 m, the floor lane's width) around it, three
    corners of 1.2 m centreline radius and, FIRST after the start, one of _TIGHT_RADIUS_M: a
    square corner. Its inside is a right-angled wall corner and its outside wall a 0.57 m
    radius arc, while the car's outer front corner sweeps about 1.0 m from the turn centre at
    full lock, so the car cannot drive round it in one go. The walled corridor of
    racer_gym_bridge expresses this corner directly (cells within the half width of the
    centreline polyline); no track support was needed.
  * In a square corner the outside wall is met head-on, so the car stops nose-in exactly like
    on the floor (bags 23-19-41 / 23-25-38) and the escape has something to escape from.
    test_gap_follow_escape_canary_round.py is the same loop with a ROUND tight corner (0.25 m),
    where the outside wall is met by the car's outer front corner instead; that one needed
    safety_node's arc corridor to bound its outer edge by the outer front corner's sweep
    (forward_sector.hpp "OUTER BOUNDARY", schema 0.11.0).
  * STACK: bridge_node -> /scan -> gap_follow_node (the full floor-2026-10-06 profile, lane
    centring and reverse escape on) -> /drive_raw -> safety_node -> /drive -> bridge_node. The
    REAL command path with safety_node in the loop, no test-only remap: the escape only fires
    when safety_node refuses the forward request, which it sees on the gated /drive.
  * PASS: the escape fires at least once (a reverse request on /drive_raw AND a reverse command
    on the gated /drive), and the car still completes a lap inside ESCAPE_MAX_LAP_S without
    leaving the corridor and without touching a wall (l5_reactive_common.run_escape_canary).

SIM LIMITS, stated: racer_gym_bridge's scan covers 4.7 rad from the car's pose, so the rear
corridor in safety_node only sees the two rear-quarter wedges behind the car, not straight
behind it; the sim places the scan at the gym pose while safety_node and gap_follow_node place
the head 0.285 m ahead of the rear axle (sensors.lidar.mount_x_m), as the other canaries note.
The lap time here is a regression band for a behaviour, not a performance number.
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

# A square corner: the polyline's corner, rounded by 2 cm only so the centreline is sampled.
_TIGHT_RADIUS_M = 0.02


def generate_test_description():
    return common.escape_canary_launch(_TIGHT_RADIUS_M, launch, launch_testing, LaunchNode)


class TestGapFollowEscapeCanary(unittest.TestCase):
    @classmethod
    def setUpClass(cls):
        rclpy.init()

    @classmethod
    def tearDownClass(cls):
        rclpy.shutdown()

    def test_backs_out_of_the_tight_corner_and_completes_the_lap(self):
        common.run_escape_canary(self, _TIGHT_RADIUS_M, "square corner")


@launch_testing.post_shutdown_test()
class TestGapFollowEscapeCanaryShutdown(unittest.TestCase):
    def test_exit_codes(self, proc_info):
        launch_testing.asserts.assertExitCodes(proc_info, allowable_exit_codes=[0, -signal.SIGINT])
