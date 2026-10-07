"""L5 sim-in-loop canary for the reverse escape (2026-10-06 night floor finding (b)).

On the floor (bags 2026-10-06T23-19-41 and 23-25-38) the car ended nose-in to a corner tighter
than its turning circle (full lock 0.4189 rad on the 0.3302 m wheelbase, about 0.8 m across) in
both lap directions and waited on safety_node's latch for ever. This canary drives a walled
loop with ONE such corner and requires the car to get round it by backing out:

  * TRACK: a counter-clockwise rounded rectangle (l5_reactive_common.rounded_rectangle), a
    _TRACK_HALF_WIDTH_M corridor (a 1.1 m lane, the floor lane's width) around it, three
    corners of _EASY_RADIUS_M centreline radius and, FIRST after the start, one of
    _TIGHT_RADIUS_M: a square corner. Its inside is a right-angled wall corner and its outside
    wall a _TIGHT_RADIUS_M + _TRACK_HALF_WIDTH_M (0.57 m) radius arc, while the car's outer
    front corner sweeps about 1.0 m from the turn centre at full lock, so the car cannot drive
    round it in one go. The walled corridor of racer_gym_bridge expresses this corner directly
    (cells within the half width of the centreline polyline); no track support was needed.
  * WHY A SQUARE CORNER and not a merely tight round one: with a round corner (centreline
    radius 0.25 m) the follower turns in early and the car's outer FRONT CORNER scrapes the
    outside wall before safety_node brakes. safety_node's arc corridor is a band of half the
    chassis width plus limits.obstacle_corridor_margin_m (0.205 m) about the REAR-AXLE path;
    at full lock (rear-axle radius 0.742 m) its outer edge is 0.947 m from the turn centre,
    while the body's outer front corner sweeps hypot(0.46, 0.897) = 1.008 m, about 6 cm
    outside it. In a square corner the outside wall is met head-on, inside the band, so the
    car stops nose-in exactly like on the floor (bags 23-19-41 / 23-25-38) and the escape has
    something to escape from. The 6 cm gap is a layer-3 finding of its own (see
    docs/notes/ttc-limit-cycle-2026-10-06.md, rear corridor section), not something this
    canary hides: the off-track check and the wall-contact check below would catch it.
  * STACK: bridge_node -> /scan -> gap_follow_node (the full floor-2026-10-06 profile, lane
    centring and reverse escape on) -> /drive_raw -> safety_node -> /drive -> bridge_node. The
    REAL command path with safety_node in the loop, no test-only remap: the escape only fires
    when safety_node refuses the forward request, which it sees on the gated /drive.
  * PASS: the escape fires at least once (a reverse request on /drive_raw AND a reverse command
    on the gated /drive), and the car still completes a lap inside _MAX_LAP_S without leaving
    the corridor and without touching a wall (l5_reactive_common.run_laps).

SIM LIMITS, stated: racer_gym_bridge's scan covers 4.7 rad from the car's pose, so the rear
corridor in safety_node only sees the two rear-quarter wedges behind the car, not straight
behind it; the sim places the scan at the gym pose while safety_node and gap_follow_node place
the head 0.285 m ahead of the rear axle (sensors.lidar.mount_x_m), as the other canaries note.
The lap time here is a regression band for a behaviour, not a performance number.
"""

from __future__ import annotations

import importlib.util
import math
import signal
import sys
import tempfile
import time
import unittest
from pathlib import Path

import launch
import launch_testing
import launch_testing.actions
import launch_testing.asserts
import rclpy
from ackermann_msgs.msg import AckermannDriveStamped
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

_TEST_SEED = 13
_TRACK_HALF_WIDTH_M = 0.55
_TRACK_WIDTH_M = 5.0
_TRACK_HEIGHT_M = 3.5
_EASY_RADIUS_M = 1.2
_TIGHT_RADIUS_M = 0.02
_CENTRELINE_DS_M = 0.05

_TARGET_LAPS = 1
# Generous: the floor profile tops out at 0.9 m/s (a lap of about 15 m is at least 17 s), and
# each escape costs escape_after_s (1.5 s) of waiting plus a couple of seconds of backing and
# re-trying. Measured locally: see docs/notes/reactive-control-port-2026-10-05.md.
_MAX_LAP_S = 150.0
_MAX_TEST_WALL_S = 200.0
# A corner that needs several back-and-forth legs makes no NEW forward progress for a while.
_STALL_TIMEOUT_S = 30.0


def _track_points():
    return common.rounded_rectangle(
        _TRACK_WIDTH_M,
        _TRACK_HEIGHT_M,
        # (bottom-right, top-right, top-left, bottom-left): the tight corner comes first.
        (_TIGHT_RADIUS_M, _EASY_RADIUS_M, _EASY_RADIUS_M, _EASY_RADIUS_M),
        _CENTRELINE_DS_M,
    )


def _raceline_path() -> Path:
    path = Path(tempfile.gettempdir()) / "l5_reactive_tight_corner_raceline.csv"
    xs, ys = _track_points()
    common.write_raceline_csv(path, xs, ys, speed_mps=1.0)
    return path


def generate_test_description():
    raceline = _raceline_path()
    bridge_node = LaunchNode(
        package="racer_gym_bridge",
        executable="bridge_node",
        name="bridge_node",
        parameters=[
            {
                "seed": _TEST_SEED,
                "raceline_path": str(raceline),
                "track_half_width_m": _TRACK_HALF_WIDTH_M,
            }
        ],
    )
    safety_node = LaunchNode(
        package="racer_safety",
        executable="safety_node",
        name="safety_node",
        parameters=[{"laser_yaw_from_vehicle_params": False}],
    )
    gap_params = common.floor_profile()
    # racer_gym_bridge's /scan is aligned to the vehicle (yaw 0), not the real car's mount.
    gap_params["laser_yaw_from_vehicle_params"] = False
    gap_follow_node = LaunchNode(
        package="racer_control",
        executable="gap_follow_node",
        name="gap_follow_node",
        parameters=[gap_params],
        output="screen",
    )
    return launch.LaunchDescription(
        [bridge_node, safety_node, gap_follow_node, launch_testing.actions.ReadyToTest()]
    )


class TestGapFollowEscapeCanary(unittest.TestCase):
    @classmethod
    def setUpClass(cls):
        rclpy.init()
        xs, ys = _track_points()
        cls.loop = common.Loop(xs, ys)
        cls.max_lateral_m = _TRACK_HALF_WIDTH_M - common.chassis_half_width_m()

    @classmethod
    def tearDownClass(cls):
        rclpy.shutdown()

    def test_backs_out_of_the_tight_corner_and_completes_the_lap(self):
        self.assertTrue(common.floor_profile()["reverse_escape"], "profile lost reverse_escape")
        node = rclpy.create_node("l5_reactive_escape_test")
        try:
            common.reset_sim(self, node)
            common.check_walls(self, node, _TRACK_HALF_WIDTH_M)
            requests: list[float] = []
            gated: list[float] = []
            node.create_subscription(
                AckermannDriveStamped,
                "/drive_raw",
                lambda m: requests.append(m.drive.speed),
                common.reliable_qos(),
            )
            node.create_subscription(
                AckermannDriveStamped,
                "/drive",
                lambda m: gated.append(m.drive.speed),
                common.reliable_qos(),
            )
            start = time.monotonic()
            result = common.run_laps(
                self,
                node,
                self.loop,
                target_laps=_TARGET_LAPS,
                max_wall_s=_MAX_TEST_WALL_S,
                stall_timeout_s=_STALL_TIMEOUT_S,
                max_lateral_m=self.max_lateral_m,
                lateral_windowed=False,
            )
            escapes = sum(
                1 for a, b in zip([0.0, *requests], requests, strict=False) if a >= 0.0 > b
            )
            print(
                f"[l5_reactive_escape] lap time {result.lap_time_s:.3f}s (limit {_MAX_LAP_S}s), "
                f"{escapes} reverse escape(s) requested, worst distance from the centreline "
                f"{result.worst_lateral_m:.3f} m (limit {self.max_lateral_m:.3f} m), wall "
                f"{time.monotonic() - start:.1f}s"
            )
            self.assertGreaterEqual(escapes, 1, "the reverse escape never fired")
            self.assertTrue(
                any(s < 0.0 for s in gated), "no reverse command ever reached the gated /drive"
            )
            self.assertLessEqual(result.lap_time_s, _MAX_LAP_S)
            self.assertTrue(math.isfinite(result.lap_time_s))
        finally:
            node.destroy_node()


@launch_testing.post_shutdown_test()
class TestGapFollowEscapeCanaryShutdown(unittest.TestCase):
    def test_exit_codes(self, proc_info):
        launch_testing.asserts.assertExitCodes(proc_info, allowable_exit_codes=[0, -signal.SIGINT])
