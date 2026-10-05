"""L5 sim-in-loop lap canary for gap_follow_node (GitHub issue 26, claude-docs/12-testing.md).

Runs racer_gym_bridge's bridge_node and racer_control's gap_follow_node together. The
controller sees ONLY the bridge's /scan (no odometry, no raceline); the test watches the
bridge's ground truth and asserts the car completes _TARGET_LAPS laps of
config/tracks/gym_oval within a wall-clock budget, always moving forward around the loop,
and never leaving the corridor.

WALLS. The tracker canary's track (Track.from_refline) has an occupancy map that is free
everywhere, so a LiDAR controller would see no walls at all and this test could not mean
anything. bridge_node therefore gained a `track_half_width_m` parameter (default 0, which
keeps the old wall-free map for every existing caller): here it builds a walled corridor of
that half width around the gym_oval raceline (racer_gym_bridge.track_loader.
build_corridor_occupancy). gym_oval is a stadium with 8 m straights and 3 m turn radius, so a
_TRACK_HALF_WIDTH_M corridor leaves the inner wall at 3 - half width from the turn centre.

"WITHOUT LEAVING THE TRACK" is checked directly from ground truth: the car's distance from
the raceline (nearest raceline point, which over-estimates the true distance by at most a
few millimetres at this raceline's 0.1 m spacing, so the check is conservative) must stay
below the corridor half width minus half the chassis width (vehicle_params chassis.width_m,
read from config/vehicle_params.yaml, never typed in here). Crossing that line means the
chassis has touched a wall. A progress stall (_STALL_TIMEOUT_S with no forward progress)
also fails, which catches a car wedged against a wall by the gym's collision handling.

SIM-ONLY TOPIC REMAP, same shim and same warning as tests/l5_tracker_lap: gap_follow_node's
/drive_raw is remapped to /drive so the bridge (which subscribes /drive) is driven directly,
with no safety_node in the loop. This is TEST-ONLY and must never be reproduced in a launch
file that could run against hardware; on the car, /drive comes only from safety_node.
Ground truth is read from /sim/ground_truth_odom by the TEST only; the controller under test
never sees it.

Lap counting uses the same windowed nearest-point arc-length tracker as
tests/l5_tracker_lap/test_tracker_lap_canary.py (safe on this simple stadium, see that file;
the oschersleben canary explains why it is not safe on a twisty real track).

Timing: wall-clock, like the tracker canary (the bridge steps at a fixed wall-clock rate).
The band is deliberately wide, for the same CI-runner variance reason that file documents.
"""

from __future__ import annotations

import csv
import math
import signal
import time
import unittest
from pathlib import Path

import launch
import launch_testing
import launch_testing.actions
import launch_testing.asserts
import rclpy
import yaml
from launch_ros.actions import Node as LaunchNode
from nav_msgs.msg import Odometry
from rclpy.qos import HistoryPolicy, QoSProfile, ReliabilityPolicy
from sensor_msgs.msg import LaserScan
from std_srvs.srv import Trigger

_REPO_ROOT = Path(__file__).resolve().parents[2]
_RACELINE_PATH = _REPO_ROOT / "config" / "tracks" / "gym_oval" / "raceline.csv"
_VEHICLE_PARAMS_PATH = _REPO_ROOT / "config" / "vehicle_params.yaml"

_TEST_SEED = 11
_TARGET_LAPS = 2

# Corridor half width (m): a 1.6 m wide track, in the range of F1TENTH-style venue tracks
# for a 1/10 car. A property of the simulated track, not of the vehicle.
_TRACK_HALF_WIDTH_M = 0.8

# gap_follow_node tuning for this canary. Everything else is the node's default.
_GAP_FOLLOW_PARAMS = {
    "max_speed_mps": 3.0,
}

# Wall-clock band for _TARGET_LAPS laps.
#   * LOW is physical, not statistical: two laps of gym_oval (~34.85 m each) at the 3.0 m/s
#     cap take at least 23.2 s of sim time, and the bridge never steps faster than wall-clock,
#     so anything under 20 s means the lap counting is broken.
#   * HIGH: measured locally in the ros-dev image (Apple silicon, Docker) at about 26 s for
#     two laps on three consecutive runs. The tracker canary saw a ~40% swing between CI
#     runs of the same code, so this allows a bit over twice the local measurement before
#     calling it a regression.
_LAP_TIME_LOW_S = 20.0
_LAP_TIME_HIGH_S = 60.0
_MAX_TEST_WALL_S = 120.0
_STALL_TIMEOUT_S = 8.0
_STALL_PROGRESS_EPSILON_M = 0.05


def _reliable_qos() -> QoSProfile:
    return QoSProfile(
        reliability=ReliabilityPolicy.RELIABLE, history=HistoryPolicy.KEEP_LAST, depth=10
    )


def _best_effort_qos() -> QoSProfile:
    return QoSProfile(
        reliability=ReliabilityPolicy.BEST_EFFORT, history=HistoryPolicy.KEEP_LAST, depth=10
    )


def _load_raceline_xy_s():
    xs, ys, ss = [], [], []
    with _RACELINE_PATH.open() as f:
        header_seen = False
        for line in f:
            if line.startswith("#") or not line.strip():
                continue
            fields = next(csv.reader([line]))
            if not header_seen:
                header_seen = True
                continue
            ss.append(float(fields[0]))
            xs.append(float(fields[1]))
            ys.append(float(fields[2]))
    return xs, ys, ss


def _chassis_half_width_m() -> float:
    with _VEHICLE_PARAMS_PATH.open() as f:
        params = yaml.safe_load(f)
    return float(params["chassis"]["width_m"]) / 2.0


def generate_test_description():
    bridge_node = LaunchNode(
        package="racer_gym_bridge",
        executable="bridge_node",
        name="bridge_node",
        parameters=[
            {
                "seed": _TEST_SEED,
                "raceline_path": str(_RACELINE_PATH),
                "track_half_width_m": _TRACK_HALF_WIDTH_M,
            }
        ],
    )
    gap_follow_node = LaunchNode(
        package="racer_control",
        executable="gap_follow_node",
        name="gap_follow_node",
        parameters=[_GAP_FOLLOW_PARAMS],
        remappings=[("/drive_raw", "/drive")],
    )
    return launch.LaunchDescription(
        [bridge_node, gap_follow_node, launch_testing.actions.ReadyToTest()]
    )


class TestGapFollowLapCanary(unittest.TestCase):
    @classmethod
    def setUpClass(cls):
        rclpy.init()
        cls.xs, cls.ys, cls.ss = _load_raceline_xy_s()
        closing_seg = math.hypot(cls.xs[0] - cls.xs[-1], cls.ys[0] - cls.ys[-1])
        cls.track_length_m = cls.ss[-1] + closing_seg
        cls.max_lateral_m = _TRACK_HALF_WIDTH_M - _chassis_half_width_m()

    @classmethod
    def tearDownClass(cls):
        rclpy.shutdown()

    def _nearest(self, x: float, y: float, prev_index: int, window: int = 50):
        """Windowed nearest raceline point: (s, index, distance). See the tracker canary's
        _nearest_s for why the search is windowed."""
        n = len(self.xs)
        best_i, best_d2 = prev_index, float("inf")
        for offset in range(-window, window + 1):
            i = (prev_index + offset) % n
            d2 = (self.xs[i] - x) ** 2 + (self.ys[i] - y) ** 2
            if d2 < best_d2:
                best_d2, best_i = d2, i
        return self.ss[best_i], best_i, math.sqrt(best_d2)

    def test_completes_laps_inside_the_corridor(self):
        node = rclpy.create_node("l5_reactive_lap_test")
        try:
            reset_client = node.create_client(Trigger, "/sim/reset")
            self.assertTrue(
                reset_client.wait_for_service(timeout_sec=30.0), "/sim/reset service not available"
            )
            future = reset_client.call_async(Trigger.Request())
            rclpy.spin_until_future_complete(node, future, timeout_sec=30.0)
            self.assertIsNotNone(future.result(), "/sim/reset call did not complete")

            # Walls sanity check: the car starts on the raceline, i.e. mid-corridor, so the rays
            # straight out to each side must read about the corridor half width. This proves
            # the walls exist and that the occupancy map is not transposed or offset (with the
            # old wall-free map these rays read the far map edge).
            scans: list[LaserScan] = []
            node.create_subscription(LaserScan, "/scan", scans.append, _best_effort_qos())
            scan_deadline = time.monotonic() + 10.0
            while not scans and time.monotonic() < scan_deadline:
                rclpy.spin_once(node, timeout_sec=0.05)
            self.assertTrue(scans, "no /scan from bridge_node")
            scan = scans[0]
            for side in (-math.pi / 2.0, math.pi / 2.0):
                index = round((side - scan.angle_min) / scan.angle_increment)
                self.assertAlmostEqual(
                    scan.ranges[index],
                    _TRACK_HALF_WIDTH_M,
                    delta=0.2,
                    msg=f"ray at {side:+.2f} rad should hit a corridor wall ~{_TRACK_HALF_WIDTH_M} m away",
                )

            samples: list[tuple[float, float, float]] = []
            node.create_subscription(
                Odometry,
                "/sim/ground_truth_odom",
                lambda msg: samples.append(
                    (msg.pose.pose.position.x, msg.pose.pose.position.y, time.monotonic())
                ),
                _reliable_qos(),
            )

            test_start = time.monotonic()
            deadline = test_start + _MAX_TEST_WALL_S
            last_progress_wall = test_start
            unwrapped_s = None
            start_s = 0.0
            prev_index = 0
            lap_walltimes: list[float] = []
            first_walltime = None
            worst_lateral_m = 0.0
            consumed = 0

            while time.monotonic() < deadline:
                rclpy.spin_once(node, timeout_sec=0.05)
                # Check EVERY ground-truth sample, not just the latest, so a brief excursion
                # between spins cannot go unseen.
                while consumed < len(samples):
                    x, y, wall_t = samples[consumed]
                    consumed += 1
                    if first_walltime is None:
                        first_walltime = wall_t
                    nearest_s, prev_index, lateral_m = self._nearest(x, y, prev_index)
                    worst_lateral_m = max(worst_lateral_m, lateral_m)
                    if lateral_m > self.max_lateral_m:
                        self.fail(
                            f"left the track: {lateral_m:.3f} m from the raceline at "
                            f"({x:.2f}, {y:.2f}), limit {self.max_lateral_m:.3f} m "
                            f"(corridor half width {_TRACK_HALF_WIDTH_M} m minus half the "
                            "chassis width)"
                        )
                    if unwrapped_s is None:
                        unwrapped_s = nearest_s
                        start_s = nearest_s
                    else:
                        delta = nearest_s - (unwrapped_s % self.track_length_m)
                        if delta < -self.track_length_m / 2.0:
                            delta += self.track_length_m
                        elif delta > self.track_length_m / 2.0:
                            delta -= self.track_length_m
                        if delta > _STALL_PROGRESS_EPSILON_M:
                            last_progress_wall = wall_t
                        unwrapped_s += delta
                    laps = int((unwrapped_s - start_s) // self.track_length_m)
                    while len(lap_walltimes) < laps:
                        lap_walltimes.append(wall_t)
                    if wall_t - last_progress_wall > _STALL_TIMEOUT_S:
                        self.fail(
                            f"no forward progress for > {_STALL_TIMEOUT_S}s at "
                            f"s={nearest_s:.2f} m (wedged against a wall or turned around)"
                        )
                if len(lap_walltimes) >= _TARGET_LAPS:
                    break

            self.assertGreaterEqual(
                len(lap_walltimes),
                _TARGET_LAPS,
                f"only completed {len(lap_walltimes)}/{_TARGET_LAPS} lap(s) within "
                f"{_MAX_TEST_WALL_S}s wall-clock",
            )
            lap_time_s = lap_walltimes[_TARGET_LAPS - 1] - first_walltime
            print(
                f"[l5_reactive_lap] {_TARGET_LAPS}-lap time {lap_time_s:.3f}s "
                f"(band [{_LAP_TIME_LOW_S}, {_LAP_TIME_HIGH_S}]s), worst distance from the "
                f"raceline {worst_lateral_m:.3f} m (limit {self.max_lateral_m:.3f} m)"
            )
            self.assertGreaterEqual(
                lap_time_s,
                _LAP_TIME_LOW_S,
                "suspiciously fast for a reactive controller capped at "
                f"{_GAP_FOLLOW_PARAMS['max_speed_mps']} m/s: check the lap counting",
            )
            self.assertLessEqual(
                lap_time_s, _LAP_TIME_HIGH_S, "regression in gap-follow lap performance"
            )
        finally:
            node.destroy_node()


@launch_testing.post_shutdown_test()
class TestGapFollowLapCanaryShutdown(unittest.TestCase):
    def test_exit_codes(self, proc_info):
        launch_testing.asserts.assertExitCodes(proc_info, allowable_exit_codes=[0, -signal.SIGINT])
