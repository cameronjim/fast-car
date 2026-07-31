"""L5 sim-in-loop tracker lap canary (roadmap task S.2, claude-docs/12-testing.md).

THIS IS THE REGRESSION CANARY for the whole classical (non-learned) stack: it runs
`racer_gym_bridge`'s `bridge_node` and `racer_control`'s `tracker_node` together against
the committed `config/tracks/gym_oval/raceline.csv` and asserts the tracker completes
N (>= 2) laps with no wall contact and a lap time inside a committed band.

SIM-ONLY TOPIC REMAPS -- read before touching this file, and see this task's PR body
"Safety impact" section for the same statement:

  * `bridge_node`'s `/sim/ground_truth_odom` is remapped to `/odom`. There is no EKF
    (`racer_state`, roadmap phase 2) yet; in the real graph `/odom` comes ONLY from a fused
    estimator. Ground truth standing in for `/odom` is valid ONLY for this sim canary.
  * `tracker_node`'s `/drive_raw` is remapped to `/drive`. There is no `safety_node`
    (`racer_safety`, roadmap phase 1 / S-track) yet; in the real graph `/drive` comes ONLY
    from `safety_node`, which gates `/drive_raw` -> `/drive` with TTC braking, covariance
    gating, watchdogs, and command sanity checks (claude-docs/05-safety.md). This remap
    lets the bridge (which subscribes `/drive`) be driven directly by the tracker for this
    sim loop ONLY. It is a TEST-ONLY SHIM and must never be reproduced in a launch file
    that could run against real hardware.

Env choice: this canary runs the STOCK f1tenth_gym env (via `bridge_node`'s existing
`build_env`), not `racer_gym.build_env`'s upgraded dynamics (roadmap S.1). Wiring the
upgraded dynamics into `bridge_node` is a real (not "small") change -- it touches how the
node constructs its env, which vehicle_params get threaded through, and interacts with the
env's own default-vs-racer_gym timestep/model wiring -- so it is left as a TODO citing
roadmap S.6 (sim dynamics regression battery), rather than attempted here. See this task's
PR body for the same statement.

Wall-contact detection: `bridge_node`'s reference tracks (both the original synthetic
refline and this raceline-derived one, via `f1tenth_gym.envs.track.Track.from_refline`)
build an occupancy map that is free EVERYWHERE (see `Track.from_refline`'s own
implementation) -- there are no walls for the car to hit, so the gym's own collision flag
structurally cannot fire true here. Per this task's explicit instruction, wall contact is
instead approximated by a PROGRESS-STALL guard: if the car's progress along the raceline
stops advancing for longer than `_STALL_TIMEOUT_S`, this test fails (a real wall contact
that spins the car out or wedges it would show up as exactly this symptom). A real
occupancy map (a later task) would let this test check the gym's own collision flag
directly instead.
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
import rclpy
from launch_ros.actions import Node as LaunchNode
from nav_msgs.msg import Odometry
from rclpy.qos import HistoryPolicy, QoSProfile, ReliabilityPolicy
from std_srvs.srv import Trigger

_REPO_ROOT = Path(__file__).resolve().parents[2]
_RACELINE_PATH = _REPO_ROOT / "config" / "tracks" / "gym_oval" / "raceline.csv"

_TEST_SEED = 11
_TARGET_LAPS = 2

# Timing band for _TARGET_LAPS laps' wall-clock time. This is the FIRST reference for this
# stack (claude-docs/12-testing.md L5: "the regression canary for the whole classical
# stack"): the real racer_control C++ tracker + racer_gym_bridge, run together in this
# exact CI job, measured 18.79s wall-clock for 2 laps (see this test's own printed
# diagnostic in CI logs). Band is +-25% around that measured value, per this task's
# instructions ("commit the band with slack... since this is the FIRST reference"; it
# tightens deliberately later, claude-docs/12-testing.md). A prior one-off Python-prototype
# pure-pursuit run against this same raceline (no ROS/launch overhead, pure gym.step)
# measured 11.66s of SIM time for 2 laps -- the real stack's wall-clock number being higher
# than that sim-time-only figure is expected (rclpy spin loop + real bridge stepping
# overhead) and is exactly why the band is centered on THIS test's own measurement, not the
# prototype's.
_LAP_TIME_LOW_S = 14.0
_LAP_TIME_HIGH_S = 24.0

# Overall wall-clock budget for the whole test before giving up.
_MAX_TEST_WALL_S = 90.0
# If progress along the raceline does not advance for this long, treat it as a stall
# (wall-contact proxy, see module docstring).
_STALL_TIMEOUT_S = 8.0
_STALL_PROGRESS_EPSILON_M = 0.05


def _reliable_qos() -> QoSProfile:
    return QoSProfile(
        reliability=ReliabilityPolicy.RELIABLE, history=HistoryPolicy.KEEP_LAST, depth=10
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


def generate_test_description():
    bridge_node = LaunchNode(
        package="racer_gym_bridge",
        executable="bridge_node",
        name="bridge_node",
        parameters=[{"seed": _TEST_SEED, "raceline_path": str(_RACELINE_PATH)}],
        remappings=[("/sim/ground_truth_odom", "/odom")],
    )
    tracker_node = LaunchNode(
        package="racer_control",
        executable="tracker_node",
        name="tracker_node",
        parameters=[{"raceline_path": str(_RACELINE_PATH)}],
        remappings=[("/drive_raw", "/drive")],
    )
    return launch.LaunchDescription(
        [
            bridge_node,
            tracker_node,
            launch_testing.actions.ReadyToTest(),
        ]
    )


class TestTrackerLapCanary(unittest.TestCase):
    @classmethod
    def setUpClass(cls):
        rclpy.init()
        cls.xs, cls.ys, cls.ss = _load_raceline_xy_s()
        # Approximate closed-loop total length: last sample's s plus its distance back to
        # the first sample (the raceline's own closing segment, per tools/raceline/io.py's
        # index-aligned closed-loop convention).
        closing_seg = math.hypot(cls.xs[0] - cls.xs[-1], cls.ys[0] - cls.ys[-1])
        cls.track_length_m = cls.ss[-1] + closing_seg

    @classmethod
    def tearDownClass(cls):
        rclpy.shutdown()

    def _nearest_s(
        self, x: float, y: float, prev_index: int, window: int = 50
    ) -> tuple[float, int]:
        """Nearest-point search restricted to a window around the previous index.

        A full-array nearest search is WRONG on a track shaped like this one: a stadium's
        two straights run close and parallel to each other (only ``2 * turn_radius_m``
        apart), so a naive global nearest-point search can jump to the physically-nearby
        far side of the loop under ordinary tracking error, corrupting the cumulative
        arc-length progress this test's lap detection depends on. A window of +/-50 points
        (roughly +/-5m at this raceline's ~0.1m point spacing) is far larger than one
        control cycle's actual travel distance at any speed this raceline commands, so it
        tracks true forward progress robustly while still ruling out the cross-track jump.
        """
        n = len(self.xs)
        best_i, best_d2 = prev_index, float("inf")
        for offset in range(-window, window + 1):
            i = (prev_index + offset) % n
            d2 = (self.xs[i] - x) ** 2 + (self.ys[i] - y) ** 2
            if d2 < best_d2:
                best_d2, best_i = d2, i
        return self.ss[best_i], best_i

    def test_completes_two_laps_within_committed_band(self):
        node = rclpy.create_node("l5_tracker_lap_test")
        try:
            reset_client = node.create_client(Trigger, "/sim/reset")
            self.assertTrue(
                reset_client.wait_for_service(timeout_sec=30.0), "/sim/reset service not available"
            )
            future = reset_client.call_async(Trigger.Request())
            rclpy.spin_until_future_complete(node, future, timeout_sec=30.0)
            self.assertIsNotNone(future.result(), "/sim/reset call did not complete")

            odom_positions = []
            node.create_subscription(
                Odometry,
                "/odom",
                lambda msg: odom_positions.append(
                    (msg.pose.pose.position.x, msg.pose.pose.position.y, time.monotonic())
                ),
                _reliable_qos(),
            )

            test_start = time.monotonic()
            deadline = test_start + _MAX_TEST_WALL_S
            last_progress_wall = test_start
            unwrapped_s = None
            prev_index = 0
            lap_crossing_walltimes: list[float] = []
            first_odom_walltime = None

            while time.monotonic() < deadline:
                rclpy.spin_once(node, timeout_sec=0.1)
                if not odom_positions:
                    continue
                x, y, wall_t = odom_positions[-1]
                if first_odom_walltime is None:
                    first_odom_walltime = wall_t
                nearest_s, prev_index = self._nearest_s(x, y, prev_index)

                if unwrapped_s is None:
                    unwrapped_s = nearest_s
                else:
                    prev_wrapped = unwrapped_s % self.track_length_m
                    delta = nearest_s - prev_wrapped
                    # Handle the wraparound at the seam (s resets from track_length back to
                    # 0): a large negative jump means a lap boundary was crossed forward.
                    if delta < -self.track_length_m / 2.0:
                        delta += self.track_length_m
                    if delta > _STALL_PROGRESS_EPSILON_M:
                        last_progress_wall = wall_t
                    unwrapped_s += delta

                laps_completed = int(unwrapped_s // self.track_length_m)
                while len(lap_crossing_walltimes) < laps_completed:
                    lap_crossing_walltimes.append(wall_t)

                if laps_completed >= _TARGET_LAPS:
                    break

                if wall_t - last_progress_wall > _STALL_TIMEOUT_S:
                    self.fail(
                        f"progress along the raceline stalled for > {_STALL_TIMEOUT_S}s "
                        f"(last nearest_s={nearest_s:.2f}m) -- treated as a wall-contact "
                        "proxy per this file's module docstring (the synthetic track's "
                        "occupancy map has no walls to collide with directly)"
                    )

            laps_completed = int((unwrapped_s or 0.0) // self.track_length_m)
            self.assertGreaterEqual(
                laps_completed,
                _TARGET_LAPS,
                f"only completed {laps_completed}/{_TARGET_LAPS} lap(s) within "
                f"{_MAX_TEST_WALL_S}s wall-clock budget",
            )

            lap_time_s = lap_crossing_walltimes[_TARGET_LAPS - 1] - first_odom_walltime
            print(
                f"[l5_tracker_lap] measured {_TARGET_LAPS}-lap time: {lap_time_s:.3f}s "
                f"(committed band: [{_LAP_TIME_LOW_S}, {_LAP_TIME_HIGH_S}]s)"
            )
            self.assertGreaterEqual(
                lap_time_s,
                _LAP_TIME_LOW_S,
                f"{_TARGET_LAPS}-lap time {lap_time_s:.3f}s below committed band "
                f"[{_LAP_TIME_LOW_S}, {_LAP_TIME_HIGH_S}]s -- suspiciously fast, check for a "
                "shortcut/skip in the tracking loop",
            )
            self.assertLessEqual(
                lap_time_s,
                _LAP_TIME_HIGH_S,
                f"{_TARGET_LAPS}-lap time {lap_time_s:.3f}s above committed band "
                f"[{_LAP_TIME_LOW_S}, {_LAP_TIME_HIGH_S}]s -- regression in tracking "
                "performance",
            )
        finally:
            node.destroy_node()


@launch_testing.post_shutdown_test()
class TestTrackerLapCanaryShutdown(unittest.TestCase):
    def test_exit_codes(self, proc_info):
        # Both nodes are stopped with SIGINT at the end of the test session; either a clean
        # exit(0) or "terminated by SIGINT" is an allowable outcome (same reasoning as
        # sim/bridge/racer_gym_bridge/test/test_bridge_node.py's shutdown test).
        launch_testing.asserts.assertExitCodes(proc_info, allowable_exit_codes=[0, -signal.SIGINT])
