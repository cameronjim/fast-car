"""L3 test of bridge_node's sim_time mode (GitHub issue 156): the bridge owns the clock of the sim.

  * /clock is published, strictly increasing, in whole env timesteps;
  * every stamp the bridge puts on /scan, /sim/ground_truth_odom and /odom/wheel is a value it
    published on /clock, not a wall time;
  * sim time never runs faster than real_time_factor_max times wall time;
  * the lockstep gate holds the clock while an acknowledging output trails sim time by more than
    lockstep_max_lag_s, and releases it when the output catches up;

(The wall timeout that opens a shut gate is covered in test_lockstep.py.) The defaults (sim_time off, nothing published on /clock) stay covered by test_bridge_node.py.
Same importorskip guard as that file: this only runs where rclpy, launch_testing and f1tenth_gym
exist.
"""

from __future__ import annotations

import itertools
import os
import signal
import time
import unittest

import pytest

pytest.importorskip("rclpy")
pytest.importorskip("launch_testing")
pytest.importorskip("f1tenth_gym")
pytest.importorskip("rosgraph_msgs")

os.environ.setdefault("ROS_DOMAIN_ID", "95")

import launch
import launch_testing
import launch_testing.actions
import launch_testing.asserts
import rclpy
from launch_ros.actions import Node as LaunchNode
from nav_msgs.msg import Odometry
from rclpy.qos import HistoryPolicy, QoSProfile, ReliabilityPolicy
from rclpy.time import Time
from rosgraph_msgs.msg import Clock
from sensor_msgs.msg import LaserScan

_ACK_TOPIC = "/test/ack"
_MAX_LAG_S = 0.1
_SHUT_GATE_TIMEOUT_S = 3.0


def _bridge(name: str, extra: dict) -> LaunchNode:
    return LaunchNode(
        package="racer_gym_bridge",
        executable="bridge_node",
        name=name,
        parameters=[
            {
                "seed": 5,
                "synthetic_track": "lane_round",
                "publish_wheel_odom": True,
                "wheel_odom_rate_hz": 50.0,
                "scan_rate_hz": 10.0,
                "sim_time": True,
                **extra,
            }
        ],
    )


@pytest.mark.launch_test
def generate_test_description():
    bridge = _bridge(
        "bridge_node",
        {
            "real_time_factor_max": 1.0,
            "lockstep_ack": [_ACK_TOPIC],
            "lockstep_max_lag_s": _MAX_LAG_S,
            "lockstep_timeout_s": _SHUT_GATE_TIMEOUT_S,
        },
    )
    return launch.LaunchDescription([bridge, launch_testing.actions.ReadyToTest()])


def _stamp_ns(stamp) -> int:
    return Time.from_msg(stamp).nanoseconds


class TestBridgeSimTime(unittest.TestCase):
    @classmethod
    def setUpClass(cls):
        rclpy.init()
        cls.node = rclpy.create_node("test_bridge_sim_time")
        cls.clock: list[int] = []
        cls.scan_stamps: list[int] = []
        cls.truth_stamps: list[int] = []
        cls.wheel_stamps: list[int] = []
        n = cls.node
        clock_qos = QoSProfile(
            reliability=ReliabilityPolicy.BEST_EFFORT, history=HistoryPolicy.KEEP_LAST, depth=1000
        )
        n.create_subscription(
            Clock, "/clock", lambda m: cls.clock.append(_stamp_ns(m.clock)), clock_qos
        )
        best_effort = QoSProfile(
            reliability=ReliabilityPolicy.BEST_EFFORT, history=HistoryPolicy.KEEP_LAST, depth=500
        )
        reliable = QoSProfile(
            reliability=ReliabilityPolicy.RELIABLE, history=HistoryPolicy.KEEP_LAST, depth=500
        )
        n.create_subscription(
            LaserScan,
            "/scan",
            lambda m: cls.scan_stamps.append(_stamp_ns(m.header.stamp)),
            best_effort,
        )
        n.create_subscription(
            Odometry,
            "/sim/ground_truth_odom",
            lambda m: cls.truth_stamps.append(_stamp_ns(m.header.stamp)),
            reliable,
        )
        n.create_subscription(
            Odometry,
            "/odom/wheel",
            lambda m: cls.wheel_stamps.append(_stamp_ns(m.header.stamp)),
            reliable,
        )
        cls.ack_pub = n.create_publisher(Odometry, _ACK_TOPIC, reliable)
        # Warm-up: the gym JITs on its first step (see test_bridge_node.py).
        deadline = time.monotonic() + 60.0
        while len(cls.clock) < 5 and time.monotonic() < deadline:
            rclpy.spin_once(n, timeout_sec=0.1)

    @classmethod
    def tearDownClass(cls):
        cls.node.destroy_node()
        rclpy.shutdown()

    def _spin(self, seconds: float, ack_with_clock: bool = False) -> None:
        end = time.monotonic() + seconds
        while time.monotonic() < end:
            if ack_with_clock and self.clock:
                self._ack(self.clock[-1])
            rclpy.spin_once(self.node, timeout_sec=0.005)

    def _ack(self, stamp_ns: int) -> None:
        msg = Odometry()
        msg.header.stamp = Time(nanoseconds=stamp_ns).to_msg()
        self.ack_pub.publish(msg)

    def test_clock_is_monotonic_in_whole_timesteps_and_stamps_come_from_it(self):
        self._spin(2.0, ack_with_clock=True)
        clock = list(self.clock)
        self.assertGreater(len(clock), 50)
        steps = {b - a for a, b in itertools.pairwise(clock)}
        # Every step is one env timestep (10 ms), exactly; a repeated value (best-effort
        # duplicate) or a skipped one (a dropped /clock message) is a multiple of it.
        self.assertTrue(all(s % 10_000_000 == 0 and s >= 0 for s in steps), steps)
        published = set(clock)
        horizon = clock[-1]
        for name, stamps in (
            ("scan", self.scan_stamps),
            ("ground truth", self.truth_stamps),
            ("wheel", self.wheel_stamps),
        ):
            self.assertTrue(stamps, f"no {name} messages")
            # A stamp the test saw on /clock, or a later one whose /clock the test dropped
            # (best effort): never a wall time, which would be around 1.7e18 ns.
            for stamp in stamps:
                self.assertLessEqual(
                    stamp, horizon + 10**9, f"{name} stamp {stamp} is not sim time"
                )
                self.assertEqual(stamp % 10_000_000, 0, f"{name} stamp {stamp} is not on a step")
        self.assertTrue(set(self.scan_stamps[-20:]) & published)

    def test_sim_time_does_not_run_faster_than_real_time(self):
        self._spin(0.5, ack_with_clock=True)
        start_sim, start_wall = self.clock[-1], time.monotonic()
        self._spin(3.0, ack_with_clock=True)
        sim_s = (self.clock[-1] - start_sim) * 1e-9
        wall_s = time.monotonic() - start_wall
        self.assertLessEqual(sim_s, 1.05 * wall_s + 0.05)
        self.assertGreater(sim_s, 0.5 * wall_s, "the sim barely advanced")

    def test_gate_holds_the_clock_until_the_acknowledging_output_catches_up(self):
        self._spin(0.5, ack_with_clock=True)
        acked = self.clock[-1]
        self._ack(acked)  # arm the gate at this sim time, then stay silent
        self._spin(1.0)
        held_at = self.clock[-1]
        # At most max_lag plus the steps in flight when the gate shut.
        self.assertLessEqual((held_at - acked) * 1e-9, _MAX_LAG_S + 0.1)
        self._spin(1.0)
        self.assertEqual(self.clock[-1], held_at, "the clock moved with the gate shut")
        # Catching up reopens it.
        self._spin(1.0, ack_with_clock=True)
        self.assertGreater(self.clock[-1], held_at + 500_000_000)


@launch_testing.post_shutdown_test()
class TestBridgeSimTimeShutdown(unittest.TestCase):
    def test_clean_exit(self, proc_info):
        launch_testing.asserts.assertExitCodes(proc_info, allowable_exit_codes=[0, -signal.SIGINT])
