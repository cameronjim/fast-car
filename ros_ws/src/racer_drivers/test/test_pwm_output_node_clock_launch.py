"""L3 regression test for pwm_output_node's /drive staleness CLOCK (claude-docs/12-testing.md
L3 "correct behavior on silence (watchdog paths)").

The sibling of racer_safety/test/test_safety_node_clock_launch.py and racer_control/test/
test_tracker_node_clock_launch.py, written for the same defect in the third node to have it.
pwm_output_node measured `age_s` on `this->now()` -- the node's ROS clock -- which is not
monotonic, and this is the LAST node in the command path: its timeout is the only thing that
takes a stale pulse off the wire.

Two ways the ROS clock breaks that, both removed by measuring on RCL_STEADY_TIME:

  * `use_sim_time:=true` with no /clock publisher pins the ROS clock at zero, so every
    measured age is exactly 0.0, `is_stale`'s `age_s >= timeout_s` is never true, and the last
    commanded throttle pulse stays on the wire forever. That is what this file reproduces --
    it is the cheap, deterministic half, and it needs no clock stepping.
  * `use_sim_time:=false` with a BACKWARDS wall-clock step (NTP on a Jetson with no RTC
    battery, a VM resync) makes the age negative, which the same comparison also reads as
    fresh. `is_stale` now fails closed on a negative age as well (defence in depth,
    table-driven in test_pwm_mapping.cpp's IsStale cases).

Against the pre-review sources this file FAILS: the throttle channel stays at the commanded
pulse instead of returning to neutral.

FAKE SYSFS: same approach as test_pwm_output_node_launch.py -- a temp directory laid out like
/sys/class/pwm, so the real SysfsPwmChannel path runs with ordinary files. The pulse values
are read out of config/vehicle_params.yaml at test time, never typed in (CLAUDE.md
invariant 2).
"""

from __future__ import annotations

import os

# Distinct from every domain already claimed in this workspace (racer_control 77/79,
# racer_safety 78/80, racer_drivers' other launch test 81, racer_bringup 82). setdefault, not
# a plain assignment, so an operator-set ROS_DOMAIN_ID is never clobbered.
os.environ.setdefault("ROS_DOMAIN_ID", "83")

import pathlib
import shutil
import tempfile
import time
import unittest

import launch
import launch_testing
import launch_testing.actions
import pytest
import rclpy
import yaml
from ackermann_msgs.msg import AckermannDriveStamped
from launch_ros.actions import Node as LaunchNode
from rclpy.qos import HistoryPolicy, QoSProfile, ReliabilityPolicy

_OUTPUT_RATE_HZ = 50.0
_DRIVE_TIMEOUT_S = 0.2

_REPO_ROOT = pathlib.Path(__file__).resolve().parents[4]
_PARAMS_PATH = _REPO_ROOT / "config" / "vehicle_params.yaml"

with open(_PARAMS_PATH, "r", encoding="utf-8") as _handle:
    _PARAMS = yaml.safe_load(_handle)

_THROTTLE_NEUTRAL_US = _PARAMS["actuation"]["throttle_pwm_neutral_us"]
_STEERING_NEUTRAL_US = _PARAMS["steering"]["pwm_neutral_us"]

_FAKE_SYSFS = tempfile.mkdtemp(prefix="racer_fake_sysfs_clock_")


def _channel_dir(channel: int) -> pathlib.Path:
    return pathlib.Path(_FAKE_SYSFS) / "pwmchip0" / f"pwm{channel}"


def _make_fake_sysfs() -> None:
    chip = pathlib.Path(_FAKE_SYSFS) / "pwmchip0"
    chip.mkdir(parents=True, exist_ok=True)
    (chip / "export").write_text("", encoding="utf-8")
    (chip / "unexport").write_text("", encoding="utf-8")
    for channel in (0, 1):
        channel_dir = chip / f"pwm{channel}"
        channel_dir.mkdir(exist_ok=True)
        (channel_dir / "period").write_text("0", encoding="utf-8")
        (channel_dir / "duty_cycle").write_text("0", encoding="utf-8")
        (channel_dir / "enable").write_text("0", encoding="utf-8")


def _read_int(path: pathlib.Path) -> int:
    return int(path.read_text(encoding="utf-8").strip() or "0")


def _duty_ns(channel: int) -> int:
    return _read_int(_channel_dir(channel) / "duty_cycle")


def _expected_duty_ns(pulse_us: float) -> int:
    return round(pulse_us * 1000.0)


def _reliable_qos() -> QoSProfile:
    return QoSProfile(
        reliability=ReliabilityPolicy.RELIABLE, history=HistoryPolicy.KEEP_LAST, depth=10
    )


@pytest.mark.launch_test
def generate_test_description():
    _make_fake_sysfs()
    pwm_output_node = LaunchNode(
        package="racer_drivers",
        executable="pwm_output_node",
        name="pwm_output_node",
        parameters=[
            {
                # The whole point of this file: nothing publishes /clock, so this node's ROS
                # clock is pinned at zero for its entire life. Only a node that measures
                # staleness on a steady clock still times out.
                "use_sim_time": True,
                "output_rate_hz": _OUTPUT_RATE_HZ,
                "drive_timeout_s": _DRIVE_TIMEOUT_S,
                "sysfs_root": _FAKE_SYSFS,
                "steering_pwmchip": 0,
                "steering_pwm_channel": 0,
                "throttle_pwmchip": 0,
                "throttle_pwm_channel": 1,
                "steering_left_is_pwm_max": True,
            }
        ],
        output="screen",
    )
    return launch.LaunchDescription([pwm_output_node, launch_testing.actions.ReadyToTest()]), {
        "fake_sysfs": _FAKE_SYSFS
    }


class TestPwmOutputNodeStalenessClock(unittest.TestCase):
    @classmethod
    def setUpClass(cls):
        rclpy.init()
        deadline = time.monotonic() + 30.0
        while time.monotonic() < deadline and _read_int(_channel_dir(0) / "enable") != 1:
            time.sleep(0.1)
        if _read_int(_channel_dir(0) / "enable") != 1:
            raise RuntimeError("pwm_output_node never enabled the steering channel")

    @classmethod
    def tearDownClass(cls):
        rclpy.shutdown()

    def setUp(self):
        self.node = rclpy.create_node("test_pwm_output_clock_client")

    def tearDown(self):
        self.node.destroy_node()

    def _spin_for(self, seconds: float) -> None:
        end = time.monotonic() + seconds
        while time.monotonic() < end:
            rclpy.spin_once(self.node, timeout_sec=0.02)

    def _publish_steadily(self, publisher, msg, seconds: float) -> None:
        end = time.monotonic() + seconds
        while time.monotonic() < end:
            publisher.publish(msg)
            rclpy.spin_once(self.node, timeout_sec=0.01)

    def test_a_node_still_writes_pulses_with_a_frozen_ros_clock(self):
        """Precondition: the output loop is a WALL timer, so a frozen ROS clock does not stop
        it. Without this the staleness assertion below would pass vacuously."""
        drive_pub = self.node.create_publisher(AckermannDriveStamped, "/drive", _reliable_qos())
        self._spin_for(0.5)
        msg = AckermannDriveStamped()
        msg.drive.steering_angle = 0.0
        msg.drive.speed = 1.0
        self._publish_steadily(drive_pub, msg, seconds=0.5)
        self.assertNotEqual(
            _duty_ns(1),
            _expected_duty_ns(_THROTTLE_NEUTRAL_US),
            "pwm_output_node never left neutral with use_sim_time:=true and no /clock; its "
            "output loop must be a wall timer, independent of the ROS clock",
        )

    def test_b_drive_silence_returns_both_channels_to_neutral_with_a_frozen_ros_clock(self):
        """The regression test. One /drive command, then silence for many multiples of
        drive_timeout_s. A node measuring age on its own (frozen) ROS clock measures 0.0 s
        forever, never times out, and leaves the commanded throttle pulse on the wire."""
        drive_pub = self.node.create_publisher(AckermannDriveStamped, "/drive", _reliable_qos())
        self._spin_for(0.5)

        msg = AckermannDriveStamped()
        msg.drive.steering_angle = 0.2
        msg.drive.speed = 1.0
        self._publish_steadily(drive_pub, msg, seconds=0.5)
        self.assertNotEqual(
            _duty_ns(1),
            _expected_duty_ns(_THROTTLE_NEUTRAL_US),
            "the node never acted on the /drive command at all",
        )

        # Silence, for 10x the timeout.
        self._spin_for(_DRIVE_TIMEOUT_S * 10.0)

        self.assertEqual(
            _duty_ns(1),
            _expected_duty_ns(_THROTTLE_NEUTRAL_US),
            "throttle channel did not return to neutral after /drive went silent, with the "
            "ROS clock frozen: staleness is being measured on the ROS clock, not "
            "RCL_STEADY_TIME. A dead /drive publisher leaves a driving pulse on the wire.",
        )
        self.assertEqual(
            _duty_ns(0),
            _expected_duty_ns(_STEERING_NEUTRAL_US),
            "steering channel did not return to neutral either",
        )


@launch_testing.post_shutdown_test()
class TestPwmOutputNodeClockShutdown(unittest.TestCase):
    def test_clean_exit(self, proc_info):
        launch_testing.asserts.assertExitCodes(proc_info)

    def test_z_cleanup_fake_sysfs(self):
        shutil.rmtree(_FAKE_SYSFS, ignore_errors=True)
