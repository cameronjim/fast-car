"""L3-style launch test for car_teleop.launch.py (claude-docs/12-testing.md L3).

Brings up the REAL car launch file -- safety_node + pwm_output_node, no sim bridge -- with
pwm_output_node pointed at a fake sysfs tree (ordinary files in a temp dir laid out like
/sys/class/pwm), and asserts the state the car must be in at first boot with nothing driving
it:

  * both nodes are actually up,
  * /drive is neutral (zero steering, zero speed) with no /drive_raw publisher anywhere,
  * both PWM channels sit at the calibrated neutral pulse and are enabled,
  * pwm_output_node subscribes to /drive and NOT to /drive_raw (CLAUDE.md invariant 1),
  * on shutdown both channels are left neutral and disabled.

foxglove_bridge is switched off here (`viz:=false`): it is not part of the command path and
binding port 8765 in CI would be a flake source, not a test.

Neutral pulse values are read from config/vehicle_params.yaml at test time rather than typed
in (CLAUDE.md invariant 2).
"""

from __future__ import annotations

import os

# Distinct DDS domain from every other launch_testing suite colcon runs concurrently
# (racer_control 77 and 79, racer_safety 78, racer_drivers 81): this file starts its OWN
# safety_node, which publishes /drive and subscribes /drive_raw, and must not see or be seen
# by theirs.
os.environ.setdefault("ROS_DOMAIN_ID", "82")

import pathlib
import shutil
import signal
import tempfile
import time
import unittest

import launch
import launch_testing
import launch_testing.actions
import launch_testing.asserts
import pytest
import rclpy
import yaml
from ackermann_msgs.msg import AckermannDriveStamped
from ament_index_python.packages import get_package_share_directory
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from rclpy.qos import HistoryPolicy, QoSProfile, ReliabilityPolicy

_REPO_ROOT = pathlib.Path(__file__).resolve().parents[4]
# The INSTALLED launch file, not the source one: this test should fail if car_teleop.launch.py
# is ever left out of racer_bringup's install() rule.
_LAUNCH_FILE = (
    pathlib.Path(get_package_share_directory("racer_bringup")) / "launch" / "car_teleop.launch.py"
)
_PARAMS_PATH = _REPO_ROOT / "config" / "vehicle_params.yaml"

with open(_PARAMS_PATH, "r", encoding="utf-8") as _handle:
    _PARAMS = yaml.safe_load(_handle)
_STEERING_NEUTRAL_NS = round(_PARAMS["steering"]["pwm_neutral_us"] * 1000.0)
_THROTTLE_NEUTRAL_NS = round(_PARAMS["actuation"]["throttle_pwm_neutral_us"] * 1000.0)

_FAKE_SYSFS = tempfile.mkdtemp(prefix="racer_car_teleop_sysfs_")


def _make_fake_sysfs() -> None:
    chip = pathlib.Path(_FAKE_SYSFS) / "pwmchip0"
    chip.mkdir(parents=True, exist_ok=True)
    (chip / "export").write_text("", encoding="utf-8")
    for channel in (0, 1):
        channel_dir = chip / f"pwm{channel}"
        channel_dir.mkdir(exist_ok=True)
        for attribute in ("period", "duty_cycle", "enable"):
            (channel_dir / attribute).write_text("0", encoding="utf-8")


def _attribute(channel: int, name: str) -> int:
    path = pathlib.Path(_FAKE_SYSFS) / "pwmchip0" / f"pwm{channel}" / name
    return int(path.read_text(encoding="utf-8").strip() or "0")


@pytest.mark.launch_test
def generate_test_description():
    _make_fake_sysfs()
    car_teleop = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(str(_LAUNCH_FILE)),
        launch_arguments={
            "viz": "false",
            "start_teleop": "false",
            "browser_teleop": "false",
            # record:=false HERE ONLY, and only because this test is about the command path
            # at rest: a recorder would write a bag into the repo on every colcon test for no
            # assertion's benefit. The recorder and rail_voltage_node defaults -- CLAUDE.md
            # invariant 5 -- are asserted by test_car_teleop_bag_launch.py, which passes no
            # `record` argument at all precisely so the DEFAULT is what it tests. Never copy
            # this line into a procedure that drives the car.
            "record": "false",
            "rail_voltage": "false",
            "sysfs_root": _FAKE_SYSFS,
            "steering_pwmchip": "0",
            "steering_pwm_channel": "0",
            "throttle_pwmchip": "0",
            "throttle_pwm_channel": "1",
        }.items(),
    )
    return launch.LaunchDescription([car_teleop, launch_testing.actions.ReadyToTest()]), {
        "fake_sysfs": _FAKE_SYSFS
    }


class TestCarTeleopLaunch(unittest.TestCase):
    @classmethod
    def setUpClass(cls):
        rclpy.init()

    @classmethod
    def tearDownClass(cls):
        rclpy.shutdown()

    def setUp(self):
        self.node = rclpy.create_node("test_car_teleop_client")

    def tearDown(self):
        self.node.destroy_node()

    def _spin_for(self, seconds: float) -> None:
        end = time.time() + seconds
        while time.time() < end:
            rclpy.spin_once(self.node, timeout_sec=0.05)

    def _wait_for_nodes(self, names, timeout_s: float = 30.0) -> list[str]:
        deadline = time.time() + timeout_s
        found: list[str] = []
        while time.time() < deadline:
            found = self.node.get_node_names()
            if all(name in found for name in names):
                return found
            self._spin_for(0.2)
        return found

    def test_a_both_command_path_nodes_come_up(self):
        found = self._wait_for_nodes(["safety_node", "pwm_output_node"])
        self.assertIn("safety_node", found, f"safety_node not in node graph: {found}")
        self.assertIn("pwm_output_node", found, f"pwm_output_node not in node graph: {found}")

    def test_b_drive_is_neutral_with_no_input(self):
        received: list[AckermannDriveStamped] = []
        self.node.create_subscription(
            AckermannDriveStamped,
            "/drive",
            received.append,
            QoSProfile(
                reliability=ReliabilityPolicy.RELIABLE, history=HistoryPolicy.KEEP_LAST, depth=10
            ),
        )
        deadline = time.time() + 30.0
        while len(received) < 5 and time.time() < deadline:
            rclpy.spin_once(self.node, timeout_sec=0.2)
        self.assertGreaterEqual(len(received), 5, "safety_node published almost no /drive")
        for msg in received[-5:]:
            self.assertEqual(msg.drive.steering_angle, 0.0)
            self.assertEqual(msg.drive.speed, 0.0)

    def test_c_both_pwm_channels_sit_at_neutral(self):
        deadline = time.time() + 30.0
        while time.time() < deadline and _attribute(0, "enable") != 1:
            self._spin_for(0.2)
        self.assertEqual(_attribute(0, "enable"), 1, "steering channel never enabled")
        self.assertEqual(_attribute(1, "enable"), 1, "throttle channel never enabled")
        # Let a few 50 Hz cycles run so this is the steady state, not just the start value.
        self._spin_for(0.5)
        self.assertEqual(_attribute(0, "duty_cycle"), _STEERING_NEUTRAL_NS)
        self.assertEqual(_attribute(1, "duty_cycle"), _THROTTLE_NEUTRAL_NS)

    def test_d_pwm_output_node_never_subscribes_to_drive_raw(self):
        self._wait_for_nodes(["pwm_output_node"])
        self._spin_for(0.5)
        topics = [
            name
            for name, _ in self.node.get_subscriber_names_and_types_by_node("pwm_output_node", "/")
        ]
        self.assertIn("/drive", topics)
        self.assertNotIn(
            "/drive_raw",
            topics,
            "pwm_output_node subscribes /drive_raw, bypassing safety_node (CLAUDE.md invariant 1)",
        )


@launch_testing.post_shutdown_test()
class TestCarTeleopShutdown(unittest.TestCase):
    def test_clean_exit(self, proc_info):
        launch_testing.asserts.assertExitCodes(proc_info, allowable_exit_codes=[0, -signal.SIGINT])

    def test_pwm_left_neutral_and_disabled(self, fake_sysfs):
        root = pathlib.Path(fake_sysfs) / "pwmchip0"
        for channel, neutral_ns in ((0, _STEERING_NEUTRAL_NS), (1, _THROTTLE_NEUTRAL_NS)):
            duty = int((root / f"pwm{channel}" / "duty_cycle").read_text().strip())
            enable = int((root / f"pwm{channel}" / "enable").read_text().strip())
            self.assertEqual(duty, neutral_ns, f"channel {channel} not left at neutral")
            self.assertEqual(enable, 0, f"channel {channel} left enabled")
        shutil.rmtree(pathlib.Path(fake_sysfs), ignore_errors=True)
