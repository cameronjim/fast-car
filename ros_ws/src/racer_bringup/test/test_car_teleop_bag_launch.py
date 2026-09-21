"""L3 launch test for car_teleop.launch.py's LOGGING (claude-docs/12-testing.md L3).

GitHub issue #64 / roadmap 1.6 / CLAUDE.md invariant 5: "Every run is logged (rosbag + rail
voltage). Code paths that drive the car without logging are bugs." This test is what stops
that regressing to a warning in a docs file. It brings the real car launch up -- no
`record:=...` argument at all, so it is asserting the DEFAULT -- with

  * pwm_output_node pointed at a fake sysfs PWM tree (as test_car_teleop_launch.py does),
  * rail_voltage_node pointed at a fake INA3221 hwmon tree of ordinary files, so the rail
    topics exist and carry known values on a machine with no Jetson in it,
  * the TEST itself acting as the teleop source -- it publishes /teleop/cmd_vel and
    /drive_raw directly, rather than starting twist_teleop_adapter_node -- so that what is
    under test is what gets RECORDED, not another node's health,

and asserts that a dated bag directory appears under bag_dir and that, once the bag is
closed, its metadata lists every topic invariant 5 requires -- including rail voltage.

Storage: `bag_storage` is left at `auto`, which resolves to sqlite3 in docker/ros-dev (stock
ros:humble-ros-base, no mcap plugin) and to mcap on the car. The assertions below are
storage-agnostic on purpose: they read metadata.yaml, which rosbag2 writes either way.
"""

from __future__ import annotations

import os

# Distinct DDS domain from every other launch_testing suite colcon runs concurrently
# (racer_control 77/79, racer_safety 78, racer_drivers 81, test_car_teleop_launch.py 82):
# this file starts its OWN safety_node and its own recorder, and a recorder that saw another
# suite's /drive would make these assertions meaningless.
os.environ.setdefault("ROS_DOMAIN_ID", "83")

import pathlib
import re
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
from geometry_msgs.msg import Twist
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from std_msgs.msg import Float32

_LAUNCH_FILE = (
    pathlib.Path(get_package_share_directory("racer_bringup")) / "launch" / "car_teleop.launch.py"
)

_FAKE_SYSFS = tempfile.mkdtemp(prefix="racer_car_teleop_bag_sysfs_")
_FAKE_INA3221 = tempfile.mkdtemp(prefix="racer_car_teleop_bag_ina3221_")
_BAG_ROOT = tempfile.mkdtemp(prefix="racer_car_teleop_bag_root_")

# Fixture rail values, in the hwmon ABI's milli-units. rail_voltage.py divides by 1000, so
# these are the volts/amps the topics must carry (CLAUDE.md invariant 4: SI on the wire).
_VDD_IN_MILLIVOLTS = 11952
_VDD_IN_MILLIAMPS = 1440

_BAG_DIR_NAME_RE = re.compile(r"^\d{4}-\d{2}-\d{2}T\d{2}-\d{2}-\d{2}_car_teleop$")

# What a drive has to be reconstructable from. /scan is deliberately NOT here: no LiDAR is
# fitted, and the launch file records by regex precisely so an absent topic is skipped rather
# than waited on -- test_g below asserts that.
_REQUIRED_BAG_TOPICS = [
    "/drive",
    "/drive_raw",
    "/safety/events",
    "/teleop/cmd_vel",
    "/rosout",
    "/parameter_events",
    "/telemetry/rail_voltage_v",
    "/telemetry/rail_current_a",
    "/telemetry/rail/vdd_in/voltage_v",
]


def _make_fake_sysfs() -> None:
    chip = pathlib.Path(_FAKE_SYSFS) / "pwmchip0"
    chip.mkdir(parents=True, exist_ok=True)
    (chip / "export").write_text("", encoding="utf-8")
    for channel in (0, 1):
        channel_dir = chip / f"pwm{channel}"
        channel_dir.mkdir(exist_ok=True)
        for attribute in ("period", "duty_cycle", "enable"):
            (channel_dir / attribute).write_text("0", encoding="utf-8")


def _make_fake_ina3221() -> None:
    """An Orin-Nano-shaped ina3221 tree: <bus-addr>/hwmon/hwmon<N>/in*_label|input."""
    hwmon = pathlib.Path(_FAKE_INA3221) / "1-0040" / "hwmon" / "hwmon3"
    hwmon.mkdir(parents=True, exist_ok=True)
    for index, label, millivolts, milliamps in (
        (1, "VDD_IN", _VDD_IN_MILLIVOLTS, _VDD_IN_MILLIAMPS),
        (2, "VDD_CPU_GPU_CV", 4832, 560),
        (3, "VDD_SOC", 3296, 912),
    ):
        (hwmon / f"in{index}_label").write_text(f"{label}\n", encoding="utf-8")
        (hwmon / f"in{index}_input").write_text(f"{millivolts}\n", encoding="utf-8")
        (hwmon / f"curr{index}_input").write_text(f"{milliamps}\n", encoding="utf-8")


def _bag_dirs() -> list[pathlib.Path]:
    return sorted(path for path in pathlib.Path(_BAG_ROOT).iterdir() if path.is_dir())


@pytest.mark.launch_test
def generate_test_description():
    _make_fake_sysfs()
    _make_fake_ina3221()
    car_teleop = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(str(_LAUNCH_FILE)),
        launch_arguments={
            # NOTE: `record` is deliberately NOT passed. The default is the thing under test.
            "viz": "false",
            "start_teleop": "false",
            # No teleop node: this test publishes /teleop/cmd_vel and /drive_raw itself (see
            # test_d), so a failure here is always a recording failure.
            "browser_teleop": "false",
            "bag_dir": _BAG_ROOT,
            "ina3221_root": _FAKE_INA3221,
            "sysfs_root": _FAKE_SYSFS,
            "steering_pwmchip": "0",
            "steering_pwm_channel": "0",
            "throttle_pwmchip": "0",
            "throttle_pwm_channel": "1",
        }.items(),
    )
    return launch.LaunchDescription([car_teleop, launch_testing.actions.ReadyToTest()]), {}


class TestCarTeleopRecords(unittest.TestCase):
    @classmethod
    def setUpClass(cls):
        rclpy.init()

    @classmethod
    def tearDownClass(cls):
        rclpy.shutdown()

    def setUp(self):
        self.node = rclpy.create_node("test_car_teleop_bag_client")

    def tearDown(self):
        self.node.destroy_node()

    def _spin_for(self, seconds: float) -> None:
        end = time.time() + seconds
        while time.time() < end:
            rclpy.spin_once(self.node, timeout_sec=0.05)

    def test_a_a_dated_bag_directory_is_created_by_default(self):
        deadline = time.time() + 30.0
        while time.time() < deadline and not _bag_dirs():
            self._spin_for(0.2)
        dirs = _bag_dirs()
        self.assertEqual(
            len(dirs), 1, f"expected exactly one bag directory under {_BAG_ROOT}, got {dirs}"
        )
        self.assertRegex(
            dirs[0].name,
            _BAG_DIR_NAME_RE,
            "bag directory name must be <ISO timestamp>_<launch name>",
        )

    def test_b_rail_voltage_is_published_in_si_units(self):
        volts: list[float] = []
        amps: list[float] = []
        self.node.create_subscription(
            Float32, "/telemetry/rail_voltage_v", lambda msg: volts.append(msg.data), 10
        )
        self.node.create_subscription(
            Float32, "/telemetry/rail_current_a", lambda msg: amps.append(msg.data), 10
        )
        deadline = time.time() + 30.0
        while time.time() < deadline and (len(volts) < 2 or len(amps) < 2):
            rclpy.spin_once(self.node, timeout_sec=0.2)
        self.assertGreaterEqual(len(volts), 2, "rail_voltage_node published almost no voltage")
        self.assertGreaterEqual(len(amps), 2, "rail_voltage_node published almost no current")
        # Volts and amps, NOT the sysfs millivolts/milliamps (CLAUDE.md invariant 4).
        self.assertAlmostEqual(volts[-1], _VDD_IN_MILLIVOLTS / 1000.0, places=4)
        self.assertAlmostEqual(amps[-1], _VDD_IN_MILLIAMPS / 1000.0, places=4)

    def test_c_per_channel_rail_topics_exist_for_every_label(self):
        deadline = time.time() + 30.0
        wanted = {
            "/telemetry/rail/vdd_in/voltage_v",
            "/telemetry/rail/vdd_cpu_gpu_cv/voltage_v",
            "/telemetry/rail/vdd_soc/voltage_v",
        }
        seen: set[str] = set()
        while time.time() < deadline and not wanted.issubset(seen):
            seen = {name for name, _ in self.node.get_topic_names_and_types()}
            self._spin_for(0.2)
        self.assertTrue(wanted.issubset(seen), f"missing rail topics: {sorted(wanted - seen)}")

    def test_d_the_command_path_is_live_so_the_bag_has_something_to_record(self):
        """Put real traffic on the topics the recorder is supposed to be capturing.

        This is not a re-test of the command path (test_car_teleop_launch.py owns that). The
        test stands in for the teleop source: it publishes /teleop/cmd_vel (what a Foxglove
        Teleop panel sends) and /drive_raw (what a teleop node would produce from it) so the
        post-shutdown metadata assertions are about a bag with data in it.
        """
        cmd_vel = self.node.create_publisher(Twist, "/teleop/cmd_vel", 10)
        drive_raw = self.node.create_publisher(AckermannDriveStamped, "/drive_raw", 10)
        deadline = time.time() + 5.0
        while time.time() < deadline:
            cmd_vel.publish(Twist())
            drive_raw.publish(AckermannDriveStamped())
            self._spin_for(0.1)
        self.assertGreater(
            self.node.count_subscribers("/drive_raw"),
            0,
            "nothing subscribed /drive_raw -- neither safety_node nor the recorder",
        )


@launch_testing.post_shutdown_test()
class TestCarTeleopBagContents(unittest.TestCase):
    """Read the CLOSED bag: rosbag2 writes metadata.yaml when the recorder shuts down."""

    def test_f_metadata_lists_every_topic_invariant_5_requires(self):
        dirs = _bag_dirs()
        self.assertEqual(len(dirs), 1, f"expected exactly one bag directory, got {dirs}")
        metadata_path = dirs[0] / "metadata.yaml"
        self.assertTrue(metadata_path.is_file(), f"no metadata.yaml in {dirs[0]}")
        with open(metadata_path, "r", encoding="utf-8") as handle:
            metadata = yaml.safe_load(handle)["rosbag2_bagfile_information"]
        recorded = {
            entry["topic_metadata"]["name"] for entry in metadata["topics_with_message_count"]
        }
        missing = [topic for topic in _REQUIRED_BAG_TOPICS if topic not in recorded]
        self.assertEqual(
            missing, [], f"topics missing from the bag: {missing}; bag has {sorted(recorded)}"
        )
        self.assertGreater(metadata["message_count"], 0, "the bag is empty")

    def test_g_absent_topics_are_skipped_not_waited_on(self):
        """/scan is in the recorded set but no LiDAR is fitted: the run must still work."""
        dirs = _bag_dirs()
        with open(dirs[0] / "metadata.yaml", "r", encoding="utf-8") as handle:
            metadata = yaml.safe_load(handle)["rosbag2_bagfile_information"]
        recorded = {
            entry["topic_metadata"]["name"] for entry in metadata["topics_with_message_count"]
        }
        self.assertNotIn("/scan", recorded)

    def test_h_processes_exited_cleanly(self, proc_info):
        # SIGTERM is allowed alongside SIGINT here, unlike test_car_teleop_launch.py: the
        # `ros2 bag record` child is a python CLI process launch stops with its own escalation
        # policy, and which signal wins the race is not a property worth asserting.
        launch_testing.asserts.assertExitCodes(
            proc_info,
            allowable_exit_codes=[0, -signal.SIGINT, -signal.SIGTERM],
        )

    def test_i_cleanup(self):
        for path in (_FAKE_SYSFS, _FAKE_INA3221, _BAG_ROOT):
            shutil.rmtree(path, ignore_errors=True)
