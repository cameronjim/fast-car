"""L3 test of park_node's odometry fallback (roadmap 2.9): require_odometry false (the sim
fixtures' setting), no /odom/wheel at all.

Launched through park.launch.py with the shared test params file and `require_odometry:=false`
as a launch argument (which also covers that argument). Checks: ~/start succeeds without
odometry, SEARCH drives, park_node WARNs about the fallback (and keeps WARNing, once per second),
~/status reports odom_fallback=true, and ~/abort stops it.
"""

from __future__ import annotations

import os

# Own domain per launch test file, see test_tracker_node_launch.py.
os.environ.setdefault("ROS_DOMAIN_ID", "91")

import importlib.util
import signal
import unittest
from pathlib import Path

import launch
import launch_testing
import launch_testing.actions
import launch_testing.asserts
import pytest
import rclpy
from ament_index_python.packages import get_package_share_directory
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource

_HERE = Path(__file__).resolve().parent
_PARAMS_FILE = str(_HERE / "fixtures" / "park_test_params.yaml")
_SPEC = importlib.util.spec_from_file_location(
    "park_launch_common", _HERE / "park_launch_common.py"
)
common = importlib.util.module_from_spec(_SPEC)
_SPEC.loader.exec_module(common)


@pytest.mark.launch_test
def generate_test_description():
    launch_file = Path(get_package_share_directory("racer_control")) / "launch" / "park.launch.py"
    park = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(str(launch_file)),
        launch_arguments={"params_file": _PARAMS_FILE, "require_odometry": "false"}.items(),
    )
    return launch.LaunchDescription([park, launch_testing.actions.ReadyToTest()])


class TestParkNodeFallback(unittest.TestCase):
    @classmethod
    def setUpClass(cls):
        rclpy.init()

    @classmethod
    def tearDownClass(cls):
        rclpy.shutdown()

    def setUp(self):
        self.node = rclpy.create_node("test_park_node_fallback_client")

    def tearDown(self):
        self.node.destroy_node()

    def test_starts_and_warns_without_odometry(self, proc_output):
        harness = common.Harness(self.node, odometry=False)
        harness.wait_for_services(self)
        harness.spin(0.5)
        response = harness.call(self, "start")
        self.assertTrue(response.success, response.message)
        seen = harness.spin(2.5)
        self.assertTrue(seen, "no /drive_raw after ~/start")
        self.assertGreater(max(s for _, s, _ in seen), 0.4)
        proc_output.assertWaitFor("FALLING BACK", timeout=5, stream="stderr")
        proc_output.assertWaitFor("ODOMETRY FALLBACK", timeout=5, stream="stderr")
        self.assertTrue(
            any("odom_fallback=true" in s for s in harness.statuses), harness.statuses[-3:]
        )
        self.assertTrue(harness.call(self, "abort").success)
        proc_output.assertWaitFor("abort requested", timeout=5, stream="stderr")


@launch_testing.post_shutdown_test()
class TestParkNodeFallbackShutdown(unittest.TestCase):
    def test_clean_exit(self, proc_info):
        launch_testing.asserts.assertExitCodes(proc_info, allowable_exit_codes=[0, -signal.SIGINT])
