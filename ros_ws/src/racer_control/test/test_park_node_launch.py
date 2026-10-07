"""L3 test of park_node (roadmap 2.9), launched through the real launch/park.launch.py with
test/fixtures/park_test_params.yaml (short timings, synthetic-scan yaw) and require_odometry left
at its default, true.

The test plays the rest of the graph (park_launch_common.py): a synthetic /scan, /odom/wheel
integrating what the "car" was given, and the gated /drive standing in for safety_node. Checks,
in order:
  * IDLE publishes nothing on /drive_raw.
  * ~/start without /odom/wheel is refused (require_odometry), and ~/abort while idle too.
  * with odometry: ~/start succeeds, SEARCH drives straight ahead at search_speed_mps (steering
    0, never reverse), ~/status says SEARCH.
  * the stand-in safety_node holds the request at zero: after blocked_abort_s park_node ABORTs,
    names the phase, publishes zero for final_hold_s, then goes quiet.
  * ~/start again, then ~/abort: ABORT, zero, quiet.
  * ~/three_point_turn in a 1.2 m lane is refused (lane too narrow) and nothing moves.
  * park_node never publishes /drive; the QoS matches (best_effort /scan, reliable /odom/wheel
    and /drive, reliable /drive_raw), proven by the node reacting to each.
"""

from __future__ import annotations

import os

# Own domain per launch test file, see test_tracker_node_launch.py.
os.environ.setdefault("ROS_DOMAIN_ID", "90")

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

_SEARCH_SPEED_MPS = 0.5  # node default
_BLOCKED_ABORT_S = 0.5  # params file
_FINAL_HOLD_S = 0.5  # params file


@pytest.mark.launch_test
def generate_test_description():
    launch_file = Path(get_package_share_directory("racer_control")) / "launch" / "park.launch.py"
    park = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(str(launch_file)),
        launch_arguments={"params_file": _PARAMS_FILE}.items(),
    )
    return launch.LaunchDescription([park, launch_testing.actions.ReadyToTest()])


class TestParkNode(unittest.TestCase):
    @classmethod
    def setUpClass(cls):
        rclpy.init()

    @classmethod
    def tearDownClass(cls):
        rclpy.shutdown()

    def setUp(self):
        self.node = rclpy.create_node("test_park_node_client")

    def tearDown(self):
        self.node.destroy_node()

    def _expect_abort_then_quiet(self, harness, gate: str) -> None:
        """Zero requests for about final_hold_s after an ABORT, then silence."""
        seen = harness.spin(_FINAL_HOLD_S + 1.0, gate=gate)
        self.assertTrue(seen, "no zero command after ABORT")
        first = seen[0][0]
        held = [s for t, s, _ in seen if t - first < _FINAL_HOLD_S - 0.1]
        self.assertTrue(held)
        self.assertTrue(all(abs(s) < 1e-6 for s in held), f"non-zero after ABORT: {held}")
        self.assertLess(seen[-1][0] - first, _FINAL_HOLD_S + 0.3, "still publishing after hold")

    def test_park_node(self, proc_output):
        harness = common.Harness(self.node, odometry=False)
        harness.wait_for_services(self)

        # 1. IDLE: nothing on /drive_raw.
        self.assertEqual(harness.spin(1.0), [], "park_node published while IDLE")

        # 2. No odometry: ~/start refused; ~/abort while idle refused.
        response = harness.call(self, "start")
        self.assertFalse(response.success)
        self.assertIn("require_odometry", response.message)
        self.assertFalse(harness.call(self, "abort").success)
        self.assertEqual(harness.spin(0.5), [], "a refused start moved the car")

        # 3. With odometry: SEARCH straight ahead.
        harness.odometry = True
        harness.spin(0.5)
        response = harness.call(self, "start")
        self.assertTrue(response.success, response.message)
        seen = harness.spin(3.0, until=lambda s: s and s[-1][1] >= _SEARCH_SPEED_MPS - 1e-3)
        self.assertTrue(seen, "no /drive_raw after ~/start")
        speeds = [s for _, s, _ in seen]
        self.assertGreaterEqual(max(speeds), _SEARCH_SPEED_MPS - 1e-3)
        self.assertLessEqual(max(speeds), _SEARCH_SPEED_MPS + 1e-3)
        self.assertGreaterEqual(min(speeds), 0.0, "SEARCH reversed")
        self.assertTrue(all(abs(st) < 1e-6 for _, _, st in seen), "SEARCH steered")
        proc_output.assertWaitFor("IDLE -> SEARCH", timeout=5, stream="stderr")
        harness.spin(0.6)
        self.assertTrue(any("phase=SEARCH" in s for s in harness.statuses), harness.statuses[-3:])

        # 4. safety_node holds the car: ABORT after blocked_abort_s, naming the phase.
        harness.spin(_BLOCKED_ABORT_S + 0.5, gate="hold", until=lambda s: s and s[-1][1] == 0.0)
        proc_output.assertWaitFor("safety_node held the car", timeout=5, stream="stderr")
        proc_output.assertWaitFor("in SEARCH", timeout=5, stream="stderr")
        self._expect_abort_then_quiet(harness, gate="hold")

        # 5. Start again, then ~/abort.
        response = harness.call(self, "start")
        self.assertTrue(response.success, response.message)
        harness.spin(1.0)
        response = harness.call(self, "abort")
        self.assertTrue(response.success, response.message)
        proc_output.assertWaitFor("abort requested", timeout=5, stream="stderr")
        self._expect_abort_then_quiet(harness, gate="pass")

        # 6. Three-point turn in a lane too narrow for it: refused, nothing moves.
        harness.scan = common.scan_of(common.NARROW_LANE_WALLS)
        harness.spin(0.5)
        response = harness.call(self, "three_point_turn")
        self.assertFalse(response.success)
        self.assertIn("lane too narrow", response.message)
        self.assertEqual(harness.spin(0.5), [], "a refused three-point turn moved the car")

        # 7. park_node never publishes /drive.
        publishers = self.node.get_publishers_info_by_topic("/drive")
        self.assertEqual([p.node_name for p in publishers if p.node_name == "park_node"], [])


@launch_testing.post_shutdown_test()
class TestParkNodeShutdown(unittest.TestCase):
    def test_clean_exit(self, proc_info):
        launch_testing.asserts.assertExitCodes(proc_info, allowable_exit_codes=[0, -signal.SIGINT])
