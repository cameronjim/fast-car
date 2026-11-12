"""L3: the geometric backend as a real node (needs rclpy and launch_testing; colcon only).

Test order matters and is by name: the silence test runs first, before any scan is published.
"""

from __future__ import annotations

import math
import signal
import unittest

import pytest

pytest.importorskip("rclpy")
pytest.importorskip("launch_testing")
pytest.importorskip("racer_pnp_msgs")
pytest.importorskip("racer_tools.vehicle_params_loader")

import launch
import launch_testing
import launch_testing.actions
import launch_testing.asserts
import rclpy
from launch_ros.actions import Node as LaunchNode
from node_harness import Harness, stamp_ns


@pytest.mark.launch_test
def generate_test_description():
    node = LaunchNode(
        package="racer_track_perception",
        executable="track_perception_node",
        name="track_perception_node",
        parameters=[{"backend": "geometric"}],
    )
    return launch.LaunchDescription([node, launch_testing.actions.ReadyToTest()])


class TestGeometricNode(unittest.TestCase):
    @classmethod
    def setUpClass(cls):
        rclpy.init()
        cls.h = Harness()

    @classmethod
    def tearDownClass(cls):
        cls.h.destroy()
        rclpy.shutdown()

    def test_a_silence_gives_no_track_and_a_status_that_says_so(self):
        self.h.spin(3.0)
        self.assertEqual(len(self.h.tracks), 0, "no /scan, so no /pnp/local_track")
        self.assertGreaterEqual(len(self.h.statuses), 1, "the status still comes at 1 Hz")
        status = self.h.statuses[-1]
        self.assertEqual(status["state"], "no_scan")
        self.assertEqual(status["scans_seen"], 0)
        self.assertEqual(status["backend"], "geometric")
        self.assertEqual(status["model_id"], "geometric-v1")

    def test_b_nominal_scan_gives_a_track_with_the_scans_stamp(self):
        self.h.run_scans(at_least=5)
        self.assertGreaterEqual(len(self.h.tracks), 5)
        published = set(self.h.published_ns)
        for msg in self.h.tracks:
            self.assertIn(stamp_ns(msg), published, "the track carries the scan's stamp")
            self.assertEqual(msg.header.frame_id, "base_link")
            self.assertEqual(msg.model_id, "geometric-v1")
            self.assertEqual(msg.backend, msg.BACKEND_GEOMETRIC)
            self.assertAlmostEqual(msg.station_spacing_m, 0.2, places=6)
            for arr in (
                msg.x_m,
                msg.y_m,
                msg.half_width_left_m,
                msg.half_width_right_m,
                msg.sigma_m,
            ):
                self.assertEqual(len(arr), 16)
            self.assertEqual(msg.valid_count, 16)
            for arr in (msg.x_m, msg.y_m, msg.half_width_left_m, msg.half_width_right_m):
                self.assertTrue(all(math.isfinite(v) for v in arr[: msg.valid_count]))
            self.assertLess(max(abs(v) for v in msg.y_m), 0.05)
            self.assertAlmostEqual(msg.half_width_left_m[0], 0.5, delta=0.05)

    def test_c_status_counts_the_scans(self):
        status = self.h.status_while_scanning()
        self.assertEqual(status["state"], "running")
        self.assertGreaterEqual(status["scans_seen"], 5)
        self.assertEqual(status["scans_published"], status["scans_seen"])
        self.assertGreater(status["valid_count_mean"], 15.0)
        self.assertIsNotNone(status["latency_ms"]["p99"])

    def test_d_the_node_never_publishes_the_command_topics(self):
        node = self.h.node
        for topic in ("/drive_raw", "/drive"):
            self.assertEqual(node.get_publishers_info_by_topic(topic), [])
        topics = {
            name
            for name, _types in node.get_publisher_names_and_types_by_node(
                "track_perception_node", "/"
            )
        }
        self.assertEqual(
            topics - {"/rosout", "/parameter_events"},
            {"/pnp/local_track", "/pnp/perception/status"},
        )


@launch_testing.post_shutdown_test()
class TestGeometricNodeShutdown(unittest.TestCase):
    def test_clean_shutdown(self, proc_info):
        launch_testing.asserts.assertExitCodes(proc_info, allowable_exit_codes=[0, -signal.SIGINT])
