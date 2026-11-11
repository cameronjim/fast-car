"""L3: the net backend as a real node, loading the tiny fixture bundle (colcon only)."""

from __future__ import annotations

import signal
import tempfile
import unittest
from pathlib import Path

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
from bundle_fixture import write_bundle
from launch_ros.actions import Node as LaunchNode
from node_harness import Harness, stamp_ns
from racer_track_perception.vehicle import load_live_vehicle

BUNDLE = write_bundle(Path(tempfile.mkdtemp()) / "bundle", load_live_vehicle())


@pytest.mark.launch_test
def generate_test_description():
    node = LaunchNode(
        package="racer_track_perception",
        executable="track_perception_node",
        name="track_perception_node",
        parameters=[{"backend": "net", "bundle_dir": str(BUNDLE)}],
    )
    return launch.LaunchDescription([node, launch_testing.actions.ReadyToTest()])


class TestNetNode(unittest.TestCase):
    @classmethod
    def setUpClass(cls):
        rclpy.init()
        cls.h = Harness()

    @classmethod
    def tearDownClass(cls):
        cls.h.destroy()
        rclpy.shutdown()

    def test_net_backend_publishes_a_track_with_the_bundles_model_id(self):
        self.h.run_scans(at_least=5)
        self.assertGreaterEqual(len(self.h.tracks), 5)
        published = set(self.h.published_ns)
        for msg in self.h.tracks:
            self.assertIn(stamp_ns(msg), published)
            self.assertEqual(msg.header.frame_id, "base_link")
            self.assertEqual(msg.model_id, "trackperc-v0-2026-11-12-tiny")
            self.assertEqual(msg.backend, msg.BACKEND_NET)
            self.assertEqual(len(msg.x_m), 16)
            self.assertEqual(msg.valid_count, 16)
        status = self.h.status_while_scanning()
        self.assertEqual(status["backend"], "net")
        self.assertEqual(len(status["payload_sha256"]), 64)


@launch_testing.post_shutdown_test()
class TestNetNodeShutdown(unittest.TestCase):
    def test_clean_shutdown(self, proc_info):
        launch_testing.asserts.assertExitCodes(proc_info, allowable_exit_codes=[0, -signal.SIGINT])
