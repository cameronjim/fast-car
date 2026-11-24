"""L3: the net backend as a real node with a bundle that declares `fused-v2`
(`io.arch.type: circ_conv1d_v0_fused_v2`, brief 55): silence, nominal scans, and the startup refusal of a
fused bundle without its frozen config (colcon only).

The net of the bundle is constant and 0.2 m off the lane the synthetic scan holds, so the track
the node publishes is right only because the refinement and the fusion ran."""

from __future__ import annotations

import signal
import tempfile
import unittest
from pathlib import Path
from typing import Any

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
from racer_track_perception.bundle import ARCH_TYPE_FUSED_V2
from racer_track_perception.fuse_v2 import FuseV2Config
from racer_track_perception.vehicle import load_live_vehicle
from test_refine_bundle import constant_net

TMP = Path(tempfile.mkdtemp())


def _fused_v2(m: dict[str, Any]) -> None:
    m["io"]["arch"]["type"] = ARCH_TYPE_FUSED_V2
    m["io"]["arch"]["fuse"] = FuseV2Config(edge_end_extension_m=0.1).as_dict()


BUNDLE = write_bundle(
    TMP / "bundle", load_live_vehicle(), _fused_v2, weights=constant_net(TMP / "weights")
)


@pytest.mark.launch_test
def generate_test_description():
    node = LaunchNode(
        package="racer_track_perception",
        executable="track_perception_node",
        name="track_perception_node",
        parameters=[{"backend": "net", "bundle_dir": str(BUNDLE)}],
    )
    return launch.LaunchDescription([node, launch_testing.actions.ReadyToTest()])


class TestFusedV2Node(unittest.TestCase):
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
        self.assertGreaterEqual(len(self.h.statuses), 1)
        status = self.h.statuses[-1]
        self.assertEqual(status["state"], "no_scan")
        self.assertEqual(status["backend"], "net")

    def test_b_nominal_scan_gives_a_fused_track(self):
        self.h.run_scans(at_least=5)
        self.assertGreaterEqual(len(self.h.tracks), 5)
        published = set(self.h.published_ns)
        for msg in self.h.tracks:
            self.assertIn(stamp_ns(msg), published)
            self.assertEqual(msg.header.frame_id, "base_link")
            self.assertEqual(msg.model_id, "trackperc-v0-2026-11-12-tiny")
            self.assertEqual(msg.backend, msg.BACKEND_NET)
            self.assertEqual(len(msg.x_m), 16)
            self.assertGreaterEqual(msg.valid_count, 15)
            # The net answers y = 0.2 m everywhere; only the refinement brings it to the lane.
            self.assertLess(max(abs(v) for v in msg.y_m[: msg.valid_count]), 0.05)
            self.assertAlmostEqual(msg.half_width_left_m[0], 0.5, delta=0.05)

    def test_c_status_counts_fusion_fallbacks(self):
        self.h.run_scans(at_least=3)
        status = self.h.status_while_scanning()
        self.assertEqual(status["fuse_net_unusable"], 0)


@launch_testing.post_shutdown_test()
class TestFusedV2NodeShutdown(unittest.TestCase):
    def test_clean_shutdown(self, proc_info):
        launch_testing.asserts.assertExitCodes(proc_info, allowable_exit_codes=[0, -signal.SIGINT])
