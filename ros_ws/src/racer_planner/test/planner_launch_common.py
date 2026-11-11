"""Shared helpers for racer_planner's L3 launch tests (not a test file).

Builds dynamics bundles from the LIVE config/vehicle_params.yaml at test time with the fixture
generator (a second implementation of the model), so these tests do not move when an unrelated
vehicle_params field does, and publishes a synthetic straight corridor on /scan,
/pnp/local_track and /odom/wheel the way the real nodes would.
"""

from __future__ import annotations

import importlib.util
import math
import shutil
import tempfile
import time
from pathlib import Path

import numpy as np
import rclpy
from nav_msgs.msg import Odometry
from racer_pnp_msgs.msg import LocalTrack
from rclpy.qos import HistoryPolicy, QoSProfile, ReliabilityPolicy
from sensor_msgs.msg import LaserScan

HERE = Path(__file__).resolve().parent
REPO_ROOT = HERE.parents[3]
_spec = importlib.util.spec_from_file_location(
    "make_fixture", HERE / "fixtures" / "make_fixture.py"
)
make_fixture = importlib.util.module_from_spec(_spec)
_spec.loader.exec_module(make_fixture)

VEHICLE = make_fixture.vehicle_numbers(
    make_fixture.read_vehicle_params(REPO_ROOT / "config" / "vehicle_params.yaml")
)
V_MAX_VALIDITY = 0.9
LANE_HALF_WIDTH_M = 0.5


def make_bundle(root: Path | None = None, name: str = "good") -> Path:
    base = Path(root) if root is not None else Path(tempfile.mkdtemp(prefix="racer_planner_l3_"))
    out = base / name
    make_fixture.make_bundle(
        out,
        VEHICLE,
        residual_on=False,
        nominal=make_fixture.NOMINAL,
        v_max=V_MAX_VALIDITY,
        model_id="dyn-l3-v0-2026-11-12",
    )
    return out


def make_bad_checksum_bundle(root: Path) -> Path:
    out = make_bundle(root, "bad_checksum")
    with open(out / "residual_mlp.f32", "ab") as handle:
        handle.write(b"\x00")
    return out


def make_vehicle_mismatch_bundle(root: Path) -> Path:
    out = make_bundle(root, "vehicle_mismatch")
    manifest = out / "manifest.yaml"
    text = manifest.read_text(encoding="utf-8")
    manifest.write_text(
        text.replace(f'schema_version: "{VEHICLE["schema_version"]}"', 'schema_version: "0.0.1"'),
        encoding="utf-8",
    )
    return out


def remove_tree(path: Path) -> None:
    shutil.rmtree(path, ignore_errors=True)


def reliable(depth: int) -> QoSProfile:
    return QoSProfile(
        reliability=ReliabilityPolicy.RELIABLE, history=HistoryPolicy.KEEP_LAST, depth=depth
    )


def best_effort(depth: int = 10) -> QoSProfile:
    return QoSProfile(
        reliability=ReliabilityPolicy.BEST_EFFORT, history=HistoryPolicy.KEEP_LAST, depth=depth
    )


def corridor_scan(stamp) -> LaserScan:
    """720 beams of a 1 m wide infinite corridor seen from the LiDAR head, vehicle aligned.

    The planner runs with laser_yaw_from_vehicle_params:=false and laser_yaw_offset_rad:=0, so a
    beam's laser bearing is its vehicle bearing. The head sits at (mount_x, mount_y) on the car.
    """
    msg = LaserScan()
    msg.header.stamp = stamp
    msg.header.frame_id = "laser"
    n = 720
    msg.angle_min = -math.pi
    msg.angle_increment = 2.0 * math.pi / n
    msg.angle_max = msg.angle_min + (n - 1) * msg.angle_increment
    msg.range_min = 0.05
    msg.range_max = 12.0
    b = msg.angle_min + np.arange(n) * msg.angle_increment
    s = np.sin(b)
    ly = VEHICLE["lidar_mount_y_m"]
    with np.errstate(divide="ignore"):
        r = np.where(
            s > 1e-6,
            (LANE_HALF_WIDTH_M - ly) / s,
            np.where(s < -1e-6, (-LANE_HALF_WIDTH_M - ly) / s, np.inf),
        )
    r = np.where(r > msg.range_max, np.inf, r)
    msg.ranges = [float(x) for x in r]
    return msg


def straight_track(stamp, valid_count: int = 16) -> LocalTrack:
    msg = LocalTrack()
    msg.header.stamp = stamp
    msg.header.frame_id = "base_link"
    msg.model_id = "geometric-v0"
    msg.backend = LocalTrack.BACKEND_GEOMETRIC
    msg.station_spacing_m = 0.2
    msg.x_m = [0.2 * k for k in range(16)]
    msg.y_m = [0.0] * 16
    msg.half_width_left_m = [LANE_HALF_WIDTH_M] * 16
    msg.half_width_right_m = [LANE_HALF_WIDTH_M] * 16
    msg.sigma_m = [0.02] * 16
    msg.valid_count = valid_count
    msg.boundary_residual_m = 0.02
    msg.intrusion_fraction = 0.0
    return msg


class Feeder:
    """Publishes the synthetic scene. Each input can be silenced independently."""

    def __init__(self, node, speed_mps: float = 0.3):
        self.node = node
        self.speed = speed_mps
        self.scan_on = True
        self.track_on = True
        self.odom_on = True
        self.track_valid_count = 16
        self.scan_pub = node.create_publisher(LaserScan, "/scan", best_effort(10))
        self.track_pub = node.create_publisher(LocalTrack, "/pnp/local_track", reliable(5))
        self.odom_pub = node.create_publisher(Odometry, "/odom/wheel", best_effort(10))
        self._last_scan = 0.0
        self._last_odom = 0.0

    def pump(self, seconds: float, on_spin=None) -> None:
        end = time.time() + seconds
        while time.time() < end:
            now = time.time()
            stamp = self.node.get_clock().now().to_msg()
            if now - self._last_scan >= 0.1:
                self._last_scan = now
                if self.scan_on:
                    self.scan_pub.publish(corridor_scan(stamp))
                if self.track_on:
                    self.track_pub.publish(straight_track(stamp, self.track_valid_count))
            if now - self._last_odom >= 0.02:
                self._last_odom = now
                if self.odom_on:
                    odom = Odometry()
                    odom.header.stamp = stamp
                    odom.twist.twist.linear.x = self.speed
                    self.odom_pub.publish(odom)
            rclpy.spin_once(self.node, timeout_sec=0.005)
            if on_spin is not None:
                on_spin()
