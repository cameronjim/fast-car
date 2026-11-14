"""track_perception_node: /scan in, /pnp/local_track out (plug-and-play part A).

All decisions live in the ROS-free modules (core, bundle, status); this file is plumbing.

Startup order, so a refusal never leaves a publisher behind:
  1. declare parameters (with descriptors and ranges),
  2. load vehicle_params through the generated binding (racer_tools) and refuse if the LiDAR
     mount or the body fields are null,
  3. for backend:=net, load and verify the bundle (any BundleRefused is logged with its specific
     class and the process exits non-zero); for geometric no bundle is read,
  4. only then create the subscription, the two publishers and the status timer.

Publishes /pnp/local_track (one message per scan, with the scan's stamp) and
/pnp/perception/status (JSON, 1 Hz). It never publishes /drive_raw or /drive.

laser_yaw_from_vehicle_params true (the car): the LiDAR yaw is sensors.lidar.mount_yaw_rad; a
non-zero laser_yaw_offset_rad that disagrees refuses to start, as in gap_follow_node. false is for
the simulator and synthetic-scan tests only: laser_yaw_offset_rad is used as given.
"""

from __future__ import annotations

import dataclasses
import math
import sys
import time
from pathlib import Path

import rclpy
from racer_tools.spin_guard import spin_until_shutdown
from rcl_interfaces.msg import FloatingPointRange, ParameterDescriptor
from rclpy.node import Node
from rclpy.qos import HistoryPolicy, QoSProfile, ReliabilityPolicy
from sensor_msgs.msg import LaserScan
from std_msgs.msg import String

from racer_track_perception import core, geometric
from racer_track_perception import track as tr
from racer_track_perception.bundle import KIND, load_bundle
from racer_track_perception.errors import BundleRefused
from racer_track_perception.runtime import CircConv1dV0, ModelFormatError
from racer_track_perception.status import PerceptionStats
from racer_track_perception.vehicle import LiveVehicle, VehicleParamsNull, from_generated

LOCAL_TRACK_TOPIC = "/pnp/local_track"
STATUS_TOPIC = "/pnp/perception/status"
SCAN_TOPIC = "/scan"
FRAME_ID = "base_link"
EXIT_REFUSED = 2


class StartupRefused(Exception):
    """The node refuses to start (bundle refusal, null vehicle_params, bad parameters)."""


def _ranged(low: float, high: float, step: float, description: str) -> ParameterDescriptor:
    return ParameterDescriptor(
        description=description,
        floating_point_range=[FloatingPointRange(from_value=low, to_value=high, step=step)],
    )


class TrackPerceptionNode(Node):
    def __init__(self) -> None:
        super().__init__("track_perception_node")
        backend = self.declare_parameter(
            "backend",
            core.BACKEND_NET,
            ParameterDescriptor(description="net (numpy forward pass of a bundle) or geometric"),
        ).value
        bundle_dir = self.declare_parameter(
            "bundle_dir",
            "",
            ParameterDescriptor(description="model bundle directory, required for backend net"),
        ).value
        sigma_max_m = self.declare_parameter(
            "sigma_max_m",
            0.25,
            _ranged(0.01, 2.0, 0.0, "stations with sigma at or above this are not valid (m)"),
        ).value
        default_half_width_m = self.declare_parameter(
            "default_half_width_m",
            0.5,
            _ranged(0.1, 1.0, 0.0, "geometric: half width used before any is measured (m)"),
        ).value
        hw_min_m = self.declare_parameter(
            "lane_half_width_min_m",
            geometric.DEFAULT_HW_MIN_M,
            _ranged(0.05, 1.0, 0.0, "geometric: smallest plausible lane half width (m)"),
        ).value
        hw_max_m = self.declare_parameter(
            "lane_half_width_max_m",
            geometric.DEFAULT_HW_MAX_M,
            _ranged(0.2, 2.0, 0.0, "geometric: largest plausible lane half width (m)"),
        ).value
        yaw_from_vp = self.declare_parameter(
            "laser_yaw_from_vehicle_params",
            True,
            ParameterDescriptor(
                description="true (the car): LiDAR yaw from vehicle_params. false: simulator and "
                "synthetic-scan tests only, use laser_yaw_offset_rad as given"
            ),
        ).value
        yaw_offset = self.declare_parameter(
            "laser_yaw_offset_rad",
            0.0,
            _ranged(
                -math.pi, math.pi, 0.0, "LiDAR mounting yaw (rad), used when the above is false"
            ),
        ).value

        if backend not in (core.BACKEND_NET, core.BACKEND_GEOMETRIC):
            raise StartupRefused(f"backend must be 'net' or 'geometric', got {backend!r}")
        live = self._load_vehicle()
        mount = live.mount
        if yaw_from_vp:
            if yaw_offset != 0.0 and abs(yaw_offset - mount.yaw_rad) > 1e-6:
                raise StartupRefused(
                    f"laser_yaw_offset_rad {yaw_offset} disagrees with vehicle_params "
                    f"sensors.lidar.mount_yaw_rad {mount.yaw_rad} while "
                    "laser_yaw_from_vehicle_params is true"
                )
            working = live
        else:
            self.get_logger().info(
                f"laser_yaw_from_vehicle_params is false: using laser_yaw_offset_rad = "
                f"{yaw_offset:.6f} (simulator or synthetic-scan fixture only)"
            )
            working = dataclasses.replace(
                live, mount=dataclasses.replace(mount, yaw_rad=yaw_offset)
            )

        net = None
        model_id = core.GEOMETRIC_MODEL_ID
        payload_sha: str | None = None
        if backend == core.BACKEND_NET:
            if not bundle_dir:
                raise StartupRefused("backend net needs the bundle_dir parameter")
            try:
                # The bundle is checked against the LIVE vehicle_params, never the sim override.
                bundle = load_bundle(Path(bundle_dir), KIND, live)
                net = CircConv1dV0.from_npz(bundle.payload_path("weights.npz"))
            except (BundleRefused, ModelFormatError) as exc:
                raise StartupRefused(f"{type(exc).__name__}: {exc}") from exc
            model_id = bundle.model_id
            payload_sha = bundle.weights_sha256

        self._perceiver = core.TrackPerceiver(
            backend,
            working,
            sigma_max_m=float(sigma_max_m),
            default_half_width_m=float(default_half_width_m),
            hw_min_m=float(hw_min_m),
            hw_max_m=float(hw_max_m),
            net=net,
            model_id=model_id,
        )
        self._backend_code = 0 if backend == core.BACKEND_NET else 1
        self._stats = PerceptionStats(backend, model_id, payload_sha)

        # Publishers and the subscription are created only now: a refusal above leaves none.
        reliable = lambda depth: QoSProfile(
            reliability=ReliabilityPolicy.RELIABLE, history=HistoryPolicy.KEEP_LAST, depth=depth
        )
        from racer_pnp_msgs.msg import LocalTrack

        self._msg_type = LocalTrack
        self._track_pub = self.create_publisher(LocalTrack, LOCAL_TRACK_TOPIC, reliable(5))
        self._status_pub = self.create_publisher(String, STATUS_TOPIC, reliable(10))
        self.create_subscription(
            LaserScan,
            SCAN_TOPIC,
            self._on_scan,
            QoSProfile(
                reliability=ReliabilityPolicy.BEST_EFFORT, history=HistoryPolicy.KEEP_LAST, depth=10
            ),
        )
        self.create_timer(1.0, self._on_status_timer)
        self.get_logger().info(
            f"track_perception_node: backend {backend}, model_id {model_id}, "
            f"payload sha256 {payload_sha}"
        )

    def _load_vehicle(self) -> LiveVehicle:
        try:
            from racer_tools.vehicle_params_loader import load_vehicle_params

            return from_generated(load_vehicle_params())
        except VehicleParamsNull as exc:
            raise StartupRefused(str(exc)) from exc

    def _now_s(self) -> float:
        return self.get_clock().now().nanoseconds * 1e-9

    def _on_scan(self, scan: LaserScan) -> None:
        self._stats.scan_seen(self._now_s())
        t0 = time.perf_counter()
        track = self._perceiver.process(
            scan.ranges, scan.angle_min, scan.angle_increment, scan.range_min, scan.range_max
        )
        msg = self._to_msg(scan, track)
        latency_ms = (time.perf_counter() - t0) * 1000.0
        self._track_pub.publish(msg)
        self._stats.scan_published(track, latency_ms)

    def _to_msg(self, scan: LaserScan, track: tr.LocalTrackData):  # type: ignore[no-untyped-def]
        msg = self._msg_type()
        msg.header.stamp = scan.header.stamp
        msg.header.frame_id = FRAME_ID
        msg.model_id = self._perceiver.model_id
        msg.backend = self._backend_code
        msg.station_spacing_m = tr.STATION_SPACING_M
        msg.x_m = track.x_m.tolist()
        msg.y_m = track.y_m.tolist()
        msg.half_width_left_m = track.half_width_left_m.tolist()
        msg.half_width_right_m = track.half_width_right_m.tolist()
        msg.sigma_m = track.sigma_m.tolist()
        msg.valid_count = int(track.valid_count)
        msg.boundary_residual_m = float(track.boundary_residual_m)
        msg.intrusion_fraction = float(track.intrusion_fraction)
        return msg

    def _on_status_timer(self) -> None:
        out = String()
        out.data = self._stats.as_json(self._now_s())
        self._status_pub.publish(out)


def main(args: list[str] | None = None) -> None:
    rclpy.init(args=args)
    node = None
    try:
        node = TrackPerceptionNode()
        spin_until_shutdown(node)
    except StartupRefused as exc:
        # Printed, not only logged: the log may be gone once the process exits non-zero.
        print(f"track_perception_node: refusing to start: {exc}", file=sys.stderr, flush=True)
        if node is not None:
            node.destroy_node()
        rclpy.try_shutdown()
        sys.exit(EXIT_REFUSED)
    except KeyboardInterrupt:
        pass
    finally:
        if node is not None and rclpy.ok():
            node.destroy_node()
        rclpy.try_shutdown()


if __name__ == "__main__":
    main()
