"""fusion_node: detections + LiDAR scan -> objects with bearing and range (roadmap 5.1).

    ros2 launch racer_perception perception.launch.py        (starts this next to the detector)

Subscribes vision_msgs/Detection2DArray on /perception/detections and sensor_msgs/LaserScan
on /scan. For every detection it computes the vehicle-frame bearing of the box centre from a
pinhole camera model and a robust LiDAR range inside the box's angular window (the maths and
every edge case are in racer_perception/fusion.py), and publishes racer_msgs/
ObjectDetectionArray on /perception/objects, one array per detection array (empty arrays too).

A missing, stale or empty scan never drops a detection: it is published with range_m NaN,
range_valid false and a range_status saying why (no_scan, stale_scan, no_returns).

It also runs the nearest-person selection (person_tracker.py): the chosen racer_msgs/
ObjectDetection goes out on /perception/target, held over for `target_hold_s` after the last
sighting, and NOTHING is published when there is no target. A consumer must check the stamp.

LIDAR MOUNT YAW comes from config/vehicle_params.yaml sensors.lidar.mount_yaw_rad through the
generated binding (CLAUDE.md invariant 2), the same rule as racer_safety and racer_control:
`laser_yaw_rad` is 0 by default; if given it must equal the binding or the node refuses to
start; `yaw_from_vehicle_params:=false` ignores the binding, ONLY for the simulator's
vehicle-aligned /scan.

The CAMERA is a parameter set (`image_width_px`, `hfov_rad`, `camera_yaw_rad`); no camera mount
is measured yet. image_width_px must be the width of the images the detector saw.

QoS: /scan best_effort, KeepLast(5) (sensor data); detections reliable, KeepLast(10) to match
detector_node's publisher; outputs reliable, KeepLast(10).
"""

from __future__ import annotations

import math
import sys

import rclpy
from racer_msgs.msg import ObjectDetection, ObjectDetectionArray
from rcl_interfaces.msg import FloatingPointRange, ParameterDescriptor
from rclpy.logging import get_logger
from rclpy.node import Node
from rclpy.qos import HistoryPolicy, QoSProfile, ReliabilityPolicy
from sensor_msgs.msg import LaserScan
from vision_msgs.msg import Detection2DArray

from racer_perception.fusion import (
    CameraModel,
    FusedObject,
    RawDetection,
    ScanView,
    fuse_detection,
    resolve_laser_yaw_rad,
)
from racer_perception.person_tracker import PersonTracker

EXIT_BAD_CONFIG = 2
BASE_FRAME = "base_link"


def stamp_to_s(stamp) -> float:
    return stamp.sec + stamp.nanosec * 1e-9


def s_to_stamp(stamp, seconds: float) -> None:
    whole = math.floor(seconds)
    stamp.sec = int(whole)
    stamp.nanosec = round((seconds - whole) * 1e9) % 1_000_000_000


def to_raw_detection(det) -> RawDetection:
    best = max(det.results, key=lambda r: r.hypothesis.score, default=None)
    return RawDetection(
        class_name=best.hypothesis.class_id if best is not None else "unknown",
        score=float(best.hypothesis.score) if best is not None else 0.0,
        cx_px=float(det.bbox.center.position.x),
        cy_px=float(det.bbox.center.position.y),
        width_px=float(det.bbox.size_x),
        height_px=float(det.bbox.size_y),
    )


def to_object_msg(obj: FusedObject) -> ObjectDetection:
    msg = ObjectDetection()
    s_to_stamp(msg.header.stamp, obj.stamp_s)
    msg.header.frame_id = BASE_FRAME
    msg.class_name = obj.class_name
    msg.score = float(obj.score)
    msg.bearing_rad = float(obj.bearing_rad)
    msg.bearing_half_width_rad = float(obj.bearing_half_width_rad)
    msg.range_m = float(obj.range_m)
    msg.range_valid = obj.range_valid
    msg.range_status = obj.range_status
    msg.range_samples = int(obj.range_samples)
    msg.box_cx_px = float(obj.cx_px)
    msg.box_cy_px = float(obj.cy_px)
    msg.box_width_px = float(obj.width_px)
    msg.box_height_px = float(obj.height_px)
    return msg


class FusionNode(Node):
    def __init__(self, **node_kwargs) -> None:
        super().__init__("fusion_node", **node_kwargs)

        def declare(name, default, description, **kw):
            self.declare_parameter(
                name, default, ParameterDescriptor(description=description, **kw)
            )

        declare("detections_topic", "/perception/detections", "Detection2DArray input.")
        declare("scan_topic", "/scan", "LaserScan input.")
        declare("objects_topic", "/perception/objects", "ObjectDetectionArray output.")
        declare("target_topic", "/perception/target", "ObjectDetection output (nearest person).")
        declare("image_width_px", 1280, "Width of the images the detector processed, pixels.")
        declare(
            "hfov_rad",
            1.48,
            "Camera horizontal field of view, rad (ELP AR0234 lens, about 85 degrees).",
            floating_point_range=[FloatingPointRange(from_value=0.1, to_value=3.0, step=0.0)],
        )
        declare("camera_yaw_rad", 0.0, "Camera yaw relative to base_link, rad, left positive.")
        declare(
            "laser_yaw_rad",
            0.0,
            "LiDAR yaw override. 0 = use vehicle_params; else it must equal the binding.",
        )
        declare("yaw_from_vehicle_params", True, "False ONLY for the simulator's /scan.")
        declare("max_scan_age_s", 0.5, "|image stamp - scan stamp| above this is a stale scan.")
        declare(
            "closest_fraction",
            0.3,
            "Range = median of this closest fraction of the valid returns in the window.",
            floating_point_range=[FloatingPointRange(from_value=0.01, to_value=1.0, step=0.0)],
        )
        declare("target_class", "person", "Class chosen for /perception/target.")
        declare(
            "target_hold_s", 1.0, "Keep publishing the last target this long after it vanished."
        )
        declare("target_min_score", 0.3, "Minimum detector score for a target candidate.")

        def get(name):
            return self.get_parameter(name).value

        binding = None
        if bool(get("yaw_from_vehicle_params")):
            from racer_tools.vehicle_params_loader import load_vehicle_params

            binding = load_vehicle_params().sensors.lidar.mount_yaw_rad
        yaw = resolve_laser_yaw_rad(
            binding, float(get("laser_yaw_rad")), bool(get("yaw_from_vehicle_params"))
        )
        if yaw is None:
            raise ValueError(
                f"laser_yaw_rad={get('laser_yaw_rad')} disagrees with "
                f"vehicle_params sensors.lidar.mount_yaw_rad={binding}: leave it at 0 to use "
                "the vehicle_params value (CLAUDE.md invariant 2)"
            )
        self._laser_yaw = yaw
        self._camera = CameraModel(
            int(get("image_width_px")), float(get("hfov_rad")), float(get("camera_yaw_rad"))
        )
        self._max_scan_age_s = float(get("max_scan_age_s"))
        self._closest_fraction = float(get("closest_fraction"))
        self._tracker = PersonTracker(
            str(get("target_class")), float(get("target_hold_s")), float(get("target_min_score"))
        )
        self._scan: ScanView | None = None

        scan_qos = QoSProfile(
            history=HistoryPolicy.KEEP_LAST, depth=5, reliability=ReliabilityPolicy.BEST_EFFORT
        )
        rel_qos = QoSProfile(
            history=HistoryPolicy.KEEP_LAST, depth=10, reliability=ReliabilityPolicy.RELIABLE
        )
        self._objects_pub = self.create_publisher(
            ObjectDetectionArray, str(get("objects_topic")), rel_qos
        )
        self._target_pub = self.create_publisher(ObjectDetection, str(get("target_topic")), rel_qos)
        self.create_subscription(LaserScan, str(get("scan_topic")), self._on_scan, scan_qos)
        self.create_subscription(
            Detection2DArray, str(get("detections_topic")), self._on_detections, rel_qos
        )
        self.get_logger().info(
            f"fusion up: lidar yaw {self._laser_yaw:.4f} rad, camera {self._camera.image_width_px} "
            f"px / {self._camera.hfov_rad:.3f} rad hfov / yaw {self._camera.yaw_rad:.3f} rad"
        )

    def _on_scan(self, msg: LaserScan) -> None:
        self._scan = ScanView(
            stamp_s=stamp_to_s(msg.header.stamp),
            angle_min_rad=msg.angle_min,
            angle_increment_rad=msg.angle_increment,
            range_min_m=msg.range_min,
            range_max_m=msg.range_max,
            ranges=tuple(msg.ranges),
            laser_yaw_rad=self._laser_yaw,
        )

    def _on_detections(self, msg: Detection2DArray) -> None:
        stamp_s = stamp_to_s(msg.header.stamp)
        fused = [
            fuse_detection(
                to_raw_detection(det),
                stamp_s,
                self._camera,
                self._scan,
                self._max_scan_age_s,
                self._closest_fraction,
            )
            for det in msg.detections
        ]
        out = ObjectDetectionArray()
        out.header = msg.header
        out.header.frame_id = BASE_FRAME
        out.detections = [to_object_msg(o) for o in fused]
        self._objects_pub.publish(out)

        now_s = self.get_clock().now().nanoseconds * 1e-9
        target, _held = self._tracker.update(fused, now_s)
        if target is not None:
            self._target_pub.publish(to_object_msg(target))


def main(args: list | None = None) -> None:
    rclpy.init(args=args)
    node: FusionNode | None = None
    code = 0
    try:
        node = FusionNode()
        rclpy.spin(node)
    except ValueError as exc:
        get_logger("fusion_node").error(str(exc))
        code = EXIT_BAD_CONFIG
    except KeyboardInterrupt:
        pass
    finally:
        if node is not None:
            node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()
    sys.exit(code)


if __name__ == "__main__":
    main()
