"""fake_detector_node: synthetic people on /perception/objects, for the sim (roadmap 5.3).

The ROS sim has no camera. This node stands in for camera + detector + fusion_node: it places
people at fixed MAP-frame positions, watches the car's pose, and publishes a
racer_msgs/ObjectDetectionArray at `rate_hz` exactly as fusion_node would (one array per frame,
empty ones included, vehicle-frame bearing, range, COCO keypoints). A person is visible when
they are inside the camera's horizontal field of view and `max_range_m`, so as the car drives
towards them they come into view, and as the spiderbot turns to face them the bearing really
goes to zero: the loop the state machine runs in is closed.

`people` is a JSON list, each {"x": m, "y": m, "pose": one of synthetic.POSES, "appear_s":
optional, "disappear_s": optional} (times are seconds after the node's first pose message).
No LiDAR return is simulated for them (they stand off the track): range comes from geometry.

Never ships on the car: it is a test and demo tool, started only by the sim launch files.
"""

from __future__ import annotations

import json
import math
import sys

import rclpy
from nav_msgs.msg import Odometry
from racer_msgs.msg import Keypoint2D, ObjectDetection, ObjectDetectionArray
from rcl_interfaces.msg import ParameterDescriptor
from rclpy.logging import get_logger
from rclpy.node import Node
from rclpy.qos import HistoryPolicy, QoSProfile

from racer_apps.spiderbot.synthetic import POSES, synthetic_keypoints


def visible_bearing_range(
    car_x: float,
    car_y: float,
    car_yaw: float,
    px: float,
    py: float,
    hfov_rad: float,
    max_range_m: float,
) -> tuple[float, float] | None:
    """(bearing, range) of a point from the car, or None if out of view. Left positive."""
    dx, dy = px - car_x, py - car_y
    rng = math.hypot(dx, dy)
    if rng > max_range_m or rng < 1e-6:
        return None
    bearing = math.atan2(
        math.sin(math.atan2(dy, dx) - car_yaw), math.cos(math.atan2(dy, dx) - car_yaw)
    )
    if abs(bearing) > hfov_rad / 2.0:
        return None
    return bearing, rng


class FakeDetectorNode(Node):
    def __init__(self, **node_kwargs) -> None:
        super().__init__("fake_detector_node", **node_kwargs)

        def declare(name, default, description):
            self.declare_parameter(name, default, ParameterDescriptor(description=description))
            return self.get_parameter(name).value

        self._people = json.loads(str(declare("people", "[]", "JSON list of people (see doc).")))
        for person in self._people:
            if person.get("pose", "standing") not in POSES:
                raise ValueError(f"pose must be one of {POSES}, got {person.get('pose')!r}")
        pose_topic = str(declare("pose_topic", "/odom", "nav_msgs/Odometry in the map frame."))
        out_topic = str(declare("objects_topic", "/perception/objects", "Output."))
        self._hfov = float(declare("hfov_rad", 1.48, "Camera horizontal field of view."))
        self._max_range = float(declare("max_range_m", 6.0, "Farthest visible person."))
        self._width = int(declare("image_width_px", 1280, "Virtual image width."))
        rate_hz = float(declare("rate_hz", 10.0, "Frames per second."))
        self._pose: tuple[float, float, float] | None = None
        self._t0: float | None = None
        qos = QoSProfile(history=HistoryPolicy.KEEP_LAST, depth=10)
        self._pub = self.create_publisher(ObjectDetectionArray, out_topic, qos)
        self.create_subscription(Odometry, pose_topic, self._on_pose, qos)
        self.create_timer(1.0 / rate_hz, self._tick)

    def _on_pose(self, msg: Odometry) -> None:
        q = msg.pose.pose.orientation
        yaw = math.atan2(2.0 * (q.w * q.z + q.x * q.y), 1.0 - 2.0 * (q.y * q.y + q.z * q.z))
        self._pose = (msg.pose.pose.position.x, msg.pose.pose.position.y, yaw)
        if self._t0 is None:
            self._t0 = self.get_clock().now().nanoseconds * 1e-9

    def _tick(self) -> None:
        if self._pose is None or self._t0 is None:
            return
        now_s = self.get_clock().now().nanoseconds * 1e-9 - self._t0
        out = ObjectDetectionArray()
        out.header.stamp = self.get_clock().now().to_msg()
        out.header.frame_id = "base_link"
        focal = (self._width / 2.0) / math.tan(self._hfov / 2.0)
        for person in self._people:
            if now_s < float(person.get("appear_s", 0.0)):
                continue
            if now_s > float(person.get("disappear_s", math.inf)):
                continue
            seen = visible_bearing_range(
                *self._pose,
                float(person["x"]),
                float(person["y"]),
                self._hfov,
                self._max_range,
            )
            if seen is None:
                continue
            bearing, rng = seen
            height_px = min(0.9 * focal / rng, 700.0)
            cx = self._width / 2.0 - focal * math.tan(bearing)
            top = 360.0 - height_px / 2.0
            det = ObjectDetection()
            det.header = out.header
            det.class_name = "person"
            det.score = 0.9
            det.bearing_rad = float(bearing)
            det.bearing_half_width_rad = 0.1
            det.range_m = float(rng)
            det.range_valid = True
            det.range_status = "ok"
            det.range_samples = 10
            det.box_cx_px, det.box_cy_px = float(cx), 360.0
            det.box_width_px, det.box_height_px = float(height_px * 0.4), float(height_px)
            det.keypoint_layout = "coco17"
            for k in synthetic_keypoints(person.get("pose", "standing"), cx, top, height_px):
                det.keypoints.append(Keypoint2D(x_px=k.x_px, y_px=k.y_px, confidence=k.confidence))
            out.detections.append(det)
        self._pub.publish(out)


def main(args: list | None = None) -> None:
    rclpy.init(args=args)
    node = None
    code = 0
    try:
        node = FakeDetectorNode()
        rclpy.spin(node)
    except (ValueError, json.JSONDecodeError) as exc:
        get_logger("fake_detector_node").error(str(exc))
        code = 2
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
