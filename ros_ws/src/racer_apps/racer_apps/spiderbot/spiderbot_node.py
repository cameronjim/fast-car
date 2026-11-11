"""spiderbot_node: patrol, and on a pose rule stop, face the person, alert, resume (roadmap 5.3).

    ros2 launch racer_apps spiderbot.launch.py patrol:=<patrol.yaml>     (the normal way)

OFF BY DEFAULT: nothing starts this but its own launch file. It is not in car_teleop.launch.py.

WHAT IT SITS BETWEEN
  /spiderbot/patrol_cmd   AckermannDriveStamped from racer_control's tracker_node following the
                          waypoint loop (the launch file remaps the tracker's /drive_raw here)
  /perception/objects     racer_msgs/ObjectDetectionArray from fusion_node (pose mode): one array
                          per camera frame, empty ones included, people with keypoints
  /odom/wheel             optional, wheel speed, to know when the car has stopped
  camera image            optional CompressedImage, for the snapshot
  TF map -> base_link     optional, where the car was when it alerted
and what it publishes
  /drive_raw              the ONLY command output. Never /drive: racer_safety's safety_node is the
                          sole publisher of /drive and gates this topic (CLAUDE.md invariant 1,
                          claude-docs/05-safety.md). Published at `control_rate_hz` (50) while
                          running, rate limited below the safety gate's bounds.
  /spiderbot/alert        racer_msgs/SpiderbotAlert, once per engagement
  /spiderbot/state        std_msgs/String, the phase name, 10 Hz
plus an event log and snapshot JPEGs under `output_dir`.

The decisions are in racer_apps/spiderbot/state_machine.py and pose_rules.py (pure, unit
tested); this file is the plumbing. Silence rules: if the patrol command goes quiet (tracker
died, pose lost) while patrolling this node publishes NOTHING, so safety_node's own watchdog
brakes, the same policy as tracker_node. While stopped it publishes zero speed.

Vehicle numbers (steering limit, acceleration, rate, speed full scale) come from
config/vehicle_params.yaml through racer_tools' loader, never typed here.

QoS: commands and alerts reliable, KeepLast(10); camera and objects as their publishers.
"""

from __future__ import annotations

import json
import math
import sys
import time

import rclpy
from ackermann_msgs.msg import AckermannDriveStamped
from nav_msgs.msg import Odometry
from racer_msgs.msg import ObjectDetectionArray, SpiderbotAlert
from rcl_interfaces.msg import ParameterDescriptor
from rclpy.logging import get_logger
from rclpy.node import Node
from rclpy.qos import HistoryPolicy, QoSProfile, ReliabilityPolicy
from sensor_msgs.msg import CompressedImage
from std_msgs.msg import String

from racer_apps.spiderbot.events import EventLog
from racer_apps.spiderbot.motion import RateLimiter, face_command, limits_from_vehicle
from racer_apps.spiderbot.pose_rules import (
    HeldForFrames,
    Person,
    RuleSettings,
    build_rule,
    choose_person,
)
from racer_apps.spiderbot.state_machine import (
    CommandKind,
    EventKind,
    MachineConfig,
    Observation,
    Phase,
    SpiderbotMachine,
)

EXIT_BAD_CONFIG = 2
_SPEED_MAX_AGE_S = 0.5


def people_from_objects(msg: ObjectDetectionArray, person_class: str = "person") -> list[Person]:
    """The people in one fused frame, as the rules read them."""
    out = []
    for det in msg.detections:
        if det.class_name != person_class:
            continue
        out.append(
            Person(
                score=float(det.score),
                bearing_rad=float(det.bearing_rad),
                range_m=float(det.range_m),
                range_valid=bool(det.range_valid),
                box_height_px=float(det.box_height_px),
                keypoints=tuple(_kp(k) for k in det.keypoints),
            )
        )
    return out


def _kp(k):
    from racer_perception.keypoints import Keypoint

    return Keypoint(float(k.x_px), float(k.y_px), float(k.confidence))


class SpiderbotNode(Node):
    def __init__(self, **node_kwargs) -> None:
        super().__init__("spiderbot_node", **node_kwargs)

        def declare(name, default, description):
            self.declare_parameter(name, default, ParameterDescriptor(description=description))
            return self.get_parameter(name).value

        rule_name = declare("rule", "arms_raised", "Pose rule: a built-in name or pkg.mod:func.")
        rule_frames = int(declare("rule_frames", 3, "Consecutive frames the rule must hold."))
        rule_gap = float(declare("rule_max_gap_s", 1.0, "Gap between frames that resets it."))
        settings = RuleSettings(
            min_score=float(declare("rule_min_score", 0.5, "Minimum person score.")),
            keypoint_confidence=float(declare("rule_keypoint_confidence", 0.5, "Joint conf.")),
            raise_margin=float(declare("rule_raise_margin", 0.5, "Wrist above shoulder, scales.")),
            max_range_m=(
                float(declare("rule_max_range_m", 0.0, "person_present max range; 0 = none"))
                or math.nan
            ),
        )
        self._machine_config = MachineConfig(
            stop_settle_s=float(declare("stop_settle_s", 1.0, "Longest wait for the car to stop.")),
            face_enabled=bool(declare("face_enabled", True, "Turn to face the person.")),
            face_tolerance_rad=float(declare("face_tolerance_rad", 0.12, "Facing tolerance.")),
            face_lost_s=float(declare("face_lost_s", 1.0, "Give up facing after this unseen.")),
            face_timeout_s=float(declare("face_timeout_s", 5.0, "Longest facing manoeuvre.")),
            min_face_range_m=float(declare("min_face_range_m", 1.0, "No creeping closer than.")),
            hold_s=float(declare("hold_s", 10.0, "Stopped this long after the alert.")),
            cooldown_s=float(declare("cooldown_s", 20.0, "Triggers ignored after resuming.")),
            require_perception=bool(declare("require_perception", True, "Stop if blind.")),
        )
        self._face_speed = float(declare("face_speed_mps", 0.3, "Creep speed while facing."))
        self._face_gain = float(declare("face_gain", 1.5, "Steering per radian of bearing."))
        self._perception_timeout_s = float(
            declare("perception_timeout_s", 2.0, "Detector silence.")
        )
        self._patrol_timeout_s = float(declare("patrol_cmd_timeout_s", 0.25, "Tracker silence."))
        rate_hz = float(declare("control_rate_hz", 50.0, "Command rate."))
        output_dir = str(declare("output_dir", "~/spiderbot", "Event log and snapshots."))
        objects_topic = str(declare("objects_topic", "/perception/objects", "Fused objects."))
        image_topic = str(declare("image_topic", "/camera/usb/image_raw/compressed", "Snapshot."))
        patrol_topic = str(declare("patrol_cmd_topic", "/spiderbot/patrol_cmd", "Tracker output."))
        drive_topic = str(declare("drive_topic", "/drive_raw", "Command output (never /drive)."))
        speed_topic = str(declare("speed_topic", "/odom/wheel", "Odometry with the wheel speed."))
        self._map_frame = str(declare("map_frame", "map", "Frame for the alert pose."))
        self._base_frame = str(declare("base_frame", "base_link", "Car frame."))
        use_vehicle = bool(declare("limits_from_vehicle_params", True, "False only in tests."))

        if drive_topic.rstrip("/") == "/drive":
            raise ValueError("drive_topic must not be /drive: only safety_node publishes it")
        if not (math.isfinite(rate_hz) and rate_hz > 0.0):
            raise ValueError(f"control_rate_hz must be positive, got {rate_hz}")
        if not (math.isfinite(self._face_speed) and self._face_speed >= 0.0):
            raise ValueError(f"face_speed_mps must be >= 0, got {self._face_speed}")

        self._rule = HeldForFrames(build_rule(str(rule_name), settings), rule_frames, rule_gap)
        self._machine = SpiderbotMachine(self._machine_config)
        if use_vehicle:
            from racer_tools.vehicle_params_loader import load_vehicle_params

            vp = load_vehicle_params()
            limits = limits_from_vehicle(
                vp.steering.max_angle_rad,
                vp.steering.min_angle_rad,
                vp.steering.max_rate_rad_per_s,
                vp.actuation.max_acceleration_mps2,
                vp.actuation.throttle_full_scale_mps,
                rate_hz,
            )
        else:
            limits = limits_from_vehicle(0.4, -0.4, 3.0, 4.0, 3.0, rate_hz)
        if self._face_speed > limits.max_speed_mps:
            raise ValueError("face_speed_mps exceeds the vehicle's speed limit")
        self._limiter = RateLimiter(limits)
        self._log = EventLog(output_dir)

        self._last_objects_t = -math.inf
        self._obs: Observation | None = None
        self._pending = None  # (Trigger, ros time of the trigger frame)
        self._patrol_cmd: tuple[float, float] | None = None
        self._patrol_cmd_t = -math.inf
        self._speed: float | None = None
        self._speed_t = -math.inf
        self._image: bytes | None = None

        self._tf_buffer = None
        try:
            import tf2_ros

            self._tf_buffer = tf2_ros.Buffer()
            self._tf_listener = tf2_ros.TransformListener(self._tf_buffer, self)
        except ImportError:  # pragma: no cover - tf2_ros ships with ros-base
            self.get_logger().warning("tf2_ros missing: alerts carry no pose")

        rel = QoSProfile(history=HistoryPolicy.KEEP_LAST, depth=10)
        best = QoSProfile(
            history=HistoryPolicy.KEEP_LAST, depth=1, reliability=ReliabilityPolicy.BEST_EFFORT
        )
        self._drive_pub = self.create_publisher(AckermannDriveStamped, drive_topic, rel)
        self._alert_pub = self.create_publisher(SpiderbotAlert, "/spiderbot/alert", rel)
        self._state_pub = self.create_publisher(String, "/spiderbot/state", rel)
        self.create_subscription(ObjectDetectionArray, objects_topic, self._on_objects, rel)
        self.create_subscription(AckermannDriveStamped, patrol_topic, self._on_patrol, rel)
        self.create_subscription(Odometry, speed_topic, self._on_speed, rel)
        self.create_subscription(CompressedImage, image_topic, self._on_image, best)
        self.create_timer(1.0 / rate_hz, self._tick)
        self.create_timer(0.1, self._publish_state)
        self.get_logger().info(
            f"spiderbot up: rule {self._rule.name} x{rule_frames}, patrol cmd {patrol_topic} -> "
            f"{drive_topic}, events in {self._log.dir}"
        )

    # -- inputs ----------------------------------------------------------------------------

    def _on_patrol(self, msg: AckermannDriveStamped) -> None:
        self._patrol_cmd = (float(msg.drive.speed), float(msg.drive.steering_angle))
        self._patrol_cmd_t = time.monotonic()

    def _on_speed(self, msg: Odometry) -> None:
        self._speed = float(msg.twist.twist.linear.x)
        self._speed_t = time.monotonic()

    def _on_image(self, msg: CompressedImage) -> None:
        if msg.format.lower().startswith("jpeg") or msg.format.lower().startswith("jpg"):
            self._image = bytes(msg.data)

    def _on_objects(self, msg: ObjectDetectionArray) -> None:
        now = time.monotonic()
        self._last_objects_t = now
        people = people_from_objects(msg)
        chosen = choose_person([p for p in people if p.score >= self._rule_min_score()])
        if chosen is not None:
            self._obs = Observation(now, chosen.bearing_rad, chosen.range_m, chosen.range_valid)
        if self._machine.phase != Phase.PATROL:
            return
        trigger = self._rule.update(now, people)
        if trigger is not None and self._pending is None:
            self._pending = (trigger, self.get_clock().now())

    def _rule_min_score(self) -> float:
        return float(self.get_parameter("rule_min_score").value)

    # -- control ---------------------------------------------------------------------------

    def _tick(self) -> None:
        now = time.monotonic()
        fresh = now - self._last_objects_t <= self._perception_timeout_s
        speed = self._speed if now - self._speed_t <= _SPEED_MAX_AGE_S else None
        pending, self._pending = self._pending, None
        command, events = self._machine.step(
            now, pending[0] if pending else None, self._obs, fresh, speed
        )
        for event in events:
            self._handle_event(event, pending)
        target = self._target(command, now)
        if target is None:
            self._limiter.reset(0.0, 0.0)
            return
        speed_cmd, steer_cmd = self._limiter.step(*target)
        out = AckermannDriveStamped()
        out.header.stamp = self.get_clock().now().to_msg()
        out.header.frame_id = self._base_frame
        out.drive.speed = float(speed_cmd)
        out.drive.steering_angle = float(steer_cmd)
        self._drive_pub.publish(out)

    def _target(self, command, now: float) -> tuple[float, float] | None:
        if command.kind == CommandKind.PASS_THROUGH:
            if self._patrol_cmd is None or now - self._patrol_cmd_t > self._patrol_timeout_s:
                return None  # silence: let safety_node's watchdog act
            return self._patrol_cmd
        if command.kind == CommandKind.FACE and command.bearing_rad is not None:
            return face_command(
                command.bearing_rad, self._face_gain, self._face_speed, self._limiter.limits
            )
        return 0.0, 0.0

    # -- events ----------------------------------------------------------------------------

    def _pose(self):
        if self._tf_buffer is None:
            return None
        try:
            tf = self._tf_buffer.lookup_transform(
                self._map_frame, self._base_frame, rclpy.time.Time()
            )
        except Exception:  # noqa: BLE001  no transform yet is an answer, not a failure
            return None
        q = tf.transform.rotation
        yaw = math.atan2(2.0 * (q.w * q.z + q.x * q.y), 1.0 - 2.0 * (q.y * q.y + q.z * q.z))
        t = tf.transform.translation
        return float(t.x), float(t.y), yaw

    def _handle_event(self, event, pending) -> None:
        wall = time.time()
        logger = self.get_logger()
        if event.kind == EventKind.ALERT:
            data = event.data
            person: Person = data["person"]
            event_id = self._log.new_event_id()
            snapshot = self._log.write_snapshot(event_id, self._image)
            pose = self._pose()
            msg = SpiderbotAlert()
            msg.header.stamp = self.get_clock().now().to_msg()
            msg.header.frame_id = self._map_frame
            msg.event_id = event_id
            msg.rule = self._rule.name
            msg.held_frames = int(min(data["frames"], 65535))
            msg.score = float(person.score)
            msg.bearing_rad = float(person.bearing_rad)
            msg.range_m = float(person.range_m)
            msg.range_valid = bool(person.range_valid)
            msg.facing_ok = bool(data["facing_ok"])
            msg.final_bearing_rad = float(data["final_bearing_rad"])
            msg.pose_valid = pose is not None
            if pose is not None:
                msg.pose_x_m, msg.pose_y_m, msg.pose_yaw_rad = pose
            msg.snapshot_path = snapshot
            self._alert_pub.publish(msg)
            self._log.append(
                "alert",
                wall,
                event_id=event_id,
                rule=self._rule.name,
                frames=data["frames"],
                score=person.score,
                bearing_rad=person.bearing_rad,
                range_m=person.range_m,
                range_valid=person.range_valid,
                facing_ok=data["facing_ok"],
                final_bearing_rad=data["final_bearing_rad"],
                pose=pose,
                snapshot=snapshot,
            )
            logger.warning(
                f"ALERT {event_id}: {self._rule.name} held {data['frames']} frames, facing_ok="
                f"{data['facing_ok']}, snapshot {snapshot or 'none'}"
            )
        elif event.kind == EventKind.ENGAGE:
            person = event.data["person"]
            self._log.append(
                "engage",
                wall,
                rule=self._rule.name,
                frames=event.data["frames"],
                bearing_rad=person.bearing_rad,
                range_m=person.range_m,
                pose=self._pose(),
            )
            logger.info("engaging: stopping")
        elif event.kind == EventKind.RESUME:
            self._rule.reset()
            self._log.append("resume", wall, engagement=event.data["engagement"])
            logger.info("hold over: resuming patrol")
        else:
            self._log.append(event.kind.value, wall)
            logger.warning(f"{event.kind.value}")

    def _publish_state(self) -> None:
        self._state_pub.publish(String(data=self._machine.phase.value))

    def state_json(self) -> str:  # for debugging and tests
        return json.dumps({"phase": self._machine.phase.value})


def main(args: list | None = None) -> None:
    rclpy.init(args=args)
    node: SpiderbotNode | None = None
    code = 0
    try:
        node = SpiderbotNode()
        rclpy.spin(node)
    except ValueError as exc:
        get_logger("spiderbot_node").error(str(exc))
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
