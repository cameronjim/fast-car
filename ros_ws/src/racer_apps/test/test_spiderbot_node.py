"""L3: spiderbot_node on mocked topics (own DDS domain per case, like racer_perception's tests).

Drives the node with a fake tracker (/spiderbot/patrol_cmd), fake fused people
(/perception/objects) and a fake camera, and reads /drive_raw, /spiderbot/state,
/spiderbot/alert and the event log. The pure decisions are L1-tested; what is checked here is
the plumbing: the output topic, ramping, silence rules, the alert content, the event log and the
snapshot file.
"""

from __future__ import annotations

import json
import math
import time

import pytest

pytest.importorskip("rclpy")
pytest.importorskip("racer_msgs")
pytest.importorskip("racer_tools")

import rclpy
from ackermann_msgs.msg import AckermannDriveStamped
from racer_apps.spiderbot.spiderbot_node import SpiderbotNode
from racer_apps.spiderbot.synthetic import synthetic_keypoints
from racer_msgs.msg import Keypoint2D, ObjectDetection, ObjectDetectionArray, SpiderbotAlert
from rclpy.context import Context
from rclpy.executors import SingleThreadedExecutor
from rclpy.parameter import Parameter
from rclpy.qos import HistoryPolicy, QoSProfile, ReliabilityPolicy
from sensor_msgs.msg import CompressedImage
from std_msgs.msg import String

_REL = QoSProfile(reliability=ReliabilityPolicy.RELIABLE, history=HistoryPolicy.KEEP_LAST, depth=10)
_BEST = QoSProfile(
    reliability=ReliabilityPolicy.BEST_EFFORT, history=HistoryPolicy.KEEP_LAST, depth=1
)


class Harness:
    def __init__(self, domain_id: int, tmp_path, **params):
        self.context = Context()
        rclpy.init(context=self.context, domain_id=domain_id)
        base = {"output_dir": str(tmp_path / "out"), "stop_settle_s": 0.3, "hold_s": 1.0}
        base.update(params)
        overrides = [Parameter(k, value=v) for k, v in base.items()]
        self.node = SpiderbotNode(context=self.context, parameter_overrides=overrides)
        self.helper = rclpy.create_node("spiderbot_test_helper", context=self.context)
        self.executor = SingleThreadedExecutor(context=self.context)
        self.executor.add_node(self.node)
        self.executor.add_node(self.helper)
        self.drive: list[tuple[float, AckermannDriveStamped]] = []
        self.states: list[str] = []
        self.alerts: list[SpiderbotAlert] = []
        self.helper.create_subscription(
            AckermannDriveStamped,
            "/drive_raw",
            lambda m: self.drive.append((time.monotonic(), m)),
            _REL,
        )
        self.helper.create_subscription(
            String, "/spiderbot/state", lambda m: self.states.append(m.data), _REL
        )
        self.helper.create_subscription(
            SpiderbotAlert, "/spiderbot/alert", self.alerts.append, _REL
        )
        self.patrol_pub = self.helper.create_publisher(
            AckermannDriveStamped, "/spiderbot/patrol_cmd", _REL
        )
        self.objects_pub = self.helper.create_publisher(
            ObjectDetectionArray, "/perception/objects", _REL
        )
        self.image_pub = self.helper.create_publisher(
            CompressedImage, "/camera/usb/image_raw/compressed", _BEST
        )
        self.patrol_speed = 0.0
        self.patrol_steer = 0.0
        self.people: list[ObjectDetection] = []
        self._last_feed = 0.0
        self._last_patrol = 0.0
        self.spin_until(lambda: self.patrol_pub.get_subscription_count() > 0)
        self.spin_until(lambda: self.objects_pub.get_subscription_count() > 0)
        self.spin_until(lambda: self.image_pub.get_subscription_count() > 0)

    def spin_for(self, seconds: float, feed: bool = True) -> None:
        end = time.monotonic() + seconds
        while time.monotonic() < end:
            self.executor.spin_once(timeout_sec=0.01)
            if feed:
                self._feed()

    def spin_until(self, predicate, timeout_s: float = 10.0, feed: bool = True) -> bool:
        end = time.monotonic() + timeout_s
        while time.monotonic() < end:
            self.executor.spin_once(timeout_sec=0.01)
            if feed:
                self._feed()
            if predicate():
                return True
        return False

    def _feed(self) -> None:
        now = time.monotonic()
        if self.patrol_speed is not None and now - self._last_patrol >= 0.02:
            self._last_patrol = now
            cmd = AckermannDriveStamped()
            cmd.drive.speed, cmd.drive.steering_angle = self.patrol_speed, self.patrol_steer
            self.patrol_pub.publish(cmd)
        if self.people is not None and now - self._last_feed >= 0.1:
            self._last_feed = now
            arr = ObjectDetectionArray()
            arr.detections = list(self.people)
            self.objects_pub.publish(arr)

    def close(self) -> None:
        self.executor.shutdown()
        self.node.destroy_node()
        self.helper.destroy_node()
        rclpy.shutdown(context=self.context)

    def last_speed(self) -> float:
        return self.drive[-1][1].drive.speed if self.drive else math.nan


def person(pose="arms_raised", bearing=0.0, rng=3.0, score=0.9) -> ObjectDetection:
    det = ObjectDetection()
    det.class_name, det.score = "person", score
    det.bearing_rad, det.range_m, det.range_valid, det.range_status = bearing, rng, True, "ok"
    det.box_height_px = 300.0
    det.keypoint_layout = "coco17"
    for k in synthetic_keypoints(pose, 640.0, 100.0, 300.0):
        det.keypoints.append(Keypoint2D(x_px=k.x_px, y_px=k.y_px, confidence=k.confidence))
    return det


@pytest.fixture
def harness(tmp_path):
    made: list[Harness] = []
    counter = iter(range(130, 160))

    def factory(**params):
        h = Harness(next(counter), tmp_path, **params)
        made.append(h)
        return h

    yield factory
    for h in made:
        h.close()


def test_patrol_command_passes_through_ramped_and_never_to_slash_drive(harness):
    h = harness(face_enabled=False)
    h.patrol_speed, h.patrol_steer = 0.5, 0.1
    assert h.spin_until(lambda: h.last_speed() == pytest.approx(0.5))
    speeds = [m.drive.speed for _, m in h.drive]
    assert speeds[0] < 0.5  # ramped, not a step
    assert max(speeds) <= 0.5 + 1e-6
    assert h.drive[-1][1].drive.steering_angle == pytest.approx(0.1, abs=1e-6)
    assert h.node.count_publishers("/drive") == 0
    assert h.node.count_publishers("/drive_raw") == 1


def test_drive_topic_slash_drive_is_refused(tmp_path):
    context = Context()
    rclpy.init(context=context, domain_id=169)
    try:
        with pytest.raises(ValueError, match="only safety_node"):
            SpiderbotNode(
                context=context,
                parameter_overrides=[
                    Parameter("drive_topic", value="/drive"),
                    Parameter("output_dir", value=str(tmp_path)),
                ],
            )
    finally:
        rclpy.shutdown(context=context)


def test_silent_patrol_command_means_no_drive_raw_at_all(harness):
    h = harness()
    h.patrol_speed = 0.4
    assert h.spin_until(lambda: len(h.drive) > 5)
    h.patrol_speed = None  # the tracker dies
    h.spin_for(0.6)
    count = len(h.drive)
    h.spin_for(0.5)
    assert len(h.drive) == count, "the node kept publishing with no patrol command"


def test_dead_perception_stops_the_car_then_it_resumes(harness):
    h = harness(perception_timeout_s=0.5)
    h.patrol_speed = 0.4
    assert h.spin_until(lambda: h.last_speed() == pytest.approx(0.4))
    h.people = None  # the detector goes quiet
    assert h.spin_until(lambda: h.last_speed() == pytest.approx(0.0), feed=True)
    assert h.spin_until(lambda: "no_perception" in h.states)
    h.people = []
    assert h.spin_until(lambda: h.last_speed() == pytest.approx(0.4), timeout_s=15.0)


def test_arms_raised_stops_alerts_logs_snapshots_and_resumes(harness, tmp_path):
    h = harness(face_enabled=False, rule_frames=3, cooldown_s=30.0)
    h.patrol_speed = 0.4
    assert h.spin_until(lambda: h.last_speed() == pytest.approx(0.4))
    image = CompressedImage()
    image.format, image.data = "jpeg", b"\xff\xd8fake-jpeg\xff\xd9"
    h.image_pub.publish(image)
    h.spin_for(0.2)
    h.people = [person("standing")]
    h.spin_for(0.6)
    assert h.last_speed() == pytest.approx(0.4)  # standing people trigger nothing
    h.people = [person("arms_raised", bearing=0.05)]
    assert h.spin_until(lambda: len(h.alerts) == 1, timeout_s=10.0)
    assert h.spin_until(lambda: h.last_speed() == pytest.approx(0.0))
    alert = h.alerts[0]
    assert alert.rule == "arms_raised" and alert.held_frames >= 3
    assert alert.range_valid and alert.range_m == pytest.approx(3.0)
    assert alert.bearing_rad == pytest.approx(0.05, abs=1e-6)
    assert alert.event_id and alert.header.frame_id == "map"
    assert alert.facing_ok is True  # 0.05 rad is inside the 0.12 tolerance
    assert alert.snapshot_path.endswith(f"{alert.event_id}.jpg")
    assert (tmp_path / "out" / "snapshots" / f"{alert.event_id}.jpg").read_bytes() == bytes(
        image.data
    )
    assert h.spin_until(lambda: "stopping" in h.states and "hold" in h.states)
    assert h.spin_until(lambda: h.states[-1] == "patrol", timeout_s=10.0)
    assert h.spin_until(lambda: h.last_speed() == pytest.approx(0.4), timeout_s=10.0)
    log = [
        json.loads(x)
        for x in (tmp_path / "out" / "spiderbot_events.jsonl").read_text().splitlines()
    ]
    assert [r["kind"] for r in log] == ["engage", "alert", "resume"]
    assert log[1]["event_id"] == alert.event_id and log[1]["facing_ok"] is True
    assert len(h.alerts) == 1, "cooldown must stop the same person re-triggering"


def test_alert_without_a_camera_frame_or_pose_says_so(harness):
    h = harness(face_enabled=False, rule="person_present", rule_frames=2)
    h.patrol_speed = 0.4
    h.spin_for(0.3)
    h.people = [person("standing")]
    assert h.spin_until(lambda: len(h.alerts) == 1)
    assert h.alerts[0].snapshot_path == "" and h.alerts[0].pose_valid is False


def test_facing_creeps_forward_and_steers_towards_the_person(harness):
    h = harness(face_enabled=True, rule_frames=2, face_speed_mps=0.3, face_timeout_s=3.0)
    h.patrol_speed = 0.4
    assert h.spin_until(lambda: h.last_speed() == pytest.approx(0.4))
    h.people = [person("arms_raised", bearing=0.4)]  # person on the left, never converges
    assert h.spin_until(lambda: len(h.alerts) == 1, timeout_s=12.0)
    creeping = [
        m.drive for t, m in h.drive if m.drive.speed > 0.05 and m.drive.steering_angle > 0.05
    ]
    assert creeping, "never steered left while facing a person on the left"
    assert max(d.speed for d in creeping) <= 0.3 + 1e-6
    assert h.alerts[0].facing_ok is False  # the fake person never moved to the centre
    assert h.alerts[0].final_bearing_rad == pytest.approx(0.4, abs=1e-6)


def test_bad_rule_name_is_a_clean_config_error(tmp_path):
    context = Context()
    rclpy.init(context=context, domain_id=168)
    try:
        with pytest.raises(ValueError, match="unknown rule"):
            SpiderbotNode(
                context=context,
                parameter_overrides=[
                    Parameter("rule", value="moonwalk"),
                    Parameter("output_dir", value=str(tmp_path)),
                ],
            )
    finally:
        rclpy.shutdown(context=context)
