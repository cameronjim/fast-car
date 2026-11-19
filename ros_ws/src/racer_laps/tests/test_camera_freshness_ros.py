"""L3, two real nodes (outside review of brief 51c, finding 1): `lap_counter_node` and
`lap_memory_node` on the camera source, a harness that plays the camera, the wheel odometry and the
planner. The counter republishes `running` with its similarity on a timer even when no image
arrives; the lap node must not call that source ready before the start, nor alive during a run.

* capture one frame, stop the images, keep both processes alive: never ready;
* frames flowing: ready; frames stop: not ready and not alive within a few seconds;
* the same during a run (after the planner's DRIVING report): not alive, the station uncertain.

Needs rclpy, cv2, racer_perception and racer_laps on the path; skipped where absent."""

from __future__ import annotations

import json
import os
import subprocess
import sys
import time
from pathlib import Path

import numpy as np
import pytest

rclpy = pytest.importorskip("rclpy")
cv2 = pytest.importorskip("cv2")
pytest.importorskip("racer_perception.lap_counter_node")
pytest.importorskip("racer_pnp_msgs.msg")
pytest.importorskip("racer_msgs.msg")
pytest.importorskip("racer_tools")

from nav_msgs.msg import Odometry
from racer_pnp_msgs.msg import PlannerStatus
from rclpy.qos import HistoryPolicy, QoSProfile, ReliabilityPolicy
from sensor_msgs.msg import CompressedImage
from std_msgs.msg import String
from std_srvs.srv import Trigger

os.environ.setdefault("ROS_DOMAIN_ID", "96")

STALE_S = 0.6


def _qos(reliable: bool) -> QoSProfile:
    return QoSProfile(
        reliability=ReliabilityPolicy.RELIABLE if reliable else ReliabilityPolicy.BEST_EFFORT,
        history=HistoryPolicy.KEEP_LAST,
        depth=10,
    )


def jpeg(seed: int) -> bytes:
    rng = np.random.default_rng(seed)
    small = rng.uniform(0, 255, size=(15, 20)).astype(np.float32)
    big = cv2.resize(small, (320, 240), interpolation=cv2.INTER_CUBIC)
    ok, buf = cv2.imencode(".jpg", np.clip(big, 0, 255).astype(np.uint8))
    assert ok
    return buf.tobytes()


class Rig:
    def __init__(self, tmp_path: Path) -> None:
        env = dict(os.environ)
        self.counter = subprocess.Popen(
            [
                sys.executable,
                "-m",
                "racer_perception.lap_counter_node",
                "--ros-args",
                "-p",
                f"frame_stale_s:={STALE_S}",
                "-p",
                "status_period_s:=0.2",
                "-p",
                "rate_hz:=0.0",
                "-p",
                "start_speed_mps:=0.0",
                "-p",
                "yaw_rate_topic:=",
            ],
            env=env,
            stdout=subprocess.PIPE,
            stderr=subprocess.STDOUT,
            text=True,
        )
        self.lap = subprocess.Popen(
            [
                sys.executable,
                "-m",
                "racer_laps.lap_memory_node",
                "--ros-args",
                "-p",
                f"output_dir:={tmp_path / 'laps'}",
                "-p",
                "yaw_gain_override:=1.0",
                "-p",
                "lap_source:=camera",
                "-p",
                "lap_mode:=observe",
                "-p",
                "status_period_s:=0.2",
            ],
            env=env,
            stdout=subprocess.PIPE,
            stderr=subprocess.STDOUT,
            text=True,
        )
        rclpy.init()
        self.node = rclpy.create_node("freshness_rig")
        self.image_pub = self.node.create_publisher(
            CompressedImage, "/camera/usb/image_raw/compressed", _qos(False)
        )
        self.odom_pub = self.node.create_publisher(Odometry, "/odom/wheel", _qos(False))
        self.planner_pub = self.node.create_publisher(
            PlannerStatus, "/pnp/planner/status", _qos(True)
        )
        self.statuses: list[dict] = []
        self.node.create_subscription(
            String,
            "/pnp/lap/status",
            lambda m: self.statuses.append(json.loads(m.data)),
            _qos(True),
        )
        self.reset = self.node.create_client(Trigger, "/lap_counter/reset")
        self.frame_k = 0

    def spin(
        self, seconds: float, frames: bool = False, odom: bool = True, driving: bool = False
    ) -> None:
        end = time.monotonic() + seconds
        next_frame = next_odom = 0.0
        while time.monotonic() < end:
            now = time.monotonic()
            if frames and now >= next_frame:
                msg = CompressedImage()
                msg.format = "jpeg"
                msg.data = jpeg(0)  # one scene: similarity 1.0 against the reference
                self.frame_k += 1
                self.image_pub.publish(msg)
                next_frame = now + 1.0 / 15.0
            if now >= next_odom:
                if odom:
                    o = Odometry()
                    o.twist.twist.linear.x = 0.4 if driving else 0.0
                    o.pose.pose.position.x = 0.4 * (now % 1000.0)
                    self.odom_pub.publish(o)
                if driving:
                    p = PlannerStatus()
                    p.state = 1
                    p.plan_min_clearance_m = 0.5
                    self.planner_pub.publish(p)
                next_odom = now + 0.05
            rclpy.spin_once(self.node, timeout_sec=0.01)

    def wait_for(self, predicate, seconds: float, **play) -> bool:
        end = time.monotonic() + seconds
        while time.monotonic() < end:
            self.spin(0.1, **play)
            if self.statuses and predicate(self.statuses[-1]):
                return True
        return False

    def call_reset(self) -> None:
        assert self.reset.wait_for_service(timeout_sec=20.0)
        future = self.reset.call_async(Trigger.Request())
        end = time.monotonic() + 10.0
        while not future.done() and time.monotonic() < end:
            self.spin(0.05)
        assert future.done() and future.result().success

    def close(self) -> None:
        for p in (self.counter, self.lap):
            p.terminate()
        for p in (self.counter, self.lap):
            try:
                p.wait(timeout=10)
            except subprocess.TimeoutExpired:
                p.kill()
        self.node.destroy_node()
        rclpy.shutdown()


@pytest.fixture
def rig(tmp_path):
    r = Rig(tmp_path)
    yield r
    r.close()


def ready(status: dict) -> bool:
    return status.get("source_ready_now") is True


def test_frames_flowing_is_ready_and_frames_stopping_is_not_ready_and_not_alive(rig) -> None:
    rig.spin(2.0)  # both nodes come up
    rig.call_reset()
    assert rig.wait_for(ready, 20.0, frames=True), rig.statuses[-1:]
    assert rig.statuses[-1]["source_camera_fresh"] is True
    # the images stop, both processes stay up and the counter's timer keeps saying `running`
    assert rig.wait_for(lambda s: s.get("source_camera_fresh") is False, 10.0, frames=False)
    assert rig.wait_for(lambda s: not ready(s), 5.0)
    assert rig.counter.poll() is None and rig.lap.poll() is None
    last = rig.statuses[-1]
    assert last["source_not_ready_reason"] or last["source_ready_now"] is False


def test_one_frame_then_no_images_is_never_ready(rig) -> None:
    rig.spin(2.0)
    rig.call_reset()
    rig.frame_k = 0
    rig.spin(0.15, frames=True)  # a frame or two: the reference is captured
    seen_ready = []
    end = time.monotonic() + 6.0
    while time.monotonic() < end:
        rig.spin(0.2, frames=False)
        if rig.statuses:
            seen_ready.append(ready(rig.statuses[-1]))
    assert rig.statuses, "the lap node published nothing"
    assert not any(seen_ready[3:]), "ready on a dead image stream"
    assert rig.counter.poll() is None and rig.lap.poll() is None


def test_frames_stopping_during_a_run_makes_the_source_not_alive_and_the_station_uncertain(
    rig,
) -> None:
    rig.spin(2.0)
    rig.call_reset()
    assert rig.wait_for(ready, 20.0, frames=True)
    # the planner reports DRIVING and the car moves: the lap node starts its run
    assert rig.wait_for(lambda s: s.get("phase") == "running", 15.0, frames=True, driving=True), (
        rig.statuses[-1:]
    )
    assert rig.statuses[-1]["source_ready"] is True and rig.statuses[-1]["source_alive"] is True
    assert rig.wait_for(
        lambda s: s.get("source_alive") is False, 10.0, frames=False, driving=True
    ), rig.statuses[-1:]
    assert rig.statuses[-1]["station_uncertain"] is True
    assert rig.counter.poll() is None and rig.lap.poll() is None
