"""L3 node test for racer_tools' camera_check (claude-docs/12-testing.md, optional cameras).

Runs the INSTALLED `camera_check` executable as a subprocess against a mocked
sensor_msgs/CompressedImage topic published from this test, and checks its exit code and
report, the same shape as test_lidar_check_node_exit_codes.py:

  * nominal: frames stamped at 60 fps carrying a hand-built 1280x720 JPEG header, published
    reliable the way usb_cam does (camera_check's best_effort subscription has to match it),
    exit 0 and report the resolution parsed out of the bytes;
  * failing stream: the same frames stamped at 30 fps against the default 54 Hz threshold exit
    1, so a slow camera is a non-zero exit and not just a log line;
  * silence: nothing published exits 2, distinct from 1.

Each case runs in its own DDS domain. Header stamps are synthetic, so the measured rate does
not depend on how fast this test manages to publish. pytest.importorskip-guarded: skipped
under the bare `uv run pytest` L1 run, runs for real under `colcon test` in the ros-dev
image. Every pass/fail rule is covered without ROS by test_frame_check.py.
"""

from __future__ import annotations

import os
import shutil
import struct
import subprocess
import time

import pytest

pytest.importorskip("rclpy")

import rclpy
from rclpy.context import Context
from rclpy.qos import HistoryPolicy, QoSProfile, ReliabilityPolicy
from sensor_msgs.msg import CompressedImage

_TOPIC = "/camera/test/image_raw/compressed"
_WINDOW_S = 3.0
_PUBLISH_PERIOD_S = 0.01
# Start-up (DDS discovery) plus the listening window, with margin.
_PROCESS_TIMEOUT_S = 60.0


def _jpeg_header(width: int, height: int) -> bytes:
    sof_payload = struct.pack(">BHHB", 8, height, width, 3) + bytes(9)
    sof = b"\xff\xc0" + struct.pack(">H", len(sof_payload) + 2) + sof_payload
    return b"\xff\xd8" + sof + b"\xff\xd9"


_JPEG = _jpeg_header(1280, 720)


def _make_frame(index: int, stamp_period_s: float) -> CompressedImage:
    msg = CompressedImage()
    stamp = index * stamp_period_s
    msg.header.stamp.sec = int(stamp)
    msg.header.stamp.nanosec = round((stamp - int(stamp)) * 1e9)
    msg.header.frame_id = "camera_test"
    msg.format = "jpeg"
    msg.data = _JPEG
    return msg


def _run_camera_check(domain_id: int, stamp_period_s: float | None) -> tuple[int, str]:
    """Run camera_check in `domain_id`, publishing frames until it exits (None: publish nothing).

    Returns (exit code, combined stdout and stderr).
    """
    ros2 = shutil.which("ros2")
    assert ros2 is not None, "ros2 CLI not on PATH; run under colcon test"
    env = dict(os.environ, ROS_DOMAIN_ID=str(domain_id))
    proc = subprocess.Popen(
        [ros2, "run", "racer_tools", "camera_check", "--ros-args",
         "-p", f"topic:={_TOPIC}", "-p", f"duration_s:={_WINDOW_S}",
         "-p", "expected_width:=1280", "-p", "expected_height:=720"],
        env=env,
        stdout=subprocess.PIPE,
        stderr=subprocess.STDOUT,
        text=True,
    )  # fmt: skip
    context = Context()
    rclpy.init(context=context, domain_id=domain_id)
    node = rclpy.create_node("camera_check_test_publisher", context=context)
    try:
        qos = QoSProfile(
            reliability=ReliabilityPolicy.RELIABLE, history=HistoryPolicy.KEEP_LAST, depth=10
        )
        pub = node.create_publisher(CompressedImage, _TOPIC, qos)
        deadline = time.monotonic() + _PROCESS_TIMEOUT_S
        index = 0
        while proc.poll() is None and time.monotonic() < deadline:
            if stamp_period_s is not None:
                pub.publish(_make_frame(index, stamp_period_s))
                index += 1
            time.sleep(_PUBLISH_PERIOD_S)
    finally:
        node.destroy_node()
        rclpy.shutdown(context=context)
    if proc.poll() is None:
        proc.kill()
    output, _ = proc.communicate(timeout=10)
    assert proc.returncode is not None
    return proc.returncode, output


def test_healthy_stream_exits_zero_and_reports_resolution():
    code, output = _run_camera_check(domain_id=89, stamp_period_s=1.0 / 60.0)
    assert code == 0, output
    assert "RESULT: PASS" in output
    assert "1280x720 (expected 1280x720)" in output


def test_stream_below_minimum_rate_exits_one():
    code, output = _run_camera_check(domain_id=90, stamp_period_s=1.0 / 30.0)
    assert code == 1, output
    assert "RESULT: FAIL" in output
    assert "is below the minimum" in output


def test_silence_exits_two():
    code, output = _run_camera_check(domain_id=91, stamp_period_s=None)
    assert code == 2, output
    assert "received 0 frame(s)" in output
