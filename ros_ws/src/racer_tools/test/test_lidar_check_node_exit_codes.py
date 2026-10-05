"""L3 node test for racer_tools' lidar_check (claude-docs/12-testing.md, roadmap 2.3).

Runs the INSTALLED `lidar_check` executable as a subprocess against a mocked /scan published
from this test, and checks its exit code and report:

  * nominal: a healthy synthetic stream (the beam count and field of view vehicle_params'
    sensors.lidar_spec implies, stamps one nominal period apart, published reliable the way
    sllidar_ros2 does, so lidar_check's best_effort subscription has to match it) exits 0;
  * failing stream: the same scans stamped at half the minimum rate exit 1, so a bad stream is
    a non-zero exit and not just a log line;
  * silence: nothing on /scan exits 2, distinct from 1, so a script can tell "LiDAR not
    running" from "LiDAR bad".

A subprocess rather than launch_testing because the process under test is SUPPOSED to exit
on its own, which launch_testing reports as "processes under test stopped before tests
completed". Each case runs in its own DDS domain (this test's rclpy context and the child's
ROS_DOMAIN_ID), so no case, and no other test in this pytest process, can feed another's
/scan. Header stamps are synthetic, so the measured rate does not depend on how fast this
test manages to publish.

pytest.importorskip-guarded like the other racer_tools node tests: skipped under the bare
`uv run pytest` L1 run, runs for real under `colcon test` in the ros-dev image. Every pass/fail
rule is covered without ROS by test_scan_check.py.
"""

from __future__ import annotations

import math
import os
import shutil
import subprocess
import time

import pytest

pytest.importorskip("rclpy")

import rclpy
from racer_tools.scan_check import angle_compensated_beam_count
from racer_tools.vehicle_params_loader import load_vehicle_params
from rclpy.context import Context
from rclpy.qos import HistoryPolicy, QoSProfile, ReliabilityPolicy
from sensor_msgs.msg import LaserScan

_SPEC = load_vehicle_params().sensors.lidar_spec
_BEAMS = angle_compensated_beam_count(_SPEC.sample_rate_hz, _SPEC.nominal_scan_rate_hz)
_PERIOD_S = 1.0 / _SPEC.nominal_scan_rate_hz
_WINDOW_S = 3.0
# Start-up (vehicle_params codegen, DDS discovery) plus the listening window, with margin.
_PROCESS_TIMEOUT_S = 60.0


def _make_scan(index: int, stamp_period_s: float) -> LaserScan:
    msg = LaserScan()
    stamp = index * stamp_period_s
    msg.header.stamp.sec = int(stamp)
    msg.header.stamp.nanosec = round((stamp - int(stamp)) * 1e9)
    msg.header.frame_id = "laser"
    msg.angle_min = -_SPEC.field_of_view_rad / 2.0
    msg.angle_max = _SPEC.field_of_view_rad / 2.0
    msg.angle_increment = _SPEC.field_of_view_rad / (_BEAMS - 1)
    msg.range_min = _SPEC.range_min_m
    msg.range_max = _SPEC.range_max_m
    # A room: everything at a third of the maximum range, one beam with no return.
    msg.ranges = [_SPEC.range_max_m / 3.0] * (_BEAMS - 1) + [math.inf]
    return msg


def _run_lidar_check(domain_id: int, stamp_period_s: float | None) -> tuple[int, str]:
    """Run lidar_check in `domain_id`, publishing scans until it exits (None: publish nothing).

    Returns (exit code, combined stdout and stderr).
    """
    ros2 = shutil.which("ros2")
    assert ros2 is not None, "ros2 CLI not on PATH; run under colcon test"
    env = dict(os.environ, ROS_DOMAIN_ID=str(domain_id))
    proc = subprocess.Popen(
        [ros2, "run", "racer_tools", "lidar_check", "--ros-args", "-p",
         f"duration_s:={_WINDOW_S}"],
        env=env,
        stdout=subprocess.PIPE,
        stderr=subprocess.STDOUT,
        text=True,
    )  # fmt: skip
    context = Context()
    rclpy.init(context=context, domain_id=domain_id)
    node = rclpy.create_node("lidar_check_test_publisher", context=context)
    try:
        qos = QoSProfile(
            reliability=ReliabilityPolicy.RELIABLE, history=HistoryPolicy.KEEP_LAST, depth=10
        )
        pub = node.create_publisher(LaserScan, "/scan", qos)
        deadline = time.monotonic() + _PROCESS_TIMEOUT_S
        index = 0
        while proc.poll() is None and time.monotonic() < deadline:
            if stamp_period_s is not None:
                pub.publish(_make_scan(index, stamp_period_s))
                index += 1
            time.sleep(_PERIOD_S)
    finally:
        node.destroy_node()
        rclpy.shutdown(context=context)
    if proc.poll() is None:
        proc.kill()
    output, _ = proc.communicate(timeout=10)
    assert proc.returncode is not None
    return proc.returncode, output


def test_healthy_stream_exits_zero_and_reports_pass():
    code, output = _run_lidar_check(domain_id=84, stamp_period_s=_PERIOD_S)
    assert code == 0, output
    assert "RESULT: PASS" in output
    assert f"{_BEAMS}..{_BEAMS}" in output


def test_stream_below_minimum_rate_exits_one():
    code, output = _run_lidar_check(domain_id=85, stamp_period_s=2.0 / _SPEC.min_scan_rate_hz)
    assert code == 1, output
    assert "RESULT: FAIL" in output
    assert "is below the minimum" in output


def test_silence_exits_two():
    code, output = _run_lidar_check(domain_id=86, stamp_period_s=None)
    assert code == 2, output
    assert "received 0 scan(s)" in output
