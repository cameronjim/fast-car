"""L3 check of rt_sched (brief 52) on the REAL car_teleop.launch.py, in the ros-dev image.

Two cases, chosen by what the container allows (the check `chrt -f <priority> true`):

  * FIFO NOT grantable (the ordinary CI container: no --ulimit rtprio, no CAP_SYS_NICE):
    `rt_sched:=on` must REFUSE. safety_node never appears in the ROS graph, the launch ends, the
    exit code is non-zero, and the output holds the registered refusal text. This is the red test
    of brief 52: there is no fallback to SCHED_OTHER.
  * FIFO grantable (a local run in a container started with `--ulimit rtprio=<n>`): both the
    check line "all FIFO <priority>" and a /drive at about 50 Hz appear, and every thread of the
    real safety_node process is FIFO at the configured priority. In CI, where this cannot run, the
    case is reported as SKIPPED, never passed.

rt_sched:=off is covered by the existing L3 suites, which pass no rt_sched at all.
"""

from __future__ import annotations

import os

os.environ.setdefault("ROS_DOMAIN_ID", "155")

import importlib.util
import pathlib
import signal
import subprocess
import tempfile
import time

import pytest
import rclpy
from ackermann_msgs.msg import AckermannDriveStamped
from rclpy.qos import HistoryPolicy, QoSProfile, ReliabilityPolicy

_REPO = pathlib.Path(__file__).resolve().parents[4]
_LAUNCH_DIR = _REPO / "ros_ws" / "src" / "racer_bringup" / "launch"


def _load(path: pathlib.Path, name: str):
    spec = importlib.util.spec_from_file_location(name, path)
    module = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(module)
    return module


_SCHED = _load(_REPO / "config" / "scheduling.py", "scheduling_l3")
_RT = _load(_LAUNCH_DIR / "rt_sched.py", "rt_sched_l3")
SCHEDULING = _SCHED.load()


def _fifo_grantable() -> bool:
    done = subprocess.run(
        ["chrt", "-f", str(SCHEDULING.safety_priority), "true"],
        capture_output=True,
        timeout=10,
        check=False,
    )
    return done.returncode == 0


FIFO_OK = _fifo_grantable()


def _fake_sysfs() -> str:
    root = tempfile.mkdtemp(prefix="racer_rt_sched_sysfs_")
    chip = pathlib.Path(root) / "pwmchip0"
    chip.mkdir()
    (chip / "export").write_text("")
    for channel in (0, 1):
        d = chip / f"pwm{channel}"
        d.mkdir()
        for attribute in ("period", "duty_cycle", "enable"):
            (d / attribute).write_text("0")
    return root


def _launch(extra: list[str]) -> subprocess.Popen:
    # record:=false and rail_voltage:=false for the same reason as test_car_teleop_launch: this
    # is about scheduling, not the bag (test_car_teleop_bag_launch owns the recorder defaults).
    return subprocess.Popen(
        [
            "ros2",
            "launch",
            "racer_bringup",
            "car_teleop.launch.py",
            "viz:=false",
            "start_teleop:=false",
            "browser_teleop:=false",
            "record:=false",
            "rail_voltage:=false",
            f"sysfs_root:={_fake_sysfs()}",
            "throttle_pwmchip:=0",
            "throttle_pwm_channel:=1",
            *extra,
        ],
        stdout=subprocess.PIPE,
        stderr=subprocess.STDOUT,
        text=True,
        start_new_session=True,
    )


def _stop(proc: subprocess.Popen) -> str:
    if proc.poll() is None:
        os.killpg(proc.pid, signal.SIGINT)
    try:
        out, _ = proc.communicate(timeout=30)
    except subprocess.TimeoutExpired:
        os.killpg(proc.pid, signal.SIGKILL)
        out, _ = proc.communicate()
    return out or ""


@pytest.fixture
def rcl():
    rclpy.init()
    node = rclpy.create_node("rt_sched_l3_client")
    yield node
    node.destroy_node()
    rclpy.shutdown()


@pytest.mark.skipif(FIFO_OK, reason="FIFO is grantable here, so there is nothing to refuse")
def test_rt_sched_on_refuses_when_fifo_is_not_granted(rcl) -> None:
    proc = _launch(["rt_sched:=on"])
    seen: set[str] = set()
    deadline = time.time() + 90
    while proc.poll() is None and time.time() < deadline:
        rclpy.spin_once(rcl, timeout_sec=0.1)
        seen.update(rcl.get_node_names())
    ended = proc.poll() is not None
    out = _stop(proc)
    assert ended, f"the launch did not end after a refusal:\n{out[-2000:]}"
    assert "safety_node" not in seen, "safety_node came up although FIFO was not granted"
    assert _SCHED.REFUSAL_PREFIX in out, out[-2000:]
    assert proc.returncode != 0, f"exit code {proc.returncode}:\n{out[-2000:]}"


@pytest.mark.skipif(not FIFO_OK, reason="FIFO is not grantable here (no --ulimit rtprio)")
def test_rt_sched_on_runs_safety_node_fifo_and_publishes_drive(rcl) -> None:
    proc = _launch(["rt_sched:=on"])
    try:
        stamps: list[float] = []
        rcl.create_subscription(
            AckermannDriveStamped,
            "/drive",
            lambda m: stamps.append(time.time()),
            QoSProfile(
                reliability=ReliabilityPolicy.RELIABLE, history=HistoryPolicy.KEEP_LAST, depth=10
            ),
        )
        deadline = time.time() + 60
        while "safety_node" not in rcl.get_node_names() and time.time() < deadline:
            rclpy.spin_once(rcl, timeout_sec=0.2)
        assert "safety_node" in rcl.get_node_names(), "safety_node never came up under chrt"
        # the check runs CHECK_DELAY_S after the start; wait past it, then measure /drive
        end = time.time() + _RT.CHECK_DELAY_S + 3.0
        while time.time() < end:
            rclpy.spin_once(rcl, timeout_sec=0.05)
        stamps.clear()
        end = time.time() + 2.0
        while time.time() < end:
            rclpy.spin_once(rcl, timeout_sec=0.01)
        assert len(stamps) >= 80, f"/drive at {len(stamps) / 2.0:.0f} Hz, expected about 50"
        pids = subprocess.run(
            ["pgrep", "-f", "lib/racer_safety/safety_node"],
            capture_output=True,
            text=True,
            check=False,
        ).stdout.split()
        assert pids, "no safety_node process found"
        threads = _RT.read_threads(int(pids[0]))
        assert threads
        bad = [t for t in threads if t[1] != _RT.FIFO or t[2] != SCHEDULING.safety_priority]
        assert not bad, f"threads not FIFO {SCHEDULING.safety_priority}: {bad}"
    finally:
        out = _stop(proc)
    assert f"all FIFO {SCHEDULING.safety_priority}" in out, out[-3000:]
