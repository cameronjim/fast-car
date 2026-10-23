"""Launch checks for perception.launch.py and detector_node's behaviour without ultralytics.

* L1 (no process started): the launch description's arguments and defaults, that both nodes
  are present with the documented executables, that detector:=false / fusion:=false are
  conditions, and that car_teleop.launch.py does NOT include perception (it must stay out
  of car-stack by default);
* L3: the INSTALLED detector_node exits with code 3 and ONE clear error naming the build
  flags when ultralytics/torch are missing. Skipped when ultralytics IS installed (it would
  try to fetch weights); the launch run of the real detector is skipped unless
  RACER_TEST_YOLO_WEIGHTS points at a weights file, so CI never downloads anything.
"""

from __future__ import annotations

import importlib.util
import os
import pathlib
import shutil
import subprocess

import pytest

pytest.importorskip("launch")
pytest.importorskip("launch_ros")

from launch import LaunchContext
from launch.actions import DeclareLaunchArgument, RegisterEventHandler
from launch_ros.actions import Node

_LAUNCH_FILE = pathlib.Path(__file__).resolve().parents[1] / "launch" / "perception.launch.py"
_CAR_TELEOP = (
    pathlib.Path(__file__).resolve().parents[2]
    / "racer_bringup"
    / "launch"
    / "car_teleop.launch.py"
)
_HAS_ULTRALYTICS = importlib.util.find_spec("ultralytics") is not None


def _load():
    spec = importlib.util.spec_from_file_location("perception_launch", _LAUNCH_FILE)
    module = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(module)
    return module


def _description():
    return _load().generate_launch_description()


def _defaults(entities):
    context = LaunchContext()
    return {
        e.name: "".join(s.perform(context) for s in e.default_value)
        for e in entities
        if isinstance(e, DeclareLaunchArgument)
    }


def test_arguments_and_defaults():
    defaults = _defaults(_description().entities)
    assert defaults["detector"] == "true" and defaults["fusion"] == "true"
    assert defaults["image_topic"] == "/camera/usb/image_raw/compressed"
    assert defaults["model"] == "yolov8n.pt"
    assert defaults["device"] == "cpu"
    assert defaults["rate_hz"] == "10.0"
    assert defaults["debug_image"] == "false"
    assert defaults["image_width_px"] == "1280"
    assert defaults["hfov_rad"] == "1.48"
    assert defaults["camera_yaw_rad"] == "0.0"
    assert defaults["scan_topic"] == "/scan"


def test_both_nodes_are_started_with_conditions():
    nodes = [e for e in _description().entities if isinstance(e, Node)]
    assert {n.node_executable for n in nodes} == {"detector_node", "fusion_node"}
    assert all(n.condition is not None for n in nodes)


def test_detector_exit_shuts_the_launch_down():
    handlers = [e for e in _description().entities if isinstance(e, RegisterEventHandler)]
    assert len(handlers) == 1


def test_car_stack_does_not_start_perception():
    text = _CAR_TELEOP.read_text()
    assert "perception" not in text.replace("racer_perception_", "")


@pytest.mark.skipif(_HAS_ULTRALYTICS, reason="ultralytics installed: would try to fetch weights")
def test_detector_without_ultralytics_exits_three_with_one_clear_error():
    pytest.importorskip("rclpy")
    ros2 = shutil.which("ros2")
    assert ros2 is not None, "ros2 CLI not on PATH; run under colcon test"
    proc = subprocess.run(
        [ros2, "run", "racer_perception", "detector_node"],
        env=dict(os.environ, ROS_DOMAIN_ID="120"),
        capture_output=True,
        text=True,
        timeout=60,
        check=False,
    )
    output = proc.stdout + proc.stderr
    assert proc.returncode == 3, output
    assert output.count("INSTALL_PERCEPTION=required") == 1, output
    assert "ERROR" in output


@pytest.mark.skipif(
    not _HAS_ULTRALYTICS or not os.environ.get("RACER_TEST_YOLO_WEIGHTS"),
    reason="needs ultralytics and RACER_TEST_YOLO_WEIGHTS (never downloaded in CI)",
)
def test_detector_starts_with_real_weights():
    ros2 = shutil.which("ros2")
    assert ros2 is not None
    proc = subprocess.Popen(
        [
            ros2, "run", "racer_perception", "detector_node", "--ros-args",
            "-p", f"model:={os.environ['RACER_TEST_YOLO_WEIGHTS']}",
        ],
        env=dict(os.environ, ROS_DOMAIN_ID="121"),
        stdout=subprocess.PIPE,
        stderr=subprocess.STDOUT,
        text=True,
    )  # fmt: skip
    try:
        proc.wait(timeout=15)
        pytest.fail(f"detector exited early with {proc.returncode}: {proc.stdout.read()}")
    except subprocess.TimeoutExpired:
        pass
    finally:
        proc.terminate()
        proc.wait(timeout=10)
