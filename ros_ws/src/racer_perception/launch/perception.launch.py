"""perception.launch.py -- detector_node + fusion_node (roadmap 5.1). OPTIONAL, NOT in car-stack.

Run it INSIDE the car-camera container (docker/car/README.md "Perception"), the one that has
the NVIDIA runtime and runs cameras.launch.py, from an image built with
INSTALL_PERCEPTION=required (ultralytics + PyTorch):

    ros2 launch racer_perception perception.launch.py device:=cuda debug_image:=true

car_teleop.launch.py does NOT include this file and nothing in the command path reads the
topics it publishes: the detector is a perception experiment on top of the camera, and car-stack
(the container that drives the car) stays free of torch and of the GPU runtime. Both
containers share the host network and ROS_DOMAIN_ID, so car-stack's recorder and Foxglove see
/perception/* published from car-camera (the recorder regex does not take them by default).

Starts:
  * detector_node (detector:=true): CompressedImage -> /perception/detections
  * fusion_node   (fusion:=true):   + /scan -> /perception/objects and /perception/target

`detector:=false` runs fusion alone (replaying recorded detections, or a different detector).
Without ultralytics detector_node exits with code 3 and a clear message, and since a launch
description cannot tell the two nodes apart on exit, that exit shuts the launch down
(OnProcessExit -> Shutdown) rather than leaving fusion_node running over nothing.

No physical constants here. The LiDAR mount yaw comes from vehicle_params in fusion_node; the
camera model values are launch arguments because no camera mount has been measured.
"""

from __future__ import annotations

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, EmitEvent, RegisterEventHandler
from launch.conditions import IfCondition
from launch.event_handlers import OnProcessExit
from launch.events import Shutdown
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue

#: Launch argument -> (default, description, node it is forwarded to, parameter name).
DETECTOR_ARGUMENTS = {
    "image_topic": (
        "/camera/usb/image_raw/compressed",
        "CompressedImage topic to run the detector on.",
        "image_topic",
    ),
    "model": ("yolov8n.pt", "YOLO weights: a bare name lives in model_cache_dir.", "model"),
    "device": ("cpu", "cpu or cuda (cuda needs --runtime nvidia and a CUDA torch).", "device"),
    "model_cache_dir": (
        "~/.cache/racer_perception",
        "Where weights are cached; downloaded here on first run, never committed.",
        "model_cache_dir",
    ),
    "rate_hz": ("10.0", "Maximum frames processed per second; the rest are dropped.", "rate_hz"),
    "score_threshold": (
        "0.25",
        "Minimum detector confidence to publish.",
        "score_threshold",
    ),
    "debug_image": (
        "false",
        "Publish /perception/debug_image/compressed with boxes drawn.",
        "publish_debug_image",
    ),
}
FUSION_ARGUMENTS = {
    "scan_topic": ("/scan", "LaserScan topic.", "scan_topic"),
    "image_width_px": ("1280", "Width of the images the detector sees, pixels.", "image_width_px"),
    "hfov_rad": ("1.48", "Camera horizontal field of view, rad (ELP, about 85 deg).", "hfov_rad"),
    "camera_yaw_rad": (
        "0.0",
        "Camera yaw relative to base_link, rad, left positive (not measured yet).",
        "camera_yaw_rad",
    ),
}


def _declare(table: dict) -> list[DeclareLaunchArgument]:
    return [
        DeclareLaunchArgument(name, default_value=default, description=description)
        for name, (default, description, _param) in table.items()
    ]


def _parameters(table: dict, types: dict) -> dict:
    """{node parameter name: launch argument}, coerced to the node's parameter type."""
    out = {}
    for name, (_default, _description, param) in table.items():
        out[param] = ParameterValue(LaunchConfiguration(name), value_type=types.get(name, str))
    return out


def generate_launch_description() -> LaunchDescription:
    detector_arg = DeclareLaunchArgument(
        "detector", default_value="true", description="Start detector_node (needs ultralytics)."
    )
    fusion_arg = DeclareLaunchArgument(
        "fusion", default_value="true", description="Start fusion_node (bearing and range)."
    )
    detector_params = _parameters(
        DETECTOR_ARGUMENTS, {"rate_hz": float, "score_threshold": float, "debug_image": bool}
    )
    detector = Node(
        package="racer_perception",
        executable="detector_node",
        name="detector_node",
        output="screen",
        parameters=[detector_params],
        condition=IfCondition(LaunchConfiguration("detector")),
    )
    fusion = Node(
        package="racer_perception",
        executable="fusion_node",
        name="fusion_node",
        output="screen",
        parameters=[
            _parameters(
                FUSION_ARGUMENTS,
                {"image_width_px": int, "hfov_rad": float, "camera_yaw_rad": float},
            )
        ],
        condition=IfCondition(LaunchConfiguration("fusion")),
    )
    stop_on_detector_exit = RegisterEventHandler(
        OnProcessExit(target_action=detector, on_exit=[EmitEvent(event=Shutdown())])
    )
    return LaunchDescription(
        [
            detector_arg,
            fusion_arg,
            *_declare(DETECTOR_ARGUMENTS),
            *_declare(FUSION_ARGUMENTS),
            detector,
            fusion,
            stop_on_detector_exit,
        ]
    )
