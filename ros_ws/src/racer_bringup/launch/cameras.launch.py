"""cameras.launch.py -- every camera at once, for the separate car-camera container. OPTIONAL.

Cameras are for detection experiments and training-data collection, outside the project
thesis. This file includes camera_usb.launch.py and camera_csi.launch.py (sensor 0) with an
on/off argument each, so one command starts both in the car-camera container
(docker/car/README.md "Cameras"):

    ros2 launch racer_bringup cameras.launch.py usb_device:=/dev/video1

car_teleop.launch.py includes THIS file under cameras:=true, for the less preferred case of
running the cameras inside car-stack itself.

Why a separate container is the preferred shape: the CSI camera needs the NVIDIA container
runtime and the host's Argus socket, and car-stack is deliberately started without either
(docs/notes/first-boot-runbook.md "Start the stack"). Both containers run on the host network
with the same ROS_DOMAIN_ID (baked into the image), so car-stack's recorder and Foxglove
bridge see /camera/* published from car-camera.

ARGUMENT NAMES are prefixed (usb_*, csi_*) and every per-camera value is forwarded explicitly
to its include. Humble's IncludeLaunchDescription does not scope launch configurations, so a
bare `width:=1920` would reach BOTH child files (the AR0234's 1920x1200 is not an IMX219
mode); forwarding sets each child's value right before that child runs. The defaults equal
the child files' own defaults (the L1 test pins that).

LAP COUNTER (`lap_counter:=true`, default false, so nothing changes for existing runs): also runs
racer_perception's lap_counter_node on one camera's compressed topic (`lap_image_topic`, default
the USB camera), publishing /lap/event and /lap/status. The run's recorder takes both. Its
thresholds are forwarded as lap_* arguments; every other parameter keeps the node's default (see
racer_perception/launch/lap_counter.launch.py and docs/notes/lap-counter-2026-11-12.md).

Recording is NOT started here: the run's bag belongs to car_teleop.launch.py's recorder,
whose regex takes /camera/.*/compressed and leaves the raw images out. To collect camera data
without the drive stack, record by hand (docs/notes/first-boot-runbook.md "Cameras first
power-up").

No physical constants, and camera mount poses are not needed for this bring-up.
"""

from __future__ import annotations

import os

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue

_LAUNCH_DIR = os.path.dirname(os.path.abspath(__file__))
#: Found next to this file (source tree and installed share alike), as car_teleop.launch.py
#: finds lidar.launch.py, so the L1 tests need no installed racer_bringup.
USB_LAUNCH_FILE = os.path.join(_LAUNCH_DIR, "camera_usb.launch.py")
CSI_LAUNCH_FILE = os.path.join(_LAUNCH_DIR, "camera_csi.launch.py")

#: This file's argument -> (child argument it is forwarded as, default, description).
USB_FORWARDED = {
    "usb_device": (
        "device",
        "/dev/video0",
        (
            "V4L2 node of the USB camera. With the IMX219 overlay enabled this is usually "
            "/dev/video1: check `v4l2-ctl --list-devices`."
        ),
    ),
    "usb_width": ("width", "1280", "USB camera capture width, pixels."),
    "usb_height": ("height", "720", "USB camera capture height, pixels."),
    "usb_fps": ("fps", "60", "USB camera frame rate, frames per second."),
}
CSI_FORWARDED = {
    "csi_width": ("width", "1280", "CSI camera 0 capture width, pixels (an IMX219 mode)."),
    "csi_height": ("height", "720", "CSI camera 0 capture height, pixels."),
    "csi_fps": ("fps", "60", "CSI camera 0 frame rate, frames per second."),
    "csi_flip_method": ("flip_method", "0", "CSI camera 0 nvvidconv flip-method, 0..7."),
}


#: This file's argument -> (lap_counter_node parameter, default, description, type). The
#: defaults equal lap_counter_node's own (racer_perception's test pins that).
LAP_FORWARDED = {
    "lap_image_topic": (
        "image_topic",
        "/camera/usb/image_raw/compressed",
        "CompressedImage topic the lap counter watches.",
        str,
    ),
    "lap_similarity_threshold": (
        "similarity_threshold",
        "0.8",
        "Lap counter: count a lap at a similarity peak at or above this.",
        float,
    ),
    "lap_rearm_threshold": (
        "rearm_threshold",
        "0.55",
        "Lap counter: similarity must drop below this before the next lap can count.",
        float,
    ),
    "lap_min_lap_time_s": ("min_lap_time_s", "6.0", "Lap counter: shortest lap, seconds.", float),
    "lap_min_lap_distance_m": (
        "min_lap_distance_m",
        "3.0",
        "Lap counter: shortest lap, metres of odometry.",
        float,
    ),
}


def _declare(table: dict) -> list[DeclareLaunchArgument]:
    return [
        DeclareLaunchArgument(name, default_value=default, description=description)
        for name, (_child, default, description) in table.items()
    ]


def _forward(table: dict) -> list[tuple[str, LaunchConfiguration]]:
    return [(child, LaunchConfiguration(name)) for name, (child, _d, _desc) in table.items()]


def generate_launch_description() -> LaunchDescription:
    usb_arg = DeclareLaunchArgument(
        "usb",
        default_value="true",
        description="Start the USB camera (camera_usb.launch.py, /camera/usb/*).",
    )
    csi_arg = DeclareLaunchArgument(
        "csi",
        default_value="true",
        description=(
            "Start CSI camera 0 (camera_csi.launch.py sensor_id 0, /camera/csi0/*). Needs the "
            "NVIDIA runtime and /tmp/argus_socket in this container."
        ),
    )
    usb_camera = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(USB_LAUNCH_FILE),
        launch_arguments=_forward(USB_FORWARDED),
        condition=IfCondition(LaunchConfiguration("usb")),
    )
    csi_camera = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(CSI_LAUNCH_FILE),
        launch_arguments=[("sensor_id", "0"), *_forward(CSI_FORWARDED)],
        condition=IfCondition(LaunchConfiguration("csi")),
    )
    lap_arg = DeclareLaunchArgument(
        "lap_counter",
        default_value="false",
        description="Also start lap_counter_node (camera lap counter, /lap/event, /lap/status).",
    )
    lap_args = [
        DeclareLaunchArgument(name, default_value=default, description=description)
        for name, (_param, default, description, _type) in LAP_FORWARDED.items()
    ]
    lap_counter = Node(
        package="racer_perception",
        executable="lap_counter_node",
        name="lap_counter_node",
        output="screen",
        parameters=[
            {
                param: ParameterValue(LaunchConfiguration(name), value_type=value_type)
                for name, (param, _default, _description, value_type) in LAP_FORWARDED.items()
            }
        ],
        condition=IfCondition(LaunchConfiguration("lap_counter")),
    )
    return LaunchDescription(
        [
            usb_arg,
            csi_arg,
            lap_arg,
            *_declare(USB_FORWARDED),
            *_declare(CSI_FORWARDED),
            *lap_args,
            usb_camera,
            csi_camera,
            lap_counter,
        ]
    )
