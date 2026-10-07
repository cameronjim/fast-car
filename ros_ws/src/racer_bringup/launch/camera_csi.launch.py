"""camera_csi.launch.py -- one IMX219 CSI camera through NVIDIA Argus and gscam. OPTIONAL.

Cameras are for detection experiments and training-data collection; they are outside the
project thesis (claude-docs/00-project-overview.md) and nothing in the command path reads
them. Hardware this was written for: a Waveshare IMX219-160 on the Orin Nano dev kit's CAM0
socket (sensor_id 0); a second IMX219 (sensor_id 1) is a second include of this file.

Publishes (sensor_msgs, interface names in claude-docs/04-architecture.md), N = sensor_id:

    /camera/csiN/image_raw              Image, rgb8
    /camera/csiN/image_raw/compressed   CompressedImage, JPEG (the one that gets recorded)
    /camera/csiN/camera_info            CameraInfo (uncalibrated until a calibration exists)

WHY ARGUS AND NOT V4L2. On Jetson the IMX219 is a raw Bayer sensor behind the ISP. Plain V4L2
on /dev/videoN gives undebayered 10-bit Bayer with no auto exposure or white balance; the
ISP is only reachable through libargus, i.e. the nvarguscamerasrc GStreamer element talking
to the host's nvargus-daemon over /tmp/argus_socket.

WHY gscam. Options considered for Humble on JetPack 6:
  * gscam (ros-drivers/gscam, ros2 branch): CHOSEN. Released into the ROS apt repo for Humble
    on arm64 (ros-humble-gscam 2.0.2, checked in the jammy arm64 package index on 2026-10-06),
    so it installs like foxglove_bridge with no source build. It takes an arbitrary GStreamer
    pipeline string, so nvarguscamerasrc works as long as the element is visible inside the
    container. Small, C++, publishes through image_transport (raw + compressed).
  * gscam2 (clydemcqueen/gscam2): same idea plus composable-node niceties nobody here needs,
    and not in the apt repo, so it would be another pinned source build like sllidar_ros2.
  * isaac_ros_argus_camera (NVIDIA Isaac ROS): rejected. It expects NVIDIA's Isaac ROS
    container and NITROS stack, a multi-gigabyte second ROS environment for a camera that is
    not part of the thesis.

THE PIPELINE (see `argus_pipeline`): nvarguscamerasrc -> NVMM NV12 at the requested mode ->
nvvidconv (hardware flip / colour conversion to BGRx) -> videoconvert (CPU, BGRx to RGB, which
is what gscam's rgb8 appsink caps ask for). gscam appends the appsink itself. width, height
and fps must name one of the sensor modes nvargus-daemon lists at start-up; 1280x720 at 60 is
one of them for the IMX219 on JetPack 6 (UNVERIFIED on this board until first power-up).

THE DRIVER is NOT an <exec_depend> of racer_bringup, same reasoning as usb_cam in
camera_usb.launch.py: this file only RUNS in the car image, in a container started with the
NVIDIA runtime and the Argus socket (docker/car/README.md "Cameras"). Its L1 test inspects the
launch description and never starts the node.

TOPIC NAMES. gscam publishes on the relative names camera/image_raw and camera/camera_info.
Remapped one by one (not namespaced) so that compressed_image_transport's parameter names stay
`camera.image_raw.jpeg_quality` / `camera.image_raw.enable_pub_plugins` (under a namespace
Humble prefixes them with a dot), and the compressed sub-topic gets its own remap because
image_transport derives it from the expanded base name. Only raw and compressed transports
are enabled.

QoS. use_sensor_data_qos true: best_effort, the convention for sensor data
(claude-docs/10-conventions.md). rosbag2 and Foxglove subscribe to match.

No physical constants. Camera mount poses are NOT needed for this bring-up and no transform is
published; frame_id is only a label until a mount is measured.
"""

from __future__ import annotations

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, OpaqueFunction
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node

#: The only image_transport plugins this camera publishes with.
ENABLED_TRANSPORTS = ["image_transport/raw", "image_transport/compressed"]

#: nvvidconv flip-method values: 0 none, 1 counter-clockwise 90, 2 rotate 180, 3 clockwise 90,
#: 4 horizontal flip, 5 upper-right diagonal, 6 vertical flip, 7 upper-left diagonal.
_FLIP_METHODS = range(8)

#: Launch argument name -> (default, description).
LAUNCH_ARGUMENTS = {
    "sensor_id": (
        "0",
        (
            "nvarguscamerasrc sensor-id. 0 = the first enabled CSI camera (CAM0 with the "
            "'Camera IMX219-A' overlay). Topics land under /camera/csi<sensor_id>."
        ),
    ),
    "width": ("1280", "Capture width, pixels. Must be an IMX219 sensor mode."),
    "height": ("720", "Capture height, pixels."),
    "fps": ("60", "Capture frame rate, frames per second."),
    "flip_method": (
        "0",
        (
            "nvvidconv flip-method, 0..7 (0 none, 2 rotate 180 for an upside-down mount, "
            "4 horizontal flip, 6 vertical flip)."
        ),
    ),
    "jpeg_quality": (
        "80",
        "JPEG quality (1-100) of /camera/csiN/image_raw/compressed. Sets the bag growth rate.",
    ),
}


def _int_in_range(name: str, text: str, low: int, high: int | None = None) -> int:
    try:
        value = int(text)
    except ValueError:
        raise RuntimeError(
            f"camera_csi.launch.py: {name} must be an integer, got {text!r}"
        ) from None
    if value < low or (high is not None and value > high):
        bound = f">= {low}" if high is None else f"{low}..{high}"
        raise RuntimeError(f"camera_csi.launch.py: {name} must be {bound}, got {value}")
    return value


def argus_pipeline(sensor_id: int, width: int, height: int, fps: int, flip_method: int) -> str:
    """The gscam_config GStreamer pipeline for one Argus camera, ending unlinked for gscam.

    Raises RuntimeError on an out-of-range flip_method or a non-positive size or rate.
    """
    if flip_method not in _FLIP_METHODS:
        raise RuntimeError(
            f"camera_csi.launch.py: flip_method must be 0..7 (nvvidconv flip-method), "
            f"got {flip_method}"
        )
    for name, value in (("width", width), ("height", height), ("fps", fps)):
        if value <= 0:
            raise RuntimeError(f"camera_csi.launch.py: {name} must be positive, got {value}")
    if sensor_id < 0:
        raise RuntimeError(f"camera_csi.launch.py: sensor_id must be >= 0, got {sensor_id}")
    return (
        f"nvarguscamerasrc sensor-id={sensor_id} ! "
        f"video/x-raw(memory:NVMM),width={width},height={height},"
        f"framerate={fps}/1,format=NV12 ! "
        f"nvvidconv flip-method={flip_method} ! "
        f"video/x-raw,width={width},height={height},format=BGRx ! "
        "videoconvert"
    )


def topic_prefix(sensor_id: int) -> str:
    """/camera/csi<sensor_id>, the namespace this camera's topics land under."""
    return f"/camera/csi{sensor_id}"


def csi_camera_node(
    *, sensor_id: int, width: int, height: int, fps: int, flip_method: int, quality: int
) -> Node:
    """The gscam node with the Argus pipeline and the /camera/csiN/* remaps."""
    prefix = topic_prefix(sensor_id)
    return Node(
        package="gscam",
        executable="gscam_node",
        name=f"camera_csi{sensor_id}",
        output="screen",
        parameters=[
            {
                "gscam_config": argus_pipeline(sensor_id, width, height, fps, flip_method),
                "camera_name": f"csi{sensor_id}",
                "frame_id": f"camera_csi{sensor_id}",
                "image_encoding": "rgb8",
                # A live source: syncing the appsink to the pipeline clock only drops frames.
                "sync_sink": False,
                "use_gst_timestamps": False,
                "use_sensor_data_qos": True,
                "camera.image_raw.jpeg_quality": quality,
                "camera.image_raw.enable_pub_plugins": ENABLED_TRANSPORTS,
            }
        ],
        remappings=[
            ("camera/image_raw", f"{prefix}/image_raw"),
            ("camera/image_raw/compressed", f"{prefix}/image_raw/compressed"),
            ("camera/camera_info", f"{prefix}/camera_info"),
            ("set_camera_info", f"{prefix}/set_camera_info"),
        ],
    )


def _launch_setup(context, *args, **kwargs):
    def arg(name: str) -> str:
        return LaunchConfiguration(name).perform(context)

    return [
        csi_camera_node(
            sensor_id=_int_in_range("sensor_id", arg("sensor_id"), 0),
            width=_int_in_range("width", arg("width"), 1),
            height=_int_in_range("height", arg("height"), 1),
            fps=_int_in_range("fps", arg("fps"), 1),
            flip_method=_int_in_range("flip_method", arg("flip_method"), 0, 7),
            quality=_int_in_range("jpeg_quality", arg("jpeg_quality"), 1, 100),
        )
    ]


def generate_launch_description() -> LaunchDescription:
    declared = [
        DeclareLaunchArgument(name, default_value=default, description=description)
        for name, (default, description) in LAUNCH_ARGUMENTS.items()
    ]
    return LaunchDescription([*declared, OpaqueFunction(function=_launch_setup)])
