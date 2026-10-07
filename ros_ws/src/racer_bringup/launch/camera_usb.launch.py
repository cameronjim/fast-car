"""camera_usb.launch.py -- the USB (UVC) camera through usb_cam. OPTIONAL subsystem.

Cameras are for detection experiments and training-data collection; they are outside the
project thesis (claude-docs/00-project-overview.md) and nothing in the command path reads
them. Hardware this was written for: an ELP AR0234 global-shutter UVC camera on USB 3 (MJPEG,
up to 1920x1200 at 120 fps). Run it on its own (docs/notes/first-boot-runbook.md "Cameras
first power-up"), through cameras.launch.py in the separate car-camera container, or through
car_teleop.launch.py with cameras:=true.

Publishes (sensor_msgs, interface names in claude-docs/04-architecture.md):

    /camera/usb/image_raw              Image, rgb8, decoded from the camera's MJPEG
    /camera/usb/image_raw/compressed   CompressedImage, JPEG (the one that gets recorded)
    /camera/usb/camera_info            CameraInfo (uncalibrated until a calibration exists)

THE DRIVER. ros-humble-usb-cam 0.8.1 from the ROS apt repo (docker/car/Dockerfile). It is
NOT an <exec_depend> of racer_bringup: CI's ros-dev image does not carry it, and nothing in
the core stack needs it, the same reasoning as sllidar_ros2 in lidar.launch.py. So this file
only RUNS in the car image; its L1 test inspects the launch description and never starts the
node.

PIXEL FORMAT. The launch argument defaults to `mjpeg2rgb`, which is how usb_cam 0.8.1 spells
"ask the camera for MJPEG, decode it to rgb8 on the CPU". The literal name `mjpeg` is not
accepted by 0.8.1 (its format table has `raw_mjpeg` and `mjpeg2rgb` only, and the node
throws on anything else), and `raw_mjpeg` copies a fixed width x height x channels buffer per
frame into a sensor_msgs/Image rather than the JPEG bytes, so neither gives a usable stream.
This launch refuses both by name with that explanation instead of letting the node die.
The compressed topic is then produced by image_transport's compressed plugin re-encoding the
rgb8 frame (compressed_image_transport, `jpeg_quality` below). That is a decode plus an encode
per frame on the CPU: measure the load on the Jetson (UNVERIFIED at 720p60).

av_device_format tells usb_cam's decoder what chroma layout the camera's JPEGs use. Its
default here is usb_cam's own, YUV422P. If the picture comes out with wrong colours or
stripes, the camera is sending 4:2:0 JPEGs: relaunch with av_device_format:=YUV420P.

TOPIC NAMES. usb_cam publishes on the relative names image_raw / camera_info. They are
remapped one by one rather than pushed under a namespace, because in Humble's
compressed_image_transport the plugin parameter names under a namespace come out with a
leading dot (`.image_raw.jpeg_quality`); in the root namespace they are the usable
`image_raw.jpeg_quality` / `image_raw.enable_pub_plugins`. image_transport resolves the
compressed sub-topic from the expanded base name, not the remapped one, so the compressed
topic needs its own remap. Only the raw and compressed transports are enabled, so no theora
or compressedDepth topics appear.

DEVICE PATH. usb_cam 0.8.1 checks `device` against the V4L2 devices it enumerates from
/sys/class/video4linux (keyed /dev/<DEVNAME>) and shuts down on anything else, and it
mangles symlinks (it prefixes the link target with /dev/). So `device` must be the plain
/dev/videoN, mapped into the container at the SAME path. Stable names come later, if ever,
from resolving /dev/v4l/by-id/ on the host before docker run (the runbook does this).

QoS. usb_cam publishes reliable, KeepLast(100); not configurable in 0.8.1. A best_effort
subscriber (camera_check, Foxglove) matches it.

No physical constants. Camera mount poses are NOT needed for this bring-up and no transform is
published; frame_id is only a label until a mount is measured.
"""

from __future__ import annotations

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, OpaqueFunction
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node

#: Namespace every topic of this camera lands under.
TOPIC_PREFIX = "/camera/usb"

#: frame_id stamped on the images: a label only, no transform is published. Deliberately not a
#: launch argument: Humble's IncludeLaunchDescription does not scope launch configurations, so
#: a `frame_id` argument here would silently pick up lidar.launch.py's `frame_id` (laser) when
#: car_teleop.launch.py includes both.
FRAME_ID = "camera_usb"

#: The only image_transport plugins this camera publishes with.
ENABLED_TRANSPORTS = ["image_transport/raw", "image_transport/compressed"]

#: usb_cam 0.8.1 pixel_format names that decode MJPEG or pass raw frames through usefully.
ACCEPTED_PIXEL_FORMATS = ("mjpeg2rgb", "yuyv2rgb", "uyvy2rgb", "yuyv", "uyvy", "rgb8", "mono8")

#: Names that look right but do not work in usb_cam 0.8.1, with the reason.
_REFUSED_PIXEL_FORMATS = {
    "mjpeg": "usb_cam 0.8.1 has no format called 'mjpeg' and throws on it",
    "raw_mjpeg": (
        "usb_cam 0.8.1 copies a fixed-size buffer per frame into a sensor_msgs/Image instead "
        "of publishing the JPEG bytes"
    ),
}

#: Launch argument name -> (default, description).
LAUNCH_ARGUMENTS = {
    "device": (
        "/dev/video0",
        (
            "V4L2 capture node of the USB camera. With the IMX219 overlay enabled the CSI sensor "
            "is usually /dev/video0 and the USB camera /dev/video1 (plus a metadata node); "
            "check with `v4l2-ctl --list-devices`. Must be the plain /dev/videoN name, the same "
            "inside the container as on the host (docker run --device /dev/videoN): usb_cam "
            "0.8.1 only accepts a device it finds under /sys/class/video4linux, so a "
            "/dev/v4l/by-id/ path is rejected. Use by-id on the host to find N."
        ),
    ),
    "width": ("1280", "Capture width, pixels. Must be a mode the camera lists for MJPEG."),
    "height": ("720", "Capture height, pixels."),
    "fps": ("60", "Capture frame rate, frames per second (usb_cam's framerate)."),
    "pixel_format": (
        "mjpeg2rgb",
        (
            "usb_cam pixel_format. mjpeg2rgb = MJPEG from the camera, decoded to rgb8. See the "
            "module docstring for why not 'mjpeg' or 'raw_mjpeg'."
        ),
    ),
    "av_device_format": (
        "YUV422P",
        "Chroma layout of the camera's JPEGs for usb_cam's decoder. YUV420P if colours are off.",
    ),
    "jpeg_quality": (
        "80",
        "JPEG quality (1-100) of /camera/usb/image_raw/compressed. Sets the bag growth rate.",
    ),
}


def _positive_int(name: str, text: str) -> int:
    try:
        value = int(text)
    except ValueError:
        raise RuntimeError(
            f"camera_usb.launch.py: {name} must be an integer, got {text!r}"
        ) from None
    if value <= 0:
        raise RuntimeError(f"camera_usb.launch.py: {name} must be positive, got {value}")
    return value


def validate_pixel_format(pixel_format: str) -> str:
    """Return pixel_format if usb_cam 0.8.1 can use it; raise RuntimeError naming why not."""
    if pixel_format in _REFUSED_PIXEL_FORMATS:
        raise RuntimeError(
            f"camera_usb.launch.py: pixel_format {pixel_format!r} refused: "
            f"{_REFUSED_PIXEL_FORMATS[pixel_format]}. Use mjpeg2rgb for an MJPEG camera."
        )
    if pixel_format not in ACCEPTED_PIXEL_FORMATS:
        raise RuntimeError(
            f"camera_usb.launch.py: pixel_format {pixel_format!r} is not one of "
            f"{', '.join(ACCEPTED_PIXEL_FORMATS)}"
        )
    return pixel_format


def jpeg_quality(text: str) -> int:
    """JPEG quality as compressed_image_transport accepts it, 1..100."""
    value = _positive_int("jpeg_quality", text)
    if value > 100:
        raise RuntimeError(f"camera_usb.launch.py: jpeg_quality must be 1..100, got {value}")
    return value


def usb_camera_node(
    *,
    device: str,
    width: int,
    height: int,
    fps: int,
    pixel_format: str,
    av_device_format: str,
    quality: int,
) -> Node:
    """The usb_cam node with its parameters and the /camera/usb/* remaps."""
    return Node(
        package="usb_cam",
        executable="usb_cam_node_exe",
        name="camera_usb",
        output="screen",
        parameters=[
            {
                "video_device": device,
                "image_width": width,
                "image_height": height,
                "framerate": float(fps),
                "pixel_format": validate_pixel_format(pixel_format),
                "av_device_format": av_device_format,
                "io_method": "mmap",
                "camera_name": "usb",
                "frame_id": FRAME_ID,
                # usb_cam 0.8.1 otherwise writes brightness 50 to the camera on start; -1 is
                # its "leave alone" value, so the camera keeps its own setting.
                "brightness": -1,
                "image_raw.jpeg_quality": quality,
                "image_raw.enable_pub_plugins": ENABLED_TRANSPORTS,
            }
        ],
        remappings=[
            ("image_raw", f"{TOPIC_PREFIX}/image_raw"),
            ("image_raw/compressed", f"{TOPIC_PREFIX}/image_raw/compressed"),
            ("camera_info", f"{TOPIC_PREFIX}/camera_info"),
            ("set_camera_info", f"{TOPIC_PREFIX}/set_camera_info"),
            ("set_capture", f"{TOPIC_PREFIX}/set_capture"),
        ],
    )


def _launch_setup(context, *args, **kwargs):
    def arg(name: str) -> str:
        return LaunchConfiguration(name).perform(context)

    return [
        usb_camera_node(
            device=arg("device"),
            width=_positive_int("width", arg("width")),
            height=_positive_int("height", arg("height")),
            fps=_positive_int("fps", arg("fps")),
            pixel_format=arg("pixel_format"),
            av_device_format=arg("av_device_format"),
            quality=jpeg_quality(arg("jpeg_quality")),
        )
    ]


def generate_launch_description() -> LaunchDescription:
    declared = [
        DeclareLaunchArgument(name, default_value=default, description=description)
        for name, (default, description) in LAUNCH_ARGUMENTS.items()
    ]
    return LaunchDescription([*declared, OpaqueFunction(function=_launch_setup)])
