"""L1 checks of the optional camera launches and car_teleop.launch.py's camera wiring.

Loads the SOURCE launch files as modules and inspects the launch descriptions they return; no
process is started, so usb_cam and gscam (installed only in the car image, see each launch
file's docstring) do not need to be installed. Same pattern as test_lidar_launch.py. What it
pins:

  * camera_usb.launch.py: arguments and defaults (1280x720 at 60, MJPEG decoded by usb_cam),
    the usb_cam parameters, the /camera/usb/* remaps, and that the two pixel_format names that
    do not work in usb_cam 0.8.1 are refused by name;
  * camera_csi.launch.py: arguments and defaults, the exact nvarguscamerasrc pipeline handed
    to gscam, the /camera/csiN/* remaps for sensor 0 and 1, and range checks;
  * cameras.launch.py: both includes, their on/off conditions, and that its forwarded defaults
    equal the child files' own defaults;
  * car_teleop.launch.py: `cameras` defaults to false and includes cameras.launch.py only when
    true; the recorder regex takes /camera/.*/compressed and NOT the raw images.

No camera publishes a transform: mount poses are not part of this bring-up.
"""

from __future__ import annotations

import importlib.util
import pathlib
import re

import pytest
from launch import LaunchContext
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, OpaqueFunction
from launch_ros.actions import Node
from launch_ros.utilities import evaluate_parameters

_LAUNCH_DIR = pathlib.Path(__file__).resolve().parents[1] / "launch"


def _load(name: str):
    spec = importlib.util.spec_from_file_location(name.replace(".", "_"), _LAUNCH_DIR / name)
    module = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(module)
    return module


def _perform(value, context):
    if hasattr(value, "evaluate"):
        return value.evaluate(context)
    if hasattr(value, "perform"):
        return value.perform(context)
    if isinstance(value, list | tuple) and value and all(hasattr(v, "perform") for v in value):
        return "".join(v.perform(context) for v in value)
    return value


def _defaults(entities):
    context = LaunchContext()
    return {
        e.name: "".join(s.perform(context) for s in e.default_value)
        for e in entities
        if isinstance(e, DeclareLaunchArgument)
    }


def _context(**configs) -> LaunchContext:
    context = LaunchContext()
    context.launch_configurations.update(configs)
    return context


def _node_parameters(node: Node, context: LaunchContext) -> dict:
    merged = {}
    for entry in evaluate_parameters(context, node._Node__parameters):
        merged.update(entry)
    return merged


def _remaps(node: Node, context: LaunchContext) -> list[tuple[str, str]]:
    return [(_perform(src, context), _perform(dst, context)) for src, dst in node._Node__remappings]


def _includes(entities) -> dict[str, IncludeLaunchDescription]:
    """File name -> include. Loading each source resolves its location (and proves the file at
    that path imports cleanly); before that, `location` is still a substitution."""
    found = {}
    for e in entities:
        if isinstance(e, IncludeLaunchDescription):
            e.launch_description_source.get_launch_description(LaunchContext())
            found[pathlib.Path(e.launch_description_source.location).name] = e
    return found


def _forwarded(include: IncludeLaunchDescription, context: LaunchContext) -> dict[str, str]:
    return {_perform(k, LaunchContext()): _perform(v, context) for k, v in include.launch_arguments}


# --- camera_usb.launch.py ---------------------------------------------------------------

_USB_DEFAULTS = {
    "device": "/dev/video0",
    "width": "1280",
    "height": "720",
    "fps": "60",
    "pixel_format": "mjpeg2rgb",
    "av_device_format": "YUV422P",
    "jpeg_quality": "80",
}


def _usb_node(**overrides) -> Node:
    module = _load("camera_usb.launch.py")
    nodes = module._launch_setup(_context(**{**_USB_DEFAULTS, **overrides}))
    assert len(nodes) == 1
    return nodes[0]


def test_usb_declares_arguments_with_defaults():
    entities = _load("camera_usb.launch.py").generate_launch_description().entities
    assert _defaults(entities) == _USB_DEFAULTS
    assert any(isinstance(e, OpaqueFunction) for e in entities)


def test_usb_node_parameters_and_topics():
    node = _usb_node()
    assert (node.node_package, node.node_executable) == ("usb_cam", "usb_cam_node_exe")
    context = _context(**_USB_DEFAULTS)
    params = _node_parameters(node, context)
    assert params["video_device"] == "/dev/video0"
    assert params["image_width"] == 1280
    assert params["image_height"] == 720
    assert isinstance(params["image_width"], int)
    # usb_cam declares framerate as a double; an int here would be rejected at start-up.
    assert params["framerate"] == 60.0
    assert isinstance(params["framerate"], float)
    assert params["pixel_format"] == "mjpeg2rgb"
    assert params["av_device_format"] == "YUV422P"
    assert params["io_method"] == "mmap"
    assert params["frame_id"] == "camera_usb"
    assert params["brightness"] == -1
    assert params["image_raw.jpeg_quality"] == 80
    assert list(params["image_raw.enable_pub_plugins"]) == [
        "image_transport/raw",
        "image_transport/compressed",
    ]
    assert _remaps(node, context) == [
        ("image_raw", "/camera/usb/image_raw"),
        ("image_raw/compressed", "/camera/usb/image_raw/compressed"),
        ("camera_info", "/camera/usb/camera_info"),
        ("set_camera_info", "/camera/usb/set_camera_info"),
        ("set_capture", "/camera/usb/set_capture"),
    ]


def test_usb_follows_non_default_arguments():
    node = _usb_node(device="/dev/video1", width="1920", height="1200", fps="120")
    params = _node_parameters(node, _context())
    assert params["video_device"] == "/dev/video1"
    assert (params["image_width"], params["image_height"]) == (1920, 1200)
    assert params["framerate"] == 120.0


@pytest.mark.parametrize("name", ["mjpeg", "raw_mjpeg"])
def test_usb_refuses_pixel_formats_that_do_not_work_in_usb_cam_0_8_1(name):
    with pytest.raises(RuntimeError, match=f"pixel_format '{name}' refused"):
        _usb_node(pixel_format=name)


def test_usb_refuses_unknown_pixel_format():
    with pytest.raises(RuntimeError, match="is not one of"):
        _usb_node(pixel_format="h264")


@pytest.mark.parametrize("fmt", ["yuyv2rgb", "uyvy2rgb", "yuyv", "uyvy", "rgb8", "mono8"])
def test_usb_accepts_the_other_usb_cam_formats(fmt):
    assert _node_parameters(_usb_node(pixel_format=fmt), _context())["pixel_format"] == fmt


@pytest.mark.parametrize(
    ("arg", "value", "message"),
    [
        ("width", "0", "width must be positive"),
        ("height", "-720", "height must be positive"),
        ("fps", "sixty", "fps must be an integer"),
        ("jpeg_quality", "101", "jpeg_quality must be 1..100"),
        ("jpeg_quality", "0", "jpeg_quality must be positive"),
    ],
)
def test_usb_refuses_bad_numbers(arg, value, message):
    with pytest.raises(RuntimeError, match=message):
        _usb_node(**{arg: value})


# --- camera_csi.launch.py ---------------------------------------------------------------

_CSI_DEFAULTS = {
    "sensor_id": "0",
    "width": "1280",
    "height": "720",
    "fps": "60",
    "flip_method": "0",
    "jpeg_quality": "80",
}

_DEFAULT_PIPELINE = (
    "nvarguscamerasrc sensor-id=0 ! "
    "video/x-raw(memory:NVMM),width=1280,height=720,framerate=60/1,format=NV12 ! "
    "nvvidconv flip-method=0 ! "
    "video/x-raw,width=1280,height=720,format=BGRx ! "
    "videoconvert"
)


def _csi_node(**overrides) -> Node:
    module = _load("camera_csi.launch.py")
    nodes = module._launch_setup(_context(**{**_CSI_DEFAULTS, **overrides}))
    assert len(nodes) == 1
    return nodes[0]


def test_csi_declares_arguments_with_defaults():
    entities = _load("camera_csi.launch.py").generate_launch_description().entities
    assert _defaults(entities) == _CSI_DEFAULTS
    assert any(isinstance(e, OpaqueFunction) for e in entities)


def test_csi_pipeline_for_the_defaults():
    module = _load("camera_csi.launch.py")
    assert module.argus_pipeline(0, 1280, 720, 60, 0) == _DEFAULT_PIPELINE


def test_csi_pipeline_follows_its_arguments():
    module = _load("camera_csi.launch.py")
    pipeline = module.argus_pipeline(1, 1920, 1080, 30, 2)
    assert pipeline.startswith("nvarguscamerasrc sensor-id=1 ! ")
    assert "width=1920,height=1080,framerate=30/1,format=NV12" in pipeline
    assert "nvvidconv flip-method=2 ! " in pipeline
    assert "video/x-raw,width=1920,height=1080,format=BGRx" in pipeline
    # gscam links its own appsink to the one unlinked src pad, so no sink in the string.
    assert pipeline.endswith("videoconvert")
    assert "appsink" not in pipeline


@pytest.mark.parametrize(
    ("args", "message"),
    [
        ((0, 1280, 720, 60, 8), "flip_method must be 0..7"),
        ((0, 1280, 720, 60, -1), "flip_method must be 0..7"),
        ((0, 0, 720, 60, 0), "width must be positive"),
        ((0, 1280, 0, 60, 0), "height must be positive"),
        ((0, 1280, 720, 0, 0), "fps must be positive"),
        ((-1, 1280, 720, 60, 0), "sensor_id must be >= 0"),
    ],
)
def test_csi_pipeline_refuses_out_of_range(args, message):
    module = _load("camera_csi.launch.py")
    with pytest.raises(RuntimeError, match=message):
        module.argus_pipeline(*args)


def test_csi_node_parameters_and_topics():
    node = _csi_node()
    assert (node.node_package, node.node_executable) == ("gscam", "gscam_node")
    context = _context(**_CSI_DEFAULTS)
    params = _node_parameters(node, context)
    assert params["gscam_config"] == _DEFAULT_PIPELINE
    assert params["image_encoding"] == "rgb8"
    assert params["camera_name"] == "csi0"
    assert params["frame_id"] == "camera_csi0"
    assert params["sync_sink"] is False
    assert params["use_gst_timestamps"] is False
    assert params["use_sensor_data_qos"] is True
    assert params["camera.image_raw.jpeg_quality"] == 80
    assert list(params["camera.image_raw.enable_pub_plugins"]) == [
        "image_transport/raw",
        "image_transport/compressed",
    ]
    assert _remaps(node, context) == [
        ("camera/image_raw", "/camera/csi0/image_raw"),
        ("camera/image_raw/compressed", "/camera/csi0/image_raw/compressed"),
        ("camera/camera_info", "/camera/csi0/camera_info"),
        ("set_camera_info", "/camera/csi0/set_camera_info"),
    ]


def test_csi_second_sensor_lands_under_csi1():
    node = _csi_node(sensor_id="1")
    context = _context()
    assert _perform(node._Node__node_name, context) == "camera_csi1"
    assert _remaps(node, context)[0] == ("camera/image_raw", "/camera/csi1/image_raw")
    assert "sensor-id=1" in _node_parameters(node, context)["gscam_config"]


@pytest.mark.parametrize(
    ("arg", "value", "message"),
    [
        ("flip_method", "9", "flip_method must be 0..7"),
        ("sensor_id", "-1", "sensor_id must be >= 0"),
        ("fps", "60.5", "fps must be an integer"),
        ("jpeg_quality", "0", "jpeg_quality must be 1..100"),
    ],
)
def test_csi_refuses_bad_arguments(arg, value, message):
    with pytest.raises(RuntimeError, match=message):
        _csi_node(**{arg: value})


def test_camera_launches_start_one_node_each_and_no_transform():
    for node in (_usb_node(), _csi_node()):
        assert node.node_package in ("usb_cam", "gscam")
        assert node.node_executable != "static_transform_publisher"


# --- cameras.launch.py ------------------------------------------------------------------


def test_cameras_defaults_match_the_child_files():
    entities = _load("cameras.launch.py").generate_launch_description().entities
    defaults = _defaults(entities)
    assert defaults["usb"] == "true"
    assert defaults["csi"] == "true"
    module = _load("cameras.launch.py")
    for name, (child, _default, _desc) in module.USB_FORWARDED.items():
        assert defaults[name] == _USB_DEFAULTS[child], name
    for name, (child, _default, _desc) in module.CSI_FORWARDED.items():
        assert defaults[name] == _CSI_DEFAULTS[child], name


def test_cameras_includes_both_with_conditions_and_forwarding():
    entities = _load("cameras.launch.py").generate_launch_description().entities
    includes = _includes(entities)
    assert set(includes) == {"camera_usb.launch.py", "camera_csi.launch.py"}
    for include in includes.values():
        assert pathlib.Path(include.launch_description_source.location).is_file()

    usb, csi = includes["camera_usb.launch.py"], includes["camera_csi.launch.py"]
    assert usb.condition.evaluate(_context(usb="true")) is True
    assert usb.condition.evaluate(_context(usb="false")) is False
    assert csi.condition.evaluate(_context(csi="true")) is True
    assert csi.condition.evaluate(_context(csi="false")) is False

    configs = {
        "usb_device": "/dev/video1",
        "usb_width": "1920",
        "usb_height": "1200",
        "usb_fps": "120",
        "csi_width": "1640",
        "csi_height": "1232",
        "csi_fps": "30",
        "csi_flip_method": "2",
    }
    context = _context(**configs)
    assert _forwarded(usb, context) == {
        "device": "/dev/video1",
        "width": "1920",
        "height": "1200",
        "fps": "120",
    }
    assert _forwarded(csi, context) == {
        "sensor_id": "0",
        "width": "1640",
        "height": "1232",
        "fps": "30",
        "flip_method": "2",
    }


# --- car_teleop.launch.py `cameras` argument and the recorder regex -----------------------


def test_car_teleop_cameras_defaults_off():
    defaults = _defaults(_load("car_teleop.launch.py").generate_launch_description().entities)
    assert defaults["cameras"] == "false"
    assert defaults["camera_usb_device"] == "/dev/video0"


def test_car_teleop_includes_cameras_launch_only_when_asked():
    entities = _load("car_teleop.launch.py").generate_launch_description().entities
    include = _includes(entities)["cameras.launch.py"]
    assert pathlib.Path(include.launch_description_source.location).is_file()
    assert include.condition.evaluate(_context(cameras="false")) is False
    assert include.condition.evaluate(_context(cameras="true")) is True
    assert _forwarded(include, _context(camera_usb_device="/dev/video1")) == {
        "usb_device": "/dev/video1"
    }


_RECORDED = [
    "/camera/usb/image_raw/compressed",
    "/camera/csi0/image_raw/compressed",
    "/camera/csi1/image_raw/compressed",
    # Everything recorded before the cameras still is.
    "/drive_raw",
    "/drive",
    "/safety/events",
    "/teleop/cmd_vel",
    "/telemetry/rail_voltage_v",
    "/scan",
    "/rosout",
    "/parameter_events",
]

_NOT_RECORDED = [
    "/camera/usb/image_raw",
    "/camera/csi0/image_raw",
    "/camera/usb/camera_info",
    "/camera/usb/image_raw/compressedDepth",
    "/camera/usb/image_raw/theora",
    "/camera/usb/image_raw/compressed/parameter_descriptions",
    "/image_raw/compressed",
]


@pytest.mark.parametrize("topic", _RECORDED)
def test_recorder_regex_takes(topic):
    regex = _load("car_teleop.launch.py")._RECORDED_TOPIC_REGEX
    assert re.search(regex, topic), topic


@pytest.mark.parametrize("topic", _NOT_RECORDED)
def test_recorder_regex_leaves_out(topic):
    regex = _load("car_teleop.launch.py")._RECORDED_TOPIC_REGEX
    assert re.search(regex, topic) is None, topic
