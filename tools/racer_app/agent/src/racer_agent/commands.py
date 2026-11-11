"""Every command the agent can run, built from validated pieces. This is the whitelist.

The docker flags are the ones the runbook uses today (docs/notes/first-boot-runbook.md,
docker/car/README.md "Cameras"); nothing here invents a new container flag. The only thing
that varies is which `--device` lines are present (docker refuses to start a container naming
a device that is not there) and the `ros2 launch` arguments, and those come from the closed
sets below.

Nothing the HTTP layer receives is ever passed through as free text: a profile must be a key
of a table in this file, a map must be a file the agent found itself, a name must match
`NAME_RE`. `shlex.quote` is applied anyway to everything that lands in a `bash -lc` string.
"""

from __future__ import annotations

import re
import shlex
from collections.abc import Sequence
from dataclasses import dataclass
from pathlib import Path

from .hostinfo import HostInfo

# -- closed sets -------------------------------------------------------------------------

#: Profile names of ros_ws/src/racer_control/launch/gap_follow.launch.py and park.launch.py.
#: tests/test_whitelist.py checks these against the launch files so they cannot drift.
GAP_FOLLOW_PROFILES = ("none", "floor-2026-10-06", "lane-2026-10-25")
PARK_PROFILES = ("none", "floor-2026-10-07")

DRIVE_MODES = ("gap_follow", "wall_follow", "tracker", "park", "ml_policy", "spiderbot")

#: Pose rules of racer_apps' spiderbot (racer_apps/spiderbot/pose_rules.py). test_spiderbot_whitelist
#: checks this against the source so the two cannot drift.
SPIDERBOT_RULES = ("arms_raised", "one_arm_raised", "person_present", "arm_swing")
#: Detector modes of racer_perception's perception.launch.py, and the devices it accepts here.
PERCEPTION_MODES = ("pose", "detect")
PERCEPTION_DEVICES = ("cuda", "cpu")
#: Patrol speed bounds in m/s. The launch file enforces its own ceiling (1.0) and vehicle_params.
SPIDERBOT_SPEED_RANGE = (0.1, 1.0)
SPIDERBOT_DEFAULT_SPEED_MPS = 0.5
PARK_SERVICES = {
    "start": "/park_node/start",
    "three_point_turn": "/park_node/three_point_turn",
    "abort": "/park_node/abort",
}

NAME_RE = re.compile(r"^[A-Za-z0-9][A-Za-z0-9_.-]{0,63}$")
#: A bag directory name as car_teleop.launch.py makes it: <ISO-ish stamp>_car_teleop.
BAG_RE = re.compile(r"^[A-Za-z0-9][A-Za-z0-9_.:+-]{0,100}$")

# -- fixed pieces ------------------------------------------------------------------------

ROS_SOURCE = "source /opt/ros/humble/setup.bash && source /workspace/ros_ws/install/setup.bash"
DRIVE_PID_FILE = "/tmp/racer-drive.pid"
DRIVE_MODE_FILE = "/tmp/racer-drive.mode"
DRIVE_LOG_FILE = "/tmp/racer-drive.log"
LIDAR_DEVICE = "/dev/ttyUSB0"
VESC_DEVICE = "/dev/ttyTHS1"
PWM_SYSFS = (
    "/sys/devices/platform/bus@0/3280000.pwm",
    "/sys/devices/platform/bus@0/32c0000.pwm",
)
FASTDDS_UDP_ONLY = "/workspace/ros_ws/src/racer_bringup/config/fastdds_udp_only.xml"
CAMERA_USB_TOPIC = "/camera/usb/image_raw/compressed"
#: Where perception caches weights inside its container (HOME is /tmp there) and where the spiderbot
#: writes its event log and snapshots, as the container sees them.
PERCEPTION_CACHE_IN_CONTAINER = "/tmp/.cache/racer_perception"
SPIDERBOT_OUTPUT_IN_CONTAINER = "/workspace/data/spiderbot"


class CommandError(ValueError):
    """A request that does not fit the whitelist. The message is safe to show to the client."""


def container_path(settings_repo: Path, host_path: Path) -> str:
    """Translate a path under the repo to where the container sees it (/workspace/...)."""
    try:
        rel = host_path.resolve().relative_to(settings_repo.resolve())
    except ValueError as exc:
        raise CommandError(f"{host_path} is not inside the repo") from exc
    return "/workspace/" + rel.as_posix()


def _quoted_ok(value: str) -> str:
    if not re.fullmatch(r"[A-Za-z0-9_./:@=+-]+", value):
        raise CommandError(f"unsafe characters in {value!r}")
    return shlex.quote(value)


# -- stack -------------------------------------------------------------------------------


@dataclass(frozen=True)
class StackFlags:
    lidar: bool = True
    vesc: bool = True
    mapping: bool = False
    localization: bool = False
    map: str = ""  # host path of the map yaml, only with localization

    def validate(self) -> None:
        if self.mapping and self.localization:
            raise CommandError("mapping and localisation are exclusive: pick one")
        if (self.mapping or self.localization) and not (self.lidar and self.vesc):
            raise CommandError("mapping and localisation need both the LiDAR and the VESC")
        if self.localization and not self.map:
            raise CommandError("localisation needs a map")
        if self.map and not self.localization:
            raise CommandError("a map is only used with localisation")


def launch_args(flags: StackFlags, container_map: str = "") -> list[str]:
    flags.validate()
    args = [f"lidar:={'true' if flags.lidar else 'false'}"]
    args.append(f"vesc:={'true' if flags.vesc else 'false'}")
    if flags.mapping:
        args.append("mapping:=true")
    if flags.localization:
        args.append("localization:=true")
        args.append(f"map:={container_map}")
    return args


def stack_run_argv(
    host: HostInfo,
    repo_dir: Path,
    image: str,
    container: str,
    flags: StackFlags,
    container_map: str = "",
) -> list[str]:
    """`docker run -d --name car-stack ...`: the runbook's stack command, flag for flag."""
    args = launch_args(flags, container_map)
    gpio = host.group_id("gpio")
    dialout = host.group_id("dialout")
    argv = [
        "docker",
        "run",
        "-d",
        "--name",
        container,
        "--network",
        "host",
        "--user",
        f"{host.uid}:{host.gid}",
    ]
    if gpio is not None:
        argv += ["--group-add", str(gpio)]
    if dialout is not None:
        argv += ["--group-add", str(dialout)]
    if flags.lidar:
        argv += ["--device", LIDAR_DEVICE]
    if flags.vesc:
        argv += ["--device", VESC_DEVICE]
    argv += ["-e", "HOME=/tmp"]
    for path in PWM_SYSFS:
        argv += ["-v", f"{path}:{path}"]
    argv += [
        "-v",
        f"{repo_dir}:/workspace",
        "-w",
        "/workspace/ros_ws",
        image,
        "bash",
        "-lc",
        "source /opt/ros/humble/setup.bash && source install/setup.bash && "
        "exec ros2 launch racer_bringup car_teleop.launch.py " + " ".join(args),
    ]
    return argv


def camera_run_argv(
    host: HostInfo, repo_dir: Path, image: str, container: str, usb_device: str | None
) -> list[str]:
    """`docker run -d --name car-camera --runtime nvidia ...`: docker/car/README.md "Cameras"."""
    argv = [
        "docker",
        "run",
        "-d",
        "--name",
        container,
        "--network",
        "host",
        "--runtime",
        "nvidia",
        "--user",
        f"{host.uid}:{host.gid}",
    ]
    video = host.group_id("video")
    if video is not None:
        argv += ["--group-add", str(video)]
    argv += [
        "-e",
        "HOME=/tmp",
        "-e",
        f"FASTRTPS_DEFAULT_PROFILES_FILE={FASTDDS_UDP_ONLY}",
        "-v",
        "/tmp/argus_socket:/tmp/argus_socket",
    ]
    launch = "ros2 launch racer_bringup cameras.launch.py"
    if usb_device:
        if not re.fullmatch(r"/dev/video\d{1,3}", usb_device):
            raise CommandError(f"not a video device: {usb_device!r}")
        argv += ["--device", usb_device]
        launch += f" usb_device:={usb_device}"
    argv += [
        "-v",
        f"{repo_dir}:/workspace",
        "-w",
        "/workspace/ros_ws",
        image,
        "bash",
        "-lc",
        f"source /opt/ros/humble/setup.bash && source install/setup.bash && exec {launch}",
    ]
    return argv


def perception_run_argv(
    host: HostInfo,
    repo_dir: Path,
    cache_dir: Path,
    image: str,
    container: str,
    mode: str = "pose",
    device: str = "cuda",
) -> list[str]:
    """`docker run -d --name car-perception --runtime nvidia ...` for perception.launch.py.

    The same flags as the camera container (docs/notes/perception-2026-10-23.md "Running it"):
    host network, the NVIDIA runtime, the UDP-only DDS profile, the repo at /workspace, and a
    weights cache volume. No device is passed through: it reads the camera topic, not the camera.
    It never goes in car-stack and publishes nothing the command path reads.
    """
    if mode not in PERCEPTION_MODES:
        raise CommandError(f"perception mode must be one of {list(PERCEPTION_MODES)}")
    if device not in PERCEPTION_DEVICES:
        raise CommandError(f"perception device must be one of {list(PERCEPTION_DEVICES)}")
    launch = f"ros2 launch racer_perception perception.launch.py mode:={mode} device:={device}"
    return [
        "docker",
        "run",
        "-d",
        "--name",
        container,
        "--network",
        "host",
        "--runtime",
        "nvidia",
        "--user",
        f"{host.uid}:{host.gid}",
        "-e",
        "HOME=/tmp",
        "-e",
        f"FASTRTPS_DEFAULT_PROFILES_FILE={FASTDDS_UDP_ONLY}",
        "-v",
        f"{cache_dir}:{PERCEPTION_CACHE_IN_CONTAINER}",
        "-v",
        f"{repo_dir}:/workspace",
        "-w",
        "/workspace/ros_ws",
        image,
        "bash",
        "-lc",
        f"source /opt/ros/humble/setup.bash && source install/setup.bash && exec {launch}",
    ]


# -- inside the stack container ------------------------------------------------------------


def exec_bash(container: str, script: str, login: bool = False) -> list[str]:
    return ["docker", "exec", container, "bash", "-lc" if login else "-c", script]


def drive_launch_script(mode: str, ros_cmd: str) -> str:
    """The script `docker exec -d` runs: record pid and mode, log output, exec the launch.

    `exec` keeps the pid, so the pid file names the ros2 process itself and a SIGINT sent to it
    shuts the launch down the way Ctrl-C does (the runbook's "Stopping" section: not SIGTERM).
    """
    return (
        f"echo $$ > {DRIVE_PID_FILE}; echo {_quoted_ok(mode)} > {DRIVE_MODE_FILE}; "
        f"{ROS_SOURCE} && exec {ros_cmd} > {DRIVE_LOG_FILE} 2>&1"
    )


def ros_cmd_for(
    mode: str, profile: str = "", raceline_container_path: str = "", pose_frame: str = "map"
) -> str:
    """The ros2 command for a driving mode. Raises CommandError for anything off the lists."""
    if mode == "gap_follow":
        if profile not in GAP_FOLLOW_PROFILES:
            raise CommandError(f"gap_follow profile must be one of {list(GAP_FOLLOW_PROFILES)}")
        return f"ros2 launch racer_control gap_follow.launch.py profile:={profile}"
    if mode == "wall_follow":
        return "ros2 launch racer_control wall_follow.launch.py"
    if mode == "park":
        if profile not in PARK_PROFILES:
            raise CommandError(f"park profile must be one of {list(PARK_PROFILES)}")
        return f"ros2 launch racer_control park.launch.py profile:={profile}"
    if mode == "tracker":
        if not raceline_container_path.startswith("/workspace/") or ".." in raceline_container_path:
            raise CommandError("raceline must be a file inside the repo")
        # tracker.launch.py only forwards raceline_path, and pose_frame:=map is what makes the
        # tracker read the localised pose, so this runs the node directly (no launch edit).
        return (
            "ros2 run racer_control tracker_node --ros-args "
            f"-p raceline_path:={_quoted_ok(raceline_container_path)} "
            f"-p pose_frame:={_quoted_ok(pose_frame)}"
        )
    raise CommandError(f"unknown driving mode {mode!r}")


def spiderbot_cmd(
    patrol_container_path: str,
    rule: str = "arms_raised",
    speed_mps: float = SPIDERBOT_DEFAULT_SPEED_MPS,
) -> str:
    """`ros2 launch racer_apps spiderbot.launch.py ...`: patrol, rule and speed from closed sets.

    The patrol is a file the agent found itself under data/patrols (the caller passes its
    container path); the rule is one of SPIDERBOT_RULES; the speed is a number inside
    SPIDERBOT_SPEED_RANGE formatted here, never passed through as text. The launch file
    enforces its own ceiling and the vehicle_params limits on top.
    """
    if not patrol_container_path.startswith("/workspace/") or ".." in patrol_container_path:
        raise CommandError("the patrol must be a file inside the repo")
    if rule not in SPIDERBOT_RULES:
        raise CommandError(f"spiderbot rule must be one of {list(SPIDERBOT_RULES)}")
    if isinstance(speed_mps, bool) or not isinstance(speed_mps, (int, float)):
        raise CommandError("patrol speed must be a number")
    low, high = SPIDERBOT_SPEED_RANGE
    if not (low <= float(speed_mps) <= high):
        raise CommandError(f"patrol speed must be between {low} and {high} m/s")
    return (
        "ros2 launch racer_apps spiderbot.launch.py "
        f"patrol:={_quoted_ok(patrol_container_path)} rule:={rule} "
        f"speed_mps:={float(speed_mps):.2f} output_dir:={SPIDERBOT_OUTPUT_IN_CONTAINER}"
    )


#: Prints the running mode and exits 0 when the drive process is alive, else exits 1.
DRIVE_STATUS_SCRIPT = (
    f'p=$(cat {DRIVE_PID_FILE} 2>/dev/null) && [ -n "$p" ] && '
    f"grep -aq ros2 /proc/$p/cmdline 2>/dev/null && cat {DRIVE_MODE_FILE}"
)


def drive_signal_script(signal: str) -> str:
    if signal not in ("INT", "TERM", "KILL"):
        raise CommandError("bad signal")
    return (
        f'p=$(cat {DRIVE_PID_FILE} 2>/dev/null) && [ -n "$p" ] && '
        f"grep -aq ros2 /proc/$p/cmdline 2>/dev/null && kill -{signal} $p"
    )


def park_service_script(service: str) -> str:
    if service not in PARK_SERVICES.values():
        raise CommandError("unknown park service")
    return f'{ROS_SOURCE} && ros2 service call {service} std_srvs/srv/Trigger "{{}}"'


def save_map_script(container_base: str) -> str:
    return (
        f"{ROS_SOURCE} && mkdir -p {_quoted_ok(str(Path(container_base).parent))} && "
        "ros2 service call /slam_toolbox/save_map slam_toolbox/srv/SaveMap "
        f'"{{name: {{data: {_quoted_ok(container_base)}}}}}"'
    )


def recorder_check_argv(container: str) -> list[str]:
    """`docker top` lists the container's processes from the host, no exec needed."""
    return ["docker", "top", container, "-eo", "args"]


def recorder_running(top_output: str) -> bool:
    return any("bag record" in line for line in top_output.splitlines())


def parse_launch_flags(cmd_json_text: str) -> dict[str, str]:
    """Read lidar:=true style arguments back out of `docker inspect ... .Config.Cmd`."""
    return dict(re.findall(r"\b([a-z_]+):=([^\s\"'\\]+)", cmd_json_text))


def tail_argv(container: str, lines: int = 60) -> Sequence[str]:
    return exec_bash(container, f"tail -n {int(lines)} {DRIVE_LOG_FILE} 2>/dev/null")
