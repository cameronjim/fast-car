"""car_teleop.launch.py -- the FIRST launch file that drives the REAL car.

    (keyboard_teleop_node | twist_teleop_adapter_node) -> /drive_raw
        -> racer_safety/safety_node -> /drive
        -> racer_drivers/pwm_output_node -> two 50 Hz PWM pulses
        -> layer-1 mux board (firmware/safety_mux/) -> steering servo + VESC PPM input

This is sim_teleop.launch.py's graph with the sim swapped for hardware: NO racer_gym_bridge,
no simulator of any kind. Everything else about the command path is deliberately identical,
including that racer_safety/safety_node is the ONLY node here that publishes /drive
(claude-docs/05-safety.md layer 3) and that pwm_output_node subscribes to /drive and never to
/drive_raw (CLAUDE.md invariant 1).

SAFETY, before running this on a real car (claude-docs/05-safety.md "Operational rules"):
wheels off the ground, the RC kill switch armed and held by a second person, and the
bench-calibration steps in docs/notes/first-boot-runbook.md done first. The mux is physically
downstream of every node started here and nothing here can reconfigure or bypass it -- but
the mux itself has never been kill-tested (firmware/safety_mux/README.md), so "the mux will
save you" is not yet a claim anyone may lean on.

PHYSICAL PARAMETERS. Every pulse bound, angle limit, speed reference AND the steering sign
convention reaches the two nodes from config/vehicle_params.yaml through their generated
bindings (CLAUDE.md invariant 2); nothing physical is passed as a launch argument here. The
steering sign (steering.pwm_left_bound) used to be a launch argument
(steering_left_is_pwm_max) because it was an unmeasured guess; it moved into
config/vehicle_params.yaml once it was actually measured on the car, 2026-09-21
(docs/notes/build-log.md). The arguments below are now only machine configuration (which
pwmchip, which teleop source), per claude-docs/10-conventions.md's "per-machine config via
launch arguments, not edits".

TELEOP, one publisher at a time. Exactly as in sim_teleop.launch.py: `start_teleop` and
`browser_teleop` each start a node that publishes /drive_raw and nothing arbitrates between
two of them. Here BOTH default to false, unlike the sim: this launch moves a physical car, so
it comes up with no command source at all -- safety_node publishes a steady neutral /drive,
pwm_output_node holds both channels at neutral, and the operator turns on exactly one teleop
source deliberately. keyboard_teleop_node needs a real interactive TTY (raw terminal mode),
which a ros2-launch-managed subprocess does not reliably provide, so the supported way to use
it is a SECOND terminal running `ros2 run racer_tools keyboard_teleop_node`; `start_teleop`
exists for environments where that stdio caveat happens not to matter.

LOGGING IS NOT OPTIONAL (CLAUDE.md invariant 5, roadmap 1.6, GitHub issue #64). This launch
starts a rosbag2 recorder AND rail_voltage_node by default, and a recorder that dies takes
the launch down with it -- see "THE RECORDER IS LOAD-BEARING" below. `record:=false` exists
for bench work where nothing can move (pin-level checks, a neutral-output verification with
the battery out); a DRIVE with `record:=false` is a bug, not a choice.
"""

import datetime
import os

from ament_index_python.resources import has_resource
from launch import LaunchDescription
from launch import logging as launch_logging
from launch.actions import (
    DeclareLaunchArgument,
    EmitEvent,
    ExecuteProcess,
    LogInfo,
    OpaqueFunction,
    RegisterEventHandler,
)
from launch.conditions import IfCondition
from launch.event_handlers import OnProcessExit
from launch.events import Shutdown
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue

_FOXGLOVE_BRIDGE_PORT = 8765

#: Everything a drive has to be reconstructable from (CLAUDE.md invariant 5). Passed to
#: `ros2 bag record --regex`, not as a positional topic list, for two reasons: a topic that
#: does not exist on this run (`/scan` -- no LiDAR is fitted yet, roadmap 2.x) is simply not
#: matched instead of being waited on, and `/telemetry/.*` picks up every rail_voltage_node
#: channel topic without this file having to know the board's INA3221 labels.
#:
#:   /drive_raw        what the teleop source asked for       } the command path, both sides
#:   /drive            what safety_node actually allowed      } of the layer-3 gate
#:   /safety/events    every intervention (claude-docs/09: interventions are reported data)
#:   /teleop/cmd_vel   the browser Teleop panel's raw input, upstream of /drive_raw
#:   /telemetry/...    rail volts/amps -- invariant 5's second half, and the only way a
#:                     brownout is distinguishable from a control fault after the fact
#:   /scan             LiDAR, when one is fitted
#:   /rosout           every node's log stream, in the same file as the data it explains
#:   /parameter_events which parameters the run actually ran with
_RECORDED_TOPIC_REGEX = (
    r"^(/drive_raw|/drive|/safety/events|/teleop/cmd_vel|/telemetry/.*|/scan"
    r"|/rosout|/parameter_events)$"
)


def _resolve_storage_id(requested: str) -> str:
    """Pick the rosbag2 storage plugin, and say so in the log rather than silently.

    `auto` (the default) uses mcap when the rosbag2_storage_mcap plugin is installed and
    sqlite3 otherwise. Those are genuinely different images, not a preference: docker/car
    installs `ros-humble-rosbag2-storage-mcap` (mcap is the rosbag2 default from Iron on, it
    writes a single self-describing file, and it survives an uncleanly killed process far
    better than a sqlite3 bag does -- which matters when the way a session ends is sometimes
    a kill switch), while docker/ros-dev is a stock `ros:humble-ros-base` where sqlite3 is
    the only plugin present, so the L3 launch test necessarily exercises sqlite3.

    An EXPLICIT `bag_storage:=mcap` is passed through untouched even if the plugin looks
    absent: this check reads the ament index, and being wrong about a plugin is not a reason
    to silently downgrade what an operator asked for -- `ros2 bag record` will refuse loudly.
    """
    if requested != "auto":
        return requested
    return "mcap" if has_resource("packages", "rosbag2_storage_mcap") else "sqlite3"


def _bag_actions(context, *args, **kwargs):
    """Build the recorder action: a dated output directory, then `ros2 bag record`.

    Runs as an OpaqueFunction because both the bag root and the storage choice are launch
    ARGUMENTS, so the directory name can only be computed once the context can resolve them.
    """
    bag_root = LaunchConfiguration("bag_dir").perform(context)
    storage_id = _resolve_storage_id(LaunchConfiguration("bag_storage").perform(context))

    # Local time, not UTC: every other artefact of a session (docs/notes entries, the bench
    # log, the person holding the kill switch) is in local time, and a bag whose name does
    # not line up with the notebook is a bag nobody finds again. Colons are not usable in a
    # path, so the ISO-8601 time separators become hyphens.
    # .astimezone() makes the local time tz-aware without changing the rendered digits; a
    # naive now() is a ruff DTZ005 error.
    stamp = datetime.datetime.now().astimezone().strftime("%Y-%m-%dT%H-%M-%S")
    bag_path = os.path.join(bag_root, f"{stamp}_car_teleop")

    # Create the ROOT here and let rosbag2 create the leaf: `ros2 bag record -o` refuses to
    # start if its output directory already exists, which is the check that stops two runs
    # ever landing in one bag (claude-docs/10-conventions.md: "Bags are immutable once
    # written"). Making the root eagerly means a fresh Jetson, or a bag_dir on a just-mounted
    # disk, does not fail the first run of the day.
    os.makedirs(bag_root, exist_ok=True)

    recorder = ExecuteProcess(
        cmd=[
            "ros2",
            "bag",
            "record",
            "--regex",
            _RECORDED_TOPIC_REGEX,
            "--storage",
            storage_id,
            "--output",
            bag_path,
        ],
        name="rosbag_recorder",
        output="screen",
        # `ros2 bag record` handles SIGINT itself and closes the bag cleanly; SIGTERM would
        # leave the metadata unwritten, so never escalate faster than the default.
        sigterm_timeout="10",
        sigkill_timeout="10",
    )

    return [
        LogInfo(
            msg=(
                f"[invariant 5] recording {storage_id} bag to {bag_path} "
                f"(topics matching {_RECORDED_TOPIC_REGEX})"
            )
        ),
        recorder,
        RegisterEventHandler(OnProcessExit(target_action=recorder, on_exit=_on_recorder_exit)),
    ]


def _on_recorder_exit(event, context):
    """THE RECORDER IS LOAD-BEARING: if it dies, the whole launch goes down.

    CLAUDE.md invariant 5 makes "drives the car without logging" a bug, so the recorder
    exiting mid-session cannot be a warning in a scrollback nobody reads. Two options were on
    the table (GitHub issue #64) and this is the one that fits claude-docs/05-safety.md:

      * NOT chosen: gate /drive on the recorder being alive. This would put a logging
        dependency inside the layer-3 command path, where safety_node is the sole publisher
        of /drive and fails CLOSED on its own criteria. Logging is an observability concern;
        making it able to change what reaches the actuators weakens layer 3 to strengthen
        something that is not a safety layer at all. Explicitly rejected.
      * NOT chosen: publish a racer_msgs/SafetyEvent on /safety/events. That topic has one
        publisher (safety_node) and a closed `source` enum of GATES, and claude-docs/09-
        evaluation.md COUNTS PHASE_ENGAGE records as interventions -- a recorder crash is not
        an intervention, and injecting one would corrupt a reported metric.
      * CHOSEN: log at error and emit a launch Shutdown. That stops the teleop source and
        every other node this launch owns, and pwm_output_node's own shutdown path writes the
        calibrated neutral and disables both channels (racer_drivers/pwm_output_driver). The
        car stops being commanded, layers 1 and 2 are untouched and still downstream of
        everything here, and no code in the command path ever consults the recorder.

    A shutdown already in progress (operator Ctrl-C) exits the recorder too; that is the
    normal path and must stay quiet, hence the is_shutdown check.
    """
    if context.is_shutdown:
        return []
    message = (
        "FATAL [CLAUDE.md invariant 5]: the rosbag recorder exited "
        f"(returncode {event.returncode}). A drive that is not being recorded is a bug, so "
        "this launch is shutting down. Check disk space and that bag_dir is writable, then "
        "relaunch. Do NOT work around this with record:=false."
    )
    # launch_logging, not LogInfo: this has to be visible AT ERROR LEVEL in the launch log,
    # and LogInfo is the only log action launch ships.
    launch_logging.get_logger("car_teleop").error(message)
    return [
        LogInfo(msg=message),
        EmitEvent(event=Shutdown(reason="rosbag recorder exited (CLAUDE.md invariant 5)")),
    ]


def generate_launch_description() -> LaunchDescription:
    start_teleop_arg = DeclareLaunchArgument(
        "start_teleop",
        default_value="false",
        description=(
            "Also start keyboard_teleop_node in this launch. NOT the supported path: it "
            "needs a real interactive TTY that a ros2-launch-managed subprocess does not "
            "reliably provide, so normally you run it in a SECOND terminal ("
            "`ros2 run racer_tools keyboard_teleop_node`). Default false. Do not set this "
            "AND browser_teleop to true at once -- see this file's module docstring."
        ),
    )
    browser_teleop_arg = DeclareLaunchArgument(
        "browser_teleop",
        default_value="false",
        description=(
            "Start racer_tools/twist_teleop_adapter_node so the car can be driven from "
            "Foxglove's Teleop panel with no second terminal. Default false on the CAR "
            "(unlike sim_teleop.launch.py, where it defaults true): a physical car comes up "
            "with no command source until an operator chooses one. Do not set this AND "
            "start_teleop to true at once."
        ),
    )
    teleop_cmd_vel_topic_arg = DeclareLaunchArgument(
        "teleop_cmd_vel_topic",
        default_value="/teleop/cmd_vel",
        description=(
            "Topic twist_teleop_adapter_node subscribes geometry_msgs/Twist on -- must match "
            "the Teleop panel's configured topic in whatever Foxglove layout is in use."
        ),
    )
    twist_timeout_s_arg = DeclareLaunchArgument(
        "twist_timeout_s",
        default_value="0.5",
        description=(
            "twist_teleop_adapter_node: seconds since the last received Twist after which it "
            "commands (and keeps commanding) zero."
        ),
    )
    allow_reverse_arg = DeclareLaunchArgument(
        "allow_reverse",
        default_value="false",
        description=(
            "Allow either teleop source to command NEGATIVE speed. Default false: the lower "
            "speed clamp becomes 0.0 m/s instead of vehicle_params limits.min_velocity_mps "
            "(-5.0), so no below-neutral throttle pulse is ever produced. What a "
            "below-neutral pulse does is a VESC PPM control-type setting that has never been "
            "applied to this ESC -- reverse current in 'Current', proportional braking in "
            "'Current No Reverse With Brake' -- so the first drives do not emit one. Turn on "
            "at the bench once that setting is known and recorded "
            "(docs/notes/first-boot-runbook.md)."
        ),
    )
    viz_arg = DeclareLaunchArgument(
        "viz",
        default_value="true",
        description=(
            "Start foxglove_bridge on port "
            f"{_FOXGLOVE_BRIDGE_PORT} (same pattern as sim_teleop.launch.py / "
            "sim_autopilot.launch.py) so a laptop can watch /drive, /safety/events and the "
            "rest live. Requires the container to publish that port."
        ),
    )
    sysfs_root_arg = DeclareLaunchArgument(
        "sysfs_root",
        default_value="/sys/class/pwm",
        description=(
            "Root of the Linux sysfs PWM interface pwm_output_node writes to. Only ever "
            "changed by tests, which point it at a fake tree of ordinary files (see "
            "racer_bringup/test/test_car_teleop_launch.py). On the car this stays "
            "/sys/class/pwm."
        ),
    )
    steering_pwmchip_arg = DeclareLaunchArgument(
        "steering_pwmchip",
        default_value="0",
        description=(
            "pwmchip index driving the steering servo pulse (Jetson 40-pin header pin 15). "
            "VERIFIED on the Jetson Orin Nano Super Dev Kit, JetPack 6.2 / L4T R36.4.4, on "
            "the actual device on 2026-09-20: pin 15 is pwmchip0. Confirm on another unit "
            "with 'ls -l /sys/class/pwm' after the /opt/nvidia/jetson-io pinmux change and a "
            "reboot -- see ros_ws/src/racer_drivers/README.md."
        ),
    )
    steering_pwm_channel_arg = DeclareLaunchArgument(
        "steering_pwm_channel",
        default_value="0",
        description="Channel index within steering_pwmchip. VERIFIED, see steering_pwmchip.",
    )
    throttle_pwmchip_arg = DeclareLaunchArgument(
        "throttle_pwmchip",
        default_value="2",
        description=(
            "pwmchip index driving the throttle/ESC pulse (Jetson 40-pin header pin 33). "
            "VERIFIED, see steering_pwmchip: pin 33 is pwmchip2 on this device -- pins 15 "
            "and 33 are different chips."
        ),
    )
    throttle_pwm_channel_arg = DeclareLaunchArgument(
        "throttle_pwm_channel",
        default_value="0",
        description="Channel index within throttle_pwmchip. VERIFIED, see steering_pwmchip.",
    )
    drive_timeout_s_arg = DeclareLaunchArgument(
        "drive_timeout_s",
        default_value="0.1",
        description=(
            "pwm_output_node: seconds of /drive silence after which both channels output "
            "neutral. Node tuning, not a physical constant, and deliberately separate from "
            "the layer-1 mux's own heartbeat window."
        ),
    )

    record_arg = DeclareLaunchArgument(
        "record",
        default_value="true",
        description=(
            "Start a rosbag2 recorder with the stack. DEFAULT TRUE and meant to stay that "
            "way: CLAUDE.md invariant 5 says every run is logged and that a code path which "
            "drives the car without logging is a bug. record:=false is for bench work where "
            "nothing can move -- a pin-level check, a neutral-output verification with the "
            "battery out. A DRIVE with record:=false is a bug, not a configuration choice."
        ),
    )
    bag_dir_arg = DeclareLaunchArgument(
        "bag_dir",
        default_value="/workspace/data/bags",
        description=(
            "Root under which each run gets its own <ISO-timestamp>_car_teleop directory; "
            "created if missing. The default is the repo's own data/bags "
            "(claude-docs/02-repo-layout.md's bag home, gitignored) as it appears INSIDE the "
            "car container, which bind-mounts the repo at /workspace -- so on the Jetson "
            "bags land in ~/car/data/bags and persist across container restarts with no "
            "extra mount. /data/bags was considered and rejected: it is not mounted into the "
            "container and creating it would need root on a host the runbook deliberately "
            "keeps unprivileged. Point this somewhere else on any other machine."
        ),
    )
    bag_storage_arg = DeclareLaunchArgument(
        "bag_storage",
        default_value="auto",
        description=(
            "rosbag2 storage plugin: 'auto' (mcap if rosbag2_storage_mcap is installed, "
            "sqlite3 otherwise), or an explicit 'mcap'/'sqlite3' which is passed through "
            "untouched. docker/car installs the mcap plugin, so the car records mcap; "
            "docker/ros-dev is stock ros:humble-ros-base and has sqlite3 only, so the L3 "
            "launch test records sqlite3. The choice is logged on every start."
        ),
    )
    rail_voltage_arg = DeclareLaunchArgument(
        "rail_voltage",
        default_value="true",
        description=(
            "Start racer_drivers/rail_voltage_node, which publishes the Jetson's onboard "
            "INA3221 rails as /telemetry/... volts and amps -- the second half of CLAUDE.md "
            "invariant 5's 'rosbag + rail voltage'. Harmless off the Jetson: with no INA3221 "
            "the node warns once and publishes nothing rather than failing."
        ),
    )
    ina3221_root_arg = DeclareLaunchArgument(
        "ina3221_root",
        default_value="/sys/bus/i2c/drivers/ina3221",
        description=(
            "Root of the kernel ina3221 sysfs tree rail_voltage_node reads. Only ever "
            "changed by tests, which point it at a fake tree of ordinary files -- the same "
            "pattern as sysfs_root above. On the car this stays at the default."
        ),
    )

    safety_node = Node(
        package="racer_safety",
        executable="safety_node",
        name="safety_node",
        output="screen",
    )
    pwm_output_node = Node(
        package="racer_drivers",
        executable="pwm_output_node",
        name="pwm_output_node",
        output="screen",
        parameters=[
            {
                # ParameterValue(..., value_type=...) on every non-string argument: a bare
                # LaunchConfiguration evaluates to TEXT, and pwm_output_node declares these
                # as int/bool/double with ranges, so an unwrapped substitution is rejected as
                # a type mismatch at node start.
                "sysfs_root": LaunchConfiguration("sysfs_root"),
                "steering_pwmchip": ParameterValue(
                    LaunchConfiguration("steering_pwmchip"), value_type=int
                ),
                "steering_pwm_channel": ParameterValue(
                    LaunchConfiguration("steering_pwm_channel"), value_type=int
                ),
                "throttle_pwmchip": ParameterValue(
                    LaunchConfiguration("throttle_pwmchip"), value_type=int
                ),
                "throttle_pwm_channel": ParameterValue(
                    LaunchConfiguration("throttle_pwm_channel"), value_type=int
                ),
                "drive_timeout_s": ParameterValue(
                    LaunchConfiguration("drive_timeout_s"), value_type=float
                ),
            }
        ],
    )
    teleop_node = Node(
        package="racer_tools",
        executable="keyboard_teleop_node",
        name="keyboard_teleop",
        output="screen",
        parameters=[
            {
                "allow_reverse": ParameterValue(
                    LaunchConfiguration("allow_reverse"), value_type=bool
                ),
            }
        ],
        condition=IfCondition(LaunchConfiguration("start_teleop")),
    )
    twist_teleop_adapter_node = Node(
        package="racer_tools",
        executable="twist_teleop_adapter_node",
        name="twist_teleop_adapter",
        output="screen",
        parameters=[
            {
                "input_topic": LaunchConfiguration("teleop_cmd_vel_topic"),
                "twist_timeout_s": ParameterValue(
                    LaunchConfiguration("twist_timeout_s"), value_type=float
                ),
                "allow_reverse": ParameterValue(
                    LaunchConfiguration("allow_reverse"), value_type=bool
                ),
            }
        ],
        condition=IfCondition(LaunchConfiguration("browser_teleop")),
    )
    rail_voltage_node = Node(
        package="racer_drivers",
        executable="rail_voltage_node",
        name="rail_voltage_node",
        output="screen",
        parameters=[{"ina3221_root": LaunchConfiguration("ina3221_root")}],
        condition=IfCondition(LaunchConfiguration("rail_voltage")),
    )
    # OpaqueFunction, not a bare ExecuteProcess: the output directory name is computed from
    # the CURRENT time and the resolved bag_dir argument, neither of which exists until the
    # launch context is live. IfCondition is applied to the whole group.
    bag_recorder = OpaqueFunction(
        function=_bag_actions, condition=IfCondition(LaunchConfiguration("record"))
    )
    foxglove_bridge_node = Node(
        package="foxglove_bridge",
        executable="foxglove_bridge",
        name="foxglove_bridge",
        output="screen",
        parameters=[{"port": _FOXGLOVE_BRIDGE_PORT}],
        condition=IfCondition(LaunchConfiguration("viz")),
    )

    return LaunchDescription(
        [
            start_teleop_arg,
            browser_teleop_arg,
            teleop_cmd_vel_topic_arg,
            twist_timeout_s_arg,
            allow_reverse_arg,
            viz_arg,
            sysfs_root_arg,
            steering_pwmchip_arg,
            steering_pwm_channel_arg,
            throttle_pwmchip_arg,
            throttle_pwm_channel_arg,
            drive_timeout_s_arg,
            record_arg,
            bag_dir_arg,
            bag_storage_arg,
            rail_voltage_arg,
            ina3221_root_arg,
            # rail_voltage_node BEFORE the recorder so its topics exist by the time
            # `ros2 bag record --regex` does its first discovery pass, and the recorder
            # before the command-path nodes for the same reason: rosbag2 does keep
            # discovering, but starting the log first is the ordering invariant 5 implies.
            rail_voltage_node,
            bag_recorder,
            safety_node,
            pwm_output_node,
            teleop_node,
            twist_teleop_adapter_node,
            foxglove_bridge_node,
        ]
    )
