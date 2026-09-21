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

PHYSICAL PARAMETERS. Every pulse bound, angle limit and speed reference reaches the two nodes
from config/vehicle_params.yaml through their generated bindings (CLAUDE.md invariant 2);
nothing physical is passed as a launch argument here. The arguments below are machine
configuration (which pwmchip, which teleop source) and bench calibration
(steering_left_is_pwm_max), per claude-docs/10-conventions.md's "per-machine config via
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
"""

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue

_FOXGLOVE_BRIDGE_PORT = 8765


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
    steering_left_is_pwm_max_arg = DeclareLaunchArgument(
        "steering_left_is_pwm_max",
        default_value="true",
        description=(
            "true = steering.pwm_max_us is full LEFT (positive road-wheel angle, "
            "claude-docs/06-vehicle-params.md). BENCH-CALIBRATED, not measured yet: confirm "
            "it with the wheels off the ground before driving "
            "(docs/notes/first-boot-runbook.md)."
        ),
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
                "steering_left_is_pwm_max": ParameterValue(
                    LaunchConfiguration("steering_left_is_pwm_max"), value_type=bool
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
            steering_left_is_pwm_max_arg,
            drive_timeout_s_arg,
            safety_node,
            pwm_output_node,
            teleop_node,
            twist_teleop_adapter_node,
            foxglove_bridge_node,
        ]
    )
