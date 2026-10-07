"""vesc_telemetry.launch.py -- VESC UART telemetry and wheel odometry, READ-ONLY.

Starts two nodes:

    vesc_driver/vesc_driver_node    (namespace /vesc) polls the VESC over its UART for
                                    COMM_GET_VALUES at 50 Hz and publishes
                                    /vesc/sensors/core (vesc_msgs/VescStateStamped)
    racer_drivers/vesc_odometry_node
                                    /vesc/sensors/core -> /odom/wheel (wheel speed and signed
                                    along-track distance) and /telemetry/vesc/* (volts, amps,
                                    temperatures, raw ERPM, fault name)

Run it on its own for the bench check (docs/notes/first-boot-runbook.md "VESC telemetry"), or
through car_teleop.launch.py with vesc:=true so everything lands in the run's bag.

NOTHING HERE COMMANDS THE VESC. The motor's only command path is the PPM pulse from
pwm_output_node through the layer-1 mux (CLAUDE.md invariant 1, claude-docs/05-safety.md). The
f1tenth driver upstream also subscribes six command topics (commands/motor/duty_cycle,
current, brake, speed, position and commands/servo/position) and forwards them to the VESC
over the same serial link. The car image builds it with docker/car/patches/
vesc_driver-readonly.patch, which removes those subscriptions and turns every set-command
in its serial interface into a no-op, so this driver has no command interface at all. As a
second, independent line: this file passes NO remapping for any of those topics, and
test_vesc_telemetry_launch.py fails if one is ever added. vesc_ackermann (which would turn
/drive into VESC commands) is not even built in the image.

THE DRIVER. f1tenth/vesc, ros2 branch, pinned commit in docker/car/Dockerfile, built into
/opt/racer_thirdparty like sllidar_ros2. It is NOT in ros_ws/src and NOT an <exec_depend> of
racer_bringup, for the same reason as sllidar_ros2 (see lidar.launch.py): the ros-dev CI image
carries vesc_msgs (racer_drivers compiles against it) but not the driver, and rosdep has no
key for either. So this file only RUNS in the car image; its L1 test inspects the launch
description and never starts the driver.

PARAMETERS. serial_port and baud are launch arguments, per-machine configuration
(claude-docs/10-conventions.md: "Per-machine config via launch arguments, not edits"), not
vehicle_params fields: they describe how this Jetson is wired to this VESC, not a physical
property of the car. /dev/ttyTHS1 is the Jetson Orin Nano dev kit's 40-pin header UART (pins
8 TXD / 10 RXD). 115200 must match the VESC Tool App Settings > UART baud rate
(app_uart_baudrate, 115200 in config/vesc/2026-09-29d-fsesc67-app.xml). The `baud` parameter
exists only because the read-only patch adds it; upstream hard-codes 115200 and also enables
hardware flow control, which the patch turns off (a 3-wire UART has no CTS to wait on).
"""

from __future__ import annotations

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue

#: Namespace the driver runs in, so its relative topic sensors/core becomes /vesc/sensors/core.
VESC_NAMESPACE = "vesc"

#: The VescStateStamped topic the driver publishes and vesc_odometry_node consumes.
STATE_TOPIC = f"/{VESC_NAMESPACE}/sensors/core"

#: Launch argument name -> (default, description).
LAUNCH_ARGUMENTS = {
    "serial_port": (
        "/dev/ttyTHS1",
        (
            "Serial device wired to the VESC's UART (COMM) port. /dev/ttyTHS1 is the Jetson "
            "Orin Nano 40-pin header UART, pins 8 (TXD) and 10 (RXD). Needs --device and the "
            "dialout group in the container, and nvgetty disabled on the host."
        ),
    ),
    "baud": (
        "115200",
        "UART baud rate. Must equal VESC Tool App Settings > UART > Baudrate (app_uart_baudrate).",
    ),
}


def vesc_nodes() -> list[Node]:
    """The (read-only patched) driver and the odometry node."""
    driver = Node(
        package="vesc_driver",
        executable="vesc_driver_node",
        name="vesc_driver",
        namespace=VESC_NAMESPACE,
        output="screen",
        parameters=[
            {
                "port": LaunchConfiguration("serial_port"),
                # The patch declares `baud` as an integer parameter.
                "baud": ParameterValue(LaunchConfiguration("baud"), value_type=int),
            }
        ],
        # Deliberately NO remappings: see the module docstring. In particular nothing maps
        # commands/* onto any topic in this graph.
    )
    odometry = Node(
        package="racer_drivers",
        executable="vesc_odometry_node",
        name="vesc_odometry_node",
        output="screen",
        parameters=[{"state_topic": STATE_TOPIC}],
    )
    return [driver, odometry]


def generate_launch_description() -> LaunchDescription:
    declared = [
        DeclareLaunchArgument(name, default_value=default, description=description)
        for name, (default, description) in LAUNCH_ARGUMENTS.items()
    ]
    return LaunchDescription([*declared, *vesc_nodes()])
