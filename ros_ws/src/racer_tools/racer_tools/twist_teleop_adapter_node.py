"""twist_teleop_adapter_node: browser-driven teleop via Foxglove's Teleop panel (roadmap
milestone 5, claude-docs/04-architecture.md's command path: this node is the SECOND possible
producer of /drive_raw alongside keyboard_teleop_node; racer_safety/safety_node remains the
SOLE publisher of /drive and gates /drive_raw -> /drive -- this node never publishes /drive
directly).

Milestone 1's keyboard_teleop_node needs a `docker exec -it` terminal in raw tty mode, which
is real friction for driving the sim (or, eventually, the car) from just the Foxglove window.
This node closes that gap: it subscribes `geometry_msgs/Twist` on a configurable topic
(default `/teleop/cmd_vel`, the topic Foxglove Studio's built-in Teleop panel publishes --
see `ros_ws/src/racer_bringup/config/foxglove_sim_viz.layout.json` for the panel wired to it,
and docs/notes/milestone-5-browser-teleop.md for the demo procedure), converts it to an
Ackermann command via `racer_tools.twist_teleop.convert_twist_to_command` (kinematic bicycle
model, clamped to the SAME `vehicle_params`-derived bounds `keyboard_teleop_node` uses), and
republishes `AckermannDriveStamped` on `/drive_raw` at a fixed rate (`control_rate_hz`,
default 50 Hz per claude-docs/04-architecture.md), reliable QoS depth 10 -- the identical
publish shape `keyboard_teleop_node` already uses.

No-Twist timeout design (read before changing `twist_timeout_s`'s default or removing the
republish timer):

`racer_safety/safety_node` already has its own independent watchdog (missing `/drive_raw` for
`watchdog_missed_cycles` cycles, default 3 at 50 Hz = 60ms, brakes -- claude-docs/04-
architecture.md). This node does NOT rely on that as its only line of defense against a
Foxglove tab going away or a dropped websocket: like `keyboard_teleop_node`, it runs its own
fixed-rate timer that ALWAYS publishes -- every cycle, forever, whether or not a fresh Twist
arrived since the last one -- so `/drive_raw` itself never goes silent while this node is
alive (the ONLY way `safety_node`'s watchdog can fire because of this node is this node's own
process dying, exactly the same failure mode `keyboard_teleop_node` already has). What
changes on a stale Twist is the COMMAND this node chooses to publish on that steady heartbeat:
if no Twist has arrived within `twist_timeout_s` (default 0.5s -- generous enough that
Foxglove's own publish cadence between button-hold ticks, or a brief GC/scheduling pause,
does not spuriously zero the car mid-drive, but short enough that letting go of the Teleop
panel's buttons stops the car quickly), this node commands a hard zero (0 steering, 0 speed)
and KEEPS commanding zero, republished every cycle, until a fresh Twist arrives -- it never
transitions to publishing nothing. This was chosen over the alternative allowed by this
milestone's brief ("zeros-then-silent", i.e. command zero once and then stop the publish
timer, leaning on safety_node's watchdog for everything after that): stopping our own publish
timer would mean a LATER bug in this node (e.g. the timer silently failing to reschedule)
degrades from "this node's clearly-labeled behavior" to "whatever safety_node's watchdog
happens to do", and 60ms is a much shorter grace window than a human deciding to reach for a
different control scheme mid-session. Continuous zero republish is strictly safer, no more
code, and keeps this node self-contained: an observer watching only `/drive_raw` sees an
honest, continuous signal of exactly what this node currently believes the driver wants,
never a silence they have to interpret via a downstream node's timeout.

The pure conversion/timeout logic (`racer_tools.twist_teleop`) is unit-tested without a
subscriber (claude-docs/12-testing.md L1); this file is thin rclpy plumbing, same split as
keyboard_teleop_node.py/keymap.py.
"""

from __future__ import annotations

import rclpy
from ackermann_msgs.msg import AckermannDriveStamped
from geometry_msgs.msg import Twist
from rcl_interfaces.msg import FloatingPointRange, ParameterDescriptor
from rclpy.node import Node
from rclpy.qos import HistoryPolicy, QoSProfile, ReliabilityPolicy

from racer_tools.twist_teleop import (
    DriveCommand,
    build_twist_teleop_config,
    convert_twist_to_command,
    should_use_zero_command,
)
from racer_tools.vehicle_params_loader import load_vehicle_params

_ZERO_COMMAND = DriveCommand(steering_angle_rad=0.0, speed_mps=0.0)


class TwistTeleopAdapterNode(Node):
    def __init__(self) -> None:
        super().__init__("twist_teleop_adapter")

        rate_descriptor = ParameterDescriptor(
            description="Publish rate for /drive_raw (claude-docs/04-architecture.md: 50 Hz).",
            floating_point_range=[FloatingPointRange(from_value=1.0, to_value=200.0, step=0.0)],
        )
        self.control_rate_hz = float(
            self.declare_parameter("control_rate_hz", 50.0, rate_descriptor).value
        )

        timeout_descriptor = ParameterDescriptor(
            description=(
                "Seconds since the last received Twist after which this node commands zero "
                "instead of the last-converted command (see this module's docstring for why "
                "zero is republished continuously rather than the node going silent)."
            ),
            floating_point_range=[FloatingPointRange(from_value=0.01, to_value=60.0, step=0.0)],
        )
        self.twist_timeout_s = float(
            self.declare_parameter("twist_timeout_s", 0.5, timeout_descriptor).value
        )

        input_topic_descriptor = ParameterDescriptor(
            description=(
                "Topic this node subscribes geometry_msgs/Twist on (Foxglove's Teleop panel "
                "default; see docs/notes/milestone-5-browser-teleop.md)."
            ),
        )
        self.input_topic = str(
            self.declare_parameter("input_topic", "/teleop/cmd_vel", input_topic_descriptor).value
        )

        vehicle_params = load_vehicle_params()
        self._config = build_twist_teleop_config(vehicle_params)

        self._latest_command: DriveCommand = _ZERO_COMMAND
        self._last_twist_monotonic: float | None = None

        twist_qos = QoSProfile(
            reliability=ReliabilityPolicy.RELIABLE, history=HistoryPolicy.KEEP_LAST, depth=10
        )
        self._twist_sub = self.create_subscription(
            Twist, self.input_topic, self._on_twist, twist_qos
        )

        drive_qos = QoSProfile(
            reliability=ReliabilityPolicy.RELIABLE, history=HistoryPolicy.KEEP_LAST, depth=10
        )
        self._drive_pub = self.create_publisher(AckermannDriveStamped, "/drive_raw", drive_qos)

        self._timer = self.create_timer(1.0 / self.control_rate_hz, self._on_timer)

        self.get_logger().info(
            f"twist_teleop_adapter up: subscribing Twist on '{self.input_topic}', publishing "
            f"/drive_raw at {self.control_rate_hz:.1f} Hz, wheelbase "
            f"{self._config.wheelbase_m:.4f} m, twist_timeout_s={self.twist_timeout_s:.2f}."
        )

    def _on_twist(self, msg: Twist) -> None:
        self._latest_command = convert_twist_to_command(self._config, msg.linear.x, msg.angular.z)
        self._last_twist_monotonic = self.get_clock().now().nanoseconds / 1e9

    def _elapsed_since_last_twist_s(self) -> float | None:
        if self._last_twist_monotonic is None:
            return None
        now_s = self.get_clock().now().nanoseconds / 1e9
        return now_s - self._last_twist_monotonic

    def _on_timer(self) -> None:
        elapsed = self._elapsed_since_last_twist_s()
        command = (
            _ZERO_COMMAND
            if should_use_zero_command(elapsed, self.twist_timeout_s)
            else self._latest_command
        )
        msg = AckermannDriveStamped()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.drive.steering_angle = command.steering_angle_rad
        msg.drive.speed = command.speed_mps
        self._drive_pub.publish(msg)


def main(args: list | None = None) -> None:
    rclpy.init(args=args)
    node = TwistTeleopAdapterNode()
    try:
        rclpy.spin(node)
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()
