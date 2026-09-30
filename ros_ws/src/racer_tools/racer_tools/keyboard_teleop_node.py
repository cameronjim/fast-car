"""keyboard_teleop_node: terminal keyboard teleop (roadmap milestone 1, claude-docs/04-
architecture.md's command path: this node is one of the two possible producers of
/drive_raw; racer_safety/safety_node is the SOLE publisher of /drive and gates
/drive_raw -> /drive -- this node never publishes /drive directly).

Publishes /drive_raw (ackermann_msgs/AckermannDriveStamped, reliable, depth 10) at a fixed
rate (`control_rate_hz` param, default 50 Hz per claude-docs/04-architecture.md). WASD or
arrow keys step throttle/steering; spacebar is immediate zero/stop; 'q' quits after
publishing a final zero command. ALL keymap/step/clamp decision logic lives in
racer_tools.keymap (claude-docs/12-testing.md L1: "Teleop keymap logic pytest", unit-tested
without a TTY); this file is thin termios/rclpy plumbing -- it reads raw terminal bytes,
hands them to racer_tools.keymap.decode_key/apply_key, and publishes whatever state comes
back.

Needs a real interactive TTY on stdin (termios raw/cbreak mode) -- run this in its own
terminal, separate from the launch file that starts the rest of the stack. See
docs/notes/milestone-1-sim-teleop.md for the exact two-terminal procedure.
"""

from __future__ import annotations

import select
import sys
import termios
import tty
from contextlib import contextmanager

import rclpy
from ackermann_msgs.msg import AckermannDriveStamped
from rcl_interfaces.msg import FloatingPointRange, ParameterDescriptor
from rclpy.node import Node
from rclpy.qos import HistoryPolicy, QoSProfile, ReliabilityPolicy

from racer_tools.keymap import (
    TeleopState,
    apply_key,
    build_teleop_config,
    decode_key,
    derived_steps,
)
from racer_tools.vehicle_params_loader import load_vehicle_params


@contextmanager
def raw_terminal_mode(stream):
    """Put `stream` (stdin) into cbreak mode (unbuffered, no line editing, keys available to
    read() one at a time with no Enter needed) for the duration of the context, restoring the
    original terminal settings on exit -- including on an exception -- so a crash never
    leaves the user's terminal in a broken state."""
    fd = stream.fileno()
    old_settings = termios.tcgetattr(fd)
    try:
        tty.setcbreak(fd)
        yield
    finally:
        termios.tcsetattr(fd, termios.TCSADRAIN, old_settings)


def _read_raw_key(stream, timeout_s: float) -> str:
    """Read one keypress's worth of raw bytes from `stream`: a plain character, or a 3-byte
    arrow-key escape sequence (ESC '[' <letter>). Returns "" if nothing arrives within
    `timeout_s`. This is the one piece of I/O this module does NOT push into
    racer_tools.keymap.decode_key (which stays pure and TTY-free, see that module's
    docstring) -- decode_key only ever sees the finished raw string this function hands it.
    """
    ready, _, _ = select.select([stream], [], [], timeout_s)
    if not ready:
        return ""
    first = stream.read(1)
    if first != "\x1b":
        return first
    # Arrow keys send ESC '[' <letter>; give the rest of the sequence a short grace window
    # so a lone ESC keypress (no more bytes coming) doesn't block waiting for bytes that
    # will never arrive.
    ready, _, _ = select.select([stream], [], [], 0.01)
    if not ready:
        return first
    second = stream.read(1)
    ready, _, _ = select.select([stream], [], [], 0.01)
    if not ready:
        return first + second
    third = stream.read(1)
    return first + second + third


class KeyboardTeleopNode(Node):
    def __init__(self) -> None:
        super().__init__("keyboard_teleop")

        rate_descriptor = ParameterDescriptor(
            description="Publish rate for /drive_raw (claude-docs/04-architecture.md: 50 Hz).",
            floating_point_range=[FloatingPointRange(from_value=1.0, to_value=200.0, step=0.0)],
        )
        self.control_rate_hz = float(
            self.declare_parameter("control_rate_hz", 50.0, rate_descriptor).value
        )

        reverse_descriptor = ParameterDescriptor(
            description=(
                "Allow commanding NEGATIVE speed (reverse). Default false: the lower speed "
                "clamp becomes 0.0 m/s instead of vehicle_params limits.min_velocity_mps, so "
                "the throttle-down key decelerates to a stop and stops there. What a "
                "below-neutral throttle pulse physically does depends on the VESC's PPM "
                "control type, which has never been set on this ESC (reverse current in "
                "'Current', proportional braking in 'Current No Reverse With Brake'), so the "
                "first drives do not emit one. Turn on deliberately at the bench once that "
                "setting is known and recorded (docs/notes/first-boot-runbook.md)."
            ),
        )
        self.allow_reverse = bool(
            self.declare_parameter("allow_reverse", False, reverse_descriptor).value
        )

        vehicle_params = load_vehicle_params()
        # Tap sizes (GitHub issue #72). The DEFAULT of each parameter is the derivation from
        # vehicle_params (one control period at the vehicle's maximum rate, see
        # racer_tools.keymap), computed here so `ros2 param describe` shows the real value; a
        # launch file or `-p` overrides it. Validation (finite, > 0) is in keymap, so a bad
        # value refuses to start rather than producing a dead or inverted key.
        derived_steering_step, derived_speed_step = derived_steps(
            vehicle_params, self.control_rate_hz
        )
        speed_step_descriptor = ParameterDescriptor(
            description=(
                "Speed change per throttle key event, m/s (GitHub issue #72). Default is "
                "vehicle_params actuation.max_acceleration_mps2 / control_rate_hz "
                f"({derived_speed_step:.4f} m/s at the current rate). An operator-interface "
                "choice, not a physical constant: car_teleop.launch.py passes a first-drive "
                "profile. Must be finite and > 0; the command is still clamped to the "
                "vehicle_params speed range. Pass a float (0.25, not 1)."
            ),
        )
        min_speed_descriptor = ParameterDescriptor(
            description=(
                "Minimum commanded speed while moving, m/s. Default 0.0 = disabled. When > 0, "
                "a throttle tap from rest sets speed to max(min_speed_mps, speed_step_mps), "
                "and a throttle-down that would land below it stops the car (exactly 0) "
                "instead of leaving a crawl. With allow_reverse the same applies to |speed| "
                "on the negative side. Must be finite, >= 0 and <= the maximum speed or the "
                "node refuses to start. car_teleop.launch.py passes the first-drive profile. "
                "Pass a float (0.8, not 1)."
            ),
        )
        steering_step_descriptor = ParameterDescriptor(
            description=(
                "Steering change per steering key event, rad (GitHub issue #72). Default is "
                "vehicle_params steering.max_rate_rad_per_s / control_rate_hz "
                f"({derived_steering_step:.4f} rad at the current rate). Must be finite and > "
                "0; the command is still clamped to the vehicle_params steering range."
            ),
        )
        speed_step_mps = float(
            self.declare_parameter(
                "speed_step_mps", derived_speed_step, speed_step_descriptor
            ).value
        )
        steering_step_rad = float(
            self.declare_parameter(
                "steering_step_rad", derived_steering_step, steering_step_descriptor
            ).value
        )
        min_speed_mps = float(
            self.declare_parameter("min_speed_mps", 0.0, min_speed_descriptor).value
        )
        self._config = build_teleop_config(
            vehicle_params,
            self.control_rate_hz,
            allow_reverse=self.allow_reverse,
            speed_step_mps=speed_step_mps,
            steering_step_rad=steering_step_rad,
            min_speed_mps=min_speed_mps,
        )
        self._state = TeleopState()

        drive_qos = QoSProfile(
            reliability=ReliabilityPolicy.RELIABLE, history=HistoryPolicy.KEEP_LAST, depth=10
        )
        self._drive_pub = self.create_publisher(AckermannDriveStamped, "/drive_raw", drive_qos)

        self.get_logger().info(
            "keyboard_teleop up: WASD or arrows to steer/throttle, SPACE to stop, q to quit. "
            f"steering step {self._config.steering_step_rad:.4f} rad, speed step "
            f"{self._config.speed_step_mps:.4f} m/s, minimum speed "
            f"{self._config.min_speed_mps:.2f} m/s"
            f"{' (disabled)' if self._config.min_speed_mps == 0.0 else ''}, "
            f"publishing /drive_raw at {self.control_rate_hz:.1f} Hz. Speed range "
            f"[{self._config.speed_min_mps:.2f}, {self._config.speed_max_mps:.2f}] m/s "
            f"(reverse {'ENABLED' if self.allow_reverse else 'disabled'})."
        )

    @property
    def should_quit(self) -> bool:
        return self._state.quit_requested

    def handle_raw_input(self, raw: str) -> None:
        key: str | None = decode_key(raw)
        self._state = apply_key(self._config, self._state, key)

    def publish_current_state(self) -> None:
        msg = AckermannDriveStamped()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.drive.steering_angle = self._state.steering_angle_rad
        msg.drive.speed = self._state.speed_mps
        self._drive_pub.publish(msg)


def main(args: list | None = None) -> None:
    rclpy.init(args=args)
    # `node` starts unbound-but-declared so that if KeyboardTeleopNode() itself raises (e.g.
    # vehicle_params_loader can't find the repo root), the `finally` below has something
    # well-defined to check instead of referencing a name that was never assigned -- that
    # NameError used to mask the real constructor exception (docs/notes/
    # first-boot-audit-2026-09-13.md finding #6).
    node: KeyboardTeleopNode | None = None
    try:
        node = KeyboardTeleopNode()
        period_s = 1.0 / node.control_rate_hz
        with raw_terminal_mode(sys.stdin):
            while rclpy.ok() and not node.should_quit:
                raw = _read_raw_key(sys.stdin, timeout_s=period_s)
                if raw:
                    node.handle_raw_input(raw)
                node.publish_current_state()
                # No subscriptions/timers of our own to service, but spinning once keeps
                # rclpy's own signal/context housekeeping (Ctrl-C -> rclpy.ok() going False)
                # responsive rather than relying solely on the blocking select() above.
                rclpy.spin_once(node, timeout_sec=0.0)
    finally:
        # Only touch `node` if construction actually succeeded -- cleanup must never itself
        # raise on top of (and mask) whatever the `try` block raised.
        if node is not None:
            node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()
