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

from racer_tools.keymap import TeleopState, apply_key, build_teleop_config, decode_key
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

        vehicle_params = load_vehicle_params()
        self._config = build_teleop_config(vehicle_params, self.control_rate_hz)
        self._state = TeleopState()

        drive_qos = QoSProfile(
            reliability=ReliabilityPolicy.RELIABLE, history=HistoryPolicy.KEEP_LAST, depth=10
        )
        self._drive_pub = self.create_publisher(AckermannDriveStamped, "/drive_raw", drive_qos)

        self.get_logger().info(
            "keyboard_teleop up: WASD or arrows to steer/throttle, SPACE to stop, q to quit. "
            f"steering step {self._config.steering_step_rad:.4f} rad, speed step "
            f"{self._config.speed_step_mps:.4f} m/s, publishing /drive_raw at "
            f"{self.control_rate_hz:.1f} Hz."
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
    node = KeyboardTeleopNode()
    period_s = 1.0 / node.control_rate_hz
    try:
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
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()
