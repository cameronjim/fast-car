"""ros node running a policy exported by gym_training, obs rebuilt from its obs_config.json."""
from __future__ import annotations

import rclpy
from ackermann_msgs.msg import AckermannDriveStamped
from nav_msgs.msg import Odometry
from rclpy.node import Node
from sensor_msgs.msg import LaserScan
from std_msgs.msg import Bool

from learned_control.rl_policy_runtime import RLPolicyRuntime

STARTUP_LOG_STEPS = 10
SCAN_ERROR_INTERVAL_NS = 2_000_000_000


class RLAgentNode(Node):
    """runs an exported policy on /scan, delta approximated by the last commanded steering."""

    def __init__(self):
        super().__init__("rl_agent_node")

        self.declare_parameter("policy_path", "")
        self.declare_parameter("obs_config_path", "")
        self.declare_parameter("odom_topic", "/ego_racecar/odom")

        policy_path = self._str("policy_path")
        obs_config_path = self._str("obs_config_path")
        odom_topic = self._str("odom_topic")

        self.runtime = RLPolicyRuntime.from_files(policy_path, obs_config_path)
        contract = self.runtime.contract
        self.get_logger().info(
            f"rl agent ready | {contract.num_beams} beams | obs {contract.obs_dim} | "
            f"{contract.control_hz:.1f} hz | steer +-{contract.steer_max_rad:.4f} rad | "
            f"speed {contract.speed_min_mps:.2f}-{contract.speed_cap_mps:.2f} m/s | "
            f"contract={obs_config_path}")

        self.stopped = False
        self.step_count = 0
        self.last_scan_error_ns = 0

        self.scan_sub = self.create_subscription(
            LaserScan, "/scan", self.scan_callback, 10)
        self.odom_sub = self.create_subscription(
            Odometry, odom_topic, self.odom_callback, 10)
        self.kys_sub = self.create_subscription(
            Bool, "/kys", self.kys_callback, 10)
        self.drive_pub = self.create_publisher(
            AckermannDriveStamped, "/drive_raw", 10)

    def _str(self, name) -> str:
        return self.get_parameter(name).get_parameter_value().string_value

    def scan_callback(self, msg: LaserScan) -> None:
        if self.stopped:
            self._publish_stop()
            return

        now_sec = self.get_clock().now().nanoseconds * 1e-9
        try:
            # driving rides on /scan, so the runtime decimates a lidar faster than control_hz
            command = self.runtime.step(msg.ranges, now_sec)
        except ValueError as err:
            self._log_scan_error(str(err))
            self._publish_stop()
            return
        if command is None:
            return

        drive_msg = AckermannDriveStamped()
        drive_msg.drive.steering_angle = command.steering_rad
        drive_msg.drive.speed = command.speed_mps
        self.drive_pub.publish(drive_msg)

        self.step_count += 1
        if self.step_count <= STARTUP_LOG_STEPS:
            self.get_logger().info(
                f"[drive #{self.step_count}] steer={command.steering_rad:.4f} "
                f"speed={command.speed_mps:.4f}")

    def odom_callback(self, msg: Odometry) -> None:
        twist = msg.twist.twist
        self.runtime.update_odom(twist.linear.x, twist.linear.y, twist.angular.z)

    def kys_callback(self, msg: Bool) -> None:
        if msg.data and not self.stopped:
            self.stopped = True
            # a latched stop repositions the car, so the policy restarts from a clean history
            self.runtime.reset()
            self.get_logger().info("emergency stop latched")
        elif not msg.data and self.stopped:
            self.stopped = False
            self.get_logger().info("emergency stop released")

    def _publish_stop(self) -> None:
        msg = AckermannDriveStamped()
        msg.drive.speed = 0.0
        msg.drive.steering_angle = 0.0
        self.drive_pub.publish(msg)

    def _log_scan_error(self, reason: str) -> None:
        now_ns = self.get_clock().now().nanoseconds
        if now_ns - self.last_scan_error_ns < SCAN_ERROR_INTERVAL_NS:
            return
        self.last_scan_error_ns = now_ns
        self.get_logger().error(f"holding stop, cannot build the observation: {reason}")


def main(args=None):
    rclpy.init(args=args)
    node = RLAgentNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        node.get_logger().info("shutting down")
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()
