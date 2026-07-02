"""ros node running a trained sac checkpoint on live scans, inference only."""
from __future__ import annotations

import numpy as np
import torch
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import LaserScan
from ackermann_msgs.msg import AckermannDriveStamped
from std_msgs.msg import Bool
from learned_control.sac.model import SACActorNet
from learned_control.preprocessing.scan import downsample_scan, normalize_scan

STARTUP_LOG_STEPS = 10


class SACDemoNode(Node):
    """sac inference node publishing on /drive_raw."""

    def __init__(self):
        super().__init__("sac_demo_node")

        self.declare_parameter("checkpoint_path", "")
        self.declare_parameter("scalers_path", "")
        self.declare_parameter("max_speed", 1.0)
        self.declare_parameter("min_speed", 0.5)

        checkpoint_path = self._str("checkpoint_path")
        scalers_path = self._str("scalers_path")
        self.max_speed = self._dbl("max_speed")
        self.min_speed = self._dbl("min_speed")

        scalers = np.load(scalers_path)
        self.lidar_scale = scalers["lidar_scale"].astype(np.float32)
        self.lidar_min = scalers["lidar_min"].astype(np.float32)
        self.action_scale = scalers["action_scale"].astype(np.float32)
        self.action_min = scalers["action_min"].astype(np.float32)
        # the exported scaler length is what fixes the policy input width
        self.num_lidar = len(self.lidar_scale)

        self.device = torch.device("cuda" if torch.cuda.is_available() else "cpu")
        self.actor = SACActorNet(self.num_lidar).to(self.device)
        ckpt = torch.load(checkpoint_path, map_location=self.device, weights_only=False)
        self.actor.load_state_dict(ckpt["actor"])
        self.actor.eval()

        self.get_logger().info(
            f"sac demo ready | {self.num_lidar} lidar features | "
            f"device={self.device} | checkpoint={checkpoint_path}")

        self.stopped = False
        self.step_count = 0
        self.prev_steering = 0.0
        self.prev_speed_cmd = 0.0

        self.scan_sub = self.create_subscription(
            LaserScan, "/scan", self.scan_callback, 10)
        self.kys_sub = self.create_subscription(
            Bool, "/kys", self.kys_callback, 10)
        self.drive_pub = self.create_publisher(
            AckermannDriveStamped, "/drive_raw", 10)

    def _str(self, name) -> str:
        return self.get_parameter(name).get_parameter_value().string_value

    def _dbl(self, name) -> float:
        return self.get_parameter(name).get_parameter_value().double_value

    def scan_callback(self, msg: LaserScan) -> None:
        if self.stopped:
            self._publish_stop()
            return

        ranges_m = downsample_scan(msg.ranges, self.num_lidar)
        state = normalize_scan(ranges_m, self.lidar_scale, self.lidar_min)

        with torch.no_grad():
            state_t = torch.from_numpy(state.reshape(1, -1)).to(self.device)
            action = self.actor.get_action(state_t, deterministic=True)
            action = action.cpu().numpy()[0]

        steering = float((action[0] - self.action_min[0]) / self.action_scale[0])
        speed = float((action[1] - self.action_min[1]) / self.action_scale[1])
        steering, speed = self._postprocess_action(steering, speed)

        drive_msg = AckermannDriveStamped()
        drive_msg.drive.steering_angle = steering
        drive_msg.drive.speed = speed
        self.drive_pub.publish(drive_msg)

        self.prev_steering = steering
        self.prev_speed_cmd = speed

        self.step_count += 1

        if self.step_count <= STARTUP_LOG_STEPS:
            self.get_logger().info(f"[drive #{self.step_count}] steer={steering:.4f} speed={speed:.4f}")

    def kys_callback(self, msg: Bool) -> None:
        if msg.data and not self.stopped:
            self.stopped = True
            self.get_logger().info("emergency stop latched")
        elif not msg.data and self.stopped:
            self.stopped = False
            self.get_logger().info("emergency stop released")

    def _publish_stop(self) -> None:
        msg = AckermannDriveStamped()
        msg.drive.speed = 0.0
        msg.drive.steering_angle = 0.0
        self.drive_pub.publish(msg)

    def _postprocess_action(self, steering, speed) -> tuple[float, float]:
        """clamp the policy output at the boundary it crosses into ros."""
        if not np.isfinite(steering):
            steering = 0.0
        if not np.isfinite(speed):
            speed = self.min_speed

        # never allow reverse commands from the learned policy
        speed = max(0.0, min(speed, self.max_speed))

        if 0.0 < speed < self.min_speed:
            speed = self.min_speed

        return steering, speed


def main(args=None):
    rclpy.init(args=args)
    node = SACDemoNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        node.get_logger().info("shutting down")
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
