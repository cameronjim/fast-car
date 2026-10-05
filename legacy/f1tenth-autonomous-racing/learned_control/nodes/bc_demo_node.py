"""ros node running the trained behavioural cloning policy on live scans."""

import numpy as np
import torch
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import LaserScan
from ackermann_msgs.msg import AckermannDriveStamped
from std_msgs.msg import Bool
from learned_control.bc.model import BCNet
from learned_control.preprocessing.scan import downsample_scan, normalize_scan


class BcDemoNode(Node):
    """behavioural cloning inference node publishing on /drive_raw."""

    def __init__(self) -> None:
        super().__init__("bc_demo_node")

        self.declare_parameter("model_path", "bc/bc_model.pth")
        self.declare_parameter("scalers_path", "processed/scalers.npz")
        self.declare_parameter("max_speed", 1.0)
        self.declare_parameter("min_speed", 0.5)
        self.declare_parameter("safety_distance", 0.3)

        model_path = self.get_parameter("model_path").get_parameter_value().string_value
        scalers_path = self.get_parameter("scalers_path").get_parameter_value().string_value
        self.max_speed = self.get_parameter("max_speed").get_parameter_value().double_value
        self.min_speed = self.get_parameter("min_speed").get_parameter_value().double_value
        self.safety_distance = self.get_parameter("safety_distance").get_parameter_value().double_value

        scalers = np.load(scalers_path)
        self.lidar_scale = scalers["lidar_scale"].astype(np.float32)
        self.lidar_min = scalers["lidar_min"].astype(np.float32)
        self.action_scale = scalers["action_scale"].astype(np.float32)
        self.action_min = scalers["action_min"].astype(np.float32)

        # the exported scaler length is what fixes the policy input width
        self.num_lidar = len(self.lidar_scale)
        self.get_logger().info(f"lidar features: {self.num_lidar}")

        self.device = torch.device("cpu")
        self.model = BCNet(num_lidar_rays=self.num_lidar).to(self.device)
        self.model.load_state_dict(torch.load(model_path, map_location=self.device))
        self.model.eval()

        self.get_logger().info(f"loaded model from {model_path} on {self.device}")

        self.stopped = False

        self.scan_sub = self.create_subscription(LaserScan, "/scan", self.scan_callback, 10)
        self.kys_sub = self.create_subscription(Bool, "/kys", self.kys_callback, 10)
        self.drive_pub = self.create_publisher(AckermannDriveStamped, "/drive_raw", 10)

    def scan_callback(self, msg: LaserScan) -> None:
        if self.stopped:
            self._publish_stop()
            return

        ranges_m = downsample_scan(msg.ranges, self.num_lidar)
        lidar_norm = normalize_scan(ranges_m, self.lidar_scale, self.lidar_min)

        with torch.no_grad():
            policy_input = torch.from_numpy(lidar_norm.reshape(1, -1)).to(self.device)
            action = self.model(policy_input).cpu().numpy()[0]

        steering_angle = float((action[0] - self.action_min[0]) / self.action_scale[0])
        speed = float((action[1] - self.action_min[1]) / self.action_scale[1])

        speed = max(self.min_speed, min(speed, self.max_speed))

        drive_msg = AckermannDriveStamped()
        drive_msg.drive.steering_angle = steering_angle
        drive_msg.drive.speed = speed
        self.drive_pub.publish(drive_msg)

    def kys_callback(self, msg: Bool) -> None:
        if msg.data:
            self.stopped = True
            self.get_logger().info("emergency stop latched")
        else:
            self.stopped = False
            self.get_logger().info("emergency stop released")

    def _publish_stop(self) -> None:
        drive_msg = AckermannDriveStamped()
        drive_msg.drive.speed = 0.0
        drive_msg.drive.steering_angle = 0.0
        self.drive_pub.publish(drive_msg)


def main(args=None):
    rclpy.init(args=args)
    node = BcDemoNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()
