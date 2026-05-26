"""
ROS2 demo node for the Behavioural Cloning model.

Subscribes to /scan (LaserScan), runs the BC model, and publishesAckermannDriveStamped to /drive_raw.

Usage (standalone):
    ros2 run learned_control bc_demo_node \
        --ros-args -p model_path:=bc/bc_model.pth \
                   -p scalers_path:=processed/scalers.npz \
                   -p max_speed:=1.0

Paths: ~/f1tenth_ws/src/learned_control/...
Runs the BC demo node with the safety node.

Launch:
    ros2 launch learned_control bc_launch.py
"""

import numpy as np
import torch
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import LaserScan
from ackermann_msgs.msg import AckermannDriveStamped
from std_msgs.msg import Bool
from learned_control.bc.model import BCNet

# Must match preprocessing constants in preprocessing.py (for LiDAR downsampling)
LIDAR_STEP = 6
MAX_RANGE = 10.0


class BcDemoNode(Node):
    """
    This class defines the BC demo node.
    """

    def __init__(self) -> None:
        """
        Initializes the BC demo node.

        Args:
            None

        Returns:
            None
        """
        super().__init__("bc_demo_node")

        # Parameters
        self.declare_parameter("model_path", "bc/bc_model.pth")
        self.declare_parameter("scalers_path", "processed/scalers.npz")
        self.declare_parameter("max_speed", 1.0)
        self.declare_parameter("min_speed", 0.5)
        self.declare_parameter("safety_distance", 0.3)

        # Get the parameters from the launch file
        model_path = self.get_parameter("model_path").get_parameter_value().string_value
        scalers_path = self.get_parameter("scalers_path").get_parameter_value().string_value
        self.max_speed = self.get_parameter("max_speed").get_parameter_value().double_value
        self.min_speed = self.get_parameter("min_speed").get_parameter_value().double_value
        self.safety_distance = self.get_parameter("safety_distance").get_parameter_value().double_value

        # Load the scalers from the .npz file
        scalers = np.load(scalers_path)
        self.lidar_scale = scalers["lidar_scale"].astype(np.float32)
        self.lidar_min = scalers["lidar_min"].astype(np.float32)
        self.action_scale = scalers["action_scale"].astype(np.float32)
        self.action_min = scalers["action_min"].astype(np.float32)

        # Get the number of LiDAR rays
        self.num_lidar = len(self.lidar_scale)
        self.get_logger().info(f"LiDAR features: {self.num_lidar}")

        # Load model (use CPU)
        self.device = torch.device("cpu")
        self.model = BCNet(num_lidar_rays=self.num_lidar).to(self.device)
        self.model.load_state_dict(torch.load(model_path, map_location=self.device))
        self.model.eval()

        # Debugging log
        self.get_logger().info(f"Loaded model from {model_path} on {self.device}")

        # Emergency stop flag
        self.stopped = False

        # ROS2 subscribers and publishers
        self.scan_sub = self.create_subscription(LaserScan, "/scan", self.scan_callback, 10)
        self.kys_sub = self.create_subscription(Bool, "/kys", self.kys_callback, 10)
        self.drive_pub = self.create_publisher(AckermannDriveStamped, "/drive_raw", 10)

    def scan_callback(self, msg: LaserScan) -> None:
        """
        Callback function for the LaserScan topic.

        Args:
            msg: The LaserScan message.

        Returns:
            None
        """
        # If the emergency stop is latched, publish a stop message
        if self.stopped:
            self._publish_stop()
            return

        # Convert the LaserScan message to a numpy array
        ranges = np.array(msg.ranges, dtype=np.float32)

        # Downsample raw scan (keep every 10th ray)
        downsampled = ranges[::LIDAR_STEP]

        # Clamp infinities / NaN to MAX_RANGE, clip to [0, MAX_RANGE]
        downsampled = np.where(np.isfinite(downsampled), downsampled, MAX_RANGE)
        downsampled = np.clip(downsampled, 0.0, MAX_RANGE)

        # Ensure the length of the array matches the number of LiDAR rays
        if len(downsampled) != self.num_lidar:
            if len(downsampled) > self.num_lidar:
                downsampled = downsampled[: self.num_lidar]
            else:
                downsampled = np.pad(
                    downsampled, (0, self.num_lidar - len(downsampled)),
                    constant_values=MAX_RANGE,
                )

        # Normalize the LiDAR data to the range [0, 1]
        lidar_norm = downsampled * self.lidar_scale + self.lidar_min

        # Run the BC model to get the predicted steering angle and speed
        with torch.no_grad():
            x = torch.from_numpy(lidar_norm.reshape(1, -1)).to(self.device)
            pred = self.model(x).cpu().numpy()[0]  # shape (2,)

        # Denormalize the predicted steering angle and speed to the original range
        steering_angle = float((pred[0] - self.action_min[0]) / self.action_scale[0])
        speed = float((pred[1] - self.action_min[1]) / self.action_scale[1])

        # Clamp the speed to the range [min_speed, max_speed]
        speed = max(self.min_speed, min(speed, self.max_speed))

        # Publish speed and steering angle
        drive_msg = AckermannDriveStamped()
        drive_msg.drive.steering_angle = steering_angle
        drive_msg.drive.speed = speed
        self.drive_pub.publish(drive_msg)

    def kys_callback(self, msg: Bool) -> None:
        """
        Callback function for the KYS topic.

        Args:
            msg: The Bool message.

        Returns:
            None
        """
        # If the KYS is latched, set the emergency stop flag to True
        if msg.data:
            self.stopped = True
            self.get_logger().info("Emergency stop latched (KYS)")
        else:
            self.stopped = False
            self.get_logger().info("Emergency stop released (KYS)")

    def _publish_stop(self) -> None:
        """
        Publish a stop message.

        Args:
            None

        Returns:
            None
        """
        # Publish a stop message
        drive_msg = AckermannDriveStamped()
        drive_msg.drive.speed = 0.0
        drive_msg.drive.steering_angle = 0.0
        self.drive_pub.publish(drive_msg)


def main(args=None):
    """
    Main function to initialize the ROS2 node.

    Args:
        args: The arguments.

    Returns:
        None
    """
    rclpy.init(args=args)
    node = BcDemoNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()
