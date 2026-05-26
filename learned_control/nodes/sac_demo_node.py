"""
ROS2 demo node for the SAC model.

Subscribes to /scan (LaserScan), runs the SAC model, and publishes AckermannDriveStamped to /drive_raw.

Usage (standalone):
    ros2 run learned_control sac_demo_node \
        --ros-args -p checkpoint_path:=sac/sac_checkpoint_best.pth \
                   -p scalers_path:=processed/scalers.npz \
                   -p max_speed:=1.0

Paths: ~/f1tenth_ws/src/learned_control/...
Runs the SAC demo node with the safety node.

Launch:
    ros2 launch learned_control sac_demo_launch.py
"""
from __future__ import annotations

import numpy as np
import torch
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import LaserScan
from ackermann_msgs.msg import AckermannDriveStamped
from std_msgs.msg import Bool
from learned_control.sac.model import SACActorNet

# Must match preprocessing constants in preprocessing.py (for LiDAR downsampling)
LIDAR_STEP = 6
MAX_RANGE = 10.0


class SACDemoNode(Node):
    """
    This class defines the SAC demo node.
    """

    def __init__(self):
        """
        Initializes the SAC demo node.

        Args:
            None

        Returns:
            None
        """
        super().__init__("sac_demo_node")

        # Parameters
        self.declare_parameter("checkpoint_path", "")
        self.declare_parameter("scalers_path", "")
        self.declare_parameter("max_speed", 1.0)
        self.declare_parameter("min_speed", 0.5)

        # Get the parameters from the launch file
        checkpoint_path = self._str("checkpoint_path")
        scalers_path = self._str("scalers_path")
        self.max_speed = self._dbl("max_speed")
        self.min_speed = self._dbl("min_speed")

        # Load the scalers from the .npz file
        scalers = np.load(scalers_path)
        self.lidar_scale = scalers["lidar_scale"].astype(np.float32)
        self.lidar_min = scalers["lidar_min"].astype(np.float32)
        self.action_scale = scalers["action_scale"].astype(np.float32)
        self.action_min = scalers["action_min"].astype(np.float32)
        self.num_lidar = len(self.lidar_scale)

        # Load the actor model from the checkpoint file
        self.device = torch.device("cuda" if torch.cuda.is_available() else "cpu")
        self.actor = SACActorNet(self.num_lidar).to(self.device)
        ckpt = torch.load(checkpoint_path, map_location=self.device, weights_only=False)
        self.actor.load_state_dict(ckpt["actor"])
        self.actor.eval()

        self.get_logger().info(
            f"SAC DEMO ready | {self.num_lidar} lidar features | "
            f"device={self.device} | checkpoint={checkpoint_path}")

        # Emergency stop flag and step count
        self.stopped = False
        self.step_count = 0
        self.prev_steering = 0.0
        self.prev_speed_cmd = 0.0

        # ROS2 subscribers and publishers
        self.scan_sub = self.create_subscription(
            LaserScan, "/scan", self.scan_callback, 10)
        self.kys_sub = self.create_subscription(
            Bool, "/kys", self.kys_callback, 10)
        self.drive_pub = self.create_publisher(
            AckermannDriveStamped, "/drive_raw", 10)

    def _str(self, n) -> str:
        """
        Helper function to get the string value of a parameter.

        Args:
            n: The name of the parameter.

        Returns:
            The string value of the parameter.
        """
        return self.get_parameter(n).get_parameter_value().string_value

    def _dbl(self, n) -> float:
        """
        Helper function to get the double value of a parameter.

        Args:
            n: The name of the parameter.

        Returns:
            The double value of the parameter.
        """
        return self.get_parameter(n).get_parameter_value().double_value

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
        raw = np.array(msg.ranges, dtype=np.float32)

        # Preprocess the LiDAR data to the range [0, 1]
        ds = raw[::LIDAR_STEP]
        ds = np.where(np.isfinite(ds), ds, MAX_RANGE)
        ds = np.clip(ds, 0.0, MAX_RANGE)
        if len(ds) > self.num_lidar:
            ds = ds[:self.num_lidar]
        elif len(ds) < self.num_lidar:
            ds = np.pad(ds, (0, self.num_lidar - len(ds)),
                        constant_values=MAX_RANGE)
        state = ds * self.lidar_scale + self.lidar_min

        # Run the SAC model to get the predicted steering angle and speed
        with torch.no_grad():
            state_t = torch.from_numpy(state.reshape(1, -1)).to(self.device)
            action = self.actor.get_action(state_t, deterministic=True)
            action = action.cpu().numpy()[0]

        # Denormalize the predicted steering angle and speed to the original range
        steering = float((action[0] - self.action_min[0]) / self.action_scale[0])
        speed = float((action[1] - self.action_min[1]) / self.action_scale[1])
        steering, speed = self._postprocess_action(steering, speed, ds)

        # Publish the steering angle and speed
        drive_msg = AckermannDriveStamped()
        drive_msg.drive.steering_angle = steering
        drive_msg.drive.speed = speed
        self.drive_pub.publish(drive_msg)

        # Update the previous steering and speed commands
        self.prev_steering = steering
        self.prev_speed_cmd = speed

        # Update the step count
        self.step_count += 1

        # Log the steering and speed if the step count is less than 10
        if self.step_count <= 10:
            self.get_logger().info(f"[DRIVE #{self.step_count}] steer={steering:.4f} speed={speed:.4f}")

    def kys_callback(self, msg: Bool) -> None:
        """
        Callback function for the KYS topic.

        Args:
            msg: The Bool message.

        Returns:
            None
        """
        # If the KYS is latched, set the emergency stop flag to True
        if msg.data and not self.stopped:
            self.stopped = True
            self.get_logger().info("Emergency stop latched")
        elif not msg.data and self.stopped:
            self.stopped = False
            self.get_logger().info("Emergency stop released")

    def _publish_stop(self) -> None:
        """
        Publish a stop message.

        Args:
            None

        Returns:
            None
        """
        # Publish a stop message
        msg = AckermannDriveStamped()
        msg.drive.speed = 0.0
        msg.drive.steering_angle = 0.0
        self.drive_pub.publish(msg)

    def _postprocess_action(self, steering, speed, raw_lidar) -> tuple[float, float]:
        """
        Filter the predicted steering angle and speed.

        Args:
            steering: The predicted steering angle.
            speed: The predicted speed.
            raw_lidar: The raw LiDAR data.
        """
        # If the steering or speed is not finite, set it to 0.0
        if not np.isfinite(steering):
            steering = 0.0
        if not np.isfinite(speed):
            speed = self.min_speed

        # Clamp the speed to the range [0.0, self.max_speed]
        speed = max(0.0, min(speed, self.max_speed))

        # If the speed is less than the minimum speed, set it to the minimum speed
        if 0.0 < speed < self.min_speed:
            speed = self.min_speed

        return steering, speed


def main(args=None):
    """
    Main function to initialize the ROS2 node.

    Args:
        args: The arguments.

    Returns:
        None
    """
    rclpy.init(args=args)
    node = SACDemoNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        node.get_logger().info("Shutting down")
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
