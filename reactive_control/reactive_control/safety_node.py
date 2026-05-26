"""
LiDAR safety node.

Monitors forward obstacle distance using LaserScan data and vehicle motion to
compute time-to-collision (TTC). Applies staged braking, publishes a gated speed
command, and triggers a latched emergency stop when necessary. Also supports
automatic recovery once the forward path clears.
"""

import rclpy
import signal
import threading
import numpy as np
from rclpy.node import Node
from nav_msgs.msg import Odometry
from ackermann_msgs.msg import AckermannDriveStamped
from sensor_msgs.msg import LaserScan
from std_msgs.msg import Bool


class SafetyNode(Node):
    """
    Reactive LiDAR-based collision avoidance controller.

    Uses LaserScan data to evaluate obstacle proximity in the direction of
    travel (based on the current steering angle), computes minimum distance and
    TTC, then applies staged braking:

        - NONE: Normal operation
        - PB1:  Partial brake (mild)
        - PB2:  Partial brake (stronger)
        - FB:   Full brake (latched emergency stop)

    Includes automatic recovery logic and controlled shutdown behaviour.
    """

    def __init__(self) -> None:
        """
        Initialize the safety node.

        Sets up ROS publishers/subscribers, declares safety thresholds,
        initializes internal state, and registers a signal handler for
        graceful shutdown.

        Returns:
            None
        """
        super().__init__('safety_node')

        odom_topic = self.declare_parameter('odom_topic', '/odom').value

        self.drive_sub = self.create_subscription(AckermannDriveStamped, '/drive', self.drive_callback, 10)
        self.lidar_sub = self.create_subscription(LaserScan, '/scan', self.lidar_callback, 10)
        self.velocity_sub = self.create_subscription(Odometry, odom_topic, self.velocity_callback, 10)

        self.speed_pub = self.create_publisher(AckermannDriveStamped, '/speed', 10)
        self.kys_pub = self.create_publisher(Bool, '/kys', 10)
        self.drive_pub = self.create_publisher(AckermannDriveStamped, '/drive', 10)

        self.distance_threshold = self.declare_parameter('distance_threshold', 0.4).value
        self.ttc_pb1 = self.declare_parameter('ttc_pb1', 1.85).value
        self.ttc_pb2 = self.declare_parameter('ttc_pb2', 1.55).value
        self.ttc_fb = self.declare_parameter('ttc_fb', 0.8).value

        self.timer = self.create_timer(0.5, self.timer_callback)

        self.kys = False
        self.last_vx = 0.0
        self.last_angle = 0.0
        self.sysready = False

        kys_msg = Bool()
        kys_msg.data = False
        self.kys_pub.publish(kys_msg)

        self.ranges = None
        self.winding_down = False
        signal.signal(signal.SIGINT, self._sigint_handler)

    def lidar_callback(self, msg: LaserScan) -> None:
        """
        LiDAR callback for collision detection.

        Processes LaserScan data to determine obstacle distance in the
        direction of motion, computes TTC, selects a braking stage, and
        publishes the corresponding speed command.

        Args:
            msg (LaserScan): Incoming LiDAR scan data.

        Returns:
            None
        """
        ranges = np.array(msg.ranges)
        ranges = np.clip(ranges, msg.range_min, msg.range_max)

        target_ray = int(self.last_angle / msg.angle_increment + len(ranges) // 2)
        target_ray = np.clip(target_ray, 0, len(ranges) - 1)

        danger_zone = int(np.arctan2(0.7, ranges[target_ray]) * 180 * 4 / 3.14)

        lower = target_ray - danger_zone if target_ray - danger_zone > 0 else 0
        upper = target_ray + danger_zone + 1 if target_ray + danger_zone + 1 < len(ranges) - 1 else len(ranges) - 1
        danger_rays = ranges[lower : upper]

        min_distance = np.min(danger_rays)

        if self.last_vx == 0:
            self.last_vx = 1.2

        ttc = min_distance / self.last_vx if self.last_vx > 0 else float('inf')

        drive_msg = AckermannDriveStamped()

        # FB
        if min_distance < self.distance_threshold or ttc < self.ttc_fb:
            drive_msg.drive.speed = 0.0
            drive_msg.drive.steering_angle = self.last_angle

            self.kys = True
            kys_msg = Bool()
            kys_msg.data = True
            self.kys_pub.publish(kys_msg)

            self.drive_pub.publish(drive_msg)
            self.get_logger().info(f"FB - Distance: {min_distance:.2f}m, TTC: {ttc:.2f}s")

        # PB2
        elif ttc < self.ttc_pb2:
            drive_msg.drive.speed = 0.75

            self.get_logger().info(f"PARTIAL BRAKE 2 - Distance: {min_distance:.2f}m, TTC: {ttc:.2f}s")

        # PB1
        elif ttc < self.ttc_pb1:
            drive_msg.drive.speed = 1.4

            self.get_logger().info(f"PARTIAL BRAKE 1 - Distance: {min_distance:.2f}m, TTC: {ttc:.2f}s")

        # NONE
        else:
            drive_msg.drive.speed = 2.0

            self.get_logger().info(f"NONE - Distance: {min_distance:.2f}m, TTC: {ttc:.2f}s")

        if self.winding_down:
            drive_msg.drive.speed = min(drive_msg.drive.speed, 0.0)

        self.speed_pub.publish(drive_msg)

    def timer_callback(self) -> None:
        """
        Periodic recovery check.

        If an emergency stop is active, checks whether the forward region is
        clear and, if so, releases the stop condition.

        Returns:
            None
        """
        if self.kys and self.ranges is not None:
            danger_forward = self.ranges[len(self.ranges) // 2 - 60 : len(self.ranges) // 2 + 60]
            if np.min(danger_forward) > 0.5:
                self.kys = False
                kys_msg = Bool()
                kys_msg.data = False
                self.kys_pub.publish(kys_msg)

    def _sigint_handler(self, sig, frame) -> None:
        """
        Graceful shutdown handler.

        Initiates a controlled slowdown when Ctrl+C is pressed, allowing other
        nodes to terminate safely before shutting down ROS. A second SIGINT
        forces an immediate shutdown.

        Args:
            sig: Signal number.
            frame: Current stack frame.

        Returns:
            None
        """
        if self.winding_down:
            rclpy.shutdown()
            return
        self.get_logger().info("Safety node winding down...")
        self.winding_down = True
        # Slightly longer than gap_follow_node so steering keeps updating during slowdown
        threading.Timer(4.5, lambda: rclpy.shutdown()).start()

    def drive_callback(self, msg: AckermannDriveStamped) -> None:
        """
        Store the latest commanded steering angle and speed.

        Args:
            msg (AckermannDriveStamped): Drive command message.

        Returns:
            None
        """
        self.last_angle = msg.drive.steering_angle
        self.last_vx = msg.drive.speed

    def velocity_callback(self, msg: Odometry) -> None:
        """
        Update vehicle velocity from odometry.

        Args:
            msg (Odometry): Odometry message containing linear velocity.

        Returns:
            None
        """
        self.last_vx = abs(msg.twist.twist.linear.x)


def main(args=None) -> None:
    """
    Entry point for the safety node.

    Args:
        args: Command-line arguments passed to rclpy.

    Returns:
        None
    """
    rclpy.init(args=args)
    safety_node = SafetyNode()
    rclpy.spin(safety_node)
    safety_node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
