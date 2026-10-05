"""ros node driving toward the largest lidar gap, with a pid on the target bearing."""

import rclpy
import signal
import threading
import numpy as np
from reactive_control.gap_logic import corner_blocked, extend_disparities, select_target_ray
from reactive_control.pid import PID
from rclpy.node import Node
from sensor_msgs.msg import LaserScan
from nav_msgs.msg import Odometry
from ackermann_msgs.msg import AckermannDriveStamped
from std_msgs.msg import Bool

DEFAULT_ANGLE_INCREMENT = float(np.radians(0.25))


class GapFollowNode(Node):
    """reactive gap-following controller; speed comes from the safety node."""

    def __init__(self) -> None:
        super().__init__('gap_follow_node')

        odom_topic = self.declare_parameter('odom_topic', '/odom').value

        self.scan_sub = self.create_subscription(LaserScan, '/scan', self.listener_callback, 10)
        self.kys_sub = self.create_subscription(Bool, '/kys', self.kys_callback, 10)
        self.vel_sub = self.create_subscription(Odometry, odom_topic, self.velocity_callback, 10)
        self.speed_sub = self.create_subscription(AckermannDriveStamped, '/speed', self.speed_callback, 10)
        self.publisher_ = self.create_publisher(AckermannDriveStamped, '/drive', 10)

        self.K_p = self.declare_parameter('K_p', 1.0).value
        self.K_i = self.declare_parameter('K_i', 0.0).value
        self.K_d = self.declare_parameter('K_d', 0.05).value

        self.max_speed = self.declare_parameter('max_speed', 1.0).value
        self.min_speed = self.declare_parameter('min_speed', 0.1).value
        self.K_speed = self.declare_parameter('K_speed', 1.0).value
        self.target_distance = self.declare_parameter('target_distance', 1.0).value

        self.clip_max_range = self.declare_parameter('clip_max_range', 3.5).value
        self.disparity_threshold = self.declare_parameter('disparity_threshold', 1.0).value
        self.vehicle_half_width = self.declare_parameter('vehicle_half_width', 0.5).value
        self.free_space_threshold = self.declare_parameter('free_space_threshold', 1.2).value
        self.corner_min_clearance = self.declare_parameter('corner_min_clearance', 0.2).value
        self.cone_left_fraction = self.declare_parameter('cone_left_fraction', 0.25).value
        self.cone_right_fraction = self.declare_parameter('cone_right_fraction', 0.75).value

        self.shutdown_speed = self.declare_parameter('shutdown_speed', 0.0).value
        self.shutdown_duration = self.declare_parameter('shutdown_duration', 2.0).value

        self.kys = self.declare_parameter('kys_latched', False).value

        self.pid = PID(self.K_p, self.K_i, self.K_d)
        self.speed = 0.0
        self.last_vel = 0.0
        self.winding_down = False
        # refreshed from every scan; the default matches the 0.25 deg/ray hokuyo config
        self.angle_increment = DEFAULT_ANGLE_INCREMENT

        self.get_logger().info("gap node initialized")

        signal.signal(signal.SIGINT, self._sigint_handler)

    def _sigint_handler(self, sig, frame) -> None:
        """first ctrl+c keeps steering while the car coasts, second one quits now."""
        if self.winding_down:
            rclpy.shutdown()
            return
        self.get_logger().info("ctrl+c caught, winding down")
        self.winding_down = True
        self.speed = self.shutdown_speed

        timer = threading.Timer(self.shutdown_duration, self._do_shutdown)
        timer.start()

    def _do_shutdown(self) -> None:
        """publish a final stop command and shut the node down."""
        drive_msg = AckermannDriveStamped()
        drive_msg.drive.speed = 0.0
        drive_msg.drive.steering_angle = 0.0
        self.publisher_.publish(drive_msg)
        self.get_logger().info("shutdown complete")
        rclpy.shutdown()

    def listener_callback(self, msg: LaserScan) -> None:
        current_time = self.get_clock().now().nanoseconds / 1e9

        if msg.angle_increment > 0.0:
            self.angle_increment = float(msg.angle_increment)

        clipped_ranges = np.clip(msg.ranges, msg.range_min, self.clip_max_range)
        filtered_ranges = extend_disparities(
            clipped_ranges, self.disparity_threshold, self.vehicle_half_width,
            self.angle_increment)
        target = select_target_ray(
            filtered_ranges, self.free_space_threshold,
            self.cone_left_fraction, self.cone_right_fraction)

        center = len(filtered_ranges) // 2
        angle = (target - center) * msg.angle_increment
        if corner_blocked(filtered_ranges, angle, self.corner_min_clearance):
            angle = 0

        pid_angle = self.pid.pid_err(angle, current_time)

        drive_msg = AckermannDriveStamped()
        drive_msg.drive.steering_angle = pid_angle

        if self.winding_down:
            self.speed = min(self.speed, self.shutdown_speed)

        drive_msg.drive.speed = self.speed
        self.publisher_.publish(drive_msg)

    def kys_callback(self, msg: Bool) -> None:
        self.kys = msg.data

    def velocity_callback(self, msg: Odometry) -> None:
        self.last_vel = abs(msg.twist.twist.linear.x)

    def speed_callback(self, msg: AckermannDriveStamped) -> None:
        self.speed = msg.drive.speed


def main(args=None) -> None:
    rclpy.init(args=args)
    gap_follow_node = GapFollowNode()
    rclpy.spin(gap_follow_node)
    gap_follow_node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
