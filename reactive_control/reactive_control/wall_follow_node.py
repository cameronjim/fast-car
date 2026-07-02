"""ros node following the right-hand wall with a pid on the lookahead distance error."""

import rclpy
import signal
import threading
import numpy as np
from reactive_control.pid import PID
from reactive_control.wall_logic import wall_distance_error
from rclpy.node import Node
from sensor_msgs.msg import LaserScan
from nav_msgs.msg import Odometry
from ackermann_msgs.msg import AckermannDriveStamped
from std_msgs.msg import Bool


class WallFollowNode(Node):
    """reactive wall-following controller; speed is capped by the safety node."""

    def __init__(self) -> None:
        super().__init__('wall_follow_node')

        odom_topic = self.declare_parameter('odom_topic', '/odom').value

        self.scan_sub = self.create_subscription(LaserScan, '/scan', self.listener_callback, 10)
        self.kys_sub = self.create_subscription(Bool, '/kys', self.kys_callback, 10)
        self.vel_sub = self.create_subscription(Odometry, odom_topic, self.velocity_callback, 10)
        self.speed_sub = self.create_subscription(AckermannDriveStamped, '/speed', self.speed_callback, 10)
        self.publisher_ = self.create_publisher(AckermannDriveStamped, '/drive', 10)

        self.K_p = self.declare_parameter('K_p', 1.5).value
        self.K_i = self.declare_parameter('K_i', 0.0).value
        self.K_d = self.declare_parameter('K_d', 0.02).value

        self.target_distance = self.declare_parameter('target_distance', 1.0).value
        self.max_speed = self.declare_parameter('max_speed', 1.0).value
        self.min_speed = self.declare_parameter('min_speed', 0.1).value
        self.K_speed = self.declare_parameter('K_speed', 1.0).value

        self.shutdown_speed = self.declare_parameter('shutdown_speed', 0.0).value
        self.shutdown_duration = self.declare_parameter('shutdown_duration', 2.0).value

        self.kys = self.declare_parameter('kys_latched', False).value

        self.pid = PID(self.K_p, self.K_i, self.K_d)
        self.speed = 0.0
        self.last_vel = 0.0
        self.prev_time = None
        self.winding_down = False

        self.get_logger().info("wall node initialized")

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

        if self.prev_time is None:
            dt = 0.0
        else:
            dt = current_time - self.prev_time

        error = wall_distance_error(msg.ranges, msg.range_min, msg.range_max,
                                    msg.angle_min, msg.angle_increment,
                                    self.target_distance, self.last_vel, dt)
        pid_angle = self.pid.pid_err(error, current_time)

        drive_msg = AckermannDriveStamped()
        drive_msg.drive.steering_angle = pid_angle

        # slow down on sharp turns, then cap at the safety-allowed speed
        desired = self.max_speed - np.abs(pid_angle) * self.K_speed
        desired = max(desired, self.min_speed)
        speed = min(desired, self.speed)

        if self.winding_down:
            speed = min(speed, self.shutdown_speed)

        drive_msg.drive.speed = float(speed)
        self.publisher_.publish(drive_msg)
        self.prev_time = current_time

    def kys_callback(self, msg: Bool) -> None:
        self.kys = msg.data

    def velocity_callback(self, msg: Odometry) -> None:
        self.last_vel = abs(msg.twist.twist.linear.x)

    def speed_callback(self, msg: AckermannDriveStamped) -> None:
        self.speed = msg.drive.speed


def main(args=None) -> None:
    rclpy.init(args=args)
    wall_follow_node = WallFollowNode()
    rclpy.spin(wall_follow_node)
    wall_follow_node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
