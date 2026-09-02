"""ros safety node gating the learned policies: staged braking, latched stop, recovery."""

import rclpy
import signal
import threading
import numpy as np
from rclpy.node import Node
from nav_msgs.msg import Odometry
from ackermann_msgs.msg import AckermannDriveStamped
from sensor_msgs.msg import LaserScan
from std_msgs.msg import Bool
from learned_control.safety_logic import (
    danger_zone_min_range,
    forward_min_range,
    forward_ray,
    time_to_collision,
    wall_steer_bias,
)

STEER_LIMIT_RAD = 0.5
PB1_SPEED_MPS = 2.9
PB2_SPEED_MPS = 2.0
LOG_INTERVAL_NS = 1_000_000_000
SHUTDOWN_DELAY_SEC = 4.5


class SafetyNode(Node):
    """staged braking from lidar ttc: pb1, pb2, then a latched full brake with auto recovery."""

    def __init__(self) -> None:
        super().__init__('safety_node')

        odom_topic = self.declare_parameter(
            'odom_topic', '/odom').value

        self.drive_sub = self.create_subscription(
            AckermannDriveStamped, '/drive_raw', self.drive_callback, 10)
        self.lidar_sub = self.create_subscription(LaserScan, '/scan', self.lidar_callback, 10)
        self.velocity_sub = self.create_subscription(
            Odometry, odom_topic, self.velocity_callback, 10)

        self.kys_pub = self.create_publisher(Bool, '/kys', 10)
        self.drive_pub = self.create_publisher(AckermannDriveStamped, '/drive', 10)

        self.distance_threshold = self.declare_parameter('distance_threshold', 0.4).value
        self.ttc_pb1 = self.declare_parameter('ttc_pb1', 1.85).value
        self.ttc_pb2 = self.declare_parameter('ttc_pb2', 1.55).value
        self.ttc_fb = self.declare_parameter('ttc_fb', 0.8).value
        self.side_margin = self.declare_parameter('side_margin', 0.7).value
        self.wall_steer_gain = self.declare_parameter('wall_steer_gain', 0.35).value
        self.max_wall_steer_bias = self.declare_parameter(
            'max_wall_steer_bias', 0.18).value

        self.recovery_holdoff_sec = self.declare_parameter(
            'recovery_holdoff_sec', 2.0).value
        self.recovery_clearance = self.declare_parameter(
            'recovery_clearance', 0.5).value

        self.timer = self.create_timer(0.5, self.timer_callback)

        self.kys = False
        self.kys_latched_at_ns = None
        self.last_vx = 0.0
        self.last_angle = 0.0
        self.last_drive_msg = AckermannDriveStamped()
        self.sysready = False

        kys_msg = Bool()
        kys_msg.data = False
        self.kys_pub.publish(kys_msg)

        self.ranges = None
        self.winding_down = False
        signal.signal(signal.SIGINT, self._sigint_handler)

        self.last_logged_state = None
        self.last_log_time_ns = 0

        self.get_logger().info("starting safety")

    def log_state(self, state: str, min_distance: float, ttc: float) -> None:
        """log the braking stage, immediately on a change and at most once a second otherwise."""
        now_ns = self.get_clock().now().nanoseconds

        if state != self.last_logged_state or (now_ns - self.last_log_time_ns) >= LOG_INTERVAL_NS:
            self.get_logger().info(
                f"{state} - Distance: {min_distance:.2f}m, TTC: {ttc:.2f}s"
            )
            self.last_logged_state = state
            self.last_log_time_ns = now_ns

    def lidar_callback(self, msg: LaserScan) -> None:
        ranges = np.array(msg.ranges)
        ranges = np.clip(ranges, msg.range_min, msg.range_max)
        # kept for timer_callback's recovery check
        self.ranges = ranges

        target_ray = forward_ray(self.last_angle, msg.angle_increment, len(ranges))
        min_distance = danger_zone_min_range(ranges, target_ray, msg.angle_increment)
        ttc = time_to_collision(min_distance, self.last_vx)

        drive_msg = AckermannDriveStamped()
        drive_msg.drive.steering_angle = self.last_drive_msg.drive.steering_angle
        drive_msg.drive.speed = max(0.0, self.last_drive_msg.drive.speed)

        side_bias = wall_steer_bias(
            ranges, self.side_margin, self.wall_steer_gain, self.max_wall_steer_bias)
        drive_msg.drive.steering_angle = np.clip(
            drive_msg.drive.steering_angle + side_bias, -STEER_LIMIT_RAD, STEER_LIMIT_RAD)

        if min_distance < self.distance_threshold or ttc < self.ttc_fb:
            drive_msg.drive.speed = 0.0
            drive_msg.drive.steering_angle = self.last_angle

            # timestamp the latch so timer_callback can hold off before recovering
            if not self.kys:
                self.kys_latched_at_ns = self.get_clock().now().nanoseconds
            self.kys = True
            kys_msg = Bool()
            kys_msg.data = True
            self.kys_pub.publish(kys_msg)

            self.drive_pub.publish(drive_msg)
            self.log_state("FB", min_distance, ttc)

        elif ttc < self.ttc_pb2:
            drive_msg.drive.speed = min(drive_msg.drive.speed, PB2_SPEED_MPS)

            self.log_state("PARTIAL BRAKE 2", min_distance, ttc)

        elif ttc < self.ttc_pb1:
            drive_msg.drive.speed = min(drive_msg.drive.speed, PB1_SPEED_MPS)

            self.log_state("PARTIAL BRAKE 1", min_distance, ttc)

        else:
            self.log_state("NONE", min_distance, ttc)

        if self.winding_down:
            drive_msg.drive.speed = 0.0

        drive_msg.drive.speed = max(0.0, drive_msg.drive.speed)
        self.drive_pub.publish(drive_msg)

    def timer_callback(self) -> None:
        """release the latch once the holdoff has passed and the forward sector is clear."""
        if not self.kys or self.ranges is None:
            return

        # holding the latch gives the car time to stop, and the sim time to reposition it
        if self.kys_latched_at_ns is None:
            self.kys_latched_at_ns = self.get_clock().now().nanoseconds
        elapsed_sec = (self.get_clock().now().nanoseconds - self.kys_latched_at_ns) / 1e9
        if elapsed_sec < self.recovery_holdoff_sec:
            return

        min_forward = forward_min_range(self.ranges)
        if min_forward <= self.recovery_clearance:
            return

        self.kys = False
        self.kys_latched_at_ns = None
        kys_msg = Bool()
        kys_msg.data = False
        self.kys_pub.publish(kys_msg)
        self.get_logger().info(
            f"recovered, forward clearance {min_forward:.2f}m after {elapsed_sec:.1f}s"
        )

    def _sigint_handler(self, sig, frame) -> None:
        """wind down on the first ctrl+c, quit immediately on the second."""
        if self.winding_down:
            rclpy.shutdown()
            return
        self.get_logger().info("safety node winding down")
        self.winding_down = True
        # outlasts the controller's wind-down so steering keeps updating during the slowdown
        threading.Timer(SHUTDOWN_DELAY_SEC, lambda: rclpy.shutdown()).start()

    def drive_callback(self, msg: AckermannDriveStamped) -> None:
        # speed for ttc comes from odometry, not this message, so a commanded speed
        # the car never reached cannot mask a standstill
        self.last_drive_msg = msg
        self.last_angle = msg.drive.steering_angle

    def velocity_callback(self, msg: Odometry) -> None:
        self.last_vx = abs(msg.twist.twist.linear.x)


def main(args=None) -> None:
    rclpy.init(args=args)
    safety_node = SafetyNode()
    try:
        rclpy.spin(safety_node)
    except KeyboardInterrupt:
        pass
    finally:
        safety_node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
