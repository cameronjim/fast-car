"""ros safety node gating the reactive controllers: staged braking, latched stop, recovery."""

import rclpy
import signal
import threading
import numpy as np
from rclpy.node import Node
from nav_msgs.msg import Odometry
from ackermann_msgs.msg import AckermannDriveStamped
from sensor_msgs.msg import LaserScan
from std_msgs.msg import Bool
from reactive_control.safety_logic import (
    danger_zone_min_range,
    forward_min_range,
    forward_ray,
    time_to_collision,
)

RECOVERY_CLEARANCE_M = 0.5
SHUTDOWN_DELAY_SEC = 4.5


class SafetyNode(Node):
    """staged braking from lidar ttc: pb1, pb2, then a latched full brake with auto recovery."""

    def __init__(self) -> None:
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
        ranges = np.array(msg.ranges)
        ranges = np.clip(ranges, msg.range_min, msg.range_max)
        # kept for timer_callback's recovery check
        self.ranges = ranges

        target_ray = forward_ray(self.last_angle, msg.angle_increment, len(ranges))
        min_distance = danger_zone_min_range(ranges, target_ray, msg.angle_increment)
        ttc = time_to_collision(min_distance, self.last_vx)

        drive_msg = AckermannDriveStamped()

        if min_distance < self.distance_threshold or ttc < self.ttc_fb:
            drive_msg.drive.speed = 0.0
            drive_msg.drive.steering_angle = self.last_angle

            self.kys = True
            kys_msg = Bool()
            kys_msg.data = True
            self.kys_pub.publish(kys_msg)

            self.drive_pub.publish(drive_msg)
            self.get_logger().info(f"FB - Distance: {min_distance:.2f}m, TTC: {ttc:.2f}s")

        elif ttc < self.ttc_pb2:
            drive_msg.drive.speed = 0.75

            self.get_logger().info(f"PARTIAL BRAKE 2 - Distance: {min_distance:.2f}m, TTC: {ttc:.2f}s")

        elif ttc < self.ttc_pb1:
            drive_msg.drive.speed = 1.4

            self.get_logger().info(f"PARTIAL BRAKE 1 - Distance: {min_distance:.2f}m, TTC: {ttc:.2f}s")

        else:
            drive_msg.drive.speed = 2.0

            self.get_logger().info(f"NONE - Distance: {min_distance:.2f}m, TTC: {ttc:.2f}s")

        if self.winding_down:
            drive_msg.drive.speed = min(drive_msg.drive.speed, 0.0)

        self.speed_pub.publish(drive_msg)

    def timer_callback(self) -> None:
        """release the emergency stop once the forward sector is clear again."""
        if self.kys and self.ranges is not None:
            if forward_min_range(self.ranges) > RECOVERY_CLEARANCE_M:
                self.kys = False
                kys_msg = Bool()
                kys_msg.data = False
                self.kys_pub.publish(kys_msg)

    def _sigint_handler(self, sig, frame) -> None:
        """wind down on the first ctrl+c, quit immediately on the second."""
        if self.winding_down:
            rclpy.shutdown()
            return
        self.get_logger().info("safety node winding down")
        self.winding_down = True
        # outlasts gap_follow_node's wind-down so steering keeps updating during the slowdown
        threading.Timer(SHUTDOWN_DELAY_SEC, lambda: rclpy.shutdown()).start()

    def drive_callback(self, msg: AckermannDriveStamped) -> None:
        # only the angle is taken; ttc uses odometry speed, since the commanded speed
        # masks a standstill and brakes against a speed the car never reached
        self.last_angle = msg.drive.steering_angle

    def velocity_callback(self, msg: Odometry) -> None:
        self.last_vx = abs(msg.twist.twist.linear.x)


def main(args=None) -> None:
    rclpy.init(args=args)
    safety_node = SafetyNode()
    rclpy.spin(safety_node)
    safety_node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
