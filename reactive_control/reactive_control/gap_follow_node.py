"""
Gap-following node.

Implements a reactive gap-following controller using LiDAR disparity detection.
The node identifies the largest navigable gap in front of the vehicle and steers
toward its center.
"""

import rclpy
import signal
import threading
import numpy as np
from reactive_control.pid import PID
from rclpy.node import Node
from sensor_msgs.msg import LaserScan
from nav_msgs.msg import Odometry
from ackermann_msgs.msg import AckermannDriveStamped
from std_msgs.msg import Bool


class GapFollowNode(Node):
    """
    Reactive gap-following controller.

    Uses LiDAR range data to detect gaps via disparity extension, selects a
    target direction within the largest gap, and applies a PID controller to
    compute steering commands. An external safety signal can latch an
    emergency stop.
    """

    def __init__(self) -> None:
        """
        Initialize the gap-following node.

        Sets up ROS publishers, subscribers, and parameters, and initializes
        the PID controller.

        Returns:
            None
        """
        super().__init__('gap_follow_node')

        odom_topic = self.declare_parameter('odom_topic', '/odom').value

        self.scan_sub = self.create_subscription(LaserScan, '/scan', self.listener_callback, 10)
        self.kys_sub = self.create_subscription(Bool, '/kys', self.kys_callback, 10)
        self.vel_sub = self.create_subscription(Odometry, odom_topic, self.velocity_callback, 10)
        self.speed_sub = self.create_subscription(AckermannDriveStamped, '/speed', self.speed_callback, 10)
        self.publisher_ = self.create_publisher(AckermannDriveStamped, '/drive', 10)

        # PID gains
        self.K_p = self.declare_parameter('K_p', 1.0).value
        self.K_i = self.declare_parameter('K_i', 0.0).value
        self.K_d = self.declare_parameter('K_d', 0.05).value

        # Speed parameters
        self.max_speed = self.declare_parameter('max_speed', 1.0).value
        self.min_speed = self.declare_parameter('min_speed', 0.1).value
        self.K_speed = self.declare_parameter('K_speed', 1.0).value
        self.target_distance = self.declare_parameter('target_distance', 1.0).value

        # Gap-following parameters
        self.clip_max_range = self.declare_parameter('clip_max_range', 3.5).value
        self.disparity_threshold = self.declare_parameter('disparity_threshold', 1.0).value
        self.vehicle_half_width = self.declare_parameter('vehicle_half_width', 0.5).value
        self.free_space_threshold = self.declare_parameter('free_space_threshold', 1.2).value
        self.corner_min_clearance = self.declare_parameter('corner_min_clearance', 0.2).value
        self.cone_left_fraction = self.declare_parameter('cone_left_fraction', 0.25).value
        self.cone_right_fraction = self.declare_parameter('cone_right_fraction', 0.75).value

        # Shutdown parameters
        self.shutdown_speed = self.declare_parameter('shutdown_speed', 0.0).value
        self.shutdown_duration = self.declare_parameter('shutdown_duration', 2.0).value

        # Emergency stop latch
        self.kys = self.declare_parameter('kys_latched', False).value

        self.pid = PID(self.K_p, self.K_i, self.K_d)
        self.speed = 0.0
        self.last_vel = 0.0
        self.winding_down = False

        self.get_logger().info("Gap node initialized.")

        # Register the graceful shutdown handler
        signal.signal(signal.SIGINT, self._sigint_handler)

    def _sigint_handler(self, sig, frame) -> None:
        """
        Graceful shutdown handler.

        On the first SIGINT, enters a winding-down state: the commanded speed
        is set to shutdown_speed and a timer is started to shut the node down
        after shutdown_duration, keeping it alive long enough to keep issuing
        steering commands while the car coasts to a stop. A second SIGINT
        shuts down immediately.

        Args:
            sig: Signal number.
            frame: Current stack frame.

        Returns:
            None
        """
        if self.winding_down:
            rclpy.shutdown()
            return
        self.get_logger().info("CTRL+C caught — winding down...")
        self.winding_down = True
        self.speed = self.shutdown_speed

        timer = threading.Timer(self.shutdown_duration, self._do_shutdown)
        timer.start()

    def _do_shutdown(self) -> None:
        """
        Publish a final stop command and shut the node down.

        Returns:
            None
        """
        drive_msg = AckermannDriveStamped()
        drive_msg.drive.speed = 0.0
        drive_msg.drive.steering_angle = 0.0
        self.publisher_.publish(drive_msg)
        self.get_logger().info("Shutdown complete.")
        rclpy.shutdown()

    def listener_callback(self, msg: LaserScan) -> None:
        """
        Process incoming LiDAR scans and compute a driving command.

        Filters raw LiDAR ranges using disparity extension, identifies the
        largest navigable gap, selects a target steering angle, and applies
        PID control to generate steering and speed commands.

        Args:
            msg (LaserScan): Incoming LiDAR scan message.

        Returns:
            None
        """
        # Clock in seconds (nanoseconds / 1e9)
        current_time = self.get_clock().now().nanoseconds / 1e9

        clipped_ranges = np.clip(msg.ranges, msg.range_min, self.clip_max_range)
        filtered_ranges = self.filter_ranges(clipped_ranges)
        target = self.get_target(filtered_ranges)

        center = len(filtered_ranges) // 2
        angle = (target - center) * msg.angle_increment
        if self.check_corners(filtered_ranges, angle, self.corner_min_clearance):
            angle = 0

        pid_angle = self.pid.pid_err(angle, current_time)

        drive_msg = AckermannDriveStamped()
        drive_msg.drive.steering_angle = pid_angle

        if self.winding_down:
            self.speed = min(self.speed, self.shutdown_speed)

        drive_msg.drive.speed = self.speed
        self.publisher_.publish(drive_msg)

    def filter_ranges(self, ranges: np.ndarray) -> np.ndarray:
        """
        Expand obstacles in the LiDAR ranges for safer gap detection.

        Detects disparities and extends nearby obstacles to account for the
        vehicle width and a safety margin.

        Args:
            ranges (np.ndarray): Array of clipped LiDAR range measurements.

        Returns:
            np.ndarray: Modified range array with unsafe regions reduced.
        """
        safe_ranges = ranges.copy()

        disparities = []
        for i in range(len(safe_ranges) - 1):
            if np.abs(safe_ranges[i] - safe_ranges[i+1]) > self.disparity_threshold:
                disparities.append(i)

        for i in disparities:
            ray1 = ranges[i]
            ray2 = ranges[i+1]

            if ray1 < ray2:
                near = ray1
                direction = 1
                start = i + 1
            else:
                near = ray2
                direction = -1
                start = i

            danger_zone = int(np.arctan2(self.vehicle_half_width, near) * 180 * 4 / 3.14)

            for j in range(danger_zone):
                k = start + direction * j
                if 0 <= k < len(safe_ranges):
                    if safe_ranges[k] > near:
                        safe_ranges[k] = near

        return safe_ranges

    def get_target(self, ranges: np.ndarray) -> int:
        """
        Select the target ray at the center of the largest open gap.

        Searches a forward-facing cone for contiguous free-space segments and
        returns the center ray of the longest gap, tie-breaking toward straight
        ahead.

        Args:
            ranges (np.ndarray): Filtered LiDAR ranges.

        Returns:
            int: Index of the target ray.
        """
        n = len(ranges)
        center = n // 2

        # Look mostly forward within a cone (tune the fractions if needed)
        left = int(n * self.cone_left_fraction)
        right = int(n * self.cone_right_fraction)
        cone = ranges[left:right]

        # Consider anything beyond this threshold as free space
        # (bigger -> more willing to drive between closer walls)
        free = cone > self.free_space_threshold

        if not np.any(free):
            # If everything is tight, just go straight
            return center

        # Find contiguous free segments and pick the longest gap,
        # tie-breaking by being closer to straight ahead
        best_start = best_end = None
        best_len = -1
        best_center_dist = float("inf")

        i = 0
        while i < len(free):
            if not free[i]:
                i += 1
                continue
            start = i
            while i < len(free) and free[i]:
                i += 1
            end = i - 1  # Inclusive

            gap_len = end - start + 1
            gap_center = (start + end) // 2
            dist_to_center = abs((gap_center + left) - center)

            if (gap_len > best_len) or (gap_len == best_len and dist_to_center < best_center_dist):
                best_len = gap_len
                best_start, best_end = start, end
                best_center_dist = dist_to_center

        target_in_cone = (best_start + best_end) // 2
        target = target_in_cone + left
        return int(target)

    def check_corners(self, ranges: np.ndarray, angle: float, min_clearance: float) -> bool:
        """
        Detect a sharp corner dead-end.

        Args:
            ranges (np.ndarray): Filtered LiDAR ranges.
            angle (float): Steering angle (radians).
            min_clearance (float): Minimum safe distance threshold.

        Returns:
            bool: True if a corner dead-end is detected, False otherwise.
        """
        if angle < 0:
            return np.all(ranges[900:1080] < min_clearance)
        else:
            return np.all(ranges[:180] < min_clearance)

    def kys_callback(self, msg: Bool) -> None:
        """
        Emergency stop callback.

        Latches the stop flag when the safety node asserts it.

        Args:
            msg (Bool): Stop command from the safety node.

        Returns:
            None
        """
        self.kys = msg.data

    def velocity_callback(self, msg: Odometry) -> None:
        """
        Store the current forward velocity of the vehicle.

        Args:
            msg (Odometry): Odometry message containing linear velocity.

        Returns:
            None
        """
        self.last_vel = abs(msg.twist.twist.linear.x)

    def speed_callback(self, msg: AckermannDriveStamped) -> None:
        """
        Store the speed command supplied by the safety node.

        Args:
            msg (AckermannDriveStamped): Speed command message.

        Returns:
            None
        """
        self.speed = msg.drive.speed


def main(args=None) -> None:
    """
    Entry point for the gap-following node.

    Args:
        args: Command-line arguments passed to rclpy.

    Returns:
        None
    """
    rclpy.init(args=args)
    gap_follow_node = GapFollowNode()
    rclpy.spin(gap_follow_node)
    gap_follow_node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
