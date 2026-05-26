"""
Wall-following node.

Follows the right-hand wall using two LiDAR rays, one pointing hard right (-90
degrees) and one pointing forward-right (-20 degrees), to estimate both the
distance to the wall and the car's orientation relative to it. A PID controller
turns the distance error into a steering command. Speed is governed by the
safety node.
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


class WallFollowNode(Node):
    """
    Reactive wall-following controller.

    Uses two right-side LiDAR rays to estimate the wall angle and distance, then
    applies a PID controller to compute a steering command. Forward speed is
    taken from the safety node and reduced on sharp turns.
    """

    def __init__(self) -> None:
        """
        Initialize the wall-following node.

        Sets up ROS publishers, subscribers, and parameters, and initializes
        the PID controller.

        Returns:
            None
        """
        super().__init__('wall_follow_node')

        odom_topic = self.declare_parameter('odom_topic', '/odom').value

        self.scan_sub = self.create_subscription(LaserScan, '/scan', self.listener_callback, 10)
        self.kys_sub = self.create_subscription(Bool, '/kys', self.kys_callback, 10)
        self.vel_sub = self.create_subscription(Odometry, odom_topic, self.velocity_callback, 10)
        self.speed_sub = self.create_subscription(AckermannDriveStamped, '/speed', self.speed_callback, 10)
        self.publisher_ = self.create_publisher(AckermannDriveStamped, '/drive', 10)

        # PID gains
        self.K_p = self.declare_parameter('K_p', 1.5).value
        self.K_i = self.declare_parameter('K_i', 0.0).value
        self.K_d = self.declare_parameter('K_d', 0.02).value

        # Wall-following parameters
        self.target_distance = self.declare_parameter('target_distance', 1.0).value
        self.max_speed = self.declare_parameter('max_speed', 1.0).value
        self.min_speed = self.declare_parameter('min_speed', 0.1).value
        self.K_speed = self.declare_parameter('K_speed', 1.0).value

        # Shutdown parameters
        self.shutdown_speed = self.declare_parameter('shutdown_speed', 0.0).value
        self.shutdown_duration = self.declare_parameter('shutdown_duration', 2.0).value

        # Emergency stop latch
        self.kys = self.declare_parameter('kys_latched', False).value

        self.pid = PID(self.K_p, self.K_i, self.K_d)
        self.speed = 0.0
        self.last_vel = 0.0
        self.prev_time = None
        self.winding_down = False

        self.get_logger().info("Wall node initialized.")

        # Register the graceful shutdown handler
        signal.signal(signal.SIGINT, self._sigint_handler)

    def _sigint_handler(self, sig, frame) -> None:
        """
        Graceful shutdown handler.

        On the first SIGINT, enters a winding-down state: the commanded speed is
        clamped to shutdown_speed and a timer is started to shut the node down
        after shutdown_duration, keeping it alive long enough to keep steering
        while the car coasts to a stop. A second SIGINT shuts down immediately.

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

        Estimates the distance error relative to the desired wall distance,
        applies PID control to produce a steering angle, reduces speed on sharp
        turns, caps the result at the speed allowed by the safety node, and
        publishes the drive command.

        Args:
            msg (LaserScan): Incoming LiDAR scan message.

        Returns:
            None
        """
        # Clock in seconds (nanoseconds / 1e9)
        current_time = self.get_clock().now().nanoseconds / 1e9

        if self.prev_time is None:
            dt = 0.0
        else:
            dt = current_time - self.prev_time

        err = get_error(msg.ranges, msg.range_min, msg.range_max,
                        self.target_distance, self.last_vel, dt)
        pid_angle = self.pid.pid_err(err, current_time)

        drive_msg = AckermannDriveStamped()
        drive_msg.drive.steering_angle = pid_angle

        # Slow down on sharp turns, then cap at the safety-allowed speed
        desired = self.max_speed - np.abs(pid_angle) * self.K_speed
        desired = max(desired, self.min_speed)
        speed = min(desired, self.speed)

        if self.winding_down:
            speed = min(speed, self.shutdown_speed)

        drive_msg.drive.speed = float(speed)
        self.publisher_.publish(drive_msg)
        self.prev_time = current_time

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
        self.last_vel = -(msg.twist.twist.linear.x)

    def speed_callback(self, msg: AckermannDriveStamped) -> None:
        """
        Store the speed command supplied by the safety node.

        Args:
            msg (AckermannDriveStamped): Speed command message.

        Returns:
            None
        """
        self.speed = msg.drive.speed


def get_range(range_data, angle) -> float:
    """
    Return the range measurement at a given angle.

    Angles are in degrees, from -135 (hard right) through 0 (straight ahead) to
    +135 (hard left). The conversion adds 135 to offset the -135 degree start
    and multiplies by 4 to convert from 0.25 degree intervals to array indices.

    Args:
        range_data: Array of distances, one per 0.25 degree interval.
        angle: Angle in degrees whose distance is requested.

    Returns:
        float: Distance measurement at the given angle.
    """
    return range_data[(angle + 135) * 4]


def get_error(range_data, range_min, range_max, dist, vel, dt) -> float:
    """
    Estimate the distance error to the right-hand wall.

    Two LiDAR rays are used: ray a at -20 degrees and ray b at -90 degrees. The
    orientation alpha of the car relative to the wall is estimated from the two
    distances using:

        alpha = arctan((a * cos(theta) - b) / (a * sin(theta)))

    where theta is the angle between the two rays. The current perpendicular
    distance AB is then projected forward by the distance the car travels in dt
    to give a lookahead distance CD, and the error is dist - CD.

    Args:
        range_data: Array of LiDAR range measurements.
        range_min (float): Minimum valid LiDAR range.
        range_max (float): Maximum valid LiDAR range.
        dist (float): Desired distance from the wall.
        vel (float | None): Current forward velocity.
        dt (float): Time step since the last update.

    Returns:
        float: Distance error (positive means too far from the wall).
    """
    theta = np.radians(70)
    dist_b = get_range(range_data, -90)
    dist_a = get_range(range_data, -20)

    if dist_a < range_min or dist_a > range_max or dist_b < range_min or dist_b > range_max:
        return 0.0

    alpha = np.arctan((dist_a * np.cos(theta) - dist_b) / (dist_a * np.sin(theta)))

    AB = dist_b * np.cos(alpha)

    if vel is None:
        vel = 0

    CD = AB + vel * dt * np.sin(alpha)

    ret = dist - CD

    if np.abs(ret) < 0.02:
        return 0.0

    return ret


def main(args=None) -> None:
    """
    Entry point for the wall-following node.

    Args:
        args: Command-line arguments passed to rclpy.

    Returns:
        None
    """
    rclpy.init(args=args)
    wall_follow_node = WallFollowNode()
    rclpy.spin(wall_follow_node)
    wall_follow_node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
