"""
Camera path-following node.

Processes RGB camera images to detect the track path using image thresholding
and contour filtering. Computes a steering angle using a PID controller and
publishes Ackermann drive commands.
"""

import rclpy
import numpy as np
import cv2
import cv_bridge
from rclpy.node import Node
from sensor_msgs.msg import Image
from ackermann_msgs.msg import AckermannDriveStamped
from std_msgs.msg import Bool
from reactive_control.pid import PID
from rcl_interfaces.msg import SetParametersResult
from rclpy.parameter import Parameter
from typing import List


class CvNode(Node):
    """
    Vision-based path-following controller.

    Detects the track from a forward RGB camera feed using morphological
    filtering and contour extraction. Computes a steering error relative to the
    image center and applies PID control to generate a steering command.

    Speed is received from an external safety node and passed through to the
    drive command.
    """

    def __init__(self) -> None:
        """
        Initialize the camera node.

        Sets up ROS publishers/subscribers, declares PID parameters, registers
        the dynamic parameter callback, and initializes internal control state.

        Returns:
            None
        """
        super().__init__('cv_node')
        self.cam_sub = self.create_subscription(Image, '/camera/color/image_raw', self.cam_callback, 10)
        self.kys_sub = self.create_subscription(Bool, '/kys', self.kys_callback, 10)
        self.drive_pub = self.create_publisher(AckermannDriveStamped, '/drive', 10)
        self.speed_sub = self.create_subscription(AckermannDriveStamped, '/speed', self.speed_callback, 10)
        self.run = 0
        self.depth_img = None

        self.K_p = self.declare_parameter('K_p', 1.5).value
        self.K_i = self.declare_parameter('K_i', 0.0).value
        self.K_d = self.declare_parameter('K_d', 0.05).value

        self.add_on_set_parameters_callback(self.on_param_change)

        self.bridge = cv_bridge.CvBridge()

        self.pid = PID(K_p=self.K_p, K_i=self.K_i, K_d=self.K_d)

        self.kys_latched = False
        self.speed = 0.0

    def cam_callback(self, msg: Image) -> None:
        """
        Camera image callback.

        Converts the ROS image to OpenCV format, extracts a binary mask
        representing the track, computes a steering target, and publishes a
        PID-controlled steering command. If no valid path is detected, no
        command is published.

        Args:
            msg (Image): Incoming RGB image message.

        Returns:
            None
        """
        if self.kys_latched:
            return

        img = self.bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')

        path_img, success = cam_filter_path(img)

        if not success:
            self.get_logger().info("No path detected.")
            return

        x, y, straight = get_target(path_img)

        img_w = img.shape[1]

        x_target = x - (img_w / 2)

        # Bias to the right to get a better view of the track
        angle = - np.arctan2(x_target, y) - 0.2

        pid_angle = self.pid.pid_err(angle, self.get_clock().now().nanoseconds * 1e-9)

        if straight:
            pid_angle = 0.0

        drive_msg = AckermannDriveStamped()
        drive_msg.drive.steering_angle = pid_angle
        drive_msg.drive.speed = self.speed

        self.drive_pub.publish(drive_msg)

    def kys_callback(self, msg: Bool) -> None:
        """
        Emergency stop callback.

        Latches the stop condition when triggered by the safety node.

        Args:
            msg (Bool): Stop flag from the safety node.

        Returns:
            None
        """
        if msg.data:
            self.kys_latched = True

    def speed_callback(self, msg: AckermannDriveStamped) -> None:
        """
        Speed update callback.

        Receives externally computed speed commands (e.g., from the safety
        node) and stores them for use in drive commands.

        Args:
            msg (AckermannDriveStamped): Speed command message.

        Returns:
            None
        """
        self.speed = msg.drive.speed

    def on_param_change(self, params: List[Parameter]) -> SetParametersResult:
        """
        Handle dynamic parameter updates.

        Updates the PID gains when parameters are changed at runtime via ROS2
        parameter services.

        Args:
            params (List[Parameter]): List of updated parameters.

        Returns:
            SetParametersResult: Result indicating whether the update succeeded.
        """
        for p in params:
            if p.name == 'K_p':
                self.K_p = float(p.value)
                self.pid.K_p = self.K_p
            elif p.name == 'K_i':
                self.K_i = float(p.value)
                self.pid.K_i = self.K_i
            elif p.name == 'K_d':
                self.K_d = float(p.value)
                self.pid.K_d = self.K_d

        return SetParametersResult(successful=True)


def cam_filter_path(img) -> tuple[np.ndarray, bool]:
    """
    Extract a binary mask representing the track path.

    Applies grayscale conversion, morphological filtering, thresholding, and
    contour selection based on area and vertical position.

    Args:
        img (np.ndarray): Input BGR image.

    Returns:
        tuple:
            mask (np.ndarray): Binary image of the detected path.
            success (bool): True if a valid path was found.
    """
    img = cv2.cvtColor(img, cv2.COLOR_BGR2GRAY)
    kernel = np.ones((9, 9), np.uint8)
    img = cv2.erode(img, kernel, iterations=2)
    img = cv2.dilate(img, kernel, iterations=2)
    ret, img = cv2.threshold(img, 127, 255, cv2.THRESH_BINARY)
    contours, hierarchy = cv2.findContours(img, cv2.RETR_TREE, cv2.CHAIN_APPROX_SIMPLE)

    mask = np.zeros(img.shape[:2], dtype="uint8")

    success = False

    for contour in contours:
        x, y, w, h = cv2.boundingRect(contour)
        area = cv2.contourArea(contour)
        if y > 200 and area > 10000:
            cv2.drawContours(mask, [contour], -1, 255, -1)
            success = True
            break

    return (mask, success)


def get_target(img, target_row=400) -> tuple[float, int, bool]:
    """
    Determine the steering target from a binary path image.

    Samples a horizontal row of the mask and computes the mean x-position of
    the detected path pixels.

    Args:
        img (np.ndarray): Binary path mask.
        target_row (int): Image row used for target extraction.

    Returns:
        tuple:
            x (float): Mean x-coordinate of detected path pixels.
            y (int): Row used for detection.
            straight (bool): True if no path was detected.
    """
    row = img[target_row, :]
    indices = np.argwhere(row > 0)
    if len(indices) == 0:
        return (0, target_row, True)
    return (np.mean(indices), target_row, False)


def main(args=None) -> None:
    """
    Entry point for the camera path-following node.

    Args:
        args: Command-line arguments passed to rclpy.

    Returns:
        None
    """
    rclpy.init(args=args)
    cv_node = CvNode()
    rclpy.spin(cv_node)
    cv_node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
