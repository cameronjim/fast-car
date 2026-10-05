"""ros node steering from a forward rgb camera by tracking the track contour."""

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

MIN_CONTOUR_TOP_ROW_PX = 200
MIN_CONTOUR_AREA_PX = 10000
TARGET_ROW_PX = 400
RIGHT_BIAS_RAD = 0.2


class CvNode(Node):
    """vision path-following controller; speed comes from the safety node."""

    def __init__(self) -> None:
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
        if self.kys_latched:
            return

        img = self.bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')

        path_img, success = cam_filter_path(img)

        if not success:
            self.get_logger().info("no path detected")
            return

        target_x, target_row, straight = get_target(path_img)

        img_width = img.shape[1]
        offset_px = target_x - (img_width / 2)

        # biased right so the camera keeps more of the track in frame
        angle = - np.arctan2(offset_px, target_row) - RIGHT_BIAS_RAD

        pid_angle = self.pid.pid_err(angle, self.get_clock().now().nanoseconds * 1e-9)

        if straight:
            pid_angle = 0.0

        drive_msg = AckermannDriveStamped()
        drive_msg.drive.steering_angle = pid_angle
        drive_msg.drive.speed = self.speed

        self.drive_pub.publish(drive_msg)

    def kys_callback(self, msg: Bool) -> None:
        self.kys_latched = msg.data

    def speed_callback(self, msg: AckermannDriveStamped) -> None:
        self.speed = msg.drive.speed

    def on_param_change(self, params: List[Parameter]) -> SetParametersResult:
        """apply pid gain changes made through the ros parameter service at runtime."""
        for param in params:
            if param.name == 'K_p':
                self.K_p = float(param.value)
                self.pid.K_p = self.K_p
            elif param.name == 'K_i':
                self.K_i = float(param.value)
                self.pid.K_i = self.K_i
            elif param.name == 'K_d':
                self.K_d = float(param.value)
                self.pid.K_d = self.K_d

        return SetParametersResult(successful=True)


def cam_filter_path(img) -> tuple[np.ndarray, bool]:
    """binary mask of the track, from an open-close pass then the first big low contour."""
    img = cv2.cvtColor(img, cv2.COLOR_BGR2GRAY)
    kernel = np.ones((9, 9), np.uint8)
    img = cv2.erode(img, kernel, iterations=2)
    img = cv2.dilate(img, kernel, iterations=2)
    _, img = cv2.threshold(img, 127, 255, cv2.THRESH_BINARY)
    contours, hierarchy = cv2.findContours(img, cv2.RETR_TREE, cv2.CHAIN_APPROX_SIMPLE)

    mask = np.zeros(img.shape[:2], dtype="uint8")

    success = False

    for contour in contours:
        x, y, w, h = cv2.boundingRect(contour)
        area = cv2.contourArea(contour)
        if y > MIN_CONTOUR_TOP_ROW_PX and area > MIN_CONTOUR_AREA_PX:
            cv2.drawContours(mask, [contour], -1, 255, -1)
            success = True
            break

    return (mask, success)


def get_target(img, target_row=TARGET_ROW_PX) -> tuple[float, int, bool]:
    """mean x of the path pixels on one image row; straight is true when the row is empty."""
    row = img[target_row, :]
    path_px = np.argwhere(row > 0)
    if len(path_px) == 0:
        return (0, target_row, True)
    return (np.mean(path_px), target_row, False)


def main(args=None) -> None:
    rclpy.init(args=args)
    cv_node = CvNode()
    rclpy.spin(cv_node)
    cv_node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
