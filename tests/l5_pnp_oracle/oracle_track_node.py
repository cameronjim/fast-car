#!/usr/bin/env python3
"""oracle_track_node: TEST ONLY, never installed on the car (measurement M-A2, brief 40).

Publishes `racer_pnp_msgs/LocalTrack` on /pnp/local_track from the simulator's ground truth, in
place of track_perception_node, on exactly the same terms: one message per /scan, stamped with
that scan, frame base_link, 16 stations 0.20 m apart. The pose is the ground truth pose
(/sim/ground_truth_odom, the CG) at the scan's stamp, linearly interpolated, moved to the rear
axle (base_link) with vehicle_params' cg_to_rear_axle_m from the generated bindings. The message
goes out in the scan callback like the perception node's, or, when the odometry of that step has
not arrived yet, in the callback of the odometry that completes it.

The track file (--ros-args -p ground_truth_track:=...) is written by track_family_runner.py
(`--backend oracle`). Run by the runner by path, not as an installed executable:

    python3 tests/l5_pnp_oracle/oracle_track_node.py --ros-args -p ground_truth_track:=FILE
"""

from __future__ import annotations

import math
import sys
from collections import deque
from pathlib import Path

HERE = Path(__file__).resolve().parent
sys.path.insert(0, str(HERE))

import oracle_core as oc
import rclpy
from nav_msgs.msg import Odometry
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node
from rclpy.qos import HistoryPolicy, QoSProfile, ReliabilityPolicy
from sensor_msgs.msg import LaserScan

FRAME_ID = "base_link"
LOCAL_TRACK_TOPIC = "/pnp/local_track"
SCAN_TOPIC = "/scan"
GROUND_TRUTH_ODOM_TOPIC = "/sim/ground_truth_odom"
ODOM_BUFFER = 400  # 4 s of 100 Hz ground truth


def stamp_s(stamp) -> float:
    return stamp.sec + stamp.nanosec * 1e-9


def yaw_of(q) -> float:
    return math.atan2(2.0 * (q.w * q.z + q.x * q.y), 1.0 - 2.0 * (q.y * q.y + q.z * q.z))


class OracleTrackNode(Node):
    def __init__(self) -> None:
        super().__init__("oracle_track_node")
        path = self.declare_parameter("ground_truth_track", "").value
        if not path:
            raise SystemExit("oracle_track_node: ground_truth_track is required")
        self._truth = oc.GroundTruthTrack.load(path)
        from racer_tools.vehicle_params_loader import load_vehicle_params

        self._cg_to_rear = float(load_vehicle_params().chassis.cg_to_rear_axle_m)
        from racer_pnp_msgs.msg import LocalTrack

        self._msg_type = LocalTrack
        reliable = lambda depth: QoSProfile(
            reliability=ReliabilityPolicy.RELIABLE, history=HistoryPolicy.KEEP_LAST, depth=depth
        )
        self._odom: deque = deque(maxlen=ODOM_BUFFER)
        self._pending: deque = deque()
        self._pub = self.create_publisher(LocalTrack, LOCAL_TRACK_TOPIC, reliable(5))
        self.create_subscription(Odometry, GROUND_TRUTH_ODOM_TOPIC, self._on_odom, reliable(200))
        self.create_subscription(
            LaserScan,
            SCAN_TOPIC,
            self._on_scan,
            QoSProfile(
                reliability=ReliabilityPolicy.BEST_EFFORT, history=HistoryPolicy.KEEP_LAST, depth=10
            ),
        )
        self.get_logger().info(
            f"oracle_track_node: ground truth LocalTrack from {path} (TEST ONLY), "
            f"cg_to_rear_axle_m {self._cg_to_rear}"
        )

    def _on_odom(self, msg: Odometry) -> None:
        p = msg.pose.pose
        self._odom.append(
            (stamp_s(msg.header.stamp), p.position.x, p.position.y, yaw_of(p.orientation))
        )
        while self._pending:
            if not self._publish(self._pending[0]):
                break
            self._pending.popleft()

    def _on_scan(self, scan: LaserScan) -> None:
        if self._pending or not self._publish(scan.header.stamp):
            self._pending.append(scan.header.stamp)

    def _publish(self, stamp) -> bool:
        pose = oc.interpolate_pose(list(self._odom), stamp_s(stamp))
        if pose is None:
            return False
        cg_x, cg_y, yaw = pose
        bx, by = oc.base_pose_from_cg(cg_x, cg_y, yaw, self._cg_to_rear)
        track = self._truth.local_track(bx, by, yaw)
        msg = self._msg_type()
        msg.header.stamp = stamp
        msg.header.frame_id = FRAME_ID
        msg.model_id = oc.MODEL_ID
        msg.backend = 1  # BACKEND_GEOMETRIC: not a net
        msg.station_spacing_m = oc.SPACING_M
        msg.x_m = track.x_m.tolist()
        msg.y_m = track.y_m.tolist()
        msg.half_width_left_m = track.half_width_left_m.tolist()
        msg.half_width_right_m = track.half_width_right_m.tolist()
        msg.sigma_m = track.sigma_m.tolist()
        msg.valid_count = int(track.valid_count)
        msg.boundary_residual_m = float(track.boundary_residual_m)
        msg.intrusion_fraction = float(track.intrusion_fraction)
        self._pub.publish(msg)
        return True


def main() -> None:
    rclpy.init()
    node = OracleTrackNode()
    try:
        rclpy.spin(node)
    except (KeyboardInterrupt, ExternalShutdownException):
        pass
    finally:
        node.destroy_node()
        rclpy.try_shutdown()


if __name__ == "__main__":
    main()
