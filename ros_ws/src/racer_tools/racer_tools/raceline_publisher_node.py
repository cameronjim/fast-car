"""raceline_publisher_node: publishes the committed raceline as a latched nav_msgs/Path
(roadmap milestone 3, claude-docs/04-architecture.md) so Foxglove can show the line the
classical tracker is following, alongside everything milestone 2 already shows (/sim/map,
/tf, /scan) -- see config/foxglove_sim_autopilot.layout.json and
docs/notes/milestone-3-sim-autopilot.md.

A tiny, standalone publisher rather than adding this to tracker_node
(ros_ws/src/racer_control, the control-critical 50 Hz path): the raceline is fixed for the
whole run and needs publishing exactly once, so it does not belong on tracker_node's hot
loop, and keeping it out of racer_control avoids adding a non-control-path publisher to
that safety-adjacent node (claude-docs/10-conventions.md: "no heap allocation in the 50 Hz
control path after init" -- building a Path message the size of the whole raceline is
exactly the kind of one-shot work that should stay off that loop).

Publishes /sim/raceline (nav_msgs/Path, transient_local depth 1 -- same "latched" pattern
as bridge_node's /sim/map) once at startup, in the `map` frame (REP-105,
claude-docs/04-architecture.md). Loads the SAME CSV format racer_control (C++) and
racer_gym_bridge (Python/numpy) each parse independently, via this package's own
raceline_loader.py + raceline_path_builder.py (pure, L1-tested; see those modules).

Required `raceline_path` parameter, fails closed exactly like tracker_node's own
`raceline_path` (claude-docs/05-safety.md: "fail closed") -- refuses to start rather than
silently publishing nothing.
"""

from __future__ import annotations

import rclpy
from geometry_msgs.msg import PoseStamped
from nav_msgs.msg import Path
from rcl_interfaces.msg import ParameterDescriptor
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, HistoryPolicy, QoSProfile, ReliabilityPolicy

from racer_tools.raceline_loader import RacelineLoadError, load_raceline_points
from racer_tools.raceline_path_builder import raceline_path_fields

_MAP_FRAME_ID = "map"


def build_raceline_path_msg(points, frame_id: str = _MAP_FRAME_ID) -> Path:
    """Thin ROS assembly over racer_tools.raceline_path_builder's pure fields -- not L1
    tested itself (it only exists to hold geometry_msgs/nav_msgs field assignments), the
    L1 coverage lives on raceline_path_fields; this function's correctness (frame ids,
    latching) is verified by this package's L3 launch test instead."""
    msg = Path()
    msg.header.frame_id = frame_id
    for field in raceline_path_fields(points):
        pose = PoseStamped()
        pose.header.frame_id = frame_id
        pose.pose.position.x = field.x_m
        pose.pose.position.y = field.y_m
        pose.pose.orientation.z = field.orientation_z
        pose.pose.orientation.w = field.orientation_w
        msg.poses.append(pose)
    return msg


class RacelinePublisherNode(Node):
    def __init__(self) -> None:
        super().__init__("raceline_publisher_node")

        descriptor = ParameterDescriptor(
            description=(
                "Path to a tools/raceline-generated CSV (claude-docs/02-repo-layout.md: "
                "config/tracks/<venue>_<layout>/raceline.csv). Required; this node refuses "
                "to start without a loadable raceline."
            ),
        )
        raceline_path = str(self.declare_parameter("raceline_path", "", descriptor).value)
        if not raceline_path:
            self.get_logger().fatal(
                "raceline_publisher_node: 'raceline_path' parameter is required and was not "
                "set. Refusing to start with no raceline (claude-docs/05-safety.md: fail "
                "closed)."
            )
            raise RuntimeError(
                "raceline_publisher_node: missing required 'raceline_path' parameter"
            )

        try:
            points = load_raceline_points(raceline_path)
        except RacelineLoadError as e:
            self.get_logger().fatal(
                f"raceline_publisher_node: failed to load raceline from '{raceline_path}': {e}"
            )
            raise

        # transient_local + explicit depth (claude-docs/10-conventions.md: "QoS: ... explicit
        # depth -- never default"), same latching pattern as bridge_node's /sim/map: the
        # raceline never changes for the life of a run, so a late-joining subscriber (e.g.
        # Foxglove connecting after this node has already published) must still receive it.
        path_qos = QoSProfile(
            reliability=ReliabilityPolicy.RELIABLE,
            durability=DurabilityPolicy.TRANSIENT_LOCAL,
            history=HistoryPolicy.KEEP_LAST,
            depth=1,
        )
        self._path_pub = self.create_publisher(Path, "/sim/raceline", path_qos)

        msg = build_raceline_path_msg(points)
        now = self.get_clock().now().to_msg()
        msg.header.stamp = now
        for pose in msg.poses:
            pose.header.stamp = now
        self._path_pub.publish(msg)

        self.get_logger().info(
            f"raceline_publisher_node up: published {len(points)} raceline point(s) "
            f"(+closing segment) on /sim/raceline from '{raceline_path}'."
        )


def main(args: list[str] | None = None) -> None:
    rclpy.init(args=args)
    node = RacelinePublisherNode()
    try:
        rclpy.spin(node)
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()
