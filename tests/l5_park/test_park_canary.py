"""L5 sim-in-loop canary for park_node's parallel park (roadmap 2.9).

SCENE. A counter-clockwise rounded rectangle (14 x 4.5 m, 1.2 m corners) with a 1.2 m walled
lane around it (racer_gym_bridge track_half_width_m 0.6). On the long bottom straight, 1.6 m
ahead of the start, a POCKET is cut into the RIGHT wall (bridge_node pocket_*): 1.8 m long and
0.7 m deep, so the "row" of obstacles is the right wall at 0.6 m from the lane centre and the
slot is the recess. The car starts on the lane centre, mid straight, facing along it.

SIM MADE TO LOOK LIKE THE CAR (all bridge_node parameters, all off by default, values read from
config/vehicle_params.yaml here, nothing typed):
  * /scan is cast from where the C1 sits on the car, sensors.lidar.mount_x_m ahead of the rear
    axle (the gym's pose is its centre of gravity, chassis.cg_to_rear_axle_m ahead of the rear
    axle), facing backwards (sensors.lidar.mount_yaw_rad), over 360 degrees at the C1's
    angular resolution. So safety_node and park_node run with the car's own LiDAR geometry from
    the binding, laser_yaw_from_vehicle_params left true, exactly as on the car; and the rear
    corridor sees straight behind the car, which the gym's own 270 degree scan from the CG
    cannot.
  * /odom/wheel is published from the sim's longitudinal speed (publish_wheel_odom), in
    vesc_odometry_node's shape, so park_node runs its floor profile with require_odometry true.
  * mirror_reverse_speed_control: f1tenth_gym's speed controller brakes a reversing car 20 times
    more weakly than a forward one (a 0.53 m roll after a stop command at -0.5 m/s), a model
    artefact the two-arc manoeuvre cannot absorb; the bridge mirrors the forward gains.

STACK. bridge_node -> /scan, /odom/wheel -> park_node (park.launch.py's floor-2026-10-07
profile) -> /drive_raw -> safety_node -> /drive -> bridge_node. The real command path, no remap;
park_node also reads the gated /drive (it aborts if safety_node holds the car).

PASS. ~/start accepted; park_node reaches DONE (SEARCH, SLOT_FOUND, DRIVE_TO_START, ARC_1,
ARC_2, STRAIGHTEN on the way, logged); the final ground-truth pose is parallel to the lane within
3 degrees and the whole body is inside the pocket; at every ground-truth sample the body (the
vehicle_params bounding box about the rear axle) keeps a clearance from the nominal walls larger
than the map's half-cell quantisation; no gym collision (yaw snap); a reverse command reached
the gated /drive; and safety_node never had to brake (no ttc / ttc_reverse brake engagement):
the plan's gate prediction kept the manoeuvre out of the corridors' brake range.
"""

from __future__ import annotations

import importlib.util
import math
import signal
import sys
import tempfile
import time
import unittest
from pathlib import Path

import launch
import launch_testing
import launch_testing.actions
import launch_testing.asserts
import rclpy
import yaml
from ackermann_msgs.msg import AckermannDriveStamped
from launch_ros.actions import Node as LaunchNode
from nav_msgs.msg import Odometry
from racer_msgs.msg import SafetyEvent
from std_msgs.msg import String
from std_srvs.srv import Trigger

REPO_ROOT = Path(__file__).resolve().parents[2]


def _load(name: str, path: Path):
    spec = importlib.util.spec_from_file_location(name, path)
    module = importlib.util.module_from_spec(spec)
    # Registered before it runs: its dataclasses look their module up in sys.modules.
    sys.modules[spec.name] = module
    spec.loader.exec_module(module)
    return module


# Track helpers shared with the reactive canaries.
common = _load(
    "l5_reactive_common", REPO_ROOT / "tests" / "l5_reactive_lap" / "l5_reactive_common.py"
)
park_launch = _load(
    "park_launch", REPO_ROOT / "ros_ws" / "src" / "racer_control" / "launch" / "park.launch.py"
)
with (REPO_ROOT / "config" / "vehicle_params.yaml").open() as _handle:
    VP = yaml.safe_load(_handle)

SEED = 17
TRACK_WIDTH_M = 14.0
TRACK_HEIGHT_M = 4.5
CORNER_RADIUS_M = 1.2
CENTRELINE_DS_M = 0.05
HALF_WIDTH_M = 0.6
POCKET_START_S_M = 1.6
POCKET_LENGTH_M = 1.8
POCKET_DEPTH_M = 0.7
# rounded_rectangle starts at (width / 2, 0) heading +x, so along the bottom straight s = x - 7.
START_X_M = TRACK_WIDTH_M / 2.0
POCKET_NEAR_X = START_X_M + POCKET_START_S_M
POCKET_FAR_X = POCKET_NEAR_X + POCKET_LENGTH_M
ROW_Y = -HALF_WIDTH_M
BACK_Y = -(HALF_WIDTH_M + POCKET_DEPTH_M)
STRAIGHT_X = (CORNER_RADIUS_M, TRACK_WIDTH_M - CORNER_RADIUS_M)
# The nominal walls of the bottom straight; the sim's walls are these quantised to its 0.05 m
# cells (bridge_node: free where a cell CENTRE is inside), so within 0.025 m of them.
WALLS = [
    ((STRAIGHT_X[0], ROW_Y), (POCKET_NEAR_X, ROW_Y)),
    ((POCKET_NEAR_X, ROW_Y), (POCKET_NEAR_X, BACK_Y)),
    ((POCKET_NEAR_X, BACK_Y), (POCKET_FAR_X, BACK_Y)),
    ((POCKET_FAR_X, BACK_Y), (POCKET_FAR_X, ROW_Y)),
    ((POCKET_FAR_X, ROW_Y), (STRAIGHT_X[1], ROW_Y)),
    ((STRAIGHT_X[0], HALF_WIDTH_M), (STRAIGHT_X[1], HALF_WIDTH_M)),
]
MAP_QUANTISATION_M = 0.025
MAX_TEST_WALL_S = 120.0
PARALLEL_LIMIT_RAD = math.radians(3.0)

CG_TO_REAR_AXLE_M = float(VP["chassis"]["cg_to_rear_axle_m"])
HALF_WIDTH_CAR_M = float(VP["chassis"]["width_m"]) / 2.0
FRONT_X_M = float(VP["chassis"]["wheelbase_m"]) + float(VP["chassis"]["front_overhang_m"])
REAR_X_M = float(VP["chassis"]["rear_overhang_m"])


def lidar_parameters() -> dict:
    lidar = VP["sensors"]["lidar"]
    beams = round(2.0 * math.pi / float(VP["sensors"]["lidar_spec"]["angular_resolution_rad"]))
    return {
        "lidar_offset_x_m": float(lidar["mount_x_m"]) - CG_TO_REAR_AXLE_M,
        "lidar_yaw_rad": float(lidar["mount_yaw_rad"]),
        # A full circle with `beams` rays exactly 2 pi / beams apart (no duplicated ray).
        "lidar_fov_rad": 2.0 * math.pi * (beams - 1) / beams,
        "lidar_num_beams": int(beams),
    }


def generate_test_description():
    raceline = Path(tempfile.gettempdir()) / "l5_park_raceline.csv"
    xs, ys = common.rounded_rectangle(
        TRACK_WIDTH_M, TRACK_HEIGHT_M, (CORNER_RADIUS_M,) * 4, CENTRELINE_DS_M
    )
    common.write_raceline_csv(raceline, xs, ys, speed_mps=1.0)
    bridge_params = {
        "seed": SEED,
        "raceline_path": str(raceline),
        "track_half_width_m": HALF_WIDTH_M,
        "pocket_start_m": POCKET_START_S_M,
        "pocket_length_m": POCKET_LENGTH_M,
        "pocket_depth_m": POCKET_DEPTH_M,
        "pocket_side": "right",
        "publish_wheel_odom": True,
        "mirror_reverse_speed_control": True,
    }
    bridge_params.update(lidar_parameters())
    bridge_node = LaunchNode(
        package="racer_gym_bridge",
        executable="bridge_node",
        name="bridge_node",
        parameters=[bridge_params],
    )
    # The car's own settings: LiDAR yaw from the binding (the bridge casts like the car's mount).
    safety_node = LaunchNode(package="racer_safety", executable="safety_node", name="safety_node")
    park_node = LaunchNode(
        package="racer_control",
        executable="park_node",
        name="park_node",
        parameters=[dict(park_launch.FLOOR_2026_10_07_PROFILE)],
        output="screen",
    )
    return launch.LaunchDescription(
        [bridge_node, safety_node, park_node, launch_testing.actions.ReadyToTest()]
    )


# -- Geometry (the planner's model, restated in Python for the ground truth) --------------------


def _point_segment(px, py, ax, ay, bx, by) -> float:
    dx, dy = bx - ax, by - ay
    length2 = dx * dx + dy * dy
    t = 0.0 if length2 == 0.0 else max(0.0, min(1.0, ((px - ax) * dx + (py - ay) * dy) / length2))
    return math.hypot(px - (ax + t * dx), py - (ay + t * dy))


def _cross(ox, oy, ax, ay, bx, by) -> float:
    return (ax - ox) * (by - oy) - (ay - oy) * (bx - ox)


def _segments_intersect(a, b, c, d) -> bool:
    d1 = _cross(*c, *d, *a)
    d2 = _cross(*c, *d, *b)
    d3 = _cross(*a, *b, *c)
    d4 = _cross(*a, *b, *d)
    return d1 * d2 < 0.0 and d3 * d4 < 0.0


def body_corners(x: float, y: float, yaw: float) -> list[tuple[float, float]]:
    """The vehicle_params bounding box about the rear axle at (x, y, yaw)."""
    c, s = math.cos(yaw), math.sin(yaw)
    local = [
        (FRONT_X_M, HALF_WIDTH_CAR_M),
        (FRONT_X_M, -HALF_WIDTH_CAR_M),
        (-REAR_X_M, -HALF_WIDTH_CAR_M),
        (-REAR_X_M, HALF_WIDTH_CAR_M),
    ]
    return [(x + c * lx - s * ly, y + s * lx + c * ly) for lx, ly in local]


def body_clearance(x: float, y: float, yaw: float) -> float:
    corners = body_corners(x, y, yaw)
    edges = [(corners[i], corners[(i + 1) % 4]) for i in range(4)]
    best = math.inf
    c, s = math.cos(yaw), math.sin(yaw)
    for a, b in WALLS:
        for p in (a, b):
            lx = c * (p[0] - x) + s * (p[1] - y)
            ly = -s * (p[0] - x) + c * (p[1] - y)
            if -REAR_X_M <= lx <= FRONT_X_M and abs(ly) <= HALF_WIDTH_CAR_M:
                return 0.0
        for e0, e1 in edges:
            if _segments_intersect(e0, e1, a, b):
                return 0.0
            best = min(
                best,
                _point_segment(*e0, *a, *b),
                _point_segment(*e1, *a, *b),
                _point_segment(*a, *e0, *e1),
                _point_segment(*b, *e0, *e1),
            )
    return best


def rear_axle(msg: Odometry) -> tuple[float, float, float]:
    """The gym's pose is its centre of gravity; the rear axle is cg_to_rear_axle_m behind it."""
    q = msg.pose.pose.orientation
    yaw = math.atan2(2.0 * (q.w * q.z + q.x * q.y), 1.0 - 2.0 * (q.y * q.y + q.z * q.z))
    x = msg.pose.pose.position.x - CG_TO_REAR_AXLE_M * math.cos(yaw)
    y = msg.pose.pose.position.y - CG_TO_REAR_AXLE_M * math.sin(yaw)
    return x, y, yaw


class TestParkCanary(unittest.TestCase):
    @classmethod
    def setUpClass(cls):
        rclpy.init()

    @classmethod
    def tearDownClass(cls):
        rclpy.shutdown()

    def test_finds_the_pocket_and_parks_in_it(self):
        node = rclpy.create_node("l5_park_test")
        try:
            self._run(node)
        finally:
            node.destroy_node()

    def _run(self, node) -> None:
        common.reset_sim(self, node)
        poses: list[tuple[float, float, float]] = []
        gated: list[float] = []
        events: list[SafetyEvent] = []
        statuses: list[str] = []
        node.create_subscription(
            Odometry,
            "/sim/ground_truth_odom",
            lambda m: poses.append(rear_axle(m)),
            common.reliable_qos(),
        )
        node.create_subscription(
            AckermannDriveStamped,
            "/drive",
            lambda m: gated.append(m.drive.speed),
            common.reliable_qos(),
        )
        node.create_subscription(
            SafetyEvent, "/safety/events", events.append, common.reliable_qos()
        )
        node.create_subscription(
            String, "/park_node/status", lambda m: statuses.append(m.data), common.reliable_qos()
        )
        start = node.create_client(Trigger, "/park_node/start")
        self.assertTrue(start.wait_for_service(timeout_sec=30.0), "/park_node/start not available")
        # Let the odometry and the scan reach park_node before asking.
        end = time.monotonic() + 2.0
        while time.monotonic() < end:
            rclpy.spin_once(node, timeout_sec=0.05)
        future = start.call_async(Trigger.Request())
        rclpy.spin_until_future_complete(node, future, timeout_sec=10.0)
        self.assertIsNotNone(future.result(), "/park_node/start did not answer")
        self.assertTrue(future.result().success, future.result().message)

        min_clearance = math.inf
        worst_at = None
        previous_yaw = None
        consumed = 0
        finished = None
        finished_at = None
        deadline = time.monotonic() + MAX_TEST_WALL_S
        while time.monotonic() < deadline:
            rclpy.spin_once(node, timeout_sec=0.05)
            while consumed < len(poses):
                x, y, yaw = poses[consumed]
                consumed += 1
                if previous_yaw is not None:
                    jump = math.atan2(math.sin(yaw - previous_yaw), math.cos(yaw - previous_yaw))
                    self.assertLess(
                        abs(jump), common.WALL_CONTACT_YAW_JUMP_RAD, f"gym collision at {x, y}"
                    )
                previous_yaw = yaw
                self.assertTrue(
                    STRAIGHT_X[0] < x < STRAIGHT_X[1], f"left the straight at ({x:.2f}, {y:.2f})"
                )
                clearance = body_clearance(x, y, yaw)
                if clearance < min_clearance:
                    min_clearance = clearance
                    worst_at = (x, y, yaw)
            if finished is None:
                for status in statuses[-5:]:
                    if "phase=DONE" in status or "phase=ABORT" in status:
                        finished = "DONE" if "phase=DONE" in status else "ABORT"
                        finished_at = time.monotonic()
            # Two seconds after the end for the car to stand still.
            if finished_at is not None and time.monotonic() - finished_at > 2.0:
                break
        self.assertIsNotNone(finished, f"park_node never finished; last status {statuses[-1:]}")
        x, y, yaw = poses[-1]
        brakes = [
            e
            for e in events
            if e.phase == SafetyEvent.PHASE_ENGAGE
            and e.severity == SafetyEvent.SEVERITY_BRAKE
            and e.source in ("ttc", "ttc_reverse")
        ]
        print(
            f"[l5_park] {finished}: final rear axle ({x:.3f}, {y:.3f}, {math.degrees(yaw):+.2f} "
            f"deg), pocket x [{POCKET_NEAR_X:.2f}, {POCKET_FAR_X:.2f}] y [{BACK_Y:.2f}, "
            f"{ROW_Y:.2f}]; min body clearance {min_clearance:.3f} m at {worst_at}; "
            f"{len(events)} safety event(s), {len(brakes)} obstacle brake(s); most reverse gated "
            f"{min(gated) if gated else float('nan'):.2f} m/s"
        )
        for e in events:
            print(
                f"[l5_park]   /safety/events: {e.source} sev {e.severity} phase {e.phase} {e.detail}"
            )
        self.assertEqual(finished, "DONE", statuses[-1:])
        self.assertLess(abs(math.atan2(math.sin(yaw), math.cos(yaw))), PARALLEL_LIMIT_RAD)
        for cx, cy in body_corners(x, y, yaw):
            self.assertGreater(cx, POCKET_NEAR_X, "body behind the pocket")
            self.assertLess(cx, POCKET_FAR_X, "body ahead of the pocket")
            self.assertLessEqual(cy, ROW_Y, "body sticks out of the pocket into the lane")
            self.assertGreater(cy, BACK_Y, "body through the pocket's back")
        self.assertGreater(min_clearance, MAP_QUANTISATION_M, f"body touched a wall at {worst_at}")
        self.assertTrue(gated and min(gated) < -0.3, "no reverse command reached the gated /drive")
        self.assertEqual(brakes, [], "safety_node had to brake the manoeuvre")


@launch_testing.post_shutdown_test()
class TestParkCanaryShutdown(unittest.TestCase):
    def test_exit_codes(self, proc_info):
        launch_testing.asserts.assertExitCodes(proc_info, allowable_exit_codes=[0, -signal.SIGINT])
