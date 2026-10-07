"""Shared harness for the L5 reactive canaries (tests/l5_reactive_lap).

Not a test file itself: each test_*.py in this directory loads it by path (launch_testing runs a
test file by path, so this directory is not on sys.path) and keeps only its own scenario:

  * test_gap_follow_lap_canary.py: config/tracks/gym_oval counter-clockwise (the raceline's own
    direction), lane centring on.
  * test_gap_follow_lap_canary_cw.py: the same track clockwise (bridge_node reverse_direction).
  * test_gap_follow_escape_canary.py: a walled loop with one SQUARE corner tighter than the
    car's turning circle, the full floor profile and safety_node in the loop; the reverse
    escape has to fire.
  * test_gap_follow_escape_canary_round.py: the same loop with that corner ROUND (0.25 m
    centreline radius), where the car's outer front corner meets the outside wall.

Everything physical is read from config/vehicle_params.yaml or the launch file's floor
profile, never typed in here.
"""

from __future__ import annotations

import csv
import importlib.util
import math
import tempfile
import time
from dataclasses import dataclass, field
from pathlib import Path

import rclpy
import yaml
from ackermann_msgs.msg import AckermannDriveStamped
from nav_msgs.msg import Odometry
from rclpy.qos import HistoryPolicy, QoSProfile, ReliabilityPolicy
from sensor_msgs.msg import LaserScan
from std_srvs.srv import Trigger

REPO_ROOT = Path(__file__).resolve().parents[2]
GYM_OVAL_RACELINE = REPO_ROOT / "config" / "tracks" / "gym_oval" / "raceline.csv"
VEHICLE_PARAMS_PATH = REPO_ROOT / "config" / "vehicle_params.yaml"
GAP_FOLLOW_LAUNCH = (
    REPO_ROOT / "ros_ws" / "src" / "racer_control" / "launch" / "gap_follow.launch.py"
)

RACELINE_HEADER = ("s_m", "x_m", "y_m", "heading_rad", "curvature_1pm", "target_speed_mps")


def reliable_qos() -> QoSProfile:
    return QoSProfile(
        reliability=ReliabilityPolicy.RELIABLE, history=HistoryPolicy.KEEP_LAST, depth=10
    )


def best_effort_qos() -> QoSProfile:
    return QoSProfile(
        reliability=ReliabilityPolicy.BEST_EFFORT, history=HistoryPolicy.KEEP_LAST, depth=10
    )


def chassis_half_width_m() -> float:
    with VEHICLE_PARAMS_PATH.open() as f:
        params = yaml.safe_load(f)
    return float(params["chassis"]["width_m"]) / 2.0


def floor_profile() -> dict:
    """gap_follow.launch.py's floor-2026-10-06 profile, loaded from the source launch file so the
    canaries always run what the car runs."""
    spec = importlib.util.spec_from_file_location("gap_follow_launch", GAP_FOLLOW_LAUNCH)
    module = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(module)
    return dict(module.FLOOR_2026_10_06_PROFILE)


def floor_centering_params() -> dict:
    """Just the lane centring entries of the floor profile."""
    return {k: v for k, v in floor_profile().items() if k.startswith("centering_")}


def load_raceline_xy(path: Path) -> tuple[list[float], list[float]]:
    xs: list[float] = []
    ys: list[float] = []
    header_seen = False
    with Path(path).open() as f:
        for line in f:
            if line.startswith("#") or not line.strip():
                continue
            fields = next(csv.reader([line]))
            if not header_seen:
                header_seen = True
                continue
            xs.append(float(fields[1]))
            ys.append(float(fields[2]))
    return xs, ys


@dataclass
class Loop:
    """A closed loop of points in the direction the car drives it, with arc length."""

    xs: list[float]
    ys: list[float]
    ss: list[float] = field(default_factory=list)
    length_m: float = 0.0

    def __post_init__(self) -> None:
        self.ss = [0.0]
        for i in range(1, len(self.xs)):
            step = math.hypot(self.xs[i] - self.xs[i - 1], self.ys[i] - self.ys[i - 1])
            self.ss.append(self.ss[-1] + step)
        closing = math.hypot(self.xs[0] - self.xs[-1], self.ys[0] - self.ys[-1])
        self.length_m = self.ss[-1] + closing

    @classmethod
    def from_raceline(cls, path: Path, reverse: bool = False) -> Loop:
        """The raceline as the car drives it: reversed for bridge_node reverse_direction:=true,
        which reverses the waypoint order the same way."""
        xs, ys = load_raceline_xy(path)
        if reverse:
            xs, ys = xs[::-1], ys[::-1]
        return cls(xs, ys)

    def signed_area(self) -> float:
        n = len(self.xs)
        return 0.5 * sum(
            self.xs[i] * self.ys[(i + 1) % n] - self.xs[(i + 1) % n] * self.ys[i] for i in range(n)
        )

    def nearest(self, x: float, y: float, prev_index: int | None, window: int):
        """(s, index, distance) of the nearest point: windowed around prev_index (see the
        tracker canary's _nearest_s for why), or over the whole loop when prev_index is None."""
        n = len(self.xs)
        indices = (
            range(n)
            if prev_index is None
            else ((prev_index + o) % n for o in range(-window, window + 1))
        )
        best_i, best_d2 = 0, math.inf
        for i in indices:
            d2 = (self.xs[i] - x) ** 2 + (self.ys[i] - y) ** 2
            if d2 < best_d2:
                best_d2, best_i = d2, i
        return self.ss[best_i], best_i, math.sqrt(best_d2)


def write_raceline_csv(path: Path, xs: list[float], ys: list[float], speed_mps: float) -> None:
    """A tools/raceline-format CSV (racer_gym_bridge's track_loader reads columns 1, 2 and 5;
    heading and curvature are filled from the points for completeness)."""
    n = len(xs)
    with Path(path).open("w", newline="") as f:
        f.write("# raceline generated by tests/l5_reactive_lap/l5_reactive_common.py (test-only)\n")
        writer = csv.writer(f)
        writer.writerow(RACELINE_HEADER)
        s = 0.0
        for i in range(n):
            j = (i + 1) % n
            k = (i - 1) % n
            heading = math.atan2(ys[j] - ys[i], xs[j] - xs[i])
            prev_heading = math.atan2(ys[i] - ys[k], xs[i] - xs[k])
            step = math.hypot(xs[j] - xs[i], ys[j] - ys[i])
            turn = math.atan2(math.sin(heading - prev_heading), math.cos(heading - prev_heading))
            curvature = turn / step if step > 0.0 else 0.0
            writer.writerow(
                [
                    f"{s:.4f}",
                    f"{xs[i]:.5f}",
                    f"{ys[i]:.5f}",
                    f"{heading:.5f}",
                    f"{curvature:.5f}",
                    f"{speed_mps:.3f}",
                ]
            )
            s += step


def rounded_rectangle(
    width_m: float, height_m: float, radii_m: tuple[float, float, float, float], ds_m: float
) -> tuple[list[float], list[float]]:
    """Centreline of a counter-clockwise rounded rectangle [0, width] x [0, height], corner radii
    (bottom-right, top-right, top-left, bottom-left), starting mid bottom straight heading +x,
    sampled about every ds_m."""
    r_br, r_tr, r_tl, r_bl = radii_m
    pieces = [
        ("line", (width_m / 2.0, 0.0), (width_m - r_br, 0.0)),
        ("arc", (width_m - r_br, r_br), r_br, -math.pi / 2.0, 0.0),
        ("line", (width_m, r_br), (width_m, height_m - r_tr)),
        ("arc", (width_m - r_tr, height_m - r_tr), r_tr, 0.0, math.pi / 2.0),
        ("line", (width_m - r_tr, height_m), (r_tl, height_m)),
        ("arc", (r_tl, height_m - r_tl), r_tl, math.pi / 2.0, math.pi),
        ("line", (0.0, height_m - r_tl), (0.0, r_bl)),
        ("arc", (r_bl, r_bl), r_bl, math.pi, 1.5 * math.pi),
        ("line", (r_bl, 0.0), (width_m / 2.0, 0.0)),
    ]
    xs: list[float] = []
    ys: list[float] = []
    for piece in pieces:
        if piece[0] == "line":
            (x0, y0), (x1, y1) = piece[1], piece[2]
            length = math.hypot(x1 - x0, y1 - y0)
            n = max(1, round(length / ds_m))
            for i in range(n):
                t = i / n
                xs.append(x0 + t * (x1 - x0))
                ys.append(y0 + t * (y1 - y0))
        else:
            (cx, cy), r, a0, a1 = piece[1], piece[2], piece[3], piece[4]
            n = max(1, round(r * abs(a1 - a0) / ds_m))
            for i in range(n):
                a = a0 + (a1 - a0) * i / n
                xs.append(cx + r * math.cos(a))
                ys.append(cy + r * math.sin(a))
    return xs, ys


def reset_sim(test, node) -> None:
    reset_client = node.create_client(Trigger, "/sim/reset")
    test.assertTrue(reset_client.wait_for_service(timeout_sec=30.0), "/sim/reset not available")
    future = reset_client.call_async(Trigger.Request())
    rclpy.spin_until_future_complete(node, future, timeout_sec=30.0)
    test.assertIsNotNone(future.result(), "/sim/reset call did not complete")


def check_walls(test, node, track_half_width_m: float) -> None:
    """The car starts on the raceline, mid-corridor: the rays straight out to each side must
    read about the corridor half width (proves the walls exist and are not offset)."""
    scans: list[LaserScan] = []
    sub = node.create_subscription(LaserScan, "/scan", scans.append, best_effort_qos())
    deadline = time.monotonic() + 10.0
    while not scans and time.monotonic() < deadline:
        rclpy.spin_once(node, timeout_sec=0.05)
    node.destroy_subscription(sub)
    test.assertTrue(scans, "no /scan from bridge_node")
    scan = scans[0]
    for side in (-math.pi / 2.0, math.pi / 2.0):
        index = round((side - scan.angle_min) / scan.angle_increment)
        test.assertAlmostEqual(
            scan.ranges[index],
            track_half_width_m,
            delta=0.2,
            msg=f"ray at {side:+.2f} rad should hit a corridor wall ~{track_half_width_m} m away",
        )


# -- The gym_oval lap canary, either direction --------------------------------------------------

LAP_SEED = 11
LAP_TARGET_LAPS = 2
# Corridor half width (m): a 1.6 m wide track, in the range of F1TENTH-style venue tracks for a
# 1/10 car. A property of the simulated track, not of the vehicle.
LAP_TRACK_HALF_WIDTH_M = 0.8
LAP_MAX_SPEED_MPS = 3.0
# Wall-clock band for LAP_TARGET_LAPS laps.
#   * LOW is physical, not statistical: two laps of gym_oval (~34.85 m each) at the 3.0 m/s cap
#     take at least 23.2 s of sim time, and the bridge never steps faster than wall-clock, so
#     anything under 20 s means the lap counting is broken.
#   * HIGH: measured locally in the ros-dev image (Apple silicon, Docker) at about 26 s for two
#     laps. The tracker canary saw a ~40% swing between CI runs of the same code, so this allows
#     a bit over twice the local measurement before calling it a regression.
LAP_TIME_LOW_S = 20.0
LAP_TIME_HIGH_S = 60.0
LAP_MAX_TEST_WALL_S = 120.0
LAP_STALL_TIMEOUT_S = 8.0


def lap_gap_follow_params() -> dict:
    """gap_follow_node for the lap canary: node defaults, the canary's speed cap, the sim's yaw,
    and the floor profile's lane centring."""
    params = {
        "max_speed_mps": LAP_MAX_SPEED_MPS,
        # racer_gym_bridge's /scan is aligned to the vehicle (yaw 0), not the real car's mount.
        "laser_yaw_from_vehicle_params": False,
    }
    params.update(floor_centering_params())
    return params


def lap_canary_launch(reverse_direction: bool, launch, launch_testing, LaunchNode):
    """The lap canary's launch description. SIM-ONLY TOPIC REMAP, same shim and same warning as
    tests/l5_tracker_lap: gap_follow_node's /drive_raw is remapped to /drive so the bridge is
    driven directly, with no safety_node in the loop. TEST-ONLY; on the car /drive comes only
    from safety_node."""
    bridge_node = LaunchNode(
        package="racer_gym_bridge",
        executable="bridge_node",
        name="bridge_node",
        parameters=[
            {
                "seed": LAP_SEED,
                "raceline_path": str(GYM_OVAL_RACELINE),
                "track_half_width_m": LAP_TRACK_HALF_WIDTH_M,
                "reverse_direction": reverse_direction,
            }
        ],
    )
    gap_follow_node = LaunchNode(
        package="racer_control",
        executable="gap_follow_node",
        name="gap_follow_node",
        parameters=[lap_gap_follow_params()],
        remappings=[("/drive_raw", "/drive")],
    )
    return launch.LaunchDescription(
        [bridge_node, gap_follow_node, launch_testing.actions.ReadyToTest()]
    )


def run_lap_canary(test, reverse_direction: bool, label: str) -> None:
    loop = Loop.from_raceline(GYM_OVAL_RACELINE, reverse=reverse_direction)
    # gym_oval's raceline runs counter-clockwise; reversed it runs clockwise.
    if reverse_direction:
        test.assertLess(loop.signed_area(), 0.0, "expected a clockwise lap")
    else:
        test.assertGreater(loop.signed_area(), 0.0, "expected a counter-clockwise lap")
    max_lateral_m = LAP_TRACK_HALF_WIDTH_M - chassis_half_width_m()
    node = rclpy.create_node("l5_reactive_lap_test")
    try:
        reset_sim(test, node)
        check_walls(test, node, LAP_TRACK_HALF_WIDTH_M)
        result = run_laps(
            test,
            node,
            loop,
            target_laps=LAP_TARGET_LAPS,
            max_wall_s=LAP_MAX_TEST_WALL_S,
            stall_timeout_s=LAP_STALL_TIMEOUT_S,
            max_lateral_m=max_lateral_m,
        )
        print(
            f"[l5_reactive_lap {label}] {LAP_TARGET_LAPS}-lap time {result.lap_time_s:.3f}s "
            f"(band [{LAP_TIME_LOW_S}, {LAP_TIME_HIGH_S}]s), worst distance from the raceline "
            f"{result.worst_lateral_m:.3f} m (limit {max_lateral_m:.3f} m), lane centring gain "
            f"{floor_centering_params()['centering_gain']}"
        )
        test.assertGreaterEqual(
            result.lap_time_s,
            LAP_TIME_LOW_S,
            f"suspiciously fast for a reactive controller capped at {LAP_MAX_SPEED_MPS} m/s: "
            "check the lap counting",
        )
        test.assertLessEqual(
            result.lap_time_s, LAP_TIME_HIGH_S, "regression in gap-follow lap performance"
        )
    finally:
        node.destroy_node()


WALL_CONTACT_YAW_JUMP_RAD = 0.3


@dataclass
class LapResult:
    laps: int
    lap_time_s: float
    worst_lateral_m: float


def run_laps(
    test,
    node,
    loop: Loop,
    *,
    target_laps: int,
    max_wall_s: float,
    stall_timeout_s: float,
    max_lateral_m: float,
    lateral_windowed: bool = True,
    progress_epsilon_m: float = 0.05,
    window: int = 50,
    on_spin=None,
) -> LapResult:
    """Watches /sim/ground_truth_odom until `target_laps` laps are done (or `max_wall_s`).

    Progress is the windowed nearest-point arc length along `loop` (in the driving direction),
    unwrapped. Fails at once if the car is ever more than `max_lateral_m` from the loop (the
    nearest point overall when `lateral_windowed` is false, which is right where the car may
    leave the centreline far behind in a tight corner), touches a wall, or makes no forward
    progress for `stall_timeout_s`. `on_spin` is called after every spin (for extra
    subscriptions).

    WALL CONTACT. f1tenth_gym checks every step whether the car's own box (wheelbase long, the
    chassis width wide, about its pose) is about to hit the map; when it is, its collision
    handler sets state[3:] to zero, which stops the car AND zeroes its yaw (state[4]), and the
    episode reports terminated. So contact shows up in ground truth as the heading snapping to
    zero between two consecutive samples, far faster than the car can turn (about 1.2 rad/s at
    full lock and 0.9 m/s, 0.012 rad per 100 Hz step). A jump of more than
    WALL_CONTACT_YAW_JUMP_RAD fails the run: a car that touched a wall did not drive the lap,
    and the zeroed heading would corrupt everything after it."""
    samples: list[tuple[float, float, float, float]] = []

    def on_odom(msg) -> None:
        q = msg.pose.pose.orientation
        yaw = math.atan2(2.0 * (q.w * q.z + q.x * q.y), 1.0 - 2.0 * (q.y * q.y + q.z * q.z))
        samples.append((msg.pose.pose.position.x, msg.pose.pose.position.y, yaw, time.monotonic()))

    node.create_subscription(Odometry, "/sim/ground_truth_odom", on_odom, reliable_qos())
    test_start = time.monotonic()
    deadline = test_start + max_wall_s
    last_progress_wall = test_start
    unwrapped_s = None
    start_s = 0.0
    prev_index = 0
    lap_walltimes: list[float] = []
    first_walltime = None
    worst_lateral_m = 0.0
    best_progress = 0.0
    previous_yaw = None
    consumed = 0
    while time.monotonic() < deadline:
        rclpy.spin_once(node, timeout_sec=0.05)
        if on_spin is not None:
            on_spin()
        while consumed < len(samples):
            x, y, yaw, wall_t = samples[consumed]
            consumed += 1
            if previous_yaw is not None:
                jump = math.atan2(math.sin(yaw - previous_yaw), math.cos(yaw - previous_yaw))
                if abs(jump) > WALL_CONTACT_YAW_JUMP_RAD:
                    test.fail(
                        f"wall contact at ({x:.2f}, {y:.2f}): the heading jumped "
                        f"{math.degrees(previous_yaw):.0f} -> {math.degrees(yaw):.0f} deg in one "
                        "step (f1tenth_gym's collision handler zeroes the yaw)"
                    )
            previous_yaw = yaw
            if first_walltime is None:
                first_walltime = wall_t
                last_progress_wall = wall_t
                prev_index = loop.nearest(x, y, None, window)[1]
            nearest_s, prev_index, windowed_lateral_m = loop.nearest(x, y, prev_index, window)
            lateral_m = (
                windowed_lateral_m if lateral_windowed else loop.nearest(x, y, None, window)[2]
            )
            worst_lateral_m = max(worst_lateral_m, lateral_m)
            if lateral_m > max_lateral_m:
                test.fail(
                    f"left the track: {lateral_m:.3f} m from the raceline at ({x:.2f}, {y:.2f}), "
                    f"limit {max_lateral_m:.3f} m"
                )
            if unwrapped_s is None:
                unwrapped_s = nearest_s
                start_s = nearest_s
            else:
                delta = nearest_s - (unwrapped_s % loop.length_m)
                if delta < -loop.length_m / 2.0:
                    delta += loop.length_m
                elif delta > loop.length_m / 2.0:
                    delta -= loop.length_m
                unwrapped_s += delta
            progress = unwrapped_s - start_s
            # Stall: no new best progress for stall_timeout_s. Backing up (a reverse escape)
            # loses progress for a while without counting as a stall until the timeout.
            if progress > best_progress + progress_epsilon_m:
                best_progress = progress
                last_progress_wall = wall_t
            if wall_t - last_progress_wall > stall_timeout_s:
                test.fail(
                    f"no forward progress for > {stall_timeout_s}s at ({x:.2f}, {y:.2f}), "
                    f"{progress:.2f} m into the run (wedged, or turned round)"
                )
            laps = int(progress // loop.length_m)
            while len(lap_walltimes) < laps:
                lap_walltimes.append(wall_t)
        if len(lap_walltimes) >= target_laps:
            break
    test.assertGreaterEqual(
        len(lap_walltimes),
        target_laps,
        f"only completed {len(lap_walltimes)}/{target_laps} lap(s) within {max_wall_s}s",
    )
    return LapResult(
        laps=len(lap_walltimes),
        lap_time_s=lap_walltimes[target_laps - 1] - first_walltime,
        worst_lateral_m=worst_lateral_m,
    )


# -- The reverse escape canary: a loop with one corner tighter than the turning circle ---------

ESCAPE_SEED = 13
ESCAPE_TRACK_HALF_WIDTH_M = 0.55
ESCAPE_TRACK_WIDTH_M = 5.0
ESCAPE_TRACK_HEIGHT_M = 3.5
ESCAPE_EASY_RADIUS_M = 1.2
ESCAPE_CENTRELINE_DS_M = 0.05
ESCAPE_TARGET_LAPS = 1
# Generous: the floor profile tops out at 0.9 m/s (a lap of about 15 m is at least 17 s), and
# each escape costs escape_after_s (1.5 s) of waiting plus a couple of seconds of backing and
# re-trying. Measured locally: see docs/notes/reactive-control-port-2026-10-05.md.
ESCAPE_MAX_LAP_S = 150.0
ESCAPE_MAX_TEST_WALL_S = 200.0
# A corner that needs several back-and-forth legs makes no NEW forward progress for a while.
ESCAPE_STALL_TIMEOUT_S = 30.0


def escape_track_points(tight_radius_m: float) -> tuple[list[float], list[float]]:
    """The escape loop's centreline: three ESCAPE_EASY_RADIUS_M corners and, first after the
    start, one of `tight_radius_m`."""
    return rounded_rectangle(
        ESCAPE_TRACK_WIDTH_M,
        ESCAPE_TRACK_HEIGHT_M,
        # (bottom-right, top-right, top-left, bottom-left): the tight corner comes first.
        (tight_radius_m, ESCAPE_EASY_RADIUS_M, ESCAPE_EASY_RADIUS_M, ESCAPE_EASY_RADIUS_M),
        ESCAPE_CENTRELINE_DS_M,
    )


def escape_canary_launch(tight_radius_m: float, launch, launch_testing, LaunchNode):
    """bridge_node -> /scan -> gap_follow_node (the full floor profile) -> /drive_raw ->
    safety_node -> /drive -> bridge_node: the real command path, no test-only remap."""
    raceline = (
        Path(tempfile.gettempdir())
        / f"l5_reactive_tight_corner_{round(tight_radius_m * 1000)}mm_raceline.csv"
    )
    xs, ys = escape_track_points(tight_radius_m)
    write_raceline_csv(raceline, xs, ys, speed_mps=1.0)
    bridge_node = LaunchNode(
        package="racer_gym_bridge",
        executable="bridge_node",
        name="bridge_node",
        parameters=[
            {
                "seed": ESCAPE_SEED,
                "raceline_path": str(raceline),
                "track_half_width_m": ESCAPE_TRACK_HALF_WIDTH_M,
            }
        ],
    )
    safety_node = LaunchNode(
        package="racer_safety",
        executable="safety_node",
        name="safety_node",
        parameters=[{"laser_yaw_from_vehicle_params": False}],
    )
    gap_params = floor_profile()
    # racer_gym_bridge's /scan is aligned to the vehicle (yaw 0), not the real car's mount.
    gap_params["laser_yaw_from_vehicle_params"] = False
    gap_follow_node = LaunchNode(
        package="racer_control",
        executable="gap_follow_node",
        name="gap_follow_node",
        parameters=[gap_params],
        output="screen",
    )
    return launch.LaunchDescription(
        [bridge_node, safety_node, gap_follow_node, launch_testing.actions.ReadyToTest()]
    )


def run_escape_canary(test, tight_radius_m: float, label: str) -> None:
    """PASS: the escape fires at least once (a reverse request on /drive_raw AND a reverse
    command on the gated /drive), and the car still completes a lap inside ESCAPE_MAX_LAP_S
    without leaving the corridor and without touching a wall (run_laps)."""
    test.assertTrue(floor_profile()["reverse_escape"], "profile lost reverse_escape")
    xs, ys = escape_track_points(tight_radius_m)
    loop = Loop(xs, ys)
    max_lateral_m = ESCAPE_TRACK_HALF_WIDTH_M - chassis_half_width_m()
    node = rclpy.create_node("l5_reactive_escape_test")
    try:
        reset_sim(test, node)
        check_walls(test, node, ESCAPE_TRACK_HALF_WIDTH_M)
        requests: list[float] = []
        gated: list[float] = []
        node.create_subscription(
            AckermannDriveStamped,
            "/drive_raw",
            lambda m: requests.append(m.drive.speed),
            reliable_qos(),
        )
        node.create_subscription(
            AckermannDriveStamped,
            "/drive",
            lambda m: gated.append(m.drive.speed),
            reliable_qos(),
        )
        start = time.monotonic()
        result = run_laps(
            test,
            node,
            loop,
            target_laps=ESCAPE_TARGET_LAPS,
            max_wall_s=ESCAPE_MAX_TEST_WALL_S,
            stall_timeout_s=ESCAPE_STALL_TIMEOUT_S,
            max_lateral_m=max_lateral_m,
            lateral_windowed=False,
        )
        escapes = sum(1 for a, b in zip([0.0, *requests], requests, strict=False) if a >= 0.0 > b)
        print(
            f"[l5_reactive_escape {label}] lap time {result.lap_time_s:.3f}s (limit "
            f"{ESCAPE_MAX_LAP_S}s), {escapes} reverse escape(s) requested, worst distance from "
            f"the centreline {result.worst_lateral_m:.3f} m (limit {max_lateral_m:.3f} m), wall "
            f"{time.monotonic() - start:.1f}s"
        )
        test.assertGreaterEqual(escapes, 1, "the reverse escape never fired")
        test.assertTrue(
            any(s < 0.0 for s in gated), "no reverse command ever reached the gated /drive"
        )
        test.assertLessEqual(result.lap_time_s, ESCAPE_MAX_LAP_S)
        test.assertTrue(math.isfinite(result.lap_time_s))
    finally:
        node.destroy_node()
