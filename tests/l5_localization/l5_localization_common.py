"""Shared harness for the L5 localisation canaries (tests/l5_localization).

Not a test file itself: each test_*.py here loads it by path (launch_testing runs a test file by
path, so this directory is not on sys.path) and keeps only its scenario and its thresholds.

What every scenario runs, in the shape the car will run it (docs/notes/localization-2026-11-03.md):

  * racer_gym_bridge with the real chain's frames: the ground-truth map -> base_link TF OFF
    (publish_ground_truth_tf), base_link at the rear axle like the car's (base_link_offset_x_m
    = -cg_to_rear_axle_m), /odom/wheel at 50 Hz like the VESC driver, and /scan cast from where
    the C1 sits on the car (vehicle_params sensors.lidar), 720 beams over 360 degrees, 10 Hz,
    cut at the C1's 12 m. With noise on (the default; L5_NOISE=0 turns it off for tuning) the
    wheel speed reads 2% high with 0.02 m/s noise and a little wheelspin, the scan has 2 cm of
    extra range noise, and the steering has a 0.01 rad trim error nobody is told about.
  * racer_bringup's own mapping.launch.py or localization.launch.py (wheel_twist_node, the
    robot_localization EKF, slam_toolbox or AMCL + map_server + lifecycle manager), unmodified.
  * racer_control's tracker_node on a raceline for the track.

SIM-ONLY TOPIC REMAP, the same test-only shim as tests/l5_tracker_lap: tracker_node's /drive_raw
is remapped to /drive, so the bridge (and wheel_twist_node, which reads /drive) see the
tracker's command directly, with no safety_node in the loop. That keeps these canaries about
localisation; the command path has its own canaries. On the car /drive comes only from
safety_node.

Ground truth. /sim/ground_truth_odom is the gym's pose, its centre of gravity. The estimate is
base_link, the rear axle, so ground truth is moved back by cg_to_rear_axle_m along the heading
before any comparison. The estimate compared is TF map -> base_link looked up at each ground-truth
sample's own stamp (AMCL's or slam_toolbox's map -> odom composed with the EKF's odom -> base_link):
the pose tracker_node drives on, at the time the car was really there.

Tuning overrides (environment variables, all optional; CI sets none of them): L5_NOISE (0/1),
L5_SPEED_MPS, L5_LAPS, L5_EKF_PARAMS, L5_AMCL_PARAMS, L5_SLAM_PARAMS (paths to parameter files
used instead of racer_state's), L5_SEED, L5_RESULTS (a JSON file the run's metrics are appended
to). Everything physical comes from config/vehicle_params.yaml.
"""

from __future__ import annotations

import csv
import json
import math
import os
import tempfile
import time
from dataclasses import dataclass, field
from pathlib import Path

import numpy as np
import rclpy
import yaml
from nav_msgs.msg import OccupancyGrid, Odometry
from rclpy.duration import Duration
from rclpy.qos import DurabilityPolicy, HistoryPolicy, QoSProfile, ReliabilityPolicy
from rclpy.time import Time
from tf2_ros import Buffer, TransformException, TransformListener

REPO_ROOT = Path(__file__).resolve().parents[2]
GYM_OVAL_RACELINE = REPO_ROOT / "config" / "tracks" / "gym_oval" / "raceline.csv"
BRINGUP_LAUNCH_DIR = REPO_ROOT / "ros_ws" / "src" / "racer_bringup" / "launch"
RACELINE_HEADER = ("s_m", "x_m", "y_m", "heading_rad", "curvature_1pm", "target_speed_mps")

with (REPO_ROOT / "config" / "vehicle_params.yaml").open() as _handle:
    VP = yaml.safe_load(_handle)

CG_TO_REAR_AXLE_M = float(VP["chassis"]["cg_to_rear_axle_m"])
CAR_HALF_WIDTH_M = float(VP["chassis"]["width_m"]) / 2.0
SPEED_CAP_MPS = float(VP["actuation"]["throttle_full_scale_mps"])
LIDAR_SPEC = VP["sensors"]["lidar_spec"]

# The noise the canaries run with (see the module docstring). Properties of the simulated
# sensors, chosen to be a little worse than what the car has shown so far: the VESC's speed
# read 0.78 for 0.80 commanded on 2026-11-01 (2.5% low; here 2% high, the sign does not matter
# to the EKF), the C1 datasheet quotes about 1 to 2% range accuracy at short range.
NOISE = {
    "wheel_speed_scale": 1.02,
    "wheel_speed_noise_std_mps": 0.02,
    "wheel_slip_s": 0.01,
    "scan_noise_std_m": 0.02,
    "steering_offset_rad": 0.01,
}

WALL_CONTACT_YAW_JUMP_RAD = 0.3


def env_float(name: str, default: float) -> float:
    value = os.environ.get(name)
    return float(value) if value else default


def env_int(name: str, default: int) -> int:
    value = os.environ.get(name)
    return int(value) if value else default


def noise_on() -> bool:
    return os.environ.get("L5_NOISE", "1") != "0"


def sim_time_on() -> bool:
    """The canaries run on simulated time (GitHub issue 156): bridge_node owns /clock and paces
    its steps, every node under test has use_sim_time:=true. L5_SIM_TIME=0 runs the old
    wall-clock sim, kept to measure the before."""
    return os.environ.get("L5_SIM_TIME", "1") != "0"


#: Outputs the bridge's lockstep gate watches (bridge_node lockstep_ack): sim time may not lead
#: them by more than LOCKSTEP_MAX_LAG_S. The EKF's /odom; the map -> odom of slam_toolbox or AMCL
#: (child frame odom, stamped up to 0.2 s ahead of its data, their transform tolerance); and the
#: tracker's command on /drive (the canaries remap /drive_raw onto it), so the plant never runs
#: ahead of the control loop. tracker_node ticks on /clock (ros_clock_timers), not on a wall
#: timer, which is what ties a command to the step it answers. Tried and rejected, see
#: docs/notes/sim-time-lockstep-2026-11-14.md: acknowledging slam_toolbox's or AMCL's /pose (they
#: publish only when the car has moved, so the gate sits on its timeout while the car stands still
#: and sim time crawls), and a lag of 0.03 s or 0 on /odom (no measurable gain, and 0 stalls).
LOCKSTEP_ACK = [
    "/odom",
    "tf:odom:0.2",
    "ackermann_msgs/msg/AckermannDriveStamped@/drive",
]
LOCKSTEP_MAX_LAG_S = 0.1


def create_node(name: str):
    """The test's own node, on /clock when the sim is."""
    if not sim_time_on():
        return rclpy.create_node(name)
    from rclpy.parameter import Parameter

    return rclpy.create_node(
        name, parameter_overrides=[Parameter("use_sim_time", Parameter.Type.BOOL, True)]
    )


def reliable_qos(depth: int = 10) -> QoSProfile:
    return QoSProfile(
        reliability=ReliabilityPolicy.RELIABLE, history=HistoryPolicy.KEEP_LAST, depth=depth
    )


def latched_qos() -> QoSProfile:
    return QoSProfile(
        reliability=ReliabilityPolicy.RELIABLE,
        durability=DurabilityPolicy.TRANSIENT_LOCAL,
        history=HistoryPolicy.KEEP_LAST,
        depth=1,
    )


def lidar_parameters() -> dict:
    """The bridge's /scan cast like the car's C1 (same recipe as tests/l5_park), plus its rate and
    maximum range."""
    lidar = VP["sensors"]["lidar"]
    beams = round(2.0 * math.pi / float(LIDAR_SPEC["angular_resolution_rad"]))
    return {
        "lidar_offset_x_m": float(lidar["mount_x_m"]) - CG_TO_REAR_AXLE_M,
        "lidar_yaw_rad": env_float("L5_LIDAR_YAW", float(lidar["mount_yaw_rad"])),
        "lidar_fov_rad": 2.0 * math.pi * (beams - 1) / beams,
        "lidar_num_beams": int(beams),
        "scan_rate_hz": float(LIDAR_SPEC["nominal_scan_rate_hz"]),
        "scan_range_max_m": float(LIDAR_SPEC["range_max_m"]),
    }


# -- Tracks --------------------------------------------------------------------------------------


@dataclass
class Track:
    """A scenario's track: what the bridge builds, the loop the car drives, and its raceline."""

    name: str
    bridge_params: dict
    xs: list[float]
    ys: list[float]
    lane_half_width_m: float
    raceline_csv: Path
    # tracker_node lookahead for this track (its defaults suit a big track, not a 1.1 m lane).
    tracker_overrides: dict = field(default_factory=dict)
    ss: list[float] = field(default_factory=list)
    length_m: float = 0.0

    def __post_init__(self) -> None:
        self.ss = [0.0]
        for i in range(1, len(self.xs)):
            step = math.hypot(self.xs[i] - self.xs[i - 1], self.ys[i] - self.ys[i - 1])
            self.ss.append(self.ss[-1] + step)
        self.length_m = self.ss[-1] + math.hypot(self.xs[0] - self.xs[-1], self.ys[0] - self.ys[-1])

    def nearest(self, x: float, y: float, prev_index: int | None, window: int = 60):
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


def write_raceline_csv(path: Path, xs, ys, speeds) -> None:
    """A tools/raceline-format CSV (tracker_node reads it; heading and curvature from the
    points)."""
    n = len(xs)
    with Path(path).open("w", newline="") as f:
        f.write("# raceline generated by tests/l5_localization (test-only)\n")
        writer = csv.writer(f)
        writer.writerow(RACELINE_HEADER)
        s = 0.0
        for i in range(n):
            j, k = (i + 1) % n, (i - 1) % n
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
                    f"{speeds[i]:.3f}",
                ]
            )
            s += step


def _load_raceline(path: Path) -> tuple[list[float], list[float], list[float]]:
    xs, ys, vs = [], [], []
    header_seen = False
    with path.open() as f:
        for line in f:
            if line.startswith("#") or not line.strip():
                continue
            fields = next(csv.reader([line]))
            if not header_seen:
                header_seen = True
                continue
            xs.append(float(fields[1]))
            ys.append(float(fields[2]))
            vs.append(float(fields[5]))
    return xs, ys, vs


def scratch_dir(label: str) -> Path:
    path = Path(tempfile.gettempdir()) / f"l5_localization_{label}"
    path.mkdir(parents=True, exist_ok=True)
    return path


def gym_oval_track(half_width_m: float, speed_cap_mps: float, label: str) -> Track:
    """config/tracks/gym_oval (the committed raceline, a 34.9 m stadium) inside a walled corridor;
    the tracker follows the raceline with its speeds capped at speed_cap_mps."""
    xs, ys, vs = _load_raceline(GYM_OVAL_RACELINE)
    raceline = scratch_dir(label) / "raceline.csv"
    write_raceline_csv(raceline, xs, ys, [min(v, speed_cap_mps) for v in vs])
    return Track(
        name="gym_oval",
        bridge_params={
            "raceline_path": str(GYM_OVAL_RACELINE),
            "track_half_width_m": half_width_m,
        },
        xs=xs,
        ys=ys,
        lane_half_width_m=half_width_m,
        raceline_csv=raceline,
    )


# tracker_node's default lookahead (0.4 to 1.5 m) cuts a 0.8 m corner of a 1.1 m lane into the
# inside wall even on ground truth at 1.5 m/s (measured, tuning table). A shorter one for the
# living-room track; a controller setting, not a localisation one.
LANE_TRACKER_LOOKAHEAD = {
    "lookahead_min_m": 0.35,
    "lookahead_max_m": 0.7,
    "lookahead_curvature_ref_1pm": 1.25,
}


def lane_track(name: str, corner_radius_m: float, speed_mps: float, label: str) -> Track:
    """One of racer_gym_bridge's living-room lane tracks (5.5 x 4.0 m centreline loop, 1.1 m
    lane, thin walls), the tracker on its centreline at a constant speed."""
    from racer_gym_bridge.track_loader import DEFAULT_LANE_WIDTH_M, build_synthetic_track

    synthetic = build_synthetic_track(name, corner_radius_m=corner_radius_m)
    xs = [float(v) for v in synthetic.centreline_x]
    ys = [float(v) for v in synthetic.centreline_y]
    raceline = scratch_dir(label) / "raceline.csv"
    write_raceline_csv(raceline, xs, ys, [speed_mps] * len(xs))
    return Track(
        name=name,
        bridge_params={"synthetic_track": name, "synthetic_corner_radius_m": corner_radius_m},
        xs=xs,
        ys=ys,
        lane_half_width_m=DEFAULT_LANE_WIDTH_M / 2.0,
        raceline_csv=raceline,
        tracker_overrides=dict(LANE_TRACKER_LOOKAHEAD),
    )


def bridge_parameters(track: Track, seed: int) -> dict:
    params = {
        "seed": seed,
        "publish_wheel_odom": True,
        "wheel_odom_rate_hz": 50.0,
        "publish_ground_truth_tf": False,
        "base_link_offset_x_m": -CG_TO_REAR_AXLE_M,
    }
    params.update(track.bridge_params)
    params.update(lidar_parameters())
    if noise_on():
        params.update(NOISE)
        # Tuning only: L5_NOISE_JSON overrides single noise values, e.g. {"wheel_speed_scale": 1.0}.
        params.update(json.loads(os.environ.get("L5_NOISE_JSON", "{}")))
    return params


# -- Ground-truth map as a map_server yaml -------------------------------------------------------


def ground_truth_map_yaml(track: Track, label: str) -> Path:
    """Writes the bridge's own occupancy map (built by the same functions bridge_node uses) as a
    map_server pgm + yaml, so AMCL can localise on the ground-truth map through the real
    localization.launch.py and nav2_map_server path.

    SURFACES ONLY, the way slam_toolbox draws a map: wall cells that touch free floor are
    occupied, the inside of a solid wall is unknown. The bridge's corridor maps are solid blocks
    of wall, and AMCL's likelihood-field model (no ray casting) scores a beam that ends anywhere
    inside a solid block as a perfect hit, so on the solid map the filter slid 0.4 to 0.6 m
    towards the infield of gym_oval, where more beams end inside the block, and the car hit the
    wall (measured, the note's tuning table). A real map never has solid walls, so this is the
    honest ground truth for AMCL. Thin synthetic walls are all surface and do not change."""
    from racer_gym_bridge.bridge_node import build_track_from_raceline, build_track_from_synthetic
    from racer_gym_bridge.track_loader import build_synthetic_track

    params = track.bridge_params
    if "synthetic_track" in params:
        gym_track = build_track_from_synthetic(
            build_synthetic_track(
                params["synthetic_track"], corner_radius_m=params["synthetic_corner_radius_m"]
            )
        )
    else:
        gym_track = build_track_from_raceline(params["raceline_path"], params["track_half_width_m"])
    occupancy = np.asarray(gym_track.occupancy_map)
    out = scratch_dir(label)
    pgm = out / "ground_truth_map.pgm"
    free = occupancy > 127
    touches_free = np.zeros_like(free)
    touches_free[1:, :] |= free[:-1, :]
    touches_free[:-1, :] |= free[1:, :]
    touches_free[:, 1:] |= free[:, :-1]
    touches_free[:, :-1] |= free[:, 1:]
    # map_server trinary pixels: 254 free, 0 occupied, 205 unknown. Track arrays have row 0 at
    # min y; image row 0 is the top (max y).
    pixels = np.where(free, 254, np.where(touches_free, 0, 205)).astype(np.uint8)
    image = np.flipud(pixels)
    with pgm.open("wb") as f:
        f.write(f"P5\n{image.shape[1]} {image.shape[0]}\n255\n".encode())
        f.write(image.tobytes())
    origin = [float(v) for v in gym_track.spec.origin]
    yaml_path = out / "ground_truth_map.yaml"
    yaml_path.write_text(
        yaml.safe_dump(
            {
                "image": pgm.name,
                "mode": "trinary",
                "resolution": float(gym_track.spec.resolution),
                "origin": [origin[0], origin[1], 0.0],
                "negate": 0,
                "occupied_thresh": 0.65,
                "free_thresh": 0.25,
            }
        )
    )
    return yaml_path


def read_map_yaml(yaml_path: Path) -> tuple[np.ndarray, float, tuple[float, float]]:
    """A map_server yaml + binary pgm as (occupied bool grid with row 0 at min y, resolution,
    origin xy). Occupied = darker than occupied_thresh, the map_server rule."""
    meta = yaml.safe_load(Path(yaml_path).read_text())
    image_path = Path(yaml_path).parent / meta["image"]
    data = image_path.read_bytes()
    # Binary PGM: magic, width, height, maxval, separated by whitespace (comments allowed).
    tokens: list[bytes] = []
    pos = 0
    while len(tokens) < 4:
        while data[pos : pos + 1].isspace():
            pos += 1
        if data[pos : pos + 1] == b"#":
            pos = data.index(b"\n", pos) + 1
            continue
        start = pos
        while not data[pos : pos + 1].isspace():
            pos += 1
        tokens.append(data[start:pos])
    pos += 1
    if tokens[0] != b"P5":
        raise ValueError(f"{image_path}: not a binary PGM")
    width, height, maxval = int(tokens[1]), int(tokens[2]), int(tokens[3])
    pixels = np.frombuffer(data[pos : pos + width * height], dtype=np.uint8).reshape(height, width)
    occupied_probability = (maxval - pixels.astype(float)) / maxval
    if meta.get("negate", 0):
        occupied_probability = 1.0 - occupied_probability
    occupied = np.flipud(occupied_probability > float(meta["occupied_thresh"]))
    return occupied, float(meta["resolution"]), (float(meta["origin"][0]), float(meta["origin"][1]))


def occupied_cell_centres(occupied: np.ndarray, resolution: float, origin) -> np.ndarray:
    rows, cols = np.nonzero(occupied)
    return np.column_stack(
        [origin[0] + (cols + 0.5) * resolution, origin[1] + (rows + 0.5) * resolution]
    )


def boundary_cells(grid: OccupancyGrid) -> np.ndarray:
    """Centres (N, 2) of the occupied cells of an OccupancyGrid that touch a free cell: the wall
    surfaces a LiDAR can see (the solid inside of a corridor map's walls is not)."""
    data = np.asarray(grid.data, dtype=np.int16).reshape(grid.info.height, grid.info.width)
    occupied = data >= 50
    free = data == 0
    touches_free = np.zeros_like(occupied)
    touches_free[1:, :] |= free[:-1, :]
    touches_free[:-1, :] |= free[1:, :]
    touches_free[:, 1:] |= free[:, :-1]
    touches_free[:, :-1] |= free[:, 1:]
    origin = (grid.info.origin.position.x, grid.info.origin.position.y)
    return occupied_cell_centres(occupied & touches_free, grid.info.resolution, origin)


def nearest_distances(points: np.ndarray, reference: np.ndarray, chunk: int = 2000) -> np.ndarray:
    """Distance from each of points (N, 2) to the nearest of reference (M, 2). Brute force in
    chunks; maps here are a few thousand cells."""
    out = np.empty(len(points))
    for start in range(0, len(points), chunk):
        block = points[start : start + chunk]
        d2 = ((block[:, None, :] - reference[None, :, :]) ** 2).sum(axis=2)
        out[start : start + chunk] = np.sqrt(d2.min(axis=1))
    return out


def rigid_align(source: np.ndarray, target: np.ndarray) -> tuple[np.ndarray, np.ndarray]:
    """Least-squares 2D rotation R and translation t with R @ source + t ~ target (Kabsch)."""
    mu_s, mu_t = source.mean(axis=0), target.mean(axis=0)
    h = (source - mu_s).T @ (target - mu_t)
    u, _s, vt = np.linalg.svd(h)
    d = np.sign(np.linalg.det(vt.T @ u.T))
    rotation = vt.T @ np.diag([1.0, d]) @ u.T
    return rotation, mu_t - rotation @ mu_s


# -- Running laps and measuring the estimate -----------------------------------------------------


def yaw_of(q) -> float:
    return math.atan2(2.0 * (q.w * q.z + q.x * q.y), 1.0 - 2.0 * (q.y * q.y + q.z * q.z))


def wrap(angle: float) -> float:
    return math.atan2(math.sin(angle), math.cos(angle))


@dataclass
class RunResult:
    laps: int = 0
    lap_times_s: list[float] = field(default_factory=list)
    worst_lateral_m: float = 0.0
    max_speed_mps: float = 0.0
    # Pairs at ground-truth stamps: (gt_x, gt_y, gt_yaw, est_x, est_y, est_yaw, gt_speed).
    pairs: list[tuple[float, ...]] = field(default_factory=list)
    lookups_failed: int = 0
    # The EKF alone: (stamp_s, gt_x, gt_y, gt_yaw, odom_x, odom_y, odom_yaw), odom -> base_link.
    odom_pairs: list[tuple[float, ...]] = field(default_factory=list)

    def position_errors(self) -> np.ndarray:
        p = np.asarray(self.pairs)
        return np.hypot(p[:, 0] - p[:, 3], p[:, 1] - p[:, 4]) if len(p) else np.zeros(0)

    def heading_errors(self) -> np.ndarray:
        return np.asarray([abs(wrap(p[2] - p[5])) for p in self.pairs])

    def along_cross_errors(self) -> tuple[np.ndarray, np.ndarray]:
        """The position error split along the car's true heading (positive: the estimate is ahead
        of the car) and across it (positive: to its left)."""
        p = np.asarray(self.pairs)
        if not len(p):
            return np.zeros(0), np.zeros(0)
        dx, dy = p[:, 3] - p[:, 0], p[:, 4] - p[:, 1]
        c, s = np.cos(p[:, 2]), np.sin(p[:, 2])
        return c * dx + s * dy, -s * dx + c * dy

    def odom_drift(self, window_s: float = 1.0) -> tuple[np.ndarray, np.ndarray]:
        """How far the EKF's own odometry (odom -> base_link) goes wrong over window_s of driving:
        position error (m) and heading error (rad) of its relative motion against ground truth,
        expressed in the frame of the window's start. This is the dead reckoning AMCL corrects."""
        rows = self.odom_pairs
        pos, hdg = [], []
        j = 0
        for i in range(len(rows)):
            while j < len(rows) and rows[j][0] - rows[i][0] < window_s:
                j += 1
            if j >= len(rows):
                break
            a, b = rows[i], rows[j]
            gt_rel = compose_pose(invert_pose(a[1:4]), b[1:4])
            od_rel = compose_pose(invert_pose(a[4:7]), b[4:7])
            pos.append(math.hypot(gt_rel[0] - od_rel[0], gt_rel[1] - od_rel[1]))
            hdg.append(abs(wrap(gt_rel[2] - od_rel[2])))
        return np.asarray(pos), np.asarray(hdg)

    def summary(self, settle_s: int = 0) -> dict:
        pos = self.position_errors()[settle_s:]
        hdg = self.heading_errors()[settle_s:]
        along, cross = self.along_cross_errors()
        along, cross = along[settle_s:], cross[settle_s:]
        drift_pos, drift_hdg = self.odom_drift()
        return {
            "odom_drift_1s_p95_m": round(float(np.percentile(drift_pos, 95)), 4)
            if len(drift_pos)
            else None,
            "odom_yaw_drift_1s_p95_rad": round(float(np.percentile(drift_hdg, 95)), 4)
            if len(drift_hdg)
            else None,
            "along_mean_m": round(float(along.mean()), 4) if len(along) else None,
            "cross_abs_mean_m": round(float(np.abs(cross).mean()), 4) if len(cross) else None,
            "laps": self.laps,
            "lap_times_s": [round(t, 2) for t in self.lap_times_s],
            "worst_lateral_m": round(self.worst_lateral_m, 3),
            "max_speed_mps": round(self.max_speed_mps, 2),
            "samples": len(pos),
            "lookups_failed": self.lookups_failed,
            "pos_mean_m": round(float(pos.mean()), 4) if len(pos) else None,
            "pos_p95_m": round(float(np.percentile(pos, 95)), 4) if len(pos) else None,
            "pos_max_m": round(float(pos.max()), 4) if len(pos) else None,
            "hdg_mean_rad": round(float(hdg.mean()), 4) if len(hdg) else None,
            "hdg_p95_rad": round(float(np.percentile(hdg, 95)), 4) if len(hdg) else None,
            "hdg_max_rad": round(float(hdg.max()), 4) if len(hdg) else None,
        }


def run_laps(
    test,
    node,
    track: Track,
    *,
    target_laps: int,
    max_wall_s: float,
    stall_timeout_s: float = 10.0,
    sample_period_s: float = 0.05,
    lookup_delay_s: float = 0.3,
) -> RunResult:
    """Watches ground truth until target_laps laps are done, failing at once on wall contact,
    leaving the lane, or a stall. Every sample_period_s it also records the estimate: TF map ->
    base_link at that ground-truth sample's stamp, looked up lookup_delay_s later so the EKF and
    the filter have published past it."""
    tf_buffer = Buffer(cache_time=Duration(seconds=30.0))
    TransformListener(tf_buffer, node, spin_thread=False)
    gt: list[tuple[Time, float, float, float, float, float]] = []

    def on_gt(msg: Odometry) -> None:
        yaw = yaw_of(msg.pose.pose.orientation)
        x = msg.pose.pose.position.x - CG_TO_REAR_AXLE_M * math.cos(yaw)
        y = msg.pose.pose.position.y - CG_TO_REAR_AXLE_M * math.sin(yaw)
        speed = math.hypot(msg.twist.twist.linear.x, msg.twist.twist.linear.y)
        gt.append((Time.from_msg(msg.header.stamp), x, y, yaw, speed, time.monotonic()))

    sub = node.create_subscription(Odometry, "/sim/ground_truth_odom", on_gt, reliable_qos(50))
    result = RunResult()
    max_lateral_m = track.lane_half_width_m - CAR_HALF_WIDTH_M
    sim = sim_time_on()
    latest_stamp_s = 0.0
    deadline = time.monotonic() + max_wall_s
    consumed = 0
    prev_index = None
    unwrapped = 0.0
    best = 0.0
    last_progress_wall = time.monotonic()
    first_wall = None
    previous_yaw = None
    previous_stamp = None
    pending: list[tuple[Time, float, float, float, float, float]] = []
    next_sample_stamp = None

    def fail(message: str) -> None:
        """test.fail with the latest estimate attached, so a failure says whether the pose was
        wrong or the controller was."""
        context = ""
        if result.pairs:
            gx, gy, gyaw, ex, ey, eyaw, _speed = result.pairs[-1]
            context = (
                f"; last estimate ({ex:.2f}, {ey:.2f}, {eyaw:.2f}) vs truth ({gx:.2f}, "
                f"{gy:.2f}, {gyaw:.2f}), {len(result.pairs)} samples, "
                f"{result.lookups_failed} failed lookups"
            )
        test.fail(message + context)

    try:
        while time.monotonic() < deadline and result.laps < target_laps:
            rclpy.spin_once(node, timeout_sec=0.02)
            while consumed < len(gt):
                stamp, x, y, yaw, speed, wall = gt[consumed]
                consumed += 1
                latest_stamp_s = stamp.nanoseconds * 1e-9
                if sim:
                    # Progress, stalls and lap times are measured in simulated time: how long
                    # the runner took says nothing about the car.
                    wall = latest_stamp_s
                if first_wall is None:
                    first_wall = wall
                    last_progress_wall = wall
                if (
                    not sim
                    and previous_stamp is not None
                    and (stamp - previous_stamp).nanoseconds > 2e9
                ):
                    # The sim stamps with the wall clock. A jump (a VM resync, a sleeping host)
                    # wrecks every estimator's dt; that run measures the host, not the stack.
                    fail(
                        f"the ROS clock jumped {(stamp - previous_stamp).nanoseconds * 1e-9:.1f}s "
                        "between two ground-truth samples; the run is not valid, rerun it"
                    )
                previous_stamp = stamp
                if previous_yaw is not None and abs(wrap(yaw - previous_yaw)) > (
                    WALL_CONTACT_YAW_JUMP_RAD
                ):
                    fail(
                        f"wall contact at ({x:.2f}, {y:.2f}): heading jumped "
                        f"{math.degrees(previous_yaw):.0f} -> {math.degrees(yaw):.0f} deg in one "
                        "step (f1tenth_gym zeroes the yaw on collision)"
                    )
                previous_yaw = yaw
                # Lane and progress are judged on the CG (the loop is the gym's reference line).
                cx = x + CG_TO_REAR_AXLE_M * math.cos(yaw)
                cy = y + CG_TO_REAR_AXLE_M * math.sin(yaw)
                s, prev_index, lateral = track.nearest(cx, cy, prev_index)
                result.worst_lateral_m = max(result.worst_lateral_m, lateral)
                result.max_speed_mps = max(result.max_speed_mps, speed)
                if lateral > max_lateral_m:
                    fail(
                        f"left the lane: CG {lateral:.3f} m from the centreline at ({cx:.2f}, "
                        f"{cy:.2f}), limit {max_lateral_m:.3f} m"
                    )
                if consumed == 1:
                    unwrapped = s
                    start_s = s
                else:
                    delta = s - (unwrapped % track.length_m)
                    if delta < -track.length_m / 2.0:
                        delta += track.length_m
                    elif delta > track.length_m / 2.0:
                        delta -= track.length_m
                    unwrapped += delta
                progress = unwrapped - start_s
                if progress > best + 0.05:
                    best = progress
                    last_progress_wall = wall
                if wall - last_progress_wall > stall_timeout_s:
                    fail(f"no progress for {stall_timeout_s}s at ({x:.2f}, {y:.2f})")
                laps = int(progress // track.length_m)
                while len(result.lap_times_s) < laps:
                    result.lap_times_s.append(wall - first_wall)
                result.laps = len(result.lap_times_s)
                if next_sample_stamp is None or stamp >= next_sample_stamp:
                    pending.append(gt[consumed - 1])
                    next_sample_stamp = stamp + Duration(seconds=sample_period_s)
            now_wall = latest_stamp_s if sim else time.monotonic()
            while (
                pending
                and now_wall - (pending[0][0].nanoseconds * 1e-9 if sim else pending[0][5])
                > lookup_delay_s
            ):
                stamp, x, y, yaw, speed, _wall = pending.pop(0)
                try:
                    tf = tf_buffer.lookup_transform("map", "base_link", stamp)
                except TransformException:
                    result.lookups_failed += 1
                    continue
                t = tf.transform
                result.pairs.append(
                    (x, y, yaw, t.translation.x, t.translation.y, yaw_of(t.rotation), speed)
                )
                try:
                    o = tf_buffer.lookup_transform("odom", "base_link", stamp).transform
                    result.odom_pairs.append(
                        (
                            stamp.nanoseconds * 1e-9,
                            x,
                            y,
                            yaw,
                            o.translation.x,
                            o.translation.y,
                            yaw_of(o.rotation),
                        )
                    )
                except TransformException:
                    pass
    finally:
        node.destroy_subscription(sub)
    return result


def record_results(label: str, payload: dict) -> None:
    """Prints the run's numbers (they land in the CI log) and appends them to L5_RESULTS."""
    tag = os.environ.get("L5_TAG")
    line = json.dumps(
        {"scenario": label, **({"tag": tag} if tag else {}), **payload}, sort_keys=True
    )
    print(f"[l5_localization] {line}", flush=True)
    path = os.environ.get("L5_RESULTS")
    if path:
        with open(path, "a") as f:
            f.write(line + "\n")


def wait_for_tf(test, node, parent: str, child: str, timeout_s: float = 60.0) -> None:
    tf_buffer = Buffer()
    TransformListener(tf_buffer, node, spin_thread=False)
    deadline = time.monotonic() + timeout_s
    while time.monotonic() < deadline:
        rclpy.spin_once(node, timeout_sec=0.05)
        if tf_buffer.can_transform(parent, child, Time()):
            return
    test.fail(f"no {parent} -> {child} transform within {timeout_s}s")


def first_ground_truth_pose(test, node, timeout_s: float = 30.0) -> tuple[float, float, float]:
    """base_link's ground-truth pose (rear axle, map frame of the sim) from /sim/ground_truth_odom."""
    received = []
    sub = node.create_subscription(
        Odometry, "/sim/ground_truth_odom", received.append, reliable_qos()
    )
    deadline = time.monotonic() + timeout_s
    while not received and time.monotonic() < deadline:
        rclpy.spin_once(node, timeout_sec=0.05)
    node.destroy_subscription(sub)
    test.assertTrue(received, "no /sim/ground_truth_odom")
    msg = received[-1]
    yaw = yaw_of(msg.pose.pose.orientation)
    return (
        msg.pose.pose.position.x - CG_TO_REAR_AXLE_M * math.cos(yaw),
        msg.pose.pose.position.y - CG_TO_REAR_AXLE_M * math.sin(yaw),
        yaw,
    )


def set_initial_pose_from_ground_truth(test, node, gt_from_map, timeout_s: float = 60.0) -> None:
    """Publishes the car's true pose (in the map's frame) on /initialpose until AMCL answers with
    map -> odom. tracker_node cannot move before that (no map -> odom, no /drive_raw), so the car
    is still on its start pose."""
    from geometry_msgs.msg import PoseWithCovarianceStamped

    start = first_ground_truth_pose(test, node)
    pose = compose_pose(invert_pose(gt_from_map), start) if gt_from_map is not None else start
    pub = node.create_publisher(PoseWithCovarianceStamped, "/initialpose", reliable_qos(1))
    tf_buffer = Buffer()
    TransformListener(tf_buffer, node, spin_thread=False)
    deadline = time.monotonic() + timeout_s
    next_publish = 0.0
    while time.monotonic() < deadline:
        if time.monotonic() >= next_publish:
            msg = PoseWithCovarianceStamped()
            msg.header.frame_id = "map"
            msg.header.stamp = node.get_clock().now().to_msg()
            msg.pose.pose.position.x = pose[0]
            msg.pose.pose.position.y = pose[1]
            msg.pose.pose.orientation.z = math.sin(pose[2] / 2.0)
            msg.pose.pose.orientation.w = math.cos(pose[2] / 2.0)
            # 5 cm and 0.05 rad: what an operator clicking a pose in Foxglove gets close to.
            msg.pose.covariance[0] = msg.pose.covariance[7] = 0.05**2
            msg.pose.covariance[35] = 0.05**2
            pub.publish(msg)
            next_publish = time.monotonic() + 1.0
        rclpy.spin_once(node, timeout_sec=0.05)
        if tf_buffer.can_transform("map", "base_link", Time()):
            node.destroy_publisher(pub)
            return
    test.fail(f"AMCL published no map -> odom within {timeout_s}s of /initialpose")


def receive_latched(test, node, msg_type, topic: str, timeout_s: float = 30.0):
    received = []
    sub = node.create_subscription(msg_type, topic, received.append, latched_qos())
    deadline = time.monotonic() + timeout_s
    while not received and time.monotonic() < deadline:
        rclpy.spin_once(node, timeout_sec=0.05)
    node.destroy_subscription(sub)
    test.assertTrue(received, f"nothing on {topic}")
    return received[-1]


# -- Scenarios -----------------------------------------------------------------------------------


def compose_pose(a: tuple[float, float, float], b: tuple[float, float, float]):
    """a composed with b, planar poses (x, y, yaw)."""
    c, s = math.cos(a[2]), math.sin(a[2])
    return (a[0] + c * b[0] - s * b[1], a[1] + s * b[0] + c * b[1], wrap(a[2] + b[2]))


def invert_pose(a: tuple[float, float, float]):
    c, s = math.cos(a[2]), math.sin(a[2])
    return (-(c * a[0] + s * a[1]), s * a[0] - c * a[1], -a[2])


TRACKER_START_DELAY_S = 8.0


def launch_actions(track: Track, mode: str, map_yaml: Path | None, seed: int):
    """The launch actions of one scenario: bridge, racer_bringup's mapping or localization launch,
    and tracker_node.

    mode "mapping": tracker_node follows GROUND TRUTH (odom_topic /sim/ground_truth_odom) and
    starts TRACKER_START_DELAY_S after everything else, so the EKF and slam_toolbox are up while
    the car is still on its start pose: slam_toolbox's map frame is then the start pose, as on
    the car when mapping starts with the car standing still.
    mode "localization": tracker_node follows the EKF's /odom composed with AMCL's map -> odom
    (pose_frame map), on the raceline written in the map's frame. It cannot move before AMCL
    publishes map -> odom (tracker_node stops on a missing transform).
    """
    from launch.actions import IncludeLaunchDescription, TimerAction
    from launch.launch_description_sources import PythonLaunchDescriptionSource
    from launch_ros.actions import Node as LaunchNode

    bridge_params = bridge_parameters(track, seed)
    if sim_time_on():
        bridge_params.update(
            {
                "sim_time": True,
                "lockstep_ack": LOCKSTEP_ACK,
                "lockstep_max_lag_s": LOCKSTEP_MAX_LAG_S,
            }
        )
    bridge = LaunchNode(
        package="racer_gym_bridge",
        executable="bridge_node",
        name="bridge_node",
        parameters=[bridge_params],
    )
    # The sim's car steers exactly as commanded, so wheel_twist_node must not apply the real
    # car's measured yaw rate gain table (vehicle_params steering.yaw_rate_gain_table).
    overrides = {"yaw_gain_override": "1.0"}
    if sim_time_on():
        overrides["use_sim_time"] = "true"
    if os.environ.get("L5_EKF_PARAMS"):
        overrides["ekf_params_file"] = os.environ["L5_EKF_PARAMS"]
    if mode == "mapping":
        if os.environ.get("L5_SLAM_PARAMS"):
            overrides["slam_params_file"] = os.environ["L5_SLAM_PARAMS"]
        stack = IncludeLaunchDescription(
            PythonLaunchDescriptionSource(str(BRINGUP_LAUNCH_DIR / "mapping.launch.py")),
            launch_arguments=overrides.items(),
        )
        tracker_params = {
            "raceline_path": str(track.raceline_csv),
            "odom_topic": "/sim/ground_truth_odom",
        }
    else:
        if os.environ.get("L5_AMCL_PARAMS"):
            overrides["amcl_params_file"] = os.environ["L5_AMCL_PARAMS"]
        # No initial pose at launch: f1tenth_gym resets the car onto a seeded random point of
        # the first metre of the line, so the test sets it from ground truth on /initialpose
        # (set_initial_pose_from_ground_truth), the way an operator sets it from Foxglove.
        overrides.update({"map": str(map_yaml), "set_initial_pose": "false"})
        stack = IncludeLaunchDescription(
            PythonLaunchDescriptionSource(str(BRINGUP_LAUNCH_DIR / "localization.launch.py")),
            launch_arguments=overrides.items(),
        )
        tracker_params = {
            "raceline_path": str(track.raceline_csv),
            "odom_topic": "/odom",
            "pose_frame": "map",
        }
        if os.environ.get("L5_TRACKER_ON_GROUND_TRUTH") == "1":
            # Tuning only: separates controller failures from localisation ones.
            tracker_params = {
                "raceline_path": str(track.raceline_csv),
                "odom_topic": "/sim/ground_truth_odom",
            }
    tracker_params.update(track.tracker_overrides)
    if sim_time_on():
        tracker_params["use_sim_time"] = True
        tracker_params["ros_clock_timers"] = True
    tracker = LaunchNode(
        package="racer_control",
        executable="tracker_node",
        name="tracker_node",
        parameters=[tracker_params],
        # SIM-ONLY shim, see the module docstring.
        remappings=[("/drive_raw", "/drive")],
    )
    extra = []
    if os.environ.get("L5_BAG"):
        # Tuning only: record what the estimators saw, for offline analysis.
        from launch.actions import ExecuteProcess

        extra.append(
            ExecuteProcess(
                cmd=["ros2", "bag", "record", "-o", os.environ["L5_BAG"], "-a"],
                output="log",
            )
        )
    if mode == "mapping":
        return [bridge, stack, TimerAction(period=TRACKER_START_DELAY_S, actions=[tracker])] + extra
    return [bridge, stack, tracker] + extra


def map_frame_raceline(track: Track, map_from_gt) -> None:
    """Rewrites the track's raceline in a map frame that is not the ground-truth one (a SLAM map
    whose origin is the start pose): the car's raceline has to be in its map's frame."""
    xs, ys, vs = _load_raceline(track.raceline_csv)
    moved = [compose_pose(map_from_gt, (x, y, 0.0)) for x, y in zip(xs, ys, strict=True)]
    write_raceline_csv(track.raceline_csv, [p[0] for p in moved], [p[1] for p in moved], vs)


def to_ground_truth_frame(result: RunResult, gt_from_map) -> None:
    """Moves the estimates of a run from a SLAM map frame into the ground-truth frame."""
    moved = []
    for gx, gy, gyaw, ex, ey, eyaw, speed in result.pairs:
        x, y, yaw = compose_pose(gt_from_map, (ex, ey, eyaw))
        moved.append((gx, gy, gyaw, x, y, yaw, speed))
    result.pairs = moved


def save_slam_map(test, node, path_stem: Path, timeout_s: float = 60.0, attempts: int = 3) -> Path:
    """Calls /slam_toolbox/save_map, the service the car's runbook uses, and waits for the yaml.

    save_map answered 255 (undefined failure) in one CI run (PR 116, 2026-11-11), intermittently.
    The service builds a nav2 map_saver INSIDE slam_toolbox that subscribes to /map and gives up
    after 2 s without a message, while slam_toolbox publishes /map from its visualisation thread
    once a second and ONLY while someone is subscribed (updateMap returns early with no
    subscriber). So the first subscriber in the process's life, the map_saver, waits for the
    publish cycle (up to 1 s plus discovery plus the time updateMap takes) inside its 2 s: on a
    starved runner that loses. This keeps a subscription of its own on /map from before the call,
    so slam_toolbox is already publishing (and /map, a latched topic, already holds the latest map)
    when the map_saver arrives, waits until a map has been seen, and retries a failed call, up to
    `attempts` times, one second apart: the retry is bounded and a map_saver that fails every time
    still fails the test."""
    from slam_toolbox.srv import SaveMap
    from std_msgs.msg import String

    seen: list = []
    keepalive = node.create_subscription(OccupancyGrid, "/map", seen.append, latched_qos())
    deadline = time.monotonic() + timeout_s
    while not seen and time.monotonic() < deadline:
        rclpy.spin_once(node, timeout_sec=0.1)
    test.assertTrue(seen, "slam_toolbox published no /map")
    client = node.create_client(SaveMap, "/slam_toolbox/save_map")
    test.assertTrue(client.wait_for_service(timeout_sec=30.0), "/slam_toolbox/save_map missing")
    yaml_path = path_stem.with_suffix(".yaml")
    result = None
    for attempt in range(attempts):
        if yaml_path.exists():
            yaml_path.unlink()
        future = client.call_async(SaveMap.Request(name=String(data=str(path_stem))))
        rclpy.spin_until_future_complete(node, future, timeout_sec=timeout_s)
        test.assertIsNotNone(future.result(), "save_map did not answer")
        result = future.result().result
        if result == SaveMap.Response.RESULT_SUCCESS:
            break
        print(f"[l5_localization] save_map attempt {attempt + 1} answered {result}", flush=True)
        end = time.monotonic() + 1.0
        while time.monotonic() < end:
            rclpy.spin_once(node, timeout_sec=0.1)
    test.assertEqual(result, SaveMap.Response.RESULT_SUCCESS, "save_map failed")
    deadline = time.monotonic() + timeout_s
    while not yaml_path.exists() and time.monotonic() < deadline:
        rclpy.spin_once(node, timeout_sec=0.1)
    test.assertTrue(yaml_path.exists(), f"save_map wrote no {yaml_path}")
    time.sleep(1.0)  # map_saver writes the pgm and yaml back to back
    node.destroy_subscription(keepalive)
    return yaml_path


def compare_maps(slam_yaml: Path, truth: OccupancyGrid, gt_from_map, tolerance_m: float) -> dict:
    """Wall agreement of a saved SLAM map with the bridge's /sim/map.

    The SLAM map's occupied cells are moved into the ground-truth frame by gt_from_map (the start
    pose, no fitting: any drift slam_toolbox has shows up). Then:
      * wall error: distance from each SLAM occupied cell to the nearest true wall surface cell
        (mean, p95, max), and the fraction further than tolerance_m (spurious);
      * coverage: fraction of true wall surface cells with a SLAM occupied cell within
        tolerance_m. Thin synthetic walls have a back face the car never sees, but it is within
        one wall thickness (0.08 m) of the front face, inside the tolerance.
    """
    occupied, resolution, origin = read_map_yaml(slam_yaml)
    slam_cells = occupied_cell_centres(occupied, resolution, origin)
    c, s = math.cos(gt_from_map[2]), math.sin(gt_from_map[2])
    rotation = np.array([[c, -s], [s, c]])
    slam_in_gt = slam_cells @ rotation.T + np.array(gt_from_map[:2])
    truth_cells = boundary_cells(truth)
    to_truth = nearest_distances(slam_in_gt, truth_cells)
    to_slam = nearest_distances(truth_cells, slam_in_gt)
    return {
        "slam_occupied_cells": len(slam_cells),
        "truth_wall_cells": len(truth_cells),
        "wall_err_mean_m": round(float(to_truth.mean()), 4),
        "wall_err_p95_m": round(float(np.percentile(to_truth, 95)), 4),
        "wall_err_max_m": round(float(to_truth.max()), 4),
        "spurious_fraction": round(float((to_truth > tolerance_m).mean()), 4),
        "coverage": round(float((to_slam <= tolerance_m).mean()), 4),
    }


def start_pose_sidecar(map_yaml: Path) -> Path:
    """Where a mapping run writes the ground-truth pose of its map frame (the start pose)."""
    return Path(map_yaml).with_suffix(".start.json")


# -- The two canary bodies -----------------------------------------------------------------------


@dataclass
class Thresholds:
    cross_abs_mean_m: float
    pos_mean_m: float
    pos_p95_m: float
    pos_max_m: float
    hdg_p95_rad: float
    hdg_max_rad: float


def localization_setup(track: Track, label: str):
    """(map yaml, gt_from_map) for a localisation scenario, at test-description time.

    Default: the bridge's own map, whose frame is the sim's (gt_from_map None). L5_MAP_YAML
    (tuning): a saved slam_toolbox map, whose frame is the mapping run's start pose (read from the
    sidecar the mapping canary writes); the raceline is rewritten into that frame, as it would
    have to be on the car."""
    if os.environ.get("L5_MAP_YAML"):
        map_yaml = Path(os.environ["L5_MAP_YAML"])
        gt_from_map = tuple(json.loads(start_pose_sidecar(map_yaml).read_text())["gt_from_map"])
        map_frame_raceline(track, invert_pose(gt_from_map))
        return map_yaml, gt_from_map
    return ground_truth_map_yaml(track, label), None


def run_localization_canary(
    test,
    label: str,
    track: Track,
    map_yaml: Path,
    gt_from_map,
    *,
    laps: int,
    max_wall_s: float,
    thresholds: Thresholds,
    extra: dict,
) -> None:
    node = create_node(f"l5_{label}")
    try:
        set_initial_pose_from_ground_truth(test, node, gt_from_map)
        result = run_laps(test, node, track, target_laps=laps, max_wall_s=max_wall_s)
        if gt_from_map is not None:
            to_ground_truth_frame(result, gt_from_map)
        summary = result.summary()
        record_results(label, {**summary, **extra, "noise": noise_on(), "map": str(map_yaml)})
        test.assertGreaterEqual(result.laps, laps)
        test.assertGreater(summary["samples"], 100)
        test.assertLessEqual(summary["cross_abs_mean_m"], thresholds.cross_abs_mean_m)
        test.assertLessEqual(summary["pos_mean_m"], thresholds.pos_mean_m)
        test.assertLessEqual(summary["pos_p95_m"], thresholds.pos_p95_m)
        test.assertLessEqual(summary["pos_max_m"], thresholds.pos_max_m)
        test.assertLessEqual(summary["hdg_p95_rad"], thresholds.hdg_p95_rad)
        test.assertLessEqual(summary["hdg_max_rad"], thresholds.hdg_max_rad)
    finally:
        node.destroy_node()


def assert_shutdown_exit_codes(test, proc_info, extra_allowed: dict[str, list[int]] | None = None):
    """Post-shutdown check: every process exited 0 or on the SIGINT/SIGTERM launch_testing sent,
    with ONE documented exception, ekf_node (robot_localization) may have died of SIGSEGV.

    GitHub issue 172: ekf_node exits -11 in the SIGINT window at the end of the run, after every
    lap was driven and judged (the test body has passed by then: a crash DURING the run would end
    the pose stream and fail the laps or the map). The EKF shares one SIGINT with every other
    process of the stack at once; the crash is inside robot_localization / Fast-DDS teardown (the
    same class of shutdown race as the slam_toolbox abort handled in test_mapping_canary.py), not
    in a racer_* node. Only ekf_node and only SIGSEGV is tolerated: any other process, and any
    other code from the EKF, still fails. `extra_allowed` adds per-process codes (process name
    substring -> codes)."""
    import signal

    allowed_extra = {"ekf_node": [-signal.SIGSEGV], **(extra_allowed or {})}
    for info in proc_info:
        allowed = [0, -signal.SIGINT, -signal.SIGTERM]
        for name, codes in allowed_extra.items():
            if name in info.process_name:
                allowed += codes
        test.assertIn(
            info.returncode, allowed, f"{info.process_name} exited with {info.returncode}"
        )
