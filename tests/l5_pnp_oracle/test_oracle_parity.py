"""The oracle and the perception node are compared on input quality only (brief 40a, item 1): on
the same scans they publish the same message type on the same topic with the same stamps, one per
scan, in the same frame and with the same station layout, and a message arrives at the same time
(both go out in the scan's callback; the oracle waits only for the odometry of that step).

Real nodes in subprocesses (track_perception_node with the geometric backend, oracle_track_node by
path), a test publisher for /scan and /sim/ground_truth_odom, both listening, told apart by
`model_id`. Skipped without a sourced ROS workspace.
"""

from __future__ import annotations

import math
import os
import subprocess
import sys
import time
from pathlib import Path

import pytest

rclpy = pytest.importorskip("rclpy")
pytest.importorskip("racer_pnp_msgs")
pytest.importorskip("racer_tools")

import numpy as np
import yaml

HERE = Path(__file__).resolve().parent
REPO_ROOT = HERE.parents[1]
sys.path.insert(0, str(HERE))
sys.path.insert(0, str(REPO_ROOT / "sim" / "bridge" / "racer_gym_bridge"))
import oracle_core as oc
from racer_gym_bridge.track_loader import build_synthetic_track
from racer_gym_bridge.track_obstacles import ground_truth_grid

N_SCANS = 30
SCAN_PERIOD_S = 0.1
ODOM_PERIOD_S = 0.01
#: Both messages of a scan arrive within this of each other (s): well under the scan period.
MAX_ARRIVAL_SPREAD_S = 0.05


def executable(package: str, name: str) -> str:
    from ament_index_python.packages import get_package_prefix

    return str(Path(get_package_prefix(package)) / "lib" / package / name)


def start(cmd: list[str], params: Path, log: Path) -> subprocess.Popen:
    return subprocess.Popen(
        [*cmd, "--ros-args", "--params-file", str(params)],
        stdout=log.open("w"),
        stderr=subprocess.STDOUT,
        env=os.environ.copy(),
    )


@pytest.mark.parametrize("scan_first", [False, True], ids=["odom_first", "scan_first"])
def test_same_scans_same_stamps_same_timing(tmp_path, scan_first):
    from nav_msgs.msg import Odometry
    from racer_pnp_msgs.msg import LocalTrack
    from rclpy.qos import HistoryPolicy, QoSProfile, ReliabilityPolicy
    from sensor_msgs.msg import LaserScan

    os.environ["ROS_DOMAIN_ID"] = str(97 if scan_first else 96)
    track = build_synthetic_track(
        "stadium", lane_width_m=1.0, straight_length_m=0.0, island_radius_m=0.5
    )
    grid, origin, res = ground_truth_grid(track)
    truth = tmp_path / "gt.npz"
    oc.write_ground_truth(truth, track.centreline_x, track.centreline_y, grid, origin, res)
    params = tmp_path / "params.yaml"
    params.write_text(
        yaml.safe_dump(
            {
                "track_perception_node": {
                    "ros__parameters": {
                        "backend": "geometric",
                        "laser_yaw_from_vehicle_params": True,
                    }
                },
                "oracle_track_node": {"ros__parameters": {"ground_truth_track": str(truth)}},
            }
        )
    )
    procs = [
        start(
            [executable("racer_track_perception", "track_perception_node")],
            params,
            tmp_path / "p.log",
        ),
        start([sys.executable, str(HERE / "oracle_track_node.py")], params, tmp_path / "o.log"),
    ]
    rclpy.init()
    node = rclpy.create_node("oracle_parity_test")
    try:
        arrivals: dict[str, dict[tuple[int, int], float]] = {"geometric-v1": {}, "oracle": {}}
        shapes: dict[str, set] = {"geometric-v1": set(), "oracle": set()}
        reliable = QoSProfile(
            reliability=ReliabilityPolicy.RELIABLE, history=HistoryPolicy.KEEP_LAST, depth=200
        )

        def on_track(msg) -> None:
            key = "oracle" if msg.model_id == oc.MODEL_ID else "geometric-v1"
            stamp = (msg.header.stamp.sec, msg.header.stamp.nanosec)
            arrivals[key][stamp] = time.monotonic()
            shapes[key].add(
                (
                    msg.header.frame_id,
                    round(msg.station_spacing_m, 6),
                    len(msg.x_m),
                    len(msg.y_m),
                    len(msg.half_width_left_m),
                    len(msg.half_width_right_m),
                    len(msg.sigma_m),
                )
            )

        node.create_subscription(LocalTrack, "/pnp/local_track", on_track, reliable)
        best_effort = QoSProfile(
            reliability=ReliabilityPolicy.BEST_EFFORT, history=HistoryPolicy.KEEP_LAST, depth=10
        )
        scan_pub = node.create_publisher(LaserScan, "/scan", best_effort)
        odom_pub = node.create_publisher(Odometry, "/sim/ground_truth_odom", reliable)

        # wait for the two nodes to be up and matched
        deadline = time.monotonic() + 60.0
        while time.monotonic() < deadline:
            rclpy.spin_once(node, timeout_sec=0.1)
            if scan_pub.get_subscription_count() >= 2 and odom_pub.get_subscription_count() >= 1:
                break
        else:
            pytest.fail("the nodes did not come up: " + (tmp_path / "o.log").read_text()[-800:])
        time.sleep(1.0)

        base = node.get_clock().now().nanoseconds * 1e-9
        stamp_of = lambda t: (int(t), round((t - int(t)) * 1e9))
        scans = []
        x0, y0 = track.centreline_x[0], track.centreline_y[0]
        n_beams = 720
        angle_inc = 2.0 * math.pi / n_beams
        ranges = np.full(n_beams, 1.5, dtype=np.float32).tolist()
        for k in range(N_SCANS):
            t_scan = base + k * SCAN_PERIOD_S
            sec, nsec = stamp_of(t_scan)
            scan = LaserScan()
            scan.header.stamp.sec, scan.header.stamp.nanosec = sec, nsec
            scan.header.frame_id = "laser"
            scan.angle_min = -math.pi
            scan.angle_increment = angle_inc
            scan.angle_max = math.pi - angle_inc
            scan.range_min = 0.05
            scan.range_max = 10.0
            scan.ranges = ranges
            scans.append((sec, nsec))

            # odometry of this scan's step and the ones around it
            def publish_odom(t: float) -> None:
                o = Odometry()
                o.header.stamp.sec, o.header.stamp.nanosec = stamp_of(t)
                o.pose.pose.position.x = float(x0 + 0.5 * (t - base))
                o.pose.pose.position.y = float(y0)
                o.pose.pose.orientation.w = 1.0
                odom_pub.publish(o)

            first = [t_scan + j * ODOM_PERIOD_S for j in range(-9, 1)]
            if scan_first:
                scan_pub.publish(scan)
                for t in first:
                    publish_odom(t)
            else:
                for t in first:
                    publish_odom(t)
                scan_pub.publish(scan)
            end = time.monotonic() + SCAN_PERIOD_S
            while time.monotonic() < end:
                rclpy.spin_once(node, timeout_sec=0.005)
        end = time.monotonic() + 3.0
        while time.monotonic() < end:
            rclpy.spin_once(node, timeout_sec=0.05)
    finally:
        node.destroy_node()
        rclpy.try_shutdown()
        for p in procs:
            p.terminate()
        for p in procs:
            p.wait(timeout=15)

    expected = set(scans)
    assert set(arrivals["geometric-v1"]) == expected, "the perception node missed scans"
    assert set(arrivals["oracle"]) == expected, "the oracle must publish one message per scan"
    assert shapes["oracle"] == shapes["geometric-v1"], (
        "same frame, spacing and array lengths: " + str(shapes)
    )
    spread = [abs(arrivals["oracle"][s] - arrivals["geometric-v1"][s]) for s in scans]
    assert max(spread) < MAX_ARRIVAL_SPREAD_S, f"arrival spread {max(spread):.3f} s"
