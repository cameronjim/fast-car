#!/usr/bin/env python3
"""Deterministic offline mapping replay: a recorded run into sync_slam_toolbox_node.

    replay_sync.py NPZ OUT_PREFIX [--old-bag] [--deskew end_rev] [--yaw-scale K] ...

Unlike bag_feeder.py (the live stack on a wall-clock replay: async slam_toolbox, the EKF,
nondeterministic) this feeds scans and an odom -> base_link dead-reckoned from the wheel twist
(mapping_eval/odometry.py, the same kinematics as wheel_twist_node) on a /clock the script drives,
into the SYNCHRONOUS node, which queues every scan and drops none. The same inputs give the same
map, so two slam_toolbox parameter files can be compared on one bag. Start slam_toolbox yourself
(see tools/track_session/replay_sync.sh) with use_sim_time and the parameter file under test.

Writes OUT_PREFIX.yaml / .pgm (SaveMap service) and OUT_PREFIX.tf.npz: the map -> odom transform
as slam_toolbox published it (t, x, y, yaw), whose jumps are the loop closures. Idle stretches
(the car parked) are cut, with 3 s kept either side.

Run inside the ros-dev container with the workspace sourced; tools/ must be importable.
"""

from __future__ import annotations

import argparse
import math
import sys
import time
from pathlib import Path

import numpy as np
import rclpy
from geometry_msgs.msg import TransformStamped
from nav_msgs.msg import OccupancyGrid
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, QoSProfile, ReliabilityPolicy
from rosgraph_msgs.msg import Clock
from sensor_msgs.msg import LaserScan
from slam_toolbox.srv import SaveMap
from tf2_msgs.msg import TFMessage
from tf2_ros import StaticTransformBroadcaster

sys.path.insert(0, str(Path(__file__).resolve().parents[1]))

from mapping_eval.deskew import deskew_scan
from mapping_eval.odometry import (
    OLD_BAGS,
    BagScale,
    YawModel,
    integrate_pose,
    load_vehicle,
    wheel_twist_series,
)


def sec(t: float) -> tuple[int, int]:
    s = math.floor(t)
    return int(s), int((t - s) * 1e9)


def main() -> int:
    p = argparse.ArgumentParser(description=__doc__)
    p.add_argument("npz", type=Path)
    p.add_argument("out_prefix")
    p.add_argument(
        "--old-bag", action="store_true", help="2026-11-10 bag: put it on the calibrated scale"
    )
    p.add_argument(
        "--deskew", default="none", choices=("none", "end_rev", "end_fwd", "start_rev", "start_fwd")
    )
    p.add_argument("--yaw-scale", type=float, default=1.0)
    p.add_argument(
        "--steer-gain",
        type=float,
        default=None,
        help="default: vehicle_params yaw_rate_gain_table; 1.0 = the model before 2026-11-12",
    )
    p.add_argument("--ground-ratio0", type=float, default=1.0)
    p.add_argument("--ground-ratio-slope", type=float, default=0.0)
    p.add_argument("--pace", type=float, default=0.05, help="wall seconds per scan")
    p.add_argument("--limit-scans", type=int, default=0)
    args = p.parse_args()

    data = np.load(args.npz)
    vehicle = load_vehicle()
    model = YawModel(args.steer_gain, args.ground_ratio0, args.ground_ratio_slope, args.yaw_scale)
    tw = wheel_twist_series(
        data["odom_t"],
        data["odom_v"],
        data["drive_t"],
        data["drive_steer"],
        vehicle,
        OLD_BAGS if args.old_bag else BagScale(),
        model,
        clamp_to_vehicle_limits=not args.old_bag,
    )
    ox, oy, oyaw = integrate_pose(tw["t"], tw["v"], tw["wz"])
    st = data["scan_t"]
    ranges = data["scan_ranges"]
    amin, ainc, rmin, rmax = (float(x) for x in data["scan_meta"])
    v_at = np.interp(st, tw["t"], tw["v"])
    moving = np.abs(v_at) > 0.1
    keep = np.zeros(len(st), bool)
    for i in np.nonzero(moving)[0]:
        keep[max(0, i - 30) : i + 30] = True
    sel = np.nonzero(keep)[0]
    if args.limit_scans:
        sel = sel[: args.limit_scans]
    print(f"{len(sel)} scans of {len(st)}", flush=True)

    rclpy.init()
    node = Node("replay_sync")
    clock = node.create_publisher(Clock, "/clock", 10)
    scan_pub = node.create_publisher(
        LaserScan, "/scan", QoSProfile(depth=10, reliability=ReliabilityPolicy.BEST_EFFORT)
    )
    tf_pub = node.create_publisher(TFMessage, "/tf", 100)
    static = StaticTransformBroadcaster(node)
    mx, my, myaw = vehicle.mount_xy_yaw
    ts = TransformStamped()
    ts.header.frame_id, ts.child_frame_id = "base_link", "laser"
    ts.transform.translation.x, ts.transform.translation.y = mx, my
    ts.transform.rotation.z, ts.transform.rotation.w = math.sin(myaw / 2), math.cos(myaw / 2)
    static.sendTransform(ts)
    map_odom: list[tuple[float, float, float, float]] = []

    def on_tf(msg: TFMessage) -> None:
        for tr in msg.transforms:
            if tr.header.frame_id == "map" and tr.child_frame_id == "odom":
                q = tr.transform.rotation
                map_odom.append(
                    (
                        tr.header.stamp.sec + tr.header.stamp.nanosec * 1e-9,
                        tr.transform.translation.x,
                        tr.transform.translation.y,
                        2.0 * math.atan2(q.z, q.w),
                    )
                )

    node.create_subscription(TFMessage, "/tf", on_tf, 200)
    map_hash = [None]

    def on_map(msg: OccupancyGrid) -> None:
        map_hash[0] = hash(bytes(np.asarray(msg.data, dtype=np.int8)))

    node.create_subscription(
        OccupancyGrid,
        "/map",
        on_map,
        QoSProfile(
            depth=1,
            durability=DurabilityPolicy.TRANSIENT_LOCAL,
            reliability=ReliabilityPolicy.RELIABLE,
        ),
    )

    def pub_clock(t: float) -> None:
        c = Clock()
        c.clock.sec, c.clock.nanosec = sec(t)
        clock.publish(c)

    def pub_tf(t: float) -> None:
        m = TransformStamped()
        m.header.stamp.sec, m.header.stamp.nanosec = sec(t)
        m.header.frame_id, m.child_frame_id = "odom", "base_link"
        m.transform.translation.x = float(np.interp(t, tw["t"], ox))
        m.transform.translation.y = float(np.interp(t, tw["t"], oy))
        yaw = float(np.interp(t, tw["t"], oyaw))
        m.transform.rotation.z, m.transform.rotation.w = math.sin(yaw / 2), math.cos(yaw / 2)
        tf_pub.publish(TFMessage(transforms=[m]))

    t0 = st[sel[0]] - 1.0
    for _ in range(40):
        pub_clock(t0)
        rclpy.spin_once(node, timeout_sec=0.05)
    prev = t0
    for k, i in enumerate(sel):
        t = float(st[i])
        for tt in np.arange(max(prev, t - 0.2), t, 0.02):
            pub_clock(float(tt))
            pub_tf(float(tt))
        pub_clock(t)
        pub_tf(t)
        r = ranges[i].astype(float)
        if args.deskew != "none":
            at, direction = args.deskew.split("_")
            r = deskew_scan(
                r,
                amin,
                ainc,
                float(np.interp(t, tw["t"], tw["v"])),
                float(np.interp(t, tw["t"], tw["wz"])),
                0.1,
                at,
                direction == "fwd",
                vehicle.mount_xy_yaw,
            )
        msg = LaserScan()
        msg.header.frame_id = "laser"
        msg.header.stamp.sec, msg.header.stamp.nanosec = sec(t)
        msg.angle_min, msg.angle_increment = amin, ainc
        msg.angle_max = amin + ainc * (len(r) - 1)
        msg.range_min, msg.range_max = rmin, rmax
        msg.scan_time, msg.time_increment = 0.1, 0.1 / len(r)
        msg.ranges = [float(x) for x in r]
        scan_pub.publish(msg)
        pub_clock(t + 0.01)
        pub_tf(t + 0.01)
        end = time.time() + args.pace
        while time.time() < end:
            rclpy.spin_once(node, timeout_sec=0.01)
        prev = t + 0.01
        if k % 500 == 0:
            print(f"replayed {k}", flush=True)
    # Let the queue drain: keep the clock moving until /map has not changed for 8 wall seconds.
    tend = float(st[sel[-1]])
    quiet_since = time.time()
    started = time.time()
    last_hash = None
    for step in range(1, 1000000):
        pub_clock(tend + 0.02 * step)
        pub_tf(tend + 0.02 * step)
        rclpy.spin_once(node, timeout_sec=0.02)
        h = map_hash[0]
        if h != last_hash:
            last_hash, quiet_since = h, time.time()
        if time.time() - quiet_since > 8.0 or time.time() - started > 900.0:
            break
    client = node.create_client(SaveMap, "/slam_toolbox/save_map")
    client.wait_for_service(timeout_sec=30.0)
    req = SaveMap.Request()
    req.name.data = args.out_prefix
    fut = client.call_async(req)
    rclpy.spin_until_future_complete(node, fut, timeout_sec=120.0)
    print("save_map", fut.result(), flush=True)
    np.savez(args.out_prefix + ".tf.npz", map_odom=np.array(map_odom))
    node.destroy_node()
    rclpy.shutdown()
    return 0


if __name__ == "__main__":
    sys.exit(main())
