#!/usr/bin/env python3
"""Regression: safety_node must exit cleanly on SIGINT while its inputs are streaming.

CI on 2026-11-10 (L5 park canary) saw safety_node die with exit code -11 (SIGSEGV) at test
shutdown. A segfault in the safety node is never acceptable, even at shutdown, and -11 is not an
allowed exit code anywhere. This starts the real safety_node executable `--iterations` times,
streams /scan, /drive_raw and /odom/wheel at their real rates (faster, to widen the race) while
it runs, sends SIGINT at a different moment after the first /drive each time, keeps streaming
while the node shuts down (a message arriving mid-teardown is the case that matters), and fails
on any exit status other than 0 or -SIGINT, naming the signal.

usage: check_safety_node_sigint.py --exe <path to safety_node> [--iterations 20]
"""

import argparse
import os
import random
import signal
import subprocess
import sys
import time

os.environ.setdefault("ROS_DOMAIN_ID", "95")

import rclpy
from ackermann_msgs.msg import AckermannDriveStamped
from nav_msgs.msg import Odometry
from rclpy.qos import HistoryPolicy, QoSProfile, ReliabilityPolicy
from sensor_msgs.msg import LaserScan

ALLOWED = (0, -signal.SIGINT)


def qos(reliable: bool) -> QoSProfile:
    return QoSProfile(
        reliability=ReliabilityPolicy.RELIABLE if reliable else ReliabilityPolicy.BEST_EFFORT,
        history=HistoryPolicy.KEEP_LAST,
        depth=10,
    )


def describe(code: int) -> str:
    if code < 0:
        return f"killed by signal {-code} ({signal.Signals(-code).name})"
    return f"exit status {code}"


def main() -> int:
    parser = argparse.ArgumentParser()
    parser.add_argument("--exe", required=True)
    parser.add_argument("--iterations", type=int, default=20)
    parser.add_argument("--seed", type=int, default=7)
    parser.add_argument(
        "--node-arg", action="append", default=[], help="extra argument for safety_node"
    )
    args = parser.parse_args()
    rng = random.Random(args.seed)

    rclpy.init()
    node = rclpy.create_node("safety_sigint_driver")
    drive_raw = node.create_publisher(AckermannDriveStamped, "/drive_raw", qos(True))
    scan_pub = node.create_publisher(LaserScan, "/scan", qos(False))
    odom_pub = node.create_publisher(Odometry, "/odom/wheel", qos(False))
    seen = {"drive": 0}
    node.create_subscription(
        AckermannDriveStamped,
        "/drive",
        lambda _m: seen.__setitem__("drive", seen["drive"] + 1),
        qos(True),
    )

    scan = LaserScan()
    scan.angle_min, scan.angle_max = -3.14159, 3.14159
    scan.angle_increment = 6.28318 / 719
    scan.range_min, scan.range_max = 0.05, 12.0
    scan.ranges = [1.5 + 0.5 * ((i * 7) % 11) / 11.0 for i in range(720)]
    odom = Odometry()
    odom.twist.twist.linear.x = 0.6
    cmd = AckermannDriveStamped()
    cmd.drive.speed = 0.8
    cmd.drive.steering_angle = 0.1

    def stream(seconds: float) -> None:
        end = time.monotonic() + seconds
        while time.monotonic() < end:
            drive_raw.publish(cmd)
            odom_pub.publish(odom)
            scan_pub.publish(scan)
            rclpy.spin_once(node, timeout_sec=0.002)

    bad = []
    for i in range(args.iterations):
        seen["drive"] = 0
        proc = subprocess.Popen(
            [args.exe, *args.node_arg], stdout=subprocess.DEVNULL, stderr=subprocess.DEVNULL
        )
        started = time.monotonic()
        while seen["drive"] == 0 and time.monotonic() - started < 20.0:
            stream(0.05)
        if seen["drive"] == 0:
            proc.kill()
            print(f"iteration {i}: safety_node never published /drive")
            return 1
        stream(rng.uniform(0.05, 1.0))
        proc.send_signal(signal.SIGINT)
        deadline = time.monotonic() + 10.0
        while proc.poll() is None and time.monotonic() < deadline:
            stream(0.01)  # keep the inputs coming during the teardown
        if proc.poll() is None:
            proc.kill()
            proc.wait()
            bad.append((i, "did not exit within 10 s of SIGINT"))
        elif proc.returncode not in ALLOWED:
            bad.append((i, describe(proc.returncode)))
    node.destroy_node()
    rclpy.shutdown()
    for i, why in bad:
        print(f"iteration {i}: {why}")
    print(f"safety_node SIGINT: {args.iterations - len(bad)} of {args.iterations} clean exits")
    return 1 if bad else 0


if __name__ == "__main__":
    sys.exit(main())
