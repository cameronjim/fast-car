#!/usr/bin/env python3
"""Scan-to-command latency and CPU cost per scan of gap_follow_node, on REAL recorded scans.

Needs the ROS workspace sourced (run inside the ros-dev container, see
tools/race_tune_docker.sh for the mounts). For each configuration it starts gap_follow_node on its
own ROS domain, publishes a bag's /scan messages at the LiDAR's rate with fresh stamps, lets the
node run, and reads its /gap_follow/status: the node itself measures the latency (scan received
-> first /drive_raw published after it, p50 / p99 / max over the last 200 scans) and the
wall-clock and thread-CPU time of each scan's perception half. Nothing else is on the graph (no
safety_node), so the reverse escape never fires and nothing here can move a car.

    tools/race_tune_docker.sh latency --bag /data/bags/2026-11-11T04-20-19_car_teleop_v3 \\
        --seconds 40

A configuration is `name=JSON` ({"profile": ..., "params": {...}}); the default compares the
recommended pre-race setup with the race profile on the timer and with compute_on_scan.
The numbers are for the machine this runs on (docker on Apple silicon), NOT the Jetson: the
Jetson Orin Nano's CPU is several times slower per core, so read the ratios and the headroom
against the 20 ms control period, not the absolute microseconds.
"""

from __future__ import annotations

import argparse
import json
import os
import subprocess
import sys
import tempfile
import time
from pathlib import Path
from typing import Any

sys.path.insert(0, str(Path(__file__).resolve().parent))

from follower_replay import bag, replay

DEFAULT_CONFIGS = {
    "lane_rec": {
        "profile": "lane-2026-10-25",
        "params": {"lane_lookahead_m": 0.8, "outside_entry_offset_m": 0.25},
    },
    "race_timer": {"profile": "race-2026-11-12", "params": {}},
    "race_event": {"profile": "race-2026-11-12", "params": {"compute_on_scan": True}},
}


def params_yaml(params: dict[str, Any]) -> str:
    """A ROS parameter file for gap_follow_node with these parameters (JSON is valid YAML, so
    the values keep their types: a float stays a float)."""
    return json.dumps({"gap_follow_node": {"ros__parameters": params}}, indent=2)


def format_table(results: dict[str, dict[str, Any]]) -> str:
    lines = [
        (
            "| Config | Scans | Latency p50 / p99 / max (ms) | Scan wall p50 / p99 (us) "
            "| Scan CPU p50 / p99 (us) | Commands |"
        ),
        "|---|---|---|---|---|---|",
    ]
    for name, r in results.items():
        s = r.get("status")
        if not s:
            lines.append(f"| {name} | no status received | | | | |")
            continue
        lat, wall, cpu = s["latency_ms"], s["scan_wall_us"], s["scan_cpu_us"]
        lines.append(
            f"| {name} | {s['scans']} | {lat['p50']:.2f} / {lat['p99']:.2f} / {lat['max']:.2f} "
            f"| {wall['p50']:.0f} / {wall['p99']:.0f} | {cpu['p50']:.0f} / {cpu['p99']:.0f} "
            f"| {r['commands']} |"
        )
    return "\n".join(lines)


def run_config(
    name: str, spec: dict[str, Any], scans: list, seconds: float, domain: int, rate_hz: float
) -> dict[str, Any]:
    os.environ["ROS_DOMAIN_ID"] = str(domain)
    import rclpy
    from ackermann_msgs.msg import AckermannDriveStamped
    from rclpy.qos import HistoryPolicy, QoSProfile, ReliabilityPolicy
    from sensor_msgs.msg import LaserScan
    from std_msgs.msg import String

    params = replay.resolve_params(spec["profile"], spec.get("params", {}))
    params = {**params, "status_rate_hz": 2.0}
    for key, value in list(params.items()):
        if (
            isinstance(value, int)
            and not isinstance(value, bool)
            and key != "target_range_median_scans"
        ):
            params[key] = float(value)
    with tempfile.TemporaryDirectory() as tmp:
        params_file = Path(tmp) / "params.yaml"
        params_file.write_text(params_yaml(params))
        node = subprocess.Popen(
            [
                "ros2",
                "run",
                "racer_control",
                "gap_follow_node",
                "--ros-args",
                "--params-file",
                str(params_file),
            ],
            stdout=subprocess.DEVNULL,
            stderr=subprocess.DEVNULL,
            env=os.environ.copy(),
        )
        rclpy.init()
        client = rclpy.create_node("latency_bench")
        status: list[dict] = []
        commands = [0]
        reliable = QoSProfile(
            reliability=ReliabilityPolicy.RELIABLE, history=HistoryPolicy.KEEP_LAST, depth=100
        )
        best_effort = QoSProfile(
            reliability=ReliabilityPolicy.BEST_EFFORT, history=HistoryPolicy.KEEP_LAST, depth=10
        )
        client.create_subscription(
            String, "/gap_follow/status", lambda m: status.append(json.loads(m.data)), reliable
        )
        client.create_subscription(
            AckermannDriveStamped,
            "/drive_raw",
            lambda m: commands.__setitem__(0, commands[0] + 1),
            reliable,
        )
        scan_pub = client.create_publisher(LaserScan, "/scan", best_effort)
        try:
            end = time.monotonic() + 2.0
            while time.monotonic() < end:  # discovery
                rclpy.spin_once(client, timeout_sec=0.05)
            period = 1.0 / rate_hz
            start = time.monotonic()
            index = 0
            next_scan = start
            while time.monotonic() - start < seconds:
                if time.monotonic() >= next_scan:
                    rec = scans[index % len(scans)]
                    index += 1
                    msg = LaserScan()
                    msg.header.frame_id = "laser"
                    msg.header.stamp = client.get_clock().now().to_msg()
                    msg.angle_min = rec.angle_min
                    msg.angle_increment = rec.angle_increment
                    msg.angle_max = rec.angle_min + rec.angle_increment * (len(rec.ranges) - 1)
                    msg.range_min = rec.range_min
                    msg.range_max = rec.range_max
                    msg.ranges = [float(r) for r in rec.ranges]
                    scan_pub.publish(msg)
                    next_scan += period
                rclpy.spin_once(client, timeout_sec=0.002)
        finally:
            client.destroy_node()
            rclpy.shutdown()
            node.terminate()
            try:
                node.wait(timeout=10)
            except subprocess.TimeoutExpired:
                node.kill()
    return {"status": status[-1] if status else None, "commands": commands[0], "params": params}


def main(argv: list[str] | None = None) -> int:
    parser = argparse.ArgumentParser(description=__doc__.split("\n\n")[0])
    parser.add_argument("--bag", type=Path, required=True, help="a bag with /scan messages")
    parser.add_argument("--seconds", type=float, default=40.0)
    parser.add_argument("--rate-hz", type=float, default=10.0, help="scan rate (the C1: 10 Hz)")
    parser.add_argument("--domain-base", type=int, default=150)
    parser.add_argument(
        "--config", action="append", default=[], help="name=JSON {profile, params} (repeatable)"
    )
    parser.add_argument("--out", type=Path)
    args = parser.parse_args(argv)
    configs = dict(DEFAULT_CONFIGS)
    if args.config:
        configs = {}
        for text in args.config:
            name, _, spec = text.partition("=")
            configs[name] = json.loads(spec)
    data = bag.read_bag(args.bag)
    if not data.scans:
        print(f"no /scan messages in {args.bag}")
        return 1
    scans = data.scans
    print(f"{len(scans)} scans of {args.bag.name}, {len(scans[0].ranges)} rays each")
    results = {}
    for offset, (name, spec) in enumerate(configs.items()):
        results[name] = run_config(
            name, spec, scans, args.seconds, args.domain_base + offset, args.rate_hz
        )
        print(f"{name}: done", flush=True)
    table = format_table(results)
    print(table)
    if args.out:
        args.out.write_text(
            json.dumps({k: v for k, v in results.items()}, indent=2, default=str) + "\n"
        )
        args.out.with_suffix(".md").write_text(table + "\n")
    return 0


if __name__ == "__main__":
    sys.exit(main())
