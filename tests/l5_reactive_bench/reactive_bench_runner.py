#!/usr/bin/env python3
"""Reactive benchmark runner: one track, one parameter set, one JSON result.

Starts racer_gym_bridge's bridge_node on a synthetic track (racer_gym_bridge.track_loader:
lane_square, lane_round, lane_open_side, chicane, open_room), racer_safety's safety_node and
racer_control's gap_follow_node as child processes, on the REAL command path (gap_follow_node
/drive_raw -> safety_node -> /drive -> bridge_node, no test-only remap), watches the topics
for a fixed time and prints the result as JSON (also to --out):

  laps, lap_time_s, obstacle_brakes, escapes, min_body_clearance_m, wall_contacts,
  mean_abs_steering_rad, steering_reversals_per_min, plus distance, speed and the run's setup.

See bench_metrics.py for exactly what each number means. Run it with the ROS workspace
sourced (ros2 packages racer_gym_bridge, racer_safety and racer_control built), e.g.

    python3 tests/l5_reactive_bench/reactive_bench_runner.py --track lane_square \\
        --profile floor-2026-10-06 --params-json '{"steering_gain": 2.4}' --out result.json

Parameters of gap_follow_node are the named launch profile (gap_follow.launch.py PROFILES)
with --params-json entries on top; the simulator's /scan is aligned to the vehicle, so
laser_yaw_from_vehicle_params is set false as in the L5 canaries. The profile `none` is the
node defaults the way gap_follow.launch.py starts them (max_speed_mps 2.0).

A contact with a wall (f1tenth_gym's own collision handler, or the body rectangle overlapping a
wall face) is counted and the car is put back at the start with /sim/reset, so one contact does
not end the run; the lap in progress is lost. ROS_DOMAIN_ID is set from --domain-id for the
children and this process, so benchmarks can run side by side.

Exit status: 0 with a result, 3 when a node died (the JSON carries "error").
"""

from __future__ import annotations

import argparse
import importlib.util
import json
import os
import signal
import subprocess
import sys
import tempfile
import time
from pathlib import Path

import numpy as np
import yaml

HERE = Path(__file__).resolve().parent
REPO_ROOT = HERE.parents[1]
VEHICLE_PARAMS_PATH = REPO_ROOT / "config" / "vehicle_params.yaml"
GAP_FOLLOW_LAUNCH = (
    REPO_ROOT / "ros_ws" / "src" / "racer_control" / "launch" / "gap_follow.launch.py"
)

TRACKS = ("lane_square", "lane_round", "lane_open_side", "chicane", "open_room")
# Seconds of run time per track when --duration is not given (a lap of the square lane takes
# four reverse escapes; the open room has no laps to finish).
DEFAULT_DURATION_S = {
    "lane_square": 150.0,
    "lane_round": 120.0,
    "lane_open_side": 90.0,
    "chicane": 90.0,
    "open_room": 60.0,
}
SEED = 11
STARTUP_TIMEOUT_S = 90.0
# After a contact the runner ignores further contacts for this long (s): the reset takes a few
# steps to arrive.
CONTACT_COOLDOWN_S = 1.0
# The geometric clearance check runs on every this-many-th odometry sample (20 Hz at 100 Hz).
CLEARANCE_CHECK_STRIDE = 5

_BRIDGE_TRACK_KEYS = {
    "lane_width_m": "synthetic_lane_width_m",
    "corner_radius_m": "synthetic_corner_radius_m",
    "box_length_m": "synthetic_box_length_m",
    "box_gap_m": "synthetic_box_gap_m",
    "jog_offset_m": "synthetic_jog_offset_m",
    "room_size_m": "synthetic_room_size_m",
}


def _load_module(name: str, path: Path):
    spec = importlib.util.spec_from_file_location(name, path)
    module = importlib.util.module_from_spec(spec)
    sys.modules[name] = module
    spec.loader.exec_module(module)
    return module


def load_profiles() -> dict[str, dict]:
    """gap_follow.launch.py's PROFILES, from the source launch file, so the bench always runs
    what the car runs. `none` carries the launch file's default max speed."""
    module = _load_module("gap_follow_launch", GAP_FOLLOW_LAUNCH)
    profiles = {name: dict(values) for name, values in module.PROFILES.items()}
    profiles["none"] = {"max_speed_mps": module._DEFAULT_MAX_SPEED_MPS}
    return profiles


def gap_follow_parameters(profile: str, overrides: dict, profiles: dict[str, dict]) -> dict:
    """The profile with ``overrides`` on top, types kept: an integer override of a profile value
    that is a float becomes a float (a double parameter rejects an int)."""
    if profile not in profiles:
        raise ValueError(f"profile must be one of {sorted(profiles)}, got {profile!r}")
    params = dict(profiles[profile])
    for key, value in overrides.items():
        if (
            isinstance(params.get(key), float)
            and isinstance(value, int)
            and not isinstance(value, bool)
        ):
            value = float(value)
        params[key] = value
    # racer_gym_bridge's /scan is aligned to the vehicle (yaw 0), not the real car's mount.
    params["laser_yaw_from_vehicle_params"] = False
    return params


def bridge_parameters(track: str, track_params: dict, seed: int, reverse: bool) -> dict:
    unknown = set(track_params) - set(_BRIDGE_TRACK_KEYS)
    if unknown:
        raise ValueError(
            f"unknown track parameter(s) {sorted(unknown)}; known: {sorted(_BRIDGE_TRACK_KEYS)}"
        )
    params: dict = {"seed": seed, "synthetic_track": track, "reverse_direction": reverse}
    for key, value in track_params.items():
        params[_BRIDGE_TRACK_KEYS[key]] = float(value)
    return params


def _write_params_file(path: Path, nodes: dict[str, dict]) -> None:
    path.write_text(
        yaml.safe_dump({name: {"ros__parameters": params} for name, params in nodes.items()})
    )


def _executable(package: str, name: str) -> str:
    from ament_index_python.packages import get_package_prefix

    return str(Path(get_package_prefix(package)) / "lib" / package / name)


def _stamp_s(stamp) -> float:
    return stamp.sec + stamp.nanosec * 1e-9


def _stop(processes: list[subprocess.Popen]) -> None:
    for p in processes:
        if p.poll() is None:
            p.send_signal(signal.SIGINT)
    deadline = time.monotonic() + 8.0
    for p in processes:
        try:
            p.wait(timeout=max(0.1, deadline - time.monotonic()))
        except subprocess.TimeoutExpired:
            p.kill()
            p.wait()


def run_benchmark(args: argparse.Namespace) -> dict:
    """Runs the stack and returns the result dict (see the module docstring)."""
    os.environ["ROS_DOMAIN_ID"] = str(args.domain_id)
    import bench_metrics as bm
    import rclpy
    from ackermann_msgs.msg import AckermannDriveStamped
    from nav_msgs.msg import Odometry
    from racer_gym_bridge import track_loader
    from racer_msgs.msg import SafetyEvent
    from rclpy.qos import HistoryPolicy, QoSProfile, ReliabilityPolicy
    from std_srvs.srv import Trigger

    profiles = load_profiles()
    overrides = json.loads(args.params_json)
    track_params = json.loads(args.track_params_json)
    gap_params = gap_follow_parameters(args.profile, overrides, profiles)
    bridge_params = bridge_parameters(args.track, track_params, args.seed, args.reverse)
    synthetic = track_loader.build_synthetic_track(args.track, **track_params)
    if args.reverse:
        synthetic = synthetic.reversed()
    vehicle = yaml.safe_load(VEHICLE_PARAMS_PATH.read_text())
    box = bm.BodyBox.from_vehicle_params(vehicle)
    lap_tracker = (
        bm.LapTracker(synthetic.centreline_x, synthetic.centreline_y) if synthetic.loop else None
    )
    duration_s = args.duration if args.duration > 0.0 else DEFAULT_DURATION_S[args.track]

    work = Path(tempfile.mkdtemp(prefix="reactive_bench_"))
    params_file = work / "params.yaml"
    _write_params_file(
        params_file,
        {
            "bridge_node": bridge_params,
            "safety_node": {"laser_yaw_from_vehicle_params": False},
            "gap_follow_node": gap_params,
        },
    )
    stack = [
        ("bridge_node", _executable("racer_gym_bridge", "bridge_node")),
        ("safety_node", _executable("racer_safety", "safety_node")),
        ("gap_follow_node", _executable("racer_control", "gap_follow_node")),
    ]
    processes: list[tuple[str, subprocess.Popen]] = []
    for name, exe in stack:
        log = (work / f"{name}.log").open("w")
        processes.append(
            (
                name,
                subprocess.Popen(
                    [exe, "--ros-args", "--params-file", str(params_file)],
                    stdout=log,
                    stderr=subprocess.STDOUT,
                    env=os.environ.copy(),
                ),
            )
        )

    def died() -> str | None:
        for name, p in processes:
            if p.poll() is not None:
                tail = (work / f"{name}.log").read_text()[-1500:]
                return f"{name} exited with {p.returncode}: {tail}"
        return None

    result: dict = {
        "track": args.track,
        "track_params": track_params,
        "reverse": args.reverse,
        "profile": args.profile,
        "overrides": overrides,
        "gap_follow_params": gap_params,
        "seed": args.seed,
        "requested_duration_s": duration_s,
    }
    rclpy.init()
    node = rclpy.create_node("reactive_bench_runner")
    recording = bm.Recording()
    reliable = QoSProfile(
        reliability=ReliabilityPolicy.RELIABLE, history=HistoryPolicy.KEEP_LAST, depth=200
    )

    def on_odom(msg) -> None:
        q = msg.pose.pose.orientation
        yaw = np.arctan2(2.0 * (q.w * q.z + q.x * q.y), 1.0 - 2.0 * (q.y * q.y + q.z * q.z))
        recording.odom.append(
            (
                _stamp_s(msg.header.stamp),
                msg.pose.pose.position.x,
                msg.pose.pose.position.y,
                float(yaw),
                msg.twist.twist.linear.x,
            )
        )

    def on_drive_raw(msg) -> None:
        recording.drive_raw.append(
            (_stamp_s(msg.header.stamp), msg.drive.speed, msg.drive.steering_angle)
        )

    def on_drive(msg) -> None:
        recording.drive.append(
            (_stamp_s(msg.header.stamp), msg.drive.speed, msg.drive.steering_angle)
        )

    def on_event(msg) -> None:
        recording.events.append((_stamp_s(msg.stamp), msg.source, msg.phase, msg.severity))

    node.create_subscription(Odometry, "/sim/ground_truth_odom", on_odom, reliable)
    node.create_subscription(AckermannDriveStamped, "/drive_raw", on_drive_raw, reliable)
    node.create_subscription(AckermannDriveStamped, "/drive", on_drive, reliable)
    node.create_subscription(SafetyEvent, "/safety/events", on_event, reliable)
    reset_client = node.create_client(Trigger, "/sim/reset")

    stopped_because = "duration"
    try:
        # Wait for the whole stack: odometry from the bridge and requests from the follower.
        deadline = time.monotonic() + STARTUP_TIMEOUT_S
        while not (recording.odom and recording.drive_raw):
            rclpy.spin_once(node, timeout_sec=0.05)
            if time.monotonic() > deadline or (why := died()):
                raise RuntimeError(why or f"stack not up after {STARTUP_TIMEOUT_S} s")
        if not reset_client.wait_for_service(timeout_sec=30.0):
            raise RuntimeError("/sim/reset not available")

        def request_reset():
            return reset_client.call_async(Trigger.Request())

        # Start from the reset pose with an empty record, so the run is what happens after it.
        future = request_reset()
        rclpy.spin_until_future_complete(node, future, timeout_sec=30.0)
        if future.result() is None:
            raise RuntimeError("/sim/reset did not complete")
        recording.clear()
        pending_reset = None
        consumed = 0
        previous = None
        cooldown_until = 0.0
        start_stamp = None
        wall_deadline = time.monotonic() + 3.0 * duration_s + 60.0
        last_check = time.monotonic()
        while True:
            rclpy.spin_once(node, timeout_sec=0.01)
            if pending_reset is not None and pending_reset.done():
                pending_reset = None
            while consumed < len(recording.odom):
                sample = recording.odom[consumed]
                consumed += 1
                t, x, y, yaw, v = sample
                if start_stamp is None:
                    start_stamp = t
                if previous is not None and bm.is_teleport(previous[1:3], (x, y)):
                    if lap_tracker is not None:
                        lap_tracker.reset()
                    cooldown_until = 0.0
                elif previous is not None and t >= cooldown_until:
                    reason = bm.gym_contact_signature(previous[3], yaw, previous[4], v)
                    if (
                        reason is None
                        and consumed % CLEARANCE_CHECK_STRIDE == 0
                        and bm.body_clearance_m(x, y, yaw, box, synthetic.walls) <= 0.0
                    ):
                        reason = "body_overlaps_wall"
                    if reason is not None:
                        recording.contacts.append((t, x, y, reason))
                        cooldown_until = t + CONTACT_COOLDOWN_S
                        if pending_reset is None:
                            pending_reset = request_reset()
                if lap_tracker is not None and t >= cooldown_until:
                    lap_tracker.update(x, y, t)
                previous = sample
            if start_stamp is not None and previous is not None:
                if previous[0] - start_stamp >= duration_s:
                    break
                if (
                    args.stop_after_laps
                    and lap_tracker
                    and lap_tracker.laps >= args.stop_after_laps
                ):
                    stopped_because = "laps"
                    break
                if args.stop_after_contacts and len(recording.contacts) >= args.stop_after_contacts:
                    stopped_because = "contacts"
                    break
            if time.monotonic() > wall_deadline:
                stopped_because = "wall_timeout"
                break
            if time.monotonic() - last_check > 1.0:
                last_check = time.monotonic()
                if why := died():
                    raise RuntimeError(why)
        metrics = bm.compute_metrics(
            recording, box=box, walls=synthetic.walls, lap_tracker=lap_tracker
        )
        result.update(metrics)
        result["stopped_because"] = stopped_because
        if args.trace_out:
            np.savez_compressed(
                args.trace_out,
                odom=np.array(recording.odom),
                drive_raw=np.array(recording.drive_raw),
                drive=np.array(recording.drive),
            )
    except RuntimeError as e:
        result["error"] = str(e)
    finally:
        node.destroy_node()
        rclpy.shutdown()
        _stop([p for _, p in processes])
    return result


def main(argv: list[str] | None = None) -> int:
    parser = argparse.ArgumentParser(
        description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter
    )
    parser.add_argument("--track", required=True, choices=TRACKS)
    parser.add_argument(
        "--profile", default="floor-2026-10-06", help="gap_follow.launch.py profile"
    )
    parser.add_argument(
        "--params-json",
        default="{}",
        help="gap_follow_node parameters on top of the profile (JSON)",
    )
    parser.add_argument(
        "--track-params-json",
        default="{}",
        help="track parameters (JSON): " + ", ".join(_BRIDGE_TRACK_KEYS),
    )
    parser.add_argument(
        "--duration", type=float, default=0.0, help="run time (s); 0 = the track's default"
    )
    parser.add_argument(
        "--stop-after-laps",
        type=int,
        default=0,
        help="stop once this many laps are done (0 = never)",
    )
    parser.add_argument(
        "--stop-after-contacts",
        type=int,
        default=0,
        help="stop after this many wall contacts (0 = never)",
    )
    parser.add_argument("--reverse", action="store_true", help="drive the loop clockwise")
    parser.add_argument("--seed", type=int, default=SEED)
    parser.add_argument("--domain-id", type=int, default=int(os.environ.get("ROS_DOMAIN_ID", "0")))
    parser.add_argument("--out", type=Path, help="write the JSON result here too")
    parser.add_argument(
        "--trace-out", type=Path, help="write the raw odometry and drive records here (npz)"
    )
    args = parser.parse_args(argv)
    sys.path.insert(0, str(HERE))
    result = run_benchmark(args)
    text = json.dumps(result, indent=2, sort_keys=True)
    print(text)
    if args.out:
        args.out.write_text(text + "\n")
    return 3 if "error" in result else 0


if __name__ == "__main__":
    sys.exit(main())
