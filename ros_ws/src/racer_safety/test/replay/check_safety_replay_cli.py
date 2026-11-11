#!/usr/bin/env python3
"""CTest for safety_replay_cli (src/safety_replay_cli.cpp), stdlib only.

Builds small synthetic streams in the CLI's text format (full-circle scans, LiDAR yaw pi as on the
car, so the binding's mount yaw is used) and checks, from the numbers themselves:

  * a straight approach to a wall across the path at a steady request: the committed rule (body
    distance, schema 0.13.0) trips at ttc_brake_s x speed off the BUMPER, the October rule
    (distance_from_body false, 0.3 s, 0.25 m floor) at 0.3 x speed off the LiDAR head, so the new
    rule trips earlier, by the bumper offset and the TTC change;
  * a /drive_raw gap longer than the watchdog produces one watchdog engage and release;
  * a request drop (1.97 -> 1.0 m/s) toward a wall trips with the speed bound on and not with it
    off (speed_bound_deceleration_mps2=0);
  * an O line faster than the request trips earlier than without odometry, and is ignored with
    use_odometry=false or once older than odom_timeout_s;
  * the trip record's predicted stop uses the measured speed when fresh;
  * --describe, and refusals: unknown key, out-of-range value, malformed or backwards stream.
"""

import argparse
import json
import math
import subprocess
import sys
import tempfile
from pathlib import Path

N_RAYS = 720
YAW = math.pi  # the car's sensors.lidar.mount_yaw_rad; the CLI takes it from the binding


def run(cli, args, stdin=None):
    return subprocess.run(
        [str(cli), *args], input=stdin, capture_output=True, text=True, check=False, timeout=60
    )


def records(proc):
    assert proc.returncode == 0, proc.stderr
    return [json.loads(line) for line in proc.stdout.splitlines() if line.strip()]


def scan_line(t, head_gap_m):
    """A wall across the path head_gap_m ahead of the LiDAR head (vehicle frame), |bearing| < 1."""
    inc = 2.0 * math.pi / N_RAYS
    angle_min = -math.pi
    ranges = []
    for i in range(N_RAYS):
        vehicle = angle_min + i * inc + YAW
        vehicle = math.atan2(math.sin(vehicle), math.cos(vehicle))
        if abs(vehicle) < 1.0:
            ranges.append(f"{head_gap_m / math.cos(vehicle):.6f}")
        else:
            ranges.append("inf")
    return f"S {t:.6f} {angle_min:.9f} {inc:.9f} 0.05 12 {N_RAYS} " + " ".join(ranges)


def approach(speed, start_gap=1.5, t_end=3.0, drive=None, odom=None, gap_from=None):
    """A stream: the car closing on a wall at `speed` (the wall's head gap shrinks with time),
    scans at 10 Hz, /drive_raw at 50 Hz (drive(t) -> (steer, speed) or None to skip), optional
    odometry at 50 Hz (odom(t) -> speed or None)."""
    lines = []
    t = 0.0
    step = 0.02
    k = 0
    while t <= t_end + 1e-9:
        gap = gap_from(t) if gap_from else start_gap - speed * t
        if k % 5 == 0:
            lines.append(scan_line(t, max(gap, 0.06)))
        command = drive(t) if drive else (0.0, speed)
        if command is not None:
            lines.append(f"D {t + 0.001:.6f} {command[0]:.6f} {command[1]:.6f}")
        if odom is not None:
            v = odom(t)
            if v is not None:
                lines.append(f"O {t + 0.002:.6f} {v:.6f}")
        t = round(t + step, 6)
        k += 1
    return "\n".join(lines) + "\n"


def first_trip(recs):
    trips = [r for r in recs if r["type"] == "trip" and r["gate"] == "ttc"]
    return trips[0] if trips else None


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument("--cli", required=True)
    args = parser.parse_args()
    cli = Path(args.cli)
    failures = []

    def check(condition, message):
        if not condition:
            failures.append(message)

    described = json.loads(run(cli, ["--describe"]).stdout)
    for key in (
        "ttc_brake_s",
        "ttc_warning_s",
        "min_forward_clearance_m",
        "distance_from_body",
        "front_overhang_m",
        "lidar_mount_x_m",
        "lidar_mount_y_m",
        "wheelbase_m",
        "stop_latency_s",
        "stop_deceleration_mps2",
        "schema_version",
    ):
        check(key in described, f"--describe lacks {key}")
    bumper = described["wheelbase_m"] + described["front_overhang_m"] - described["lidar_mount_x_m"]
    ttc_new = described["ttc_brake_s"]

    with tempfile.TemporaryDirectory() as tmp:
        stream = Path(tmp) / "approach.txt"
        speed = 0.8
        stream.write_text(approach(speed))
        new = first_trip(records(run(cli, ["--stream", str(stream)])))
        old = first_trip(
            records(
                run(
                    cli,
                    [
                        "--stream",
                        str(stream),
                        "--set",
                        "distance_from_body=false",
                        "--set",
                        "ttc_brake_s=0.3",
                        "--set",
                        "ttc_warning_s=0.45",
                        "--set",
                        "min_forward_clearance_m=0.25",
                        "--set",
                        "speed_bound_deceleration_mps2=0",
                    ],
                )
            )
        )
        check(new is not None and old is not None, "both rules must trip on the approach")
        if new and old:
            # Scans are 10 Hz while the wall closes at 0.8 m/s, so the judged distance is the
            # last scan's: within one scan step (0.08 m) below the threshold.
            check(new["cause"] == "ttc", f"new rule cause {new['cause']}")
            check(
                ttc_new * speed - 0.081 <= new["distance_m"] <= ttc_new * speed + 1e-6,
                f"new rule distance {new['distance_m']} vs {ttc_new * speed} off the bumper",
            )
            check(
                0.3 * speed - 0.081 <= old["distance_m"] <= 0.3 * speed + 1e-6,
                f"old rule distance {old['distance_m']} vs {0.3 * speed} off the head",
            )
            check(new["t"] < old["t"], "the body rule must trip earlier on a straight approach")
            head_gap_new = new["distance_m"] + bumper
            check(
                head_gap_new > old["distance_m"] + 0.15,
                f"body rule trips {head_gap_new:.3f} m from the head, old {old['distance_m']:.3f}",
            )
            expected_stop = speed * described["stop_latency_s"] + speed**2 / (
                2 * described["stop_deceleration_mps2"]
            )
            check(
                math.isclose(new["predicted_stop_m"], expected_stop, abs_tol=1e-6),
                f"predicted stop {new['predicted_stop_m']} vs {expected_stop}",
            )
            check(
                math.isclose(
                    new["predicted_clearance_m"], new["distance_m"] - expected_stop, abs_tol=1e-6
                ),
                "predicted clearance = distance - stop",
            )
            check(
                math.isclose(new["ttc_speed_mps"], speed, abs_tol=1e-6),
                f"steady approach: TTC speed is the request, got {new['ttc_speed_mps']}",
            )

        # Watchdog: no /drive_raw for 0.1 s at t 1.0 (far from the wall).
        stream.write_text(
            approach(0.3, start_gap=5.0, drive=lambda t: None if 1.0 <= t < 1.1 else (0.0, 0.3))
        )
        recs = records(run(cli, ["--stream", str(stream)]))
        # (The first tick, before any /drive_raw has arrived, is a watchdog engagement too, as
        # in safety_node; count the ones after it.)
        engages = [
            r
            for r in recs
            if r["type"] == "event"
            and r["source"] == "watchdog"
            and r["phase"] == "engage"
            and r["t"] > 0.5
        ]
        released = [
            r
            for r in recs
            if r["type"] == "event"
            and r["source"] == "watchdog"
            and r["phase"] == "release"
            and r["t"] > 0.5
        ]
        check(
            len(engages) == 1 and len(released) == 1,
            f"one watchdog engagement expected, got {len(engages)} / {len(released)}",
        )
        if engages:
            check(1.06 <= engages[0]["t"] <= 1.1, f"watchdog engaged at {engages[0]['t']}")
        check(first_trip(recs) is None, "no obstacle trip 5 m from the wall")

        # Request drop: 1.97 m/s for 1 s, then 1.0 m/s; from the next scan on, a wall 0.4 m off
        # the bumper (held there): 0.4 s of TTC on the request, about 0.2 s on the bound.
        def drop(t):
            return (0.0, 1.97 if t < 1.0 else 1.0)

        stream.write_text(
            approach(
                0.0, drive=drop, gap_from=lambda t: 3.0 if t < 1.05 else 0.40 + bumper, t_end=1.5
            )
        )
        # This scenario's numbers (0.4 s of TTC on the request, about 0.2 s on the bound) are
        # written for a 0.34 s brake threshold, so it pins that rule instead of the committed one
        # (0.60 s since schema 0.14.0).
        rule = ["--set", "ttc_brake_s=0.34", "--set", "ttc_warning_s=0.45"]
        with_bound = first_trip(records(run(cli, ["--stream", str(stream), *rule])))
        without = first_trip(
            records(
                run(
                    cli,
                    ["--stream", str(stream), *rule, "--set", "speed_bound_deceleration_mps2=0"],
                )
            )
        )
        check(with_bound is not None, "the speed bound must trip after the request drop")
        if with_bound:
            check(
                with_bound["ttc_speed_mps"] > 1.5,
                f"TTC speed after the drop {with_bound['ttc_speed_mps']}",
            )
        check(without is None, "without the bound the request alone (0.4 s) must not trip")

        # Odometry: the car measured at 1.2 m/s while asked for 0.6 m/s.
        stream.write_text(approach(0.6, odom=lambda t: 1.2))
        odo = first_trip(records(run(cli, ["--stream", str(stream)])))
        no_odo = first_trip(
            records(run(cli, ["--stream", str(stream), "--set", "use_odometry=false"]))
        )
        check(odo is not None and no_odo is not None, "both odometry runs must trip")
        if odo and no_odo:
            check(odo["t"] < no_odo["t"], "a faster measured speed must trip earlier")
            check(math.isclose(odo["ttc_speed_mps"], 1.2, abs_tol=1e-6), "TTC speed = measured")
            check(math.isclose(odo["stop_speed_mps"], 1.2, abs_tol=1e-6), "stop speed = measured")
            check(math.isclose(no_odo["stop_speed_mps"], 0.6, abs_tol=1e-6), "stop speed = TTC")
        # Odometry that stops arriving after 0.2 s is stale by the time the car is close.
        stream.write_text(approach(0.6, odom=lambda t: 1.2 if t < 0.2 else None))
        stale = first_trip(records(run(cli, ["--stream", str(stream)])))
        check(
            stale is not None and no_odo is not None and math.isclose(stale["t"], no_odo["t"]),
            "stale odometry must be ignored",
        )

        # Refusals.
        check(run(cli, ["--set", "nope=1", "--describe"]).returncode == 2, "unknown key")
        check(run(cli, ["--set", "ttc_brake_s=99", "--describe"]).returncode == 2, "range")
        check(run(cli, ["--set", "ttc_brake_s", "--describe"]).returncode == 2, "no '='")
        check(run(cli, ["--set", "ttc_brake_s=abc", "--describe"]).returncode == 2, "not a number")
        check(run(cli, ["--bogus"]).returncode == 2, "unknown flag")
        check(run(cli, ["--stream", str(Path(tmp) / "missing.txt")]).returncode == 2, "no file")
        check(run(cli, [], stdin="X 0 1\n").returncode == 3, "unknown kind")
        check(run(cli, [], stdin="D 0 1\n").returncode == 3, "short D line")
        check(run(cli, [], stdin="O 0\n").returncode == 3, "short O line")
        check(run(cli, [], stdin="S 0 0 0.1 0.05 12 3 1 2\n").returncode == 3, "n mismatch")
        check(run(cli, [], stdin="S 0 0 0.1 0.05\n").returncode == 3, "short S line")
        check(run(cli, [], stdin="D 1 0 0\nD 0.5 0 0\n").returncode == 3, "backwards stamps")
        check(run(cli, [], stdin="D nan 0 0\n").returncode == 3, "non-finite stamp")
        check(run(cli, [], stdin="D 0 x 0\n").returncode == 3, "bad number")
        empty = run(cli, [], stdin="# nothing\n\n")
        check(empty.returncode == 0 and empty.stdout == "", "an empty stream is no output")

    for message in failures:
        print("FAIL:", message)
    if failures:
        return 1
    print("check_safety_replay_cli: all checks passed")
    return 0


if __name__ == "__main__":
    sys.exit(main())
