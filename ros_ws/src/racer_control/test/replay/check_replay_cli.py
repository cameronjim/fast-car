#!/usr/bin/env python3
"""CTest for gap_follow_replay_cli (src/gap_follow_replay_cli.cpp), stdlib only.

Runs the CLI on scenes.txt, five synthetic full-circle scans (1 degree rays, LiDAR yaw pi as on
the car, so the binding's mount yaw is used) at 10 Hz:

  0.0  a lane with walls 0.6 m either side, open ahead
  0.1  a wall 0.45 m ahead, opening to the LEFT      -> steer left (positive) to the limit
  0.2  the mirror of that                            -> steer right (negative)
  0.3  the left opening, plus an object 0.25 m from the head at 100 degrees left, inside the
       car's side margin -> the swept-path clamp must forbid the left turn
  0.4  every return NaN                              -> an invalid scan, no command

with params.json (the steering part of the floor-2026-10-06 profile), and checks:
  * the sign convention and the clamp / invalid flags above, from the numbers themselves;
  * the corner override (a large corner_min_clearance_m turns every sector ray into a block);
  * the low-pass against its closed form, stepped the way gap_follow_node's timer steps it;
  * parameter validation (unknown key, out of range, yaw disagreeing with the binding) and the
    key=value params format, stdin input and malformed scan lines;
  * target_mode lane_centre (2026-10-24): the centred lane of scan 0.0 gives zero pure-pursuit
    steering, target_mode quoted in JSON and bare in --set, the deprecated target_deepest_ray,
    refusals, (with --floor-excerpt) seven corner scans of the 2026-10-24 floor bag, and (with
    --apex-excerpt) four apex scans of the 2026-10-25 one;
  * the whole output against expected.jsonl within 1e-6. Regenerate that file only on purpose
    (--write-expected) and say why in the PR: a silent golden refresh is a defect
    (claude-docs/12-testing.md, L4).
"""

import argparse
import json
import math
import subprocess
import sys
import tempfile
from pathlib import Path

TOLERANCE = 1e-6
MAX_STEERING_RAD = 0.4189  # config/vehicle_params.yaml steering.max_angle_rad
WHEELBASE_M = 0.3302  # config/vehicle_params.yaml chassis.wheelbase_m
# The target_mode lane_centre output keys (null / [] in the gap modes).
LANE_KEYS = {"lane_aim_x_m", "lane_aim_y_m", "lane_half_width_m", "lane_width_m", "lane_trace"}


def run(cli, args, stdin=None):
    return subprocess.run(
        [str(cli), *args], input=stdin, capture_output=True, text=True, check=False, timeout=60
    )


def rows(proc):
    assert proc.returncode == 0, proc.stderr
    return [json.loads(line) for line in proc.stdout.splitlines()]


def close(a, b):
    return abs(a - b) <= TOLERANCE


def check_close_rows(got, expected):
    """Every key of the expected rows matches. The lane_centre keys added on 2026-10-24 are not
    in expected.jsonl (it predates them and was NOT regenerated); in the gap modes it covers
    they must be null / empty, which is checked here instead."""
    assert len(got) == len(expected), (len(got), len(expected))
    for i, (g, e) in enumerate(zip(got, expected)):
        assert set(g) - set(e) <= LANE_KEYS and set(e) <= set(g), (i, g.keys(), e.keys())
        for key in set(g) - set(e):
            assert g[key] in (None, []), (i, key, g[key])
        for key, ev in e.items():
            gv = g[key]
            if isinstance(ev, float) and gv is not None:
                assert close(gv, ev), (i, key, gv, ev)
            else:
                assert gv == ev, (i, key, gv, ev)


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument("--cli", required=True, type=Path)
    parser.add_argument("--fixture-dir", required=True, type=Path)
    parser.add_argument("--write-expected", action="store_true")
    parser.add_argument(
        "--floor-excerpt", type=Path, help="tools/tests/fixtures/floor_2026-10-24_corners.txt"
    )
    parser.add_argument(
        "--apex-excerpt", type=Path, help="tools/tests/fixtures/floor_2026-10-25_apex.txt"
    )
    args = parser.parse_args()
    cli = args.cli
    scenes = args.fixture_dir / "scenes.txt"
    params = args.fixture_dir / "params.json"
    expected_path = args.fixture_dir / "expected.jsonl"

    out = rows(run(cli, ["--params", str(params), "--scans", str(scenes)]))
    assert len(out) == 5

    # --describe: the binding's geometry (LiDAR yaw pi on this car, a 0.31 m wide body).
    desc = json.loads(run(cli, ["--params", str(params), "--describe"]).stdout)
    assert abs(desc["laser_yaw_rad"] - math.pi) < 1e-5
    assert close(desc["max_steering_rad"], MAX_STEERING_RAD) and desc["half_width_m"] > 0

    # Sign convention and the flags.
    assert abs(out[0]["steering_rad"]) < 1e-3 and out[0]["gap_found"]
    assert close(out[1]["steering_rad"], MAX_STEERING_RAD) and out[1]["target_bearing_rad"] > 0.5
    assert close(out[2]["steering_rad"], -MAX_STEERING_RAD) and out[2]["target_bearing_rad"] < -0.5
    assert out[3]["clamped"] and close(out[3]["wanted_steering_rad"], MAX_STEERING_RAD)
    # The object sits beside the car behind the front axle, 9 cm off the flank (inside the 0.1 m
    # margin): since 2026-10-25 the turn may take half that clearance (gap_follow.hpp, BESIDE THE
    # CAR) instead of being zeroed, so the clamp cuts full lock to about 0.35 rad.
    assert 0.3 < out[3]["steering_rad"] < MAX_STEERING_RAD - 0.05, out[3]
    assert not any(r["clamped"] for r in out[:3])
    assert out[4]["valid"] is False and out[4]["steering_rad"] is None
    assert out[4]["target_bearing_rad"] is None and not out[4]["gap_found"]
    # An invalid scan does not replace the latest result: the filter keeps stepping toward the
    # previous (clamped) steering.
    held = out[3]["steering_rad"]
    assert abs(out[4]["filtered_steering_rad"] - held) < abs(out[3]["filtered_steering_rad"] - held)

    if args.write_expected:
        expected_path.write_text("".join(json.dumps(r) + "\n" for r in out), encoding="utf-8")
        print(f"wrote {expected_path}")
    expected = [json.loads(x) for x in expected_path.read_text(encoding="utf-8").splitlines()]
    check_close_rows(out, expected)

    scene_lines = [
        ln for ln in scenes.read_text(encoding="utf-8").splitlines() if not ln.startswith("#")
    ]

    # Corner override: with a clearance larger than any range every sector ray is "too close".
    with tempfile.TemporaryDirectory() as tmp:
        one = Path(tmp) / "one.txt"
        one.write_text(scene_lines[1] + "\n", encoding="utf-8")
        blocked = rows(
            run(
                cli,
                [
                    "--params",
                    str(params),
                    "--scans",
                    str(one),
                    "--set",
                    "corner_min_clearance_m=6",
                    "--set",
                    "swept_path_clamp=false",
                ],
            )
        )[0]
        assert blocked["corner_blocked"] and blocked["steering_rad"] == 0.0
        assert close(blocked["wanted_steering_rad"], MAX_STEERING_RAD)
        free = rows(run(cli, ["--params", str(params), "--scans", str(one)]))[0]
        assert not free["corner_blocked"]

        # Low-pass closed form. Three identical scans 0.1 s apart, 50 Hz timer, tau 0.15 s:
        # five ticks per scan, the very first one steps by 0 (as on_timer does), so scan 0 sees
        # four steps of a = dt / (tau + dt) and the others five.
        tau, dt = 0.15, 0.02
        a = dt / (tau + dt)
        three = Path(tmp) / "three.txt"
        body = scene_lines[1].split(maxsplit=1)[1]
        three.write_text("".join(f"{0.1 * k:.3f} {body}\n" for k in range(3)), encoding="utf-8")
        lp = rows(run(cli, ["--params", str(params), "--scans", str(three)]))
        x = MAX_STEERING_RAD
        y = x * (1 - (1 - a) ** 4)
        assert close(lp[0]["filtered_steering_rad"], y), (lp[0], y)
        y = x - (x - y) * (1 - a) ** 5
        assert close(lp[1]["filtered_steering_rad"], y)
        y = x - (x - y) * (1 - a) ** 5
        assert close(lp[2]["filtered_steering_rad"], y)
        # tau 0 passes the input straight through.
        direct = rows(
            run(
                cli,
                [
                    "--params",
                    str(params),
                    "--scans",
                    str(three),
                    "--set",
                    "steering_time_constant_s=0",
                ],
            )
        )
        assert close(direct[0]["filtered_steering_rad"], x)
        # A /scan gap longer than scan_timeout_s: the node stops publishing once the latest scan
        # is that old, so with a 0.31 s timeout scan 0 sees ticks at 0 .. 0.30 s (the first one a
        # zero step, then 15 steps) and the rest of the 2 s gap steps nothing.
        gap = Path(tmp) / "gap.txt"
        gap.write_text(f"0.000 {body}\n2.000 {body}\n", encoding="utf-8")
        g = rows(
            run(cli, ["--params", str(params), "--scans", str(gap), "--set", "scan_timeout_s=0.31"])
        )
        assert close(g[0]["filtered_steering_rad"], x * (1 - (1 - a) ** 15)), g[0]

        # stdin works the same as --scans.
        piped = rows(run(cli, ["--params", str(params)], stdin=three.read_text(encoding="utf-8")))
        check_close_rows(piped, lp)

        # key=value params, with the node's speed and escape keys accepted and ignored.
        kv = Path(tmp) / "params.txt"
        kv.write_text(
            "# comment\nsteering_gain = 1.6  # trailing\nmax_speed_mps=0.9\nescape_speed_mps=-0.5\n"
            "reverse_escape=true\nswept_path_clamp=false\n",
            encoding="utf-8",
        )
        kvrows = rows(run(cli, ["--params", str(kv), "--scans", str(one)]))
        assert close(kvrows[0]["steering_rad"], MAX_STEERING_RAD) and not kvrows[0]["clamped"]

        # Bad input is refused, never run with a default.
        for bad in (
            ["--set", "no_such_param=1"],
            ["--set", "steering_gain=11"],
            ["--set", "forward_preference=abc"],
            ["--set", "laser_yaw_offset_rad=1.0"],
            ["--set", "corner_sector_inner_rad=3", "--set", "corner_sector_outer_rad=1"],
        ):
            proc = run(cli, ["--params", str(params), "--scans", str(one), *bad])
            assert proc.returncode == 2 and proc.stdout == "", (bad, proc.returncode, proc.stderr)
        broken = Path(tmp) / "broken.txt"
        broken.write_text("0.0 -3.14 0.01 5 1 2 3\n", encoding="utf-8")
        proc = run(cli, ["--params", str(params), "--scans", str(broken)])
        assert proc.returncode == 3 and "line 1" in proc.stderr, proc.stderr

        # target_mode lane_centre on scene 0 (walls 0.6 m either side, open ahead): the lane is
        # 1.2 m wide, the car is on its centreline, the aim point is straight ahead and the
        # pure-pursuit steering is zero. Quoted in a JSON params file or bare in --set.
        lane_json = Path(tmp) / "lane.json"
        loaded = json.loads(params.read_text(encoding="utf-8"))
        lane_json.write_text(json.dumps({**loaded, "target_mode": "lane_centre"}), encoding="utf-8")
        zero = Path(tmp) / "zero.txt"
        zero.write_text(scene_lines[0] + "\n", encoding="utf-8")
        lane = rows(run(cli, ["--params", str(lane_json), "--scans", str(zero)]))[0]
        assert lane["valid"] and abs(lane["lane_width_m"] - 1.2) < 0.02, lane
        assert abs(lane["lane_aim_y_m"]) < 0.01 and abs(lane["wanted_steering_rad"]) < 0.01, lane
        assert len(lane["lane_trace"]) == 13 and abs(lane["lane_trace"][-1][0] - 0.6) < 0.01
        same = rows(
            run(
                cli,
                ["--params", str(params), "--scans", str(zero), "--set", "target_mode=lane_centre"],
            )
        )[0]
        check_close_rows([same], [lane])
        # The aim point is the pure-pursuit target: steering = atan(2 L sin(alpha) / l_d).
        ax, ay = lane["lane_aim_x_m"], lane["lane_aim_y_m"]
        pp = math.atan(2.0 * WHEELBASE_M * ay / (ax * ax + ay * ay))
        assert abs(lane["wanted_steering_rad"] - pp) < 1e-6, (lane, pp)
        # The deprecated bool: true means gap_deepest (with a warning), and disagrees with
        # lane_centre (refused, as the node refuses to start).
        deepest = run(
            cli, ["--params", str(params), "--scans", str(one), "--set", "target_deepest_ray=true"]
        )
        assert deepest.returncode == 0 and "deprecated" in deepest.stderr, deepest.stderr
        explicit = rows(
            run(
                cli,
                ["--params", str(params), "--scans", str(one), "--set", "target_mode=gap_deepest"],
            )
        )
        check_close_rows(rows(deepest), explicit)
        for bad in (
            ["--set", "target_mode=lane_center"],
            ["--set", "target_mode=lane_centre", "--set", "target_deepest_ray=true"],
            ["--set", "lane_lookahead_m=0"],
        ):
            proc = run(cli, ["--params", str(params), "--scans", str(one), *bad])
            assert proc.returncode == 2 and proc.stdout == "", (bad, proc.returncode, proc.stderr)

    # Seven scans of the 2026-10-24 floor bag (tools/tests/fixtures), the loop's corners with
    # the car on the inside edge: lane_centre turns left (the loop is counter-clockwise), never
    # right, with the aim point and the lane width defined.
    if args.floor_excerpt:
        floor = rows(
            run(
                cli,
                [
                    "--params",
                    str(params),
                    "--scans",
                    str(args.floor_excerpt),
                    "--set",
                    "target_mode=lane_centre",
                    "--set",
                    "range_max_m=16",
                ],
            )
        )
        assert len(floor) == 7
        for r in floor:
            # Never away from the bend (46.04 s is just before it, about straight).
            assert r["valid"] and r["wanted_steering_rad"] > -0.05, r
            assert r["lane_aim_x_m"] is not None and r["lane_half_width_m"] > 0.3, r
        assert sum(1 for r in floor if r["wanted_steering_rad"] > 0.1) >= 5, floor

    # Four scans of the 2026-10-25 floor bag (tools/tests/fixtures): three inside corners with the
    # car beside the apex and a bend where the outer wall crosses the trace heading. At the 0.6 m
    # lookahead the trace walks a whole lookahead away from the car (it never doubles back), the
    # aim point is in the lane (at least 0.6 h from every return), the command is off the lock,
    # and the swept-path clamp does not cut it (gap_follow.hpp, CENTRELINE TRACE and BESIDE THE
    # CAR).
    if args.apex_excerpt:
        apex = rows(
            run(
                cli,
                [
                    "--params",
                    str(params),
                    "--scans",
                    str(args.apex_excerpt),
                    "--set",
                    "target_mode=lane_centre",
                    "--set",
                    "lane_lookahead_m=0.6",
                    "--set",
                    "range_max_m=16",
                ],
            )
        )
        scans = [
            ln.split()
            for ln in args.apex_excerpt.read_text().splitlines()
            if not ln.startswith("#")
        ]
        assert len(apex) == len(scans) == 4
        for r, w in zip(apex, scans):
            assert r["valid"] and len(r["lane_trace"]) == 13, r["stamp_s"]
            aim = (r["lane_aim_x_m"], r["lane_aim_y_m"])
            assert math.hypot(*aim) > 0.4, (r["stamp_s"], aim)
            assert abs(r["wanted_steering_rad"]) < MAX_STEERING_RAD - 1e-3, r["stamp_s"]
            assert not r["clamped"] and not r["corner_blocked"], r["stamp_s"]
            angle_min, increment = float(w[1]), float(w[2])
            clearance = math.inf
            for i, value in enumerate(w[4:]):
                rng = float(value)
                if not (0.05 <= rng <= 16.0):
                    continue
                bearing = angle_min + i * increment + desc["laser_yaw_rad"]
                x = rng * math.cos(bearing) + desc["lidar_mount_x_m"]
                y = rng * math.sin(bearing) + desc["lidar_mount_y_m"]
                clearance = min(clearance, math.hypot(x - aim[0], y - aim[1]))
            assert clearance >= 0.6 * r["lane_half_width_m"], (r["stamp_s"], clearance)

    assert math.isfinite(out[0]["filtered_steering_rad"])
    print("gap_follow_replay_cli checks passed")
    return 0


if __name__ == "__main__":
    sys.exit(main())
