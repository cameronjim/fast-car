#!/usr/bin/env python3
"""New track session: one command per step, from a mapping run to a raceline the car can follow.

  1. map       a mapping run -> saved map. Offline from a recorded teleop bag (in the ros-dev image:
               extract, replay through the car's own EKF + slam_toolbox, save), or live on the car
               (`save-map`, after driving a lap with car_teleop.launch.py mapping:=true).
  2. analyze   map -> lane width, island, end radii, centreline (JSON + CSV).
  3. raceline  map -> racing line for the car's real turning limits (both directions), speed
               profile, CSV for tracker_node, a JSON report and a PNG; exit 2 if the line is not
               feasible (curvature or body clearance broken), so a script can stop.
  4. tracker   prints the two launch commands for the car (the localisation stack on the map and
               tracker_node on the raceline), with the checklist.

    tools/track_session/track_session.py map /data/bags/<bag> --name oval --data ~/code/car-data
    tools/track_session/track_session.py raceline ~/code/car-data/maps/oval/oval.yaml --v-max 0.64
    tools/track_session/track_session.py tracker ~/code/car-data/maps/oval/oval_ccw.csv \\
        ~/code/car-data/maps/oval/oval.yaml

`map` and the live `save-map` need the ROS side (docker and ros-dev, or ros2 on the car); the rest
needs only the tools venv (`uv run --project tools python tools/track_session/track_session.py ...`).
Physical constants, steering limits and speed scale included, come from vehicle_params. See docs/notes/first-boot-runbook.md,
"New track session".
"""

from __future__ import annotations

import argparse
import json
import subprocess
import sys
from pathlib import Path

HERE = Path(__file__).resolve().parent
REPO_ROOT = HERE.parents[1]
sys.path.insert(0, str(HERE))

import pipeline

AUTO_CLEARANCES = (0.15, 0.12, 0.10, 0.08, 0.06, 0.05)


def cmd_map(args: argparse.Namespace) -> int:
    data = Path(args.data).expanduser().resolve()
    out_dir = (
        Path(args.out_dir).expanduser().resolve() if args.out_dir else data / "maps" / args.name
    )
    try:
        container_prefix = "/data/" + str((out_dir / args.name).relative_to(data))
    except ValueError:
        print(
            f"the output ({out_dir}) must be under --data ({data}): the container mounts it",
            file=sys.stderr,
        )
        return 2
    out_dir.mkdir(parents=True, exist_ok=True)
    command = [
        str(HERE / "map_from_bag.sh"),
        str(Path(args.input).expanduser().resolve()),
        container_prefix,
        "--speed-divisor",
        str(args.speed_divisor),
        "--steer-scale-left",
        str(args.steer_scale_left),
        "--steer-scale-right",
        str(args.steer_scale_right),
        "--deskew",
        args.deskew,
        "--twist-source",
        args.twist_source,
        "--rate",
        str(args.rate),
    ]
    env = {"DATA": str(data)}
    import os

    status = subprocess.run(command, env={**os.environ, **env}, check=False).returncode
    yaml_path = out_dir / f"{args.name}.yaml"
    if status == 0 and yaml_path.is_file():
        print(f"map saved: {yaml_path}")
        return 0
    print(f"mapping failed (status {status}); see the feeder output above", file=sys.stderr)
    return status or 1


def cmd_save_map(args: argparse.Namespace) -> int:
    """On the car: the service calls the runbook gives, as one command."""
    prefix = str(Path(args.out).expanduser().resolve())
    Path(prefix).parent.mkdir(parents=True, exist_ok=True)
    calls = [
        ["slam_toolbox/srv/SaveMap", "/slam_toolbox/save_map", f"{{name: {{data: '{prefix}'}}}}"],
        [
            "slam_toolbox/srv/SerializePoseGraph",
            "/slam_toolbox/serialize_map",
            f"{{filename: '{prefix}'}}",
        ],
    ]
    for srv_type, name, request in calls:
        status = subprocess.run(
            ["ros2", "service", "call", name, srv_type, request], check=False
        ).returncode
        if status != 0:
            return status
    print(f"map saved: {prefix}.yaml")
    return 0


def _read_map_args(args: argparse.Namespace):
    return Path(args.map).expanduser().resolve()


def cmd_analyze(args: argparse.Namespace) -> int:
    from raceline import map_track as mt

    yaml_path = _read_map_args(args)
    grid = mt.load_map(yaml_path)
    seed = (args.seed_x, args.seed_y)
    cx, cy = mt.trace_centreline(grid, seed)
    report = mt.lane_report(grid, cx, cy).as_dict()
    out_dir = Path(args.out_dir).expanduser().resolve() if args.out_dir else yaml_path.parent
    out_dir.mkdir(parents=True, exist_ok=True)
    pipeline.write_centreline_csv(out_dir / f"{yaml_path.stem}_centreline.csv", cx, cy)
    (out_dir / f"{yaml_path.stem}_lane.json").write_text(json.dumps(report, indent=2) + "\n")
    print(json.dumps(report, indent=2))
    return 0


def cmd_raceline(args: argparse.Namespace) -> int:
    yaml_path = _read_map_args(args)
    out_dir = Path(args.out_dir).expanduser().resolve() if args.out_dir else yaml_path.parent
    out_dir.mkdir(parents=True, exist_ok=True)
    directions = ["ccw", "cw"] if args.direction == "both" else [args.direction]
    status = 0
    for direction in directions:
        out_csv = out_dir / f"{yaml_path.stem}_{direction}.csv"
        result = pipeline.raceline_from_map(
            yaml_path,
            out_csv,
            direction=direction,
            seed_xy=(args.seed_x, args.seed_y),
            margin=args.margin,
            body_clearance_m=args.body_clearance or 0.15,
            clearance_candidates=None if args.body_clearance else AUTO_CLEARANCES,
            v_max_mps=args.v_max,
            a_lat_mps2=args.a_lat,
            a_long_mps2=args.a_long,
            front_overhang_m=args.front_overhang,
            rear_overhang_m=args.rear_overhang,
            centre_only=args.centre_only,
            centre_csv=out_dir / f"{yaml_path.stem}_centreline.csv",
        )
        report = result.report
        ok = pipeline.feasible(report)
        print(
            f"{direction}: {out_csv.name}  length {report['length_m']} m, min radius "
            f"left {report['line_min_radius_left_m']} m right {report['line_min_radius_right_m']} m "
            f"(limits {1 / report['curvature_limit_left_1pm']:.2f} / "
            f"{1 / report['curvature_limit_right_1pm']:.2f} m with the margin), body clearance "
            f"{report['min_body_clearance_m']} m (asked {report['body_clearance_used_m']}), offset from centre up to "
            f"{report['max_offset_from_centre_m']} m: {'OK' if ok else 'NOT FEASIBLE'}"
        )
        if not ok:
            status = 2
        if not args.no_png:
            try:
                _plot(yaml_path, result, out_csv.with_suffix(".png"))
            except ImportError:
                print("(matplotlib not installed: no PNG; uv sync --extra plots in tools/)")
    if status:
        print(
            "The line breaks a limit. Options: a lower --margin, a smaller --body-clearance, or "
            "check the map for a wall that closes the lane.",
            file=sys.stderr,
        )
    return status


def _plot(yaml_path: Path, result, out_png: Path) -> None:
    import matplotlib

    matplotlib.use("Agg")
    import matplotlib.pyplot as plt
    import numpy as np
    from raceline import map_track as mt

    grid = mt.load_map(yaml_path)
    h, w = grid.shape
    extent = [
        grid.origin_xy[0],
        grid.origin_xy[0] + w * grid.resolution_m,
        grid.origin_xy[1],
        grid.origin_xy[1] + h * grid.resolution_m,
    ]
    image = np.where(grid.occupied, 0, np.where(grid.free, 254, 205))
    car = pipeline.car_geometry()
    fig, ax = plt.subplots(figsize=(7, 5))
    ax.imshow(np.flipud(image), cmap="gray", extent=extent, origin="upper")
    ax.plot(*result.centre_xy, "b-", lw=1, label="centreline")
    ax.plot(result.line.x_m, result.line.y_m, "r-", lw=1.5, label="racing line")
    x, y = result.line.x_m, result.line.y_m
    heading = np.arctan2(np.roll(y, -1) - np.roll(y, 1), np.roll(x, -1) - np.roll(x, 1))
    outline = car.perimeter()
    for i in range(0, len(x), max(len(x) // 24, 1)):
        c, s = np.cos(heading[i]), np.sin(heading[i])
        ax.plot(
            x[i] + c * outline[:, 0] - s * outline[:, 1],
            y[i] + s * outline[:, 0] + c * outline[:, 1],
            "g.",
            ms=1,
        )
    ax.set_aspect("equal")
    ax.legend(loc="upper right", fontsize=7)
    fig.savefig(out_png, dpi=90)
    plt.close(fig)


def cmd_tracker(args: argparse.Namespace) -> int:
    raceline = Path(args.raceline).expanduser().resolve()
    map_yaml = Path(args.map).expanduser().resolve()
    print(f"""\
Checklist (docs/notes/first-boot-runbook.md, "New track session"):
  * the track is built as it was when it was mapped; the lane is clear, behind the car too
  * the RC kill switch is armed and held by a second person; first run slow (the raceline's speeds)
  * put the car where mapping started, facing the same way (the map origin), or set the pose in
    Foxglove (/initialpose) once the localisation is up

Terminal 1 (car container), the sensors, the safety layer, the recorder and AMCL on this map:
  ros2 launch racer_bringup car_teleop.launch.py lidar:=true vesc:=true \\
      localization:=true map:={map_yaml}

Terminal 2, the tracker:
  ros2 launch racer_control track_follow.launch.py raceline:={raceline}

Stop: the RC kill switch, then Ctrl-C both. Every run is in the bag Terminal 1 records.""")
    return 0


def main(argv: list[str] | None = None) -> int:
    parser = argparse.ArgumentParser(
        description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter
    )
    sub = parser.add_subparsers(dest="cmd", required=True)

    p = sub.add_parser("map", help="bag -> saved map (offline, in the ros-dev image)")
    p.add_argument("input", help="a bag directory or .mcap, or an extract_bag.py .npz")
    p.add_argument("--name", required=True, help="track id: maps/<name>/<name>.yaml")
    p.add_argument("--data", default="~/code/car-data", help="data tree, mounted at /data")
    p.add_argument("--out-dir", help="default <data>/maps/<name>")
    p.add_argument(
        "--speed-divisor",
        type=float,
        default=1.0,
        help="divide the bag's /odom/wheel speeds by this: 1.0 for a bag from the calibrated car "
        "(vehicle_params 0.14.0 on); 1.401 for the 2026-11-10 bags",
    )
    p.add_argument(
        "--steer-scale-left",
        type=float,
        default=1.0,
        help="true wheel angle per radian of left command in the bag: 1.0 calibrated, 1.203 for "
        "the 2026-11-10 bags",
    )
    p.add_argument(
        "--steer-scale-right", type=float, default=1.0, help="as left; 1.072 for 2026-11-10"
    )
    p.add_argument(
        "--deskew",
        default="end_rev",
        choices=("none", "start_fwd", "start_rev", "end_fwd", "end_rev"),
    )
    p.add_argument("--twist-source", default="feeder", choices=("feeder", "node"))
    p.add_argument("--rate", type=float, default=1.0)
    p.set_defaults(func=cmd_map)

    p = sub.add_parser("save-map", help="on the car: save the live slam_toolbox map")
    p.add_argument("--out", required=True, help="path without extension")
    p.set_defaults(func=cmd_save_map)

    def map_args(sp: argparse.ArgumentParser) -> None:
        sp.add_argument("map", help="map_server yaml")
        sp.add_argument("--out-dir", help="default: next to the map")
        sp.add_argument(
            "--seed-x", type=float, default=0.0, help="a point on the lane (map origin)"
        )
        sp.add_argument("--seed-y", type=float, default=0.0)

    p = sub.add_parser("analyze", help="map -> lane report and centreline")
    map_args(p)
    p.set_defaults(func=cmd_analyze)

    p = sub.add_parser("raceline", help="map -> racing line CSV (both directions)")
    map_args(p)
    p.add_argument("--direction", default="both", choices=("both", "ccw", "cw"))
    p.add_argument("--v-max", type=float, default=0.64, help="top TRUE speed, m/s")
    p.add_argument(
        "--a-lat", type=float, default=2.0, help="lateral acceleration for the speed profile"
    )
    p.add_argument("--a-long", type=float, default=1.0)
    p.add_argument(
        "--margin", type=float, default=0.85, help="share of the steering's curvature to use"
    )
    p.add_argument(
        "--body-clearance",
        type=float,
        default=None,
        help="body outline to wall, m; default: the widest of 0.15 0.12 0.10 0.08 0.06 0.05 the "
        "track allows",
    )
    p.add_argument("--front-overhang", type=float, default=None)
    p.add_argument("--rear-overhang", type=float, default=None)
    p.add_argument("--centre-only", action="store_true", help="the centreline itself, no offset")
    p.add_argument("--no-png", action="store_true")
    p.set_defaults(func=cmd_raceline)

    p = sub.add_parser("tracker", help="print the car's launch commands")
    p.add_argument("raceline")
    p.add_argument("map")
    p.set_defaults(func=cmd_tracker)

    args = parser.parse_args(argv)
    return args.func(args)


if __name__ == "__main__":
    sys.exit(main())
