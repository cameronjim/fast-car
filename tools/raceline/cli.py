"""CLI entry point: generate a committed raceline file for a track.

Run by a person (or a one-off CI regeneration step), never imported by anything on the
control path. Currently only knows the ``stadium`` synthetic track generator
(``synthetic_tracks.stadium_centerline``) -- the network-free reference track this task
commits one raceline for. A real venue map / occupancy-grid input would be a separate
``--track-mask <path>`` mode built on ``occupancy.py`` when a real map exists.

Usage:
    python -m raceline.cli --track-id gym_oval --straight-length-m 8.0 --turn-radius-m 3.0 \
        --out-dir <repo_root>/config/tracks
"""

from __future__ import annotations

import argparse
import sys
from pathlib import Path

from raceline.io import write_raceline_csv
from raceline.params_loader import load_vehicle_params
from raceline.raceline import build_raceline_from_centerline
from raceline.synthetic_tracks import stadium_centerline

DEFAULT_OUT_DIR = Path("config/tracks")


def main(argv: list[str] | None = None) -> int:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--track-id", required=True, help="e.g. gym_oval (venue_layout)")
    parser.add_argument("--straight-length-m", type=float, default=8.0)
    parser.add_argument("--turn-radius-m", type=float, default=3.0)
    parser.add_argument("--points-per-meter", type=float, default=10.0)
    parser.add_argument("--resample-ds-m", type=float, default=0.1)
    parser.add_argument("--smooth-window", type=int, default=5)
    parser.add_argument("--out-dir", type=Path, default=DEFAULT_OUT_DIR)
    args = parser.parse_args(argv)

    vehicle_params = load_vehicle_params()
    a_max = vehicle_params.actuation.max_acceleration_mps2
    v_max = vehicle_params.limits.global_speed_cap_mps
    if a_max is None or v_max is None:
        print(
            "ERROR: actuation.max_acceleration_mps2 and limits.global_speed_cap_mps must "
            "both be set in config/vehicle_params.yaml (this tool does not invent a "
            "conservative fallback fraction silently; add one explicitly if these ever "
            "go null).",
            file=sys.stderr,
        )
        return 2

    x, y = stadium_centerline(
        args.straight_length_m, args.turn_radius_m, points_per_meter=args.points_per_meter
    )
    result = build_raceline_from_centerline(
        x,
        y,
        a_lat_max_mps2=a_max,
        a_max_mps2=a_max,
        v_max_mps=v_max,
        resample_ds_m=args.resample_ds_m,
        smooth_window=args.smooth_window,
    )

    out_path = args.out_dir / args.track_id / "raceline.csv"
    write_raceline_csv(
        out_path,
        result,
        track_id=args.track_id,
        vehicle_params_schema_version=vehicle_params.meta.schema_version,
        vehicle_params_sysid_session_id=vehicle_params.meta.sysid_session_id,
        generation_params={
            "track_shape": "stadium",
            "straight_length_m": args.straight_length_m,
            "turn_radius_m": args.turn_radius_m,
            "points_per_meter": args.points_per_meter,
            "resample_ds_m": args.resample_ds_m,
            "smooth_window": args.smooth_window,
            "a_lat_max_mps2": a_max,
            "a_max_mps2": a_max,
            "v_max_mps": v_max,
        },
    )
    print(f"Wrote {len(result)} raceline point(s) to {out_path}", file=sys.stderr)
    return 0


if __name__ == "__main__":  # pragma: no cover - exercised via main() directly in tests
    sys.exit(main())
