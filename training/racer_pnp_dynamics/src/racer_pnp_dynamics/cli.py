"""Real-bag entry point (thin glue, needs the bags, torch for the residual and matplotlib for
the profile plot):

uv sync --group train
uv run --with matplotlib python -m racer_pnp_dynamics.cli --date 2026-11-12
"""

from __future__ import annotations

import argparse
import datetime as dt
import logging
import subprocess
from pathlib import Path

from .dataset import by_split
from .pipeline import PnpRuns, run_fit_and_export
from .pnp_eval import read_legacy_bundle
from .recordings import all_segments, load_recordings
from .vehicle import load_vehicle, repo_root

DATA_ROOT = Path("/Users/cameronjim/code/car-data")
PNP_BAG = "2026-11-12T03-04-56_car_teleop"
OLD_BUNDLE = "dyn-v0-2026-11-12"


def main(argv: list[str] | None = None) -> int:  # pragma: no cover - real data
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--date", default=dt.datetime.now(dt.timezone.utc).strftime("%Y-%m-%d"))
    parser.add_argument("--bags", type=Path, default=DATA_ROOT / "bags")
    parser.add_argument("--models", type=Path, default=DATA_ROOT / "pnp" / "models")
    parser.add_argument("--cache", type=Path, default=DATA_ROOT / "pnp" / "cache")
    parser.add_argument("--workers", type=int, default=8)
    args = parser.parse_args(argv)
    logging.basicConfig(level=logging.INFO)

    root = repo_root()
    vehicle = load_vehicle()
    runs, motions, dropped = load_recordings(args.bags, vehicle, args.cache, args.workers)
    segments = all_segments(runs, motions)
    model_id = f"dyn-v0.1-{args.date}"
    pnp_run = next((r for r in runs if r.name == PNP_BAG), None)
    pnp = (
        PnpRuns(
            pnp_run,
            motions.get(PNP_BAG),
            pnp_run.scans[0].log_time_s,
            read_legacy_bundle(args.models / OLD_BUNDLE),
        )
        if pnp_run is not None and pnp_run.scans and (args.models / OLD_BUNDLE).is_dir()
        else None
    )
    git_sha = subprocess.run(
        ["git", "-C", str(root), "rev-parse", "HEAD"], capture_output=True, text=True, check=True
    ).stdout.strip()
    data_lines = [
        f"bag {r.name}: {r.duration_s:.0f} s, odometry {'yes' if r.has_odom else 'no'}"
        for r in runs
    ] + [f"dropped as a nested copy of a longer bag: {name}" for name in dropped]
    run_fit_and_export(
        segments,
        vehicle,
        model_id=model_id,
        created_utc=dt.datetime.now(dt.timezone.utc).strftime("%Y-%m-%dT%H:%M:%SZ"),
        git_sha=git_sha,
        bundle_dir=args.models / model_id,
        pointer_dir=root / "training" / "pnp_models" / model_id,
        data_lines=data_lines,
        pnp=pnp,
        profile_png=True,
        notes={
            "train_segments": len(by_split(segments, "train")),
            "dropped_nested_bags": dropped,
        },
    )
    return 0


if __name__ == "__main__":  # pragma: no cover
    raise SystemExit(main())
