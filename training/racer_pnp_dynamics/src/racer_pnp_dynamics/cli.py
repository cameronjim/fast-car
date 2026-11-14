"""Real-bag entry point of dyn-v0.2 (thin glue, needs the bags, torch for the residual and
matplotlib for the profile plot). Two stages, so the held-out segments are not read before the
nominal fit, the command delay, the validity speed and the shipping seed are frozen:

    uv sync --group train
    uv run --with matplotlib python -m racer_pnp_dynamics.cli --stage fit
    uv run --with matplotlib python -m racer_pnp_dynamics.cli --stage evaluate

`--stage fit` writes the frozen state (pickle, under the cache directory) and prints validation
numbers only. `--stage evaluate` loads it, scores the held-out segments as pre-registered
(docs/notes/preregistration-pnp-v0.2.md section 3), decides GP2, and writes the bundle and the
pointer. The split file is checked against its pre-registered SHA-256 on every load.
"""

from __future__ import annotations

import argparse
import datetime as dt
import logging
import pickle
import subprocess
from pathlib import Path

from . import export, pipeline_v02
from .dataset import by_split, make_segments
from .recordings import load_recordings
from .split import Split, default_split_path, load_split
from .vehicle import load_vehicle, repo_root

DATA_ROOT = Path("/Users/cameronjim/code/car-data")
RUNS_BAG = "2026-11-13T22-20-18_car_teleop"  # R0 to R8
V01_BUNDLE = "dyn-v0.1-2026-11-12"
MAIN_CHECKOUT_SPLIT = Path("/Users/cameronjim/code/car/evaluation/prereg/pnp-v0.2-split.yaml")
LOG = logging.getLogger(__name__)


def split_bags(split: Split) -> set[str]:
    """Bags with something for the dynamics workstream."""
    return {
        e.name
        for e in split.bags.values()
        if e.whole_bag_role not in (None, "unused", "heldout_layout") or e.segments
    }


def main(argv: list[str] | None = None) -> int:  # pragma: no cover - real data
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--stage", choices=("fit", "evaluate"), required=True)
    parser.add_argument("--date", default=dt.datetime.now(dt.timezone.utc).strftime("%Y-%m-%d"))
    parser.add_argument("--bags", type=Path, default=DATA_ROOT / "bags")
    parser.add_argument("--models", type=Path, default=DATA_ROOT / "pnp" / "models")
    parser.add_argument("--cache", type=Path, default=DATA_ROOT / "pnp" / "cache")
    parser.add_argument("--split", type=Path, default=None)
    parser.add_argument("--workers", type=int, default=8)
    args = parser.parse_args(argv)
    logging.basicConfig(level=logging.INFO)

    root = repo_root()
    split_path = args.split or default_split_path(root)
    if not split_path.is_file():  # the file is untracked in the main checkout until its PR lands
        split_path = MAIN_CHECKOUT_SPLIT
    split = load_split(split_path)
    vehicle = load_vehicle()
    runs, motions, _ = load_recordings(
        args.bags, vehicle, args.cache, args.workers, only=split_bags(split)
    )
    segments = [s for r in runs for s in make_segments(r, motions.get(r.name), split)]
    checked = pipeline_v02.check_segments_against_split(segments, split)
    LOG.info("%d segments checked against the split file", checked)
    table = pipeline_v02.data_table(segments)
    LOG.info("seconds by bag: %s", table["seconds_by_bag"])
    frozen_path = args.cache / "dyn-v0.2-frozen.pkl"
    if args.stage == "fit":
        frozen = pipeline_v02.fit_stage(segments, vehicle)
        frozen_path.write_bytes(pickle.dumps(frozen))
        LOG.info("nominal %s", frozen.nominal)
        LOG.info("validity %s", frozen.validity["v_max_mps"])
        LOG.info("residual selection on validation P1: %s", frozen.selection)
        LOG.info("frozen state written to %s", frozen_path)
        return 0

    frozen = pickle.loads(frozen_path.read_bytes())
    model_id = f"dyn-v0.2-{args.date}"
    fvehicle = pipeline_v02.fit_vehicle_of(vehicle)
    v01 = export.read_bundle(args.models / V01_BUNDLE)
    heldout = by_split(segments, "heldout", odom_only=True)
    report = pipeline_v02.evaluate_heldout(
        frozen, heldout, fvehicle, vehicle, (v01.nominal, v01.residual), V01_BUNDLE
    )
    ship = frozen.shipping
    variants = {
        V01_BUNDLE: (v01.nominal, None),
        "dyn-v0.2 nominal": (frozen.nominal, None),
        f"dyn-v0.2 + residual seed {ship.seed}": (frozen.nominal, ship.residual),
    }
    legacy = pipeline_v02.legacy_report(by_split(segments, "legacy_heldout"), fvehicle, variants)
    runs_table = pipeline_v02.run_table(segments, RUNS_BAG, variants, fvehicle)
    metrics = pipeline_v02.build_metrics(
        frozen,
        report,
        split=split,
        segments=segments,
        vehicle=vehicle,
        fvehicle=fvehicle,
        v01_nominal=v01.nominal,
        v01_id=V01_BUNDLE,
        legacy=legacy,
        runs_table=runs_table,
    )
    git_sha = subprocess.run(
        ["git", "-C", str(root), "rev-parse", "HEAD"], capture_output=True, text=True, check=True
    ).stdout.strip()
    data_lines = [f"bag {r.name}: {r.duration_s:.0f} s, odometry {'yes' if r.has_odom else 'no'}"
                  for r in runs]  # fmt: skip
    pipeline_v02.write_outputs(
        frozen,
        metrics,
        vehicle,
        model_id=model_id,
        created_utc=dt.datetime.now(dt.timezone.utc).strftime("%Y-%m-%dT%H:%M:%SZ"),
        git_sha=git_sha,
        bundle_dir=args.models / model_id,
        pointer_dir=root / "training" / "pnp_models" / model_id,
        data_lines=data_lines,
        profile_png=True,
    )
    LOG.info("wrote %s", args.models / model_id)
    return 0


if __name__ == "__main__":  # pragma: no cover
    raise SystemExit(main())
