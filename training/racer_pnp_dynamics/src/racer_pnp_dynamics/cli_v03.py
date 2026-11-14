"""Real-bag entry points of dyn-v0.3 (thin glue, needs the bags; reached through `cli.py`).

    python -m racer_pnp_dynamics.cli mb1      # M-B1: dyn-v0.1 and dyn-v0.1r under T2 and T1

Pre-registration: docs/notes/preregistration-pnp-v0.3.md section 2. The v0.3 constants file is
checked against its pre-registered SHA-256 on every run.
"""

from __future__ import annotations

import hashlib
import logging
import pickle
from pathlib import Path

from . import pipeline_v03, pose_check
from .dataset import by_split, make_segments
from .icp_motion import SEED_IDENTITY, SEED_WHEEL
from .pipeline import fit_vehicle_of
from .recordings import load_motions, load_recordings
from .split import load_split
from .vehicle import load_vehicle, repo_root

LOG = logging.getLogger(__name__)

PREREG_V03_RELATIVE = Path("evaluation") / "prereg" / "pnp-v0.3.yaml"
PREREG_V03_SHA256 = "4415b3719a29cfb08d5f5242ac6cdc5b35fee87e132c6d2939dfa0758808d7a1"
POSE_BAGS = ("2026-11-13T22-00-40_car_teleop", "2026-11-13T23-27-23_car_teleop")


def check_prereg(root: Path) -> None:  # pragma: no cover - real data
    digest = hashlib.sha256((root / PREREG_V03_RELATIVE).read_bytes()).hexdigest()
    if digest != PREREG_V03_SHA256:
        raise ValueError(f"{PREREG_V03_RELATIVE} hashes to {digest}, not the pre-registered one")


def pose_recordings(bags: Path, cache: Path):  # pragma: no cover - real data
    from yaw_rate.bagio import load_recording

    return {n: load_recording(bags / n, cache / f"yaw_rate_{n}.npz") for n in POSE_BAGS}


def segments_for_truth(runs, motions, split):  # pragma: no cover - real data
    return [s for r in runs for s in make_segments(r, motions.get(r.name), split)]


def load_truths(args, split, vehicle):  # pragma: no cover - real data
    """Runs and the segments of both truths (same runs, T1 and T2 motion series)."""
    from .cli import split_bags

    runs, m1, _ = load_recordings(
        args.bags, vehicle, args.cache, args.workers, only=split_bags(split), seed=SEED_IDENTITY
    )
    m2 = load_motions(runs, vehicle, args.cache, SEED_WHEEL, args.workers)
    return runs, segments_for_truth(runs, m1, split), segments_for_truth(runs, m2, split)


def mb1(args) -> int:  # pragma: no cover - real data
    from .cli import V01_BUNDLE, read_pointer_nominal

    root = repo_root()
    check_prereg(root)
    split = load_split(args.split_path)
    vehicle = load_vehicle()
    fvehicle = fit_vehicle_of(vehicle)
    _runs, seg_t1, seg_t2 = load_truths(args, split, vehicle)
    v01 = read_pointer_nominal(root / "training" / "pnp_models" / V01_BUNDLE / "manifest.yaml")
    frozen = pickle.loads((args.cache / "dyn-v0.1r-frozen.pkl").read_bytes())
    models = {
        "dyn-v0.1": (v01.nominal, None),
        f"dyn-v0.1r (v0.2 residual, seed {frozen.shipping.seed})": (
            frozen.nominal,
            frozen.shipping.residual,
        ),
    }
    names = list(models)
    recs = pose_recordings(args.bags, args.cache)
    out = {}
    for truth, segs in ((pipeline_v03.TRUTH_T2, seg_t2), (pipeline_v03.TRUTH_T1, seg_t1)):
        held = by_split(segs, "heldout", odom_only=True)
        poses = pose_check.build_pose_segments(held, recs)
        out[truth] = pipeline_v03.score_models(held, fvehicle, models, truth, poses)
        LOG.info("%s: %d windows", truth, out[truth].windows)
    notes = [
        (
            "dyn-v0.1 nominal read from the committed pointer manifest "
            f"training/pnp_models/{V01_BUNDLE}/manifest.yaml, residual disabled."
        ),
        (
            "The dyn-v0.1r bundle was never exported (it failed GP2a, v0.2 exported passing "
            "bundles only); the same weights are read from the frozen state car-data/pnp/cache/"
            f"dyn-v0.1r-frozen.pkl (config {frozen.shipping.config.name}, seed "
            f"{frozen.shipping.seed}, trained on the dyn-v0.1 nominal)."
        ),
        (
            "Held-out windows differ between T1 and T2 (a failed ICP interval drops a window), "
            "so the T1 block is the v0.2 rule on the current code."
        ),
    ]
    text = pipeline_v03.render_mb1(
        out[pipeline_v03.TRUTH_T2], out[pipeline_v03.TRUTH_T1], notes, tuple(names)
    )
    target = root / "training" / "pnp_models" / f"dyn-v0.3-{args.date}" / "M-B1.md"
    target.parent.mkdir(parents=True, exist_ok=True)
    target.write_text(text, encoding="utf-8")
    LOG.info("wrote %s", target)
    print(text)
    return 0


def main(argv: list[str]) -> int:  # pragma: no cover - real data
    import argparse
    import datetime as dt

    from .cli import DATA_ROOT, MAIN_CHECKOUT_SPLIT
    from .split import default_split_path

    parser = argparse.ArgumentParser(prog="racer_pnp_dynamics.cli")
    parser.add_argument("command", choices=("mb1",))
    parser.add_argument("--date", default=dt.datetime.now(dt.timezone.utc).strftime("%Y-%m-%d"))
    parser.add_argument("--bags", type=Path, default=DATA_ROOT / "bags")
    parser.add_argument("--models", type=Path, default=DATA_ROOT / "pnp" / "models")
    parser.add_argument("--cache", type=Path, default=DATA_ROOT / "pnp" / "cache")
    parser.add_argument("--workers", type=int, default=8)
    args = parser.parse_args(argv)
    logging.basicConfig(level=logging.INFO)
    args.split_path = default_split_path(repo_root())
    if not args.split_path.is_file():
        args.split_path = MAIN_CHECKOUT_SPLIT
    return {"mb1": mb1}[args.command](args)
