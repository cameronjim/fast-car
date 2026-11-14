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


class Context:  # pragma: no cover - real data
    """Everything the fit and evaluate stages share, loaded once."""

    def __init__(self, args) -> None:
        from .cli import V01_BUNDLE, read_pointer_nominal

        self.root = repo_root()
        check_prereg(self.root)
        self.split = load_split(args.split_path)
        self.vehicle = load_vehicle()
        self.fvehicle = fit_vehicle_of(self.vehicle)
        _runs, self.seg_t1, self.seg_t2 = load_truths(args, self.split, self.vehicle)
        self.v01 = read_pointer_nominal(
            self.root / "training" / "pnp_models" / V01_BUNDLE / "manifest.yaml"
        )
        self.recs = pose_recordings(args.bags, args.cache)

    def heldout(self):
        t2 = by_split(self.seg_t2, "heldout", odom_only=True)
        t1 = by_split(self.seg_t1, "heldout", odom_only=True)
        return (
            t2,
            t1,
            pose_check.build_pose_segments(t2, self.recs),
            pose_check.build_pose_segments(t1, self.recs),
        )

    def git_sha(self) -> str:
        import subprocess

        return subprocess.run(
            ["git", "-C", str(self.root), "rev-parse", "HEAD"],
            capture_output=True,
            text=True,
            check=True,
        ).stdout.strip()


def export_candidate(  # pragma: no cover - real data
    ctx: Context,
    args,
    model_id: str,
    nominal,
    residual,
    v_max_mps: float,
    metrics: dict,
    readme: str,
    data_lines: list[str],
    png=None,
):
    """The bundle under car-data/pnp/models/ (golden rollouts included) and the pointer directory
    under training/pnp_models/ (README, metrics, manifest, plots). Exported whether the gate
    passed or not: a failed candidate carries `do-not-deploy` on the first line of its README."""
    import datetime as dt
    import json
    import shutil

    from . import export
    from .pipeline import _json

    metrics = _json(metrics)
    bundle_dir = args.models / model_id
    pointer = ctx.root / "training" / "pnp_models" / model_id
    export.write_bundle(
        bundle_dir,
        model_id=model_id,
        created_utc=dt.datetime.now(dt.timezone.utc).strftime("%Y-%m-%dT%H:%M:%SZ"),
        git_sha=ctx.git_sha(),
        training_config_sha256=export.config_sha256(
            {"model_id": model_id, "nominal": nominal.to_dict(), "prereg": PREREG_V03_SHA256}
        ),
        vehicle=ctx.vehicle,
        nominal=nominal,
        residual=residual,
        v_max_mps=v_max_mps,
        data=data_lines,
        metrics=metrics,
        golden_residual_rollouts=bool(residual.enabled),
    )
    (bundle_dir / "README.md").write_text(readme, encoding="utf-8")
    pointer.mkdir(parents=True, exist_ok=True)
    shutil.copy(bundle_dir / export.MANIFEST_FILE, pointer / export.MANIFEST_FILE)
    (pointer / "metrics.json").write_text(json.dumps(metrics, indent=2), encoding="utf-8")
    (pointer / "README.md").write_text(readme, encoding="utf-8")
    if png is not None:
        shutil.copy(png, pointer / "profiles.png")
    LOG.info("wrote %s and %s", bundle_dir, pointer)


def y_fit(args) -> int:  # pragma: no cover - real data
    """Stage 1 of candidate Y: the refit on train data only; prints validation numbers only."""
    ctx = Context(args)
    refit = pipeline_v03.yaw_refit_fit(ctx.seg_t2, ctx.fvehicle, ctx.v01.nominal)
    (args.cache / "dyn-v0.3y-frozen.pkl").write_bytes(pickle.dumps(refit))
    LOG.info("Y nominal %s", {n: getattr(refit.nominal, n) for n in refit.free})
    LOG.info("dyn-v0.1 %s", {n: getattr(refit.base, n) for n in refit.free})
    LOG.info("on a bound %s; unidentified %s", refit.on_bound, refit.unidentified)
    LOG.info("validation yaw RMSE %s; counts %s", refit.yaw_rmse_val, refit.counts)
    return 0


def y_evaluate(args) -> int:  # pragma: no cover - real data
    """Stage 2 of candidate Y: held-out scored once, GP2c-Y decided, bundle and pointer written."""
    import tempfile
    from dataclasses import asdict

    from . import report_v03

    ctx = Context(args)
    refit = pickle.loads((args.cache / "dyn-v0.3y-frozen.pkl").read_bytes())
    held_t2, held_t1, poses_t2, poses_t1 = ctx.heldout()
    ev = pipeline_v03.evaluate_y(refit, held_t2, held_t1, ctx.fvehicle, poses_t2, poses_t1)
    model_id = f"dyn-v0.3y-{args.date}"
    metrics = {
        "split_sha256": ctx.split.sha256,
        "vehicle_params": f"{ctx.vehicle.schema_version} / {ctx.vehicle.sysid_session_id}",
        "refit": {
            **{k: v for k, v in asdict(refit).items() if k not in ("base", "nominal")},
            "base": refit.base.to_dict(),
            "nominal": refit.nominal.to_dict(),
        },
        "heldout": ev,
    }
    readme = report_v03.render_y_report(model_id, metrics)
    png = Path(tempfile.mkdtemp()) / "profiles.png"
    pipeline_v03.plot_yaw_profiles(png, refit, f"{model_id}: yaw parameter profiles")
    from .residual_train import disabled_residual

    export_candidate(
        ctx, args, model_id, refit.nominal, disabled_residual(), ctx.v01.v_max_mps, metrics,
        readme, [f"yaw refit of dyn-v0.1 on the train role under T2, split {ctx.split.sha256}"],
        png,
    )  # fmt: skip
    print(readme)
    return 0


def r_fit(args) -> int:  # pragma: no cover - real data
    """Stage 1 of candidate R: the base nominal is decided from the recorded GP2c-Y result BEFORE
    any residual is trained and written down; then the candidates and seeds are trained on train
    data only and the config and shipping seed are chosen on validation P1."""
    import json
    from dataclasses import replace

    from . import pipeline_v02
    from .residual_train import DEFAULT_CANDIDATES

    ctx = Context(args)
    y_id = f"dyn-v0.3y-{args.date}"
    y_metrics = json.loads(
        (ctx.root / "training" / "pnp_models" / y_id / "metrics.json").read_text(encoding="utf-8")
    )
    y_passed = bool(y_metrics["heldout"]["passes"])
    refit = pickle.loads((args.cache / "dyn-v0.3y-frozen.pkl").read_bytes())
    if y_passed:
        name, nominal = "Y (dyn-v0.3y)", refit.nominal
    else:
        name, nominal = pipeline_v03.BASE_NAME, ctx.v01.nominal
    addendum = (
        f"Base nominal decided before any residual was trained: GP2c-Y "
        f"{'PASSED' if y_passed else 'FAILED'} (pointer {y_id}), so the residual is trained on "
        f"top of {name}."
    )
    LOG.info(addendum)
    configs = tuple(replace(c, name=f"yaw_{c.name}", yaw_only=True) for c in DEFAULT_CANDIDATES)
    frozen = pipeline_v02.strict_fit_stage(
        ctx.seg_t2, ctx.vehicle, nominal, ctx.v01.v_max_mps, configs=configs
    )
    (args.cache / "dyn-v0.3yr-frozen.pkl").write_bytes(
        pickle.dumps({"frozen": frozen, "nominal_name": name, "addenda": [addendum]})
    )
    LOG.info("validation selection: %s", frozen.selection)
    LOG.info("shipping %s seed %d", frozen.shipping.config.name, frozen.shipping.seed)
    return 0


def r_evaluate(args) -> int:  # pragma: no cover - real data
    from . import report_v03

    ctx = Context(args)
    state = pickle.loads((args.cache / "dyn-v0.3yr-frozen.pkl").read_bytes())
    frozen = state["frozen"]
    held_t2, held_t1, poses_t2, poses_t1 = ctx.heldout()
    ev = pipeline_v03.evaluate_r(
        state["nominal_name"], frozen.nominal, frozen.candidates, frozen.shipping,
        held_t2, held_t1, ctx.fvehicle, ctx.vehicle, frozen.v_max_mps, poses_t2, poses_t1,
    )  # fmt: skip
    model_id = f"dyn-v0.3yr-{args.date}"
    metrics = {
        "split_sha256": ctx.split.sha256,
        "vehicle_params": f"{ctx.vehicle.schema_version} / {ctx.vehicle.sysid_session_id}",
        "addenda": state["addenda"],
        "selection": frozen.selection,
        "heldout": ev,
    }
    readme = report_v03.render_r_report(model_id, metrics)
    export_candidate(
        ctx, args, model_id, frozen.nominal, frozen.shipping.residual, frozen.v_max_mps, metrics,
        readme, [f"yaw-only residual on {state['nominal_name']}, train role under T2"],
    )  # fmt: skip
    print(readme)
    return 0


def confirm_cmd(args) -> int:  # pragma: no cover - real data
    """The fresh confirmation of pre-registration 2.4 on a new session bag."""
    from . import confirm

    if args.bag is None or args.candidate is None or args.deployed is None:
        raise SystemExit("confirm needs --bag, --candidate and --deployed")
    result = confirm.run_confirmation(
        args.bag, args.candidate, args.deployed, args.cache, args.workers
    )
    print(confirm.format_confirmation(result, args.candidate.name, args.deployed.name))
    return 0 if result.verdict == "PASS" else 1


COMMANDS = {
    "mb1": mb1,
    "confirm": confirm_cmd,
    "y-fit": y_fit,
    "y-evaluate": y_evaluate,
    "r-fit": r_fit,
    "r-evaluate": r_evaluate,
}


def main(argv: list[str]) -> int:  # pragma: no cover - real data
    import argparse
    import datetime as dt

    from .cli import DATA_ROOT, MAIN_CHECKOUT_SPLIT
    from .split import default_split_path

    parser = argparse.ArgumentParser(prog="racer_pnp_dynamics.cli")
    parser.add_argument("command", choices=tuple(COMMANDS))
    parser.add_argument("--date", default=dt.datetime.now(dt.timezone.utc).strftime("%Y-%m-%d"))
    parser.add_argument("--bags", type=Path, default=DATA_ROOT / "bags")
    parser.add_argument("--models", type=Path, default=DATA_ROOT / "pnp" / "models")
    parser.add_argument("--cache", type=Path, default=DATA_ROOT / "pnp" / "cache")
    parser.add_argument("--workers", type=int, default=8)
    parser.add_argument("--bag", type=Path, default=None, help="confirm: the session bag")
    parser.add_argument("--candidate", type=Path, default=None, help="confirm: candidate bundle")
    parser.add_argument("--deployed", type=Path, default=None, help="confirm: deployed bundle")
    args = parser.parse_args(argv)
    logging.basicConfig(level=logging.INFO)
    args.split_path = default_split_path(repo_root())
    if not args.split_path.is_file():
        args.split_path = MAIN_CHECKOUT_SPLIT
    return COMMANDS[args.command](args)
