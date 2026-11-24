"""Real-bag stages of the prototype bundle (`proto.py`; workstream W1 of 2026-11-23).

    uv run --group train python -m racer_pnp_dynamics.stages_proto split
    uv run --group train python -m racer_pnp_dynamics.stages_proto truth --workers 4
    uv run --group train python -m racer_pnp_dynamics.stages_proto fit
    uv run --group train python -m racer_pnp_dynamics.stages_proto dev-eval   # after the fit commit
    uv run --group train python -m racer_pnp_dynamics.stages_proto export

`split` writes the role file into the pointer directory (commit it before `fit`). `truth` computes
T2 of every part of the newer bags (the 2026-11-15 session reuses its v0.5 cache). `fit` trains on
the train parts, chooses on the validation parts, and pickles the result. `dev-eval` scores the
development parts once and writes the table. `export` writes the bundle under car-data/pnp/models
and the pointer (manifest, README, metrics) under training/pnp_models.
"""

from __future__ import annotations

import datetime as dt
import json
import logging
import pickle
import subprocess
from pathlib import Path

import numpy as np

from . import proto

LOG = logging.getLogger(__name__)

DATA_ROOT = Path("/Users/cameronjim/code/car-data")
SEEDS = (0, 1, 2)
V_MAX_MPS = 1.3  # the planner validity range (dyn-v0.1), not widened by a prototype
V01_BUNDLE = "dyn-v0.1-2026-11-12"

# Where each session's bags are on this machine.
BAG_DIRS = {
    "2026-11-16T02-18-54_car_teleop": DATA_ROOT / "bags",
    "2026-11-16T02-29-27_car_teleop": DATA_ROOT / "bags",
    "2026-11-16T22-38-47_car_teleop": DATA_ROOT / "bags",
    "2026-11-16T22-45-39_car_teleop": DATA_ROOT / "bags",
    "2026-11-16T23-04-09_car_teleop": DATA_ROOT / "bags",
    "2026-11-23T03-50-32_car_teleop": DATA_ROOT / "session-2026-11-22",
    "2026-11-24T04-36-54_car_teleop": DATA_ROOT / "session-2026-11-23",
}
V05_BAGS = frozenset(b for b in BAG_DIRS if b.startswith("2026-11-16"))
SESSION_OF = {b: ("2026-11-15" if b in V05_BAGS else b[:10]) for b in BAG_DIRS}


def _configs():
    """Bounded configs only: every candidate's output is capped by its weights (|dv/dt| <= 2.0
    m/s2, |dyaw/dt| <= 0.5 rad/s), so the planner can never see a residual above that."""
    from .residual_train import MULTI_STEP, MULTI_STEP_FROM_ONE, ONE_STEP, ResidualConfig

    cap = (2.0, 0.5)
    return (
        ResidualConfig("one_step_capped_wd1e-2", ONE_STEP, 1e-2, out_cap=cap),
        ResidualConfig("multi_step_capped_wd1e-2", MULTI_STEP, 1e-2, out_cap=cap),
        ResidualConfig("multi_step_from_one_capped_wd1e-2", MULTI_STEP_FROM_ONE, 1e-2, out_cap=cap),
    )


def git(root: Path, *args: str) -> str:  # pragma: no cover - real checkout
    return subprocess.run(
        ["git", "-C", str(root), *args], capture_output=True, text=True, check=True
    ).stdout.strip()


def pointer_dir(root: Path) -> Path:
    return root / "training" / "pnp_models" / proto.MODEL_ID


def runs_tables(cache: Path) -> tuple[dict[str, list[dict]], dict[str, float]]:  # pragma: no cover
    runs, first = {}, {}
    for bag in BAG_DIRS:
        path = cache / f"{bag}.runs-v05.json"
        if not path.is_file():
            raise SystemExit(
                f"{path} is missing: `uv run --project tools --extra session python "
                f"tools/pnp_data.py runs-v05 <bag> --json --cache {cache} > {path}`"
            )
        doc = json.loads(path.read_text())
        runs[bag] = doc["runs"]
        first[bag] = float(doc["first_scan_log_time_s"])
    return runs, first


def split_cmd(args) -> int:  # pragma: no cover - real data
    from .vehicle import repo_root

    runs, first = runs_tables(args.cache)
    parts = proto.plan_parts(runs)
    proto.check_roles(parts)
    out = pointer_dir(repo_root())
    out.mkdir(parents=True, exist_ok=True)
    (out / "dev-split.yaml").write_text(proto.split_document(parts, first), encoding="utf-8")
    for role in proto.ROLES:
        s = sum(p.duration_s for p in parts if p.role == role)
        LOG.info("%s: %d parts, %.1f s", role, sum(p.role == role for p in parts), s)
    return 0


def load_parts(root: Path) -> list[proto.RunPart]:  # pragma: no cover - real checkout
    """The committed role file, read back (the fit refuses an uncommitted one)."""
    import yaml

    path = pointer_dir(root) / "dev-split.yaml"
    status = git(root, "status", "--porcelain", "--", str(path))
    if status or not git(root, "ls-files", "--", str(path)):
        raise SystemExit(f"{path} must be committed, unchanged, before any fit or score")
    doc = yaml.safe_load(path.read_text())
    parts = [
        proto.RunPart(bag, str(p["run"]), p["role"], float(p["t_from_s"]), float(p["t_to_s"]))
        for bag, entry in doc["bags"].items()
        for p in entry["parts"]
    ]
    proto.check_roles(parts)
    return parts


def _part_path(cache: Path, part: proto.RunPart) -> Path:
    return cache / proto.CACHE_DIR / f"{part.bag}.{part.run_id}.{part.role}.motion-wheel.npz"


def _v05_slices(cache: Path, bag: str):  # pragma: no cover - real data
    """Every cached v0.5 T2 part of a 2026-11-15 bag (train and the spent held-out parts)."""
    from . import truth_v04, truth_v05

    paths = sorted((cache / truth_v05.TRAIN_DIR).glob(f"{bag}.*.motion-wheel.npz"))
    paths += sorted((cache / truth_v05.HELDOUT_DIR).glob(f"{bag}.*.motion-wheel.npz"))
    return [truth_v04.read_slice(p) for p in paths]


def bag_segments(args, parts, roles, vehicle):  # pragma: no cover - real data
    """Segments of the parts in `roles`, with their T2 (computed and cached when missing)."""
    from . import truth_v04
    from .bags import load_run
    from .bags import motion as scan_motion_tools
    from .dataset import Segment
    from .icp_motion import body_of, mount_of
    from .recordings import recorded_for

    out = []
    for bag in sorted({p.bag for p in parts}):
        mine = [p for p in parts if p.bag == bag and p.role in roles]
        if not mine:
            continue
        run = load_run(BAG_DIRS[bag] / bag, recorded_for(bag))
        t_ref = float(run.scans[0].log_time_s)
        stamps = np.array([s.stamp_s for s in run.scans])
        if bag in V05_BAGS:
            slices = _v05_slices(args.cache, bag)
        else:
            slices, ignore = [], None
            for p in mine:
                path = _part_path(args.cache, p)
                if not path.is_file():
                    if ignore is None:
                        ignore = scan_motion_tools.static_self_returns(
                            run.scans, mount_of(vehicle), body_of(vehicle)
                        )
                    LOG.info("T2 of %s (%.1f s)", p.name, p.duration_s)
                    sl = truth_v04.compute_segment_slice(
                        run, vehicle, ignore, t_ref + p.t_from_s, t_ref + p.t_to_s, args.workers,
                        truth_v04.T2,
                    )  # fmt: skip
                    truth_v04.save_slice(path, sl)
                slices.append(truth_v04.read_slice(path))
        if args.truth_only:
            continue
        motion = truth_v04.assemble(stamps, slices)
        for p in mine:
            i0 = int(np.searchsorted(run.t_s, t_ref + p.t_from_s, side="left"))
            i1 = int(np.searchsorted(run.t_s, t_ref + p.t_to_s, side="right"))
            out.append(
                Segment(run, motion, i0, i1, p.role, p.name, "by_heading", None, strict_start=True)
            )
    return out


def truth_cmd(args) -> int:  # pragma: no cover - real data
    from .vehicle import load_vehicle, repo_root

    args.truth_only = True
    parts = load_parts(repo_root())
    bag_segments(args, parts, proto.ROLES, load_vehicle())
    return 0


def _frozen_path(args) -> Path:
    return args.cache / f"{proto.MODEL_ID}-frozen.pkl"


def v01_nominal(root: Path):  # pragma: no cover - real checkout
    from .cli import read_pointer_nominal

    return read_pointer_nominal(root / "training" / "pnp_models" / V01_BUNDLE / "manifest.yaml")


def fit_cmd(args) -> int:  # pragma: no cover - real data
    from . import pipeline_v02
    from .vehicle import load_vehicle, repo_root

    root = repo_root()
    args.truth_only = False
    parts = load_parts(root)
    vehicle = load_vehicle()
    segs = bag_segments(args, parts, (proto.TRAIN, proto.VAL), vehicle)
    segs = [s for s in segs if s.split in (proto.TRAIN, proto.VAL)]
    v01 = v01_nominal(root)
    frozen = pipeline_v02.strict_fit_stage(
        segs, vehicle, v01.nominal, V_MAX_MPS, configs=_configs(), seeds=SEEDS
    )
    _frozen_path(args).write_bytes(pickle.dumps(frozen))
    LOG.info(
        "shipping %s seed %d, validation P1 %.4f m (dyn-v0.1 %.4f m); samples %s",
        frozen.shipping.config.name,
        frozen.shipping.seed,
        frozen.shipping.val_p1_m,
        frozen.nominal_val["P1"],
        frozen.samples,
    )
    return 0


def dev_eval_cmd(args) -> int:  # pragma: no cover - real data
    from .evaluate import HORIZONS_S, eval_set
    from .ground_motion import contract1_window_errors
    from .session_v05 import with_heading_direction
    from .strata import assign_strata, block_bootstrap_ratio
    from .vehicle import load_vehicle, repo_root

    root = repo_root()
    args.truth_only = False
    parts = load_parts(root)
    vehicle = load_vehicle()
    from .pipeline import fit_vehicle_of

    fvehicle = fit_vehicle_of(vehicle)
    frozen = pickle.loads(_frozen_path(args).read_bytes())
    sha = git(root, "rev-parse", "HEAD")
    dev = [s for s in bag_segments(args, parts, (proto.DEV,), vehicle) if s.run.has_odom]
    es = with_heading_direction(eval_set(dev), dev)
    strata = assign_strata(es.direction, es.start_speed)
    v01 = v01_nominal(root).nominal
    models = {"dyn-v0.1": None, "proto": frozen.shipping.residual}
    for c in frozen.candidates:
        if c.config.name == frozen.shipping.config.name:
            models[f"proto seed {c.seed}"] = c.residual
    errs = {n: contract1_window_errors(es, v01, fvehicle, r, HORIZONS_S) for n, r in models.items()}
    # The online speed adaptation, replayed causally along each run (adaptation.py).
    from . import adaptation

    adapt_info = {}
    for name, res in (("dyn-v0.1", None), ("proto", frozen.shipping.residual)):
        corr = adaptation.window_corrections(es, v01, fvehicle, res)
        errs[f"{name} + adaptation"] = adaptation.corrected_window_errors(
            es, v01, fvehicle, res, corr, HORIZONS_S
        )
        adapt_info[name] = {
            "bias_mps2_p5_p50_p95": np.percentile(corr[:, 0], [5, 50, 95]).round(4).tolist(),
            "gain_p5_p50_p95": np.percentile(corr[:, 1], [5, 50, 95]).round(4).tolist(),
        }
    session = np.array([proto_session(es.runs[i].name) for i in es.run_idx])
    groups = {
        "all": np.ones(len(es), dtype=bool),
        "cw": es.direction == "cw",
        "ccw": es.direction == "ccw",
        "session 2026-11-15 (bags 2026-11-16)": session == "2026-11-16",
        "session 2026-11-23 (bag 2026-11-24)": session == "2026-11-24",
        "start >= 1.0 m/s": es.start_speed >= 1.0,
    }
    rows = proto.summarise(errs, strata.label, groups, "dyn-v0.1")
    boot = block_bootstrap_ratio(
        errs["proto"][1.0]["pos"], errs["dyn-v0.1"][1.0]["pos"], strata, es.block
    )
    out = {
        "frozen_git_sha": sha,
        "windows": len(es),
        "strata": {n: strata.counts[n] for n in strata.names},
        "shipping": {"config": frozen.shipping.config.name, "seed": frozen.shipping.seed},
        "validation_P1_m": {
            "proto": frozen.shipping.val_p1_m,
            "dyn-v0.1": frozen.nominal_val["P1"],
        },
        "samples": frozen.samples,
        "rows": rows,
        "P1_ratio_bootstrap": boot,
        "residual_output_saturation": saturation(frozen.shipping.residual, es, v01, fvehicle),
        "adaptation_corrections_at_window_starts": adapt_info,
    }
    (args.cache / f"{proto.MODEL_ID}-dev.json").write_text(json.dumps(out, indent=2, default=float))
    print(proto.render_rows(rows))
    print(json.dumps({k: out[k] for k in ("frozen_git_sha", "windows", "P1_ratio_bootstrap")}))
    return 0


ADAPT_GRID = tuple((w, f) for w in (0.25, 0.5, 1.0) for f in (0.95, 0.98))


def adapt_select_cmd(args) -> int:  # pragma: no cover - real data
    """Choose the adaptation window and forgetting on the VALIDATION parts (never the dev parts):
    macro S1 and P1 at 1.0 s of the shipping residual with and without the correction."""
    from dataclasses import replace

    from . import adaptation
    from .evaluate import HORIZONS_S, eval_set
    from .ground_motion import contract1_window_errors
    from .pipeline import fit_vehicle_of
    from .session_v05 import with_heading_direction
    from .strata import assign_strata
    from .vehicle import load_vehicle, repo_root

    root = repo_root()
    parts = load_parts(root)
    vehicle = load_vehicle()
    fvehicle = fit_vehicle_of(vehicle)
    frozen = pickle.loads(_frozen_path(args).read_bytes())
    val = [s for s in bag_segments(args, parts, (proto.VAL,), vehicle) if s.run.has_odom]
    es = with_heading_direction(eval_set(val), val)
    strata = assign_strata(es.direction, es.start_speed)
    v01 = v01_nominal(root).nominal
    res = frozen.shipping.residual
    errs = {"proto": contract1_window_errors(es, v01, fvehicle, res, HORIZONS_S)}
    for w, f in ADAPT_GRID:
        ap = replace(adaptation.AdaptationParams(), window_s=w, forgetting=f)
        corr = adaptation.window_corrections(es, v01, fvehicle, res, ap)
        errs[f"proto + adaptation w{w} f{f}"] = adaptation.corrected_window_errors(
            es, v01, fvehicle, res, corr, HORIZONS_S
        )
    rows = proto.summarise(errs, strata.label, {"val": np.ones(len(es), bool)}, "proto")
    print(proto.render_rows(rows))
    (args.cache / f"{proto.MODEL_ID}-adapt-select.json").write_text(json.dumps(rows, indent=2))
    return 0


def proto_session(run_name: str) -> str:
    return run_name[:10]


def saturation(residual, es, nominal, fvehicle) -> dict:  # pragma: no cover - real data
    """Share of development steps on which a residual output is at or above 0.9 of its cap."""
    from .evaluate import predict
    from .model import mlp_forward

    states = predict(es.data(nominal.command_delay_s), nominal, fvehicle, residual)
    cmds = es.commands(nominal.command_delay_s)[:, -states.shape[1] + 1 :, :]
    x = np.stack(
        [states[:, :-1, 3], states[:, :-1, 4], cmds[:, :, 0], cmds[:, :, 1]], axis=-1
    ).reshape(-1, 4)
    out = residual.output_scale * mlp_forward(
        residual.weights, (x - residual.input_mean) / residual.input_std
    )
    cap = np.array([2.0, 0.5])
    return {
        "dv_dt_share_at_0.9_cap": float(np.mean(np.abs(out[:, 0]) >= 0.9 * cap[0])),
        "dyaw_dt_share_at_0.9_cap": float(np.mean(np.abs(out[:, 1]) >= 0.9 * cap[1])),
        "max_abs": [float(np.max(np.abs(out[:, 0]))), float(np.max(np.abs(out[:, 1])))],
    }


def export_cmd(args) -> int:  # pragma: no cover - real data
    from . import export
    from .vehicle import load_vehicle, repo_root

    root = repo_root()
    vehicle = load_vehicle()
    frozen = pickle.loads(_frozen_path(args).read_bytes())
    dev_path = args.cache / f"{proto.MODEL_ID}-dev.json"
    dev = json.loads(dev_path.read_text()) if dev_path.is_file() else {}
    sha = dev.get("frozen_git_sha") or git(root, "rev-parse", "HEAD")
    created = dt.datetime.now(dt.timezone.utc).strftime("%Y-%m-%dT%H:%M:%SZ")
    split_text = (pointer_dir(root) / "dev-split.yaml").read_text()
    cfg_sha = export.config_sha256(
        {
            "split": export.sha256_text(split_text),
            "git": sha,
            "configs": [c.name for c in _configs()],
        }
    )
    data = [
        (
            f"development split {export.sha256_text(split_text)} (training/pnp_models/"
            f"{proto.MODEL_ID}/dev-split.yaml), truth T2"
        ),
        *[f"bag {b}" for b in sorted(BAG_DIRS)],
    ]
    metrics = {
        "status": "prototype, unregistered (owner decision 29); not a gate result",
        "shipping": dev.get("shipping", {}),
        "validation_P1_m": dev.get("validation_P1_m", {}),
        "dev_heldout_1s": [r for r in dev.get("rows", []) if r["model"] in ("dyn-v0.1", "proto")],
        "P1_ratio_bootstrap": dev.get("P1_ratio_bootstrap", {}),
    }
    bundle = args.models / proto.MODEL_ID
    manifest = export.write_bundle(
        bundle, model_id=proto.MODEL_ID, created_utc=created, git_sha=sha,
        training_config_sha256=cfg_sha, vehicle=vehicle, nominal=frozen.nominal,
        residual=frozen.shipping.residual, v_max_mps=V_MAX_MPS, data=data,
        metrics=json.loads(json.dumps(metrics, default=float)), golden_residual_rollouts=True,
    )  # fmt: skip
    ptr = pointer_dir(root)
    ptr.mkdir(parents=True, exist_ok=True)
    import shutil

    shutil.copy(bundle / export.MANIFEST_FILE, ptr / export.MANIFEST_FILE)
    (ptr / "metrics.json").write_text(json.dumps(dev, indent=2, default=float), encoding="utf-8")
    LOG.info("wrote %s (residual enabled %s)", bundle, manifest["io"]["residual"]["enabled"])
    return 0


STAGE_COMMANDS = {
    "split": split_cmd,
    "truth": truth_cmd,
    "fit": fit_cmd,
    "adapt-select": adapt_select_cmd,
    "dev-eval": dev_eval_cmd,
    "export": export_cmd,
}


def main(argv: list[str]) -> int:  # pragma: no cover - real data
    import argparse

    parser = argparse.ArgumentParser(prog="racer_pnp_dynamics.stages_proto")
    parser.add_argument("command", choices=tuple(STAGE_COMMANDS))
    parser.add_argument("--models", type=Path, default=DATA_ROOT / "pnp" / "models")
    parser.add_argument("--cache", type=Path, default=DATA_ROOT / "pnp" / "cache")
    parser.add_argument("--workers", type=int, default=4)
    args = parser.parse_args(argv)
    args.truth_only = False
    logging.basicConfig(level=logging.INFO)
    return STAGE_COMMANDS[args.command](args)


if __name__ == "__main__":  # pragma: no cover
    import sys

    raise SystemExit(main(sys.argv[1:]))
