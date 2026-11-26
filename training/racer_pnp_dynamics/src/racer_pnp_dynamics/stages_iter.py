"""Real-bag stages of the dynamics iteration on dyn-v0.7p's line (`iter_roles.py`, `iter_rule.py`).

    uv run --group train python -m racer_pnp_dynamics.stages_iter explore --configs a,b   # validation only
    uv run --group train python -m racer_pnp_dynamics.stages_iter fit --version v0.8
    uv run --group train python -m racer_pnp_dynamics.stages_iter fit-summary --version v0.8
    uv run --group train python -m racer_pnp_dynamics.stages_iter bench-bundle --version v0.8
    uv run --group train python -m racer_pnp_dynamics.stages_iter freeze-eval --version v0.8 --freeze-sha <sha>

`explore` and `fit` read train and validation data only. `freeze-eval` is the one pass of a
version: it needs the version's rule file, fit summary and solve file committed, HEAD equal to
`--freeze-sha` and a clean checkout, then reads H1 (the GP2g-R held-out parts of 2026-11-22, under
T2 and T2s) and H2 (dyn-v0.7p's 2026-11-15 development parts, reported only), scores the version
against dyn-v0.7p (and dyn-v0.1, for information), applies the rule and writes the bundle.
"""

from __future__ import annotations

import datetime as dt
import json
import logging
import pickle
from dataclasses import dataclass, replace
from pathlib import Path

import numpy as np

from . import iter_roles, iter_rule, proto

LOG = logging.getLogger(__name__)

DATA_ROOT = Path("/Users/cameronjim/code/car-data")
V07P = "dyn-v0.7p"
V01 = "dyn-v0.1"
V07P_ID = "dyn-v0.7p-2026-11-24"
CACHE_SUB = "dyn-iter"
DEVICE_NOTE = "cpu, float64, 4 threads"
SOLVE_FILE = "solve.json"
CAP = (2.0, 0.5)  # the output bound of every dyn-v0.7p config, unchanged


def cfg(name: str, mode: str, hidden: int = 32, wd: float = 1e-2, **kw):
    from .residual_train import ResidualConfig

    return ResidualConfig(name, mode, wd, out_cap=CAP, hidden=hidden, **kw)


def config_grid():
    """Every config this iteration may train, by name (dyn-v0.7p's three at width 32 first)."""
    from .residual_train import MULTI_STEP, MULTI_STEP_FROM_ONE, ONE_STEP

    out = {}
    for h in (32, 24, 16, 12, 8):
        tag = "" if h == 32 else f"_h{h}"
        out[f"one_step_capped_wd1e-2{tag}"] = cfg(f"one_step_capped_wd1e-2{tag}", ONE_STEP, h)
        out[f"multi_step_capped_wd1e-2{tag}"] = cfg(f"multi_step_capped_wd1e-2{tag}", MULTI_STEP, h)
        out[f"multi_step_from_one_capped_wd1e-2{tag}"] = cfg(
            f"multi_step_from_one_capped_wd1e-2{tag}", MULTI_STEP_FROM_ONE, h
        )
        out[f"multi_step_from_one_capped_wd1e-3{tag}"] = cfg(
            f"multi_step_from_one_capped_wd1e-3{tag}", MULTI_STEP_FROM_ONE, h, 1e-3
        )
        # the position and yaw terms of the rollout loss weighted up against the wheel speed term
        out[f"multi_step_from_one_capped_wd1e-2_posw{tag}"] = cfg(
            f"multi_step_from_one_capped_wd1e-2_posw{tag}", MULTI_STEP_FROM_ONE, h,
            loss_weights=(0.25, 1.0, 2.0),
        )  # fmt: skip
        for wtag, w in (("pos2", (1.0, 1.0, 2.0)), ("sp05pos2", (0.5, 1.0, 2.0)),
                        ("yaw2pos2", (1.0, 2.0, 2.0))):  # fmt: skip
            out[f"multi_step_from_one_capped_wd1e-2_{wtag}{tag}"] = cfg(
                f"multi_step_from_one_capped_wd1e-2_{wtag}{tag}", MULTI_STEP_FROM_ONE, h,
                loss_weights=w,
            )  # fmt: skip
        # dyn-v0.9: the contract 1 ground scale learned with the residual (bundle 1.2)
        out[f"multi_step_from_one_capped_wd1e-2_gs{tag}"] = cfg(
            f"multi_step_from_one_capped_wd1e-2_gs{tag}", MULTI_STEP_FROM_ONE, h,
            learn_ground_scale=True,
        )  # fmt: skip
        out[f"multi_step_from_one_capped_wd1e-2_gs_yaw2{tag}"] = cfg(
            f"multi_step_from_one_capped_wd1e-2_gs_yaw2{tag}", MULTI_STEP_FROM_ONE, h,
            learn_ground_scale=True, loss_weights=(1.0, 2.0, 1.0),
        )  # fmt: skip
        out[f"multi_step_from_one_capped_wd1e-2_posw4{tag}"] = cfg(
            f"multi_step_from_one_capped_wd1e-2_posw4{tag}", MULTI_STEP_FROM_ONE, h,
            loss_weights=(0.25, 1.0, 4.0),
        )  # fmt: skip
    return out


@dataclass(frozen=True)
class Version:
    key: str
    model_id: str
    configs: tuple[str, ...]
    seeds: tuple[int, ...] = (0, 1, 2)
    width_tolerance: float = 0.01  # iter_rule.select_cost_aware


VERSIONS: dict[str, Version] = {
    # dyn-v0.8: every allowed train bag (no session 2) and the residual width chosen cost aware
    # (32, 16 or 8; the narrowest within 1 percent of the best mean validation P1)
    "v0.8": Version(
        "v0.8",
        "dyn-v0.8-2026-11-25",
        (
            "multi_step_from_one_capped_wd1e-2",
            "multi_step_from_one_capped_wd1e-2_h16",
            "multi_step_from_one_capped_wd1e-2_h8",
        ),
    ),
    # dyn-v0.9: v0.8 plus the contract 1 ground scale learned with the residual (bundle 1.2.0)
    "v0.9": Version(
        "v0.9",
        "dyn-v0.9-2026-11-25",
        (
            "multi_step_from_one_capped_wd1e-2_gs",
            "multi_step_from_one_capped_wd1e-2_gs_h16",
            "multi_step_from_one_capped_wd1e-2_gs_h8",
        ),
    ),
}


def version(key: str) -> Version:
    if key not in VERSIONS:
        raise SystemExit(f"unknown version {key!r}; known {sorted(VERSIONS)}")
    return VERSIONS[key]


def cache_dir(args) -> Path:
    d = args.cache / CACHE_SUB
    d.mkdir(parents=True, exist_ok=True)
    return d


def pointer_dir(root: Path, model_id: str) -> Path:
    return root / "training" / "pnp_models" / model_id


class IterContext:  # pragma: no cover - real data
    """Train and validation segments of the iteration (`iter_roles`), dyn-v0.1 and dyn-v0.7p."""

    def __init__(self, args) -> None:
        from . import stages_proto
        from .cli_v04 import Stage1
        from .evaluate import eval_set
        from .export import read_bundle
        from .stages_v06 import with_measured_direction

        self.stage1 = Stage1(args)
        self.root = self.stage1.root
        self.vehicle = self.stage1.vehicle
        self.fvehicle = self.stage1.fvehicle
        self.parts = stages_proto.load_parts(self.root)
        fit = iter_roles.fit_parts(self.parts)
        proto_args = replace_args(args, truth_only=False)
        w1 = stages_proto.bag_segments(proto_args, fit, iter_roles.FIT_ROLES, self.vehicle)
        w1 = [s for s in w1 if s.split in iter_roles.FIT_ROLES]
        self.train = [*self.stage1.train, *w1]
        iter_roles.check_fit_segments(
            [s.name for s in self.train], [s.split for s in self.train],
            [s.run.name for s in self.train],
        )  # fmt: skip
        val = [s for s in self.train if s.split == proto.VAL and s.run.has_odom]
        self.val = with_measured_direction(eval_set(val), val)
        self.v01 = self.stage1.v01().nominal
        self.v07p = read_bundle(args.models / V07P_ID).residual
        LOG.info(
            "train %d segments (%d of dyn-v0.7p's split outside session 2); validation %d windows",
            len(self.train), len(w1), len(self.val),
        )  # fmt: skip


def replace_args(args, **kw):
    import argparse

    d = dict(vars(args))
    d.update(kw)
    return argparse.Namespace(**d)


def val_p1(ctx: IterContext, residual) -> float:  # pragma: no cover - real data
    from .residual_train import validation_p1

    return validation_p1(ctx.v01, residual, ctx.fvehicle, ctx.val)


def cand_path(args, name: str, seed: int) -> Path:
    """One trained (config, seed). The fit is deterministic given the data, the config and the
    seed, so `explore` and `fit` share these (same train and validation data in both)."""
    return cache_dir(args) / "cands" / f"{name}-seed{seed}.pkl"


def train_configs(ctx, names, seeds, args):  # pragma: no cover - real data
    """Train every (config, seed), cached per (config, seed) under the iteration cache."""
    import torch

    from .pipeline_v02 import candidate_residuals

    torch.set_num_threads(4)
    grid = config_grid()
    train = [s for s in ctx.train if s.split == proto.TRAIN]
    val = [s for s in ctx.train if s.split == proto.VAL]
    out = []
    for name in names:
        for seed in seeds:
            path = cand_path(args, name, seed)
            if path.is_file():
                (cand,) = pickle.loads(path.read_bytes())
            else:
                (cand,), counts = candidate_residuals(
                    [grid[name]], (seed,), ctx.v01, ctx.fvehicle, train, val
                )
                path.parent.mkdir(parents=True, exist_ok=True)
                path.write_bytes(pickle.dumps((cand,)))
                LOG.info("samples %s", counts)
            # the score every choice uses: macro P1 at 1.0 s on the validation windows with the
            # measured heading direction (the same number for dyn-v0.1 and dyn-v0.7p)
            out.append(replace(cand, val_p1_m=val_p1(ctx, cand.residual)))
    return out


def explore_cmd(args) -> int:  # pragma: no cover - real data
    """Validation numbers only: P1, Y1, S1 at 1.0 s of each config and seed, and of dyn-v0.1 and
    dyn-v0.7p on the same validation windows."""
    from .evaluate import HORIZONS_S
    from .ground_motion import contract1_window_errors
    from .strata import assign_strata

    ctx = IterContext(args)
    names = args.configs.split(",")
    cands = train_configs(ctx, names, tuple(int(s) for s in args.seeds.split(",")), args)
    strata = assign_strata(ctx.val.direction, ctx.val.start_speed)
    models = {V01: None, V07P: ctx.v07p}
    for c in cands:
        models[f"{c.config.name} s{c.seed}"] = c.residual
    errs = {
        n: contract1_window_errors(ctx.val, ctx.v01, ctx.fvehicle, r, HORIZONS_S)
        for n, r in models.items()
    }
    groups = {
        "val": np.ones(len(ctx.val), bool),
        "cw": ctx.val.direction == "cw",
        "ccw": ctx.val.direction == "ccw",
    }
    rows = proto.summarise(errs, strata.label, groups, V07P)
    text = proto.render_rows(rows)
    print(text)
    (cache_dir(args) / f"explore-{args.label}.md").write_text(text + "\n")
    return 0


def adapt_val_cmd(args) -> int:  # pragma: no cover - real data
    """Validation only: the planner's online speed adaptation replayed causally (adaptation.py)
    on dyn-v0.1, dyn-v0.7p and the given candidates, with and without the correction."""
    from . import adaptation
    from .evaluate import HORIZONS_S
    from .ground_motion import contract1_window_errors
    from .strata import assign_strata

    ctx = IterContext(args)
    names = [n for n in args.configs.split(",") if n]
    cands = train_configs(ctx, names, tuple(int(s) for s in args.seeds.split(",")), args)
    models = {
        V01: None,
        V07P: ctx.v07p,
        **{f"{c.config.name} s{c.seed}": c.residual for c in cands},
    }
    errs = {}
    for n, r in models.items():
        errs[n] = contract1_window_errors(ctx.val, ctx.v01, ctx.fvehicle, r, HORIZONS_S)
        corr = adaptation.window_corrections(ctx.val, ctx.v01, ctx.fvehicle, r)
        errs[f"{n} + adaptation"] = adaptation.corrected_window_errors(
            ctx.val, ctx.v01, ctx.fvehicle, r, corr, HORIZONS_S
        )
    strata = assign_strata(ctx.val.direction, ctx.val.start_speed)
    rows = proto.summarise(errs, strata.label, {"val": np.ones(len(ctx.val), bool)}, V07P)
    text = proto.render_rows(rows)
    print(text)
    (cache_dir(args) / f"adapt-{args.label}.md").write_text(text + "\n")
    return 0


def _fit_pkl(args, v: Version) -> Path:
    return cache_dir(args) / f"{v.model_id}-fit.pkl"


def fit_cmd(args) -> int:  # pragma: no cover - real data
    v = version(args.version)
    ctx = IterContext(args)
    cands = train_configs(ctx, v.configs, v.seeds, args)
    config, ship, table = iter_rule.select_cost_aware(cands, v.width_tolerance)
    fit = {
        "version": v.key,
        "candidates": cands,
        "shipping_config": config.name,
        "shipping_seed": ship.seed,
        "shipping_hidden": config.hidden,
        "selection_table": table,
        "val_p1": {V01: val_p1(ctx, None), V07P: val_p1(ctx, ctx.v07p)},
        "val_windows": len(ctx.val),
        "train_segments": len(ctx.train),
    }
    _fit_pkl(args, v).write_bytes(pickle.dumps(fit))
    LOG.info(
        "%s shipping %s seed %d: validation P1 %.4f (dyn-v0.7p %.4f, dyn-v0.1 %.4f)",
        v.model_id, config.name, ship.seed, ship.val_p1_m, fit["val_p1"][V07P], fit["val_p1"][V01],
    )  # fmt: skip
    return 0


def load_fit(args, v: Version) -> dict:  # pragma: no cover - real data
    return pickle.loads(_fit_pkl(args, v).read_bytes())


def seeds_of(fit: dict) -> dict:
    return {
        c.seed: c.residual for c in fit["candidates"] if c.config.name == fit["shipping_config"]
    }


def fit_summary_cmd(args) -> int:  # pragma: no cover - real data
    from .stages_v06 import sha256_file
    from .vehicle import repo_root

    v = version(args.version)
    fit = load_fit(args, v)
    lines = [
        f"# {v.model_id} fit, frozen before its held-out pass",
        "",
        "Validation numbers only; no H1 or H2 number of this version exists when this is written.",
        "",
        (
            f"- Train segments: {fit['train_segments']} (iter_roles: older train roles, dyn-v0.7p's "
            "2026-11-15 and 2026-11-22 train and validation parts; no session 2 bag)."
        ),
        f"- Validation windows: {fit['val_windows']}.",
        f"- Configs: {list(v.configs)}; seeds {list(v.seeds)}; device {DEVICE_NOTE}.",
        f"- Fit pickle SHA-256: {sha256_file(_fit_pkl(args, v))}.",
        "",
        (
            f"Shipping: {fit['shipping_config']} seed {fit['shipping_seed']} (hidden "
            f"{fit['shipping_hidden']}), by `iter_rule.select_cost_aware` (width tolerance "
            f"{v.width_tolerance})."
        ),
        "",
        f"Validation P1: dyn-v0.1 {fit['val_p1'][V01]:.4f} m, dyn-v0.7p {fit['val_p1'][V07P]:.4f} m.",
        "",
        "| config | mean validation P1 (m) | min | per seed |",
        "|---|---|---|---|",
    ]
    for name, row in fit["selection_table"].items():
        seeds = ", ".join(f"{k} {val:.4f}" for k, val in row.items() if k.startswith("seed"))
        lines.append(f"| {name} | {row['mean']:.4f} | {row['min']:.4f} | {seeds} |")
    out = pointer_dir(repo_root(), v.model_id)
    out.mkdir(parents=True, exist_ok=True)
    (out / "fit-summary.md").write_text("\n".join(lines) + "\n", encoding="utf-8")
    print("\n".join(lines))
    return 0


def write_bundle(args, v: Version, dest: Path, git_sha: str, metrics: dict):  # pragma: no cover
    from . import export
    from .stages_v06 import prereg_v06_v01, sha256_file
    from .vehicle import load_vehicle, repo_root

    fit = load_fit(args, v)
    res = seeds_of(fit)[fit["shipping_seed"]]
    root = repo_root()
    created = dt.datetime.now(dt.timezone.utc).strftime("%Y-%m-%dT%H:%M:%SZ")
    cfg_sha = export.config_sha256(
        {"fit": sha256_file(_fit_pkl(args, v)), "rule": iter_rule.rule_sha256(root, v.model_id)}
    )
    data = [
        (
            "train and validation: older train roles (v0.2 split, legacy, 2026-11-14 train), "
            f"{V07P_ID} development split train and validation parts of 2026-11-15 and 2026-11-22"
        ),
        "no bag of the 2026-11-23 session (confirmation only), truth T2",
    ]
    return export.write_bundle(
        dest, model_id=v.model_id, created_utc=created, git_sha=git_sha,
        training_config_sha256=cfg_sha, vehicle=load_vehicle(), nominal=prereg_v06_v01(root),
        residual=res, v_max_mps=1.3, data=data, metrics=metrics, golden_residual_rollouts=True,
    )  # fmt: skip


def bench_bundle_cmd(args) -> int:  # pragma: no cover - real data
    """A copy of the frozen weights for the solve time item (written before the pass)."""
    v = version(args.version)
    dest = cache_dir(args) / "bench" / v.model_id
    write_bundle(args, v, dest, "0" * 40, {"status": "bench copy for the solve item"})
    LOG.info("wrote %s", dest)
    return 0


def check_freeze(root: Path, sha: str, v: Version):  # pragma: no cover - real checkout
    from . import truth_v04
    from .stages_v06 import git, require_committed

    if git(root, "rev-parse", "HEAD") != sha:
        raise SystemExit("--freeze-sha is not the HEAD of this checkout")
    if git(root, "status", "--porcelain", "--untracked-files=no"):
        raise SystemExit("the checkout has uncommitted changes: freeze first")
    require_committed(root, iter_rule.rule_relative(v.model_id).as_posix())
    for name in ("fit-summary.md", SOLVE_FILE):
        require_committed(root, f"training/pnp_models/{v.model_id}/{name}")
    return truth_v04.freeze(sha)


def freeze_eval_cmd(args) -> int:  # pragma: no cover - real data
    """The ONE pass of a version. Refits nothing; takes no option that changes a model."""
    import shutil

    from . import gp2, pose_check, prereg_v06, scoring_v05, stages_proto, stages_v06
    from .cli_v03 import pose_recordings
    from .evaluate import HORIZONS_S, eval_set
    from .ground_motion import contract1_window_errors
    from .pass_v05 import score_pass
    from .pipeline import _json
    from .session_v05 import with_heading_direction
    from .strata import assign_strata, block_bootstrap_ratio
    from .truth_v04 import T2, T2S

    v = version(args.version)
    ctx = IterContext(args)
    root = ctx.root
    token = check_freeze(root, args.freeze_sha, v)
    rule = iter_rule.load_rule(root, v.model_id)
    c06 = stages_v06.constants()
    fit = load_fit(args, v)
    seeds = seeds_of(fit)
    ship = fit["shipping_seed"]
    me = f"{v.model_id} seed {ship}"
    models = {V01: scoring_v05.Spec(ctx.v01), V07P: scoring_v05.Spec(ctx.v01, ctx.v07p)}
    for s, res in sorted(seeds.items()):
        models[f"{v.model_id} seed {s}"] = scoring_v05.Spec(ctx.v01, res)
    LOG.info("freeze %s; reading H1 and H2", token.git_sha)
    roles = stages_v06.roles(root)
    held_t2 = stages_v06.session22_segments(args, roles, ctx.vehicle, prereg_v06.HELDOUT, token, T2)
    held_t2s = stages_v06.session22_segments(
        args, roles, ctx.vehicle, prereg_v06.HELDOUT, token, T2S
    )
    recs = pose_recordings(args.bags, args.cache)
    older_held = [s for s in ctx.stage1.older_all if s.split == "heldout" and s.run.has_odom]
    pose_segs = pose_check.build_pose_segments(older_held, recs)
    pair_segs = pose_check.build_pose_pair_segments(older_held, recs)
    p = score_pass(c06, held_t2, held_t2s, ctx.fvehicle, models, ctx.v01, pose_segs, pair_segs)
    metrics = {n: p.metrics(n, []) for n in models}
    boot = p.bootstrap(me, V07P)
    sweep = gp2.stability_sweep(
        ctx.v01, seeds[ship], ctx.vehicle, 1.3, samples=int(rule.gate["sweep_samples"])
    )
    sat = stages_v06.saturation_on(
        p.es, ctx.v01, ctx.fvehicle, seeds[ship], float(rule.gate["saturation"]["level"])
    )
    solve = json.loads((pointer_dir(root, v.model_id) / SOLVE_FILE).read_text())
    result = iter_rule.decide(
        rule, p.es.direction, p.strata, metrics[V07P], metrics[me], boot["hi"],
        {s: metrics[f"{v.model_id} seed {s}"].macro["P1"] for s in seeds}, sweep, sat["share"],
        iter_rule.Solve(**solve["solve"]),
    )  # fmt: skip
    # information: the same items against dyn-v0.1
    boot01 = p.bootstrap(me, V01)
    # H2: dyn-v0.7p's 2026-11-15 development parts, reported only (PR 245 table form)
    h2 = [
        s
        for s in stages_proto.bag_segments(
            replace_args(args, truth_only=False), iter_roles.h2_parts(ctx.parts), (proto.DEV,),
            ctx.vehicle,
        )
        if s.run.has_odom
    ]  # fmt: skip
    es2 = with_heading_direction(eval_set(h2), h2)
    st2 = assign_strata(es2.direction, es2.start_speed)
    e2 = {
        n: contract1_window_errors(es2, ctx.v01, ctx.fvehicle, m.residual, HORIZONS_S)
        for n, m in models.items()
    }
    h2_rows = proto.summarise(
        e2, st2.label,
        {"all": np.ones(len(es2), bool), "cw": es2.direction == "cw", "ccw": es2.direction == "ccw"},
        V07P,
    )  # fmt: skip
    h2_boot = block_bootstrap_ratio(e2[me][1.0]["pos"], e2[V07P][1.0]["pos"], st2, es2.block)
    out = {
        "version": v.key,
        "model_id": v.model_id,
        "freeze_sha": token.git_sha,
        "rule_sha256": rule.sha256,
        "windows": len(p.es),
        "direction_windows": {d: int(np.sum(p.es.direction == d)) for d in ("ccw", "cw")},
        "strata": dict(p.strata.counts),
        "gate": result.to_dict(),
        "bootstrap_P1_vs_v07p": boot,
        "bootstrap_P1_vs_v01": boot01,
        "sweep": sweep.to_dict(),
        "saturation": sat,
        "solve": solve,
        "metrics": {n: m.__dict__ for n, m in metrics.items()},
        "h2_windows": len(es2),
        "h2_rows": h2_rows,
        "h2_bootstrap_P1_vs_v07p": h2_boot,
        "shipping": {
            "config": fit["shipping_config"],
            "seed": ship,
            "hidden": fit["shipping_hidden"],
        },
    }
    folder = pointer_dir(root, v.model_id)
    (folder / "pass-result.json").write_text(
        json.dumps(_json(out), indent=2, default=str), encoding="utf-8"
    )
    report = iter_rule.render_report(out, metrics, me)
    (folder / "pass-report.md").write_text(report, encoding="utf-8")
    passed = result.outcome == iter_rule.PASS
    dest_root = args.models if passed else args.models.parent / "models-candidate"
    mm = {
        "rule": result.outcome,
        "P1_ratio_vs_v07p": metrics[me].macro["P1"] / metrics[V07P].macro["P1"],
        "Y1_ratio_vs_v07p": metrics[me].macro["Y1"] / metrics[V07P].macro["Y1"],
        "P1_ratio_vs_v01": metrics[me].macro["P1"] / metrics[V01].macro["P1"],
    }
    bundle = dest_root / v.model_id
    write_bundle(args, v, bundle, token.git_sha, _json(mm))
    shutil.copy(bundle / "manifest.yaml", folder / "manifest.yaml")
    (folder / "metrics.json").write_text(json.dumps(_json(mm), indent=2), encoding="utf-8")
    print(report)
    return 0


STAGE_COMMANDS = {
    "explore": explore_cmd,
    "adapt-val": adapt_val_cmd,
    "fit": fit_cmd,
    "fit-summary": fit_summary_cmd,
    "bench-bundle": bench_bundle_cmd,
    "freeze-eval": freeze_eval_cmd,
}


def main(argv: list[str]) -> int:  # pragma: no cover - real data
    import argparse

    parser = argparse.ArgumentParser(prog="racer_pnp_dynamics.stages_iter")
    parser.add_argument("command", choices=tuple(STAGE_COMMANDS))
    parser.add_argument("--version", default=None)
    parser.add_argument("--configs", default="")
    parser.add_argument("--seeds", default="0,1,2")
    parser.add_argument("--label", default="run")
    parser.add_argument("--bags", type=Path, default=DATA_ROOT / "bags")
    parser.add_argument("--models", type=Path, default=DATA_ROOT / "pnp" / "models")
    parser.add_argument("--cache", type=Path, default=DATA_ROOT / "pnp" / "cache")
    parser.add_argument("--workers", type=int, default=4)
    parser.add_argument("--freeze-sha", default=None)
    args = parser.parse_args(argv)
    logging.basicConfig(level=logging.INFO)
    return STAGE_COMMANDS[args.command](args)


if __name__ == "__main__":  # pragma: no cover
    import sys

    raise SystemExit(main(sys.argv[1:]))
