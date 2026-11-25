"""Real-bag stages of the yaw-only residual attempt R6 (brief 54, gate GP2g-R).

    uv run --group train python -m racer_pnp_dynamics.stages_v06 truth           # 2026-11-22 train parts
    uv run --group train python -m racer_pnp_dynamics.stages_v06 mb3             # commit M-B3.md
    uv run --group train python -m racer_pnp_dynamics.stages_v06 fit             # R6, seeds 0 to 2
    uv run --group train python -m racer_pnp_dynamics.stages_v06 fit-summary     # commit = freeze
    uv run --group train python -m racer_pnp_dynamics.stages_v06 bench-bundle    # items 12 and 13
    uv run --group train python -m racer_pnp_dynamics.stages_v06 freeze-eval --freeze-sha <sha>

Order of the yaml: split, m_b3, fit, freeze, heldout_pass. Train is every v0.5 train role
(`stages_v05.FitContext`) plus the train parts of the 2026-11-22 runs. Selection (never fitted on)
is the 2026-11-14 held-out segments and the 2026-11-15 session held-out parts, both read before.
The 2026-11-22 held-out parts are computed and read only by `freeze-eval`, with the freeze token.
A bag recorded after 2026-11-22 is refused everywhere (`prereg_v06.Roles.check_bag`).
"""

from __future__ import annotations

import datetime as dt
import hashlib
import json
import logging
import pickle
import subprocess
from dataclasses import replace
from pathlib import Path

import numpy as np

from . import prereg_v06
from .truth_v04 import T2, T2S

LOG = logging.getLogger(__name__)

DATA_ROOT = Path("/Users/cameronjim/code/car-data")
SESSION_DIR = DATA_ROOT / "session-2026-11-22"
V01 = "dyn-v0.1"
V04_FREEZE_SHA = "416e68501c424caeb1f6dc80b6a0ee9edc7a288c"  # PR 193: 2026-11-14 held-out read
V05_FREEZE_SHA = "24da648c9ac58b0afbf0f0f9eb4fc0290490ea29"  # PR 197: 2026-11-15 held-out read
DEVICE_NOTE = "cpu, float64, 4 threads"
SOLVE_CANARY_FILE = "solve-and-canary.json"


def model_id(date: str) -> str:
    return f"dyn-v0.6r-{date}"


def pointer_dir(root: Path, date: str) -> Path:
    return root / "training" / "pnp_models" / model_id(date)


def r6_configs():
    """R4's configs as `stages_v04.r4_fit` builds them: the v0.3 yaw-only grid, unchanged."""
    from .residual_train import DEFAULT_CANDIDATES

    return tuple(replace(c, name=f"yaw_{c.name}", yaw_only=True) for c in DEFAULT_CANDIDATES)


def select_by(candidates, scores: dict[tuple[str, int], float]):
    """Config with the lowest mean selection P1 over its seeds, then its seed with the lowest
    selection P1 (`residual_train.select_candidate` on selection P1)."""
    from .residual_train import select_candidate

    renamed = [replace(c, val_p1_m=scores[(c.config.name, c.seed)]) for c in candidates]
    config, ship, table = select_candidate(renamed)
    return (
        config,
        next(c for c in candidates if c.config.name == config.name and c.seed == ship.seed),
        table,
    )


def with_measured_direction(es, segments):
    """Windows labelled `by_heading` (session parts) take the direction of the measured T2
    heading; the others keep the direction their split gives."""
    from .session_v05 import heading_direction

    if not (es.direction == "by_heading").any():
        return es
    hd = heading_direction(es, [s for s in segments if s.direction == "by_heading"])
    return replace(
        es, direction=np.where(es.direction == "by_heading", hd, es.direction).astype(str)
    )


def git(root: Path, *args: str) -> str:  # pragma: no cover - real checkout
    return subprocess.run(
        ["git", "-C", str(root), *args], capture_output=True, text=True, check=True
    ).stdout.strip()


def sha256_file(path: Path) -> str:
    return hashlib.sha256(Path(path).read_bytes()).hexdigest()


def constants() -> prereg_v06.ConstantsV06:  # pragma: no cover - real checkout
    from .cli_v04 import _find

    return prereg_v06.load_constants_v06(_find(prereg_v06.V06_RELATIVE))


def roles(root: Path) -> prereg_v06.Roles:  # pragma: no cover - real checkout
    return prereg_v06.Roles(prereg_v06.load_session_v06(root / prereg_v06.SESSION_RELATIVE))


def session22_segments(  # pragma: no cover - real data
    args, r: prereg_v06.Roles, vehicle, role: str, token=None, variant: str = T2
) -> list:
    """Segments of the 2026-11-22 parts of `role` with their truth. Train parts are cached under
    pnp-v06-t2-train and read through `load_train_slice`; held-out parts under pnp-v06-t2-heldout,
    computed and read only with the freeze token."""
    from . import truth_v04
    from .bags import load_run
    from .bags import motion as scan_motion_tools
    from .dataset import _t_ref, make_segments
    from .icp_motion import body_of, mount_of
    from .recordings import recorded_for

    if role == prereg_v06.TRAIN:
        parts = r.train_parts()
    else:
        parts = r.heldout_parts(token)
    out = []
    for name, entry in r.split.bags.items():
        r.check_bag(name)
        wanted = [s for s in parts if s.bag == name]
        if not wanted:
            continue
        run = load_run(SESSION_DIR / name, recorded_for(name))
        t_ref = _t_ref(run, entry, None)
        stamps = np.array([s.stamp_s for s in run.scans])
        ignore, slices = None, []
        for seg in wanted:
            path = (
                prereg_v06.train_path(args.cache, name, seg.name, variant)
                if role == prereg_v06.TRAIN
                else prereg_v06.heldout_path(args.cache, name, seg.name, variant)
            )
            if not path.is_file():
                if ignore is None:
                    ignore = scan_motion_tools.static_self_returns(
                        run.scans, mount_of(vehicle), body_of(vehicle)
                    )
                LOG.info("T2%s (%s) of %s/%s", variant and f" {variant}", role, name, seg.name)
                sl = truth_v04.compute_segment_slice(
                    run, vehicle, ignore, t_ref + seg.t_from_s, t_ref + seg.t_to_s,
                    args.workers, variant,
                )  # fmt: skip
                path.parent.mkdir(parents=True, exist_ok=True)
                truth_v04.save_slice(path, sl)
            if role == prereg_v06.TRAIN:
                slices.append(prereg_v06.load_train_slice(r, args.cache, name, seg.name, variant))
            else:
                slices.append(
                    prereg_v06.load_heldout_slice(r, args.cache, name, seg.name, token, variant)
                )
        motion = truth_v04.assemble(stamps, slices)
        keep = ("train", "val") if role == prereg_v06.TRAIN else ("heldout",)
        out.extend(s for s in make_segments(run, motion, r.split) if s.split in keep)
    return out


class FitContextV06:  # pragma: no cover - real data
    """Train segments (v0.5 train roles plus the 2026-11-22 train parts) and the selection windows
    (2026-11-14 held-out segments and 2026-11-15 held-out parts, both read before)."""

    def __init__(self, args) -> None:
        from . import session_v05, stages_v05, truth_v04
        from .cli_v04 import segments_of_v04_bags
        from .evaluate import eval_set

        self.c = constants()
        self.v05 = stages_v05.FitContext(args)
        self.root = self.v05.root
        self.vehicle = self.v05.vehicle
        self.fvehicle = self.v05.fvehicle
        self.roles = roles(self.root)
        self.s22_train = session22_segments(args, self.roles, self.vehicle, prereg_v06.TRAIN)
        self.train = [*self.v05.train, *self.s22_train]
        for s in self.train:
            self.roles.check_bag(s.run.name)
        seg14 = [
            s
            for s in segments_of_v04_bags(
                args, self.v05.stage1.v04, self.vehicle, "heldout", truth_v04.freeze(V04_FREEZE_SHA)
            )
            if s.run.has_odom
        ]
        path05 = session_v05.require_committed_session(self.root)
        seg15 = [
            s
            for s in stages_v05.session_segments(
                args,
                session_v05.load_session_split(path05),
                stages_v05.part_meta(path05),
                self.vehicle,
                "heldout",
                truth_v04.freeze(V05_FREEZE_SHA),
                T2,
            )
            if s.run.has_odom
        ]
        self.selection_segments = [*seg14, *seg15]
        self.selection = with_measured_direction(
            eval_set(self.selection_segments), self.selection_segments
        )
        LOG.info(
            "train %d segments (%d of 2026-11-22); selection %d windows (%d segments)",
            len(self.train), len(self.s22_train), len(self.selection), len(self.selection_segments),
        )  # fmt: skip

    def v01(self):
        return self.v05.v01().nominal


def truth_cmd(args) -> int:  # pragma: no cover - real data
    """T2 of the 2026-11-22 train parts (the held-out parts are not computed here)."""
    from .vehicle import load_vehicle, repo_root

    r = roles(repo_root())
    segs = session22_segments(args, r, load_vehicle(), prereg_v06.TRAIN)
    LOG.info("2026-11-22 train segments: %d", len(segs))
    return 0


def render_mb3(budget_, c: prereg_v06.ConstantsV06, notes) -> str:
    """M-B3 of this attempt: the v0.4 table, the v0.6 threshold and sentence."""
    from . import budget

    text = budget.render_mb3(budget_, "dyn-v0.1", notes)
    head, _, _ = text.rpartition("Perfect yaw ratio (macro)")
    mb = c.doc["measurements"]["m_b3"]
    limit = float(mb["perfect_yaw_ratio_out_of_reach_above"])
    ratio = budget_.perfect_yaw_ratio()
    head = head.replace(
        "Pre-registration v0.4 section 2.2. A measurement on TRAIN data only (train and the "
        "internal validation part of the train segments, truth T2), produced and committed before "
        "any v0.4 fit. It decides nothing.",
        "Brief 54, pnp-v0.6-dynamics-yaw.yaml `measurements.m_b3` (the v0.4 section 2.2 method). "
        "A measurement on the TRAIN data of this attempt only (train and the internal validation "
        "part of every train role, truth T2), committed before any R6 fit. It decides nothing.",
    )
    lines = [head.rstrip("\n"), "", f"Perfect yaw ratio (macro): {ratio:.3f}.", ""]
    if ratio > limit:
        lines.append(
            f"The perfect yaw ratio is above {limit:g}: {mb['sentence_if_above']}. R6 is still "
            "trained and scored, and the gate is still applied as written."
        )
    else:
        lines.append(
            f"The perfect yaw ratio is not above {limit:g}: the registered sentence is not written."
        )
    return "\n".join(lines) + "\n"


def mb3_cmd(args) -> int:  # pragma: no cover - real data
    from . import budget
    from .evaluate import eval_set

    ctx = FitContextV06(args)
    segs = [s for s in ctx.train if s.run.has_odom]
    es = with_measured_direction(eval_set(segs), segs)
    b = budget.error_budget(es, ctx.v01(), ctx.fvehicle)
    notes = [
        (
            f"Windows: {len(es)} (train and internal validation parts of every v0.5 train role and "
            f"of the 2026-11-22 train parts), strata {', '.join(b.strata.names)}; the direction of "
            "a session window is its measured T2 heading."
        ),
        f"Constants file evaluation/prereg/pnp-v0.6-dynamics-yaw.yaml, SHA-256 {ctx.c.sha256}.",
        (
            "Session file evaluation/prereg/sessions/2026-11-22-dynamics-v06.yaml, SHA-256 "
            f"{ctx.roles.split.sha256}."
        ),
        "Model: dyn-v0.1 nominal read from its committed pointer manifest, residual disabled.",
    ]
    text = render_mb3(b, ctx.c, notes)
    out = pointer_dir(ctx.root, args.date)
    out.mkdir(parents=True, exist_ok=True)
    (out / "M-B3.md").write_text(text, encoding="utf-8")
    print(text)
    return 0


def _fit_pkl(args) -> Path:
    return args.cache / f"{model_id(args.date)}-fit.pkl"


def require_committed(root: Path, rel: str) -> None:  # pragma: no cover - real checkout
    if not git(root, "ls-files", "--", rel) or git(root, "status", "--porcelain", "--", rel):
        raise SystemExit(f"{rel} must be committed, unchanged, first")


def fit_cmd(args) -> int:  # pragma: no cover - real data
    """R6: R4's recipe on dyn-v0.1 (the six yaw-only configs, seeds 0 to 2, early stopping on the
    internal validation parts), then config and shipping seed by selection P1."""
    import torch

    from . import pipeline_v02
    from .residual_train import validation_p1

    torch.set_num_threads(4)
    ctx = FitContextV06(args)
    require_committed(ctx.root, f"training/pnp_models/{model_id(args.date)}/M-B3.md")
    v01 = ctx.v01()
    frozen = pipeline_v02.strict_fit_stage(
        ctx.train, ctx.vehicle, v01, ctx.c.validity_v_max_mps, configs=r6_configs(),
        seeds=ctx.c.seeds,
    )  # fmt: skip
    scores = {
        (c.config.name, c.seed): validation_p1(v01, c.residual, ctx.fvehicle, ctx.selection)
        for c in frozen.candidates
    }
    nominal_sel = validation_p1(v01, None, ctx.fvehicle, ctx.selection)
    config, ship, table = select_by(frozen.candidates, scores)
    _fit_pkl(args).write_bytes(
        pickle.dumps(
            {
                "frozen": frozen,
                "selection_p1": {f"{k[0]} seed {k[1]}": v for k, v in scores.items()},
                "selection_table": table,
                "selection_nominal_p1": nominal_sel,
                "selection_windows": len(ctx.selection),
                "shipping_config": config.name,
                "shipping_seed": ship.seed,
                "train_segments": len(ctx.train),
            }
        )
    )
    LOG.info(
        "R6 shipping %s seed %d: selection P1 %.4f m (dyn-v0.1 %.4f m)",
        config.name, ship.seed, scores[(config.name, ship.seed)], nominal_sel,
    )  # fmt: skip
    return 0


def load_fit(args) -> dict:  # pragma: no cover - real data
    return pickle.loads(_fit_pkl(args).read_bytes())


def r6_seeds(fit: dict) -> dict:
    """seed -> residual of the shipping config."""
    name = fit["shipping_config"]
    return {c.seed: c.residual for c in fit["frozen"].candidates if c.config.name == name}


def render_fit_summary(fit: dict, pkl_sha: str, notes) -> str:
    fr = fit["frozen"]
    lines = [
        "# R6 fit, frozen before the held-out pass",
        "",
        "Selection and train numbers only; no 2026-11-22 held-out number exists when this is written.",
        "",
        *[f"- {n}" for n in notes],
        f"- Frozen fit pickle SHA-256: {pkl_sha}.",
        f"- Samples: {fr.samples}.",
        "",
        f"Shipping: {fit['shipping_config']}, seed {fit['shipping_seed']} (chosen by selection P1).",
        "",
        (
            f"dyn-v0.1 selection P1 {fit['selection_nominal_p1']:.4f} m on "
            f"{fit['selection_windows']} selection windows; internal validation P1 "
            f"{fr.nominal_val['P1']:.4f} m."
        ),
        "",
        "| config | mean selection P1 (m) | min | per seed | mean internal validation P1 (m) |",
        "|---|---|---|---|---|",
    ]
    for name, row in fit["selection_table"].items():
        seeds = ", ".join(f"{k} {v:.4f}" for k, v in row.items() if k.startswith("seed"))
        val = fr.selection.get(name, {}).get("mean", float("nan"))
        lines.append(f"| {name} | {row['mean']:.4f} | {row['min']:.4f} | {seeds} | {val:.4f} |")
    return "\n".join(lines) + "\n"


def fit_summary_cmd(args) -> int:  # pragma: no cover - real data
    from .vehicle import repo_root

    root = repo_root()
    c = constants()
    fit = load_fit(args)
    notes = [
        f"Constants SHA-256 {c.sha256}.",
        f"Session file SHA-256 {sha256_file(root / prereg_v06.SESSION_RELATIVE)}.",
        f"Train segments: {fit['train_segments']} (every v0.5 train role plus the 2026-11-22 train parts).",
        "Selection: the 2026-11-14 held-out segments and the 2026-11-15 session held-out parts.",
        f"Recipe: R4's (stages_v04.r4_fit) on dyn-v0.1, configs {[x.name for x in r6_configs()]}, seeds {list(c.seeds)}.",
        f"Training device: {DEVICE_NOTE}.",
    ]
    text = render_fit_summary(fit, sha256_file(_fit_pkl(args)), notes)
    out = pointer_dir(root, args.date)
    out.mkdir(parents=True, exist_ok=True)
    (out / "fit-summary.md").write_text(text, encoding="utf-8")
    print(text)
    return 0


def write_bundle(
    args, dest: Path, git_sha: str, metrics: dict, data: list[str]
):  # pragma: no cover
    from . import export
    from .vehicle import load_vehicle, repo_root

    c = constants()
    fit = load_fit(args)
    res = r6_seeds(fit)[fit["shipping_seed"]]
    root = repo_root()
    v01 = prereg_v06_v01(root)
    created = dt.datetime.now(dt.timezone.utc).strftime("%Y-%m-%dT%H:%M:%SZ")
    cfg = export.config_sha256(
        {"prereg": c.sha256, "session": sha256_file(root / prereg_v06.SESSION_RELATIVE),
         "fit": sha256_file(_fit_pkl(args))}
    )  # fmt: skip
    return export.write_bundle(
        dest, model_id=model_id(args.date), created_utc=created, git_sha=git_sha,
        training_config_sha256=cfg, vehicle=load_vehicle(), nominal=v01, residual=res,
        v_max_mps=c.validity_v_max_mps, data=data, metrics=metrics, golden_residual_rollouts=True,
    )  # fmt: skip


def prereg_v06_v01(root: Path):  # pragma: no cover - real checkout
    from .cli import read_pointer_nominal

    return read_pointer_nominal(
        root / "training" / "pnp_models" / "dyn-v0.1-2026-11-12" / "manifest.yaml"
    ).nominal


def bench_bundle_cmd(args) -> int:  # pragma: no cover - real data
    """The R6 bundle for items 12 and 13 (planner solve time and the L5 canary), written under
    the cache before the pass. Its weights are the frozen ones; the final bundle is written again
    by `freeze-eval` with the gate result in its metrics."""
    dest = args.cache / "v06-bench" / model_id(args.date)
    write_bundle(
        args, dest, "0" * 40, {"status": "bench copy for GP2g-R items 12 and 13"}, ["bench copy"]
    )
    LOG.info("wrote %s", dest)
    return 0


def check_freeze(root: Path, sha: str, date: str):  # pragma: no cover - real checkout
    from . import truth_v04

    token = truth_v04.freeze(sha)
    if git(root, "rev-parse", "HEAD") != sha:
        raise SystemExit("--freeze-sha is not the HEAD of this checkout")
    if git(root, "status", "--porcelain", "--untracked-files=no"):
        raise SystemExit("the checkout has uncommitted changes: freeze first")
    for name in ("M-B3.md", "fit-summary.md", SOLVE_CANARY_FILE):
        require_committed(root, f"training/pnp_models/{model_id(date)}/{name}")
    return token


def saturation_on(es, nominal, fvehicle, residual, level: float) -> dict:  # pragma: no cover
    from . import gp2g
    from .evaluate import predict
    from .windows import WARM_N

    data = es.data(nominal.command_delay_s)
    states = predict(data, nominal, fvehicle, residual)[:, :-1]
    k = states.shape[1]
    ctrl = data.cmds[:, WARM_N : WARM_N + k, :]
    x = np.stack([states[..., 3], states[..., 4], ctrl[..., 0], ctrl[..., 1]], axis=-1).reshape(
        -1, 4
    )
    out = residual.forward(x)
    bound = gp2g.output_bound(residual)
    return {
        "share": gp2g.saturation_share(out, bound, level),
        "steps": len(x),
        "bound": bound.tolist(),
        "max_abs": np.abs(out).max(axis=0).tolist(),
    }


def freeze_eval_cmd(args) -> int:  # pragma: no cover - real data
    """The ONE held-out pass of GP2g-R. Takes no option that changes a model and refits nothing."""
    import shutil

    from . import gp2, gp2g, pass_v05, pose_check, scoring_v05
    from .cli_v03 import pose_recordings
    from .evaluate import eval_set
    from .pipeline import _json

    ctx = FitContextV06(args)
    root = ctx.root
    token = check_freeze(root, args.freeze_sha, args.date)
    c = ctx.c
    fit = load_fit(args)
    v01 = ctx.v01()
    seeds = r6_seeds(fit)
    ship = fit["shipping_seed"]
    ship_name = f"R6 seed {ship}"
    models = {V01: scoring_v05.Spec(v01)}
    for s, res in sorted(seeds.items()):
        models[f"R6 seed {s}"] = scoring_v05.Spec(v01, res)
    LOG.info("freeze %s; reading the 2026-11-22 held-out parts for the first time", token.git_sha)
    held_t2 = session22_segments(args, ctx.roles, ctx.vehicle, prereg_v06.HELDOUT, token, T2)
    held_t2s = session22_segments(args, ctx.roles, ctx.vehicle, prereg_v06.HELDOUT, token, T2S)
    recs = pose_recordings(args.bags, args.cache)
    older_held = [s for s in ctx.v05.stage1.older_all if s.split == "heldout" and s.run.has_odom]
    pose_segs = pose_check.build_pose_segments(older_held, recs)
    pair_segs = pose_check.build_pose_pair_segments(older_held, recs)
    p = pass_v05.score_pass(c, held_t2, held_t2s, ctx.fvehicle, models, v01, pose_segs, pair_segs)
    metrics = {n: p.metrics(n, []) for n in models}
    boot = p.bootstrap(ship_name, V01)
    lat = p.bootstrap(ship_name, V01, "across")
    sweep = gp2.stability_sweep(
        v01, seeds[ship], ctx.vehicle, c.validity_v_max_mps, samples=int(c.gate["sweep_samples"])
    )
    sat = saturation_on(p.es, v01, ctx.fvehicle, seeds[ship], float(c.gate["saturation"]["level"]))
    sc = json.loads((pointer_dir(root, args.date) / SOLVE_CANARY_FILE).read_text())
    solve = gp2g.SolveCost(**sc["solve"])
    canary = gp2g.Canary(**sc["canary"])
    result = gp2g.decide_gp2g_r(
        c, p.es.direction, p.strata, metrics[V01], metrics[ship_name], boot["hi"],
        {s: metrics[f"R6 seed {s}"].macro["P1"] for s in seeds}, sweep, sat["share"], solve, canary,
    )  # fmt: skip
    heading = gp2g.heading_outcome(c, metrics[V01], metrics[ship_name], lat["ratio"], lat["hi"])
    # the v0.2 split held-out segments: reported only, deciding nothing
    older_t2 = [s for s in older_held]
    reported = {}
    if older_t2:
        es_old = eval_set(older_t2)
        strata_old, errs_old = scoring_v05.score_models(es_old, models, ctx.fvehicle)
        reported = {n: scoring_v05.macro_at(errs_old[n][1.0], strata_old) for n in models}
    out = {
        "freeze_sha": token.git_sha,
        "constants_sha256": c.sha256,
        "session_sha256": ctx.roles.split.sha256,
        "windows": len(p.es),
        "windows_only_under_T2": p.windows_t2_only,
        "windows_only_under_T2s": p.windows_t2s_only,
        "direction_windows": {d: int(np.sum(p.es.direction == d)) for d in ("ccw", "cw")},
        "strata": dict(p.strata.counts),
        "strata_merges": list(p.strata.merges),
        "gate": result.to_dict(),
        "heading_outcome": heading.to_dict(),
        "bootstrap_P1": boot,
        "bootstrap_across": lat,
        "sweep": sweep.to_dict(),
        "saturation": sat,
        "solve_and_canary": sc,
        "metrics": {n: m.__dict__ for n, m in metrics.items()},
        "reported_v02_heldout_T2": reported,
        "shipping": {"config": fit["shipping_config"], "seed": ship},
    }
    folder = pointer_dir(root, args.date)
    (folder / "pass-result.json").write_text(
        json.dumps(_json(out), indent=2, default=str), encoding="utf-8"
    )
    report = render_pass_report(out, metrics, ship_name)
    (folder / "pass-report.md").write_text(report, encoding="utf-8")
    passed = result.outcome == gp2g.PASS
    dest_root = args.models if passed else args.models.parent / "models-candidate"
    data = [
        f"train role under T2: every v0.5 train role and the 2026-11-22 train parts, constants {c.sha256}",
        f"session file {ctx.roles.split.sha256}",
        "selection: the 2026-11-14 held-out segments and the 2026-11-15 held-out parts",
    ]
    manifest_metrics = {"GP2g-R": result.outcome, "heading_outcome": heading.label,
                        "P1_ratio": metrics[ship_name].macro["P1"] / metrics[V01].macro["P1"],
                        "Y1_ratio": metrics[ship_name].macro["Y1"] / metrics[V01].macro["Y1"]}  # fmt: skip
    bundle = dest_root / model_id(args.date)
    write_bundle(args, bundle, token.git_sha, _json(manifest_metrics), data)
    shutil.copy(bundle / "manifest.yaml", folder / "manifest.yaml")
    (folder / "metrics.json").write_text(
        json.dumps(_json(manifest_metrics), indent=2), encoding="utf-8"
    )
    (folder / "README.md").write_text(
        render_readme(out, args.date, bundle, passed), encoding="utf-8"
    )
    print(report)
    return 0


def render_items(decision: dict) -> list[str]:
    lines = ["| item | value | threshold | result | note |", "|---|---|---|---|---|"]
    for i in decision["items"]:
        res = "n/a" if i["comparison"] == "n/a" else ("PASS" if i["passed"] else "FAIL")
        lines.append(
            f"| {i['name']} | {i['value']:.6g} | {i['comparison']} {i['threshold']:g} | {res} | {i['note']} |"
        )
    return lines


def render_pass_report(out: dict, metrics: dict, ship_name: str) -> str:
    g = out["gate"]
    lines = [
        f"# GP2g-R held-out pass: {g['outcome']}",
        "",
        f"Freeze commit: `{out['freeze_sha']}`. Constants SHA-256 {out['constants_sha256']}; session file SHA-256 {out['session_sha256']}.",
        "",
        (
            f"Windows: {out['windows']} on identical windows under T2 and T2s "
            f"({out['windows_only_under_T2']} only under T2, {out['windows_only_under_T2s']} only under T2s, "
            f"dropped); per direction {out['direction_windows']}; strata {out['strata']}."
        ),
        "",
        f"Shipping: {out['shipping']['config']} seed {out['shipping']['seed']} ({ship_name}).",
        "",
        "## Item 0 enough data",
        "",
        *render_items(g["enough_data"]),
        "",
        "## GP2g-R items",
        "",
        *render_items(g["decision"]),
        "",
        f"Named outcome (not a deployment rule): {out['heading_outcome']['label']}.",
        "",
        "## Metrics at 1.0 s (macro over strata)",
        "",
        "| model | P1 (m) | A1 (m) | C1 (m) | Y1 (rad) | S1 (m/s) | P1 under T2s (m) | Yp (rad) |",
        "|---|---|---|---|---|---|---|---|",
    ]
    for n, m in metrics.items():
        a = m.macro
        lines.append(
            f"| {n} | {a['P1']:.4f} | {a['A1']:.4f} | {a['C1']:.4f} | {a['Y1']:.4f} | {a['S1']:.4f} | "
            f"{m.p1_alt:.4f} | {m.yp:.4f} |"
        )
    b = out["bootstrap_P1"]
    lines += [
        "",
        f"P1 ratio bootstrap: {b['ratio']:.4f} ({b['lo']:.4f} to {b['hi']:.4f}).",
        (
            f"Saturation: {out['saturation']['share']:.4%} of {out['saturation']['steps']} held-out "
            f"steps; bound {out['saturation']['bound']}, largest |output| "
            f"{out['saturation']['max_abs']}."
        ),
        "",
    ]
    if out["reported_v02_heldout_T2"]:
        lines += ["## Reported only: v0.2 split held-out segments under T2 (deciding nothing)", "",
                  "| model | P1 | Y1 | S1 |", "|---|---|---|---|"]  # fmt: skip
        for n, a in out["reported_v02_heldout_T2"].items():
            lines.append(f"| {n} | {a['P1']:.4f} | {a['Y1']:.4f} | {a['S1']:.4f} |")
        lines.append("")
    return "\n".join(lines) + "\n"


def render_readme(out: dict, date: str, bundle: Path, passed: bool) -> str:
    g = out["gate"]
    first = (
        "do-not-deploy (until GP2g-C)" if passed else "do-not-deploy (candidate, GP2g-R not passed)"
    )
    lines = [
        first,
        "",
        f"# {model_id(date)}",
        "",
        (
            "R6: the yaw-only residual on dyn-v0.1 (brief 54), shipping "
            f"{out['shipping']['config']} seed {out['shipping']['seed']}. Bundle schema 1.x, validity 1.3 m/s."
        ),
        "",
        f"GP2g-R: {g['outcome']}. Named outcome: {out['heading_outcome']['label']}.",
        "",
        (
            f"Bundle: `{bundle}`. Launch (only after GP2g-C and the owner's yes): "
            f"`dynamics_bundle:=/workspace/data/pnp/{bundle.parent.name}/{bundle.name}`; the "
            "default stays dyn-v0.1."
        ),
        "",
        "Pass report: `pass-report.md`; numbers: `pass-result.json`; fit: `fit-summary.md`; M-B3: `M-B3.md`.",
        "",
    ]
    return "\n".join(lines)


STAGE_COMMANDS = {
    "truth": truth_cmd,
    "mb3": mb3_cmd,
    "fit": fit_cmd,
    "fit-summary": fit_summary_cmd,
    "bench-bundle": bench_bundle_cmd,
    "freeze-eval": freeze_eval_cmd,
}


def main(argv: list[str]) -> int:  # pragma: no cover - real data
    import argparse

    parser = argparse.ArgumentParser(prog="racer_pnp_dynamics.stages_v06")
    parser.add_argument("command", choices=tuple(STAGE_COMMANDS))
    parser.add_argument("--bags", type=Path, default=DATA_ROOT / "bags")
    parser.add_argument("--models", type=Path, default=DATA_ROOT / "pnp" / "models")
    parser.add_argument("--cache", type=Path, default=DATA_ROOT / "pnp" / "cache")
    parser.add_argument("--workers", type=int, default=4)
    parser.add_argument("--date", default="2026-11-24")
    parser.add_argument("--freeze-sha", default=None)
    args = parser.parse_args(argv)
    logging.basicConfig(level=logging.INFO)
    return STAGE_COMMANDS[args.command](args)


if __name__ == "__main__":  # pragma: no cover
    import sys

    raise SystemExit(main(sys.argv[1:]))
