"""The fit stages and the single held-out pass of v0.4 (real-bag glue, brief 37b sections 4 to 7).

    n4-fit    N4 on train (the classical refit)                      -> cache dyn-v0.4-n4.pkl
    r4-fit    R4 on BOTH bases (dyn-v0.1 and N4), seeds 0 to 2       -> cache dyn-v0.4-r4-<base>.pkl
    t4-fit    T4 on both bases                                       -> cache dyn-v0.4-t4-<base>.pkl
    tl4-fit   TL4 on both bases (on the T4 of that base)             -> cache dyn-v0.4-tl4-<base>.pkl
    freeze-eval --freeze-sha <git sha>   the ONE held-out pass, decisions, bundles, pointers

Stage 1 commands open train segments only. `freeze-eval` refuses to run unless the SHA given is the
HEAD of a clean checkout (the freeze commit), builds the held-out truth for the first and only time
(T2 and T2s, in the held-out caches that stage 1 cannot open), scores every frozen model once and
writes the results. It takes no option that changes a model and refits nothing.

M-B3, M-B2 and M-B4 (cli_v04 `budget`, `mb4`) must have been committed before any of these runs;
`require_measurements_committed` checks the three files are tracked by git.
"""

from __future__ import annotations

import datetime as dt
import json
import logging
import pickle
import subprocess
from dataclasses import replace
from pathlib import Path

from . import (
    export,
    export_v2,
    fit_data_v04,
    gp2,
    ground_motion,
    learned_gm,
    pipeline_v02,
    pipeline_v04,
    pose_check,
    refit_v04,
    report_v04,
    table_fit,
    truth_v04,
)
from .cli_v04 import OUT_DIR, Stage1, _find, segments_of_v04_bags
from .pipeline import _json
from .residual_train import DEFAULT_CANDIDATES, disabled_residual

LOG = logging.getLogger(__name__)

MEASUREMENTS = ("M-B3.md", "M-B2.md", "M-B4.md")
SEEDS = (0, 1, 2)
DEVICE_NOTE = (
    "cpu, float64 (Apple MPS is available but has no float64 and the residual recipe is float64)"
)


def git(root: Path, *args: str) -> str:  # pragma: no cover - real checkout
    return subprocess.run(
        ["git", "-C", str(root), *args], capture_output=True, text=True, check=True
    ).stdout.strip()


def require_measurements_committed(root: Path) -> None:  # pragma: no cover - real checkout
    """No fit before M-B3, M-B2 and M-B4 are committed (the commit order is the evidence)."""
    tracked = set(git(root, "ls-files", f"training/pnp_models/{OUT_DIR}").splitlines())
    for name in MEASUREMENTS:
        rel = f"training/pnp_models/{OUT_DIR}/{name}"
        if rel not in tracked:
            raise SystemExit(f"{rel} is not committed: the measurements come before any fit")


def _pkl(args, name: str) -> Path:
    return args.cache / f"dyn-v0.4-{name}.pkl"


def _base_name(base: str) -> str:
    return base.replace(".", "").replace("-", "")


def n4_fit(args) -> int:  # pragma: no cover - real data
    ctx = Stage1(args)
    require_measurements_committed(ctx.root)
    result = refit_v04.fit_n4(ctx.train, ctx.fvehicle, ctx.v01().nominal)
    _pkl(args, "n4").write_bytes(pickle.dumps(result))
    out = ctx.root / "training" / "pnp_models" / OUT_DIR
    out.mkdir(parents=True, exist_ok=True)
    refit_v04.plot_profiles(out / "N4-profiles.png", result, "N4: rollout objective profiles")
    LOG.info("N4 parameters %s", result.nominal.to_dict())
    LOG.info(
        "on a bound %s; unidentified %s; cost %s", result.on_bound, result.unidentified, result.cost
    )
    return 0


def _bases(args, ctx):
    n4 = pickle.loads(_pkl(args, "n4").read_bytes())
    return {pipeline_v04.V01: ctx.v01().nominal, pipeline_v04.N4: n4.nominal}


def r4_fit(args) -> int:  # pragma: no cover - real data
    """R4 on both bases. The residual recipe is brief 37's (the v0.3 yaw-only configs) as written."""
    ctx = Stage1(args)
    require_measurements_committed(ctx.root)
    v_max = pipeline_v02.v_max_of(ctx.train, ctx.vehicle)["v_max_mps"]
    configs = tuple(replace(c, name=f"yaw_{c.name}", yaw_only=True) for c in DEFAULT_CANDIDATES)
    for base, nominal in _bases(args, ctx).items():
        frozen = pipeline_v02.strict_fit_stage(
            ctx.train, ctx.vehicle, nominal, v_max, configs=configs, seeds=SEEDS
        )
        _pkl(args, f"r4-{_base_name(base)}").write_bytes(pickle.dumps(frozen))
        LOG.info(
            "R4 on %s: shipping %s seed %d; device %s",
            base,
            frozen.shipping.config.name,
            frozen.shipping.seed,
            DEVICE_NOTE,
        )
    return 0


def _fit_windows(ctx, base_nominal, v01_nominal):
    """Train and validation FitWindows on `base_nominal` with the loss scales of dyn-v0.1."""
    train, val = fit_data_v04.fit_validation_split(ctx.train)
    ref = fit_data_v04.build_fit_windows(train, v01_nominal, ctx.fvehicle)
    ftr = fit_data_v04.build_fit_windows(train, base_nominal, ctx.fvehicle, ref.scales)
    fva = fit_data_v04.build_fit_windows(val, base_nominal, ctx.fvehicle, ref.scales)
    return ftr, fva


def t4_fit(args) -> int:  # pragma: no cover - real data
    ctx = Stage1(args)
    require_measurements_committed(ctx.root)
    c = ground_motion.load_constants(_find(ground_motion.ADDENDUM_RELATIVE))
    v01 = ctx.v01().nominal
    for base, nominal in _bases(args, ctx).items():
        ftr, fva = _fit_windows(ctx, nominal, v01)
        result = table_fit.fit_t4(ftr, fva, nominal, ctx.fvehicle, c)
        _pkl(args, f"t4-{_base_name(base)}").write_bytes(pickle.dumps(result))
        LOG.info(
            "T4 on %s: lambda %g, validation P1 %.4f against %.4f, on a bound %s",
            base,
            result.smoothness,
            result.validation_p1[result.smoothness],
            result.nominal_validation_p1,
            result.on_bound,
        )
    return 0


def tl4_fit(args) -> int:  # pragma: no cover - real data
    ctx = Stage1(args)
    require_measurements_committed(ctx.root)
    v01 = ctx.v01().nominal
    for base, nominal in _bases(args, ctx).items():
        t4 = pickle.loads(_pkl(args, f"t4-{_base_name(base)}").read_bytes())
        ftr, fva = _fit_windows(ctx, nominal, v01)
        cands, ship, table = learned_gm.fit_tl4(
            ftr, fva, nominal, ctx.fvehicle, t4.constants, t4.table
        )
        _pkl(args, f"tl4-{_base_name(base)}").write_bytes(
            pickle.dumps({"candidates": cands, "shipping": ship, "selection": table})
        )
        LOG.info(
            "TL4 on %s: %s seed %d; %s", base, ship.config.name, ship.seed, learned_gm.DEVICE_NOTE
        )
    return 0


def assemble_frozen(args, ctx) -> pipeline_v04.Frozen:  # pragma: no cover - real data
    n4 = pickle.loads(_pkl(args, "n4").read_bytes())
    v_max = pipeline_v02.v_max_of(ctx.train, ctx.vehicle)["v_max_mps"]
    r4, r4_seed, t4, t4_bound, tl4, tl4_seed = {}, {}, {}, {}, {}, {}
    for base in pipeline_v04.BASES:
        tag = _base_name(base)
        fr = pickle.loads(_pkl(args, f"r4-{tag}").read_bytes())
        name = fr.shipping.config.name
        r4[base] = {c.seed: c.residual for c in fr.candidates if c.config.name == name}
        r4_seed[base] = fr.shipping.seed
        t = pickle.loads(_pkl(args, f"t4-{tag}").read_bytes())
        t4[base] = t.groundmotion()
        t4_bound[base] = list(t.on_bound)
        tl = pickle.loads(_pkl(args, f"tl4-{tag}").read_bytes())
        ship = tl["shipping"]
        tl4[base] = {
            c.seed: ground_motion.GroundMotion(t.constants, t.table, c.learned)
            for c in tl["candidates"]
            if c.config.name == ship.config.name
        }
        tl4_seed[base] = ship.seed
    # the frozen v0.3 candidates, printed in the pass and deciding nothing
    y3 = pickle.loads((args.cache / "dyn-v0.3y-frozen.pkl").read_bytes())
    yr3 = pickle.loads((args.cache / "dyn-v0.3yr-frozen.pkl").read_bytes())["frozen"]
    v03 = {
        "dyn-v0.3y": (y3.nominal, None),
        "dyn-v0.3yr": (yr3.nominal, yr3.shipping.residual),
    }
    return pipeline_v04.Frozen(
        v01=ctx.v01().nominal,
        n4=n4.nominal,
        n4_on_bound=list(n4.on_bound),
        r4=r4,
        r4_shipping_seed=r4_seed,
        t4=t4,
        tl4=tl4,
        tl4_shipping_seed=tl4_seed,
        v_max_rule_mps=v_max,
        v03=v03,
        t4_on_bound=t4_bound,
    )


def _check_freeze(
    root: Path, sha: str
) -> truth_v04.FreezeToken:  # pragma: no cover - real checkout
    token = truth_v04.freeze(sha)
    if git(root, "rev-parse", "HEAD") != sha:
        raise SystemExit("--freeze-sha is not the HEAD of this checkout")
    if git(root, "status", "--porcelain", "--untracked-files=no"):
        raise SystemExit("the checkout has uncommitted changes: freeze first")
    return token


def freeze_eval(args) -> int:  # pragma: no cover - real data
    """The one pass. Everything frozen is read from the caches; held-out truth is built here."""
    ctx = Stage1(args)
    require_measurements_committed(ctx.root)
    rehearse = bool(getattr(args, "rehearse", False))
    frozen = assemble_frozen(args, ctx)
    c = ground_motion.load_constants(_find(ground_motion.ADDENDUM_RELATIVE))
    if rehearse:
        # A dry run of every line of this stage on TRAIN segments in place of the held-out ones
        # (the numbers mean nothing: the models were fitted there), writing under a scratch root.
        # It never builds, opens or names a held-out truth, and its output is never committed.
        scratch = args.cache / "rehearsal"
        ctx.root = scratch
        args.models = scratch / "models"
        token = truth_v04.FreezeToken("0" * 40)
        LOG.info("REHEARSAL on train segments, scratch root %s", scratch)
        held_t2 = [s for s in ctx.new if s.split in ("train", "val") and s.run.has_odom]
        held_t2s = [
            s
            for s in segments_of_v04_bags(
                args, ctx.v04, ctx.vehicle, "train", variant=truth_v04.T2S
            )
            if s.split in ("train", "val") and s.run.has_odom
        ]
    else:
        token = _check_freeze(ctx.root, args.freeze_sha)
        LOG.info(
            "freeze %s; reading the held-out segments of 2026-11-14 for the first time",
            token.git_sha,
        )
        held_t2 = segments_of_v04_bags(args, ctx.v04, ctx.vehicle, "heldout", token)
        held_t2s = segments_of_v04_bags(
            args, ctx.v04, ctx.vehicle, "heldout", token, variant=truth_v04.T2S
        )
    from .cli_v03 import pose_recordings

    recs = pose_recordings(args.bags, args.cache)
    older_held = [s for s in ctx.older_all if s.split == "heldout" and s.run.has_odom]
    pose_segs = pose_check.build_pose_segments(older_held, recs)
    pair_segs = pose_check.build_pose_pair_segments(older_held, recs)
    models = frozen.models()
    p = pipeline_v04.score_pass(held_t2, held_t2s, ctx.fvehicle, models, pose_segs, pair_segs)
    v_max = frozen.v_max_rule_mps
    result = pipeline_v04.decide(p, frozen, ctx.vehicle, min(v_max, gp2.BASE_VALIDITY_MPS))
    ship = result["shipping_nominal"]
    names = [
        pipeline_v04.V01, pipeline_v04.N4,
        *(n for n in models if n.startswith(("R4[", "T4[", "TL4["))),
        *(n for n in models if n.startswith("dyn-v0.3")),
    ]  # fmt: skip
    comparator = pipeline_v04.render_comparator(pipeline_v04.comparator_table(p, names))
    # the v0.2 held-out segments: printed, labelled, deciding nothing
    older_t2 = [s for s in ctx.older_all if s.split == "heldout"]
    read_before = ""
    if older_t2:
        pb = pipeline_v04.score_pass(older_t2, older_t2, ctx.fvehicle, models)
        read_before = pipeline_v04.render_comparator(
            pipeline_v04.comparator_table(pb, names)
        ).replace("under T2s", "under T2 (no T2s for these segments)")
        read_before += "\nPp and Yp are not computed here (see the main table).\n"
    out_dir = ctx.root / "training" / "pnp_models" / OUT_DIR
    out_dir.mkdir(parents=True, exist_ok=True)
    notes = [
        f"Split SHA-256 of 2026-11-14: {ctx.v04.sha256}; addendum constants SHA-256: {c.sha256}.",
        (
            f"Windows: {p.windows} on identical windows under T2 and T2s ({p.windows_t2_only} "
            f"only under T2, {p.windows_t2s_only} only under T2s, dropped)."
        ),
        f"Torch device of the learned fits: {DEVICE_NOTE}.",
        "Pp and Yp are a re-read of G1b and T2b (the only pose data), labelled as such.",
    ]
    report = report_v04.pass_report(result, comparator, token.git_sha, notes, read_before)
    (out_dir / "pass-report.md").write_text(report, encoding="utf-8")
    (out_dir / "pass-result.json").write_text(
        json.dumps(_json({"decisions": result, "metrics": p.to_dict()}), indent=2), encoding="utf-8"
    )
    export_all(args, ctx, frozen, p, result, ship, token.git_sha)
    print(report)
    return 0


def export_all(args, ctx, frozen, p, result, ship, git_sha) -> None:  # pragma: no cover - real data
    """Bundles under car-data/pnp/models and pointers under training/pnp_models, exported whether
    or not the gate passed (a failed candidate says so on the first README line)."""
    import shutil

    date = args.date or dt.datetime.now(dt.timezone.utc).strftime("%Y-%m-%d")
    created = dt.datetime.now(dt.timezone.utc).strftime("%Y-%m-%dT%H:%M:%SZ")
    root = ctx.root
    data = [
        f"train role under T2, split {ctx.v04.sha256}, addendum {ground_motion.ADDENDUM_SHA256}"
    ]

    def write(model_id, bundle_dir, readme_text, metrics):
        pointer = root / "training" / "pnp_models" / model_id
        pointer.mkdir(parents=True, exist_ok=True)
        shutil.copy(bundle_dir / export.MANIFEST_FILE, pointer / export.MANIFEST_FILE)
        (pointer / "README.md").write_text(readme_text, encoding="utf-8")
        (pointer / "metrics.json").write_text(
            json.dumps(_json(metrics), indent=2), encoding="utf-8"
        )

    sha_cfg = export.config_sha256({"prereg": ctx.v04.sha256, "addendum": c_sha(), "git": git_sha})

    def contract1(model_id, nominal, residual, v_max, gate, passed, summary):
        bundle = args.models / model_id
        export.write_bundle(
            bundle, model_id=model_id, created_utc=created, git_sha=git_sha,
            training_config_sha256=sha_cfg, vehicle=ctx.vehicle, nominal=nominal, residual=residual,
            v_max_mps=v_max, data=data, metrics=_json({"decision": gate}),
            golden_residual_rollouts=bool(residual.enabled),
        )  # fmt: skip
        text = report_v04.readme(
            model_id, contract2=False, passed=passed, summary=summary,
            sections=[report_v04.render_items(gate)], data_lines=data,
            device_note=DEVICE_NOTE if residual.enabled else "",
        )  # fmt: skip
        write(model_id, bundle, text, {"decision": gate})

    def contract2(model_id, nominal, ground, v_max, gate, passed, summary, extra):
        bundle = args.models / model_id
        export_v2.write_bundle_v2(
            bundle, model_id=model_id, created_utc=created, git_sha=git_sha,
            training_config_sha256=sha_cfg, vehicle=ctx.vehicle, nominal=nominal, ground=ground,
            v_max_mps=v_max, data=data, metrics=_json({"decision": gate}),
        )  # fmt: skip
        text = report_v04.readme(
            model_id, contract2=True, passed=passed, summary=summary,
            sections=[report_v04.render_items(gate), *extra], data_lines=data,
            device_note=learned_gm.DEVICE_NOTE if ground.learned else "",
        )  # fmt: skip
        write(model_id, bundle, text, {"decision": gate})

    v_nom = result["validity"]["v_max_mps"]
    gate_n = result["GP2d_N"]
    n_val = gp2.decide_validity(frozen.v_max_rule_mps, gate_n["passed"], p.fast[pipeline_v04.N4])
    contract1(f"dyn-v0.4n-{date}", frozen.n4, disabled_residual(), n_val.v_max_mps, gate_n,
              gate_n["passed"], "N4: the classical refit of dyn-v0.1 on the rollout objective.")  # fmt: skip
    r_name = ("dyn-v0.4nr-" if ship == pipeline_v04.N4 else "dyn-v0.4r-") + date
    r_gate = result["GP2d_R"]
    seed = frozen.r4_shipping_seed[ship]
    contract1(r_name, frozen.nominal_of(ship), frozen.r4[ship][seed], v_nom, r_gate,
              r_gate["passed"], f"R4: the yaw-only residual on {ship}, shipping seed {seed}.")  # fmt: skip
    contract2(f"dyn-v0.4t-{date}", frozen.nominal_of(ship), frozen.t4[ship],
              result["validity_T4"]["v_max_mps"], result["GP2e_T"], result["GP2e_T"]["passed"],
              f"T4: {ship} plus the ground motion table (step contract 2).",
              [report_v04.render_validity(result["validity_T4"])])  # fmt: skip
    tl_seed = frozen.tl4_shipping_seed[ship]
    contract2(f"dyn-v0.4tl-{date}", frozen.nominal_of(ship), frozen.tl4[ship][tl_seed],
              result["validity_TL4"]["v_max_mps"], result["GP2e_L"],
              result["GP2e_L"]["passed"],
              f"TL4: T4 plus the bounded learned correction, shipping seed {tl_seed}.",
              [report_v04.render_validity(result["validity_TL4"])])  # fmt: skip
    LOG.info("exported the v0.4 candidates for %s", date)


def c_sha() -> str:  # pragma: no cover - real checkout
    return ground_motion.ADDENDUM_SHA256


def render_fit_summary(n4, r4: dict, t4: dict, tl4: dict) -> str:
    """The frozen fits in one markdown file (validation numbers only, no held-out number): N4's
    parameters with bounds and flags, the validation selection of R4, the T4 tables with their
    support and ties, the validation selection of TL4."""
    lines = ["# v0.4 fits, frozen before the held-out pass", "", "## N4", ""]
    lines += ["| parameter | dyn-v0.1 | N4 | lower | upper | free in yaw fit | on a bound | flag |",
              "|---|---|---|---|---|---|---|---|"]  # fmt: skip
    for r in n4.parameter_table():
        lines.append(
            f"| {r['parameter']} | {r['dyn_v0.1']:.5g} | {r['n4']:.5g} | {r['lower']:.4g} | "
            f"{r['upper']:.4g} | {r['free_in_yaw_fit']} | {r['on_bound']} | {r['flag']} |"
        )
    lines += ["", f"Rollout objective on the validation windows: {n4.cost}.",
              f"Unidentified: {n4.unidentified or 'none'}.", ""]  # fmt: skip
    for base, fr in r4.items():
        lines += [f"## R4 on {base}", "",
                  f"Shipping {fr.shipping.config.name}, seed {fr.shipping.seed}.", "",
                  "| config | mean validation P1 | min | per seed |", "|---|---|---|---|"]  # fmt: skip
        for name, row in fr.selection.items():
            if name.startswith("_"):
                continue
            seeds = ", ".join(f"{k} {v:.4f}" for k, v in row.items() if k.startswith("seed"))
            lines.append(f"| {name} | {row['mean']:.4f} | {row['min']:.4f} | {seeds} |")
        lines.append("")
    for base, t in t4.items():
        text = (
            f"Smoothness weight {t.smoothness}; validation P1 by weight {t.validation_p1}; "
            f"base validation P1 {t.nominal_validation_p1:.4f}; supported nodes on a bound: "
            f"{t.on_bound or 'none'}."
        )
        head = "| steering share | speed (m/s) | alpha | supporting windows | tied to node | on a bound |"
        lines += [f"## T4 on {base}", "", text, "", head, "|---|---|---|---|---|---|"]
        for r in t.table_rows():
            lines.append(
                f"| {r['steering_share']:+.4f} | {r['speed_mps']:g} | {r['alpha']:.4f} | "
                f"{r['supporting_windows']} | {r['tied_to'] if r['tied_to'] is not None else ''} | "
                f"{r['on_bound']} |"
            )
        lines.append("")
    for base, d in tl4.items():
        ship = d["shipping"]
        lines += [f"## TL4 on {base}", "",
                  f"Shipping {ship.config.name}, seed {ship.seed}, device {ship.device}.", "",
                  "| config | mean validation P1 | min | per seed |", "|---|---|---|---|"]  # fmt: skip
        for name, row in d["selection"].items():
            seeds = ", ".join(f"{k} {v:.4f}" for k, v in row.items() if k.startswith("seed"))
            lines.append(f"| {name} | {row['mean']:.4f} | {row['min']:.4f} | {seeds} |")
        lines.append("")
    return "\n".join(lines) + "\n"


def fit_summary(args) -> int:  # pragma: no cover - real data
    root = Path(__file__).resolve().parents[4]
    n4 = pickle.loads(_pkl(args, "n4").read_bytes())
    bases = pipeline_v04.BASES
    r4 = {b: pickle.loads(_pkl(args, f"r4-{_base_name(b)}").read_bytes()) for b in bases}
    t4 = {b: pickle.loads(_pkl(args, f"t4-{_base_name(b)}").read_bytes()) for b in bases}
    tl4 = {b: pickle.loads(_pkl(args, f"tl4-{_base_name(b)}").read_bytes()) for b in bases}
    text = render_fit_summary(n4, r4, t4, tl4)
    out = root / "training" / "pnp_models" / OUT_DIR / "fit-summary.md"
    out.write_text(text, encoding="utf-8")
    print(text)
    return 0


STAGE_COMMANDS = {
    "fit-summary": fit_summary,
    "n4-fit": n4_fit,
    "r4-fit": r4_fit,
    "t4-fit": t4_fit,
    "tl4-fit": tl4_fit,
    "freeze-eval": freeze_eval,
}
