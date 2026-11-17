"""The v0.5 stages (real-bag glue, brief 47). Stage 1 commands fit on TRAIN data only.

    mb5     M-B5: the T5 method on the data that exists before the 2026-11-15 session
    mb6     M-B6: the frozen v0.4 models on every PnP run of the session, whole (before the split)

The roles are those of `evaluation/prereg/pnp-v0.5-dynamics.yaml` `data_roles`. The 2026-11-14
held-out segments are the SELECTION set: they are read here, only through `selection_eval_set`,
only to choose a smoothness weight and to print P1, A1 and Y1; no fit function accepts them
(`table_fit_v05.RoleViolation`). Nothing in this module opens a 2026-11-15 held-out part.
"""

from __future__ import annotations

import json
import logging
import pickle
from pathlib import Path

import numpy as np

from . import ground_motion_v21, scoring_v05, session_v05, table_fit_v05, truth_v04, truth_v05
from .cli_v04 import OUT_DIR as V04_OUT_DIR  # noqa: F401  (documents where the v0.4 numbers are)
from .cli_v04 import Stage1, _find, segments_of_v04_bags
from .evaluate import EvalSet, eval_set

LOG = logging.getLogger(__name__)

# The freeze commit of the v0.4 held-out pass (PR 193). The 2026-11-14 held-out segments were read
# there; their truth cache is read here, as the SELECTION set, with this token.
V04_FREEZE_SHA = "416e68501c424caeb1f6dc80b6a0ee9edc7a288c"


def out_dir(root: Path, date: str) -> Path:  # pragma: no cover - real checkout
    return root / "training" / "pnp_models" / f"dyn-v0.5-{date}"


def constants() -> ground_motion_v21.ConstantsV05:  # pragma: no cover - real checkout
    return ground_motion_v21.load_constants_v05(_find(ground_motion_v21.V05_RELATIVE))


def selection_eval_set(args, ctx: Stage1) -> EvalSet:  # pragma: no cover - real data
    """The selection set: the 2026-11-14 held-out segments (K1, K2, X2b, X3b, X4b, X5b, X6b, X7b)
    under T2 as 1.5 s windows. Never fitted on; the only use is choosing settings and printing."""
    token = truth_v04.freeze(V04_FREEZE_SHA)
    segs = segments_of_v04_bags(args, ctx.v04, ctx.vehicle, "heldout", token)
    return eval_set([s for s in segs if s.run.has_odom])


def mb5(args) -> int:  # pragma: no cover - real data
    """M-B5. Train without 2026-11-15; selection on the 2026-11-14 held-out segments."""
    from .report_v05 import render_mb5

    ctx = Stage1(args)
    c = constants()
    v01 = ctx.v01().nominal
    win = table_fit_v05.build_t5_windows(ctx.train, v01, ctx.fvehicle, c)
    sel = selection_eval_set(args, ctx)
    LOG.info("M-B5: %d train windows, %d selection windows", len(win), len(sel))
    fit = table_fit_v05.fit_t5(win, sel, v01, ctx.fvehicle, c)
    gm = fit.groundmotion()
    base = table_fit_v05.score(sel, v01, ctx.fvehicle, table_fit_v05.unit_motion(c))
    mine = table_fit_v05.score(sel, v01, ctx.fvehicle, gm)
    (args.cache / "dyn-v0.5-mb5.pkl").write_bytes(
        pickle.dumps({"fit": fit, "t5": mine, "v01": base})
    )
    gs = c.doc["gate_support"]
    notes = [
        f"Constants file evaluation/prereg/pnp-v0.5-dynamics.yaml, SHA-256 {c.sha256}.",
        (
            f"Train windows: {len(win)} of 0.5 s (start every 0.25 s, no wheel speed sample under "
            f"{c.min_wheel_speed_mps:g} m/s) from the v0.2 split train segments, the legacy train "
            "part and the 2026-11-14 train segments (train and validation-of-train parts)."
        ),
        f"Selection windows: {len(sel)} (1.5 s windows of the 2026-11-14 held-out segments, T2).",
        (
            "Targets: ground distance = sum of the T2 ICP step lengths (the quantity M-B4 measures), "
            "yaw change = sum of the T2 yaw changes; inputs: the measured wheel speed and the "
            "nominal's steering state; base nominal dyn-v0.1."
        ),
    ]
    text = render_mb5(
        fit, mine, base,
        limit_alpha=float(gs["max_split_half_difference_alpha"]),
        limit_gamma=float(gs["max_split_half_difference_gamma"]),
        notes=notes,
    )  # fmt: skip
    folder = out_dir(ctx.root, args.date)
    folder.mkdir(parents=True, exist_ok=True)
    (folder / "M-B5.md").write_text(text, encoding="utf-8")
    print(text)
    return 0


def pnp_runs(cache: Path) -> dict[str, dict]:  # pragma: no cover - real data
    """The PnP runs of every bag of the session, from the run tables `tools/pnp_data.py runs-v05
    --json` wrote next to the other caches (kind `pnp`: a /pnp/planner/status stretch)."""
    out = {}
    for bag in session_v05.SESSION_BAGS:
        path = cache / f"{bag}.runs-v05.json"
        if not path.is_file():
            raise SystemExit(
                f"{path} is missing: run `uv run --project tools --extra session python "
                f"tools/pnp_data.py runs-v05 <bag> --json > {path}` first"
            )
        doc = json.loads(path.read_text())
        rows = [r for r in doc["runs"] if r["kind"] == "pnp"]
        if rows:
            out[bag] = {"first_scan_log_time_s": doc["first_scan_log_time_s"], "runs": rows}
    return out


def whole_run_segments(args, ctx_vehicle, runs: dict[str, dict]):  # pragma: no cover - real data
    """One whole-run segment per PnP run of the session with its T2 (the carry-over cache read by
    M-B6 only). The truth of a run is computed once and cached under pnp-v05-t2-whole."""
    from .bags import load_run
    from .bags import motion as scan_motion_tools
    from .dataset import Segment
    from .icp_motion import body_of, mount_of
    from .recordings import recorded_for

    segs = []
    for bag, info in runs.items():
        run = load_run(args.bags / bag, recorded_for(bag))
        t_ref = float(run.scans[0].log_time_s)
        stamps = np.array([sc.stamp_s for sc in run.scans])
        ignore = None
        for row in info["runs"]:
            path = truth_v05.whole_path(args.cache, bag, row["run_id"])
            if not path.is_file():
                if ignore is None:
                    ignore = scan_motion_tools.static_self_returns(
                        run.scans, mount_of(ctx_vehicle), body_of(ctx_vehicle)
                    )
                LOG.info("T2 of %s %s (%.1f s)", bag, row["run_id"], row["duration_s"])
                sl = truth_v04.compute_segment_slice(
                    run, ctx_vehicle, ignore, t_ref + row["start_s"], t_ref + row["end_s"],
                    args.workers, truth_v04.T2,
                )  # fmt: skip
                truth_v04.save_slice(path, sl)
            sl = truth_v05.load_whole_motion(
                args.cache, bag, row["run_id"], truth_v05.CARRY_OVER_PURPOSE
            )
            motion = truth_v04.assemble(stamps, [sl])
            i0 = int(np.searchsorted(run.t_s, t_ref + row["start_s"], side="left"))
            i1 = int(np.searchsorted(run.t_s, t_ref + row["end_s"], side="right"))
            name = f"{bag[11:19]}:{row['run_id']}"
            segs.append(Segment(run, motion, i0, i1, "heldout", name, "by_heading", None))
    return segs


def mb6(args) -> int:  # pragma: no cover - real data
    """M-B6: dyn-v0.1, N4, R4, T4, TL4 of the v0.4 freeze on every PnP run of the session, whole,
    under T2, on identical windows. Printed, deciding nothing. Committed before the split file."""
    from .pipeline import fit_vehicle_of
    from .report_v05 import render_mb6
    from .vehicle import load_vehicle, repo_root

    root = repo_root()
    vehicle = load_vehicle()
    fvehicle = fit_vehicle_of(vehicle)
    from .cli import read_pointer_nominal

    v01 = read_pointer_nominal(
        root / "training" / "pnp_models" / "dyn-v0.1-2026-11-12" / "manifest.yaml"
    ).nominal
    models, hashes = scoring_v05.frozen_v04_specs(args.cache, v01)
    runs = pnp_runs(args.cache)
    segs = whole_run_segments(args, vehicle, runs)
    es = eval_set([s for s in segs if s.run.has_odom and s.motion is not None])
    es = session_v05.with_heading_direction(es, segs)
    strata, errs = scoring_v05.score_models(es, models, fvehicle)
    summary = scoring_v05.summarise(es, strata, errs)
    (args.cache / "dyn-v0.5-mb6.json").write_text(json.dumps(_plain(summary), indent=2))
    notes = [
        f"Constants file evaluation/prereg/pnp-v0.5-dynamics.yaml, SHA-256 {ground_motion_v21.V05_SHA256}.",
        (
            "Runs: every PnP run of the session as tools/pnp_data finds it (a "
            f"/pnp/planner/status stretch): {sum(len(v['runs']) for v in runs.values())} runs "
            f"in {len(runs)} bags."
        ),
        (
            "Truth T2 of each whole run (wheel seeded ICP, flags of pnp-v0.4.yaml); windows as "
            "the gate builds them (1.5 s, start every 0.25 s, start speed at least 0.25 m/s); "
            "direction of a window from the measured T2 heading of its 5 s block, not from a "
            "block name."
        ),
        (
            "Models: dyn-v0.1; N4; R4 and TL4 at their shipping seeds, T4, all on dyn-v0.1 (the "
            "shipping nominal of the v0.4 pass), loaded from the v0.4 pickles under the cache."
        ),
    ]
    text = render_mb6(summary, notes, hashes)
    folder = out_dir(root, args.date)
    folder.mkdir(parents=True, exist_ok=True)
    (folder / "M-B6.md").write_text(text, encoding="utf-8")
    print(text)
    return 0


def _plain(obj):  # pragma: no cover - real data
    if isinstance(obj, dict):
        return {str(k): _plain(v) for k, v in obj.items()}
    if isinstance(obj, (list, tuple)):
        return [_plain(v) for v in obj]
    if isinstance(obj, np.generic):
        return obj.item()
    return obj


def part_meta(path: Path) -> dict[str, dict]:  # pragma: no cover - real data
    """Part name -> {bag, run, kind} of the session split file (the fields the split parser of
    the dataset module does not keep)."""
    import yaml

    raw = yaml.safe_load(path.read_text(encoding="utf-8"))
    return {
        str(s["name"]): {"bag": bag, "run": str(s["run"]), "kind": str(s["kind"])}
        for bag, body in raw["bags"].items()
        for s in body["segments"]
    }


def session_segments(  # pragma: no cover - real data
    args,
    split,
    meta: dict[str, dict],
    vehicle,
    role: str,
    token: truth_v04.FreezeToken | None = None,
    variant: str = truth_v04.T2,
    build_only: bool = False,
) -> list:
    """Segments (train and val parts, or held-out parts) of the session bags under T2 or T2s.

    Train parts are cached under pnp-v05-t2-train and held-out parts under pnp-v05-t2-heldout; a
    held-out part is only computed or read with the freeze token, so a fit stage (`role='train'`)
    cannot reach it. A missing T2 part of a PnP run is cut from the whole-run cache of M-B6 (bit
    equal to computing it); everything else is computed from the scans."""
    from .bags import load_run
    from .bags import motion as scan_motion_tools
    from .dataset import _t_ref, make_segments
    from .icp_motion import body_of, mount_of
    from .recordings import recorded_for

    out = []
    for name, entry in split.bags.items():
        wanted = [s for s in entry.segments if s.role == role]
        if not wanted:
            continue
        run = load_run(args.bags / name, recorded_for(name))
        t_ref = _t_ref(run, entry, None)
        stamps = np.array([s.stamp_s for s in run.scans])
        ignore = None
        slices = []
        for seg in wanted:
            path = (
                truth_v05.train_path(args.cache, name, seg.name, variant)
                if role == "train"
                else truth_v05.heldout_path(args.cache, name, seg.name, variant)
            )
            if not path.is_file():
                a, b = truth_v05.part_intervals(stamps, t_ref + seg.t_from_s, t_ref + seg.t_to_s)
                whole = truth_v05.whole_path(args.cache, name, meta[seg.name]["run"])
                if variant == truth_v04.T2 and whole.is_file():
                    sl = truth_v05.cut_slice(
                        truth_v05.load_whole_motion(
                            args.cache, name, meta[seg.name]["run"], truth_v05.SLICE_PURPOSE
                        ),
                        a,
                        b,
                    )
                else:
                    if ignore is None:
                        ignore = scan_motion_tools.static_self_returns(
                            run.scans, mount_of(vehicle), body_of(vehicle)
                        )
                    LOG.info("T2 (%s, %s) of %s/%s", role, variant or "T2", name, seg.name)
                    sl = truth_v04.compute_segment_slice(
                        run, vehicle, ignore, t_ref + seg.t_from_s, t_ref + seg.t_to_s,
                        args.workers, variant,
                    )  # fmt: skip
                truth_v04.save_slice(path, sl)
            if build_only:  # the cache file is written; nothing is read back
                continue
            if role == "train":
                slices.append(
                    truth_v05.load_train_motion(args.cache, name, seg.name, role, variant)
                )
            else:
                assert token is not None
                slices.append(
                    truth_v05.load_heldout_motion(args.cache, name, seg.name, role, token, variant)
                )
        if build_only:
            continue
        motion = truth_v04.assemble(stamps, slices)
        keep = ("train", "val") if role == "train" else ("heldout",)
        out.extend(s for s in make_segments(run, motion, split) if s.split in keep)
    return out


def session_truth(args) -> int:  # pragma: no cover - real data
    """T2 of every TRAIN part of the session, and T2 and T2s of every held-out part, into the
    train cache and the separate held-out cache (nothing is scored or read back here)."""
    from .vehicle import load_vehicle, repo_root

    root = repo_root()
    path = session_v05.require_committed_session(root)
    split = session_v05.load_session_split(path)
    meta = part_meta(path)
    vehicle = load_vehicle()
    session_segments(args, split, meta, vehicle, "train", build_only=True)
    for variant in (truth_v04.T2, truth_v04.T2S):  # written, never read back here: no token
        session_segments(args, split, meta, vehicle, "heldout", variant=variant, build_only=True)
    return 0


class FitContext:  # pragma: no cover - real data
    """What the fit stages share: the TRAIN segments (the older ones, the 2026-11-14 train segments
    and the train parts of the 2026-11-15 session) and the SELECTION windows. Built only after the
    session split file is written and committed (`require_committed_session`); it never builds a
    held-out part of the session."""

    def __init__(self, args) -> None:
        self.stage1 = Stage1(args)
        self.root = self.stage1.root
        path = session_v05.require_committed_session(self.root)
        split = session_v05.load_session_split(path)
        meta = part_meta(path)
        self.fvehicle = self.stage1.fvehicle
        self.vehicle = self.stage1.vehicle
        self.session_train = session_segments(args, split, meta, self.vehicle, "train")
        self.train = [*self.stage1.train, *self.session_train]
        self.selection = selection_eval_set(args, self.stage1)
        LOG.info(
            "train: %d segments (%d of the session); selection windows %d",
            len(self.train),
            len(self.session_train),
            len(self.selection),
        )

    def v01(self):
        return self.stage1.v01()


def fit_context(args) -> FitContext:  # pragma: no cover - real data
    return FitContext(args)


def _t5_pkl(args) -> Path:  # pragma: no cover - real data
    return args.cache / "dyn-v0.5-t5.pkl"


def _tl5_pkl(args) -> Path:  # pragma: no cover - real data
    return args.cache / "dyn-v0.5-tl5.pkl"


def t5_fit(args) -> int:  # pragma: no cover - real data
    """T5 on the full train set (v0.2 split, legacy, 2026-11-14 and the session's train parts); the
    smoothness weights are chosen on the selection set. Prints selection and train numbers only."""
    ctx = fit_context(args)
    c = constants()
    v01 = ctx.v01().nominal
    win = table_fit_v05.build_t5_windows(ctx.train, v01, ctx.fvehicle, c)
    fit = table_fit_v05.fit_t5(win, ctx.selection, v01, ctx.fvehicle, c)
    base = table_fit_v05.score(ctx.selection, v01, ctx.fvehicle, table_fit_v05.unit_motion(c))
    mine = table_fit_v05.score(ctx.selection, v01, ctx.fvehicle, fit.groundmotion())
    _t5_pkl(args).write_bytes(pickle.dumps({"fit": fit, "t5": mine, "v01": base}))
    LOG.info(
        "T5: %d windows; lambda alpha %g gamma %g; on a bound %s %s; selection P1 %.4f against %.4f",
        len(win), fit.lam_alpha, fit.lam_gamma, fit.on_bound_alpha, fit.on_bound_gamma,
        mine.p1, base.p1,
    )  # fmt: skip
    return 0


def tl5_fit(args) -> int:  # pragma: no cover - real data
    """TL5 on the frozen T5: two configurations, seeds 0, 1, 2, chosen by selection P1."""
    from . import fit_data_v04, learned_gm_v21

    ctx = fit_context(args)
    c = constants()
    v01 = ctx.v01().nominal
    t5 = pickle.loads(_t5_pkl(args).read_bytes())["fit"]
    train, val = fit_data_v04.fit_validation_split(ctx.train)
    ref = fit_data_v04.build_fit_windows(train, v01, ctx.fvehicle)
    ftr = fit_data_v04.build_fit_windows(train, v01, ctx.fvehicle, ref.scales)
    fva = fit_data_v04.build_fit_windows(val, v01, ctx.fvehicle, ref.scales)
    cands, ship, table = learned_gm_v21.fit_tl5(ftr, fva, ctx.selection, v01, ctx.fvehicle, c, t5)
    _tl5_pkl(args).write_bytes(
        pickle.dumps({"candidates": cands, "shipping": ship, "selection": table})
    )
    LOG.info("TL5: %s seed %d; %s", ship.config.name, ship.seed, table)
    return 0


def fit_summary(args) -> int:  # pragma: no cover - real data
    """fit-summary.md of the frozen candidates (selection and train numbers only)."""
    from .report_gp2f import render_fit_summary
    from .vehicle import repo_root

    root = repo_root()
    c = constants()
    t5 = pickle.loads(_t5_pkl(args).read_bytes())
    tl5 = pickle.loads(_tl5_pkl(args).read_bytes()) if _tl5_pkl(args).is_file() else None
    gs = c.doc["gate_support"]
    notes = [
        f"Constants file evaluation/prereg/pnp-v0.5-dynamics.yaml, SHA-256 {c.sha256}.",
        f"Session split file sha256 {session_v05_sha(root)}.",
        f"Train windows: {t5['fit'].counts['windows']} of 0.5 s.",
    ]
    text = render_fit_summary(
        t5["fit"], t5["t5"], t5["v01"], tl5, notes,
        float(gs["max_split_half_difference_alpha"]), float(gs["max_split_half_difference_gamma"]),
    )  # fmt: skip
    out = out_dir(root, args.date)
    out.mkdir(parents=True, exist_ok=True)
    (out / "fit-summary.md").write_text(text, encoding="utf-8")
    print(text)
    return 0


def session_v05_sha(root: Path) -> str:  # pragma: no cover - real checkout
    return scoring_v05.sha256_file(root / session_v05.SESSION_RELATIVE)


def git(root: Path, *args: str) -> str:  # pragma: no cover - real checkout
    import subprocess

    return subprocess.run(
        ["git", "-C", str(root), *args], capture_output=True, text=True, check=True
    ).stdout.strip()


def check_freeze(root: Path, sha: str) -> truth_v04.FreezeToken:  # pragma: no cover - real checkout
    """The freeze token of the held-out pass: the SHA must be the HEAD of a clean checkout, and
    the fit summary must be committed in it (the candidates are frozen in git)."""
    token = truth_v04.freeze(sha)
    if git(root, "rev-parse", "HEAD") != sha:
        raise SystemExit("--freeze-sha is not the HEAD of this checkout")
    if git(root, "status", "--porcelain", "--untracked-files=no"):
        raise SystemExit("the checkout has uncommitted changes: freeze first")
    tracked = set(git(root, "ls-files", "training/pnp_models").splitlines())
    if not any(p.endswith("/fit-summary.md") and "dyn-v0.5-" in p for p in tracked):
        raise SystemExit("fit-summary.md of v0.5 is not committed: freeze the candidates first")
    return token


def freeze_eval(args) -> int:  # pragma: no cover - real data
    """The ONE pass: GP2f-S, GP2f-T, GP2f-L, GP2f-V on the held-out parts of the session, with
    every metric of every model on identical windows, then the bundles and the pointers. It
    takes no option that changes a model and refits nothing."""
    import datetime as dt
    import shutil

    from . import export_v21, pass_v05, pose_check
    from .cli_v03 import pose_recordings
    from .ground_motion_v21 import GroundMotionV21
    from .pipeline import _json
    from .pipeline_v02 import v_max_of
    from .report_gp2f import (
        readme_v05,
        render_comparator,
        render_items,
        render_metric_tables,
        render_pass_report,
        render_strata,
    )

    ctx = fit_context(args)  # train data and selection windows (the rule input), no held-out
    repo = ctx.root
    rehearse = bool(getattr(args, "rehearse", False))
    if rehearse:
        # A dry run of every line of this stage on TRAIN parts in place of the held-out ones (the
        # numbers mean nothing: the models were fitted there), writing under a scratch root. It
        # never builds, opens or names a held-out truth, and its output is never committed.
        root = args.cache / "rehearsal-v05"
        args.models = root / "models"
        token = None  # a rehearsal has no freeze token, so it cannot open a held-out part
        LOG.info("REHEARSAL on train parts, scratch root %s", root)
    else:
        root = repo
        token = check_freeze(root, args.freeze_sha)
    freeze_sha = token.git_sha if token is not None else "0" * 40
    c = constants()
    v01 = ctx.v01().nominal
    t5 = pickle.loads(_t5_pkl(args).read_bytes())["fit"]
    tl5 = pickle.loads(_tl5_pkl(args).read_bytes())
    ship = tl5["shipping"]
    specs, hashes = scoring_v05.frozen_v04_specs(args.cache, v01)
    models = {"dyn-v0.1": specs["dyn-v0.1"]}
    models["T5"] = scoring_v05.Spec(v01, ground21=t5.groundmotion())
    tl5_motion = {}
    for cand in sorted(tl5["candidates"], key=lambda x: x.seed):
        if cand.config.name == ship.config.name:
            gm = GroundMotionV21(c, t5.alpha, t5.gamma, cand.learned, t5.supported)
            tl5_motion[cand.seed] = gm
            models[f"TL5 seed {cand.seed}"] = scoring_v05.Spec(v01, ground21=gm)
    for name in ("N4", "R4", "T4", "TL4"):
        models[name] = specs[name]
    path = session_v05.require_committed_session(repo)
    if rehearse:
        held_t2 = held_t2s = ctx.session_train  # train parts: no held-out part is opened
    else:
        LOG.info(
            "freeze %s; reading the held-out parts of the session for the first time",
            freeze_sha,
        )
        split = session_v05.load_session_split(path)
        meta = part_meta(path)
        held_t2 = session_segments(args, split, meta, ctx.vehicle, "heldout", token, truth_v04.T2)
        held_t2s = session_segments(args, split, meta, ctx.vehicle, "heldout", token, truth_v04.T2S)
    recs = pose_recordings(args.bags, args.cache)
    older_held = [s for s in ctx.stage1.older_all if s.split == "heldout" and s.run.has_odom]
    pose_segs = pose_check.build_pose_segments(older_held, recs)
    pair_segs = pose_check.build_pose_pair_segments(older_held, recs)
    p = pass_v05.score_pass(c, held_t2, held_t2s, ctx.fvehicle, models, v01, pose_segs, pair_segs)
    rule = v_max_of(ctx.train, ctx.vehicle)["v_max_mps"]
    tl5_name = f"TL5 seed {ship.seed}"
    sh = t5.split_half or {}
    result = pass_v05.decide_all(
        c, p,
        t5="T5", tl5_seeds={s: f"TL5 seed {s}" for s in tl5_motion}, tl5_shipping=tl5_name,
        v01="dyn-v0.1", on_bound=[*t5.on_bound_alpha, *t5.on_bound_gamma],
        split_half_alpha_max=float(np.nanmax(sh["alpha_diff"])) if sh else float("nan"),
        split_half_gamma_max=float(np.nanmax(sh["gamma_diff"])) if sh else float("nan"),
        t5_motion=t5.groundmotion(), tl5_motion=tl5_motion[ship.seed], nominal=v01,
        vehicle=ctx.fvehicle, rule_mps=float(rule),
        supported_alpha=t5.supported, supported_gamma=t5.supported,
    )  # fmt: skip
    names = [n for n in models if n != "dyn-v0.1"]
    names = ["dyn-v0.1", *names]
    speeds = [float(s["min_start_speed_mps"]) for s in c.doc["gate_validity"]["fast_items"]]
    metrics = {n: p.metrics(n, speeds) for n in names}
    comparator = render_comparator(pass_v05.comparator_rows(p, names))
    tables = render_metric_tables(metrics, names)
    strata = render_strata(dict(p.strata.counts), {n: metrics[n].stratum_pos for n in names})
    reported = ""
    older_t2 = [s for s in ctx.stage1.older_all if s.split == "heldout" and s.run.has_odom]
    if older_t2:
        es_old = eval_set(older_t2)
        strata_old, errs_old = scoring_v05.score_models(es_old, models, ctx.fvehicle)
        rows_old = [
            {"model": n, **scoring_v05.macro_at(errs_old[n][1.0], strata_old),
             "P1_T2s": float("nan"), "Pp": float("nan"), "Yp": float("nan")}
            for n in names
        ]  # fmt: skip
        reported = (
            render_comparator(rows_old) + "\nT2s, Pp and Yp are not computed for these segments.\n"
        )
    notes = [
        f"Constants SHA-256 {c.sha256}; session split file SHA-256 {session_v05_sha(repo)}.",
        (
            f"Windows: {len(p.es)} on identical windows under T2 and T2s ({p.windows_t2_only} only "
            f"under T2, {p.windows_t2s_only} only under T2s, dropped); direction of a window from "
            "the measured T2 heading of its 5 s block."
        ),
        "Pp and Yp are a re-read of G1b and T2b (the only pose data), labelled as such.",
        "Frozen v0.4 pickles (SHA-256): "
        + ", ".join(f"{k} {v[:16]}" for k, v in hashes.items())
        + ".",
        f"TL5 shipping: {ship.config.name} seed {ship.seed} (chosen by selection P1).",
    ]
    report = render_pass_report(
        pass_v05.decisions_to_dict(result),
        comparator, tables, strata, freeze_sha, notes, reported,
    )  # fmt: skip
    folder = out_dir(root, args.date)
    folder.mkdir(parents=True, exist_ok=True)
    (folder / "pass-report.md").write_text(report, encoding="utf-8")
    (folder / "pass-result.json").write_text(
        json.dumps(_json(pass_v05.decisions_to_dict(result)), indent=2, default=str),
        encoding="utf-8",
    )
    # bundles and pointers, whatever the gates said: every v0.5 bundle is do-not-deploy
    date = args.date or dt.datetime.now(dt.timezone.utc).strftime("%Y-%m-%d")
    created = dt.datetime.now(dt.timezone.utc).strftime("%Y-%m-%dT%H:%M:%SZ")
    data = [
        f"train role under T2, session split {session_v05_sha(repo)}, constants {c.sha256}",
        "selection: the 2026-11-14 held-out segments, never fitted on",
    ]
    sha_cfg = export_v21_config_sha(c, freeze_sha)

    def write(model_id, gm, v_max, gates, summary):
        bundle = args.models / model_id
        export_v21.write_bundle_v21(
            bundle, model_id=model_id, created_utc=created, git_sha=freeze_sha,
            training_config_sha256=sha_cfg, vehicle=ctx.vehicle, nominal=v01, ground=gm,
            v_max_mps=v_max, data=data, metrics=_json({"outcome": result["outcome"]}),
        )  # fmt: skip
        pointer = root / "training" / "pnp_models" / model_id
        pointer.mkdir(parents=True, exist_ok=True)
        shutil.copy(bundle / "manifest.yaml", pointer / "manifest.yaml")
        (pointer / "README.md").write_text(
            readme_v05(
                model_id, summary=summary,
                sections=[*(render_items(g.to_dict()) for g in gates)],
                data_lines=data,
            ),
            encoding="utf-8",
        )  # fmt: skip
        (pointer / "metrics.json").write_text(
            json.dumps(_json({"outcome": result["outcome"], "v_max_mps": v_max}), indent=2),
            encoding="utf-8",
        )

    v_t5 = result["validity_T5"].v_max_mps
    v_tl5 = result["validity_TL5"].v_max_mps
    write(
        f"dyn-v0.5t-{date}", t5.groundmotion(), v_t5,
        [result["support"], result["table"]],
        "T5: dyn-v0.1 plus the alpha and gamma tables (step contract 2.1).",
    )  # fmt: skip
    write(
        f"dyn-v0.5tl-{date}", tl5_motion[ship.seed], v_tl5,
        [result["support"], result["table"], result["learned"]],
        f"TL5: T5 plus the bounded learned correction, shipping seed {ship.seed}.",
    )  # fmt: skip
    print(report)
    return 0


def export_v21_config_sha(c, git_sha: str) -> str:  # pragma: no cover - real checkout
    from . import export

    return export.config_sha256({"prereg": c.sha256, "git": git_sha})


def confirm_cmd(args) -> int:  # pragma: no cover - real data
    """The fresh confirmation of GP2f-C item 2 on a session bag (the v0.5 form of `confirm`)."""
    from . import confirm_v05

    if args.bag is None or args.candidate is None or args.deployed is None:
        raise SystemExit("confirm needs --bag, --candidate and --deployed")
    result, c = confirm_v05.run_confirmation_v05(
        args.bag, args.candidate, args.deployed, args.cache, args.run, workers=args.workers
    )
    print(confirm_v05.format_confirmation_v05(result, c, args.candidate.name, args.deployed.name))
    return 0


STAGE_COMMANDS = {
    "confirm": confirm_cmd,
    "mb5": mb5,
    "mb6": mb6,
    "session-truth": session_truth,
    "t5-fit": t5_fit,
    "tl5-fit": tl5_fit,
    "fit-summary": fit_summary,
    "freeze-eval": freeze_eval,
}
