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


STAGE_COMMANDS = {"mb5": mb5, "mb6": mb6, "session-truth": session_truth}
