"""Real-bag entry points of dyn-v0.4 (thin glue, needs the bags; brief 37).

    python -m racer_pnp_dynamics.cli_v04 truth-train   # T2 of the train segments of 2026-11-14
    python -m racer_pnp_dynamics.cli_v04 budget        # M-B3 and M-B2 (train only, before any fit)
    python -m racer_pnp_dynamics.cli_v04 truth-variants  # T2s and clock shifted T2 (train, M-B4)
    python -m racer_pnp_dynamics.cli_v04 mb4           # M-B4 (train only, before any fit)

Stages that fit (`n4-fit`, `r4-fit`) and the single held-out pass (`freeze-eval`) are added by the
commits that introduce them. The v0.4 constants file is checked against its pre-registered SHA-256
on every run (`split_v04.load_split_v04`), and so are the v0.2 split and the v0.3 file.

Stage 1 (everything before `freeze-eval`) opens train segments only: `Context04` never builds a
held-out truth, and `truth_v04.load_stage1_motion` refuses one.
"""

from __future__ import annotations

import logging
from pathlib import Path

import numpy as np

from . import budget, label_sensitivity, split_v04, truth_v04
from .dataset import Segment, by_split, drop_nested, make_segments
from .evaluate import eval_set
from .icp_motion import SEED_WHEEL
from .pipeline import fit_vehicle_of
from .recordings import load_motions, recorded_for
from .split import Split, load_split
from .vehicle import load_vehicle, repo_root

LOG = logging.getLogger(__name__)

DATA_ROOT = Path("/Users/cameronjim/code/car-data")
MAIN_CHECKOUT = Path("/Users/cameronjim/code/car")
V01_BUNDLE = "dyn-v0.1-2026-11-12"
BASE_NAME = "dyn-v0.1"
OUT_DIR = "dyn-v0.4-2026-11-14"  # pointer directory of the stage 1 measurements
PREREG_V03_SHA256 = "4415b3719a29cfb08d5f5242ac6cdc5b35fee87e132c6d2939dfa0758808d7a1"
PREREG_V03_RELATIVE = Path("evaluation") / "prereg" / "pnp-v0.3.yaml"


def _find(relative: Path) -> Path:  # pragma: no cover - real checkout layout
    """A committed constants file: this checkout, else the main checkout."""
    for base in (repo_root(), MAIN_CHECKOUT):
        if (base / relative).is_file():
            return base / relative
    raise FileNotFoundError(f"{relative} is in neither this checkout nor {MAIN_CHECKOUT}")


def check_v03_file() -> None:  # pragma: no cover - real checkout layout
    import hashlib

    path = _find(PREREG_V03_RELATIVE)
    digest = hashlib.sha256(path.read_bytes()).hexdigest()
    if digest != PREREG_V03_SHA256:
        raise ValueError(f"{path} hashes to {digest}, not the pre-registered one")


def segments_of_v04_bags(  # pragma: no cover - real data
    args,
    split: Split,
    vehicle,
    role: str,
    token: truth_v04.FreezeToken | None = None,
    variant: str = truth_v04.T2,
) -> list[Segment]:
    """Segments (train and val parts, or heldout) of the 2026-11-14 bags. Train slices are cached
    under TRAIN_DIR, held-out slices under HELDOUT_DIR; held-out ones are only computed or read
    with the freeze token, so a stage 1 call (`role='train'`) cannot reach them."""
    from .bags import load_run
    from .bags import motion as scan_motion_tools
    from .dataset import _t_ref
    from .icp_motion import body_of, mount_of

    out: list[Segment] = []
    for name, entry in split.bags.items():
        wanted = [s for s in entry.segments if s.role == role]
        if not wanted:
            continue
        split_v04.require_split(name, repo_root())
        run = load_run(args.bags / name, recorded_for(name))
        t_ref = _t_ref(run, entry, None)
        stamps = [s.stamp_s for s in run.scans]
        ignore = None
        slices = []
        for seg in wanted:
            if role == "train":
                path = truth_v04.train_path(args.cache, name, seg.name, variant)
            else:
                path = truth_v04.heldout_path(args.cache, name, seg.name, variant)
            if not path.is_file():
                if ignore is None:
                    ignore = scan_motion_tools.static_self_returns(
                        run.scans, mount_of(vehicle), body_of(vehicle)
                    )
                LOG.info("T2 (%s) of %s/%s", role, name, seg.name)
                sl = truth_v04.compute_segment_slice(
                    run,
                    vehicle,
                    ignore,
                    t_ref + seg.t_from_s,
                    t_ref + seg.t_to_s,
                    args.workers,
                    variant,
                )
                truth_v04.save_slice(path, sl)
            if role == "train":
                slices.append(
                    truth_v04.load_stage1_motion(args.cache, name, seg.name, role, variant)
                )
            else:
                assert token is not None
                slices.append(
                    truth_v04.load_heldout_motion(args.cache, name, seg.name, role, token, variant)
                )
        motion = truth_v04.assemble(np.array(stamps), slices)
        keep = ("train", "val") if role == "train" else ("heldout",)
        out.extend(s for s in make_segments(run, motion, split) if s.split in keep)
    return out


class Stage1:  # pragma: no cover - real data
    """What every stage 1 command shares: vehicle, the split files, and the TRAIN segments (the
    train and internal validation parts) with their T2 motion. No held-out truth is built."""

    def __init__(self, args) -> None:
        from .bags import load_run
        from .cli import split_bags
        from .recordings import bag_dirs

        self.root = repo_root()
        check_v03_file()
        self.v04 = split_v04.load_split_v04(_find(split_v04.V04_RELATIVE))
        self.v02 = load_split(_find(Path("evaluation") / "prereg" / "pnp-v0.2-split.yaml"))
        self.split = split_v04.combined_split(self.v02, self.v04)
        self.vehicle = load_vehicle()
        self.fvehicle = fit_vehicle_of(self.vehicle)
        want = split_bags(self.v02)
        runs = drop_nested(
            [load_run(p, recorded_for(p.name)) for p in bag_dirs(args.bags) if p.name in want]
        )
        motions = load_motions(runs, self.vehicle, args.cache, SEED_WHEEL, args.workers)
        older = [s for r in runs for s in make_segments(r, motions.get(r.name), self.v02)]
        self.older_all = older  # incl. the v0.2 held-out segments, read only by `freeze-eval`
        self.older = [s for s in older if s.split in ("train", "val")]
        self.new = segments_of_v04_bags(args, self.v04, self.vehicle, "train")
        self.train = [s for s in self.new if s.split in ("train", "val")] + self.older
        LOG.info(
            "train data: %d older segments, %d segments of 2026-11-14",
            len(self.older),
            len(self.new),
        )

    def v01(self):
        from .cli import read_pointer_nominal

        return read_pointer_nominal(
            self.root / "training" / "pnp_models" / V01_BUNDLE / "manifest.yaml"
        )


def truth_train(args) -> int:  # pragma: no cover - real data
    """T2 of every train segment of 2026-11-14, cached per segment (about 45 minutes in all)."""
    ctx = Stage1(args)
    LOG.info("train segments with T2: %d", len(by_split(ctx.new, "train")))
    return 0


def truth_variants(args) -> int:  # pragma: no cover - real data
    """T2s and the two clock shifted truths of the train segments of 2026-11-14 (M-B4)."""
    ctx = Stage1(args)
    for variant in (truth_v04.T2S, truth_v04.CLOCK_PLUS, truth_v04.CLOCK_MINUS):
        segs = segments_of_v04_bags(args, ctx.v04, ctx.vehicle, "train", variant=variant)
        LOG.info("variant %s: %d segments", variant or "T2", len(segs))
    return 0


def mb4_cmd(args) -> int:  # pragma: no cover - real data
    """M-B4 on the train segments of 2026-11-14 (the variants are computed only there)."""
    ctx = Stage1(args)
    base = [s for s in ctx.new if s.split in ("train", "val") and s.run.has_odom]
    variants = {}
    for variant in (truth_v04.T2S, truth_v04.CLOCK_PLUS, truth_v04.CLOCK_MINUS):
        segs = segments_of_v04_bags(args, ctx.v04, ctx.vehicle, "train", variant=variant)
        variants[variant] = [s for s in segs if s.split in ("train", "val") and s.run.has_odom]
    shifts = {v: truth_v04.VARIANTS[v][1] for v in variants}
    table = label_sensitivity.sensitivity_table(base, variants, shifts)
    rejection = label_sensitivity.rejection_table(base)
    text = label_sensitivity.render_mb4(
        table,
        rejection,
        list(variants),
        [
            (
                "Segments: the train and internal validation parts of the train segments of "
                "2026-11-14 only; the variants are computed there and not on the older bags."
            ),
            f"Constants file evaluation/prereg/pnp-v0.4.yaml, SHA-256 {ctx.v04.sha256}.",
        ],
    )
    out = ctx.root / "training" / "pnp_models" / OUT_DIR
    out.mkdir(parents=True, exist_ok=True)
    (out / "M-B4.md").write_text(text, encoding="utf-8")
    print(text)
    return 0


def budget_cmd(args) -> int:  # pragma: no cover - real data
    """M-B3 and M-B2 for dyn-v0.1, train data only; writes the two markdown files."""
    ctx = Stage1(args)
    v01 = ctx.v01()
    es_all = eval_set([s for s in ctx.train if s.run.has_odom])
    b = budget.error_budget(es_all, v01.nominal, ctx.fvehicle)
    notes = [
        (
            f"Windows: {len(es_all)} (train and the internal validation part of the train "
            "segments of the v0.2 split, the legacy train part and the 2026-11-14 train "
            f"segments), strata {', '.join(b.strata.names)}."
        ),
        f"Constants file evaluation/prereg/pnp-v0.4.yaml, SHA-256 {ctx.v04.sha256}.",
        "Model: dyn-v0.1 nominal read from its committed pointer manifest, residual disabled.",
    ]
    out = ctx.root / "training" / "pnp_models" / OUT_DIR
    out.mkdir(parents=True, exist_ok=True)
    text = budget.render_mb3(b, BASE_NAME, notes)
    (out / "M-B3.md").write_text(text, encoding="utf-8")
    new_segs = [s for s in ctx.new if s.split in ("train", "val") and s.run.has_odom]
    es_new = eval_set(new_segs)
    rows = budget.asymmetry_table(new_segs, es_new, v01.nominal, ctx.fvehicle)
    radii = [budget.lap_radius(s) for s in new_segs]
    text2 = budget.render_mb2(
        rows,
        radii,
        BASE_NAME,
        [
            f"Windows: {len(es_new)} on the train segments of 2026-11-14 only.",
            (
                "Lap radius from wheel distance and the T2 yaw of each segment (brief 36's lap "
                "reference was not on main)."
            ),
        ],
    )
    (out / "M-B2.md").write_text(text2, encoding="utf-8")
    print(text)
    print(text2)
    return 0


def _stage(name: str):  # pragma: no cover - real data
    def run(args) -> int:
        from .stages_v04 import STAGE_COMMANDS

        return STAGE_COMMANDS[name](args)

    return run


COMMANDS = {
    "truth-train": truth_train,
    "truth-variants": truth_variants,
    "budget": budget_cmd,
    "mb4": mb4_cmd,
    **{
        n: _stage(n)
        for n in ("n4-fit", "r4-fit", "t4-fit", "tl4-fit", "fit-summary", "freeze-eval")
    },
}


def main(argv: list[str]) -> int:  # pragma: no cover - real data
    import argparse

    parser = argparse.ArgumentParser(prog="racer_pnp_dynamics.cli_v04")
    parser.add_argument("command", choices=tuple(COMMANDS))
    parser.add_argument("--bags", type=Path, default=DATA_ROOT / "bags")
    parser.add_argument("--models", type=Path, default=DATA_ROOT / "pnp" / "models")
    parser.add_argument("--cache", type=Path, default=DATA_ROOT / "pnp" / "cache")
    parser.add_argument("--workers", type=int, default=2)
    parser.add_argument("--date", default=None, help="freeze-eval: the date of the model ids")
    parser.add_argument("--freeze-sha", default=None, help="freeze-eval: the freeze commit")
    parser.add_argument(
        "--rehearse",
        action="store_true",
        help="freeze-eval: dry run on train segments, scratch root",
    )
    args = parser.parse_args(argv)
    logging.basicConfig(level=logging.INFO)
    return COMMANDS[args.command](args)


if __name__ == "__main__":  # pragma: no cover
    import sys

    raise SystemExit(main(sys.argv[1:]))
