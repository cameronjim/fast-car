"""Train one model of the frozen recipe (recipes/v2.yaml).

    uv run --group train --group bags python recipes/train_v2.py <full|F1|F2> <seed> [--out-root DIR] [--device mps]

Runs `racer_pnp_perception.train.train` with exactly the arguments of the recipe file; nothing is
typed here. Output: <out-root>/<model>/seed<seed>/ (model.pt, config.json, history.json).
"""

from __future__ import annotations

import argparse
import logging
import sys
from pathlib import Path

import yaml

RECIPE = Path(__file__).with_name("v2.yaml")
DEFAULT_OUT = Path("/Users/cameronjim/code/car-data/pnp/runs-v2")


def main(argv: list[str] | None = None) -> int:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("model", choices=["full", "F1", "F2"])
    parser.add_argument("seed", type=int)
    parser.add_argument("--out-root", type=Path, default=DEFAULT_OUT)
    parser.add_argument("--device", default="cpu", help="how it is run, not part of the recipe")
    args = parser.parse_args(argv)
    logging.basicConfig(level=logging.INFO, format="%(message)s")
    from racer_pnp_perception import train

    recipe = yaml.safe_load(RECIPE.read_text())
    t = recipe["train"]
    if args.seed not in t["seeds"]:
        parser.error(f"seed must be one of {t['seeds']}")
    labels = Path(t["labels_dir"])
    train.train(
        Path(t["sim_train"]),
        Path(t["sim_val"]),
        args.out_root / args.model / f"seed{args.seed}",
        epochs=int(t["epochs"]),
        batch_size=int(t["batch_size"]),
        lr=float(t["lr"]),
        seed=args.seed,
        real_paths=[labels / f"{b}.npz" for b in t["label_bags"]],
        real_fraction=float(t["real_fraction"]),
        init=Path(t["init"]),
        scale_aug=bool(recipe["added"]["scale_augmentation"]["enabled"]),
        scale_probability=float(recipe["added"]["scale_augmentation"]["probability"]),
        fold=recipe["models"][args.model]["fold"],
        device=args.device,
    )
    return 0


if __name__ == "__main__":
    sys.exit(main())
