"""Choose a checkpoint by VALIDATION data only (never the held-out test pools).

Scores every `checkpoint_*.zip` of every run directory on a larger validation set (64 paired
episodes on the validation track pool, seeds disjoint from training/tuning/test), then ranks by
`score` (mean of mean-speed-if-completed-else-0, the same quantity the base was tuned on).
Ties and near-ties are not broken cleverly: the highest validation score wins, and the whole
table is written out so the choice can be audited.

    uv run python -m racer_train.select_checkpoint --config <yaml> --run-dirs <d1> <d2> ... --out <json>
"""

from __future__ import annotations

import argparse
import dataclasses
import json
from concurrent.futures import ProcessPoolExecutor
from pathlib import Path
from typing import Any

from racer_train.config import load_config
from racer_train.train import make_validation_env, validation_score


def _score_one(args: tuple[str, str, int, int]) -> dict[str, Any]:
    config_path, checkpoint, seed, n_episodes = args
    from stable_baselines3 import SAC

    from racer_train.calibrate import collect_observations
    from racer_train.env import REPO_ROOT
    from racer_train.normalization import fit_normalization
    from racer_train.observation import LIDAR_CLIP_M
    from racer_train.train import _policy_fn

    config = dataclasses.replace(load_config(config_path), seed=seed)
    val_env = make_validation_env(config, REPO_ROOT)
    try:
        obs, _ = collect_observations(config, config.sac.normalization_steps, seed=seed + 777)
        low = len(val_env.observation_fields)
        norm = fit_normalization(obs, low, LIDAR_CLIP_M)
        model = SAC.load(checkpoint, device="cpu")
        result = validation_score(val_env, _policy_fn(model, norm), n_episodes, {})
    finally:
        val_env.close()
    return {"checkpoint": checkpoint, "seed": seed, **result}


def main(argv: list[str] | None = None) -> int:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--config", type=Path, required=True)
    parser.add_argument("--run-dirs", type=Path, nargs="+", required=True)
    parser.add_argument("--seeds", type=int, nargs="+", required=True, help="seed of each run dir")
    parser.add_argument("--episodes", type=int, default=64)
    parser.add_argument("--workers", type=int, default=8)
    parser.add_argument("--out", type=Path, required=True)
    args = parser.parse_args(argv)
    if len(args.seeds) != len(args.run_dirs):
        parser.error("--seeds must have one entry per --run-dirs")
    tasks = [
        (str(args.config), str(ck), seed, args.episodes)
        for d, seed in zip(args.run_dirs, args.seeds, strict=True)
        for ck in sorted(d.glob("checkpoint_*.zip"))
    ]
    with ProcessPoolExecutor(max_workers=args.workers) as pool:
        rows = list(pool.map(_score_one, tasks))
    rows.sort(key=lambda r: -r["score_delta"])
    args.out.write_text(json.dumps({"episodes": args.episodes, "ranked": rows}, indent=2))
    for r in rows[:8]:
        print(
            Path(r["checkpoint"]).parent.name,
            Path(r["checkpoint"]).name,
            f"delta={r['score_delta']:+.3f} completion {r['completion_learned']:.2f} vs {r['completion_tracker']:.2f} lap% {r['lap_time_diff_pct']:+.2f}",
        )
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
