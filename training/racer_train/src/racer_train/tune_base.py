"""Tune the BASE controller (the classical tracker) in sim before any residual is trained.

08-learning.md / 09-evaluation.md: the residual's base controller IS the evaluation baseline,
and a residual that "improves" a strawman tracker proves nothing. So before training, the
tracker's own knobs (pure-pursuit lookahead range and the raceline speed profile's lateral
acceleration and top speed) are swept in sim, under the SAME randomisation the residual will
train against, and the fastest setting that still completes laps reliably is the base. The
sweep is cheap and fully logged: `tuning budget = grid size x episodes`, recorded in the
output JSON and in the v0 note (09-evaluation.md: "tuning budget parity ... in logged hours").

Score (higher is better) = mean over episodes of (mean speed if the 2-lap run completed, else
0): it rewards speed AND completion in one number. A setting is only eligible if its
completion rate is at least `--min-completion`.

Tuning uses a track pool (seed base 3000) and episode seeds (60000+) that are disjoint from
the training pool (1000..) and from the evaluation pools (10000.. and 20000..).

    uv run python -m racer_train.tune_base --config <yaml> --out <json>
"""

from __future__ import annotations

import argparse
import dataclasses
import itertools
import json
import time
from concurrent.futures import ProcessPoolExecutor
from pathlib import Path
from typing import Any

import numpy as np

from racer_train.config import ExperimentConfig, load_config

TUNING_TRACK_SEED_BASE = 3000
TUNING_EPISODE_SEED_BASE = 60000


def with_base_settings(
    config: ExperimentConfig,
    lookahead_min_m: float,
    lookahead_max_m: float,
    lookahead_curvature_ref_1pm: float,
    a_lat_max_mps2: float,
    v_max_mps: float,
    pool_size: int,
    track_seed_base: int,
    max_episode_steps: int,
) -> ExperimentConfig:
    assert config.env.tracks is not None
    family = dataclasses.replace(
        config.env.tracks.family, a_lat_max_mps2=a_lat_max_mps2, v_max_mps=v_max_mps
    )
    tracks = dataclasses.replace(
        config.env.tracks,
        family=family,
        train_pool_size=pool_size,
        train_seed_base=track_seed_base,
    )
    env = dataclasses.replace(config.env, tracks=tracks, max_episode_steps=max_episode_steps)
    pp = dataclasses.replace(
        config.pure_pursuit,
        lookahead_min_m=lookahead_min_m,
        lookahead_max_m=lookahead_max_m,
        lookahead_curvature_ref_1pm=lookahead_curvature_ref_1pm,
    )
    return dataclasses.replace(config, env=env, pure_pursuit=pp)


def _evaluate_setting(args: tuple[str, dict[str, Any], int, int]) -> dict[str, Any]:
    config_path, setting, n_episodes, pool_size = args
    from racer_train.env import ResidualRacerEnv
    from racer_train.evaluate import evaluation_options, run_episode

    base_config = load_config(config_path)
    config = with_base_settings(
        base_config,
        pool_size=pool_size,
        track_seed_base=TUNING_TRACK_SEED_BASE,
        max_episode_steps=3000,
        **setting,
    )
    env = ResidualRacerEnv(config)
    results = []
    try:
        for i in range(n_episodes):
            options = evaluation_options(i, TUNING_EPISODE_SEED_BASE, len(env.track_pool), 2.0)
            results.append(run_episode(env, options, None))
    finally:
        env.close()
    done = [r for r in results if r.success]
    score = float(np.mean([r.mean_speed_mps if r.success else 0.0 for r in results]))
    return {
        "setting": setting,
        "episodes": n_episodes,
        "completion_rate": len(done) / n_episodes,
        "crashes": sum(r.crashed for r in results),
        "timeouts": sum(r.timed_out for r in results),
        "mean_speed_completed_mps": float(np.mean([r.mean_speed_mps for r in done]))
        if done
        else 0.0,
        "mean_abs_cte_m": float(np.mean([r.mean_abs_cte_m for r in results])),
        "score": score,
    }


def main(argv: list[str] | None = None) -> int:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--config", type=Path, required=True)
    parser.add_argument("--out", type=Path, required=True)
    parser.add_argument("--episodes", type=int, default=48)
    parser.add_argument("--pool-size", type=int, default=24)
    parser.add_argument("--workers", type=int, default=12)
    parser.add_argument("--min-completion", type=float, default=0.9)
    parser.add_argument("--lookahead-min", type=float, nargs="+", default=[0.3, 0.45, 0.6])
    parser.add_argument("--lookahead-max", type=float, nargs="+", default=[0.7, 1.0, 1.5])
    parser.add_argument("--curvature-ref", type=float, nargs="+", default=[0.4])
    parser.add_argument("--a-lat", type=float, nargs="+", default=[1.5, 2.0, 2.5, 3.0])
    parser.add_argument("--v-max", type=float, nargs="+", default=[1.6, 1.8, 2.0])
    args = parser.parse_args(argv)

    grid = [
        {
            "lookahead_min_m": lmin,
            "lookahead_max_m": lmax,
            "lookahead_curvature_ref_1pm": cref,
            "a_lat_max_mps2": a_lat,
            "v_max_mps": v_max,
        }
        for lmin, lmax, cref, a_lat, v_max in itertools.product(
            args.lookahead_min, args.lookahead_max, args.curvature_ref, args.a_lat, args.v_max
        )
        if lmax >= lmin
    ]
    t0 = time.time()
    tasks = [(str(args.config), s, args.episodes, args.pool_size) for s in grid]
    with ProcessPoolExecutor(max_workers=args.workers) as pool:
        results = list(pool.map(_evaluate_setting, tasks))
    wall_s = time.time() - t0
    eligible = [r for r in results if r["completion_rate"] >= args.min_completion]
    best = max(eligible, key=lambda r: r["score"]) if eligible else None
    payload = {
        "grid_size": len(grid),
        "episodes_per_setting": args.episodes,
        "total_episodes": len(grid) * args.episodes,
        "wall_clock_s": wall_s,
        "cpu_hours_approx": wall_s * args.workers / 3600.0,
        "min_completion": args.min_completion,
        "best": best,
        "results": sorted(results, key=lambda r: -r["score"]),
    }
    args.out.parent.mkdir(parents=True, exist_ok=True)
    args.out.write_text(json.dumps(payload, indent=2))
    print(f"{len(grid)} settings x {args.episodes} episodes in {wall_s:.0f}s; best={best}")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
