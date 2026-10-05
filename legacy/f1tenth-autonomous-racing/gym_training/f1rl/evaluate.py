# rolls a trained policy out deterministically and reports lap statistics

from __future__ import annotations

import argparse
import sys
from pathlib import Path

import numpy as np
from stable_baselines3 import PPO, SAC

from .envs import build_env, load_config
from .leaderboard import model_rows, pure_pursuit_rows, write_leaderboard
from .run_planner import add_planner_args, config_from_args

ALGOS = {"sac": SAC, "ppo": PPO}


def load_model(model_path: str, algo: str):
    """load an sb3 zip, failing loudly rather than half-initialised."""
    path = Path(model_path)
    if not path.is_file() and not path.with_suffix(".zip").is_file():
        raise FileNotFoundError(f"model not found: {path.resolve()}")
    algo = algo.lower()
    if algo not in ALGOS:
        raise ValueError(f"unknown algo {algo!r}, expected one of {sorted(ALGOS)}")
    return ALGOS[algo].load(str(path), device="cpu")


def run_episodes(model, env, episodes: int, seed: int) -> list[dict]:
    """one dict per episode with progress, lap timing, and how it ended."""
    results = []
    for episode in range(episodes):
        obs, _ = env.reset(seed=seed + episode)
        progress_m = 0.0
        steps = 0
        lap_times_sec: list[float] = []
        terminated = truncated = False
        collided = False
        while not (terminated or truncated):
            action, _ = model.predict(obs, deterministic=True)
            obs, _, terminated, truncated, info = env.step(action)
            progress_m += info["progress_m"]
            collided = collided or info["is_collision"]
            if "lap_time_sec" in info:
                lap_times_sec.append(float(info["lap_time_sec"]))
            steps += 1
        results.append(
            {
                "episode": episode,
                "progress_m": progress_m,
                "steps": steps,
                "laps": info["lap_count"],
                # lap 1 starts from a standstill, so the later laps are the honest lap time
                "lap_time_sec": lap_times_sec[0] if lap_times_sec else None,
                "lap_times_sec": lap_times_sec,
                "flying_lap_times_sec": lap_times_sec[1:],
                "collided": collided,
                "wrong_way": info["wrong_way"],
            }
        )
    return results


def run_leaderboard(args) -> None:
    """lap-time table for pure pursuit plus, when a model zip is given, the learned policy."""
    rows = pure_pursuit_rows(args.maps, args.laps, args.seed, config_from_args(args))
    episodes = None
    if args.model is not None:
        cfg = load_config(args.config)
        model = load_model(args.model, cfg["algo"].get("name", "sac"))
        episodes = args.episodes
        rows += model_rows(
            args.maps, model, cfg, episodes, args.seed, f"rl {cfg['algo'].get('name', 'sac')}"
        )
    out = write_leaderboard(args.out, rows, args.laps, episodes)
    print(f"leaderboard written to {out.resolve()}")


def main() -> None:
    parser = argparse.ArgumentParser(description="evaluate a trained racing policy")
    parser.add_argument("--model", default=None)
    parser.add_argument("--config", default=None)
    parser.add_argument("--episodes", type=int, default=20)
    parser.add_argument("--seed", type=int, default=0)
    parser.add_argument("--map", default=None)
    parser.add_argument("--speed-cap", type=float, default=None, help="override the config cap")
    parser.add_argument("--leaderboard", action="store_true")
    parser.add_argument("--maps", nargs="+", default=["Spielberg"])
    parser.add_argument("--laps", type=int, default=5)
    parser.add_argument("--out", default="leaderboard.md")
    add_planner_args(parser)
    args = parser.parse_args()

    if args.leaderboard:
        if args.model is not None and args.config is None:
            parser.error("--config is required to evaluate a model")
        run_leaderboard(args)
        return
    if args.model is None or args.config is None:
        parser.error("--model and --config are required unless --leaderboard is given")

    cfg = load_config(args.config)
    if args.speed_cap is not None:
        cfg["env"]["speed_cap_mps"] = args.speed_cap
    try:
        model = load_model(args.model, cfg["algo"].get("name", "sac"))
    except (FileNotFoundError, ValueError) as failure:
        print(f"could not load model: {failure}", file=sys.stderr)
        sys.exit(1)

    env = build_env(cfg, seed_offset=args.seed, map_name=args.map)
    print(
        f"{args.map or cfg['env']['maps'][0]}: {args.episodes} deterministic episodes at a "
        f"{env.action_bounds.speed_cap_mps:.2f} m/s cap"
    )
    results = run_episodes(model, env, args.episodes, args.seed)
    env.close()

    progress = np.array([r["progress_m"] for r in results])
    lap_times = [t for r in results for t in r["lap_times_sec"]]
    flying = [t for r in results for t in r["flying_lap_times_sec"]]
    collisions = np.array([r["collided"] for r in results])
    for r in results:
        laps = " ".join(f"{t:.2f}s" for t in r["lap_times_sec"]) or "no lap"
        print(
            f"episode {r['episode']:>3}  progress {r['progress_m']:8.1f}m  "
            f"steps {r['steps']:>5}  laps {r['laps']}  {laps}  "
            f"{'collision' if r['collided'] else 'clean'}"
        )
    print(
        f"progress mean {progress.mean():.1f}m std {progress.std():.1f}m, "
        f"collision free {1.0 - collisions.mean():.2%}, "
        f"laps completed {len(lap_times)} over {len(results)} episodes"
    )
    if lap_times:
        print(f"lap time best {min(lap_times):.2f}s mean {np.mean(lap_times):.2f}s")
    if flying:
        # lap 1 is a standing start, so this is the number worth comparing to a planner
        print(f"flying lap best {min(flying):.2f}s mean {np.mean(flying):.2f}s over {len(flying)} laps")


if __name__ == "__main__":
    main()
