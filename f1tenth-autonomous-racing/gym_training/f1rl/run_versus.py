# head-to-head eval: overtake rate, time to pass, and how the failures fail

from __future__ import annotations

import argparse
from pathlib import Path

import numpy as np

from .envs import build_env, load_config
from .evaluate import load_model

# a crash this close to the opponent is contact, not a solo mistake
CONTACT_RADIUS_M = 2.0

OUTCOMES = ("passed", "contact", "solo crash", "stuck behind")


def race_episode(model, env, seed: int) -> dict:
    """one deterministic head-to-head rollout, reported as a race outcome."""
    obs, info = env.reset(seed=seed)
    overtake_time_sec = None
    closest_m = float("inf")
    terminated = truncated = False
    while not (terminated or truncated):
        action = (
            np.zeros(2, dtype=np.float32)
            if model is None
            else model.predict(obs, deterministic=True)[0]
        )
        obs, _, terminated, truncated, info = env.step(action)
        closest_m = min(closest_m, float(info["opponent_distance_m"]))
        if "overtake_time_sec" in info:
            overtake_time_sec = float(info["overtake_time_sec"])

    ego_collision = bool(info["is_collision"])
    overtaken = bool(info["overtaken"])
    if ego_collision:
        outcome = "contact" if info["opponent_distance_m"] <= CONTACT_RADIUS_M else "solo crash"
    else:
        outcome = "passed" if overtaken else "stuck behind"
    return {
        "seed": seed,
        "overtaken": overtaken,
        "overtake_time_sec": overtake_time_sec,
        "ego_collision": ego_collision,
        "opponent_collision": bool(info["opponent_collision"]),
        "gap_m": float(info["gap_m"]),
        "closest_m": closest_m,
        "opponent_speed_cap_mps": float(info["opponent_speed_cap_mps"]),
        "laps": int(info["lap_count"]),
        "sim_time_sec": float(info["sim_time"]),
        "outcome": outcome,
    }


def summarize(races: list[dict]) -> dict:
    passes = [r["overtake_time_sec"] for r in races if r["overtake_time_sec"] is not None]
    return {
        "episodes": len(races),
        "overtake_rate": float(np.mean([r["overtaken"] for r in races])),
        "ego_collision_rate": float(np.mean([r["ego_collision"] for r in races])),
        "opponent_collision_rate": float(np.mean([r["opponent_collision"] for r in races])),
        "mean_time_to_pass_sec": float(np.mean(passes)) if passes else None,
        "clean_overtake_rate": float(
            np.mean([r["overtaken"] and not r["ego_collision"] for r in races])
        ),
        "outcomes": {name: sum(r["outcome"] == name for r in races) for name in OUTCOMES},
    }


def report(summary: dict, races: list[dict]) -> str:
    lines = [
        f"{'seed':>5} {'outcome':>12} {'pass at':>8} {'end gap':>8} {'closest':>8} "
        f"{'opp cap':>8} {'laps':>5}"
    ]
    for race in races:
        pass_at = "-" if race["overtake_time_sec"] is None else f"{race['overtake_time_sec']:.1f}s"
        lines.append(
            f"{race['seed']:>5} {race['outcome']:>12} {pass_at:>8} {race['gap_m']:>7.1f}m "
            f"{race['closest_m']:>7.1f}m {race['opponent_speed_cap_mps']:>7.2f} {race['laps']:>5}"
        )
    pass_time = summary["mean_time_to_pass_sec"]
    lines += [
        "",
        f"overtake success {summary['overtake_rate']:.0%} over {summary['episodes']} episodes",
        f"clean overtakes  {summary['clean_overtake_rate']:.0%}",
        f"ego collisions   {summary['ego_collision_rate']:.0%}",
        f"opponent crashes {summary['opponent_collision_rate']:.0%}",
        f"mean time to pass {'n/a' if pass_time is None else f'{pass_time:.2f}s'}",
        "outcomes " + ", ".join(f"{name} {count}" for name, count in summary["outcomes"].items()),
    ]
    return "\n".join(lines)


def main() -> None:
    parser = argparse.ArgumentParser(description="evaluate a policy against the scripted opponent")
    parser.add_argument("--config", required=True)
    parser.add_argument("--model", default=None, help="omit to drive the zero-action anchor")
    parser.add_argument("--episodes", type=int, default=20)
    parser.add_argument("--seed", type=int, default=0)
    parser.add_argument("--map", default=None)
    parser.add_argument("--opponent-cap", type=float, default=None, help="pin the opponent's cap")
    parser.add_argument("--out", default=None, help="write the report here as well as stdout")
    args = parser.parse_args()

    cfg = load_config(args.config)
    if not (cfg["env"].get("versus") or {}).get("enabled"):
        raise ValueError(f"{args.config} does not enable env.versus, so there is no opponent")
    if args.opponent_cap is not None:
        cfg["env"]["versus"]["opponent_speed_min_mps"] = args.opponent_cap
        cfg["env"]["versus"]["opponent_speed_max_mps"] = args.opponent_cap

    model = None if args.model is None else load_model(args.model, cfg["algo"].get("name", "sac"))
    env = build_env(cfg, seed_offset=args.seed, map_name=args.map)
    races = [race_episode(model, env, args.seed + episode) for episode in range(args.episodes)]
    env.close()

    text = report(summarize(races), races)
    print(text)
    if args.out is not None:
        path = Path(args.out)
        path.parent.mkdir(parents=True, exist_ok=True)
        path.write_text(text + "\n")
        print(f"report written to {path.resolve()}")


if __name__ == "__main__":
    main()
