# records an mp4 of a trained policy driving, headless

from __future__ import annotations

import argparse
import os
from pathlib import Path

from stable_baselines3 import PPO, SAC

from .envs import build_env, load_config

ALGOS = {"sac": SAC, "ppo": PPO}


def main() -> None:
    parser = argparse.ArgumentParser(description="record a policy rollout to mp4")
    parser.add_argument("--model", required=True)
    parser.add_argument("--config", required=True)
    parser.add_argument("--out-dir", default="videos")
    parser.add_argument("--map", default=None)
    parser.add_argument("--speed-cap", type=float, default=None, help="override the config cap")
    parser.add_argument("--seed", type=int, default=0)
    parser.add_argument("--max-steps", type=int, default=3000)
    parser.add_argument("--name", default="rollout")
    args = parser.parse_args()

    # the opengl renderer needs a display; offscreen is the headless one
    os.environ.setdefault("QT_QPA_PLATFORM", "offscreen")
    import gymnasium as gym

    cfg = load_config(args.config)
    if args.speed_cap is not None:
        cfg["env"]["speed_cap_mps"] = args.speed_cap
    model = ALGOS[str(cfg["algo"].get("name", "sac")).lower()].load(args.model, device="cpu")

    out_dir = Path(args.out_dir)
    out_dir.mkdir(parents=True, exist_ok=True)
    env = build_env(cfg, seed_offset=args.seed, map_name=args.map, render_mode="rgb_array")
    env = gym.wrappers.RecordVideo(
        env, video_folder=str(out_dir), name_prefix=args.name, episode_trigger=lambda _: True
    )

    obs, _ = env.reset(seed=args.seed)
    for _ in range(args.max_steps):
        action, _ = model.predict(obs, deterministic=True)
        obs, _, terminated, truncated, _ = env.step(action)
        if terminated or truncated:
            break
    env.close()
    print(f"video written under {out_dir.resolve()}")


if __name__ == "__main__":
    main()
