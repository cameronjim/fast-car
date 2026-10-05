# drives a classical planner in the raw simulator at 100 hz and reports lap timing

from __future__ import annotations

import argparse
import os
from pathlib import Path

import numpy as np

from .planners import PurePursuitConfig, PurePursuitPlanner
from .track import raceline_index_from_csv

SIM_TIMESTEP_SEC = 0.01
POSE_FEATURES = ("pose_x", "pose_y", "pose_theta", "linear_vel_x")
DEFAULT_MAX_STEPS = 60000


def build_planner_env(
    map_name: str,
    seed: int = 0,
    render_mode: str | None = None,
    num_beams: int = 108,
    max_steps: int | None = None,
):
    """raw single-agent env exposing pose and speed, without the rl wrapper."""
    import gymnasium as gym
    from f1tenth_gym.envs.env_config import (
        EnvConfig,
        ObservationConfig,
        ResetConfig,
        SimulationConfig,
        TerminationConfig,
    )
    from f1tenth_gym.envs.lidar import LiDARConfig
    from f1tenth_gym.envs.observation import ObservationType
    from f1tenth_gym.envs.reset import ReferenceLine, ResetStrategy
    from f1tenth_gym.envs.wrappers import SingleAgentWrapper

    config = EnvConfig(
        seed=seed,
        map_name=map_name,
        num_agents=1,
        observation_config=ObservationConfig(
            type=ObservationType.FEATURES, features=POSE_FEATURES
        ),
        # the scan feeds collision detection even though the planner never reads it
        lidar_config=LiDARConfig(num_beams=num_beams, noise_std=0.0),
        reset_config=ResetConfig(
            strategy=ResetStrategy.RL_GRID_STATIC, reference_line=ReferenceLine.RACELINE
        ),
        simulation_config=SimulationConfig(timestep=SIM_TIMESTEP_SEC, max_laps=None),
        termination_config=TerminationConfig(
            max_episode_steps=max_steps, terminate_on_collision=True
        ),
        render_enabled=render_mode is not None,
    )
    inner = gym.make("f1tenth_gym:f1tenth-v0", config=config, render_mode=render_mode)
    return SingleAgentWrapper(inner)


def run_laps(env, planner, laps: int = 5, seed: int = 0, max_steps: int = DEFAULT_MAX_STEPS) -> dict:
    """drive until the lap target is reached or the episode ends, collecting per-lap times."""
    obs, _ = env.reset(seed=seed)
    lap_times: list[float] = []
    lap_count = 0
    top_speed_mps = 0.0
    collided = False
    steps = 0
    sim_time_sec = 0.0

    while steps < max_steps and lap_count < laps:
        speed_mps = float(obs["linear_vel_x"])
        top_speed_mps = max(top_speed_mps, speed_mps)
        steering, speed_cmd = planner.plan(
            float(obs["pose_x"]), float(obs["pose_y"]), float(obs["pose_theta"]), speed_mps
        )
        obs, _, terminated, truncated, info = env.step(
            np.array([steering, speed_cmd], dtype=np.float32)
        )
        steps += 1
        sim_time_sec = float(info["sim_time"])
        collided = collided or bool(np.asarray(info["collisions"]).reshape(-1)[0] > 0)
        counted = int(np.asarray(info["lap_counts"]).reshape(-1)[0])
        if counted > lap_count:
            lap_count = counted
            # lap_times holds the duration of the lap just finished, not a running total
            lap_times.append(float(np.asarray(info["lap_times"]).reshape(-1)[0]))
        if terminated or truncated:
            break

    return {
        "laps": lap_count,
        "lap_times_sec": lap_times,
        "collided": collided,
        "top_speed_mps": top_speed_mps,
        "steps": steps,
        "sim_time_sec": sim_time_sec,
    }


def summarize(result: dict) -> str:
    """one line naming laps, best and mean lap time, and how the run ended."""
    lap_times = result["lap_times_sec"]
    ending = "collision" if result["collided"] else "clean"
    if not lap_times:
        return (
            f"no lap completed in {result['sim_time_sec']:.1f}s of sim time, "
            f"top speed {result['top_speed_mps']:.2f} m/s, {ending}"
        )
    return (
        f"{len(lap_times)} laps, best {min(lap_times):.2f}s mean {np.mean(lap_times):.2f}s, "
        f"top speed {result['top_speed_mps']:.2f} m/s, {ending}"
    )


def add_planner_args(parser: argparse.ArgumentParser) -> None:
    """the pure pursuit tunables, shared by the runner and the leaderboard."""
    parser.add_argument("--lookahead-gain", type=float, default=None)
    parser.add_argument("--lookahead-min", type=float, default=None)
    parser.add_argument("--lookahead-max", type=float, default=None)
    parser.add_argument("--speed-scale", type=float, default=None)
    parser.add_argument("--speed-lookahead", type=float, default=None)
    parser.add_argument("--fallback-speed", type=float, default=None)


def config_from_args(args) -> PurePursuitConfig:
    """pure pursuit config with only the flags the caller actually passed applied."""
    overrides = {
        "lookahead_gain_sec": args.lookahead_gain,
        "lookahead_min_m": args.lookahead_min,
        "lookahead_max_m": args.lookahead_max,
        "speed_scale": args.speed_scale,
        "speed_lookahead_m": args.speed_lookahead,
        "fallback_speed_mps": args.fallback_speed,
    }
    return PurePursuitConfig.from_dict({k: v for k, v in overrides.items() if v is not None})


def main() -> None:
    parser = argparse.ArgumentParser(description="run a classical planner in the raw simulator")
    parser.add_argument("--map", default="Spielberg")
    parser.add_argument("--laps", type=int, default=5)
    parser.add_argument("--seed", type=int, default=0)
    parser.add_argument("--max-steps", type=int, default=DEFAULT_MAX_STEPS)
    parser.add_argument("--video", action="store_true")
    parser.add_argument("--video-dir", default="videos")
    parser.add_argument("--name", default="pure_pursuit")
    parser.add_argument("--centerline", action="store_true", help="lap the centerline instead")
    parser.add_argument(
        "--raceline-csv", default=None, help="lap this raceline csv instead of the map's shipped one"
    )
    add_planner_args(parser)
    args = parser.parse_args()

    # the opengl renderer needs a display; offscreen is the headless one
    os.environ.setdefault("QT_QPA_PLATFORM", "offscreen")
    import gymnasium as gym

    env = build_planner_env(
        args.map,
        seed=args.seed,
        render_mode="rgb_array" if args.video else None,
        max_steps=args.max_steps,
    )
    if args.raceline_csv and args.centerline:
        raise SystemExit("--raceline-csv and --centerline pick different lines, so pass only one")
    line = raceline_index_from_csv(args.raceline_csv) if args.raceline_csv else env.unwrapped.track
    planner = PurePursuitPlanner(line, config_from_args(args), use_centerline=args.centerline)
    if not planner.has_speed_profile:
        print(f"{args.map}: no raceline speed profile, driving at a fixed speed")

    if args.video:
        out_dir = Path(args.video_dir)
        out_dir.mkdir(parents=True, exist_ok=True)
        env = gym.wrappers.RecordVideo(
            env,
            video_folder=str(out_dir),
            name_prefix=f"{args.name}_{args.map}",
            episode_trigger=lambda _: True,
        )

    result = run_laps(env, planner, laps=args.laps, seed=args.seed, max_steps=args.max_steps)
    env.close()

    for index, lap_time in enumerate(result["lap_times_sec"], start=1):
        print(f"lap {index:>2}  {lap_time:6.2f}s")
    print(f"{args.map}: {summarize(result)}")
    if args.video:
        print(f"video written under {Path(args.video_dir).resolve()}")


if __name__ == "__main__":
    main()
