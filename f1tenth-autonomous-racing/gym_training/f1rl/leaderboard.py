# builds the markdown lap-time table comparing controllers across maps

from __future__ import annotations

from datetime import date
from pathlib import Path

import numpy as np

from .planners import PurePursuitConfig, PurePursuitPlanner
from .run_planner import build_planner_env, run_laps

COLUMNS = ("map", "controller", "best lap", "mean lap", "crash rate", "top speed")


def _cell(value, unit: str) -> str:
    return "n/a" if value is None else f"{value:.2f} {unit}"


def make_row(map_name: str, controller: str, lap_times, crash_rate: float, top_speed_mps: float) -> dict:
    """one leaderboard row, with n/a lap times when the controller never finished a lap."""
    lap_times = list(lap_times)
    return {
        "map": map_name,
        "controller": controller,
        "best lap": _cell(min(lap_times) if lap_times else None, "s"),
        "mean lap": _cell(float(np.mean(lap_times)) if lap_times else None, "s"),
        "crash rate": f"{crash_rate:.0%}",
        "top speed": _cell(top_speed_mps, "m/s"),
    }


def render_table(rows: list[dict]) -> str:
    """markdown table, sorted by map then controller."""
    ordered = sorted(rows, key=lambda row: (row["map"], row["controller"]))
    lines = [
        "| " + " | ".join(COLUMNS) + " |",
        "| " + " | ".join("---" for _ in COLUMNS) + " |",
    ]
    lines += ["| " + " | ".join(str(row[name]) for name in COLUMNS) + " |" for row in ordered]
    return "\n".join(lines)


def _attempt(env, map_name: str, label: str, config, laps: int, seed: int, use_centerline: bool):
    planner = PurePursuitPlanner(env.unwrapped.track, config, use_centerline=use_centerline)
    result = run_laps(env, planner, laps=laps, seed=seed)
    row = make_row(
        map_name,
        label,
        result["lap_times_sec"],
        1.0 if result["collided"] else 0.0,
        result["top_speed_mps"],
    )
    return row, result


def pure_pursuit_rows(
    maps, laps: int, seed: int, config: PurePursuitConfig, centerline_config=None
) -> list[dict]:
    """run the classical baseline live on each map, one deterministic attempt per reference line."""
    rows = []
    for map_name in maps:
        env = build_planner_env(map_name, seed=seed)
        row, result = _attempt(env, map_name, "pure pursuit", config, laps, seed, False)
        rows.append(row)
        # some shipped racelines clear the walls by less than half a car width
        if not result["lap_times_sec"]:
            print(f"{map_name}: no lap on the raceline, retrying on the centerline")
            fallback, _ = _attempt(
                env,
                map_name,
                "pure pursuit (centerline)",
                centerline_config or config,
                laps,
                seed,
                True,
            )
            rows.append(fallback)
        env.close()
    return rows


def model_rows(maps, model, cfg: dict, episodes: int, seed: int, label: str) -> list[dict]:
    """run a trained policy live on each map through the rl wrapper it was trained with."""
    from .envs import build_env

    rows = []
    for map_name in maps:
        env = build_env(cfg, seed_offset=seed, map_name=map_name)
        speed_slice = env.obs_cfg.slices["linear_vel_x"]
        speed_norm = env.obs_cfg.speed_norm_mps
        lap_times: list[float] = []
        crashes = 0
        top_speed_mps = 0.0
        for episode in range(episodes):
            obs, _ = env.reset(seed=seed + episode)
            terminated = truncated = False
            while not (terminated or truncated):
                action, _ = model.predict(obs, deterministic=True)
                obs, _, terminated, truncated, info = env.step(action)
                # obs is clipped at 1.0, so this saturates above the speed normalizer
                top_speed_mps = max(top_speed_mps, float(obs[speed_slice][0]) * speed_norm)
                if "lap_time_sec" in info:
                    lap_times.append(float(info["lap_time_sec"]))
                if info["is_collision"]:
                    crashes += 1
        env.close()
        rows.append(make_row(map_name, label, lap_times, crashes / episodes, top_speed_mps))
    return rows


def write_leaderboard(path, rows: list[dict], laps: int, episodes: int | None) -> Path:
    """render the table into leaderboard.md with the run conditions above it."""
    path = Path(path)
    conditions = [
        f"Generated {date.today().isoformat()}.",
        f"Pure pursuit runs the raw 100 Hz env for {laps} laps per map, one deterministic attempt, "
        "so its crash rate is 0% or 100%.",
    ]
    if any("centerline" in row["controller"] for row in rows):
        conditions.append(
            "A centerline row means the map's shipped raceline runs closer to a wall than half "
            "the car's width, so no lap on it is possible at any speed."
        )
    if episodes is not None:
        conditions.append(
            f"The learned policy runs {episodes} deterministic episodes per map through the 25 Hz "
            "rl wrapper, and its top speed saturates at the observation normalizer."
        )
    body = "\n".join(f"{line}\n" for line in conditions)
    path.write_text(f"# Lap time leaderboard\n\n{body}\n{render_table(rows)}\n")
    return path
