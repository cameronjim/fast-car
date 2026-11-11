"""Paired simulator evaluation: tracker alone vs tracker + residual.

This is a SIM sanity evaluation of a pre-sysid v0 policy, NOT the pre-registered hardware
evaluation of claude-docs/09-evaluation.md. It borrows that protocol's structure where it
makes sense in sim:

  * PAIRED: both arms get the identical world (track, direction, randomisation draw, sensor
    noise stream, start). The only difference is whether the residual is applied. The
    tracker-only arm goes through the SAME envelope (same rate limits, same speed cap), so
    the comparison isolates the residual.
  * Lap time is reported over completed episodes only, ALONGSIDE the completion rate (DNFs
    are never imputed or dropped from the narrative).
  * Rates get Wilson intervals; the lap-time difference gets a percentile-bootstrap CI over
    the episodes BOTH arms completed.
  * Safety interventions: envelope events (residual clipped, rate limited, OOD fallback) are
    exact; the TTC-brake count is a PROXY (see `ResidualRacerEnv._ttc_proxy`), the real layer-3
    gate is C++ and not in this loop.

Torch-free; a `policy` is any callable `obs (np.ndarray) -> action (np.ndarray, shape (2,))`.
"""

from __future__ import annotations

import dataclasses
import math
from collections.abc import Callable, Sequence
from dataclasses import dataclass
from typing import Any

import numpy as np

from racer_train.env import ResidualRacerEnv

PolicyFn = Callable[[np.ndarray], np.ndarray]


@dataclass(frozen=True)
class EpisodeResult:
    track_label: str
    track_index: int
    episode_seed: int
    residual: bool
    success: bool
    crashed: bool
    timed_out: bool
    steps: int
    time_s: float
    laps: float
    lap_time_s: float | None
    mean_speed_mps: float
    mean_abs_cte_m: float
    max_abs_cte_m: float
    ttc_engagements: int
    clipped_fraction: float
    rate_limited_fraction: float
    ood_fraction: float
    mean_abs_residual_steering_rad: float
    mean_abs_residual_speed_mps: float

    def as_dict(self) -> dict[str, Any]:
        return dataclasses.asdict(self)


def run_episode(
    env: ResidualRacerEnv,
    options: dict[str, Any],
    policy: PolicyFn | None,
) -> EpisodeResult:
    """One episode. `policy is None` runs the tracker alone (residual disabled)."""
    opts = dict(options)
    opts["residual_enabled"] = policy is not None
    obs, info0 = env.reset(options=opts)
    n = 0
    cte_abs: list[float] = []
    speeds: list[float] = []
    clipped = rate_limited = ood = 0
    res_steer: list[float] = []
    res_speed: list[float] = []
    truth: dict[str, Any] = {}
    truncated = False
    zero = np.zeros(2, dtype=np.float32)
    while True:
        action = zero if policy is None else np.asarray(policy(obs), dtype=np.float32)
        obs, _r, terminated, truncated, info = env.step(action)
        n += 1
        truth = info["truth"]
        cte_abs.append(abs(truth["cross_track_error_m"]))
        speeds.append(truth["speed_mps"])
        e = info["envelope"]
        clipped += int(e["residual_clipped"])
        rate_limited += int(e["rate_limited"])
        ood += int(e["ood_triggered"])
        res_steer.append(abs(e["residual_command"]["steering_rad"]))
        res_speed.append(abs(e["residual_command"]["speed_mps"]))
        if terminated or truncated:
            break
    dt = env.config.env.timestep_s
    time_s = n * dt
    success = bool(truth["success"])
    laps = float(truth["laps"])
    return EpisodeResult(
        track_label=info0["track_label"],
        track_index=info0["track_index"],
        episode_seed=info0["episode_seed"],
        residual=policy is not None,
        success=success,
        crashed=bool(truth["crashed"]),
        timed_out=bool(truncated and not success and not truth["crashed"]),
        steps=n,
        time_s=time_s,
        laps=laps,
        lap_time_s=(time_s / laps) if success and laps > 0 else None,
        mean_speed_mps=float(np.mean(speeds)),
        mean_abs_cte_m=float(np.mean(cte_abs)),
        max_abs_cte_m=float(np.max(cte_abs)),
        ttc_engagements=int(truth["ttc_engagements"]),
        clipped_fraction=clipped / n,
        rate_limited_fraction=rate_limited / n,
        ood_fraction=ood / n,
        mean_abs_residual_steering_rad=float(np.mean(res_steer)),
        mean_abs_residual_speed_mps=float(np.mean(res_speed)),
    )


def evaluation_options(index: int, seed_base: int, n_tracks: int, laps: float) -> dict[str, Any]:
    """The world for paired episode `index`: track cycles through the pool (both directions
    are in the pool already), the seed fixes the draw, noise and start."""
    return {
        "episode_seed": seed_base + index,
        "track_index": index % n_tracks,
        "start": "standing",
        "laps": laps,
    }


def evaluate_paired(
    env: ResidualRacerEnv,
    policy: PolicyFn,
    n_episodes: int,
    seed_base: int,
    laps: float = 2.0,
) -> tuple[list[EpisodeResult], list[EpisodeResult]]:
    """(tracker-only results, tracker+residual results), index-aligned."""
    base: list[EpisodeResult] = []
    learned: list[EpisodeResult] = []
    n_tracks = len(env.track_pool)
    for i in range(n_episodes):
        options = evaluation_options(i, seed_base, n_tracks, laps)
        base.append(run_episode(env, options, None))
        learned.append(run_episode(env, options, policy))
    return base, learned


# -- statistics (tested against hand-computed answers in tests/test_evaluate_stats.py) --------


def wilson_interval(successes: int, n: int, z: float = 1.959964) -> tuple[float, float]:
    """Wilson score interval for a binomial proportion (09-evaluation.md: rates get Wilson
    intervals)."""
    if n <= 0:
        raise ValueError("n must be > 0")
    if not 0 <= successes <= n:
        raise ValueError("successes must be in [0, n]")
    p = successes / n
    denom = 1.0 + z * z / n
    centre = (p + z * z / (2.0 * n)) / denom
    half = z * math.sqrt(p * (1.0 - p) / n + z * z / (4.0 * n * n)) / denom
    return max(0.0, centre - half), min(1.0, centre + half)


def paired_bootstrap_ci(
    differences: Sequence[float],
    n_boot: int = 10000,
    alpha: float = 0.05,
    seed: int = 0,
) -> tuple[float, float, float]:
    """(mean, lo, hi): percentile-bootstrap CI of the mean of paired differences."""
    d = np.asarray(differences, dtype=float)
    if d.size == 0:
        raise ValueError("no paired differences")
    rng = np.random.default_rng(seed)
    idx = rng.integers(0, d.size, size=(n_boot, d.size))
    means = d[idx].mean(axis=1)
    lo, hi = np.quantile(means, [alpha / 2.0, 1.0 - alpha / 2.0])
    return float(d.mean()), float(lo), float(hi)


def summarize_pair(
    base: Sequence[EpisodeResult], learned: Sequence[EpisodeResult], seed: int = 0
) -> dict[str, Any]:
    """Headline numbers for one (tracker, tracker+residual) paired set."""
    if len(base) != len(learned) or not base:
        raise ValueError("base and learned must be equal-length, non-empty")
    n = len(base)

    def arm(results: Sequence[EpisodeResult]) -> dict[str, Any]:
        done = [r for r in results if r.success]
        lo, hi = wilson_interval(len(done), n)
        out: dict[str, Any] = {
            "episodes": n,
            "completed": len(done),
            "completion_rate": len(done) / n,
            "completion_wilson_95": (lo, hi),
            "crashes": sum(r.crashed for r in results),
            "timeouts": sum(r.timed_out for r in results),
            "mean_abs_cte_m": float(np.mean([r.mean_abs_cte_m for r in results])),
            "ttc_engagements_per_episode": float(np.mean([r.ttc_engagements for r in results])),
            "clipped_fraction": float(np.mean([r.clipped_fraction for r in results])),
            "rate_limited_fraction": float(np.mean([r.rate_limited_fraction for r in results])),
            "ood_fraction": float(np.mean([r.ood_fraction for r in results])),
            "mean_speed_mps": float(np.mean([r.mean_speed_mps for r in results])),
        }
        if done:
            out["lap_time_mean_s"] = float(np.mean([r.lap_time_s for r in done]))
        else:
            out["lap_time_mean_s"] = None
        return out

    both = [(b, ln) for b, ln in zip(base, learned, strict=True) if b.success and ln.success]
    paired: dict[str, Any] = {"n_both_completed": len(both)}
    if both:
        diffs = [ln.lap_time_s - b.lap_time_s for b, ln in both]
        rel = [100.0 * (ln.lap_time_s - b.lap_time_s) / b.lap_time_s for b, ln in both]
        mean, lo, hi = paired_bootstrap_ci(diffs, seed=seed)
        rmean, rlo, rhi = paired_bootstrap_ci(rel, seed=seed + 1)
        paired.update(
            {
                "lap_time_diff_mean_s": mean,
                "lap_time_diff_ci95_s": (lo, hi),
                "lap_time_diff_mean_pct": rmean,
                "lap_time_diff_ci95_pct": (rlo, rhi),
            }
        )
    return {"tracker": arm(base), "tracker_plus_residual": arm(learned), "paired": paired}
