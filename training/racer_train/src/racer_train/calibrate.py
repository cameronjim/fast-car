"""Pre-training calibration: observation statistics and the OOD score distribution.

Both come from rollouts of the env BEFORE any learning, so they describe the world the
tracker drives in under the configured randomisation, not whatever the policy later does:

  * `collect_observations` -> the data `normalization.fit_normalization` is fitted on (half
    tracker-only, half uniform-random residual so the statistics cover mild excursions);
  * `ood_score_percentiles` -> how often the envelope's distance-based OOD scorer would fire
    for the tracker alone at a given threshold, used to choose the threshold and scale
    (08-learning.md: "OOD detection ... fall back to pure base controller") so fallback is a
    rare event in normal driving rather than a constant one.

    uv run python -m racer_train.calibrate --config <yaml> --steps 40000
"""

from __future__ import annotations

import argparse
from pathlib import Path

import numpy as np

from racer_train.config import ExperimentConfig, load_config
from racer_train.env import REPO_ROOT, ResidualRacerEnv
from racer_train.observation import OOD_STATE_DIM


def collect_observations(
    config: ExperimentConfig,
    n_steps: int,
    seed: int = 12345,
    repo_root: Path = REPO_ROOT,
    random_residual_fraction: float = 0.5,
) -> tuple[np.ndarray, np.ndarray]:
    """(observations N x D, ood_states N x 5) from `n_steps` env steps."""
    env = ResidualRacerEnv(config, repo_root=repo_root)
    rng = np.random.default_rng(seed)
    obs_rows: list[np.ndarray] = []
    ood_rows: list[tuple[float, ...]] = []
    try:
        episode = 0
        while len(obs_rows) < n_steps:
            obs, _ = env.reset(
                options={
                    "episode_seed": seed * 1000 + episode,
                    "residual_enabled": True,
                }
            )
            use_random = rng.random() < random_residual_fraction
            done = False
            while not done and len(obs_rows) < n_steps:
                obs_rows.append(obs)
                ood_rows.append(env._cur_ood_state)
                action = rng.uniform(-1.0, 1.0, size=2) if use_random else np.zeros(2)
                obs, _r, terminated, truncated, _info = env.step(action)
                done = bool(terminated or truncated)
            episode += 1
    finally:
        env.close()
    return np.asarray(obs_rows, dtype=np.float32), np.asarray(ood_rows, dtype=np.float64)[
        :, :OOD_STATE_DIM
    ]


def ood_score_percentiles(
    ood_states: np.ndarray,
    reference: tuple[float, ...],
    scale: tuple[float, ...],
    percentiles: tuple[float, ...] = (50.0, 90.0, 99.0, 99.9),
) -> dict[float, float]:
    ref = np.asarray(reference)
    sc = np.asarray(scale)
    scores = np.sqrt((((ood_states - ref) / sc) ** 2).sum(axis=1))
    return {p: float(np.percentile(scores, p)) for p in percentiles}


def main(argv: list[str] | None = None) -> int:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--config", type=Path, required=True)
    parser.add_argument("--steps", type=int, default=40000)
    args = parser.parse_args(argv)
    config = load_config(args.config)
    _obs, ood = collect_observations(config, args.steps, random_residual_fraction=0.0)
    print("tracker-only OOD state mean:", np.round(ood.mean(axis=0), 3))
    print("tracker-only OOD state std: ", np.round(ood.std(axis=0), 3))
    print(
        "OOD state p1/p99:",
        np.round(np.percentile(ood, 1, axis=0), 3),
        np.round(np.percentile(ood, 99, axis=0), 3),
    )
    env_cfg = config.envelope
    pct = ood_score_percentiles(ood, env_cfg.ood_reference_state, env_cfg.ood_reference_scale)
    print("configured threshold:", env_cfg.ood_threshold, "score percentiles:", pct)
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
