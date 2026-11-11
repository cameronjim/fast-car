"""SAC training entry point (roadmap S.3, extended for the v0 run): config in, checkpoints +
TensorBoard logs + a loadable deployment contract directory out.

Boring-choices justification (claude-docs/08-learning.md "Algorithm: SAC (fallback PPO). No
algorithm shopping without a logged reason", CLAUDE.md): stable-baselines3 is used as a
maintained dependency rather than a hand-rolled SAC implementation -- it is the standard,
well-tested reference implementation for this algorithm, and hand-rolling actor-critic +
replay buffer + target-network-update machinery would only add a second, unrelated thing to
debug on top of this task's actual scope (the residual/envelope/contract wiring).

What a run does, in order:

  1. build the env (procedural track pool + domain randomisation when configured);
  2. FIT the frozen observation normalisation from pre-training rollouts
     (`racer_train.calibrate`, `racer_train.normalization`);
  3. train SAC on normalised observations, logging to TensorBoard, saving periodic
     checkpoints, and scoring each checkpoint on a held-aside VALIDATION pool (paired against
     the tracker alone) so a checkpoint is chosen on validation data, not on training return;
  4. export the best-by-validation checkpoint as a policy directory
     (`racer_train.policy_export`).

torch and stable_baselines3 are imported LAZILY inside functions, not at module level (same
pattern as ros_ws/src/racer_policy/src/racer_policy/model.py's `load_model`), so importing
this module -- e.g. `python -m racer_train.train --help` -- never requires them installed.
Running an actual training job does require them (`uv sync --group train`, see this
package's pyproject.toml).

Reproducibility (08-learning.md): a policy is reproducible from config file + git SHA + seed;
all three are written into `policy_meta.yaml`.
"""

from __future__ import annotations

import argparse
import dataclasses
import json
import time
from pathlib import Path
from typing import Any

import gymnasium as gym
import numpy as np

from racer_train.calibrate import collect_observations
from racer_train.config import ExperimentConfig, load_config
from racer_train.env import REPO_ROOT, ResidualRacerEnv
from racer_train.evaluate import evaluation_options, run_episode
from racer_train.normalization import ObsNormalization, fit_normalization
from racer_train.observation import LIDAR_CLIP_M
from racer_train.policy_export import export_policy_dir

# Validation pool / episode seeds: disjoint from training (1000..), tuning (3000.., 60000..)
# and the held-out test pools (10000.. and 20000.., see racer_train.heldout_report).
VALIDATION_TRACK_SEED_BASE = 5000
VALIDATION_EPISODE_SEED_BASE = 70000


class NormalizedObsEnv(gym.ObservationWrapper):
    """Applies the FROZEN `ObsNormalization` to every observation the agent sees."""

    def __init__(self, env: gym.Env, norm: ObsNormalization) -> None:
        super().__init__(env)
        self._norm = norm

    def observation(self, observation: np.ndarray) -> np.ndarray:
        return self._norm.normalize(observation)


def make_validation_env(config: ExperimentConfig, repo_root: Path) -> ResidualRacerEnv:
    """An env over the validation track pool, long enough for 2-lap standing-start runs."""
    cfg = config
    if config.env.tracks is not None:
        tracks = dataclasses.replace(
            config.env.tracks, train_pool_size=16, train_seed_base=VALIDATION_TRACK_SEED_BASE
        )
        cfg = dataclasses.replace(
            config, env=dataclasses.replace(config.env, tracks=tracks, max_episode_steps=3000)
        )
    return ResidualRacerEnv(cfg, repo_root=repo_root)


def _policy_fn(sac_model: Any, norm: ObsNormalization):
    def policy(obs: np.ndarray) -> np.ndarray:
        action, _ = sac_model.predict(norm.normalize(obs), deterministic=True)
        return action

    return policy


def validation_score(
    val_env: ResidualRacerEnv,
    policy,
    n_episodes: int,
    baseline_cache: dict[str, Any],
) -> dict[str, float]:
    """Paired validation of `policy` against the tracker alone on the same worlds.

    score = mean over episodes of (mean speed if the 2-lap run completed else 0): speed and
    completion in one number (the same quantity `racer_train.tune_base` ranks bases by).
    """
    n_tracks = len(val_env.track_pool)
    if "base" not in baseline_cache or baseline_cache.get("n") != n_episodes:
        baseline_cache["base"] = [
            run_episode(
                val_env,
                evaluation_options(i, VALIDATION_EPISODE_SEED_BASE, n_tracks, 2.0),
                None,
            )
            for i in range(n_episodes)
        ]
        baseline_cache["n"] = n_episodes
    base = baseline_cache["base"]
    learned = [
        run_episode(
            val_env, evaluation_options(i, VALIDATION_EPISODE_SEED_BASE, n_tracks, 2.0), policy
        )
        for i in range(n_episodes)
    ]

    def score(rs):
        return float(np.mean([r.mean_speed_mps if r.success else 0.0 for r in rs]))

    done_both = [(b, ln) for b, ln in zip(base, learned, strict=True) if b.success and ln.success]
    lap_pct = (
        float(
            np.mean([100.0 * (ln.lap_time_s - b.lap_time_s) / b.lap_time_s for b, ln in done_both])
        )
        if done_both
        else float("nan")
    )
    return {
        "score_tracker": score(base),
        "score_learned": score(learned),
        "score_delta": score(learned) - score(base),
        "completion_tracker": float(np.mean([r.success for r in base])),
        "completion_learned": float(np.mean([r.success for r in learned])),
        "lap_time_diff_pct": lap_pct,
        "ood_fraction": float(np.mean([r.ood_fraction for r in learned])),
        "clipped_fraction": float(np.mean([r.clipped_fraction for r in learned])),
    }


def train(
    config: ExperimentConfig,
    config_path: Path,
    output_dir: Path,
    repo_root: Path = REPO_ROOT,
    log_dir: Path | None = None,
    torch_threads: int | None = None,
    verbose: int = 0,
) -> Path:
    """Run SAC training per `config`, then emit a deployment contract directory under
    `output_dir/contract`. Returns that contract directory's path."""
    import torch
    from stable_baselines3 import SAC
    from stable_baselines3.common.callbacks import BaseCallback
    from stable_baselines3.common.monitor import Monitor

    if torch_threads is not None:
        torch.set_num_threads(torch_threads)

    repo_root = Path(repo_root)
    output_dir = Path(output_dir)
    output_dir.mkdir(parents=True, exist_ok=True)
    t_start = time.time()

    base_env = ResidualRacerEnv(config, repo_root=repo_root)
    low_dim_count = len(base_env.observation_fields)

    # 2. Frozen normalisation.
    calib_obs, _ood = collect_observations(
        config, config.sac.normalization_steps, seed=config.seed + 777, repo_root=repo_root
    )
    norm = fit_normalization(calib_obs, low_dim_count, LIDAR_CLIP_M)
    del calib_obs

    train_env = Monitor(NormalizedObsEnv(base_env, norm))
    val_env = make_validation_env(config, repo_root)
    baseline_cache: dict[str, Any] = {}

    tb_log = str(log_dir) if log_dir is not None else None
    if tb_log is not None:
        try:
            import tensorboard  # noqa: F401
        except ImportError:
            tb_log = None

    best = {"score_delta": -1e9, "step": 0}
    history: list[dict[str, Any]] = []

    class RunCallback(BaseCallback):
        def __init__(self) -> None:
            super().__init__()
            self._episodes = 0
            self._crashes: list[float] = []
            self._ep_progress = 0.0
            self._env_flags = np.zeros(3)
            self._flag_steps = 0

        def _on_step(self) -> bool:
            info = self.locals["infos"][0]
            env_info = info.get("envelope", {})
            self._env_flags += [
                env_info.get("residual_clipped", False),
                env_info.get("rate_limited", False),
                env_info.get("ood_triggered", False),
            ]
            self._flag_steps += 1
            if self.locals["dones"][0]:
                self._crashes.append(float(info.get("truth", {}).get("crashed", False)))
                self._crashes = self._crashes[-100:]
                self._episodes += 1
            n = self.num_timesteps
            if n % 5000 == 0 and self._flag_steps:
                self.logger.record(
                    "custom/train_crash_rate_last100", float(np.mean(self._crashes or [0.0]))
                )
                self.logger.record(
                    "custom/envelope_clipped_fraction", self._env_flags[0] / self._flag_steps
                )
                self.logger.record(
                    "custom/envelope_rate_limited_fraction", self._env_flags[1] / self._flag_steps
                )
                self.logger.record(
                    "custom/envelope_ood_fraction", self._env_flags[2] / self._flag_steps
                )
                self._env_flags[:] = 0
                self._flag_steps = 0
            every = config.sac.eval_every_steps
            if every and n % every == 0:
                self._validate_and_checkpoint(n)
            elif config.sac.checkpoint_every_steps and n % config.sac.checkpoint_every_steps == 0:
                self.model.save(output_dir / f"checkpoint_{n}.zip")
            return True

        def _validate_and_checkpoint(self, n: int) -> None:
            self.model.save(output_dir / f"checkpoint_{n}.zip")
            result = validation_score(
                val_env, _policy_fn(self.model, norm), config.sac.eval_episodes, baseline_cache
            )
            for key, value in result.items():
                self.logger.record(f"val/{key}", value)
            row = {"step": n, "wall_s": time.time() - t_start, **result}
            history.append(row)
            if result["score_delta"] > best["score_delta"]:
                best.update(score_delta=result["score_delta"], step=n)
                self.model.save(output_dir / "best_model.zip")
            (output_dir / "progress.json").write_text(
                json.dumps({"best": best, "history": history}, indent=2)
            )

    model = SAC(
        "MlpPolicy",
        train_env,
        learning_rate=config.sac.learning_rate,
        buffer_size=config.sac.buffer_size,
        batch_size=config.sac.batch_size,
        learning_starts=config.sac.learning_starts,
        train_freq=config.sac.train_freq,
        gradient_steps=config.sac.gradient_steps,
        gamma=config.sac.gamma,
        tau=config.sac.tau,
        policy_kwargs={"net_arch": list(config.sac.net_arch)},
        seed=config.seed,
        tensorboard_log=tb_log,
        device="cpu",
        verbose=verbose,
    )
    model.learn(
        total_timesteps=config.sac.total_timesteps,
        callback=RunCallback(),
        tb_log_name=f"{config.name}_seed{config.seed}",
        progress_bar=False,
    )
    model.save(output_dir / "sac_checkpoint.zip")

    # 4. Export. Prefer the best-by-validation checkpoint when validation ran.
    chosen = model
    chosen_label = "final"
    best_path = output_dir / "best_model.zip"
    if config.sac.eval_every_steps and best_path.is_file():
        chosen = SAC.load(best_path, device="cpu")
        chosen_label = f"best_by_validation@{best['step']}"
    contract_dir = export_policy_dir(
        out_dir=output_dir / "contract",
        sac_model=chosen,
        env=base_env,
        config=config,
        config_path=config_path,
        norm=norm,
        repo_root=repo_root,
        meta_extra={
            "checkpoint": chosen_label,
            "total_timesteps": config.sac.total_timesteps,
            "training_wall_clock_s": time.time() - t_start,
            "validation_history": history,
        },
    )
    (output_dir / "normalization.json").write_text(
        json.dumps(
            {
                "mean": list(norm.mean),
                "std": list(norm.std),
                "low_dim_count": norm.low_dim_count,
                "lidar_clip_m": norm.lidar_clip_m,
            }
        )
    )
    base_env.close()
    val_env.close()
    return contract_dir


def export_from_checkpoint(
    config: ExperimentConfig,
    config_path: Path,
    checkpoint: Path,
    out_dir: Path,
    repo_root: Path = REPO_ROOT,
    meta_extra: dict[str, Any] | None = None,
) -> Path:
    """Export a saved checkpoint (e.g. a run stopped early) as a policy directory.

    The frozen normalisation is refitted with the SAME seed and steps `train()` used, which
    is deterministic, so the statistics equal the ones the network was trained with.
    """
    from stable_baselines3 import SAC

    env = ResidualRacerEnv(config, repo_root=repo_root)
    try:
        calib_obs, _ood = collect_observations(
            config, config.sac.normalization_steps, seed=config.seed + 777, repo_root=repo_root
        )
        norm = fit_normalization(calib_obs, len(env.observation_fields), LIDAR_CLIP_M)
        model = SAC.load(checkpoint, device="cpu")
        return export_policy_dir(
            out_dir=out_dir,
            sac_model=model,
            env=env,
            config=config,
            config_path=config_path,
            norm=norm,
            repo_root=repo_root,
            meta_extra={"checkpoint": str(checkpoint.name), **(meta_extra or {})},
        )
    finally:
        env.close()


def main(argv: list[str] | None = None) -> int:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument(
        "--config", type=Path, required=True, help="Path to an experiment config YAML."
    )
    parser.add_argument(
        "--output-dir",
        type=Path,
        required=True,
        help="Where to write checkpoints and contract/.",
    )
    parser.add_argument("--repo-root", type=Path, default=REPO_ROOT)
    parser.add_argument("--seed", type=int, default=None, help="Override the config's seed.")
    parser.add_argument("--log-dir", type=Path, default=None, help="TensorBoard log directory.")
    parser.add_argument("--torch-threads", type=int, default=None)
    parser.add_argument("--total-timesteps", type=int, default=None, help="Override (pilots only).")
    parser.add_argument(
        "--export-checkpoint",
        type=Path,
        default=None,
        help="Do not train: export this saved checkpoint to --output-dir/contract.",
    )
    args = parser.parse_args(argv)

    config = load_config(args.config)
    if args.seed is not None:
        config = dataclasses.replace(config, seed=args.seed)
    if args.total_timesteps is not None:
        config = dataclasses.replace(
            config, sac=dataclasses.replace(config.sac, total_timesteps=args.total_timesteps)
        )
    if args.export_checkpoint is not None:
        out = export_from_checkpoint(
            config,
            args.config,
            args.export_checkpoint,
            args.output_dir / "contract",
            repo_root=args.repo_root,
        )
        print(f"Wrote deployment contract to {out}")
        return 0
    contract_dir = train(
        config,
        args.config,
        args.output_dir,
        repo_root=args.repo_root,
        log_dir=args.log_dir,
        torch_threads=args.torch_threads,
    )
    print(f"Wrote deployment contract to {contract_dir}")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
