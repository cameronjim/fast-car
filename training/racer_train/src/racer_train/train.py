"""SAC training entry point (roadmap S.3): config in, checkpoint + a loadable deployment
contract directory out.

Boring-choices justification (claude-docs/08-learning.md "Algorithm: SAC (fallback PPO). No
algorithm shopping without a logged reason", CLAUDE.md): stable-baselines3 is used as a
maintained dependency rather than a hand-rolled SAC implementation -- it is the standard,
well-tested reference implementation for this algorithm, and hand-rolling actor-critic +
replay buffer + target-network-update machinery would only add a second, unrelated thing to
debug on top of this task's actual scope (the residual/envelope/contract wiring).

torch and stable_baselines3 are imported LAZILY inside `train()`, not at module level (same
pattern as ros_ws/src/racer_policy/src/racer_policy/model.py's `load_model`), so importing
this module -- e.g. `python -m racer_train.train --help` -- never requires them installed.
Running an actual training job does require them (`uv sync --group train`, see this
package's pyproject.toml).
"""

from __future__ import annotations

import argparse
import subprocess
from pathlib import Path

import gymnasium as gym
import numpy as np

from racer_train.config import ExperimentConfig, config_sha256, load_config
from racer_train.contract_export import build_contract_manifest, write_contract_dir
from racer_train.env import REPO_ROOT, ResidualRacerEnv

POLICY_FILENAME = "policy.pt"


def _git_sha(repo_root: Path) -> str:
    result = subprocess.run(
        ["git", "rev-parse", "HEAD"],
        cwd=repo_root,
        capture_output=True,
        text=True,
        check=True,
    )
    return result.stdout.strip()


class _ObservationStatsWrapper(gym.ObservationWrapper):
    """Tracks running mean/variance of every observation seen, WITHOUT altering the
    observation returned to the agent (this env intentionally trains on raw observations --
    normalization is recorded for the contract per claude-docs/08-learning.md's
    "Normalization statistics from training time", applied by whatever consumes the contract
    at deploy time, roadmap 5.2). Welford's online algorithm; no torch dependency."""

    def __init__(self, env: gym.Env) -> None:
        super().__init__(env)
        dim = env.observation_space.shape[0]
        self.count = 1e-4
        self.mean = np.zeros(dim, dtype=np.float64)
        self._m2 = np.zeros(dim, dtype=np.float64)

    def observation(self, observation: np.ndarray) -> np.ndarray:
        x = np.asarray(observation, dtype=np.float64)
        self.count += 1
        delta = x - self.mean
        self.mean += delta / self.count
        delta2 = x - self.mean
        self._m2 += delta * delta2
        return observation

    @property
    def std(self) -> np.ndarray:
        variance = self._m2 / max(self.count - 1.0, 1e-4)
        return np.sqrt(np.maximum(variance, 1e-8))


def train(
    config: ExperimentConfig, config_path: Path, output_dir: Path, repo_root: Path = REPO_ROOT
) -> Path:
    """Run SAC training per `config`, then emit a deployment contract directory under
    `output_dir/contract`. Returns that contract directory's path."""
    import torch
    from stable_baselines3 import SAC

    repo_root = Path(repo_root)
    output_dir = Path(output_dir)

    base_env = ResidualRacerEnv(config, repo_root=repo_root)
    stats_env = _ObservationStatsWrapper(base_env)

    model = SAC(
        "MlpPolicy",
        stats_env,
        learning_rate=config.sac.learning_rate,
        buffer_size=config.sac.buffer_size,
        batch_size=config.sac.batch_size,
        learning_starts=config.sac.learning_starts,
        train_freq=config.sac.train_freq,
        gradient_steps=config.sac.gradient_steps,
        policy_kwargs={"net_arch": list(config.sac.net_arch)},
        seed=config.seed,
        verbose=0,
    )
    model.learn(total_timesteps=config.sac.total_timesteps)

    output_dir.mkdir(parents=True, exist_ok=True)
    model.save(output_dir / "sac_checkpoint.zip")

    contract_dir = output_dir / "contract"
    contract_dir.mkdir(parents=True, exist_ok=True)

    # Export the actor's DETERMINISTIC policy to TorchScript -- this env's action_space is
    # already Box(-1, 1) per channel (see env.py), matching SAC's internal squashed-action
    # convention exactly, so no extra scale/unscale wrapping is needed here: the traced
    # module's output IS the raw residual action racer_train.env.ResidualRacerEnv.step
    # expects.
    class _DeterministicActor(torch.nn.Module):
        def __init__(self, actor: torch.nn.Module) -> None:
            super().__init__()
            self.actor = actor

        def forward(self, obs: torch.Tensor) -> torch.Tensor:
            return self.actor(obs, deterministic=True)

    actor_module = _DeterministicActor(model.policy.actor).eval()
    example_obs = torch.zeros((1, base_env.observation_space.shape[0]), dtype=torch.float32)
    with torch.no_grad():
        traced = torch.jit.trace(actor_module, example_obs)
    policy_path = contract_dir / POLICY_FILENAME
    traced.save(str(policy_path))

    import hashlib

    policy_checksum = hashlib.sha256(policy_path.read_bytes()).hexdigest()

    manifest = build_contract_manifest(
        env=base_env,
        policy_filename=POLICY_FILENAME,
        policy_checksum_sha256=policy_checksum,
        normalization_mean=list(stats_env.mean),
        normalization_std=list(stats_env.std),
        config_hash=config_sha256(config_path),
        git_sha=_git_sha(repo_root),
    )
    write_contract_dir(contract_dir, manifest)

    base_env.close()
    return contract_dir


def main(argv: list[str] | None = None) -> int:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument(
        "--config", type=Path, required=True, help="Path to an experiment config YAML."
    )
    parser.add_argument(
        "--output-dir",
        type=Path,
        required=True,
        help="Where to write the checkpoint and contract/.",
    )
    parser.add_argument("--repo-root", type=Path, default=REPO_ROOT)
    args = parser.parse_args(argv)

    config = load_config(args.config)
    contract_dir = train(config, args.config, args.output_dir, repo_root=args.repo_root)
    print(f"Wrote deployment contract to {contract_dir}")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
