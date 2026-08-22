"""The L5 nightly training smoke test (claude-docs/12-testing.md: "Nightly long run:
training smoke test -- a tiny SAC run (minutes) completes, loss is finite, checkpoint loads
through the deployment contract. Catches pipeline rot early.").

Marked `slow` (see pyproject.toml's `markers`) and skipped via `pytest.importorskip` when
torch/stable_baselines3 are not installed -- the normal state in the per-push L1 job, since
this package's pyproject.toml deliberately keeps them out of the "dev" dependency group (see
that file's comment). `.github/workflows/nightly.yml` runs `uv sync --group train --dev`
before this test, so it actually executes there.
"""

from __future__ import annotations

import math
import tempfile
from pathlib import Path

import pytest
from conftest import REPO_ROOT, SMOKE_CONFIG_PATH
from racer_train.config import load_config

torch = pytest.importorskip("torch")
pytest.importorskip("stable_baselines3")


@pytest.mark.slow
def test_tiny_sac_run_completes_and_emits_a_loadable_contract():
    from racer_policy import load_contract, load_model, verify_against_environment
    from racer_policy.contract import ObservationField
    from racer_policy.environment import LiveEnvironment, LiveObservationSchema
    from racer_train.env import ResidualRacerEnv
    from racer_train.train import train

    config = load_config(SMOKE_CONFIG_PATH)

    with tempfile.TemporaryDirectory() as tmp:
        output_dir = Path(tmp) / "out"
        contract_dir = train(config, SMOKE_CONFIG_PATH, output_dir, repo_root=REPO_ROOT)

        # 1. Checkpoint was saved.
        assert (output_dir / "sac_checkpoint.zip").is_file()

        # 2. The contract loads through the REAL S.5 loader (racer_policy.load_contract),
        #    static checks only (manifest shape, contract_version, policy checksum).
        contract = load_contract(contract_dir)

        # 3. It also verifies against a live environment built from the same config (the
        #    dynamic half of the loader).
        env = ResidualRacerEnv(config, repo_root=REPO_ROOT)
        try:
            live = LiveEnvironment(
                vehicle_params=env.vehicle_params,
                observation=LiveObservationSchema(
                    fields=tuple(
                        ObservationField(name=n, dtype=d, units=u)
                        for n, d, u in env.observation_fields
                    ),
                    lidar_beam_count=env.lidar_config.beam_count,
                    lidar_fov_rad=env.lidar_config.fov_rad,
                    lidar_downsample_factor=env.lidar_config.downsample_factor,
                ),
            )
            verify_against_environment(contract, live)

            # 4. The exported TorchScript policy actually loads and runs (load_model is the
            #    one part of the S.5 loader that DOES need torch -- available here since this
            #    whole test is torch-gated).
            model = load_model(contract)
            example_obs = torch.zeros((1, env.observation_space.shape[0]), dtype=torch.float32)
            with torch.no_grad():
                output = model(example_obs)
            output_np = output.cpu().numpy()
            assert output_np.shape == (1, 2)
            assert bool(((-1.0 - 1e-6) <= output_np).all() and (output_np <= (1.0 + 1e-6)).all())
        finally:
            env.close()


@pytest.mark.slow
def test_tiny_sac_run_produces_finite_loss_and_return():
    """A more direct check of claude-docs/12-testing.md's "loss is finite" requirement: run
    the same tiny config's SAC training with a logger capturing the training losses, and
    assert every recorded scalar is finite."""
    from racer_train.env import ResidualRacerEnv
    from stable_baselines3 import SAC
    from stable_baselines3.common.logger import KVWriter, Logger

    config = load_config(SMOKE_CONFIG_PATH)
    env = ResidualRacerEnv(config, repo_root=REPO_ROOT)

    recorded: dict[str, float] = {}

    class _CapturingWriter(KVWriter):
        def write(self, key_values, key_excluded, step=0):
            for key, value in key_values.items():
                if isinstance(value, (int, float)):
                    recorded[key] = float(value)

        def close(self):
            pass

    try:
        model = SAC(
            "MlpPolicy",
            env,
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
        model.set_logger(Logger(folder=None, output_formats=[_CapturingWriter()]))
        model.learn(total_timesteps=config.sac.total_timesteps, log_interval=1)
    finally:
        env.close()

    loss_like = {k: v for k, v in recorded.items() if "loss" in k or "return" in k}
    assert loss_like, "expected at least one loss/return scalar to have been logged"
    for key, value in loss_like.items():
        assert math.isfinite(value), f"{key} is not finite: {value}"
