# exports a trained policy to torchscript plus the obs_config.json deploy contract

from __future__ import annotations

import argparse
import sys
from pathlib import Path

import numpy as np
import torch
from stable_baselines3 import PPO, SAC

from .envs import build_env, load_config, write_deploy_contract

ALGOS = {"sac": SAC, "ppo": PPO}

ROUND_TRIP_TOL = 1e-5


class DeterministicPolicy(torch.nn.Module):
    """obs vector in, action in [-1, 1] out, with no sb3 machinery attached."""

    def __init__(self, policy: torch.nn.Module):
        super().__init__()
        self.policy = policy

    def forward(self, obs: torch.Tensor) -> torch.Tensor:
        return torch.clamp(self.policy._predict(obs, deterministic=True), -1.0, 1.0)


def export(model, obs_sample: np.ndarray, out_path: Path) -> torch.jit.ScriptModule:
    """trace the deterministic actor and write it to torchscript."""
    module = DeterministicPolicy(model.policy).eval()
    example = torch.as_tensor(obs_sample, dtype=torch.float32).reshape(1, -1)
    with torch.no_grad():
        scripted = torch.jit.trace(module, example)
    scripted.save(str(out_path))
    return scripted


def round_trip_error(model, scripted, obs_batch: np.ndarray) -> float:
    """largest action mismatch between sb3 predict and the exported module."""
    sb3_actions = np.stack([model.predict(obs, deterministic=True)[0] for obs in obs_batch])
    with torch.no_grad():
        exported = scripted(torch.as_tensor(obs_batch, dtype=torch.float32)).numpy()
    return float(np.abs(sb3_actions - exported).max())


def main() -> None:
    parser = argparse.ArgumentParser(description="export a trained policy for deployment")
    parser.add_argument("--model", required=True)
    parser.add_argument("--config", required=True)
    parser.add_argument("--out-dir", default=None)
    parser.add_argument("--samples", type=int, default=32)
    parser.add_argument(
        "--speed-cap",
        type=float,
        default=None,
        help="cap the exported contract carries, when a curriculum moved it off the config",
    )
    args = parser.parse_args()

    cfg = load_config(args.config)
    if args.speed_cap is not None:
        cfg["env"]["speed_cap_mps"] = args.speed_cap
    algo = str(cfg["algo"].get("name", "sac")).lower()
    if algo not in ALGOS:
        raise ValueError(f"unknown algo {algo!r}, expected one of {sorted(ALGOS)}")
    model_path = Path(args.model)
    if not model_path.is_file() and not model_path.with_suffix(".zip").is_file():
        raise FileNotFoundError(f"model not found: {model_path.resolve()}")
    model = ALGOS[algo].load(str(model_path), device="cpu")

    out_dir = Path(args.out_dir) if args.out_dir else model_path.parent / "export"
    out_dir.mkdir(parents=True, exist_ok=True)

    env = build_env(cfg)
    obs, _ = env.reset()
    obs_batch = [obs]
    for _ in range(args.samples - 1):
        action, _ = model.predict(obs_batch[-1], deterministic=True)
        obs, _, terminated, truncated, _ = env.step(action)
        if terminated or truncated:
            obs, _ = env.reset()
        obs_batch.append(obs)
    obs_batch = np.stack(obs_batch).astype(np.float32)

    policy_path = out_dir / "policy.pt"
    scripted = export(model, obs_batch[0], policy_path)
    error = round_trip_error(model, scripted, obs_batch)
    contract_path = write_deploy_contract(
        out_dir / "obs_config.json", env.obs_cfg, env.action_bounds
    )
    undeployable = env.obs_cfg.undeployable_features()
    env.close()

    print(f"wrote {policy_path} and {contract_path}")
    print(f"round trip max action error {error:.2e} over {len(obs_batch)} observations")
    if undeployable:
        print(
            f"warning: features {list(undeployable)} cannot be built on the ros car, so this "
            f"export is sim only; retrain with a deploy feature set for the demo"
        )
    if error > ROUND_TRIP_TOL:
        print(f"round trip error exceeds {ROUND_TRIP_TOL:g}", file=sys.stderr)
        sys.exit(1)


if __name__ == "__main__":
    main()
