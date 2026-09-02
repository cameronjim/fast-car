# trains an sb3 agent against the f1tenth gym

from __future__ import annotations

import argparse
import json
from pathlib import Path

import torch
from stable_baselines3 import PPO, SAC
from stable_baselines3.common.callbacks import CheckpointCallback

from .envs import (
    RacingEvalCallback,
    SpeedCapCurriculum,
    SpeedCapSchedule,
    build_eval_env,
    build_vec_env,
    load_config,
)

ALGOS = {"sac": SAC, "ppo": PPO}


def _policy_kwargs(algo_cfg: dict) -> dict:
    kwargs = dict(algo_cfg.get("policy_kwargs") or {})
    net_arch = algo_cfg.get("net_arch")
    if net_arch is not None:
        kwargs["net_arch"] = list(net_arch)
    activation = kwargs.pop("activation_fn", None)
    if activation is not None:
        kwargs["activation_fn"] = getattr(torch.nn, activation)
    return kwargs


def _model_kwargs(algo_cfg: dict) -> dict:
    skip = {"name", "policy", "net_arch", "policy_kwargs"}
    kwargs = {k: v for k, v in algo_cfg.items() if k not in skip}
    if "train_freq" in kwargs and isinstance(kwargs["train_freq"], list):
        kwargs["train_freq"] = tuple(kwargs["train_freq"])
    return kwargs


def build_model(cfg: dict, vec_env, tensorboard_log: Path):
    algo_cfg = cfg["algo"]
    name = str(algo_cfg.get("name", "sac")).lower()
    if name not in ALGOS:
        raise ValueError(f"unknown algo {name!r}, expected one of {sorted(ALGOS)}")
    model = ALGOS[name](
        algo_cfg.get("policy", "MlpPolicy"),
        vec_env,
        seed=int(cfg["run"].get("seed", 0)),
        verbose=1,
        tensorboard_log=str(tensorboard_log),
        policy_kwargs=_policy_kwargs(algo_cfg),
        **_model_kwargs(algo_cfg),
    )
    resume_from = cfg["run"].get("resume_from")
    if resume_from:
        warm_start(model, ALGOS[name], resume_from)
    return model


def warm_start(model, algo, resume_from) -> None:
    """copy a saved policy into a freshly built model, leaving the replay buffer empty."""
    path = Path(resume_from)
    if not path.is_file() and not path.with_suffix(".zip").is_file():
        raise FileNotFoundError(f"resume_from checkpoint not found: {path.resolve()}")
    donor = algo.load(str(path), device="cpu")
    if donor.observation_space.shape != model.observation_space.shape:
        raise ValueError(
            f"resume_from obs shape {donor.observation_space.shape} does not match the "
            f"config's {model.observation_space.shape}"
        )
    model.policy.load_state_dict(donor.policy.state_dict())
    model.policy.to(model.device)
    # auto entropy would restart at its initial value and immediately undo the warm start
    if hasattr(model, "log_ent_coef") and getattr(donor, "log_ent_coef", None) is not None:
        with torch.no_grad():
            model.log_ent_coef.copy_(donor.log_ent_coef.to(model.device))
        print(f"carried the learned entropy coefficient {float(model.log_ent_coef.exp()):.4f}")
    print(f"warm started from {path}, replay buffer left empty")


def build_callbacks(cfg: dict, eval_env, log_dir: Path, run_name: str, n_envs: int) -> list:
    """eval, curriculum, and checkpoint callbacks, with run-level step counts per worker."""
    run_cfg = cfg["run"]
    curriculum_cfg = cfg.get("curriculum")
    curriculum = None
    if curriculum_cfg is not None:
        curriculum = SpeedCapCurriculum(SpeedCapSchedule.from_config(curriculum_cfg))
    eval_callback = RacingEvalCallback(
        eval_env,
        callback_after_eval=curriculum,
        racing_best_path=str(log_dir / "best" / "best_racing_model"),
        speed_cap_mps=float(cfg["env"].get("speed_cap_mps", 3.0)),
        best_model_save_path=str(log_dir / "best"),
        log_path=str(log_dir / "eval"),
        eval_freq=max(1, int(run_cfg.get("eval_freq", 25000)) // n_envs),
        n_eval_episodes=int(run_cfg.get("eval_episodes", 5)),
        deterministic=True,
    )
    checkpoint_callback = CheckpointCallback(
        save_freq=max(1, int(run_cfg.get("checkpoint_freq", 50000)) // n_envs),
        save_path=str(log_dir / "checkpoints"),
        name_prefix=run_name,
    )
    return [eval_callback, checkpoint_callback]


def main() -> None:
    parser = argparse.ArgumentParser(description="train an rl racing policy")
    parser.add_argument("--config", required=True)
    parser.add_argument("--total-steps", type=int, default=None)
    parser.add_argument("--n-envs", type=int, default=None)
    parser.add_argument("--log-dir", default=None)
    parser.add_argument("--run-name", default=None)
    parser.add_argument("--resume-from", default=None)
    args = parser.parse_args()

    cfg = load_config(args.config)
    run_cfg = cfg["run"]
    if args.total_steps is not None:
        run_cfg["total_steps"] = args.total_steps
    if args.n_envs is not None:
        run_cfg["n_envs"] = args.n_envs
    if args.log_dir is not None:
        run_cfg["log_dir"] = args.log_dir
    if args.resume_from is not None:
        run_cfg["resume_from"] = args.resume_from

    run_name = args.run_name or Path(args.config).stem
    log_dir = Path(run_cfg.get("log_dir", "runs")) / run_name
    log_dir.mkdir(parents=True, exist_ok=True)
    (log_dir / "config.json").write_text(json.dumps(cfg, indent=2, default=str) + "\n")

    n_envs = int(run_cfg.get("n_envs", 8))
    vec_env = build_vec_env(cfg, n_envs=n_envs)
    eval_env = build_eval_env(cfg)

    # callback frequencies are per worker, so divide the run-level step counts
    callbacks = build_callbacks(cfg, eval_env, log_dir, run_name, n_envs)

    model = build_model(cfg, vec_env, log_dir / "tb")
    try:
        model.learn(
            total_timesteps=int(run_cfg.get("total_steps", 1_000_000)),
            callback=callbacks,
            progress_bar=False,
        )
    finally:
        model.save(str(log_dir / "final_model"))
        vec_env.close()
        eval_env.close()
    print(f"training done, artifacts under {log_dir}")


if __name__ == "__main__":
    main()
