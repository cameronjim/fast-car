# runs learned_control's hand-rolled sac against the same gym env sb3 trained on

from __future__ import annotations

import argparse
import csv
import json
import sys
import time
from pathlib import Path

import numpy as np
import torch

from .envs import build_env, load_config
from .envs.curriculum import EvalRound, racing_score
from .evaluate import run_episodes

REPO_ROOT = Path(__file__).resolve().parents[2]

ACTION_DIM = 2

# sac_train_node's online cadence, copied rather than tuned: the point is the student trainer
UPDATE_EVERY_STEPS = 10
BATCH_SIZE = 256
LEARNING_STARTS_STEPS = 10_000
# the node declared lr_actor 1e-4, but sac_m2.yaml gives sb3 3e-4 everywhere, so this
# matches sb3 and leaves the implementation as the only difference worth reading
LEARNING_RATE = 3e-4


def import_handrolled():
    """learned_control's sac trainer and nets, imported by path out of the ros package."""
    if str(REPO_ROOT) not in sys.path:
        sys.path.insert(0, str(REPO_ROOT))
    try:
        from learned_control.sac.model import SACActorNet, SACCriticNet
        from learned_control.sac.train import SACTrainer
    except ImportError as failure:
        raise ImportError(
            f"could not import learned_control.sac from {REPO_ROOT}: {failure}"
        ) from failure
    return SACTrainer, SACActorNet, SACCriticNet


def unit_action(handrolled) -> np.ndarray:
    """the actor's [0, 1] output onto the wrapper's [-1, 1] action box."""
    action = np.asarray(handrolled, dtype=np.float32).reshape(ACTION_DIM)
    return np.clip(2.0 * action - 1.0, -1.0, 1.0).astype(np.float32)


def handrolled_action(unit) -> np.ndarray:
    """a [-1, 1] unit action back into the [0, 1] range the replay buffer and critics use."""
    action = np.clip(np.asarray(unit, dtype=np.float32).reshape(ACTION_DIM), -1.0, 1.0)
    return (0.5 * (action + 1.0)).astype(np.float32)


class HandrolledPolicy:
    """gives the hand-rolled actor the predict() signature f1rl.evaluate drives."""

    def __init__(self, actor, device):
        self.actor = actor
        self.device = device

    def predict(self, obs, deterministic: bool = True):
        state = torch.as_tensor(np.asarray(obs, dtype=np.float32), device=self.device).reshape(1, -1)
        raw = self.actor.get_action(state, deterministic).cpu().numpy()[0]
        return unit_action(raw), None


def build_trainer(cfg: dict, state_dim: int, device: str, buffer_size: int):
    """the unmodified SACTrainer wired to this env's observation width."""
    SACTrainer, SACActorNet, SACCriticNet = import_handrolled()
    algo_cfg = cfg["algo"]
    return SACTrainer(
        SACActorNet(state_dim, ACTION_DIM),
        SACCriticNet(state_dim, ACTION_DIM),
        SACCriticNet(state_dim, ACTION_DIM),
        state_dim=state_dim,
        action_dim=ACTION_DIM,
        lr_actor=LEARNING_RATE,
        lr_critic=LEARNING_RATE,
        lr_alpha=LEARNING_RATE,
        gamma=float(algo_cfg.get("gamma", 0.99)),
        tau=float(algo_cfg.get("tau", 0.005)),
        buffer_size=buffer_size,
        batch_size=BATCH_SIZE,
        device=device,
    )


def eval_round(results: list[dict], speed_cap_mps: float) -> EvalRound:
    """one evaluation block scored the same way RacingEvalCallback scores sb3's."""
    lap_times = [t for r in results for t in r["lap_times_sec"]]
    return EvalRound(
        speed_cap_mps=speed_cap_mps,
        episodes=len(results),
        collision_free_rate=float(np.mean([not r["collided"] for r in results])),
        laps=int(sum(r["laps"] for r in results)),
        lap_times_sec=lap_times,
    )


def gate_lines(results: list[dict]) -> list[str]:
    """the per-episode gate report, in the same shape as the m2 gate log."""
    lines = []
    for r in results:
        laps = " ".join(f"{t:.2f}s" for t in r["lap_times_sec"]) or "no lap"
        lines.append(
            f"episode {r['episode']:>3}  progress {r['progress_m']:8.1f}m  "
            f"steps {r['steps']:>5}  laps {r['laps']}  {laps}  "
            f"{'collision' if r['collided'] else 'clean'}"
        )
    progress = np.array([r["progress_m"] for r in results])
    lap_times = [t for r in results for t in r["lap_times_sec"]]
    collisions = np.array([r["collided"] for r in results])
    lines.append(
        f"progress mean {progress.mean():.1f}m std {progress.std():.1f}m, "
        f"collision rate {collisions.mean():.2%}, "
        f"laps completed {len(lap_times)}/{len(results)}"
    )
    if lap_times:
        lines.append(f"lap time best {min(lap_times):.2f}s mean {np.mean(lap_times):.2f}s")
    else:
        lines.append("lap time none: no episode completed a lap")
    return lines


def evaluate_policy(trainer, env, episodes: int, seed: int) -> list[dict]:
    return run_episodes(HandrolledPolicy(trainer.actor, trainer.device), env, episodes, seed)


def save_checkpoint(trainer, path: Path, steps: int, best_score: float) -> None:
    """trainer state plus the step counter; the replay buffer is not part of it."""
    trainer.save(str(path))
    path.with_suffix(".json").write_text(
        json.dumps({"steps": steps, "best_score": best_score}, indent=2) + "\n"
    )


def load_checkpoint(trainer, path: Path) -> tuple[int, float]:
    trainer.load(str(path))
    sidecar = path.with_suffix(".json")
    if not sidecar.is_file():
        raise FileNotFoundError(f"checkpoint {path} has no step sidecar at {sidecar}")
    saved = json.loads(sidecar.read_text())
    return int(saved["steps"]), float(saved["best_score"])


def train(args) -> None:
    cfg = load_config(args.config)
    out_dir = Path(args.out_dir)
    out_dir.mkdir(parents=True, exist_ok=True)

    # one env, not eight: the hand-rolled trainer has no vec support at all
    env = build_env(cfg, seed_offset=0)
    eval_env = build_env(cfg, seed_offset=1000)
    speed_cap_mps = float(env.action_bounds.speed_cap_mps)
    state_dim = int(env.observation_space.shape[0])

    device = "cuda" if torch.cuda.is_available() else "cpu"
    trainer = build_trainer(cfg, state_dim, device, args.buffer_size)

    checkpoint = out_dir / "checkpoint.pth"
    best_path = out_dir / "best.pth"
    curve_path = out_dir / "eval_curve.csv"

    step = 0
    best_score = -np.inf
    if args.resume and checkpoint.is_file():
        step, best_score = load_checkpoint(trainer, checkpoint)
        print(f"resumed from {checkpoint} at step {step}, replay buffer starts empty")
    if not curve_path.is_file():
        with curve_path.open("w", newline="") as handle:
            csv.writer(handle).writerow(
                ["step", "clean_rate", "laps", "best_lap_sec", "mean_lap_sec", "alpha", "critic1_loss"]
            )

    print(
        f"hand-rolled sac: {state_dim} obs dims, {args.total_steps} steps on {device}, "
        f"1 env, update every {UPDATE_EVERY_STEPS} steps, batch {BATCH_SIZE}, "
        f"{speed_cap_mps:.1f} m/s cap"
    )

    obs, _ = env.reset(seed=int(cfg["run"].get("seed", 0)))
    episode_reward = 0.0
    episode_steps = 0
    episodes = 0
    metrics: dict | None = None
    started_sec = time.time()

    while step < args.total_steps:
        if step < LEARNING_STARTS_STEPS:
            action = np.random.uniform(0.0, 1.0, size=ACTION_DIM).astype(np.float32)
        else:
            state = torch.as_tensor(obs, device=trainer.device).reshape(1, -1)
            action = trainer.actor.get_action(state, deterministic=False).cpu().numpy()[0]

        unit = unit_action(action)
        next_obs, reward, terminated, truncated, info = env.step(unit)
        # store the clipped action the wrapper actually drove, in the actor's own [0, 1] range
        trainer.store(obs, handrolled_action(unit), reward, next_obs, terminated)
        episode_reward += reward
        episode_steps += 1
        step += 1
        obs = next_obs

        if step >= LEARNING_STARTS_STEPS and step % UPDATE_EVERY_STEPS == 0:
            metrics = trainer.update() or metrics

        if terminated or truncated:
            episodes += 1
            elapsed_min = (time.time() - started_sec) / 60.0
            print(
                f"episode {episodes} at step {step}: reward {episode_reward:.1f} over "
                f"{episode_steps} steps, laps {info['lap_count']}, "
                f"{'collision' if info['is_collision'] else 'clean'}, "
                f"alpha {trainer.alpha:.4f}, {elapsed_min:.1f} min",
                flush=True,
            )
            obs, _ = env.reset()
            episode_reward = 0.0
            episode_steps = 0

        if step % args.eval_every == 0:
            results = evaluate_policy(trainer, eval_env, args.eval_episodes, args.seed)
            finished = eval_round(results, speed_cap_mps)
            score = racing_score(finished)
            best_lap = finished.best_lap_time_sec
            mean_lap = finished.mean_lap_time_sec
            print(
                f"eval at {step} steps: clean {finished.collision_free_rate:.0%} over "
                f"{finished.episodes} episodes, {finished.laps} laps, "
                f"{f'best lap {best_lap:.2f}s' if best_lap else 'no lap'}, score {score:.2f}",
                flush=True,
            )
            with curve_path.open("a", newline="") as handle:
                csv.writer(handle).writerow(
                    [
                        step,
                        round(finished.collision_free_rate, 4),
                        finished.laps,
                        None if best_lap is None else round(best_lap, 3),
                        None if mean_lap is None else round(mean_lap, 3),
                        round(trainer.alpha, 5),
                        None if metrics is None else metrics["critic1_loss"],
                    ]
                )
            if score > best_score:
                best_score = score
                save_checkpoint(trainer, best_path, step, best_score)
                print(f"new best hand-rolled policy at score {score:.2f}", flush=True)
            save_checkpoint(trainer, checkpoint, step, best_score)

    save_checkpoint(trainer, checkpoint, step, best_score)
    print(f"training done in {(time.time() - started_sec) / 60.0:.1f} min over {episodes} episodes")
    env.close()
    eval_env.close()


def gate(args) -> None:
    cfg = load_config(args.config)
    out_dir = Path(args.out_dir)
    checkpoint = Path(args.checkpoint) if args.checkpoint else out_dir / "best.pth"
    if not checkpoint.is_file():
        raise FileNotFoundError(f"no checkpoint to gate at {checkpoint.resolve()}")

    env = build_env(cfg, seed_offset=1000)
    device = "cuda" if torch.cuda.is_available() else "cpu"
    # gating never stores a transition, so the buffer only has to exist
    trainer = build_trainer(cfg, int(env.observation_space.shape[0]), device, 1)
    steps, _ = load_checkpoint(trainer, checkpoint)

    header = (
        f"hand-rolled sac from {checkpoint.name} at {steps} training steps, "
        f"{args.gate_episodes} deterministic episodes at a "
        f"{env.action_bounds.speed_cap_mps:.2f} m/s cap"
    )
    results = evaluate_policy(trainer, env, args.gate_episodes, args.seed)
    env.close()

    lines = [header] + gate_lines(results)
    report = "\n".join(lines) + "\n"
    print(report, end="")
    (out_dir / f"gate_eval_{args.gate_episodes}ep.log").write_text(report)


def main() -> None:
    parser = argparse.ArgumentParser(
        description="train or gate the hand-rolled sac on the sb3 training env"
    )
    parser.add_argument("--config", default="configs/sac_m2.yaml")
    parser.add_argument("--out-dir", default="artifacts/handrolled")
    parser.add_argument("--total-steps", type=int, default=600_000)
    parser.add_argument("--buffer-size", type=int, default=600_000)
    parser.add_argument("--eval-every", type=int, default=50_000)
    parser.add_argument("--eval-episodes", type=int, default=5)
    parser.add_argument("--gate-episodes", type=int, default=20)
    parser.add_argument("--seed", type=int, default=0)
    parser.add_argument("--resume", action="store_true")
    parser.add_argument("--gate-only", action="store_true")
    parser.add_argument("--checkpoint", default=None, help="gate this file instead of best.pth")
    args = parser.parse_args()

    if args.gate_only:
        gate(args)
        return
    train(args)
    gate(args)


if __name__ == "__main__":
    main()
