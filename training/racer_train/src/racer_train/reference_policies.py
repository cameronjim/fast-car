"""Hand-written reference policies, evaluated exactly like a learned one.

A learned residual's lap-time change means little without knowing what a policy that has learned
nothing can do with the same authority. `constant` here is a policy that outputs one fixed
raw action every step (for example [0, +1]: always ask for the full speed residual, never touch
the steering), and `evaluate` runs it through the SAME paired held-out conditions, envelope and
seeds as `racer_train.heldout_report`, so its row sits next to the learned policy's.

It answers one question: how much headroom does the residual bound leave on this base
controller? If "always +0.1 m/s" already costs completions, the speed channel has no free
lunch; if it is faster at no cost, a learned policy that is not at least that fast has not
found the structure it was supposed to.

    uv run python -m racer_train.reference_policies --config <yaml> --action 0 1 \
        --condition heldout_family --episodes 120 --out <json>
"""

from __future__ import annotations

import argparse
import json
from pathlib import Path

import numpy as np

from racer_train.config import load_config
from racer_train.env import REPO_ROOT, ResidualRacerEnv
from racer_train.evaluate import PolicyFn, evaluate_paired, summarize_pair
from racer_train.heldout_report import HELDOUT_EPISODE_SEED_BASE, conditions


def constant(action: tuple[float, float]) -> PolicyFn:
    """A policy that ignores the observation and returns `action` (raw, in [-1, 1])."""
    fixed = np.asarray(action, dtype=np.float32)
    if fixed.shape != (2,):
        raise ValueError("action must be two numbers")

    def policy(_obs: np.ndarray) -> np.ndarray:
        return fixed

    return policy


def evaluate(config_path: Path, action: tuple[float, float], condition: str, episodes: int):
    cfg = conditions(load_config(config_path))[condition]
    env = ResidualRacerEnv(cfg, repo_root=REPO_ROOT)
    try:
        base, learned = evaluate_paired(
            env, constant(action), episodes, HELDOUT_EPISODE_SEED_BASE, laps=2.0
        )
    finally:
        env.close()
    return summarize_pair(base, learned, seed=1)


def main(argv: list[str] | None = None) -> int:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--config", type=Path, required=True)
    parser.add_argument("--action", type=float, nargs=2, required=True)
    parser.add_argument("--condition", default="heldout_family")
    parser.add_argument("--episodes", type=int, default=120)
    parser.add_argument("--out", type=Path, required=True)
    args = parser.parse_args(argv)
    summary = evaluate(args.config, (args.action[0], args.action[1]), args.condition, args.episodes)
    args.out.write_text(json.dumps(summary, indent=2, default=float))
    print(json.dumps(summary["paired"], default=float))
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
