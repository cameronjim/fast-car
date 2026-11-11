"""Held-out paired evaluation of a policy DIRECTORY, driven through the deployment contract.

The policy under test is loaded the way the deploy node will load it: `racer_policy.
load_contract` (checksum, schema), `verify_against_environment` against a live-environment
snapshot built from the eval env, then `load_model` (TorchScript). Observations go in RAW,
exactly as the contract's schema declares them; the module normalises internally. Nothing from
the training process is reused, so a pass here is also a check that the exported artifact, not
the in-memory SB3 model, is what was evaluated.

Conditions (each paired: tracker alone vs tracker + residual, identical worlds):

  heldout_family   unseen draws of the training shape families (stadium, rounded rectangle)
  heldout_shapes   shapes the policy never saw (smooth random closed curves, racing-room
                   ovals), both directions
  stress_x1.5      held-out family with every randomisation range widened 1.5x about its centre
  nominal          held-out family, no randomisation (nominal real-car numbers, no noise)

This is a SIM evaluation of a pre-sysid policy. It is not the pre-registered hardware
evaluation (09-evaluation.md) and makes no claim about the real car.

    uv run python -m racer_train.heldout_report --policy-dir <dir> --config <yaml> --out-dir <dir>
"""

from __future__ import annotations

import argparse
import dataclasses
import json
import time
from pathlib import Path
from typing import Any

import numpy as np

from racer_train.config import ExperimentConfig, load_config
from racer_train.env import REPO_ROOT, ResidualRacerEnv
from racer_train.evaluate import evaluate_paired, summarize_pair
from racer_train.randomization import RandomizationConfig

HELDOUT_FAMILY_TRACK_SEED_BASE = 10000
HELDOUT_SHAPES_TRACK_SEED_BASE = 20000
HELDOUT_EPISODE_SEED_BASE = 90000


def load_contract_policy(policy_dir: Path, env: ResidualRacerEnv):
    """(callable obs -> action, contract). Raises a racer_policy ContractError on any refusal."""
    import torch
    from racer_policy import load_contract, load_model, verify_against_environment
    from racer_policy.contract import ObservationField
    from racer_policy.environment import LiveEnvironment, LiveObservationSchema

    contract = load_contract(Path(policy_dir))
    live = LiveEnvironment(
        vehicle_params=env.committed_vehicle_params,
        observation=LiveObservationSchema(
            fields=tuple(
                ObservationField(name=n, dtype=d, units=u) for n, d, u in env.observation_fields
            ),
            lidar_beam_count=env.lidar_config.beam_count,
            lidar_fov_rad=env.lidar_config.fov_rad,
            lidar_downsample_factor=env.lidar_config.downsample_factor,
        ),
    )
    verify_against_environment(contract, live)
    model = load_model(contract)

    def policy(obs: np.ndarray) -> np.ndarray:
        with torch.no_grad():
            out = model(torch.from_numpy(np.asarray(obs, dtype=np.float32)[None, :]))
        return out[0].numpy()

    return policy, contract


def _condition_config(
    config: ExperimentConfig,
    shapes: tuple[str, ...],
    weights: tuple[float, ...],
    seed_base: int,
    pool_size: int,
    randomization: RandomizationConfig,
) -> ExperimentConfig:
    assert config.env.tracks is not None
    family = dataclasses.replace(config.env.tracks.family, shapes=shapes, shape_weights=weights)
    tracks = dataclasses.replace(
        config.env.tracks, family=family, train_pool_size=pool_size, train_seed_base=seed_base
    )
    env = dataclasses.replace(config.env, tracks=tracks, max_episode_steps=3000)
    return dataclasses.replace(config, env=env, randomization=randomization)


def conditions(config: ExperimentConfig) -> dict[str, ExperimentConfig]:
    rnd = config.randomization
    nominal_rnd = rnd.nominal()
    stress_rnd = dataclasses.replace(rnd.widened(1.5), enabled=True)
    fam = (("stadium", "rounded_rect"), (0.5, 0.5))
    return {
        "heldout_family": _condition_config(config, *fam, HELDOUT_FAMILY_TRACK_SEED_BASE, 40, rnd),
        "heldout_shapes": _condition_config(
            config, ("blob", "large"), (0.5, 0.5), HELDOUT_SHAPES_TRACK_SEED_BASE, 24, rnd
        ),
        "stress_x1.5": _condition_config(
            config, *fam, HELDOUT_FAMILY_TRACK_SEED_BASE, 40, stress_rnd
        ),
        "nominal": _condition_config(config, *fam, HELDOUT_FAMILY_TRACK_SEED_BASE, 40, nominal_rnd),
    }


def _run_condition(
    args: tuple[str, Any, str, int, str],
) -> tuple[str, dict[str, Any], dict[str, Any], dict[str, Any]]:
    """One condition end to end (its own env, its own contract load): (name, summary,
    per-episode rows, contract provenance). A module-level function so conditions can run in
    separate processes; the result for a condition does not depend on which process ran it."""
    name, cfg, policy_dir, episodes, repo_root = args
    env = ResidualRacerEnv(cfg, repo_root=Path(repo_root))
    try:
        policy, contract = load_contract_policy(Path(policy_dir), env)
        base, learned = evaluate_paired(env, policy, episodes, HELDOUT_EPISODE_SEED_BASE, laps=2.0)
    finally:
        env.close()
    summary = summarize_pair(base, learned, seed=1)
    summary["by_direction"] = _by_direction(base, learned)
    rows = {
        "tracker": [r.as_dict() for r in base],
        "tracker_plus_residual": [r.as_dict() for r in learned],
    }
    provenance = {
        "contract_version": contract.contract_version,
        "config_hash": contract.training.config_hash,
        "git_sha": contract.training.git_sha,
    }
    return name, summary, rows, provenance


def run_report(
    policy_dir: Path,
    config: ExperimentConfig,
    out_dir: Path,
    episodes: int = 120,
    repo_root: Path = REPO_ROOT,
    workers: int = 1,
) -> dict[str, Any]:
    """`workers` > 1 runs the four conditions in separate processes; every episode is seeded
    by its own index, so the numbers are identical to the serial run."""
    out_dir = Path(out_dir)
    out_dir.mkdir(parents=True, exist_ok=True)
    report: dict[str, Any] = {"policy_dir": str(policy_dir), "episodes_per_condition": episodes}
    per_episode: dict[str, Any] = {}
    t0 = time.time()
    tasks = [
        (name, cfg, str(policy_dir), episodes, str(repo_root))
        for name, cfg in conditions(config).items()
    ]
    if workers > 1:
        from concurrent.futures import ProcessPoolExecutor

        with ProcessPoolExecutor(max_workers=min(workers, len(tasks))) as pool:
            results = list(pool.map(_run_condition, tasks))
    else:
        results = [_run_condition(t) for t in tasks]
    provenance: dict[str, Any] = {}
    for name, summary, rows, provenance in results:
        report[name] = summary
        per_episode[name] = rows
    report["contract"] = provenance
    report["wall_clock_s"] = time.time() - t0
    (out_dir / "eval_summary.json").write_text(json.dumps(report, indent=2, default=float))
    (out_dir / "eval_episodes.json").write_text(json.dumps(per_episode, default=float))
    return report


def _by_direction(base, learned) -> dict[str, Any]:
    out: dict[str, Any] = {}
    for sign, label in ((1, "ccw"), (-1, "cw")):
        idx = [i for i, r in enumerate(base) if (("ccw" in r.track_label) == (sign > 0))]
        if not idx:
            continue
        b = [base[i] for i in idx]
        ln = [learned[i] for i in idx]
        summary = summarize_pair(b, ln, seed=2)
        out[label] = {
            "episodes": len(idx),
            "completion_tracker": summary["tracker"]["completion_rate"],
            "completion_residual": summary["tracker_plus_residual"]["completion_rate"],
            "lap_time_diff_mean_pct": summary["paired"].get("lap_time_diff_mean_pct"),
        }
    return out


def format_markdown(report: dict[str, Any]) -> str:
    lines = []
    for name in ("heldout_family", "heldout_shapes", "stress_x1.5", "nominal"):
        if name not in report:
            continue
        r = report[name]
        t, p, pr = r["tracker"], r["tracker_plus_residual"], r["paired"]
        lines.append(f"### {name}\n")
        lines.append("| metric | tracker alone | tracker + residual |")
        lines.append("|---|---|---|")
        lines.append(
            f"| episodes completed (2 laps) | {t['completed']}/{t['episodes']} "
            f"({100 * t['completion_rate']:.1f}%, Wilson {100 * t['completion_wilson_95'][0]:.0f}-"
            f"{100 * t['completion_wilson_95'][1]:.0f}%) | {p['completed']}/{p['episodes']} "
            f"({100 * p['completion_rate']:.1f}%, Wilson {100 * p['completion_wilson_95'][0]:.0f}-"
            f"{100 * p['completion_wilson_95'][1]:.0f}%) |"
        )
        lines.append(f"| crashes | {t['crashes']} | {p['crashes']} |")
        lines.append(
            f"| mean lap time, completed (s) | {_f(t['lap_time_mean_s'])} | {_f(p['lap_time_mean_s'])} |"
        )
        lines.append(
            f"| mean abs cross-track error (m) | {t['mean_abs_cte_m']:.3f} | {p['mean_abs_cte_m']:.3f} |"
        )
        lines.append(
            f"| TTC-proxy engagements / episode | {t['ttc_engagements_per_episode']:.2f} | "
            f"{p['ttc_engagements_per_episode']:.2f} |"
        )
        lines.append(
            f"| envelope: residual clipped (steps) | {100 * t['clipped_fraction']:.2f}% | {100 * p['clipped_fraction']:.2f}% |"
        )
        lines.append(
            f"| envelope: rate limited (steps) | {100 * t['rate_limited_fraction']:.2f}% | {100 * p['rate_limited_fraction']:.2f}% |"
        )
        lines.append(
            f"| OOD fallback (steps) | {100 * t['ood_fraction']:.2f}% | {100 * p['ood_fraction']:.2f}% |"
        )
        if pr.get("n_both_completed"):
            lo, hi = pr["lap_time_diff_ci95_pct"]
            lines.append(
                f"\nPaired lap-time change over {pr['n_both_completed']} episodes both arms completed: "
                f"{pr['lap_time_diff_mean_pct']:+.2f}% (95% bootstrap CI {lo:+.2f}% to {hi:+.2f}%); "
                f"negative is faster.\n"
            )
        else:
            lines.append("\nNo episode was completed by both arms.\n")
    return "\n".join(lines)


def _f(v: float | None) -> str:
    return "n/a" if v is None else f"{v:.2f}"


def main(argv: list[str] | None = None) -> int:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--policy-dir", type=Path, required=True)
    parser.add_argument("--config", type=Path, required=True)
    parser.add_argument("--out-dir", type=Path, required=True)
    parser.add_argument("--episodes", type=int, default=120)
    parser.add_argument("--workers", type=int, default=1, help="conditions run in parallel")
    args = parser.parse_args(argv)
    report = run_report(
        args.policy_dir,
        load_config(args.config),
        args.out_dir,
        args.episodes,
        workers=args.workers,
    )
    md = format_markdown(report)
    (args.out_dir / "eval_tables.md").write_text(md)
    print(md)
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
