"""The held-out report's pools must not overlap training, tuning or validation data."""

from __future__ import annotations

from conftest import REPO_ROOT
from racer_train import heldout_report, train, tune_base
from racer_train.config import load_config

V0_CONFIG = REPO_ROOT / "training" / "configs" / "v0" / "sac_residual_v0_0.1.0.yaml"


def _track_seeds(base: int, size: int) -> set[int]:
    return set(range(base, base + size))


def test_heldout_track_pools_are_disjoint_from_training_tuning_and_validation():
    cfg = load_config(V0_CONFIG)
    train_pool = _track_seeds(cfg.env.tracks.train_seed_base, cfg.env.tracks.train_pool_size)
    tuning_pool = _track_seeds(tune_base.TUNING_TRACK_SEED_BASE, 24)
    val_pool = _track_seeds(train.VALIDATION_TRACK_SEED_BASE, 16)
    conds = heldout_report.conditions(cfg)
    for name, c in conds.items():
        pool = _track_seeds(c.env.tracks.train_seed_base, c.env.tracks.train_pool_size)
        for other in (train_pool, tuning_pool, val_pool):
            assert not (pool & other), name
    assert not (
        _track_seeds(heldout_report.HELDOUT_FAMILY_TRACK_SEED_BASE, 40)
        & _track_seeds(heldout_report.HELDOUT_SHAPES_TRACK_SEED_BASE, 24)
    )


def test_heldout_episode_seeds_are_disjoint_from_other_seed_ranges():
    held = set(
        range(
            heldout_report.HELDOUT_EPISODE_SEED_BASE, heldout_report.HELDOUT_EPISODE_SEED_BASE + 500
        )
    )
    tuning = set(
        range(tune_base.TUNING_EPISODE_SEED_BASE, tune_base.TUNING_EPISODE_SEED_BASE + 500)
    )
    val = set(range(train.VALIDATION_EPISODE_SEED_BASE, train.VALIDATION_EPISODE_SEED_BASE + 500))
    assert not (held & tuning) and not (held & val) and not (tuning & val)


def test_conditions_cover_the_four_named_cases_and_heldout_shapes_are_unseen_families():
    cfg = load_config(V0_CONFIG)
    conds = heldout_report.conditions(cfg)
    assert set(conds) == {"heldout_family", "heldout_shapes", "stress_x1.5", "nominal"}
    assert set(conds["heldout_shapes"].env.tracks.family.shapes) == {"blob", "large"}
    assert set(cfg.env.tracks.family.shapes).isdisjoint({"blob", "large"})
    assert conds["nominal"].randomization.velocity_noise_std_mps == (0.0, 0.0)
    wide = conds["stress_x1.5"].randomization.friction_mu
    base = cfg.randomization.friction_mu
    assert wide[1] - wide[0] > base[1] - base[0]
    assert all(c.env.max_episode_steps == 3000 for c in conds.values())


def test_markdown_tables_render_for_a_synthetic_report():
    arm = {
        "episodes": 4,
        "completed": 3,
        "completion_rate": 0.75,
        "completion_wilson_95": (0.3, 0.95),
        "crashes": 1,
        "timeouts": 0,
        "mean_abs_cte_m": 0.05,
        "ttc_engagements_per_episode": 2.0,
        "clipped_fraction": 0.0,
        "rate_limited_fraction": 0.1,
        "ood_fraction": 0.0,
        "mean_speed_mps": 1.5,
        "lap_time_mean_s": 6.0,
    }
    report = {
        "nominal": {
            "tracker": arm,
            "tracker_plus_residual": dict(arm, lap_time_mean_s=None),
            "paired": {
                "n_both_completed": 3,
                "lap_time_diff_mean_pct": -1.0,
                "lap_time_diff_ci95_pct": (-2.0, 0.0),
            },
        }
    }
    md = heldout_report.format_markdown(report)
    assert "### nominal" in md and "75.0%" in md and "n/a" in md and "-1.00%" in md
