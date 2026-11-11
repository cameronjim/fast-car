"""Reference policies: the constant-action policy and what a paired evaluation does with it."""

from __future__ import annotations

import dataclasses

import numpy as np
import pytest
from conftest import FIXTURES_DIR, REPO_ROOT
from racer_train.config import load_config
from racer_train.env import ResidualRacerEnv
from racer_train.evaluate import evaluate_paired
from racer_train.reference_policies import constant


def test_constant_policy_ignores_the_observation_and_validates_its_shape() -> None:
    p = constant((0.0, 1.0))
    assert np.array_equal(p(np.zeros(47)), np.array([0.0, 1.0], dtype=np.float32))
    assert np.array_equal(p(np.ones(3)), np.array([0.0, 1.0], dtype=np.float32))
    with pytest.raises(ValueError):
        constant((1.0, 2.0, 3.0))  # type: ignore[arg-type]


@pytest.fixture(scope="module")
def env():
    cfg = load_config(FIXTURES_DIR / "v0_tiny_config.yaml")
    cfg = dataclasses.replace(cfg, vehicle_overrides={})
    e = ResidualRacerEnv(cfg, repo_root=REPO_ROOT)
    yield e
    e.close()


def test_a_zero_action_is_the_tracker_alone_in_a_paired_evaluation(env) -> None:
    """The pairing is only worth anything if "residual enabled, zero action" reproduces the
    tracker-only arm exactly: same world, same noise, same envelope."""
    base, zero = evaluate_paired(env, constant((0.0, 0.0)), 2, 4242, laps=0.15)
    for b, z in zip(base, zero, strict=True):
        assert b.success == z.success and b.steps == z.steps
        assert b.lap_time_s == z.lap_time_s
        assert b.mean_abs_cte_m == z.mean_abs_cte_m


def test_a_nonzero_constant_action_changes_the_run(env) -> None:
    base, fast = evaluate_paired(env, constant((0.0, 1.0)), 2, 4242, laps=0.15)
    assert any(
        b.steps != f.steps or b.mean_speed_mps != f.mean_speed_mps for b, f in zip(base, fast)
    )
