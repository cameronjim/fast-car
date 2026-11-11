"""PolicyCore: bounded residual, the envelope in the loop, and every fallback to the tracker.
ROS-free, torch-free (the model is a plain function)."""

from __future__ import annotations

import math
from pathlib import Path
from typing import Any

import numpy as np
import pytest
from policy_fixture import SPEED_SCALE_MPS, STEERING_SCALE_RAD, build_policy_dir
from racer_policy import live_observation as lo
from racer_policy import policy_core as pc
from racer_policy.deploy import ensure_envelope_importable, load_deployment

ensure_envelope_importable()

BEAMS = np.full(36, 3.0, dtype=np.float32)


def _line() -> lo.Raceline:
    n = 400
    t = np.linspace(0.0, 2.0 * math.pi, n, endpoint=False)
    return lo.Raceline(2.0 * np.cos(t), 2.0 * np.sin(t), t + math.pi / 2, np.full(n, 0.5), 2.0 * t)


def _core(tmp_path: Path, vp: Any, model: Any, **kw: Any) -> pc.PolicyCore:
    dep = load_deployment(build_policy_dir(tmp_path, vp), vp)
    return pc.PolicyCore(dep, vp, _line(), model, **kw)


def _step(core: pc.PolicyCore, t: float, **over: Any) -> pc.StepResult:
    args: dict[str, Any] = {
        "now_s": t,
        "velocity_mps": 1.8,
        "yaw_rate_rad_s": 0.0,
        "x_m": 2.0,
        "y_m": 0.0,
        "yaw_rad": math.pi / 2.0,
        "base_steering_rad": 0.1,
        "base_speed_mps": 1.8,
        "beams": BEAMS,
        "scan_age_s": 0.05,
    }
    args.update(over)
    return core.step(**args)


def test_zero_action_is_exactly_the_tracker_command(tmp_path: Path, real_vehicle_params: Any):
    core = _core(tmp_path, real_vehicle_params, lambda obs: np.zeros(2))
    r = _step(core, 0.0)
    assert r.active and r.reason == ""
    assert r.steering_rad == pytest.approx(0.1) and r.speed_mps == pytest.approx(1.8)


def test_the_network_sees_the_training_observation(tmp_path: Path, real_vehicle_params: Any):
    seen: list[np.ndarray] = []

    def model(obs: np.ndarray) -> np.ndarray:
        seen.append(obs.copy())
        return np.zeros(2)

    core = _core(tmp_path, real_vehicle_params, model)
    _step(core, 0.0)
    _step(core, 0.02, base_steering_rad=0.12)
    assert seen[0].shape == (47,) and seen[0].dtype == np.float32
    assert seen[0][5] == pytest.approx(0.1) and seen[1][5] == pytest.approx(0.12)
    # prev_cmd is the enveloped command of the previous step, not the base command of this one.
    assert seen[1][9] == pytest.approx(0.1)


def test_a_hostile_policy_is_held_to_the_envelope_bound(tmp_path: Path, real_vehicle_params: Any):
    """Max action in both channels every step: the output never differs from the base command
    by more than the residual bound, and the clipped flag never lies."""
    core = _core(tmp_path, real_vehicle_params, lambda obs: np.array([1.0, 1.0]))
    cfg = core.envelope_config
    span = cfg.steering_range[1] - cfg.steering_range[0]
    bound = cfg.residual_fraction_steering * span
    for i in range(200):
        r = _step(core, 0.02 * i, base_steering_rad=0.1, base_speed_mps=1.8)
        assert abs(r.steering_rad - 0.1) <= bound + 1e-9
        assert r.speed_mps <= cfg.speed_cap_mps + 1e-9
    assert bound == pytest.approx(0.025 * 0.898)
    assert STEERING_SCALE_RAD < bound  # the action scale itself sits inside the bound
    assert SPEED_SCALE_MPS < cfg.residual_fraction_speed * (cfg.speed_range[1] - cfg.speed_range[0])


def test_a_residual_is_added_to_the_base_command(tmp_path: Path, real_vehicle_params: Any):
    core = _core(tmp_path, real_vehicle_params, lambda obs: np.array([0.5, -0.5]))
    r = _step(core, 0.0)
    assert r.steering_rad == pytest.approx(0.1 + 0.5 * STEERING_SCALE_RAD)
    assert r.speed_mps == pytest.approx(1.8 - 0.5 * SPEED_SCALE_MPS)
    assert r.residual_steering_rad == pytest.approx(0.5 * STEERING_SCALE_RAD)


def test_speed_cap_applies_to_the_base_command_too(tmp_path: Path, real_vehicle_params: Any):
    core = _core(tmp_path, real_vehicle_params, lambda obs: np.zeros(2))
    r = _step(core, 0.0, base_speed_mps=5.0)
    assert r.speed_mps <= 2.0 + 1e-9


@pytest.mark.parametrize(
    ("over", "reason"),
    [
        ({"beams": None, "scan_age_s": None}, pc.REASON_NO_SCAN),
        ({"scan_age_s": 0.9}, pc.REASON_SCAN_STALE),
    ],
)
def test_missing_or_stale_scan_falls_back_to_the_tracker(
    tmp_path: Path, real_vehicle_params: Any, over: dict[str, Any], reason: str
):
    called: list[int] = []

    def model(obs: np.ndarray) -> np.ndarray:
        called.append(1)
        return np.array([1.0, 1.0])

    core = _core(tmp_path, real_vehicle_params, model)
    r = _step(core, 0.0, **over)
    assert not r.active and r.reason == reason
    assert not called  # the network is not even consulted without a usable scan
    assert r.steering_rad == pytest.approx(0.1) and r.speed_mps == pytest.approx(1.8)


@pytest.mark.parametrize(
    "bad", [np.array([np.nan, 0.0]), np.array([np.inf, 0.0]), np.zeros(3), np.zeros((2, 2))]
)
def test_a_non_finite_or_misshapen_action_falls_back(
    tmp_path: Path, real_vehicle_params: Any, bad: np.ndarray
):
    core = _core(tmp_path, real_vehicle_params, lambda obs: bad)
    r = _step(core, 0.0)
    assert not r.active and r.reason == pc.REASON_NONFINITE
    assert r.steering_rad == pytest.approx(0.1)


def test_a_raising_network_falls_back_and_does_not_crash(tmp_path: Path, real_vehicle_params: Any):
    def boom(obs: np.ndarray) -> np.ndarray:
        raise RuntimeError("inference failed")

    core = _core(tmp_path, real_vehicle_params, boom)
    r = _step(core, 0.0)
    assert not r.active and r.reason == pc.REASON_MODEL_ERROR
    assert r.speed_mps == pytest.approx(1.8)


def test_ood_state_falls_back_to_the_plain_tracker(tmp_path: Path, real_vehicle_params: Any):
    core = _core(tmp_path, real_vehicle_params, lambda obs: np.array([1.0, 1.0]))
    far = _step(core, 0.0, x_m=2.0 + 3.0, velocity_mps=1.8)  # 3 m off the line: lateral >> scale
    assert not far.active and far.reason == pc.REASON_OOD and far.ood
    # The residual was zeroed by the envelope: the command is the base command.
    assert far.steering_rad == pytest.approx(0.1)


def test_fallback_transitions_are_one_engage_and_one_release_with_duration(
    tmp_path: Path, real_vehicle_params: Any
):
    core = _core(tmp_path, real_vehicle_params, lambda obs: np.zeros(2))
    events = []
    for i in range(10):
        stale = 3 <= i < 7
        r = _step(core, 0.02 * i, scan_age_s=2.0 if stale else 0.05)
        if r.transition:
            events.append((i, r.transition, r.fallback_duration_s))
    assert [(i, kind) for i, kind, _ in events] == [(3, "engage"), (7, "release")]
    assert events[1][2] == pytest.approx(0.08)
    assert core.counters.fallback_engagements == 1
    assert core.counters.fallback_steps == 4 and core.counters.scan_stale_steps == 4


def test_reset_forgets_the_rate_limit_memory(tmp_path: Path, real_vehicle_params: Any):
    core = _core(tmp_path, real_vehicle_params, lambda obs: np.zeros(2))
    _step(core, 0.0, base_steering_rad=0.4)
    core.reset()
    # After a reset the first command seeds the limiter at the base command, so a very
    # different steering is passed through rather than rate-limited from the old value.
    r = _step(core, 5.0, base_steering_rad=-0.3)
    assert r.steering_rad == pytest.approx(-0.3)
    assert not r.rate_limited


def test_steering_rate_limit_is_enforced_without_a_reset(tmp_path: Path, real_vehicle_params: Any):
    core = _core(tmp_path, real_vehicle_params, lambda obs: np.zeros(2))
    _step(core, 0.0, base_steering_rad=0.4)
    r = _step(core, 0.02, base_steering_rad=-0.4)
    assert r.rate_limited
    assert abs(r.steering_rad - 0.4) <= core.envelope_config.max_delta_steering_rad + 1e-12


def test_jitter_histograms_and_snapshot(tmp_path: Path, real_vehicle_params: Any):
    core = _core(tmp_path, real_vehicle_params, lambda obs: np.zeros(2))
    for i in range(50):
        _step(core, 0.02 * i + (0.015 if i == 25 else 0.0))
    snap = core.snapshot()
    assert snap["state"] == "active"
    assert snap["inference_us"]["n"] == 50
    assert snap["loop_period_ms"]["n"] == 49
    # One late sample (35 ms period) lands in the 30..40 ms bin and the max remembers it.
    assert snap["loop_period_ms"]["max"] == pytest.approx(35.0)
    assert snap["counters"]["steps"] == 50
    assert sum(snap["inference_us"]["counts"]) == 50
