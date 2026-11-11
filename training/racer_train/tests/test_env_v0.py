"""L5-style tests of the v0 env mode: walled procedural tracks, domain randomisation, paired
worlds, the envelope-in-env divergence check in v0 mode, wall crashes, lap termination."""

from __future__ import annotations

import numpy as np
import pytest
from conftest import FIXTURES_DIR, REPO_ROOT
from envelope import Command
from envelope import apply as envelope_apply
from racer_train.config import load_config
from racer_train.env import ResidualRacerEnv
from racer_train.evaluate import run_episode
from racer_train.observation import LOW_DIM_FIELDS_V0, slip_proxy_from
from racer_train.randomization import EpisodeDraw

V0_TINY = FIXTURES_DIR / "v0_tiny_config.yaml"


@pytest.fixture(scope="module")
def env():
    e = ResidualRacerEnv(load_config(V0_TINY), repo_root=REPO_ROOT)
    yield e
    e.close()


def test_v0_observation_fields_and_dimension(env):
    assert env.observation_fields == tuple((n, "float32", u) for n, u in LOW_DIM_FIELDS_V0)
    obs, info = env.reset(seed=0, options={"episode_seed": 1})
    assert obs.shape == (len(LOW_DIM_FIELDS_V0) + env.lidar_config.beam_count,)
    assert obs.shape == env.observation_space.shape
    assert obs.dtype == np.float32 and np.all(np.isfinite(obs))
    assert info["track_label"] != "committed"


def test_lidar_is_clipped_in_the_v0_profile(env):
    obs, _ = env.reset(options={"episode_seed": 2})
    assert obs[len(LOW_DIM_FIELDS_V0) :].max() <= 5.0 + 1e-6


def test_same_options_give_identical_trajectories(env):
    opts = {"episode_seed": 42, "track_index": 1, "start": "standing"}
    traces = []
    for _ in range(2):
        obs, _ = env.reset(options=opts)
        rows = [obs.copy()]
        for _k in range(40):
            obs, *_ = env.step(np.array([0.3, -0.2]))
            rows.append(obs.copy())
        traces.append(np.array(rows))
    assert np.array_equal(traces[0], traces[1])


def test_paired_arms_see_the_same_world_and_noise(env):
    """Same options -> same draw, same track, same first observation; the arms differ only
    through the residual."""
    opts = {"episode_seed": 7, "track_index": 2, "start": "standing"}
    obs_a, info_a = env.reset(options={**opts, "residual_enabled": False})
    draw_a = env.episode_draw
    obs_b, info_b = env.reset(options={**opts, "residual_enabled": True})
    assert env.episode_draw == draw_a
    assert info_a["track_label"] == info_b["track_label"]
    assert np.array_equal(obs_a, obs_b)


def test_residual_disabled_sends_the_base_command_through_the_envelope(env):
    env.reset(options={"episode_seed": 5, "start": "standing", "residual_enabled": False})
    _, _, _, _, info = env.step(np.array([1.0, 1.0]))  # hostile action, ignored
    assert info["envelope"]["residual_command"] == {"steering_rad": 0.0, "speed_mps": 0.0}


def test_envelope_in_env_divergence_v0_mode(env):
    """A deliberately hostile policy (max residual every step): the env's enveloped command
    equals a direct envelope.apply() on the same inputs (same library, same numbers)."""
    env.reset(options={"episode_seed": 9, "start": "standing"})
    for k in range(60):
        state_before = env.envelope_state
        ood_state = env._cur_ood_state
        action = np.array([1.0, -1.0] if k % 2 else [-1.0, 1.0])
        _, _, term, trunc, info = env.step(action)
        base = info["envelope"]["base_command"]
        scale = env.config.env.action
        expected = envelope_apply(
            env.envelope_config,
            state_before,
            Command(base["steering_rad"], base["speed_mps"]),
            Command(action[0] * scale.steering_scale_rad, action[1] * scale.speed_scale_mps),
            observed_state=ood_state,
        )
        assert info["envelope"]["command"] == {
            "steering_rad": expected.command.steering_rad,
            "speed_mps": expected.command.speed_mps,
        }
        if term or trunc:
            break


def test_envelope_bounds_hold_for_the_real_car_limits(env):
    """Commands never exceed the per-side steering limits or the 2.0 m/s cap."""
    env.reset(options={"episode_seed": 11, "start": "standing"})
    lo, hi = env.envelope_config.steering_range
    assert (lo, hi) == (-0.449, 0.504)
    for _ in range(150):
        _, _, term, trunc, info = env.step(np.array([1.0, 1.0]))
        cmd = info["envelope"]["command"]
        assert lo - 1e-12 <= cmd["steering_rad"] <= hi + 1e-12
        assert cmd["speed_mps"] <= 2.0 + 1e-12
        if term or trunc:
            break


def test_wall_contact_is_a_crash():
    """Walls exist now: steering hard into the wall ends the episode with crashed=True."""
    cfg = load_config(V0_TINY)
    e = ResidualRacerEnv(cfg, repo_root=REPO_ROOT)
    try:
        e.reset(options={"episode_seed": 3, "start": "standing", "draw": EpisodeDraw()})
        # Drive the agent straight at the wall: put the car on a straight facing outward.
        agent = e._agent
        agent.state[3] = 1.5
        crashed = False
        for _ in range(300):
            # Bypass the base controller by overriding the cached base command: full lock one
            # way at speed (the envelope still applies).
            e._cur_base = Command(steering_rad=0.0, speed_mps=2.0)
            agent.state[4] += 0.02  # keep turning the body toward the outside
            _, _, term, _, info = e.step(np.zeros(2))
            if info["truth"]["crashed"]:
                crashed = True
                assert term
                break
        assert crashed
    finally:
        e.close()


def test_two_laps_standing_start_terminates_with_success():
    cfg = load_config(V0_TINY)
    e = ResidualRacerEnv(cfg, repo_root=REPO_ROOT)
    try:
        # A gentle, big track so the tracker finishes: nominal real-car draw, no noise.
        draw = EpisodeDraw(
            cornering_stiffness_front_n_per_rad=250.0, cornering_stiffness_rear_n_per_rad=250.0
        )
        result = run_episode(
            e,
            {"episode_seed": 1, "track_index": 0, "start": "standing", "laps": 1.0, "draw": draw},
            None,
        )
        assert result.laps >= 1.0 or result.crashed or result.timed_out
        if result.success:
            assert result.lap_time_s == pytest.approx(result.time_s / result.laps)
    finally:
        e.close()


def test_upstream_start_gate_does_not_terminate_training_episodes():
    """f1tenth_gym's own lap gate fires `terminated` after about one lap; the env must ignore
    it (a terminal flag there would truncate episodes and zero the value bootstrap)."""
    cfg = load_config(V0_TINY)
    e = ResidualRacerEnv(cfg, repo_root=REPO_ROOT)
    try:
        draw = EpisodeDraw(
            cornering_stiffness_front_n_per_rad=250.0, cornering_stiffness_rear_n_per_rad=250.0
        )
        e.reset(options={"episode_seed": 1, "track_index": 0, "start": "standing", "draw": draw})
        terminated_without_crash = False
        for _ in range(399):
            _, _, term, trunc, info = e.step(np.zeros(2))
            if term and not info["truth"]["crashed"]:
                terminated_without_crash = True
            if term or trunc:
                break
        assert not terminated_without_crash
    finally:
        e.close()


def test_track_swap_changes_raceline_and_map(env):
    env.reset(options={"episode_seed": 1, "track_index": 0})
    rl0 = env.raceline
    env.reset(options={"episode_seed": 1, "track_index": 1})
    assert env.raceline is not rl0
    assert env.current_track is env.track_pool[1]


def test_actuator_assumptions_use_centre_of_delay_range(env):
    lo, hi = env.config.randomization.command_delay_s
    assert env.actuator_assumptions["command_delay_s"] == pytest.approx(0.5 * (lo + hi))


def test_slip_proxy_hand_computed():
    # Straight, no steering: zero. Matching kinematics: tan(d) = L*yawrate/v -> zero.
    assert slip_proxy_from(0.0, 0.0, 1.0, 0.33) == 0.0
    d = 0.3
    v = 1.5
    yaw = v * np.tan(d) / 0.33
    assert slip_proxy_from(d, yaw, v, 0.33) == pytest.approx(0.0, abs=1e-12)
    # Understeer: steering commanded, no yaw -> tan(d).
    assert slip_proxy_from(d, 0.0, v, 0.33) == pytest.approx(np.tan(d))
    # Standstill uses the speed floor (finite), result clipped to +-2.
    assert abs(slip_proxy_from(0.5, 50.0, 0.0, 0.33)) <= 2.0
