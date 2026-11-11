"""The live observation builder (racer_policy.live_observation, what policy_node feeds the
network on the car) against training's builder (racer_train.observation and ResidualRacerEnv's
`_sense`): the same inputs must give the same vector, and every constant the two share must be
the same number. A policy fed a different observation than it trained on is the quiet failure
this test exists to prevent.

Three layers:
  1. constants and the field schema, tied to training's own;
  2. function-level parity on random inputs (scan pooling, slip proxy, raceline-relative pose,
     envelope configuration);
  3. an end-to-end replay: run the real env (walled track, both directions, the car's dynamics,
     randomisation off so the observation is a deterministic function of true state), and at
     every step rebuild the observation with the LIVE builder from the raw quantities the env
     used (raw 1080-beam scan, pose, speed, yaw rate, the base command, the previous command)
     and require the vector the env returned.
"""

from __future__ import annotations

import dataclasses
import math

import numpy as np
import pytest
from conftest import FIXTURES_DIR, REPO_ROOT
from racer_policy import live_observation as lo
from racer_policy.deploy import Deployment, EnvelopeSettings, build_envelope_config
from racer_train import env as env_module
from racer_train import observation as tobs
from racer_train.config import load_config
from racer_train.env import ResidualRacerEnv
from racer_train.envelope_config import EnvelopeSettings as TrainEnvelopeSettings
from racer_train.envelope_config import build_envelope_config as train_build_envelope_config
from racer_train.raceline import Raceline as TrainRaceline
from racer_train.tracks import TrackFamilyConfig, build_pool

V0_TINY = FIXTURES_DIR / "v0_tiny_config.yaml"


# -- 1. constants and schema -------------------------------------------------------------------


def test_field_schema_is_trainings() -> None:
    assert lo.LIVE_FIELDS_V0 == tuple((n, tobs.OBS_DTYPE, u) for n, u in tobs.LOW_DIM_FIELDS_V0)


def test_shared_constants_are_trainings() -> None:
    assert lo.LIDAR_CLIP_M == tobs.LIDAR_CLIP_M
    assert lo.SLIP_PROXY_SPEED_FLOOR_MPS == tobs.SLIP_PROXY_SPEED_FLOOR_MPS
    assert lo.SLIP_PROXY_LIMIT == tobs.SLIP_PROXY_LIMIT
    assert lo.OOD_STATE_DIM == tobs.OOD_STATE_DIM
    assert lo.CURVATURE_NEAR_M == env_module.CURVATURE_NEAR_M
    assert lo.CURVATURE_FAR_M == env_module.CURVATURE_FAR_M


def test_lidar_block_is_what_the_v0_and_v1_configs_train_with() -> None:
    for name in ("v0/sac_residual_v0_quarter_0.1.0.yaml", "v1/sac_residual_v1_0.1.0.yaml"):
        cfg = load_config(REPO_ROOT / "training" / "configs" / name)
        assert cfg.env.lidar_downsample_factor == lo.LIDAR_DOWNSAMPLE_FACTOR
    env = ResidualRacerEnv(load_config(V0_TINY), repo_root=REPO_ROOT)
    try:
        assert env.lidar_config.beam_count == lo.LIDAR_BEAM_COUNT
        assert env.lidar_config.fov_rad == lo.LIDAR_FOV_RAD
        assert env.lidar_config.downsample_factor == lo.LIDAR_DOWNSAMPLE_FACTOR
        assert env.lidar_config.raw_beam_count == 1080
    finally:
        env.close()


# -- 2. function-level parity ------------------------------------------------------------------


@pytest.mark.parametrize("seed", range(5))
def test_pool_scan_equals_trainings_min_pool_on_a_training_scan(seed: int) -> None:
    rng = np.random.default_rng(seed)
    n = 1080
    ranges = rng.uniform(0.05, 12.0, n)
    ranges[rng.integers(0, n, 20)] = 0.3  # a few close returns
    angles = np.linspace(-lo.LIDAR_FOV_RAD / 2.0, lo.LIDAR_FOV_RAD / 2.0, n)
    live = lo.pool_scan(ranges, float(angles[0]), float(angles[1] - angles[0]), 0.0, 0.0)
    trained = np.minimum(
        tobs.downsample_scan(ranges, lo.LIDAR_DOWNSAMPLE_FACTOR), tobs.LIDAR_CLIP_M
    ).astype(np.float32)
    np.testing.assert_array_equal(live, trained)


def test_pool_scan_of_the_cars_scan_equals_pooling_the_same_world_in_training_geometry() -> None:
    """The car sees 360 degrees through a head mounted at pi; the same distances sampled on that
    layout must pool to what the training layout gives for the same world."""

    def world(bearing: np.ndarray) -> np.ndarray:
        return 1.5 + 1.2 * np.abs(np.sin(2.0 * bearing)) + 0.3 * np.cos(5.0 * bearing)

    n_car = 500
    inc = 2.0 * math.pi / n_car
    laser = -math.pi + inc * np.arange(n_car)
    vehicle_bearing = (laser + math.pi + math.pi) % (2.0 * math.pi) - math.pi
    live = lo.pool_scan(world(vehicle_bearing), -math.pi, inc, math.pi, 0.05)

    fine = np.linspace(-lo.LIDAR_FOV_RAD / 2.0, lo.LIDAR_FOV_RAD / 2.0, 1080)
    trained = tobs.downsample_scan(world(fine), lo.LIDAR_DOWNSAMPLE_FACTOR)
    # 500 live beams sample the world more coarsely than 1080 training beams, so the minima can
    # differ by the world's variation across one beam spacing; they must agree to that scale.
    assert np.max(np.abs(live - trained)) < 0.15


@pytest.mark.parametrize("seed", range(5))
def test_slip_proxy_equals_trainings(seed: int) -> None:
    rng = np.random.default_rng(seed)
    for _ in range(200):
        delta, w, v = rng.uniform(-0.5, 0.5), rng.uniform(-6, 6), rng.uniform(-0.5, 3.0)
        assert lo.slip_proxy(delta, w, v, 0.3302) == tobs.slip_proxy_from(delta, w, v, 0.3302)


def test_raceline_relative_pose_and_neighbours_equal_trainings(tmp_path) -> None:
    family = TrackFamilyConfig(inner_radius_m=(0.3, 0.3))
    bundle = build_pool(family, 1, 4242)[0]
    csv_path = REPO_ROOT / "config" / "tracks" / "gym_oval" / "raceline.csv"
    for train_line, live_line in (
        (bundle.raceline, _live_from_train(bundle.raceline)),
        (TrainRaceline.load_from_csv(csv_path), lo.Raceline.from_csv(csv_path)),
    ):
        rng = np.random.default_rng(0)
        for _ in range(100):
            i = int(rng.integers(0, len(train_line)))
            p = train_line.at(i)
            x = p.x_m + rng.uniform(-0.4, 0.4)
            y = p.y_m + rng.uniform(-0.4, 0.4)
            yaw = rng.uniform(-math.pi, math.pi)
            t_lat, t_head = tobs.raceline_relative_pose(train_line, x, y, yaw)
            l_lat, l_head, l_idx = lo.raceline_relative_pose(live_line, x, y, yaw)
            assert (l_lat, l_head) == (t_lat, t_head)
            assert l_idx == train_line.nearest_index(x, y)
            for d in (env_module.CURVATURE_NEAR_M, env_module.CURVATURE_FAR_M):
                assert live_line.curvature[live_line.index_ahead(l_idx, d)] == (
                    train_line.at(train_line.index_ahead(l_idx, d)).curvature_1pm
                )
        assert live_line.length_m == pytest.approx(train_line.length_m)


def _live_from_train(line: TrainRaceline) -> lo.Raceline:
    pts = [line.at(i) for i in range(len(line))]
    return lo.Raceline(
        np.array([p.x_m for p in pts]),
        np.array([p.y_m for p in pts]),
        np.array([p.heading_rad for p in pts]),
        np.array([p.curvature_1pm for p in pts]),
        np.array([p.s_m for p in pts]),
    )


def test_envelope_config_equals_trainings(real_vehicle_params) -> None:
    train_cfg = train_build_envelope_config(
        TrainEnvelopeSettings(
            residual_fraction_steering=0.025,
            residual_fraction_speed=0.004,
            speed_cap_mps=2.0,
            ood_threshold=3.0,
            ood_reference_state=(1.8, 0.0, 0.0, 0.0, 0.0),
            ood_reference_scale=(0.8, 2.5, 0.3, 0.2, 0.4),
        ),
        real_vehicle_params,
        0.02,
    )
    dep = Deployment(
        contract=None,  # type: ignore[arg-type]
        envelope=EnvelopeSettings(
            residual_fraction_steering=0.025,
            residual_fraction_speed=0.004,
            speed_cap_mps=2.0,
            ood_threshold=3.0,
            ood_reference_state=(1.8, 0.0, 0.0, 0.0, 0.0),
            ood_reference_scale=(0.8, 2.5, 0.3, 0.2, 0.4),
            control_rate_hz=50.0,
        ),
        steering_scale_rad=0.0222,
        speed_scale_mps=0.099,
        base_controller=None,  # type: ignore[arg-type]
        meta={},
        policy_dir=REPO_ROOT,
    )
    assert build_envelope_config(dep, real_vehicle_params) == train_cfg


# -- 3. end-to-end replay ----------------------------------------------------------------------


def _nominal_config():
    cfg = load_config(V0_TINY)
    return dataclasses.replace(
        cfg,
        vehicle_overrides={},
        randomization=dataclasses.replace(cfg.randomization, enabled=False),
    )


@pytest.mark.parametrize("track_index", [0, 1, 2])
def test_live_builder_reproduces_the_env_observation_at_every_step(track_index: int) -> None:
    env = ResidualRacerEnv(_nominal_config(), repo_root=REPO_ROOT)
    try:
        rng = np.random.default_rng(track_index)
        obs, _info = env.reset(
            options={
                "episode_seed": 11 + track_index,
                "track_index": track_index,
                "start": "standing",
            }
        )
        live = lo.LiveObservationBuilder(
            _live_from_train(env.raceline), float(env.vehicle_params.chassis.wheelbase_m)
        )
        agent = env._agent
        angles = np.asarray(agent.scan_angles)
        inc = float(angles[1] - angles[0])
        checked = 0
        for step in range(260):
            # The previous command and the base command the env used to build THIS observation.
            gym_obs = env._last_gym_obs
            pose = env._pose_from_obs(gym_obs)
            beams = lo.pool_scan(np.asarray(gym_obs["scans"])[0], float(angles[0]), inc, 0.0, 0.0)
            rebuilt = live.build(
                lo.LiveInputs(
                    velocity_mps=float(np.asarray(gym_obs["linear_vels_x"])[0]),
                    yaw_rate_rad_s=float(np.asarray(gym_obs["ang_vels_z"])[0]),
                    x_m=pose[0],
                    y_m=pose[1],
                    yaw_rad=pose[2],
                    base_steering_rad=env._cur_base.steering_rad,
                    base_speed_mps=env._cur_base.speed_mps,
                    prev_steering_cmd_rad=env._prev_cmd[0],
                    prev_speed_cmd_mps=env._prev_cmd[1],
                    beams=beams,
                )
            )
            np.testing.assert_allclose(rebuilt, obs, rtol=0.0, atol=1e-6, err_msg=f"step {step}")
            checked += 1
            action = rng.uniform(-1.0, 1.0, size=2).astype(np.float32)
            obs, _r, terminated, truncated, _i = env.step(action)
            if terminated or truncated:
                break
        assert checked >= 100  # the replay covered a real stretch of driving, corners included
    finally:
        env.close()
