"""unit tests for the exported-policy deploy runtime, no ros."""

import json
import sys
from pathlib import Path

import numpy as np
import pytest

REPO_ROOT = Path(__file__).resolve().parents[2]
sys.path.insert(0, str(REPO_ROOT))
sys.path.insert(0, str(REPO_ROOT / "learned_control" / "nodes"))

from rl_policy_runtime import (  # noqa: E402
    DeployContract,
    RateGate,
    RLPolicyRuntime,
    VehicleState,
    load_contract,
    load_policy,
)

M2_DIR = REPO_ROOT / "gym_training" / "artifacts" / "m2"

NUM_BEAMS = 32
NUM_RAYS = 1080
SCAN_RANGE_MAX_M = 30.0
SPEED_NORM_MPS = 8.0
YAW_RATE_NORM_RPS = 5.0
STEER_NORM_RAD = 0.4189
EY_NORM_M = 5.0
REF_LATERAL_NORM_M = 2.0
CURVATURE_NORM_RADPM = 0.5
CURVATURE_HORIZONS_M = [5.0, 10.0, 15.0, 20.0, 30.0]
CONTROL_HZ = 25.0
STEER_MAX_RAD = 0.4189
SPEED_MIN_MPS = 0.5
SPEED_CAP_MPS = 3.0

DEPLOY_FEATURES = ("scan", "linear_vel_x", "ang_vel_z", "delta")


def _scale(norm: float) -> float:
    # obs.py stores the scale array as float32 before json.dumps, so mirror that rounding
    return float(np.float32(1.0 / norm))


def deploy_blob() -> dict:
    """a deploy-shaped obs_config.json, mirroring what obs.py writes without sim-only features."""
    return {
        "version": 1,
        "features": [
            {
                "name": "ang_vel_z", "start": 0, "size": 1,
                "scale": [_scale(YAW_RATE_NORM_RPS)], "deployable": True,
            },
            {
                "name": "delta", "start": 1, "size": 1,
                "scale": [_scale(STEER_NORM_RAD)], "deployable": True,
            },
            {
                "name": "linear_vel_x", "start": 2, "size": 1,
                "scale": [_scale(SPEED_NORM_MPS)], "deployable": True,
            },
            {
                "name": "scan", "start": 3, "size": NUM_BEAMS,
                "scale": [_scale(SCAN_RANGE_MAX_M)] * NUM_BEAMS, "deployable": True,
            },
        ],
        "feature_order": ["ang_vel_z", "delta", "linear_vel_x", "scan"],
        "configured_features": list(DEPLOY_FEATURES),
        # a deploy export carries no reference-line context, but the keys still ship
        "context_features": [],
        "context_dim": 0,
        "num_beams": NUM_BEAMS,
        "raw_dim": 3 + NUM_BEAMS,
        "obs_dim": 5 + NUM_BEAMS,
        "include_prev_action": True,
        "clip_abs": 1.0,
        "control_hz": CONTROL_HZ,
        "norm": {
            "scan_range_max_m": SCAN_RANGE_MAX_M,
            "speed_norm_mps": SPEED_NORM_MPS,
            "yaw_rate_norm_rps": YAW_RATE_NORM_RPS,
            "steer_norm_rad": STEER_NORM_RAD,
            "ey_norm_m": EY_NORM_M,
            "track_length_m": None,
            "ref_lateral_norm_m": REF_LATERAL_NORM_M,
            "curvature_norm_radpm": CURVATURE_NORM_RADPM,
            "curvature_horizons_m": CURVATURE_HORIZONS_M,
        },
        "action": {
            "steer_max_rad": STEER_MAX_RAD,
            "speed_min_mps": SPEED_MIN_MPS,
            "speed_cap_mps": SPEED_CAP_MPS,
        },
    }


def fake_ranges(seed: int = 0) -> np.ndarray:
    rng = np.random.default_rng(seed)
    return rng.uniform(0.2, 25.0, size=NUM_RAYS).astype(np.float32)


class RecordingPolicy:
    """stub policy returning a fixed unit action and keeping every observation it saw."""

    def __init__(self, unit_action):
        self.unit_action = np.asarray(unit_action, dtype=np.float32)
        self.seen = []

    def __call__(self, obs):
        self.seen.append(np.asarray(obs, dtype=np.float32).copy())
        return self.unit_action


def test_the_real_m2_export_is_refused_and_names_the_sim_only_feature():
    with pytest.raises(ValueError) as err:
        load_contract(M2_DIR / "obs_config.json")

    assert "frenet_pose" in str(err.value)
    assert str(M2_DIR / "obs_config.json") in str(err.value)


def test_a_feature_this_node_has_no_sensor_for_is_refused_even_if_flagged_deployable():
    blob = deploy_blob()
    blob["features"][0]["name"] = "pose_theta"
    blob["feature_order"][0] = "pose_theta"

    with pytest.raises(ValueError, match="pose_theta"):
        DeployContract(blob)


def test_a_deployable_false_flag_is_refused():
    blob = deploy_blob()
    blob["features"][3]["deployable"] = False

    with pytest.raises(ValueError, match="scan"):
        DeployContract(blob)


def test_missing_top_level_keys_are_named():
    blob = deploy_blob()
    del blob["action"]
    del blob["control_hz"]

    with pytest.raises(ValueError) as err:
        DeployContract(blob, source="policies/obs_config.json")

    assert "action" in str(err.value)
    assert "control_hz" in str(err.value)
    assert "policies/obs_config.json" in str(err.value)


def test_missing_action_bounds_are_named():
    blob = deploy_blob()
    del blob["action"]["speed_cap_mps"]

    with pytest.raises(ValueError, match="speed_cap_mps"):
        DeployContract(blob)


def test_an_unknown_contract_version_is_refused():
    blob = deploy_blob()
    blob["version"] = 2

    with pytest.raises(ValueError, match="version"):
        DeployContract(blob)


def test_a_beam_count_that_disagrees_with_the_scan_span_is_refused():
    blob = deploy_blob()
    blob["num_beams"] = NUM_BEAMS + 1

    with pytest.raises(ValueError, match="beams"):
        DeployContract(blob)


def test_a_feature_layout_with_a_gap_is_refused():
    blob = deploy_blob()
    blob["features"][3]["start"] = 4
    blob["raw_dim"] = 4 + NUM_BEAMS
    blob["obs_dim"] = 6 + NUM_BEAMS

    with pytest.raises(ValueError, match="tile without gaps"):
        DeployContract(blob)


def test_an_obs_dim_that_forgets_the_previous_action_is_refused():
    blob = deploy_blob()
    blob["obs_dim"] = blob["raw_dim"]

    with pytest.raises(ValueError, match="obs_dim"):
        DeployContract(blob)


def test_a_scale_list_shorter_than_its_feature_is_refused():
    blob = deploy_blob()
    blob["features"][3]["scale"] = blob["features"][3]["scale"][:-1]

    with pytest.raises(ValueError, match="scale factors"):
        DeployContract(blob)


def test_the_observation_is_ordered_and_scaled_by_the_contract():
    contract = DeployContract(deploy_blob())
    ranges = fake_ranges()
    state = VehicleState(linear_vel_x_mps=4.0, ang_vel_z_rps=1.0, steering_rad=0.1)

    obs = contract.build_obs(ranges, state, prev_action=[0.25, -0.5])

    assert obs.shape == (contract.obs_dim,)
    assert obs.dtype == np.float32
    assert obs[0] == pytest.approx(1.0 / YAW_RATE_NORM_RPS, abs=1e-6)
    assert obs[1] == pytest.approx(0.1 / STEER_NORM_RAD, abs=1e-6)
    assert obs[2] == pytest.approx(4.0 / SPEED_NORM_MPS, abs=1e-6)
    assert np.all(np.abs(obs) <= contract.clip_abs)
    assert obs[-2:] == pytest.approx([0.25, -0.5])


def test_the_scan_is_downsampled_across_the_whole_sweep():
    contract = DeployContract(deploy_blob())
    ranges = np.full(NUM_RAYS, SCAN_RANGE_MAX_M, dtype=np.float32)
    ranges[0] = 1.0
    ranges[NUM_RAYS - 1] = 2.0
    scan = contract.slices["scan"]

    obs = contract.build_obs(ranges, VehicleState())

    assert obs[scan.start] == pytest.approx(1.0 / SCAN_RANGE_MAX_M, abs=1e-6)
    assert obs[scan.stop - 1] == pytest.approx(2.0 / SCAN_RANGE_MAX_M, abs=1e-6)
    assert obs[scan.start + 1] == pytest.approx(1.0)


def test_non_finite_and_over_range_rays_read_as_max_range():
    contract = DeployContract(deploy_blob())
    ranges = np.full(NUM_RAYS, 3.0, dtype=np.float32)
    ranges[0] = np.inf
    ranges[NUM_RAYS - 1] = 1e6
    scan = contract.slices["scan"]

    obs = contract.build_obs(ranges, VehicleState())

    assert obs[scan.start] == pytest.approx(1.0)
    assert obs[scan.stop - 1] == pytest.approx(1.0)


def test_non_finite_odometry_never_reaches_the_policy():
    contract = DeployContract(deploy_blob())
    state = VehicleState(linear_vel_x_mps=np.nan, ang_vel_z_rps=np.inf)

    obs = contract.build_obs(fake_ranges(), state)

    assert np.all(np.isfinite(obs))
    assert obs[0] == pytest.approx(contract.clip_abs)
    assert obs[2] == pytest.approx(0.0)


def test_a_scan_shorter_than_the_trained_beam_count_is_refused():
    contract = DeployContract(deploy_blob())

    with pytest.raises(ValueError, match="fewer than the 32 beams"):
        contract.build_obs(np.full(16, 3.0, dtype=np.float32), VehicleState())


def test_the_action_maps_onto_the_exported_bounds():
    contract = DeployContract(deploy_blob())

    assert contract.drive_command([-1.0, -1.0]) == pytest.approx(
        (-STEER_MAX_RAD, SPEED_MIN_MPS))
    assert contract.drive_command([1.0, 1.0]) == pytest.approx((STEER_MAX_RAD, SPEED_CAP_MPS))
    assert contract.drive_command([0.0, 0.0]) == pytest.approx(
        (0.0, 0.5 * (SPEED_MIN_MPS + SPEED_CAP_MPS)))


def test_an_out_of_box_action_is_clipped_before_it_is_mapped():
    contract = DeployContract(deploy_blob())

    assert contract.drive_command([5.0, 5.0]) == pytest.approx((STEER_MAX_RAD, SPEED_CAP_MPS))
    assert contract.drive_command([-5.0, -5.0]) == pytest.approx(
        (-STEER_MAX_RAD, SPEED_MIN_MPS))


def test_a_non_finite_action_becomes_straight_and_slow():
    contract = DeployContract(deploy_blob())

    assert contract.drive_command([np.nan, np.nan]) == pytest.approx((0.0, SPEED_MIN_MPS))


def test_prev_action_is_the_executed_clipped_unit_action():
    contract = DeployContract(deploy_blob())
    policy = RecordingPolicy([2.0, -3.0])
    runtime = RLPolicyRuntime(contract, policy)
    ranges = fake_ranges()

    runtime.step(ranges, now_sec=0.0)
    runtime.step(ranges, now_sec=1.0)

    assert runtime.prev_action == pytest.approx([1.0, -1.0])
    assert policy.seen[0][-2:] == pytest.approx([0.0, 0.0])
    assert policy.seen[1][-2:] == pytest.approx([1.0, -1.0])


def test_delta_carries_the_steering_the_last_action_commanded():
    contract = DeployContract(deploy_blob())
    policy = RecordingPolicy([0.5, 0.0])
    runtime = RLPolicyRuntime(contract, policy)
    ranges = fake_ranges()

    runtime.step(ranges, now_sec=0.0)
    runtime.step(ranges, now_sec=1.0)

    delta = contract.slices["delta"].start
    assert policy.seen[0][delta] == pytest.approx(0.0)
    assert policy.seen[1][delta] == pytest.approx(0.5 * STEER_MAX_RAD / STEER_NORM_RAD, abs=1e-6)


def test_odometry_reaches_the_observation():
    contract = DeployContract(deploy_blob())
    policy = RecordingPolicy([0.0, 0.0])
    runtime = RLPolicyRuntime(contract, policy)
    runtime.update_odom(2.0, 0.0, -1.5)

    runtime.step(fake_ranges(), now_sec=0.0)

    assert policy.seen[0][0] == pytest.approx(-1.5 / YAW_RATE_NORM_RPS, abs=1e-6)
    assert policy.seen[0][2] == pytest.approx(2.0 / SPEED_NORM_MPS, abs=1e-6)


def test_a_faster_lidar_is_decimated_to_the_control_rate():
    contract = DeployContract(deploy_blob())
    runtime = RLPolicyRuntime(contract, RecordingPolicy([0.0, 0.0]))
    ranges = fake_ranges()

    commands = [runtime.step(ranges, now_sec=i * 0.01) for i in range(101)]

    driven = [command for command in commands if command is not None]
    assert len(driven) == int(CONTROL_HZ) + 1


def test_a_lidar_at_the_control_rate_is_never_decimated():
    contract = DeployContract(deploy_blob())
    runtime = RLPolicyRuntime(contract, RecordingPolicy([0.0, 0.0]))
    ranges = fake_ranges()

    commands = [runtime.step(ranges, now_sec=i * 0.04) for i in range(50)]

    assert all(command is not None for command in commands)


def test_a_clock_that_restarts_resyncs_instead_of_stalling():
    gate = RateGate(CONTROL_HZ)

    assert gate.due(100.0)
    assert not gate.due(100.01)
    assert gate.due(0.0)


def test_reset_forgets_the_previous_action_and_the_control_clock():
    contract = DeployContract(deploy_blob())
    policy = RecordingPolicy([1.0, 1.0])
    runtime = RLPolicyRuntime(contract, policy)
    ranges = fake_ranges()

    runtime.step(ranges, now_sec=0.0)
    runtime.reset()
    runtime.step(ranges, now_sec=0.001)

    assert runtime.prev_action == pytest.approx([1.0, 1.0])
    assert len(policy.seen) == 2
    assert policy.seen[1][-2:] == pytest.approx([0.0, 0.0])


def test_the_hand_written_fixture_still_matches_what_the_exporter_writes():
    sys.path.insert(0, str(REPO_ROOT / "gym_training" / "f1rl" / "envs"))
    obs_module = pytest.importorskip("obs", reason="gym_training obs module is not importable")

    obs_cfg = obs_module.ObsConfig(
        features=DEPLOY_FEATURES,
        num_beams=NUM_BEAMS,
        scan_range_max_m=SCAN_RANGE_MAX_M,
        speed_norm_mps=SPEED_NORM_MPS,
        yaw_rate_norm_rps=YAW_RATE_NORM_RPS,
        steer_norm_rad=STEER_NORM_RAD,
        ey_norm_m=EY_NORM_M,
        control_hz=CONTROL_HZ,
    )
    bounds = obs_module.ActionBounds(
        steer_max_rad=STEER_MAX_RAD,
        speed_min_mps=SPEED_MIN_MPS,
        speed_cap_mps=SPEED_CAP_MPS,
    )

    assert obs_module.deploy_contract(obs_cfg, bounds) == deploy_blob()


def test_the_deploy_observation_equals_what_training_normalized():
    sys.path.insert(0, str(REPO_ROOT / "gym_training" / "f1rl" / "envs"))
    obs_module = pytest.importorskip("obs", reason="gym_training obs module is not importable")
    scan_module = pytest.importorskip("learned_control.preprocessing.scan")

    obs_cfg = obs_module.ObsConfig(
        features=DEPLOY_FEATURES,
        num_beams=NUM_BEAMS,
        scan_range_max_m=SCAN_RANGE_MAX_M,
        speed_norm_mps=SPEED_NORM_MPS,
        yaw_rate_norm_rps=YAW_RATE_NORM_RPS,
        steer_norm_rad=STEER_NORM_RAD,
        ey_norm_m=EY_NORM_M,
        control_hz=CONTROL_HZ,
    )
    contract = DeployContract(deploy_blob())
    ranges = fake_ranges(seed=7)
    prev_action = np.array([0.3, -0.8], dtype=np.float32)
    state = VehicleState(linear_vel_x_mps=3.5, ang_vel_z_rps=-0.9, steering_rad=0.2)

    beams = ranges[scan_module.downsample_indices(NUM_RAYS, NUM_BEAMS)]
    raw = np.concatenate((
        [state.ang_vel_z_rps, state.steering_rad, state.linear_vel_x_mps], beams,
    )).astype(np.float32)

    trained = obs_cfg.normalize(raw, prev_action)
    deployed = contract.build_obs(ranges, state, prev_action)

    assert deployed == pytest.approx(trained, abs=1e-7)


def test_the_exported_m2_policy_maps_its_contract_width_onto_two_bounded_actions():
    pytest.importorskip("torch", reason="torch is not installed on the windows dev box")
    # m2 is a sim-only export, so its width is read straight from the json the loader refuses
    obs_dim = json.loads((M2_DIR / "obs_config.json").read_text())["obs_dim"]
    policy = load_policy(M2_DIR / "policy.pt")

    action = policy(np.zeros(obs_dim, dtype=np.float32))

    assert action.shape == (2,)
    assert np.all(np.isfinite(action))
    assert np.all(np.abs(action) <= 1.0)
