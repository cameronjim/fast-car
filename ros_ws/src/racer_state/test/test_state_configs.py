"""L1 checks of racer_state's shipped EKF, slam_toolbox and AMCL configs.

These files are interfaces: the launch files remap by the frame and topic names written here,
and the LiDAR range limits must agree with vehicle_params' sensors.lidar_spec (CLAUDE.md
invariant 2: the configs may not claim a longer range than the sensor has). Plain YAML reads,
no ROS.
"""

from __future__ import annotations

from pathlib import Path

import yaml

_PKG = Path(__file__).resolve().parents[1]
_REPO_ROOT = Path(__file__).resolve().parents[4]


def _params(name: str, node: str) -> dict:
    with (_PKG / "config" / name).open() as f:
        return yaml.safe_load(f)[node]["ros__parameters"]


def _lidar_spec() -> dict:
    with (_REPO_ROOT / "config" / "vehicle_params.yaml").open() as f:
        return yaml.safe_load(f)["sensors"]["lidar_spec"]


def test_ekf_is_planar_odometry_from_the_wheel_twist():
    p = _params("ekf.yaml", "ekf_node")
    assert p["two_d_mode"] is True
    assert p["world_frame"] == p["odom_frame"] == "odom"
    assert p["base_link_frame"] == "base_link"
    assert p["publish_tf"] is True
    assert p["frequency"] >= 100.0  # claude-docs/04-architecture.md: /odom >= 100 Hz
    assert p["twist0"] == "/odom/wheel_twist"
    config = p["twist0_config"]
    assert len(config) == 15
    # vx, vy, vyaw fused; nothing else.
    assert [i for i, used in enumerate(config) if used] == [6, 7, 11]
    # The IMU slot is reserved, not filled: there is no IMU on the car yet.
    assert "imu0" not in p
    for key in ("process_noise_covariance", "initial_estimate_covariance"):
        assert len(p[key]) == 225
        assert all(p[key][i * 16] > 0.0 for i in range(15)), f"{key} needs a positive diagonal"


def test_slam_toolbox_frames_topics_and_range():
    p = _params("slam_toolbox.yaml", "slam_toolbox")
    assert p["mode"] == "mapping"
    assert (p["map_frame"], p["odom_frame"], p["base_frame"]) == ("map", "odom", "base_link")
    assert p["scan_topic"] == "/scan"
    spec = _lidar_spec()
    assert p["max_laser_range"] <= spec["range_max_m"]
    assert p["min_laser_range"] >= spec["range_min_m"]
    assert p["enable_interactive_mode"] is False


def test_amcl_frames_topics_and_range():
    p = _params("amcl.yaml", "amcl")
    assert (p["global_frame_id"], p["odom_frame_id"], p["base_frame_id"]) == (
        "map",
        "odom",
        "base_link",
    )
    assert p["tf_broadcast"] is True
    assert p["robot_model_type"] == "nav2_amcl::DifferentialMotionModel"
    spec = _lidar_spec()
    assert p["laser_max_range"] <= spec["range_max_m"]
    assert p["laser_min_range"] >= spec["range_min_m"]
    assert p["min_particles"] <= p["max_particles"]
    assert 0 < p["max_beams"] <= round(spec["field_of_view_rad"] / spec["angular_resolution_rad"])
