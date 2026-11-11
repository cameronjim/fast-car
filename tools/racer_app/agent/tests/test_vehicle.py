"""The UI draws with vehicle_params values served by the agent, never with its own constants."""

from __future__ import annotations

from pathlib import Path

import pytest


def test_vehicle_comes_from_vehicle_params_yaml(env):
    cfg = env.repo / "config"
    cfg.mkdir(exist_ok=True)
    (cfg / "vehicle_params.yaml").write_text(
        "meta:\n  schema_version: '9.9.9'\n"
        "sensors:\n  lidar:\n    mount_x_m: 0.3\n    mount_y_m: 0.0\n    mount_yaw_rad: 1.5\n"
        "chassis:\n  wheelbase_m: 0.33\n  length_m: 0.58\n  width_m: 0.31\n"
        "steering:\n  max_angle_rad: 0.42\n"
        "actuation:\n  throttle_full_scale_mps: 2.9\n"
    )
    v = env.controller.vehicle()
    assert v["schema_version"] == "9.9.9"
    assert v["lidar"] == {"mount_x_m": 0.3, "mount_y_m": 0.0, "mount_yaw_rad": 1.5}
    assert v["chassis"]["wheelbase_m"] == 0.33 and v["chassis"]["front_overhang_m"] is None
    assert v["steering"] == {"max_angle_rad": 0.42}
    assert v["actuation"] == {"throttle_full_scale_mps": 2.9}


def test_vehicle_with_no_file_is_all_null_not_guessed(env):
    v = env.controller.vehicle()
    assert v["lidar"] == {"mount_x_m": None, "mount_y_m": None, "mount_yaw_rad": None}
    (env.repo / "config").mkdir(exist_ok=True)
    (env.repo / "config" / "vehicle_params.yaml").write_text("{ not yaml")
    assert env.controller.vehicle()["lidar"]["mount_x_m"] is None


def test_vehicle_reads_the_real_repo_file_the_ui_depends_on(env):
    real = Path(__file__).resolve().parents[4] / "config" / "vehicle_params.yaml"
    if not real.exists():
        pytest.skip("repo vehicle_params.yaml not present")
    (env.repo / "config").mkdir(exist_ok=True)
    (env.repo / "config" / "vehicle_params.yaml").write_text(real.read_text())
    v = env.controller.vehicle()
    assert isinstance(v["lidar"]["mount_x_m"], float)
    assert isinstance(v["lidar"]["mount_yaw_rad"], float)
    assert isinstance(v["chassis"]["wheelbase_m"], float)
    assert isinstance(v["steering"]["max_angle_rad"], float)
    assert isinstance(v["actuation"]["throttle_full_scale_mps"], float)
    assert v["schema_version"]
