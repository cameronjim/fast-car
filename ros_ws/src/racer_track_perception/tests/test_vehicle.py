from __future__ import annotations

from pathlib import Path
from types import SimpleNamespace

import pytest
from racer_track_perception import vehicle as vh


def fake_params(**overrides):
    lidar = {"mount_x_m": 0.1, "mount_y_m": 0.0, "mount_yaw_rad": 3.0}
    chassis = {
        "rear_overhang_m": 0.1,
        "front_overhang_m": 0.1,
        "wheelbase_m": 0.3,
        "width_m": 0.2,
    }
    for key, value in overrides.items():
        (lidar if key.startswith("mount") else chassis)[key] = value
    return SimpleNamespace(
        sensors=SimpleNamespace(lidar=SimpleNamespace(**lidar)),
        chassis=SimpleNamespace(**chassis),
        meta=SimpleNamespace(schema_version="9.9.9", sysid_session_id="s"),
    )


def test_from_generated_reads_every_field() -> None:
    live = vh.from_generated(fake_params())
    assert live.mount == vh.LidarMount(0.1, 0.0, 3.0)
    assert live.body.x_max_m == pytest.approx(0.4) and live.body.x_min_m == -0.1
    assert live.body.half_width_m == pytest.approx(0.1)
    assert (live.schema_version, live.sysid_session_id) == ("9.9.9", "s")


@pytest.mark.parametrize(
    "field",
    [
        "mount_x_m",
        "mount_y_m",
        "mount_yaw_rad",
        "rear_overhang_m",
        "front_overhang_m",
        "wheelbase_m",
        "width_m",
    ],
)
def test_a_null_field_refuses(field: str) -> None:
    with pytest.raises(vh.VehicleParamsNull, match="null"):
        vh.from_generated(fake_params(**{field: None}))


def test_live_vehicle_comes_from_the_committed_yaml(live: vh.LiveVehicle) -> None:
    assert live.mount.yaw_rad > 3.0 and live.body.width_m > 0.0
    assert vh.find_repo_root().joinpath("config", "vehicle_params.yaml").is_file()


def test_find_repo_root_env_and_failure(
    monkeypatch: pytest.MonkeyPatch, tmp_path: Path, live: vh.LiveVehicle
) -> None:
    root = vh.find_repo_root()
    monkeypatch.setenv("RACER_REPO_ROOT", str(root))
    assert vh.find_repo_root() == root
    monkeypatch.setenv("RACER_REPO_ROOT", str(tmp_path))  # no yaml there: ignored
    assert vh.find_repo_root() == root
    with pytest.raises(FileNotFoundError):
        vh.find_repo_root(tmp_path)
