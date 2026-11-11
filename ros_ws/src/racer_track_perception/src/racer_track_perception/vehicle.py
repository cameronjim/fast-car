"""The slice of vehicle_params this package needs: LiDAR mount, body footprint, and the two
strings a model bundle is checked against.

Nothing here is a constant: every number comes from config/vehicle_params.yaml through the
generated binding (tools/gen_params.py). The node gets the binding from
`racer_tools.vehicle_params_loader`; offline code (tests, training, the bag gate) gets the same
generated object from `load_live_vehicle`, which imports tools/gen_params.py by path exactly the
way that loader does.
"""

from __future__ import annotations

import importlib.util
import os
import sys
import types
from dataclasses import dataclass
from pathlib import Path
from typing import Any

_REPO_MARKER = Path("config") / "vehicle_params.yaml"


class VehicleParamsNull(ValueError):
    """A vehicle_params field this package needs is null (not yet measured)."""


@dataclass(frozen=True)
class LidarMount:
    """LiDAR head pose in base_link: position (m) and mounting yaw (rad, CCW positive)."""

    x_m: float
    y_m: float
    yaw_rad: float


@dataclass(frozen=True)
class BodyFootprint:
    """Body rectangle in base_link: x from -rear_overhang_m to wheelbase_m + front_overhang_m,
    |y| up to width_m / 2."""

    rear_overhang_m: float
    front_overhang_m: float
    wheelbase_m: float
    width_m: float

    @property
    def x_min_m(self) -> float:
        return -self.rear_overhang_m

    @property
    def x_max_m(self) -> float:
        return self.wheelbase_m + self.front_overhang_m

    @property
    def half_width_m(self) -> float:
        return self.width_m / 2.0


@dataclass(frozen=True)
class LiveVehicle:
    """What a bundle is validated against, taken from the live vehicle_params."""

    mount: LidarMount
    body: BodyFootprint
    schema_version: str
    sysid_session_id: str


def from_generated(vp: Any) -> LiveVehicle:
    """Build a LiveVehicle from the generated VEHICLE_PARAMS object.

    Raises VehicleParamsNull if a LiDAR mount field is null: the caller must refuse to start.
    """
    lidar = vp.sensors.lidar
    mount_fields = {
        "sensors.lidar.mount_x_m": lidar.mount_x_m,
        "sensors.lidar.mount_y_m": lidar.mount_y_m,
        "sensors.lidar.mount_yaw_rad": lidar.mount_yaw_rad,
    }
    body_fields = {
        "chassis.rear_overhang_m": vp.chassis.rear_overhang_m,
        "chassis.front_overhang_m": vp.chassis.front_overhang_m,
        "chassis.wheelbase_m": vp.chassis.wheelbase_m,
        "chassis.width_m": vp.chassis.width_m,
    }
    for name, value in {**mount_fields, **body_fields}.items():
        if value is None:
            raise VehicleParamsNull(f"vehicle_params {name} is null; refusing to continue")
    return LiveVehicle(
        mount=LidarMount(
            x_m=float(lidar.mount_x_m),
            y_m=float(lidar.mount_y_m),
            yaw_rad=float(lidar.mount_yaw_rad),
        ),
        body=BodyFootprint(
            rear_overhang_m=float(vp.chassis.rear_overhang_m),
            front_overhang_m=float(vp.chassis.front_overhang_m),
            wheelbase_m=float(vp.chassis.wheelbase_m),
            width_m=float(vp.chassis.width_m),
        ),
        schema_version=str(vp.meta.schema_version),
        sysid_session_id=str(vp.meta.sysid_session_id),
    )


def find_repo_root(start: Path | None = None) -> Path:
    """The repository root: RACER_REPO_ROOT, else the nearest ancestor of this file or of the
    working directory (or of `start`) that holds config/vehicle_params.yaml."""
    env = os.environ.get("RACER_REPO_ROOT")
    if env and (Path(env) / _REPO_MARKER).is_file():
        return Path(env)
    starts = [start] if start is not None else [Path(__file__).resolve(), Path.cwd()]
    for s in starts:
        for parent in [s, *s.parents]:
            if (parent / _REPO_MARKER).is_file():
                return parent
    raise FileNotFoundError(
        "cannot find the repo root (config/vehicle_params.yaml); set RACER_REPO_ROOT"
    )


def load_generated_params(repo_root: Path | None = None) -> Any:
    """Regenerate the Python vehicle_params binding from the committed yaml (tools/gen_params.py)
    and return its VEHICLE_PARAMS instance. Refuses (raises) if the yaml fails its schema."""
    root = repo_root if repo_root is not None else find_repo_root()
    gen_path = root / "tools" / "gen_params.py"
    spec = importlib.util.spec_from_file_location("racer_track_perception._gen_params", gen_path)
    if spec is None or spec.loader is None:  # pragma: no cover - defensive
        raise ImportError(f"cannot load {gen_path}")
    gen = importlib.util.module_from_spec(spec)
    sys.modules[spec.name] = gen
    spec.loader.exec_module(gen)
    params = gen.load_yaml_file(root / "config" / "vehicle_params.yaml")
    schema = gen.load_schema_file(root / "config" / "vehicle_params.schema.json")
    gen.validate_params(params, schema)
    source = gen.generate_python(params, schema)
    module = types.ModuleType("racer_track_perception._vehicle_params_generated")
    module.__file__ = "<generated by tools/gen_params.py, not committed>"
    sys.modules[module.__name__] = module
    exec(compile(source, module.__file__, "exec"), module.__dict__)  # noqa: S102
    return module.VEHICLE_PARAMS


def load_live_vehicle(repo_root: Path | None = None) -> LiveVehicle:
    """The live vehicle_params as a LiveVehicle, for offline code."""
    return from_generated(load_generated_params(repo_root))
