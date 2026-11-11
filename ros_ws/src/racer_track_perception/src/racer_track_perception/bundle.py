"""Load and verify a track_perception model bundle, refusing on any mismatch.

A bundle is a directory holding manifest.yaml and the payload files it lists. `load_bundle`
validates the manifest against config/pnp/bundle.schema.json, then checks, in this order, the
schema major version, the kind, every payload file (present, SHA-256 equal), the vehicle_params
identity (exact string match) and the io contract (every value equal to what this code
implements). Each failure raises its own `BundleRefused` subclass (errors.py); nothing is ever
downgraded to a warning.
"""

from __future__ import annotations

import hashlib
import json
import math
import os
from dataclasses import dataclass
from pathlib import Path
from typing import Any

import yaml
from jsonschema import Draft202012Validator

from racer_track_perception import preprocess
from racer_track_perception import track as tr
from racer_track_perception.errors import (
    BundleSchemaVersionMismatch,
    ChecksumMismatch,
    IoContractMismatch,
    KindMismatch,
    ManifestInvalid,
    ManifestMissing,
    PayloadMissing,
    VehicleParamsMismatch,
)
from racer_track_perception.vehicle import LiveVehicle, find_repo_root

SUPPORTED_SCHEMA_MAJOR = 1
KIND = "track_perception"
ARCH_TYPE = "circ_conv1d_v0"
MOUNT_TOLERANCE = 1e-6
WEIGHTS_FILE = "weights.npz"
SCHEMA_RELATIVE = Path("config") / "pnp" / "bundle.schema.json"


@dataclass(frozen=True)
class Bundle:
    path: Path
    manifest: dict[str, Any]

    @property
    def model_id(self) -> str:
        return str(self.manifest["model_id"])

    @property
    def weights_sha256(self) -> str:
        for entry in self.manifest["payload"]:
            if entry["file"] == WEIGHTS_FILE:
                return str(entry["sha256"])
        raise PayloadMissing(f"{WEIGHTS_FILE} is not listed in the payload")

    def payload_path(self, name: str) -> Path:
        return self.path / name


def _schema(schema_path: Path | None) -> dict[str, Any]:
    path = schema_path
    if path is None:
        env = os.environ.get("RACER_PNP_BUNDLE_SCHEMA")
        path = Path(env) if env else find_repo_root() / SCHEMA_RELATIVE
    try:
        loaded = json.loads(path.read_text(encoding="utf-8"))
    except (OSError, ValueError) as exc:
        raise ManifestInvalid(f"cannot read the bundle schema {path}: {exc}") from exc
    if not isinstance(loaded, dict):
        raise ManifestInvalid(f"the bundle schema {path} is not an object")
    return loaded


def expected_io(vehicle: LiveVehicle) -> dict[str, Any]:
    """The io block this code implements, with the lidar mount from the live vehicle_params."""
    return {
        "input": {
            "tensor": list(preprocess_tensor_shape()),
            "n_bins": preprocess.N_BINS,
            "range_clip_m": preprocess.RANGE_CLIP_M,
            "origin": "lidar",
            "pooling": "min",
            "body_filter_margin_m": preprocess.BODY_FILTER_MARGIN_M,
        },
        "output": {
            "tensor": [tr.K_STATIONS, 5],
            "stations": tr.K_STATIONS,
            "station_spacing_m": tr.STATION_SPACING_M,
            "frame": "base_link",
        },
        "lidar_mount": {
            "x_m": vehicle.mount.x_m,
            "y_m": vehicle.mount.y_m,
            "yaw_rad": vehicle.mount.yaw_rad,
        },
        "arch": {"type": ARCH_TYPE},
    }


def preprocess_tensor_shape() -> tuple[int, int, int]:
    return (1, 2, preprocess.N_BINS)


def _same(a: Any, b: Any, tol: float = 0.0) -> bool:
    if isinstance(a, bool) or isinstance(b, bool):
        return a is b
    if isinstance(a, (int, float)) and isinstance(b, (int, float)):
        return math.isfinite(a) and math.isfinite(b) and abs(a - b) <= tol
    return bool(a == b)


def _check_io(io: dict[str, Any], vehicle: LiveVehicle) -> None:
    want = expected_io(vehicle)
    for section in ("input", "output"):
        got_s = io.get(section, {})
        for key, value in want[section].items():
            got = got_s.get(key)
            ok = (
                (isinstance(got, list) and got == value)
                if isinstance(value, list)
                else _same(got, value, 1e-9)
            )
            if not ok:
                raise IoContractMismatch(
                    f"io.{section}.{key} is {got!r}, this node needs {value!r}"
                )
    for key, value in want["lidar_mount"].items():
        got = io.get("lidar_mount", {}).get(key)
        if not _same(got, value, MOUNT_TOLERANCE):
            raise IoContractMismatch(
                f"io.lidar_mount.{key} is {got!r}, the live vehicle_params has {value!r}"
            )
    arch = io.get("arch", {}).get("type")
    if arch != ARCH_TYPE:
        raise IoContractMismatch(f"io.arch.type is {arch!r}, this node implements {ARCH_TYPE!r}")


def _sha256(path: Path) -> str:
    h = hashlib.sha256()
    with path.open("rb") as f:
        for chunk in iter(lambda: f.read(1 << 20), b""):
            h.update(chunk)
    return h.hexdigest()


def load_bundle(
    path: str | Path,
    expected_kind: str,
    live_vehicle_params: LiveVehicle,
    schema_path: Path | None = None,
) -> Bundle:
    """Load the bundle at `path`, or raise the specific `BundleRefused` subclass."""
    root = Path(path)
    manifest_path = root / "manifest.yaml"
    try:
        manifest = yaml.safe_load(manifest_path.read_text(encoding="utf-8"))
    except (OSError, yaml.YAMLError, UnicodeDecodeError) as exc:
        raise ManifestMissing(f"no readable manifest.yaml in {root}: {exc}") from exc
    if not isinstance(manifest, dict):
        raise ManifestInvalid("manifest.yaml is not a mapping")

    errors = sorted(
        Draft202012Validator(_schema(schema_path)).iter_errors(manifest),
        key=lambda e: list(e.absolute_path),
    )
    if errors:
        first = errors[0]
        where = "/".join(str(p) for p in first.absolute_path) or "(top level)"
        raise ManifestInvalid(f"manifest.yaml fails the schema at {where}: {first.message}")

    major = int(str(manifest["bundle_schema_version"]).split(".")[0])
    if major != SUPPORTED_SCHEMA_MAJOR:
        raise BundleSchemaVersionMismatch(
            f"bundle_schema_version {manifest['bundle_schema_version']} has major {major}, "
            f"this loader reads major {SUPPORTED_SCHEMA_MAJOR}"
        )
    if manifest["kind"] != expected_kind:
        raise KindMismatch(
            f"bundle kind is {manifest['kind']!r}, this node deploys {expected_kind!r}"
        )

    listed = {entry["file"]: entry["sha256"] for entry in manifest["payload"]}
    if WEIGHTS_FILE not in listed:
        raise PayloadMissing(f"{WEIGHTS_FILE} is not listed in the manifest payload")
    for name, want_sha in listed.items():
        if Path(name).name != name:
            raise ManifestInvalid(f"payload file {name!r} must be a plain file name")
        file = root / name
        if not file.is_file():
            raise PayloadMissing(f"payload file {name} is not in {root}")
        got = _sha256(file)
        if got != want_sha:
            raise ChecksumMismatch(f"{name} has sha256 {got}, the manifest says {want_sha}")

    vp = manifest["vehicle_params"]
    for field, live in (
        ("schema_version", live_vehicle_params.schema_version),
        ("sysid_session_id", live_vehicle_params.sysid_session_id),
    ):
        if vp[field] != live:
            raise VehicleParamsMismatch(
                f"vehicle_params.{field} is {vp[field]!r} in the bundle, {live!r} live"
            )

    _check_io(manifest["io"], live_vehicle_params)
    return Bundle(path=root, manifest=manifest)
