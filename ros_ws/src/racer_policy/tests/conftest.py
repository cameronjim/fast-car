"""Shared fixtures for racer_policy tests.

`_TEMPLATE` is one fully-valid, schema-satisfying `contract.yaml` body. Every test that
needs a "mostly valid, one thing wrong" manifest starts from `copy.deepcopy(_TEMPLATE)` (via
the `valid_manifest` fixture) and mutates exactly the field(s) under test -- this is what
makes the mismatch tests in test_verify_against_environment.py and the property test in
test_manifest_property.py isolate a single variable at a time.
"""

from __future__ import annotations

import copy
import hashlib
import subprocess
import sys
from pathlib import Path
from typing import Any

import pytest
import yaml
from racer_policy.contract import ObservationField
from racer_policy.environment import LiveEnvironment, LiveObservationSchema

# tests/conftest.py -> tests/ -> racer_policy/ -> src/ -> ros_ws/ -> repo root.
REPO_ROOT = Path(__file__).resolve().parents[4]
GEN_PARAMS_SCRIPT = REPO_ROOT / "tools" / "gen_params.py"

DEFAULT_POLICY_BYTES = b"not-a-real-torchscript-model, just fixture bytes for hashing tests"

_TEMPLATE: dict[str, Any] = {
    "contract_version": "1.0.0",
    "policy": {
        "filename": "policy.pt",
        # Filled in by write_contract_dir() to match whatever bytes are actually written;
        # the placeholder here is never used as-is.
        "checksum_sha256": "0" * 64,
    },
    "observation_schema": {
        "fields": [
            {"name": "velocity_mps", "dtype": "float32", "units": "m/s"},
            {"name": "yaw_rate_rad_s", "dtype": "float32", "units": "rad/s"},
            {"name": "slip_proxy", "dtype": "float32", "units": "dimensionless"},
            {"name": "raceline_lateral_error_m", "dtype": "float32", "units": "m"},
            {"name": "raceline_heading_error_rad", "dtype": "float32", "units": "rad"},
        ],
        "lidar": {"beam_count": 108, "fov_rad": 4.712, "downsample_factor": 10},
    },
    "normalization": {
        "mean": [0.0] * (5 + 108),
        "std": [1.0] * (5 + 108),
    },
    "action_space": {
        "fields": [
            {"name": "steering_residual_rad", "low": -0.1, "high": 0.1, "units": "rad"},
            {"name": "speed_residual_mps", "low": -1.0, "high": 1.0, "units": "m/s"},
        ],
        "scaling": "linear",
        "residual_limits": {"steering_fraction": 0.2, "speed_fraction": 0.15},
    },
    "actuator_assumptions": {
        "rate_limit_steering_rad_s": 3.2,
        "rate_limit_speed_mps_s": 9.51,
        "command_delay_s": 0.02,
    },
    "vehicle_params": {
        # Must match config/vehicle_params.yaml's meta.schema_version exactly (see
        # test_vehicle_params_wiring.py, which checks this fixture against the real,
        # freshly-regenerated committed file). Bumped 0.1.0 -> 0.2.0 with that file for
        # milestone 4 (hardware-arrival prep, roadmap task 1.3).
        "schema_version": "0.2.0",
        "sysid_session_id": "none-preliminary",
    },
    "training": {
        "config_hash": hashlib.sha256(b"fixture-training-config").hexdigest(),
        "git_sha": "abc1234",
    },
}


def fresh_valid_manifest() -> dict[str, Any]:
    """A fresh, deep-copied, fully schema-valid manifest dict. Mutate freely.

    Plain function (not a fixture) so hypothesis-driven tests can call it once per example
    to get an independent dict -- a pytest fixture value is computed once per test *node*,
    not once per hypothesis example, so mutating a fixture-provided dict in place would leak
    across examples within the same `@given` run.
    """
    return copy.deepcopy(_TEMPLATE)


@pytest.fixture
def valid_manifest() -> dict[str, Any]:
    """A fresh, deep-copied, fully schema-valid manifest dict. Mutate freely."""
    return fresh_valid_manifest()


def write_contract_dir(
    tmp_path: Path,
    manifest: dict[str, Any],
    *,
    policy_bytes: bytes = DEFAULT_POLICY_BYTES,
    checksum_override: str | None = None,
) -> Path:
    """Write `manifest` (mutated in place: policy.checksum_sha256 is set/overridden) plus a
    `policy.*` artifact containing `policy_bytes` into a fresh directory, and return it."""
    directory = tmp_path / "contract"
    directory.mkdir()
    (directory / manifest["policy"]["filename"]).write_bytes(policy_bytes)
    manifest["policy"]["checksum_sha256"] = (
        checksum_override
        if checksum_override is not None
        else hashlib.sha256(policy_bytes).hexdigest()
    )
    (directory / "contract.yaml").write_text(
        yaml.safe_dump(manifest, sort_keys=False), encoding="utf-8"
    )
    return directory


@pytest.fixture
def valid_contract_dir(tmp_path: Path, valid_manifest: dict[str, Any]) -> Path:
    return write_contract_dir(tmp_path, valid_manifest)


def live_environment_matching(manifest: dict[str, Any]) -> LiveEnvironment:
    """Build a `LiveEnvironment` that matches `manifest` field-for-field -- the "everything
    agrees" baseline every mismatch test starts from and then perturbs exactly one field of.
    """
    obs = manifest["observation_schema"]
    fields = tuple(ObservationField(**f) for f in obs["fields"])
    lidar = obs["lidar"]

    class _Meta:
        def __init__(self, schema_version: str, sysid_session_id: str) -> None:
            self.schema_version = schema_version
            self.sysid_session_id = sysid_session_id

    class _VehicleParams:
        def __init__(self, meta: _Meta) -> None:
            self.meta = meta

    vp = manifest["vehicle_params"]
    return LiveEnvironment(
        vehicle_params=_VehicleParams(_Meta(vp["schema_version"], vp["sysid_session_id"])),
        observation=LiveObservationSchema(
            fields=fields,
            lidar_beam_count=lidar["beam_count"],
            lidar_fov_rad=lidar["fov_rad"],
            lidar_downsample_factor=lidar["downsample_factor"],
        ),
    )


@pytest.fixture
def matching_live_environment(valid_manifest: dict[str, Any]) -> LiveEnvironment:
    return live_environment_matching(valid_manifest)


@pytest.fixture
def real_vehicle_params(tmp_path: Path) -> Any:
    """Regenerate the Python vehicle_params binding from the committed
    config/vehicle_params.yaml via tools/gen_params.py (claude-docs/06-vehicle-params.md
    rule 3: bindings are generated, never hand-written, never committed) and return its
    VEHICLE_PARAMS instance. Mirrors training/envelope/tests/conftest.py's fixture of the
    same name and purpose.
    """
    out_dir = tmp_path / "generated"
    subprocess.run(
        [sys.executable, str(GEN_PARAMS_SCRIPT), "--out-dir", str(out_dir)],
        cwd=REPO_ROOT,
        check=True,
        capture_output=True,
        text=True,
    )
    generated_path = out_dir / "vehicle_params_generated.py"
    source = generated_path.read_text(encoding="utf-8")
    namespace: dict[str, Any] = {}
    # Deliberate: executes gen_params.py's own freshly-regenerated output (never committed,
    # never touched by anything untrusted) purely so the test doesn't need a second
    # sys.path-manipulation mechanism just to import a tmp_path module.
    exec(compile(source, str(generated_path), "exec"), namespace)  # noqa: S102
    return namespace["VEHICLE_PARAMS"]
