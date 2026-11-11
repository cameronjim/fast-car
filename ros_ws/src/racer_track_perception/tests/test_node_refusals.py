"""L3: startup refusals exit non-zero with the specific error (needs rclpy; colcon only).

Each case starts the real node in a subprocess and checks the exit code and the named error. The
node is refused before it creates a publisher, so there is nothing to shut down.
"""

from __future__ import annotations

import subprocess
import sys
from pathlib import Path
from typing import Any

import pytest

pytest.importorskip("rclpy")
pytest.importorskip("racer_pnp_msgs")
pytest.importorskip("racer_tools.vehicle_params_loader")

from bundle_fixture import write_bundle
from racer_track_perception.vehicle import load_live_vehicle

EXIT_REFUSED = 2


def run_node(*params: str) -> subprocess.CompletedProcess[str]:
    cmd = [sys.executable, "-m", "racer_track_perception.track_perception_node", "--ros-args"]
    for p in params:
        cmd += ["-p", p]
    return subprocess.run(cmd, capture_output=True, text=True, timeout=60, check=False)


def refused(proc: subprocess.CompletedProcess[str], name: str) -> None:
    assert proc.returncode == EXIT_REFUSED, (proc.returncode, proc.stderr[-800:])
    assert name in proc.stderr, proc.stderr[-800:]
    assert "refusing to start" in proc.stderr


def test_bad_checksum_exits_non_zero(tmp_path: Path) -> None:
    b = write_bundle(tmp_path / "b", load_live_vehicle())
    data = (b / "weights.npz").read_bytes()
    (b / "weights.npz").write_bytes(data[:-1] + bytes([data[-1] ^ 1]))
    refused(run_node("backend:=net", f"bundle_dir:={b}"), "ChecksumMismatch")


def test_vehicle_params_mismatch_exits_non_zero(tmp_path: Path) -> None:
    def other_session(m: dict[str, Any]) -> None:
        m["vehicle_params"]["sysid_session_id"] = "sysid-not-the-live-one"

    b = write_bundle(tmp_path / "b", load_live_vehicle(), other_session)
    refused(run_node("backend:=net", f"bundle_dir:={b}"), "VehicleParamsMismatch")


def test_missing_manifest_exits_non_zero(tmp_path: Path) -> None:
    (tmp_path / "empty").mkdir()
    refused(run_node("backend:=net", f"bundle_dir:={tmp_path / 'empty'}"), "ManifestMissing")


def test_net_without_a_bundle_dir_exits_non_zero() -> None:
    refused(run_node("backend:=net"), "bundle_dir")


def test_unknown_backend_exits_non_zero() -> None:
    refused(run_node("backend:=magic"), "backend")


def test_disagreeing_yaw_with_vehicle_params_exits_non_zero() -> None:
    refused(run_node("backend:=geometric", "laser_yaw_offset_rad:=1.0"), "laser_yaw_offset_rad")
