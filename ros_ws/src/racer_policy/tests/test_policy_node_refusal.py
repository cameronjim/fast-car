"""policy_node refuses to start, with the specific reason and a non-zero exit, on every
deployment mismatch (CLAUDE.md invariant 3), before it subscribes to anything.

Each case starts the real node as a process (`python3 -m racer_policy.policy_node`) against a
policy directory with exactly one thing wrong. Needs rclpy and the colcon-built racer_tools (for
the generated vehicle_params binding), so it skips under the bare `uv run pytest` and runs in
the ros-dev image under `colcon test`. Needs no torch: every refusal here happens before the
model is loaded.
"""

from __future__ import annotations

import os
import subprocess
import sys
from pathlib import Path
from typing import Any

import pytest
from policy_fixture import build_policy_dir

LINE_CSV = "s_m,x_m,y_m,heading_rad,curvature_1pm,target_speed_mps\n0,0,0,0,0,1\n1,1,0,0,0,1\n2,1,1,1.5,1,1\n"


@pytest.fixture(scope="module")
def vehicle_params() -> Any:
    from racer_tools.vehicle_params_loader import load_vehicle_params

    return load_vehicle_params()


@pytest.fixture
def raceline(tmp_path: Path) -> str:
    p = tmp_path / "line.csv"
    p.write_text(LINE_CSV, encoding="utf-8")
    return str(p)


def _run_node(*params: str) -> subprocess.CompletedProcess[str]:
    env = dict(os.environ)
    env["ROS_DOMAIN_ID"] = "96"
    args = [sys.executable, "-m", "racer_policy.policy_node", "--ros-args"]
    for p in params:
        args += ["-p", p]
    return subprocess.run(args, env=env, capture_output=True, text=True, timeout=60, check=False)


def _assert_refused(proc: subprocess.CompletedProcess[str], reason: str) -> None:
    assert proc.returncode == 2, proc.stdout[-800:] + proc.stderr[-800:]
    assert "REFUSING TO START" in proc.stderr
    assert reason in proc.stderr, proc.stderr[-800:]


def test_refuses_without_a_policy_dir(raceline: str) -> None:
    _assert_refused(_run_node(f"raceline_path:={raceline}"), "policy_dir")


def test_refuses_a_policy_trained_against_another_vehicle_params_version(
    tmp_path: Path, vehicle_params: Any, raceline: str
) -> None:
    d = build_policy_dir(
        tmp_path,
        vehicle_params,
        mutate_contract=lambda c: c["vehicle_params"].update(schema_version="0.13.0"),
    )
    _assert_refused(
        _run_node(f"policy_dir:={d}", f"raceline_path:={raceline}"),
        "VehicleParamsVersionMismatchError",
    )


def test_refuses_a_flipped_policy_byte(tmp_path: Path, vehicle_params: Any, raceline: str) -> None:
    d = build_policy_dir(tmp_path, vehicle_params)
    (d / "policy.pt").write_bytes(b"x" + (d / "policy.pt").read_bytes()[1:])
    _assert_refused(
        _run_node(f"policy_dir:={d}", f"raceline_path:={raceline}"), "ChecksumMismatchError"
    )


def test_refuses_a_missing_sidecar(tmp_path: Path, vehicle_params: Any, raceline: str) -> None:
    d = build_policy_dir(tmp_path, vehicle_params, write_meta=False)
    _assert_refused(_run_node(f"policy_dir:={d}", f"raceline_path:={raceline}"), "PolicyMetaError")


def test_refuses_a_swapped_observation_field_order(
    tmp_path: Path, vehicle_params: Any, raceline: str
) -> None:
    def swap(c: dict[str, Any]) -> None:
        f = c["observation_schema"]["fields"]
        f[3], f[4] = f[4], f[3]

    d = build_policy_dir(tmp_path, vehicle_params, mutate_contract=swap)
    _assert_refused(
        _run_node(f"policy_dir:={d}", f"raceline_path:={raceline}"),
        "ObservationSchemaMismatchError",
    )


def test_refuses_a_missing_raceline(tmp_path: Path, vehicle_params: Any) -> None:
    d = build_policy_dir(tmp_path, vehicle_params)
    _assert_refused(
        _run_node(f"policy_dir:={d}", f"raceline_path:={tmp_path / 'nope.csv'}"), "raceline"
    )


def test_there_is_no_parameter_that_skips_a_check(
    tmp_path: Path, vehicle_params: Any, raceline: str
) -> None:
    """Undeclared parameters are ignored by rclpy, so an 'ignore_mismatch' style override
    cannot exist; prove the refusal stands when someone tries."""
    d = build_policy_dir(
        tmp_path,
        vehicle_params,
        mutate_contract=lambda c: c["vehicle_params"].update(sysid_session_id="other"),
    )
    proc = _run_node(
        f"policy_dir:={d}",
        f"raceline_path:={raceline}",
        "ignore_mismatch:=true",
        "allow_version_mismatch:=true",
        "strict:=false",
    )
    _assert_refused(proc, "VehicleParamsVersionMismatchError")
