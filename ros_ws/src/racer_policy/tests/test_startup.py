"""Startup refusals of policy_node (everything `build_core` does before the node subscribes to
anything), plus the pure helpers. Torch-free where possible; the tests that load a real
TorchScript module are skipped without torch (they run in the car image and nightly)."""

from __future__ import annotations

import importlib.util
import math
from pathlib import Path
from typing import Any

import numpy as np
import pytest
from policy_fixture import build_policy_dir, torch_policy_bytes
from racer_policy import check
from racer_policy.startup import (
    StartupRefusal,
    build_core,
    clamp_for_float32,
    yaw_from_quaternion,
)

LINE_CSV = (
    "s_m,x_m,y_m,heading_rad,curvature_1pm,target_speed_mps\n"
    "0,0,0,0,0,1\n1,1,0,0,0,1\n2,1,1,1.5,1,1\n3,0,1,3.1,1,1\n"
)


@pytest.fixture
def raceline_csv(tmp_path: Path) -> str:
    p = tmp_path / "line.csv"
    p.write_text(LINE_CSV, encoding="utf-8")
    return str(p)


def test_refuses_without_a_policy_dir_or_raceline(raceline_csv: str, real_vehicle_params: Any):
    with pytest.raises(StartupRefusal, match="policy_dir"):
        build_core("", raceline_csv, real_vehicle_params, 0.5)
    with pytest.raises(StartupRefusal, match="raceline_path"):
        build_core("/x", "", real_vehicle_params, 0.5)


def test_refuses_a_missing_directory(tmp_path: Path, raceline_csv: str, real_vehicle_params: Any):
    with pytest.raises(StartupRefusal):
        build_core(str(tmp_path / "nope"), raceline_csv, real_vehicle_params, 0.5)


def test_refuses_a_contract_mismatch_with_the_specific_reason(
    tmp_path: Path, raceline_csv: str, real_vehicle_params: Any
):
    d = build_policy_dir(
        tmp_path,
        real_vehicle_params,
        mutate_contract=lambda c: c["vehicle_params"].update(schema_version="0.13.0"),
    )
    with pytest.raises(StartupRefusal, match="VehicleParamsVersionMismatchError.*0.13.0"):
        build_core(str(d), raceline_csv, real_vehicle_params, 0.5)


def test_refuses_a_bad_raceline(tmp_path: Path, real_vehicle_params: Any):
    d = build_policy_dir(tmp_path, real_vehicle_params)
    with pytest.raises(StartupRefusal, match="raceline"):
        build_core(str(d), str(tmp_path / "missing.csv"), real_vehicle_params, 0.5)


def test_refuses_when_the_model_cannot_load(
    tmp_path: Path, raceline_csv: str, real_vehicle_params: Any
):
    """Fixture bytes are not a TorchScript module: without torch that is the missing
    dependency, with torch it is a load failure. Either way the node refuses."""
    d = build_policy_dir(tmp_path, real_vehicle_params)
    with pytest.raises(StartupRefusal):
        build_core(str(d), raceline_csv, real_vehicle_params, 0.5)


def test_check_cli_accepts_and_refuses(
    tmp_path: Path, real_vehicle_params: Any, monkeypatch: pytest.MonkeyPatch, capsys: Any
):
    good = build_policy_dir(tmp_path, real_vehicle_params, name="good")
    bad = build_policy_dir(
        tmp_path,
        real_vehicle_params,
        name="bad",
        mutate_contract=lambda c: c["vehicle_params"].update(schema_version="0.13.0"),
    )
    import sys
    import types

    fake = types.ModuleType("racer_tools.vehicle_params_loader")
    fake.load_vehicle_params = lambda: real_vehicle_params  # type: ignore[attr-defined]
    monkeypatch.setitem(sys.modules, "racer_tools", types.ModuleType("racer_tools"))
    monkeypatch.setitem(sys.modules, "racer_tools.vehicle_params_loader", fake)
    assert check.main([str(good)]) == 0
    assert "OK" in capsys.readouterr().out
    assert check.main([str(bad)]) == 1
    assert "REFUSED VehicleParamsVersionMismatchError" in capsys.readouterr().err
    assert check.main([]) == 2
    assert check.main([str(tmp_path / "nowhere")]) == 1


def test_clamp_for_float32_never_lands_outside_the_bound():
    high = 0.449
    for value in (0.449, 0.4490000001, 0.5, 100.0):
        out = clamp_for_float32(value, -high, high)
        assert out <= high and out >= -high
        assert float(np.float32(out)) == out  # exactly representable on the wire
    assert clamp_for_float32(-9.0, -high, high) >= -high
    assert clamp_for_float32(0.1, -high, high) == pytest.approx(0.1, abs=1e-7)


def test_yaw_from_quaternion_known_answers():
    assert yaw_from_quaternion(1.0, 0.0, 0.0, 0.0) == 0.0
    s = math.sin(math.pi / 4.0)
    assert yaw_from_quaternion(s, 0.0, 0.0, s) == pytest.approx(math.pi / 2.0)


# -- with torch --------------------------------------------------------------------------------

needs_torch = pytest.mark.skipif(
    importlib.util.find_spec("torch") is None, reason="needs torch (car image, nightly)"
)


@needs_torch
def test_a_real_module_loads_and_probes(
    tmp_path: Path, raceline_csv: str, real_vehicle_params: Any
):
    d = build_policy_dir(tmp_path, real_vehicle_params, policy_bytes=torch_policy_bytes(1.0))
    core, deployment = build_core(str(d), raceline_csv, real_vehicle_params, 0.5)
    out = core.model(np.zeros(47, dtype=np.float32))
    assert out.shape == (2,) and np.all(np.abs(out) <= 1.0)
    assert deployment.contract.policy.filename == "policy.pt"


@needs_torch
def test_refuses_a_module_that_rejects_the_observation_width(
    tmp_path: Path, raceline_csv: str, real_vehicle_params: Any
):
    import io

    import torch

    class Wrong(torch.nn.Module):
        def __init__(self) -> None:
            super().__init__()
            self.lin = torch.nn.Linear(5, 2)

        def forward(self, obs: torch.Tensor) -> torch.Tensor:
            return self.lin(obs)

    buf = io.BytesIO()
    torch.jit.save(torch.jit.script(Wrong().eval()), buf)
    d = build_policy_dir(tmp_path, real_vehicle_params, policy_bytes=buf.getvalue())
    with pytest.raises(StartupRefusal, match="rejected a 47-wide observation"):
        build_core(str(d), raceline_csv, real_vehicle_params, 0.5)


@needs_torch
def test_refuses_a_module_with_the_wrong_output_shape(
    tmp_path: Path, raceline_csv: str, real_vehicle_params: Any
):
    import io

    import torch

    class Three(torch.nn.Module):
        def __init__(self) -> None:
            super().__init__()
            self.lin = torch.nn.Linear(47, 3)

        def forward(self, obs: torch.Tensor) -> torch.Tensor:
            return self.lin(obs)

    buf = io.BytesIO()
    torch.jit.save(torch.jit.script(Three().eval()), buf)
    d = build_policy_dir(tmp_path, real_vehicle_params, policy_bytes=buf.getvalue())
    with pytest.raises(StartupRefusal, match="expected"):
        build_core(str(d), raceline_csv, real_vehicle_params, 0.5)
