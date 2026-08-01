"""Tests for `racer_policy.model.load_model`.

`torch` is a deploy-time-only, lazily-imported dependency (see that module's docstring) --
it is deliberately NOT installed in this package's test environment, so
`test_raises_when_torch_is_not_installed` below exercises the real ImportError path, not a
simulated one. The success path is exercised by injecting a minimal fake module into
`sys.modules["torch"]` (the standard trick for testing a lazy `import x` without installing
`x`): Python's `import` statement, when the name is already present in `sys.modules`, binds
it directly without touching the real import machinery, so this needs no real torch
installed to prove `load_model` calls `torch.jit.load` with the right path.
"""

from __future__ import annotations

import sys
import types
from pathlib import Path
from typing import Any

import pytest
from racer_policy.contract import load_contract
from racer_policy.errors import MissingTorchDependencyError
from racer_policy.model import load_model


def test_raises_when_torch_is_not_installed(valid_contract_dir: Path) -> None:
    assert "torch" not in sys.modules, (
        "torch must not be installed/imported in this test environment for this test to "
        "prove anything (claude-docs/08-learning.md: torch is deploy-time-only)"
    )
    contract = load_contract(valid_contract_dir)

    with pytest.raises(MissingTorchDependencyError, match="torch is required"):
        load_model(contract)


def test_loads_via_torch_jit_load_when_torch_is_available(
    valid_contract_dir: Path, monkeypatch: pytest.MonkeyPatch
) -> None:
    contract = load_contract(valid_contract_dir)
    calls: list[tuple[str, Any]] = []

    fake_torch = types.SimpleNamespace(
        jit=types.SimpleNamespace(
            load=lambda path, map_location=None: (
                calls.append((path, map_location)) or "sentinel-model"
            )
        )
    )
    monkeypatch.setitem(sys.modules, "torch", fake_torch)

    model = load_model(contract)

    assert model == "sentinel-model"
    assert calls == [(str(contract.policy_path), "cpu")]
