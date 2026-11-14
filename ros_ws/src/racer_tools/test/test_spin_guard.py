"""L1 test of racer_tools.spin_guard.spin_until_shutdown (GitHub issue 50), with a fake rclpy."""

from __future__ import annotations

import sys
import types

import pytest
from racer_tools.spin_guard import spin_until_shutdown


def _fake_rclpy(monkeypatch, *, ok: bool, raises: BaseException | None):
    calls: list[object] = []
    fake = types.ModuleType("rclpy")

    def spin(node):
        calls.append(node)
        if raises is not None:
            raise raises

    fake.spin = spin
    fake.ok = lambda: ok
    monkeypatch.setitem(sys.modules, "rclpy", fake)
    return calls


def test_clean_spin_returns(monkeypatch):
    calls = _fake_rclpy(monkeypatch, ok=True, raises=None)
    spin_until_shutdown("node")
    assert calls == ["node"]


def test_runtime_error_with_context_down_is_swallowed(monkeypatch):
    calls = _fake_rclpy(
        monkeypatch,
        ok=False,
        raises=RuntimeError("Unable to convert call argument to Python object"),
    )
    spin_until_shutdown("node")  # must not raise: the node exits 0
    assert calls == ["node"]


def test_runtime_error_with_context_up_is_reraised(monkeypatch):
    _fake_rclpy(monkeypatch, ok=True, raises=RuntimeError("real failure"))
    with pytest.raises(RuntimeError, match="real failure"):
        spin_until_shutdown("node")


def test_other_exceptions_propagate_even_with_context_down(monkeypatch):
    _fake_rclpy(monkeypatch, ok=False, raises=KeyboardInterrupt())
    with pytest.raises(KeyboardInterrupt):
        spin_until_shutdown("node")
