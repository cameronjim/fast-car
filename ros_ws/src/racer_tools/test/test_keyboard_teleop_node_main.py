"""Regression test for docs/notes/first-boot-audit-2026-09-13.md finding #6:
`keyboard_teleop_node.main`'s `finally` block used to reference `node` even when
`KeyboardTeleopNode()` itself raised, before `node` was ever assigned. That produced a
`NameError` on cleanup which masked the real constructor exception (e.g. a
vehicle_params_loader failure) -- the thing an operator actually needed to see at first
boot.

Same pytest.importorskip-guarded shape as test_raceline_publisher_node_launch.py: skipped
cleanly under the bare `uv run pytest` L1 run (no rclpy there), runs for real under
`colcon test` in the ros-dev image. No TTY is needed since the constructor is made to fail
before `raw_terminal_mode` is ever entered, so this stays a minimal rclpy-only test rather
than a full L3 launch test.
"""

from __future__ import annotations

import pytest

pytest.importorskip("rclpy")

import rclpy
from racer_tools import keyboard_teleop_node


class _VehicleParamsLoaderFailure(RuntimeError):
    """Stands in for a real constructor failure, e.g. vehicle_params_loader not finding the
    repo root."""


def test_main_surfaces_the_original_constructor_exception_not_a_nameerror(monkeypatch):
    def _raise_on_construct(*args, **kwargs):
        raise _VehicleParamsLoaderFailure("could not locate config/vehicle_params.yaml")

    monkeypatch.setattr(keyboard_teleop_node, "KeyboardTeleopNode", _raise_on_construct)

    # Guard against a leftover context from a previous test/process; main() calls its own
    # rclpy.init()/rclpy.shutdown() internally.
    if rclpy.ok():
        rclpy.shutdown()

    with pytest.raises(_VehicleParamsLoaderFailure):
        keyboard_teleop_node.main(args=[])

    # The finally block's cleanup must still have run (context shut down), and must not have
    # raised its own NameError/AttributeError on top of the original exception.
    assert not rclpy.ok()
