"""L3: scan_deskew_node refuses to start without the two parameters M-DSK chooses, or with a
value that is not one of the choices (needs rclpy; colcon only). Each case starts the real node
in a subprocess; the node refuses before it creates a publisher, so there is nothing to shut
down."""

from __future__ import annotations

import subprocess
import sys

import pytest

pytest.importorskip("rclpy")
pytest.importorskip("racer_tools.vehicle_params_loader")

EXIT_REFUSED = 2


def run_node(*params: str) -> subprocess.CompletedProcess[str]:
    cmd = [sys.executable, "-m", "racer_track_perception.scan_deskew_node", "--ros-args"]
    for p in params:
        cmd += ["-p", p]
    return subprocess.run(cmd, capture_output=True, text=True, timeout=60, check=False)


def refused(proc: subprocess.CompletedProcess[str], name: str) -> None:
    assert proc.returncode == EXIT_REFUSED, (proc.returncode, proc.stderr[-800:])
    assert name in proc.stderr, proc.stderr[-800:]
    assert "refusing to start" in proc.stderr


def test_refuses_to_start_without_stamp_is() -> None:
    refused(run_node("beam_order:=index_ascending"), "stamp_is")


def test_refuses_to_start_without_beam_order() -> None:
    refused(run_node("stamp_is:=sweep_start"), "beam_order")


def test_refuses_to_start_with_nothing() -> None:
    refused(run_node(), "stamp_is")


@pytest.mark.parametrize(
    ("params", "name"),
    [
        (("stamp_is:=middle", "beam_order:=index_ascending"), "stamp_is"),
        (("stamp_is:=sweep_start", "beam_order:=sideways"), "beam_order"),
        (("stamp_is:=sweep_start", "beam_order:=index_ascending", "reference:=noon"), "reference"),
    ],
)
def test_refuses_an_unknown_value(params, name) -> None:
    refused(run_node(*params), name)
