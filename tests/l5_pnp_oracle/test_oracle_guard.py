"""The oracle track never reaches the car (brief 40): the bringup package and pnp.launch.py cannot
launch it, nothing under ros_ws or the car's deployment names it, and this package installs
nothing. Text checks run without ROS; the launch refusal runs where `launch` is importable."""

from __future__ import annotations

import importlib.util
import re
from pathlib import Path

import pytest

REPO = Path(__file__).resolve().parents[2]
HERE = Path(__file__).resolve().parent
PNP_LAUNCH = REPO / "ros_ws" / "src" / "racer_bringup" / "launch" / "pnp.launch.py"
NAMES = ("oracle_track_node", "l5_pnp_oracle", "oracle_core", "oracle-ground-truth")
CAR_SIDE_DIRS = ("ros_ws", "docker", "firmware", "config", "tools")
TEXT_SUFFIXES = {".py", ".launch", ".xml", ".yaml", ".yml", ".txt", ".cfg", ".toml", ".sh", ".cpp"}


def test_pnp_launch_has_no_oracle_backend_or_reference():
    text = PNP_LAUNCH.read_text()
    assert re.search(r"BACKENDS\s*=\s*\(\"net\",\s*\"geometric\"\)", text), (
        "the launch's backends are net and geometric only"
    )
    for name in NAMES:
        assert name not in text
    assert "oracle" not in text.lower()


def test_nothing_the_car_runs_names_the_oracle():
    hits = []
    for top in CAR_SIDE_DIRS:
        for path in (REPO / top).rglob("*"):
            if not (path.is_file() and path.suffix in TEXT_SUFFIXES and ".git" not in path.parts):
                continue
            if any(name in path.read_text(errors="ignore") for name in NAMES):
                hits.append(str(path.relative_to(REPO)))
    assert hits == [], f"the oracle track is test-only, but these name it: {hits}"


def test_bringup_package_does_not_install_or_depend_on_the_test_package():
    bringup = REPO / "ros_ws" / "src" / "racer_bringup"
    for name in ("package.xml", "setup.py", "CMakeLists.txt"):
        f = bringup / name
        if f.exists():
            assert "l5_pnp_oracle" not in f.read_text()


def test_the_oracle_package_installs_nothing():
    cmake = (HERE / "CMakeLists.txt").read_text()
    assert "install(" not in cmake.lower().replace("ament_package", "")
    assert not (HERE / "setup.py").exists()


def test_the_oracle_node_says_it_is_test_only():
    assert "TEST ONLY" in (HERE / "oracle_track_node.py").read_text()
    assert "TEST ONLY" in (HERE / "oracle_core.py").read_text()


def test_launch_refuses_an_oracle_backend():
    if importlib.util.find_spec("launch") is None or importlib.util.find_spec("launch_ros") is None:
        pytest.skip("launch is not importable here")
    spec = importlib.util.spec_from_file_location("pnp_launch_guard", PNP_LAUNCH)
    module = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(module)
    with pytest.raises(RuntimeError, match="backend must be one of"):
        module.parameters_for("", "/some/dynamics", "oracle", 0.5, True)
    assert "oracle" not in module.BACKENDS
