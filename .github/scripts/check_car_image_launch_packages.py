#!/usr/bin/env python3
"""Keep docker/car's apt package list in sync with car_teleop.launch.py's ON-BY-DEFAULT nodes.

Regression guard for the bug found during the first real Jetson bring-up on 2026-09-21
(docs/notes/build-log.md): `car_teleop.launch.py` declared `viz` with `default_value="true"`
and started `foxglove_bridge` under it, while `docker/car/Dockerfile` deliberately excluded
`ros-humble-foxglove-bridge` as "dev visualization tooling". The result was that the one
launch file the car image exists to run could not be launched with its own defaults inside
that image: `package 'foxglove_bridge' not found`.

Neither side was wrong in isolation, which is exactly why nothing caught it -- the launch
file's L3 test runs in `ros-dev` (which HAS the bridge) and passes `viz:=false`, and CI never
builds the car image at all (no arm64/L4T runner -- see the `docker-car-lint` job's comment).
This check is the cheap static link between the two files: if a node in `car_teleop.launch.py`
is gated on a launch argument that DEFAULTS TO TRUE, the package providing it must be
installed in the car image.

Run by the `docker-car-lint` CI job. No Docker, no ROS, no third-party imports.
"""

from __future__ import annotations

import ast
import pathlib
import re
import sys

REPO_ROOT = pathlib.Path(__file__).resolve().parents[2]
LAUNCH_FILE = REPO_ROOT / "ros_ws/src/racer_bringup/launch/car_teleop.launch.py"
DOCKERFILE = REPO_ROOT / "docker/car/Dockerfile"

# ROS package name -> the apt package that provides it in the car image. Only packages that
# come from the ROS apt repo need an entry; packages built from ros_ws/src (racer_*) are
# compiled on-device and are deliberately absent here.
APT_PACKAGE_FOR = {
    "foxglove_bridge": "ros-humble-foxglove-bridge",
}

# Same idea for the rosbag2 recorder car_teleop.launch.py starts when `record` defaults to
# true (CLAUDE.md invariant 5, roadmap 1.6). The recorder is an ExecuteProcess, not a Node, so
# the loop below cannot see it, and its `bag_storage:=auto` FALLS BACK to sqlite3 rather than
# failing when the mcap plugin is missing -- which is a silent downgrade of the storage format
# the Dockerfile and the runbook both document. This check makes that non-silent.
RECORD_ARGUMENT = "record"
MCAP_APT_PACKAGE = "ros-humble-rosbag2-storage-mcap"


def _condition_arg(node_call: ast.Call) -> str | None:
    """Return the launch-argument name a Node(...) call's `condition=` is gated on."""
    for keyword in node_call.keywords:
        if keyword.arg != "condition":
            continue
        for sub in ast.walk(keyword.value):
            if (
                isinstance(sub, ast.Call)
                and isinstance(sub.func, ast.Name)
                and sub.func.id == "LaunchConfiguration"
                and sub.args
                and isinstance(sub.args[0], ast.Constant)
            ):
                return str(sub.args[0].value)
    return None


def _package_of(node_call: ast.Call) -> str | None:
    for keyword in node_call.keywords:
        if keyword.arg == "package" and isinstance(keyword.value, ast.Constant):
            return str(keyword.value.value)
    return None


def _declared_defaults(tree: ast.AST) -> dict[str, str]:
    """name -> default_value for every DeclareLaunchArgument in the launch file."""
    defaults: dict[str, str] = {}
    for call in ast.walk(tree):
        if not (
            isinstance(call, ast.Call)
            and isinstance(call.func, ast.Name)
            and call.func.id == "DeclareLaunchArgument"
        ):
            continue
        if not (call.args and isinstance(call.args[0], ast.Constant)):
            continue
        name = str(call.args[0].value)
        for keyword in call.keywords:
            if keyword.arg == "default_value" and isinstance(keyword.value, ast.Constant):
                defaults[name] = str(keyword.value.value)
    return defaults


def main() -> int:
    tree = ast.parse(LAUNCH_FILE.read_text())
    defaults = _declared_defaults(tree)
    dockerfile = DOCKERFILE.read_text()

    problems: list[str] = []
    checked = 0

    for call in ast.walk(tree):
        if not (
            isinstance(call, ast.Call)
            and isinstance(call.func, ast.Name)
            and call.func.id == "Node"
        ):
            continue
        package = _package_of(call)
        if package is None or package.startswith("racer_"):
            continue
        gate = _condition_arg(call)
        # Unconditional, or gated on an argument that defaults to true: the package must be
        # in the image. Gated on a false default: the operator opted in, still must be there
        # if the image claims to support it, but that is not what this check enforces.
        if gate is not None and defaults.get(gate, "false").lower() != "true":
            continue
        apt_package = APT_PACKAGE_FOR.get(package)
        if apt_package is None:
            problems.append(
                f"{LAUNCH_FILE.name} starts package '{package}' by default, but "
                f"{__file__}'s APT_PACKAGE_FOR has no mapping for it. Add the mapping (and "
                f"the apt package to {DOCKERFILE}) rather than deleting this check."
            )
            continue
        checked += 1
        if not re.search(rf"^\s*{re.escape(apt_package)}\s*\\?\s*$", dockerfile, re.MULTILINE):
            problems.append(
                f"{LAUNCH_FILE.name} starts '{package}' by default"
                + (f" (launch argument '{gate}' defaults to true)" if gate else "")
                + f", but docker/car/Dockerfile does not install '{apt_package}'. "
                "`ros2 launch racer_bringup car_teleop.launch.py` would fail in the car "
                f"image with \"package '{package}' not found\"."
            )

    if defaults.get(RECORD_ARGUMENT, "false").lower() == "true":
        checked += 1
        if not re.search(rf"^\s*{re.escape(MCAP_APT_PACKAGE)}\s*\\?\s*$", dockerfile, re.MULTILINE):
            problems.append(
                f"{LAUNCH_FILE.name} records a rosbag by default ('{RECORD_ARGUMENT}' defaults "
                f"to true), but docker/car/Dockerfile does not install '{MCAP_APT_PACKAGE}'. "
                "The launch would silently fall back to sqlite3 instead of the mcap format "
                "the Dockerfile and docs/notes/first-boot-runbook.md document."
            )

    if problems:
        for problem in problems:
            print(f"ERROR: {problem}", file=sys.stderr)
        return 1

    print(f"car image / car_teleop.launch.py package check OK ({checked} package(s) verified)")
    return 0


if __name__ == "__main__":
    sys.exit(main())
