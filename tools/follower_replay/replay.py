"""Parameters, the scan text file, and running gap_follow_replay_cli.

Profiles come from ros_ws/src/racer_control/launch/gap_follow.launch.py, the file the car's
launch reads, so a replay with `--profile floor-2026-10-06` uses the values the car ran. The
launch file imports launch and launch_ros, which exist only in a ROS install, so it is not
imported here: the module is parsed and its top-level dict literals (PROFILES and the profile
dicts it names) are evaluated with ast. Same values, no ROS.
"""

from __future__ import annotations

import ast
import json
import math
import os
import shutil
import subprocess
from collections.abc import Iterable, Sequence
from pathlib import Path
from typing import Any

from follower_replay.bag import ScanRecord

REPO_ROOT = Path(__file__).resolve().parents[2]
LAUNCH_FILE = REPO_ROOT / "ros_ws/src/racer_control/launch/gap_follow.launch.py"
CLI_NAME = "gap_follow_replay_cli"
CLI_ENV = "GAP_FOLLOW_REPLAY_CLI"

ParamValue = float | int | bool | str

# gap_follow_node's string parameters; any other --param value must be a number or true/false.
STRING_PARAMETERS = {"target_mode"}


class ReplayError(RuntimeError):
    """A problem the user can fix (bad profile name, missing CLI); printed without a trace."""


def _literal(node: ast.expr, env: dict[str, Any]) -> Any:
    """ast.literal_eval that also resolves names bound earlier in the module."""
    if isinstance(node, ast.Name):
        if node.id not in env:
            raise ReplayError(f"{LAUNCH_FILE.name}: cannot resolve name {node.id!r}")
        return env[node.id]
    if isinstance(node, ast.Dict):
        return {
            ast.literal_eval(k): _literal(v, env)
            for k, v in zip(node.keys, node.values, strict=True)
            if k is not None
        }
    return ast.literal_eval(node)


def load_profiles(launch_file: Path = LAUNCH_FILE) -> dict[str, dict[str, ParamValue]]:
    """PROFILES of gap_follow.launch.py as {name: {parameter: value}}."""
    tree = ast.parse(launch_file.read_text(encoding="utf-8"), filename=str(launch_file))
    env: dict[str, Any] = {}
    for node in tree.body:
        target: ast.expr | None = None
        value: ast.expr | None = None
        if isinstance(node, ast.Assign) and len(node.targets) == 1:
            target, value = node.targets[0], node.value
        elif isinstance(node, ast.AnnAssign) and node.value is not None:
            target, value = node.target, node.value
        if not isinstance(target, ast.Name) or value is None:
            continue
        if isinstance(value, ast.Dict | ast.Constant):
            env[target.id] = _literal(value, env)
    profiles = env.get("PROFILES")
    if not isinstance(profiles, dict) or not profiles:
        raise ReplayError(f"{launch_file} has no PROFILES dict")
    return {str(name): dict(params) for name, params in profiles.items()}


def parse_value(text: str, name: str = "") -> ParamValue:
    """A --param value: the text itself for a string parameter (target_mode), else true /
    false, an integer, or a float. The CLI checks a string value, as the node does."""
    if name in STRING_PARAMETERS:
        return text.strip()
    lowered = text.strip().lower()
    if lowered in ("true", "false"):
        return lowered == "true"
    try:
        return int(lowered)
    except ValueError:
        pass
    try:
        return float(lowered)
    except ValueError:
        raise ReplayError(f"parameter value {text!r} is not a number or true/false") from None


def parse_overrides(items: Iterable[str]) -> dict[str, ParamValue]:
    overrides: dict[str, ParamValue] = {}
    for item in items:
        key, sep, value = item.partition("=")
        if not sep or not key.strip():
            raise ReplayError(f"--param wants name=value, got {item!r}")
        overrides[key.strip()] = parse_value(value, key.strip())
    return overrides


def resolve_params(
    profile: str,
    overrides: dict[str, ParamValue],
    profiles: dict[str, dict[str, ParamValue]] | None = None,
) -> dict[str, ParamValue]:
    """The profile's parameters with the overrides on top (overrides win, as launch args do)."""
    profiles = load_profiles() if profiles is None else profiles
    if profile not in profiles:
        raise ReplayError(f"unknown profile {profile!r}; choose from {sorted(profiles)}")
    return {**profiles[profile], **overrides}


def format_number(value: float) -> str:
    """strtod-readable, exact for a float32 (9 significant digits)."""
    if math.isnan(value):
        return "nan"
    if math.isinf(value):
        return "inf" if value > 0 else "-inf"
    return f"{value:.9g}"


def scan_line(scan: ScanRecord) -> str:
    """One scan in the CLI's text format: stamp angle_min angle_increment n ranges..."""
    head = f"{scan.stamp_s:.9f} {scan.angle_min:.9g} {scan.angle_increment:.9g} {len(scan.ranges)}"
    return head + " " + " ".join(format_number(r) for r in scan.ranges)


def write_scan_file(scans: Sequence[ScanRecord], path: Path) -> None:
    with path.open("w", encoding="utf-8") as out:
        out.write("# stamp angle_min angle_increment n ranges... (replay_follower.py)\n")
        for scan in scans:
            out.write(scan_line(scan) + "\n")


def find_cli(explicit: str | None = None) -> Path:
    """--cli, else $GAP_FOLLOW_REPLAY_CLI, else a colcon install tree, else PATH."""
    if explicit:
        if not Path(explicit).is_file():
            raise ReplayError(f"--cli {explicit}: no such file")
        return Path(explicit)
    candidates = [
        os.environ.get(CLI_ENV, ""),
        str(REPO_ROOT / "ros_ws/install/racer_control/lib/racer_control" / CLI_NAME),
    ]
    for candidate in candidates:
        if candidate and Path(candidate).is_file():
            return Path(candidate)
    found = shutil.which(CLI_NAME)
    if found:
        return Path(found)
    raise ReplayError(
        f"{CLI_NAME} not found. Build it (colcon build --packages-select racer_control in the "
        f"ros-dev container) and pass --cli, or set ${CLI_ENV}. See docs/notes/replay-follower.md."
    )


def _run(cmd: list[str], stdin: str | None = None) -> str:
    proc = subprocess.run(cmd, input=stdin, capture_output=True, text=True, check=False)
    if proc.returncode != 0:
        raise ReplayError(f"{CLI_NAME} exited {proc.returncode}: {proc.stderr.strip()}")
    return proc.stdout


def run_cli(
    cli: Path, scans_file: Path, params_file: Path, extra: Sequence[str] = ()
) -> list[dict[str, Any]]:
    """Run the CLI on a scan file; one dict per scan."""
    out = _run([str(cli), "--params", str(params_file), "--scans", str(scans_file), *extra])
    return [json.loads(line) for line in out.splitlines() if line.strip()]


def describe_cli(cli: Path, params_file: Path) -> dict[str, float]:
    """The vehicle geometry the CLI resolved (LiDAR yaw and mount, half width, ...)."""
    return json.loads(_run([str(cli), "--params", str(params_file), "--describe"]))


def write_params_file(params: dict[str, ParamValue], path: Path) -> None:
    path.write_text(json.dumps(params, indent=2, sort_keys=True) + "\n", encoding="utf-8")


def lidar_range_limits(scans: Sequence[ScanRecord]) -> dict[str, ParamValue]:
    """range_min_m / range_max_m from the first scan, for the CLI (the text line has no room)."""
    if not scans:
        return {}
    lo, hi = scans[0].range_min, scans[0].range_max
    if not (math.isfinite(lo) and math.isfinite(hi) and 0.0 <= lo < hi <= 1000.0):
        return {}  # an unset or infinite limit: keep the CLI's RPLIDAR C1 defaults
    return {"range_min_m": lo, "range_max_m": hi}
