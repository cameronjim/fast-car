"""Launch description checks, the fake detector, and static guards for racer_apps.

Pure where it can be (the visibility geometry, the source guard); the launch description is
evaluated in-process with no node started (needs launch_ros and racer_tools, skipped without).
"""

from __future__ import annotations

import importlib.util
import math
import re
from pathlib import Path

import pytest

PKG = Path(__file__).resolve().parents[1]
LAUNCH = PKG / "launch" / "spiderbot.launch.py"
# Eight waypoints on a 2 m radius circle: a loop the car can drive (curvature 0.5 1/m).
SQUARE = f"frame: map\nloop: true\nwaypoints: {[[2.0, 0.0], [1.414, 1.414], [0.0, 2.0], [-1.414, 1.414], [-2.0, 0.0], [-1.414, -1.414], [-0.0, -2.0], [1.414, -1.414]]}\n"


# -- static guards (no ROS) -----------------------------------------------------------------


def test_nothing_in_racer_apps_publishes_slash_drive():
    """Only safety_node publishes /drive (CLAUDE.md invariant 1). Source-level guard."""
    offenders = []
    for path in list(PKG.glob("racer_apps/**/*.py")) + list(PKG.glob("launch/*.py")):
        text = path.read_text(encoding="utf-8")
        for match in re.finditer(r"create_publisher\([^)]*\)", text, flags=re.DOTALL):
            if re.search(r"""["']/drive["']""", match.group(0)):
                offenders.append(str(path))
        if re.search(r"""\(\s*["']/drive_raw["']\s*,\s*["']/drive["']""", text):
            offenders.append(f"{path} remaps /drive_raw onto /drive")
    assert offenders == []


def test_spiderbot_is_not_part_of_the_default_stack():
    teleop = (PKG.parent / "racer_bringup" / "launch" / "car_teleop.launch.py").read_text()
    assert "racer_apps" not in teleop and "spiderbot.launch" not in teleop


def test_no_vehicle_constants_are_hand_written_in_the_node_or_launch():
    # Invariant 2: steering limits, accelerations and the wheelbase come from vehicle_params.
    for path in (
        PKG / "racer_apps" / "spiderbot" / "spiderbot_node.py",
        LAUNCH,
    ):
        text = path.read_text(encoding="utf-8")
        assert "0.4189" not in text and "0.3302" not in text and "9.51" not in text


# -- fake detector geometry -----------------------------------------------------------------


def _geometry():
    spec = importlib.util.find_spec("racer_apps.spiderbot.fake_detector_node")
    if spec is None:
        pytest.skip("racer_apps not importable")
    pytest.importorskip("rclpy")
    pytest.importorskip("racer_msgs")
    from racer_apps.spiderbot.fake_detector_node import visible_bearing_range

    return visible_bearing_range


def test_person_ahead_is_at_bearing_zero_and_in_range():
    f = _geometry()
    bearing, rng = f(0.0, 0.0, 0.0, 3.0, 0.0, 1.48, 6.0)
    assert bearing == pytest.approx(0.0) and rng == pytest.approx(3.0)


def test_left_is_positive_and_the_car_heading_is_subtracted():
    f = _geometry()
    assert f(0.0, 0.0, 0.0, 3.0, 1.0, 1.48, 6.0)[0] > 0
    assert f(0.0, 0.0, 0.0, 3.0, -1.0, 1.48, 6.0)[0] < 0
    # Car turned left 20 degrees: a person straight along +x is now 20 degrees on its right.
    turned = math.radians(20.0)
    assert f(0.0, 0.0, turned, 3.0, 0.0, 1.48, 6.0)[0] == pytest.approx(-turned)


def test_out_of_view_or_range_is_none():
    f = _geometry()
    assert f(0.0, 0.0, 0.0, -3.0, 0.0, 1.48, 6.0) is None  # behind
    assert f(0.0, 0.0, 0.0, 9.0, 0.0, 1.48, 6.0) is None  # too far
    assert f(0.0, 0.0, 0.0, 3.0, 3.0, 1.0, 6.0) is None  # 45 degrees, fov +-28
    assert f(1.0, 1.0, 0.0, 1.0, 1.0, 1.48, 6.0) is None  # on top of the car


# -- the launch description -----------------------------------------------------------------


@pytest.fixture
def launch_module():
    pytest.importorskip("launch_ros")
    pytest.importorskip("racer_tools")
    spec = importlib.util.spec_from_file_location("spiderbot_launch", LAUNCH)
    module = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(module)
    return module


def _context(tmp_path, patrol_text=SQUARE, **over):
    from launch import LaunchContext

    patrol = tmp_path / "patrol.yaml"
    patrol.write_text(patrol_text)
    ctx = LaunchContext()
    cfg = {"patrol": str(patrol), "speed_mps": "", "odom_topic": "/odom", "pose_frame": "map"}
    cfg.update(over)
    ctx.launch_configurations.update(cfg)
    return ctx


def test_launch_arguments_have_the_documented_defaults(launch_module):
    from launch import LaunchContext
    from launch.actions import DeclareLaunchArgument

    ctx = LaunchContext()
    desc = launch_module.generate_launch_description()
    defaults = {
        e.name: "".join(s.perform(ctx) for s in e.default_value)
        for e in desc.entities
        if isinstance(e, DeclareLaunchArgument)
    }
    assert defaults["rule"] == "arms_raised" and defaults["rule_frames"] == "3"
    assert defaults["face_enabled"] == "true" and defaults["require_perception"] == "true"
    assert defaults["pose_frame"] == "map" and defaults["odom_topic"] == "/odom"
    assert defaults["patrol"] == ""  # no default patrol: the operator names one
    assert launch_module.MAX_PATROL_SPEED_MPS <= 1.0


def test_launch_starts_the_tracker_remapped_away_from_drive_raw_and_the_spiderbot(
    launch_module, tmp_path
):
    nodes = launch_module._launch_nodes(_context(tmp_path))
    by_exec = {n.node_executable: n for n in nodes}
    assert set(by_exec) == {"tracker_node", "spiderbot_node"}
    ctx = _context(tmp_path)
    remaps = {
        ("".join(s.perform(ctx) for s in src), "".join(s.perform(ctx) for s in dst))
        for src, dst in by_exec["tracker_node"]._Node__remappings
    }
    assert ("/drive_raw", "/spiderbot/patrol_cmd") in remaps
    assert ("/drive_raw", "/drive") not in remaps


def test_launch_writes_a_raceline_the_tracker_can_load(launch_module, tmp_path, monkeypatch):
    from racer_tools.raceline_loader import load_raceline_points

    out = tmp_path / "gen"
    out.mkdir()
    monkeypatch.setattr(launch_module.tempfile, "mkdtemp", lambda prefix="": str(out))
    launch_module._launch_nodes(_context(tmp_path, speed_mps="0.6"))
    points = load_raceline_points(out / "patrol_raceline.csv")
    assert len(points) > 100 and all(p.target_speed_mps == 0.6 for p in points)


def test_launch_refuses_before_starting_anything(launch_module, tmp_path):
    from racer_apps.spiderbot.patrol import PatrolError

    with pytest.raises(PatrolError, match="apart|tighter"):
        launch_module._launch_nodes(
            _context(tmp_path, "frame: map\nloop: true\nwaypoints: [[0,0],[0.05,0],[3,3],[0,3]]\n")
        )
    with pytest.raises(PatrolError, match="outside"):
        launch_module._launch_nodes(_context(tmp_path, speed_mps="5.0"))
    with pytest.raises(PatrolError, match="outside"):
        launch_module._launch_nodes(_context(tmp_path, speed_mps="0"))
    with pytest.raises(RuntimeError, match="required"):
        launch_module._launch_nodes(_context(tmp_path, patrol=""))
    with pytest.raises(PatrolError, match="loop"):
        launch_module._launch_nodes(
            _context(tmp_path, "frame: map\nloop: false\nwaypoints: [[0,0],[3,0],[3,3]]\n")
        )
    with pytest.raises(PatrolError, match="tighter"):
        launch_module._launch_nodes(
            _context(tmp_path, "frame: map\nloop: true\nwaypoints: [[0,0],[1,0],[1,0.3],[0,0.3]]\n")
        )
