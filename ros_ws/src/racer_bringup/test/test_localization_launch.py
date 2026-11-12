"""L1 checks of odometry.launch.py, mapping.launch.py, localization.launch.py and
car_teleop.launch.py's mapping / localization / map arguments (2026-11-03).

Loads the SOURCE launch files and inspects their launch descriptions; nothing is started (the
L5 canaries in tests/l5_localization run them for real). What it pins:

  * odometry: wheel_twist_node + robot_localization's ekf_node, the EKF's output remapped to
    /odom (claude-docs/04-architecture.md), parameters from racer_state's ekf.yaml;
  * mapping: odometry included by default, slam_toolbox's async node on racer_state's config;
  * localization: map_server, AMCL with amcl_pose remapped to /pose, a lifecycle manager that
    manages exactly those two, and a refusal to start without a map yaml;
  * car_teleop: both arguments default false, an empty map, at most one of the two includes,
    a refusal when both are asked for, and a recorder regex that keeps the localisation topics;
  * nothing in any of these files publishes /drive or /drive_raw (they are not on the command
    path; wheel_twist_node only reads /drive).
"""

from __future__ import annotations

import importlib.util
import pathlib
import re

import pytest
from launch import LaunchContext
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, OpaqueFunction
from launch.substitutions.substitution_failure import SubstitutionFailure
from launch_ros.actions import Node
from launch_ros.utilities import evaluate_parameters

_LAUNCH_DIR = pathlib.Path(__file__).resolve().parents[1] / "launch"
_STATE_CONFIG = pathlib.Path(__file__).resolve().parents[2] / "racer_state" / "config"


def _load(name: str):
    spec = importlib.util.spec_from_file_location(name.replace(".", "_"), _LAUNCH_DIR / name)
    module = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(module)
    return module


def _entities(name: str):
    return _load(name).generate_launch_description().entities


def _defaults(entities) -> dict:
    """Argument defaults; one that depends on another argument (sim_autopilot's raceline_path
    on track) reads as None."""
    context = LaunchContext()
    out = {}
    for e in entities:
        if isinstance(e, DeclareLaunchArgument):
            try:
                out[e.name] = "".join(s.perform(context) for s in e.default_value)
            except SubstitutionFailure:
                out[e.name] = None
    return out


def _nodes(entities) -> list[Node]:
    return [e for e in entities if isinstance(e, Node)]


def _remaps(node: Node) -> dict:
    context = LaunchContext()
    out = {}
    for src, dst in node._Node__remappings or []:
        src = src if isinstance(src, str) else "".join(s.perform(context) for s in src)
        dst = dst if isinstance(dst, str) else "".join(s.perform(context) for s in dst)
        out[src] = dst
    return out


def _include_name(include: IncludeLaunchDescription) -> str:
    """The file an include points at. Loading the source resolves `location` (until then it is a
    substitution) and proves the file imports cleanly; same approach as test_camera_launch.py."""
    include.launch_description_source.get_launch_description(LaunchContext())
    return pathlib.Path(include.launch_description_source.location).name


def _context(**configs) -> LaunchContext:
    context = LaunchContext()
    context.launch_configurations.update(configs)
    return context


# --- odometry.launch.py ------------------------------------------------------------------


def test_odometry_starts_the_twist_node_and_the_ekf_on_slash_odom():
    nodes = _nodes(_entities("odometry.launch.py"))
    assert [(n.node_package, n.node_executable) for n in nodes] == [
        ("racer_state", "wheel_twist_node"),
        ("robot_localization", "ekf_node"),
    ]
    assert _remaps(nodes[1]) == {"odometry/filtered": "/odom"}
    assert _remaps(nodes[0]) == {}


def test_odometry_defaults():
    defaults = _defaults(_entities("odometry.launch.py"))
    assert defaults["wheel_odom_topic"] == "/odom/wheel"
    # 0 = use vehicle_params' measured yaw rate gain table; sim launches pass 1.0.
    assert defaults["yaw_gain_override"] == "0.0"
    assert set(defaults) == {"ekf_params_file", "wheel_odom_topic", "yaw_gain_override"}


# --- mapping.launch.py -------------------------------------------------------------------


def test_mapping_runs_slam_toolbox_async_and_includes_odometry_by_default():
    entities = _entities("mapping.launch.py")
    assert _defaults(entities)["odometry"] == "true"
    nodes = _nodes(entities)
    assert [(n.node_package, n.node_executable) for n in nodes] == [
        ("slam_toolbox", "async_slam_toolbox_node")
    ]
    includes = [e for e in entities if isinstance(e, IncludeLaunchDescription)]
    assert len(includes) == 1
    assert _include_name(includes[0]) == "odometry.launch.py"


# --- localization.launch.py --------------------------------------------------------------


def test_localization_runs_map_server_amcl_and_a_lifecycle_manager():
    entities = _entities("localization.launch.py")
    nodes = {n.node_executable: n for n in _nodes(entities)}
    assert set(nodes) == {"map_server", "amcl", "lifecycle_manager"}
    assert _remaps(nodes["amcl"]) == {"amcl_pose": "/pose"}
    defaults = _defaults(entities)
    assert defaults["map"] == ""
    assert defaults["odometry"] == "true"
    assert defaults["set_initial_pose"] == "true"
    assert (defaults["initial_pose_x"], defaults["initial_pose_y"]) == ("0.0", "0.0")
    params = {}
    for entry in evaluate_parameters(LaunchContext(), nodes["lifecycle_manager"]._Node__parameters):
        params.update(entry)
    assert list(params["node_names"]) == ["map_server", "amcl"]
    assert params["autostart"] is True


def test_localization_refuses_without_a_map(tmp_path):
    module = _load("localization.launch.py")
    with pytest.raises(RuntimeError, match="map:=.*required"):
        module._require_map(_context(map=""))
    with pytest.raises(RuntimeError, match="not found"):
        module._require_map(_context(map=str(tmp_path / "missing.yaml")))
    real = tmp_path / "track.yaml"
    real.write_text("image: track.pgm\n")
    assert module._require_map(_context(map=str(real))) == []


def test_configs_the_launch_files_point_at_exist():
    for name in ("ekf.yaml", "slam_toolbox.yaml", "amcl.yaml"):
        assert (_STATE_CONFIG / name).is_file(), name


@pytest.mark.parametrize(
    "name", ["odometry.launch.py", "mapping.launch.py", "localization.launch.py"]
)
def test_not_on_the_command_path(name):
    """No node here publishes to the actuator side: nothing remaps onto /drive or /drive_raw."""
    for node in _nodes(_entities(name)):
        for dst in _remaps(node).values():
            assert dst not in ("/drive", "/drive_raw"), (name, node.node_executable)


# --- car_teleop.launch.py ----------------------------------------------------------------


def test_car_teleop_mapping_and_localization_default_off():
    defaults = _defaults(_entities("car_teleop.launch.py"))
    assert defaults["mapping"] == "false"
    assert defaults["localization"] == "false"
    assert defaults["map"] == ""


def test_car_teleop_includes_at_most_one_and_refuses_both():
    module = _load("car_teleop.launch.py")
    assert any(
        isinstance(e, OpaqueFunction)
        and e._OpaqueFunction__function is module._localization_actions
        for e in module.generate_launch_description().entities
    )
    assert module._localization_actions(_context(mapping="false", localization="false")) == []
    mapping = module._localization_actions(_context(mapping="true", localization="false"))
    assert len(mapping) == 1
    assert _include_name(mapping[0]) == "mapping.launch.py"
    localization = module._localization_actions(
        _context(mapping="false", localization="true", map="/data/maps/floor.yaml")
    )
    assert len(localization) == 1
    assert _include_name(localization[0]) == "localization.launch.py"
    with pytest.raises(RuntimeError, match="both own map -> odom"):
        module._localization_actions(_context(mapping="true", localization="true"))


@pytest.mark.parametrize(
    "topic", ["/odom", "/odom/wheel_twist", "/pose", "/tf", "/tf_static", "/map", "/odom/wheel"]
)
def test_car_teleop_records_the_localisation_topics(topic):
    regex = _load("car_teleop.launch.py")._RECORDED_TOPIC_REGEX
    assert re.search(regex, topic), topic


def test_car_teleop_records_gap_follow_status():
    """/gap_follow/status (scan-to-command latency, 2026-11-12) is in every bag, and only that
    exact topic: a lookalike is not swept in."""
    regex = _load("car_teleop.launch.py")._RECORDED_TOPIC_REGEX
    assert re.search(regex, "/gap_follow/status")
    assert re.search(regex, "/gap_follow/status_extra") is None
    assert re.search(regex, "/gap_follow") is None


@pytest.mark.parametrize("topic", ["/particle_cloud", "/map_updates", "/odometry/filtered"])
def test_car_teleop_leaves_out_bulky_or_unused_topics(topic):
    regex = _load("car_teleop.launch.py")._RECORDED_TOPIC_REGEX
    assert re.search(regex, topic) is None, topic


# --- sim_autopilot.launch.py -------------------------------------------------------------


def _sim_context(**overrides) -> LaunchContext:
    configs = {
        "track": "gym_oval",
        "raceline_path": "../config/tracks/gym_oval/raceline.csv",
        "mapping": "false",
        "localization": "false",
        "map": "",
        "track_half_width_m": "0.0",
        "sensor_noise": "false",
    }
    configs.update(overrides)
    return _context(**configs)


def test_sim_autopilot_new_arguments_default_to_the_old_behaviour():
    defaults = _defaults(_entities("sim_autopilot.launch.py"))
    assert defaults["mapping"] == "false"
    assert defaults["localization"] == "false"
    assert defaults["track_half_width_m"] == "0.0"
    assert defaults["sensor_noise"] == "false"
    module = _load("sim_autopilot.launch.py")
    actions = module._autopilot_nodes(_sim_context())
    nodes = {n.node_executable: n for n in actions if isinstance(n, Node)}
    assert set(nodes) == {"bridge_node", "safety_node", "tracker_node", "raceline_publisher_node"}
    assert not [a for a in actions if isinstance(a, IncludeLaunchDescription)]
    # Unchanged default: the tracker follows ground truth through the same remap as before.
    assert _remaps(nodes["tracker_node"]) == {"/odom": "/sim/ground_truth_odom"}
    bridge = {}
    for entry in evaluate_parameters(_sim_context(), nodes["bridge_node"]._Node__parameters):
        bridge.update(entry)
    assert "publish_ground_truth_tf" not in bridge
    assert bridge["track_half_width_m"] == 0.0


def test_sim_autopilot_refuses_both_modes():
    module = _load("sim_autopilot.launch.py")
    with pytest.raises(RuntimeError, match="pick one"):
        module._autopilot_nodes(_sim_context(mapping="true", localization="true"))


def test_car_like_bridge_parameters_follow_vehicle_params():
    from types import SimpleNamespace

    module = _load("sim_autopilot.launch.py")
    vp = SimpleNamespace(
        chassis=SimpleNamespace(cg_to_rear_axle_m=0.2),
        sensors=SimpleNamespace(
            lidar=SimpleNamespace(mount_x_m=0.3, mount_yaw_rad=3.0),
            lidar_spec=SimpleNamespace(
                angular_resolution_rad=2.0 * 3.141592653589793 / 360.0,
                nominal_scan_rate_hz=10.0,
                range_max_m=12.0,
            ),
        ),
    )
    params = module.car_like_bridge_parameters(vp, sensor_noise=False)
    assert params["publish_ground_truth_tf"] is False
    assert params["base_link_offset_x_m"] == -0.2
    assert abs(params["lidar_offset_x_m"] - 0.1) < 1e-12
    assert params["lidar_yaw_rad"] == 3.0
    assert params["lidar_num_beams"] == 360
    assert params["scan_rate_hz"] == 10.0
    assert params["scan_range_max_m"] == 12.0
    assert "wheel_speed_scale" not in params
    noisy = module.car_like_bridge_parameters(vp, sensor_noise=True)
    assert noisy["wheel_speed_scale"] == module.SIM_SENSOR_NOISE["wheel_speed_scale"]
