from __future__ import annotations

import re
from pathlib import Path

import pytest
from fakes import World, make_host
from racer_agent import commands as cmd

REPO_ROOT = Path(__file__).resolve().parents[4]
REPO = Path("/home/racer/car")


@pytest.fixture
def host(tmp_path: Path):
    return make_host(tmp_path, World(tmp_path))


def test_stack_command_matches_the_runbook_flag_for_flag(host):
    argv = cmd.stack_run_argv(host, REPO, "car:local", "car-stack", cmd.StackFlags())
    assert argv == [
        "docker", "run", "-d", "--name", "car-stack", "--network", "host",
        "--user", "1000:1000",
        "--group-add", "997", "--group-add", "20",
        "--device", "/dev/ttyUSB0", "--device", "/dev/ttyTHS1",
        "-e", "HOME=/tmp",
        "-v", "/sys/devices/platform/bus@0/3280000.pwm:/sys/devices/platform/bus@0/3280000.pwm",
        "-v", "/sys/devices/platform/bus@0/32c0000.pwm:/sys/devices/platform/bus@0/32c0000.pwm",
        "-v", "/home/racer/car:/workspace", "-w", "/workspace/ros_ws",
        "car:local", "bash", "-lc",
        (
            "source /opt/ros/humble/setup.bash && source install/setup.bash && "
            "exec ros2 launch racer_bringup car_teleop.launch.py lidar:=true vesc:=true"
        ),
    ]  # fmt: skip


def test_stack_never_gets_privileged_or_nvidia_runtime(host):
    argv = cmd.stack_run_argv(host, REPO, "car:local", "car-stack", cmd.StackFlags())
    assert "--privileged" not in argv and "--runtime" not in argv


def test_missing_devices_are_left_off_when_flag_is_false(host):
    flags = cmd.StackFlags(lidar=False, vesc=True)
    argv = cmd.stack_run_argv(host, REPO, "car:local", "car-stack", flags)
    assert "/dev/ttyUSB0" not in argv and "/dev/ttyTHS1" in argv
    assert argv[-1].endswith("lidar:=false vesc:=true")


def test_group_adds_skip_groups_that_do_not_exist(tmp_path):
    host = make_host(tmp_path, World(tmp_path))
    host._group_lookup = lambda name: None
    argv = cmd.stack_run_argv(host, REPO, "car:local", "car-stack", cmd.StackFlags())
    assert "--group-add" not in argv


def test_mapping_and_localisation_args(host):
    mapping = cmd.StackFlags(mapping=True)
    assert cmd.launch_args(mapping) == ["lidar:=true", "vesc:=true", "mapping:=true"]
    loc = cmd.StackFlags(localization=True, map="floor.yaml")
    args = cmd.launch_args(loc, "/workspace/data/maps/floor.yaml")
    assert args[-2:] == ["localization:=true", "map:=/workspace/data/maps/floor.yaml"]


@pytest.mark.parametrize(
    "flags",
    [
        cmd.StackFlags(mapping=True, localization=True, map="a.yaml"),
        cmd.StackFlags(mapping=True, lidar=False),
        cmd.StackFlags(localization=True, vesc=False, map="a.yaml"),
        cmd.StackFlags(localization=True),
        cmd.StackFlags(map="a.yaml"),
    ],
)
def test_invalid_flag_combinations_are_refused(flags):
    with pytest.raises(cmd.CommandError):
        flags.validate()


def test_camera_command_matches_docker_car_readme(host):
    argv = cmd.camera_run_argv(host, REPO, "car:local", "car-camera", "/dev/video1")
    assert argv[:8] == [
        "docker",
        "run",
        "-d",
        "--name",
        "car-camera",
        "--network",
        "host",
        "--runtime",
    ]
    assert "nvidia" in argv and "--group-add" in argv and "44" in argv
    assert "-v" in argv and "/tmp/argus_socket:/tmp/argus_socket" in argv
    assert (
        "FASTRTPS_DEFAULT_PROFILES_FILE=/workspace/ros_ws/src/racer_bringup/config/"
        "fastdds_udp_only.xml" in argv
    )
    assert argv[argv.index("--device") + 1] == "/dev/video1"
    assert argv[-1].endswith(
        "exec ros2 launch racer_bringup cameras.launch.py usb_device:=/dev/video1"
    )


@pytest.mark.parametrize("bad", ["/dev/video1; reboot", "/dev/by-id/x", "video1", "/dev/ttyUSB0"])
def test_camera_device_must_be_a_video_node(host, bad):
    with pytest.raises(cmd.CommandError):
        cmd.camera_run_argv(host, REPO, "car:local", "car-camera", bad)


def test_gap_follow_and_park_commands():
    assert (
        cmd.ros_cmd_for("gap_follow", "lane-2026-10-25")
        == "ros2 launch racer_control gap_follow.launch.py profile:=lane-2026-10-25"
    )
    assert cmd.ros_cmd_for("wall_follow") == "ros2 launch racer_control wall_follow.launch.py"
    assert (
        cmd.ros_cmd_for("park", "floor-2026-10-07")
        == "ros2 launch racer_control park.launch.py profile:=floor-2026-10-07"
    )


@pytest.mark.parametrize(
    "mode,profile",
    [
        ("gap_follow", "x; rm -rf /"),
        ("gap_follow", "$(reboot)"),
        ("gap_follow", ""),
        ("gap_follow", "floor-2026-10-07"),
        ("park", "lane-2026-10-25"),
        ("teleop", ""),
        ("ml_policy", ""),
    ],
)
def test_anything_off_the_lists_is_refused(mode, profile):
    with pytest.raises(cmd.CommandError):
        cmd.ros_cmd_for(mode, profile)


def test_ml_policy_command_launches_policy_launch_with_checked_paths():
    out = cmd.ros_cmd_for(
        "ml_policy",
        raceline_container_path="/workspace/data/maps/a/raceline.csv",
        policy_container_path="/workspace/data/policies/v1-2026-11-11",
    )
    assert out == (
        "ros2 launch racer_bringup policy.launch.py "
        "policy_dir:=/workspace/data/policies/v1-2026-11-11 "
        "raceline:=/workspace/data/maps/a/raceline.csv pose_frame:=map"
    )


@pytest.mark.parametrize(
    "policy,raceline",
    [
        ("/etc/policies/x", "/workspace/data/a.csv"),
        ("/workspace/data/policies/../../etc", "/workspace/data/a.csv"),
        ("/workspace/data/policies/x", "/home/me/a.csv"),
        ("/workspace/data/policies/x", "/workspace/../a.csv"),
        ("/workspace/data/policies/x; reboot", "/workspace/data/a.csv"),
        ("", "/workspace/data/a.csv"),
        ("/workspace/data/policies/x", ""),
    ],
)
def test_ml_policy_command_refuses_paths_off_the_repo_or_with_shell_syntax(policy, raceline):
    with pytest.raises(cmd.CommandError):
        cmd.ros_cmd_for("ml_policy", raceline_container_path=raceline, policy_container_path=policy)


def test_tracker_command_runs_the_node_with_pose_frame_map():
    out = cmd.ros_cmd_for(
        "tracker", raceline_container_path="/workspace/config/tracks/a/raceline.csv"
    )
    assert out == (
        "ros2 run racer_control tracker_node --ros-args "
        "-p raceline_path:=/workspace/config/tracks/a/raceline.csv -p pose_frame:=map"
    )


@pytest.mark.parametrize(
    "path",
    ["", "/etc/passwd", "/workspace/../etc/passwd", "/workspace/a b.csv", "/workspace/a;b.csv"],
)
def test_tracker_path_is_confined(path):
    with pytest.raises(cmd.CommandError):
        cmd.ros_cmd_for("tracker", raceline_container_path=path)


def test_container_path_translation_and_confinement(tmp_path):
    repo = tmp_path / "car"
    (repo / "data").mkdir(parents=True)
    assert cmd.container_path(repo, repo / "data" / "x.yaml") == "/workspace/data/x.yaml"
    with pytest.raises(cmd.CommandError):
        cmd.container_path(repo, tmp_path / "elsewhere")


def test_drive_launch_script_records_pid_mode_and_execs():
    script = cmd.drive_launch_script("gap_follow:lane-2026-10-25", "ros2 launch a b")
    assert script.startswith("echo $$ > /tmp/racer-drive.pid; echo gap_follow:lane-2026-10-25 > ")
    assert "exec ros2 launch a b > /tmp/racer-drive.log 2>&1" in script


def test_signal_scripts_only_accept_known_signals():
    assert cmd.drive_signal_script("INT").endswith("kill -INT $p")
    with pytest.raises(cmd.CommandError):
        cmd.drive_signal_script("STOP; reboot")


def test_park_services_are_a_closed_set():
    assert set(cmd.PARK_SERVICES) == {"start", "three_point_turn", "abort"}
    assert "std_srvs/srv/Trigger" in cmd.park_service_script("/park_node/abort")
    with pytest.raises(cmd.CommandError):
        cmd.park_service_script("/park_node/abort; reboot")


def test_save_map_script_shape():
    script = cmd.save_map_script("/workspace/data/maps/floor-1")
    assert "mkdir -p /workspace/data/maps" in script
    assert '"{name: {data: /workspace/data/maps/floor-1}}"' in script
    with pytest.raises(cmd.CommandError):
        cmd.save_map_script("/workspace/data/maps/a'; reboot; '")


def test_recorder_detection_reads_docker_top():
    assert cmd.recorder_running("ARGS\nros2 bag record -o x\n")
    assert not cmd.recorder_running("ARGS\nros2 launch racer_bringup car_teleop\n")
    assert cmd.recorder_check_argv("car-stack") == ["docker", "top", "car-stack", "-eo", "pid,args"]


def test_parse_launch_flags():
    text = '["bash","-lc","exec ros2 launch racer_bringup car_teleop.launch.py lidar:=true vesc:=false"]'
    assert cmd.parse_launch_flags(text) == {"lidar": "true", "vesc": "false"}


def _profile_keys(launch_file: str, var: str) -> set[str]:
    text = (REPO_ROOT / "ros_ws/src/racer_control/launch" / launch_file).read_text()
    block = re.search(rf"{var}[^=]*=\s*\{{(.*?)\n\}}", text, re.DOTALL).group(1)
    return set(re.findall(r'^\s+"([a-z0-9-]+)":', block, re.MULTILINE))


@pytest.mark.skipif(
    not (REPO_ROOT / "ros_ws/src/racer_control/launch/gap_follow.launch.py").exists(),
    reason="repo launch files not present",
)
def test_profile_whitelists_match_the_launch_files():
    assert _profile_keys("gap_follow.launch.py", "PROFILES") == set(cmd.GAP_FOLLOW_PROFILES)
    assert _profile_keys("park.launch.py", "PROFILES") == set(cmd.PARK_PROFILES)


# -- Plug-and-Play ---------------------------------------------------------------------------

PNP = "/workspace/data/pnp/models/"


def test_pnp_is_a_drive_mode():
    assert "pnp" in cmd.DRIVE_MODES


def test_pnp_net_command_launches_pnp_launch_with_checked_bundles():
    out = cmd.ros_cmd_for(
        "pnp",
        pnp_perception_container_path=PNP + "trackperc-v0-2026-11-12",
        pnp_dynamics_container_path=PNP + "dyn-v0-2026-11-12",
        pnp_backend="net",
        pnp_v_max_mps=0.5,
    )
    assert out == (
        "ros2 launch racer_bringup pnp.launch.py "
        f"perception_bundle:={PNP}trackperc-v0-2026-11-12 "
        f"dynamics_bundle:={PNP}dyn-v0-2026-11-12 backend:=net v_max_mps:=0.50"
    )


def test_pnp_geometric_command_has_no_perception_bundle():
    out = cmd.ros_cmd_for(
        "pnp",
        pnp_dynamics_container_path=PNP + "dyn-v0-2026-11-12",
        pnp_backend="geometric",
        pnp_v_max_mps=0.4,
    )
    assert "perception_bundle" not in out
    assert out.endswith("backend:=geometric v_max_mps:=0.40")


def test_pnp_command_names_no_drive_topic_and_does_not_disable_the_recorder_guard():
    out = cmd.ros_cmd_for("pnp", pnp_dynamics_container_path=PNP + "d", pnp_backend="geometric")
    assert "/drive" not in out and "require_recorder" not in out


@pytest.mark.parametrize("speed", [0.2, 0.5, 1.0, 1.5, 1])
def test_pnp_speed_inside_the_range_is_formatted(speed):
    out = cmd.ros_cmd_for(
        "pnp",
        pnp_dynamics_container_path=PNP + "d",
        pnp_backend="geometric",
        pnp_v_max_mps=speed,
    )
    assert f"v_max_mps:={float(speed):.2f}" in out


@pytest.mark.parametrize("speed", [0.19, 1.51, 0, -0.5, 9.0, True, "0.5", None, float("nan")])
def test_pnp_speed_outside_the_range_or_not_a_number_is_refused(speed):
    with pytest.raises(cmd.CommandError):
        cmd.ros_cmd_for(
            "pnp",
            pnp_dynamics_container_path=PNP + "d",
            pnp_backend="geometric",
            pnp_v_max_mps=speed,
        )


@pytest.mark.parametrize("backend", ["", "NET", "net; reboot", "classical"])
def test_pnp_backend_must_be_on_the_list(backend):
    with pytest.raises(cmd.CommandError, match="backend"):
        cmd.ros_cmd_for("pnp", pnp_dynamics_container_path=PNP + "d", pnp_backend=backend)


@pytest.mark.parametrize(
    "dynamics",
    [
        "",
        "/etc/pnp/d",
        "/workspace/data/policies/d",
        "/workspace/data/pnp/models/",
        "/workspace/data/pnp/models/../../etc",
        "/workspace/data/pnp/models/a/b",
        "/workspace/data/pnp/models/d; reboot",
        "/workspace/data/pnp/models/$(reboot)",
        "/workspace/data/pnp/models/d d",
    ],
)
def test_pnp_bundle_paths_must_be_one_directory_under_the_models_dir(dynamics):
    with pytest.raises(cmd.CommandError):
        cmd.ros_cmd_for("pnp", pnp_dynamics_container_path=dynamics, pnp_backend="geometric")


def test_pnp_net_backend_needs_a_perception_bundle_under_the_models_dir():
    with pytest.raises(cmd.CommandError):
        cmd.ros_cmd_for(
            "pnp",
            pnp_dynamics_container_path=PNP + "d",
            pnp_perception_container_path="",
            pnp_backend="net",
        )
    with pytest.raises(cmd.CommandError):
        cmd.ros_cmd_for(
            "pnp",
            pnp_dynamics_container_path=PNP + "d",
            pnp_perception_container_path="/workspace/data/x",
            pnp_backend="net",
        )
