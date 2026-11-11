"""Perception and the spiderbot patrol: whitelist, availability, preconditions, safety rules.

The new buttons must not loosen anything: the spiderbot is a driving launch, so it goes through
the same pre-drive gate, the same hard refusals (stack, recorder, disk) with no override, the
same STOP ALL, and perception (which drives nothing) is disabled with a reason when the GPU image
is not on the car.
"""

from __future__ import annotations

import re
from pathlib import Path

import pytest
from conftest import TOKEN
from racer_agent import commands as cmd
from racer_agent.controller import ActionError
from test_controller_drive import detached_execs
from test_service import make_client

REPO_ROOT = Path(__file__).resolve().parents[4]
HOST_NOT_PRESENT = not (REPO_ROOT / "ros_ws/src/racer_apps").exists()
AUTH = {"Authorization": f"Bearer {TOKEN}"}
LOC_FLAGS = "lidar:=true vesc:=true localization:=true map:=/workspace/data/maps/a.yaml"


# -- commands -----------------------------------------------------------------------------------


def test_perception_container_command_is_the_documented_one(env):
    argv = cmd.perception_run_argv(
        env.host,
        Path("/home/racer/car"),
        Path("/home/racer/car/data/perception-cache"),
        "car:perception",
        "car-perception",
        "pose",
        "cuda",
    )
    assert argv[:6] == ["docker", "run", "-d", "--name", "car-perception", "--network"]
    assert "--runtime" in argv and argv[argv.index("--runtime") + 1] == "nvidia"
    assert argv[argv.index("--user") + 1] == "1000:1000"
    assert "--privileged" not in argv and "--device" not in argv
    assert "/home/racer/car/data/perception-cache:/tmp/.cache/racer_perception" in argv
    assert argv[-1].endswith(
        "exec ros2 launch racer_perception perception.launch.py mode:=pose device:=cuda"
    )
    assert any(a.startswith("FASTRTPS_DEFAULT_PROFILES_FILE=") for a in argv)


@pytest.mark.parametrize(
    ("mode", "device"), [("segment", "cuda"), ("pose", "mps"), ("x; reboot", "cpu")]
)
def test_perception_refuses_anything_off_the_lists(env, mode, device):
    with pytest.raises(cmd.CommandError):
        cmd.perception_run_argv(
            env.host, Path("/r"), Path("/c"), "car:perception", "n", mode, device
        )


def test_spiderbot_command_uses_closed_sets_and_formats_the_speed():
    line = cmd.spiderbot_cmd("/workspace/data/patrols/loop.yaml", "arms_raised", 0.5)
    assert line == (
        "ros2 launch racer_apps spiderbot.launch.py patrol:=/workspace/data/patrols/loop.yaml "
        "rule:=arms_raised speed_mps:=0.50 output_dir:=/workspace/data/spiderbot"
    )


@pytest.mark.parametrize(
    "kwargs",
    [
        {"patrol_container_path": "/etc/passwd"},
        {"patrol_container_path": "/workspace/../etc/passwd"},
        {"patrol_container_path": "/workspace/data/patrols/a b.yaml"},
        {"patrol_container_path": "/workspace/p.yaml", "rule": "arms_raised; reboot"},
        {"patrol_container_path": "/workspace/p.yaml", "rule": "moonwalk"},
        {"patrol_container_path": "/workspace/p.yaml", "speed_mps": 5.0},
        {"patrol_container_path": "/workspace/p.yaml", "speed_mps": 0.0},
        {"patrol_container_path": "/workspace/p.yaml", "speed_mps": True},
        {"patrol_container_path": "/workspace/p.yaml", "speed_mps": "fast"},
    ],
)
def test_spiderbot_command_refuses_off_list_input(kwargs):
    with pytest.raises(cmd.CommandError):
        cmd.spiderbot_cmd(**kwargs)


@pytest.mark.skipif(HOST_NOT_PRESENT, reason="repo racer_apps not present")
def test_whitelists_match_the_racer_apps_sources():
    rules_src = (REPO_ROOT / "ros_ws/src/racer_apps/racer_apps/spiderbot/pose_rules.py").read_text()
    block = re.search(r"_REGISTRY: dict\[.*?\] = \{(.*?)\n\}", rules_src, re.DOTALL).group(1)
    assert set(re.findall(r'^\s+"([a-z_]+)":', block, re.MULTILINE)) == set(cmd.SPIDERBOT_RULES)
    launch_src = (REPO_ROOT / "ros_ws/src/racer_apps/launch/spiderbot.launch.py").read_text()
    ceiling = float(re.search(r"MAX_PATROL_SPEED_MPS = ([0-9.]+)", launch_src).group(1))
    assert cmd.SPIDERBOT_SPEED_RANGE[1] <= ceiling
    assert 'SPIDERBOT_ARGUMENTS = {\n    "rule": ("arms_raised"' in launch_src
    assert cmd.SPIDERBOT_DEFAULT_SPEED_MPS <= ceiling
    perception_src = (
        REPO_ROOT / "ros_ws/src/racer_perception/launch/perception.launch.py"
    ).read_text()
    assert '"mode": ("detect"' in perception_src and "pose" in perception_src
    assert set(cmd.PERCEPTION_MODES) == {"pose", "detect"}


# -- perception ---------------------------------------------------------------------------------


@pytest.fixture
def car(env):
    env.running_car()
    return env


def start_cameras(car):
    car.controller.start_cameras()


def test_perception_is_disabled_with_a_reason_when_the_gpu_image_is_missing(car):
    car.world.perception_image_present = False
    status = car.controller.perception_status()
    assert status["available"] is False and "car:perception is not built" in status["reason"]
    assert "INSTALL_TORCH=required" in status["reason"]
    start_cameras(car)
    with pytest.raises(ActionError, match="not built") as exc:
        car.controller.start_perception()
    assert exc.value.status == 409
    assert "car-perception" not in car.world.containers
    assert car.last_log()["ok"] is False and car.last_log()["action"] == "perception.start"


def test_perception_needs_the_cameras_first(car):
    with pytest.raises(ActionError, match="start the cameras first"):
        car.controller.start_perception()
    assert "car-perception" not in car.world.containers


def test_perception_starts_and_stops_like_the_camera_container(car):
    start_cameras(car)
    result = car.controller.start_perception("pose", "cuda", actor="9.9.9.9")
    assert result["mode"] == "pose"
    assert car.world.containers["car-perception"]["state"] == "running"
    assert car.settings.perception_cache_dir.is_dir()
    assert car.last_log()["action"] == "perception.start" and car.last_log()["ok"]
    assert car.controller.status()["perception"] == {
        "available": True,
        "running": True,
        "reason": "",
    }
    with pytest.raises(ActionError, match="already running"):
        car.controller.start_perception()
    car.controller.stop_perception()
    assert "car-perception" not in car.world.containers
    assert ("docker", "kill", "-s", "INT", "car-perception") in car.runner.calls


def test_perception_refuses_bad_mode_with_a_400_and_a_log_line(car):
    start_cameras(car)
    with pytest.raises(ActionError) as exc:
        car.controller.start_perception("segment", "cuda")
    assert exc.value.status == 400 and car.last_log()["ok"] is False


def test_perception_never_starts_a_driving_launch(car):
    start_cameras(car)
    car.controller.start_perception()
    assert not detached_execs(car)


def test_a_stale_perception_container_is_replaced_and_docker_failures_reported(car):
    start_cameras(car)
    car.world.containers["car-perception"] = {"state": "exited", "cmd": [], "argv": ()}
    car.controller.start_perception()
    assert car.world.containers["car-perception"]["state"] == "running"
    car.controller.stop_perception()
    car.world.docker_run_fails = "no such runtime: nvidia"
    with pytest.raises(ActionError, match="no such runtime"):
        car.controller.start_perception()


# -- spiderbot ----------------------------------------------------------------------------------


@pytest.fixture
def sb(car):
    """A car ready for the spiderbot: built racer_apps, a patrol, cameras, perception, pose."""
    (car.repo / "ros_ws" / "install" / "racer_apps").mkdir()
    car.settings.patrols_dir.mkdir(parents=True)
    (car.settings.patrols_dir / "loop.yaml").write_text("frame: map\n")
    car.world.start_stack(LOC_FLAGS)
    car.controller.start_cameras()
    car.controller.start_perception()
    for _ in range(40):
        car.ros.count("/pose")
    for _ in range(40):
        car.ros.count("/perception/objects")
    return car


def test_spiderbot_is_in_the_drive_modes_and_the_options(sb):
    options = sb.controller.options()
    assert "spiderbot" in options["drive_modes"] and options["patrols"] == ["loop"]
    assert options["spiderbot_rules"] == list(cmd.SPIDERBOT_RULES)
    assert options["spiderbot"] == {"available": True, "reason": ""}


def test_spiderbot_starts_with_the_documented_launch(sb):
    result = sb.controller.start_drive(
        "spiderbot", patrol="loop", rule="arms_raised", speed_mps=0.4
    )
    assert result["mode"] == "spiderbot:loop"
    (call,) = detached_execs(sb)
    assert (
        "ros2 launch racer_apps spiderbot.launch.py patrol:=/workspace/data/patrols/loop.yaml "
        "rule:=arms_raised speed_mps:=0.40 output_dir:=/workspace/data/spiderbot"
    ) in call[-1]
    assert call[:5] == ("docker", "exec", "-d", "car-stack", "bash")  # in the stack, not perception
    entry = sb.last_log()
    assert entry["action"] == "drive.start.spiderbot" and entry["ok"]
    assert entry["params"]["patrol"] == "loop" and entry["params"]["rule"] == "arms_raised"


@pytest.mark.parametrize(
    ("setup", "message"),
    [
        ("no_image", "the spiderbot is disabled: the GPU image car:perception is not built"),
        ("no_build", "racer_apps is not built"),
        ("no_patrol", "no patrol file under data/patrols"),
    ],
)
def test_spiderbot_is_disabled_with_a_clear_reason(sb, setup, message):
    if setup == "no_image":
        sb.world.perception_image_present = False
    elif setup == "no_build":
        (sb.repo / "ros_ws" / "install" / "racer_apps").rmdir()
    else:
        (sb.settings.patrols_dir / "loop.yaml").unlink()
    status = sb.controller.spiderbot_status()
    assert status["available"] is False and message.split(": ", 1)[-1] in status["reason"]
    with pytest.raises(ActionError, match="spiderbot is disabled") as exc:
        sb.controller.start_drive("spiderbot", patrol="loop")
    assert exc.value.status == 409
    assert not detached_execs(sb)


def test_spiderbot_needs_localisation_and_a_live_pose(sb):
    sb.world.start_stack("lidar:=true vesc:=true")  # replaces the stack: no localisation
    with pytest.raises(ActionError, match="needs the stack running with localisation"):
        sb.controller.start_drive("spiderbot", patrol="loop")
    assert not detached_execs(sb)


def test_spiderbot_needs_live_perception(sb):
    sb.controller.stop_perception()
    with pytest.raises(ActionError, match="needs perception running"):
        sb.controller.start_drive("spiderbot", patrol="loop")
    sb.controller.start_perception()
    sb.ros._stamps["/perception/objects"].clear()  # running, but silent
    with pytest.raises(ActionError, match="/perception/objects is silent"):
        sb.controller.start_drive("spiderbot", patrol="loop")
    assert not detached_execs(sb)


@pytest.mark.parametrize(
    "kwargs",
    [
        {"patrol": "../../etc/passwd"},
        {"patrol": "nope"},
        {"patrol": ""},
        {"patrol": "loop", "rule": "arms_raised; reboot"},
        {"patrol": "loop", "rule": "moonwalk"},
        {"patrol": "loop", "speed_mps": 3.0},
        {"patrol": "loop", "speed_mps": -1.0},
    ],
)
def test_spiderbot_refuses_off_list_requests_with_a_400(sb, kwargs):
    with pytest.raises(ActionError) as exc:
        sb.controller.start_drive("spiderbot", **kwargs)
    assert exc.value.status == 400
    assert not detached_execs(sb)


def test_only_patrol_files_the_agent_found_are_offered(sb):
    folder = sb.settings.patrols_dir
    (folder / "bad name.yaml").write_text("x")
    (folder / "notes.txt").write_text("x")
    (folder / "real-link.yaml").symlink_to(folder / "loop.yaml")
    (folder / "sub").mkdir()
    assert sb.controller.patrols() == ["loop"]


# -- the safety rules did not loosen ------------------------------------------------------------


def test_spiderbot_is_refused_without_a_recorder_even_with_an_override(sb):
    sb.world.recorder = False
    with pytest.raises(ActionError, match="invariant 5") as exc:
        sb.controller.start_drive(
            "spiderbot", patrol="loop", override=True, override_reason="in a hurry"
        )
    assert exc.value.data["overridable"] is False
    assert not detached_execs(sb)


def test_spiderbot_is_refused_when_the_stack_is_down_and_on_a_full_disk(sb):
    sb.world.free_gb = 2.0
    with pytest.raises(ActionError) as exc:
        sb.controller.start_drive(
            "spiderbot", patrol="loop", override=True, override_reason="x y z"
        )
    assert exc.value.data["overridable"] is False
    sb.world.free_gb = 100.0
    sb.controller.stop_stack()
    with pytest.raises(ActionError, match="car-stack is not running") as exc2:
        sb.controller.start_drive("spiderbot", patrol="loop")
    assert exc2.value.data["overridable"] is False
    assert not detached_execs(sb)


def test_spiderbot_amber_check_needs_an_override_with_a_reason(sb):
    sb.world.free_gb = 10.0
    with pytest.raises(ActionError, match="not all green"):
        sb.controller.start_drive("spiderbot", patrol="loop")
    sb.controller.start_drive(
        "spiderbot", patrol="loop", override=True, override_reason="bench test"
    )
    assert sb.last_log()["override"] is True


def test_stop_all_stops_a_spiderbot_patrol_and_leaves_perception_alone(sb):
    sb.controller.start_drive("spiderbot", patrol="loop")
    assert sb.controller.stop_all()["was_running"] is True
    assert sb.controller.drive_mode() is None
    assert sb.world.containers["car-perception"]["state"] == "running"


def test_only_one_driving_launch_even_with_the_spiderbot(sb):
    sb.controller.start_drive("spiderbot", patrol="loop")
    with pytest.raises(ActionError, match="already running"):
        sb.controller.start_drive("wall_follow")


# -- the HTTP layer -----------------------------------------------------------------------------


def test_new_routes_need_the_token_and_reject_unknown_fields(sb):
    client = make_client(sb)
    assert client.post("/api/perception/start", json={}).status_code == 401
    assert client.post("/api/perception/stop").status_code == 401
    assert client.post("/api/drive/start", json={"mode": "spiderbot"}).status_code == 401
    bad = client.post("/api/perception/start", headers=AUTH, json={"mode": "pose", "cmd": "x"})
    assert bad.status_code == 422
    bad = client.post("/api/drive/start", headers=AUTH, json={"mode": "spiderbot", "cmd": "x"})
    assert bad.status_code == 422


def test_perception_and_spiderbot_routes_work_and_status_reports_them(sb):
    client = make_client(sb)
    assert client.post("/api/perception/stop", headers=AUTH).status_code == 200
    assert client.get("/api/status", headers=AUTH).json()["perception"]["running"] is False
    assert client.post("/api/perception/start", headers=AUTH, json={}).status_code == 200
    r = client.post(
        "/api/drive/start",
        headers=AUTH,
        json={"mode": "spiderbot", "patrol": "loop", "rule": "one_arm_raised", "speed_mps": 0.3},
    )
    assert r.status_code == 200, r.text
    status = client.get("/api/status", headers=AUTH).json()
    assert status["drive"]["mode"] == "spiderbot:loop" and status["spiderbot"]["available"]
    refused = client.post(
        "/api/drive/start", headers=AUTH, json={"mode": "spiderbot", "patrol": "../x"}
    )
    assert refused.status_code in (400, 409)


def test_disabled_reason_reaches_the_ui_through_status(env):
    env.world.perception_image_present = False
    client = make_client(env)
    status = client.get("/api/status", headers=AUTH).json()
    assert status["perception"]["available"] is False
    assert "car:perception" in status["perception"]["reason"]
    assert status["spiderbot"]["available"] is False
