"""The racer app v3 additions: saved drive settings, the camera recording toggle, disk usage,
bag delete over HTTP, docker cleanup, and gap follow's two physics tuning values."""

from __future__ import annotations

import json
import os
import re
from pathlib import Path

import pytest
from conftest import TOKEN
from fastapi.testclient import TestClient
from racer_agent import commands as cmd
from racer_agent.controller import ActionError
from racer_agent.prefs import Prefs, PrefsStore
from racer_agent.service import create_app

AUTH = {"Authorization": f"Bearer {TOKEN}"}
REPO_ROOT = Path(__file__).resolve().parents[4]
GAP_LAUNCH = REPO_ROOT / "ros_ws/src/racer_control/launch/gap_follow.launch.py"
CAMERAS_LAUNCH = REPO_ROOT / "ros_ws/src/racer_bringup/launch/cameras.launch.py"


@pytest.fixture
def client(env):
    app = create_app(env.controller, env.settings, start_background=False)
    return TestClient(app, client=("192.168.1.50", 40000))


def make_bag(env, name: str, size: int = 1000, age: float = 0.0) -> Path:
    folder = env.settings.bags / name
    folder.mkdir(parents=True)
    (folder / "a.mcap").write_bytes(b"x" * size)
    stamp = env.now - age
    os.utime(folder, (stamp, stamp))
    return folder


# -- saved settings ---------------------------------------------------------------------------


def test_prefs_default_when_nothing_is_saved(client):
    body = client.get("/api/prefs", headers=AUTH).json()
    assert body["prefs"] == body["defaults"]
    assert body["prefs"]["record_camera"] is True
    assert body["prefs"]["gap_follow"]["max_speed_mps"] is None


def test_prefs_need_the_token(client):
    assert client.get("/api/prefs").status_code == 401
    assert client.post("/api/prefs", json={}).status_code == 401


def test_prefs_round_trip_and_survive_a_restart(env, client):
    want = Prefs().model_dump()
    want["record_camera"] = False
    want["gap_follow"].update(
        {"profile": "race-2026-11-13", "max_speed_mps": 1.0, "min_speed_mps": 0.4}
    )
    want["gap_follow"].update({"a_lat_max_mps2": 1.8, "side_cap_min_speed_mps": 0.3})
    want["pnp"].update({"v_max_mps": 0.7, "backend": "net", "dynamics": "dyn-v1"})
    r = client.post("/api/prefs", headers=AUTH, json=want)
    assert r.status_code == 200, r.text
    assert client.get("/api/prefs", headers=AUTH).json()["prefs"] == want
    # a new store on the same file (an agent restart) reads the same values
    assert PrefsStore(env.settings.prefs_path).load().model_dump() == want
    assert env.last_log()["action"] == "prefs.save"


@pytest.mark.parametrize(
    "patch",
    [
        {"gap_follow": {"profile": "fastest"}},
        {"gap_follow": {"max_speed_mps": 9.0}},
        {"gap_follow": {"max_speed_mps": 0.5, "min_speed_mps": 0.6}},
        {"gap_follow": {"min_speed_mps": -0.1}},
        {"gap_follow": {"a_lat_max_mps2": 0.0}},
        {"gap_follow": {"a_lat_max_mps2": 25.0}},
        {"gap_follow": {"side_cap_min_speed_mps": 3.0}},
        {"pnp": {"v_max_mps": 3.0}},
        {"pnp": {"backend": "magic"}},
        {"pnp": {"dynamics": "../../etc"}},
        {"pnp": {"params_file": "x.sh"}},
        {"stack": {"mode": "localization", "map": ""}},
        {"stack": {"mode": "teleop", "map": "nope.yaml"}},
        {"tracker": {"raceline": "track:nowhere"}},
    ],
)
def test_prefs_out_of_bounds_are_refused_and_not_saved(env, client, patch):
    body = Prefs().model_dump()
    for section, values in patch.items():
        body[section].update(values)
    r = client.post("/api/prefs", headers=AUTH, json=body)
    assert r.status_code == 400, r.text
    assert not env.settings.prefs_path.exists()


def test_prefs_reject_unknown_fields_and_wrong_types(env, client):
    body = Prefs().model_dump()
    body["docker_args"] = ["--privileged"]
    assert client.post("/api/prefs", headers=AUTH, json=body).status_code == 422
    body = Prefs().model_dump()
    body["gap_follow"]["command"] = "reboot"
    assert client.post("/api/prefs", headers=AUTH, json=body).status_code == 422
    body = Prefs().model_dump()
    body["stack"]["mode"] = "anything"
    assert client.post("/api/prefs", headers=AUTH, json=body).status_code == 422
    assert not env.settings.prefs_path.exists()


def test_prefs_accept_a_known_map_and_raceline(env, client):
    env.settings.maps.mkdir(parents=True)
    (env.settings.maps / "lab.yaml").write_text("image: lab.pgm\n")
    body = Prefs().model_dump()
    body["stack"] = {"mode": "localization", "map": "lab.yaml"}
    body["tracker"] = {"raceline": "track:gym_oval"}
    assert client.post("/api/prefs", headers=AUTH, json=body).status_code == 200


def test_a_corrupt_prefs_file_falls_back_to_defaults(env):
    env.settings.prefs_path.parent.mkdir(parents=True, exist_ok=True)
    env.settings.prefs_path.write_text("{not json")
    assert env.controller.get_prefs()["prefs"] == Prefs().model_dump()
    env.settings.prefs_path.write_text(json.dumps({"record_camera": "yes please", "x": 1}))
    assert env.controller.get_prefs()["prefs"] == Prefs().model_dump()


# -- record camera toggle -----------------------------------------------------------------------


def _camera_launch(env) -> str:
    return env.world.containers["car-camera"]["argv"][-1]


def test_cameras_record_by_default(env):
    env.controller.start_cameras()
    assert "record_streams" not in _camera_launch(env)
    assert "camera recorded in the bag" in env.last_log()["detail"]


def test_record_camera_off_leaves_the_camera_stream_out_of_the_bag(env):
    prefs = Prefs(record_camera=False)
    env.controller.set_prefs(prefs)
    reply = env.controller.start_cameras()
    assert reply["record"] is False
    assert _camera_launch(env).endswith(
        "cameras.launch.py usb_device:=/dev/video1 record_streams:=false"
    )
    assert "NOT recorded" in env.last_log()["detail"]


def test_record_camera_never_touches_the_stack_recorder(env):
    """Invariant 5: the stack's bag (LiDAR, drive, telemetry, rail voltage) always records."""
    env.controller.set_prefs(Prefs(record_camera=False))
    env.controller.start_stack(cmd.StackFlags())
    launch = env.world.containers["car-stack"]["argv"][-1]
    assert "record" not in launch.split("car_teleop.launch.py", 1)[1]
    assert "record" not in Prefs.model_fields["stack"].annotation.model_fields


def test_status_reports_the_running_camera_setting(env, client):
    assert client.get("/api/status", headers=AUTH).json()["cameras"] == {
        "running": False,
        "record": None,
    }
    env.controller.set_prefs(Prefs(record_camera=False))
    env.controller.start_cameras()
    env.controller._invalidate()
    cams = client.get("/api/status", headers=AUTH).json()["cameras"]
    assert cams == {"running": True, "record": False}
    # changing the setting does not pretend the running container changed
    env.controller.set_prefs(Prefs(record_camera=True))
    env.controller._invalidate()
    assert client.get("/api/status", headers=AUTH).json()["cameras"]["record"] is False


@pytest.mark.skipif(not CAMERAS_LAUNCH.exists(), reason="needs the repo's launch files")
def test_record_streams_is_an_argument_of_the_cameras_launch():
    assert '"record_streams"' in CAMERAS_LAUNCH.read_text()


# -- disk usage, bag list and delete over HTTP -------------------------------------------------


def test_bags_route_reports_disk_usage_and_the_copy_hint(env, client):
    make_bag(env, "b1", size=2_000_000, age=100)
    make_bag(env, "b2", size=500_000, age=10)
    body = client.get("/api/bags", headers=AUTH).json()
    assert [b["name"] for b in body["bags"]] == ["b2", "b1"]
    assert body["bags_mb"] == 2.5
    assert body["disk"] == {"total_gb": 200.0, "used_gb": 100.0, "free_gb": 100.0}
    assert body["bag_dir"] == str(env.settings.bags)
    assert isinstance(body["user"], str)


@pytest.mark.parametrize("name", ["..", "../car", "a/b", "../../etc/passwd", ".hidden", ""])
def test_bag_delete_route_refuses_paths_outside_the_bag_dir(env, client, name):
    make_bag(env, "real")
    r = client.post("/api/bags/delete", headers=AUTH, json={"name": name, "confirm": name})
    assert r.status_code in (400, 404)
    assert (env.settings.bags / "real").exists()
    assert env.repo.exists()


def test_bag_delete_route_refuses_the_bag_being_recorded(env, client):
    env.running_car()
    make_bag(env, "old", age=1000)
    make_bag(env, "new", age=10)
    r = client.post("/api/bags/delete", headers=AUTH, json={"name": "new", "confirm": "new"})
    assert r.status_code == 409
    assert (env.settings.bags / "new").exists()
    r = client.post("/api/bags/delete", headers=AUTH, json={"name": "old", "confirm": "old"})
    assert r.status_code == 200 and not (env.settings.bags / "old").exists()


def test_bag_delete_route_needs_the_token(env, client):
    make_bag(env, "b1")
    r = client.post("/api/bags/delete", json={"name": "b1", "confirm": "b1"})
    assert r.status_code == 401 and (env.settings.bags / "b1").exists()


# -- docker cleanup ---------------------------------------------------------------------------


def test_cleanup_needs_confirmation_and_runs_nothing_without_it(env, client):
    r = client.post("/api/system/cleanup", headers=AUTH, json={"confirm": "yes"})
    assert r.status_code == 400
    assert env.runner.calls_matching("docker", "image", "prune") == []
    assert client.post("/api/system/cleanup", json={"confirm": "cleanup"}).status_code == 401


def test_cleanup_prunes_dangling_images_and_build_cache_only(env, client):
    r = client.post("/api/system/cleanup", headers=AUTH, json={"confirm": "cleanup"})
    assert r.status_code == 200, r.text
    body = r.json()
    assert body["message"] == "reclaimed 1.2GB of images, 640MB of build cache"
    assert "$ docker image prune -f" in body["output"]
    prunes = [c for c in env.runner.calls if "prune" in c]
    assert prunes == [("docker", "image", "prune", "-f"), ("docker", "builder", "prune", "-f")]
    assert all("-a" not in c and "--all" not in c for c in prunes)
    assert env.last_log()["action"] == "system.cleanup" and env.last_log()["ok"] is True


def test_a_failed_cleanup_says_why(env):
    env.world.prune_fails = "Cannot connect to the Docker daemon"
    with pytest.raises(ActionError) as exc:
        env.controller.cleanup_docker("cleanup")
    assert exc.value.status == 500 and "Docker daemon" in exc.value.message
    assert env.last_log()["ok"] is False


# -- gap follow tuning ------------------------------------------------------------------------


def test_gap_follow_tuning_values_reach_the_launch(env):
    env.running_car()
    env.controller.start_drive(
        "gap_follow",
        profile="race-2026-11-13",
        max_speed_mps=1.0,
        a_lat_max_mps2=1.8,
        side_cap_min_speed_mps=0.3,
    )
    assert env.world.drive_cmd == (
        "ros2 launch racer_control gap_follow.launch.py profile:=race-2026-11-13 "
        "max_speed_mps:=1.00 a_lat_max_mps2:=1.80 side_cap_min_speed_mps:=0.30"
    )
    assert "a_lat_max_mps2=1.80" in env.last_log()["detail"]


@pytest.mark.parametrize(
    "kw,match",
    [
        ({"a_lat_max_mps2": 0.05}, "lateral acceleration"),
        ({"a_lat_max_mps2": True}, "lateral acceleration"),
        ({"side_cap_min_speed_mps": 2.0}, "side cap min speed"),
        ({"max_speed_mps": 0.5, "side_cap_min_speed_mps": 0.6}, "side cap min speed"),
    ],
)
def test_gap_follow_tuning_out_of_bounds_is_refused_before_docker(env, kw, match):
    env.running_car()
    with pytest.raises(ActionError, match=match) as exc:
        env.controller.start_drive("gap_follow", profile="race-2026-11-13", **kw)
    assert exc.value.status == 400
    assert env.runner.calls_matching("docker", "exec", "-d") == []


def test_gap_follow_tuning_is_only_for_gap_follow(env):
    env.running_car()
    with pytest.raises(ActionError, match="only used with gap follow"):
        env.controller.start_drive("wall_follow", a_lat_max_mps2=2.0)


def test_drive_route_accepts_the_tuning_fields(env, client):
    env.running_car()
    r = client.post(
        "/api/drive/start",
        headers=AUTH,
        json={"mode": "gap_follow", "profile": "race-2026-11-13", "a_lat_max_mps2": 2.5},
    )
    assert r.status_code == 200, r.text
    assert "a_lat_max_mps2:=2.50" in env.world.drive_cmd


@pytest.mark.skipif(not GAP_LAUNCH.exists(), reason="needs the repo's launch files")
def test_gap_follow_tuning_args_exist_in_the_launch_file():
    text = GAP_LAUNCH.read_text()
    for name in ("a_lat_max_mps2", "side_cap_min_speed_mps", "min_speed_mps"):
        assert f'"{name}": (' in text
    low, high = cmd.GAP_FOLLOW_A_LAT_RANGE
    assert re.search(rf"\[{low:g}, {high:g}\]", text)
