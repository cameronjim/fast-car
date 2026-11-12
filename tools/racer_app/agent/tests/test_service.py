"""HTTP layer: auth, LAN-only, the whitelist, and one test per route."""

from __future__ import annotations

import pytest
from conftest import TOKEN
from fastapi.testclient import TestClient
from racer_agent.service import FAIL_LIMIT, create_app, is_lan

AUTH = {"Authorization": f"Bearer {TOKEN}"}


def make_client(env, host="192.168.1.50", dist=None):
    app = create_app(env.controller, env.settings, web_dist=dist, start_background=False)
    return TestClient(app, client=(host, 40000))


@pytest.fixture
def client(env):
    return make_client(env)


def test_health_needs_no_token_and_names_the_agent(client):
    r = client.get("/health")
    assert r.status_code == 200
    body = r.json()
    assert body["name"] == "racer-agent" and body["ok"] is True
    assert r.headers["cache-control"] == "no-store"


def test_api_needs_a_token(client):
    for method, path in (("get", "/api/status"), ("post", "/api/stack/stop"), ("get", "/api/log")):
        assert getattr(client, method)(path).status_code == 401
    assert client.get("/api/status", headers={"Authorization": "Bearer nope"}).status_code == 401
    assert client.get("/api/status", headers={"Authorization": TOKEN}).status_code == 401
    assert client.get("/api/status", headers=AUTH).status_code == 200


def test_a_refused_request_never_reaches_docker(env, client):
    client.post("/api/stack/start", json={})
    client.post("/api/drive/start", json={"mode": "wall_follow"})
    assert env.runner.calls == []


def test_repeated_wrong_tokens_lock_the_client_out(client):
    for _ in range(FAIL_LIMIT):
        assert client.get("/api/status", headers={"Authorization": "Bearer x"}).status_code == 401
    assert client.get("/api/status", headers=AUTH).status_code == 429


def test_lockout_is_per_address(env):
    bad = make_client(env, host="192.168.1.60")
    for _ in range(FAIL_LIMIT):
        bad.get("/api/status", headers={"Authorization": "Bearer x"})
    assert make_client(env, host="192.168.1.61").get("/api/status", headers=AUTH).status_code == 200


@pytest.mark.parametrize(
    "host,lan",
    [
        ("192.168.1.5", True),
        ("10.0.0.226", True),
        ("172.16.4.4", True),
        ("127.0.0.1", True),
        ("169.254.3.3", True),
        ("::1", True),
        ("fe80::1", True),
        ("::ffff:192.168.1.5", True),
        ("8.8.8.8", False),
        ("2001:4860:4860::8888", False),
        ("testclient", False),
        ("", False),
    ],
)
def test_lan_only_address_check(host, lan):
    assert is_lan(host) is lan


def test_clients_outside_the_lan_are_refused_everywhere(env):
    outside = make_client(env, host="8.8.8.8")
    for path in ("/health", "/api/status", "/"):
        assert outside.get(path, headers=AUTH).status_code == 403
    assert outside.post("/api/stop").status_code == 403
    assert env.runner.calls == []


def test_stop_all_needs_no_token_but_is_logged_with_the_address(env, client):
    env.running_car()
    r = client.post("/api/stop")
    assert r.status_code == 200 and r.json()["ok"] is True
    assert env.last_log()["actor"] == "192.168.1.50" and env.last_log()["action"] == "stop_all"


def test_docs_and_openapi_are_off(client):
    assert client.get("/docs").status_code == 404
    assert client.get("/openapi.json").status_code == 404


def test_unknown_routes_and_methods(client):
    assert client.post("/api/shell", headers=AUTH, json={"cmd": "reboot"}).status_code == 404
    assert client.get("/api/stack/start", headers=AUTH).status_code == 405
    assert client.post("/api/exec", headers=AUTH, json={}).status_code == 404


def test_unknown_body_fields_are_rejected(env, client):
    r = client.post("/api/drive/start", headers=AUTH, json={"mode": "wall_follow", "cmd": "reboot"})
    assert r.status_code == 422
    r = client.post(
        "/api/stack/start", headers=AUTH, json={"lidar": True, "docker_args": ["--privileged"]}
    )
    assert r.status_code == 422
    assert env.runner.calls == []


def test_status_options_log_bags_and_drive_log(env, client):
    env.running_car()
    assert client.get("/api/status", headers=AUTH).json()["stack"]["running"] is True
    assert "gap_follow_profiles" in client.get("/api/options", headers=AUTH).json()
    assert client.get("/api/log?limit=5", headers=AUTH).json()["entries"] == []
    assert client.get("/api/bags", headers=AUTH).json()["bags"] == []
    assert client.get("/api/vehicle", headers=AUTH).json()["lidar"]["mount_x_m"] is None
    assert "launch said" in client.get("/api/drive/log", headers=AUTH).json()["log"]


def test_log_limit_is_clamped(env, client):
    for i in range(3):
        env.actions.add("x", True, detail=str(i))
    assert len(client.get("/api/log?limit=0", headers=AUTH).json()["entries"]) == 1
    assert len(client.get("/api/log?limit=9999", headers=AUTH).json()["entries"]) == 3


def test_stack_routes(env, client):
    r = client.post("/api/stack/start", headers=AUTH, json={"lidar": True, "vesc": True})
    assert r.status_code == 200 and r.json()["ok"]
    assert env.world.containers["car-stack"]["state"] == "running"
    r = client.post("/api/stack/start", headers=AUTH, json={})
    assert r.status_code == 409 and "already running" in r.json()["error"]
    r = client.post("/api/stack/restart", headers=AUTH, json={})
    assert r.status_code == 200
    r = client.post("/api/stack/restart", headers=AUTH, json={"flags": {"mapping": True}})
    assert r.status_code == 200
    assert client.post("/api/stack/stop", headers=AUTH).status_code == 200
    assert "car-stack" not in env.world.containers


def test_bad_stack_flags_are_a_400(client):
    r = client.post("/api/stack/start", headers=AUTH, json={"mapping": True, "localization": True})
    assert r.status_code == 400


def test_camera_routes(env, client):
    assert client.post("/api/cameras/start", headers=AUTH).status_code == 200
    assert client.post("/api/cameras/start", headers=AUTH).status_code == 409
    assert client.post("/api/cameras/stop", headers=AUTH).status_code == 200


def test_drive_route_and_override_error_shape(env, client):
    env.running_car()
    env.world.free_gb = 10.0
    r = client.post("/api/drive/start", headers=AUTH, json={"mode": "wall_follow"})
    assert r.status_code == 409
    assert r.json()["overridable"] is True and r.json()["failing"] == ["disk"]
    r = client.post(
        "/api/drive/start",
        headers=AUTH,
        json={"mode": "wall_follow", "override": True, "override_reason": "bench"},
    )
    assert r.status_code == 200 and r.json()["override"] is True
    assert client.post("/api/stop").json()["was_running"] is True


def test_drive_route_refuses_without_a_recorder(env, client):
    env.running_car()
    env.world.recorder = False
    r = client.post(
        "/api/drive/start",
        headers=AUTH,
        json={
            "mode": "gap_follow",
            "profile": "lane-2026-10-25",
            "override": True,
            "override_reason": "pls",
        },
    )
    assert r.status_code == 409 and r.json()["overridable"] is False


def test_ml_policy_route_says_why_not(env, client):
    env.running_car()
    r = client.post("/api/drive/start", headers=AUTH, json={"mode": "ml_policy"})
    assert r.status_code == 409 and "ML policy is disabled" in r.json()["error"]


def test_pnp_route_says_why_not(env, client):
    env.running_car()
    body = {"mode": "pnp", "pnp_backend": "geometric"}
    r = client.post("/api/drive/start", headers=AUTH, json=body)
    assert r.status_code == 409 and "Plug-and-Play is disabled" in r.json()["error"]


def test_park_route(env, client):
    env.running_car()
    client.post("/api/drive/start", headers=AUTH, json={"mode": "park"})
    assert client.post("/api/park/start", headers=AUTH, json={}).status_code == 200
    assert client.post("/api/park/abort", headers=AUTH, json={}).status_code == 200
    assert client.post("/api/park/fly", headers=AUTH, json={}).status_code == 400


def test_map_route(env, client):
    env.world.start_stack("lidar:=true vesc:=true mapping:=true")
    assert client.post("/api/map/save", headers=AUTH, json={"name": "floor-1"}).status_code == 200
    assert client.post("/api/map/save", headers=AUTH, json={"name": "a b"}).status_code == 400


def test_bag_route(env, client):
    folder = env.settings.bags / "b1"
    folder.mkdir(parents=True)
    (folder / "x.mcap").write_bytes(b"1")
    assert client.get("/api/bags", headers=AUTH).json()["bags"][0]["name"] == "b1"
    bad = client.post("/api/bags/delete", headers=AUTH, json={"name": "b1", "confirm": "no"})
    assert bad.status_code == 400
    ok = client.post("/api/bags/delete", headers=AUTH, json={"name": "b1", "confirm": "b1"})
    assert ok.status_code == 200 and not folder.exists()


def test_power_routes_need_confirmation(env, client):
    assert (
        client.post("/api/system/reboot", headers=AUTH, json={"confirm": "no"}).status_code == 400
    )
    assert (
        client.post("/api/system/reboot", headers=AUTH, json={"confirm": "reboot"}).status_code
        == 200
    )
    assert (
        client.post("/api/system/shutdown", headers=AUTH, json={"confirm": "shutdown"}).status_code
        == 200
    )
    assert len(env.deferred) == 2


def test_settings_route(env, client):
    assert client.post("/api/settings", headers=AUTH, json={"autostart": True}).json()["autostart"]
    assert env.controller.autostart_enabled()
    assert (
        client.post("/api/settings", headers=AUTH, json={"autostart": "maybe"}).status_code == 422
    )


def test_every_host_side_command_is_docker_systemctl_or_narrow_sudo(env, client):
    """Whitelist: whatever the client sends, the host only ever runs these programs."""
    env.running_car()
    env.world.start_stack("lidar:=true vesc:=true mapping:=true")
    calls = [
        ("post", "/api/stack/restart", {}),
        ("post", "/api/cameras/start", None),
        ("post", "/api/drive/start", {"mode": "park"}),
        ("post", "/api/park/start", {}),
        ("post", "/api/map/save", {"name": "m1"}),
        ("post", "/api/stop", None),
        ("post", "/api/system/reboot", {"confirm": "reboot"}),
    ]
    for method, path, body in calls:
        getattr(client, method)(path, headers=AUTH, **({"json": body} if body is not None else {}))
    for _, go in env.deferred:
        go()
    programs = {c[0] for c in env.runner.calls}
    assert programs <= {"docker", "systemctl", "sudo"}
    for c in env.runner.calls:
        if c[0] == "sudo":
            assert c in (
                ("sudo", "-n", "/usr/bin/systemctl", "reboot"),
                ("sudo", "-n", "/usr/bin/systemctl", "poweroff"),
            )
        if c[0] == "systemctl":
            assert c == ("systemctl", "is-active", "racer-heartbeat.service")
        if c[0] == "docker":
            assert c[1] in {"ps", "inspect", "image", "run", "rm", "kill", "top", "logs", "exec"}
            assert "--privileged" not in c and "--pid" not in c and "--cap-add" not in c


def test_static_ui_is_served_when_built(env, tmp_path):
    dist = tmp_path / "dist"
    dist.mkdir()
    (dist / "index.html").write_text("<title>ui</title>")
    c = make_client(env, dist=dist)
    assert "<title>ui</title>" in c.get("/").text
    assert c.get("/health").json()["ok"] is True
    assert c.get("/api/status").status_code == 401  # API routes win over the static mount


def test_placeholder_page_without_a_build(client):
    r = client.get("/")
    assert r.status_code == 200 and "web UI has not been built" in r.text
