"""The HTTP layer and the CLI carry the 2026-11-12 session's new fields."""

from __future__ import annotations

from conftest import TOKEN
from fastapi.testclient import TestClient
from racer_agent import cli
from racer_agent.service import create_app

DYN = "dyn-v0-2026-11-12"


def test_the_api_takes_the_new_drive_fields_and_still_rejects_unknown_ones(env):
    env.running_car()
    folder = env.settings.pnp_models_dir / DYN
    folder.mkdir(parents=True)
    (folder / "manifest.yaml").write_text("kind: dynamics\n")
    launch = env.repo / "ros_ws/src/racer_bringup/launch"
    launch.mkdir(parents=True, exist_ok=True)
    (launch / "pnp.launch.py").write_text("")
    (env.settings.pnp_dir / "planner-2026-11-12.yaml").write_text("x: 1\n")

    app = create_app(env.controller, env.settings, web_dist=None, start_background=False)
    client = TestClient(app, client=("192.168.1.50", 40000))
    auth = {"Authorization": f"Bearer {TOKEN}"}
    r = client.post(
        "/api/drive/start",
        headers=auth,
        json={
            "mode": "pnp",
            "pnp_backend": "geometric",
            "pnp_dynamics": DYN,
            "pnp_params": "planner-2026-11-12.yaml",
        },
    )
    assert r.status_code == 200, r.text
    client.post("/api/stop")
    r = client.post(
        "/api/drive/start", headers=auth, json={"mode": "gap_follow", "max_speed_mps": 5.0}
    )
    assert r.status_code == 400 and "max speed" in r.json()["error"]
    r = client.post(
        "/api/drive/start",
        headers=auth,
        json={"mode": "gap_follow", "max_speed_mps": 0.5, "min_speed_mps": 0.6},
    )
    assert r.status_code == 400 and "min speed" in r.json()["error"]
    r = client.post("/api/drive/start", headers=auth, json={"mode": "gap_follow", "max_speed": 1})
    assert r.status_code == 422
    assert client.get("/api/status", headers=auth).json()["drive"]["external"] == []
    assert "gap_follow_speed" in client.get("/api/options", headers=auth).json()


def test_cli_stop_stack_sigints_and_reports(tmp_path, monkeypatch, capsys):
    monkeypatch.setenv("RACER_APP_CONFIG", str(tmp_path / "c.json"))
    monkeypatch.setenv("RACER_APP_STATE_DIR", str(tmp_path / "state"))
    monkeypatch.setenv("RACER_REPO_DIR", str(tmp_path / "car"))
    seen = {}

    class Fake:
        def stop_stack(self, actor=""):
            seen["actor"] = actor
            return {"message": "stack stopped", "bag": "b", "bag_finalized": True}

    monkeypatch.setattr(cli, "build_controller", lambda settings: Fake())
    assert cli.main(["stop-stack"]) == 0
    assert seen["actor"] == "systemd" and "stack stopped" in capsys.readouterr().out

    class Unfinished(Fake):
        def stop_stack(self, actor=""):
            return {
                "message": "stack stopped; bag b has NO metadata.yaml",
                "bag": "b",
                "bag_finalized": False,
            }

    monkeypatch.setattr(cli, "build_controller", lambda settings: Unfinished())
    assert cli.main(["stop-stack"]) == 1
