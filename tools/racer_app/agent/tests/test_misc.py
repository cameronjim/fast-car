"""Config and token, action log, host info, disk guard and the command line."""

from __future__ import annotations

import json
import os
import stat

import pytest
from fakes import World, make_host
from racer_agent import cli, config, diskguard
from racer_agent.actionlog import ActionLog
from racer_agent.config import Settings, Thresholds
from racer_agent.hostinfo import GIB, HostInfo

T = Thresholds()

# -- config and token ------------------------------------------------------------------------


def test_token_is_created_once_stored_hashed_and_private(tmp_path):
    path = tmp_path / "etc" / "config.json"
    token = config.write_token(path)
    assert token and len(token) >= 30
    raw = json.loads(path.read_text())
    assert raw["token_sha256"] == config.hash_token(token)
    assert token not in path.read_text()
    assert stat.S_IMODE(os.stat(path).st_mode) == 0o600
    assert config.write_token(path) is None  # idempotent: the saved token stays valid
    assert json.loads(path.read_text())["token_sha256"] == config.hash_token(token)


def test_rotate_replaces_the_token_and_keeps_other_settings(tmp_path):
    path = tmp_path / "config.json"
    path.write_text(json.dumps({"port": 9000, "token_sha256": config.hash_token("old")}))
    new = config.write_token(path, rotate=True)
    raw = json.loads(path.read_text())
    assert new != "old" and raw["port"] == 9000
    assert raw["token_sha256"] == config.hash_token(new)


def test_token_matching():
    s = Settings(token_sha256=config.hash_token("secret"))
    assert config.token_matches(s, "secret")
    assert not config.token_matches(s, "Secret")
    assert not config.token_matches(s, "")
    assert not config.token_matches(s, None)
    assert not config.token_matches(Settings(), "anything")  # no token configured: nobody gets in


def test_load_settings_defaults_and_overrides(tmp_path, monkeypatch):
    monkeypatch.delenv("RACER_REPO_DIR", raising=False)
    monkeypatch.delenv("RACER_APP_STATE_DIR", raising=False)
    path = tmp_path / "config.json"
    defaults = config.load_settings(path)  # missing file
    assert defaults.port == 8080 and defaults.token_sha256 == ""
    assert defaults.thresholds == Thresholds()
    path.write_text(
        json.dumps(
            {
                "repo_dir": str(tmp_path / "repo"),
                "port": 9001,
                "bag_dir": str(tmp_path / "bags"),
                "map_dir": str(tmp_path / "maps"),
                "token_sha256": "abc",
                "foxglove_url": "ws://x:1",
                "thresholds": {"battery_cells": 4, "not_a_threshold": 1},
            }
        )
    )
    s = config.load_settings(path)
    assert s.port == 9001 and s.bags == tmp_path / "bags" and s.maps == tmp_path / "maps"
    assert s.thresholds.battery_cells == 4 and s.foxglove_url == "ws://x:1"
    monkeypatch.setenv("RACER_REPO_DIR", str(tmp_path / "other"))
    assert config.load_settings(path).repo_dir == tmp_path / "other"


def test_settings_derived_paths(tmp_path):
    s = Settings(repo_dir=tmp_path / "car", state_dir=tmp_path / "st")
    assert s.bags == tmp_path / "car" / "data" / "bags"
    assert s.maps == tmp_path / "car" / "data" / "maps"
    assert s.action_log_path == tmp_path / "st" / "actions.jsonl"
    assert s.autostart_flag.name == "autostart.enabled"


def test_config_path_env(monkeypatch, tmp_path):
    monkeypatch.setenv("RACER_APP_CONFIG", str(tmp_path / "c.json"))
    assert config.config_path() == tmp_path / "c.json"


# -- action log ------------------------------------------------------------------------------


def test_action_log_round_trip_newest_first(tmp_path):
    t = [100.0]
    log = ActionLog(tmp_path / "d" / "a.jsonl", clock=lambda: t[0])
    for i in range(5):
        t[0] += 1
        log.add(f"a{i}", ok=i % 2 == 0, actor="1.1.1.1", detail=str(i), params={"i": i})
    entries = log.tail(3)
    assert [e["action"] for e in entries] == ["a4", "a3", "a2"]
    assert entries[0]["params"] == {"i": 4} and entries[0]["ts"] == 105.0


def test_action_log_records_overrides(tmp_path):
    log = ActionLog(tmp_path / "a.jsonl")
    log.add("drive", True, override=True, override_reason="bench")
    assert log.tail(1)[0]["override_reason"] == "bench"


def test_action_log_is_capped_and_survives_corruption(tmp_path):
    log = ActionLog(tmp_path / "a.jsonl", max_bytes=2000)
    for i in range(200):
        log.add("x", True, detail=str(i))
    size = (tmp_path / "a.jsonl").stat().st_size
    assert size < 4000
    assert log.tail(1)[0]["detail"] == "199"
    with (tmp_path / "a.jsonl").open("a") as fh:
        fh.write("{broken\n")
    log.add("after", True)
    assert log.tail(1)[0]["action"] == "after"
    assert all("action" in e for e in log.tail(500))


def test_action_log_tail_of_missing_file(tmp_path):
    log = ActionLog(tmp_path / "a.jsonl")
    assert log.tail() == []
    (tmp_path / "a.jsonl").unlink(missing_ok=True)


# -- host info -------------------------------------------------------------------------------


def test_thermal_wifi_uptime_and_devices(tmp_path):
    host = make_host(tmp_path, World(tmp_path))
    assert host.thermal_zones() == {"cpu-thermal": 48.0}
    assert host.uptime_s() == 5000.0
    assert host.exists("/dev/ttyUSB0") and not host.exists("/dev/nothing")
    assert host.usb_camera_node() == "/dev/video1"
    assert [d.index for d in host.video_devices()] == [0, 1]


def test_thermal_ignores_unreadable_zones(tmp_path):
    host = make_host(tmp_path, World(tmp_path))
    bad = host.path("/sys/class/thermal/thermal_zone1")
    bad.mkdir(parents=True)
    (bad / "type").write_text("gpu-thermal\n")
    (bad / "temp").write_text("garbage\n")
    assert host.thermal_zones() == {"cpu-thermal": 48.0}
    assert HostInfo(root=tmp_path / "empty").thermal_zones() == {}


def wireless(tmp_path, level):
    host = HostInfo(root=tmp_path)
    (tmp_path / "proc/net").mkdir(parents=True)
    (tmp_path / "proc/net/wireless").write_text(
        "Inter-| sta-|   Quality        |   Discarded packets\n"
        " face | tus | link level noise |  nwid\n"
        f"wlan0: 0000   60.  {level}.  -256.        0\n"
    )
    return host


@pytest.mark.parametrize("raw,dbm", [("-52", -52.0), ("204", -52.0), ("40", -70.0)])
def test_wifi_signal_formats(tmp_path, raw, dbm):
    assert wireless(tmp_path, raw).wifi_signal_dbm() == dbm


def test_wifi_absent(tmp_path):
    assert HostInfo(root=tmp_path).wifi_signal_dbm() is None
    (tmp_path / "proc/net").mkdir(parents=True)
    (tmp_path / "proc/net/wireless").write_text("header\nheader\n")
    assert HostInfo(root=tmp_path).wifi_signal_dbm() is None


def test_uptime_unreadable(tmp_path):
    assert HostInfo(root=tmp_path).uptime_s() is None
    (tmp_path / "proc").mkdir()
    (tmp_path / "proc/uptime").write_text("junk")
    assert HostInfo(root=tmp_path).uptime_s() is None


def test_usb_camera_prefers_the_capture_node_and_handles_none(tmp_path):
    host = HostInfo(root=tmp_path)
    assert host.usb_camera_node() is None
    for n, idx in ((2, 1), (3, 0)):
        d = tmp_path / f"sys/class/video4linux/video{n}"
        d.mkdir(parents=True)
        (d / "name").write_text("ELP Global Shutter\n")
        (d / "index").write_text(f"{idx}\n")
    (tmp_path / "sys/class/video4linux/video0").mkdir()
    (tmp_path / "sys/class/video4linux/video0/name").write_text("vi-output, imx219\n")
    (tmp_path / "sys/class/video4linux/videoX").mkdir()
    assert host.usb_camera_node() == "/dev/video3"
    only_meta = HostInfo(root=tmp_path)
    (tmp_path / "sys/class/video4linux/video3/index").write_text("not a number\n")
    assert only_meta.usb_camera_node() == "/dev/video3"


def test_disk_numbers_use_the_nearest_existing_parent(tmp_path):
    seen = []

    def usage(path):
        seen.append(path)
        return 100 * GIB, 60 * GIB, 40 * GIB

    host = HostInfo(disk_usage=usage)
    assert host.disk_free_gb(tmp_path / "does" / "not" / "exist") == 40.0
    assert host.disk_total_gb(tmp_path) == 100.0
    assert seen[0] == str(tmp_path)

    def broken(path):
        raise OSError("gone")

    assert HostInfo(disk_usage=broken).disk_free_gb(tmp_path) is None
    assert HostInfo(disk_usage=broken).disk_total_gb(tmp_path) is None


def test_real_host_defaults_work():
    host = HostInfo()
    assert host.uid == os.getuid() and host.gid == os.getgid()
    assert host.group_id("definitely-not-a-group") is None
    assert host.disk_free_gb(os.getcwd()) > 0


# -- disk guard ------------------------------------------------------------------------------


@pytest.mark.parametrize(
    "free,cams,drive,stack,expected",
    [
        (50.0, True, True, True, []),
        (7.0, True, True, True, []),  # exactly at the threshold: not below
        (6.9, True, True, True, ["stop_cameras"]),
        (6.9, False, True, True, []),
        (2.9, True, True, True, ["stop_cameras", "stop_drive"]),
        (2.9, False, False, True, []),
        (1.4, True, True, True, ["stop_cameras", "stop_drive", "stop_stack"]),
        (1.4, False, False, True, ["stop_stack"]),
        (1.4, False, False, False, []),
        (None, True, True, True, []),
    ],
)
def test_guard_decisions(free, cams, drive, stack, expected):
    assert diskguard.decide(free, T, cams, drive, stack) == expected


def guard_for(env):
    return diskguard.DiskGuard(env.controller)


def test_guard_stops_cameras_first_and_logs_it(env):
    env.running_car()
    env.controller.start_cameras()
    env.world.free_gb = 6.0
    assert guard_for(env).tick() == ["stop_cameras"]
    assert "car-camera" not in env.world.containers
    assert env.world.containers["car-stack"]["state"] == "running"
    entry = env.last_log()
    assert entry["action"] == "guard.stop_cameras" and entry["actor"] == "disk-guard"


def test_guard_stops_the_drive_but_keeps_the_stack_recording(env):
    env.running_car()
    env.controller.start_drive("wall_follow")
    env.world.free_gb = 2.5
    assert guard_for(env).tick() == ["stop_drive"]
    assert env.world.drive_label is None
    assert env.world.containers["car-stack"]["state"] == "running"


def test_guard_shuts_the_stack_down_cleanly_when_nearly_full(env):
    env.running_car()
    env.world.free_gb = 1.0
    assert guard_for(env).tick() == ["stop_stack"]
    assert ("docker", "kill", "-s", "INT", "car-stack") in env.runner.calls


def test_guard_does_nothing_when_there_is_room(env):
    env.running_car()
    assert guard_for(env).tick() == []


def test_guard_background_loop_runs_and_stops(env):
    env.running_car()
    env.world.free_gb = 1.0
    g = diskguard.DiskGuard(env.controller, interval_s=0.01)
    g.start()
    g.start()
    import time

    for _ in range(100):
        if "car-stack" not in env.world.containers:
            break
        time.sleep(0.02)
    g.stop()
    assert "car-stack" not in env.world.containers


def test_guard_survives_a_failing_tick(env, monkeypatch):
    g = diskguard.DiskGuard(env.controller, interval_s=0.01)
    calls = []

    def boom():
        calls.append(1)
        raise RuntimeError("docker exploded")

    monkeypatch.setattr(g, "tick", boom)
    g.start()
    import time

    time.sleep(0.15)
    g.stop()
    assert len(calls) >= 2


# -- command line ----------------------------------------------------------------------------


def test_cli_init_prints_the_token_once(tmp_path, monkeypatch, capsys):
    monkeypatch.setenv("RACER_APP_CONFIG", str(tmp_path / "c.json"))
    assert cli.main(["init"]) == 0
    first = capsys.readouterr().out
    token = first.split("save it now):")[1].split()[0]
    assert config.token_matches(config.load_settings(tmp_path / "c.json"), token)
    assert cli.main(["init"]) == 0
    second = capsys.readouterr().out
    assert token not in second and "left unchanged" in second
    assert cli.main(["init", "--rotate-token"]) == 0
    assert token not in capsys.readouterr().out.split("save it now):")[1]


def test_cli_serve_refuses_without_a_token(tmp_path, monkeypatch, capsys):
    monkeypatch.setenv("RACER_APP_CONFIG", str(tmp_path / "none.json"))
    assert cli.main(["serve"]) == 2
    assert "racer-agent init" in capsys.readouterr().err


def test_cli_serve_starts_uvicorn_with_the_app(tmp_path, monkeypatch):
    path = tmp_path / "c.json"
    monkeypatch.setenv("RACER_APP_CONFIG", str(path))
    monkeypatch.setenv("RACER_APP_STATE_DIR", str(tmp_path / "state"))
    monkeypatch.setenv("RACER_REPO_DIR", str(tmp_path / "car"))
    config.write_token(path)
    started = {}
    import uvicorn

    monkeypatch.setattr(uvicorn, "run", lambda app, **kw: started.update(app=app, **kw))
    assert cli.main(["serve"]) == 0
    assert started["port"] == 8080 and started["host"] == "0.0.0.0"
    assert started["app"].title == "racer-agent"


def test_cli_autostart(tmp_path, monkeypatch, capsys):
    path = tmp_path / "c.json"
    monkeypatch.setenv("RACER_APP_CONFIG", str(path))
    monkeypatch.setenv("RACER_APP_STATE_DIR", str(tmp_path / "state"))
    monkeypatch.setenv("RACER_REPO_DIR", str(tmp_path / "car"))
    assert cli.main(["autostart"]) == 0  # flag not set: does nothing
    assert "autostart is off" in capsys.readouterr().out
    (tmp_path / "state" / "autostart.enabled").touch()
    monkeypatch.setattr("racer_agent.cli.Runner", lambda: _NoDocker())
    original = cli.build_controller

    def fast(settings):
        controller = original(settings)
        controller._sleep = lambda seconds: None  # do not really wait for the devices
        return controller

    monkeypatch.setattr(cli, "build_controller", fast)
    assert cli.main(["autostart"]) == 1  # docker missing: refuses, exit code 1
    assert "autostart refused" in capsys.readouterr().err


class _NoDocker:
    def run(self, argv, timeout=30.0):
        from racer_agent.runner import CmdResult

        return CmdResult(tuple(argv), 127, "", "no docker")


def test_web_dist_location(tmp_path, monkeypatch):
    s = Settings(repo_dir=tmp_path)
    monkeypatch.delenv("RACER_WEB_DIST", raising=False)
    assert cli.web_dist(s) == tmp_path / "tools" / "racer_app" / "web" / "dist"
    monkeypatch.setenv("RACER_WEB_DIST", "/x/y")
    assert str(cli.web_dist(s)) == "/x/y"


def test_real_runner_runs_argv_without_a_shell():
    from racer_agent.runner import Runner

    r = Runner()
    ok = r.run(["echo", "a; b $(c)"])
    assert ok.ok and ok.stdout.strip() == "a; b $(c)"  # no shell: metacharacters are literal
    assert r.run(["definitely-not-a-command-xyz"]).returncode == 127
    slow = r.run(["sleep", "5"], timeout=0.1)
    assert slow.returncode == 124 and "timed out" in slow.stderr
    assert r.run(["false"]).tail() == ""


def test_thermal_skips_zone_whose_read_fails_with_eagain(tmp_path, monkeypatch):
    # Jetson cv*-thermal zones return EAGAIN; Python's buffered read surfaces that as TypeError.
    host = HostInfo(root=tmp_path)
    for i, kind in enumerate(["cpu-thermal", "cv0-thermal"]):
        z = host.path(f"/sys/class/thermal/thermal_zone{i}")
        z.mkdir(parents=True)
        (z / "type").write_text(kind + "\n")
        (z / "temp").write_text("45000\n")
    real = type(host.path("/")).read_text

    def flaky(self, *a, **k):
        if self.name == "temp" and self.parent.name == "thermal_zone1":
            raise TypeError("can't concat NoneType to bytes")
        return real(self, *a, **k)

    monkeypatch.setattr(type(host.path("/")), "read_text", flaky)
    assert host.thermal_zones() == {"cpu-thermal": 45.0}
