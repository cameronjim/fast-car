"""Stack, camera, power and autostart actions against the fake docker world."""

from __future__ import annotations

import pytest
from racer_agent import commands as cmd
from racer_agent.controller import ActionError

DEFAULT = cmd.StackFlags()


def run_calls(env):
    return env.runner.calls_matching("docker", "run")


def test_start_stack_runs_the_whitelisted_docker_command(env):
    result = env.controller.start_stack(DEFAULT, actor="1.2.3.4")
    assert result["flags"]["lidar"] is True
    (call,) = run_calls(env)
    assert call[:7] == ("docker", "run", "-d", "--name", "car-stack", "--network", "host")
    assert env.world.containers["car-stack"]["state"] == "running"
    assert env.last_log()["action"] == "stack.start" and env.last_log()["ok"]
    assert env.last_log()["actor"] == "1.2.3.4"


def test_start_stack_creates_the_bag_directory(env):
    env.controller.start_stack(DEFAULT)
    assert env.settings.bags.is_dir()


def test_start_refuses_when_already_running(env):
    env.running_car()
    with pytest.raises(ActionError, match="already running"):
        env.controller.start_stack(DEFAULT)
    assert not run_calls(env)
    assert env.last_log()["ok"] is False


def test_start_removes_an_exited_container_first(env):
    env.world.start_stack(state="exited")
    env.controller.start_stack(DEFAULT)
    assert env.runner.calls_matching("docker", "rm", "car-stack")
    assert env.world.containers["car-stack"]["state"] == "running"


@pytest.mark.parametrize("free,ok", [(5.0, True), (4.9, False), (0.0, False)])
def test_disk_guard_refuses_to_start_recording_below_five_gib(env, free, ok):
    env.world.free_gb = free
    if ok:
        env.controller.start_stack(DEFAULT)
    else:
        with pytest.raises(ActionError, match="recorder would die"):
            env.controller.start_stack(DEFAULT)
        assert not run_calls(env)


def test_start_refuses_without_a_built_workspace(env):
    (env.repo / "ros_ws" / "install" / "setup.bash").unlink()
    with pytest.raises(ActionError, match="not built"):
        env.controller.start_stack(DEFAULT)


def test_start_refuses_without_the_image(env):
    env.world.image_present = False
    with pytest.raises(ActionError, match="image car:local is missing"):
        env.controller.start_stack(DEFAULT)


def test_start_refuses_a_missing_lidar_or_vesc_device(env):
    (env.host.root / "dev/ttyUSB0").unlink()
    with pytest.raises(ActionError, match="ttyUSB0"):
        env.controller.start_stack(DEFAULT)
    env.controller.start_stack(cmd.StackFlags(lidar=False))
    assert "--device" in run_calls(env)[0] and "/dev/ttyUSB0" not in run_calls(env)[0]


def test_start_refuses_a_missing_vesc_device(env):
    (env.host.root / "dev/ttyTHS1").unlink()
    with pytest.raises(ActionError, match="ttyTHS1"):
        env.controller.start_stack(DEFAULT)


def test_start_reports_a_docker_failure(env):
    env.world.docker_run_fails = "no such device"
    with pytest.raises(ActionError, match="no such device") as exc:
        env.controller.start_stack(DEFAULT)
    assert exc.value.status == 500


def test_start_reports_a_stack_that_exits_immediately(env):
    env.world.stack_dies_on_start = True
    with pytest.raises(ActionError, match="exited right after starting"):
        env.controller.start_stack(DEFAULT)


def test_invalid_flags_are_a_400_and_logged(env):
    with pytest.raises(ActionError) as exc:
        env.controller.start_stack(cmd.StackFlags(mapping=True, localization=True, map="a.yaml"))
    assert exc.value.status == 400
    assert env.last_log()["ok"] is False


def test_localisation_needs_a_map_the_agent_found(env):
    flags = cmd.StackFlags(localization=True, map="../../etc/passwd")
    with pytest.raises(ActionError, match="unknown map"):
        env.controller.start_stack(flags)
    (env.settings.maps).mkdir(parents=True)
    (env.settings.maps / "floor.yaml").write_text("image: floor.pgm\n")
    env.controller.start_stack(cmd.StackFlags(localization=True, map="floor.yaml"))
    assert run_calls(env)[0][-1].endswith("localization:=true map:=/workspace/data/maps/floor.yaml")


def test_stop_stack_uses_sigint_not_docker_stop(env):
    env.running_car()
    env.controller.stop_stack(actor="x")
    assert ("docker", "kill", "-s", "INT", "car-stack") in env.runner.calls
    assert not env.runner.calls_matching("docker", "stop")
    assert "car-stack" not in env.world.containers


def test_stop_stack_escalates_to_sigkill_when_it_will_not_exit(env):
    env.running_car()
    env.world.stuck_on_int = True
    result = env.controller.stop_stack()
    assert ("docker", "kill", "car-stack") in env.runner.calls
    assert "killed" in result["message"]
    assert sum(env.sleeps) >= 30


def test_stop_stack_when_absent_is_harmless(env):
    assert env.controller.stop_stack()["message"] == "stack not present"


def test_stop_stack_stops_a_running_drive_first(env):
    env.running_car()
    env.controller.start_drive("wall_follow")
    env.controller.stop_stack()
    assert env.world.drive_label is None


def test_restart_reuses_the_running_flags(env):
    env.world.start_stack("lidar:=false vesc:=true")
    env.controller.restart_stack(None)
    assert run_calls(env)[0][-1].endswith("lidar:=false vesc:=true")


def test_restart_with_new_flags(env):
    env.running_car()
    env.controller.restart_stack(cmd.StackFlags(mapping=True))
    assert run_calls(env)[0][-1].endswith("mapping:=true")


# -- cameras -------------------------------------------------------------------------------


def test_cameras_start_finds_the_usb_capture_node(env):
    result = env.controller.start_cameras()
    assert result["usb_device"] == "/dev/video1"
    argv = run_calls(env)[0]
    assert argv[argv.index("--device") + 1] == "/dev/video1"
    assert "nvidia" in argv
    assert "nothing is being recorded" in result["message"]


def test_cameras_with_a_running_stack_say_nothing_about_recording(env):
    env.running_car()
    assert "nothing is being recorded" not in env.controller.start_cameras()["message"]


@pytest.mark.parametrize("free,ok", [(10.0, True), (9.9, False)])
def test_cameras_refuse_below_ten_gib(env, free, ok):
    env.world.free_gb = free
    if ok:
        env.controller.start_cameras()
    else:
        with pytest.raises(ActionError, match="cameras record about 6 MB/s"):
            env.controller.start_cameras()


def test_cameras_refuse_without_the_usb_camera(env):
    for p in (env.host.root / "sys/class/video4linux").glob("video*/name"):
        p.write_text("something else\n")
    with pytest.raises(ActionError, match="USB camera"):
        env.controller.start_cameras()


def test_cameras_refuse_when_already_running_and_stop_cleanly(env):
    env.controller.start_cameras()
    with pytest.raises(ActionError, match="already running"):
        env.controller.start_cameras()
    env.controller.stop_cameras()
    assert "car-camera" not in env.world.containers
    assert ("docker", "kill", "-s", "INT", "car-camera") in env.runner.calls


def test_cameras_report_docker_failure(env):
    env.world.docker_run_fails = "unknown runtime nvidia"
    with pytest.raises(ActionError, match="unknown runtime"):
        env.controller.start_cameras()


def test_camera_restart_removes_a_stale_container(env):
    env.world.containers["car-camera"] = {"state": "exited", "cmd": [], "argv": ()}
    env.controller.start_cameras()
    assert env.world.containers["car-camera"]["state"] == "running"


# -- power ---------------------------------------------------------------------------------


@pytest.mark.parametrize("kind,verb", [("reboot", "reboot"), ("shutdown", "poweroff")])
def test_power_stops_everything_cleanly_then_calls_systemctl_via_sudo(env, kind, verb):
    env.running_car()
    env.controller.start_cameras()
    env.controller.power(kind, kind)
    assert "car-stack" not in env.world.containers and "car-camera" not in env.world.containers
    assert not env.runner.calls_matching("sudo")  # deferred so the HTTP reply gets out first
    (delay, go) = env.deferred[0]
    assert delay == 2.0
    go()
    assert ("sudo", "-n", "/usr/bin/systemctl", verb) in env.runner.calls


def test_power_needs_the_word_typed(env):
    with pytest.raises(ActionError, match="to confirm"):
        env.controller.power("reboot", "yes")
    with pytest.raises(ActionError):
        env.controller.power("halt", "halt")
    assert not env.deferred


# -- autostart and boot bookkeeping -----------------------------------------------------------


def test_autostart_flag_round_trip(env):
    assert env.controller.autostart_enabled() is False
    env.controller.set_autostart(True, "me")
    assert env.controller.autostart_enabled() is True
    env.controller.set_autostart(False, "me")
    assert env.controller.autostart_enabled() is False
    assert env.last_log()["action"] == "settings.autostart"


def test_autostart_boot_does_nothing_when_disabled(env):
    env.controller.autostart_boot()
    assert not run_calls(env)


def test_autostart_boot_starts_lidar_and_vesc_without_cameras(env):
    env.controller.set_autostart(True)
    env.controller.autostart_boot()
    (call,) = run_calls(env)
    assert call[-1].endswith("lidar:=true vesc:=true")
    assert not env.runner.calls_matching("docker", "run", "-d", "--name", "car-camera")
    assert env.last_log()["actor"] == "boot"


def test_autostart_boot_is_idempotent(env):
    env.controller.set_autostart(True)
    env.running_car()
    assert env.controller.autostart_boot()["message"] == "stack already running"
    assert not run_calls(env)


def test_autostart_refuses_on_a_full_disk(env):
    env.controller.set_autostart(True)
    env.world.free_gb = 1.0
    with pytest.raises(ActionError, match="recorder would die"):
        env.controller.autostart_boot()
    assert not run_calls(env)


def test_autostart_waits_for_devices_then_starts_degraded(env):
    env.controller.set_autostart(True)
    (env.host.root / "dev/ttyUSB0").unlink()
    env.controller.autostart_boot(wait_s=3.0)
    assert sum(env.sleeps) >= 3.0
    assert run_calls(env)[0][-1].endswith("lidar:=false vesc:=true")
    messages = [e["detail"] for e in env.actions.tail(10)]
    assert any("degraded" in m and "lidar MISSING" in m for m in messages)


def test_unclean_shutdown_is_noticed_on_the_next_start(env):
    assert env.controller.startup_check() is None  # first ever start
    assert env.controller.startup_check() is not None  # marker still says "running"
    env.controller.shutdown_mark()
    assert env.controller.startup_check() is None  # clean stop in between
    assert env.settings.run_marker.read_text().strip() == "running"
    entries = [e for e in env.actions.tail(10) if e["action"] == "agent.start"]
    assert any("unclean shutdown" in e["detail"] for e in entries)


def test_shutdown_mark_survives_an_unwritable_state_dir(env):
    import shutil

    shutil.rmtree(env.settings.state_dir)
    env.settings.state_dir.write_text("not a directory")
    env.controller.shutdown_mark()  # must not raise
