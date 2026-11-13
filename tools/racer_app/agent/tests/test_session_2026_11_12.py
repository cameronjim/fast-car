"""Fixes from the first real car session (2026-11-12): drives started from a terminal, the PnP
card's explicit bundle and planner params file, and a bounded gap follow max speed."""

from __future__ import annotations

import ast
import re
from pathlib import Path

import pytest
from fakes import terminal_drive
from racer_agent import commands as cmd
from racer_agent.controller import TERMINAL_SUFFIX, ActionError

REPO_ROOT = Path(__file__).resolve().parents[4]
DYN = "dyn-v0-2026-11-12"
SEP = "\x1f"


def detached_execs(env):
    return [c for c in env.runner.calls if c[:3] == ("docker", "exec", "-d")]


@pytest.fixture
def car(env):
    env.running_car()
    return env


def wire_pnp(car, params=("planner-2026-11-12.yaml",)):
    folder = car.settings.pnp_models_dir / DYN
    folder.mkdir(parents=True)
    (folder / "manifest.yaml").write_text("kind: dynamics\n")
    launch = car.repo / "ros_ws/src/racer_bringup/launch"
    launch.mkdir(parents=True, exist_ok=True)
    (launch / "pnp.launch.py").write_text("")
    for name in params:
        (car.settings.pnp_dir / name).write_text("mppi_planner_node: {}\n")


# -- bug 3: a drive started from a terminal --------------------------------------------------


def scan_line(pid, *argv):
    return f"{pid} " + SEP.join(argv) + "\n"


def test_parse_finds_driving_launches_by_their_own_command_line():
    out = (
        scan_line(
            1,
            "python3",
            "/opt/ros/humble/bin/ros2",
            "launch",
            "racer_bringup",
            "car_teleop.launch.py",
        )
        + scan_line(
            30,
            "python3",
            "/opt/ros/humble/bin/ros2",
            "launch",
            "racer_control",
            "gap_follow.launch.py",
            "profile:=x",
        )
        + scan_line(
            31,
            "/usr/bin/python3",
            "/opt/ros/humble/bin/ros2",
            "launch",
            "racer_bringup",
            "pnp.launch.py",
            "v_max_mps:=0.5",
        )
        + scan_line(
            32,
            "python3",
            "/opt/ros/humble/bin/ros2",
            "run",
            "racer_control",
            "tracker_node",
            "--ros-args",
        )
        + scan_line(
            33, "ros2", "launch", "/workspace/ros_ws/src/racer_control/launch/park.launch.py"
        )
        + scan_line(
            34, "python3", "/opt/ros/humble/bin/ros2", "launch", "racer_bringup", "policy.launch.py"
        )
        + scan_line(
            35,
            "python3",
            "/opt/ros/humble/bin/ros2",
            "launch",
            "racer_control",
            "wall_follow.launch.py",
        )
        + scan_line(
            36, "python3", "/opt/ros/humble/bin/ros2", "launch", "racer_apps", "spiderbot.launch.py"
        )
    )
    found = {p.pid: p.mode for p in cmd.parse_drive_processes(out)}
    assert found == {
        30: "gap_follow",
        31: "pnp",
        32: "tracker",
        33: "park",
        34: "ml_policy",
        35: "wall_follow",
        36: "spiderbot",
    }


def test_parse_ignores_wrappers_the_stack_the_recorder_and_junk():
    out = (
        scan_line(40, "bash", "-c", "ros2 launch racer_control gap_follow.launch.py")  # a wrapper
        + scan_line(41, "python3", "/opt/ros/humble/bin/ros2", "bag", "record", "-o", "x")
        + scan_line(
            42,
            "python3",
            "/opt/ros/humble/bin/ros2",
            "launch",
            "racer_bringup",
            "cameras.launch.py",
        )
        + scan_line(
            43, "python3", "/opt/ros/humble/bin/ros2", "topic", "echo", "/gap_follow.launch.py"
        )
        + scan_line(44, "python3", "/opt/ros/humble/bin/ros2")
        + "not a pid line\n"
        + "x y\n"
    )
    assert cmd.parse_drive_processes(out) == []


def test_the_scan_script_reads_proc_and_the_signal_script_is_pid_only():
    assert "/proc/" in cmd.DRIVE_SCAN_SCRIPT and "docker" not in cmd.DRIVE_SCAN_SCRIPT
    script = cmd.drive_pids_signal_script([30, 31], "INT")
    assert script.startswith("for p in 30 31; do") and "kill -INT $p" in script
    for bad in ([], [0], [1], ["30; reboot"]):
        with pytest.raises((cmd.CommandError, ValueError)):
            cmd.drive_pids_signal_script(bad, "INT")
    with pytest.raises(cmd.CommandError):
        cmd.drive_pids_signal_script([30], "STOP; reboot")


def test_a_terminal_drive_is_shown_in_status(car):
    car.world.external_drives.append(
        terminal_drive(
            77, "launch", "racer_control", "gap_follow.launch.py", "profile:=race-2026-11-12"
        )
    )
    status = car.controller.status()
    assert status["drive"]["running"] is True
    assert status["drive"]["mode"] == "gap_follow" + TERMINAL_SUFFIX
    (ext,) = status["drive"]["external"]
    assert (
        ext["pid"] == 77
        and ext["mode"] == "gap_follow"
        and "profile:=race-2026-11-12" in ext["command"]
    )


def test_an_app_drive_is_not_listed_as_external(car):
    car.controller.start_drive("wall_follow")
    status = car.controller.status()
    assert status["drive"]["mode"] == "wall_follow" and status["drive"]["external"] == []


def test_stop_all_sigints_a_terminal_drive_too(car):
    car.world.external_drives.append(terminal_drive(77, "launch", "racer_bringup", "pnp.launch.py"))
    result = car.controller.stop_all(actor="phone")
    assert result["was_running"] is True
    assert car.world.external_drives == []
    kills = [
        c[-1]
        for c in car.runner.calls
        if c[:3] == ("docker", "exec", "car-stack") and "for p in" in c[-1]
    ]
    assert kills and "kill -INT $p" in kills[0] and "for p in 77;" in kills[0]
    assert "1 started from a terminal" in car.last_log()["detail"]
    assert car.world.containers["car-stack"]["state"] == "running"  # the stack keeps recording


def test_stop_all_stops_the_app_drive_and_a_terminal_drive_together(car):
    car.controller.start_drive("wall_follow")
    car.world.external_drives.append(
        terminal_drive(77, "launch", "racer_control", "gap_follow.launch.py")
    )
    car.controller.stop_all()
    assert car.world.drive_label is None and car.world.external_drives == []


def test_stop_all_escalates_on_a_terminal_drive_that_ignores_sigint(car):
    car.world.external_drives.append(
        terminal_drive(77, "launch", "racer_control", "gap_follow.launch.py")
    )
    car.world.ignore_int_external = True
    assert car.controller.stop_all()["was_running"] is True
    assert car.world.external_drives == []
    signals = [
        re.search(r"kill -(\w+)", c[-1]).group(1)
        for c in car.runner.calls
        if c[:3] == ("docker", "exec", "car-stack") and "for p in" in c[-1]
    ]
    assert signals[0] == "INT" and signals[-1] == "TERM"


def test_the_app_refuses_to_start_a_drive_next_to_a_terminal_drive(car):
    car.world.external_drives.append(
        terminal_drive(77, "launch", "racer_control", "gap_follow.launch.py")
    )
    with pytest.raises(ActionError, match="already running .*started from a terminal"):
        car.controller.start_drive("wall_follow")
    assert not detached_execs(car)


def test_stack_stop_and_the_disk_guard_stop_a_terminal_drive_first(car):
    car.world.external_drives.append(
        terminal_drive(77, "launch", "racer_control", "gap_follow.launch.py")
    )
    car.controller.stop_stack()
    assert car.world.external_drives == []
    assert car.runner.calls_matching("docker", "exec", "car-stack", "bash", "-c")


def test_the_disk_guard_sees_a_terminal_drive(car):
    from racer_agent.diskguard import DiskGuard

    car.world.free_gb = 2.0
    car.world.external_drives.append(
        terminal_drive(77, "launch", "racer_control", "gap_follow.launch.py")
    )
    done = DiskGuard(car.controller).tick()
    assert "stop_drive" in done and car.world.external_drives == []


def test_park_commands_work_with_a_terminal_started_park_launch(car):
    car.world.external_drives.append(
        terminal_drive(77, "launch", "racer_control", "park.launch.py")
    )
    assert car.controller.park_command("abort")["message"] == "abort: ok"


# -- bug 2: the dynamics bundle is explicit, in the request, the command and the log ---------


def test_pnp_logs_the_chosen_bundle_and_planner_file(car):
    wire_pnp(car)
    car.controller.start_drive(
        "pnp",
        pnp_backend="geometric",
        pnp_dynamics=DYN,
        pnp_params="planner-2026-11-12.yaml",
        pnp_v_max_mps=0.6,
    )
    command = detached_execs(car)[0][-1]
    assert f"dynamics_bundle:=/workspace/data/pnp/models/{DYN} " in command
    assert "params_file:=/workspace/data/pnp/planner-2026-11-12.yaml" in command
    assert "v_max_mps:=0.60" in command
    entry = car.last_log()
    assert entry["params"]["dynamics"] == DYN
    assert entry["params"]["params_file"] == "planner-2026-11-12.yaml"
    assert f"dynamics={DYN}" in entry["detail"]
    assert "planner_params=planner-2026-11-12.yaml" in entry["detail"]


def test_pnp_refuses_an_empty_dynamics_even_with_exactly_one_bundle(car):
    wire_pnp(car)
    with pytest.raises(ActionError, match=f"no dynamics bundle in the request.*{DYN}") as exc:
        car.controller.start_drive("pnp", pnp_backend="geometric")
    assert exc.value.status == 400 and not detached_execs(car)
    assert car.last_log()["ok"] is False


def test_pnp_refuses_when_there_is_no_bundle_at_all(car):
    with pytest.raises(ActionError, match="Plug-and-Play is disabled: no dynamics bundle"):
        car.controller.start_drive("pnp", pnp_backend="geometric", pnp_dynamics=DYN)
    assert not detached_execs(car)


# -- bug 4: planner params file picker -------------------------------------------------------


def test_pnp_lists_planner_params_files_safely(car):
    wire_pnp(car, params=("planner-2026-11-12.yaml", "other.yaml"))
    base = car.settings.pnp_dir
    (base / "notes.txt").write_text("x")
    (base / "has space.yaml").write_text("x")
    (base / "semi;colon.yaml").write_text("x")
    (base / "dir.yaml").mkdir()
    (base / "link.yaml").symlink_to(base / "other.yaml")
    status = car.controller.pnp_status()
    assert status["params_files"] == ["other.yaml", "planner-2026-11-12.yaml"]
    assert car.controller.options()["pnp"]["params_files"] == status["params_files"]


def test_pnp_without_a_params_file_passes_none(car):
    wire_pnp(car)
    car.controller.start_drive("pnp", pnp_backend="geometric", pnp_dynamics=DYN)
    assert "params_file" not in detached_execs(car)[0][-1]


@pytest.mark.parametrize(
    "name", ["../models/x.yaml", "/etc/passwd", "nope.yaml", "a b.yaml", "x.yaml; reboot", "models"]
)
def test_pnp_only_accepts_params_files_the_agent_listed(car, name):
    wire_pnp(car)
    with pytest.raises(ActionError, match="unknown planner params file") as exc:
        car.controller.start_drive(
            "pnp", pnp_backend="geometric", pnp_dynamics=DYN, pnp_params=name
        )
    assert exc.value.status == 400 and not detached_execs(car)


def test_the_params_file_command_is_shell_safe():
    ok = "/workspace/data/pnp/planner-2026-11-12.yaml"
    assert cmd.ros_cmd_for(
        "pnp",
        pnp_dynamics_container_path=f"{cmd.PNP_MODELS_IN_CONTAINER}d",
        pnp_backend="geometric",
        pnp_params_container_path=ok,
    ).endswith(f"params_file:={ok}")
    for bad in (
        "/workspace/data/pnp/models/x.yaml",
        "/workspace/data/pnp/a.txt",
        "/workspace/data/pnp/x y.yaml",
        "/workspace/data/pnp/x;y.yaml",
        "/workspace/data/pnp/../../etc/x.yaml",
        "/etc/passwd",
    ):
        with pytest.raises(cmd.CommandError):
            cmd.ros_cmd_for(
                "pnp",
                pnp_dynamics_container_path=f"{cmd.PNP_MODELS_IN_CONTAINER}d",
                pnp_backend="geometric",
                pnp_params_container_path=bad,
            )


# -- bug 4: gap follow max speed -------------------------------------------------------------


def test_gap_follow_without_a_speed_passes_no_speed_argument(car):
    car.controller.start_drive("gap_follow", profile="lane-2026-10-25")
    assert "max_speed_mps" not in detached_execs(car)[0][-1]
    assert "max_speed=profile default" in car.last_log()["detail"]


def test_gap_follow_with_a_speed_passes_it_and_logs_it(car):
    car.controller.start_drive("gap_follow", profile="race-2026-11-12", max_speed_mps=0.6)
    assert "profile:=race-2026-11-12 max_speed_mps:=0.60" in detached_execs(car)[0][-1]
    assert car.last_log()["params"]["max_speed_mps"] == 0.6
    assert "max_speed=0.60 m/s" in car.last_log()["detail"]


@pytest.mark.parametrize(
    "profile,speed,ok",
    [
        ("race-2026-11-12", 1.0, True),
        ("race-2026-11-12", 1.01, False),
        ("lane-2026-10-25", 0.64, True),
        ("lane-2026-10-25", 0.7, False),
        ("none", 1.43, True),
        ("none", 1.5, False),
        ("race-2026-11-12", 0.36, True),
        ("race-2026-11-12", 0.35, False),
        ("race-2026-11-12", 0.0, False),
        ("race-2026-11-12", -1.0, False),
    ],
)
def test_gap_follow_speed_is_bounded_by_the_profile(car, profile, speed, ok):
    if ok:
        car.controller.start_drive("gap_follow", profile=profile, max_speed_mps=speed)
        assert detached_execs(car)
    else:
        with pytest.raises(ActionError, match="max speed") as exc:
            car.controller.start_drive("gap_follow", profile=profile, max_speed_mps=speed)
        assert exc.value.status == 400 and not detached_execs(car)


def test_gap_follow_speed_is_also_bounded_by_vehicle_params(car):
    (car.repo / "config").mkdir(exist_ok=True)
    (car.repo / "config" / "vehicle_params.yaml").write_text(
        "limits:\n  global_speed_cap_mps: 0.5\n"
    )
    assert car.controller.gap_follow_speed_limits("race-2026-11-12") == (0.36, 0.5)
    with pytest.raises(ActionError, match="max speed"):
        car.controller.start_drive("gap_follow", profile="race-2026-11-12", max_speed_mps=0.8)
    car.controller.start_drive("gap_follow", profile="race-2026-11-12", max_speed_mps=0.5)


def test_a_null_or_missing_vehicle_cap_leaves_the_profile_ceiling(car):
    assert car.controller.gap_follow_speed_limits("race-2026-11-12") == (0.36, 1.0)
    (car.repo / "config").mkdir(exist_ok=True)
    (car.repo / "config" / "vehicle_params.yaml").write_text(
        "limits:\n  global_speed_cap_mps: null\n"
    )
    assert car.controller.gap_follow_speed_limits("race-2026-11-12") == (0.36, 1.0)
    (car.repo / "config" / "vehicle_params.yaml").write_text(": : [\n")
    assert car.controller.gap_follow_speed_limits("lane-2026-10-25") == (0.36, 0.64)


def test_options_carry_the_gap_follow_speed_limits(car):
    limits = car.controller.options()["gap_follow_speed"]
    assert set(limits) == set(cmd.GAP_FOLLOW_PROFILES)
    assert limits["race-2026-11-12"] == {"min": 0.36, "max": 1.0}


def test_a_speed_is_only_for_gap_follow(car):
    with pytest.raises(ActionError, match="only used with gap follow"):
        car.controller.start_drive("wall_follow", max_speed_mps=0.5)


def test_the_speed_must_be_a_number_in_the_command():
    with pytest.raises(cmd.CommandError, match="number"):
        cmd.ros_cmd_for("gap_follow", profile="none", max_speed_mps="1; reboot")  # type: ignore[arg-type]
    with pytest.raises(cmd.CommandError, match="number"):
        cmd.ros_cmd_for("gap_follow", profile="none", max_speed_mps=True)  # type: ignore[arg-type]


# -- gap follow min speed ---------------------------------------------------------------------


def test_gap_follow_without_a_min_speed_passes_no_min_argument(car):
    car.controller.start_drive("gap_follow", profile="race-2026-11-13", max_speed_mps=1.0)
    assert "min_speed_mps" not in detached_execs(car)[0][-1]
    assert car.last_log()["params"]["min_speed_mps"] is None
    assert "min_speed=profile default" in car.last_log()["detail"]


def test_gap_follow_with_a_min_speed_passes_it_and_logs_it(car):
    car.controller.start_drive(
        "gap_follow", profile="race-2026-11-12", max_speed_mps=0.6, min_speed_mps=0.5
    )
    assert (
        "profile:=race-2026-11-12 max_speed_mps:=0.60 min_speed_mps:=0.50"
        in detached_execs(car)[0][-1]
    )
    assert car.last_log()["params"]["min_speed_mps"] == 0.5
    assert "min_speed=0.50 m/s" in car.last_log()["detail"]


def test_gap_follow_min_speed_alone_passes_only_the_min(car):
    car.controller.start_drive("gap_follow", profile="race-2026-11-13", min_speed_mps=1)
    launched = detached_execs(car)[0][-1]
    assert "profile:=race-2026-11-13 min_speed_mps:=1.00" in launched
    assert "max_speed_mps" not in launched


@pytest.mark.parametrize(
    "profile,max_speed,min_speed,ok",
    [
        # no cap set: bounded by the profile's own ceiling
        ("race-2026-11-12", None, 1.0, True),
        ("race-2026-11-12", None, 1.01, False),
        ("lane-2026-10-25", None, 0.64, True),
        ("lane-2026-10-25", None, 0.65, False),
        ("none", None, 1.43, True),
        ("none", None, 1.5, False),
        ("race-2026-11-12", None, 0.0, True),
        ("race-2026-11-12", None, -0.01, False),
        ("race-2026-11-12", None, float("nan"), False),
        ("race-2026-11-12", None, float("inf"), False),
        # a cap set: bounded by that cap, never above it
        ("race-2026-11-12", 0.6, 0.6, True),
        ("race-2026-11-12", 0.6, 0.61, False),
        ("race-2026-11-13", 0.4, 0.36, True),
        ("race-2026-11-13", 0.4, 1.0, False),
    ],
)
def test_gap_follow_min_speed_is_bounded_by_the_max_in_effect(
    car, profile, max_speed, min_speed, ok
):
    kwargs = {"profile": profile, "max_speed_mps": max_speed, "min_speed_mps": min_speed}
    if ok:
        car.controller.start_drive("gap_follow", **kwargs)
        assert detached_execs(car)
    else:
        with pytest.raises(ActionError, match="min speed") as exc:
            car.controller.start_drive("gap_follow", **kwargs)
        assert exc.value.status == 400 and not detached_execs(car)
        assert "cannot exceed the max speed" in exc.value.message


def test_gap_follow_min_speed_is_also_bounded_by_vehicle_params(car):
    (car.repo / "config").mkdir(exist_ok=True)
    (car.repo / "config" / "vehicle_params.yaml").write_text(
        "limits:\n  global_speed_cap_mps: 0.5\n"
    )
    with pytest.raises(ActionError, match="between 0 and 0.5"):
        car.controller.start_drive("gap_follow", profile="race-2026-11-12", min_speed_mps=0.6)
    car.controller.start_drive("gap_follow", profile="race-2026-11-12", min_speed_mps=0.5)


def test_an_out_of_range_max_is_reported_before_the_min(car):
    with pytest.raises(ActionError, match="max speed"):
        car.controller.start_drive(
            "gap_follow", profile="race-2026-11-12", max_speed_mps=5.0, min_speed_mps=0.5
        )
    assert not detached_execs(car)


@pytest.mark.parametrize("bad", ["0.5", True])
def test_the_controller_refuses_a_min_speed_that_is_not_a_number(car, bad):
    with pytest.raises(ActionError, match="min speed") as exc:
        car.controller.start_drive("gap_follow", profile="race-2026-11-12", min_speed_mps=bad)
    assert exc.value.status == 400 and not detached_execs(car)


def test_a_min_speed_is_only_for_gap_follow(car):
    with pytest.raises(ActionError, match="min speed is only used with gap follow"):
        car.controller.start_drive("wall_follow", min_speed_mps=0.5)
    assert not detached_execs(car)


def test_the_min_speed_must_be_a_number_in_the_command():
    with pytest.raises(cmd.CommandError, match="min speed must be a number"):
        cmd.ros_cmd_for("gap_follow", profile="none", min_speed_mps="1; reboot")  # type: ignore[arg-type]
    with pytest.raises(cmd.CommandError, match="min speed must be a number"):
        cmd.ros_cmd_for("gap_follow", profile="none", min_speed_mps=False)  # type: ignore[arg-type]
    assert cmd.ros_cmd_for("gap_follow", profile="none", min_speed_mps=0).endswith(
        "profile:=none min_speed_mps:=0.00"
    )


# -- the profile ceilings mirror the launch file ----------------------------------------------


def _literal_dicts(tree: ast.Module) -> dict[str, dict]:
    """Module-level `NAME = {...}` dicts of numbers, resolving `**OTHER` spreads."""
    out: dict[str, dict] = {}
    for node in tree.body:
        value = node.value if isinstance(node, (ast.Assign, ast.AnnAssign)) else None
        if not isinstance(value, ast.Dict):
            continue
        target = node.targets[0] if isinstance(node, ast.Assign) else node.target
        merged: dict = {}
        for key, val in zip(value.keys, value.values):
            if key is None and isinstance(val, ast.Name) and val.id in out:
                merged.update(out[val.id])
            elif isinstance(key, ast.Constant):
                try:
                    merged[key.value] = ast.literal_eval(val)
                except ValueError:
                    merged[key.value] = val  # names such as FLOOR_2026_10_06_PROFILE
        out[target.id] = merged
    return out


@pytest.mark.skipif(
    not (REPO_ROOT / "ros_ws/src/racer_control/launch/gap_follow.launch.py").exists(),
    reason="repo launch files not present",
)
def test_gap_follow_speed_ceilings_match_the_launch_file():
    text = (REPO_ROOT / "ros_ws/src/racer_control/launch/gap_follow.launch.py").read_text()
    tree = ast.parse(text)
    dicts = _literal_dicts(tree)
    default = float(
        re.search(r"^_DEFAULT_MAX_SPEED_MPS\s*=\s*([0-9.]+)", text, re.MULTILINE).group(1)
    )
    profiles = {
        "none": {},
        "floor-2026-10-06": dicts["FLOOR_2026_10_06_PROFILE"],
        "lane-2026-10-25": dicts["LANE_2026_10_25_PROFILE"],
        "race-2026-11-12": dicts["RACE_2026_11_12_PROFILE"],
        "race-2026-11-13": dicts["RACE_2026_11_13_PROFILE"],
    }
    assert set(profiles) == set(cmd.GAP_FOLLOW_PROFILES)
    for name, values in profiles.items():
        expected = values.get("max_speed_mps", default)
        assert cmd.GAP_FOLLOW_PROFILE_MAX_SPEED_MPS[name] == pytest.approx(expected), name
        assert values.get("min_speed_mps", 0.36) == cmd.GAP_FOLLOW_MIN_SPEED_MPS
