"""Driving launches, the pre-drive gate, STOP ALL, park services, maps and bags."""

from __future__ import annotations

import os

import pytest
from racer_agent import commands as cmd
from racer_agent.controller import ActionError


def detached_execs(env):
    return [c for c in env.runner.calls if c[:3] == ("docker", "exec", "-d")]


@pytest.fixture
def car(env):
    env.running_car()
    return env


# -- the pre-drive gate ----------------------------------------------------------------------


def test_gap_follow_starts_with_all_green(car):
    result = car.controller.start_drive("gap_follow", profile="lane-2026-10-25", actor="9.9.9.9")
    assert result["mode"] == "gap_follow:lane-2026-10-25" and result["override"] is False
    (call,) = detached_execs(car)
    assert "ros2 launch racer_control gap_follow.launch.py profile:=lane-2026-10-25" in call[-1]
    assert call[-1].startswith("echo $$ > /tmp/racer-drive.pid;")
    entry = car.last_log()
    assert entry["action"] == "drive.start.gap_follow" and entry["ok"] and "override" not in entry


def test_default_profiles(car):
    assert car.controller.start_drive("gap_follow")["mode"] == "gap_follow:lane-2026-10-25"
    car.controller.stop_all()
    assert car.controller.start_drive("park")["mode"] == "park:floor-2026-10-07"


def test_wall_follow_has_no_profile(car):
    assert car.controller.start_drive("wall_follow")["mode"] == "wall_follow"


def test_refused_when_the_stack_is_not_running(env):
    with pytest.raises(ActionError, match="car-stack is not running") as exc:
        env.controller.start_drive("gap_follow")
    assert exc.value.data["overridable"] is False
    assert not detached_execs(env)


def test_refused_without_a_recorder_even_with_an_override(car):
    car.world.recorder = False
    with pytest.raises(ActionError, match="invariant 5") as exc:
        car.controller.start_drive("gap_follow", override=True, override_reason="in a hurry")
    assert exc.value.data == {"failing": ["recorder"], "overridable": False}
    assert not detached_execs(car)
    assert car.last_log()["ok"] is False


def test_refused_on_a_nearly_full_disk_even_with_an_override(car):
    car.world.free_gb = 2.0
    with pytest.raises(ActionError) as exc:
        car.controller.start_drive("gap_follow", override=True, override_reason="please")
    assert exc.value.data["overridable"] is False
    assert not detached_execs(car)


def test_amber_check_needs_an_override_with_a_reason(car):
    car.world.free_gb = 10.0  # amber disk
    with pytest.raises(ActionError, match="not all green") as exc:
        car.controller.start_drive("gap_follow")
    assert exc.value.data == {"failing": ["disk"], "overridable": True}
    with pytest.raises(ActionError, match="needs a reason") as exc2:
        car.controller.start_drive("gap_follow", override=True, override_reason="  ")
    assert exc2.value.status == 400
    assert not detached_execs(car)


def test_override_with_reason_drives_and_is_logged(car):
    car.ros.set_value("voltage_v", 10.8)  # amber battery
    result = car.controller.start_drive(
        "wall_follow", override=True, override_reason="short test on the bench"
    )
    assert result["override"] is True
    entry = car.last_log()
    assert entry["override"] is True and entry["override_reason"] == "short test on the bench"
    assert entry["ok"] is True


def test_red_required_check_can_be_overridden_but_not_the_hard_ones(car):
    car.ros.set_value("fault", "FAULT_CODE_OVER_VOLTAGE")
    with pytest.raises(ActionError) as exc:
        car.controller.start_drive("wall_follow")
    assert exc.value.data["overridable"] is True
    car.controller.start_drive("wall_follow", override=True, override_reason="known glitch")


def test_unknown_mode_profile_and_injection_attempts_are_400(car):
    for kwargs in (
        {"mode": "teleop"},
        {"mode": "gap_follow", "profile": "x; reboot"},
        {"mode": "park", "profile": "lane-2026-10-25"},
        {"mode": "tracker", "raceline": "../../etc/passwd"},
    ):
        with pytest.raises(ActionError) as exc:
            car.controller.start_drive(**kwargs)
        assert exc.value.status == 400
    assert not detached_execs(car)


def test_only_one_driving_launch_at_a_time(car):
    car.controller.start_drive("wall_follow")
    with pytest.raises(ActionError, match="already running"):
        car.controller.start_drive("gap_follow")
    assert len(detached_execs(car)) == 1


def test_a_launch_that_dies_at_once_is_reported(car):
    car.world.launch_dies_on_start = True
    with pytest.raises(ActionError, match="exited right away") as exc:
        car.controller.start_drive("wall_follow")
    assert exc.value.status == 500


# -- tracker and ML policy ---------------------------------------------------------------------


def test_tracker_needs_localisation_and_a_live_pose(car):
    with pytest.raises(ActionError, match="needs the stack running with localisation"):
        car.controller.start_drive("tracker", raceline="track:gym_oval")
    car.world.start_stack(
        "lidar:=true vesc:=true localization:=true map:=/workspace/data/maps/a.yaml"
    )
    with pytest.raises(ActionError, match="live /pose"):
        car.controller.start_drive("tracker", raceline="track:gym_oval")
    for _ in range(40):
        car.ros.count("/pose")
    car.controller.start_drive("tracker", raceline="track:gym_oval")
    (call,) = detached_execs(car)
    assert "ros2 run racer_control tracker_node" in call[-1]
    assert "-p raceline_path:=/workspace/config/tracks/gym_oval/raceline.csv" in call[-1]
    assert "-p pose_frame:=map" in call[-1]


def test_user_racelines_are_offered_and_usable(car):
    folder = car.settings.racelines_user_dir
    folder.mkdir(parents=True)
    (folder / "lane1.csv").write_text("x,y\n")
    assert "data:lane1" in car.controller.options()["racelines"]
    assert "track:gym_oval" in car.controller.options()["racelines"]


def test_ml_policy_is_disabled_with_a_clear_reason(car):
    status = car.controller.policy_status()
    assert status["available"] is False and "does not exist yet" in status["reason"]
    with pytest.raises(ActionError, match="ML policy is disabled: no policy directory"):
        car.controller.start_drive("ml_policy")
    assert not detached_execs(car)
    assert car.last_log()["ok"] is False


def make_policy_dir(env, name="p1"):
    folder = env.settings.policies_dir / name
    folder.mkdir(parents=True)
    (folder / "contract.yaml").write_text("x: 1\n")
    return folder


def test_ml_policy_stays_off_when_the_contract_refuses(car):
    make_policy_dir(car)
    car.runner.responders.insert(
        0,
        lambda argv: (1, "") if "--rm" in argv else None,
    )
    status = car.controller.policy_status()
    assert not status["available"] and "refused every policy" in status["reason"]
    with pytest.raises(ActionError, match="ML policy is disabled"):
        car.controller.start_drive("ml_policy", raceline="track:gym_oval")


def test_ml_policy_off_without_a_launch_file_even_if_the_contract_passes(car):
    make_policy_dir(car)
    car.runner.responders.insert(0, lambda argv: (0, "OK\n") if "--rm" in argv else None)
    status = car.controller.policy_status()
    assert status["valid"] == ["p1"] and not status["available"]
    assert "policy.launch.py" in status["reason"]


def test_ml_policy_check_runs_the_nodes_own_deploy_checks_in_the_container(car):
    make_policy_dir(car)
    car.runner.responders.insert(0, lambda argv: (0, "OK\n") if "--rm" in argv else None)
    car.controller.policy_status()
    (call,) = car.runner.calls_matching("docker", "run", "--rm")
    assert tuple(call[-3:-1]) == ("-m", "racer_policy.check")
    assert call[-1] == "/workspace/data/policies/p1"
    env = call[call.index("-e") + 1]
    # The checker needs the loader, the vehicle_params binding's loader and the envelope library.
    assert "ros_ws/src/racer_policy/src" in env and "ros_ws/src/racer_tools" in env
    assert "training/envelope/src" in env


def wire_ml_policy(car, names=("p1",)):
    for name in names:
        make_policy_dir(car, name)
    launch = car.repo / "ros_ws/src/racer_bringup/launch"
    launch.mkdir(parents=True)
    (launch / "policy.launch.py").write_text("")
    car.runner.responders.insert(0, lambda argv: (0, "OK\n") if "--rm" in argv else None)


def localised_with_pose(car):
    car.world.start_stack(
        "lidar:=true vesc:=true localization:=true map:=/workspace/data/maps/a.yaml"
    )
    for _ in range(40):
        car.ros.count("/pose")


def test_ml_policy_available_when_valid_and_wired(car):
    wire_ml_policy(car)
    assert car.controller.policy_status()["available"] is True
    localised_with_pose(car)
    result = car.controller.start_drive("ml_policy", raceline="track:gym_oval")
    assert result["mode"] == "ml_policy:p1"
    command = detached_execs(car)[0][-1]
    assert "ros2 launch racer_bringup policy.launch.py" in command
    assert "policy_dir:=/workspace/data/policies/p1" in command
    assert "raceline:=/workspace/config/tracks/gym_oval/raceline.csv" in command
    assert "pose_frame:=map" in command


def test_ml_policy_needs_a_raceline_the_app_knows(car):
    wire_ml_policy(car)
    localised_with_pose(car)
    with pytest.raises(ActionError, match="unknown raceline"):
        car.controller.start_drive("ml_policy", raceline="")
    with pytest.raises(ActionError, match="unknown raceline"):
        car.controller.start_drive("ml_policy", raceline="../../etc/passwd")
    assert not detached_execs(car)


def test_ml_policy_needs_localisation_and_a_live_pose_like_the_tracker(car):
    wire_ml_policy(car)
    with pytest.raises(ActionError, match="needs the stack running with localisation"):
        car.controller.start_drive("ml_policy", raceline="track:gym_oval")
    assert not detached_execs(car)


def test_ml_policy_with_two_valid_policies_must_be_told_which(car):
    wire_ml_policy(car, ("p1", "p2"))
    localised_with_pose(car)
    with pytest.raises(ActionError, match="choose a policy"):
        car.controller.start_drive("ml_policy", raceline="track:gym_oval")
    with pytest.raises(ActionError, match="not one the deploy checks accept"):
        car.controller.start_drive("ml_policy", raceline="track:gym_oval", policy="p3")
    car.controller.start_drive("ml_policy", raceline="track:gym_oval", policy="p2")
    assert "policy_dir:=/workspace/data/policies/p2" in detached_execs(car)[0][-1]


def test_ml_policy_only_names_directories_that_passed_the_check(car):
    """A directory the checker refused is not selectable even by name."""
    wire_ml_policy(car, ("good", "bad"))
    car.runner.responders.insert(
        0,
        lambda argv: (
            (1, "REFUSED VehicleParamsVersionMismatchError")
            if argv[-1].endswith("/bad") and "--rm" in argv
            else None
        ),
    )
    status = car.controller.policy_status()
    assert status["valid"] == ["good"] and status["available"]
    localised_with_pose(car)
    with pytest.raises(ActionError, match="not one the deploy checks accept"):
        car.controller.start_drive("ml_policy", raceline="track:gym_oval", policy="bad")


def test_ml_policy_keeps_the_pre_drive_rules(car):
    """It goes through the same gate as every other driving mode: no recorder, no drive."""
    wire_ml_policy(car)
    localised_with_pose(car)
    car.world.recorder = False
    with pytest.raises(ActionError) as exc:
        car.controller.start_drive("ml_policy", raceline="track:gym_oval")
    assert exc.value.data == {"failing": ["recorder"], "overridable": False}
    assert not detached_execs(car)


# -- STOP ALL -------------------------------------------------------------------------------


def test_stop_all_sigints_the_launch_and_keeps_the_stack(car):
    car.controller.start_drive("gap_follow")
    result = car.controller.stop_all(actor="phone")
    assert result["was_running"] is True
    assert car.world.drive_label is None
    assert car.world.containers["car-stack"]["state"] == "running"
    assert not car.runner.calls_matching("docker", "kill")
    assert car.last_log()["action"] == "stop_all"


def test_stop_all_with_nothing_running_is_fine(car):
    assert car.controller.stop_all()["was_running"] is False


def test_stop_all_with_no_stack_is_fine(env):
    assert env.controller.stop_all()["was_running"] is False


def test_stop_all_escalates_when_sigint_is_ignored(car):
    car.controller.start_drive("gap_follow")
    car.world.ignore_int = True
    assert car.controller.stop_all()["was_running"] is True
    assert car.world.drive_label is None


def test_stop_all_while_a_start_is_in_flight_wins(car):
    """A STOP that lands between the launch and the first check must still end it."""
    started = []

    def stop_arrives(argv):
        if argv[:3] == ("docker", "exec", "-d") and not started:
            started.append(1)
            with car.controller._gen_lock:
                car.controller._stop_generation += 1  # what stop_all's first line does

    car.runner.responders.insert(0, stop_arrives)
    with pytest.raises(ActionError, match="STOP ALL arrived"):
        car.controller.start_drive("gap_follow")
    assert car.world.drive_label is None
    assert car.last_log()["ok"] is False


# -- park -------------------------------------------------------------------------------------


def test_park_services_need_the_park_launch(car):
    with pytest.raises(ActionError, match="start the park launch first"):
        car.controller.park_command("start")
    car.controller.start_drive("wall_follow")
    with pytest.raises(ActionError, match="start the park launch first"):
        car.controller.park_command("start")


def test_park_start_turn_and_abort(car):
    car.controller.start_drive("park", profile="floor-2026-10-07")
    car.controller.park_command("start")
    car.controller.park_command("three_point_turn")
    car.controller.park_command("abort")
    scripts = [c[-1] for c in car.runner.calls if "ros2 service call" in c[-1]]
    assert [s.split()[-3] for s in scripts] == [
        "/park_node/start",
        "/park_node/three_point_turn",
        "/park_node/abort",
    ]
    assert all(s.endswith('std_srvs/srv/Trigger "{}"') for s in scripts)


def test_park_start_is_gated_but_abort_is_not(car):
    car.controller.start_drive("park")
    car.ros.set_value("fault", "FAULT_CODE_UNDER_VOLTAGE")
    with pytest.raises(ActionError, match="not all green"):
        car.controller.park_command("start")
    car.controller.park_command("start", override=True, override_reason="bench")
    assert car.last_log()["override"] is True
    car.controller.park_command("abort")  # no checks in the way


def test_park_abort_works_even_with_a_dead_recorder(car):
    car.controller.start_drive("park")
    car.world.recorder = False
    car.controller.park_command("abort")
    with pytest.raises(ActionError):
        car.controller.park_command("start")


def test_park_service_failure_is_reported(car):
    car.controller.start_drive("park")
    car.world.park_reply = "service not available"
    with pytest.raises(ActionError, match="service call failed"):
        car.controller.park_command("start")


def test_unknown_park_command(car):
    with pytest.raises(ActionError) as exc:
        car.controller.park_command("reverse_into_wall")
    assert exc.value.status == 400


# -- maps -------------------------------------------------------------------------------------


def test_save_map_needs_mapping_running(car):
    with pytest.raises(ActionError, match="start the stack with mapping"):
        car.controller.save_map("floor-1")


def test_save_map_writes_the_map_and_lists_it(env):
    env.world.start_stack("lidar:=true vesc:=true mapping:=true")
    result = env.controller.save_map("floor-1", actor="me")
    assert result["name"] == "floor-1"
    assert (env.settings.maps / "floor-1.yaml").exists()
    assert "floor-1.yaml" in env.controller.maps()
    script = next(c[-1] for c in env.runner.calls if "save_map" in c[-1])
    assert "data: /workspace/data/maps/floor-1}" in script
    with pytest.raises(ActionError, match="already exists"):
        env.controller.save_map("floor-1")


@pytest.mark.parametrize("name", ["", "../x", "a b", "a;b", "x" * 80, ".hidden", "a/b"])
def test_save_map_rejects_bad_names(env, name):
    env.world.start_stack("lidar:=true vesc:=true mapping:=true")
    with pytest.raises(ActionError) as exc:
        env.controller.save_map(name)
    assert exc.value.status == 400


def test_save_map_failure_is_reported(env):
    env.world.start_stack("lidar:=true vesc:=true mapping:=true")
    env.runner.responders.insert(0, lambda argv: (1, "") if "save_map" in argv[-1] else None)
    with pytest.raises(ActionError, match="save_map failed"):
        env.controller.save_map("floor-2")


# -- bags -------------------------------------------------------------------------------------


def make_bag(env, name, size=10, age=0):
    folder = env.settings.bags / name
    folder.mkdir(parents=True)
    (folder / "a.mcap").write_bytes(b"x" * size)
    stamp = env.now - age
    os.utime(folder, (stamp, stamp))
    return folder


def test_bag_listing_has_sizes_newest_first_and_marks_the_active_one(car):
    make_bag(car, "old", size=2_000_000, age=1000)
    make_bag(car, "new", size=500_000, age=10)
    bags = car.controller.list_bags()
    assert [b["name"] for b in bags] == ["new", "old"]
    assert bags[1]["size_mb"] == 2.0
    assert bags[0]["active"] is True and bags[1]["active"] is False


def test_nothing_is_active_without_a_recorder(env):
    make_bag(env, "only")
    assert env.controller.list_bags()[0]["active"] is False


def test_bag_listing_without_a_bag_dir(env):
    assert env.controller.list_bags() == []


def test_delete_bag_needs_the_name_repeated(car):
    make_bag(car, "old", age=1000)
    make_bag(car, "new", age=10)
    with pytest.raises(ActionError, match="repeat the bag name"):
        car.controller.delete_bag("old", "yes")
    car.controller.delete_bag("old", "old", actor="me")
    assert not (car.settings.bags / "old").exists()
    assert car.last_log()["action"] == "bag.delete"


def test_the_bag_being_recorded_cannot_be_deleted(car):
    make_bag(car, "old", age=1000)
    make_bag(car, "new", age=10)
    with pytest.raises(ActionError, match="being recorded"):
        car.controller.delete_bag("new", "new")
    assert (car.settings.bags / "new").exists()


@pytest.mark.parametrize("name", ["..", "../car", "a/b", ".hidden", "", "x;y y"])
def test_bag_names_are_confined(env, name):
    make_bag(env, "real")
    with pytest.raises(ActionError) as exc:
        env.controller.delete_bag(name, name)
    assert exc.value.status == 400
    assert (env.settings.bags / "real").exists()


def test_symlinked_and_missing_bags_are_not_deleted(env, tmp_path):
    elsewhere = tmp_path / "precious"
    elsewhere.mkdir()
    env.settings.bags.mkdir(parents=True)
    (env.settings.bags / "link").symlink_to(elsewhere, target_is_directory=True)
    with pytest.raises(ActionError) as exc:
        env.controller.delete_bag("link", "link")
    assert exc.value.status == 404 and elsewhere.exists()
    with pytest.raises(ActionError) as exc2:
        env.controller.delete_bag("ghost", "ghost")
    assert exc2.value.status == 404


# -- status ---------------------------------------------------------------------------------


def test_status_summarises_the_car(car):
    car.controller.start_drive("wall_follow")
    status = car.controller.status()
    assert status["stack"] == {
        "running": True,
        "recorder": True,
        "flags": {"lidar": "true", "vesc": "true"},
    }
    assert status["drive"] == {"mode": "wall_follow", "running": True, "external": []}
    assert status["checks"]["ready"] is True
    assert status["policy"]["available"] is False
    assert status["autostart"] is False and status["ros_connected"] is True


def test_status_with_the_car_off(env):
    status = env.controller.status()
    assert status["stack"]["running"] is False and status["drive"]["mode"] is None
    assert status["checks"]["ready"] is False


def test_snapshot_is_cached_briefly(car):
    car.controller.snapshot()
    n = len(car.runner.calls)
    car.controller.snapshot()
    assert len(car.runner.calls) == n
    car.now += 5
    car.controller.snapshot()
    assert len(car.runner.calls) > n


def test_drive_log_and_options(car):
    assert "launch said something" in car.controller.drive_log()
    options = car.controller.options()
    assert options["gap_follow_profiles"] == list(cmd.GAP_FOLLOW_PROFILES)
    assert options["drive_modes"] == list(cmd.DRIVE_MODES)


def test_status_survives_docker_being_down(env):
    env.runner.responders.insert(0, lambda argv: (1, "") if argv[0] == "docker" else None)
    status = env.controller.status()
    assert status["containers"] == {} and status["stack"]["running"] is False


def test_heartbeat_unknown_when_systemctl_is_missing(env):
    env.runner.responders.insert(0, lambda argv: (127, "") if argv[0] == "systemctl" else None)
    assert env.controller.snapshot(fresh=True).heartbeat_active is None


# -- Plug-and-Play -------------------------------------------------------------------------------


def make_pnp_bundle(env, name, kind="dynamics"):
    folder = env.settings.pnp_models_dir / name
    folder.mkdir(parents=True)
    (folder / "manifest.yaml").write_text(f"kind: {kind}\n")
    return folder


def wire_pnp(car, dynamics=("dyn-v0-2026-11-12",), perception=("trackperc-v0-2026-11-12",)):
    for name in dynamics:
        make_pnp_bundle(car, name, "dynamics")
    for name in perception:
        make_pnp_bundle(car, name, "track_perception")
    launch = car.repo / "ros_ws/src/racer_bringup/launch"
    launch.mkdir(parents=True, exist_ok=True)
    (launch / "pnp.launch.py").write_text("")


def test_pnp_is_disabled_without_a_dynamics_bundle(car):
    status = car.controller.pnp_status()
    assert not status["available"] and "no dynamics bundle" in status["reason"]
    with pytest.raises(ActionError, match="Plug-and-Play is disabled: no dynamics bundle") as exc:
        car.controller.start_drive("pnp", pnp_backend="geometric")
    assert exc.value.status == 409
    assert not detached_execs(car)


def test_pnp_is_disabled_without_the_launch_file(car):
    make_pnp_bundle(car, "dyn-v0-2026-11-12")
    status = car.controller.pnp_status()
    assert not status["available"] and "pnp.launch.py" in status["reason"]


def test_pnp_lists_bundles_by_kind_and_ignores_the_rest(car):
    wire_pnp(car, dynamics=("dyn-a", "dyn-b"), perception=("trackperc-a",))
    base = car.settings.pnp_models_dir
    (base / "no-manifest").mkdir()
    (base / "bad-yaml").mkdir()
    (base / "bad-yaml" / "manifest.yaml").write_text(": : [\n")
    (base / "not-a-mapping").mkdir()
    (base / "not-a-mapping" / "manifest.yaml").write_text("- 1\n")
    make_pnp_bundle(car, "other-kind", "lap_memory")
    (base / "stray-file").write_text("x")
    status = car.controller.pnp_status()
    assert status["available"] is True and status["reason"] == ""
    assert status["dynamics"] == ["dyn-a", "dyn-b"]
    assert status["perception"] == ["trackperc-a"]
    assert status["backends"] == ["net", "geometric"]
    assert status["speed_range"] == [0.2, 1.5]
    assert car.controller.options()["pnp"] == status


def test_pnp_starts_with_the_geometric_backend_and_needs_no_pose(car):
    wire_pnp(car)
    result = car.controller.start_drive(
        "pnp", pnp_backend="geometric", pnp_v_max_mps=0.4, pnp_dynamics="dyn-v0-2026-11-12"
    )
    assert result["mode"] == "pnp:geometric"
    command = detached_execs(car)[0][-1]
    assert "ros2 launch racer_bringup pnp.launch.py" in command
    assert "dynamics_bundle:=/workspace/data/pnp/models/dyn-v0-2026-11-12" in command
    assert "backend:=geometric v_max_mps:=0.40" in command
    assert "perception_bundle" not in command
    assert car.last_log()["action"] == "drive.start.pnp" and car.last_log()["ok"]


def test_pnp_net_backend_names_both_bundles(car):
    wire_pnp(car)
    car.controller.start_drive(
        "pnp",
        pnp_backend="net",
        pnp_dynamics="dyn-v0-2026-11-12",
        pnp_perception="trackperc-v0-2026-11-12",
    )
    command = detached_execs(car)[0][-1]
    assert "perception_bundle:=/workspace/data/pnp/models/trackperc-v0-2026-11-12" in command
    assert "backend:=net v_max_mps:=0.50" in command


def test_pnp_net_backend_without_a_perception_bundle_is_refused(car):
    wire_pnp(car, perception=())
    with pytest.raises(ActionError, match="needs a perception bundle") as exc:
        car.controller.start_drive("pnp", pnp_backend="net", pnp_dynamics="dyn-v0-2026-11-12")
    assert exc.value.status == 400 and not detached_execs(car)


def test_pnp_with_several_bundles_must_be_told_which_and_only_accepts_listed_ones(car):
    wire_pnp(car, dynamics=("dyn-a", "dyn-b"), perception=("tp-a", "tp-b"))
    with pytest.raises(ActionError, match="no dynamics bundle in the request"):
        car.controller.start_drive("pnp", pnp_backend="geometric")
    with pytest.raises(ActionError, match="unknown dynamics bundle"):
        car.controller.start_drive("pnp", pnp_dynamics="../../etc", pnp_backend="geometric")
    with pytest.raises(ActionError, match="needs a perception bundle"):
        car.controller.start_drive("pnp", pnp_dynamics="dyn-b", pnp_backend="net")
    car.controller.start_drive(
        "pnp", pnp_dynamics="dyn-b", pnp_perception="tp-b", pnp_backend="net"
    )
    command = detached_execs(car)[0][-1]
    assert "dynamics_bundle:=/workspace/data/pnp/models/dyn-b" in command
    assert "perception_bundle:=/workspace/data/pnp/models/tp-b" in command


def test_pnp_refuses_a_bad_backend_or_speed_before_anything_starts(car):
    wire_pnp(car)
    with pytest.raises(ActionError, match="backend must be one of") as exc:
        car.controller.start_drive("pnp", pnp_backend="classical", pnp_dynamics="dyn-v0-2026-11-12")
    assert exc.value.status == 400
    for speed in (0.1, 2.0):
        with pytest.raises(ActionError, match="speed cap") as exc:
            car.controller.start_drive(
                "pnp",
                pnp_backend="geometric",
                pnp_v_max_mps=speed,
                pnp_dynamics="dyn-v0-2026-11-12",
            )
        assert exc.value.status == 400
    assert not detached_execs(car)


def test_pnp_goes_through_the_same_pre_drive_gate_and_recorder_check(car):
    wire_pnp(car)
    car.world.recorder = False
    with pytest.raises(ActionError, match="invariant 5") as exc:
        car.controller.start_drive(
            "pnp",
            pnp_backend="geometric",
            pnp_dynamics="dyn-v0-2026-11-12",
            override=True,
            override_reason="in a hurry",
        )
    assert exc.value.data["overridable"] is False
    assert not detached_execs(car)


def test_pnp_one_launch_at_a_time_and_stop_all_stops_it(car):
    wire_pnp(car)
    car.controller.start_drive("pnp", pnp_backend="geometric", pnp_dynamics="dyn-v0-2026-11-12")
    with pytest.raises(ActionError, match="already running"):
        car.controller.start_drive("pnp", pnp_backend="geometric", pnp_dynamics="dyn-v0-2026-11-12")
    with pytest.raises(ActionError, match="already running"):
        car.controller.start_drive("gap_follow")
    car.controller.stop_all()
    assert car.controller.drive_mode() is None


# ---- bag growth rate (lean bags: the app shows how fast the recording bag is growing) ------


def grow(folder, extra: int):
    with open(folder / "a.mcap", "ab") as fh:
        fh.write(b"x" * extra)


def test_growth_rate_is_none_on_the_first_poll_then_megabytes_per_second(car):
    folder = make_bag(car, "run", size=1_000_000, age=10)
    first = car.controller.list_bags()[0]
    assert first["active"] is True and first["growth_mb_s"] is None
    car.now += 10.0
    grow(folder, 5_000_000)
    second = car.controller.list_bags()[0]
    assert second["growth_mb_s"] == 0.5
    assert second["size_mb"] == 6.0


def test_growth_rate_is_zero_for_an_idle_bag(car):
    make_bag(car, "run", size=1_000_000, age=10)
    car.controller.list_bags()
    car.now += 6.0
    assert car.controller.list_bags()[0]["growth_mb_s"] == 0.0


def test_growth_rate_averages_over_the_window_not_just_the_last_poll(car):
    folder = make_bag(car, "run", size=0, age=10)
    car.controller.list_bags()
    for _ in range(4):  # 6 MB per 6 s poll: 1 MB/s
        car.now += 6.0
        grow(folder, 6_000_000)
        rate = car.controller.list_bags()[0]["growth_mb_s"]
    assert rate == 1.0
    car.now += 6.0  # then the recorder goes quiet for one poll
    assert car.controller.list_bags()[0]["growth_mb_s"] == pytest.approx(0.8, abs=0.01)


def test_two_polls_within_a_second_do_not_skew_the_rate(car):
    folder = make_bag(car, "run", size=0, age=10)
    car.controller.list_bags()
    car.now += 10.0
    grow(folder, 10_000_000)
    car.controller.list_bags()
    car.now += 0.2
    assert car.controller.list_bags()[0]["growth_mb_s"] == 1.0


def test_growth_rate_does_not_average_across_a_long_gap(car):
    folder = make_bag(car, "run", size=0, age=10)
    car.controller.list_bags()
    car.now += 3600.0
    grow(folder, 3_600_000)
    assert car.controller.list_bags()[0]["growth_mb_s"] is None  # starts over
    car.now += 6.0
    grow(folder, 6_000_000)
    assert car.controller.list_bags()[0]["growth_mb_s"] == 1.0


def test_growth_rate_starts_over_for_a_new_bag_and_only_the_active_bag_has_one(car):
    make_bag(car, "old", size=1_000_000, age=1000)
    folder = make_bag(car, "new", size=1_000_000, age=10)
    car.controller.list_bags()
    car.now += 10.0
    grow(folder, 10_000_000)
    bags = car.controller.list_bags()
    assert bags[0]["growth_mb_s"] == 1.0
    assert "growth_mb_s" not in bags[1]
    make_bag(car, "newer", size=0, age=0)
    assert car.controller.list_bags()[0]["growth_mb_s"] is None


def test_growth_state_is_forgotten_when_nothing_is_recording(env):
    make_bag(env, "run", size=1_000_000, age=10)
    assert "growth_mb_s" not in env.controller.list_bags()[0]
    env.running_car()
    env.controller._invalidate()
    env.controller.list_bags()
    env.now += 10.0
    assert env.controller.list_bags()[0]["growth_mb_s"] == 0.0
    env.world.containers["car-stack"]["state"] = "exited"
    env.controller._invalidate()
    assert env.controller.list_bags()[0]["active"] is False
    assert env.controller._growth_samples == type(env.controller._growth_samples)()
