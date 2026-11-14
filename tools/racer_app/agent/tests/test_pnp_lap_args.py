"""Brief 35: the lap learning, lap counter and shadow net arguments of pnp.launch.py, through the
same closed whitelist the PnP card already uses (docs 13 section 15.7)."""

from __future__ import annotations

import re
from pathlib import Path

import pytest
from conftest import TOKEN
from fastapi.testclient import TestClient
from hypothesis import given, settings
from hypothesis import strategies as st
from racer_agent import commands as cmd
from racer_agent.controller import ActionError
from racer_agent.prefs import Prefs
from racer_agent.service import create_app

REPO_ROOT = Path(__file__).resolve().parents[4]
LAUNCH = REPO_ROOT / "ros_ws/src/racer_bringup/launch/pnp.launch.py"
PNP = "/workspace/data/pnp/models/"
DYN = "dyn-v0-2026-11-12"
NET = "trackperc-v0-2026-11-12"
SHADOW = "trackperc-shadow"
AUTH = {"Authorization": f"Bearer {TOKEN}"}

#: Two fixture launch texts: one with an `observe` lap mode, one without (brief 29 adds it).
LAUNCH_WITHOUT_OBSERVE = """
LAP_MODES = ("v0", "v1")
LAP_SOURCES = ("start_scan", "map_pose")
def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument("lap_memory", default_value="false", choices=["true", "false"]),
        DeclareLaunchArgument("lap_mode", default_value="v0", choices=list(LAP_MODES)),
        DeclareLaunchArgument("lap_source", default_value="start_scan", choices=list(LAP_SOURCES)),
        DeclareLaunchArgument("lap_counter", default_value="true", choices=["true", "false"]),
        DeclareLaunchArgument("require_lap_counter", default_value="false"),
        DeclareLaunchArgument("v_lap1_mps", default_value=""),
        DeclareLaunchArgument("shadow_perception_bundle", default_value=""),
    ])
"""
LAUNCH_WITH_OBSERVE = LAUNCH_WITHOUT_OBSERVE.replace(
    'LAP_MODES = ("v0", "v1")', 'LAP_MODES = ("v0", "v1", "observe")'
)
OLD_LAUNCH = """
def generate_launch_description():
    return LaunchDescription([DeclareLaunchArgument("backend", default_value="geometric")])
"""

DECL_NO_OBSERVE = cmd.parse_launch_declarations(LAUNCH_WITHOUT_OBSERVE)
DECL_OBSERVE = cmd.parse_launch_declarations(LAUNCH_WITH_OBSERVE)

# The command of main, byte for byte, before this brief.
MAIN_GEOMETRIC = (
    f"ros2 launch racer_bringup pnp.launch.py dynamics_bundle:={PNP}{DYN} "
    "backend:=geometric v_max_mps:=0.50"
)
MAIN_NET_PARAMS = (
    f"ros2 launch racer_bringup pnp.launch.py perception_bundle:={PNP}{NET} "
    f"dynamics_bundle:={PNP}{DYN} backend:=net v_max_mps:=0.80 "
    "params_file:=/workspace/data/pnp/planner-a.yaml"
)


def build(lap: cmd.PnpLap | None = None, launch=DECL_OBSERVE, **kw) -> str:
    args = {
        "pnp_dynamics_container_path": PNP + DYN,
        "pnp_backend": "geometric",
        "pnp_v_max_mps": 0.5,
    }
    args.update(kw)
    return cmd.ros_cmd_for("pnp", pnp_lap=lap, pnp_launch=launch, **args)


def lap(**kw) -> cmd.PnpLap:
    return cmd.PnpLap(**kw)


# -- defaults are today's command ----------------------------------------------------------------


def test_the_default_request_is_the_command_of_main_byte_for_byte():
    assert build() == MAIN_GEOMETRIC
    assert build(None, launch=None) == MAIN_GEOMETRIC
    assert build(lap(), launch=DECL_NO_OBSERVE) == MAIN_GEOMETRIC
    # even against a launch file that has none of the lap arguments (nothing is sent)
    assert build(lap(), launch=cmd.parse_launch_declarations(OLD_LAUNCH)) == MAIN_GEOMETRIC
    full = cmd.ros_cmd_for(
        "pnp",
        pnp_perception_container_path=PNP + NET,
        pnp_dynamics_container_path=PNP + DYN,
        pnp_backend="net",
        pnp_v_max_mps=0.8,
        pnp_params_container_path="/workspace/data/pnp/planner-a.yaml",
        pnp_lap=lap(),
        pnp_launch=DECL_OBSERVE,
    )
    assert full == MAIN_NET_PARAMS


# -- each field, accepted values -----------------------------------------------------------------


def test_v_lap1_is_formatted_and_needs_lap_memory():
    out = build(lap(v_lap1_mps=0.3, lap_memory=True), pnp_v_max_mps=0.8)
    assert out == f"{MAIN_GEOMETRIC.replace('0.50', '0.80')} v_lap1_mps:=0.30 lap_memory:=true"
    assert " v_lap1_mps:=1.00 " not in out
    assert build(lap(v_lap1_mps=1, lap_memory=True), pnp_v_max_mps=1).endswith(
        "v_lap1_mps:=1.00 lap_memory:=true"
    )


@pytest.mark.parametrize("value", [True, False])
def test_lap_memory_true_is_sent_and_false_is_not(value):
    out = build(lap(lap_memory=value))
    assert out.endswith(" lap_memory:=true") == value
    assert ("lap_memory" in out) == value


@pytest.mark.parametrize(
    ("mode", "tail"),
    [
        ("v0", " lap_memory:=true"),
        ("v1", " lap_memory:=true lap_mode:=v1"),
        ("observe", " lap_memory:=true lap_mode:=observe"),
    ],
)
def test_lap_mode_values(mode, tail):
    assert build(lap(lap_memory=True, lap_mode=mode)) == MAIN_GEOMETRIC + tail


def test_observe_without_lap_memory_sends_both():
    assert build(lap(lap_mode="observe")) == (
        MAIN_GEOMETRIC + " lap_memory:=true lap_mode:=observe"
    )


def test_lap_source_values():
    assert build(lap(lap_memory=True, lap_source="start_scan")).endswith("lap_memory:=true")
    assert build(lap(lap_memory=True, lap_source="map_pose")).endswith(
        "lap_memory:=true lap_source:=map_pose"
    )


def test_lap_counter_and_require_values():
    assert build(lap(lap_counter=True)) == MAIN_GEOMETRIC
    assert build(lap(lap_counter=False)) == MAIN_GEOMETRIC + " lap_counter:=false"
    assert build(lap(require_lap_counter=True)) == MAIN_GEOMETRIC + " require_lap_counter:=true"


def test_the_shadow_bundle_is_sent_through_the_bundle_check():
    out = build(lap(shadow_container_path=PNP + SHADOW))
    assert out == f"{MAIN_GEOMETRIC} shadow_perception_bundle:={PNP}{SHADOW}"


def test_every_argument_together_in_a_fixed_order():
    out = build(
        lap(
            v_lap1_mps=0.4,
            lap_memory=True,
            lap_mode="v1",
            lap_source="start_scan",
            lap_counter=False,
            require_lap_counter=False,
            shadow_container_path=PNP + SHADOW,
        ),
        pnp_v_max_mps=0.9,
        pnp_perception_container_path=PNP + NET,
        pnp_backend="net",
    )
    assert out.endswith(
        " v_lap1_mps:=0.40 lap_memory:=true lap_mode:=v1 lap_counter:=false "
        f"shadow_perception_bundle:={PNP}{SHADOW}"
    )


# -- refusals ------------------------------------------------------------------------------------


@pytest.mark.parametrize(
    ("kw", "message"),
    [
        ({"lap_mode": "v1"}, "needs lap memory"),
        ({"lap_memory": True, "lap_mode": "v1", "lap_source": "map_pose"}, "start_scan"),
        ({"v_lap1_mps": 0.3}, "needs lap memory on or lap mode observe"),
        ({"v_lap1_mps": 0.6, "lap_memory": True}, "cannot exceed the speed cap"),
        ({"v_lap1_mps": 0.1, "lap_memory": True}, "between"),
        ({"v_lap1_mps": 9, "lap_memory": True}, "between"),
        ({"require_lap_counter": True, "lap_counter": False}, "needs the lap counter on"),
        ({"lap_mode": "v2", "lap_memory": True}, "lap mode must be one of"),
        ({"lap_mode": "v0; reboot"}, "lap mode must be one of"),
        ({"lap_source": "gps"}, "lap source must be one of"),
    ],
)
def test_each_rule_is_a_command_error_with_a_plain_message(kw, message):
    with pytest.raises(cmd.CommandError, match=message):
        build(lap(**kw))


def test_the_shadow_bundle_may_not_be_the_driving_one():
    with pytest.raises(cmd.CommandError, match="cannot be the driving one"):
        build(
            lap(shadow_container_path=PNP + NET),
            pnp_perception_container_path=PNP + NET,
            pnp_backend="net",
        )
    # a geometric drive has no driving perception bundle, so any listed bundle is fine
    assert SHADOW in build(lap(shadow_container_path=PNP + SHADOW))


@pytest.mark.parametrize(
    "kw",
    [
        {"lap_memory": "true"},
        {"lap_memory": 1},
        {"lap_memory": None},
        {"lap_counter": "false"},
        {"lap_counter": 0},
        {"require_lap_counter": "true"},
        {"require_lap_counter": 1},
        {"v_lap1_mps": "0.3", "lap_memory": True},
        {"v_lap1_mps": True, "lap_memory": True},
        {"v_lap1_mps": [0.3], "lap_memory": True},
        {"lap_mode": 1},
        {"lap_source": ["start_scan"]},
        {"shadow_container_path": 5},
    ],
)
def test_wrong_types_are_refused(kw):
    with pytest.raises(cmd.CommandError):
        build(lap(**kw))


@pytest.mark.parametrize(
    "name",
    ["x; reboot", "$(reboot)", "x y", "a/b", "..", "x`id`", "x|y", "x&y", "x\nreboot", "x>y"],
)
def test_shell_metacharacters_in_the_shadow_bundle_name_are_refused(name):
    with pytest.raises(cmd.CommandError):
        build(lap(shadow_container_path=PNP + name))


# -- the launch file decides what exists in this build ------------------------------------------


def test_observe_is_refused_by_a_launch_file_without_it_and_accepted_by_one_with_it():
    request = lap(lap_memory=True, lap_mode="observe")
    with pytest.raises(cmd.CommandError, match="not available in this build"):
        build(request, launch=DECL_NO_OBSERVE)
    assert "lap_mode:=observe" in build(request, launch=DECL_OBSERVE)


def test_a_launch_file_without_the_lap_arguments_refuses_each_of_them():
    old = cmd.parse_launch_declarations(OLD_LAUNCH)
    for request in (
        lap(lap_memory=True),
        lap(v_lap1_mps=0.3, lap_memory=True),
        lap(lap_counter=False),
        lap(require_lap_counter=True),
        lap(shadow_container_path=PNP + SHADOW),
    ):
        with pytest.raises(cmd.CommandError, match="not available in this build"):
            build(request, launch=old)


def test_the_declaration_parser_reads_choices_by_name_and_literal():
    decl = cmd.parse_launch_declarations(LAUNCH_WITH_OBSERVE)
    assert decl["lap_mode"] == ("v0", "v1", "observe")
    assert decl["lap_memory"] == ("true", "false")
    assert decl["v_lap1_mps"] is None
    assert cmd.parse_launch_declarations("def broken(:") == {}


# -- mirror: the app only sends what the real launch file declares -------------------------------


@pytest.mark.skipif(not LAUNCH.exists(), reason="repo launch file not present")
def test_every_argument_the_app_sends_is_declared_by_pnp_launch_py():
    declared = cmd.parse_launch_declarations(LAUNCH.read_text())
    sent: set[str] = set()
    for counter, require in ((False, False), (True, True)):
        out = cmd.ros_cmd_for(
            "pnp",
            pnp_perception_container_path=PNP + NET,
            pnp_dynamics_container_path=PNP + DYN,
            pnp_backend="net",
            pnp_v_max_mps=1.0,
            pnp_params_container_path="/workspace/data/pnp/p.yaml",
            pnp_lap=lap(
                v_lap1_mps=0.5,
                lap_memory=True,
                lap_mode="v1",
                lap_source="start_scan",
                lap_counter=counter,
                require_lap_counter=require,
                shadow_container_path=PNP + SHADOW,
            ),
            pnp_launch=declared,
        )
        sent |= set(re.findall(r" ([a-z0-9_]+):=", out))
    assert {
        "v_lap1_mps", "lap_memory", "lap_mode", "lap_counter", "require_lap_counter",
        "shadow_perception_bundle",
    } <= sent  # fmt: skip
    for name in sent:
        assert name in declared, f"the app sends {name}, pnp.launch.py does not declare it"


@pytest.mark.skipif(not LAUNCH.exists(), reason="repo launch file not present")
def test_every_choice_the_app_allows_is_a_declared_choice():
    declared = cmd.parse_launch_declarations(LAUNCH.read_text())
    # `observe` is added by brief 29: until then the agent refuses it by reading the file, so it
    # is the one allowed mode that may be missing from the launch file.
    assert set(cmd.PNP_LAP_MODES) - {"observe"} <= set(declared["lap_mode"])
    assert set(declared["lap_mode"]) <= set(cmd.PNP_LAP_MODES)
    assert set(cmd.PNP_LAP_SOURCES) == set(declared["lap_source"])
    for name in ("lap_memory", "lap_counter", "require_lap_counter"):
        assert name in declared


# -- L2: any request is refused or made only of whitelisted tokens -------------------------------

_TOKEN = re.compile(
    r"^ros2 launch racer_bringup pnp\.launch\.py "
    r"(perception_bundle:=/workspace/data/pnp/models/[A-Za-z0-9][A-Za-z0-9_.-]{0,63} )?"
    r"dynamics_bundle:=/workspace/data/pnp/models/[A-Za-z0-9][A-Za-z0-9_.-]{0,63} "
    r"backend:=(net|geometric) v_max_mps:=[0-9]+\.[0-9]{2}"
    r"( params_file:=/workspace/data/pnp/[A-Za-z0-9][A-Za-z0-9_.-]{0,63}\.yaml)?"
    r"( v_lap1_mps:=[0-9]+\.[0-9]{2})?"
    r"( lap_memory:=true)?"
    r"( lap_mode:=(v1|observe))?"
    r"( lap_source:=map_pose)?"
    r"( lap_counter:=false)?"
    r"( require_lap_counter:=true)?"
    r"( shadow_perception_bundle:=/workspace/data/pnp/models/[A-Za-z0-9][A-Za-z0-9_.-]{0,63})?$"
)

_junk = st.one_of(
    st.none(),
    st.booleans(),
    st.integers(-5, 5),
    st.floats(allow_nan=True, allow_infinity=True),
    st.text(max_size=20),
    st.lists(st.integers(), max_size=2),
    st.dictionaries(st.text(max_size=3), st.integers(), max_size=2),
)
_names = st.one_of(
    st.just(""),
    st.sampled_from([PNP + DYN, PNP + NET, PNP + SHADOW, "/workspace/data/pnp/models/x;id"]),
    st.text(max_size=30).map(lambda t: PNP + t),
    _junk,
)


@settings(max_examples=400, deadline=None)
@given(
    backend=st.sampled_from(["net", "geometric", "x"]),
    speed=st.one_of(st.floats(0.0, 2.0), _junk),
    perception=_names,
    dynamics=_names,
    v_lap1=st.one_of(st.none(), st.floats(0.0, 2.0), _junk),
    memory=st.one_of(st.booleans(), _junk),
    mode=st.one_of(st.sampled_from(["v0", "v1", "observe"]), _junk),
    source=st.one_of(st.sampled_from(["start_scan", "map_pose"]), _junk),
    counter=st.one_of(st.booleans(), _junk),
    require=st.one_of(st.booleans(), _junk),
    shadow=_names,
    launch=st.sampled_from([None, DECL_NO_OBSERVE, DECL_OBSERVE]),
)
def test_any_request_is_refused_or_made_only_of_whitelisted_tokens(
    backend, speed, perception, dynamics, v_lap1, memory, mode, source, counter, require, shadow,
    launch,
):  # fmt: skip
    try:
        out = cmd.ros_cmd_for(
            "pnp",
            pnp_perception_container_path=perception,
            pnp_dynamics_container_path=dynamics,
            pnp_backend=backend,
            pnp_v_max_mps=speed,
            pnp_lap=cmd.PnpLap(v_lap1, memory, mode, source, counter, require, shadow),
            pnp_launch=launch,
        )
    except cmd.CommandError:
        return
    assert _TOKEN.match(out), out


# -- the controller, the options endpoint and the HTTP layer -------------------------------------


def make_bundle(env, name, kind):
    folder = env.settings.pnp_models_dir / name
    folder.mkdir(parents=True)
    (folder / "manifest.yaml").write_text(f"kind: {kind}\n")


@pytest.fixture
def pnp_car(env):
    env.running_car()
    make_bundle(env, DYN, "dynamics")
    make_bundle(env, NET, "track_perception")
    make_bundle(env, SHADOW, "track_perception")
    launch = env.repo / "ros_ws/src/racer_bringup/launch"
    launch.mkdir(parents=True, exist_ok=True)
    (launch / "pnp.launch.py").write_text(LAUNCH_WITHOUT_OBSERVE)
    return env


@pytest.fixture
def client(pnp_car):
    app = create_app(pnp_car.controller, pnp_car.settings, web_dist=None, start_background=False)
    return TestClient(app, client=("192.168.1.50", 40000))


def post(client, **body):
    return client.post(
        "/api/drive/start",
        headers=AUTH,
        json={"mode": "pnp", "pnp_backend": "geometric", "pnp_dynamics": DYN, **body},
    )


def test_the_options_list_the_modes_and_sources_the_launch_file_declares(pnp_car):
    status = pnp_car.controller.options()["pnp"]
    assert status["lap_modes"] == ["v0", "v1"]
    assert status["lap_sources"] == ["start_scan", "map_pose"]
    assert set(status) - {"lap_modes", "lap_sources"} == {
        "available", "reason", "perception", "dynamics", "backends", "speed_range",
        "default_speed", "params_files",
    }  # fmt: skip


def test_the_options_list_observe_once_the_launch_file_has_it(pnp_car):
    launch = pnp_car.repo / "ros_ws/src/racer_bringup/launch/pnp.launch.py"
    launch.write_text(LAUNCH_WITH_OBSERVE)
    assert pnp_car.controller.options()["pnp"]["lap_modes"] == ["v0", "v1", "observe"]


def test_a_default_http_drive_is_the_command_of_main(client, pnp_car):
    r = post(client)
    assert r.status_code == 200, r.text
    assert pnp_car.world.drive_cmd == MAIN_GEOMETRIC


def test_a_drive_request_with_the_lap_fields_round_trips(client, pnp_car):
    r = post(
        client,
        pnp_v_max_mps=0.8,
        pnp_v_lap1_mps=0.4,
        pnp_lap_memory=True,
        pnp_lap_mode="v1",
        pnp_lap_counter=False,
        pnp_shadow_perception=SHADOW,
    )
    assert r.status_code == 200, r.text
    sent = pnp_car.world.drive_cmd
    assert sent.endswith(
        " v_lap1_mps:=0.40 lap_memory:=true lap_mode:=v1 lap_counter:=false "
        f"shadow_perception_bundle:={PNP}{SHADOW}"
    )


@pytest.mark.parametrize(
    "body",
    [
        {"pnp_lap_mode": "v1"},
        {"pnp_lap_memory": "true"},
        {"pnp_lap_memory": True, "pnp_lap_mode": "v1", "pnp_lap_source": "map_pose"},
        {"pnp_v_lap1_mps": 0.3},
        {"pnp_lap_memory": True, "pnp_v_lap1_mps": "0.3"},
        {"pnp_lap_memory": True, "pnp_v_lap1_mps": 0.9},
        {"pnp_require_lap_counter": True, "pnp_lap_counter": False},
        {"pnp_lap_mode": "observe"},  # this fixture's launch file has no observe
        {"pnp_shadow_perception": "nope"},
        {"pnp_shadow_perception": "x; reboot"},
        {"pnp_lap_counter": 0},
    ],
)
def test_a_bad_lap_request_is_a_400_and_starts_nothing(client, pnp_car, body):
    r = post(client, **body)
    assert r.status_code == 400, r.text
    assert r.json()["error"]
    assert pnp_car.world.drive_cmd == ""


def test_the_shadow_bundle_may_not_be_the_driving_bundle_over_http(client, pnp_car):
    r = post(client, pnp_backend="net", pnp_perception=NET, pnp_shadow_perception=NET)
    assert r.status_code == 400 and "driving one" in r.json()["error"]


def test_an_unknown_field_is_still_a_422(client):
    assert post(client, pnp_lap_modee="v1").status_code == 422


def test_the_controller_refuses_a_shadow_bundle_that_is_not_a_perception_bundle(pnp_car):
    with pytest.raises(ActionError, match="unknown shadow perception bundle"):
        pnp_car.controller.start_drive(
            "pnp", pnp_backend="geometric", pnp_dynamics=DYN, pnp_shadow_perception=DYN
        )


# -- prefs ---------------------------------------------------------------------------------------


def test_the_default_prefs_reproduce_todays_command():
    n = Prefs().pnp
    assert (n.v_lap1_mps, n.lap_memory, n.lap_mode, n.lap_source) == (
        None,
        False,
        "v0",
        "start_scan",
    )
    assert (n.lap_counter, n.require_lap_counter, n.shadow_perception) == (True, False, "")


def test_prefs_with_the_lap_fields_save_and_a_broken_combination_does_not(pnp_car):
    prefs = Prefs()
    prefs.pnp.lap_memory = True
    prefs.pnp.lap_mode = "v1"
    prefs.pnp.v_lap1_mps = 0.4
    prefs.pnp.shadow_perception = SHADOW
    pnp_car.controller.set_prefs(prefs)
    assert pnp_car.controller.get_prefs()["prefs"]["pnp"]["lap_mode"] == "v1"
    prefs.pnp.lap_source = "map_pose"
    with pytest.raises(ActionError, match="start_scan"):
        pnp_car.controller.set_prefs(prefs)
    prefs.pnp.lap_source = "start_scan"
    prefs.pnp.shadow_perception = "x; reboot"
    with pytest.raises(ActionError):
        pnp_car.controller.set_prefs(prefs)
