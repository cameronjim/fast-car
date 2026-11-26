"""L1: pnp.launch.py profiles in the sim harness (pnp_profile.py), ROS free."""

from __future__ import annotations

import json
from pathlib import Path

import pnp_profile as pp
import pytest

LAUNCH = Path(__file__).resolve().parents[2] / "ros_ws/src/racer_bringup/launch/pnp.launch.py"
MODELS = "/data/models"


def test_reads_the_launch_profiles_without_ros():
    profiles = pp.load_profiles(LAUNCH)
    assert profiles["none"] == {}
    proto = profiles["prototype"]
    assert proto["backend"] == "net"
    assert proto["lap_source"] == "camera"


def test_prototype_maps_every_key_and_moves_the_bundles():
    argv, notes = pp.profile_argv(pp.load_profiles(LAUNCH)["prototype"], MODELS)
    proto = pp.load_profiles(LAUNCH)["prototype"]

    def value(opt):
        return argv[argv.index(opt) + 1]

    assert value("--backend") == proto["backend"]
    assert value("--perception-bundle") == MODELS + "/" + Path(proto["perception_bundle"]).name
    assert value("--dynamics-bundle") == MODELS + "/" + Path(proto["dynamics_bundle"]).name
    assert value("--v-max-mps") == proto["v_max_mps"]
    assert value("--v-lap1-mps") == proto["v_lap1_mps"]
    assert value("--dv1-variant") == proto["dv1_variant"]
    assert "--dynamics-fast-mlp" in argv and "--no-dynamics-adaptation" in argv
    i = argv.index("--lap-length-ratio")
    assert argv[i + 1 : i + 3] == [proto["lap_length_ratio_min"], proto["lap_length_ratio_max"]]
    assert value("--lap-memory") == "v1"
    # the camera has no sim counterpart: the simulator's oracle on simulated time stands in
    assert value("--lap-source") == "truth" and "--sim-time" in argv
    assert notes["not_applied"] == {"lap_counter": "true", "lap_rate_hz": proto["lap_rate_hz"]}


def test_an_unknown_profile_key_refuses():
    with pytest.raises(ValueError, match="does not know"):
        pp.profile_argv({"backend": "net", "rt_sched": "on"}, MODELS)


def test_bad_boolean_refuses():
    with pytest.raises(ValueError, match="true or false"):
        pp.profile_argv({"dynamics_fast_mlp": "yes"}, MODELS)


@pytest.mark.parametrize(
    ("memory", "mode", "want"),
    [("false", "v1", "none"), ("true", "v0", "v0"), ("true", "observe", "off")],
)
def test_lap_memory_and_mode(memory, mode, want):
    argv, _ = pp.profile_argv({"lap_memory": memory, "lap_mode": mode}, MODELS)
    assert argv == ["--lap-memory", want]


def test_a_path_outside_the_car_models_dir_is_kept():
    argv, _ = pp.profile_argv({"dynamics_bundle": "/somewhere/dyn"}, MODELS)
    assert argv == ["--dynamics-bundle", "/somewhere/dyn"]


def test_expand_puts_the_profile_first_so_explicit_arguments_win():
    profiles = {"none": {}, "p": {"v_max_mps": "1.3"}}
    argv, notes = pp.expand_profile_argv(["--pnp-profile", "p", "--v-max-mps", "0.9"], profiles)
    assert argv == ["--v-max-mps", "1.3", "--pnp-profile", "p", "--v-max-mps", "0.9"]
    assert notes["profile"] == "p"
    same, none = pp.expand_profile_argv(["--v-max-mps", "0.9"], profiles)
    assert same == ["--v-max-mps", "0.9"] and none == {}
    argv, _ = pp.expand_profile_argv(
        ["--pnp-profile=p", f"--pnp-models-dir={MODELS}"],
        {"p": {"dynamics_bundle": pp.CAR_MODELS_DIR + "/d"}},
    )
    assert argv[:2] == ["--dynamics-bundle", MODELS + "/d"]
    with pytest.raises(ValueError, match="must be one of"):
        pp.expand_profile_argv(["--pnp-profile", "nope"], profiles)


def test_stats_count_only_from_driving_and_find_cuts_and_v_collapses():
    s = pp.ProfileStats()
    s.on_planner(0, 0.050, 512, "d", "p")  # STARTING: not counted
    s.on_track(0, "p")  # before DRIVING: not counted
    for solve, samples in ((0.003, 512), (0.020, 256), (0.004, 512)):
        s.on_planner(1, solve, samples, "d", "p")
    for v in (16, 1, 0, 16, 0, 15):
        s.on_track(v, "p")
    s.on_perception_status(json.dumps({"model_id": "p"}))
    s.on_perception_status("not json")
    out = s.summary()
    assert out["solve_ms"]["n"] == 3 and out["solve_ms"]["over_15ms"] == 1
    assert out["solve_ms"]["max"] == pytest.approx(20.0)
    assert out["samples"] == {
        "max": 512,
        "min": 256,
        "ticks_below_max": 1,
        "share_below_max": 0.3333,
    }
    v = out["valid_count"]
    assert (
        v["frames"],
        v["zero"],
        v["unusable"],
        v["unusable_runs"],
        v["unusable_longest_frames"],
    ) == (
        6,
        2,
        3,
        2,
        2,
    )
    assert out["planner_model_ids"] == {"d|p": 3}
    assert out["perception_status_count"] == 2 and "unparsed" in out["perception_status"]


def test_empty_stats():
    out = pp.ProfileStats().summary()
    assert out["solve_ms"]["p50"] is None and out["samples"]["max"] == 0
    assert out["valid_count"]["mean"] is None


def test_node_log_summary(tmp_path):
    (tmp_path / "mppi_planner_node.log").write_text(
        "[WARN] [1.0] [mppi_planner_node]: sample count 512 -> 256 (last solve 16.0 ms)\n"
        "[WARN] [2.0] [mppi_planner_node]: sample count 256 -> 384 (last solve 3.0 ms)\n"
        "[ERROR] [3.0] [mppi_planner_node]: something\n"
    )
    (tmp_path / "bridge_node.log").write_text("Traceback (most recent call last):\n")
    out = pp.node_log_summary(tmp_path, ["mppi_planner_node", "bridge_node", "absent"])
    assert out["mppi_planner_node"]["sample_cuts"] == 1
    assert out["mppi_planner_node"]["WARN"] == 2 and out["mppi_planner_node"]["ERROR"] == 1
    assert len(out["mppi_planner_node"]["first_errors"]) == 1
    assert out["bridge_node"]["traceback"] == 1
    assert "absent" not in out
