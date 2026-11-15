"""L1 and L4 tests of tools/pnp_session/v04.py and its CLI commands (brief 41): the prereg hash
check, D3-C0's reference rule, lapref next to it, the steps the results allow, the variant form
check, and the golden v04 report on the committed R2 fixture (tolerance 1e-9 relative)."""

from __future__ import annotations

import json
import shutil
from pathlib import Path

import pytest
import yaml

pytest.importorskip("racer_eval")
pytest.importorskip("mcap")

import numpy as np
from pnp_session import cli, laps, v04
from pnp_session.bagread import read_session_bag
from racer_eval import variant_form
from test_pnp_session import make_bag

REPO = Path(__file__).resolve().parents[2]
FIX = Path(__file__).resolve().parent / "fixtures"
CUT = FIX / "pnp_session_r2_cut.mcap"
GOLDEN = FIX / "pnp_session_r2_v04_golden.json"
PROCEDURE = REPO / v04.PROCEDURE


# ------------------------------------------------------------------------------------ prereg


def test_check_prereg_on_this_checkout():
    assert v04.check_prereg() == v04.PREREG_SHA256
    assert cli.main(["check-prereg"]) == 0


def test_check_prereg_refuses_a_changed_or_missing_file(tmp_path, monkeypatch, capsys):
    (tmp_path / "evaluation/prereg").mkdir(parents=True)
    with pytest.raises(v04.PreregMismatch, match="pull main"):
        v04.check_prereg(tmp_path)
    shutil.copy(REPO / v04.PREREG_YAML, tmp_path / v04.PREREG_YAML)
    with pytest.raises(v04.PreregMismatch, match="addendum-1"):
        v04.check_prereg(tmp_path)  # the addendum's constants file is checked too
    shutil.copy(REPO / v04.ADDENDUM_YAML, tmp_path / v04.ADDENDUM_YAML)
    assert v04.check_prereg(tmp_path) == v04.PREREG_SHA256
    with (tmp_path / v04.PREREG_YAML).open("a") as f:
        f.write("# edited\n")
    with pytest.raises(v04.PreregMismatch, match="refusing"):
        v04.check_prereg(tmp_path)
    monkeypatch.setattr(v04, "REPO_ROOT", tmp_path)
    assert cli.main(["check-prereg"]) == cli.EXIT_PREREG
    assert "REFUSED" in capsys.readouterr().err


# ------------------------------------------------------------------------------- reference


@pytest.mark.parametrize(
    ("owner", "held", "expected"),
    [
        (True, True, "camera"),
        (True, False, "video"),
        (False, True, "video"),
        (False, False, "video"),
    ],
)
def test_reference_is_the_camera_only_if_accepted_and_m_cam_held(owner, held, expected):
    assert v04.choose_reference(owner, held) == expected


# ------------------------------------------------------------------------------ lapref next


def run_input(times=(5.5, 5.6, 5.7), ref=(5.5, 5.7, 5.7)) -> laps.RunLapInput:
    return laps.RunLapInput("R0", tuple(times), tuple(ref))


def test_read_lapref_keys(tmp_path):
    p = tmp_path / "R0.json"
    p.write_text(json.dumps({"lap_times_s": [5.5, 5.6]}))
    assert v04.read_lapref(p) == (5.5, 5.6)
    p.write_text(json.dumps({"lap_times_s": None}))
    assert v04.read_lapref(p) == ()  # "no periodic lap found": no laps
    p.write_text(json.dumps({"laps": 3}))
    with pytest.raises(ValueError, match="no lap_times_s"):
        v04.read_lapref(p)


def test_lapref_next_to_the_reference_decides_nothing(tmp_path):
    none = v04.lapref_next_to([run_input()], None)
    assert none["rows"][0]["lapref"] == "not given" and none["decides_nothing"] is True
    assert v04.lapref_next_to([run_input()], tmp_path)["rows"][0]["lapref"] == "not given"
    (tmp_path / "R0.json").write_text(json.dumps({"lap_times_s": [5.5, 6.1, 5.7]}))
    row = v04.lapref_next_to([run_input()], tmp_path)["rows"][0]
    assert row["count_equal"] and row["laps_compared"] == 3 and row["laps_within"] == 2
    assert row["diffs_s"] == pytest.approx([0.0, 0.4, 0.0])
    (tmp_path / "R0.json").write_text(json.dumps({"lap_times_s": [5.5, 5.7]}))
    row = v04.lapref_next_to([run_input()], tmp_path)["rows"][0]
    assert not row["count_equal"] and row["laps_compared"] == 2 and row["n_reference"] == 3
    edge = v04.lapref_next_to([run_input(ref=(5.0, 5.0, 5.0))], tmp_path)  # exactly 0.5 s away
    assert edge["rows"][0]["laps_within"] == 0


# -------------------------------------------------------------------------- steps allowed


def test_default_statuses_open_only_the_steps_without_results_to_wait_for():
    steps = v04.allowed_steps(v04.load_procedure(), {})
    open_steps = {s["step"] for s in steps if s["runnable"]}
    assert open_steps == {0, 1, 2, 6, 8, 9}
    ids = {s["id"] for s in steps if s["runnable"]}
    assert {"s9a_excitation", "s9_data_laps", "s1_c_timing_and_d3c0"} <= ids
    assert "s1b_gp3_r1" not in ids
    blocked = {s["step"]: s["blocked_by"] for s in steps if not s["runnable"]}
    assert blocked[5][:3] == ["d3_s0 is unknown", "d3_s1 is unknown", "d3_c0 is unknown"]
    assert blocked[5][3].startswith("either (code_equals_b4 is unknown) or (d5_s0 is unknown")
    assert blocked[1] == ["m_cam is unknown"]  # GP3-R1 needs M-CAM
    assert {s["step"] for s in steps if s["minimum_session"]} == {1, 2, 9}
    assert {s["id"] for s in steps if s["minimum_session"]} == {
        "s1_c_timing_and_d3c0",
        "s2_dynamics_confirmation",
        "s9a_excitation",
        "s9_data_laps",
    }


def test_results_open_steps_and_unmet_blocks_like_unknown():
    proc = v04.load_procedure()
    met = {"d3_s0": "met", "d3_s1": "met", "d3_c0": "met"}

    def runnable(results, step):
        return next(s for s in v04.allowed_steps(proc, results) if s["step"] == step)["runnable"]

    assert not runnable(met, 5)  # B4 gates alone: neither the B4 code nor B6
    assert runnable({**met, "code_equals_b4": "met"}, 5)
    assert runnable({**met, "d5_s0": "met", "d5_s1": "met"}, 5)
    assert not runnable({**met, "d5_s0": "met"}, 5)
    assert not runnable({**met, "d3_c0": "unmet", "code_equals_b4": "met"}, 5)
    d4 = {"d4_s0": "met", "d4_s1": "met", "step5_passed": "met", "code_equals_b4": "met"}
    assert runnable(d4, 7)
    b6 = {"d4_s0": "met", "d4_s1": "met", "step5_passed": "met", "d5_s0": "met", "d5_s1": "met"}
    assert not runnable(b6, 7)  # D6 in addition
    assert runnable({**b6, "d6_s0": "met", "d6_s1": "met"}, 7)
    assert runnable({"m_cam": "met"}, 1)
    with pytest.raises(ValueError, match="unknown result"):
        v04.allowed_steps(proc, {"nope": "met"})
    with pytest.raises(ValueError, match="use one of"):
        v04.allowed_steps(proc, {"d3_s0": "yes"})


def test_load_procedure_refuses_another_file(tmp_path):
    p = tmp_path / "x.yaml"
    p.write_text("name: x\nsteps: []\n")
    with pytest.raises(ValueError, match="not a v0.4"):
        v04.load_procedure(p)


def test_procedure_agrees_with_the_code():
    doc = yaml.safe_load(PROCEDURE.read_text())
    proto = doc["variant_protocol"]
    assert proto["min_objects"] == variant_form.MIN_OBJECTS
    assert proto["min_lane_change_m"] == variant_form.MIN_LANE_CHANGE_M
    assert proto["min_places_changing_lane"] == variant_form.MIN_PLACES_CHANGING_LANE
    assert proto["min_remaining_lane_m"] == variant_form.MIN_REMAINING_LANE_M
    assert f"{variant_form.LIDAR_HEIGHT_MARGIN_M:.2f} m" in proto["min_height_over_lidar_plane"]
    from racer_eval import gp4v

    assert proto["gp4v_label"] == gp4v.LABEL
    assert doc["fixed"]["battery_window_v"] == pytest.approx(0.8)
    assert v04.PREREG_SHA256 in json.dumps(doc["steps"][0]["pass_rule"])
    parser = cli.build_parser()
    choices = next(a for a in parser._actions if a.dest == "command").choices
    for step in doc["steps"]:
        for cmd in step.get("after", []):
            if "tools/pnp_session_report.py" in cmd:
                sub = cmd[cmd.index("tools/pnp_session_report.py") + 1]
                assert sub in choices, (step["id"], sub)
                parser.parse_args(
                    [
                        a.replace("<bag>", "b").replace("<out>", "o").replace("<caps>", "1.0")
                        for a in cmd[cmd.index(sub) :]
                    ]
                )


# ----------------------------------------------------------------------------- the report


def r2_report(**kw):
    bag = read_session_bag(CUT)
    return v04.v04_report(bag, caps=[1.0], reference="camera", **kw)


def _close(a, b, path=""):
    if isinstance(a, dict):
        assert a.keys() == b.keys(), path
        for k in a:
            _close(a[k], b[k], f"{path}/{k}")
    elif isinstance(a, list):
        assert len(a) == len(b), path
        for i, (x, y) in enumerate(zip(a, b, strict=True)):
            _close(x, y, f"{path}/{i}")
    elif isinstance(a, float) or isinstance(b, float):
        assert a == pytest.approx(b, rel=1e-9, abs=1e-12), path  # tolerance: 1e-9 relative
    else:
        assert a == b, path


def test_golden_v04_report_of_the_r2_cut():
    doc = r2_report()
    _close(json.loads(v04.to_json(doc)), json.loads(GOLDEN.read_text()))


def test_v04_report_content_on_the_r2_cut():
    doc = r2_report()
    assert doc["report"] == "v04" and doc["schema"] == "pnp-session/1"
    assert doc["preregistration"]["sha256"] == v04.PREREG_SHA256
    assert doc["d3_c0"]["verdict"] in ("NO REFERENCE", "INSUFFICIENT")
    assert doc["results_used"]["d3_c0"] == "unknown"  # not a PASS, so step 5 stays shut
    assert not next(s for s in doc["steps"] if s["step"] == 5)["runnable"]
    text = v04.markdown(doc)
    for needle in ("M-T timing", "## D3-C0", "lapref next to it", "Later steps", "minimum session"):
        assert needle in text
    with pytest.raises(ValueError, match="camera or video"):
        v04.v04_report(read_session_bag(CUT), caps=None, reference="gps")


def test_a_passing_d3c0_opens_step_five_only_with_b4():
    bag = make_bag()
    bag.cam_t = np.array([3.0, 6.0])
    bag.cam_lap = np.array([1, 2])
    bag.cam_lap_time_s = np.array([3.0, 3.0])
    bag.cam_distance_m = np.array([7.4, 7.4])
    bag.pnp_t = np.array([3.0, 6.0])
    bag.pnp_lap_index = np.array([1, 2])
    bag.pnp_lap_time_s = np.array([3.0, 3.0])
    # one run only: D3-C0 needs two, so it is INSUFFICIENT and not a pass
    doc = v04.v04_report(bag, caps=[0.5], reference="camera")
    assert doc["d3_c0"]["verdict"] == "INSUFFICIENT"
    assert doc["results_used"]["d3_c0"] == "unknown"
    # with a (synthetic) PASS verdict the status maps to met; B4 results come from the owner
    assert v04._result_status("PASS") == "met" and v04._result_status("FAIL") == "unmet"
    doc = v04.v04_report(
        bag,
        caps=[0.5],
        reference="camera",
        results={"d3_c0": "met", "d3_s0": "met", "d3_s1": "met"},
    )
    assert not next(s for s in doc["steps"] if s["step"] == 5)["runnable"]  # code or B6 missing
    doc = v04.v04_report(
        bag,
        caps=[0.5],
        reference="camera",
        results={"d3_c0": "met", "d3_s0": "met", "d3_s1": "met"},
        code_equals_b4=("met", "same tree"),
    )
    assert next(s for s in doc["steps"] if s["step"] == 5)["runnable"]


def test_video_reference_without_times_has_no_reference():
    bag = make_bag()
    doc = v04.v04_report(bag, caps=[0.5], reference="video", video_times={})
    assert doc["d3_c0"]["verdict"] == "NO REFERENCE" and doc["d3_c0"]["reference"] == "video"


# --------------------------------------------------------------------------------------- cli


def test_cli_v04_runs_prints_and_writes(tmp_path, monkeypatch, capsys):
    bag = make_bag()
    monkeypatch.setattr(cli, "_read", lambda p: bag)
    out = tmp_path / "v04.json"
    code = cli.main(
        [
            "v04",
            "--bag",
            "x",
            "--caps",
            "0.5",
            "--no-limiter",
            "--result",
            "d3_s0=met",
            "--out",
            str(out),
        ]
    )
    assert code == 2  # no reference: nothing decided
    printed = capsys.readouterr()
    assert "phone video" in printed.err and "Later steps" in printed.out
    assert json.loads(out.read_text())["results_used"]["d3_s0"] == "met"
    csv = tmp_path / "t.csv"
    csv.write_text("run,lap,lap_time_s\nR0,1,5.0\n")
    assert cli.main(["v04", "--bag", "x", "--no-limiter", "--times", str(csv)]) == 2
    assert (
        cli.main(["v04", "--bag", "x", "--no-limiter", "--owner-accepted-camera", "--m-cam-held"])
        == 2
    )
    assert cli.main(["v04", "--bag", "x", "--limiter-cli", str(tmp_path / "nope")]) == 2
    assert "NOT RUN" in capsys.readouterr().out
    with pytest.raises(SystemExit):
        cli.main(["v04", "--bag", "x", "--result", "d3_s0=maybe"])
    monkeypatch.setattr(v04, "REPO_ROOT", tmp_path)
    assert cli.main(["v04", "--bag", "x", "--no-limiter"]) == cli.EXIT_PREREG


def test_markdown_lists_lapref_rows_and_to_json_handles_numpy(tmp_path):
    (tmp_path / "R0.json").write_text(json.dumps({"lap_times_s": [5.5, 6.1, 5.7]}))
    doc = v04.v04_report(make_bag(), caps=[0.5], reference="camera")
    doc["lapref"] = v04.lapref_next_to([run_input()], tmp_path)
    text = v04.markdown(doc)
    assert "R0: lapref 3 laps, reference 3, count equal, 2 of 3 within 0.3 s, worst 0.400 s" in text
    out = json.loads(v04.to_json({"a": np.int64(3), "b": np.float64(0.5), "c": np.bool_(True)}))
    assert out == {"a": 3, "b": 0.5, "c": True}
    with pytest.raises(TypeError):
        v04.to_json({"a": object()})


def test_lidar_height_comes_from_the_vehicle_binding():
    expected = yaml.safe_load((REPO / "config/vehicle_params.yaml").read_text())["sensors"]["lidar"]
    assert v04.lidar_mount_z_m() == pytest.approx(expected["mount_z_m"])


GOOD_FORM = {
    "variant_id": "variant-2",
    "photo": "variant2.jpg",
    "seed": 77,
    "driven_before_first_protocol_run": False,
    "objects": [
        {"id": "o1", "height_m": 0.30, "size_m": [0.3, 0.2], "wall": "outer",
         "distance_along_outer_wall_m": 1.0, "protrusion_m": 0.2, "lane_width_m": 0.9,
         "against_wall": True, "heavy": True, "reflective_or_glass": False},
        {"id": "o2", "height_m": 0.30, "size_m": [0.3, 0.2], "wall": "inner",
         "distance_along_outer_wall_m": 3.0, "protrusion_m": 0.2, "lane_width_m": 0.9,
         "against_wall": True, "heavy": True, "reflective_or_glass": False},
    ],
}  # fmt: skip


def test_cli_variant_form_check_receipt_and_order(tmp_path, monkeypatch, capsys):
    monkeypatch.setattr(v04, "lidar_mount_z_m", lambda root=None: 0.20)
    form = tmp_path / "variant2_form.yaml"
    form.write_text(yaml.safe_dump(GOOD_FORM))
    assert cli.main(["variant-form", "--form", str(form)]) == 1  # the photo is not there
    assert "photo" in capsys.readouterr().err
    (tmp_path / "variant2.jpg").write_bytes(b"x")
    assert cli.main(["variant-form", "--form", str(form), "--conditions", "d_on,d_off"]) == 0
    printed = capsys.readouterr().out
    assert "run order (seed 77)" in printed and (tmp_path / "variant2_form.checked.json").is_file()
    assert variant_form.load_checked(form).form.variant_id == "variant-2"
    # a form that reuses an earlier variant's positions is refused with the reason
    earlier = tmp_path / "variant1_form.yaml"
    earlier.write_text(yaml.safe_dump({**GOOD_FORM, "variant_id": "variant-1"}))
    assert cli.main(["variant-form", "--form", str(form), "--earlier-forms", str(earlier)]) == 1
    assert "used before" in capsys.readouterr().err
    low = {**GOOD_FORM, "objects": [{**o, "height_m": 0.10} for o in GOOD_FORM["objects"]]}
    form.write_text(yaml.safe_dump(low))
    assert cli.main(["variant-form", "--form", str(form)]) == 1
    assert "under the LiDAR plane" in capsys.readouterr().err


def test_the_report_carries_the_gp3_record_and_the_timing_chain_line():
    doc = r2_report()
    assert doc["records"]["gp3"] == (
        "GP3 not met as registered; 20 clean laps observed, camera count 10 of 20"
    )
    assert "GP3-R1" in doc["records"]["gp3_repeat"] and "never GP3" in doc["records"]["gp3_repeat"]
    text = v04.markdown(doc)
    assert "GP3 not met as registered" in text and "Timing chain" in text
    assert "timing_chain" in text  # not on main: says so; on main: says how to run it


def test_report_uses_the_code_identity_result():
    doc = v04.v04_report(
        make_bag(), caps=[0.5], reference="camera", code_equals_b4=("met", "equal to abc")
    )
    assert doc["results_used"]["code_equals_b4"] == "met" and doc["code_identity"] == "equal to abc"


def _git(root, *args):
    import subprocess

    subprocess.run(
        ["git", "-C", str(root), "-c", "user.name=t", "-c", "user.email=t@t", *args],
        check=True,
        capture_output=True,
    )


def test_code_identity_compares_the_trees_of_the_planner_and_lap_directories(tmp_path):
    import subprocess

    assert v04.code_identity(None)[0] == "unknown"
    for d in v04.CODE_DIRS:
        (tmp_path / d).mkdir(parents=True)
        (tmp_path / d / "a.txt").write_text("one")
    (tmp_path / "other.txt").write_text("x")
    _git(tmp_path, "init", "-q")
    _git(tmp_path, "add", ".")
    _git(tmp_path, "commit", "-q", "-m", "b4")
    b4 = subprocess.run(
        ["git", "-C", str(tmp_path), "rev-parse", "HEAD"],
        capture_output=True,
        text=True,
        check=False,
    ).stdout.strip()
    assert v04.code_identity(b4, tmp_path)[0] == "met"
    (tmp_path / "other.txt").write_text("y")  # outside the two directories: still the same code
    _git(tmp_path, "commit", "-q", "-am", "elsewhere")
    assert v04.code_identity(b4, tmp_path)[0] == "met"
    (tmp_path / v04.CODE_DIRS[1] / "a.txt").write_text("hot copied")  # uncommitted
    status, why = v04.code_identity(b4, tmp_path)
    assert status == "unmet" and "uncommitted" in why
    _git(tmp_path, "commit", "-q", "-am", "planner changed")
    status, why = v04.code_identity(b4, tmp_path)
    assert status == "unmet" and "differs" in why
    assert v04.code_identity("nosuchsha", tmp_path)[0] == "unknown"
    assert v04.code_identity(b4, tmp_path / "not-a-repo")[0] == "unknown"


def test_requirement_blockers_all_shapes():
    status = {"a": "met", "b": "unknown", "c": "met", "d": "unmet"}
    assert v04.requirement_blockers({"result": "a"}, status) == (["a"], [])
    assert v04.requirement_blockers({"result": "b"}, status) == (["b"], ["b is unknown"])
    ids, why = v04.requirement_blockers({"any_of": [["b"], ["a", "c"]]}, status)
    assert ids == ["b", "a", "c"] and why == []
    ids, why = v04.requirement_blockers({"any_of": [["b"], ["a", "d"]]}, status)
    assert why == ["either (b is unknown) or (d is unmet)"]


def test_cli_v04_with_a_b4_commit_that_cannot_be_resolved(tmp_path, monkeypatch, capsys):
    monkeypatch.setattr(cli, "_read", lambda p: make_bag())
    assert cli.main(["v04", "--bag", "x", "--no-limiter", "--b4-commit", "nosuchsha"]) == 2
    assert "Later steps" in capsys.readouterr().out


def test_lapref_json_and_the_procedure_commands_match_what_shipped(tmp_path):
    """Brief 36 (PR 194): lapref writes `lap_times_s`; pnp_data takes `confirm --bag`."""
    from lapref.method import LaprefResult
    from pnp_data import cli as pnp_data_cli

    out = tmp_path / "R0.json"
    out.write_text(json.dumps(LaprefResult(n_laps=2, lap_times_s=[5.5, 5.6]).to_dict()))
    assert v04.read_lapref(out) == (5.5, 5.6)
    doc = yaml.safe_load(PROCEDURE.read_text())
    parser = pnp_data_cli.build_parser()
    seen = 0
    for step in doc["steps"]:
        for cmd in step.get("after", []):
            if "tools/pnp_data.py" in cmd:
                args = cmd[cmd.index("tools/pnp_data.py") + 1 :]
                args = [
                    a.replace("<bag>", "b").replace("<session_date>", "2026-11-20") for a in args
                ]
                args = [a.replace("<dyn>", "d") if isinstance(a, str) else a for a in args]
                parser.parse_args(args)
                seen += 1
            if "tools/lapref.py" in cmd:
                flags = cmd[cmd.index("tools/lapref.py") + 1 :]
                assert "--run" in flags and "--out" in flags
                seen += 1
    assert seen == 4  # confirm, runs, split, lapref
