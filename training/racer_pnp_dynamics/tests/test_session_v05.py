"""L1 for the 2026-11-15 session as a data source (truth_v05.py, session_v05.py, scoring_v05.py):
the three truth caches and their guards, the split file as a Split, the committed-file guard,
the heading direction, and the M-B6 scoring and text."""

from __future__ import annotations

import inspect
import subprocess
from dataclasses import replace

import numpy as np
import pytest
from racer_pnp_dynamics import ground_motion as gmo
from racer_pnp_dynamics import ground_motion_v21 as g21
from racer_pnp_dynamics import report_v05 as rep
from racer_pnp_dynamics import scoring_v05 as sc
from racer_pnp_dynamics import session_v05 as sv
from racer_pnp_dynamics import stages_v05, truth_v04, truth_v05
from racer_pnp_dynamics.dataset import by_split, make_segments
from racer_pnp_dynamics.evaluate import eval_set, window_errors
from racer_pnp_dynamics.icp_motion import MotionSeries
from racer_pnp_dynamics.split import SplitInvalid
from racer_pnp_dynamics.vehicle import repo_root
from synth import make_run, make_split
from test_fit import TRUTH
from test_step_v21 import C5
from test_step_v21 import motion as motion21

GOLDEN = repo_root() / "tools" / "tests" / "fixtures" / "pnp_data_session_v05_golden.yaml"
SHA = "a" * 40


def _slice(first: int = 4, n: int = 40) -> truth_v05.Slice:
    t = np.arange(n) * 0.1
    rng = np.random.default_rng(0)
    return truth_v05.Slice(
        first,
        MotionSeries(
            t_s=t, dt_s=np.full(n, 0.1), dx_m=rng.normal(size=n), dy_m=rng.normal(size=n),
            dyaw_rad=rng.normal(size=n), rms_m=np.zeros(n), inlier_share=np.ones(n),
            ok=np.ones(n, bool), seeded=np.ones(n, bool),
        ),
    )  # fmt: skip


# ---- truth caches ----------------------------------------------------------------------------


def test_the_three_caches_are_siblings_and_the_variant_is_in_the_name(tmp_path):
    a = truth_v05.train_path(tmp_path, "bag", "S1a")
    b = truth_v05.heldout_path(tmp_path, "bag", "S1b")
    c = truth_v05.whole_path(tmp_path, "bag", "R0")
    assert {p.parent.name for p in (a, b, c)} == {
        truth_v05.TRAIN_DIR,
        truth_v05.HELDOUT_DIR,
        truth_v05.WHOLE_DIR,
    }
    assert len({p.parent for p in (a, b, c)}) == 3 and a.parent.parent == b.parent.parent
    assert a.name == "bag.S1a.motion-wheel.npz"
    assert truth_v05.train_path(tmp_path, "bag", "S1a", truth_v04.T2S).name.endswith("-s0.80.npz")
    with pytest.raises(ValueError, match="unknown truth variant"):
        truth_v05.train_path(tmp_path, "bag", "S1a", "nope")


def test_roundtrip_and_the_guards_of_each_cache(tmp_path):
    sl = _slice()
    for path in (
        truth_v05.train_path(tmp_path, "bag", "S1a"),
        truth_v05.heldout_path(tmp_path, "bag", "S1b"),
        truth_v05.whole_path(tmp_path, "bag", "R0"),
    ):
        truth_v04.save_slice(path, sl)
    got = truth_v05.load_train_motion(tmp_path, "bag", "S1a", "train")
    assert got.first == 4 and np.array_equal(got.motion.dx_m, sl.motion.dx_m)
    with pytest.raises(truth_v04.HeldoutLocked, match="train parts"):
        truth_v05.load_train_motion(tmp_path, "bag", "S1b", "heldout")
    token = truth_v04.freeze(SHA)
    assert truth_v05.load_heldout_motion(tmp_path, "bag", "S1b", "heldout", token).first == 4
    with pytest.raises(truth_v04.HeldoutLocked, match="FreezeToken"):
        truth_v05.load_heldout_motion(tmp_path, "bag", "S1b", "heldout", SHA)  # a bare string
    with pytest.raises(truth_v04.HeldoutLocked, match="not heldout"):
        truth_v05.load_heldout_motion(tmp_path, "bag", "S1a", "train", token)
    with pytest.raises(truth_v04.HeldoutLocked, match="not a git SHA"):
        truth_v04.freeze("HEAD")
    for purpose in (truth_v05.CARRY_OVER_PURPOSE, truth_v05.SLICE_PURPOSE):
        assert truth_v05.load_whole_motion(tmp_path, "bag", "R0", purpose).first == 4
    with pytest.raises(truth_v04.HeldoutLocked, match="read by"):
        truth_v05.load_whole_motion(tmp_path, "bag", "R0", "fit")


def test_a_part_cut_from_a_whole_run_equals_the_part_computed_alone():
    whole = _slice(first=10, n=60)
    part = truth_v05.cut_slice(whole, 20, 35)
    assert part.first == 20 and len(part.motion.t_s) == 15
    for k in ("dx_m", "dy_m", "dyaw_rad", "t_s", "ok", "seeded"):
        assert np.array_equal(getattr(part.motion, k), getattr(whole.motion, k)[10:25])
    with pytest.raises(ValueError, match="outside"):
        truth_v05.cut_slice(whole, 5, 20)
    with pytest.raises(ValueError, match="outside"):
        truth_v05.cut_slice(whole, 20, 80)
    with pytest.raises(ValueError, match="outside"):
        truth_v05.cut_slice(whole, 30, 20)


def test_part_intervals_need_both_scans_inside():
    stamps = np.arange(0.0, 10.0, 0.1)
    assert truth_v05.part_intervals(stamps, 1.0, 2.0) == (10, 20)
    assert truth_v05.part_intervals(stamps[:1], 0.0, 1.0) == (0, 0)


def test_only_the_named_stages_can_reach_a_cache_they_must_not_open():
    """No fit stage opens the held-out cache, the whole-run cache or the selection set; the
    functions that do are a known short list, each of them not a fit."""
    forbidden = (
        "load_heldout_motion",
        "heldout_path",
        "load_whole_motion",
        "whole_path",
        "selection_eval_set",
        "V04_FREEZE_SHA",
    )
    users = {
        name
        for name, fn in inspect.getmembers(stages_v05, inspect.isfunction)
        if fn.__module__ == stages_v05.__name__
        and any(sym in inspect.getsource(fn) for sym in forbidden)
    }
    # `mb5` and `fit-selection` read the selection set to choose a weight; `mb6` and the truth
    # builders read the carry-over; the frozen pass (added with the held-out stage) reads the rest.
    allowed = {
        "selection_eval_set",  # defines the selection reader
        "mb5",  # selects a weight on it, fits nothing on it
        "mb6",  # carry-over
        "whole_run_segments",  # carry-over
        "session_segments",  # the one builder, guarded by role and token
        "t5_fit",  # selects the smoothness weight on the selection set, never fits on it
        "tl5_fit",  # the same for the choice between configurations and seeds
        "freeze_eval",  # the one held-out pass
        "selection_summary",
        "fit_context",
    }
    assert users <= allowed, users - allowed
    assert "session_segments" in users and "selection_eval_set" in users


# ---- the split file as a Split ----------------------------------------------------------------


def test_the_golden_session_file_parses_as_a_split():
    split = sv.load_session_split(GOLDEN)
    assert set(split.bags) == {"2026-11-16T22-45-39_car_teleop", "2026-11-16T23-04-09_car_teleop"}
    segs = {s.name: s for e in split.bags.values() for s in e.segments}
    assert [segs[n].role for n in ("S1", "S2a", "S2b", "S4", "S5a", "S5b")] == [
        "unused", "train", "heldout", "unused", "train", "heldout",
    ]  # fmt: skip
    assert segs["S2a"].direction == "by_heading"
    assert split.entry("2026-11-16T22-45-39_car_teleop").first_scan_log_time_s == pytest.approx(
        1794869142.979
    )
    assert split.role_of("2026-11-16T22-45-39_car_teleop", 270.0, 280.0) == "train"
    assert split.role_of("2026-11-16T22-45-39_car_teleop", 319.0, 321.0) is None  # the guard


def test_a_file_that_names_another_rule_or_overlaps_is_refused(tmp_path):
    text = GOLDEN.read_text()
    bad = tmp_path / "s.yaml"
    bad.write_text(text.replace(g21.V05_SHA256, "b" * 64))
    with pytest.raises(SplitInvalid, match="registered"):
        sv.load_session_split(bad)
    overlap = tmp_path / "o.yaml"
    overlap.write_text(text.replace("from: 320.1", "from: 300"))
    with pytest.raises(SplitInvalid, match="overlap"):
        sv.load_session_split(overlap)


def test_fits_refuse_the_session_without_a_committed_split_file(tmp_path):
    with pytest.raises(sv.SessionUnsplit, match="not there"):
        sv.require_committed_session(tmp_path)
    target = tmp_path / sv.SESSION_RELATIVE
    target.parent.mkdir(parents=True)
    target.write_text(GOLDEN.read_text())
    assert sv.require_committed_session(tmp_path, require_committed=False) == target
    with pytest.raises(sv.SessionUnsplit, match="not committed"):
        sv.require_committed_session(tmp_path)  # not a git repository at all
    git = ["git", "-C", str(tmp_path)]
    subprocess.run([*git, "init", "-q"], check=True)
    subprocess.run([*git, "add", str(sv.SESSION_RELATIVE)], check=True)
    with pytest.raises(sv.SessionUnsplit, match="not committed"):  # staged, not committed
        sv.require_committed_session(tmp_path)
    subprocess.run(
        [*git, "-c", "user.name=t", "-c", "user.email=t@example.com", "commit", "-q", "-m", "s"],
        check=True,
    )
    assert sv.require_committed_session(tmp_path) == target


def test_the_session_bags_are_the_five_of_the_identity_note():
    note = (repo_root() / "docs" / "notes" / "dynamics-v05-session-identity.md").read_text()
    assert len(sv.SESSION_BAGS) == 5
    assert all(b in note for b in sv.SESSION_BAGS)
    assert sv.SESSION_RELATIVE.name == "2026-11-15-dynamics-v05.yaml"


# ---- heading direction -------------------------------------------------------------------------


def _segments(vehicle, steer: float, seed: int, flip_at: float | None = None):
    name = f"bag{seed}"
    run, motion = make_run(
        TRUTH, vehicle, seconds=120.0, seed=seed, name=name, v_hi=0.9, exact_model_dt=True,
        steer_const=steer,
    )  # fmt: skip
    if flip_at is not None:  # the car is turned round: the heading changes sign
        flip = motion.t_s > run.t_s[0] + flip_at
        motion = replace(motion, dyaw_rad=np.where(flip, -motion.dyaw_rad, motion.dyaw_rad))
    split = make_split(name, [("S", 5.0, 110.0, "train", "by_heading")])
    return make_segments(run, motion, split, t_ref_s=run.t_s[0]), run


def test_direction_is_the_measured_heading_not_the_label(toy_vehicle):
    left, _ = _segments(toy_vehicle, 0.3, 1)
    right, _ = _segments(toy_vehicle, -0.3, 2)
    segs = [s for s in left + right if s.split in ("train", "val")]
    es = eval_set([s for s in segs if s.run.has_odom])
    assert set(es.direction.tolist()) == {"by_heading"}
    labels = sv.heading_direction(es, segs)
    by_run = {r.name: labels[es.run_idx == i] for i, r in enumerate(es.runs)}
    assert set(by_run["bag1"].tolist()) == {"ccw"} and set(by_run["bag2"].tolist()) == {"cw"}
    out = sv.with_heading_direction(es, segs)
    assert np.array_equal(out.direction, labels) and out.t_s is es.t_s


def test_a_run_that_turns_round_changes_direction_inside_the_run(toy_vehicle):
    segs, run = _segments(toy_vehicle, 0.3, 3, flip_at=55.0)
    segs = [s for s in segs if s.split in ("train", "val")]
    es = eval_set([s for s in segs if s.run.has_odom])
    labels = sv.heading_direction(es, segs)
    t = es.t_s - run.t_s[0]
    assert set(labels[t < 45.0].tolist()) == {"ccw"}
    assert set(labels[t > 70.0].tolist()) == {"cw"}  # F ccw 1.5: the tail was driven clockwise


def test_a_block_with_no_good_interval_falls_back_to_the_window_yaw(toy_vehicle):
    segs, _ = _segments(toy_vehicle, 0.3, 4)
    segs = [s for s in segs if s.split in ("train", "val")]
    es = eval_set([s for s in segs if s.run.has_odom])
    dead = [replace(s, motion=replace(s.motion, ok=np.zeros_like(s.motion.ok))) for s in segs]
    labels = sv.heading_direction(es, dead)
    expect = np.where(es.poses[:, -1, 2] > 0.0, "ccw", "cw")
    assert np.array_equal(labels, expect)
    assert sv.heading_direction(es, [replace(s, motion=None) for s in segs]).shape == (len(es),)


# ---- scoring and M-B6 -----------------------------------------------------------------------


@pytest.fixture(scope="module")
def windows(toy_vehicle_module):
    run, mot = make_run(
        TRUTH, toy_vehicle_module, seconds=300.0, seed=8, name="b", v_hi=1.0, exact_model_dt=True
    )
    split = make_split("b", [("A", 5.0, 140.0, "train", "ccw"), ("B", 150.0, 290.0, "train", "cw")])
    segs = make_segments(run, mot, split, t_ref_s=run.t_s[0])
    es = eval_set(by_split(segs, "train", odom_only=True))
    assert len(es) > 80
    return es


def test_spec_dispatch_matches_the_underlying_functions(windows, toy_vehicle_module):
    v = toy_vehicle_module
    plain = sc.Spec(TRUTH).errors(windows, v)
    old = window_errors(windows, TRUTH, v, None, (1.0,))
    np.testing.assert_allclose(plain[1.0]["pos"], old[1.0]["pos"], atol=1e-12)
    table = gmo.GroundTable.constant(C5, 0.9)
    v2 = sc.Spec(TRUTH, ground=gmo.GroundMotion(C5, table)).errors(windows, v)
    np.testing.assert_allclose(
        v2[1.0]["pos"],
        gmo.window_errors_v2(windows, TRUTH, v, gmo.GroundMotion(C5, table))[1.0]["pos"],
    )
    gm21 = motion21(5)
    v21 = sc.Spec(TRUTH, ground21=gm21).errors(windows, v)
    np.testing.assert_allclose(
        v21[1.0]["yaw"], g21.window_errors_v21(windows, TRUTH, v, gm21)[1.0]["yaw"]
    )
    assert isinstance(sc.Spec(TRUTH, ground21=gm21).pose_model(v).ground, g21.GroundMotionV21)
    assert sc.Spec(TRUTH, ground=gmo.GroundMotion(C5, table)).pose_model(v).ground is not None
    assert sc.Spec(TRUTH).pose_model(v).ground is None


def test_summary_and_the_m_b6_text(windows, toy_vehicle_module):
    v = toy_vehicle_module
    models = {
        "dyn-v0.1": sc.Spec(TRUTH),
        "T5": sc.Spec(TRUTH, ground21=motion21(5)),
    }
    strata, errs = sc.score_models(windows, models, v)
    summary = sc.summarise(windows, strata, errs)
    assert summary["windows"] == len(windows)
    assert summary["ratio"]["dyn-v0.1"]["P1"] == pytest.approx(1.0)
    assert set(summary["per_direction"]["T5"]) == {"ccw", "cw"}
    assert sum(r["windows"] for r in summary["per_run"].values()) == len(windows)
    assert summary["macro"]["T5"]["P1"] == pytest.approx(sc.macro_at(errs["T5"][1.0], strata)["P1"])
    text = rep.render_mb6(summary, ["a note"], {"n4": "0" * 64})
    assert text.startswith("# M-B6") and "- a note" in text and "n4 " + "0" * 16 in text
    assert "| T5 |" in text and "## Strata (windows)" in text and "ccw" in text
    assert "deciding nothing" in text


def test_direction_macro_of_a_direction_with_no_windows_is_nan(windows):
    strata, errs = sc.score_models(windows, {"m": sc.Spec(TRUTH)}, _vehicle())
    out = sc.direction_macro(errs["m"][1.0], strata, "sideways")
    assert np.isnan(out["P1"]) and np.isnan(out["Y1"])
    assert np.isnan(sc.rmse(np.zeros(0)))


def _vehicle():
    from racer_pnp_dynamics.vehicle import Vehicle

    return Vehicle(
        0.5, -0.4, 0.4, 2.0, 2.0, "9.9.9", "toy", 0.5, -0.4, 0.6, 0.1, 0.15, 0.3, 0.0, 3.141593
    )


def test_pose_model_steps_a_contract_2_1_model(toy_vehicle):
    """The pose check (Pp, Yp) runs the contract 2.1 step when the model has the yaw table."""
    from racer_pnp_dynamics import pose_check

    gm = motion21(6)
    assert isinstance(gm, g21.GroundMotionV21)
    model = pose_check.YawModel(TRUTH, None, toy_vehicle, gm)
    run, _ = make_run(TRUTH, toy_vehicle, seconds=30.0, seed=2, name="p", v_hi=1.0)
    t0 = run.t_s[100:110]
    states, ok, _steps, _n = pose_check._model_states(model, run, t0, np.full(10, 0.5))
    zero = pose_check.YawModel(
        TRUTH, None, toy_vehicle, g21.GroundMotionV21(C5, gm.table, g21.zero_yaw_table(gm.table))
    )
    other, *_ = pose_check._model_states(zero, run, t0, np.full(10, 0.5))
    assert ok.all() and states.shape == other.shape
    assert not np.allclose(states[:, :, 2], other[:, :, 2])  # gamma moved the yaw
    np.testing.assert_allclose(states[:, :, 3], other[:, :, 3])  # the wheel channel did not move
