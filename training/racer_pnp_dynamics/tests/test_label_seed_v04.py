"""L1 for the label seed variants and M-B4 (addendum 1 section 2.4)."""

from __future__ import annotations

from dataclasses import replace

import numpy as np
import pytest
from racer_pnp_dynamics import icp_motion as im
from racer_pnp_dynamics import label_sensitivity as ls
from racer_pnp_dynamics import truth_v04 as t4
from racer_pnp_dynamics import wheel_seed
from racer_pnp_dynamics.dataset import by_split, make_segments
from racer_pnp_dynamics.icp_motion import MotionSeries
from synth import make_run, make_split
from test_fit import TRUTH
from test_truth_t2 import _round_room, _run, _scan, _turning_scene

SEGS = [("T1", 5.0, 200.0, "train", "ccw"), ("T2", 205.0, 400.0, "train", "cw")]
QUIET = {"yaw_noise": 0.0005, "speed_noise": 0.0005}


def test_seed_scale_one_is_bit_identical_and_scale_acts_on_the_path_only(toy_vehicle):
    run = _run(1.0, 0.2)
    a = wheel_seed.interval_seed(run, toy_vehicle, [100.5, 101.0], [100.6, 101.1])
    b = wheel_seed.interval_seed(run, toy_vehicle, [100.5, 101.0], [100.6, 101.1], 1.0)
    np.testing.assert_array_equal(a[0], b[0])
    np.testing.assert_array_equal(a[1], b[1])
    c = wheel_seed.interval_seed(run, toy_vehicle, [100.5, 101.0], [100.6, 101.1], 0.8)
    np.testing.assert_array_equal(c[0], a[0])
    np.testing.assert_allclose(c[1], 0.8 * a[1])


def test_defaults_of_compute_motion_are_bit_identical_to_explicit_defaults(toy_vehicle):
    scans, run = _turning_scene(toy_vehicle)
    kw = {"ignore": frozenset(), "seed": im.SEED_WHEEL, "run": run}
    a = im.compute_motion(scans, toy_vehicle, **kw)
    b = im.compute_motion(scans, toy_vehicle, seed_scale=1.0, clock_shift_s=0.0, **kw)
    for name in ("dx_m", "dy_m", "dyaw_rad", "rms_m"):
        np.testing.assert_array_equal(getattr(a, name), getattr(b, name))


def test_a_scene_with_slip_gives_a_ratio_near_0_8_under_both_seeds(toy_vehicle):
    """The car covers 0.08 m while the wheels report 0.10 m (slip 0.8) and turns 0.2 rad. The
    matcher answers the scans, so the ground over wheel ratio is 0.8 whichever way it is started
    (the seed is only a start value)."""
    mount = im.mount_of(toy_vehicle)
    room = _round_room()
    half = 0.1
    path = 0.08
    poses = [(0.0, 0.0, 0.0), (path * np.cos(half), path * np.sin(half), 0.2)]
    scans = [_scan(room, p, mount, 100.0 + 0.1 * i) for i, p in enumerate(poses)]
    _, wheel_run = _turning_scene(toy_vehicle, turn=0.2, path=0.1)  # the wheels say 0.10 m
    wheel_v = 1.0
    for scale in (1.0, 0.8):
        m = im.compute_motion(
            scans, toy_vehicle, ignore=frozenset(), seed=im.SEED_WHEEL, run=wheel_run,
            seed_scale=scale,
        )  # fmt: skip
        assert m.ok[0]
        assert m.ground_speed_mps()[0] / wheel_v == pytest.approx(0.8, abs=0.06), scale


def test_the_variant_names_the_cache_file_and_unknown_variants_are_refused(tmp_path):
    base = t4.train_path(tmp_path, "bag", "X1")
    assert base.name == "bag.X1.motion-wheel.npz"  # T2 keeps its name
    names = {t4.train_path(tmp_path, "bag", "X1", v).name for v in t4.VARIANTS}
    assert len(names) == len(t4.VARIANTS)
    assert t4.heldout_path(tmp_path, "bag", "X2b", t4.T2S).parent != base.parent
    with pytest.raises(ValueError, match="unknown truth variant"):
        t4.train_path(tmp_path, "bag", "X1", "s0.5")
    assert t4.VARIANTS[t4.T2S] == (0.80, 0.0)
    assert t4.VARIANTS[t4.CLOCK_PLUS][1] == pytest.approx(0.010)
    assert t4.VARIANTS[t4.CLOCK_MINUS][1] == pytest.approx(-0.010)
    # stage 1 still cannot read a held-out variant
    with pytest.raises(t4.HeldoutLocked):
        t4.load_stage1_motion(tmp_path, "bag", "X2b", "heldout", t4.T2S)


# ---- the M-B4 table --------------------------------------------------------------------------


def _segments(vehicle, seed=3):
    run, motion = make_run(
        TRUTH, vehicle, seconds=420.0, seed=seed, name="bag", v_hi=1.0, exact_model_dt=True,
        wheel_over_read=0.8, **QUIET,
    )  # fmt: skip
    segs = make_segments(run, motion, make_split("bag", SEGS), t_ref_s=run.t_s[0])
    return by_split(segs, "train", odom_only=True)


def _scaled(segments, factor):
    out = []
    for s in segments:
        m = s.motion
        out.append(
            replace(
                s,
                motion=MotionSeries(
                    **{**m.__dict__, "dx_m": m.dx_m * factor, "dy_m": m.dy_m * factor}
                ),
            )
        )
    return out


def test_ground_over_wheel_follows_the_plant_slip(toy_vehicle_module):
    segs = _segments(toy_vehicle_module)
    rows = ls.sensitivity_table(segs, {"s0.80": segs})
    assert rows and all(0.7 < r["median_T2"] < 1.02 for r in rows)
    # a plant with slip has ground below wheel in the strata that turn hard
    assert min(r["median_T2"] for r in rows) < 0.95
    assert not any(r["label_depends_on_the_seed"] for r in rows)  # the same truth twice


def test_a_variant_whose_ground_differs_is_reported_as_depending_on_the_seed(toy_vehicle_module):
    segs = _segments(toy_vehicle_module)
    rows = ls.sensitivity_table(segs, {"s0.80": _scaled(segs, 0.9)})
    flagged = [r for r in rows if r["label_depends_on_the_seed"]]
    assert flagged and all(r["windows"] >= ls.MIN_WINDOWS for r in flagged)
    for r in flagged:
        assert r["median_T2"] - r["median_s0.80"] > ls.DEPENDENT_IF_RATIO_DIFFERS_BY
    text = ls.render_mb4(rows, ls.rejection_table(segs), ["s0.80"], ["note"])
    assert ls.DEPENDENT_TEXT in text and "- note" in text and "| T2 |" in text


def test_a_small_difference_or_few_windows_is_not_flagged(toy_vehicle_module):
    segs = _segments(toy_vehicle_module)
    rows = ls.sensitivity_table(segs, {"s0.80": _scaled(segs, 0.99)})
    assert not any(r["label_depends_on_the_seed"] for r in rows)


def test_the_clock_shift_moves_the_wheel_distance_of_the_window(toy_vehicle_module):
    segs = _segments(toy_vehicle_module)
    es = ls.eval_set(segs)
    a = ls.ground_over_wheel(es)
    b = ls.ground_over_wheel(es, 0.010)
    assert np.isfinite(a).all() and not np.array_equal(a, b)
    assert np.nanmedian(np.abs(a - b)) < 0.05  # 10 ms is small


def test_rejection_counts_windows_the_quality_flags_drop(toy_vehicle_module):
    segs = _segments(toy_vehicle_module)
    clean = ls.rejection_table(segs)
    assert all(r["rejected_share"] == 0.0 for r in clean)
    spoiled = []
    for s in segs:
        m = s.motion
        ok = m.ok.copy()
        ok[::40] = False
        spoiled.append(replace(s, motion=MotionSeries(**{**m.__dict__, "ok": ok})))
    rows = ls.rejection_table(spoiled)
    assert rows and all(0.0 < r["rejected_share"] < 1.0 for r in rows)
    assert all(r["kept"] + 0 <= r["candidates"] for r in rows)


def test_no_windows_is_refused(toy_vehicle_module):
    with pytest.raises(ValueError, match="no T2 windows"):
        ls.sensitivity_table([], {})
