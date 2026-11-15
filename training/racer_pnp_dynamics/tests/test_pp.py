"""L1 for Pp (pose_check.py, addendum 1 section 2.5): position against the map pose."""

from __future__ import annotations

import numpy as np
import pytest
from racer_pnp_dynamics import ground_motion as gmo
from racer_pnp_dynamics import pose_check as pc
from test_pose_check import DELTA, V, _model, _model_rate, _run, _segment
from test_step_v2 import C


def _arc_recording(vehicle, stamps, scale=1.0):
    """Poses on the arc the model drives (rate and speed constant), the path scaled by `scale`."""
    rate = _model_rate(vehicle)
    t = stamps - stamps[0]
    radius = V * scale / rate
    return {
        "t0_log_s": np.array(0.0),
        "pose_stamp": stamps,
        "pose_x": radius * np.sin(rate * t),
        "pose_y": radius * (1.0 - np.cos(rate * t)),
        "pose_yaw": rate * t,
    }


def test_poses_on_the_models_own_arc_give_a_tiny_pp(toy_vehicle, toy_nominal):
    run = _run(30.0)
    stamps = np.arange(104.0, 125.0, 0.25)
    rec = _arc_recording(toy_vehicle, stamps)
    seg = _segment(run, "G1b")
    pairs, counts = pc.select_pose_pairs(rec, seg)
    assert counts["kept"] == len(pairs) >= 15 and counts["jumped"] == 0
    assert (pairs.dt >= pc.PP_HORIZON_S).all() and pairs.dt.max() < pc.PP_HORIZON_S + 0.3
    split = pc.build_pose_pair_segments([seg], {"r": rec})
    assert pc.pp_macro(_model(toy_vehicle, toy_nominal), split) < 0.03


def test_a_car_that_covers_less_ground_is_seen_by_pp_and_repaired_by_a_ground_table(
    toy_vehicle, toy_nominal
):
    run = _run(30.0)
    stamps = np.arange(104.0, 125.0, 0.25)
    rec = _arc_recording(toy_vehicle, stamps, scale=0.8)  # the map pose says 0.8 of the wheel path
    seg = _segment(run, "G1b")
    split = pc.build_pose_pair_segments([seg], {"r": rec})
    base = _model(toy_vehicle, toy_nominal)
    one_speed = pc.pp_macro(base, split)
    table = gmo.GroundMotion(C, gmo.GroundTable.constant(C, 0.8))
    contract2 = pc.pp_macro(pc.YawModel(base.nominal, None, toy_vehicle, table), split)
    assert one_speed > 0.15  # about 0.2 of a metre of path per pair
    assert contract2 < 0.25 * one_speed
    # contract 2 changes position only: Yp of the two models is the same
    yp1 = pc.yp_macro(base, pc.build_pose_segments([seg], {"r": rec}))
    yp2 = pc.yp_macro(
        pc.YawModel(base.nominal, None, toy_vehicle, table),
        pc.build_pose_segments([seg], {"r": rec}),
    )
    assert yp1 == pytest.approx(yp2, abs=1e-12)


def test_a_relocalisation_jump_and_an_empty_segment_are_not_used(toy_vehicle, toy_nominal):
    run = _run(30.0)
    stamps = np.arange(104.0, 125.0, 0.25)
    rec = _arc_recording(toy_vehicle, stamps)
    rec["pose_x"] = rec["pose_x"].copy()
    rec["pose_x"][30:] += 5.0
    seg = _segment(run, "G1b")
    pairs, counts = pc.select_pose_pairs(rec, seg)
    assert counts["jumped"] == 1 and counts["kept"] == len(pairs)
    empty = _arc_recording(toy_vehicle, np.array([104.0]))
    p0, c0 = pc.select_pose_pairs(empty, seg)
    assert len(p0) == 0 and c0["kept"] == 0
    split = pc.build_pose_pair_segments([seg], {"r": empty})
    assert np.isnan(pc.pp_macro(_model(toy_vehicle, toy_nominal), split))
    per = pc.pp_by_segment(
        _model(toy_vehicle, toy_nominal), pc.build_pose_pair_segments([seg], {"r": rec})
    )
    assert per["G1b"]["pairs"] == counts["kept"] and per["T2b"]["pairs"] == 0
    assert DELTA > 0
