"""L1 for pose_check.py (Yp): synthetic poses with a known yaw bias, the jump filter, a segment
with no usable interval."""

from __future__ import annotations

import numpy as np
import pytest
from racer_pnp_dynamics import pose_check as pc
from racer_pnp_dynamics.bags import GRID_DT_S, Run
from racer_pnp_dynamics.dataset import Segment

V = 1.0
DELTA = 0.05


def _run(seconds: float = 20.0) -> Run:
    t = 100.0 + np.arange(int(seconds / GRID_DT_S)) * GRID_DT_S
    return Run("r", t, np.full_like(t, V), np.full_like(t, DELTA), np.full_like(t, V), [])


def _segment(run: Run, name: str) -> Segment:
    return Segment(run, None, 0, len(run.t_s), "heldout", name, "ccw")


def _model(toy_vehicle, toy_nominal) -> pc.YawModel:
    nominal = toy_nominal.replace(speed_tau_s=0.05, steer_tau_s=0.05, command_delay_s=0.0)
    return pc.YawModel(nominal, None, toy_vehicle)


def _recording(rate: float, stamps: np.ndarray, jump_at: int | None = None) -> dict:
    yaw = rate * (stamps - stamps[0])
    x = V * (stamps - stamps[0])
    if jump_at is not None:
        x[jump_at:] += 5.0
    return {
        "t0_log_s": np.array(0.0),
        "pose_stamp": stamps,
        "pose_x": x,
        "pose_y": np.zeros_like(x),
        "pose_yaw": yaw,
    }


def _model_rate(toy_vehicle) -> float:
    return V * np.tan(DELTA) / toy_vehicle.wheelbase_m


def test_a_pose_that_agrees_with_the_model_gives_zero_yp(toy_vehicle, toy_nominal):
    run = _run()
    stamps = np.arange(104.0, 112.0, 0.5)
    rec = _recording(_model_rate(toy_vehicle), stamps)
    seg = _segment(run, "G1b")
    iv = pc.pose_yaw_intervals(rec, seg)
    assert len(iv) == len(stamps) - 1
    assert pc.yp(_model(toy_vehicle, toy_nominal), rec, seg, iv) == pytest.approx(0.0, abs=2e-3)


def test_a_known_yaw_bias_is_the_yp(toy_vehicle, toy_nominal):
    run = _run()
    stamps = np.arange(104.0, 112.0, 0.5)
    rate = _model_rate(toy_vehicle)
    rec = _recording(1.2 * rate, stamps)  # the pose turns 20 percent faster than the model
    seg = _segment(run, "G1b")
    iv = pc.pose_yaw_intervals(rec, seg)
    expected = 0.2 * rate * 0.5
    assert pc.yp(_model(toy_vehicle, toy_nominal), rec, seg, iv) == pytest.approx(
        expected, rel=0.03
    )


def test_the_jump_filter_and_the_interval_limit(toy_vehicle):
    run = _run()
    stamps = np.array([104.0, 104.5, 105.0, 105.5, 108.0, 108.5])  # a 2.5 s gap before 108.0
    rec = _recording(_model_rate(toy_vehicle), stamps, jump_at=3)  # a 5 m relocalisation at 105.5
    seg = _segment(run, "G1b")
    iv, counts = pc.select_pose_intervals(rec, seg)
    assert counts["pairs"] == 5
    assert counts["too_long"] == 1
    assert counts["jumped"] == 1  # the pair 105.0 to 105.5
    assert counts["kept"] == len(iv) == 3


def test_intervals_outside_the_segment_are_not_used(toy_vehicle):
    run = _run()
    stamps = np.arange(90.0, 130.0, 0.5)  # the run spans 100 to 120 s
    rec = _recording(_model_rate(toy_vehicle), stamps)
    seg = _segment(run, "G1b")
    iv = pc.pose_yaw_intervals(rec, seg)
    assert iv.t0.min() >= seg.t0_s and iv.t1.max() <= seg.t1_s


def test_a_segment_with_no_usable_interval_is_left_out_of_the_mean(toy_vehicle, toy_nominal):
    run = _run()
    model = _model(toy_vehicle, toy_nominal)
    rate = _model_rate(toy_vehicle)
    good = _segment(run, "G1b")
    empty = _segment(run, "T2b")
    rec_good = _recording(1.2 * rate, np.arange(104.0, 112.0, 0.5))
    rec_empty = _recording(rate, np.array([500.0, 500.5]))  # no pose inside the segment
    split = pc.build_pose_segments([good, empty], {"r": rec_good})
    # both segments belong to the same run name, so the same recording serves both here
    per = pc.yp_by_segment(model, split)
    assert per["G1b"]["intervals"] > 0 and per["T2b"]["intervals"] > 0
    split = pc.build_pose_segments([good], {"r": rec_good})
    per = pc.yp_by_segment(model, split)
    assert np.isnan(per["T2b"]["yp"]) and per["T2b"]["intervals"] == 0
    assert pc.yp_macro(model, split) == pytest.approx(per["G1b"]["yp"])  # the empty one is left out
    only_empty = pc.build_pose_segments([good], {"r": rec_empty})
    assert np.isnan(pc.yp_macro(model, only_empty))  # cannot be applied: NaN, nothing substituted


def test_missing_commands_make_the_interval_unusable(toy_vehicle, toy_nominal):
    run = _run()
    run.speed_cmd_mps[:] = np.nan
    run.steering_cmd_rad[:] = np.nan
    rec = _recording(_model_rate(toy_vehicle), np.arange(104.0, 108.0, 0.5))
    seg = _segment(run, "G1b")
    iv = pc.pose_yaw_intervals(rec, seg)
    assert np.isnan(pc.yp(_model(toy_vehicle, toy_nominal), rec, seg, iv))
