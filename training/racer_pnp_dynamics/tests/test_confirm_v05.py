"""L1 for the v0.5 form of the fresh confirmation (confirm_v05.py, GP2f-C item 2)."""

from __future__ import annotations

import math
from dataclasses import replace

import numpy as np
import pytest
from racer_pnp_dynamics import confirm_v05 as cf
from racer_pnp_dynamics.dataset import make_segments
from synth import make_run, make_split
from test_fit import TRUTH
from test_step_v21 import C5

G = C5.doc["gate_confirmation"]["fresh_driving"]
BOTH = {"ccw": 80.0, "cw": 70.0}
GOOD = {
    "overall": {"P1": 0.9, "Y1": 0.8},
    "ccw": {"P1": 0.9, "Y1": 0.8},
    "cw": {"P1": 0.9, "Y1": 0.8},
}


def decide(driving=150.0, dirs=BOTH, ratios=GOOD, share=0.99):
    return cf.decide_confirmation_v05(C5, driving, dirs, ratios, share)


def test_the_numbers_are_the_registered_ones():
    assert (G["min_total_s"], G["min_per_direction_s"]) == (120, 40)
    assert (G["max_p1_ratio"], G["max_y1_ratio"]) == (1.0, 1.0)
    assert G["min_step_share_in_supported_cells"] == 0.95


def test_a_better_candidate_on_enough_driving_in_both_directions_passes():
    c = decide()
    assert c.verdict == "PASS" and not c.reasons
    assert c.to_dict()["verdict"] == "PASS"


@pytest.mark.parametrize("scope", ["overall", "ccw", "cw"])
@pytest.mark.parametrize("key", ["P1", "Y1"])
def test_every_ratio_scope_passes_at_one_and_fails_just_over_or_on_nan(scope, key):
    def with_ratio(x):
        ratios = {s: dict(v) for s, v in GOOD.items()}
        ratios[scope][key] = x
        return decide(ratios=ratios)

    assert with_ratio(1.0).verdict == "PASS"
    over = with_ratio(1.0 + 1e-9)
    assert over.verdict == "FAIL" and f"{scope} {key} ratio" in over.reasons[0]
    assert with_ratio(math.nan).verdict == "FAIL"


def test_the_supported_cell_share_is_at_least_95_percent():
    assert decide(share=0.95).verdict == "PASS"
    low = decide(share=0.95 - 1e-9)
    assert low.verdict == "FAIL" and "supported cells" in low.reasons[0]
    assert decide(share=math.nan).verdict == "FAIL"


def test_a_missing_scope_is_a_failure_not_a_pass():
    assert decide(ratios={"overall": GOOD["overall"]}).verdict == "FAIL"
    assert decide(ratios={}).verdict == "FAIL"


def test_too_little_driving_in_total_or_in_a_direction_cannot_decide():
    assert decide(driving=119.9).verdict == "CANNOT DECIDE"
    assert decide(driving=120.0).verdict == "PASS"
    thin = decide(dirs={"ccw": 200.0, "cw": 39.9})
    assert thin.verdict == "CANNOT DECIDE" and "cw" in thin.reasons[0]
    assert decide(dirs={"ccw": 200.0, "cw": 40.0}).verdict == "PASS"
    assert decide(dirs={"ccw": 200.0}).verdict == "CANNOT DECIDE"
    both = decide(driving=50.0, dirs={"ccw": 10.0, "cw": 40.0})
    assert both.verdict == "CANNOT DECIDE" and len(both.reasons) == 2
    # good ratios do not rescue too little driving
    assert decide(driving=10.0, dirs={"ccw": 5.0, "cw": 5.0}).verdict == "CANNOT DECIDE"


def test_the_printout_names_every_scope_the_share_and_the_verdict():
    c = decide(ratios={**GOOD, "cw": {"P1": 1.02, "Y1": 0.9}})
    text = cf.format_confirmation_v05(c, C5, "cand", "dep")
    assert "candidate cand against deployed dep" in text and "direction by measured heading" in text
    assert "cw: P1 ratio 1.0200 (limit 1), Y1 ratio 0.9000 (limit 1)" in text
    assert "overall: P1 ratio 0.9000" in text and "steps in supported cells 0.9900" in text
    assert text.endswith("FAIL") and "cw P1 ratio 1.0200 is above 1" in text


# ---- driving time per direction -------------------------------------------------------------


def _segments(vehicle, steer, seed, seconds=60.0, flip_at=None):
    name = f"c{seed}"
    run, motion = make_run(
        TRUTH, vehicle, seconds=seconds, seed=seed, name=name, v_hi=0.9, exact_model_dt=True,
        steer_const=steer,
    )  # fmt: skip
    if flip_at is not None:
        late = motion.t_s > run.t_s[0] + flip_at
        motion = replace(motion, dyaw_rad=np.where(late, -motion.dyaw_rad, motion.dyaw_rad))
    split = make_split(name, [("S", 5.0, seconds - 5.0, "heldout", "by_heading")])
    return make_segments(run, motion, split, t_ref_s=run.t_s[0])


def test_driving_time_is_split_by_the_measured_heading(toy_vehicle):
    left = _segments(toy_vehicle, 0.3, 1)
    right = _segments(toy_vehicle, -0.3, 2)
    sec_left, sec_right = cf.direction_seconds(left), cf.direction_seconds(right)
    assert sec_left["cw"] == 0.0 and sec_left["ccw"] > 20.0
    assert sec_right["ccw"] == 0.0 and sec_right["cw"] > 20.0
    both = cf.direction_seconds(left + right)
    assert both["ccw"] == pytest.approx(sec_left["ccw"]) and both["cw"] == pytest.approx(
        sec_right["cw"]
    )


def test_a_run_that_turns_round_counts_each_direction_for_its_own_time(toy_vehicle):
    segs = _segments(toy_vehicle, 0.3, 3, seconds=120.0, flip_at=60.0)
    out = cf.direction_seconds(segs)
    assert out["ccw"] > 25.0 and out["cw"] > 25.0
    seg_s = sum(s.t1_s - s.t0_s for s in segs)
    assert out["ccw"] + out["cw"] <= seg_s + 1e-6  # never more than the segment holds


def test_no_motion_and_no_good_interval_count_no_time(toy_vehicle):
    segs = _segments(toy_vehicle, 0.3, 4)
    assert cf.direction_seconds([replace(s, motion=None) for s in segs]) == {"ccw": 0.0, "cw": 0.0}
    dead = [replace(s, motion=replace(s.motion, ok=np.zeros_like(s.motion.ok))) for s in segs]
    assert cf.direction_seconds(dead) == {"ccw": 0.0, "cw": 0.0}
