"""L1 for the fresh confirmation rule (pre-registration v0.3 section 2.4)."""

from __future__ import annotations

import math

import pytest
from racer_pnp_dynamics import confirm

BOTH = {"ccw": 80.0, "cw": 70.0}


def test_constants_are_the_pre_registered_ones():
    assert confirm.MIN_DRIVING_S == 120.0
    assert confirm.P1_MAX_RATIO == 1.00 and confirm.Y1_MAX_RATIO == 1.00


def test_a_better_candidate_on_enough_driving_passes():
    c = confirm.decide_confirmation(150.0, BOTH, 0.93, 0.80)
    assert c.verdict == "PASS" and not c.reasons


@pytest.mark.parametrize(
    ("p1", "y1", "verdict"),
    [
        (1.0, 1.0, "PASS"),  # exactly at both limits
        (1.0 + 1e-9, 0.9, "FAIL"),
        (0.9, 1.0 + 1e-9, "FAIL"),
        (math.nan, 0.9, "FAIL"),
        (0.9, math.nan, "FAIL"),
    ],
)
def test_the_two_ratio_limits(p1, y1, verdict):
    assert confirm.decide_confirmation(130.0, BOTH, p1, y1).verdict == verdict


def test_too_little_driving_cannot_decide_even_with_good_ratios():
    c = confirm.decide_confirmation(119.9, BOTH, 0.5, 0.5)
    assert c.verdict == "CANNOT DECIDE" and "119.9" in c.reasons[0]
    assert confirm.decide_confirmation(120.0, BOTH, 0.5, 0.5).verdict == "PASS"


def test_a_missing_direction_cannot_decide():
    c = confirm.decide_confirmation(200.0, {"ccw": 200.0, "cw": 0.0}, 0.5, 0.5)
    assert c.verdict == "CANNOT DECIDE" and "cw" in c.reasons[0]
    c = confirm.decide_confirmation(200.0, {"ccw": 200.0}, 0.5, 0.5)
    assert c.verdict == "CANNOT DECIDE"


def test_direction_is_the_sign_of_the_net_turn():
    assert confirm.direction_of(5.0) == "ccw" and confirm.direction_of(-5.0) == "cw"


def test_the_printout_names_both_ratios_and_the_verdict():
    c = confirm.decide_confirmation(150.0, BOTH, 1.02, 0.9)
    text = confirm.format_confirmation(c, "cand", "dep")
    assert "P1 ratio 1.0200" in text and "Y1 ratio 0.9000" in text and text.endswith("FAIL")
