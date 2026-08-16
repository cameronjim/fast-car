"""Unit tests for racer_train.reward: the ONLY three reward terms
(claude-docs/08-learning.md "progress along raceline - crash - envelope violation. Nothing
else."), plus the progress term's closed-loop wraparound handling.
"""

from __future__ import annotations

import pytest
from racer_train.reward import RewardWeights, compute_reward, progress_reward


def test_progress_reward_forward_step_is_positive():
    assert progress_reward(s_prev_m=1.0, s_new_m=2.0, track_length_m=100.0) == pytest.approx(1.0)


def test_progress_reward_backward_step_is_negative():
    assert progress_reward(s_prev_m=2.0, s_new_m=1.0, track_length_m=100.0) == pytest.approx(-1.0)


def test_progress_reward_lap_completion_wraps_to_small_positive():
    # Near the end of the lap (s=99) stepping across the start/finish line to s=1: naive
    # delta is -98, but the true forward progress is +2 (1 + (100-99)).
    delta = progress_reward(s_prev_m=99.0, s_new_m=1.0, track_length_m=100.0)
    assert delta == pytest.approx(2.0)


def test_progress_reward_backward_across_line_wraps_to_small_negative():
    delta = progress_reward(s_prev_m=1.0, s_new_m=99.0, track_length_m=100.0)
    assert delta == pytest.approx(-2.0)


def test_progress_reward_rejects_nonpositive_track_length():
    with pytest.raises(ValueError):
        progress_reward(s_prev_m=0.0, s_new_m=1.0, track_length_m=0.0)


def test_compute_reward_nominal_step_has_no_crash_or_envelope_penalty():
    terms = compute_reward(
        s_prev_m=0.0,
        s_new_m=1.0,
        track_length_m=100.0,
        crashed=False,
        envelope_intervened=False,
        weights=RewardWeights(
            progress_weight=1.0, crash_penalty=10.0, envelope_violation_penalty=0.1
        ),
    )
    assert terms.progress == pytest.approx(1.0)
    assert terms.crash == 0.0
    assert terms.envelope_violation == 0.0
    assert terms.total == pytest.approx(1.0)


def test_compute_reward_crash_applies_crash_penalty():
    terms = compute_reward(
        s_prev_m=0.0,
        s_new_m=1.0,
        track_length_m=100.0,
        crashed=True,
        envelope_intervened=False,
        weights=RewardWeights(
            progress_weight=1.0, crash_penalty=10.0, envelope_violation_penalty=0.1
        ),
    )
    assert terms.crash == pytest.approx(10.0)
    assert terms.total == pytest.approx(1.0 - 10.0)


def test_compute_reward_envelope_intervention_applies_penalty():
    terms = compute_reward(
        s_prev_m=0.0,
        s_new_m=1.0,
        track_length_m=100.0,
        crashed=False,
        envelope_intervened=True,
        weights=RewardWeights(
            progress_weight=1.0, crash_penalty=10.0, envelope_violation_penalty=0.1
        ),
    )
    assert terms.envelope_violation == pytest.approx(0.1)
    assert terms.total == pytest.approx(1.0 - 0.1)


def test_compute_reward_progress_weight_scales_progress_only():
    terms = compute_reward(
        s_prev_m=0.0,
        s_new_m=1.0,
        track_length_m=100.0,
        crashed=False,
        envelope_intervened=False,
        weights=RewardWeights(
            progress_weight=2.0, crash_penalty=10.0, envelope_violation_penalty=0.1
        ),
    )
    assert terms.progress == pytest.approx(2.0)
