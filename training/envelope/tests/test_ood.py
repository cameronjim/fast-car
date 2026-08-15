"""L1 unit tests for envelope.ood: OODScorer protocol + the DistanceOODScorer reference
implementation. Every branch in DistanceOODScorer.score, every boundary value
(claude-docs/12-testing.md)."""

from __future__ import annotations

import math

from envelope.ood import DistanceOODScorer, OODScorer


def test_zero_distance_from_reference_scores_zero() -> None:
    scorer = DistanceOODScorer(reference=(1.0, -2.0, 0.5))
    assert scorer.score((1.0, -2.0, 0.5)) == 0.0


def test_default_scale_is_unnormalized_euclidean_distance() -> None:
    scorer = DistanceOODScorer(reference=(0.0, 0.0))
    assert scorer.score((3.0, 4.0)) == 5.0


def test_explicit_scale_normalizes_each_dimension() -> None:
    scorer = DistanceOODScorer(reference=(0.0, 0.0), scale=(2.0, 1.0))
    # (4/2)^2 + (3/1)^2 = 4 + 9 = 13
    assert scorer.score((4.0, 3.0)) == math.sqrt(13.0)


def test_zero_scale_dimension_is_not_normalized_not_a_division_error() -> None:
    scorer = DistanceOODScorer(reference=(0.0,), scale=(0.0,))
    assert scorer.score((5.0,)) == 5.0


def test_state_length_mismatch_scores_infinite() -> None:
    scorer = DistanceOODScorer(reference=(0.0, 0.0))
    assert scorer.score((0.0,)) == math.inf


def test_scale_length_mismatch_scores_infinite() -> None:
    scorer = DistanceOODScorer(reference=(0.0, 0.0), scale=(1.0,))
    assert scorer.score((0.0, 0.0)) == math.inf


def test_empty_reference_and_state_score_zero() -> None:
    scorer = DistanceOODScorer(reference=())
    assert scorer.score(()) == 0.0


def test_nan_in_state_scores_infinite() -> None:
    scorer = DistanceOODScorer(reference=(0.0,))
    assert scorer.score((math.nan,)) == math.inf


def test_inf_in_state_scores_infinite() -> None:
    scorer = DistanceOODScorer(reference=(0.0,))
    assert scorer.score((math.inf,)) == math.inf


def test_nan_reference_scores_infinite() -> None:
    scorer = DistanceOODScorer(reference=(math.nan,))
    assert scorer.score((0.0,)) == math.inf


def test_nan_scale_scores_infinite() -> None:
    scorer = DistanceOODScorer(reference=(0.0,), scale=(math.nan,))
    assert scorer.score((1.0,)) == math.inf


def test_overflowing_distance_scores_infinite_not_nan() -> None:
    # Both operands finite; the subtraction itself overflows past the largest finite float,
    # producing +inf rather than raising -- score() must still report +inf, not nan or a
    # crash, exercising the final isfinite(result) == False branch.
    scorer = DistanceOODScorer(reference=(1e308,))
    result = scorer.score((-1e308,))
    assert result == math.inf


def test_distance_ood_scorer_satisfies_the_ood_scorer_protocol() -> None:
    scorer = DistanceOODScorer(reference=(0.0,))
    assert isinstance(scorer, OODScorer)
