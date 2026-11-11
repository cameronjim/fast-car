"""The jitter histogram, against hand-counted answers."""

from __future__ import annotations

import pytest
from racer_policy.jitter import Histogram


def test_bins_and_overflow_by_hand() -> None:
    h = Histogram((1.0, 2.0, 5.0))
    for v in (0.5, 1.0, 1.5, 2.0, 4.99, 5.0, 100.0):
        h.add(v)
    # [<1) [1,2) [2,5) [>=5)
    assert h.counts == [1, 2, 2, 2]
    assert h.n == 7 and h.max == 100.0


def test_quantiles_are_upper_edges_and_never_underestimate() -> None:
    h = Histogram((10.0, 20.0, 40.0))
    for v in [5.0] * 90 + [15.0] * 9 + [30.0]:
        h.add(v)
    assert h.quantile(0.5) == 10.0
    assert h.quantile(0.99) == 20.0
    assert h.quantile(1.0) == 40.0
    assert h.quantile(0.5) >= 5.0


def test_overflow_quantile_is_the_exact_max() -> None:
    h = Histogram((1.0,))
    h.add(7.5)
    assert h.quantile(0.99) == 7.5


def test_empty_histogram_reports_zeros() -> None:
    d = Histogram((1.0, 2.0)).to_dict()
    assert d["n"] == 0 and d["p99"] == 0.0 and d["max"] == 0.0 and d["mean"] == 0.0


def test_rejects_non_finite_samples_and_bad_edges() -> None:
    h = Histogram((1.0,))
    with pytest.raises(ValueError):
        h.add(float("nan"))
    with pytest.raises(ValueError):
        Histogram((2.0, 1.0))
    with pytest.raises(ValueError):
        Histogram(())
