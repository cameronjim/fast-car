"""L2 (hypothesis) properties."""

from __future__ import annotations

from itertools import pairwise

import pytest
from hypothesis import assume, given, settings
from hypothesis import strategies as st
from racer_eval.curves import RunLaps, lap_curve
from racer_eval.interleave import run_order
from racer_eval.stats import wilson_interval

COUNTS = st.lists(st.integers(min_value=1, max_value=8), min_size=1, max_size=5)


def feasible(counts: list[int]) -> bool:
    """Some order has no value more than twice in a row iff the largest count m fits between
    the others: m <= 2 * (rest + 1)."""
    return all(m <= 2 * (sum(counts) - m + 1) for m in counts)


def max_in_a_row(order: list[str]) -> int:
    best = run = 1
    for a, b in pairwise(order):
        run = run + 1 if a == b else 1
        best = max(best, run)
    return best


@settings(max_examples=200, deadline=None)
@given(counts=COUNTS, seed=st.integers(min_value=0, max_value=10_000))
def test_run_order_is_balanced_and_never_three_in_a_row(counts: list[int], seed: int) -> None:
    names = [f"c{i}" for i in range(len(counts))]
    wanted = dict(zip(names, counts, strict=True))
    if not feasible(counts):
        with pytest.raises(ValueError):
            run_order(names, wanted, seed)
        return
    order = run_order(names, wanted, seed)
    assert len(order) == sum(counts)
    assert all(order.count(n) == c for n, c in wanted.items())
    assert max_in_a_row(order) <= 2
    assert order == run_order(names, wanted, seed)


@settings(max_examples=200, deadline=None)
@given(
    conditions=st.integers(min_value=2, max_value=6),
    each=st.integers(min_value=1, max_value=12),
    seed=st.integers(min_value=0, max_value=10_000),
)
def test_run_order_equal_counts(conditions: int, each: int, seed: int) -> None:
    names = [f"c{i}" for i in range(conditions)]
    order = run_order(names, each, seed)
    assert all(order.count(n) == each for n in names) and max_in_a_row(order) <= 2


@settings(max_examples=300, deadline=None)
@given(
    n=st.integers(min_value=1, max_value=5000),
    frac=st.floats(min_value=0.0, max_value=1.0),
    level=st.floats(min_value=0.5, max_value=0.999),
)
def test_wilson_interval_is_inside_unit_interval_and_contains_the_rate(
    n: int, frac: float, level: float
) -> None:
    k = round(frac * n)
    assume(0 <= k <= n)
    i = wilson_interval(k, n, level)
    assert 0.0 <= i.lo <= i.estimate + 1e-12
    assert i.estimate - 1e-12 <= i.hi <= 1.0
    assert i.lo <= k / n <= i.hi


@settings(max_examples=200, deadline=None)
@given(
    times=st.lists(st.floats(min_value=1.0, max_value=60.0), max_size=15),
    dnf_after=st.none() | st.integers(min_value=0, max_value=15),
)
def test_curves_never_use_a_lap_from_beyond_a_dnf(
    times: list[float], dnf_after: int | None
) -> None:
    run = RunLaps("r", "c", tuple(times), dnf_after=dnf_after)
    limit = min(len(times), 10, len(times) if dnf_after is None else dnf_after)
    assert list(run.usable) == times[:limit]
    assume(limit > 0)
    curve = lap_curve([run, run])
    assert [p.lap for p in curve.points] == list(range(1, limit + 1))
    assert [p.mean for p in curve.points] == pytest.approx(times[:limit])
