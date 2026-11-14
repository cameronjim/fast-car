"""L1 tests of racer_eval.stats against values worked by hand."""

from __future__ import annotations

import numpy as np
import pytest
from racer_eval.stats import (
    block_bootstrap_ratio,
    macro_rmse,
    paired_t_interval,
    rmse,
    t_interval,
    wilson_interval,
)


def test_t_interval_five_numbers() -> None:
    """x = 1..5: mean 3, sample sd sqrt(2.5) = 1.5811, se = sd / sqrt(5) = 0.70711, t(0.975, 4)
    = 2.77645, half width 2.77645 * 0.70711 = 1.96324, interval (1.03676, 4.96324)."""
    i = t_interval([1, 2, 3, 4, 5])
    assert i.estimate == pytest.approx(3.0)
    assert i.lo == pytest.approx(1.03676, abs=1e-4)
    assert i.hi == pytest.approx(4.96324, abs=1e-4)
    assert i.level == 0.95


def test_t_interval_other_level_is_narrower() -> None:
    wide, narrow = t_interval([1, 2, 3, 4, 5], 0.99), t_interval([1, 2, 3, 4, 5], 0.90)
    assert narrow.hi - narrow.lo < wide.hi - wide.lo


def test_paired_t_interval() -> None:
    """a - b = 2, 3, 3, 4: mean 3, sd sqrt(2/3) = 0.81650, se 0.40825, t(0.975, 3) = 3.18245,
    half width 1.29924, interval (1.70076, 4.29924)."""
    i = paired_t_interval([3, 5, 7, 9], [1, 2, 4, 5])
    assert i.estimate == pytest.approx(3.0)
    assert i.lo == pytest.approx(1.70076, abs=1e-4)
    assert i.hi == pytest.approx(4.29924, abs=1e-4)


def test_t_interval_refusals() -> None:
    with pytest.raises(ValueError):
        t_interval([1.0])
    with pytest.raises(ValueError):
        t_interval([[1.0, 2.0]])
    with pytest.raises(ValueError):
        t_interval([1.0, 2.0], level=1.0)
    with pytest.raises(ValueError):
        paired_t_interval([1, 2, 3], [1, 2])


def test_wilson_zero_of_twenty() -> None:
    """k = 0, n = 20, z = 1.95996: z^2 = 3.84146, z^2/n = 0.19207, centre = 0.09604 / 1.19207 =
    0.08057, half = 1.95996 * sqrt(3.84146 / 1600) / 1.19207 = 0.08057: (0, 0.16113)."""
    i = wilson_interval(0, 20)
    assert i.estimate == 0.0
    assert i.lo == 0.0
    assert i.hi == pytest.approx(0.16113, abs=1e-4)


def test_wilson_one_of_twenty() -> None:
    """k = 1, n = 20: centre = (0.05 + 0.09604) / 1.19207 = 0.12249, half = 1.95996 *
    sqrt(0.05 * 0.95 / 20 + 3.84146 / 1600) / 1.19207 = 0.11363: (0.00886, 0.23612)."""
    i = wilson_interval(1, 20)
    assert i.estimate == 0.05
    assert i.lo == pytest.approx(0.00886, abs=2e-4)
    assert i.hi == pytest.approx(0.23612, abs=2e-4)


def test_wilson_interval_contains_the_rate_at_the_ends_despite_rounding() -> None:
    """Found by the hypothesis test: k = 0, n = 19, level 0.5 gave a lower end of 1.7e-18."""
    assert wilson_interval(0, 19, 0.5).lo == 0.0
    assert wilson_interval(19, 19, 0.5).hi == 1.0


def test_wilson_twenty_of_twenty() -> None:
    """The mirror of 0 of 20: (0.83887, 1)."""
    i = wilson_interval(20, 20)
    assert i.hi == 1.0
    assert i.lo == pytest.approx(1.0 - 0.16113, abs=1e-4)


@pytest.mark.parametrize("k,n", [(-1, 5), (6, 5), (0, 0), (1, -3)])
def test_wilson_refusals(k: int, n: int) -> None:
    with pytest.raises(ValueError):
        wilson_interval(k, n)


def test_interval_contains() -> None:
    i = wilson_interval(5, 10)
    assert i.contains(0.5) and not i.contains(0.99)


def test_rmse() -> None:
    assert rmse([3.0, 4.0]) == pytest.approx(np.sqrt(12.5))


def test_block_bootstrap_ratio_exact_ratio() -> None:
    """num is exactly twice den on every sample, so every resample's ratio is 2."""
    den = np.array([1.0, 2.0, 3.0, 4.0, 5.0, 6.0])
    r = block_bootstrap_ratio(2.0 * den, den, [0, 0, 1, 1, 2, 2], n_resamples=50)
    assert (r.estimate, r.lo, r.hi) == (2.0, 2.0, 2.0)


def test_block_bootstrap_ratio_is_seeded_and_uses_the_aggregate() -> None:
    rng = np.random.default_rng(1)
    a, b = np.abs(rng.normal(size=60)), np.abs(rng.normal(size=60))
    ids = np.repeat(np.arange(12), 5)
    r1 = block_bootstrap_ratio(a, b, ids, n_resamples=200, seed=3, aggregate=rmse)
    r2 = block_bootstrap_ratio(a, b, ids, n_resamples=200, seed=3, aggregate=rmse)
    r3 = block_bootstrap_ratio(a, b, ids, n_resamples=200, seed=4, aggregate=rmse)
    assert r1 == r2
    assert r1 != r3
    assert r1.estimate == pytest.approx(rmse(a) / rmse(b))
    assert r1.lo <= r1.estimate <= r1.hi


def test_block_bootstrap_ratio_refusals() -> None:
    with pytest.raises(ValueError):
        block_bootstrap_ratio([1.0, 2.0], [1.0], [0, 0])
    with pytest.raises(ValueError):
        block_bootstrap_ratio([], [], [])
    with pytest.raises(ValueError):
        block_bootstrap_ratio([1.0], [1.0], [0], n_resamples=0)


def _ar1(rng: np.random.Generator, n: int, phi: float) -> np.ndarray:
    z = np.empty(n)
    z[0] = rng.normal()
    s = np.sqrt(1.0 - phi * phi)
    for t in range(1, n):
        z[t] = phi * z[t - 1] + s * rng.normal()
    return z


def test_block_bootstrap_ratio_covers_a_known_ratio_on_autocorrelated_data() -> None:
    """Two models' absolute errors are |AR(1)| series with the same expectation scaled by 1.2,
    so the true ratio of means is 1.2. 2.0 s blocks of 20 samples at 10 Hz."""
    rng = np.random.default_rng(7)
    ids = np.repeat(np.arange(30), 20)
    hits, trials = 0, 150
    for _ in range(trials):
        num = 1.2 * np.abs(_ar1(rng, 600, 0.5))
        den = np.abs(_ar1(rng, 600, 0.5))
        r = block_bootstrap_ratio(num, den, ids, n_resamples=300, seed=0)
        hits += r.contains(1.2)
    assert 0.88 <= hits / trials <= 0.99


def test_macro_rmse_with_a_merge() -> None:
    """Chain lo < mid < hi, min_count 3. hi has 4 samples of 3 (RMSE 3). mid has 2 samples of 2
    (too few) and merges into lo, which has 3 samples of 1: pooled RMSE sqrt((2*4 + 3*1) / 5) =
    sqrt(2.2) = 1.48324. Macro = (3 + 1.48324) / 2 = 2.24162."""
    errors = [1, 1, 1, 2, 2, 3, 3, 3, 3]
    strata = ["lo"] * 3 + ["mid"] * 2 + ["hi"] * 4
    m = macro_rmse(errors, strata, min_count=3, merge_order=[["lo", "mid", "hi"]])
    assert m.value == pytest.approx(2.24162, abs=1e-4)
    assert m.merged == (("mid", "lo"),)
    assert m.undersized == ()
    assert {g.label: g.count for g in m.groups} == {"hi": 4, "lo": 5}


def test_macro_rmse_without_a_merge_is_the_plain_mean_of_stratum_rmse() -> None:
    errors = [3, 4, 5, 6]  # a: RMSE 3.5355 over [3, 4], b: RMSE 5.5227 over [5, 6]
    m = macro_rmse(errors, ["a", "a", "b", "b"], min_count=2, merge_order=[["a", "b"]])
    assert m.merged == ()
    assert m.value == pytest.approx((np.sqrt(12.5) + np.sqrt(30.5)) / 2)


def test_macro_rmse_chained_merge_and_two_directions() -> None:
    """hi (1 sample) merges into mid (1 sample), still under 3, and both into lo (2): one group of
    4. The other direction has a stratum of its own."""
    errors = [2, 2, 2, 2, 5, 5, 5]
    strata = ["lo", "lo", "mid", "hi", "x", "x", "x"]
    m = macro_rmse(errors, strata, min_count=3, merge_order=[["lo", "mid", "hi"], ["x"]])
    assert m.merged == (("hi", "lo"), ("mid", "lo"))
    assert m.value == pytest.approx((2.0 + 5.0) / 2)


def test_macro_rmse_lowest_bin_too_small_is_kept_and_listed() -> None:
    m = macro_rmse([1.0, 1.0], ["lo", "lo"], min_count=3, merge_order=[["lo", "hi"]])
    assert m.undersized == ("lo",)
    assert m.value == pytest.approx(1.0)


def test_macro_rmse_refusals() -> None:
    with pytest.raises(ValueError):
        macro_rmse([1.0], ["a", "b"], merge_order=[["a"]])
    with pytest.raises(ValueError):
        macro_rmse([1.0], ["zzz"], merge_order=[["a"]])
    with pytest.raises(ValueError):
        macro_rmse([], [], merge_order=[["a"]])
