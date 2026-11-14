"""L1 for strata.py: bins, the merge rule on hand-built cases, macro RMSE against a hand-computed
value, and the paired block bootstrap recovering a known ratio on autocorrelated errors."""

from __future__ import annotations

import numpy as np
import pytest
from racer_pnp_dynamics import strata as st


def test_bins_follow_the_pre_registered_edges():
    speeds = np.array([0.10, 0.25, 0.59, 0.60, 0.84, 0.85, 1.24, 1.25, 3.0])
    assert st.bin_of(speeds).tolist() == [0, 0, 0, 1, 1, 2, 2, 3, 3]
    assert st.bin_label(0, 0) == "0.25-0.6" and st.bin_label(3, 3) == "1.25+"
    assert st.bin_label(0, 1) == "0.25-0.85" and st.bin_label(1, 3) == "0.6+"


def _case(counts_by_direction):
    d, v = [], []
    centres = (0.4, 0.7, 1.0, 1.4)
    for direction, counts in counts_by_direction.items():
        for b, n in enumerate(counts):
            d += [direction] * n
            v += [centres[b]] * n
    return np.array(d), np.array(v)


def test_a_stratum_with_fewer_than_30_windows_merges_into_the_next_lower_bin():
    d, v = _case({"cw": (50, 40, 10, 5), "ccw": (40, 5, 0, 100)})
    s = st.assign_strata(d, v)
    # cw: bin 3 (5) joins bin 2 (15 together, still short) which joins bin 1 (55 together)
    assert s.counts["cw:0.25-0.6"] == 50 and s.counts["cw:0.6+"] == 55
    # ccw: bin 1 (5) joins bin 0 (45 together); bin 2 is empty and makes no stratum
    assert s.counts["ccw:0.25-0.85"] == 45 and s.counts["ccw:1.25+"] == 100
    assert set(s.names) == {"cw:0.25-0.6", "cw:0.6+", "ccw:0.25-0.85", "ccw:1.25+"}
    assert s.unmerged_small == ()
    merges = {(m["direction"], m["from"], m["into"], m["windows"]) for m in s.merges}
    assert ("cw", "1.25+", "0.85-1.25", 5) in merges
    assert ("cw", "0.85+", "0.6-0.85", 15) in merges
    assert ("ccw", "0.6-0.85", "0.25-0.6", 5) in merges
    assert len(s.merges) == 3
    assert sum(s.counts.values()) == len(d)  # every window is in exactly one stratum


def test_a_short_lowest_bin_has_no_lower_neighbour_and_is_listed():
    d, v = _case({"cw": (12, 60, 60, 60)})
    s = st.assign_strata(d, v)
    assert s.counts["cw:0.25-0.6"] == 12 and s.unmerged_small == ("cw:0.25-0.6",)
    assert s.merges == ()
    # a threshold of 10 makes the same data fine
    assert st.assign_strata(d, v, min_windows=10).unmerged_small == ()


def test_macro_rmse_is_the_unweighted_mean_of_per_stratum_rmse():
    d = np.array(["cw"] * 2 + ["cw"] * 3)
    v = np.array([0.4, 0.4, 1.0, 1.0, 1.0])
    s = st.assign_strata(d, v, min_windows=1)
    err = np.array([3.0, 4.0, 1.0, -1.0, 1.0])
    per = st.stratum_rmse(err, s)
    assert per["cw:0.25-0.6"] == pytest.approx(np.sqrt((9 + 16) / 2))
    assert per["cw:0.85-1.25"] == pytest.approx(1.0)
    assert st.macro_rmse(err, s) == pytest.approx((np.sqrt(12.5) + 1.0) / 2)
    # 100 s of one cell cannot hide the other: with 98 easy windows the pooled RMSE falls to
    # about 1.2 while the macro stays at (3.54 + 1) / 2
    d2 = np.array(["cw"] * 2 + ["cw"] * 98)
    v2 = np.array([0.4, 0.4] + [1.0] * 98)
    s2 = st.assign_strata(d2, v2, min_windows=1)
    err2 = np.array([3.0, 4.0] + [1.0] * 98)
    assert st.macro_rmse(err2, s2) == pytest.approx((np.sqrt(12.5) + 1.0) / 2)
    assert np.sqrt(np.mean(err2**2)) < 1.3 < st.macro_rmse(err2, s2)
    assert np.isnan(st.macro_rmse(np.zeros(0), st.assign_strata(np.array([]), np.array([]))))


def _autocorrelated_errors(rng, n_blocks, per_block, rho=0.9):
    """Window errors whose scale wanders from block to block (and within it): what a rollout
    error looks like along a lap."""
    out, level = [], 1.0
    for _ in range(n_blocks):
        level = rho * level + (1 - rho) * rng.lognormal(0.0, 0.5)
        out.append(rng.normal(0.0, level, per_block))
    return np.concatenate(out)


def test_bootstrap_recovers_a_known_ratio_on_autocorrelated_errors():
    rng = np.random.default_rng(5)
    n_blocks, per = 120, 10
    base = _autocorrelated_errors(rng, n_blocks, per)
    better = 0.8 * base * (1.0 + 0.05 * rng.standard_normal(len(base)))  # known ratio 0.8
    block = np.repeat(np.arange(n_blocks), per)
    s = st.assign_strata(np.array(["cw"] * len(base)), np.full(len(base), 1.0))
    r = st.block_bootstrap_ratio(better, base, s, block)
    assert r["lo"] < 0.8 < r["hi"] and r["hi"] - r["lo"] < 0.1
    assert r["ratio"] == pytest.approx(0.8, abs=0.03) and r["hi"] < 1.0
    assert (r["resamples"], r["seed"], r["blocks"]) == (2000, 0, n_blocks)
    again = st.block_bootstrap_ratio(better, base, s, block)
    assert again == r  # seed 0, deterministic
    # an unrelated model of the same quality is not distinguishable from 1
    other = _autocorrelated_errors(np.random.default_rng(6), n_blocks, per)
    flat = st.block_bootstrap_ratio(other, base, s, block)
    assert flat["lo"] < 1.0 < flat["hi"] or abs(flat["ratio"] - 1.0) < 0.35


def test_bootstrap_with_independent_models_at_a_true_ratio():
    rng = np.random.default_rng(1)
    n = 4000
    block = np.arange(n) // 10
    base = rng.normal(0.0, 1.0, n)
    other = rng.normal(0.0, 0.8, n)
    s = st.assign_strata(np.array(["ccw"] * n), np.full(n, 0.5))
    r = st.block_bootstrap_ratio(other, base, s, block)
    assert r["lo"] < 0.8 < r["hi"] < 1.0


def test_bootstrap_resamples_whole_blocks_and_keeps_strata():
    """Two strata, errors constant inside a block: the interval must be wide when only a few
    blocks carry a large error, and strata missing from a resample are counted."""
    block = np.repeat(np.arange(6), 5)
    d = np.array(["cw"] * 15 + ["ccw"] * 15)
    s = st.assign_strata(d, np.full(30, 0.4), min_windows=1)
    a = np.where(block == 0, 2.0, 1.0)
    b = np.ones(30)
    r = st.block_bootstrap_ratio(a, b, s, block, n_resamples=500)
    assert r["empty_stratum_share"] > 0.0  # some resamples draw no block of a stratum
    assert r["lo"] < r["ratio"] < r["hi"] + 1e-9


def test_per_stratum_table_and_window_counts():
    d = np.array(["cw", "cw", "ccw"])
    v = np.array([0.4, 0.9, 1.5])
    s = st.assign_strata(d, v, min_windows=1)
    t = st.per_stratum_table({"m": np.array([1.0, 2.0, 3.0])}, s)
    assert t["ccw:1.25+"] == {"windows": 1, "m": 3.0}
    c = st.window_counts(d, v)
    assert c["cw:0.25-0.6"] == 1 and c["cw:0.85-1.25"] == 1 and c["ccw:1.25+"] == 1
    assert c["ccw:0.25-0.6"] == 0
