"""L1 tests of lapref.compare: pairing, the two readings of M-L1, the differing-lap list and the
tables."""

from __future__ import annotations

import sys
from pathlib import Path

import pytest

sys.path.insert(0, str(Path(__file__).resolve().parents[1]))

from lapref import compare
from lapref.compare import LapTiming, compare_run, m_l1_verdict

TIMING = LapTiming(0.3, 0.95)
RULE_YAML = Path(__file__).resolve().parents[2] / "evaluation" / "prereg" / "pnp-v0.4.yaml"


def run(name, a, b):
    return compare_run(name, "lapref", a, "gate", b, TIMING.tolerance_s)


def test_pairing_by_order_and_share() -> None:
    r = run("K1", [5.5, 5.6, 5.7], [5.6, 5.9, 5.0])
    assert r.counts_equal and r.n_compared == 3
    assert r.diffs == pytest.approx([0.1, 0.3, -0.7])
    assert r.n_within == 2  # 0.3 is on the limit and counts as within
    assert r.share_within == pytest.approx(2 / 3)
    assert [d[0] for d in r.differing()] == [3]


def test_a_missing_lap_is_a_miss_and_is_listed() -> None:
    r = run("X2", [13.8, 13.9, 13.8], [13.8, 13.9])
    assert not r.counts_equal and r.n_laps_max == 3 and r.n_within == 2
    assert r.share_within == pytest.approx(2 / 3)
    assert r.differing() == [(3, 13.8, None, None)]
    extra = run("X4", [8.1], [8.1, 8.0])
    assert extra.differing() == [(2, None, 8.0, None)]


def test_empty_runs() -> None:
    r = run("none", [], [])
    assert r.counts_equal and r.share_within == 0.0 and r.differing() == []
    assert m_l1_verdict([], TIMING).headline == "FAIL"  # no run, no pass


def test_both_readings_agree_on_a_clean_result() -> None:
    runs = [run("A", [5.0] * 20, [5.1] * 20), run("B", [8.0] * 6, [8.2] * 6)]
    v = m_l1_verdict(runs, TIMING)
    assert v.per_run and v.pooled and v.headline == "PASS" and not v.readings_differ
    assert v.pooled_share == 1.0 and v.counts_equal_everywhere


def test_the_readings_differ_when_one_short_run_has_a_bad_lap() -> None:
    # 1 bad lap in a run of 5 (80 percent) and 24 clean laps elsewhere (pooled 29 of 30)
    runs = [run("A", [5.0] * 25, [5.0] * 25), run("B", [14.0] * 5, [14.0] * 4 + [15.0])]
    v = m_l1_verdict(runs, TIMING)
    assert not v.per_run and v.pooled and v.readings_differ
    assert v.headline == "FAIL"  # the headline is the stricter reading
    text = compare.format_m_l1(runs, TIMING)
    assert "different verdicts" in text and "B lap 5" in text and "+1.00 s" in text


def test_unequal_counts_fail_both_readings() -> None:
    runs = [run("A", [5.0] * 10, [5.0] * 9)]
    v = m_l1_verdict(runs, TIMING)
    assert not v.per_run and not v.pooled and not v.counts_equal_everywhere


def test_format_m_l1_lists_every_differing_lap() -> None:
    runs = [run("K1", [5.0, 5.5], [5.0, 6.5]), run("K2", [5.0, 5.0, 5.0], [5.0, 5.0])]
    text = compare.format_m_l1(runs, TIMING)
    assert "| K1 | 2 | 2 | equal | 1 of 2 (50 percent)" in text
    assert "| K2 | 3 | 2 | DIFFER |" in text
    assert "K1 lap 2: lapref 5.50 s, gate 6.50 s, difference +1.00 s" in text
    assert "K2 lap 3: lapref 5.00 s, gate -" in text
    assert "reading A" in text and "reading B" in text and "FAIL" in text


def test_format_m_lr() -> None:
    runs = [
        compare_run("T0", "pose", [12.0, 12.2], "lapref", [12.1, 12.1], 0.3),
        compare_run("G1a", "pose", [], "lapref", [], 0.3),
    ]
    text = compare.format_m_lr(runs)
    assert "| T0 | 2 | 2 | equal | +0.10 -0.10 | 12.10 | 12.10 |" in text
    assert "| G1a | 0 | 0 | equal | - | - | - |" in text


@pytest.mark.skipif(not RULE_YAML.is_file(), reason="pnp-v0.4.yaml is not in this checkout yet")
def test_thresholds_come_from_the_registered_yaml() -> None:
    timing = compare.load_lap_timing(RULE_YAML)
    assert timing == LapTiming(0.3, 0.95)
