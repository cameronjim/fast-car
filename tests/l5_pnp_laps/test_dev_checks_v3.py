"""The development checks A3, A4 and A6 of brief 50 as pure functions (dev_checks_v3.py): their
thresholds at pass, at the threshold and epsilon over. The replays themselves need the 13
development dumps and two car bags, which are outside git: the notes record their numbers."""

from __future__ import annotations

import math

import dev_checks_v3 as dc
import pytest
from dev_checks_v3 import LapRow, Replayed


def laps(
    n: int,
    lap_s: float = 5.2,
    source: str = "start_scan",
    share: float = 0.0,
    missed: bool = False,
) -> list[LapRow]:
    return [LapRow(k + 1, 10.0 * (k + 1), lap_s, 5.2, source, missed, share) for k in range(n)]


def run(rows: list[LapRow], scan_ms: list[float] | None = None) -> Replayed:
    return Replayed(laps=rows, scan_ms=scan_ms or [])


def test_the_thresholds_are_the_ones_written_in_the_brief() -> None:
    assert (dc.A3_MAX_LAP_DIFF_S, dc.A3_MAX_ALIAS_SHARE) == (0.02, 0.05)
    assert (dc.A4_LAPS, dc.A4_MAX_LAP_DIFF_S) == (20, 0.10)
    assert (dc.A6_MEAN_MS, dc.A6_P99_MS) == (12.0, 25.0)


@pytest.mark.parametrize(
    ("diff", "ok"), [(0.0, True), (0.02, True), (0.0201, False), (-0.0201, False)]
)
def test_a3_every_lap_within_0_02_s_of_v21(diff: float, ok: bool) -> None:
    v3 = run(laps(5))
    v21_laps = laps(5)
    v21_laps[2] = LapRow(3, 30.0, 5.2 + diff, 5.2, "start_scan", False)
    verdict, numbers = dc.a3_verdict(v3, run(v21_laps))
    assert verdict is ok and numbers["max_abs_diff_s"] == pytest.approx(abs(diff))


@pytest.mark.parametrize(
    ("share", "source", "ok"),
    [
        (0.0, "start_scan", True),
        (0.05, "start_scan", True),
        (0.0501, "start_scan", False),
        (0.0, "anchor", False),
        (0.0, "winding", False),
    ],
)
def test_a3_start_scan_everywhere_and_an_alias_share_of_at_most_0_05(
    share: float, source: str, ok: bool
) -> None:
    verdict, _ = dc.a3_verdict(run(laps(5, source=source, share=share)), run(laps(5)))
    assert verdict is ok


def test_a3_a_lap_missing_from_either_side_fails_and_an_empty_replay_fails() -> None:
    assert dc.a3_verdict(run(laps(4)), run(laps(5)))[0] is False
    assert dc.a3_verdict(run(laps(5)), run(laps(4)))[0] is False
    assert dc.a3_verdict(run([]), run([]))[0] is False


def test_a4_20_and_20_laps_within_0_10_s_none_missed() -> None:
    assert dc.a4_verdict(run(laps(20)), run(laps(20)))[0] is True
    assert dc.a4_verdict(run(laps(19)), run(laps(20)))[0] is False
    assert dc.a4_verdict(run(laps(20)), run(laps(19)))[0] is False
    off = laps(20)
    off[7] = LapRow(8, 80.0, 5.2 + 0.1001, 5.2, "start_scan", False)
    assert dc.a4_verdict(run(off), run(laps(20)))[0] is False
    edge = laps(20)
    edge[7] = LapRow(8, 80.0, 5.2 + 0.1, 5.2, "start_scan", False)
    assert dc.a4_verdict(run(edge), run(laps(20)))[0] is True
    verdict, numbers = dc.a4_verdict(run(laps(20, missed=True)), run(laps(20)))
    assert verdict is False and numbers["missed"] == 20


def test_a6_mean_at_most_12_ms_and_p99_at_most_25_ms() -> None:
    ok, numbers = dc.a6_verdict([run([], [3.0] * 990 + [24.0] * 10)])
    assert ok and numbers["p99_ms"] == 24.0
    assert not dc.a6_verdict([run([], [3.0] * 980 + [26.0] * 20)])[0]  # p99 above 25
    assert not dc.a6_verdict([run([], [12.5] * 100)])[0]  # mean above 12
    assert dc.a6_verdict([run([], [12.0] * 100)])[0] is True  # exactly 12 ms is within
    assert dc.a6_verdict([run([], [12.0001] * 100)])[0] is False
    empty_ok, empty = dc.a6_verdict([])
    assert not empty_ok and math.isnan(empty["mean_ms"])


def test_a6_pools_every_replay_together() -> None:
    ok, numbers = dc.a6_verdict([run([], [2.0] * 100), run([], [4.0] * 100)])
    assert ok and numbers["scans"] == 200 and numbers["mean_ms"] == pytest.approx(3.0)


def test_lap_diffs_pairs_laps_by_order() -> None:
    a, b = run(laps(3)), run(laps(3, lap_s=5.0))
    assert dc.lap_diffs(a, b) == pytest.approx([0.2, 0.2, 0.2])
    assert dc.lap_diffs(run(laps(3)), run(laps(2)))[2] == math.inf


def test_percentile_of_nothing_is_nan() -> None:
    assert math.isnan(dc.percentile([], 0.99))
    assert dc.percentile([1.0, 2.0, 3.0], 0.0) == 1.0
