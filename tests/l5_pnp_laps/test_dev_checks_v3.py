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


def test_c2_every_lap_winding_inside_the_tolerance_none_missed() -> None:
    truth = [12.0] * 5
    ok, d = dc.c2_verdict(run(laps(5, lap_s=12.2, source="winding")), truth)
    assert ok and d["largest_error_s"] == pytest.approx(0.2) and d["outside"] == []
    # 0.5 s on a 12 s lap is outside max(0.3, 0.48)
    bad = laps(5, lap_s=12.0, source="winding")
    bad[2] = LapRow(3, 30.0, 12.5, 5.2, "winding", False)
    ok, d = dc.c2_verdict(run(bad), truth)
    assert not ok and d["outside"] == [3]
    assert not dc.c2_verdict(run(laps(5, lap_s=12.0, source="start_scan")), truth)[0]
    assert not dc.c2_verdict(run(laps(5, lap_s=12.0, source="winding", missed=True)), truth)[0]
    assert not dc.c2_verdict(run([]), truth)[0]


def test_the_gap_numbers_are_read_from_the_status() -> None:
    r = Replayed(
        status={"unique_gap_min": 0.05, "unique_gap_median": 0.1, "ambiguous_gap_max": 0.02}
    )
    assert dc.gap_numbers(r) == {
        "gap_unique_min": 0.05,
        "gap_unique_median": 0.1,
        "gap_ambiguous_max": 0.02,
    }
    assert math.isnan(dc.gap_numbers(Replayed())["gap_unique_min"])


# -- brief 50d: M1, M2, M3 and the three-class gap report -----------------------------------------


def with_gaps(
    unique: list[float], unknown: list[float], ambiguous: list[float], **kw: object
) -> Replayed:
    rows = kw.pop("laps", laps(3))
    return Replayed(
        laps=rows,  # type: ignore[arg-type]
        status={
            "gaps_unique": unique,
            "gaps_unknown": unknown,
            "gaps_ambiguous": ambiguous,
            **kw,
        },
    )


def test_the_m_thresholds_are_the_ones_written_in_the_brief() -> None:
    assert (dc.M1P_MAX_SHARE_A3, dc.M2_MIN_AMBIGUOUS_SHARE) == (0.5, 0.75)
    assert (dc.M4_MIN_SHARE, dc.M4_MAX_LAP_DIFF_S) == (0.9, 0.05)
    assert (dc.M3_SHIFT, dc.M3_MAX_LAP_DIFF_S) == (0.003, 0.05)


def test_gap_classes_count_and_range_each_class() -> None:
    r = with_gaps([0.05, 0.04], [0.03], [])
    assert dc.gap_classes(r) == {
        "unique": "2 (0.040 to 0.050)",
        "unknown": "1 (0.030 to 0.030)",
        "ambiguous": "0",
    }


@pytest.mark.parametrize(
    ("entries", "share", "source", "counted", "ok"),
    [
        (0, 0.0, "start_scan", 10, True),
        (0, 0.5, "start_scan", 10, True),  # exactly 0.5 is allowed
        (0, 0.51, "start_scan", 10, False),
        (1, 0.0, "start_scan", 10, False),  # entered the aliased state once
        (0, 0.0, "winding", 10, False),
        (0, 0.0, "start_scan", 0, False),  # nothing counted is no evidence
    ],
)
def test_m1p_a_placed_track_is_never_aliased_and_never_mostly_ambiguous(
    entries: int, share: float, source: str, counted: int, ok: bool
) -> None:
    r = with_gaps(
        [],
        [],
        [],
        laps=laps(3, source=source),
        aliased_entries=entries,
        share_max_a3=share,
        life_unique=counted,
        life_ambiguous=0,
    )
    assert dc.m1p_verdict(r)[0] is ok


@pytest.mark.parametrize(
    ("releases", "nu", "na", "diff", "ok"),
    [
        (0, 1, 9, 0.05, True),  # share 0.9 and 0.05 s are the limits
        (0, 1, 8, 0.0, False),  # 0.889
        (1, 0, 9, 0.0, False),  # released once
        (0, 1, 9, 0.051, False),
    ],
)
def test_m4_a_ring_is_never_released_is_over_0_9_ambiguous_and_keeps_round_4_laps(
    releases: int, nu: int, na: int, diff: float, ok: bool
) -> None:
    r = with_gaps(
        [],
        [],
        [],
        laps=laps(3, lap_s=5.2 + diff, source="winding"),
        start_verdict="aliased",
        latched_at_scan=10,
        aliased_releases=releases,
        life_unique=nu,
        life_ambiguous=na,
    )
    assert dc.m4_verdict(r, [5.2] * 3)[0] is ok
    assert not dc.m4_verdict(r, None)[0]  # no round 4 reference: not a pass
    assert not dc.m4_verdict(r, [5.2] * 2)[0]


@pytest.mark.parametrize(
    ("verdict", "source", "latched", "nu", "na", "ok"),
    [
        ("aliased", "winding", 12, 2, 6, True),  # share exactly 0.75
        ("aliased", "winding", 12, 3, 6, False),  # 0.667
        ("proven", "winding", None, 0, 9, False),
        ("aliased", "anchor", 12, 0, 9, False),
        ("aliased", "winding", None, 0, 9, False),
    ],
)
def test_m2_a_ring_is_latched_winds_every_lap_and_is_mostly_ambiguous(
    verdict: str, source: str, latched: int | None, nu: int, na: int, ok: bool
) -> None:
    r = with_gaps(
        [],
        [],
        [],
        laps=laps(3, source=source),
        start_verdict=verdict,
        latched_at_scan=latched,
        life_unique=nu,
        life_ambiguous=na,
    )
    assert dc.m2_verdict(r)[0] is ok


def test_m3_the_shifted_replays_must_keep_verdict_sources_and_lap_times() -> None:
    base = with_gaps([], [], [], start_verdict="proven")
    same = with_gaps([], [], [], start_verdict="proven")
    assert dc.m3_verdict(base, [same, same])[0]
    other = with_gaps([], [], [], start_verdict="aliased")
    assert not dc.m3_verdict(base, [same, other])[0]
    wound = with_gaps([], [], [], laps=laps(3, source="winding"), start_verdict="proven")
    assert not dc.m3_verdict(base, [wound])[0]
    near = with_gaps([], [], [], laps=laps(3, lap_s=5.2 + 0.049), start_verdict="proven")
    assert dc.m3_verdict(base, [near])[0]
    far = with_gaps([], [], [], laps=laps(3, lap_s=5.2 + 0.051), start_verdict="proven")
    assert not dc.m3_verdict(base, [far])[0]
    short = with_gaps([], [], [], laps=laps(2), start_verdict="proven")
    assert not dc.m3_verdict(base, [short])[0]
