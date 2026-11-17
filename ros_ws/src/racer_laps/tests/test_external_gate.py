"""L1: the lap source rule of brief 51 (`racer_laps.external_gate`), table-driven: every reason at
pass, at the threshold, epsilon over, NaN and inf. Pre-registration
`preregistration-pnp-v0.5-laps-camera.md`, constants of `pnp-v0.5-laps-camera.yaml`."""

from __future__ import annotations

import hashlib
import math
from pathlib import Path

import pytest
from external_support import DT, LAP_M, Rig, steering_for
from racer_laps.external_gate import (
    DOUBTFUL_REASONS,
    REJECT_REASONS,
    ExternalConfig,
    ExternalLapGate,
)
from racer_laps.gate import BAND_ABS_M, BAND_REL, GateConfig

TWO_PI = 2.0 * math.pi
EPS = 1e-6  # "epsilon over" of the tables: far above the gate's 1e-9 noise tolerance

YAML = Path(__file__).resolve().parents[4] / "evaluation/prereg/pnp-v0.5-laps-camera.yaml"
YAML_SHA256 = "013cb2f8bec3ce96d2abf456361c3de93d8f85da48c01900c374287d7473807f"


def reasons_of(rig: Rig, n: int = -1) -> list[str]:
    return list(rig.gate.doubtful[n]["reasons"])  # type: ignore[arg-type]


# -- the constants are the registered ones --------------------------------------------------------


def source_rule(text: str) -> dict[str, float]:
    """The numeric lines of the yaml's `source_rule` block (no yaml dependency in this package)."""
    out: dict[str, float] = {}
    inside = False
    for line in text.splitlines():
        if line.startswith("source_rule:"):
            inside = True
            continue
        if inside and not line.startswith("  "):
            break
        if inside:
            key, _, rest = line.strip().partition(":")
            value = rest.split("#")[0].strip()
            try:
                out[key] = float(value)
            except ValueError:
                pass
    return out


@pytest.mark.skipif(not YAML.exists(), reason="the pre-registration yaml is outside this checkout")
def test_defaults_are_the_registered_source_rule() -> None:
    assert hashlib.sha256(YAML.read_bytes()).hexdigest() == YAML_SHA256
    rule = source_rule(YAML.read_text())
    cfg = ExternalConfig()
    for name in (
        "max_event_latency_s",
        "future_tol_s",
        "min_lap_length_m",
        "heading_slack_rad",
        "band_abs_m",
        "band_rel",
        "length_median_laps",
        "overdue_factor",
        "source_stale_s",
        "ready_similarity_min",
        "odom_extrapolation_s",
    ):
        assert getattr(cfg, name) == rule[name], name
    assert cfg.band_abs_m == BAND_ABS_M and cfg.band_rel == BAND_REL


def test_config_refuses_nonsense() -> None:
    with pytest.raises(ValueError):
        ExternalConfig(max_event_latency_s=math.nan)
    with pytest.raises(ValueError):
        ExternalConfig(length_median_laps=0)
    with pytest.raises(ValueError):
        ExternalConfig(overdue_factor=1.0)
    with pytest.raises(ValueError):
        ExternalLapGate(ExternalConfig(), GateConfig(wheelbase_m=0.33), "start_scan")


# -- timestamp ------------------------------------------------------------------------------------


@pytest.mark.parametrize("latency", [0.0, 0.26, 2.0, 2.5])
def test_lap_ends_at_the_crossing_time_not_the_arrival(latency: float) -> None:
    """F: a version that placed the lap at the arrival time is off by latency * speed (0.26 m at
    1 m/s) in length and by the latency in lap time."""
    rig = Rig()
    t_cross = LAP_M / 1.0
    crossing = rig.event(t_cross, 1, latency)
    assert crossing is not None and not crossing.missed
    assert crossing.length_m == pytest.approx(LAP_M, abs=1e-6)
    assert crossing.lap_time_s == pytest.approx(t_cross, abs=1e-9)
    assert crossing.t_cross_s == t_cross
    # what was driven during the latency belongs to the next lap
    assert rig.gate.station_m == pytest.approx(latency * 1.0, abs=0.03)
    assert rig.gate.heading_since_closure_rad == pytest.approx(
        TWO_PI * latency / LAP_M, abs=0.03 * TWO_PI / LAP_M + 1e-6
    )


def test_second_lap_is_measured_between_two_crossings() -> None:
    rig = Rig()
    rig.event(LAP_M, 1, 0.26)
    c2 = rig.event(2 * LAP_M, 2, 0.09)
    assert c2 is not None and not c2.missed
    assert c2.length_m == pytest.approx(LAP_M, abs=1e-6)
    assert c2.lap_time_s == pytest.approx(LAP_M, abs=1e-9)


def test_crossing_time_between_odometry_samples_is_interpolated() -> None:
    rig = Rig()
    t = LAP_M + 0.3 * DT  # a third of the way between two 50 Hz samples
    c = rig.event(t, 1, 0.1)
    assert c is not None and c.length_m == pytest.approx(t, abs=1e-9)


# -- rejections -----------------------------------------------------------------------------------


def _snapshot(rig: Rig) -> tuple[int, float, float, int]:
    g = rig.gate
    return (g.laps_completed, g.station_m, g.l_ref_m if g.confirmed else -1.0, g.confirmed)


REJECT_CASES = [
    # name, reason, arrival, lap_no, crossing time
    ("latency just over", "latency", 5.2 + 2.5 + EPS, 1, 5.2),
    ("latency far over", "latency", 5.2 + 4.0, 1, 5.2),
    ("future just over", "future", 5.2 - 0.05 - EPS, 1, 5.2),
    ("future far", "future", 5.0, 1, 5.5),
    ("crossing at the start", "not_monotone", 0.1, 1, 0.0),
    ("crossing before the start", "not_monotone", 0.1, 1, -0.5),
    ("nan crossing", "not_finite", 5.3, 1, math.nan),
    ("inf crossing", "not_finite", 5.3, 1, math.inf),
    ("nan arrival", "not_finite", math.nan, 1, 5.2),
    ("inf arrival", "not_finite", math.inf, 1, 5.2),
    ("minus inf crossing", "not_finite", 5.3, 1, -math.inf),
]


@pytest.mark.parametrize(("name", "reason", "arrival", "lap_no", "t_cross"), REJECT_CASES)
def test_rejected_events_change_nothing(
    name: str, reason: str, arrival: float, lap_no: int, t_cross: float
) -> None:
    rig = Rig()
    rig.drive_until(6.0)
    before = _snapshot(rig)
    assert rig.gate.on_event(arrival, lap_no, t_cross) is None
    assert _snapshot(rig) == before
    assert rig.gate.rejected[reason] == 1
    assert sum(rig.gate.rejected.values()) == 1


def test_rejected_before_the_start() -> None:
    rig = Rig(start=False)
    rig.drive_until(6.0)
    assert rig.gate.on_event(5.4, 1, 5.2) is None
    assert rig.gate.rejected["before_start"] == 1
    assert rig.gate.laps_completed == 0 and not rig.gate.started


def test_not_monotone_after_a_crossing() -> None:
    rig = Rig()
    assert rig.event(LAP_M, 1) is not None
    before = _snapshot(rig)
    rig.drive_until(LAP_M + 1.0)
    assert rig.gate.on_event(LAP_M + 1.0, 2, LAP_M) is None  # same crossing time
    assert rig.gate.on_event(LAP_M + 1.0, 2, LAP_M - 0.5) is None  # earlier
    assert rig.gate.rejected["not_monotone"] == 2
    assert _snapshot(rig)[0] == before[0]


def test_duplicate_lap_number_is_rejected() -> None:
    rig = Rig()
    assert rig.event(LAP_M, 1) is not None
    rig.drive_until(LAP_M + 2.0)
    before = _snapshot(rig)
    assert rig.gate.on_event(LAP_M + 2.0, 1, LAP_M + 1.9) is None  # a later crossing, same number
    assert rig.gate.rejected["duplicate"] == 1
    assert _snapshot(rig) == before


@pytest.mark.parametrize(("latency", "accepted"), [(2.5, True), (2.5 + EPS, False)])
def test_latency_threshold(latency: float, accepted: bool) -> None:
    rig = Rig()
    c = rig.event(LAP_M, 1, latency)
    assert (c is not None) is accepted


@pytest.mark.parametrize(("ahead", "accepted"), [(0.05, True), (0.05 + EPS, False)])
def test_future_threshold(ahead: float, accepted: bool) -> None:
    rig = Rig()
    rig.drive_until(LAP_M + 0.3)
    arrival = LAP_M
    c = rig.gate.on_event(arrival, 1, arrival + ahead)
    assert (c is not None) is accepted


# -- doubtful laps, each reason alone -------------------------------------------------------------


def first_lap(rig: Rig, no: int = 1, latency: float = 0.16):  # type: ignore[no-untyped-def]
    return rig.event(LAP_M, no, latency)


READY_CASES = [
    ("ready", "running", 0, 1.0, 0.0, True),
    ("similarity at the threshold", "running", 0, 0.90, 0.0, True),
    ("similarity epsilon under", "running", 0, 0.90 - EPS, 0.0, False),
    ("state waiting", "waiting", 0, 1.0, 0.0, False),
    ("laps already 1", "running", 1, 1.0, 0.0, False),
    ("status exactly at the limit", "running", 0, 1.0, 3.0, True),
    ("status epsilon stale", "running", 0, 1.0, 3.0 + EPS, False),
    ("similarity nan", "running", 0, math.nan, 0.0, False),
]


@pytest.mark.parametrize(("name", "state", "laps", "sim", "age", "ready"), READY_CASES)
def test_ready_is_decided_at_the_start(
    name: str, state: str, laps: int, sim: float, age: float, ready: bool
) -> None:
    rig = Rig(start=False, status_period_s=None, status_at_start=False)
    rig.drive_until(3.0)
    rig.gate.on_source_status(rig.t - age, state, laps, sim)
    assert rig.gate.source_ready_now is ready
    assert rig.gate.source_ready is False  # not started
    assert rig.gate.set_start(rig.t)
    assert rig.gate.source_ready is ready
    # the run's laps carry the verdict
    rig.status_period_s = 1.0  # the camera speaks on from here
    rig.deliver_status(rig.t)
    t0 = rig.t
    c = rig.event(t0 + LAP_M, 1)
    assert c is not None
    assert c.missed is (not ready)
    if not ready:
        # (a status gap that straddles the start is a gap of the lap as well)
        expected = ["ready", "alive"] if age > 3.0 else ["ready"]
        assert reasons_of(rig) == expected
    else:
        assert not rig.gate.doubtful
    assert rig.gate.source_ready is ready  # fixed at the start


def test_no_status_at_all_is_not_ready() -> None:
    rig = Rig(start=False, status_period_s=None, status_at_start=False)
    assert not rig.gate.source_ready_now
    assert "no source status" in rig.gate.source_not_ready_reason
    rig.gate.set_start(0.0)
    assert not rig.gate.source_ready


def test_source_ready_now_follows_the_status_before_the_start() -> None:
    rig = Rig(start=False, status_period_s=None, status_at_start=False)
    rig.gate.on_source_status(0.0, "waiting", 0, 0.1)
    assert not rig.gate.source_ready_now
    rig.gate.on_source_status(0.5, "running", 0, 1.0)
    assert rig.gate.source_ready_now
    assert rig.gate.source_not_ready_reason == ""
    rig.drive_until(3.6)  # the status gets old (the odometry clock runs on)
    assert not rig.gate.source_ready_now
    assert "old" in rig.gate.source_not_ready_reason


def test_sequence_skip_and_restart() -> None:
    rig = Rig()
    c = first_lap(rig, no=2)  # the first event must be number 1
    assert c is not None and c.missed
    assert reasons_of(rig) == ["sequence"]
    rig = Rig()
    assert not first_lap(rig, 1).missed  # type: ignore[union-attr]
    c = rig.event(2 * LAP_M, 3)  # skipped 2
    assert c is not None and reasons_of(rig) == ["sequence"]
    c = rig.event(3 * LAP_M, 1)  # restarted at 1
    assert c is not None and reasons_of(rig) == ["sequence"]
    c = rig.event(4 * LAP_M, 2)  # and counts on from there
    assert c is not None and not c.missed


@pytest.mark.parametrize(
    ("stop_s", "accepted"),
    [(LAP_M - 0.2, True), (LAP_M - 0.2 - 0.001, False)],
)
def test_odometry_must_cover_the_crossing(stop_s: float, accepted: bool) -> None:
    """The odometry stops at `stop_s`; the crossing is 0.2 s after it (covered, extrapolated at
    the last speed) or 0.201 s (not covered)."""
    rig = Rig()
    rig.drive_until(stop_s)
    t_cross = LAP_M
    c = rig.gate.on_event(t_cross + 0.1, 1, t_cross)
    assert c is not None
    assert c.missed is (not accepted)
    if accepted:
        assert c.length_m == pytest.approx(LAP_M, abs=1e-6)
    else:
        assert reasons_of(rig) == ["odometry"]


def test_odometry_older_than_the_history_is_not_covered() -> None:
    rig = Rig()
    rig.drive_until(40.0)  # 30 s of history kept: t = 5.2 is gone
    c = rig.gate.on_event(40.0, 1, LAP_M)
    assert c is None  # also 34.8 s of latency: rejected first
    assert rig.gate.rejected["latency"] == 1


@pytest.mark.parametrize(
    ("gap_s", "alive"), [(2.0, True), (3.0, True), (3.0 + EPS, False), (6.0, False)]
)
def test_the_source_must_stay_alive(gap_s: float, alive: bool) -> None:
    """Statuses at 0 and at `gap_s`, then every second; odometry runs through the gap."""
    rig = Rig(status_period_s=None)
    rig.gate.on_source_status(0.0, "running", 0, 1.0)
    rig.drive_until(gap_s)
    rig.gate.on_source_status(gap_s, "running", 0, 1.0)
    rig.status_period_s = 1.0
    rig.next_status_s = gap_s + 1.0
    c = rig.event(LAP_M, 1)
    assert c is not None
    assert c.missed is (not alive)
    if not alive:
        assert reasons_of(rig) == ["alive"]


def test_a_gap_is_latched_by_the_clock_alone() -> None:
    g = Rig(status_period_s=None).gate
    g.on_source_status(0.0, "running", 0, 1.0)
    g.on_clock(3.0)
    assert g.source_alive
    g.on_clock(3.0 + EPS)
    assert not g.source_alive


def test_alive_flag_is_per_lap() -> None:
    """A silent stretch inside lap 1 makes lap 1 doubtful; lap 2 is confirmed again."""
    rig = Rig(status_period_s=None)
    rig.gate.on_source_status(0.0, "running", 0, 1.0)
    rig.drive_until(4.0)  # silent from 0 to 4 s: dead during lap 1
    rig.gate.on_source_status(4.0, "running", 0, 1.0)
    rig.status_period_s = 1.0
    rig.next_status_s = 5.0
    rig.state = "running"
    c1 = rig.event(LAP_M, 1)
    assert c1 is not None and reasons_of(rig) == ["alive"]
    c2 = rig.event(2 * LAP_M, 2)
    assert c2 is not None and not c2.missed


LENGTH_CASES = [
    # name, lap 1 length (reference), lap 2 length, confirmed?
    ("short", 2.9, None, False),
    ("just long enough", 3.0, None, True),
    ("epsilon short", 3.0 - EPS, None, False),
    ("abs band edge", 5.2, 5.2 + 0.5, True),
    ("abs band epsilon over", 5.2, 5.2 + 0.5 + EPS, False),
    ("abs band edge under", 5.2, 5.2 - 0.5, True),
    ("abs band epsilon under", 5.2, 5.2 - 0.5 - EPS, False),
    ("rel band edge (8 m, 0.64)", 8.0, 8.0 + 0.64, True),
    ("rel band epsilon over (8 m)", 8.0, 8.0 + 0.64 + EPS, False),
    ("abs wins below 6.25 m (4 m, 0.5)", 4.0, 4.0 + 0.5, True),
    ("abs wins epsilon over (4 m)", 4.0, 4.0 + 0.5 + EPS, False),
    ("switch at 6.25 m, edge", 6.25, 6.25 + 0.5, True),
    ("switch at 6.25 m, epsilon over", 6.25, 6.25 + 0.5 + EPS, False),
    ("just over the switch", 6.5, 6.5 + 0.52, True),
    ("just over the switch, epsilon over", 6.5, 6.5 + 0.52 + EPS, False),
    ("a double lap", 5.2, 10.4, False),
    ("half a lap", 5.2, 2.6, False),
]


@pytest.mark.parametrize(("name", "l1", "l2", "confirmed"), LENGTH_CASES)
def test_length_rule(name: str, l1: float, l2: float | None, confirmed: bool) -> None:
    if l2 is None:  # the first lap alone, no reference yet
        rig = Rig(lap_m=l1)
        c = rig.event(l1, 1)
        assert c is not None and c.missed is (not confirmed)
        if not confirmed:
            assert "length" in reasons_of(rig)
        return
    rig = Rig(lap_m=l1)
    assert not rig.event(l1, 1).missed  # type: ignore[union-attr]
    c = rig.event(l1 + l2, 2)
    assert c is not None
    assert c.missed is (not confirmed), reasons_of(rig) if rig.gate.doubtful else "confirmed"
    if not confirmed:
        reasons = reasons_of(rig)
        assert "length" in reasons
        if abs(l2 - l1) < 0.3 * l1 and l2 > 3.0:  # a band miss alone is a length verdict alone
            assert reasons == ["length"]


@pytest.mark.parametrize(
    ("gain", "confirmed"),
    [
        (1.0, True),
        ((TWO_PI - 1.2) / TWO_PI, True),
        ((TWO_PI - 1.2 - EPS) / TWO_PI, False),
        ((TWO_PI + 1.2) / TWO_PI, True),
        ((TWO_PI + 1.2 + EPS) / TWO_PI, False),
        (2.0, False),  # 4 pi: two turns
        (0.0, False),  # no turning
    ],
)
def test_heading_rule(gain: float, confirmed: bool) -> None:
    rig = Rig(gain=gain)
    c = first_lap(rig)
    assert c is not None and c.missed is (not confirmed)
    if not confirmed:
        assert reasons_of(rig) == ["heading"]
    assert c.heading_rad == pytest.approx(gain * TWO_PI, abs=1e-6)


def test_clockwise_heading_counts_in_magnitude() -> None:
    rig = Rig()
    rig.gate.on_steering(-steering_for(LAP_M))
    c = first_lap(rig)
    assert c is not None and not c.missed
    assert c.heading_rad == pytest.approx(-TWO_PI, abs=1e-6)


def test_every_failed_reason_is_listed() -> None:
    rig = Rig(ready=False, gain=2.0)
    c = rig.event(2 * LAP_M, 3)  # not ready, skipped numbers, two turns
    assert c is not None
    # no reference yet, so a double first lap is not a length miss; two turns of heading are
    assert sorted(reasons_of(rig)) == sorted(["ready", "sequence", "heading"])
    assert set(reasons_of(rig)) <= set(DOUBTFUL_REASONS)


# -- the reference length -------------------------------------------------------------------------


def drive_laps(rig: Rig, lengths: list[float]) -> None:
    """Laps of the given lengths one after the other at 1 m/s; steering re-set for each so the
    heading stays one turn."""
    t = 0.0
    for i, length in enumerate(lengths):
        rig.gate.on_steering(steering_for(length))
        t += length
        rig.event(t, i + 1)


def test_reference_is_the_median_of_the_last_five_confirmed_laps() -> None:
    rig = Rig()
    lengths = [5.2, 5.3, 5.1, 5.25, 5.15, 5.3, 5.2]
    drive_laps(rig, lengths)
    assert rig.gate.confirmed == 7
    assert rig.gate.l_ref_m == pytest.approx(sorted(lengths[-5:])[2], abs=1e-6)


def test_a_doubtful_lap_never_enters_the_reference() -> None:
    rig = Rig()
    drive_laps(rig, [5.2, 5.2, 10.4, 5.2])  # lap 3 is a double lap (doubtful), lap 4 is fine
    assert [d["lap"] for d in rig.gate.doubtful] == [3]
    assert rig.gate.confirmed == 3
    assert rig.gate.l_ref_m == pytest.approx(5.2, abs=1e-6)
    assert rig.gate.missed_laps == [3]
    assert rig.gate.laps_completed == 4


def test_lap_one_doubtful_then_lap_two_confirmed_by_the_lap_one_rule() -> None:
    """Lap 1 doubtful (event numbers start at 2): no reference exists, so lap 2 is judged by the
    minimum length and the heading alone."""
    rig = Rig()
    c1 = rig.event(LAP_M, 2)
    assert c1 is not None and c1.missed
    assert math.isnan(rig.gate.l_ref_m)
    c2 = rig.event(2 * LAP_M, 3)
    assert c2 is not None and not c2.missed
    assert rig.gate.l_ref_m == pytest.approx(LAP_M, abs=1e-6)


def test_reference_trust_needs_two_agreeing_laps() -> None:
    rig = Rig()
    assert not rig.gate.l_ref_trusted
    drive_laps(rig, [5.2])
    assert not rig.gate.l_ref_trusted
    drive_laps_more = rig.event(2 * LAP_M, 2)
    assert drive_laps_more is not None
    assert rig.gate.l_ref_trusted
    assert rig.gate.suspect_laps == []


# -- overdue and a dead source --------------------------------------------------------------------


def test_never_overdue_before_a_reference_exists() -> None:
    rig = Rig(status_period_s=1.0)
    rig.drive_to_total(40.0)
    assert not rig.gate.poll_uncertain()
    assert not rig.gate.station_uncertain


def test_overdue_with_one_confirmed_lap_at_the_overdue_factor() -> None:
    rig = Rig()
    assert not rig.event(LAP_M, 1).missed  # type: ignore[union-attr]
    # the car keeps going; the reference is 5.2, overdue at 1.25 * 5.2 = 6.5 m of station
    rig.drive_to_total(LAP_M + 1.25 * LAP_M - 0.1)
    assert not rig.gate.poll_uncertain()
    rig.drive_to_total(LAP_M + 1.25 * LAP_M + 0.05)
    assert rig.gate.poll_uncertain()


def test_overdue_with_two_confirmed_laps_at_the_reference_plus_band() -> None:
    rig = Rig()
    rig.event(LAP_M, 1)
    rig.event(2 * LAP_M, 2)
    rig.drive_to_total(2 * LAP_M + LAP_M + 0.5 - 0.1)
    assert not rig.gate.poll_uncertain()
    rig.drive_to_total(2 * LAP_M + LAP_M + 0.5 + 0.05)
    assert rig.gate.poll_uncertain()


def test_overdue_latches_until_the_next_crossing() -> None:
    rig = Rig()
    rig.event(LAP_M, 1)
    rig.event(2 * LAP_M, 2)
    rig.drive_to_total(3 * LAP_M + 0.8)
    assert rig.gate.poll_uncertain()
    # the station cannot go back below the band, but the latch is what is asked: poll again
    assert rig.gate.poll_uncertain()
    c = rig.event(3 * LAP_M + 1.0, 3)  # a (doubtful, long) lap closes: the latch is cleared
    assert c is not None
    assert not rig.gate.poll_uncertain()
    assert not rig.gate.status()["station_uncertain"]


def test_a_dead_source_sets_it_and_a_returning_status_does_not_clear_it() -> None:
    rig = Rig(status_period_s=None)
    rig.gate.on_source_status(0.0, "running", 0, 1.0)
    rig.drive_until(2.0)
    assert not rig.gate.poll_uncertain()
    rig.drive_until(3.2)  # 3.2 s without a status
    assert rig.gate.poll_uncertain()
    assert not rig.gate.source_alive
    rig.gate.on_source_status(rig.t, "running", 0, 1.0)  # the camera comes back
    assert rig.gate.source_alive
    assert rig.gate.poll_uncertain()  # still latched
    rig.status_period_s = 1.0
    rig.next_status_s = rig.t + 0.5
    c = rig.event(LAP_M, 1)  # the next crossing clears it (and the lap is doubtful: it was dead)
    assert c is not None and c.missed
    assert not rig.gate.poll_uncertain()


def test_before_the_start_nothing_is_uncertain() -> None:
    rig = Rig(start=False, status_period_s=None, status_at_start=False)
    rig.drive_until(10.0)
    assert not rig.gate.poll_uncertain()


# -- the status -----------------------------------------------------------------------------------


def test_status_keys() -> None:
    rig = Rig()
    rig.event(LAP_M, 1, 0.2)
    rig.event(2 * LAP_M, 3, 0.1)  # a skipped number: doubtful
    s = rig.gate.status()
    for key in (
        "source_ready",
        "source_ready_now",
        "source_not_ready_reason",
        "source_alive",
        "source_status_age_s",
        "confirmed_laps",
        "doubtful_laps",
        "rejected_events",
        "event_latency_s_last",
        "event_latency_s_max",
        "line_source",
        "station_m",
        "l_ref_m",
        "l_ref_trusted",
        "station_uncertain",
        "missed_gate_laps",
    ):
        assert key in s, key
    assert s["line_source"] == "camera"
    assert s["confirmed_laps"] == 1
    assert s["doubtful_laps"] == [{"lap": 2, "event_lap": 3, "reasons": ["sequence"]}]
    assert s["missed_gate_laps"] == [2]
    assert s["event_latency_s_last"] == pytest.approx(0.1, abs=1e-9)
    assert s["event_latency_s_max"] == pytest.approx(0.2, abs=1e-9)
    assert set(s["rejected_events"]) == set(REJECT_REASONS)  # type: ignore[arg-type]


def test_malformed_is_counted() -> None:
    rig = Rig()
    rig.gate.note_malformed()
    assert rig.gate.status()["rejected_events"]["malformed"] == 1  # type: ignore[index]


def test_truth_source_name() -> None:
    rig = Rig(source="truth")
    c = rig.event(LAP_M, 1)
    assert c is not None and c.line_source == "truth"
    assert rig.gate.status()["line_source"] == "truth"


def test_odometry_edge_cases() -> None:
    g = Rig().gate
    n = len(g._hist)
    g.on_odom(math.nan, 1.0)
    g.on_odom(1.0, math.inf)
    g.on_odom(-1.0, 1.0)  # older than the last sample: ignored
    assert len(g._hist) == n
    g.on_steering(math.nan)  # ignored
    # a negative speed adds neither distance nor heading
    before = (g.total_m, g.heading_since_closure_rad)
    t = g._hist[-1].t
    g.on_odom(t + 0.02, -1.0)
    g.on_odom(t + 0.04, -1.0)
    assert g.total_m >= before[0] and g.total_m < before[0] + 0.02
    assert g.set_start(t) is False  # already started
    assert not ExternalLapGate(ExternalConfig(), g.odom).set_start(0.0)  # no odometry yet


def test_right_turns_use_the_right_scale() -> None:
    cfg = GateConfig(wheelbase_m=0.33, yaw_gain_table=((0.0, 1.0),), yaw_gain_right_scale=0.5)
    g = ExternalLapGate(ExternalConfig(), cfg)
    g.on_odom(0.0, 1.0)
    g.set_start(0.0)
    g.on_steering(-steering_for(LAP_M))
    for k in range(1, 261):
        g.on_odom(k * DT, 1.0)
    assert g.heading_since_closure_rad == pytest.approx(-0.5 * TWO_PI, abs=1e-6)
