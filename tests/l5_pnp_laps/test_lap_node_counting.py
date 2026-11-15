"""L1 tests of lap_node_counting (brief 40): the counter of crossing candidates the innovation gate
rejects. It wraps racer_laps' StartScanGate._consider from outside; here with the real GateConfig
and LapLengthEstimate and stand-ins for the rest. No ROS."""

from __future__ import annotations

import json
import math
from types import SimpleNamespace

import lap_node_counting as lnc
import pytest
from racer_laps.gate import GateConfig, LapLengthEstimate

TWO_PI = 2.0 * math.pi


def gate(trusted_length: float | None = 9.0, origin: float = 100.0, heading_origin: float = 0.0):
    est = LapLengthEstimate()
    if trusted_length is not None:
        est.add(trusted_length)
        est.add(trusted_length)
    g = SimpleNamespace(
        cfg=GateConfig(wheelbase_m=0.3),
        _est=est,
        _origin_m=origin,
        _heading_origin=heading_origin,
        _cands=[],
        laps_completed=0,
    )
    return g


def match(x_m: float = 0.0, violation: float = 0.0):
    return SimpleNamespace(x_m=x_m, freespace_violation=violation)


def now(total_m: float, heading: float = TWO_PI):
    return SimpleNamespace(total_m=total_m, heading=heading)


def test_a_line_where_the_lap_length_says_is_not_rejected() -> None:
    g = gate()  # trusted length 9 m from origin 100: the line should be at 109
    assert not lnc.rejected_by_innovation(g, match(), now(109.0), TWO_PI)


def test_an_alias_metres_early_is_rejected() -> None:
    g = gate()
    # the band is max(BAND_ABS_M, BAND_REL * value); a line 4.5 m early is far outside it
    assert lnc.rejected_by_innovation(g, match(), now(104.5), TWO_PI)


def test_band_edge_both_sides() -> None:
    g = gate()
    band = g._est.band_m()
    assert not lnc.rejected_by_innovation(g, match(), now(109.0 - band + 1e-6), TWO_PI)
    assert lnc.rejected_by_innovation(g, match(), now(109.0 - band - 1e-3), TWO_PI)
    assert not lnc.rejected_by_innovation(g, match(), now(109.0 + band - 1e-6), TWO_PI)
    assert lnc.rejected_by_innovation(g, match(), now(109.0 + band + 1e-3), TWO_PI)


def test_without_a_trusted_length_nothing_is_an_innovation_rejection() -> None:
    assert not lnc.rejected_by_innovation(gate(trusted_length=None), match(), now(104.5), TWO_PI)
    g = gate()
    g._est.add(5.0)  # two laps that disagree: not trusted
    assert not g._est.trusted
    assert not lnc.rejected_by_innovation(g, match(), now(104.5), TWO_PI)


@pytest.mark.parametrize(
    "res, nw",
    [
        (match(x_m=0.7), now(104.5)),  # outside the consensus window
        (match(violation=0.5), now(104.5)),  # not a strong match
        (match(), now(104.5, heading=1.0)),  # not a lap's worth of heading yet
        (match(), now(101.0)),  # closer than the minimum lap length to the last line
    ],
)
def test_checks_before_the_innovation_test_are_not_counted_as_rejections(res, nw) -> None:
    g = gate()
    assert not lnc.rejected_by_innovation(g, res, nw, TWO_PI)
    ok, _ = lnc.passes_checks_before_innovation(g, res, nw, TWO_PI)
    assert not ok


def test_install_counts_per_lap_and_writes_the_file_after_every_event(tmp_path) -> None:
    out = tmp_path / "innovation.json"
    counter = lnc.Counter(out)

    class FakeGate(SimpleNamespace):
        pass

    calls = []

    def original(self, t, res, source, nw):
        calls.append(t)
        if res.keep:
            self._cands.append(object())

    FakeGate._consider = original
    lnc.install(FakeGate, counter, TWO_PI)
    g = FakeGate(**gate().__dict__)
    # lap 1: two aliases rejected, one real candidate kept
    g._consider(
        0.0, SimpleNamespace(x_m=0.0, freespace_violation=0.0, keep=False), None, now(104.5)
    )
    g._consider(
        0.1, SimpleNamespace(x_m=0.0, freespace_violation=0.0, keep=False), None, now(105.0)
    )
    g._consider(0.2, SimpleNamespace(x_m=0.0, freespace_violation=0.0, keep=True), None, now(109.0))
    assert json.loads(out.read_text()) == {
        "per_lap": [2],
        "accepted_per_lap": [1],
        "total": 2,
        "accepted_total": 1,
    }
    g.laps_completed = 2  # lap 3 in progress: one rejection
    g._consider(
        1.0, SimpleNamespace(x_m=0.0, freespace_violation=0.0, keep=False), None, now(104.5)
    )
    data = json.loads(out.read_text())
    assert data["per_lap"] == [2, 0, 1] and data["total"] == 3 and data["accepted_total"] == 1
    assert calls == [0.0, 0.1, 0.2, 1.0], "the original ran every time"


def test_a_candidate_the_original_kept_is_never_a_rejection() -> None:
    counter = lnc.Counter()

    class FakeGate(SimpleNamespace):
        pass

    def original(self, t, res, source, nw):
        self._cands.append(object())

    FakeGate._consider = original
    lnc.install(FakeGate, counter, TWO_PI)
    g = FakeGate(**gate().__dict__)
    g._consider(0.0, SimpleNamespace(x_m=0.0, freespace_violation=0.0), None, now(104.5))
    assert counter.as_dict()["total"] == 0 and counter.as_dict()["accepted_total"] == 1


def test_counter_without_events_has_empty_laps_and_writes_nothing_without_a_path() -> None:
    c = lnc.Counter()
    assert c.as_dict() == {"per_lap": [], "accepted_per_lap": [], "total": 0, "accepted_total": 0}
    c.note(1, True, False)  # no file: must not raise
    assert c.as_dict()["per_lap"] == [1]


def test_the_real_gate_class_has_what_the_wrapper_needs() -> None:
    from racer_laps import gate as g

    assert hasattr(g.StartScanGate, "_consider") and hasattr(g.StartScanGate, "laps_completed")
    assert math.isclose(g.TWO_PI, TWO_PI)
