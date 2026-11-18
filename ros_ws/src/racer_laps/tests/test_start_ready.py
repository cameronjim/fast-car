"""L1: the start rule of brief 51c (`racer_laps.start_ready.StartReadyGate`), table-driven. The
starter (the launch on the car, the runner in sim) does not start the planner until the lap node on
the external source has said ready on two consecutive statuses. Constants: the `start_gate` block
of `evaluation/prereg/pnp-v0.5-laps-camera-2.yaml`, pinned here by its hash."""

from __future__ import annotations

import hashlib
import math
from pathlib import Path
from typing import Any

import pytest
from racer_laps.start_ready import (
    REFUSAL_NO_STATUS,
    REFUSAL_RUNNING,
    StartGateConfig,
    StartReadyGate,
)

EPS = 1e-6  # "epsilon over" of the tables: far above the gate's own float tolerance

YAML = Path(__file__).resolve().parents[4] / "evaluation/prereg/pnp-v0.5-laps-camera-2.yaml"
YAML_SHA256 = "823f641c8a40337e1a81aaa440d55976585d19984ae2db55f06f0eee1289bdcd"


def ready(**changes: Any) -> dict[str, Any]:
    """A lap node status that says ready (the keys the rule reads), with `changes` applied."""
    out: dict[str, Any] = {
        "lap_source": "camera",
        "phase": "waiting_for_driving",
        "source_ready_now": True,
        "source_alive": True,
        "source_not_ready_reason": "",
    }
    out.update(changes)
    return out


def not_ready(reason: str = "similarity 0.50 below 0.90") -> dict[str, Any]:
    return ready(source_ready_now=False, source_not_ready_reason=reason)


def gate(t0: float = 100.0) -> StartReadyGate:
    return StartReadyGate(StartGateConfig(), t0)


# -- the constants are the registered ones --------------------------------------------------------


def start_gate_block(text: str) -> dict[str, str]:
    """The scalar lines of the yaml's `start_gate` block (no yaml dependency in this package)."""
    out: dict[str, str] = {}
    inside = False
    for line in text.splitlines():
        if line.startswith("start_gate:"):
            inside = True
            continue
        if inside and not line.startswith("  "):
            break
        if inside:
            key, _, rest = line.strip().partition(":")
            out[key] = rest.split("#")[0].strip()
    return out


@pytest.mark.skipif(not YAML.exists(), reason="the pre-registration yaml is outside this checkout")
def test_defaults_are_the_registered_start_gate() -> None:
    assert hashlib.sha256(YAML.read_bytes()).hexdigest() == YAML_SHA256
    block = start_gate_block(YAML.read_text())
    cfg = StartGateConfig()
    assert cfg.ready_statuses_required == int(block["ready_statuses_required"]) == 2
    assert cfg.status_gap_max_s == float(block["status_gap_max_s"]) == 2.5
    assert cfg.wait_s == float(block["wait_s"]) == 30.0


def test_config_refuses_nonsense() -> None:
    with pytest.raises(ValueError):
        StartGateConfig(ready_statuses_required=0)
    for bad in (math.nan, math.inf, -1.0, 0.0):
        with pytest.raises(ValueError):
            StartGateConfig(status_gap_max_s=bad)
        with pytest.raises(ValueError):
            StartGateConfig(wait_s=bad)
    with pytest.raises(ValueError):
        StartReadyGate(StartGateConfig(), math.nan)


# -- start ----------------------------------------------------------------------------------------


def test_one_ready_status_waits_and_two_start() -> None:
    g = gate()
    g.on_status(101.0, ready())
    assert g.decide(101.0) == ("wait", "")
    g.on_status(102.0, ready())
    assert g.decide(102.0) == ("start", "")


@pytest.mark.parametrize(
    ("gap_s", "verdict"),
    [
        (1.0, "start"),
        (2.5, "start"),  # exactly at the limit: consecutive
        (2.5 + EPS, "wait"),  # epsilon over: the count restarts
        (10.0, "wait"),
    ],
)
def test_two_ready_statuses_at_the_gap_limit(gap_s: float, verdict: str) -> None:
    g = gate()
    g.on_status(101.0, ready())
    g.on_status(101.0 + gap_s, ready())
    assert g.decide(101.0 + gap_s)[0] == verdict


def test_a_gap_over_the_limit_restarts_the_count_and_two_more_start() -> None:
    g = gate()
    g.on_status(101.0, ready())
    g.on_status(110.0, ready())  # too far from the first: this one counts as the first
    assert g.decide(110.0)[0] == "wait"
    g.on_status(111.0, ready())
    assert g.decide(111.0)[0] == "start"


def test_ready_not_ready_ready_restarts_the_count() -> None:
    g = gate()
    g.on_status(101.0, ready())
    g.on_status(102.0, not_ready())
    g.on_status(103.0, ready())
    assert g.decide(103.0)[0] == "wait"
    g.on_status(104.0, ready())
    assert g.decide(104.0)[0] == "start"


def test_a_more_than_two_required_config() -> None:
    g = StartReadyGate(StartGateConfig(ready_statuses_required=3), 100.0)
    for t in (101.0, 102.0):
        g.on_status(t, ready())
        assert g.decide(t)[0] == "wait"
    g.on_status(103.0, ready())
    assert g.decide(103.0)[0] == "start"


# -- what says ready ---------------------------------------------------------------------------------


@pytest.mark.parametrize(
    "bad",
    [
        {"lap_source": "start_scan"},
        {"lap_source": "map_pose"},
        {"lap_source": ""},
        {"lap_source": None},
        {"lap_source": 3},
        {"phase": None},
        {"phase": 3},
        {"source_ready_now": False},
        {"source_ready_now": None},
        {"source_ready_now": 1},  # not a bool
        {"source_ready_now": "true"},
        {"source_alive": False},
        {"source_alive": None},
        {"source_alive": 1},
        {"source_alive": "true"},
    ],
)
def test_a_status_that_does_not_say_ready_never_counts(bad: dict[str, Any]) -> None:
    g = gate()
    g.on_status(101.0, ready())
    g.on_status(102.0, ready(**bad))
    g.on_status(103.0, ready())
    assert g.decide(103.0)[0] == "wait"  # the bad status reset the count: ready, bad, ready


@pytest.mark.parametrize(
    "key", ["lap_source", "phase", "source_ready_now", "source_alive"]
)
def test_a_missing_key_resets_the_count(key: str) -> None:
    g = gate()
    g.on_status(101.0, ready())
    broken = ready()
    del broken[key]
    g.on_status(102.0, broken)
    g.on_status(103.0, ready())
    assert g.decide(103.0)[0] == "wait"


@pytest.mark.parametrize("source", ["camera", "truth"])
def test_both_external_sources_can_start(source: str) -> None:
    g = gate()
    g.on_status(101.0, ready(lap_source=source))
    g.on_status(102.0, ready(lap_source=source))
    assert g.decide(102.0)[0] == "start"


def test_a_start_scan_lap_node_never_starts() -> None:
    g = gate()
    t = 100.0
    while t < 140.0:
        t += 1.0
        g.on_status(t, ready(lap_source="start_scan"))
        verdict, reason = g.decide(t)
        assert verdict != "start"
    assert verdict == "refuse"


def test_a_status_that_is_not_a_mapping_resets_the_count() -> None:
    g = gate()
    g.on_status(101.0, ready())
    g.on_status(102.0, None)  # type: ignore[arg-type]
    g.on_status(103.0, ready())
    assert g.decide(103.0)[0] == "wait"


# -- phase running -------------------------------------------------------------------------------------


def test_a_lap_node_already_running_refuses_with_its_reason() -> None:
    g = gate()
    g.on_status(101.0, ready(phase="running", source_ready_now=True))
    assert g.decide(101.0) == ("refuse", REFUSAL_RUNNING)
    assert REFUSAL_RUNNING == "lap node already running"


def test_running_after_one_ready_status_refuses_and_a_later_ready_status_changes_nothing() -> None:
    g = gate()
    g.on_status(101.0, ready())
    g.on_status(102.0, ready(phase="running"))
    assert g.decide(102.0) == ("refuse", REFUSAL_RUNNING)
    g.on_status(103.0, ready())
    g.on_status(104.0, ready())
    assert g.decide(104.0) == ("refuse", REFUSAL_RUNNING)


# -- the wait -----------------------------------------------------------------------------------------


def test_no_status_for_wait_s_exactly_waits_and_epsilon_over_refuses() -> None:
    g = gate(100.0)
    assert g.decide(100.0) == ("wait", "")
    assert g.decide(129.0)[0] == "wait"
    assert g.decide(130.0)[0] == "wait"  # exactly 30.0 s after the gate's creation
    assert g.decide(130.0 + EPS) == ("refuse", REFUSAL_NO_STATUS)
    assert REFUSAL_NO_STATUS == "no lap node status"


def test_the_refusal_names_the_last_statuses_own_reason() -> None:
    g = gate(100.0)
    g.on_status(105.0, not_ready("source state 'waiting', not 'running'"))
    g.on_status(131.0, not_ready("similarity 0.50 below 0.90"))
    assert g.decide(131.0) == ("refuse", "similarity 0.50 below 0.90")


def test_a_status_without_a_reason_still_refuses_with_a_reason() -> None:
    g = gate(100.0)
    g.on_status(105.0, ready(source_ready_now=False, source_not_ready_reason=""))
    verdict, reason = g.decide(131.0)
    assert verdict == "refuse" and reason  # never an empty reason
    g2 = gate(100.0)
    broken = not_ready()
    del broken["source_not_ready_reason"]
    g2.on_status(105.0, broken)
    verdict, reason = g2.decide(131.0)
    assert verdict == "refuse" and reason
    g3 = gate(100.0)
    g3.on_status(105.0, not_ready(reason=7))  # type: ignore[arg-type]
    verdict, reason = g3.decide(131.0)
    assert verdict == "refuse" and isinstance(reason, str) and reason


def test_a_ready_status_at_the_deadline_is_not_a_refusal_if_two_came() -> None:
    g = gate(100.0)
    g.on_status(129.5, ready())
    g.on_status(130.0, ready())
    assert g.decide(130.0 + EPS)[0] == "start"  # the count was met: start wins over the deadline


def test_one_ready_status_at_the_deadline_refuses_after_it() -> None:
    g = gate(100.0)
    g.on_status(130.0, ready())
    assert g.decide(130.0)[0] == "wait"
    assert g.decide(130.0 + EPS)[0] == "refuse"


# -- a stale last status ----------------------------------------------------------------------------


def test_a_lap_node_that_died_after_one_good_status_never_starts() -> None:
    g = gate(100.0)
    g.on_status(101.0, ready())
    assert g.decide(103.5)[0] == "wait"  # 2.5 s old: still counted
    assert g.decide(103.5 + EPS)[0] == "wait"  # older than 2.5 s: the count resets, no refusal yet
    g.on_status(120.0, ready())  # a status much later is the first of a new count
    assert g.decide(120.0)[0] == "wait"
    assert g.decide(131.0)[0] == "refuse"


def test_two_good_statuses_then_silence_gives_start_at_the_second_not_later() -> None:
    g = gate(100.0)
    g.on_status(101.0, ready())
    g.on_status(102.0, ready())
    assert g.decide(102.0)[0] == "start"
    assert g.decide(500.0)[0] == "start"  # final


def test_a_stale_pair_is_not_a_start_when_decided_late() -> None:
    g = gate(100.0)
    g.on_status(101.0, ready())
    g.on_status(102.0, ready())
    assert g.decide(102.0 + 2.5 + EPS)[0] == "wait"  # nobody asked in time and the node went quiet


# -- final ---------------------------------------------------------------------------------------------


def test_start_is_final() -> None:
    g = gate()
    g.on_status(101.0, ready())
    g.on_status(102.0, ready())
    assert g.decide(102.0) == ("start", "")
    g.on_status(103.0, not_ready())
    g.on_status(104.0, ready(phase="running"))
    assert g.decide(200.0) == ("start", "")


def test_refuse_is_final() -> None:
    g = gate(100.0)
    assert g.decide(131.0)[0] == "refuse"
    g.on_status(132.0, ready())
    g.on_status(133.0, ready())
    assert g.decide(133.0) == ("refuse", REFUSAL_NO_STATUS)


# -- non-finite and backwards times ---------------------------------------------------------------------


@pytest.mark.parametrize("bad", [math.nan, math.inf, -math.inf])
def test_a_non_finite_status_time_is_ignored_and_counted(bad: float) -> None:
    g = gate()
    g.on_status(101.0, ready())
    g.on_status(bad, ready())
    g.on_status(102.0, ready())
    assert g.decide(102.0)[0] == "start"  # the ignored status neither counted nor reset
    assert g.trace()["ignored_non_finite"] == 1


@pytest.mark.parametrize("bad", [math.nan, math.inf, -math.inf])
def test_a_non_finite_decide_time_is_ignored_and_counted(bad: float) -> None:
    g = gate()
    g.on_status(101.0, ready())
    assert g.decide(bad) == ("wait", "")
    assert g.trace()["ignored_non_finite"] == 1
    g.on_status(102.0, ready())
    assert g.decide(102.0)[0] == "start"


def test_a_non_finite_decide_time_after_the_deadline_does_not_refuse_by_itself() -> None:
    g = gate(100.0)
    assert g.decide(math.inf) == ("wait", "")
    assert g.decide(131.0)[0] == "refuse"


def test_a_status_that_goes_back_in_time_is_ignored_and_counted() -> None:
    g = gate()
    g.on_status(105.0, ready())
    g.on_status(104.0, ready())
    assert g.trace()["ignored_non_monotone"] == 1
    g.on_status(106.0, ready())
    assert g.decide(106.0)[0] == "start"


# -- the trace -------------------------------------------------------------------------------------------


def test_the_trace_says_what_happened() -> None:
    g = gate(100.0)
    g.on_status(103.0, not_ready("no source status yet"))
    g.on_status(104.0, ready())
    g.on_status(105.0, ready())
    assert g.decide(105.0) == ("start", "")
    t = g.trace()
    assert t["created_s"] == 100.0
    assert t["statuses"] == 3
    assert t["first_status_s"] == 103.0
    assert t["ready_first_s"] == 104.0
    assert t["start_s"] == 105.0
    assert t["refuse_s"] is None
    assert t["verdict"] == "start" and t["reason"] == ""
    assert t["config"] == {"ready_statuses_required": 2, "status_gap_max_s": 2.5, "wait_s": 30.0}


def test_the_trace_of_a_refusal_and_of_an_empty_gate() -> None:
    g = gate(100.0)
    t = g.trace()
    assert t["statuses"] == 0 and t["first_status_s"] is None and t["ready_first_s"] is None
    assert t["verdict"] == "wait" and t["start_s"] is None
    g.decide(131.0)
    t = g.trace()
    assert t["verdict"] == "refuse" and t["refuse_s"] == 131.0
    assert t["reason"] == REFUSAL_NO_STATUS
