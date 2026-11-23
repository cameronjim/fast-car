"""L1 tests of the starvation rule the L5 canaries share (starvation.py) and of the /drive gap summary
the runner feeds it (`pnp_timing_summary`). No ROS."""

from __future__ import annotations

from types import SimpleNamespace

import pytest
import starvation
import track_family_runner as runner

# main run 44199589402, the contract 2 clockwise canary: safety_node published nothing for 0.34 s
# (stamps 3.69 then 4.03) while the planner's /drive_raw kept its 20 ms cadence.
HEALTHY = {"drive_raw": {"max_gap_s": 0.03}, "safety_drive": {"max_gap_s": 0.03}}
SAFETY_STARVED = {"drive_raw": {"max_gap_s": 0.02}, "safety_drive": {"max_gap_s": 0.34}}
PLANNER_STARVED = {"drive_raw": {"max_gap_s": 0.2}, "safety_drive": {"max_gap_s": 0.02}}


def stamps(start: float, stop: float, step: float) -> list[float]:
    n = round((stop - start) / step)
    return [round(start + k * step, 6) for k in range(n + 1)]


def recording(raw: list[float], drive: list[float]) -> SimpleNamespace:
    return SimpleNamespace(
        drive_raw=[(t, 0.4, 0.0) for t in raw],
        drive=[(t, 0.4, 0.0) for t in drive],
        events=[],
    )


def test_the_gap_is_the_longer_of_the_two_streams_and_zero_without_either() -> None:
    assert starvation.starvation_gap_s(SAFETY_STARVED) == pytest.approx(0.34)
    assert starvation.starvation_gap_s(PLANNER_STARVED) == pytest.approx(0.2)
    assert starvation.starvation_gap_s(HEALTHY) == pytest.approx(0.03)
    assert starvation.starvation_gap_s({}) == 0.0
    assert starvation.starvation_gap_s({"drive_raw": {}}) == 0.0


def run_sequence(results: list[dict], failing: set[int]):
    """A run_once that hands out `results` in order and a judge that fails the attempts in
    `failing` (by index), with the log of what was said."""
    calls: list[int] = []
    said: list[str] = []

    def run_once() -> dict:
        calls.append(len(calls))
        return {**results[len(calls) - 1], "attempt": len(calls) - 1}

    def judge(result: dict) -> None:
        assert result["attempt"] not in failing, f"attempt {result['attempt']} failed"

    return run_once, judge, said, calls


def judged(results, failing, **kwargs):
    run_once, judge, said, calls = run_sequence(results, failing)
    out = starvation.run_judged(run_once, judge, lambda r: "report", said.append, **kwargs)
    return out, said, calls


def test_a_passing_run_is_run_once_whatever_its_gaps() -> None:
    out, said, calls = judged([SAFETY_STARVED], set())
    assert out["attempt"] == 0 and len(calls) == 1
    assert said == ["report (attempt 1)"]


def test_a_failing_run_without_a_gap_fails_at_once() -> None:
    with pytest.raises(AssertionError, match="attempt 0 failed"):
        judged([HEALTHY, HEALTHY, HEALTHY], {0})


def test_a_failing_starved_run_is_run_again_and_the_clean_rerun_is_judged_the_same_way() -> None:
    out, said, calls = judged([SAFETY_STARVED, HEALTHY], {0})
    assert out["attempt"] == 1 and len(calls) == 2
    assert any("starved a process" in line and "0.34" in line for line in said)


def test_a_planner_gap_counts_as_starvation_too() -> None:
    out, _, calls = judged([PLANNER_STARVED, HEALTHY], {0})
    assert out["attempt"] == 1 and len(calls) == 2


def test_a_defect_that_fails_every_attempt_still_fails_the_test() -> None:
    # starved or not, three failing attempts raise: the rule is a rerun, not an excuse
    with pytest.raises(AssertionError, match="attempt 2 failed"):
        judged([SAFETY_STARVED] * 3, {0, 1, 2})


def test_a_rerun_that_fails_without_a_gap_is_not_run_a_third_time() -> None:
    with pytest.raises(AssertionError, match="attempt 1 failed"):
        judged([SAFETY_STARVED, HEALTHY, HEALTHY], {0, 1})


def test_the_attempt_limit_is_honoured() -> None:
    with pytest.raises(AssertionError, match="attempt 1 failed"):
        judged([SAFETY_STARVED] * 2, {0, 1}, max_attempts=2)
    with pytest.raises(AssertionError, match="at least 1"):
        judged([HEALTHY], set(), max_attempts=0)


def test_a_custom_starvation_test_replaces_the_default() -> None:
    out, _, calls = judged([HEALTHY, HEALTHY], {0}, starved=lambda r: r["attempt"] == 0)
    assert out["attempt"] == 1 and len(calls) == 2


def test_the_runner_reports_a_safety_node_that_was_not_scheduled() -> None:
    raw = stamps(1.05, 4.5, 0.02)
    drive = [t for t in stamps(1.05, 4.5, 0.02) if not 3.69 < t < 4.03]
    out = runner.pnp_timing_summary(recording(raw, drive), [(4.5, 1)])
    assert out["drive_raw"]["max_gap_s"] == pytest.approx(0.02)
    assert out["safety_drive"]["max_gap_s"] == pytest.approx(0.34)
    assert out["safety_drive"]["gaps_over_50ms"] == 1
    assert starvation.starvation_gap_s(out) > starvation.STARVED_GAP_S


def test_the_runner_reports_a_healthy_run_below_the_starvation_gap() -> None:
    out = runner.pnp_timing_summary(
        recording(stamps(1.05, 4.5, 0.02), stamps(1.05, 4.5, 0.02)), [(4.5, 1)]
    )
    assert out["safety_drive"]["gaps_over_50ms"] == 0
    assert starvation.starvation_gap_s(out) <= starvation.STARVED_GAP_S


def test_stamps_before_the_clock_arrived_do_not_count_as_a_gap() -> None:
    # a node on use_sim_time stamps 0 until its first /clock
    out = runner.pnp_timing_summary(
        recording([0.0, 0.0, 1.05, 1.07], [0.0, 0.0, 1.05, 1.07]), [(1.07, 1)]
    )
    assert out["safety_drive"] == {"messages": 2, "max_gap_s": 0.02, "gaps_over_50ms": 0}


def test_a_gap_of_four_sim_steps_is_starved_and_three_is_not() -> None:
    four = {"drive_raw": {"max_gap_s": 0.04}, "safety_drive": {"max_gap_s": 0.02}}
    three = {"drive_raw": {"max_gap_s": 0.03}, "safety_drive": {"max_gap_s": 0.03}}
    assert starvation.starvation_gap_s(four) > starvation.STARVED_GAP_S
    assert starvation.starvation_gap_s(three) <= starvation.STARVED_GAP_S


def test_a_run_with_one_drive_message_has_no_summary() -> None:
    out = runner.pnp_timing_summary(recording([1.05, 1.07], [1.05]), [(1.07, 1)])
    assert "safety_drive" not in out


def test_the_pnp_canary_timing_cases_use_the_shared_rule_not_a_drive_raw_only_one() -> None:
    """Main run 44459625336: the timing case failed on attempt 1 with watchdog 1 and rate_limit 1 and
    a clean /drive_raw (max gap 0.0207 s), because that case keyed its rerun on /drive_raw alone and
    so never saw a stalled safety_node. Every rerun decision lives in starvation.py."""
    from pathlib import Path

    source = (Path(__file__).resolve().parents[1] / "l5_pnp" / "test_pnp_canary.py").read_text()
    assert "STARVED_GAP_S" not in source
    assert 'get("max_gap_s"' not in source
    assert source.count("starvation.run_judged(") >= 2
