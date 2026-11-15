"""L2 properties of the B4 decisions: run order never matters, and adding passing runs never
turns a pass into a fail for the count items (contacts and completions)."""

from __future__ import annotations

import lap_battery_b4 as b4
from hypothesis import HealthCheck, given, settings
from hypothesis import strategies as st
from test_lap_battery_b4 import CONST, full, keys, mk

SETTINGS = settings(max_examples=60, deadline=None, suppress_health_check=list(HealthCheck))
CASES_18 = ("N1", "N2", "N3")


def snapshot(runs):
    rep = b4.build_report(runs, CONST)
    return {n: (d.passed, {k: v[0] for k, v in d.items.items()}) for n, d in rep.decisions.items()}


@st.composite
def messy_battery(draw, cases=("N1", "N2", "N3", "N4")):
    """A full battery in which each run may have a contact, be short, be slow, or carry a system
    lap time that is off."""
    runs = []
    for cond in b4.CONDITIONS:
        for case, d, s in keys(cases):
            laps = draw(st.sampled_from([12, 12, 12, 9]))
            late = draw(st.sampled_from([9.0, 11.0, 12.0, 12.7, 14.0]))
            contacts = draw(st.sampled_from([0, 0, 0, 0, 1]))
            off = draw(st.sampled_from([0.0, 0.0, 0.0, 0.4, 1.2]))
            times = [27.0] + [late] * (laps - 1)
            system = [times[0] + off] + times[1:]
            runs.append(
                mk(
                    cond,
                    case,
                    d,
                    s,
                    laps=laps,
                    t1=27.0,
                    late=late,
                    contacts=contacts,
                    system=system,
                )
            )
    return runs


@SETTINGS
@given(messy_battery(), st.randoms(use_true_random=False))
def test_shuffling_the_run_order_never_changes_a_decision(runs, rnd) -> None:
    shuffled = list(runs)
    rnd.shuffle(shuffled)
    assert snapshot(shuffled) == snapshot(runs)


def count_items(decision):
    """The count items in order (their names carry the run count, which changes with N4)."""
    return [v[0] for k, v in decision.items.items() if "wall contacts" in k or "complete" in k]


def count_decisions(runs):
    return {
        "c": count_items(b4.d3_s1(runs, CONST)),
        "d": count_items(b4.d3_s2(runs, "d", CONST)),
        "e": count_items(b4.d3_s2(runs, "e", CONST)),
    }


@SETTINGS
@given(messy_battery(CASES_18))
def test_adding_passing_runs_never_turns_a_count_pass_into_a_fail(runs18) -> None:
    """16 of 18 becomes 22 of 24 when the six passing N4 runs of every condition are added: the
    registered scaling is monotone in a passing run."""
    before = count_decisions(runs18)
    after = count_decisions(runs18 + full(("N4",)))
    for cond, items in before.items():
        for i, ok in enumerate(items):
            if ok:
                assert after[cond][i], (cond, i)


@SETTINGS
@given(st.integers(0, 24), st.integers(0, 24))
def test_more_short_runs_never_help_the_completion_count(n_a, n_b) -> None:
    def decide(n_short):
        ks = set(keys()[:n_short])
        runs = full(c=lambda case, d, s: {"laps": 9} if (case, d, s) in ks else {})
        return count_items(b4.d3_s1(runs, CONST))[1]  # item 2: completions

    lo, hi = sorted((n_a, n_b))
    assert decide(hi) <= decide(lo)
