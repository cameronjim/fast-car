"""L2 (hypothesis), brief 50e: the property half of test_gate_v3e.py, in its own file so the L3
job's system python (no hypothesis) leaves it out through conftest.py."""

from __future__ import annotations

from hypothesis import given, settings
from hypothesis import strategies as st
from racer_laps.ring import is_proven
from test_gate_v3e import CFG, aliased


@settings(max_examples=300, deadline=None)
@given(st.lists(st.booleans(), max_size=120))
def test_the_state_changes_only_at_a_counted_match_and_aliased_means_a_majority_ambiguous(
    seq: list[bool],
) -> None:
    from racer_laps.ring import is_released

    u = a = 0
    state = "none"  # "aliased" or "free"
    for unique in seq:
        u, a = (u + 1, a) if unique else (u, a + 1)
        before = state
        if state == "aliased":
            if is_released(u, a, CFG.alias_release_share):  # type: ignore[attr-defined]
                state = "free"
        elif aliased(u, a):
            state = "aliased"
        if state == "aliased":
            assert a > u
        assert not (before == "aliased" and state == "free" and is_proven(u, a, 4, 0.0, 0.4)), (
            u,
            a,
        )
