"""L1/L2 tests for racer_replay.mutators (claude-docs/12-testing.md L4).

Each mutator's *contract* is property-tested with hypothesis against
synthetic in-memory streams, per the roadmap 0.9 brief:
  - drop-mutator output is a subsequence
  - NaN-mutator preserves count and timestamps
  - jump-mutator only alters timestamps
  - out-of-order-mutator output is a permutation
  - frozen-sensor-mutator preserves count/timestamps, freezes data
"""

from __future__ import annotations

import math

import pytest
from hypothesis import given
from hypothesis import strategies as st
from racer_replay.mutators import (
    DropFrameMutator,
    FrozenSensorMutator,
    NaNMutator,
    OutOfOrderMutator,
    TimestampJumpMutator,
    compose,
)
from racer_replay.streams import InMemoryMessageStream, StreamMessage


# Each message carries a unique `id` field so mutator contracts (subsequence,
# permutation, "only timestamps changed") can be checked by identity rather
# than by value, even after a mutator rewrites other fields.
def _build_stream(n: int) -> list[StreamMessage]:
    return [
        StreamMessage(
            topic=["/scan", "/ingest/imu", "/odom"][i % 3],
            timestamp_ns=1_000_000 * i,
            data={"id": i, "value": float(i) * 1.5},
        )
        for i in range(n)
    ]


message_lists = st.integers(min_value=1, max_value=30).map(_build_stream)


class TestNaNMutator:
    def test_targets_only_selected_fields_and_indices(self):
        stream = InMemoryMessageStream(_build_stream(4))
        mutated = NaNMutator(indices=[1, 3], fields=["value"])(stream)
        assert math.isnan(mutated[1].data["value"])
        assert math.isnan(mutated[3].data["value"])
        assert mutated[0].data["value"] == 0.0
        assert mutated[2].data["value"] == 3.0

    def test_missing_field_is_skipped_not_added(self):
        stream = InMemoryMessageStream([StreamMessage("/scan", 0, {"id": 0})])
        mutated = NaNMutator(indices=[0], fields=["nonexistent"])(stream)
        assert "nonexistent" not in mutated[0].data

    @given(messages=message_lists)
    def test_preserves_count_and_timestamps(self, messages):
        mutated = NaNMutator(indices=range(len(messages)), fields=["value"])(
            InMemoryMessageStream(messages)
        )
        assert len(mutated) == len(messages)
        assert [m.timestamp_ns for m in mutated] == [m.timestamp_ns for m in messages]
        assert [m.topic for m in mutated] == [m.topic for m in messages]

    @given(messages=message_lists)
    def test_untouched_fields_survive(self, messages):
        mutated = NaNMutator(indices=range(len(messages)), fields=["value"])(
            InMemoryMessageStream(messages)
        )
        assert [m.data["id"] for m in mutated] == [m.data["id"] for m in messages]


class TestTimestampJumpMutator:
    @given(messages=message_lists, from_index=st.integers(min_value=0, max_value=29))
    def test_only_timestamps_from_index_onward_change(self, messages, from_index):
        delta = 5_000_000
        mutated = TimestampJumpMutator(from_index=from_index, delta_ns=delta)(
            InMemoryMessageStream(messages)
        )
        assert len(mutated) == len(messages)
        for i, (orig, new) in enumerate(zip(messages, mutated)):
            assert new.topic == orig.topic
            assert new.data == orig.data
            if i < from_index:
                assert new.timestamp_ns == orig.timestamp_ns
            else:
                assert new.timestamp_ns == orig.timestamp_ns + delta

    def test_forward_jump_increases_timestamp(self):
        stream = InMemoryMessageStream(_build_stream(3))
        mutated = TimestampJumpMutator(from_index=1, delta_ns=10)(stream)
        assert mutated[1].timestamp_ns > 1_000_000

    def test_backward_jump_decreases_timestamp(self):
        stream = InMemoryMessageStream(_build_stream(3))
        mutated = TimestampJumpMutator(from_index=1, delta_ns=-500_000)(stream)
        assert mutated[1].timestamp_ns == 1_000_000 - 500_000


class TestDropFrameMutator:
    @given(messages=message_lists, drop_every=st.integers(min_value=2, max_value=5))
    def test_output_is_a_subsequence_by_id(self, messages, drop_every):
        mutated = DropFrameMutator(predicate=lambda i, m: i % drop_every == 0)(
            InMemoryMessageStream(messages)
        )
        kept_ids = [m.data["id"] for m in mutated]
        all_ids = [m.data["id"] for m in messages]
        # Subsequence: kept_ids appears in all_ids in the same relative
        # order, i.e. can be obtained by deleting zero or more elements.
        it = iter(all_ids)
        assert all(any(x == target for x in it) for target in kept_ids)

    def test_drop_by_explicit_indices(self):
        stream = InMemoryMessageStream(_build_stream(5))
        mutated = DropFrameMutator(indices=[0, 2, 4])(stream)
        assert [m.data["id"] for m in mutated] == [1, 3]

    def test_requires_indices_or_predicate(self):
        with pytest.raises(ValueError):
            DropFrameMutator()

    @given(messages=message_lists)
    def test_dropping_nothing_is_identity(self, messages):
        mutated = DropFrameMutator(indices=[])(InMemoryMessageStream(messages))
        assert mutated == messages


class TestOutOfOrderMutator:
    @given(messages=message_lists)
    def test_output_is_a_permutation(self, messages):
        if len(messages) < 2:
            return
        mutated = OutOfOrderMutator(swaps=[(0, len(messages) - 1)])(InMemoryMessageStream(messages))
        assert sorted(m.data["id"] for m in mutated) == sorted(m.data["id"] for m in messages)
        assert len(mutated) == len(messages)

    def test_swap_actually_reorders(self):
        stream = InMemoryMessageStream(_build_stream(4))
        mutated = OutOfOrderMutator(swaps=[(0, 3)])(stream)
        assert mutated[0].data["id"] == 3
        assert mutated[3].data["id"] == 0

    def test_result_is_no_longer_timestamp_sorted(self):
        stream = InMemoryMessageStream(_build_stream(4))
        mutated = OutOfOrderMutator(swaps=[(0, 3)])(stream)
        timestamps = [m.timestamp_ns for m in mutated]
        assert timestamps != sorted(timestamps)


class TestFrozenSensorMutator:
    def test_freezes_topic_payload_for_the_window(self):
        stream = InMemoryMessageStream(_build_stream(9))  # topics cycle /scan,/imu,/odom
        mutated = FrozenSensorMutator(topic="/scan", start_index=0, length=9)(stream)
        scan_payloads = [m.data for m in mutated if m.topic == "/scan"]
        assert all(p == scan_payloads[0] for p in scan_payloads)

    def test_other_topics_unaffected(self):
        stream = InMemoryMessageStream(_build_stream(9))
        original = _build_stream(9)
        mutated = FrozenSensorMutator(topic="/scan", start_index=0, length=9)(stream)
        for orig, new in zip(original, mutated):
            if orig.topic != "/scan":
                assert new.data == orig.data

    @given(messages=message_lists)
    def test_preserves_count_topics_and_timestamps(self, messages):
        mutated = FrozenSensorMutator(topic="/scan", start_index=0, length=len(messages))(
            InMemoryMessageStream(messages)
        )
        assert len(mutated) == len(messages)
        assert [m.topic for m in mutated] == [m.topic for m in messages]
        assert [m.timestamp_ns for m in mutated] == [m.timestamp_ns for m in messages]

    def test_freezes_at_last_value_before_window(self):
        stream = InMemoryMessageStream(
            [
                StreamMessage("/scan", 0, {"value": 1.0}),
                StreamMessage("/scan", 1, {"value": 2.0}),
                StreamMessage("/scan", 2, {"value": 3.0}),
                StreamMessage("/scan", 3, {"value": 4.0}),
            ]
        )
        mutated = FrozenSensorMutator(topic="/scan", start_index=2, length=2)(stream)
        assert mutated[0].data["value"] == 1.0
        assert mutated[1].data["value"] == 2.0
        assert mutated[2].data["value"] == 2.0  # frozen at last pre-window value
        assert mutated[3].data["value"] == 2.0


class TestCompose:
    def test_composes_multiple_mutators_in_order(self):
        stream = InMemoryMessageStream(_build_stream(5))
        pipeline = compose(
            DropFrameMutator(indices=[0]),
            NaNMutator(indices=[0], fields=["value"]),  # index 0 post-drop is orig id=1
        )
        result = pipeline(stream)
        assert len(result) == 4
        assert math.isnan(result[0].data["value"])

    def test_empty_composition_is_identity(self):
        stream = InMemoryMessageStream(_build_stream(3))
        result = compose()(stream)
        assert result == _build_stream(3)
