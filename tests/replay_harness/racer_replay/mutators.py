"""Bag-mutation fault injectors (claude-docs/12-testing.md L4).

Each mutator is a small callable: ``Mutator = Callable[[MessageStream],
list[StreamMessage]]``. They compose left-to-right via :func:`compose` so a
test can build "drop two frames, then jump a timestamp, then freeze a
sensor" out of independently-tested pieces.

Every mutator here operates only on :class:`~racer_replay.streams.StreamMessage`
/ :class:`~racer_replay.streams.MessageStream`, per the fault list in
``12-testing.md`` L4: NaNs, timestamp jumps (forward and backward), dropped
frames, out-of-order messages, and frozen sensors. They are unit-tested (and
property-tested with hypothesis) against synthetic in-memory streams now;
the same functions are meant to run unchanged against a rosbag2-backed
:class:`~racer_replay.streams.MessageStream` once real bags exist.
"""

from __future__ import annotations

import math
from collections.abc import Callable, Iterable, Sequence
from typing import Any

from racer_replay.streams import MessageStream, StreamMessage, materialize

Mutator = Callable[[MessageStream], list[StreamMessage]]


def compose(*mutators: Mutator) -> Mutator:
    """Chain mutators left to right: ``compose(a, b)(s) == b(a(s))``."""

    def _composed(stream: MessageStream) -> list[StreamMessage]:
        current: MessageStream = stream
        result: list[StreamMessage] = materialize(current)
        for mutator in mutators:
            result = mutator(result)
        return result

    return _composed


class NaNMutator:
    """Replace numeric fields with ``float("nan")`` on selected messages.

    Contract (property-tested in ``tests/test_mutators.py``):
      - output length equals input length (no messages added or removed).
      - every output timestamp equals the corresponding input timestamp.
      - only the named ``fields`` are touched; every other field of every
        message, and every field of unselected messages, is unchanged.

    ``indices`` selects which messages (by position in the input) get
    corrupted; ``fields`` selects which numeric fields of ``data`` on those
    messages become NaN. A field missing from a message's ``data`` is
    silently skipped for that message (this mutator never adds fields).
    """

    def __init__(self, indices: Iterable[int], fields: Sequence[str]):
        self._indices = frozenset(indices)
        self._fields = tuple(fields)

    def __call__(self, stream: MessageStream) -> list[StreamMessage]:
        messages = materialize(stream)
        out: list[StreamMessage] = []
        for i, msg in enumerate(messages):
            if i not in self._indices:
                out.append(msg)
                continue
            new_data = dict(msg.data)
            for field_name in self._fields:
                if field_name in new_data:
                    new_data[field_name] = math.nan
            out.append(msg.with_updates(data=new_data))
        return out


class TimestampJumpMutator:
    """Shift timestamps forward or backward from a given point onward.

    Contract: message count, topics, and ``data`` payloads are unchanged;
    only ``timestamp_ns`` differs, and only for messages at or after
    ``from_index``. ``delta_ns`` may be negative (backward jump) or
    positive (forward jump) per the L4 fault list, which asks for both.
    """

    def __init__(self, from_index: int, delta_ns: int):
        self._from_index = from_index
        self._delta_ns = delta_ns

    def __call__(self, stream: MessageStream) -> list[StreamMessage]:
        messages = materialize(stream)
        out: list[StreamMessage] = []
        for i, msg in enumerate(messages):
            if i < self._from_index:
                out.append(msg)
            else:
                out.append(msg.with_updates(timestamp_ns=msg.timestamp_ns + self._delta_ns))
        return out


class DropFrameMutator:
    """Remove messages at the given indices (or matching a predicate).

    Contract: the output is a subsequence of the input -- every remaining
    message is untouched (same topic/timestamp/data) and relative order is
    preserved; nothing is inserted, reordered, or mutated.
    """

    def __init__(
        self,
        indices: Iterable[int] | None = None,
        *,
        predicate: Callable[[int, StreamMessage], bool] | None = None,
    ):
        if indices is None and predicate is None:
            raise ValueError("DropFrameMutator needs either indices or a predicate")
        self._indices = frozenset(indices) if indices is not None else None
        self._predicate = predicate

    def _should_drop(self, i: int, msg: StreamMessage) -> bool:
        if self._indices is not None:
            return i in self._indices
        assert self._predicate is not None
        return self._predicate(i, msg)

    def __call__(self, stream: MessageStream) -> list[StreamMessage]:
        messages = materialize(stream)
        return [msg for i, msg in enumerate(messages) if not self._should_drop(i, msg)]


class OutOfOrderMutator:
    """Swap pairs of messages so the stream is no longer timestamp-sorted.

    Contract: the output is a permutation of the input (same multiset of
    messages, by identity) -- nothing is added, removed, or mutated, only
    reordered. ``swaps`` is a list of ``(i, j)`` index pairs to swap,
    applied in order against the working list.
    """

    def __init__(self, swaps: Sequence[tuple[int, int]]):
        self._swaps = list(swaps)

    def __call__(self, stream: MessageStream) -> list[StreamMessage]:
        messages = materialize(stream)
        for i, j in self._swaps:
            messages[i], messages[j] = messages[j], messages[i]
        return messages


class FrozenSensorMutator:
    """Freeze one topic's payload for a run of messages (stuck sensor).

    From ``start_index``, for the next ``length`` messages on ``topic``,
    ``data`` is replaced with a copy of the payload from the last message
    on that topic strictly before the freeze window (or the first
    in-window value if there is none before it, matching "the sensor got
    stuck at whatever it last read"). Messages on other topics, and
    timestamps on every message, are untouched.

    Contract: message count and every timestamp are unchanged; every
    message's topic is unchanged; within the frozen window, all ``data``
    payloads on ``topic`` are identical (deep-equal) to each other.
    """

    def __init__(self, topic: str, start_index: int, length: int):
        self._topic = topic
        self._start_index = start_index
        self._length = length

    def __call__(self, stream: MessageStream) -> list[StreamMessage]:
        messages = materialize(stream)
        end_index = self._start_index + self._length

        frozen_value: dict[str, Any] | None = None
        for i in range(self._start_index - 1, -1, -1):
            if messages[i].topic == self._topic:
                frozen_value = dict(messages[i].data)
                break

        out: list[StreamMessage] = []
        for i, msg in enumerate(messages):
            if msg.topic != self._topic or not (self._start_index <= i < end_index):
                out.append(msg)
                continue
            if frozen_value is None:
                frozen_value = dict(msg.data)
            out.append(msg.with_updates(data=dict(frozen_value)))
        return out
