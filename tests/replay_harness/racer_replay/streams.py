"""Abstract message-stream interface for the L4 fault injectors.

A "message stream" here is just an ordered sequence of timestamped,
topic-tagged messages. This is deliberately the smallest common shape a
rosbag2 reader and an in-memory test fixture both satisfy:

- rosbag2 (``rosbag2_py.SequentialReader``) yields ``(topic, serialized_data,
  timestamp_ns)`` tuples in order.
- A synthetic test fixture is just a Python list.

``mutators.py`` is written entirely against :class:`StreamMessage` /
:class:`MessageStream` and never imports rosbag2 or rclpy, so every mutator
is unit-testable now, with no ROS installation, on
:class:`InMemoryMessageStream`. When real bags land (roadmap task 2.8), a
thin adapter implementing :class:`MessageStream` over
``rosbag2_py.SequentialReader`` is all that is needed to reuse every
mutator and the golden engine unchanged -- that adapter is intentionally
not written yet, per this task's scope (scaffold only, no real bags).
"""

from __future__ import annotations

from collections.abc import Iterable, Iterator
from dataclasses import dataclass, field
from typing import Any, Protocol, runtime_checkable


@dataclass(frozen=True)
class StreamMessage:
    """One message in a stream.

    ``timestamp_ns`` is the recording/bag timestamp (nanoseconds since
    epoch or since bag start -- mutators never assume which, they only
    compare/shift it), not necessarily the same as any timestamp carried
    inside ``data``. ``data`` is a plain mapping of field name -> value
    (float, int, str, bool, or None) so mutators can inspect and rewrite
    individual fields without depending on any ROS message type.
    """

    topic: str
    timestamp_ns: int
    data: dict[str, Any] = field(default_factory=dict)

    def with_updates(
        self,
        *,
        timestamp_ns: int | None = None,
        data: dict[str, Any] | None = None,
    ) -> StreamMessage:
        """Return a copy with ``timestamp_ns`` and/or ``data`` replaced."""
        return StreamMessage(
            topic=self.topic,
            timestamp_ns=self.timestamp_ns if timestamp_ns is None else timestamp_ns,
            data=self.data if data is None else data,
        )


@runtime_checkable
class MessageStream(Protocol):
    """Anything iterable over :class:`StreamMessage` in stream order."""

    def __iter__(self) -> Iterator[StreamMessage]: ...


class InMemoryMessageStream:
    """A :class:`MessageStream` backed by a plain in-memory sequence.

    This is the fixture type used by every mutator test in this package.
    It is also the type production code should build when replaying a
    small synthetic scenario without a real bag.
    """

    def __init__(self, messages: Iterable[StreamMessage]):
        self._messages: list[StreamMessage] = list(messages)

    def __iter__(self) -> Iterator[StreamMessage]:
        return iter(self._messages)

    def __len__(self) -> int:
        return len(self._messages)

    def __repr__(self) -> str:  # pragma: no cover - debugging aid only
        return f"InMemoryMessageStream({self._messages!r})"


def materialize(stream: MessageStream) -> list[StreamMessage]:
    """Drain a stream into a list. Convenience for tests and mutators."""
    return list(stream)
