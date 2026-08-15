"""L1 unit tests for racer_replay.streams."""

from __future__ import annotations

from racer_replay.streams import InMemoryMessageStream, MessageStream, StreamMessage, materialize


class TestStreamMessage:
    def test_with_updates_replaces_only_given_fields(self):
        msg = StreamMessage(topic="/scan", timestamp_ns=100, data={"a": 1})
        updated = msg.with_updates(timestamp_ns=200)
        assert updated.timestamp_ns == 200
        assert updated.topic == "/scan"
        assert updated.data == {"a": 1}

    def test_with_updates_replaces_data(self):
        msg = StreamMessage(topic="/scan", timestamp_ns=100, data={"a": 1})
        updated = msg.with_updates(data={"b": 2})
        assert updated.data == {"b": 2}
        assert updated.timestamp_ns == 100

    def test_with_no_updates_is_equal_copy(self):
        msg = StreamMessage(topic="/scan", timestamp_ns=100, data={"a": 1})
        assert msg.with_updates() == msg

    def test_is_frozen(self):
        msg = StreamMessage(topic="/scan", timestamp_ns=100, data={"a": 1})
        try:
            msg.timestamp_ns = 5  # type: ignore[misc]
        except AttributeError:
            pass
        else:
            raise AssertionError("StreamMessage should be immutable")


class TestInMemoryMessageStream:
    def test_iterates_in_order(self):
        messages = [StreamMessage("/scan", i, {}) for i in range(3)]
        stream = InMemoryMessageStream(messages)
        assert list(stream) == messages

    def test_is_a_message_stream(self):
        stream = InMemoryMessageStream([])
        assert isinstance(stream, MessageStream)

    def test_len(self):
        stream = InMemoryMessageStream(
            [StreamMessage("/scan", 0, {}), StreamMessage("/scan", 1, {})]
        )
        assert len(stream) == 2

    def test_can_be_iterated_more_than_once(self):
        stream = InMemoryMessageStream([StreamMessage("/scan", 0, {})])
        assert list(stream) == list(stream)


def test_materialize_drains_stream_to_list():
    messages = [StreamMessage("/scan", 0, {}), StreamMessage("/odom", 1, {})]
    result = materialize(InMemoryMessageStream(messages))
    assert result == messages
    assert isinstance(result, list)
