"""ROS-side facts, read through the foxglove_bridge websocket the stack already runs.

Why the websocket and not a status node in the stack: the stack is the thing that drives the
car, and every node added to it is one more thing that can break it. foxglove_bridge is
already there (port 8765, `viz` defaults on), the browser UI uses the same socket, and a
client of it can only read. So the agent is a second subscriber, no launch file changes, no
rebuild, and it keeps working whichever launch started the stack. The cost is a hand-written
decoder for the two std_msgs types the checks need (Float32 and String, both trivial CDR);
everything else is only counted, which gives message RATES for /scan, /pose and /odom/wheel and
the time each was LAST seen. /tf is read only as far as its frame ids, so the agent can tell
that AMCL is broadcasting map -> odom (it republishes the last transform on every scan, also
while the car stands still and /pose is silent).

The agent never advertises or publishes anything over this socket.
"""

from __future__ import annotations

import asyncio
import json
import logging
import math
import struct
import threading
import time
from collections import deque
from collections.abc import Callable
from dataclasses import dataclass, field

import websockets

log = logging.getLogger(__name__)

SUBPROTOCOL = "foxglove.websocket.v1"
#: foxglove_bridge 3.x only accepts this name (same wire format). Offer both, newest first.
SDK_SUBPROTOCOL = "foxglove.sdk.v1"
OP_MESSAGE_DATA = 0x01

#: Topics whose values the checks need. Decoded.
VALUE_TOPICS = {
    "/telemetry/vesc/voltage_v": "voltage_v",
    "/telemetry/vesc/current_input_a": "current_input_a",
    "/telemetry/vesc/temp_fet_degc": "temp_fet_degc",
    "/telemetry/vesc/temp_motor_degc": "temp_motor_degc",
    "/telemetry/vesc/fault": "fault",
}
#: Topics that are only counted, for a rate. Not decoded.
RATE_TOPICS = ("/scan", "/pose", "/odom/wheel", "/perception/objects")
#: Topics whose presence in the bridge's channel list is itself a check.
PRESENCE_TOPICS = ("/drive", "/drive_raw", "/safety/events", "/map")

#: Topics read only for the parent -> child frame ids of their transforms.
TF_TOPICS = ("/tf",)

RATE_WINDOW_S = 4.0


def tf_key(parent: str, child: str) -> str:
    """The `last_age` key of one transform seen on /tf."""
    return f"tf:{parent.lstrip('/')}->{child.lstrip('/')}"


def decode_float32(payload: bytes) -> float | None:
    """CDR std_msgs/Float32: 4 byte encapsulation header, then one float32."""
    if len(payload) < 8:
        return None
    fmt = "<f" if payload[1] == 1 else ">f"
    return struct.unpack_from(fmt, payload, 4)[0]


def decode_string(payload: bytes) -> str | None:
    """CDR std_msgs/String: header, uint32 length (including the NUL), characters."""
    if len(payload) < 8:
        return None
    fmt = "<I" if payload[1] == 1 else ">I"
    (length,) = struct.unpack_from(fmt, payload, 4)
    raw = payload[8 : 8 + length]
    if len(raw) < length:
        return None
    return raw.rstrip(b"\x00").decode("utf-8", errors="replace")


def decode_tf_frames(payload: bytes) -> list[tuple[str, str]] | None:
    """CDR tf2_msgs/TFMessage: the (frame_id, child_frame_id) of every transform.

    Layout per transform: int32 sec, uint32 nanosec, string frame_id, string child_frame_id,
    then 7 float64 (translation, rotation) aligned to 8. Alignment counts from after the 4 byte
    encapsulation header. None for a truncated or malformed message.
    """
    if len(payload) < 8:
        return None
    end = "<" if payload[1] == 1 else ">"
    body = payload[4:]
    pos = 0

    def align(n: int) -> None:
        nonlocal pos
        pos += (-pos) % n

    def u32() -> int:
        nonlocal pos
        align(4)
        if pos + 4 > len(body):
            raise ValueError("truncated")
        (v,) = struct.unpack_from(end + "I", body, pos)
        pos += 4
        return v

    def text() -> str:
        nonlocal pos
        n = u32()
        if n == 0 or pos + n > len(body):
            raise ValueError("bad string")
        raw = body[pos : pos + n]
        pos += n
        return raw.rstrip(b"\x00").decode("utf-8", errors="replace")

    try:
        count = u32()
        frames: list[tuple[str, str]] = []
        for _ in range(count):
            u32()  # stamp.sec
            u32()  # stamp.nanosec
            parent = text()
            child = text()
            align(8)
            pos += 7 * 8
            if pos > len(body):
                raise ValueError("truncated")
            frames.append((parent, child))
    except ValueError:
        return None
    return frames


@dataclass(frozen=True)
class RosSnapshot:
    connected: bool
    topics: frozenset[str]
    rates_hz: dict[str, float]
    values: dict[str, float | str]
    value_age_s: dict[str, float]
    error: str = ""
    #: seconds since the last message on a counted topic, or the last `tf_key` transform, in
    #: this bridge connection. A key that is missing was never seen.
    last_age_s: dict[str, float] = field(default_factory=dict)

    def rate(self, topic: str) -> float:
        return self.rates_hz.get(topic, 0.0)

    def last_age(self, key: str) -> float | None:
        return self.last_age_s.get(key)

    def value(self, key: str) -> float | str | None:
        return self.values.get(key)


class RosState:
    """Thread-safe latest-values store, written by the websocket client, read by the checks."""

    def __init__(self, clock: Callable[[], float] = time.monotonic) -> None:
        self._clock = clock
        self._lock = threading.Lock()
        self._connected = False
        self._topics: set[str] = set()
        self._stamps: dict[str, deque[float]] = {t: deque() for t in RATE_TOPICS}
        self._values: dict[str, float | str] = {}
        self._value_t: dict[str, float] = {}
        self._last_t: dict[str, float] = {}
        self._error = ""

    def set_connected(self, connected: bool, error: str = "") -> None:
        with self._lock:
            self._connected = connected
            self._error = error
            if not connected:
                self._topics.clear()
                self._last_t.clear()
                for dq in self._stamps.values():
                    dq.clear()

    def set_topics(self, topics: set[str]) -> None:
        with self._lock:
            self._topics = set(topics)

    def add_topics(self, topics: set[str]) -> None:
        with self._lock:
            self._topics |= topics

    def remove_topics(self, topics: set[str]) -> None:
        with self._lock:
            self._topics -= topics

    def count(self, topic: str) -> None:
        now = self._clock()
        with self._lock:
            dq = self._stamps.setdefault(topic, deque())
            dq.append(now)
            self._trim(dq, now)
            self._last_t[topic] = now

    def note_tf(self, parent: str, child: str) -> None:
        now = self._clock()
        with self._lock:
            self._last_t[tf_key(parent, child)] = now

    def set_value(self, key: str, value: float | str) -> None:
        with self._lock:
            self._values[key] = value
            self._value_t[key] = self._clock()

    @staticmethod
    def _trim(dq: deque[float], now: float) -> None:
        while dq and now - dq[0] > RATE_WINDOW_S:
            dq.popleft()

    def snapshot(self) -> RosSnapshot:
        now = self._clock()
        with self._lock:
            rates: dict[str, float] = {}
            for topic, dq in self._stamps.items():
                self._trim(dq, now)
                rates[topic] = (len(dq) / RATE_WINDOW_S) if self._connected else 0.0
            return RosSnapshot(
                connected=self._connected,
                topics=frozenset(self._topics),
                rates_hz=rates,
                values=dict(self._values),
                value_age_s={k: now - t for k, t in self._value_t.items()},
                error=self._error,
                last_age_s={k: now - t for k, t in self._last_t.items()},
            )


@dataclass
class _Channel:
    id: int
    topic: str
    schema_name: str = ""


@dataclass
class FoxgloveClient:
    """Websocket client loop. `run()` reconnects forever until `stop()`."""

    url: str
    state: RosState
    reconnect_s: float = 3.0
    _stop: asyncio.Event | None = field(default=None, init=False, repr=False)
    _loop: asyncio.AbstractEventLoop | None = field(default=None, init=False, repr=False)
    _thread: threading.Thread | None = field(default=None, init=False, repr=False)

    # -- thread wrapper used by the service -----------------------------------------------
    def start_thread(self) -> None:
        if self._thread is not None:
            return
        self._thread = threading.Thread(
            target=self._thread_main, name="foxglove-client", daemon=True
        )
        self._thread.start()

    def stop_thread(self, timeout: float = 3.0) -> None:
        if self._thread is None:
            return
        loop, stop = self._loop, self._stop
        if loop is not None and stop is not None:
            try:
                loop.call_soon_threadsafe(stop.set)
            except RuntimeError:  # the loop already finished
                pass
        self._thread.join(timeout)
        self._thread = None

    def _thread_main(self) -> None:
        asyncio.run(self.run())

    # -- the loop -------------------------------------------------------------------------
    async def run(self) -> None:
        self._loop = asyncio.get_running_loop()
        self._stop = asyncio.Event()
        while not self._stop.is_set():
            try:
                await self._session()
            except asyncio.CancelledError:
                raise
            except Exception as exc:  # noqa: BLE001 - any failure means reconnect
                self.state.set_connected(False, f"{type(exc).__name__}: {exc}")
            else:
                self.state.set_connected(False, "closed")
            try:
                await asyncio.wait_for(self._stop.wait(), self.reconnect_s)
            except asyncio.TimeoutError:
                pass

    async def _session(self) -> None:
        assert self._stop is not None
        async with websockets.connect(
            self.url,
            subprotocols=[
                websockets.Subprotocol(SDK_SUBPROTOCOL),
                websockets.Subprotocol(SUBPROTOCOL),
            ],
            open_timeout=5,
            max_size=None,
            ping_interval=10,
        ) as ws:
            self.state.set_connected(True)
            channels: dict[int, _Channel] = {}
            sub_ids: dict[int, _Channel] = {}  # subscription id -> channel
            next_sub = 1
            recv_task: asyncio.Task | None = None
            stop_task = asyncio.ensure_future(self._stop.wait())
            try:
                while True:
                    recv_task = asyncio.ensure_future(ws.recv())
                    done, _ = await asyncio.wait(
                        {recv_task, stop_task}, return_when=asyncio.FIRST_COMPLETED
                    )
                    if stop_task in done:
                        recv_task.cancel()
                        return
                    message = recv_task.result()
                    if isinstance(message, bytes):
                        self.handle_binary(message, sub_ids)
                        continue
                    wanted = self.handle_text(message, channels)
                    for channel in wanted:
                        sub = next_sub
                        next_sub += 1
                        sub_ids[sub] = channel
                        await ws.send(
                            json.dumps(
                                {
                                    "op": "subscribe",
                                    "subscriptions": [{"id": sub, "channelId": channel.id}],
                                }
                            )
                        )
            finally:
                stop_task.cancel()
                if recv_task is not None and not recv_task.done():
                    recv_task.cancel()

    # -- message handling (pure enough to unit test without a socket) ----------------------
    def handle_text(self, text: str, channels: dict[int, _Channel]) -> list[_Channel]:
        """Process one JSON server message; return the channels to subscribe to."""
        try:
            msg = json.loads(text)
        except json.JSONDecodeError:
            return []
        op = msg.get("op")
        wanted: list[_Channel] = []
        if op == "advertise":
            for ch in msg.get("channels", []):
                channel = _Channel(int(ch["id"]), str(ch["topic"]), str(ch.get("schemaName", "")))
                channels[channel.id] = channel
                self.state.add_topics({channel.topic})
                if (
                    channel.topic in VALUE_TOPICS
                    or channel.topic in RATE_TOPICS
                    or channel.topic in TF_TOPICS
                ):
                    wanted.append(channel)
        elif op == "unadvertise":
            for cid in msg.get("channelIds", []):
                channel = channels.pop(int(cid), None)
                if channel is not None:
                    self.state.remove_topics({channel.topic})
        return wanted

    def handle_binary(self, data: bytes, sub_ids: dict[int, _Channel]) -> None:
        if len(data) < 13 or data[0] != OP_MESSAGE_DATA:
            return
        (sub_id,) = struct.unpack_from("<I", data, 1)
        channel = sub_ids.get(sub_id)
        if channel is None:
            return
        payload = data[13:]
        topic = channel.topic
        if topic in RATE_TOPICS:
            self.state.count(topic)
            return
        if topic in TF_TOPICS:
            for parent, child in decode_tf_frames(payload) or ():
                self.state.note_tf(parent, child)
            return
        key = VALUE_TOPICS.get(topic)
        if key is None:
            return
        if key == "fault":
            text = decode_string(payload)
            if text is not None:
                self.state.set_value(key, text)
        else:
            number = decode_float32(payload)
            if number is not None and math.isfinite(number):
                self.state.set_value(key, number)
