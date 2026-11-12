from __future__ import annotations

import asyncio
import json
import struct

import pytest
import websockets
from racer_agent import rosstate as rs

LE = b"\x00\x01\x00\x00"  # CDR little-endian encapsulation header


def f32(value: float) -> bytes:
    return LE + struct.pack("<f", value)


def string(text: str) -> bytes:
    raw = text.encode() + b"\x00"
    return LE + struct.pack("<I", len(raw)) + raw


def frame(sub_id: int, payload: bytes) -> bytes:
    return struct.pack("<BIQ", 1, sub_id, 0) + payload


def test_decode_float32_both_endiannesses_and_garbage():
    assert rs.decode_float32(f32(12.5)) == 12.5
    assert rs.decode_float32(b"\x00\x00\x00\x00" + struct.pack(">f", -3.25)) == -3.25
    assert rs.decode_float32(b"\x00\x01") is None


def test_decode_string():
    assert rs.decode_string(string("FAULT_CODE_NONE")) == "FAULT_CODE_NONE"
    assert rs.decode_string(string("")) == ""
    big = b"\x00\x00\x00\x00" + struct.pack(">I", 3) + b"ab\x00"
    assert rs.decode_string(big) == "ab"
    assert rs.decode_string(LE) is None
    assert rs.decode_string(LE + struct.pack("<I", 50) + b"short") is None


def advertise(*topics: str) -> str:
    return json.dumps(
        {
            "op": "advertise",
            "channels": [
                {"id": i + 1, "topic": t, "encoding": "cdr", "schemaName": "x/msg/X"}
                for i, t in enumerate(topics)
            ],
        }
    )


def test_advertise_collects_topics_and_picks_the_ones_to_subscribe_to():
    state = rs.RosState()
    client = rs.FoxgloveClient("ws://x", state)
    channels: dict = {}
    wanted = client.handle_text(
        advertise("/drive", "/scan", "/telemetry/vesc/voltage_v", "/camera/usb/image_raw"), channels
    )
    assert [c.topic for c in wanted] == ["/scan", "/telemetry/vesc/voltage_v"]
    state.set_connected(True)
    assert state.snapshot().topics == {
        "/drive",
        "/scan",
        "/telemetry/vesc/voltage_v",
        "/camera/usb/image_raw",
    }


def test_never_subscribes_to_cameras_or_the_command_path():
    state = rs.RosState()
    client = rs.FoxgloveClient("ws://x", state)
    wanted = client.handle_text(
        advertise("/camera/csi0/image_raw/compressed", "/drive", "/drive_raw", "/map"), {}
    )
    assert wanted == []


def test_unadvertise_removes_the_topic():
    state = rs.RosState()
    client = rs.FoxgloveClient("ws://x", state)
    channels: dict = {}
    client.handle_text(advertise("/drive", "/map"), channels)
    client.handle_text(json.dumps({"op": "unadvertise", "channelIds": [1, 99]}), channels)
    state.set_connected(True)
    assert state.snapshot().topics == {"/map"}


def test_bad_json_and_other_ops_are_ignored():
    client = rs.FoxgloveClient("ws://x", rs.RosState())
    assert client.handle_text("not json", {}) == []
    assert client.handle_text(json.dumps({"op": "serverInfo", "name": "x"}), {}) == []


def make_client_with_subs():
    state = rs.RosState()
    client = rs.FoxgloveClient("ws://x", state)
    channels: dict = {}
    topics = [
        "/telemetry/vesc/voltage_v",
        "/telemetry/vesc/fault",
        "/scan",
        "/pose",
        "/telemetry/vesc/temp_fet_degc",
    ]
    wanted = client.handle_text(advertise(*topics), channels)
    subs = {i + 10: ch for i, ch in enumerate(wanted)}
    return state, client, subs, {ch.topic: sid for sid, ch in subs.items()}


def test_binary_messages_update_values_and_rates():
    state, client, subs, sid = make_client_with_subs()
    state.set_connected(True)
    client.handle_binary(frame(sid["/telemetry/vesc/voltage_v"], f32(11.9)), subs)
    client.handle_binary(frame(sid["/telemetry/vesc/fault"], string("NONE")), subs)
    client.handle_binary(frame(sid["/telemetry/vesc/temp_fet_degc"], f32(41.0)), subs)
    for _ in range(8):
        client.handle_binary(frame(sid["/scan"], b"\x00" * 40), subs)
    snap = state.snapshot()
    assert snap.value("voltage_v") == pytest.approx(11.9, abs=1e-5)
    assert snap.value("fault") == "NONE" and snap.value("temp_fet_degc") == 41.0
    assert snap.rate("/scan") == pytest.approx(8 / rs.RATE_WINDOW_S)
    assert snap.rate("/pose") == 0.0


def test_bad_binary_frames_are_ignored():
    state, client, subs, sid = make_client_with_subs()
    client.handle_binary(b"\x01", subs)
    client.handle_binary(frame(999, f32(1.0)), subs)  # unknown subscription
    client.handle_binary(b"\x02" + b"\x00" * 20, subs)  # not MessageData
    client.handle_binary(frame(sid["/telemetry/vesc/voltage_v"], f32(float("nan"))), subs)
    client.handle_binary(frame(sid["/telemetry/vesc/voltage_v"], b"\x00\x01"), subs)
    client.handle_binary(frame(sid["/telemetry/vesc/fault"], LE), subs)
    assert state.snapshot().values == {}


def test_rates_age_out_and_drop_to_zero_when_disconnected():
    now = [100.0]
    state = rs.RosState(clock=lambda: now[0])
    state.set_connected(True)
    for _ in range(40):
        state.count("/scan")
    assert state.snapshot().rate("/scan") == pytest.approx(10.0)
    now[0] += rs.RATE_WINDOW_S + 1
    assert state.snapshot().rate("/scan") == 0.0
    state.count("/scan")
    state.set_connected(False, "boom")
    snap = state.snapshot()
    assert snap.rate("/scan") == 0.0 and snap.error == "boom" and not snap.connected


def test_value_age_grows():
    now = [0.0]
    state = rs.RosState(clock=lambda: now[0])
    state.set_value("voltage_v", 12.0)
    now[0] = 2.5
    assert state.snapshot().value_age_s["voltage_v"] == 2.5


def test_add_and_remove_topics_directly():
    state = rs.RosState()
    state.set_topics({"/a"})
    state.add_topics({"/b"})
    state.remove_topics({"/a"})
    state.set_connected(True)
    assert state.snapshot().topics == {"/b"}
    state.set_connected(False)
    assert state.snapshot().topics == frozenset()


# -- a real websocket round trip -------------------------------------------------------------


def test_client_talks_to_a_fake_foxglove_bridge_end_to_end():
    received_ops: list[dict] = []

    async def scenario() -> rs.RosSnapshot:
        async def bridge(ws):
            await ws.send(json.dumps({"op": "serverInfo", "name": "fake", "capabilities": []}))
            await ws.send(advertise("/telemetry/vesc/voltage_v", "/scan", "/drive"))
            sub_ids: dict[int, int] = {}
            while len(sub_ids) < 2:
                msg = json.loads(await ws.recv())
                received_ops.append(msg)
                for sub in msg["subscriptions"]:
                    sub_ids[sub["channelId"]] = sub["id"]
            await ws.send(frame(sub_ids[1], f32(12.34)))
            for _ in range(20):
                await ws.send(frame(sub_ids[2], b"\x00" * 16))
            await asyncio.sleep(0.5)

        async with websockets.serve(
            bridge, "127.0.0.1", 0, subprotocols=[websockets.Subprotocol(rs.SUBPROTOCOL)]
        ) as server:
            port = server.sockets[0].getsockname()[1]
            state = rs.RosState()
            client = rs.FoxgloveClient(f"ws://127.0.0.1:{port}", state, reconnect_s=0.1)
            task = asyncio.create_task(client.run())
            snap = state.snapshot()
            for _ in range(100):
                await asyncio.sleep(0.05)
                snap = state.snapshot()
                if snap.value("voltage_v") and snap.rate("/scan") > 0:
                    break
            assert client._stop is not None
            client._stop.set()
            await asyncio.wait_for(task, 5)
            return snap

    snap = asyncio.run(scenario())
    assert snap.connected is True
    assert snap.value("voltage_v") == pytest.approx(12.34, abs=1e-4)
    assert snap.rate("/scan") > 0
    assert "/drive" in snap.topics
    # it subscribed to exactly the two topics it needs, and sent nothing else
    assert [m["op"] for m in received_ops] == ["subscribe", "subscribe"]


def test_client_reconnects_after_the_bridge_goes_away():
    async def scenario() -> list[bool]:
        seen: list[bool] = []

        async def bridge(ws):
            await ws.send(advertise("/drive"))
            await asyncio.sleep(0.2)

        async with websockets.serve(
            bridge, "127.0.0.1", 0, subprotocols=[websockets.Subprotocol(rs.SUBPROTOCOL)]
        ) as server:
            port = server.sockets[0].getsockname()[1]
            state = rs.RosState()
            client = rs.FoxgloveClient(f"ws://127.0.0.1:{port}", state, reconnect_s=0.05)
            task = asyncio.create_task(client.run())
            for _ in range(80):
                await asyncio.sleep(0.05)
                seen.append(state.snapshot().connected)
                if seen.count(True) >= 2 and False in seen[seen.index(True) :]:
                    break
            assert client._stop is not None
            client._stop.set()
            await asyncio.wait_for(task, 5)
        return seen

    seen = asyncio.run(scenario())
    first = seen.index(True)
    assert False in seen[first:], "the disconnect was not noticed"
    assert seen[first:].count(True) >= 2


def test_client_reports_an_error_when_nothing_is_listening():
    async def scenario() -> str:
        state = rs.RosState()
        client = rs.FoxgloveClient("ws://127.0.0.1:1", state, reconnect_s=0.05)
        task = asyncio.create_task(client.run())
        error = ""
        for _ in range(60):
            await asyncio.sleep(0.05)
            error = state.snapshot().error
            if error:
                break
        assert client._stop is not None
        client._stop.set()
        await asyncio.wait_for(task, 5)
        return error

    assert asyncio.run(scenario())


def test_thread_wrapper_starts_and_stops_cleanly():
    client = rs.FoxgloveClient("ws://127.0.0.1:1", rs.RosState(), reconnect_s=0.05)
    client.start_thread()
    client.start_thread()  # second call is a no-op
    import time

    time.sleep(0.2)
    client.stop_thread()
    assert client._thread is None
    client.stop_thread()  # and so is a second stop


def test_client_connects_to_a_bridge_that_only_speaks_the_sdk_subprotocol():
    # foxglove_bridge 3.x rejects foxglove.websocket.v1 and only accepts foxglove.sdk.v1.
    async def scenario() -> bool:
        async def bridge(ws):
            await ws.send(advertise("/drive"))
            await asyncio.sleep(1.0)

        async with websockets.serve(
            bridge, "127.0.0.1", 0, subprotocols=[websockets.Subprotocol(rs.SDK_SUBPROTOCOL)]
        ) as server:
            port = server.sockets[0].getsockname()[1]
            state = rs.RosState()
            client = rs.FoxgloveClient(f"ws://127.0.0.1:{port}", state, reconnect_s=0.05)
            task = asyncio.create_task(client.run())
            connected = False
            for _ in range(40):
                await asyncio.sleep(0.05)
                if state.snapshot().connected:
                    connected = True
                    break
            assert client._stop is not None
            client._stop.set()
            await asyncio.wait_for(task, 5)
        return connected

    assert asyncio.run(scenario())
