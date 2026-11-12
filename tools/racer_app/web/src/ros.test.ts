import { parse } from "@foxglove/rosmsg";
import { MessageWriter } from "@foxglove/rosmsg2-serialization";
import { afterEach, beforeEach, describe, expect, it, vi } from "vitest";
import { RosLink, type LinkStatus } from "./ros";

class FakeSocket {
  binaryType = "blob";
  protocol = "foxglove.websocket.v1";
  onerror: ((e: unknown) => void) | null | undefined;
  onopen: ((e: unknown) => void) | null | undefined;
  onclose: ((e: unknown) => void) | null | undefined;
  onmessage: ((e: unknown) => void) | null | undefined;
  sent: unknown[] = [];
  closed = false;
  constructor(
    public url: string,
    public protocols: string[],
  ) {}
  send(data: unknown) {
    this.sent.push(data);
  }
  close() {
    this.closed = true;
  }
  // test helpers
  open() {
    this.onopen?.({});
  }
  serverJson(obj: unknown) {
    this.onmessage?.({ data: JSON.stringify(obj) });
  }
  serverMessage(subId: number, payload: Uint8Array) {
    const buf = new Uint8Array(13 + payload.length);
    const view = new DataView(buf.buffer);
    view.setUint8(0, 1);
    view.setUint32(1, subId, true);
    view.setBigUint64(5, 0n, true);
    buf.set(payload, 13);
    this.onmessage?.({ data: buf.buffer });
  }
  drop() {
    this.onclose?.({});
  }
  ops(): { op: string; [k: string]: unknown }[] {
    return this.sent.filter((s) => typeof s === "string").map((s) => JSON.parse(s as string));
  }
}

function cdr(schema: string, value: unknown): Uint8Array {
  return new MessageWriter(parse(schema, { ros2: true })).writeMessage(value);
}

const FLOAT_SCHEMA = "float32 data";
const STRING_SCHEMA = "string data";
const channel = (id: number, topic: string, schemaName: string, schema: string) => ({
  id,
  topic,
  encoding: "cdr",
  schemaName,
  schema,
  schemaEncoding: "ros2msg",
});

let sockets: FakeSocket[];
const factory = (url: string, protocols: string[]) => {
  const s = new FakeSocket(url, protocols);
  sockets.push(s);
  return s as never;
};

beforeEach(() => {
  sockets = [];
});
afterEach(() => {
  vi.useRealTimers();
});

describe("RosLink", () => {
  it("connects with the foxglove subprotocol and reports status", () => {
    const link = new RosLink("ws://car:8765", factory);
    const seen: LinkStatus[] = [];
    link.onStatus((s) => seen.push(s));
    link.connect();
    expect(sockets[0].url).toBe("ws://car:8765");
    expect(sockets[0].protocols).toEqual(["foxglove.sdk.v1", "foxglove.websocket.v1"]);
    sockets[0].open();
    expect(seen.at(-1)).toEqual({ connected: true, topics: [] });
    sockets[0].serverJson({ op: "advertise", channels: [channel(1, "/drive", "x/msg/X", "float32 data")] });
    expect(seen.at(-1)?.topics).toEqual(["/drive"]);
  });

  it("decodes real CDR messages for a topic subscribed before the channel appeared", () => {
    const link = new RosLink("ws://car", factory);
    const volts: number[] = [];
    link.subscribe<{ data: number }>("/telemetry/vesc/voltage_v", (m) => volts.push(m.data));
    link.connect();
    const s = sockets[0];
    s.open();
    expect(s.ops()).toEqual([]); // nothing to subscribe to yet
    s.serverJson({
      op: "advertise",
      channels: [channel(7, "/telemetry/vesc/voltage_v", "std_msgs/msg/Float32", FLOAT_SCHEMA)],
    });
    expect(s.ops()).toEqual([{ op: "subscribe", subscriptions: [{ id: 0, channelId: 7 }] }]);
    s.serverMessage(0, cdr(FLOAT_SCHEMA, { data: 11.5 }));
    expect(volts).toEqual([11.5]);
  });

  it("decodes strings and nested messages", () => {
    const link = new RosLink("ws://car", factory);
    const got: string[] = [];
    link.subscribe<{ data: string }>("/telemetry/vesc/fault", (m) => got.push(m.data));
    link.connect();
    const s = sockets[0];
    s.open();
    s.serverJson({
      op: "advertise",
      channels: [channel(2, "/telemetry/vesc/fault", "std_msgs/msg/String", STRING_SCHEMA)],
    });
    s.serverMessage(0, cdr(STRING_SCHEMA, { data: "FAULT_CODE_NONE" }));
    expect(got).toEqual(["FAULT_CODE_NONE"]);
  });

  it("ignores garbage payloads and messages for unknown subscriptions", () => {
    const link = new RosLink("ws://car", factory);
    const got: number[] = [];
    link.subscribe<{ data: number }>("/v", (m) => got.push(m.data));
    link.connect();
    const s = sockets[0];
    s.open();
    s.serverJson({ op: "advertise", channels: [channel(1, "/v", "std_msgs/msg/Float32", FLOAT_SCHEMA)] });
    s.serverMessage(0, new Uint8Array([1, 2])); // too short to decode
    s.serverMessage(99, cdr(FLOAT_SCHEMA, { data: 1 })); // not ours
    expect(got).toEqual([]);
  });

  it("ignores channels that are not CDR or whose schema does not parse", () => {
    const link = new RosLink("ws://car", factory);
    const got: unknown[] = [];
    link.subscribe("/j", (m) => got.push(m));
    link.subscribe("/bad", (m) => got.push(m));
    link.connect();
    const s = sockets[0];
    s.open();
    s.serverJson({
      op: "advertise",
      channels: [
        { id: 1, topic: "/j", encoding: "json", schemaName: "x", schema: "{}" },
        channel(2, "/bad", "x/msg/X", "this is not ::: a schema ((("),
      ],
    });
    s.serverMessage(0, new Uint8Array([0, 1, 0, 0, 0, 0, 0, 0]));
    s.serverMessage(1, new Uint8Array([0, 1, 0, 0, 0, 0, 0, 0]));
    expect(got).toEqual([]);
  });

  it("unsubscribing stops delivery and tells the bridge", () => {
    const link = new RosLink("ws://car", factory);
    const got: number[] = [];
    const off = link.subscribe<{ data: number }>("/v", (m) => got.push(m.data));
    link.connect();
    const s = sockets[0];
    s.open();
    s.serverJson({ op: "advertise", channels: [channel(1, "/v", "std_msgs/msg/Float32", FLOAT_SCHEMA)] });
    off();
    expect(s.ops().at(-1)).toEqual({ op: "unsubscribe", subscriptionIds: [0] });
    s.serverMessage(0, cdr(FLOAT_SCHEMA, { data: 1 }));
    expect(got).toEqual([]);
  });

  it("unadvertise drops the topic and a re-advertise resubscribes", () => {
    const link = new RosLink("ws://car", factory);
    const got: number[] = [];
    link.subscribe<{ data: number }>("/v", (m) => got.push(m.data));
    link.connect();
    const s = sockets[0];
    s.open();
    const ch = channel(1, "/v", "std_msgs/msg/Float32", FLOAT_SCHEMA);
    s.serverJson({ op: "advertise", channels: [ch] });
    s.serverJson({ op: "unadvertise", channelIds: [1] });
    expect(link.status().topics).toEqual([]);
    s.serverJson({ op: "advertise", channels: [{ ...ch, id: 5 }] });
    const subs = s.ops().filter((o) => o.op === "subscribe");
    expect(subs.length).toBe(2);
    s.serverMessage(1, cdr(FLOAT_SCHEMA, { data: 4 }));
    expect(got).toEqual([4]);
  });

  it("reconnects after the socket drops and resubscribes", () => {
    vi.useFakeTimers();
    const link = new RosLink("ws://car", factory, 1000);
    const got: number[] = [];
    link.subscribe<{ data: number }>("/v", (m) => got.push(m.data));
    const statuses: boolean[] = [];
    link.onStatus((st) => statuses.push(st.connected));
    link.connect();
    sockets[0].open();
    sockets[0].serverJson({ op: "advertise", channels: [channel(1, "/v", "std_msgs/msg/Float32", FLOAT_SCHEMA)] });
    sockets[0].drop();
    expect(link.status()).toEqual({ connected: false, topics: [] });
    expect(sockets.length).toBe(1);
    vi.advanceTimersByTime(1000);
    expect(sockets.length).toBe(2);
    sockets[1].open();
    sockets[1].serverJson({ op: "advertise", channels: [channel(1, "/v", "std_msgs/msg/Float32", FLOAT_SCHEMA)] });
    sockets[1].serverMessage(0, cdr(FLOAT_SCHEMA, { data: 2 }));
    expect(got).toEqual([2]);
    expect(statuses).toContain(false);
    expect(statuses.at(-1)).toBe(true);
  });

  it("does not reconnect after close()", () => {
    vi.useFakeTimers();
    const link = new RosLink("ws://car", factory, 1000);
    link.connect();
    sockets[0].open();
    link.close();
    expect(sockets[0].closed).toBe(true);
    sockets[0].drop();
    vi.advanceTimersByTime(5000);
    expect(sockets.length).toBe(1);
  });

  it("a socket error alone does not throw", () => {
    const link = new RosLink("ws://car", factory);
    link.connect();
    expect(() => sockets[0].onerror?.({ error: new Error("boom") })).not.toThrow();
  });

  it("duty-cycles a camera: one frame, unsubscribe, wait, subscribe again", () => {
    vi.useFakeTimers();
    const link = new RosLink("ws://car", factory);
    const frames: number[] = [];
    link.subscribe<{ data: number }>("/cam", (m) => frames.push(m.data), { oneShotEveryMs: 700 });
    link.connect();
    const s = sockets[0];
    s.open();
    s.serverJson({ op: "advertise", channels: [channel(3, "/cam", "std_msgs/msg/Float32", FLOAT_SCHEMA)] });
    expect(s.ops().map((o) => o.op)).toEqual(["subscribe"]);
    s.serverMessage(0, cdr(FLOAT_SCHEMA, { data: 1 }));
    s.serverMessage(0, cdr(FLOAT_SCHEMA, { data: 2 })); // a second frame already in flight
    expect(frames).toEqual([1]); // the second one is dropped: the subscription is gone
    expect(s.ops().map((o) => o.op)).toEqual(["subscribe", "unsubscribe"]);
    vi.advanceTimersByTime(699);
    expect(s.ops().length).toBe(2);
    vi.advanceTimersByTime(1);
    expect(s.ops().map((o) => o.op)).toEqual(["subscribe", "unsubscribe", "subscribe"]);
    s.serverMessage(1, cdr(FLOAT_SCHEMA, { data: 3 }));
    expect(frames).toEqual([1, 3]);
  });

  it("stops the duty cycle when the caller unsubscribes", () => {
    vi.useFakeTimers();
    const link = new RosLink("ws://car", factory);
    const off = link.subscribe<{ data: number }>("/cam", () => undefined, { oneShotEveryMs: 700 });
    link.connect();
    const s = sockets[0];
    s.open();
    s.serverJson({ op: "advertise", channels: [channel(3, "/cam", "std_msgs/msg/Float32", FLOAT_SCHEMA)] });
    s.serverMessage(0, cdr(FLOAT_SCHEMA, { data: 1 }));
    off();
    vi.advanceTimersByTime(5000);
    expect(s.ops().filter((o) => o.op === "subscribe").length).toBe(1);
  });

  it("never sends anything but subscribe and unsubscribe", () => {
    const link = new RosLink("ws://car", factory);
    link.subscribe("/a", () => undefined);
    link.subscribe("/b", () => undefined, { oneShotEveryMs: 10 });
    link.connect();
    const s = sockets[0];
    s.open();
    s.serverJson({
      op: "advertise",
      channels: [
        channel(1, "/a", "std_msgs/msg/Float32", FLOAT_SCHEMA),
        channel(2, "/b", "std_msgs/msg/Float32", FLOAT_SCHEMA),
        channel(3, "/drive_raw", "std_msgs/msg/Float32", FLOAT_SCHEMA),
      ],
    });
    s.serverMessage(1, cdr(FLOAT_SCHEMA, { data: 1 }));
    expect(s.sent.every((x) => typeof x === "string")).toBe(true); // no binary client messages
    const ops = new Set(s.ops().map((o) => o.op));
    expect([...ops].sort()).toEqual(["subscribe", "unsubscribe"]);
  });
});
