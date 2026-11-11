import { parse } from "@foxglove/rosmsg";
import { MessageWriter } from "@foxglove/rosmsg2-serialization";
import { describe, expect, it, vi } from "vitest";
import { emptyLive, startLive } from "./live";
import { RosLink } from "./ros";

// A minimal stand-in for RosLink: records subscriptions and lets the test deliver messages.
class FakeLink {
  subs = new Map<string, { handler: (m: never) => void; opts: unknown }>();
  subscribe(topic: string, handler: (m: never) => void, opts: unknown = {}) {
    this.subs.set(topic, { handler, opts });
    return () => this.subs.delete(topic);
  }
  deliver(topic: string, msg: unknown) {
    this.subs.get(topic)!.handler(msg as never);
  }
}

describe("startLive", () => {
  it("subscribes to the dashboard topics and nothing on the command path", () => {
    const link = new FakeLink();
    startLive(link as unknown as RosLink, emptyLive(), () => true);
    expect([...link.subs.keys()].sort()).toEqual(
      [
        "/camera/csi0/image_raw/compressed",
        "/camera/usb/image_raw/compressed",
        "/drive",
        "/lap/status",
        "/map",
        "/odom/wheel",
        "/pose",
        "/safety/events",
        "/scan",
        "/telemetry/vesc/current_input_a",
        "/telemetry/vesc/fault",
        "/telemetry/vesc/temp_fet_degc",
        "/telemetry/vesc/temp_motor_degc",
        "/telemetry/vesc/voltage_v",
      ].sort(),
    );
    expect(link.subs.has("/drive_raw")).toBe(false);
  });

  it("only the cameras are duty-cycled", () => {
    const link = new FakeLink();
    startLive(link as unknown as RosLink, emptyLive(), () => true);
    for (const [topic, { opts }] of link.subs) {
      const isCam = topic.startsWith("/camera/");
      expect(Boolean((opts as { oneShotEveryMs?: number }).oneShotEveryMs), topic).toBe(isCam);
    }
  });

  it("fills the live values from messages", () => {
    const link = new FakeLink();
    const live = emptyLive();
    startLive(link as unknown as RosLink, live, () => true);
    link.deliver("/odom/wheel", { twist: { twist: { linear: { x: 1.5 } } } });
    link.deliver("/drive", { drive: { steering_angle: -0.1, speed: 1.4 } });
    link.deliver("/telemetry/vesc/voltage_v", { data: 11.8 });
    link.deliver("/telemetry/vesc/fault", { data: "NONE" });
    link.deliver("/lap/status", { data: '{"state":"running","similarity":0.5,"laps":2,"last_lap_s":18.1,"best_lap_s":17.9}' });
    link.deliver("/lap/status", { data: "garbage" }); // ignored: the last good status stays
    link.deliver("/pose", {
      pose: { pose: { position: { x: 1, y: 2 }, orientation: { x: 0, y: 0, z: 0, w: 1 } } },
    });
    link.deliver("/safety/events", {
      stamp: { sec: 1, nanosec: 2 },
      severity: 2,
      phase: 0,
      duration_s: 0,
      source: "watchdog",
      detail: "silent",
    });
    expect(live.speed?.value).toBe(1.5);
    expect(live.steering?.value).toEqual({ angleRad: -0.1, speedMps: 1.4 });
    expect(live.voltage?.value).toBe(11.8);
    expect(live.fault?.value).toBe("NONE");
    expect(live.lap?.value).toMatchObject({ laps: 2, lastLapS: 18.1, bestLapS: 17.9 });
    expect(live.pose?.value).toEqual({ x: 1, y: 2, yaw: 0 });
    expect(live.events[0].text).toContain("watchdog engage [BRAKE]");
  });

  it("keeps the newest camera frame only and frees the old blob", () => {
    const link = new FakeLink();
    const live = emptyLive();
    const revoked: string[] = [];
    const origCreate = URL.createObjectURL;
    const origRevoke = URL.revokeObjectURL;
    let n = 0;
    URL.createObjectURL = () => `blob:${++n}`;
    URL.revokeObjectURL = (u: string) => void revoked.push(u);
    try {
      startLive(link as unknown as RosLink, live, () => true);
      const jpeg = { data: new Uint8Array([1, 2, 3]) };
      link.deliver("/camera/usb/image_raw/compressed", jpeg);
      link.deliver("/camera/usb/image_raw/compressed", jpeg);
      expect(live.camUsb).toBe("blob:2");
      expect(revoked).toEqual(["blob:1"]);
    } finally {
      URL.createObjectURL = origCreate;
      URL.revokeObjectURL = origRevoke;
    }
  });

  it("drops camera frames while the page is hidden", () => {
    const link = new FakeLink();
    const live = emptyLive();
    startLive(link as unknown as RosLink, live, () => false);
    const create = vi.spyOn(URL, "createObjectURL");
    link.deliver("/camera/csi0/image_raw/compressed", { data: new Uint8Array([1]) });
    expect(live.camCsi).toBeNull();
    expect(create).not.toHaveBeenCalled();
    create.mockRestore();
  });

  it("the unsubscribe function removes every subscription", () => {
    const link = new FakeLink();
    const stop = startLive(link as unknown as RosLink, emptyLive(), () => true);
    stop();
    expect(link.subs.size).toBe(0);
  });

  it("stores a map and a scan", () => {
    const link = new FakeLink();
    const live = emptyLive();
    startLive(link as unknown as RosLink, live, () => true);
    const writer = new MessageWriter(parse("float32 data", { ros2: true }));
    expect(writer.writeMessage({ data: 1 }).length).toBeGreaterThan(0); // the schema helpers load
    const grid = { info: { resolution: 1, width: 1, height: 1, origin: { position: { x: 0, y: 0 }, orientation: { x: 0, y: 0, z: 0, w: 1 } } }, data: [0] };
    link.deliver("/map", grid);
    link.deliver("/scan", { angle_min: 0, angle_increment: 1, range_min: 0, range_max: 1, ranges: [1] });
    expect(live.map).toBe(grid);
    expect(live.scan?.value.ranges.length).toBe(1);
  });
});
