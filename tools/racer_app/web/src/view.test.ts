import { describe, expect, it } from "vitest";
import { DemoCar } from "./demoAgent";
import { OpsStore } from "./ops";
import {
  ageLabel,
  batteryPill,
  connectionState,
  driveBlocked,
  driveState,
  elapsedLabel,
  freshness,
  modeMatches,
  readyPill,
  recordingPill,
  stackState,
} from "./view";

const never = () => new Promise<never>(() => undefined);

function running() {
  const car = new DemoCar();
  car.containerUp = car.nodesUp = car.recorderUp = true;
  return car;
}

describe("stale values keep their value and show their age", () => {
  it("is fresh just after it arrives and stale later, never blank", () => {
    const s = { value: 1.2, at: 10_000 };
    expect(freshness(s, 10_500)).toMatchObject({ value: 1.2, stale: false });
    const old = freshness(s, 17_000);
    expect(old.value).toBe(1.2);
    expect(old.stale).toBe(true);
    expect(old.ageS).toBeCloseTo(7);
  });
  it("never seen is stale with no value", () => {
    expect(freshness(null, 5)).toEqual({ value: null, ageS: null, stale: true });
  });
  it("labels ages in plain words", () => {
    expect(ageLabel(null)).toBe("no data");
    expect(ageLabel(0.4)).toBe("live");
    expect(ageLabel(7.2)).toBe("7 s ago");
    expect(ageLabel(150)).toBe("3 min ago");
  });
  it("formats elapsed time", () => {
    expect(elapsedLabel(4200)).toBe("4 s");
    expect(elapsedLabel(125_000)).toBe("2 min 05 s");
  });
});

describe("the optimistic state wins over the poll until the operation ends", () => {
  it("stack: starting while a start is in flight, even though the poll says stopped", () => {
    const store = new OpsStore();
    const car = new DemoCar();
    expect(stackState(car.status(), store)).toBe("off");
    store.begin({ kind: "stack-start" }, never);
    expect(stackState(car.status(), store)).toBe("starting");
  });
  it("stack: stopping while the bag closes, though the poll still says running", () => {
    const store = new OpsStore();
    store.begin({ kind: "stack-stop" }, never);
    expect(stackState(running().status(), store)).toBe("stopping");
  });
  it("stack: restarting", () => {
    const store = new OpsStore();
    store.begin({ kind: "stack-restart" }, never);
    expect(stackState(running().status(), store)).toBe("restarting");
  });
  it("falls back to the poll once the operation has failed", async () => {
    const store = new OpsStore();
    store.begin({ kind: "stack-start" }, () => Promise.reject(new Error("nope")));
    await new Promise((r) => setTimeout(r, 0));
    expect(stackState(new DemoCar().status(), store)).toBe("off");
  });
  it("drive: starting shows the target mode before the car reports it", () => {
    const store = new OpsStore();
    store.begin({ kind: "drive-start", target: "pnp" }, never);
    expect(driveState(running().status(), store)).toEqual({ state: "starting", mode: "pnp" });
  });
  it("drive: stopping while STOP ALL is in flight and something still runs", () => {
    const store = new OpsStore();
    const car = running();
    car.driveMode = "gap_follow";
    store.begin({ kind: "stop-all" }, never);
    expect(driveState(car.status(), store)).toEqual({ state: "stopping", mode: "gap_follow" });
  });
  it("drive: idle and running from the poll", () => {
    const store = new OpsStore();
    const car = running();
    expect(driveState(car.status(), store).state).toBe("idle");
    car.driveMode = "park";
    expect(driveState(car.status(), store)).toEqual({ state: "running", mode: "park" });
  });
  it("a park launch matches the park card", () => {
    expect(modeMatches("park", "park")).toBe(true);
    expect(modeMatches("park_turn", "park")).toBe(true);
    expect(modeMatches("gap_follow", "park")).toBe(false);
    expect(modeMatches(null, "park")).toBe(false);
  });
});

describe("why a drive button is disabled", () => {
  it("explains each case in a few words", () => {
    const store = new OpsStore();
    expect(driveBlocked(null, store, true)).toBe("Loading");
    expect(driveBlocked(new DemoCar().status(), store, false)).toBe("Car offline");
    expect(driveBlocked(new DemoCar().status(), store, true)).toBe("Start the stack first");
    expect(driveBlocked(running().status(), store, true)).toBe("");
    const car = running();
    car.driveMode = "gap_follow";
    expect(driveBlocked(car.status(), store, true)).toBe("gap_follow is running");
  });
  it("blocks while the stack is starting or stopping", () => {
    const a = new OpsStore();
    a.begin({ kind: "stack-start" }, never);
    expect(driveBlocked(new DemoCar().status(), a, true)).toBe("Stack starting");
    const b = new OpsStore();
    b.begin({ kind: "stack-stop" }, never);
    expect(driveBlocked(running().status(), b, true)).toBe("Stack stopping");
  });
});

describe("pills", () => {
  it("ready and not ready (with the failing count)", () => {
    expect(readyPill(null).text).toBe("Checking");
    const car = running();
    car.cameras = true;
    expect(readyPill(car.status())).toMatchObject({ level: "green", text: "Ready" });
    expect(readyPill(running().status())).toMatchObject({ level: "amber", text: "Ready" }); // optional cameras off
    const stopped = readyPill(new DemoCar().status());
    expect(stopped.level).toBe("red");
    expect(stopped.text).toMatch(/^Not ready \(\d+\)$/);
    expect(stopped.title).toContain("Stack container");
  });
  it("recording shows the size and the rate", () => {
    const car = running();
    car.bags = [{ name: "run_x", size_mb: 3200, mtime: 0, active: true, growth_mb_s: 0.52 }];
    expect(recordingPill(car.bags, car.status()).text).toBe("REC 3.2 GB +0.52 MB/s");
    expect(recordingPill([], new DemoCar().status()).text).toBe("Not recording");
  });
  it("the battery pill greys out when the reading is stale", () => {
    const fresh = batteryPill({ value: 11.8, ageS: 0.2, stale: false }, 3);
    expect(fresh).toMatchObject({ level: "green", text: "11.8 V" });
    const stale = batteryPill({ value: 11.8, ageS: 30, stale: true }, 3);
    expect(stale.level).toBe("neutral");
    expect(stale.text).toBe("11.8 V (30 s ago)");
    expect(batteryPill({ value: null, ageS: null, stale: true }, 3).text).toBe("No battery data");
    expect(batteryPill({ value: 10.2, ageS: 0, stale: false }, 3).level).toBe("red");
  });
  it("the connection pill: agent down means reconnecting; a down ROS link only counts with the stack up", () => {
    expect(connectionState(false, false, true)).toBe("reconnecting");
    expect(connectionState(true, true, true)).toBe("connected");
    expect(connectionState(true, false, true)).toBe("live-down");
    expect(connectionState(true, false, false)).toBe("connected");
  });
});
