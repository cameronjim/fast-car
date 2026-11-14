import { afterEach, describe, expect, it, vi } from "vitest";
import { Api, ApiError, type TokenStore } from "./api";
import { DemoCar, DEMO_PRESETS, parsePreset } from "./demoAgent";
import { DemoRos } from "./demoRos";
import { isDemo } from "./demo";
import { OpsStore } from "./ops";

const tokens: TokenStore = { get: () => "demo", set: () => undefined, clear: () => undefined };

// speed 0.001 turns every scripted delay into a few milliseconds of real time
function rig() {
  const car = new DemoCar(() => Date.now(), (ms) => new Promise((r) => setTimeout(r, ms)), 0.001);
  const api = new Api(tokens, car.fetch);
  return { car, api };
}
const wait = (ms: number) => new Promise((r) => setTimeout(r, ms));

afterEach(() => vi.useRealTimers());

describe("demo fake agent", () => {
  it("parses the demo query", () => {
    expect(isDemo("?demo=1")).toBe(true);
    expect(isDemo("?demo=0")).toBe(false);
    expect(isDemo("")).toBe(false);
    expect(parsePreset("driving")).toBe("driving");
    expect(parsePreset("nonsense")).toBe("idle");
    expect(DEMO_PRESETS).toContain("failed");
  });

  it("starts the stack in stages: container, then nodes, then recorder, then ready", async () => {
    const { car, api } = rig();
    expect((await api.status()).stack.running).toBe(false);
    const reply = await api.startStack({ lidar: true, vesc: true, mapping: false, localization: false, map: "" });
    expect(reply.ok).toBe(true);
    const early = await api.status();
    expect(early.stack.running).toBe(true);
    expect(early.stack.recorder).toBe(false);
    expect(early.checks.ready).toBe(false);
    await wait(60);
    const late = await api.status();
    expect(late.stack.recorder).toBe(true);
    expect(late.checks.ready).toBe(true);
    expect(car.bags[0].active).toBe(true);
  });

  it("a failed start returns the agent's error and leaves the stack down", async () => {
    const { car, api } = rig();
    car.failNext = true;
    await expect(api.startStack({ lidar: true, vesc: true, mapping: false, localization: false, map: "" })).rejects.toMatchObject({
      status: 500,
      message: expect.stringContaining("VESC port"),
    });
    expect((await api.status()).stack.running).toBe(false);
    expect(car.failNext).toBe(false); // the next start works
  });

  it("a stop takes a while (closing the bag) and closes the bag", async () => {
    const { car, api } = rig();
    car.preset("running");
    const stop = api.stopStack();
    expect((await api.status()).stack.running).toBe(true); // still closing
    await stop;
    const s = await api.status();
    expect(s.stack.running).toBe(false);
    expect(car.bags.some((b) => b.active)).toBe(false);
  });

  it("refuses a drive without the stack, and never lets an override past the recorder", async () => {
    const { car, api } = rig();
    await expect(api.startDrive({ mode: "gap_follow" })).rejects.toMatchObject({ status: 409 });
    car.preset("running");
    car.recorderUp = false;
    await expect(api.startDrive({ mode: "gap_follow", override: true, override_reason: "test" })).rejects.toMatchObject({
      body: { overridable: false },
    });
  });

  it("the demo agent takes the lap fields and refuses what the car refuses", async () => {
    const { car, api } = rig();
    car.preset("running");
    const base = { mode: "pnp", pnp_backend: "geometric", pnp_dynamics: "dyn-2026-10-20" };
    for (const bad of [
      { pnp_lap_mode: "v1" },
      { pnp_lap_memory: "true" },
      { pnp_lap_memory: true, pnp_lap_mode: "v1", pnp_lap_source: "map_pose" },
      { pnp_v_lap1_mps: 0.3 },
      { pnp_lap_memory: true, pnp_v_lap1_mps: 0.9 },
      { pnp_require_lap_counter: true, pnp_lap_counter: false },
    ]) {
      await expect(api.startDrive({ ...base, ...bad } as never)).rejects.toMatchObject({ status: 400 });
    }
    await api.startDrive({ ...base, pnp_lap_memory: true, pnp_lap_mode: "v1", pnp_v_lap1_mps: 0.3, pnp_shadow_perception: "net-2026-11-04" });
    expect((await api.status()).drive.mode).toBe("pnp");
  });

  it("the override flow: refused as overridable, accepted with a reason, logged", async () => {
    const { car, api } = rig();
    car.preset("running");
    car.lowBattery = true;
    const refused = await api.startDrive({ mode: "wall_follow" }).catch((e: ApiError) => e);
    expect(refused).toBeInstanceOf(ApiError);
    expect((refused as ApiError).body?.overridable).toBe(true);
    await expect(api.startDrive({ mode: "wall_follow", override: true, override_reason: "" })).rejects.toMatchObject({ status: 400 });
    await api.startDrive({ mode: "wall_follow", override: true, override_reason: "bench test" });
    expect((await api.status()).drive.mode).toBe("wall_follow");
    expect(car.log[0]).toMatchObject({ action: "drive/start", override: true, override_reason: "bench test" });
  });

  it("STOP ALL needs no token and stops the drive", async () => {
    const { car, api } = rig();
    car.preset("driving");
    await api.stopAll();
    expect((await api.status()).drive.mode).toBeNull();
  });

  it("a dropped connection rejects like a network error, then recovers", async () => {
    const { car, api } = rig();
    car.preset("running");
    const drop = car.dropConnection(20);
    await expect(api.status()).rejects.toMatchObject({ status: 0 });
    await drop;
    expect((await api.status()).stack.running).toBe(true);
  });

  it("drives the optimistic store end to end: start the stack and watch it confirm", async () => {
    const { car, api } = rig();
    const store = new OpsStore();
    store.begin({ kind: "stack-start" }, () => api.startStack({ lidar: true, vesc: true, mapping: false, localization: false, map: "" }));
    expect(store.active()).toHaveLength(1);
    for (let i = 0; i < 60 && store.active().length; i++) {
      await wait(5);
      store.tick({ status: await api.status(), linkConnected: car.rosUp(), bags: [] });
    }
    expect(store.all()[0].phase).toBe("confirmed");
  });

  it("fails the optimistic start on the agent's refusal", async () => {
    const { car, api } = rig();
    car.failNext = true;
    const store = new OpsStore();
    store.begin({ kind: "stack-start" }, () => api.startStack({ lidar: true, vesc: true, mapping: false, localization: false, map: "" }));
    await wait(30);
    expect(store.all()[0]).toMatchObject({ phase: "failed" });
    expect(store.all()[0].error).toContain("VESC port");
  });
});

describe("demo fake ROS link", () => {
  it("is down while the stack is down, connects and emits when it is up, drops with the link", async () => {
    vi.useFakeTimers();
    const car = new DemoCar();
    const ros = new DemoRos(car);
    const states: boolean[] = [];
    ros.onStatus((s) => states.push(s.connected));
    const speeds: number[] = [];
    ros.subscribe<{ twist: { twist: { linear: { x: number } } } }>("/odom/wheel", (m) => speeds.push(m.twist.twist.linear.x));
    ros.connect();
    vi.advanceTimersByTime(300);
    expect(states).toEqual([false]);
    expect(speeds).toHaveLength(0);

    car.preset("driving");
    vi.advanceTimersByTime(300);
    expect(states).toEqual([false, true]);
    expect(speeds.length).toBeGreaterThan(0);
    expect(speeds.some((v) => v > 0)).toBe(true);

    car.offline = true;
    vi.advanceTimersByTime(200);
    expect(states).toEqual([false, true, false]);
    ros.close();
  });

  it("only subscribes: it has no way to publish", () => {
    const ros = new DemoRos(new DemoCar()) as unknown as Record<string, unknown>;
    for (const k of ["advertise", "publish", "sendMessage", "callService", "setParameters"]) expect(ros[k]).toBeUndefined();
  });
});
