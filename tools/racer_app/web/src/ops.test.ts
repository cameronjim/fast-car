import { describe, expect, it, vi } from "vitest";
import { ApiError } from "./api";
import { DemoCar } from "./demoAgent";
import { OpsStore, opSteps, TIMEOUT_MS, type Snapshot } from "./ops";

function setup() {
  let now = 1_000_000;
  const store = new OpsStore(() => now, 6000);
  const car = new DemoCar(() => now);
  let seq = 0;
  const snap = (over: Partial<Snapshot> = {}): Snapshot => ({
    status: car.status(),
    linkConnected: car.rosUp(),
    bags: [],
    seq: ++seq,
    ...over,
  });
  const advance = (ms: number) => (now += ms);
  const flush = () => new Promise((r) => setTimeout(r, 0));
  return { store, car, snap, advance, flush, clock: () => now };
}

const states = (steps: { state: string }[]) => steps.map((s) => s.state);

describe("optimistic start: requested -> container -> nodes -> recorder -> ready", () => {
  it("shows the target at once, then follows the status, then confirms", async () => {
    const t = setup();
    const op = t.store.begin({ kind: "stack-start" }, () => Promise.resolve({ ok: true }))!;
    expect(op.phase).toBe("requesting");
    expect(t.store.activeOf("stack-start")).toBeDefined();
    // nothing is up yet: only "requested" is done, "container up" is the active step
    expect(states(opSteps(op, t.snap()))).toEqual(["done", "active", "todo", "todo", "todo"]);

    await t.flush();
    expect(t.store.all()[0].phase).toBe("waiting");

    t.car.containerUp = true;
    t.store.tick(t.snap());
    expect(states(opSteps(t.store.all()[0], t.snap()))).toEqual(["done", "done", "active", "todo", "todo"]);

    t.car.nodesUp = true;
    t.store.tick(t.snap());
    t.car.recorderUp = true;
    expect(t.store.all()[0].phase).toBe("waiting");
    t.store.tick(t.snap());
    expect(t.store.all()[0].phase).toBe("confirmed");
    expect(states(opSteps(t.store.all()[0], t.snap()))).toEqual(["done", "done", "done", "done", "done"]);
    expect(t.store.active()).toHaveLength(0);
  });

  it("does not confirm on the request reply alone: only the status can confirm", async () => {
    const t = setup();
    t.store.begin({ kind: "stack-start" }, () => Promise.resolve({ ok: true }));
    await t.flush();
    t.store.tick(t.snap());
    expect(t.store.all()[0].phase).toBe("waiting");
  });

  it("a refused request fails at once with the agent's reason and sends nothing twice", async () => {
    const t = setup();
    const request = vi.fn(() => Promise.reject(new ApiError("VESC port busy", 500, null)));
    t.store.begin({ kind: "stack-start" }, request);
    expect(t.store.begin({ kind: "stack-start" }, request)).toBeNull(); // double click
    await t.flush();
    expect(request).toHaveBeenCalledTimes(1);
    const op = t.store.all()[0];
    expect(op.phase).toBe("failed");
    expect(op.error).toBe("VESC port busy");
    expect(t.store.active()).toHaveLength(0);
    // failures stay until dismissed, even long after
    t.advance(10 * 60_000);
    t.store.tick(t.snap());
    expect(t.store.all()).toHaveLength(1);
    t.store.dismiss(op.id);
    expect(t.store.all()).toHaveLength(0);
  });

  it("times out when the target is not reached, naming the first unmet step", async () => {
    const t = setup();
    t.store.begin({ kind: "stack-start" }, () => Promise.resolve({ ok: true }));
    await t.flush();
    t.car.containerUp = true; // container comes up, the nodes never do
    t.store.tick(t.snap());
    t.advance(TIMEOUT_MS["stack-start"] + 1);
    t.store.tick(t.snap());
    const op = t.store.all()[0];
    expect(op.phase).toBe("timeout");
    expect(op.error).toContain("nodes up");
    expect(states(opSteps(op, t.snap()))).toEqual(["done", "done", "failed", "todo", "todo"]);
  });

  it("a timeout at the last step says which required checks are red", async () => {
    const t = setup();
    t.car.lowBattery = true;
    t.store.begin({ kind: "stack-start" }, () => Promise.resolve({ ok: true }));
    await t.flush();
    t.car.containerUp = t.car.nodesUp = t.car.recorderUp = true;
    t.advance(TIMEOUT_MS["stack-start"] + 1);
    t.store.tick(t.snap());
    const op = t.store.all()[0];
    expect(op.phase).toBe("timeout");
    expect(op.error).toContain("Not ready: Battery");
  });

  it("times out with a plain message when the car never answers", async () => {
    const t = setup();
    t.store.begin({ kind: "stack-start" }, () => new Promise(() => undefined));
    t.advance(TIMEOUT_MS["stack-start"] + 1);
    t.store.tick({ status: null, linkConnected: false, bags: [] });
    expect(t.store.all()[0].phase).toBe("timeout");
    expect(t.store.all()[0].error).toContain("No status from the car");
  });

  it("drops a confirmed operation after it has been shown for a while", async () => {
    const t = setup();
    t.car.containerUp = t.car.nodesUp = t.car.recorderUp = true;
    t.store.begin({ kind: "cameras-start" }, () => Promise.resolve({ ok: true }));
    t.car.cameras = true;
    t.store.tick(t.snap());
    expect(t.store.all()[0].phase).toBe("confirmed");
    t.advance(7000);
    t.store.tick(t.snap());
    expect(t.store.all()).toHaveLength(0);
  });
});

describe("optimistic stop: closing the bag", () => {
  it("stays in flight while the container is up, confirms when the car reports it down", async () => {
    const t = setup();
    t.car.containerUp = t.car.nodesUp = t.car.recorderUp = true;
    t.store.begin({ kind: "stack-stop" }, () => new Promise(() => undefined));
    t.store.tick(t.snap());
    expect(states(opSteps(t.store.all()[0], t.snap()))).toEqual(["done", "active", "todo"]);
    t.advance(120_000); // a big bag takes minutes: not a failure
    t.store.tick(t.snap());
    expect(t.store.all()[0].phase).toBe("requesting");
    t.car.containerUp = t.car.nodesUp = t.car.recorderUp = false;
    t.store.tick(t.snap());
    expect(t.store.all()[0].phase).toBe("confirmed");
  });

  it("has a timeout longer than the agent's own SIGINT wait", () => {
    expect(TIMEOUT_MS["stack-stop"]).toBeGreaterThan(600_000);
  });
});

describe("restart waits for a poll made after the reply", () => {
  it("does not confirm on the old running status", async () => {
    const t = setup();
    t.car.containerUp = t.car.nodesUp = t.car.recorderUp = true;
    t.store.begin({ kind: "stack-restart" }, () => Promise.resolve({ ok: true }));
    t.store.tick(t.snap({ seq: 5 })); // polls before the reply still show the old stack
    await t.flush();
    expect(t.store.all()[0].ackSeq).toBe(5);
    t.store.tick(t.snap({ seq: 6 })); // may have been in flight when the reply came
    expect(t.store.all()[0].phase).toBe("waiting");
    t.store.tick(t.snap({ seq: 7 }));
    expect(t.store.all()[0].phase).toBe("confirmed");
  });

  it("confirms as soon as the old container is seen going down and the new one up", async () => {
    const t = setup();
    t.car.containerUp = t.car.nodesUp = t.car.recorderUp = true;
    t.store.begin({ kind: "stack-restart" }, () => new Promise(() => undefined));
    t.car.containerUp = false;
    t.store.tick(t.snap());
    expect(t.store.all()[0].sawDown).toBe(true);
    t.car.containerUp = t.car.nodesUp = t.car.recorderUp = true;
    t.store.tick(t.snap());
    expect(t.store.all()[0].phase).toBe("confirmed");
  });
});

describe("drives and STOP ALL", () => {
  it("a drive start confirms when the status reports that mode", async () => {
    const t = setup();
    t.car.containerUp = t.car.nodesUp = t.car.recorderUp = true;
    t.store.begin({ kind: "drive-start", target: "gap_follow" }, () => Promise.resolve({ ok: true }));
    await t.flush();
    t.store.tick(t.snap());
    expect(t.store.all()[0].phase).toBe("waiting");
    t.car.driveMode = "gap_follow";
    t.store.tick(t.snap());
    expect(t.store.all()[0].phase).toBe("confirmed");
  });

  it("a different running mode does not confirm it", async () => {
    const t = setup();
    t.car.containerUp = t.car.nodesUp = t.car.recorderUp = true;
    t.car.driveMode = "wall_follow";
    t.store.begin({ kind: "drive-start", target: "gap_follow" }, () => Promise.resolve({ ok: true }));
    await t.flush();
    t.store.tick(t.snap());
    expect(t.store.all()[0].phase).toBe("waiting");
  });

  it("STOP ALL confirms only when the car reports nothing driving", async () => {
    const t = setup();
    t.car.containerUp = t.car.nodesUp = t.car.recorderUp = true;
    t.car.driveMode = "gap_follow";
    t.store.begin({ kind: "stop-all" }, () => Promise.resolve({ ok: true }));
    await t.flush();
    t.store.tick(t.snap());
    expect(t.store.all()[0].phase).toBe("waiting");
    t.car.driveMode = null;
    t.store.tick(t.snap());
    expect(t.store.all()[0].phase).toBe("confirmed");
  });

  it("notifies subscribers on every change", async () => {
    const t = setup();
    const cb = vi.fn();
    t.store.subscribe(cb);
    t.store.begin({ kind: "cameras-start" }, () => Promise.resolve({ ok: true }));
    await t.flush();
    expect(cb).toHaveBeenCalled();
  });
});
