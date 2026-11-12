import { afterEach, beforeEach, describe, expect, it, vi } from "vitest";
import { ApiError } from "./api";
import { ActionRunner, ConnectionTracker, errorText, lastSeenText, ToastStore } from "./feedback";

beforeEach(() => vi.useFakeTimers());
afterEach(() => vi.useRealTimers());

describe("ActionRunner (the states of every action button)", () => {
  it("goes idle -> pending -> success -> idle", async () => {
    const r = new ActionRunner({ successMs: 1500 });
    const seen: string[] = [];
    r.subscribe(() => seen.push(r.state.phase));
    let done!: (v: { message: string }) => void;
    const p = r.run(() => new Promise((res) => (done = res)));
    expect(r.state.phase).toBe("pending");
    done({ message: "stack started" });
    const out = await p;
    expect(out).toEqual({ ok: true, message: "stack started" });
    expect(r.state.phase).toBe("success");
    vi.advanceTimersByTime(1499);
    expect(r.state.phase).toBe("success");
    vi.advanceTimersByTime(2);
    expect(r.state.phase).toBe("idle");
    expect(seen).toEqual(["pending", "success", "idle"]);
  });

  it("shows the reason inline on error and keeps it until the next run", async () => {
    const r = new ActionRunner();
    const out = await r.run(() => Promise.reject(new ApiError("the stack is not running", 409, null)));
    expect(out?.ok).toBe(false);
    expect(r.state).toEqual({ phase: "error", error: "the stack is not running" });
    vi.advanceTimersByTime(60_000);
    expect(r.state.phase).toBe("error");
    await r.run(() => Promise.resolve({ message: "ok" }));
    expect(r.state.phase).toBe("success");
    expect(r.state.error).toBe("");
  });

  it("can clear an error by itself after a while", async () => {
    const r = new ActionRunner({ errorMs: 5000 });
    await r.run(() => Promise.reject(new Error("boom")));
    expect(r.state.phase).toBe("error");
    vi.advanceTimersByTime(5001);
    expect(r.state.phase).toBe("idle");
  });

  it("ignores a second click while pending (no double send)", async () => {
    const r = new ActionRunner();
    const fn = vi.fn(() => new Promise<{ message: string }>(() => undefined));
    void r.run(fn);
    expect(await r.run(fn)).toBeNull();
    expect(fn).toHaveBeenCalledTimes(1);
  });

  it("a reentrant runner (STOP ALL) sends every click", async () => {
    const r = new ActionRunner({ reentrant: true });
    const fn = vi.fn(() => new Promise<{ message: string }>(() => undefined));
    void r.run(fn);
    void r.run(fn);
    expect(fn).toHaveBeenCalledTimes(2);
  });

  it("the latest run decides the final state when two overlap", async () => {
    const r = new ActionRunner({ reentrant: true });
    let first!: (v: { message: string }) => void;
    void r.run(() => new Promise((res) => (first = res)));
    await r.run(() => Promise.reject(new Error("second failed")));
    first({ message: "late" });
    await Promise.resolve();
    expect(r.state.phase).toBe("error");
  });

  it("never throws: a thrown non-Error becomes an outcome", async () => {
    const r = new ActionRunner();
    const out = await r.run(() => Promise.reject("weird"));
    expect(out?.ok).toBe(false);
    expect(r.state.error).toBe("weird");
  });
});

describe("errorText", () => {
  it("says plainly when the car cannot be reached", () => {
    expect(errorText(new ApiError("cannot reach the car (network)", 0, null))).toBe("Cannot reach the car");
    expect(errorText(new ApiError("nope", 409, null))).toBe("nope");
  });
});

describe("ToastStore", () => {
  it("stacks, caps, and auto-dismisses (errors stay longer)", () => {
    const s = new ToastStore(3);
    s.push("a");
    s.push("b");
    s.push("c", "error");
    s.push("d");
    expect(s.all().map((t) => t.text)).toEqual(["b", "c", "d"]);
    vi.advanceTimersByTime(3600);
    expect(s.all().map((t) => t.text)).toEqual(["c"]);
    vi.advanceTimersByTime(6000);
    expect(s.all()).toHaveLength(0);
  });

  it("does not stack the same message twice, it refreshes it", () => {
    const s = new ToastStore();
    s.push("Lost contact");
    vi.advanceTimersByTime(3000);
    s.push("Lost contact");
    expect(s.all()).toHaveLength(1);
    vi.advanceTimersByTime(1000);
    expect(s.all()).toHaveLength(1); // the timer restarted
  });

  it("can be dismissed by hand and tells subscribers", () => {
    const s = new ToastStore();
    const cb = vi.fn();
    s.subscribe(cb);
    const id = s.push("x");
    s.dismiss(id);
    expect(s.all()).toHaveLength(0);
    expect(cb).toHaveBeenCalledTimes(2);
  });
});

describe("ConnectionTracker (reconnect handling)", () => {
  it("ignores one failed poll, goes down on the second, reports the reconnect", () => {
    const c = new ConnectionTracker(2);
    expect(c.ok(1000)).toBeNull();
    expect(c.fail()).toBeNull();
    expect(c.state.up).toBe(true);
    expect(c.fail()).toBe("lost");
    expect(c.state.up).toBe(false);
    expect(c.fail()).toBeNull(); // already lost: no second alarm
    expect(c.state.lastSeen).toBe(1000);
    expect(c.ok(9000)).toBe("reconnected");
    expect(c.state).toEqual({ up: true, lastSeen: 9000, failures: 0 });
  });

  it("a good reply between failures resets the count", () => {
    const c = new ConnectionTracker(2);
    c.fail();
    c.ok(5);
    expect(c.fail()).toBeNull();
    expect(c.state.up).toBe(true);
  });

  it("mirrors a link that reports its own state", () => {
    const c = new ConnectionTracker(1);
    expect(c.set(true, 10)).toBeNull();
    expect(c.set(false, 20)).toBe("lost");
    expect(c.set(false, 30)).toBeNull();
    expect(c.state.lastSeen).toBe(10);
    expect(c.set(true, 40)).toBe("reconnected");
  });
});

describe("lastSeenText", () => {
  it("is plain about when the car was last heard", () => {
    expect(lastSeenText(null, 0)).toBe("not seen yet");
    expect(lastSeenText(1000, 13_000)).toBe("last seen 12 s ago");
    expect(lastSeenText(0, 5 * 60_000)).toBe("last seen 5 min ago");
  });
});
