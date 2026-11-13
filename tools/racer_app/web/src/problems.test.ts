import { describe, expect, it } from "vitest";
import type { Check, LogEntry, Status } from "./api";
import { badgeCount, failedActions, liveProblems, ProblemStore } from "./problems";

function status(running: boolean, checks: Partial<Check>[]): Status {
  return {
    stack: { running, recorder: running, flags: {} },
    checks: {
      overall: "red",
      ready: false,
      checks: checks.map((c, i) => ({ id: `c${i}`, label: `C${i}`, level: "green", detail: "", required: true, ...c })),
    },
  } as unknown as Status;
}

describe("ProblemStore", () => {
  it("keeps failures newest first and counts the unseen ones", () => {
    let t = 1000;
    const s = new ProblemStore(() => t);
    s.add("Starting the car", "VESC port busy");
    t = 2000;
    s.add("Delete x", "being recorded");
    expect(s.all().map((p) => p.title)).toEqual(["Delete x", "Starting the car"]);
    expect(s.unseen()).toHaveLength(2);
    t = 3000;
    s.markSeen();
    expect(s.unseen()).toHaveLength(0);
    expect(s.all()).toHaveLength(2); // seen, still listed
    t = 4000;
    s.add("Stop", "network");
    expect(s.unseen()).toHaveLength(1);
    s.clear();
    expect(s.all()).toHaveLength(0);
  });

  it("does not list the same keyed problem twice and caps the list", () => {
    const s = new ProblemStore(() => 1, 3);
    s.add("a", "b", "k");
    s.add("a", "b", "k");
    expect(s.all()).toHaveLength(1);
    for (let i = 0; i < 5; i++) s.add(`p${i}`, "", `k${i}`);
    expect(s.all()).toHaveLength(3);
  });

  it("tells subscribers", () => {
    const s = new ProblemStore();
    let n = 0;
    const off = s.subscribe(() => n++);
    s.add("x", "y");
    s.markSeen();
    off();
    s.add("z", "w");
    expect(n).toBe(2);
  });
});

describe("liveProblems", () => {
  it("is empty with the stack off: red stack checks are a state then, not a problem", () => {
    expect(liveProblems(status(false, [{ level: "red" }]), 0)).toEqual([]);
    expect(liveProblems(null, 0)).toEqual([]);
  });

  it("lists required red checks while the stack runs, not optional or amber ones", () => {
    const list = liveProblems(
      status(true, [{ level: "red", label: "Battery" }, { level: "red", required: false }, { level: "amber" }, { level: "green" }]),
      5,
    );
    expect(list.map((p) => p.title)).toEqual(["Battery"]);
    expect(list[0].kind).toBe("live");
  });
});

describe("failedActions", () => {
  it("picks failures from the agent's log since a time", () => {
    const log: LogEntry[] = [
      { ts: 100, action: "drive.start.pnp", ok: false, actor: "a", detail: "no bundle" },
      { ts: 90, action: "stack.start", ok: true, actor: "a", detail: "" },
      { ts: 10, action: "old", ok: false, actor: "a", detail: "" },
    ];
    const list = failedActions(log, 50);
    expect(list.map((p) => p.title)).toEqual(["drive.start.pnp"]);
    expect(list[0].at).toBe(100_000);
  });
});

describe("badgeCount", () => {
  it("adds live problems and unseen failures", () => {
    const s = new ProblemStore(() => 1);
    s.add("x", "y");
    expect(badgeCount(liveProblems(status(true, [{ level: "red" }]), 0), s.unseen())).toBe(2);
  });
});
