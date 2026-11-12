// Optimistic operations: the UI shows the target state the moment the owner clicks, with a
// timeline of steps, and reconciles it against what the agent's status poll and the ROS link
// actually report. Truth always wins: a step is done only when the status says so, and an
// operation that has not reached its target in time is shown as failed with the first unmet step.
//
// Pure and framework-free (the store takes a clock), so ops.test.ts covers the whole machine.

import type { BagInfo, Status } from "./api";

export type OpKind =
  | "stack-start"
  | "stack-stop"
  | "stack-restart"
  | "cameras-start"
  | "cameras-stop"
  | "perception-start"
  | "perception-stop"
  | "drive-start"
  | "stop-all";

/** requesting: the HTTP call is out. waiting: it was accepted, the status has not caught up. */
export type OpPhase = "requesting" | "waiting" | "confirmed" | "failed" | "timeout";

export interface Snapshot {
  status: Status | null;
  /** The foxglove link to the car is up. */
  linkConnected: boolean;
  bags: BagInfo[];
  /** Counts completed status polls; lets a restart insist on a poll made after its reply. */
  seq?: number;
}

export interface Op {
  id: number;
  kind: OpKind;
  /** Short title, for example "Starting the stack". */
  title: string;
  /** For drive-start: the mode the status must report. */
  target?: string;
  startedAt: number;
  timeoutMs: number;
  phase: OpPhase;
  /** The HTTP request returned success. */
  acked: boolean;
  /** Restart only: the old container was seen going down. */
  sawDown: boolean;
  /** The poll count when the request returned (restart waits for two more polls). */
  ackSeq?: number;
  error?: string;
  finishedAt?: number;
}

export type StepState = "done" | "active" | "todo" | "failed";
export interface Step {
  id: string;
  label: string;
  state: StepState;
}

export const TIMEOUT_MS: Record<OpKind, number> = {
  "stack-start": 120_000,
  // The agent waits up to 600 s for the recorder to close the bag, then 20 s more.
  "stack-stop": 660_000,
  "stack-restart": 780_000,
  "cameras-start": 45_000,
  "cameras-stop": 150_000,
  "perception-start": 60_000,
  "perception-stop": 150_000,
  "drive-start": 30_000,
  "stop-all": 20_000,
};

const TITLES: Record<OpKind, string> = {
  "stack-start": "Starting the stack",
  "stack-stop": "Stopping the stack",
  "stack-restart": "Restarting the stack",
  "cameras-start": "Starting cameras",
  "cameras-stop": "Stopping cameras",
  "perception-start": "Starting perception",
  "perception-stop": "Stopping perception",
  "drive-start": "Starting drive",
  "stop-all": "Stopping everything",
};

const DONE: Record<OpKind, string> = {
  "stack-start": "Stack is up and ready",
  "stack-stop": "Stack stopped, bag closed",
  "stack-restart": "Stack restarted and ready",
  "cameras-start": "Cameras are on",
  "cameras-stop": "Cameras are off",
  "perception-start": "Perception is on",
  "perception-stop": "Perception is off",
  "drive-start": "Drive is running",
  "stop-all": "Car reports nothing driving",
};

/** The toast for a confirmed operation: the car itself reported the target state. */
export function doneText(kind: OpKind): string {
  return DONE[kind];
}

// -- what the snapshot says -----------------------------------------------------------------

function safetyGreen(s: Status): boolean {
  return s.checks.checks.find((c) => c.id === "safety")?.level === "green";
}

function failingRequired(s: Status): string[] {
  return s.checks.checks.filter((c) => c.required && c.level === "red").map((c) => c.label);
}

function driveMatches(s: Status, mode: string | undefined): boolean {
  const m = s.drive.mode;
  if (!m) return false;
  return !mode || m === mode || m.startsWith(mode);
}

type StepDef = { id: string; label: string; done: (snap: Snapshot, op: Op) => boolean };

const requested: StepDef = { id: "requested", label: "Requested", done: () => true };

/** A restart is atomic on the agent, so only a poll made after its reply shows the new stack. */
function restartSettled(n: Snapshot, op: Op): boolean {
  return op.sawDown || (op.acked && (n.seq ?? 0) >= (op.ackSeq ?? 0) + 2);
}

function startSteps(): StepDef[] {
  return [
    requested,
    {
      id: "container",
      label: "Container up",
      done: (n, op) => !!n.status?.stack.running && (op.kind !== "stack-restart" || restartSettled(n, op)),
    },
    {
      id: "nodes",
      label: "Nodes up",
      done: (n) => !!n.status && safetyGreen(n.status) && (n.status.ros_connected || n.linkConnected),
    },
    { id: "recorder", label: "Recorder up", done: (n) => !!n.status?.stack.recorder },
    { id: "ready", label: "Ready", done: (n) => !!n.status?.checks.ready },
  ];
}

const DEFS: Record<OpKind, StepDef[]> = {
  "stack-start": startSteps(),
  "stack-restart": [
    requested,
    { id: "closed", label: "Bag closed", done: (n, op) => restartSettled(n, op) },
    ...startSteps().slice(1),
  ],
  "stack-stop": [
    requested,
    { id: "closing", label: "Closing the bag", done: (n) => !!n.status && !n.status.stack.running },
    { id: "stopped", label: "Stopped", done: (n) => !!n.status && !n.status.stack.running && !n.status.stack.recorder },
  ],
  "cameras-start": [requested, { id: "cameras", label: "Cameras up", done: (n) => !!n.status?.cameras.running }],
  "cameras-stop": [requested, { id: "cameras", label: "Cameras stopped", done: (n) => !!n.status && !n.status.cameras.running }],
  "perception-start": [
    requested,
    { id: "perception", label: "Perception up", done: (n) => !!n.status?.perception.running },
  ],
  "perception-stop": [
    requested,
    { id: "perception", label: "Perception stopped", done: (n) => !!n.status && !n.status.perception.running },
  ],
  "drive-start": [
    requested,
    { id: "accepted", label: "Accepted", done: (n, op) => op.acked || (!!n.status && driveMatches(n.status, op.target)) },
    { id: "running", label: "Running", done: (n, op) => !!n.status && driveMatches(n.status, op.target) },
  ],
  "stop-all": [
    requested,
    {
      id: "stopped",
      label: "Stopped",
      done: (n) => !!n.status && !n.status.drive.mode && !(n.status.drive.external?.length ?? 0),
    },
  ],
};

/** The timeline for the view: every step with its state. */
export function opSteps(op: Op, snap: Snapshot): Step[] {
  const defs = DEFS[op.kind];
  const done = defs.map((d) => d.done(snap, op));
  const finished = op.phase === "confirmed";
  const firstUnmet = done.findIndex((d) => !d);
  return defs.map((d, i) => {
    let state: StepState;
    if (finished || done[i]) state = "done";
    else if (i === firstUnmet) state = op.phase === "failed" || op.phase === "timeout" ? "failed" : "active";
    else state = "todo";
    return { id: d.id, label: d.label, state };
  });
}

function reached(op: Op, snap: Snapshot): boolean {
  return DEFS[op.kind].every((d) => d.done(snap, op));
}

function unmetReason(op: Op, snap: Snapshot): string {
  const defs = DEFS[op.kind];
  const unmet = defs.find((d) => !d.done(snap, op));
  const seconds = Math.round(op.timeoutMs / 1000);
  const when = seconds >= 120 ? `${Math.round(seconds / 60)} min` : `${seconds} s`;
  if (!snap.status) return `No status from the car after ${when}.`;
  let tail = "";
  if (unmet?.id === "ready") {
    const bad = failingRequired(snap.status);
    if (bad.length) tail = ` Not ready: ${bad.join(", ")}.`;
  }
  return `Not done after ${when}: waiting for "${(unmet?.label ?? "target").toLowerCase()}".${tail}`;
}

// -- the store --------------------------------------------------------------------------------

export interface BeginSpec {
  kind: OpKind;
  title?: string;
  target?: string;
}

export class OpsStore {
  private list: Op[] = [];
  private nextId = 1;
  private lastSeq = 0;
  private listeners = new Set<() => void>();

  constructor(
    private now: () => number = () => Date.now(),
    /** How long a finished operation stays visible before it is dropped. */
    private keepMs = 6000,
  ) {}

  subscribe(cb: () => void): () => void {
    this.listeners.add(cb);
    return () => this.listeners.delete(cb);
  }

  private emit(): void {
    this.listeners.forEach((cb) => cb());
  }

  all(): Op[] {
    return this.list;
  }

  /** Operations still in flight (requesting or waiting). */
  active(): Op[] {
    return this.list.filter((o) => o.phase === "requesting" || o.phase === "waiting");
  }

  activeOf(...kinds: OpKind[]): Op | undefined {
    return this.active().find((o) => kinds.includes(o.kind));
  }

  /**
   * Show the target state now and send the request. Returns null (and sends nothing) when the
   * same kind is already in flight, so a double click can never double-send.
   */
  begin(spec: BeginSpec, request: () => Promise<unknown>): Op | null {
    if (this.activeOf(spec.kind)) return null;
    const op: Op = {
      id: this.nextId++,
      kind: spec.kind,
      title: spec.title ?? TITLES[spec.kind],
      target: spec.target,
      startedAt: this.now(),
      timeoutMs: TIMEOUT_MS[spec.kind],
      phase: "requesting",
      acked: false,
      sawDown: false,
    };
    // An older finished op of the same kind is replaced by the new one.
    this.list = [...this.list.filter((o) => o.kind !== spec.kind || o.phase === "requesting" || o.phase === "waiting"), op];
    this.emit();
    request().then(
      () => this.ack(op.id),
      (e: unknown) => this.fail(op.id, e instanceof Error ? e.message : String(e)),
    );
    return op;
  }

  private patch(id: number, f: (o: Op) => Op): void {
    let changed = false;
    this.list = this.list.map((o) => {
      if (o.id !== id) return o;
      const n = f(o);
      if (n !== o) changed = true;
      return n;
    });
    if (changed) this.emit();
  }

  private ack(id: number): void {
    const seq = this.lastSeq;
    this.patch(id, (o) =>
      o.phase === "requesting"
        ? { ...o, acked: true, ackSeq: seq, phase: "waiting" }
        : { ...o, acked: true, ackSeq: seq },
    );
  }

  fail(id: number, message: string): void {
    this.patch(id, (o) =>
      o.phase === "requesting" || o.phase === "waiting"
        ? { ...o, phase: "failed", error: message, finishedAt: this.now() }
        : o,
    );
  }

  dismiss(id: number): void {
    const before = this.list.length;
    this.list = this.list.filter((o) => o.id !== id);
    if (this.list.length !== before) this.emit();
  }

  /** Feed a fresh snapshot (status poll or ROS link change): confirm, time out, expire. */
  tick(snap: Snapshot): void {
    const now = this.now();
    this.lastSeq = snap.seq ?? this.lastSeq;
    let changed = false;
    const next: Op[] = [];
    for (const o of this.list) {
      let op = o;
      if (op.phase === "requesting" || op.phase === "waiting") {
        if (op.kind === "stack-restart" && !op.sawDown && snap.status && !snap.status.stack.running) {
          op = { ...op, sawDown: true };
        }
        if (reached(op, snap)) {
          op = { ...op, phase: "confirmed", finishedAt: now };
        } else if (now - op.startedAt > op.timeoutMs) {
          op = { ...op, phase: "timeout", error: unmetReason(op, snap), finishedAt: now };
        }
      } else if (op.phase === "confirmed" && op.finishedAt !== undefined && now - op.finishedAt > this.keepMs) {
        changed = true;
        continue; // drop it; failures stay until dismissed
      }
      if (op !== o) changed = true;
      next.push(op);
    }
    if (changed) {
      this.list = next;
      this.emit();
    }
  }
}
