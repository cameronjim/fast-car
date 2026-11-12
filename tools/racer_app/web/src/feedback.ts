// The feedback machinery behind every click: a per-button action runner (idle, pending, success,
// error), a toast stack, and the connection tracker that decides when to show "reconnecting".
// Framework-free so feedback.test.ts can drive it with fake timers.

import { ApiError } from "./api";

// -- action runner (one per button) ---------------------------------------------------------------

export type ActionPhase = "idle" | "pending" | "success" | "error";

export interface ActionState {
  phase: ActionPhase;
  /** Short reason, set in the error phase. */
  error: string;
}

export interface RunnerOptions {
  /** How long the check mark stays. */
  successMs?: number;
  /** How long an inline error stays (0 keeps it until the next run). */
  errorMs?: number;
  /** Allow a new run while one is pending (STOP ALL must always be clickable). */
  reentrant?: boolean;
}

export type Outcome = { ok: true; message: string } | { ok: false; error: ApiError };

/** What the owner reads when a request fails: the agent's reason, or a plain network line. */
export function errorText(e: unknown): string {
  if (e instanceof ApiError) return e.status === 0 ? "Cannot reach the car" : e.message;
  return e instanceof Error ? e.message : String(e);
}

export function toApiError(e: unknown): ApiError {
  return e instanceof ApiError ? e : new ApiError(e instanceof Error ? e.message : String(e), 0, null);
}

export class ActionRunner {
  private s: ActionState = { phase: "idle", error: "" };
  private listeners = new Set<() => void>();
  private timer: ReturnType<typeof setTimeout> | null = null;
  private runId = 0;

  constructor(private opts: RunnerOptions = {}) {}

  get state(): ActionState {
    return this.s;
  }

  subscribe(cb: () => void): () => void {
    this.listeners.add(cb);
    return () => this.listeners.delete(cb);
  }

  private set(s: ActionState): void {
    this.s = s;
    this.listeners.forEach((cb) => cb());
  }

  private clearTimer(): void {
    if (this.timer) clearTimeout(this.timer);
    this.timer = null;
  }

  /**
   * Run `fn`. A second call while pending is ignored (returns null), so a double click cannot
   * send twice. Never throws: the outcome says what happened.
   */
  async run(fn: () => Promise<{ message?: string } | void>): Promise<Outcome | null> {
    if (this.s.phase === "pending" && !this.opts.reentrant) return null;
    this.clearTimer();
    const id = ++this.runId;
    this.set({ phase: "pending", error: "" });
    try {
      const r = await fn();
      if (id === this.runId) {
        this.set({ phase: "success", error: "" });
        this.timer = setTimeout(() => this.reset(), this.opts.successMs ?? 1500);
      }
      return { ok: true, message: (r && r.message) || "Done" };
    } catch (e) {
      const err = toApiError(e);
      if (id === this.runId) {
        this.set({ phase: "error", error: errorText(e) });
        const ms = this.opts.errorMs ?? 0;
        if (ms > 0) this.timer = setTimeout(() => this.reset(), ms);
      }
      return { ok: false, error: err };
    }
  }

  reset(): void {
    this.clearTimer();
    if (this.s.phase !== "pending") this.set({ phase: "idle", error: "" });
  }

  dispose(): void {
    this.clearTimer();
    this.listeners.clear();
  }
}

// -- toasts -------------------------------------------------------------------------------------

export type ToastKind = "ok" | "error" | "info";

export interface Toast {
  id: number;
  kind: ToastKind;
  text: string;
}

export class ToastStore {
  private list: Toast[] = [];
  private nextId = 1;
  private timers = new Map<number, ReturnType<typeof setTimeout>>();
  private listeners = new Set<() => void>();

  constructor(private max = 4) {}

  subscribe(cb: () => void): () => void {
    this.listeners.add(cb);
    return () => this.listeners.delete(cb);
  }

  all(): Toast[] {
    return this.list;
  }

  private emit(): void {
    this.listeners.forEach((cb) => cb());
  }

  push(text: string, kind: ToastKind = "ok"): number {
    // The same message again refreshes the existing toast instead of stacking duplicates.
    const dup = this.list.find((t) => t.text === text && t.kind === kind);
    if (dup) {
      this.arm(dup.id, kind);
      return dup.id;
    }
    const id = this.nextId++;
    this.list = [...this.list, { id, kind, text }].slice(-this.max);
    this.arm(id, kind);
    this.emit();
    return id;
  }

  private arm(id: number, kind: ToastKind): void {
    const old = this.timers.get(id);
    if (old) clearTimeout(old);
    this.timers.set(id, setTimeout(() => this.dismiss(id), kind === "error" ? 9000 : 3500));
  }

  dismiss(id: number): void {
    const t = this.timers.get(id);
    if (t) clearTimeout(t);
    this.timers.delete(id);
    const before = this.list.length;
    this.list = this.list.filter((x) => x.id !== id);
    if (this.list.length !== before) this.emit();
  }
}

// -- connection tracker ---------------------------------------------------------------------------

export interface ConnState {
  /** false after `failuresToDown` failed polls in a row. */
  up: boolean;
  /** Date.now() of the last good reply, null if there has not been one. */
  lastSeen: number | null;
  failures: number;
}

/**
 * Tracks one link (the agent, or the ROS socket). One failed poll is a blip and is ignored; a
 * second in a row flips to down, so the "reconnecting" banner does not flicker on a slow reply.
 */
export class ConnectionTracker {
  private s: ConnState = { up: true, lastSeen: null, failures: 0 };

  constructor(private failuresToDown = 2) {}

  get state(): ConnState {
    return this.s;
  }

  /** Returns "reconnected" when this success ends an outage, "lost" when this failure starts one. */
  ok(now: number): "reconnected" | null {
    const was = this.s;
    this.s = { up: true, lastSeen: now, failures: 0 };
    return !was.up ? "reconnected" : null;
  }

  fail(): "lost" | null {
    const failures = this.s.failures + 1;
    const up = failures < this.failuresToDown;
    const lost = this.s.up && !up;
    this.s = { ...this.s, failures, up };
    return lost ? "lost" : null;
  }

  /** Mirror a link that reports its own state (the foxglove socket). */
  set(connected: boolean, now: number): "reconnected" | "lost" | null {
    if (connected) return this.ok(now);
    const was = this.s.up;
    this.s = { ...this.s, up: false, failures: this.failuresToDown };
    return was ? "lost" : null;
  }
}

/** "last seen 12 s ago", or "not seen yet". */
export function lastSeenText(lastSeen: number | null, now: number): string {
  if (lastSeen === null) return "not seen yet";
  const s = Math.max(0, Math.round((now - lastSeen) / 1000));
  if (s < 60) return `last seen ${s} s ago`;
  return `last seen ${Math.round(s / 60)} min ago`;
}
