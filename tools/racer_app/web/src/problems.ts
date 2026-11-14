// Problems: everything that went wrong, collected in one list for the Logs tab. The main tabs
// only show a small red badge with the count, which links here. Pure (the store takes a clock)
// so problems.test.ts covers it.

import type { LogEntry, Status } from "./api";

export interface Problem {
  /** Stable key, so the same problem is not listed twice. */
  key: string;
  /** Date.now() when it was seen. */
  at: number;
  title: string;
  detail: string;
  /** "live": a check that is red right now (goes away by itself). "event": something that failed. */
  kind: "live" | "event";
}

/** Failures this app saw: a refused or failed request, an operation that timed out. */
export class ProblemStore {
  private list: Problem[] = [];
  private seenAt = 0;
  private listeners = new Set<() => void>();

  constructor(
    private now: () => number = () => Date.now(),
    private max = 50,
  ) {}

  subscribe(cb: () => void): () => void {
    this.listeners.add(cb);
    return () => this.listeners.delete(cb);
  }

  private emit(): void {
    this.listeners.forEach((cb) => cb());
  }

  add(title: string, detail: string, key?: string): void {
    const at = this.now();
    const k = key ?? `${title}|${detail}|${at}`;
    if (this.list.some((p) => p.key === k)) return;
    this.list = [{ key: k, at, title, detail, kind: "event" as const }, ...this.list].slice(0, this.max);
    this.emit();
  }

  all(): Problem[] {
    return this.list;
  }

  /** Problems newer than the last "mark as seen". */
  unseen(): Problem[] {
    return this.list.filter((p) => p.at > this.seenAt);
  }

  markSeen(): void {
    this.seenAt = this.now();
    this.emit();
  }

  clear(): void {
    this.list = [];
    this.seenAt = this.now();
    this.emit();
  }
}

/**
 * Checks that are red right now and matter: only while the stack runs (with the stack off, its
 * own checks are red by design, and that is a state, not a problem).
 */
export function liveProblems(status: Status | null, now: number): Problem[] {
  if (!status?.stack.running) return [];
  return status.checks.checks
    .filter((c) => c.required && c.level === "red")
    .map((c) => ({ key: `check:${c.id}`, at: now, title: c.label, detail: c.detail, kind: "live" as const }));
}

/** Failed actions in the agent's log since `since` (seconds), newest first. */
export function failedActions(log: LogEntry[], since: number): Problem[] {
  return log
    .filter((e) => !e.ok && e.ts >= since)
    .map((e) => ({ key: `log:${e.ts}:${e.action}`, at: e.ts * 1000, title: e.action, detail: e.detail, kind: "event" as const }));
}

/** The number on the red badge: live problems plus unseen events. */
export function badgeCount(live: Problem[], unseen: Problem[]): number {
  return live.length + unseen.length;
}
