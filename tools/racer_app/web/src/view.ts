// Pure view helpers: what each card and pill shows, derived from the agent's status, the live
// ROS values and the in-flight operations. No DOM here, so view.test.ts covers it.

import type { BagInfo, Status } from "./api";
import { batteryReading, fmtBagSize, fmtGrowth, type Level } from "./model";
import type { Op, OpsStore } from "./ops";

// -- stale values -------------------------------------------------------------------------------

export const STALE_MS = 2500;

export interface Freshness<T> {
  value: T | null;
  /** Seconds since it arrived, null if it never has. */
  ageS: number | null;
  stale: boolean;
}

/** The last value with its age. Stale values are kept (greyed out), not blanked. */
export function freshness<T>(s: { value: T; at: number } | null, now: number, staleMs = STALE_MS): Freshness<T> {
  if (!s) return { value: null, ageS: null, stale: true };
  const ageMs = Math.max(0, now - s.at);
  return { value: s.value, ageS: ageMs / 1000, stale: ageMs >= staleMs };
}

/** "3 s ago", "2 min ago", "no data". */
export function ageLabel(ageS: number | null): string {
  if (ageS === null) return "no data";
  if (ageS < 1.5) return "live";
  if (ageS < 60) return `${Math.round(ageS)} s ago`;
  if (ageS < 3600) return `${Math.round(ageS / 60)} min ago`;
  return `${Math.round(ageS / 3600)} h ago`;
}

// -- services (stack, cameras, perception) -----------------------------------------------------

export type ServiceState = "off" | "starting" | "on" | "stopping";

/**
 * The state to show: an in-flight start or stop wins over the polled value (that is the
 * optimistic update), and the poll takes over the moment the operation is done.
 */
export function serviceState(running: boolean, store: OpsStore, startKinds: Op["kind"][], stopKinds: Op["kind"][]): ServiceState {
  if (store.activeOf(...stopKinds)) return "stopping";
  if (store.activeOf(...startKinds)) return "starting";
  return running ? "on" : "off";
}

export function stackState(status: Status | null, store: OpsStore): ServiceState | "restarting" {
  if (store.activeOf("stack-restart")) return "restarting";
  return serviceState(status?.stack.running ?? false, store, ["stack-start"], ["stack-stop"]);
}

export interface DriveView {
  state: "idle" | "starting" | "running" | "stopping";
  /** The mode that is running, or being started. */
  mode: string | null;
}

export function driveState(status: Status | null, store: OpsStore): DriveView {
  const starting = store.activeOf("drive-start");
  const stopping = store.activeOf("stop-all");
  const running = status?.drive.mode ?? null;
  if (stopping && (running || starting)) return { state: "stopping", mode: running ?? starting?.target ?? null };
  if (starting && !running) return { state: "starting", mode: starting.target ?? null };
  if (running) return { state: "running", mode: running };
  return { state: "idle", mode: null };
}

/** Does a running mode (for example "park" or "gap_follow") belong to this card? */
export function modeMatches(running: string | null, card: string): boolean {
  return !!running && (running === card || running.startsWith(card));
}

/**
 * Why a drive card cannot start right now, in a few words, or "" when it can. The agent still
 * checks everything again (and the override flow is unchanged): this only explains the button.
 */
export function driveBlocked(status: Status | null, store: OpsStore, agentUp: boolean): string {
  if (!agentUp) return "Car offline";
  if (!status) return "Loading";
  const stack = stackState(status, store);
  if (stack === "starting" || stack === "restarting") return "Stack starting";
  if (stack === "stopping") return "Stack stopping";
  if (stack !== "on") return "Start the stack first";
  const d = driveState(status, store);
  if (d.state === "starting") return "Another drive is starting";
  if (d.state === "stopping") return "Stopping";
  if (d.state === "running") return `${d.mode} is running`;
  const ext = status.drive.external ?? [];
  if (ext.length) return `${ext[0].mode} is running from a terminal`;
  return "";
}

// -- pills -------------------------------------------------------------------------------------

export interface PillView {
  level: Level | "neutral" | "busy";
  text: string;
  title?: string;
}

export function readyPill(status: Status | null): PillView {
  if (!status) return { level: "neutral", text: "Checking" };
  if (status.checks.ready) return { level: status.checks.overall === "amber" ? "amber" : "green", text: "Ready" };
  const bad = status.checks.checks.filter((c) => c.required && c.level === "red");
  return {
    level: "red",
    text: bad.length ? `Not ready (${bad.length})` : "Not ready",
    title: bad.map((c) => c.label).join(", "),
  };
}

export function recordingPill(bags: BagInfo[], status: Status | null): PillView {
  const bag = bags.find((b) => b.active);
  if (!bag) {
    if (status?.stack.running && !status.stack.recorder) return { level: "red", text: "Not recording" };
    return { level: "neutral", text: "Not recording" };
  }
  return {
    level: "red",
    text: `REC ${fmtBagSize(bag.size_mb)} ${fmtGrowth(bag.growth_mb_s)}`,
    title: bag.name,
  };
}

export function batteryPill(volts: Freshness<number>, cells: number): PillView {
  if (volts.value === null) return { level: "neutral", text: "No battery data" };
  const b = batteryReading(volts.value, { cells, cellGreenV: 3.7, cellRedV: 3.5 });
  return {
    level: volts.stale ? "neutral" : b.level,
    text: `${volts.value.toFixed(1)} V${volts.stale ? ` (${ageLabel(volts.ageS)})` : ""}`,
    title: b.perCell === null ? undefined : `${b.perCell.toFixed(2)} V per cell`,
  };
}

export type ConnLabel = "connected" | "reconnecting" | "live-down";

/** The ROS link only counts once the stack is up: with the stack off it is meant to be down. */
export function connectionState(agentUp: boolean, linkUp: boolean, stackRunning: boolean): ConnLabel {
  if (!agentUp) return "reconnecting";
  return linkUp || !stackRunning ? "connected" : "live-down";
}

export function elapsedLabel(ms: number): string {
  const s = Math.max(0, Math.round(ms / 1000));
  if (s < 60) return `${s} s`;
  return `${Math.floor(s / 60)} min ${String(s % 60).padStart(2, "0")} s`;
}

const MODE_NAMES: Record<string, string> = {
  gap_follow: "Gap follow",
  pnp: "ML driver",
  tracker: "Racer",
  ml_policy: "ML policy",
  wall_follow: "Wall follow",
  track_follow: "Track follow",
  park: "Park",
  spiderbot: "Spiderbot",
};

/** "gap_follow:lane-2026-10-25 (started from a terminal)" becomes "Gap follow". */
export function modeLabel(mode: string): string {
  const base = mode.replace(/ \(started from a terminal\)$/, "").split(":")[0];
  return MODE_NAMES[base] ?? base;
}
