// The Data tab's arithmetic: the disk bar, the run dates and the "copy to your computer" command.
// Pure, covered by data.test.ts.

import type { BagsReply } from "./api";

export interface DiskBar {
  /** Percent of the disk, each 0..100, summing to at most 100. */
  bagsPct: number;
  otherPct: number;
  freePct: number;
  totalGb: number | null;
  freeGb: number | null;
  bagsGb: number;
  level: "green" | "amber" | "red";
}

const clampPct = (x: number) => Math.max(0, Math.min(100, x));

/** Split the disk into runs, everything else, and free. Thresholds are the agent's (GiB free). */
export function diskBar(reply: BagsReply | null, redGb = 5, amberGb = 15): DiskBar {
  // sizes come in MB (10^6 bytes), the disk in GiB
  const bagsGb = (reply?.bags_mb ?? (reply?.bags ?? []).reduce((a, b) => a + b.size_mb, 0)) / 1073.741824;
  const total = reply?.disk?.total_gb ?? null;
  const free = reply?.disk?.free_gb ?? reply?.disk_free_gb ?? null;
  const level = free === null ? "amber" : free < redGb ? "red" : free < amberGb ? "amber" : "green";
  if (!total || free === null) return { bagsPct: 0, otherPct: 0, freePct: 0, totalGb: total, freeGb: free, bagsGb, level };
  const freePct = clampPct((free / total) * 100);
  const bagsPct = Math.min(clampPct((bagsGb / total) * 100), 100 - freePct);
  const otherPct = clampPct(100 - freePct - bagsPct);
  return { bagsPct, otherPct, freePct, totalGb: total, freeGb: free, bagsGb, level };
}

export function fmtGb(gb: number | null): string {
  if (gb === null) return "--";
  return gb >= 100 ? `${Math.round(gb)} GB` : `${gb.toFixed(1)} GB`;
}

/** "Thu 13 Nov, 15:02" from a unix time in seconds, in the viewer's time zone. */
export function fmtRunDate(mtimeS: number, locale?: string): string {
  const d = new Date(mtimeS * 1000);
  const day = d.toLocaleDateString(locale, { weekday: "short", day: "numeric", month: "short" });
  const time = d.toLocaleTimeString(locale, { hour: "2-digit", minute: "2-digit", hour12: false });
  return `${day}, ${time}`;
}

const SAFE = /^[A-Za-z0-9_.:+@/-]+$/;

/**
 * The command that copies one run to this computer, for pasting into a terminal. Only names that
 * are safe in a shell unquoted are offered (the agent's bag names always are); anything else gets
 * no command rather than a wrong one.
 */
export function copyCommand(user: string | undefined, host: string, bagDir: string | undefined, name: string): string | null {
  if (!bagDir || !host) return null;
  const who = user ? `${user}@${host}` : host;
  const path = `${bagDir.replace(/\/+$/, "")}/${name}`;
  if (![who, path].every((x) => SAFE.test(x))) return null;
  return `scp -r ${who}:${path} ~/Downloads/`;
}

/** The car's host name for the copy command: the address the app talks to. */
export function carHost(apiBase: string, pageHost: string | undefined): string {
  if (apiBase) {
    try {
      return new URL(apiBase).hostname;
    } catch {
      /* fall through */
    }
  }
  return pageHost ?? "";
}
