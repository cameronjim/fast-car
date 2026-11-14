// The saved drive settings (Tune tab) and what the Drive buttons send from them, worked out in
// one place so it can be tested. The agent checks every value again on save and on every drive
// start; this only keeps the UI from saving or sending something it knows will be refused.

import type { LapMode, LapSource, Options, Prefs, Status } from "./api";
import { selectedBundle, selectedParamsFile, type SpeedBounds } from "./drivePick";

/** The agent's defaults, used until the first GET /api/prefs answers. */
export const DEFAULT_PREFS: Prefs = {
  record_camera: true,
  stack: { mode: "teleop", map: "" },
  gap_follow: {
    profile: "lane-2026-10-25",
    max_speed_mps: null,
    min_speed_mps: null,
    a_lat_max_mps2: null,
    side_cap_min_speed_mps: null,
  },
  pnp: {
    v_max_mps: 0.5,
    backend: "geometric",
    dynamics: "",
    perception: "",
    params_file: "",
    v_lap1_mps: null,
    lap_memory: false,
    lap_mode: "v0",
    lap_source: "start_scan",
    lap_counter: true,
    require_lap_counter: false,
    shadow_perception: "",
  },
  tracker: { raceline: "" },
};

/** gap_follow_node's own defaults, shown where a value is left to the profile. */
export const GAP_NODE_DEFAULTS = { min_speed_mps: 0.36, a_lat_max_mps2: 2.0, side_cap_min_speed_mps: 0.36 };
/** physics_speed's lateral acceleration range (the node's declared range). */
export const A_LAT_RANGE: [number, number] = [0.1, 20];

export type GapField = "max_speed_mps" | "min_speed_mps" | "a_lat_max_mps2" | "side_cap_min_speed_mps";

/** Per field: the reason it cannot be saved, or nothing. */
export type FieldErrors<K extends string> = Partial<Record<K, string>>;

const num = (v: number | null): v is number => v !== null && Number.isFinite(v);

/** The bounds the agent enforces for gap follow, given the profile's max speed bounds. */
export function gapErrors(g: Prefs["gap_follow"], bounds: SpeedBounds | undefined): FieldErrors<GapField> {
  const e: FieldErrors<GapField> = {};
  if (num(g.max_speed_mps) && bounds && (g.max_speed_mps < bounds.min || g.max_speed_mps > bounds.max)) {
    e.max_speed_mps = `${bounds.min} to ${bounds.max}`;
  }
  const maxOk = num(g.max_speed_mps) && !e.max_speed_mps;
  const ceiling = maxOk ? (g.max_speed_mps as number) : bounds?.max;
  for (const f of ["min_speed_mps", "side_cap_min_speed_mps"] as const) {
    const v = g[f];
    if (num(v) && (v < 0 || (ceiling !== undefined && v > ceiling))) e[f] = `0 to ${ceiling ?? "max"}`;
  }
  const a = g.a_lat_max_mps2;
  if (num(a) && (a < A_LAT_RANGE[0] || a > A_LAT_RANGE[1])) e.a_lat_max_mps2 = `${A_LAT_RANGE[0]} to ${A_LAT_RANGE[1]}`;
  return e;
}

export type PnpField =
  | "v_max_mps"
  | "v_lap1_mps"
  | "lap_mode"
  | "lap_source"
  | "require_lap_counter"
  | "shadow_perception";

/** The lap learning choice the card shows: off, observe, D v0, D v1. */
export type LapChoice = "off" | LapMode;

export const LAP_CHOICES: { id: LapChoice; label: string }[] = [
  { id: "off", label: "off" },
  { id: "observe", label: "observe" },
  { id: "v0", label: "D v0" },
  { id: "v1", label: "D v1" },
];

/** Observe is lap memory in observe mode (the launch is given lap_memory:=true as well). */
export function lapChoice(p: Prefs["pnp"]): LapChoice {
  if (p.lap_mode === "observe") return "observe";
  return p.lap_memory ? p.lap_mode : "off";
}

/** The saved fields for a lap learning choice (off goes back to D v0 so no stale mode stays). */
export function lapChoicePatch(c: LapChoice): Pick<Prefs["pnp"], "lap_memory" | "lap_mode"> {
  return c === "off" ? { lap_memory: false, lap_mode: "v0" } : { lap_memory: true, lap_mode: c };
}

/** Whether the car's launch file offers a lap mode / lap source. An agent that does not report
 * the lists (older build) is taken to offer them: it checks again when the drive starts. */
export function lapModeOffered(options: Options | null, mode: LapMode): boolean {
  const list = options?.pnp?.lap_modes;
  return !list || list.includes(mode);
}
export function lapSourceOffered(options: Options | null, source: LapSource): boolean {
  const list = options?.pnp?.lap_sources;
  return !list || list.includes(source);
}

/** The driving perception bundle that will be sent (net backend only), or "". */
function drivingPerception(p: Prefs["pnp"], options: Options | null): string {
  return p.backend === "net" ? selectedBundle(p.perception, options?.pnp?.perception) : "";
}

/**
 * Per field: why the ML driver settings cannot be saved or started, or nothing. The wording of
 * the lap rules is the agent's, so the same refusal reads the same in the card and from the car.
 */
export function pnpErrors(p: Prefs["pnp"], options: Options | null): FieldErrors<PnpField> {
  const e: FieldErrors<PnpField> = {};
  const range = options?.pnp?.speed_range ?? [0.2, 1.5];
  if (!Number.isFinite(p.v_max_mps) || p.v_max_mps < range[0] || p.v_max_mps > range[1]) {
    e.v_max_mps = `${range[0]} to ${range[1]}`;
  }
  const on = lapChoice(p) !== "off";
  if (p.lap_mode !== "v0" && !p.lap_memory && p.lap_mode !== "observe") {
    e.lap_mode = `pnp lap mode ${p.lap_mode} needs lap memory on`;
  } else if (on && !lapModeOffered(options, p.lap_mode)) {
    e.lap_mode = `lap_mode=${p.lap_mode} is not available in this build`;
  }
  if (p.lap_mode === "v1" && p.lap_source !== "start_scan") {
    e.lap_source = "pnp lap mode v1 needs lap source start_scan";
  } else if (on && !lapSourceOffered(options, p.lap_source)) {
    e.lap_source = `lap_source=${p.lap_source} is not available in this build`;
  }
  const cap = p.v_lap1_mps;
  if (cap !== null) {
    if (!on) {
      e.v_lap1_mps = "pnp lap 1 cap needs lap memory on or lap mode observe";
    } else if (!Number.isFinite(cap) || cap < range[0] || cap > range[1]) {
      e.v_lap1_mps = `pnp lap 1 cap must be between ${range[0]} and ${range[1]} m/s`;
    } else if (cap > p.v_max_mps) {
      e.v_lap1_mps = "pnp lap 1 cap cannot exceed the speed cap";
    }
  }
  if (p.require_lap_counter && !p.lap_counter) {
    e.require_lap_counter = "require lap counter needs the lap counter on";
  }
  const driving = drivingPerception(p, options);
  if (p.shadow_perception && driving && p.shadow_perception === driving) {
    e.shadow_perception = "the shadow perception bundle cannot be the driving one";
  }
  return e;
}

/** Every reason the settings cannot be saved, flattened, or [] when they can. */
export function prefsProblems(p: Prefs, options: Options | null): string[] {
  const bounds = options?.gap_follow_speed?.[p.gap_follow.profile];
  const out: string[] = [];
  for (const [k, v] of Object.entries(gapErrors(p.gap_follow, bounds))) out.push(`gap follow ${k}: ${v}`);
  for (const [k, v] of Object.entries(pnpErrors(p.pnp, options))) out.push(`ML driver ${k}: ${v}`);
  if (p.stack.mode === "localization" && !p.stack.map) out.push("start mode: choose a map");
  return out;
}

/** Turn a number box's text into a value: "" is null (the profile's own), junk is NaN. */
export function parseNum(text: string): number | null {
  const t = text.trim();
  if (t === "") return null;
  const n = Number(t);
  return Number.isFinite(n) ? n : Number.NaN;
}

const opt = (v: number | null): number | undefined => (v === null ? undefined : v);

/** What the Gap follow button sends. */
export function gapBody(p: Prefs) {
  const g = p.gap_follow;
  return {
    mode: "gap_follow",
    profile: g.profile,
    max_speed_mps: opt(g.max_speed_mps),
    min_speed_mps: opt(g.min_speed_mps),
    a_lat_max_mps2: opt(g.a_lat_max_mps2),
    side_cap_min_speed_mps: opt(g.side_cap_min_speed_mps),
  };
}

/**
 * What the ML driver (Plug-and-Play) button sends: the saved bundles if still on offer. The lap
 * learning fields are left out while they are at their default, so an unchanged card sends
 * today's request.
 */
export function pnpBody(p: Prefs, options: Options | null) {
  const n = p.pnp;
  const pnp = options?.pnp;
  const on = lapChoice(n) !== "off";
  const shadow = selectedParamsFile(n.shadow_perception, pnp?.perception);
  return {
    mode: "pnp",
    pnp_backend: n.backend,
    pnp_dynamics: selectedBundle(n.dynamics, pnp?.dynamics),
    pnp_perception: n.backend === "net" ? selectedBundle(n.perception, pnp?.perception) : "",
    pnp_params: selectedParamsFile(n.params_file, pnp?.params_files),
    pnp_v_max_mps: n.v_max_mps,
    pnp_v_lap1_mps: on && n.v_lap1_mps !== null ? n.v_lap1_mps : undefined,
    pnp_lap_memory: on ? true : undefined,
    pnp_lap_mode: on && n.lap_mode !== "v0" ? n.lap_mode : undefined,
    pnp_lap_source: on && n.lap_source !== "start_scan" ? n.lap_source : undefined,
    pnp_lap_counter: n.lap_counter ? undefined : false,
    pnp_require_lap_counter: n.require_lap_counter ? true : undefined,
    pnp_shadow_perception: shadow || undefined,
  };
}

export function trackerBody(p: Prefs) {
  return { mode: "tracker", raceline: p.tracker.raceline };
}

/** What Start car sends: LiDAR and VESC, in the saved start mode. */
export function stackBody(p: Prefs) {
  return {
    lidar: true,
    vesc: true,
    mapping: p.stack.mode === "mapping",
    localization: p.stack.mode === "localization",
    map: p.stack.mode === "localization" ? p.stack.map : "",
  };
}

/** Why a mode's own settings stop it from starting ("" when they do not). Few words. */
export function gapWhy(p: Prefs, options: Options | null): string {
  return Object.keys(gapErrors(p.gap_follow, options?.gap_follow_speed?.[p.gap_follow.profile])).length ? "Fix Tune values" : "";
}

export function pnpWhy(p: Prefs, options: Options | null): string {
  const pnp = options?.pnp;
  if (!pnp?.available) return pnp?.reason ? "Unavailable" : "Loading";
  const b = pnpBody(p, options);
  if (!b.pnp_dynamics) return "Pick a dynamics bundle in Tune";
  if (p.pnp.backend === "net" && !b.pnp_perception) return "Pick a perception bundle in Tune";
  if (Object.keys(pnpErrors(p.pnp, options)).length) return "Fix Tune values";
  return "";
}

export function trackerWhy(p: Prefs, status: Status | null): string {
  if (!p.tracker.raceline) return "Pick a raceline in Tune";
  if (status?.stack.running && status.stack.flags.localization !== "true") return "Needs a map: set Start mode in Tune";
  return "";
}

/** One short line under each Drive button: what it will run with. */
export function gapSummary(p: Prefs): string {
  const g = p.gap_follow;
  const speed = g.max_speed_mps === null ? "profile speed" : `${g.max_speed_mps.toFixed(2)} m/s`;
  return `${g.profile}, ${speed}`;
}

/** What the ML driver will run, from the same fields pnpBody sends. */
export function pnpSummary(p: Prefs, options: Options | null = null): string {
  const n = p.pnp;
  const choice = lapChoice(n);
  const parts = [n.backend, `cap ${n.v_max_mps.toFixed(2)}`];
  if (choice === "off") {
    parts.push("lap learning off");
  } else {
    const label = LAP_CHOICES.find((c) => c.id === choice)?.label ?? choice;
    parts.push(`${label} from ${(n.v_lap1_mps ?? n.v_max_mps).toFixed(2)}`);
    if (n.lap_source !== "start_scan") parts.push(`source ${n.lap_source}`);
  }
  parts.push(n.lap_counter ? (n.require_lap_counter ? "lap counter required" : "lap counter on") : "lap counter off");
  const shadow = pnpBody(p, options).pnp_shadow_perception;
  if (shadow) parts.push(`shadow ${shadow}`);
  return parts.join(", ");
}

export function trackerSummary(p: Prefs): string {
  return p.tracker.raceline || "no raceline";
}

/** Deep copy with one section patched. */
export function patch<K extends keyof Prefs>(p: Prefs, key: K, value: Partial<Prefs[K]> | Prefs[K]): Prefs {
  const cur = p[key];
  const next = typeof cur === "object" && cur !== null ? { ...(cur as object), ...(value as object) } : value;
  return { ...p, [key]: next } as Prefs;
}

export function samePrefs(a: Prefs, b: Prefs): boolean {
  return JSON.stringify(a) === JSON.stringify(b);
}
