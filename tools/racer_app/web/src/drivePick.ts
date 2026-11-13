// What the driving cards send, worked out in one place so it can be tested: which bundle is
// selected, the planner file, and the gap follow max speed. The agent re-checks all of it; this
// only keeps the UI from sending a request it knows will be refused, and makes the choice visible.

/**
 * The bundle that is selected: the user's pick if it is still on offer, else the only one when
 * there is exactly one, else nothing (the card says to choose). Never an empty string passed on
 * as "default": the request always names the bundle.
 */
export function selectedBundle(picked: string, offered: readonly string[] | undefined): string {
  const list = offered ?? [];
  if (list.includes(picked)) return picked;
  return list.length === 1 ? list[0] : "";
}

/** The planner params file, or "" for the launch's own defaults. Only a file on offer counts. */
export function selectedParamsFile(picked: string, offered: readonly string[] | undefined): string {
  return (offered ?? []).includes(picked) ? picked : "";
}

/** One line saying exactly what the Plug-and-Play button will launch with. */
export function pnpSelectionText(p: {
  backend: string;
  dynamics: string;
  perception: string;
  paramsFile: string;
  speed: number;
}): string {
  const parts = [
    `dynamics bundle: ${p.dynamics || "NONE selected"}`,
    ...(p.backend === "net" ? [`perception bundle: ${p.perception || "NONE selected"}`] : []),
    `planner file: ${p.paramsFile || "default (none)"}`,
    `speed cap ${p.speed.toFixed(2)} m/s`,
  ];
  return `Will launch with ${parts.join(", ")}.`;
}

export interface SpeedBounds {
  min: number;
  max: number;
}

/**
 * The gap follow max speed box. Empty means "the profile's own speed" (no argument sent). A
 * number must lie inside the bounds the agent reported for the profile (its ceiling and
 * vehicle_params' cap). Returns what to send and why not, if it cannot be sent.
 */
export function gapSpeed(
  text: string,
  bounds: SpeedBounds | undefined,
): { value: number | undefined; error: string } {
  const t = text.trim();
  if (t === "") return { value: undefined, error: "" };
  const n = Number(t);
  if (!Number.isFinite(n)) return { value: undefined, error: "max speed is not a number" };
  if (bounds && (n < bounds.min || n > bounds.max)) {
    return {
      value: undefined,
      error: `max speed must be between ${bounds.min} and ${bounds.max} m/s for this profile`,
    };
  }
  return { value: n, error: "" };
}

/**
 * The gap follow min speed box. Empty means "the profile's own floor" (0.36, no argument sent).
 * A number must lie in [0, the max speed in effect]: the max box's value when it holds a valid
 * number, else the profile's max from the agent. The node refuses min above max. Returns what to
 * send and why not, if it cannot be sent.
 */
export function gapMinSpeed(
  text: string,
  bounds: SpeedBounds | undefined,
  maxSet: number | undefined,
): { value: number | undefined; error: string } {
  const t = text.trim();
  if (t === "") return { value: undefined, error: "" };
  const n = Number(t);
  if (!Number.isFinite(n)) return { value: undefined, error: "min speed is not a number" };
  const ceiling = maxSet ?? bounds?.max;
  if (n < 0 || (ceiling !== undefined && n > ceiling)) {
    const why = maxSet !== undefined ? "the max speed set here" : "this profile's max speed";
    return {
      value: undefined,
      error:
        ceiling === undefined
          ? "min speed cannot be negative"
          : `min speed must be between 0 and ${ceiling} m/s (it cannot exceed ${why})`,
    };
  }
  return { value: n, error: "" };
}

/** The warning for a driving launch the app did not start, or null. */
export function externalDriveText(
  external: ReadonlyArray<{ pid: number; mode: string }> | undefined,
): string | null {
  if (!external || external.length === 0) return null;
  const list = external.map((e) => `${e.mode} (pid ${e.pid})`).join(", ");
  return `Running from a terminal, not started by this app: ${list}. STOP ALL stops it too (SIGINT, like Ctrl-C).`;
}
