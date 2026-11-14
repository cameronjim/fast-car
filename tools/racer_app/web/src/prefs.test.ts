import { describe, expect, it } from "vitest";
import type { Options, Prefs, Status } from "./api";
import {
  DEFAULT_PREFS,
  gapBody,
  gapErrors,
  gapSummary,
  gapWhy,
  parseNum,
  patch,
  pnpBody,
  pnpErrors,
  pnpWhy,
  prefsProblems,
  samePrefs,
  stackBody,
  trackerBody,
  trackerWhy,
} from "./prefs";

const prefs = (over: Partial<Prefs> = {}): Prefs => ({ ...structuredClone(DEFAULT_PREFS), ...over });

const options = {
  gap_follow_profiles: ["lane-2026-10-25", "race-2026-11-13"],
  gap_follow_speed: { "lane-2026-10-25": { min: 0.36, max: 0.64 }, "race-2026-11-13": { min: 0.36, max: 1.3 } },
  pnp: {
    available: true,
    reason: "",
    perception: ["net-a"],
    dynamics: ["dyn-a", "dyn-b"],
    backends: ["geometric", "net"],
    speed_range: [0.2, 1.5],
    default_speed: 0.5,
    params_files: ["tight.yaml"],
  },
} as unknown as Options;

describe("defaults", () => {
  it("record the camera, leave gap follow to the profile, start plain", () => {
    expect(DEFAULT_PREFS.record_camera).toBe(true);
    expect(DEFAULT_PREFS.gap_follow.max_speed_mps).toBeNull();
    expect(DEFAULT_PREFS.stack.mode).toBe("teleop");
  });
});

describe("gap follow bounds", () => {
  const b = { min: 0.36, max: 1.3 };
  it("accepts nulls and values inside the bounds", () => {
    expect(gapErrors(DEFAULT_PREFS.gap_follow, b)).toEqual({});
    expect(gapErrors({ ...DEFAULT_PREFS.gap_follow, max_speed_mps: 1.0, min_speed_mps: 0.5, a_lat_max_mps2: 2.5, side_cap_min_speed_mps: 0.3 }, b)).toEqual({});
  });
  it("refuses a max outside the profile bounds", () => {
    expect(gapErrors({ ...DEFAULT_PREFS.gap_follow, max_speed_mps: 2 }, b).max_speed_mps).toBe("0.36 to 1.3");
  });
  it("checks min and the side cap against the max in effect", () => {
    const e = gapErrors({ ...DEFAULT_PREFS.gap_follow, max_speed_mps: 0.5, min_speed_mps: 0.6, side_cap_min_speed_mps: 0.6 }, b);
    expect(e.min_speed_mps).toBe("0 to 0.5");
    expect(e.side_cap_min_speed_mps).toBe("0 to 0.5");
    // with the max left to the profile, the profile's ceiling is the limit
    expect(gapErrors({ ...DEFAULT_PREFS.gap_follow, min_speed_mps: 1.2 }, b)).toEqual({});
    expect(gapErrors({ ...DEFAULT_PREFS.gap_follow, min_speed_mps: -0.1 }, b).min_speed_mps).toBeDefined();
  });
  it("keeps the lateral acceleration inside the node's range", () => {
    expect(gapErrors({ ...DEFAULT_PREFS.gap_follow, a_lat_max_mps2: 0.05 }, b).a_lat_max_mps2).toBe("0.1 to 20");
    expect(gapErrors({ ...DEFAULT_PREFS.gap_follow, a_lat_max_mps2: 21 }, b).a_lat_max_mps2).toBe("0.1 to 20");
  });
});

describe("ML driver bounds", () => {
  it("keeps the speed cap inside the agent's range", () => {
    expect(pnpErrors(DEFAULT_PREFS.pnp, options)).toEqual({});
    expect(pnpErrors({ ...DEFAULT_PREFS.pnp, v_max_mps: 1.6 }, options).v_max_mps).toBe("0.2 to 1.5");
    expect(pnpErrors({ ...DEFAULT_PREFS.pnp, v_max_mps: Number.NaN }, options).v_max_mps).toBeDefined();
  });
});

describe("prefsProblems", () => {
  it("lists every reason not to save", () => {
    expect(prefsProblems(prefs(), options)).toEqual([]);
    const bad = prefs({
      gap_follow: { ...DEFAULT_PREFS.gap_follow, max_speed_mps: 5 },
      stack: { mode: "localization", map: "" },
    });
    const list = prefsProblems(bad, options);
    expect(list).toHaveLength(2);
    expect(list.join(" ")).toContain("choose a map");
  });
});

describe("parseNum", () => {
  it("turns an empty box into null and junk into NaN", () => {
    expect(parseNum("")).toBeNull();
    expect(parseNum("  ")).toBeNull();
    expect(parseNum("0.5")).toBe(0.5);
    expect(parseNum("0")).toBe(0);
    expect(Number.isNaN(parseNum("abc") as number)).toBe(true);
  });
});

describe("what the Drive buttons send", () => {
  it("gap follow sends only the values that are set", () => {
    expect(gapBody(prefs())).toEqual({
      mode: "gap_follow",
      profile: "lane-2026-10-25",
      max_speed_mps: undefined,
      min_speed_mps: undefined,
      a_lat_max_mps2: undefined,
      side_cap_min_speed_mps: undefined,
    });
    const p = prefs({ gap_follow: { profile: "race-2026-11-13", max_speed_mps: 1, min_speed_mps: 0, a_lat_max_mps2: 1.8, side_cap_min_speed_mps: null } });
    const body = gapBody(p);
    expect(body.max_speed_mps).toBe(1);
    expect(body.min_speed_mps).toBe(0); // zero is a value, not "unset"
    expect(body.a_lat_max_mps2).toBe(1.8);
    expect(body.side_cap_min_speed_mps).toBeUndefined();
    expect(JSON.parse(JSON.stringify(body))).not.toHaveProperty("side_cap_min_speed_mps");
  });

  it("the ML driver names a bundle only when it is on offer", () => {
    const p = prefs({ pnp: { ...DEFAULT_PREFS.pnp, dynamics: "dyn-b", params_file: "gone.yaml" } });
    expect(pnpBody(p, options)).toMatchObject({ pnp_dynamics: "dyn-b", pnp_perception: "", pnp_params: "" });
    const stale = prefs({ pnp: { ...DEFAULT_PREFS.pnp, dynamics: "deleted" } });
    expect(pnpBody(stale, options).pnp_dynamics).toBe(""); // two offered, none chosen
    const net = prefs({ pnp: { ...DEFAULT_PREFS.pnp, backend: "net", dynamics: "dyn-a" } });
    expect(pnpBody(net, options).pnp_perception).toBe("net-a"); // the only one
  });

  it("the racer sends the saved raceline, Start car the saved mode", () => {
    expect(trackerBody(prefs({ tracker: { raceline: "track:gym_oval" } }))).toEqual({ mode: "tracker", raceline: "track:gym_oval" });
    expect(stackBody(prefs())).toEqual({ lidar: true, vesc: true, mapping: false, localization: false, map: "" });
    expect(stackBody(prefs({ stack: { mode: "localization", map: "lab.yaml" } }))).toMatchObject({ localization: true, map: "lab.yaml" });
    expect(stackBody(prefs({ stack: { mode: "mapping", map: "lab.yaml" } }))).toMatchObject({ mapping: true, map: "" });
  });
});

describe("why a button is disabled", () => {
  it("gap follow: only when a saved value is out of bounds", () => {
    expect(gapWhy(prefs(), options)).toBe("");
    expect(gapWhy(prefs({ gap_follow: { ...DEFAULT_PREFS.gap_follow, max_speed_mps: 3 } }), options)).toBe("Fix Tune values");
  });
  it("ML driver: needs a dynamics bundle", () => {
    expect(pnpWhy(prefs(), options)).toBe("Pick a dynamics bundle in Tune");
    expect(pnpWhy(prefs({ pnp: { ...DEFAULT_PREFS.pnp, dynamics: "dyn-a" } }), options)).toBe("");
    expect(pnpWhy(prefs(), { ...options, pnp: { ...options.pnp!, available: false, reason: "no bundles" } })).toBe("Unavailable");
  });
  it("racer: needs a raceline and a localised stack", () => {
    const running = { stack: { running: true, recorder: true, flags: { localization: "false" } } } as unknown as Status;
    expect(trackerWhy(prefs(), null)).toBe("Pick a raceline in Tune");
    const p = prefs({ tracker: { raceline: "track:gym_oval" } });
    expect(trackerWhy(p, running)).toContain("Start mode");
    expect(trackerWhy(p, { ...running, stack: { ...running.stack, flags: { localization: "true" } } })).toBe("");
  });
});

describe("helpers", () => {
  it("patch copies, never mutates", () => {
    const a = prefs();
    const b = patch(a, "gap_follow", { max_speed_mps: 0.5 });
    expect(a.gap_follow.max_speed_mps).toBeNull();
    expect(b.gap_follow).toEqual({ ...a.gap_follow, max_speed_mps: 0.5 });
    expect(patch(a, "record_camera", false).record_camera).toBe(false);
    expect(samePrefs(a, prefs())).toBe(true);
    expect(samePrefs(a, b)).toBe(false);
  });
  it("summaries say what will run", () => {
    expect(gapSummary(prefs())).toBe("lane-2026-10-25, profile speed");
    expect(gapSummary(prefs({ gap_follow: { ...DEFAULT_PREFS.gap_follow, max_speed_mps: 1 } }))).toBe("lane-2026-10-25, 1.00 m/s");
  });
});
