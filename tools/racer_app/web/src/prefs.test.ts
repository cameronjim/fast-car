import { describe, expect, it } from "vitest";
import type { Options, Prefs, Status } from "./api";
import {
  DEFAULT_PREFS,
  gapBody,
  gapErrors,
  gapSummary,
  gapWhy,
  lapChoice,
  lapChoicePatch,
  parseNum,
  patch,
  pnpBody,
  pnpErrors,
  pnpSummary,
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
    lap_modes: ["v0", "v1"],
    lap_sources: ["start_scan", "map_pose"],
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

// -- lap learning (brief 35) ---------------------------------------------------------------------

const pnpPrefs = (over: Partial<Prefs["pnp"]>): Prefs => prefs({ pnp: { ...DEFAULT_PREFS.pnp, dynamics: "dyn-a", ...over } });
const errs = (over: Partial<Prefs["pnp"]>) => pnpErrors({ ...DEFAULT_PREFS.pnp, ...over }, options);

describe("lap learning choice", () => {
  it("maps the four buttons to the saved fields and back", () => {
    expect(lapChoice(DEFAULT_PREFS.pnp)).toBe("off");
    for (const c of ["off", "observe", "v0", "v1"] as const) {
      expect(lapChoice({ ...DEFAULT_PREFS.pnp, ...lapChoicePatch(c) })).toBe(c);
    }
    expect(lapChoicePatch("observe")).toEqual({ lap_memory: true, lap_mode: "observe" });
  });
});

describe("pnpErrors lap rules, in the agent's words", () => {
  it("defaults are valid", () => expect(errs({})).toEqual({}));
  it("a mode other than v0 needs lap memory, except observe", () => {
    expect(errs({ lap_mode: "v1", lap_memory: false }).lap_mode).toBe("pnp lap mode v1 needs lap memory on");
    expect(errs({ lap_mode: "observe", lap_memory: false }).lap_mode).toBe("lap_mode=observe is not available in this build");
  });
  it("v1 needs start_scan", () => {
    expect(errs({ lap_memory: true, lap_mode: "v1", lap_source: "map_pose" }).lap_source).toBe("pnp lap mode v1 needs lap source start_scan");
    expect(errs({ lap_memory: true, lap_mode: "v0", lap_source: "map_pose" })).toEqual({});
  });
  it("the lap 1 cap needs lap memory or observe, a valid range and at most the speed cap", () => {
    expect(errs({ v_lap1_mps: 0.3 }).v_lap1_mps).toBe("pnp lap 1 cap needs lap memory on or lap mode observe");
    expect(errs({ v_lap1_mps: 0.3, lap_memory: true })).toEqual({});
    expect(errs({ v_lap1_mps: 0.6, lap_memory: true }).v_lap1_mps).toBe("pnp lap 1 cap cannot exceed the speed cap");
    expect(errs({ v_lap1_mps: 0.1, lap_memory: true }).v_lap1_mps).toBe("pnp lap 1 cap must be between 0.2 and 1.5 m/s");
    expect(errs({ v_lap1_mps: Number.NaN, lap_memory: true }).v_lap1_mps).toBeDefined();
  });
  it("require needs the counter", () => {
    expect(errs({ require_lap_counter: true, lap_counter: false }).require_lap_counter).toBe("require lap counter needs the lap counter on");
    expect(errs({ require_lap_counter: true })).toEqual({});
  });
  it("the shadow net may not be the driving net", () => {
    const net = { backend: "net", perception: "net-a", shadow_perception: "net-a" };
    expect(errs(net).shadow_perception).toBe("the shadow perception bundle cannot be the driving one");
    expect(errs({ ...net, backend: "geometric" })).toEqual({});
  });
  it("a mode the car's launch file does not declare is not available in this build", () => {
    const observe = { ...DEFAULT_PREFS.pnp, ...lapChoicePatch("observe") };
    expect(pnpErrors(observe, options).lap_mode).toBe("lap_mode=observe is not available in this build");
    const withObserve = { ...options, pnp: { ...options.pnp!, lap_modes: ["v0", "v1", "observe"] } } as Options;
    expect(pnpErrors(observe, withObserve)).toEqual({});
    // an older agent that reports no list is not second-guessed
    const old = { ...options, pnp: { ...options.pnp!, lap_modes: undefined } } as Options;
    expect(pnpErrors(observe, old)).toEqual({});
  });
  it("shows up in prefsProblems and stops the button", () => {
    const bad = pnpPrefs({ lap_mode: "v1", lap_memory: false });
    expect(prefsProblems(bad, options).join(" ")).toContain("needs lap memory on");
    expect(pnpWhy(bad, options)).toBe("Fix Tune values");
  });
});

describe("pnpBody lap learning", () => {
  const json = (p: Prefs) => JSON.parse(JSON.stringify(pnpBody(p, options)));
  it("an untouched card sends today's request: no lap fields", () => {
    expect(json(pnpPrefs({}))).toEqual({
      mode: "pnp",
      pnp_backend: "geometric",
      pnp_dynamics: "dyn-a",
      pnp_perception: "",
      pnp_params: "",
      pnp_v_max_mps: 0.5,
    });
  });
  it("includes only the fields that changed", () => {
    expect(json(pnpPrefs({ lap_memory: true, v_lap1_mps: 0.4 }))).toMatchObject({ pnp_lap_memory: true, pnp_v_lap1_mps: 0.4 });
    const v1 = json(pnpPrefs({ ...lapChoicePatch("v1"), lap_counter: false, shadow_perception: "net-a" }));
    expect(v1).toMatchObject({ pnp_lap_memory: true, pnp_lap_mode: "v1", pnp_lap_counter: false, pnp_shadow_perception: "net-a" });
    expect(v1).not.toHaveProperty("pnp_lap_source");
    expect(v1).not.toHaveProperty("pnp_require_lap_counter");
    expect(json(pnpPrefs({ lap_memory: true, lap_source: "map_pose" }))).toMatchObject({ pnp_lap_source: "map_pose" });
    expect(json(pnpPrefs({ require_lap_counter: true }))).toMatchObject({ pnp_require_lap_counter: true });
    expect(json(pnpPrefs({ ...lapChoicePatch("observe") }))).toMatchObject({ pnp_lap_mode: "observe", pnp_lap_memory: true });
  });
  it("lap settings are not sent while lap learning is off, and a stale shadow name is dropped", () => {
    const off = json(pnpPrefs({ v_lap1_mps: 0.3, lap_source: "map_pose", shadow_perception: "gone" }));
    expect(off).not.toHaveProperty("pnp_v_lap1_mps");
    expect(off).not.toHaveProperty("pnp_lap_source");
    expect(off).not.toHaveProperty("pnp_lap_memory");
    expect(off).not.toHaveProperty("pnp_shadow_perception");
  });
});

describe("pnpSummary", () => {
  it("says what will run", () => {
    expect(pnpSummary(pnpPrefs({}), options)).toBe("geometric, cap 0.50, lap learning off, lap counter on");
    const p = pnpPrefs({ v_max_mps: 1, v_lap1_mps: 0.5, ...lapChoicePatch("v0") });
    expect(pnpSummary(p, options)).toBe("geometric, cap 1.00, D v0 from 0.50, lap counter on");
    const full = pnpPrefs({ ...lapChoicePatch("v1"), lap_counter: true, require_lap_counter: true, shadow_perception: "net-a" });
    expect(pnpSummary(full, options)).toBe("geometric, cap 0.50, D v1 from 0.50, lap counter required, shadow net-a");
    expect(pnpSummary(pnpPrefs({ lap_counter: false }), options)).toContain("lap counter off");
    expect(pnpSummary(pnpPrefs({ ...lapChoicePatch("observe"), lap_source: "map_pose" }), options)).toContain("observe from 0.50, source map_pose");
  });
});
