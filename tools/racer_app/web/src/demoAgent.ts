// An in-browser fake of the racer agent, for `?demo=1` and the vitest suite. It answers the same
// HTTP API as the real agent (same shapes, same refusals, delayed so pending states are visible)
// and plays scripted transitions: a stack that starts in stages, a slow stop that "closes the bag",
// a drive, a dropped connection and a failed start. Nothing here talks to a car.

import type { ActionReply, BagInfo, Check, FetchLike, LogEntry, Options, Prefs, Status } from "./api";
import type { VehicleInfo } from "./model";
import { DEFAULT_PREFS } from "./prefs";

export type DemoPreset = "idle" | "running" | "driving" | "starting" | "offline" | "failed";

export const DEMO_PRESETS: DemoPreset[] = ["idle", "running", "driving", "starting", "offline", "failed"];

export function parsePreset(v: string | null | undefined): DemoPreset {
  return DEMO_PRESETS.includes(v as DemoPreset) ? (v as DemoPreset) : "idle";
}

export interface DemoEvent {
  severity: number;
  phase: number;
  durationS: number;
  source: string;
  detail: string;
}

type Sleep = (ms: number) => Promise<void>;

interface Reply {
  status: number;
  body: unknown;
}

const ok = (message: string, extra: Record<string, unknown> = {}): Reply => ({
  status: 200,
  body: { ok: true, message, ...extra },
});
const fail = (status: number, error: string, extra: Record<string, unknown> = {}): Reply => ({
  status,
  body: { ok: false, error, ...extra },
});

export class DemoCar {
  // what the "containers" are doing
  containerUp = false;
  nodesUp = false;
  recorderUp = false;
  closing = false;
  cameras = false;
  /** The running camera container's record_streams (what the real agent reads back). */
  camerasRecord = true;
  perception = false;
  prefs: Prefs = structuredClone(DEFAULT_PREFS);
  driveMode: string | null = null;
  autostart = true;
  flags: Record<string, string> = {};
  // scripted trouble
  offline = false;
  lowBattery = false;
  failNext = false;
  bagStart = 0;
  bagBaseMb = 0;
  uptimeS = 5400;
  private bornAt: number;
  private pendingEvents: DemoEvent[] = [];
  bags: BagInfo[] = [
    { name: "2026-11-12T14-18-03_car_teleop", size_mb: 4120, mtime: 1_762_982_283, active: false },
    { name: "2026-11-12T09-40-52_car_teleop", size_mb: 612, mtime: 1_762_965_652, active: false },
    { name: "2026-11-11T16-02-11_car_teleop", size_mb: 2480, mtime: 1_762_902_131, active: false },
  ];
  log: LogEntry[] = [];

  constructor(
    private now: () => number = () => Date.now(),
    private sleep: Sleep = (ms) => new Promise((r) => setTimeout(r, ms)),
    /** Scales every scripted delay (tests pass a small number). */
    private speed = 1,
  ) {
    this.bornAt = now();
    this.addLog("agent", true, "agent started");
  }

  preset(p: DemoPreset): void {
    this.offline = false;
    this.failNext = false;
    if (p === "running" || p === "driving") this.bringUp();
    if (p === "driving") {
      this.driveMode = "gap_follow";
      this.pendingEvents.push({ severity: 0, phase: 0, durationS: 0, source: "gap_follow", detail: "drive started" });
      this.addLog("drive/start", true, "mode=gap_follow profile=lane-2026-10-25");
    }
    if (p === "starting") void this.startStack();
    if (p === "offline") {
      this.bringUp();
      this.offline = true;
    }
    if (p === "failed") {
      this.failNext = true;
      void this.startStack().catch(() => undefined);
    }
  }

  private bringUp(): void {
    this.containerUp = true;
    this.nodesUp = true;
    this.recorderUp = true;
    this.flags = { lidar: "true", vesc: "true", mapping: "false", localization: "false" };
    this.openBag(3200);
    this.addLog("stack/start", true, "lidar=true vesc=true");
    this.cameras = true;
  }

  private openBag(baseMb: number): void {
    this.bagStart = this.now();
    this.bagBaseMb = baseMb;
    this.bags = [
      { name: "2026-11-13T10-31-27_car_teleop", size_mb: baseMb, mtime: this.now() / 1000, active: true, growth_mb_s: 0.52 },
      ...this.bags.filter((b) => !b.active),
    ];
  }

  private closeBag(): void {
    this.bags = this.bags.map((b) => ({ ...b, active: false, growth_mb_s: undefined }));
  }

  private grow(): BagInfo[] {
    return this.bags.map((b) =>
      b.active
        ? { ...b, size_mb: this.bagBaseMb + ((this.now() - this.bagStart) / 1000) * 0.52, growth_mb_s: 0.5 + ((this.now() / 1000) % 3) * 0.03 }
        : b,
    );
  }

  addLog(action: string, ok: boolean, detail: string, extra: Partial<LogEntry> = {}): void {
    this.log = [{ ts: this.now() / 1000, action, ok, actor: "app", detail, ...extra }, ...this.log].slice(0, 80);
  }

  drainEvents(): DemoEvent[] {
    const e = this.pendingEvents;
    this.pendingEvents = [];
    return e;
  }

  /** The ROS bridge lives in the stack container. */
  rosUp(): boolean {
    return !this.offline && this.containerUp;
  }

  pushEvent(e: DemoEvent): void {
    this.pendingEvents.push(e);
  }

  // -- scripted trouble, for the demo panel ---------------------------------------------------------

  async dropConnection(ms: number): Promise<void> {
    this.offline = true;
    await this.sleep(ms);
    this.offline = false;
  }

  // -- status -------------------------------------------------------------------------------------

  checks(): Check[] {
    const up = this.containerUp;
    const lvl = (good: boolean): Check["level"] => (good ? "green" : "red");
    const batt = this.lowBattery ? 10.9 : 11.8;
    const battLevel: Check["level"] = this.lowBattery ? "red" : "green";
    return [
      { id: "stack", label: "Stack container", level: lvl(up), detail: up ? "car-stack is running" : "car-stack is not running", required: true },
      { id: "recorder", label: "Bag recorder", level: lvl(this.recorderUp), detail: this.recorderUp ? "ros2 bag record is running" : "stack is not running", required: true },
      { id: "safety", label: "Safety node", level: lvl(this.nodesUp), detail: this.nodesUp ? "/drive is being advertised (safety_node up)" : "nothing publishes /drive: safety_node is not up", required: true },
      { id: "lidar", label: "LiDAR", level: lvl(this.nodesUp), detail: this.nodesUp ? "/scan at 10.1 Hz" : "no /scan", required: true },
      { id: "vesc", label: "VESC", level: lvl(this.nodesUp), detail: this.nodesUp ? "telemetry fresh, no fault" : "no VESC telemetry (UART silent)", required: true },
      { id: "battery", label: "Battery", level: this.nodesUp ? battLevel : "red", detail: this.nodesUp ? `${batt.toFixed(1)} V, ${(batt / 3).toFixed(2)} V per cell${this.lowBattery ? ", too low: charge it" : ""}` : "no VESC voltage reading", required: true },
      { id: "disk", label: "Disk space", level: "green", detail: "142.0 GiB free", required: true },
      { id: "heartbeat", label: "Heartbeat service", level: "green", detail: "racer-heartbeat.service is active", required: true },
      { id: "temps", label: "Temperatures", level: "green", detail: "SoC 48 C, GPU 46 C", required: true },
      { id: "wifi", label: "Wi-Fi link", level: "green", detail: "wlan0 -58 dBm", required: false },
      { id: "cameras", label: "Cameras", level: this.cameras ? "green" : "amber", detail: this.cameras ? "USB and CSI streaming" : "cameras are off", required: false },
    ];
  }

  status(): Status {
    const checks = this.checks();
    const reqRed = checks.some((c) => c.required && c.level === "red");
    const anyRed = checks.some((c) => c.level === "red");
    const anyAmber = checks.some((c) => c.level === "amber");
    return {
      time: this.now() / 1000,
      checks: { overall: anyRed ? "red" : anyAmber ? "amber" : "green", ready: !reqRed, checks },
      stack: { running: this.containerUp, recorder: this.recorderUp, flags: this.containerUp ? this.flags : {} },
      cameras: { running: this.cameras, record: this.cameras ? this.camerasRecord : null },
      drive: { mode: this.driveMode, running: !!this.driveMode },
      containers: { "car-stack": this.containerUp ? "running" : "exited", "car-cameras": this.cameras ? "running" : "exited" },
      disk_free_gb: 142,
      uptime_s: this.uptimeS + (this.now() - this.bornAt) / 1000,
      ros_connected: this.rosUp() && this.nodesUp,
      autostart: this.autostart,
      policy: { available: true, reason: "", valid: ["resid-v2"] },
      perception: { available: true, reason: "", running: this.perception },
      spiderbot: { available: true, reason: "" },
      thresholds: { disk_red_gb: 5, cameras_min_free_gb: 10, battery_cells: 3 },
    };
  }

  options(): Options {
    return {
      gap_follow_profiles: ["none", "floor-2026-10-06", "lane-2026-10-25", "race-2026-11-12", "race-2026-11-13"],
      park_profiles: ["none", "floor-2026-10-07"],
      maps: ["lab-2026-10-30.yaml", "garage-2026-11-02.yaml"],
      racelines: ["track:gym_oval", "data:lab-line-v3"],
      drive_modes: ["gap_follow", "wall_follow", "tracker", "park", "pnp", "ml_policy", "spiderbot"],
      gap_follow_speed: {
        none: { min: 0.36, max: 1.43 },
        "floor-2026-10-06": { min: 0.36, max: 0.64 },
        "lane-2026-10-25": { min: 0.36, max: 0.64 },
        "race-2026-11-12": { min: 0.36, max: 1.0 },
        "race-2026-11-13": { min: 0.36, max: 1.3 },
      },
      policy: { available: true, reason: "", valid: ["resid-v2"] },
      pnp: {
        available: true,
        reason: "",
        perception: ["net-2026-11-04"],
        dynamics: ["dyn-2026-10-20"],
        backends: ["geometric", "net"],
        speed_range: [0.2, 1.5],
        default_speed: 0.5,
        params_files: ["planner-tight.yaml", "planner-wide.yaml"],
        lap_modes: ["v0", "v1", "observe"],
        lap_sources: ["start_scan", "map_pose"],
      },
      patrols: ["loop-a"],
      spiderbot_rules: ["arms_raised"],
      spiderbot_speed_range: [0.1, 1.0],
      spiderbot_default_speed: 0.5,
      perception: { available: true, reason: "", running: this.perception },
      spiderbot: { available: true, reason: "" },
    };
  }

  vehicle(): VehicleInfo {
    return {
      schema_version: "1.4.0",
      lidar: { mount_x_m: 0.16, mount_y_m: 0, mount_yaw_rad: 0 },
      chassis: { wheelbase_m: 0.31, length_m: 0.44, width_m: 0.2, front_overhang_m: 0.08, rear_overhang_m: 0.05 },
      steering: { max_angle_rad: 0.42 },
      actuation: { throttle_full_scale_mps: 3 },
    };
  }

  // -- actions ------------------------------------------------------------------------------------

  private async startStack(flags: Record<string, unknown> = {}): Promise<Reply> {
    if (this.containerUp) return fail(409, "the stack is already running");
    const f = (ms: number) => this.sleep(ms * this.speed);
    if (this.failNext) {
      this.failNext = false;
      await f(1400);
      this.addLog("stack/start", false, "car-stack exited with code 1: VESC port /dev/ttyTHS1 is busy");
      return fail(500, "car-stack exited with code 1: VESC port /dev/ttyTHS1 is busy");
    }
    await f(800);
    this.containerUp = true;
    this.flags = {
      lidar: String(flags.lidar ?? true),
      vesc: String(flags.vesc ?? true),
      mapping: String(flags.mapping ?? false),
      localization: String(flags.localization ?? false),
    };
    this.addLog("stack/start", true, Object.entries(this.flags).map(([k, v]) => `${k}=${v}`).join(" "));
    // The request returns once the container is up; the nodes and the recorder follow.
    void f(2000).then(() => (this.nodesUp = true));
    void f(3600).then(() => {
      this.recorderUp = true;
      this.openBag(0);
    });
    return ok("stack started");
  }

  private async stopStack(): Promise<Reply> {
    if (!this.containerUp) return fail(409, "the stack is not running");
    const f = (ms: number) => this.sleep(ms * this.speed);
    this.driveMode = null;
    this.closing = true;
    await f(5000); // SIGINT, then the recorder closes the bag
    this.closeBag();
    this.containerUp = this.nodesUp = this.recorderUp = false;
    this.closing = false;
    this.cameras = false;
    this.perception = false;
    this.addLog("stack/stop", true, "stopped with SIGINT after 5.0 s, exit 0, bag has metadata.yaml");
    return ok("stack stopped, bag closed");
  }

  /** The lap learning rules of the agent (commands.py), word for word, so the demo refuses what the car refuses. */
  private checkPnpLaps(b: Record<string, unknown>): string | null {
    const isBool = (v: unknown) => v === undefined || typeof v === "boolean";
    for (const k of ["pnp_lap_memory", "pnp_lap_counter", "pnp_require_lap_counter"]) {
      if (!isBool(b[k])) return `${k} must be true or false`;
    }
    const cap = b.pnp_v_lap1_mps;
    if (cap !== undefined && (typeof cap !== "number" || !Number.isFinite(cap))) return "pnp lap 1 cap must be a number";
    const mode = b.pnp_lap_mode ?? "v0";
    if (!["v0", "v1", "observe"].includes(String(mode))) return "pnp lap mode must be one of ['v0', 'v1', 'observe']";
    const source = b.pnp_lap_source ?? "start_scan";
    if (!["start_scan", "map_pose"].includes(String(source))) return "pnp lap source must be one of ['start_scan', 'map_pose']";
    const memory = b.pnp_lap_memory === true || mode === "observe";
    if (mode !== "v0" && !memory) return `pnp lap mode ${mode} needs lap memory on`;
    if (mode === "v1" && source !== "start_scan") return "pnp lap mode v1 needs lap source start_scan";
    if (cap !== undefined) {
      if (!memory) return "pnp lap 1 cap needs lap memory on or lap mode observe";
      if (cap < 0.2 || cap > 1.5) return "pnp lap 1 cap must be between 0.2 and 1.5 m/s";
      if (cap > Number(b.pnp_v_max_mps ?? 0.5)) return "pnp lap 1 cap cannot exceed the speed cap";
    }
    if (b.pnp_require_lap_counter === true && b.pnp_lap_counter === false) return "require lap counter needs the lap counter on";
    const shadow = b.pnp_shadow_perception;
    if (shadow && shadow === b.pnp_perception) return "the shadow perception bundle cannot be the driving one";
    return null;
  }

  private checkDrive(body: Record<string, unknown>): Reply | null {
    if (!this.containerUp) return fail(409, "the stack is not running");
    if (!this.recorderUp) return fail(409, "the bag recorder is not running (cannot be overridden)", { overridable: false });
    if (this.driveMode) return fail(409, `already driving: ${this.driveMode}`);
    if (body.mode === "pnp") {
      const bad = this.checkPnpLaps(body);
      if (bad) return fail(400, bad);
    }
    if (this.lowBattery && !body.override) {
      const msg = "Not ready: Battery (10.9 V, too low: charge it)";
      return fail(409, msg, { overridable: true, failing: ["battery"] });
    }
    if (this.lowBattery && body.override && !String(body.override_reason ?? "").trim()) {
      return fail(400, "an override needs a reason");
    }
    return null;
  }

  async handle(method: string, path: string, body: Record<string, unknown>): Promise<Reply> {
    const f = (ms: number) => this.sleep(ms * this.speed);
    if (method === "GET") {
      await f(150);
      if (path.startsWith("/api/status")) return { status: 200, body: this.status() };
      if (path.startsWith("/api/options")) return { status: 200, body: this.options() };
      if (path.startsWith("/api/vehicle")) return { status: 200, body: this.vehicle() };
      if (path.startsWith("/api/bags")) {
        const bags = this.grow();
        return {
          status: 200,
          body: {
            bags,
            disk_free_gb: 142,
            disk: { total_gb: 232.2, used_gb: 90.2, free_gb: 142 },
            bags_mb: Math.round(bags.reduce((a, b) => a + b.size_mb, 0) * 10) / 10,
            bag_dir: "/home/racer/car/data/bags",
            user: "racer",
          },
        };
      }
      if (path.startsWith("/api/prefs")) return { status: 200, body: { prefs: this.prefs, defaults: DEFAULT_PREFS } };
      if (path.startsWith("/api/log")) return { status: 200, body: { entries: this.log } };
      if (path.startsWith("/api/drive/log")) return { status: 200, body: { log: "" } };
      return fail(404, "not found");
    }
    switch (path) {
      case "/api/stop": {
        await f(500);
        const had = this.driveMode;
        this.driveMode = null;
        this.addLog("stop", true, had ? `stopped ${had}` : "nothing was driving");
        return ok(had ? `stopped ${had}` : "nothing was driving");
      }
      case "/api/stack/start":
        return this.startStack(body);
      case "/api/stack/stop":
        return this.stopStack();
      case "/api/stack/restart": {
        const r = await this.stopStack();
        if (!r.body || (r.body as ActionReply).ok === false) return r;
        return this.startStack({ lidar: true, vesc: true });
      }
      case "/api/cameras/start":
        await f(1200);
        if (this.cameras) return fail(409, "the cameras are already running");
        this.cameras = true;
        this.camerasRecord = this.prefs.record_camera;
        this.addLog("cameras.start", true, `usb /dev/video1, camera ${this.camerasRecord ? "recorded" : "NOT recorded"} in the bag`);
        return ok("cameras starting", { record: this.camerasRecord });
      case "/api/prefs": {
        await f(400);
        const next = body as unknown as Prefs;
        const g = next.gap_follow;
        const lim = this.options().gap_follow_speed?.[g?.profile ?? ""];
        if (g && lim && g.max_speed_mps !== null && (g.max_speed_mps < lim.min || g.max_speed_mps > lim.max)) {
          return fail(400, `gap follow max speed for profile ${g.profile} must be between ${lim.min} and ${lim.max} m/s`);
        }
        this.prefs = structuredClone(next);
        this.addLog("prefs.save", true, `record camera ${next.record_camera ? "on" : "off"}`);
        return ok("settings saved", { prefs: this.prefs });
      }
      case "/api/system/cleanup":
        await f(1500);
        if (body.confirm !== "cleanup") return fail(400, "type 'cleanup' to confirm");
        this.addLog("system.cleanup", true, "reclaimed 1.2GB of images, 640MB of build cache");
        return ok("reclaimed 1.2GB of images, 640MB of build cache", {
          output: "$ docker image prune -f\nDeleted Images:\nsha256:3f1c...\n\nTotal reclaimed space: 1.2GB\n\n$ docker builder prune -f\nTotal reclaimed space: 640MB",
        });
      case "/api/cameras/stop":
        await f(900);
        this.cameras = false;
        this.perception = false;
        this.addLog("cameras/stop", true, "");
        return ok("cameras stopped");
      case "/api/perception/start":
        await f(1500);
        this.perception = true;
        this.addLog("perception/start", true, "pose cuda");
        return ok("perception started");
      case "/api/perception/stop":
        await f(900);
        this.perception = false;
        return ok("perception stopped");
      case "/api/drive/start": {
        await f(700);
        const refused = this.checkDrive(body);
        if (refused) {
          this.addLog("drive/start", false, String((refused.body as ActionReply).error));
          return refused;
        }
        this.driveMode = String(body.mode);
        const override = !!body.override;
        this.addLog("drive/start", true, `mode=${body.mode}`, override ? { override: true, override_reason: String(body.override_reason ?? "") } : {});
        this.pushEvent({ severity: 0, phase: 0, durationS: 0, source: String(body.mode), detail: "drive started" });
        return ok(`${body.mode} started`);
      }
      case "/api/map/save":
        await f(900);
        return ok(`map ${String(body.name)} saved`);
      case "/api/bags/delete":
        await f(700);
        if (this.bags.some((b) => b.active && b.name === body.name)) return fail(409, "that bag is being recorded right now");
        this.bags = this.bags.filter((b) => b.name !== body.name || b.active);
        this.addLog("bags/delete", true, String(body.name));
        return ok(`deleted ${String(body.name)}`);
      case "/api/system/reboot":
      case "/api/system/shutdown":
        await f(800);
        return ok("the Jetson is going down");
      case "/api/settings":
        await f(500);
        this.autostart = !!body.autostart;
        return ok(`autostart ${this.autostart ? "on" : "off"}`);
      default:
        if (path.startsWith("/api/park/")) {
          await f(600);
          const name = decodeURIComponent(path.slice("/api/park/".length));
          this.addLog("park", true, name);
          return ok(`park ${name}`);
        }
        return fail(404, "not found");
    }
  }

  /** The fetch the Api class uses. A dropped connection rejects like a network error. */
  fetch: FetchLike = async (input, init) => {
    if (this.offline) {
      await this.sleep(300 * this.speed);
      throw new TypeError("Failed to fetch");
    }
    const path = input.replace(/^https?:\/\/[^/]+/, "");
    let body: Record<string, unknown> = {};
    try {
      body = init?.body ? (JSON.parse(String(init.body)) as Record<string, unknown>) : {};
    } catch {
      /* no body */
    }
    const r = await this.handle(init?.method ?? "GET", path, body);
    if (this.offline) throw new TypeError("Failed to fetch");
    return new Response(JSON.stringify(r.body), { status: r.status, headers: { "Content-Type": "application/json" } });
  };
}
