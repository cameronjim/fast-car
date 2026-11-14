// Client for the racer agent's HTTP API. In a browser the page is served by the agent, so paths
// are same-origin; in the desktop app the platform layer passes the car's http://host:port base.

export interface Check {
  id: string;
  label: string;
  level: "green" | "amber" | "red";
  detail: string;
  required: boolean;
}

/** A driving launch the app did not start (`docker exec ... ros2 launch` in a terminal). */
export interface ExternalDrive {
  pid: number;
  mode: string;
  command: string;
}

/** A feature that can be disabled on this car, with the reason shown next to its button. */
export interface Capability {
  available: boolean;
  reason: string;
}

export interface Status {
  time: number;
  checks: { overall: "green" | "amber" | "red"; ready: boolean; checks: Check[] };
  stack: { running: boolean; recorder: boolean; flags: Record<string, string> };
  /** record: does the RUNNING camera container feed the bag (null while the cameras are off). */
  cameras: { running: boolean; record?: boolean | null };
  drive: { mode: string | null; running: boolean; external?: ExternalDrive[] };
  containers: Record<string, string>;
  disk_free_gb: number | null;
  uptime_s: number | null;
  ros_connected: boolean;
  autostart: boolean;
  policy: { available: boolean; reason: string; valid: string[] };
  perception: Capability & { running: boolean };
  spiderbot: Capability;
  thresholds: { disk_red_gb: number; cameras_min_free_gb: number; battery_cells: number };
}

/** The Plug-and-Play driver: bundles found under data/pnp/models, by kind. */
export interface PnpOptions {
  available: boolean;
  reason: string;
  perception: string[];
  dynamics: string[];
  backends: string[];
  speed_range: [number, number];
  default_speed: number;
  /** Planner parameter files found under data/pnp/*.yaml (names only). */
  params_files?: string[];
  /** Lap modes and lap sources the car's pnp.launch.py declares (absent on an older agent). */
  lap_modes?: string[];
  lap_sources?: string[];
}

export interface Options {
  gap_follow_profiles: string[];
  park_profiles: string[];
  maps: string[];
  racelines: string[];
  drive_modes: string[];
  /** Gap follow's max speed bounds in m/s per profile (profile ceiling, vehicle_params cap). */
  gap_follow_speed?: Record<string, { min: number; max: number }>;
  policy: { available: boolean; reason: string; valid: string[] };
  pnp?: PnpOptions;
  patrols?: string[];
  spiderbot_rules?: string[];
  spiderbot_speed_range?: [number, number];
  spiderbot_default_speed?: number;
  perception_modes?: string[];
  perception_devices?: string[];
  perception?: Capability & { running: boolean };
  spiderbot?: Capability;
}

export interface BagInfo {
  name: string;
  size_mb: number;
  mtime: number;
  active: boolean;
  /** MB/s over the last 30 s; only on the active bag, null until two samples exist. */
  growth_mb_s?: number | null;
}

/** GET /api/bags: the runs, the disk they are on, and what the copy hint needs. */
export interface BagsReply {
  bags: BagInfo[];
  disk_free_gb: number | null;
  disk?: { total_gb: number | null; used_gb: number | null; free_gb: number | null };
  bags_mb?: number;
  bag_dir?: string;
  user?: string;
}

export type LapMode = "v0" | "v1" | "observe";
export type LapSource = "start_scan" | "map_pose";

/** The owner's saved drive settings (the Tune tab), kept on the car. null: the profile's own. */
export interface Prefs {
  record_camera: boolean;
  stack: { mode: "teleop" | "mapping" | "localization"; map: string };
  gap_follow: {
    profile: string;
    max_speed_mps: number | null;
    min_speed_mps: number | null;
    a_lat_max_mps2: number | null;
    side_cap_min_speed_mps: number | null;
  };
  pnp: {
    v_max_mps: number;
    backend: string;
    dynamics: string;
    perception: string;
    params_file: string;
    /** Lap 1 cap in m/s; null: equal to the speed cap (not sent). */
    v_lap1_mps: number | null;
    lap_memory: boolean;
    lap_mode: LapMode;
    lap_source: LapSource;
    lap_counter: boolean;
    require_lap_counter: boolean;
    /** A perception bundle for the shadow net, or "" for none. */
    shadow_perception: string;
  };
  tracker: { raceline: string };
}

export interface PrefsReply {
  prefs: Prefs;
  defaults: Prefs;
}

export interface LogEntry {
  ts: number;
  action: string;
  ok: boolean;
  actor: string;
  detail: string;
  override?: boolean;
  override_reason?: string;
}

export interface ActionReply {
  ok: boolean;
  message?: string;
  error?: string;
  overridable?: boolean;
  failing?: string[];
  [k: string]: unknown;
}

/** What the login screen says for a failed token check. */
export function loginError(err: unknown): string {
  if (err instanceof ApiError) {
    if (err.status === 401) return "That token was not accepted. Check it and paste it again.";
    if (err.status === 429) return "Too many wrong tokens from this computer. Wait a minute and try again.";
    if (err.status === 0) return "Cannot reach the car. Check it is on and on the same Wi-Fi.";
  }
  return String(err);
}

export class ApiError extends Error {
  constructor(
    message: string,
    public status: number,
    public body: ActionReply | null,
  ) {
    super(message);
  }
}

export interface TokenStore {
  get(): string | null;
  set(token: string): void;
  clear(): void;
}

const KEY = "racer-token";

export function browserTokenStore(): TokenStore {
  return {
    get: () => {
      try {
        return localStorage.getItem(KEY);
      } catch {
        return null;
      }
    },
    set: (t) => {
      try {
        localStorage.setItem(KEY, t);
      } catch {
        /* private mode: the token lives until reload */
      }
    },
    clear: () => {
      try {
        localStorage.removeItem(KEY);
      } catch {
        /* nothing to clear */
      }
    },
  };
}

export type FetchLike = (input: string, init?: RequestInit) => Promise<Response>;

export class Api {
  onUnauthorized: () => void = () => undefined;

  constructor(
    private tokens: TokenStore,
    private fetchImpl: FetchLike = (i, init) => fetch(i, init),
    private base = "",
  ) {}

  hasToken(): boolean {
    return !!this.tokens.get();
  }

  setToken(token: string): void {
    this.tokens.set(token.trim());
  }

  /**
   * Ask the car whether `token` is accepted, without storing it and without the "unauthorized"
   * callback. The login form uses it so a mistyped token never replaces a good saved one.
   */
  async verifyToken(token: string): Promise<void> {
    await this.call<unknown>("GET", "/api/status", undefined, true, token.trim());
  }

  private async call<T>(
    method: string,
    path: string,
    body?: unknown,
    auth = true,
    tokenOverride?: string,
  ): Promise<T> {
    const headers: Record<string, string> = {};
    const token = tokenOverride ?? this.tokens.get();
    if (auth && token) headers["Authorization"] = `Bearer ${token}`;
    if (body !== undefined) headers["Content-Type"] = "application/json";
    let res: Response;
    try {
      res = await this.fetchImpl(this.base + path, {
        method,
        headers,
        body: body === undefined ? undefined : JSON.stringify(body),
      });
    } catch {
      throw new ApiError("cannot reach the car (network)", 0, null);
    }
    let json: unknown = null;
    try {
      json = await res.json();
    } catch {
      /* not JSON */
    }
    if (res.status === 401 && tokenOverride === undefined) this.onUnauthorized();
    if (!res.ok) {
      const reply = (json ?? null) as ActionReply | null;
      throw new ApiError(reply?.error ?? `HTTP ${res.status}`, res.status, reply);
    }
    return json as T;
  }

  status = () => this.call<Status>("GET", "/api/status");
  options = () => this.call<Options>("GET", "/api/options");
  vehicle = () => this.call<import("./model").VehicleInfo>("GET", "/api/vehicle");
  bags = () => this.call<BagsReply>("GET", "/api/bags");
  prefs = () => this.call<PrefsReply>("GET", "/api/prefs");
  savePrefs = (p: Prefs) => this.call<ActionReply & { prefs?: Prefs }>("POST", "/api/prefs", p);
  cleanup = () => this.call<ActionReply & { output?: string }>("POST", "/api/system/cleanup", { confirm: "cleanup" });
  log = (limit = 40) => this.call<{ entries: LogEntry[] }>("GET", `/api/log?limit=${limit}`);
  driveLog = () => this.call<{ log: string }>("GET", "/api/drive/log");

  /** STOP ALL needs no token on purpose: a phone with a stale token must still stop the car. */
  stopAll = () => this.call<ActionReply>("POST", "/api/stop", undefined, false);

  startStack = (flags: { lidar: boolean; vesc: boolean; mapping: boolean; localization: boolean; map: string }) =>
    this.call<ActionReply>("POST", "/api/stack/start", flags);
  stopStack = () => this.call<ActionReply>("POST", "/api/stack/stop");
  restartStack = () => this.call<ActionReply>("POST", "/api/stack/restart", {});
  startCameras = () => this.call<ActionReply>("POST", "/api/cameras/start");
  stopCameras = () => this.call<ActionReply>("POST", "/api/cameras/stop");
  startDrive = (b: {
    mode: string;
    profile?: string;
    raceline?: string;
    policy?: string;
    override?: boolean;
    override_reason?: string;
    patrol?: string;
    rule?: string;
    speed_mps?: number;
    pnp_perception?: string;
    pnp_dynamics?: string;
    pnp_backend?: string;
    pnp_v_max_mps?: number;
    pnp_params?: string;
    pnp_v_lap1_mps?: number;
    pnp_lap_memory?: boolean;
    pnp_lap_mode?: LapMode;
    pnp_lap_source?: LapSource;
    pnp_lap_counter?: boolean;
    pnp_require_lap_counter?: boolean;
    pnp_shadow_perception?: string;
    max_speed_mps?: number;
    min_speed_mps?: number;
    a_lat_max_mps2?: number;
    side_cap_min_speed_mps?: number;
  }) => this.call<ActionReply>("POST", "/api/drive/start", b);
  startPerception = (b: { mode: string; device: string }) =>
    this.call<ActionReply>("POST", "/api/perception/start", b);
  stopPerception = () => this.call<ActionReply>("POST", "/api/perception/stop");
  park = (name: string, override = false, override_reason = "") =>
    this.call<ActionReply>("POST", `/api/park/${encodeURIComponent(name)}`, { override, override_reason });
  saveMap = (name: string) => this.call<ActionReply>("POST", "/api/map/save", { name });
  deleteBag = (name: string) => this.call<ActionReply>("POST", "/api/bags/delete", { name, confirm: name });
  power = (kind: "reboot" | "shutdown") =>
    this.call<ActionReply>("POST", `/api/system/${kind}`, { confirm: kind });
  setAutostart = (autostart: boolean) => this.call<ActionReply>("POST", "/api/settings", { autostart });
}
