// Client for the racer agent's HTTP API. In a browser the page is served by the agent, so paths
// are same-origin; in the desktop app the platform layer passes the car's http://host:port base.

export interface Check {
  id: string;
  label: string;
  level: "green" | "amber" | "red";
  detail: string;
  required: boolean;
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
  cameras: { running: boolean };
  drive: { mode: string | null; running: boolean };
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
}

export interface Options {
  gap_follow_profiles: string[];
  park_profiles: string[];
  maps: string[];
  racelines: string[];
  drive_modes: string[];
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

  private async call<T>(method: string, path: string, body?: unknown, auth = true): Promise<T> {
    const headers: Record<string, string> = {};
    const token = this.tokens.get();
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
    if (res.status === 401) this.onUnauthorized();
    if (!res.ok) {
      const reply = (json ?? null) as ActionReply | null;
      throw new ApiError(reply?.error ?? `HTTP ${res.status}`, res.status, reply);
    }
    return json as T;
  }

  status = () => this.call<Status>("GET", "/api/status");
  options = () => this.call<Options>("GET", "/api/options");
  vehicle = () => this.call<import("./model").VehicleInfo>("GET", "/api/vehicle");
  bags = () => this.call<{ bags: BagInfo[]; disk_free_gb: number | null }>("GET", "/api/bags");
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
