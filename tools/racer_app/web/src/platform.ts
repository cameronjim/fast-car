// The only place that knows whether the UI runs in a browser served by the agent or inside the
// desktop app (Tauri). Everything else takes a Target (where the car is) and a TokenStore, so the
// rest of the UI is the same code in both.
//
// Desktop: the Rust side finds the car (mDNS, then a LAN scan), keeps the agent token in the OS
// keychain and makes HTTP calls through the http plugin (no CORS: the page origin is tauri://).
// This file only forwards to those commands. It has no way to publish to ROS and nothing here
// talks to the car's topics: that stays in ros.ts, which only subscribes.

import type { FetchLike, TokenStore } from "./api";
import { browserTokenStore } from "./api";

export const FOXGLOVE_PORT = 8765;

/** Where the car is: the agent's API base ("" means same origin) and the foxglove socket. */
export interface Target {
  apiBase: string;
  rosUrl: string;
}

export interface Car {
  host: string;
  port: number;
  hostname: string;
  source: "mdns" | "name" | "scan" | "manual" | "saved";
}

export type Phase = "searching" | "connected" | "lost";

/** What the Rust discovery loop reports. `car` is the one the app is using. */
export interface DiscoveryState {
  phase: Phase;
  car: Car | null;
  cars: Car[];
}

export function webTarget(loc: { hostname: string }): Target {
  return { apiBase: "", rosUrl: `ws://${loc.hostname}:${FOXGLOVE_PORT}` };
}

export function carTarget(car: Pick<Car, "host" | "port">): Target {
  const host = car.host.includes(":") ? `[${car.host}]` : car.host;
  return { apiBase: `http://${host}:${car.port}`, rosUrl: `ws://${host}:${FOXGLOVE_PORT}` };
}

export function carLabel(car: Car): string {
  return car.hostname ? `${car.hostname} (${car.host})` : car.host;
}

/** The shape of window.__TAURI__ that we use (withGlobalTauri is on in tauri.conf.json). */
export interface TauriGlobal {
  core: { invoke<T>(cmd: string, args?: Record<string, unknown>): Promise<T> };
  event: { listen<T>(name: string, cb: (e: { payload: T }) => void): Promise<() => void> };
  http: { fetch: FetchLike };
}

export function getTauri(scope: object = globalThis): TauriGlobal | null {
  const t = (scope as { __TAURI__?: Partial<TauriGlobal> }).__TAURI__;
  return t?.core && t.event && t.http ? (t as TauriGlobal) : null;
}

/**
 * A synchronous TokenStore (the Api asks for the token on every call) backed by a cache that is
 * written through to the keychain. A failed write keeps the token for this session only.
 */
export function cachedTokenStore(
  initial: string | null,
  persist: (token: string | null) => Promise<void>,
  onError: (e: unknown) => void = () => undefined,
): TokenStore {
  let value = initial;
  const write = (t: string | null) => {
    value = t;
    persist(t).catch(onError);
  };
  return { get: () => value, set: (t) => write(t), clear: () => write(null) };
}

export interface Desktop {
  tokens: TokenStore;
  fetch: FetchLike;
  state(): Promise<DiscoveryState>;
  onState(cb: (s: DiscoveryState) => void): Promise<() => void>;
  selectCar(car: Car): Promise<DiscoveryState>;
  addManual(address: string): Promise<DiscoveryState>;
  rescan(): Promise<void>;
}

export async function loadDesktop(t: TauriGlobal): Promise<Desktop> {
  const invoke = t.core.invoke;
  let initial: string | null = null;
  try {
    initial = await invoke<string | null>("token_get");
  } catch {
    /* no keychain on this machine: the token is asked for on each launch */
  }
  return {
    tokens: cachedTokenStore(initial, (tok) =>
      tok === null ? invoke("token_clear") : invoke("token_set", { token: tok }),
    ),
    fetch: (url, init) => t.http.fetch(url, init),
    state: () => invoke<DiscoveryState>("discovery_state"),
    onState: (cb) => t.event.listen<DiscoveryState>("racer://state", (e) => cb(e.payload)),
    selectCar: (car) => invoke<DiscoveryState>("select_car", { car }),
    addManual: (address) => invoke<DiscoveryState>("add_manual", { address }),
    rescan: () => invoke<void>("rescan"),
  };
}

export interface Platform {
  kind: "web" | "desktop";
  tokens: TokenStore;
  fetch?: FetchLike;
  desktop?: Desktop;
  target?: Target;
}

export async function loadPlatform(scope: object = globalThis): Promise<Platform> {
  const t = getTauri(scope);
  if (!t) {
    const loc = (scope as { location?: { hostname: string } }).location ?? { hostname: "localhost" };
    return { kind: "web", tokens: browserTokenStore(), target: webTarget(loc) };
  }
  const desktop = await loadDesktop(t);
  return { kind: "desktop", tokens: desktop.tokens, fetch: desktop.fetch, desktop };
}
