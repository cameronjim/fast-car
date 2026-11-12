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

/** Where the desktop app keeps the agent token. `memory`: it could not be saved anywhere. */
export interface TokenStorageInfo {
  storage: "keychain" | "file" | "memory";
  /** The technical reason when it is not the keychain (shown small, for the owner). */
  note: string;
}

/** What the UI says about where the token lives, or null when the keychain is fine. */
export function describeTokenStorage(info: TokenStorageInfo | null): string | null {
  if (!info || info.storage === "keychain") return null;
  if (info.storage === "file") {
    return (
      "The macOS keychain refused access, so the token is kept in the app's own data folder " +
      "(a file only you can read) and survives reinstalls." +
      (info.note ? ` (${info.note})` : "")
    );
  }
  return (
    "The token could not be saved anywhere, so it is kept until you quit the app and you will " +
    "be asked again next time." +
    (info.note ? ` (${info.note})` : "")
  );
}

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
  persist: (token: string | null) => Promise<TokenStorageInfo | void>,
  onError: (e: unknown) => void = () => undefined,
  onSaved: (info: TokenStorageInfo) => void = () => undefined,
): TokenStore {
  let value = initial;
  const write = (t: string | null) => {
    value = t;
    persist(t)
      .then((info) => {
        if (info && t !== null) onSaved(info);
      })
      .catch(onError);
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
  /** Run a full search now (mDNS, then a subnet scan) and say how many cars it found. */
  search(): Promise<SearchOutcome>;
  /** Where the token is kept, now and whenever it changes. */
  tokenStorage: {
    current(): TokenStorageInfo | null;
    subscribe(cb: (info: TokenStorageInfo | null) => void): () => void;
  };
}

export interface SearchOutcome {
  found: number;
  state: DiscoveryState;
}

/** What the Search again button shows. */
export type SearchView =
  | { kind: "idle" }
  | { kind: "searching" }
  | { kind: "found"; count: number }
  | { kind: "none" }
  | { kind: "error"; message: string };

export function searchText(v: SearchView): string {
  switch (v.kind) {
    case "idle":
      return "";
    case "searching":
      return "Searching the network (about 5 seconds)...";
    case "found":
      return v.count === 1 ? "Found 1 car." : `Found ${v.count} cars.`;
    case "none":
      return "No car found on this network. Check that the car is on and on the same Wi-Fi, or enter its address below.";
    case "error":
      return `The search failed: ${v.message}`;
  }
}

/** Run a search and report each step to `show`: searching, then found or none (or error). */
export async function runSearch(
  desktop: Pick<Desktop, "search">,
  show: (v: SearchView) => void,
): Promise<void> {
  show({ kind: "searching" });
  try {
    const out = await desktop.search();
    show(out.found > 0 ? { kind: "found", count: out.found } : { kind: "none" });
  } catch (e) {
    show({ kind: "error", message: String(e) });
  }
}

export async function loadDesktop(t: TauriGlobal): Promise<Desktop> {
  const invoke = t.core.invoke;
  let initial: string | null = null;
  try {
    initial = await invoke<string | null>("token_get");
  } catch {
    /* nowhere to read it from: the token is asked for on this launch */
  }
  let storage: TokenStorageInfo | null = null;
  try {
    storage = await invoke<TokenStorageInfo>("token_storage");
  } catch {
    /* an older native side: say nothing about where the token lives */
  }
  const listeners = new Set<(info: TokenStorageInfo | null) => void>();
  const setStorage = (info: TokenStorageInfo | null) => {
    storage = info;
    listeners.forEach((cb) => cb(info));
  };
  return {
    tokens: cachedTokenStore(
      initial,
      (tok) =>
        tok === null
          ? invoke<void>("token_clear")
          : invoke<TokenStorageInfo>("token_set", { token: tok }),
      (e) => setStorage({ storage: "memory", note: String(e) }),
      setStorage,
    ),
    tokenStorage: {
      current: () => storage,
      subscribe: (cb) => {
        listeners.add(cb);
        return () => listeners.delete(cb);
      },
    },
    fetch: (url, init) => t.http.fetch(url, init),
    state: () => invoke<DiscoveryState>("discovery_state"),
    onState: (cb) => t.event.listen<DiscoveryState>("racer://state", (e) => cb(e.payload)),
    selectCar: (car) => invoke<DiscoveryState>("select_car", { car }),
    addManual: (address) => invoke<DiscoveryState>("add_manual", { address }),
    search: () => invoke<SearchOutcome>("search_cars"),
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
