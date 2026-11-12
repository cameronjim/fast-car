import { describe, expect, it, vi } from "vitest";
import { Api } from "./api";
import {
  cachedTokenStore,
  carLabel,
  carTarget,
  describeTokenStorage,
  getTauri,
  loadDesktop,
  loadPlatform,
  runSearch,
  searchText,
  webTarget,
  type Car,
  type SearchView,
  type TauriGlobal,
} from "./platform";

const car: Car = { host: "192.168.1.50", port: 8080, hostname: "racer-car", source: "mdns" };

function fakeTauri(token: string | null = null, setReply: unknown = undefined) {
  const calls: Array<[string, unknown]> = [];
  const invoke = vi.fn(async (cmd: string, args?: Record<string, unknown>) => {
    calls.push([cmd, args]);
    if (cmd === "token_get") return token;
    if (cmd === "token_storage") return { storage: "keychain", note: "" };
    if (cmd === "token_set") return setReply ?? { storage: "keychain", note: "" };
    if (cmd === "search_cars") return { found: 0, state: { phase: "searching", car: null, cars: [] } };
    return null;
  });
  const listen = vi.fn(async () => () => undefined);
  const fetch = vi.fn(async () => new Response("{}", { status: 200 }));
  const t = { core: { invoke }, event: { listen }, http: { fetch } } as unknown as TauriGlobal;
  return { t, calls, invoke, listen, fetch };
}

describe("targets", () => {
  it("the web build uses the page's own host for the foxglove socket and same-origin API", () => {
    expect(webTarget({ hostname: "racer-car.local" })).toEqual({
      apiBase: "",
      rosUrl: "ws://racer-car.local:8765",
    });
  });

  it("the desktop build is told the car's address", () => {
    expect(carTarget(car)).toEqual({ apiBase: "http://192.168.1.50:8080", rosUrl: "ws://192.168.1.50:8765" });
  });

  it("brackets an IPv6 address", () => {
    expect(carTarget({ host: "fe80::1", port: 8080 }).rosUrl).toBe("ws://[fe80::1]:8765");
  });

  it("labels a car with its host name when it has one", () => {
    expect(carLabel(car)).toBe("racer-car (192.168.1.50)");
    expect(carLabel({ ...car, hostname: "" })).toBe("192.168.1.50");
  });
});

describe("Api base", () => {
  it("prefixes every path with the car's address", async () => {
    const fetchImpl = vi.fn().mockResolvedValue(new Response("{}", { status: 200 }));
    const api = new Api(cachedTokenStore("abc", async () => undefined), fetchImpl, "http://10.0.0.5:8080");
    await api.status();
    await api.stopAll();
    expect(fetchImpl.mock.calls[0][0]).toBe("http://10.0.0.5:8080/api/status");
    expect(fetchImpl.mock.calls[1][0]).toBe("http://10.0.0.5:8080/api/stop");
  });
});

describe("detection", () => {
  it("is a browser when there is no Tauri global", async () => {
    const p = await loadPlatform({ location: { hostname: "racer-car.local" } });
    expect(p.kind).toBe("web");
    expect(p.target?.apiBase).toBe("");
  });

  it("ignores a half-present global", () => {
    expect(getTauri({ __TAURI__: { core: {} } })).toBeNull();
  });

  it("is the desktop app when the global is injected", async () => {
    const { t } = fakeTauri();
    const p = await loadPlatform({ __TAURI__: t });
    expect(p.kind).toBe("desktop");
    expect(p.desktop).toBeDefined();
    expect(p.target).toBeUndefined();
  });
});

describe("token store", () => {
  it("answers synchronously and writes through", async () => {
    const persist = vi.fn().mockResolvedValue(undefined);
    const s = cachedTokenStore("old", persist);
    expect(s.get()).toBe("old");
    s.set("new");
    expect(s.get()).toBe("new");
    expect(persist).toHaveBeenCalledWith("new");
    s.clear();
    expect(s.get()).toBeNull();
    expect(persist).toHaveBeenLastCalledWith(null);
  });

  it("keeps the token for the session when the keychain write fails", async () => {
    const errors: unknown[] = [];
    const s = cachedTokenStore(null, () => Promise.reject(new Error("no keychain")), (e) => errors.push(e));
    s.set("abc");
    await Promise.resolve();
    await Promise.resolve();
    expect(s.get()).toBe("abc");
    expect(errors).toHaveLength(1);
  });
});

describe("desktop bridge", () => {
  it("loads the saved token from the keychain at start", async () => {
    const { t } = fakeTauri("saved-token");
    const d = await loadDesktop(t);
    expect(d.tokens.get()).toBe("saved-token");
  });

  it("starts without a token when the keychain is unavailable", async () => {
    const { t, invoke } = fakeTauri();
    invoke.mockRejectedValueOnce(new Error("no secret service"));
    const d = await loadDesktop(t);
    expect(d.tokens.get()).toBeNull();
  });

  it("stores and clears the token through the keychain commands", async () => {
    const { t, calls } = fakeTauri();
    const d = await loadDesktop(t);
    d.tokens.set("abc");
    d.tokens.clear();
    expect(calls).toContainEqual(["token_set", { token: "abc" }]);
    expect(calls.map((c) => c[0])).toContain("token_clear");
  });

  it("forwards discovery commands and sends HTTP through the plugin, not the page", async () => {
    const { t, calls, fetch } = fakeTauri();
    const d = await loadDesktop(t);
    await d.selectCar(car);
    await d.addManual("192.168.1.9");
    await d.search();
    await d.fetch("http://192.168.1.50:8080/api/status", { method: "GET" });
    expect(calls).toContainEqual(["select_car", { car }]);
    expect(calls).toContainEqual(["add_manual", { address: "192.168.1.9" }]);
    expect(calls.map((c) => c[0])).toContain("search_cars");
    expect(fetch).toHaveBeenCalledOnce();
  });

  it("subscribes to the state event the Rust loop emits", async () => {
    const { t, listen } = fakeTauri();
    const d = await loadDesktop(t);
    await d.onState(() => undefined);
    expect(listen).toHaveBeenCalledWith("racer://state", expect.any(Function));
  });
});

describe("where the token lives", () => {
  it("says nothing while the keychain works", () => {
    expect(describeTokenStorage(null)).toBeNull();
    expect(describeTokenStorage({ storage: "keychain", note: "" })).toBeNull();
  });

  it("says so when the keychain was refused and the token is in the data folder", () => {
    const text = describeTokenStorage({ storage: "file", note: "keychain read refused: denied" });
    expect(text).toContain("keychain refused");
    expect(text).toContain("data folder");
    expect(text).toContain("denied");
  });

  it("says so when the token could not be saved at all", () => {
    expect(describeTokenStorage({ storage: "memory", note: "disk full" })).toContain("could not be saved");
  });

  it("the desktop bridge reports the storage at start and when saving falls back to the file", async () => {
    const { t } = fakeTauri(null, { storage: "file", note: "denied" });
    const d = await loadDesktop(t);
    expect(d.tokenStorage.current()).toEqual({ storage: "keychain", note: "" });
    const seen: unknown[] = [];
    d.tokenStorage.subscribe((info) => seen.push(info));
    d.tokens.set("abc");
    await new Promise((r) => setTimeout(r, 0));
    expect(seen).toEqual([{ storage: "file", note: "denied" }]);
    expect(d.tokenStorage.current()?.storage).toBe("file");
  });

  it("the bridge reports an unsaved token as memory only", async () => {
    const { t, invoke } = fakeTauri();
    const d = await loadDesktop(t);
    invoke.mockImplementation(async (cmd: string) => {
      if (cmd === "token_set") throw new Error("nowhere to write");
      return null;
    });
    d.tokens.set("abc");
    await new Promise((r) => setTimeout(r, 0));
    expect(d.tokens.get()).toBe("abc");
    expect(d.tokenStorage.current()?.storage).toBe("memory");
  });

  it("an older native side without token_storage just says nothing", async () => {
    const { t, invoke } = fakeTauri();
    invoke.mockImplementation(async (cmd: string) => {
      if (cmd === "token_storage") throw new Error("unknown command");
      return null;
    });
    const d = await loadDesktop(t);
    expect(d.tokenStorage.current()).toBeNull();
  });
});

describe("Search again feedback", () => {
  const state = { phase: "searching" as const, car: null, cars: [] };

  it("shows searching, then found", async () => {
    const seen: SearchView[] = [];
    await runSearch({ search: async () => ({ found: 2, state }) }, (v) => seen.push(v));
    expect(seen).toEqual([{ kind: "searching" }, { kind: "found", count: 2 }]);
    expect(searchText(seen[0])).toContain("Searching");
    expect(searchText(seen[1])).toBe("Found 2 cars.");
  });

  it("shows searching, then not found", async () => {
    const seen: SearchView[] = [];
    await runSearch({ search: async () => ({ found: 0, state }) }, (v) => seen.push(v));
    expect(seen).toEqual([{ kind: "searching" }, { kind: "none" }]);
    expect(searchText(seen[1])).toContain("No car found");
  });

  it("is searching for the whole time the native search runs", async () => {
    let finish: (v: { found: number; state: typeof state }) => void = () => undefined;
    const pending = new Promise<{ found: number; state: typeof state }>((r) => (finish = r));
    const seen: SearchView[] = [];
    const done = runSearch({ search: () => pending }, (v) => seen.push(v));
    expect(seen).toEqual([{ kind: "searching" }]);
    finish({ found: 1, state });
    await done;
    expect(seen.at(-1)).toEqual({ kind: "found", count: 1 });
    expect(searchText({ kind: "found", count: 1 })).toBe("Found 1 car.");
  });

  it("reports a failed search instead of staying silent", async () => {
    const seen: SearchView[] = [];
    await runSearch({ search: async () => Promise.reject(new Error("no network")) }, (v) => seen.push(v));
    expect(seen.at(-1)).toEqual({ kind: "error", message: "Error: no network" });
    expect(searchText({ kind: "idle" })).toBe("");
  });
});
