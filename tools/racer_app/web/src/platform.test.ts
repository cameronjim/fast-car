import { describe, expect, it, vi } from "vitest";
import { Api } from "./api";
import {
  cachedTokenStore,
  carLabel,
  carTarget,
  getTauri,
  loadDesktop,
  loadPlatform,
  webTarget,
  type Car,
  type TauriGlobal,
} from "./platform";

const car: Car = { host: "192.168.1.50", port: 8080, hostname: "racer-car", source: "mdns" };

function fakeTauri(token: string | null = null) {
  const calls: Array<[string, unknown]> = [];
  const invoke = vi.fn(async (cmd: string, args?: Record<string, unknown>) => {
    calls.push([cmd, args]);
    if (cmd === "token_get") return token;
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
    await d.rescan();
    await d.fetch("http://192.168.1.50:8080/api/status", { method: "GET" });
    expect(calls).toContainEqual(["select_car", { car }]);
    expect(calls).toContainEqual(["add_manual", { address: "192.168.1.9" }]);
    expect(calls.map((c) => c[0])).toContain("rescan");
    expect(fetch).toHaveBeenCalledOnce();
  });

  it("subscribes to the state event the Rust loop emits", async () => {
    const { t, listen } = fakeTauri();
    const d = await loadDesktop(t);
    await d.onState(() => undefined);
    expect(listen).toHaveBeenCalledWith("racer://state", expect.any(Function));
  });
});
