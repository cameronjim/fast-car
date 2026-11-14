import { describe, expect, it, vi } from "vitest";
import { Api, ApiError, browserTokenStore, type TokenStore } from "./api";

function memoryTokens(initial: string | null = null): TokenStore & { value: string | null } {
  const store = {
    value: initial,
    get: () => store.value,
    set: (t: string) => {
      store.value = t;
    },
    clear: () => {
      store.value = null;
    },
  };
  return store;
}

function reply(status: number, body: unknown) {
  return new Response(JSON.stringify(body), { status, headers: { "Content-Type": "application/json" } });
}

describe("Api", () => {
  it("sends the bearer token and JSON bodies", async () => {
    const fetchImpl = vi.fn().mockResolvedValue(reply(200, { ok: true, message: "stack starting" }));
    const api = new Api(memoryTokens("abc"), fetchImpl);
    await api.startStack({ lidar: true, vesc: true, mapping: false, localization: false, map: "" });
    const [path, init] = fetchImpl.mock.calls[0];
    expect(path).toBe("/api/stack/start");
    expect(init.method).toBe("POST");
    expect(init.headers).toMatchObject({ Authorization: "Bearer abc", "Content-Type": "application/json" });
    expect(JSON.parse(init.body)).toEqual({ lidar: true, vesc: true, mapping: false, localization: false, map: "" });
  });

  it("GETs carry no body and no content type", async () => {
    const fetchImpl = vi.fn().mockResolvedValue(reply(200, {}));
    await new Api(memoryTokens("t"), fetchImpl).status();
    const init = fetchImpl.mock.calls[0][1];
    expect(init.method).toBe("GET");
    expect(init.body).toBeUndefined();
    expect(init.headers["Content-Type"]).toBeUndefined();
  });

  it("STOP ALL works without a token and never sends one", async () => {
    const fetchImpl = vi.fn().mockResolvedValue(reply(200, { ok: true }));
    await new Api(memoryTokens("stale"), fetchImpl).stopAll();
    const [path, init] = fetchImpl.mock.calls[0];
    expect(path).toBe("/api/stop");
    expect(init.headers.Authorization).toBeUndefined();
  });

  it("turns a refusal into an ApiError that keeps the override hint", async () => {
    const body = { ok: false, error: "pre-drive checks are not all green: ...", overridable: true, failing: ["disk"] };
    const api = new Api(memoryTokens("t"), vi.fn().mockResolvedValue(reply(409, body)));
    const err = await api.startDrive({ mode: "wall_follow" }).catch((e) => e);
    expect(err).toBeInstanceOf(ApiError);
    expect(err.status).toBe(409);
    expect(err.message).toContain("not all green");
    expect(err.body.overridable).toBe(true);
  });

  it("starts the ML policy with the policy and raceline it was given, as a normal drive start", async () => {
    const fetchImpl = vi.fn().mockResolvedValue(reply(200, { ok: true, message: "ml_policy:v1 started" }));
    await new Api(memoryTokens("t"), fetchImpl).startDrive({ mode: "ml_policy", raceline: "data:lane1", policy: "v1-2026-11-11" });
    const [path, init] = fetchImpl.mock.calls[0];
    expect(path).toBe("/api/drive/start");
    expect(JSON.parse(init.body)).toEqual({ mode: "ml_policy", raceline: "data:lane1", policy: "v1-2026-11-11" });
    expect(init.headers.Authorization).toBe("Bearer t");
  });

  it("starts Plug-and-Play through the normal drive route with bundle names, backend and cap only", async () => {
    const fetchImpl = vi.fn().mockResolvedValue(reply(200, { ok: true, message: "pnp:geometric started" }));
    await new Api(memoryTokens("t"), fetchImpl).startDrive({
      mode: "pnp",
      pnp_backend: "geometric",
      pnp_dynamics: "dyn-v0-2026-11-12",
      pnp_perception: "",
      pnp_v_max_mps: 0.4,
    });
    const [path, init] = fetchImpl.mock.calls[0];
    expect(path).toBe("/api/drive/start");
    expect(JSON.parse(init.body)).toEqual({
      mode: "pnp",
      pnp_backend: "geometric",
      pnp_dynamics: "dyn-v0-2026-11-12",
      pnp_perception: "",
      pnp_v_max_mps: 0.4,
    });
    expect(init.headers.Authorization).toBe("Bearer t");
  });

  it("sends the lap learning fields as given and nothing else", async () => {
    const fetchImpl = vi.fn().mockResolvedValue(reply(200, { ok: true, message: "pnp:geometric started" }));
    const body = {
      mode: "pnp",
      pnp_backend: "geometric",
      pnp_dynamics: "dyn-v0-2026-11-12",
      pnp_v_max_mps: 0.8,
      pnp_v_lap1_mps: 0.4,
      pnp_lap_memory: true,
      pnp_lap_mode: "v1" as const,
      pnp_lap_source: "start_scan" as const,
      pnp_lap_counter: false,
      pnp_require_lap_counter: false,
      pnp_shadow_perception: "trackperc-shadow",
    };
    await new Api(memoryTokens("t"), fetchImpl).startDrive(body);
    const [path, init] = fetchImpl.mock.calls[0];
    expect(path).toBe("/api/drive/start");
    expect(JSON.parse(init.body)).toEqual(body);
  });

  it("a disabled Plug-and-Play reply keeps the reason for the UI", async () => {
    const body = { ok: false, error: "Plug-and-Play is disabled: no dynamics bundle" };
    const api = new Api(memoryTokens("t"), vi.fn().mockResolvedValue(reply(409, body)));
    const err = await api.startDrive({ mode: "pnp", pnp_backend: "geometric" }).catch((e) => e);
    expect(err).toBeInstanceOf(ApiError);
    expect(err.status).toBe(409);
    expect(err.message).toContain("no dynamics bundle");
  });

  it("perception and spiderbot go through the fixed routes with their fields", async () => {
    const fetchImpl = vi.fn().mockResolvedValue(reply(200, { ok: true }));
    const api = new Api(memoryTokens("t"), fetchImpl);
    await api.startPerception({ mode: "pose", device: "cuda" });
    await api.stopPerception();
    await api.startDrive({ mode: "spiderbot", patrol: "loop", rule: "arms_raised", speed_mps: 0.5 });
    const calls = fetchImpl.mock.calls.map((c) => `${c[1].method} ${c[0]}`);
    expect(calls).toEqual([
      "POST /api/perception/start",
      "POST /api/perception/stop",
      "POST /api/drive/start",
    ]);
    expect(JSON.parse(fetchImpl.mock.calls[0][1].body)).toEqual({ mode: "pose", device: "cuda" });
    expect(JSON.parse(fetchImpl.mock.calls[2][1].body)).toEqual({
      mode: "spiderbot",
      patrol: "loop",
      rule: "arms_raised",
      speed_mps: 0.5,
    });
    for (const c of fetchImpl.mock.calls) expect(c[1].headers.Authorization).toBe("Bearer t");
  });

  it("a disabled spiderbot reply keeps the reason for the UI", async () => {
    const body = { ok: false, error: "the spiderbot is disabled: the GPU image car:perception is not built" };
    const api = new Api(memoryTokens("t"), vi.fn().mockResolvedValue(reply(409, body)));
    const err = await api.startDrive({ mode: "spiderbot", patrol: "loop" }).catch((e) => e);
    expect(err.status).toBe(409);
    expect(err.message).toContain("car:perception");
  });

  it("a 401 calls onUnauthorized", async () => {
    const api = new Api(memoryTokens("bad"), vi.fn().mockResolvedValue(reply(401, { ok: false, error: "missing or wrong token" })));
    const handler = vi.fn();
    api.onUnauthorized = handler;
    await expect(api.status()).rejects.toThrow("missing or wrong token");
    expect(handler).toHaveBeenCalledOnce();
  });

  it("a network failure is status 0, and a non-JSON error still has a message", async () => {
    const down = new Api(memoryTokens("t"), vi.fn().mockRejectedValue(new TypeError("failed to fetch")));
    const err = await down.status().catch((e) => e);
    expect(err.status).toBe(0);
    const html = new Api(memoryTokens("t"), vi.fn().mockResolvedValue(new Response("<html>", { status: 502 })));
    await expect(html.status()).rejects.toThrow("HTTP 502");
  });

  it("builds the whitelisted routes with encoded names", async () => {
    const fetchImpl = vi.fn().mockResolvedValue(reply(200, { ok: true }));
    const api = new Api(memoryTokens("t"), fetchImpl);
    await api.park("three_point_turn");
    await api.park("../x");
    await api.deleteBag("2026-11-10T10:00_car_teleop");
    await api.power("reboot");
    await api.setAutostart(false);
    await api.saveMap("floor-1");
    await api.startCameras();
    await api.stopCameras();
    await api.stopStack();
    await api.restartStack();
    await api.log(5);
    const calls = fetchImpl.mock.calls.map((c) => `${c[1].method} ${c[0]}`);
    expect(calls).toEqual([
      "POST /api/park/three_point_turn",
      "POST /api/park/..%2Fx",
      "POST /api/bags/delete",
      "POST /api/system/reboot",
      "POST /api/settings",
      "POST /api/map/save",
      "POST /api/cameras/start",
      "POST /api/cameras/stop",
      "POST /api/stack/stop",
      "POST /api/stack/restart",
      "GET /api/log?limit=5",
    ]);
    expect(JSON.parse(fetchImpl.mock.calls[2][1].body)).toEqual({
      name: "2026-11-10T10:00_car_teleop",
      confirm: "2026-11-10T10:00_car_teleop",
    });
    expect(JSON.parse(fetchImpl.mock.calls[3][1].body)).toEqual({ confirm: "reboot" });
  });

  it("trims a pasted token and reports whether one is stored", () => {
    const tokens = memoryTokens();
    const api = new Api(tokens, vi.fn());
    expect(api.hasToken()).toBe(false);
    api.setToken("  secret \n");
    expect(tokens.value).toBe("secret");
    expect(api.hasToken()).toBe(true);
  });
});

describe("browserTokenStore", () => {
  it("works when localStorage throws (private windows)", () => {
    const original = globalThis.localStorage;
    Object.defineProperty(globalThis, "localStorage", {
      configurable: true,
      get() {
        throw new Error("blocked");
      },
    });
    try {
      const store = browserTokenStore();
      expect(store.get()).toBeNull();
      expect(() => store.set("x")).not.toThrow();
      expect(() => store.clear()).not.toThrow();
    } finally {
      Object.defineProperty(globalThis, "localStorage", { configurable: true, value: original });
    }
  });

  it("round-trips through localStorage when available", () => {
    const data = new Map<string, string>();
    Object.defineProperty(globalThis, "localStorage", {
      configurable: true,
      value: {
        getItem: (k: string) => data.get(k) ?? null,
        setItem: (k: string, v: string) => void data.set(k, v),
        removeItem: (k: string) => void data.delete(k),
      },
    });
    const store = browserTokenStore();
    store.set("tok");
    expect(store.get()).toBe("tok");
    store.clear();
    expect(store.get()).toBeNull();
  });
});
