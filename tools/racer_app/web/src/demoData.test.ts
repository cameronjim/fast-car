// The demo car answers the v3 endpoints the same way the agent does, so the demo (and the
// screenshots) show real behaviour: saved settings, the camera toggle, disk usage, cleanup.

import { describe, expect, it } from "vitest";
import { Api, type TokenStore } from "./api";
import { DemoCar } from "./demoAgent";
import { DEFAULT_PREFS } from "./prefs";

const tokens: TokenStore = { get: () => "demo", set: () => undefined, clear: () => undefined };
function rig() {
  const car = new DemoCar(() => Date.now(), (ms) => new Promise((r) => setTimeout(r, ms)), 0.001);
  return { car, api: new Api(tokens, car.fetch) };
}

describe("demo car, v3 endpoints", () => {
  it("saves settings and returns them with the defaults", async () => {
    const { api } = rig();
    const first = await api.prefs();
    expect(first.prefs).toEqual(DEFAULT_PREFS);
    const next = { ...structuredClone(DEFAULT_PREFS), record_camera: false };
    next.gap_follow.max_speed_mps = 0.5;
    expect((await api.savePrefs(next)).prefs).toEqual(next);
    expect((await api.prefs()).prefs).toEqual(next);
    expect((await api.prefs()).defaults).toEqual(DEFAULT_PREFS);
  });

  it("refuses an out-of-bounds speed like the agent", async () => {
    const { api } = rig();
    const bad = structuredClone(DEFAULT_PREFS);
    bad.gap_follow.max_speed_mps = 3;
    await expect(api.savePrefs(bad)).rejects.toMatchObject({ status: 400 });
  });

  it("the camera reports the record setting it was started with", async () => {
    const { api } = rig();
    expect((await api.status()).cameras).toEqual({ running: false, record: null });
    await api.savePrefs({ ...structuredClone(DEFAULT_PREFS), record_camera: false });
    await api.startCameras();
    expect((await api.status()).cameras).toEqual({ running: true, record: false });
    // a later change does not pretend the running cameras changed
    await api.savePrefs(structuredClone(DEFAULT_PREFS));
    expect((await api.status()).cameras.record).toBe(false);
  });

  it("reports disk usage and the copy hint, refuses deleting the recording run", async () => {
    const { car, api } = rig();
    car.preset("running");
    const b = await api.bags();
    expect(b.disk?.total_gb).toBeGreaterThan(0);
    expect(b.user).toBe("racer");
    const active = b.bags.find((x) => x.active)!;
    await expect(api.deleteBag(active.name)).rejects.toMatchObject({ status: 409 });
    const old = b.bags.find((x) => !x.active)!;
    await api.deleteBag(old.name);
    expect((await api.bags()).bags.some((x) => x.name === old.name)).toBe(false);
  });

  it("cleanup returns the raw output", async () => {
    const { api } = rig();
    const r = await api.cleanup();
    expect(r.message).toContain("reclaimed");
    expect(r.output).toContain("docker image prune -f");
  });
});
