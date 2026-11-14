import { describe, expect, it } from "vitest";
import type { BagsReply } from "./api";
import { carHost, copyCommand, diskBar, fmtGb, fmtRunDate } from "./data";
import { modeLabel } from "./view";

const reply = (over: Partial<BagsReply> = {}): BagsReply => ({
  bags: [],
  disk_free_gb: 100,
  disk: { total_gb: 200, used_gb: 100, free_gb: 100 },
  bags_mb: 1073.741824 * 20, // 20 GiB of runs
  ...over,
});

describe("diskBar", () => {
  it("splits the disk into runs, other and free", () => {
    const b = diskBar(reply());
    expect(b.freePct).toBeCloseTo(50);
    expect(b.bagsPct).toBeCloseTo(10);
    expect(b.otherPct).toBeCloseTo(40);
    expect(b.bagsGb).toBeCloseTo(20);
    expect(b.level).toBe("green");
  });

  it("goes amber then red as the disk fills", () => {
    expect(diskBar(reply({ disk: { total_gb: 200, used_gb: 190, free_gb: 10 } })).level).toBe("amber");
    expect(diskBar(reply({ disk: { total_gb: 200, used_gb: 197, free_gb: 3 } })).level).toBe("red");
  });

  it("never adds up past 100 percent, and copes with no disk reading", () => {
    const b = diskBar(reply({ bags_mb: 1073.741824 * 500 }));
    expect(b.bagsPct + b.otherPct + b.freePct).toBeLessThanOrEqual(100.0001);
    const none = diskBar(reply({ disk: undefined, disk_free_gb: null }));
    expect(none).toMatchObject({ bagsPct: 0, freePct: 0, level: "amber", totalGb: null });
    expect(diskBar(null).freeGb).toBeNull();
  });

  it("falls back to summing the runs when the agent sends no total", () => {
    const b = diskBar(reply({ bags_mb: undefined, bags: [{ name: "a", size_mb: 1073.741824, mtime: 0, active: false }] }));
    expect(b.bagsGb).toBeCloseTo(1);
  });
});

describe("copyCommand", () => {
  it("builds the scp line for one run", () => {
    expect(copyCommand("racer", "racer-car.local", "/home/racer/car/data/bags/", "2026-11-13T10-31-27_car_teleop")).toBe(
      "scp -r racer@racer-car.local:/home/racer/car/data/bags/2026-11-13T10-31-27_car_teleop ~/Downloads/",
    );
    expect(copyCommand("", "10.0.0.5", "/data", "x")).toBe("scp -r 10.0.0.5:/data/x ~/Downloads/");
  });

  it("offers nothing rather than a command that would break in a shell", () => {
    expect(copyCommand("racer", "car", "/data", "a b")).toBeNull();
    expect(copyCommand("racer", "car", "/data", "x;rm -rf ~")).toBeNull();
    expect(copyCommand("racer", "car", "/da ta", "x")).toBeNull();
    expect(copyCommand("racer", "", "/data", "x")).toBeNull();
    expect(copyCommand("racer", "car", undefined, "x")).toBeNull();
  });
});

describe("formatting", () => {
  it("formats sizes and dates", () => {
    expect(fmtGb(null)).toBe("--");
    expect(fmtGb(142.4)).toBe("142 GB");
    expect(fmtGb(9.66)).toBe("9.7 GB");
    const text = fmtRunDate(Date.UTC(2026, 10, 13, 15, 2) / 1000, "en-GB");
    expect(text).toMatch(/13/);
    expect(text).toMatch(/\d\d:\d\d$/);
  });

  it("finds the car's host from the API base or the page", () => {
    expect(carHost("http://10.0.0.226:8080", "localhost")).toBe("10.0.0.226");
    expect(carHost("", "racer-car.local")).toBe("racer-car.local");
    expect(carHost("not a url", undefined)).toBe("");
  });

  it("names drive modes in plain words", () => {
    expect(modeLabel("gap_follow:lane-2026-10-25")).toBe("Gap follow");
    expect(modeLabel("pnp")).toBe("ML driver");
    expect(modeLabel("tracker:track:gym_oval")).toBe("Racer");
    expect(modeLabel("gap_follow (started from a terminal)")).toBe("Gap follow");
    expect(modeLabel("something_new")).toBe("something_new");
  });
});
