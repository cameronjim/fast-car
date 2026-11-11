import { describe, expect, it } from "vitest";
import {
  ageText,
  batteryReading,
  bytesToBlobUrl,
  eventLine,
  fmt,
  fmtDuration,
  occupancyToImage,
  poseOf,
  pushEvent,
  quatToYaw,
  scanToPoints,
  speedFromOdom,
  steeringFromDrive,
  worldToPixel,
  type LidarMount,
  type OccupancyGridLike,
} from "./model";

// The mount used in these tests is the shape of vehicle_params' sensors.lidar. The UI itself
// has no copy of these numbers: they arrive from /api/vehicle.
const MOUNT: LidarMount = { mount_x_m: 0.285, mount_y_m: 0, mount_yaw_rad: Math.PI };

describe("speed and steering", () => {
  it("reads wheel speed from the odometry twist", () => {
    expect(speedFromOdom({ twist: { twist: { linear: { x: 1.25 } } } })).toBe(1.25);
    expect(speedFromOdom({ twist: { twist: { linear: { x: -0.4 } } } })).toBe(-0.4);
  });
  it("reads the steering the safety node allowed", () => {
    expect(steeringFromDrive({ drive: { steering_angle: 0.2, speed: 1.1 } })).toEqual({
      angleRad: 0.2,
      speedMps: 1.1,
    });
  });
});

describe("battery", () => {
  it.each([
    [12.6, "green"],
    [11.1, "green"],
    [11.09, "amber"],
    [10.5, "amber"],
    [10.49, "red"],
  ])("%f V on 3S is %s", (v, level) => {
    expect(batteryReading(v).level).toBe(level);
  });
  it("shows per cell voltage", () => {
    const r = batteryReading(12);
    expect(r.perCell).toBe(4);
    expect(r.text).toBe("12.00 V (4.00 V per cell)");
  });
  it("treats a missing or non-finite reading as red", () => {
    expect(batteryReading(null).level).toBe("red");
    expect(batteryReading(Number.NaN).level).toBe("red");
    expect(batteryReading(null).text).toBe("no reading");
  });
  it("honours another cell count", () => {
    expect(batteryReading(7.6, { cells: 2, cellGreenV: 3.7, cellRedV: 3.5 }).level).toBe("green");
  });
});

describe("scanToPoints", () => {
  const scan = (ranges: number[], angle_min = 0) => ({
    angle_min,
    angle_increment: Math.PI / 2,
    range_min: 0.05,
    range_max: 10,
    ranges,
  });

  it("with laser yaw pi, a return at laser bearing 0 is BEHIND the car", () => {
    // laser x points backwards (yaw pi), so 2 m at bearing 0 is 2 m behind the mount
    const pts = scanToPoints(scan([2]), MOUNT)!;
    expect(pts[0]).toBeCloseTo(0.285 - 2, 5);
    expect(pts[1]).toBeCloseTo(0, 5);
  });

  it("a laser bearing of pi (what sllidar calls straight ahead) lands in front of the car", () => {
    const pts = scanToPoints(scan([3], Math.PI), MOUNT)!;
    expect(pts[0]).toBeCloseTo(0.285 + 3, 5);
    expect(pts[1]).toBeCloseTo(0, 5);
  });

  it("keeps left positive: laser bearing -100 deg is on the driver's left (project convention)", () => {
    // vehicle bearing = laser bearing + pi = 80 deg, left of straight ahead, so y > 0
    const pts = scanToPoints({ ...scan([1]), angle_min: (-100 * Math.PI) / 180 }, MOUNT)!;
    expect(pts[1]).toBeGreaterThan(0.9);
    expect(pts[0]).toBeGreaterThan(0.285);
  });

  it("identity mount leaves the scan where it is, offset by x and y", () => {
    const pts = scanToPoints(scan([1]), { mount_x_m: 1, mount_y_m: 2, mount_yaw_rad: 0 })!;
    expect(Array.from(pts)).toEqual([2, 2]);
  });

  it("drops NaN, infinity, below range_min and above range_max", () => {
    const pts = scanToPoints(scan([Number.NaN, Infinity, 0.01, 11, 2, -1]), MOUNT)!;
    expect(pts.length).toBe(2);
  });

  it("returns null instead of guessing when the mount is missing", () => {
    expect(scanToPoints(scan([1]), null)).toBeNull();
    expect(scanToPoints(scan([1]), { mount_x_m: null, mount_y_m: 0, mount_yaw_rad: 0 })).toBeNull();
    expect(scanToPoints(scan([1]), { mount_x_m: 0, mount_y_m: null, mount_yaw_rad: 0 })).toBeNull();
    expect(scanToPoints(scan([1]), { mount_x_m: 0, mount_y_m: 0, mount_yaw_rad: null })).toBeNull();
  });

  it("an empty scan is an empty point list", () => {
    expect(scanToPoints(scan([]), MOUNT)!.length).toBe(0);
  });
});

describe("map and pose", () => {
  const grid: OccupancyGridLike = {
    info: {
      resolution: 0.5,
      width: 3,
      height: 2,
      origin: { position: { x: -1, y: -1 }, orientation: { x: 0, y: 0, z: 0, w: 1 } },
    },
    // row 0 is the bottom row (y = origin)
    data: [-1, 0, 100, 50, 0, -1],
  };

  it("draws north up: grid row 0 ends up at the bottom of the image", () => {
    const img = occupancyToImage(grid);
    expect([img.width, img.height]).toEqual([3, 2]);
    const px = (col: number, row: number) => Array.from(img.rgba.slice((row * 3 + col) * 4, (row * 3 + col) * 4 + 4));
    expect(px(0, 1)).toEqual([205, 205, 205, 255]); // unknown, bottom-left
    expect(px(2, 1)).toEqual([20, 20, 20, 255]); // wall 100, bottom-right
    expect(px(1, 1)).toEqual([250, 250, 250, 255]); // free
    expect(px(0, 0)[0]).toBe(135); // 50 is halfway between free and wall
    expect(px(2, 0)).toEqual([205, 205, 205, 255]); // unknown, top-right
  });

  it("clamps values above 100", () => {
    const img = occupancyToImage({ ...grid, data: [127, 0, 0, 0, 0, 0] });
    expect(img.rgba[12]).toBe(20); // grid row 0 is the image's bottom row
  });

  it("maps world metres to image pixels", () => {
    // origin (-1,-1), 0.5 m cells, height 2: the origin is at the bottom-left corner
    expect(worldToPixel(grid, -1, -1)).toEqual({ px: 0, py: 2 });
    expect(worldToPixel(grid, 0, 0)).toEqual({ px: 2, py: 0 });
    expect(worldToPixel(grid, -0.5, -0.5)).toEqual({ px: 1, py: 1 });
  });

  it("converts a quaternion to yaw, left positive", () => {
    expect(quatToYaw({ x: 0, y: 0, z: 0, w: 1 })).toBeCloseTo(0);
    expect(quatToYaw({ x: 0, y: 0, z: Math.SQRT1_2, w: Math.SQRT1_2 })).toBeCloseTo(Math.PI / 2);
    expect(quatToYaw({ x: 0, y: 0, z: 1, w: 0 })).toBeCloseTo(Math.PI);
  });

  it("reads a pose", () => {
    const p = poseOf({
      pose: { pose: { position: { x: 1, y: 2 }, orientation: { x: 0, y: 0, z: Math.SQRT1_2, w: Math.SQRT1_2 } } },
    });
    expect(p.x).toBe(1);
    expect(p.y).toBe(2);
    expect(p.yaw).toBeCloseTo(Math.PI / 2);
  });
});

describe("safety events", () => {
  const ev = (over = {}) => ({
    stamp: { sec: 10, nanosec: 5 },
    severity: 2,
    phase: 0,
    duration_s: 0,
    source: "ttc",
    detail: "clearance 0.12 m",
    ...over,
  });

  it("formats an engage", () => {
    const l = eventLine(ev(), 0);
    expect(l.text).toBe("ttc engage [BRAKE] clearance 0.12 m");
    expect(l.severity).toBe("brake");
  });
  it("formats a release with its duration", () => {
    const l = eventLine(ev({ phase: 1, duration_s: 0.4, severity: 0, detail: "" }), 1);
    expect(l.text).toBe("ttc release [info] (0.40 s)");
    expect(l.severity).toBe("info");
  });
  it("maps warning and unknown codes", () => {
    expect(eventLine(ev({ severity: 1 }), 2).severity).toBe("warning");
    expect(eventLine(ev({ severity: 9, phase: 7 }), 3).text).toContain("sev9");
  });
  it("gives each line a unique key even for identical stamps", () => {
    expect(eventLine(ev(), 1).key).not.toBe(eventLine(ev(), 2).key);
  });
  it("keeps the newest lines first and caps the list", () => {
    let lines = [] as ReturnType<typeof eventLine>[];
    for (let i = 0; i < 40; i++) lines = pushEvent(lines, eventLine(ev({ detail: String(i) }), i), 30);
    expect(lines.length).toBe(30);
    expect(lines[0].text).toContain("39");
  });
});

describe("small formatters", () => {
  it("fmt handles missing and non-finite values", () => {
    expect(fmt(1.234, 1, " m")).toBe("1.2 m");
    expect(fmt(null)).toBe("--");
    expect(fmt(undefined)).toBe("--");
    expect(fmt(Number.NaN)).toBe("--");
  });
  it("ageText", () => {
    expect(ageText(null)).toBe("never");
    expect(ageText(0.2)).toBe("live");
    expect(ageText(10)).toBe("10 s ago");
    expect(ageText(300)).toBe("5 min ago");
  });
  it("fmtDuration", () => {
    expect(fmtDuration(null)).toBe("--");
    expect(fmtDuration(300)).toBe("5 min");
    expect(fmtDuration(7200)).toBe("2.0 h");
  });
  it("makes a blob URL from a JPEG view without sharing the camera buffer", () => {
    const made: Blob[] = [];
    const orig = URL.createObjectURL;
    URL.createObjectURL = (b: Blob | MediaSource) => {
      made.push(b as Blob);
      return "blob:test";
    };
    try {
      const backing = new Uint8Array([9, 9, 1, 2, 3, 9]);
      const url = bytesToBlobUrl(backing.subarray(2, 5));
      expect(url).toBe("blob:test");
      expect(made[0].size).toBe(3);
      expect(made[0].type).toBe("image/jpeg");
    } finally {
      URL.createObjectURL = orig;
    }
  });
});
