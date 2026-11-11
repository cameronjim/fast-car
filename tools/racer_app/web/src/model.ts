// Pure data handling for the dashboard: decoded ROS messages in, numbers and pixels out.
// No DOM and no sockets in here, so every function is unit tested (model.test.ts).

export interface LidarMount {
  mount_x_m: number | null;
  mount_y_m: number | null;
  mount_yaw_rad: number | null;
}

export interface VehicleInfo {
  schema_version: string | null;
  lidar: LidarMount;
  chassis: {
    wheelbase_m: number | null;
    length_m: number | null;
    width_m: number | null;
    front_overhang_m: number | null;
    rear_overhang_m: number | null;
  };
  steering: { max_angle_rad: number | null };
  actuation: { throttle_full_scale_mps: number | null };
}

export interface LaserScanLike {
  angle_min: number;
  angle_increment: number;
  range_min: number;
  range_max: number;
  ranges: ArrayLike<number>;
}

export interface OdometryLike {
  twist: { twist: { linear: { x: number } } };
}

export interface DriveLike {
  drive: { steering_angle: number; speed: number };
}

export interface OccupancyGridLike {
  info: {
    resolution: number;
    width: number;
    height: number;
    origin: { position: { x: number; y: number }; orientation: Quat };
  };
  data: ArrayLike<number>;
}

export interface Quat {
  x: number;
  y: number;
  z: number;
  w: number;
}

export interface PoseStampedLike {
  pose: { pose: { position: { x: number; y: number }; orientation: Quat } };
}

export interface SafetyEventLike {
  stamp: { sec: number; nanosec: number };
  severity: number;
  phase: number;
  duration_s: number;
  source: string;
  detail: string;
}

export function speedFromOdom(msg: OdometryLike): number {
  return msg.twist.twist.linear.x;
}

export function steeringFromDrive(msg: DriveLike): { angleRad: number; speedMps: number } {
  return { angleRad: msg.drive.steering_angle, speedMps: msg.drive.speed };
}

// -- battery ---------------------------------------------------------------------------------

export type Level = "green" | "amber" | "red";

export interface BatteryThresholds {
  cells: number;
  cellGreenV: number;
  cellRedV: number;
}

// Same numbers as the agent's pre-drive check (agent config.Thresholds). The agent's check is
// the one that gates driving; this is only the colour of the gauge.
export const DEFAULT_BATTERY: BatteryThresholds = { cells: 3, cellGreenV: 3.7, cellRedV: 3.5 };

export function batteryReading(volts: number | null, t: BatteryThresholds = DEFAULT_BATTERY) {
  if (volts === null || !Number.isFinite(volts)) {
    return { volts: null, perCell: null, level: "red" as Level, text: "no reading" };
  }
  const perCell = Math.round((volts / t.cells) * 1000) / 1000;
  let level: Level = "green";
  if (perCell < t.cellRedV) level = "red";
  else if (perCell < t.cellGreenV) level = "amber";
  return {
    volts,
    perCell,
    level,
    text: `${volts.toFixed(2)} V (${perCell.toFixed(2)} V per cell)`,
  };
}

// -- lidar -----------------------------------------------------------------------------------

/**
 * LaserScan to points in the vehicle frame (x forward, y left), as [x0, y0, x1, y1, ...].
 *
 * The mount (x, y, yaw) comes from vehicle_params via the agent, never from this file. The
 * laser frame is rotated by `yaw` and moved by (x, y) relative to base_link. Returns that
 * the mount is missing as `null`, so the caller can say so instead of drawing a guess.
 */
export function scanToPoints(scan: LaserScanLike, mount: LidarMount | null): Float32Array | null {
  if (!mount || mount.mount_x_m === null || mount.mount_y_m === null || mount.mount_yaw_rad === null) {
    return null;
  }
  const { mount_x_m: mx, mount_y_m: my, mount_yaw_rad: yaw } = mount;
  const cy = Math.cos(yaw);
  const sy = Math.sin(yaw);
  const out = new Float32Array(scan.ranges.length * 2);
  let n = 0;
  for (let i = 0; i < scan.ranges.length; i++) {
    const r = scan.ranges[i];
    if (!Number.isFinite(r) || r < scan.range_min || r > scan.range_max) continue;
    const a = scan.angle_min + i * scan.angle_increment;
    const lx = r * Math.cos(a);
    const ly = r * Math.sin(a);
    out[n++] = mx + lx * cy - ly * sy;
    out[n++] = my + lx * sy + ly * cy;
  }
  return out.slice(0, n);
}

// -- map and pose ----------------------------------------------------------------------------

export function quatToYaw(q: Quat): number {
  return Math.atan2(2 * (q.w * q.z + q.x * q.y), 1 - 2 * (q.y * q.y + q.z * q.z));
}

export function poseOf(msg: PoseStampedLike): { x: number; y: number; yaw: number } {
  const p = msg.pose.pose;
  return { x: p.position.x, y: p.position.y, yaw: quatToYaw(p.orientation) };
}

const UNKNOWN = [205, 205, 205];
const FREE = [250, 250, 250];
const WALL = [20, 20, 20];

/** OccupancyGrid to an RGBA image, row 0 at the TOP (north up) for canvas drawing. */
export function occupancyToImage(grid: OccupancyGridLike): { width: number; height: number; rgba: Uint8ClampedArray<ArrayBuffer> } {
  const { width, height } = grid.info;
  const rgba = new Uint8ClampedArray(new ArrayBuffer(width * height * 4));
  for (let row = 0; row < height; row++) {
    const srcRow = height - 1 - row; // grid row 0 is the bottom (y = origin)
    for (let col = 0; col < width; col++) {
      const v = grid.data[srcRow * width + col];
      let c = UNKNOWN;
      if (v >= 0) {
        const t = Math.min(100, v) / 100;
        c = [0, 1, 2].map((k) => Math.round(FREE[k] + (WALL[k] - FREE[k]) * t));
      }
      const o = (row * width + col) * 4;
      rgba[o] = c[0];
      rgba[o + 1] = c[1];
      rgba[o + 2] = c[2];
      rgba[o + 3] = 255;
    }
  }
  return { width, height, rgba };
}

/** World (map frame, metres) to image pixel for an occupancyToImage picture. */
export function worldToPixel(grid: OccupancyGridLike, x: number, y: number): { px: number; py: number } {
  const { resolution, height, origin } = grid.info;
  return {
    px: (x - origin.position.x) / resolution,
    py: height - (y - origin.position.y) / resolution,
  };
}

// -- safety events ---------------------------------------------------------------------------

const SEVERITY = ["info", "warning", "BRAKE"];
const PHASE = ["engage", "release"];

export interface EventLine {
  key: string;
  text: string;
  severity: "info" | "warning" | "brake";
}

export function eventLine(ev: SafetyEventLike, counter: number): EventLine {
  const sev = SEVERITY[ev.severity] ?? `sev${ev.severity}`;
  const phase = PHASE[ev.phase] ?? `phase${ev.phase}`;
  const dur = ev.phase === 1 ? ` (${ev.duration_s.toFixed(2)} s)` : "";
  return {
    key: `${ev.stamp.sec}.${ev.stamp.nanosec}-${counter}`,
    text: `${ev.source} ${phase} [${sev}]${dur} ${ev.detail}`.trim(),
    severity: ev.severity >= 2 ? "brake" : ev.severity === 1 ? "warning" : "info",
  };
}

/** Keep the newest `max` lines, newest first. */
export function pushEvent(lines: EventLine[], line: EventLine, max = 30): EventLine[] {
  return [line, ...lines].slice(0, max);
}

// -- small helpers ---------------------------------------------------------------------------

export function fmt(value: number | null | undefined, digits = 2, unit = ""): string {
  if (value === null || value === undefined || !Number.isFinite(value)) return "--";
  return `${value.toFixed(digits)}${unit}`;
}

export function ageText(seconds: number | null): string {
  if (seconds === null) return "never";
  if (seconds < 1.5) return "live";
  if (seconds < 60) return `${seconds.toFixed(0)} s ago`;
  return `${(seconds / 60).toFixed(0)} min ago`;
}

export function bytesToBlobUrl(data: ArrayBufferView, mime = "image/jpeg"): string {
  const copy = new Uint8Array(data.buffer, data.byteOffset, data.byteLength).slice();
  return URL.createObjectURL(new Blob([copy], { type: mime }));
}

export function fmtDuration(seconds: number | null): string {
  if (seconds === null) return "--";
  if (seconds < 3600) return `${Math.round(seconds / 60)} min`;
  return `${(seconds / 3600).toFixed(1)} h`;
}
