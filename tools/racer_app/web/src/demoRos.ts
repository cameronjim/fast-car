// A fake ROS link for demo mode: same interface the app uses on the real foxglove socket
// (subscribe, status, connect, close), emitting plausible values while the demo car's stack is up.
// Subscribe only, like the real link. Nothing here can publish.

import type { DemoCar } from "./demoAgent";
import { LIVE_TOPICS, CAMERA_CSI, CAMERA_USB } from "./live";
import type { LinkStatus, RosSource, SubscribeOptions, Unsubscribe } from "./ros";

type Handler = (msg: unknown) => void;

const TOPICS_UP = [...Object.values(LIVE_TOPICS), CAMERA_USB, CAMERA_CSI].sort();

/** A rounded corridor loop: distance from the middle of the track to the wall along a bearing. */
function scanRanges(t: number, n = 360): number[] {
  const out: number[] = [];
  for (let i = 0; i < n; i++) {
    const a = (i / n) * Math.PI * 2 - Math.PI;
    // walls 0.55 m left and right, opening ahead, a corner swinging with time
    const side = 0.55 / Math.max(0.12, Math.abs(Math.sin(a)));
    const ahead = 3.4 + 0.9 * Math.sin(t / 3);
    const front = Math.cos(a) > 0.05 ? ahead / Math.cos(a) : Infinity;
    const back = Math.cos(a) < -0.05 ? 2.2 / -Math.cos(a) : Infinity;
    const r = Math.min(side, front, back) + (((i * 7919) % 13) - 6) * 0.004;
    out.push(Math.max(0.15, Math.min(r, 8)));
  }
  return out;
}

function svgFrame(label: string, hue: number, t: number): Uint8Array {
  const x = 40 + ((Math.sin(t / 2) + 1) / 2) * 240;
  const svg =
    `<svg xmlns="http://www.w3.org/2000/svg" width="320" height="240" viewBox="0 0 320 240">` +
    `<defs><linearGradient id="g" x1="0" y1="0" x2="0" y2="1"><stop offset="0" stop-color="hsl(${hue} 25% 38%)"/>` +
    `<stop offset="1" stop-color="hsl(${hue} 20% 18%)"/></linearGradient></defs>` +
    `<rect width="320" height="240" fill="url(#g)"/><rect y="150" width="320" height="90" fill="hsl(${hue} 10% 30%)"/>` +
    `<path d="M0 240 L130 150 L190 150 L320 240Z" fill="hsl(${hue} 8% 22%)"/>` +
    `<circle cx="${x.toFixed(0)}" cy="120" r="16" fill="hsl(${hue + 150} 50% 55%)"/>` +
    `<text x="10" y="22" font-family="monospace" font-size="13" fill="#fffa">${label}</text></svg>`;
  return new TextEncoder().encode(svg);
}

export class DemoRos implements RosSource {
  private handlers = new Map<string, Set<Handler>>();
  private statusListeners = new Set<(s: LinkStatus) => void>();
  private timer: ReturnType<typeof setInterval> | null = null;
  private connected = false;
  private t0 = Date.now();
  private mapSent = false;
  private lastCam = 0;
  private lastLap = 0;

  constructor(private car: DemoCar) {}

  onStatus(cb: (s: LinkStatus) => void): Unsubscribe {
    this.statusListeners.add(cb);
    cb(this.status());
    return () => this.statusListeners.delete(cb);
  }

  status(): LinkStatus {
    return { connected: this.connected, topics: this.connected ? TOPICS_UP : [] };
  }

  connect(): void {
    if (this.timer) return;
    this.timer = setInterval(() => this.tick(), 100);
  }

  close(): void {
    if (this.timer) clearInterval(this.timer);
    this.timer = null;
  }

  subscribe<T = unknown>(topic: string, handler: (msg: T) => void, _opts: SubscribeOptions = {}): Unsubscribe {
    const set = this.handlers.get(topic) ?? new Set<Handler>();
    set.add(handler as Handler);
    this.handlers.set(topic, set);
    return () => set.delete(handler as Handler);
  }

  private emit(topic: string, msg: unknown): void {
    this.handlers.get(topic)?.forEach((h) => h(msg));
  }

  private setConnected(c: boolean): void {
    if (c === this.connected) return;
    this.connected = c;
    if (!c) this.mapSent = false;
    const s = this.status();
    this.statusListeners.forEach((cb) => cb(s));
  }

  private tick(): void {
    const car = this.car;
    this.setConnected(car.rosUp());
    if (!this.connected) return;
    const now = Date.now();
    const t = (now - this.t0) / 1000;
    const driving = !!car.driveMode;
    const speed = driving ? 1.1 + 0.25 * Math.sin(t / 2.2) : 0;
    const steer = driving ? 0.22 * Math.sin(t / 1.6) : 0;
    const volts = car.lowBattery ? 10.9 : 11.8 - 0.002 * (t % 120);
    if (car.nodesUp) {
      this.emit(LIVE_TOPICS.odom, { twist: { twist: { linear: { x: speed } } } });
      this.emit(LIVE_TOPICS.drive, { drive: { steering_angle: steer, speed } });
      this.emit(LIVE_TOPICS.voltage, { data: volts });
      this.emit(LIVE_TOPICS.currentIn, { data: driving ? 3.2 + Math.sin(t) * 0.6 : 0.4 });
      this.emit(LIVE_TOPICS.tempFet, { data: 41 + (driving ? t % 20 : 0) * 0.2 });
      this.emit(LIVE_TOPICS.tempMotor, { data: 0 });
      this.emit(LIVE_TOPICS.fault, { data: "FAULT_CODE_NONE" });
      this.emit(LIVE_TOPICS.scan, {
        angle_min: -Math.PI,
        angle_increment: (Math.PI * 2) / 360,
        range_min: 0.12,
        range_max: 12,
        ranges: scanRanges(t),
      });
      if (driving && now - this.lastLap > 1000) {
        this.lastLap = now;
        this.emit(LIVE_TOPICS.lap, {
          data: JSON.stringify({ state: "running", similarity: 0.91, laps: Math.floor(t / 14), last_lap_s: 13.8, best_lap_s: 13.2 }),
        });
      }
      if (car.flags.localization === "true" || car.flags.mapping === "true" || driving) {
        if (!this.mapSent) {
          this.mapSent = true;
          this.emit(LIVE_TOPICS.map, demoGrid());
        }
        const ang = t / 6;
        this.emit(LIVE_TOPICS.pose, {
          pose: { pose: { position: { x: 1.6 * Math.cos(ang), y: 1.0 * Math.sin(ang) }, orientation: { x: 0, y: 0, z: Math.sin((ang + Math.PI / 2) / 2), w: Math.cos((ang + Math.PI / 2) / 2) } } },
        });
      }
    }
    for (const e of car.drainEvents()) {
      this.emit(LIVE_TOPICS.events, {
        stamp: { sec: Math.floor(now / 1000), nanosec: (now % 1000) * 1e6 },
        severity: e.severity,
        phase: e.phase,
        duration_s: e.durationS,
        source: e.source,
        detail: e.detail,
      });
    }
    if (car.cameras && now - this.lastCam > 900) {
      this.lastCam = now;
      this.emit(CAMERA_USB, { data: svgFrame("usb global shutter", 205, t), mime: "image/svg+xml" });
      this.emit(CAMERA_CSI, { data: svgFrame("csi 0", 30, t + 3), mime: "image/svg+xml" });
    }
  }
}

/** A ring-shaped track in an occupancy grid, 100 x 70 cells at 5 cm. */
export function demoGrid(): { info: { resolution: number; width: number; height: number; origin: { position: { x: number; y: number }; orientation: { x: number; y: number; z: number; w: number } } }; data: Int8Array } {
  const width = 100;
  const height = 70;
  const res = 0.05;
  const data = new Int8Array(width * height).fill(-1);
  const cx = width / 2;
  const cy = height / 2;
  for (let y = 0; y < height; y++) {
    for (let x = 0; x < width; x++) {
      const dx = (x - cx) / 38;
      const dy = (y - cy) / 25;
      const r = Math.sqrt(dx * dx + dy * dy);
      if (r < 0.9) data[y * width + x] = r < 0.48 ? 100 : 0;
      else if (r < 0.99) data[y * width + x] = 100;
    }
  }
  return {
    info: { resolution: res, width, height, origin: { position: { x: -cx * res, y: -cy * res }, orientation: { x: 0, y: 0, z: 0, w: 1 } } },
    data,
  };
}
