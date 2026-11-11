import { useEffect, useRef } from "preact/hooks";
import type { Check, Status } from "./api";
import { CAMERA_CSI, CAMERA_USB, type Live } from "./live";
import {
  batteryReading,
  fmt,
  fmtDuration,
  occupancyToImage,
  scanToPoints,
  worldToPixel,
  type BatteryThresholds,
  type VehicleInfo,
} from "./model";

const STALE_MS = 2500;

function fresh<T>(s: { value: T; at: number } | null): T | null {
  return s && Date.now() - s.at < STALE_MS ? s.value : null;
}

export function Gauges({ live, status, vehicle }: { live: Live; status: Status | null; vehicle: VehicleInfo | null }) {
  const speed = fresh(live.speed);
  const steer = fresh(live.steering);
  const volts = fresh(live.voltage);
  const battery = batteryReading(volts, {
    cells: status?.thresholds.battery_cells ?? 3,
    cellGreenV: 3.7,
    cellRedV: 3.5,
  } satisfies BatteryThresholds);
  const maxSpeed = vehicle?.actuation.throttle_full_scale_mps ?? null;
  const maxSteer = vehicle?.steering.max_angle_rad ?? null;
  const fet = fresh(live.tempFet);
  const motor = fresh(live.tempMotor);
  const fault = fresh(live.fault);
  const faultBad = fault !== null && !["", "NONE", "FAULT_CODE_NONE"].includes(fault.trim().toUpperCase());
  const steerDeg = steer ? (steer.angleRad * 180) / Math.PI : null;
  return (
    <section class="card gauges" aria-label="Gauges">
      <div class="gauge">
        <h3>Speed</h3>
        <div class="big">{fmt(speed, 2)}<small> m/s</small></div>
        <Bar value={speed === null ? null : Math.abs(speed)} max={maxSpeed} />
        <div class="sub">wheel speed from the VESC{maxSpeed ? `, full scale ${maxSpeed} m/s` : ""}</div>
      </div>
      <div class="gauge">
        <h3>Steering</h3>
        <div class="big">{fmt(steerDeg, 1)}<small> deg</small></div>
        <SteerBar angle={steer?.angleRad ?? null} max={maxSteer} />
        <div class="sub">what safety_node let through (/drive), left is positive</div>
      </div>
      <div class="gauge">
        <h3>Battery</h3>
        <div class={`big lvl-${battery.level}`}>{volts === null ? "--" : battery.volts?.toFixed(2)}<small> V</small></div>
        <div class="sub">{battery.perCell === null ? "no VESC voltage" : `${battery.perCell.toFixed(2)} V per cell (${status?.thresholds.battery_cells ?? 3}S)`}</div>
      </div>
      <div class="gauge">
        <h3>VESC</h3>
        <div class={`big ${faultBad ? "lvl-red" : ""}`}>{fault === null ? "no data" : faultBad ? fault : "no fault"}</div>
        <div class="sub">FET {fmt(fet, 0, " C")}, motor {motor && motor > 0 ? fmt(motor, 0, " C") : "no sensor"}, in {fmt(fresh(live.currentIn), 1, " A")}</div>
      </div>
    </section>
  );
}

function Bar({ value, max }: { value: number | null; max: number | null }) {
  const pct = value === null || !max ? 0 : Math.min(100, (value / max) * 100);
  return (
    <div class="bar"><div class="fill" style={{ width: `${pct}%` }} /></div>
  );
}

function SteerBar({ angle, max }: { angle: number | null; max: number | null }) {
  const frac = angle === null || !max ? 0 : Math.max(-1, Math.min(1, angle / max));
  // left positive: the fill grows to the left of centre for a left turn
  const style = frac >= 0 ? { right: "50%", width: `${frac * 50}%` } : { left: "50%", width: `${-frac * 50}%` };
  return (
    <div class="bar centre"><div class="fill" style={style} /></div>
  );
}

// -- canvases --------------------------------------------------------------------------------

export function LidarView({ live, vehicle }: { live: Live; vehicle: VehicleInfo | null }) {
  const ref = useRef<HTMLCanvasElement>(null);
  const scan = fresh(live.scan);
  useEffect(() => {
    const canvas = ref.current;
    if (!canvas) return;
    const ctx = canvas.getContext("2d");
    if (!ctx) return;
    const css = getComputedStyle(canvas);
    const w = canvas.width;
    const h = canvas.height;
    ctx.clearRect(0, 0, w, h);
    const range = 4; // metres from the rear axle to the edge of the view, each way
    const scale = Math.min(w, h) / 2 / range;
    // vehicle frame: x forward = canvas up, y left = canvas left; origin at the middle
    const cx = w / 2;
    const cy = h / 2;
    const toPx = (x: number, y: number): [number, number] => [cx - y * scale, cy - x * scale];
    ctx.strokeStyle = css.getPropertyValue("--grid") || "#8883";
    ctx.lineWidth = 1;
    for (let r = 1; r <= range; r++) {
      ctx.beginPath();
      ctx.arc(cx, cy, r * scale, 0, Math.PI * 2);
      ctx.stroke();
    }
    ctx.beginPath();
    ctx.moveTo(cx, 0);
    ctx.lineTo(cx, h);
    ctx.moveTo(0, cy);
    ctx.lineTo(w, cy);
    ctx.stroke();
    const ch = vehicle?.chassis;
    if (ch?.length_m && ch.width_m && ch.wheelbase_m) {
      const front = ch.wheelbase_m + (ch.front_overhang_m ?? 0);
      const rear = -(ch.rear_overhang_m ?? 0);
      const [x0, y0] = toPx(front, ch.width_m / 2);
      const [x1, y1] = toPx(rear, -ch.width_m / 2);
      ctx.fillStyle = css.getPropertyValue("--car") || "#2a7";
      ctx.fillRect(Math.min(x0, x1), Math.min(y0, y1), Math.abs(x1 - x0), Math.abs(y1 - y0));
    }
    if (scan) {
      const pts = scanToPoints(scan, vehicle?.lidar ?? null);
      if (pts) {
        ctx.fillStyle = css.getPropertyValue("--accent") || "#08f";
        for (let i = 0; i < pts.length; i += 2) {
          const [px, py] = toPx(pts[i], pts[i + 1]);
          ctx.fillRect(px - 1, py - 1, 2.5, 2.5);
        }
      }
    }
  });
  const unknownMount = vehicle && (vehicle.lidar.mount_x_m === null || vehicle.lidar.mount_yaw_rad === null);
  return (
    <section class="card" aria-label="LiDAR">
      <h3>LiDAR <span class="muted">{scan ? "live" : "no /scan"}</span></h3>
      <canvas ref={ref} width={420} height={420} class="square" />
      <div class="sub">
        {unknownMount ? "LiDAR mount is not set in vehicle_params: scan not drawn. " : "Vehicle frame, front is up, rings every metre. "}
        Mount from vehicle_params{vehicle?.schema_version ? ` ${vehicle.schema_version}` : ""}.
      </div>
    </section>
  );
}

export function MapView({ live }: { live: Live }) {
  const ref = useRef<HTMLCanvasElement>(null);
  const grid = live.map;
  const pose = fresh(live.pose);
  useEffect(() => {
    const canvas = ref.current;
    if (!canvas || !grid) return;
    const ctx = canvas.getContext("2d");
    if (!ctx) return;
    const img = occupancyToImage(grid);
    const off = document.createElement("canvas");
    off.width = img.width;
    off.height = img.height;
    off.getContext("2d")?.putImageData(new ImageData(img.rgba, img.width, img.height), 0, 0);
    const k = Math.min(canvas.width / img.width, canvas.height / img.height);
    ctx.clearRect(0, 0, canvas.width, canvas.height);
    ctx.imageSmoothingEnabled = false;
    ctx.drawImage(off, 0, 0, img.width * k, img.height * k);
    if (pose) {
      const { px, py } = worldToPixel(grid, pose.x, pose.y);
      const x = px * k;
      const y = py * k;
      ctx.fillStyle = "#e33";
      ctx.beginPath();
      ctx.arc(x, y, 5, 0, Math.PI * 2);
      ctx.fill();
      ctx.strokeStyle = "#e33";
      ctx.lineWidth = 2;
      ctx.beginPath();
      ctx.moveTo(x, y);
      ctx.lineTo(x + Math.cos(pose.yaw) * 14, y - Math.sin(pose.yaw) * 14); // map y is up
      ctx.stroke();
    }
  });
  return (
    <section class="card" aria-label="Map">
      <h3>Map <span class="muted">{pose ? "pose live" : grid ? "no pose" : "no map"}</span></h3>
      <canvas ref={ref} width={420} height={420} class="square" />
      <div class="sub">/map with /pose (red) when mapping or localisation runs. {pose ? `x ${pose.x.toFixed(2)} y ${pose.y.toFixed(2)} m` : ""}</div>
    </section>
  );
}

export function Cameras({ live }: { live: Live }) {
  return (
    <section class="card cams" aria-label="Cameras">
      <h3>Cameras <span class="muted">about 1 frame a second</span></h3>
      <div class="camgrid">
        <figure>
          {live.camUsb ? <img src={live.camUsb} alt="USB camera" /> : <div class="ph">no {CAMERA_USB}</div>}
          <figcaption>USB global shutter</figcaption>
        </figure>
        <figure>
          {live.camCsi ? <img src={live.camCsi} alt="CSI camera" /> : <div class="ph">no {CAMERA_CSI}</div>}
          <figcaption>CSI camera 0</figcaption>
        </figure>
      </div>
    </section>
  );
}

export function EventsFeed({ live }: { live: Live }) {
  return (
    <section class="card" aria-label="Safety events">
      <h3>Safety events <span class="muted">{live.events.length ? `${live.events.length} shown` : "none yet"}</span></h3>
      <ul class="events">
        {live.events.map((e) => (
          <li key={e.key} class={`ev-${e.severity}`}>{e.text}</li>
        ))}
        {live.events.length === 0 && <li class="muted">Nothing from safety_node since this page opened.</li>}
      </ul>
    </section>
  );
}

export function Checks({ checks, status }: { checks: Check[]; status: Status | null }) {
  return (
    <section class="card" aria-label="Pre-drive checks">
      <h3>
        Pre-drive checks{" "}
        <span class={`pill lvl-${status?.checks.overall ?? "red"}`}>{status?.checks.ready ? "ready" : "not ready"}</span>
      </h3>
      <ul class="checks">
        {checks.map((c) => (
          <li key={c.id}>
            <span class={`dot lvl-${c.level}`} aria-label={c.level} />
            <b>{c.label}</b>
            {!c.required && <span class="muted"> optional</span>}
            <div class="sub">{c.detail}</div>
          </li>
        ))}
      </ul>
      {status && (
        <div class="sub">
          Disk {fmt(status.disk_free_gb, 1, " GiB")} free, up {fmtDuration(status.uptime_s)}.
        </div>
      )}
    </section>
  );
}
