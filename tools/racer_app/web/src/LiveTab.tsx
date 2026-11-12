// The Live tab: speed, steering, battery, VESC, the LiDAR view, the map, cameras and safety
// events. Values that stop arriving are kept, greyed out, with their age, never silently blanked.

import { useEffect, useRef } from "preact/hooks";
import { useApp } from "./ctx";
import { CAMERA_CSI, CAMERA_USB, type Live } from "./live";
import {
  batteryReading,
  fmt,
  lapSubline,
  occupancyToImage,
  scanToPoints,
  worldToPixel,
  type VehicleInfo,
} from "./model";
import { Card, Skeleton } from "./ui";
import { ageLabel, freshness, type Freshness } from "./view";

function Age<T>({ f }: { f: Freshness<T> }) {
  return f.stale && f.value !== null ? <span class="age">{ageLabel(f.ageS)}</span> : null;
}

export function LiveTab({ live, vehicle }: { live: Live; vehicle: VehicleInfo | null }) {
  const c = useApp();
  return (
    <div class="stack-col">
      {c.status && !c.status.stack.running && (
        <div class="warnline">No live data while the stack is off. Start it from Drive or System.</div>
      )}
      <Gauges live={live} vehicle={vehicle} />
      <div class="grid wide">
        <LidarView live={live} vehicle={vehicle} />
        <MapView live={live} />
        <Cameras live={live} />
        <EventsFeed live={live} />
      </div>
    </div>
  );
}

function Gauges({ live, vehicle }: { live: Live; vehicle: VehicleInfo | null }) {
  const c = useApp();
  const now = c.now;
  const speed = freshness(live.speed, now);
  const steer = freshness(live.steering, now);
  const volts = freshness(live.voltage, now);
  const fet = freshness(live.tempFet, now);
  const motor = freshness(live.tempMotor, now);
  const amps = freshness(live.currentIn, now);
  const fault = freshness(live.fault, now);
  const lap = freshness(live.lap, now);
  const cells = c.status?.thresholds.battery_cells ?? 3;
  const battery = batteryReading(volts.value, { cells, cellGreenV: 3.7, cellRedV: 3.5 });
  const maxSpeed = vehicle?.actuation.throttle_full_scale_mps ?? null;
  const maxSteer = vehicle?.steering.max_angle_rad ?? null;
  const faultText = fault.value;
  const faultBad = faultText !== null && !["", "NONE", "FAULT_CODE_NONE"].includes(faultText.trim().toUpperCase());
  const steerDeg = steer.value ? (steer.value.angleRad * 180) / Math.PI : null;
  const loading = !c.status;
  return (
    <section class="gauges" aria-label="Gauges">
      <div class="gauge">
        <div class="label">Speed <Age f={speed} /></div>
        {loading ? <Skeleton h={36} /> : <div class={`big ${speed.stale ? "stale" : ""}`}>{fmt(speed.value, 2)}<small>m/s</small></div>}
        <Bar value={speed.value === null ? null : Math.abs(speed.value)} max={maxSpeed} stale={speed.stale} />
      </div>
      <div class="gauge">
        <div class="label">Steering <Age f={steer} /></div>
        {loading ? <Skeleton h={36} /> : <div class={`big ${steer.stale ? "stale" : ""}`}>{fmt(steerDeg, 1)}<small>deg</small></div>}
        <SteerBar angle={steer.value?.angleRad ?? null} max={maxSteer} stale={steer.stale} />
        <div class="sub">Left is positive</div>
      </div>
      <div class="gauge">
        <div class="label">Battery <Age f={volts} /></div>
        {loading ? (
          <Skeleton h={36} />
        ) : (
          <div class={`big ${volts.stale ? "stale" : `lvl-${battery.level}`}`}>{volts.value === null ? "--" : battery.volts?.toFixed(2)}<small>V</small></div>
        )}
        <div class="sub">{battery.perCell === null ? "No VESC voltage" : `${battery.perCell.toFixed(2)} V per cell, ${cells}S`}</div>
      </div>
      <div class="gauge">
        <div class="label">VESC <Age f={fault} /></div>
        {loading ? (
          <Skeleton h={30} />
        ) : (
          <div class={`big text ${fault.stale ? "stale" : faultBad ? "lvl-red" : ""}`}>{faultText === null ? "No data" : faultBad ? faultText : "No fault"}</div>
        )}
        <div class="sub num">
          FET {fmt(fet.value, 0, " C")} / motor {motor.value && motor.value > 0 ? fmt(motor.value, 0, " C") : "no sensor"} / {fmt(amps.value, 1, " A")}
        </div>
      </div>
      {lap.value && (
        <div class="gauge">
          <div class="label">Laps <Age f={lap} /></div>
          <div class={`big ${lap.stale ? "stale" : ""}`}>{lap.value.laps}</div>
          <div class="sub">{lapSubline(lap.value)}</div>
        </div>
      )}
    </section>
  );
}

function Bar({ value, max, stale }: { value: number | null; max: number | null; stale: boolean }) {
  const pct = value === null || !max ? 0 : Math.min(100, (value / max) * 100);
  return (
    <div class={`bar ${stale ? "stale" : ""}`}><div class="fill" style={{ width: `${pct}%` }} /></div>
  );
}

function SteerBar({ angle, max, stale }: { angle: number | null; max: number | null; stale: boolean }) {
  const frac = angle === null || !max ? 0 : Math.max(-1, Math.min(1, angle / max));
  // left positive: the fill grows to the left of centre for a left turn
  const style = frac >= 0 ? { right: "50%", left: "auto", width: `${frac * 50}%` } : { left: "50%", width: `${-frac * 50}%` };
  return (
    <div class={`bar centre ${stale ? "stale" : ""}`}><div class="fill" style={style} /></div>
  );
}

// -- canvases --------------------------------------------------------------------------------

function LidarView({ live, vehicle }: { live: Live; vehicle: VehicleInfo | null }) {
  const c = useApp();
  const ref = useRef<HTMLCanvasElement>(null);
  const scan = freshness(live.scan, c.now);
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
    if (scan.value) {
      const pts = scanToPoints(scan.value, vehicle?.lidar ?? null);
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
    <Card
      title="LiDAR"
      label="LiDAR"
      info={`Vehicle frame, front is up, rings every metre. Mount from vehicle_params${vehicle?.schema_version ? ` ${vehicle.schema_version}` : ""}.${unknownMount ? " The LiDAR mount is not set there, so the scan is not drawn." : ""}`}
      badge={<span class={`muted ${scan.stale && scan.value ? "lvl-amber" : ""}`}>{scan.value ? (scan.stale ? `stale, ${ageLabel(scan.ageS)}` : "live") : "no /scan"}</span>}
    >
      <canvas ref={ref} width={420} height={420} class={`square ${scan.stale ? "stale" : ""}`} />
    </Card>
  );
}

function MapView({ live }: { live: Live }) {
  const c = useApp();
  const ref = useRef<HTMLCanvasElement>(null);
  const grid = live.map;
  const pose = freshness(live.pose, c.now);
  const poseValue = pose.value;
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
    if (poseValue) {
      const { px, py } = worldToPixel(grid, poseValue.x, poseValue.y);
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
      ctx.lineTo(x + Math.cos(poseValue.yaw) * 14, y - Math.sin(poseValue.yaw) * 14); // map y is up
      ctx.stroke();
    }
  });
  return (
    <Card
      title="Map"
      info="The /map topic with the car's /pose (red) while mapping or localisation runs."
      badge={<span class="muted">{pose.value ? (pose.stale ? `pose ${ageLabel(pose.ageS)}` : "pose live") : grid ? "no pose" : "no map"}</span>}
    >
      {grid ? (
        <canvas ref={ref} width={420} height={420} class={`square ${pose.stale ? "stale" : ""}`} />
      ) : (
        <div class="empty">No map. Start the stack with mapping or localisation.</div>
      )}
      {poseValue && <div class="sub num">x {poseValue.x.toFixed(2)} y {poseValue.y.toFixed(2)} m</div>}
    </Card>
  );
}

function Cameras({ live }: { live: Live }) {
  return (
    <Card title="Cameras" info="About one frame a second, only while this tab is open. The bag records its own slower stream.">
      <div class="camgrid">
        <figure>
          {live.camUsb ? <img src={live.camUsb} alt="USB camera" /> : <div class="ph">No {CAMERA_USB}</div>}
          <figcaption>USB global shutter</figcaption>
        </figure>
        <figure>
          {live.camCsi ? <img src={live.camCsi} alt="CSI camera" /> : <div class="ph">No {CAMERA_CSI}</div>}
          <figcaption>CSI camera 0</figcaption>
        </figure>
      </div>
    </Card>
  );
}

function EventsFeed({ live }: { live: Live }) {
  return (
    <Card
      title="Safety events"
      label="Safety events"
      badge={<span class="muted">{live.events.length ? `${live.events.length} shown` : "none yet"}</span>}
      info="What safety_node reported since this page opened. BRAKE is red, warnings amber."
    >
      <ul class="events">
        {live.events.map((e) => (
          <li key={e.key} class={`ev-${e.severity}`}>{e.text}</li>
        ))}
        {live.events.length === 0 && <li class="empty">Nothing yet.</li>}
      </ul>
    </Card>
  );
}
