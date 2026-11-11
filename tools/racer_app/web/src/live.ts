// Wiring from the ROS link to one mutable "latest values" object the dashboard reads.

import type { RosLink, Unsubscribe } from "./ros";
import {
  bytesToBlobUrl,
  eventLine,
  pushEvent,
  poseOf,
  speedFromOdom,
  steeringFromDrive,
  type DriveLike,
  type EventLine,
  type LaserScanLike,
  type OccupancyGridLike,
  type OdometryLike,
  type PoseStampedLike,
  type SafetyEventLike,
} from "./model";

export interface Stamped<T> {
  value: T;
  at: number; // Date.now() when it arrived
}

export interface Live {
  speed: Stamped<number> | null;
  steering: Stamped<{ angleRad: number; speedMps: number }> | null;
  voltage: Stamped<number> | null;
  currentIn: Stamped<number> | null;
  tempFet: Stamped<number> | null;
  tempMotor: Stamped<number> | null;
  fault: Stamped<string> | null;
  scan: Stamped<LaserScanLike> | null;
  map: OccupancyGridLike | null;
  pose: Stamped<{ x: number; y: number; yaw: number }> | null;
  events: EventLine[];
  camUsb: string | null;
  camCsi: string | null;
}

export function emptyLive(): Live {
  return {
    speed: null,
    steering: null,
    voltage: null,
    currentIn: null,
    tempFet: null,
    tempMotor: null,
    fault: null,
    scan: null,
    map: null,
    pose: null,
    events: [],
    camUsb: null,
    camCsi: null,
  };
}

export const CAMERA_USB = "/camera/usb/image_raw/compressed";
export const CAMERA_CSI = "/camera/csi0/image_raw/compressed";
const CAMERA_PERIOD_MS = 700;

interface FloatMsg {
  data: number;
}
interface StringMsg {
  data: string;
}
interface CompressedImageLike {
  data: ArrayBufferView;
}

export function startLive(ros: RosLink, live: Live, showCameras: () => boolean): Unsubscribe {
  const now = () => Date.now();
  let eventCounter = 0;
  const unsubs: Unsubscribe[] = [];
  const num = (topic: string, key: "voltage" | "currentIn" | "tempFet" | "tempMotor") =>
    ros.subscribe<FloatMsg>(topic, (m) => {
      live[key] = { value: m.data, at: now() };
    });

  unsubs.push(
    ros.subscribe<OdometryLike>("/odom/wheel", (m) => {
      live.speed = { value: speedFromOdom(m), at: now() };
    }),
    ros.subscribe<DriveLike>("/drive", (m) => {
      live.steering = { value: steeringFromDrive(m), at: now() };
    }),
    num("/telemetry/vesc/voltage_v", "voltage"),
    num("/telemetry/vesc/current_input_a", "currentIn"),
    num("/telemetry/vesc/temp_fet_degc", "tempFet"),
    num("/telemetry/vesc/temp_motor_degc", "tempMotor"),
    ros.subscribe<StringMsg>("/telemetry/vesc/fault", (m) => {
      live.fault = { value: m.data, at: now() };
    }),
    ros.subscribe<LaserScanLike>("/scan", (m) => {
      live.scan = { value: m, at: now() };
    }),
    ros.subscribe<OccupancyGridLike>("/map", (m) => {
      live.map = m;
    }),
    ros.subscribe<PoseStampedLike>("/pose", (m) => {
      live.pose = { value: poseOf(m), at: now() };
    }),
    ros.subscribe<SafetyEventLike>("/safety/events", (m) => {
      live.events = pushEvent(live.events, eventLine(m, eventCounter++));
    }),
  );

  // Cameras: one frame every CAMERA_PERIOD_MS, and none at all while the card is hidden.
  const camera = (topic: string, key: "camUsb" | "camCsi") =>
    ros.subscribe<CompressedImageLike>(
      topic,
      (m) => {
        if (!showCameras()) return;
        const url = bytesToBlobUrl(m.data);
        const old = live[key];
        live[key] = url;
        if (old) URL.revokeObjectURL(old);
      },
      { oneShotEveryMs: CAMERA_PERIOD_MS },
    );
  unsubs.push(camera(CAMERA_USB, "camUsb"), camera(CAMERA_CSI, "camCsi"));

  return () => unsubs.forEach((u) => u());
}
