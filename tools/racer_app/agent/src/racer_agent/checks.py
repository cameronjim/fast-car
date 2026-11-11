"""Pre-drive checks: pure functions from a `Snapshot` to green / amber / red.

No I/O in here. `collect.py` builds the Snapshot from docker, systemd, sysfs and the foxglove
socket; this module only judges it, so every threshold is table-testable.

A check is REQUIRED when a drive should not start while it is not green. Optional checks (Wi-Fi,
cameras, uptime, containers) are shown but never gate. `hard` marks the few that can never be
overridden, because overriding them would break CLAUDE.md invariant 5 (every run is logged):
the stack and its recorder must be running, and the disk must not be so full that the
recorder will die mid-run.
"""

from __future__ import annotations

import math
from dataclasses import dataclass, field

from .config import Thresholds
from .rosstate import RosSnapshot

GREEN, AMBER, RED = "green", "amber", "red"
_ORDER = {GREEN: 0, AMBER: 1, RED: 2}


def worst(levels: list[str]) -> str:
    return max(levels, key=lambda lv: _ORDER[lv], default=GREEN)


@dataclass(frozen=True)
class Check:
    id: str
    label: str
    level: str
    detail: str
    required: bool = True
    hard: bool = False


@dataclass(frozen=True)
class Snapshot:
    """Everything the checks look at. Built by collect.py; built by hand in tests."""

    stack_running: bool = False
    recorder_running: bool = False
    containers: dict[str, str] = field(default_factory=dict)  # name -> state
    stack_flags: dict[str, str] = field(default_factory=dict)
    disk_free_gb: float | None = None
    lidar_device: bool = False
    vesc_device: bool = False
    heartbeat_active: bool | None = None
    zones_c: dict[str, float] = field(default_factory=dict)
    wifi_dbm: float | None = None
    camera_devices: list[str] = field(default_factory=list)
    uptime_s: float | None = None
    ros: RosSnapshot | None = None


def _num(value: object) -> float | None:
    if isinstance(value, bool) or not isinstance(value, (int, float)):
        return None
    return float(value) if math.isfinite(float(value)) else None


# -- individual checks -----------------------------------------------------------------------


def check_stack(s: Snapshot) -> Check:
    if s.stack_running:
        return Check("stack", "Stack container", GREEN, "car-stack is running", hard=True)
    return Check("stack", "Stack container", RED, "car-stack is not running", hard=True)


def check_recorder(s: Snapshot) -> Check:
    if not s.stack_running:
        return Check("recorder", "Bag recorder", RED, "stack is not running", hard=True)
    if s.recorder_running:
        return Check("recorder", "Bag recorder", GREEN, "ros2 bag record is running", hard=True)
    return Check(
        "recorder",
        "Bag recorder",
        RED,
        "no recorder in the stack: a drive without a bag is a bug (invariant 5)",
        hard=True,
    )


def check_battery(s: Snapshot, t: Thresholds) -> Check:
    label = "Battery"
    volts = _num(s.ros.value("voltage_v")) if s.ros else None
    age = s.ros.value_age_s.get("voltage_v") if s.ros else None
    if volts is None or age is None or age > t.vesc_max_age_s * 3:
        return Check("battery", label, RED, "no VESC voltage reading")
    if volts < t.min_plausible_pack_v:
        return Check("battery", label, RED, f"{volts:.1f} V: no pack on the VESC (USB powered?)")
    cell = round(volts / t.battery_cells, 3)  # rounded so 11.1 V is exactly 3.70 per cell
    detail = f"{volts:.2f} V, {cell:.2f} V per cell ({t.battery_cells}S)"
    if cell > t.cell_max_v:
        return Check("battery", label, AMBER, detail + ", above a full cell: is this really 3S?")
    if cell >= t.cell_green_v:
        return Check("battery", label, GREEN, detail)
    if cell >= t.cell_red_v:
        return Check("battery", label, AMBER, detail + ", getting low")
    return Check("battery", label, RED, detail + ", too low: charge it")


def check_disk(s: Snapshot, t: Thresholds) -> Check:
    label = "Disk free"
    if s.disk_free_gb is None:
        return Check("disk", label, RED, "cannot read the disk", hard=True)
    free = s.disk_free_gb
    detail = f"{free:.1f} GiB free"
    if free < t.disk_red_gb:
        return Check(
            "disk", label, RED, detail + f", under {t.disk_red_gb:g}: no recording", hard=True
        )
    if free < t.disk_green_gb:
        return Check("disk", label, AMBER, detail + ", clear old bags soon", hard=True)
    return Check("disk", label, GREEN, detail, hard=True)


def check_lidar(s: Snapshot, t: Thresholds) -> Check:
    label = "LiDAR"
    if not s.lidar_device:
        return Check("lidar", label, RED, "/dev/ttyUSB0 is not there")
    if s.stack_flags.get("lidar") == "false":
        return Check("lidar", label, RED, "stack was started without the LiDAR")
    if not s.ros or not s.ros.connected:
        return Check("lidar", label, RED, "no ROS connection, cannot read /scan")
    hz = s.ros.rate("/scan")
    if hz >= t.scan_green_hz:
        return Check("lidar", label, GREEN, f"/scan at {hz:.1f} Hz")
    if hz >= t.scan_red_hz:
        return Check("lidar", label, AMBER, f"/scan only {hz:.1f} Hz")
    return Check("lidar", label, RED, "no /scan")


def check_vesc(s: Snapshot, t: Thresholds) -> Check:
    label = "VESC"
    if not s.vesc_device:
        return Check("vesc", label, RED, "/dev/ttyTHS1 is not there")
    if s.stack_flags.get("vesc") == "false":
        return Check("vesc", label, RED, "stack was started without the VESC")
    if not s.ros or not s.ros.connected:
        return Check("vesc", label, RED, "no ROS connection, cannot read telemetry")
    age = s.ros.value_age_s.get("voltage_v")
    fault = s.ros.value("fault")
    if age is None or age > t.vesc_max_age_s * 3:
        return Check("vesc", label, RED, "no VESC telemetry (UART silent)")
    if isinstance(fault, str) and fault.strip().upper() not in ("", "NONE", "FAULT_CODE_NONE"):
        return Check("vesc", label, RED, f"fault: {fault}")
    return Check("vesc", label, GREEN, "telemetry fresh, no fault")


def check_safety_node(s: Snapshot) -> Check:
    label = "Safety node"
    if not s.ros or not s.ros.connected:
        return Check("safety", label, RED, "no ROS connection")
    if "/drive" in s.ros.topics:
        return Check("safety", label, GREEN, "/drive is being advertised (safety_node up)")
    return Check("safety", label, RED, "nothing publishes /drive: safety_node is not up")


def check_heartbeat(s: Snapshot) -> Check:
    label = "Heartbeat service"
    if s.heartbeat_active is None:
        return Check("heartbeat", label, AMBER, "cannot ask systemd")
    if s.heartbeat_active:
        return Check("heartbeat", label, GREEN, "racer-heartbeat.service is active")
    return Check(
        "heartbeat", label, RED, "racer-heartbeat.service is not active: mux sees a dead Jetson"
    )


def check_temps(s: Snapshot, t: Thresholds) -> Check:
    label = "Temperatures"
    parts: list[str] = []
    levels: list[str] = []
    for name, temp in sorted(s.zones_c.items()):
        if "cpu" in name or "gpu" in name:
            parts.append(f"{name} {temp:.0f} C")
            if temp >= t.cpu_red_c:
                levels.append(RED)
            elif temp >= t.cpu_amber_c:
                levels.append(AMBER)
            else:
                levels.append(GREEN)
    if s.ros:
        fet = _num(s.ros.value("temp_fet_degc"))
        if fet is not None:
            parts.append(f"VESC FET {fet:.0f} C")
            levels.append(RED if fet >= t.vesc_red_c else AMBER if fet >= t.vesc_amber_c else GREEN)
        motor = _num(s.ros.value("temp_motor_degc"))
        if motor is not None and motor > 0:  # 0 means "no motor sensor"
            parts.append(f"motor {motor:.0f} C")
            levels.append(
                RED if motor >= t.vesc_red_c else AMBER if motor >= t.vesc_amber_c else GREEN
            )
    if not parts:
        return Check("temps", label, AMBER, "no temperature readings")
    return Check("temps", label, worst(levels), ", ".join(parts))


def check_wifi(s: Snapshot, t: Thresholds) -> Check:
    label = "Wi-Fi"
    if s.wifi_dbm is None:
        return Check("wifi", label, GREEN, "no wireless interface (wired?)", required=False)
    detail = f"{s.wifi_dbm:.0f} dBm"
    if s.wifi_dbm < t.wifi_red_dbm:
        return Check("wifi", label, RED, detail + ", STOP may not get through", required=False)
    if s.wifi_dbm < t.wifi_amber_dbm:
        return Check("wifi", label, AMBER, detail + ", weak", required=False)
    return Check("wifi", label, GREEN, detail, required=False)


def check_cameras(s: Snapshot) -> Check:
    label = "Cameras"
    running = s.containers.get("car-camera") == "running"
    devices = ", ".join(s.camera_devices) if s.camera_devices else "no camera devices"
    if running:
        return Check("cameras", label, GREEN, f"car-camera running ({devices})", required=False)
    return Check(
        "cameras", label, AMBER, f"car-camera is off ({devices}); optional", required=False
    )


def check_containers(s: Snapshot) -> Check:
    label = "Containers"
    if not s.containers:
        return Check("containers", label, AMBER, "none present", required=False)
    detail = ", ".join(f"{k}: {v}" for k, v in sorted(s.containers.items()))
    bad = [k for k, v in s.containers.items() if v not in ("running", "exited", "created")]
    return Check("containers", label, AMBER if bad else GREEN, detail, required=False)


def check_uptime(s: Snapshot, t: Thresholds) -> Check:
    label = "Uptime"
    if s.uptime_s is None:
        return Check("uptime", label, AMBER, "unknown", required=False)
    minutes = s.uptime_s / 60.0
    detail = f"up {minutes:.0f} min" if minutes < 120 else f"up {minutes / 60.0:.1f} h"
    if s.uptime_s < t.uptime_amber_s:
        return Check("uptime", label, AMBER, detail + ", just booted", required=False)
    return Check("uptime", label, GREEN, detail, required=False)


# -- the whole report ------------------------------------------------------------------------


@dataclass(frozen=True)
class Report:
    checks: list[Check]

    @property
    def overall(self) -> str:
        return worst([c.level for c in self.checks if c.required])

    @property
    def all_required_green(self) -> bool:
        return self.overall == GREEN

    @property
    def hard_failures(self) -> list[Check]:
        return [c for c in self.checks if c.hard and c.level == RED]

    @property
    def not_green_required(self) -> list[Check]:
        return [c for c in self.checks if c.required and c.level != GREEN]

    def as_dict(self) -> dict:
        return {
            "overall": self.overall,
            "ready": self.all_required_green,
            "checks": [
                {
                    "id": c.id,
                    "label": c.label,
                    "level": c.level,
                    "detail": c.detail,
                    "required": c.required,
                }
                for c in self.checks
            ],
        }


def evaluate(s: Snapshot, t: Thresholds) -> Report:
    return Report(
        [
            check_stack(s),
            check_recorder(s),
            check_battery(s, t),
            check_disk(s, t),
            check_safety_node(s),
            check_lidar(s, t),
            check_vesc(s, t),
            check_heartbeat(s),
            check_temps(s, t),
            check_wifi(s, t),
            check_cameras(s),
            check_containers(s),
            check_uptime(s, t),
        ]
    )
