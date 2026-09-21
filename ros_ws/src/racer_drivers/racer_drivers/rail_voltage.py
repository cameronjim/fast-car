"""ROS-free parsing/discovery for the Jetson's onboard INA3221 power monitor.

Roadmap 1.6 / CLAUDE.md invariant 5: "Every run is logged (rosbag + rail voltage)."
claude-docs/05-safety.md, Operational rules: "Rail voltage logged on every run; a brownout
must never be mistakable for a control failure."

The INA226 that claude-docs/11-hardware.md specs for the compute rail is NOT fitted yet. The
Jetson Orin Nano carries its own INA3221 on the carrier board, and the kernel exposes it
through the standard Linux hwmon sysfs ABI at

    /sys/bus/i2c/drivers/ina3221/<bus>-<addr>/hwmon/hwmon<N>/
        in<C>_label     e.g. "VDD_IN", "VDD_CPU_GPU_CV", "VDD_SOC"
        in<C>_input     bus voltage, MILLIVOLTS   (hwmon ABI)
        curr<C>_input   channel current, MILLIAMPS (hwmon ABI)

This module is the pure half of rail_voltage_node (claude-docs/10-conventions.md: "Gate/
decision logic is always separated from node plumbing so it is testable without ROS"). It
does file discovery and string parsing only; it never imports rclpy and never raises on a
missing or malformed file -- every reader returns None instead, and the node decides what to
log. That matters because this is a LOGGING driver: it must be impossible for it to take the
stack down (claude-docs/05-safety.md's layering -- logging must not weaken layers 1-3).

UNITS (CLAUDE.md invariant 4: SI everywhere in code and messages). The millivolt/milliamp to
volt/amp conversion happens HERE, at the driver boundary, which is exactly where invariant 4
says a non-SI wire format must be converted. The factor of 1000 is the hwmon sysfs ABI
(Documentation/hwmon/sysfs-interface.rst), a kernel interface constant -- it is NOT a vehicle
parameter and deliberately does not live in config/vehicle_params.yaml (claude-docs/06-
vehicle-params.md: that file holds physical constants of THIS CAR; a kernel ABI is neither
physical nor per-car).
"""

from __future__ import annotations

import glob
import os
import re
from dataclasses import dataclass

#: Default location of the in-tree ina3221 driver's sysfs entries on JetPack 6 / L4T R36.
#: Overridable as a node parameter so tests can point at a fixture tree of ordinary files,
#: the same pattern pwm_output_node's `sysfs_root` uses.
DEFAULT_INA3221_ROOT = "/sys/bus/i2c/drivers/ina3221"

#: hwmon ABI scale factors. See this module's docstring on why they are not vehicle params.
_MILLIVOLTS_PER_VOLT = 1000.0
_MILLIAMPS_PER_AMP = 1000.0

_LABEL_FILE_RE = re.compile(r"^in(\d+)_label$")
_SLUG_RE = re.compile(r"[^a-z0-9]+")


@dataclass(frozen=True)
class RailChannel:
    """One INA3221 channel: its hwmon index, its board label, and the topic slug for it."""

    index: int
    label: str

    @property
    def slug(self) -> str:
        return slugify_label(self.label)


@dataclass(frozen=True)
class RailReading:
    """One sample of one channel. `volts`/`amps` are None when that file was unreadable."""

    channel: RailChannel
    volts: float | None
    amps: float | None


def slugify_label(label: str) -> str:
    """Board label -> a ROS-topic-safe name fragment. "VDD_CPU_GPU_CV" -> "vdd_cpu_gpu_cv"."""
    slug = _SLUG_RE.sub("_", label.strip().lower()).strip("_")
    return slug or "unlabelled"


def parse_millivolts(text: str | None) -> float | None:
    """hwmon in<C>_input (millivolts, integer) -> volts. None on anything unparseable."""
    return _scaled(text, _MILLIVOLTS_PER_VOLT)


def parse_milliamps(text: str | None) -> float | None:
    """hwmon curr<C>_input (milliamps, signed integer) -> amps. None on unparseable."""
    return _scaled(text, _MILLIAMPS_PER_AMP)


def parse_label(text: str | None) -> str | None:
    """hwmon in<C>_label -> the trimmed board label. None when empty or absent."""
    if text is None:
        return None
    label = text.strip()
    return label or None


def _scaled(text: str | None, divisor: float) -> float | None:
    if text is None:
        return None
    stripped = text.strip()
    if not stripped:
        return None
    try:
        # float(), not int(): the ABI says integer milli-units, but a driver that ever emits
        # "11952.0" should be read, not dropped. A non-numeric value still returns None.
        return float(stripped) / divisor
    except ValueError:
        return None


def read_text(path: str) -> str | None:
    """Read a sysfs attribute. None on ANY OS-level failure -- missing file, EACCES, EIO.

    sysfs reads can fail transiently (the i2c transaction behind in<C>_input can return
    -EIO), so a failed read is an ordinary outcome here, not an exception.
    """
    try:
        with open(path, "r", encoding="utf-8") as handle:
            return handle.read()
    except OSError:
        return None


def find_hwmon_dir(root: str = DEFAULT_INA3221_ROOT) -> str | None:
    """Locate the hwmon directory under an ina3221 driver root, or None if not fitted.

    The i2c bus-address directory and the hwmon index are both enumeration-order dependent
    (`1-0040`, `hwmon3`, ...), so they are globbed rather than assumed. The lowest-sorting
    match wins, which makes the choice deterministic on a device with more than one INA3221.
    """
    matches = sorted(glob.glob(os.path.join(root, "*", "hwmon", "hwmon*")))
    for candidate in matches:
        if os.path.isdir(candidate):
            return candidate
    return None


def discover_channels(hwmon_dir: str) -> list[RailChannel]:
    """Every labelled channel in an hwmon directory, ordered by hwmon index.

    Only channels with BOTH an in<C>_label and an in<C>_input are returned: the ina3221
    driver also exposes unlabelled shunt-voltage attributes on some kernels, and a channel
    with no voltage file is not a rail this node can report.
    """
    try:
        entries = os.listdir(hwmon_dir)
    except OSError:
        return []

    channels: list[RailChannel] = []
    for entry in sorted(entries):
        match = _LABEL_FILE_RE.match(entry)
        if match is None:
            continue
        index = int(match.group(1))
        label = parse_label(read_text(os.path.join(hwmon_dir, entry)))
        if label is None:
            continue
        if not os.path.exists(os.path.join(hwmon_dir, f"in{index}_input")):
            continue
        channels.append(RailChannel(index=index, label=label))
    return sorted(channels, key=lambda channel: channel.index)


def read_channel(hwmon_dir: str, channel: RailChannel) -> RailReading:
    """Sample one channel. Volts and amps are independently None-able: the INA3221's current
    files are absent on some kernels, and a rail with no current reading is still a rail
    whose VOLTAGE invariant 5 wants logged."""
    volts = parse_millivolts(read_text(os.path.join(hwmon_dir, f"in{channel.index}_input")))
    amps = parse_milliamps(read_text(os.path.join(hwmon_dir, f"curr{channel.index}_input")))
    return RailReading(channel=channel, volts=volts, amps=amps)


def read_all(hwmon_dir: str, channels: list[RailChannel]) -> list[RailReading]:
    return [read_channel(hwmon_dir, channel) for channel in channels]
