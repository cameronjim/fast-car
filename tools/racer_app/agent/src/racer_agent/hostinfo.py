"""Read-only facts about the Jetson host: files, devices, temperatures, disk.

Everything reads under `root` (default `/`) so tests can build a fake sysfs/proc tree in a
temporary directory instead of mocking individual calls.
"""

from __future__ import annotations

import grp
import os
import shutil
from collections.abc import Callable
from dataclasses import dataclass
from pathlib import Path

GIB = 1024**3


@dataclass(frozen=True)
class VideoDevice:
    node: str  # /dev/videoN
    name: str
    index: int  # v4l2 index: 0 is the capture node, 1 the metadata node on a UVC camera


def _default_group_lookup(name: str) -> int | None:
    try:
        return grp.getgrnam(name).gr_gid
    except KeyError:
        return None


def _default_disk_usage(path: str) -> tuple[int, int, int]:
    usage = shutil.disk_usage(path)
    return usage.total, usage.used, usage.free


class HostInfo:
    def __init__(
        self,
        root: Path = Path("/"),
        uid: int | None = None,
        gid: int | None = None,
        group_lookup: Callable[[str], int | None] = _default_group_lookup,
        disk_usage: Callable[[str], tuple[int, int, int]] = _default_disk_usage,
    ) -> None:
        self.root = Path(root)
        self._uid = os.getuid() if uid is None else uid
        self._gid = os.getgid() if gid is None else gid
        self._group_lookup = group_lookup
        self._disk_usage = disk_usage

    # -- identity ------------------------------------------------------------------------
    @property
    def uid(self) -> int:
        return self._uid

    @property
    def gid(self) -> int:
        return self._gid

    def group_id(self, name: str) -> int | None:
        return self._group_lookup(name)

    # -- files ---------------------------------------------------------------------------
    def path(self, absolute: str) -> Path:
        return self.root / absolute.lstrip("/")

    def exists(self, absolute: str) -> bool:
        return self.path(absolute).exists()

    def read(self, absolute: str) -> str | None:
        try:
            return self.path(absolute).read_text().strip()
        except (OSError, UnicodeDecodeError, TypeError):  # EAGAIN reads surface as TypeError
            return None

    # -- disk ----------------------------------------------------------------------------
    def _usage(self, directory: Path) -> tuple[int, int, int] | None:
        probe = Path(directory)
        while not probe.exists() and probe != probe.parent:
            probe = probe.parent
        try:
            return self._disk_usage(str(probe))
        except OSError:
            return None

    def disk_free_gb(self, directory: Path) -> float | None:
        """Free GiB on the filesystem holding `directory` (nearest parent that exists)."""
        usage = self._usage(directory)
        return None if usage is None else usage[2] / GIB

    def disk_total_gb(self, directory: Path) -> float | None:
        usage = self._usage(directory)
        return None if usage is None else usage[0] / GIB

    # -- sensors -------------------------------------------------------------------------
    def uptime_s(self) -> float | None:
        text = self.read("/proc/uptime")
        try:
            return float(text.split()[0]) if text else None
        except (ValueError, IndexError):
            return None

    def thermal_zones(self) -> dict[str, float]:
        """Zone type -> degrees C, from /sys/class/thermal (Jetson: cpu-thermal, gpu-thermal)."""
        out: dict[str, float] = {}
        base = self.path("/sys/class/thermal")
        if not base.is_dir():
            return out
        for zone in sorted(base.glob("thermal_zone*")):
            try:
                kind = (zone / "type").read_text().strip()
                milli = int((zone / "temp").read_text().strip())
            except (OSError, ValueError, TypeError):  # Jetson cv*-thermal: EAGAIN
                continue
            out[kind] = milli / 1000.0
        return out

    def wifi_signal_dbm(self) -> float | None:
        """Signal level (dBm) of the first wireless interface in /proc/net/wireless, if any."""
        text = self.read("/proc/net/wireless")
        if not text:
            return None
        for line in text.splitlines()[2:]:
            parts = line.replace(":", " ").split()
            if len(parts) >= 4:
                try:
                    level = float(parts[3].rstrip("."))
                except ValueError:
                    continue
                # Drivers report either dBm (negative, sometimes shown as 256 - x) or a
                # 0..70 quality-style number. Convert the first to dBm, the second to a
                # rough dBm (quality 70 is about -40 dBm, 0 is about -110).
                if level > 100:
                    level -= 256
                elif level > 0:
                    level -= 110
                return level
        return None

    def video_devices(self) -> list[VideoDevice]:
        base = self.path("/sys/class/video4linux")
        out: list[VideoDevice] = []
        if not base.is_dir():
            return out
        for entry in base.glob("video*"):
            suffix = entry.name[len("video") :]
            if not suffix.isdigit():
                continue
            try:
                name = (entry / "name").read_text().strip()
            except OSError:
                continue
            try:
                index = int((entry / "index").read_text().strip())
            except (OSError, ValueError, TypeError):
                index = 0
            out.append(VideoDevice(f"/dev/video{suffix}", name, index))
        return sorted(out, key=lambda d: int(d.node[len("/dev/video") :]))

    def usb_camera_node(self, name_fragment: str = "Global Shutter") -> str | None:
        """The capture node of the USB camera whose v4l2 name contains `name_fragment`."""
        matches = [d for d in self.video_devices() if name_fragment in d.name]
        capture = [d for d in matches if d.index == 0]
        pick = capture or matches
        return pick[0].node if pick else None
