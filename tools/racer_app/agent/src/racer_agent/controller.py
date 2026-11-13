"""The actions: what the app can make the car's computer do.

Everything here is a method with a fixed shape; the HTTP layer maps one route to one method.
Nothing takes a command string. Nothing here publishes to a ROS topic: starting a driving
launch only starts a process that publishes to /drive_raw, and /drive_raw only reaches the
wheels through safety_node (CLAUDE.md invariant 1). The hardware RC mux and the kill knob are
not touched and cannot be from here.

Safety rules enforced in `_gate_drive`:
  * the stack must be running and its bag recorder must be running (invariant 5), and the disk
    must not be so full the recorder will die: these can NOT be overridden;
  * every other required pre-drive check must be green, or the caller passes `override` with a
    reason, and the override is written to the action log.
"""

from __future__ import annotations

import logging
import shutil
import threading
import time
from collections import deque
from collections.abc import Callable
from dataclasses import dataclass
from pathlib import Path
from typing import Any

import yaml

from . import commands as cmd
from .actionlog import ActionLog
from .checks import Report, Snapshot, evaluate
from .config import Settings
from .hostinfo import HostInfo
from .rosstate import RosState
from .runner import CmdResult, Runner

log = logging.getLogger(__name__)

SNAPSHOT_TTL_S = 2.0  # a status poll costs about five docker CLI calls; share them
HEARTBEAT_UNIT = "racer-heartbeat.service"
GROWTH_WINDOW_S = 30.0  # the bag growth rate is averaged over this long
GROWTH_MIN_SPAN_S = 2.0  # and is not reported until the samples are this far apart


class ActionError(Exception):
    """A refused or failed action. `status` is the HTTP status the API answers with."""

    def __init__(self, message: str, status: int = 409, data: dict[str, Any] | None = None):
        super().__init__(message)
        self.message = message
        self.status = status
        self.data = data or {}


TERMINAL_SUFFIX = " (started from a terminal)"


@dataclass(frozen=True)
class DriveState:
    """`label`: the drive the app started (None if none). `external`: other driving launches."""

    label: str | None
    external: list[cmd.DriveProcess]

    @property
    def mode(self) -> str | None:
        if self.label:
            return self.label
        if self.external:
            return self.external[0].mode + TERMINAL_SUFFIX
        return None


@dataclass(frozen=True)
class Raceline:
    id: str
    path: Path
    source: str  # "repo" or "data"


def _describe(mode: str, params: dict[str, Any]) -> str:
    """The choices a drive was started with, for the action log line (the UI shows `detail`)."""
    if mode == "pnp":
        return (
            f" dynamics={params.get('dynamics') or 'NONE'}"
            f" perception={params.get('perception') or 'none'}"
            f" planner_params={params.get('params_file') or 'default'}"
            f" v_max={params.get('v_max_mps')}"
        )
    if mode == "gap_follow":
        cap = params.get("max_speed_mps")
        floor = params.get("min_speed_mps")
        return (
            f" max_speed={'profile default' if cap is None else f'{cap:.2f} m/s'}"
            f" min_speed={'profile default' if floor is None else f'{floor:.2f} m/s'}"
        )
    return ""


class Controller:
    def __init__(
        self,
        settings: Settings,
        runner: Runner,
        host: HostInfo,
        ros: RosState,
        actions: ActionLog,
        clock: Callable[[], float] = time.time,
        sleep: Callable[[float], None] = time.sleep,
        defer: Callable[[float, Callable[[], None]], None] | None = None,
    ) -> None:
        self.s = settings
        self.runner = runner
        self.host = host
        self.ros = ros
        self.actions = actions
        self._clock = clock
        self._sleep = sleep
        self._defer = defer or self._timer_defer
        self._lock = threading.RLock()  # stack / camera operations
        self._drive_lock = threading.RLock()  # driving start / stop
        self._gen_lock = threading.Lock()
        self._stop_generation = 0
        self._snap_lock = threading.Lock()
        self._snap: tuple[float, Snapshot] | None = None
        self._growth_lock = threading.Lock()
        self._growth_bag: str | None = None
        self._growth_samples: deque[tuple[float, int]] = deque()
        self.last_stopped_external = 0  # terminal-started launches the last _kill_drive saw

    @staticmethod
    def _timer_defer(delay: float, fn: Callable[[], None]) -> None:
        timer = threading.Timer(delay, fn)
        timer.daemon = True
        timer.start()

    # ------------------------------------------------------------------------------------
    # reading state
    # ------------------------------------------------------------------------------------

    def container_states(self) -> dict[str, str]:
        res = self.runner.run(
            ["docker", "ps", "-a", "--format", "{{.Names}}\t{{.State}}"], timeout=10
        )
        states: dict[str, str] = {}
        if not res.ok:
            return states
        ours = {self.s.stack_container, self.s.camera_container, self.s.perception_container}
        for line in res.stdout.splitlines():
            name, _, state = line.partition("\t")
            if name in ours:
                states[name] = state.strip()
        return states

    def _stack_flags(self) -> dict[str, str]:
        res = self.runner.run(
            ["docker", "inspect", "-f", "{{json .Config.Cmd}}", self.s.stack_container], timeout=10
        )
        if not res.ok:
            return {}
        return cmd.parse_launch_flags(res.stdout)

    def _recorder_running(self) -> bool:
        res = self.runner.run(cmd.recorder_check_argv(self.s.stack_container), timeout=10)
        return res.ok and cmd.recorder_running(res.stdout)

    def drive_state(self) -> DriveState:
        """Every driving launch in the container. Asks the container, not our own memory.

        The one the app started is named by its pid file; any other (`docker exec ... ros2
        launch` from a terminal) is found by scanning the container's processes for a driving
        launch, so it shows in the app and STOP ALL can stop it too.
        """
        res = self.runner.run(
            cmd.exec_bash(self.s.stack_container, cmd.DRIVE_STATUS_SCRIPT), timeout=10
        )
        label, app_pid = None, None
        lines = res.stdout.strip().splitlines() if res.ok else []
        if len(lines) >= 2 and lines[0].isdigit():
            app_pid, label = int(lines[0]), lines[1].strip() or None
        scan = self.runner.run(
            cmd.exec_bash(self.s.stack_container, cmd.DRIVE_SCAN_SCRIPT), timeout=10
        )
        found = cmd.parse_drive_processes(scan.stdout) if scan.ok else []
        return DriveState(label, [p for p in found if p.pid != app_pid])

    def drive_mode(self) -> str | None:
        """The running driving mode (the app's, else one started from a terminal), or None."""
        return self.drive_state().mode

    def _heartbeat_active(self) -> bool | None:
        res = self.runner.run(["systemctl", "is-active", HEARTBEAT_UNIT], timeout=5)
        if res.returncode == 127:
            return None
        return res.stdout.strip() == "active"

    def snapshot(self, fresh: bool = False) -> Snapshot:
        with self._snap_lock:
            now = self._clock()
            if not fresh and self._snap and now - self._snap[0] < SNAPSHOT_TTL_S:
                return self._snap[1]
            containers = self.container_states()
            stack_running = containers.get(self.s.stack_container) == "running"
            snap = Snapshot(
                stack_running=stack_running,
                recorder_running=stack_running and self._recorder_running(),
                containers=containers,
                stack_flags=self._stack_flags() if stack_running else {},
                disk_free_gb=self.host.disk_free_gb(self.s.bags),
                lidar_device=self.host.exists(cmd.LIDAR_DEVICE),
                vesc_device=self.host.exists(cmd.VESC_DEVICE),
                heartbeat_active=self._heartbeat_active(),
                zones_c=self.host.thermal_zones(),
                wifi_dbm=self.host.wifi_signal_dbm(),
                camera_devices=[d.node for d in self.host.video_devices()],
                uptime_s=self.host.uptime_s(),
                ros=self.ros.snapshot(),
            )
            self._snap = (now, snap)
            return snap

    def report(self, fresh: bool = False) -> Report:
        return evaluate(self.snapshot(fresh), self.s.thresholds)

    def status(self) -> dict[str, Any]:
        snap = self.snapshot()
        report = evaluate(snap, self.s.thresholds)
        drive = self.drive_state() if snap.stack_running else DriveState(None, [])
        mode = drive.mode
        return {
            "time": self._clock(),
            "checks": report.as_dict(),
            "stack": {
                "running": snap.stack_running,
                "recorder": snap.recorder_running,
                "flags": snap.stack_flags,
            },
            "cameras": {"running": snap.containers.get(self.s.camera_container) == "running"},
            "drive": {
                "mode": mode,
                "running": mode is not None,
                # driving launches the app did not start (docker exec ... ros2 launch)
                "external": [
                    {"pid": p.pid, "mode": p.mode, "command": p.command} for p in drive.external
                ],
            },
            "containers": snap.containers,
            "disk_free_gb": snap.disk_free_gb,
            "uptime_s": snap.uptime_s,
            "ros_connected": bool(snap.ros and snap.ros.connected),
            "autostart": self.autostart_enabled(),
            "policy": self.policy_status(),
            "perception": self.perception_status(snap),
            "spiderbot": self.spiderbot_status(),
            "thresholds": {
                "disk_red_gb": self.s.thresholds.disk_red_gb,
                "cameras_min_free_gb": self.s.thresholds.cameras_min_free_gb,
                "battery_cells": self.s.thresholds.battery_cells,
            },
        }

    # ------------------------------------------------------------------------------------
    # files the agent offers as choices
    # ------------------------------------------------------------------------------------

    def maps(self) -> list[str]:
        base = self.s.maps
        return sorted(p.name for p in base.glob("*.yaml")) if base.is_dir() else []

    def racelines(self) -> list[Raceline]:
        found: list[Raceline] = []
        tracks = self.s.repo_dir / "config" / "tracks"
        if tracks.is_dir():
            for p in sorted(tracks.glob("*/raceline.csv")):
                found.append(Raceline(f"track:{p.parent.name}", p, "repo"))
        user = self.s.racelines_user_dir
        if user.is_dir():
            for p in sorted(user.glob("*.csv")):
                found.append(Raceline(f"data:{p.stem}", p, "data"))
        return found

    def patrols(self) -> list[str]:
        """Patrol files the agent found itself: data/patrols/NAME.yaml, plain files, valid names."""
        base = self.s.patrols_dir
        if not base.is_dir():
            return []
        return sorted(
            p.stem
            for p in base.glob("*.yaml")
            if p.is_file() and not p.is_symlink() and cmd.NAME_RE.match(p.stem)
        )

    def options(self) -> dict[str, Any]:
        return {
            "gap_follow_profiles": list(cmd.GAP_FOLLOW_PROFILES),
            "park_profiles": list(cmd.PARK_PROFILES),
            "maps": self.maps(),
            "racelines": [r.id for r in self.racelines()],
            "drive_modes": list(cmd.DRIVE_MODES),
            "gap_follow_speed": {
                p: dict(zip(("min", "max"), self.gap_follow_speed_limits(p)))
                for p in cmd.GAP_FOLLOW_PROFILES
            },
            "policy": self.policy_status(),
            "pnp": self.pnp_status(),
            "patrols": self.patrols(),
            "spiderbot_rules": list(cmd.SPIDERBOT_RULES),
            "spiderbot_speed_range": list(cmd.SPIDERBOT_SPEED_RANGE),
            "spiderbot_default_speed": cmd.SPIDERBOT_DEFAULT_SPEED_MPS,
            "perception_modes": list(cmd.PERCEPTION_MODES),
            "perception_devices": list(cmd.PERCEPTION_DEVICES),
            "perception": self.perception_status(),
            "spiderbot": self.spiderbot_status(),
        }

    def vehicle(self) -> dict[str, Any]:
        """The few vehicle_params the UI draws with, read from the single source of truth.

        CLAUDE.md invariant 2: the UI hand-writes no mount offset or yaw. It asks here, and
        this reads config/vehicle_params.yaml. A missing file or a null field comes back as
        null and the UI says so instead of guessing.
        """
        path = self.s.repo_dir / "config" / "vehicle_params.yaml"
        try:
            raw = yaml.safe_load(path.read_text(encoding="utf-8")) or {}
        except (OSError, yaml.YAMLError):
            raw = {}

        def pick(section: str, *keys: str) -> dict[str, Any]:
            block = raw.get(section) or {}
            return {k: block.get(k) for k in keys}

        lidar = (raw.get("sensors") or {}).get("lidar") or {}
        return {
            "schema_version": (raw.get("meta") or {}).get("schema_version"),
            "lidar": {k: lidar.get(k) for k in ("mount_x_m", "mount_y_m", "mount_yaw_rad")},
            "chassis": pick(
                "chassis",
                "wheelbase_m",
                "length_m",
                "width_m",
                "front_overhang_m",
                "rear_overhang_m",
            ),
            "steering": pick("steering", "max_angle_rad"),
            "actuation": pick("actuation", "throttle_full_scale_mps"),
        }

    # ------------------------------------------------------------------------------------
    # logging helper
    # ------------------------------------------------------------------------------------

    def _log(self, action: str, ok: bool, actor: str, detail: str = "", **kw: Any) -> None:
        self.actions.add(action, ok, actor=actor, detail=detail, **kw)

    # ------------------------------------------------------------------------------------
    # stack
    # ------------------------------------------------------------------------------------

    def _resolve_map(self, map_name: str) -> str:
        if map_name not in self.maps():
            raise ActionError(f"unknown map {map_name!r}", 400)
        return cmd.container_path(self.s.repo_dir, self.s.maps / map_name)

    def _check_stack_start(self, flags: cmd.StackFlags) -> None:
        try:
            flags.validate()
        except cmd.CommandError as exc:
            raise ActionError(str(exc), 400) from exc
        free = self.host.disk_free_gb(self.s.bags)
        if free is None or free < self.s.thresholds.disk_red_gb:
            shown = "unreadable" if free is None else f"{free:.1f} GiB"
            raise ActionError(
                f"disk has {shown} free, under {self.s.thresholds.disk_red_gb:g} GiB: "
                "the recorder would die. Delete old bags first.",
                409,
            )
        if not (self.s.repo_dir / "ros_ws" / "install" / "setup.bash").exists():
            raise ActionError("ros_ws is not built (no install/setup.bash)", 409)
        if self.runner.run(["docker", "image", "inspect", self.s.image], timeout=15).returncode:
            raise ActionError(f"docker image {self.s.image} is missing", 409)
        if flags.lidar and not self.host.exists(cmd.LIDAR_DEVICE):
            raise ActionError(
                f"{cmd.LIDAR_DEVICE} is not there: plug in the LiDAR or untick it", 409
            )
        if flags.vesc and not self.host.exists(cmd.VESC_DEVICE):
            raise ActionError(f"{cmd.VESC_DEVICE} is not there: untick the VESC", 409)

    def start_stack(self, flags: cmd.StackFlags, actor: str = "") -> dict[str, Any]:
        params = {
            "lidar": flags.lidar,
            "vesc": flags.vesc,
            "mapping": flags.mapping,
            "localization": flags.localization,
            "map": flags.map,
        }
        with self._lock:
            try:
                self._check_stack_start(flags)
                states = self.container_states()
                name = self.s.stack_container
                if states.get(name) == "running":
                    raise ActionError("the stack is already running (use restart)", 409)
                container_map = self._resolve_map(flags.map) if flags.localization else ""
                if name in states:
                    self.runner.run(["docker", "rm", name], timeout=20)
                self.s.bags.mkdir(parents=True, exist_ok=True)
                argv = cmd.stack_run_argv(
                    self.host, self.s.repo_dir, self.s.image, name, flags, container_map
                )
                res = self.runner.run(argv, timeout=120)
                if not res.ok:
                    raise ActionError(f"docker run failed: {res.tail()}", 500)
                self._sleep(2.0)
                if self.container_states().get(name) != "running":
                    logs = self.runner.run(["docker", "logs", "--tail", "20", name], timeout=10)
                    raise ActionError(
                        f"the stack exited right after starting: {(logs.stderr or logs.stdout)[-400:]}",
                        500,
                    )
            except ActionError as exc:
                self._log("stack.start", False, actor, exc.message, params=params)
                raise
            except cmd.CommandError as exc:
                self._log("stack.start", False, actor, str(exc), params=params)
                raise ActionError(str(exc), 400) from exc
            self._invalidate()
            self._log("stack.start", True, actor, "started", params=params)
            return {"message": "stack starting", "flags": params}

    def _wait_stopped(self, container: str, seconds: float) -> bool:
        poll = 0.5 if seconds <= 60.0 else 1.0  # a 10 minute wait is not 1200 docker calls
        waited = 0.0
        while waited < seconds:
            res = self.runner.run(
                ["docker", "inspect", "-f", "{{.State.Running}}", container], timeout=10
            )
            if not res.ok or res.stdout.strip() != "true":
                return True
            self._sleep(poll)
            waited += poll
        return False

    def _exit_code(self, container: str) -> str:
        res = self.runner.run(
            ["docker", "inspect", "-f", "{{.State.ExitCode}}", container], timeout=10
        )
        return res.stdout.strip() if res.ok else "?"

    def _stop_container(self, container: str, timeout_s: float) -> tuple[str, str]:
        """SIGINT, wait up to `timeout_s`, then SIGTERM, a short wait, then SIGKILL, then rm.

        Not `docker stop` (SIGTERM first: the runbook "Stopping" table shows it leaves the PWM
        channels enabled at neutral, and it cuts the bag recorder off before it writes the mcap
        footer and metadata.yaml). SIGINT is what Ctrl-C sends and what ros2 launch shuts down
        cleanly on; the wait is long because closing a multi-GB bag takes minutes. Returns
        (short outcome for the UI, detail for the action log).
        """
        states = self.container_states()
        if container not in states:
            return "not present", "not present"
        how, detail = "stopped", "was not running"
        if states[container] == "running":
            start = self._clock()
            killed = self.runner.run(["docker", "kill", "-s", "INT", container], timeout=15)
            if not killed.ok:
                log.warning("docker kill -s INT %s failed: %s", container, killed.tail())
            if self._wait_stopped(container, timeout_s):
                took = self._clock() - start
                how = "stopped"
                detail = (
                    f"SIGINT, clean exit after {took:.0f} s, exit code {self._exit_code(container)}"
                )
            else:
                log.warning(
                    "%s still running %.0f s after SIGINT: sending SIGTERM", container, timeout_s
                )
                self.runner.run(["docker", "kill", "-s", "TERM", container], timeout=15)
                if self._wait_stopped(container, self.s.stop_term_grace_s):
                    how = "needed SIGTERM"
                    detail = (
                        f"no exit {timeout_s:.0f} s after SIGINT, exited after SIGTERM: "
                        "a bag it was writing may be unfinished"
                    )
                else:
                    log.warning("%s ignored SIGTERM: sending SIGKILL", container)
                    self.runner.run(["docker", "kill", container], timeout=15)
                    how = "had to be killed"
                    detail = (
                        f"ignored SIGINT for {timeout_s:.0f} s and SIGTERM for "
                        f"{self.s.stop_term_grace_s:.0f} s, SIGKILL: a bag it was writing is unfinished"
                    )
        self.runner.run(["docker", "rm", container], timeout=20)
        return how, detail

    def _recording_bag(self) -> str | None:
        """Name of the bag the recorder is writing now (the newest folder), or None."""
        if not self.snapshot(fresh=True).recorder_running or not self.s.bags.is_dir():
            return None
        dirs = [p for p in self.s.bags.iterdir() if p.is_dir() and not p.is_symlink()]
        return max(dirs, key=lambda p: p.stat().st_mtime).name if dirs else None

    def _bag_finalized(self, name: str | None) -> str:
        """After the stack stopped: did the recorder close that bag (metadata.yaml is written last)."""
        if name is None:
            return ""
        if (self.s.bags / name / "metadata.yaml").is_file():
            return f"; bag {name} finalized (metadata.yaml present)"
        return (
            f"; bag {name} has NO metadata.yaml: the recorder did not close it "
            "(try `ros2 bag reindex`)"
        )

    def stop_stack(self, actor: str = "") -> dict[str, Any]:
        with self._lock:
            bag = self._recording_bag()
            self._kill_drive()
            how, detail = self._stop_container(self.s.stack_container, self.s.stop_timeout_stack_s)
            self._invalidate()
            bag_note = self._bag_finalized(bag) if how != "not present" else ""
            ok = how in ("stopped", "not present") and "NO metadata" not in bag_note
            self._log("stack.stop", ok, actor, detail + bag_note)
            return {
                "message": f"stack {how}{bag_note}",
                "bag": bag,
                "bag_finalized": bool(bag) and "NO metadata" not in bag_note,
            }

    def restart_stack(self, flags: cmd.StackFlags | None, actor: str = "") -> dict[str, Any]:
        with self._lock:
            if flags is None:
                running = self._stack_flags()
                flags = cmd.StackFlags(
                    lidar=running.get("lidar", "true") != "false",
                    vesc=running.get("vesc", "true") != "false",
                )
            self.stop_stack(actor)
            return self.start_stack(flags, actor)

    # ------------------------------------------------------------------------------------
    # cameras
    # ------------------------------------------------------------------------------------

    def start_cameras(self, actor: str = "") -> dict[str, Any]:
        with self._lock:
            try:
                free = self.host.disk_free_gb(self.s.bags)
                need = self.s.thresholds.cameras_min_free_gb
                if free is None or free < need:
                    shown = "unreadable" if free is None else f"{free:.1f} GiB"
                    raise ActionError(
                        f"disk has {shown} free; cameras record about 6 MB/s each and need "
                        f"{need:g} GiB free to start",
                        409,
                    )
                usb = self.host.usb_camera_node()
                if usb is None:
                    raise ActionError("the USB camera ('Global Shutter') is not plugged in", 409)
                states = self.container_states()
                name = self.s.camera_container
                if states.get(name) == "running":
                    raise ActionError("the cameras are already running", 409)
                if name in states:
                    self.runner.run(["docker", "rm", name], timeout=20)
                argv = cmd.camera_run_argv(self.host, self.s.repo_dir, self.s.image, name, usb)
                res = self.runner.run(argv, timeout=120)
                if not res.ok:
                    raise ActionError(f"docker run failed: {res.tail()}", 500)
            except ActionError as exc:
                self._log("cameras.start", False, actor, exc.message)
                raise
            self._invalidate()
            note = (
                ""
                if self.container_states().get(self.s.stack_container) == "running"
                else (" (the stack is not running, so nothing is being recorded)")
            )
            self._log("cameras.start", True, actor, f"usb {usb}")
            return {"message": "cameras starting" + note, "usb_device": usb}

    def stop_cameras(self, actor: str = "") -> dict[str, Any]:
        with self._lock:
            how, detail = self._stop_container(self.s.camera_container, self.s.stop_timeout_other_s)
            self._invalidate()
            self._log("cameras.stop", how in ("stopped", "not present"), actor, detail)
            return {"message": f"cameras {how}"}

    # ------------------------------------------------------------------------------------
    # driving
    # ------------------------------------------------------------------------------------

    # ------------------------------------------------------------------------------------
    # perception and the spiderbot (roadmap 5.1, 5.3). Both need the GPU image.
    # ------------------------------------------------------------------------------------

    def _perception_image_present(self) -> bool:
        return self.runner.run(
            ["docker", "image", "inspect", self.s.perception_image], timeout=15
        ).ok

    def perception_status(self, snap: Snapshot | None = None) -> dict[str, Any]:
        """Can perception be started, and is it running. Disabled with a reason, never hidden."""
        if snap is not None:
            running = snap.containers.get(self.s.perception_container) == "running"
        else:
            running = self.container_states().get(self.s.perception_container) == "running"
        if not self._perception_image_present():
            return {
                "available": False,
                "running": running,
                "reason": f"the GPU image {self.s.perception_image} is not built on this car "
                "(docker build with INSTALL_TORCH=required and INSTALL_PERCEPTION=required, "
                "docker/car/README.md Perception)",
            }
        return {"available": True, "running": running, "reason": ""}

    def spiderbot_status(self) -> dict[str, Any]:
        """Can the spiderbot patrol be launched at all. Dynamic conditions (stack, pose,
        perception running) are checked, and refused with their own message, at start."""
        perception = self.perception_status()
        if not perception["available"]:
            return {"available": False, "reason": perception["reason"]}
        if not (self.s.repo_dir / "ros_ws" / "install" / "racer_apps").exists():
            return {
                "available": False,
                "reason": "racer_apps is not built in ros_ws (colcon build on the car)",
            }
        if not self.patrols():
            return {
                "available": False,
                "reason": "no patrol file under data/patrols (record a loop on the map first)",
            }
        return {"available": True, "reason": ""}

    def start_perception(
        self, mode: str = "pose", device: str = "cuda", actor: str = ""
    ) -> dict[str, Any]:
        params = {"mode": mode, "device": device}
        with self._lock:
            try:
                status = self.perception_status()
                if not status["available"]:
                    raise ActionError(status["reason"], 409)
                states = self.container_states()
                if states.get(self.s.camera_container) != "running":
                    raise ActionError(
                        "start the cameras first: perception reads the camera topic", 409
                    )
                name = self.s.perception_container
                if states.get(name) == "running":
                    raise ActionError("perception is already running", 409)
                if name in states:
                    self.runner.run(["docker", "rm", name], timeout=20)
                self.s.perception_cache_dir.mkdir(parents=True, exist_ok=True)
                argv = cmd.perception_run_argv(
                    self.host,
                    self.s.repo_dir,
                    self.s.perception_cache_dir,
                    self.s.perception_image,
                    name,
                    mode,
                    device,
                )
                res = self.runner.run(argv, timeout=120)
                if not res.ok:
                    raise ActionError(f"docker run failed: {res.tail()}", 500)
            except ActionError as exc:
                self._log("perception.start", False, actor, exc.message, params=params)
                raise
            except cmd.CommandError as exc:
                self._log("perception.start", False, actor, str(exc), params=params)
                raise ActionError(str(exc), 400) from exc
            self._invalidate()
            self._log("perception.start", True, actor, f"{mode} on {device}", params=params)
            return {"message": f"perception starting ({mode}, {device})", **params}

    def stop_perception(self, actor: str = "") -> dict[str, Any]:
        with self._lock:
            how, detail = self._stop_container(
                self.s.perception_container, self.s.stop_timeout_other_s
            )
            self._invalidate()
            self._log("perception.stop", how in ("stopped", "not present"), actor, detail)
            return {"message": f"perception {how}"}

    def _spiderbot_precondition(self, snap: Snapshot) -> None:
        if snap.containers.get(self.s.perception_container) != "running":
            raise ActionError("the spiderbot needs perception running: start it first", 409)
        if (snap.ros.rate("/perception/objects") if snap.ros else 0.0) <= 0.0:
            raise ActionError(
                "perception is running but /perception/objects is silent (is the camera "
                "publishing, and is perception in pose mode?)",
                409,
            )

    def policy_status(self) -> dict[str, Any]:
        """Is an ML policy launchable? Only if a policy dir passes racer_policy's own loader."""
        base = self.s.policies_dir
        dirs = (
            sorted(p for p in base.glob("*/") if (p / "contract.yaml").is_file())
            if base.is_dir()
            else []
        )
        if not dirs:
            return {
                "available": False,
                "reason": "no policy directory with a contract.yaml under data/policies "
                "(a learned policy does not exist yet)",
                "valid": [],
            }
        valid: list[str] = []
        errors: list[str] = []
        for d in dirs:
            res = self._run_loader(d)
            if res.ok and "OK" in res.stdout:
                valid.append(d.name)
            else:
                errors.append(f"{d.name}: {res.tail(160) or 'refused'}")
        if not valid:
            return {
                "available": False,
                "reason": "the deploy checks refused every policy directory: " + "; ".join(errors),
                "valid": [],
            }
        launch = (
            self.s.repo_dir / "ros_ws" / "src" / "racer_bringup" / "launch" / "policy.launch.py"
        )
        if not launch.exists():
            return {
                "available": False,
                "reason": f"policy {valid[0]} passes the deploy checks, but racer_bringup has no "
                "policy.launch.py",
                "valid": valid,
            }
        return {"available": True, "reason": "", "valid": valid}

    def pnp_status(self) -> dict[str, Any]:
        """Can the Plug-and-Play driver be launched, and with which bundles. A bundle is listed
        when data/pnp/models/<id>/manifest.yaml parses and names a kind; the nodes themselves
        refuse any bundle that does not match this car (checksums, vehicle_params, io)."""
        found: dict[str, list[str]] = {"track_perception": [], "dynamics": []}
        base = self.s.pnp_models_dir
        if base.is_dir():
            for d in sorted(base.iterdir()):
                manifest = d / "manifest.yaml"
                if d.is_symlink() or not d.is_dir() or not cmd.NAME_RE.match(d.name):
                    continue
                if not manifest.is_file():
                    continue
                try:
                    kind = yaml.safe_load(manifest.read_text(encoding="utf-8")).get("kind")
                except (OSError, ValueError, AttributeError, yaml.YAMLError):
                    continue
                if kind in found:
                    found[kind].append(d.name)
        status: dict[str, Any] = {
            "perception": found["track_perception"],
            "dynamics": found["dynamics"],
            "backends": list(cmd.PNP_BACKENDS),
            "speed_range": list(cmd.PNP_SPEED_RANGE),
            "default_speed": cmd.PNP_DEFAULT_SPEED_MPS,
            "params_files": self.pnp_params_files(),
        }
        launch = self.s.repo_dir / "ros_ws" / "src" / "racer_bringup" / "launch" / "pnp.launch.py"
        if not found["dynamics"]:
            reason = "no dynamics bundle (manifest.yaml, kind dynamics) under data/pnp/models"
        elif not launch.exists():
            reason = "racer_bringup has no pnp.launch.py"
        else:
            reason = ""
        return {"available": not reason, "reason": reason, **status}

    def pnp_params_files(self) -> list[str]:
        """Planner parameter files the PnP card offers: data/pnp/*.yaml, plain files with a
        shell-safe name, found by the agent (a name from the request must be one of these)."""
        base = self.s.pnp_dir
        if not base.is_dir():
            return []
        return sorted(
            p.name
            for p in base.glob("*.yaml")
            if p.is_file() and not p.is_symlink() and cmd.NAME_RE.match(p.name)
        )

    def _run_loader(self, directory: Path) -> CmdResult:
        """Run `python3 -m racer_policy.check`, the same load_deployment the node runs at
        startup (contract checksum and version, live vehicle_params version, observation
        schema, sidecar settings), so the button is only offered for a directory the node
        will not refuse. It prints OK and exits 0, or the specific refusal and exits 1."""
        host_dir = cmd.container_path(self.s.repo_dir, directory)
        return self.runner.run(
            [
                "docker",
                "run",
                "--rm",
                "--user",
                f"{self.host.uid}:{self.host.gid}",
                "-e",
                (
                    "PYTHONPATH=/workspace/ros_ws/src/racer_policy/src:"
                    "/workspace/ros_ws/src/racer_tools:/workspace/training/envelope/src"
                ),
                "-v",
                f"{self.s.repo_dir}:/workspace",
                "-w",
                "/workspace",
                "--entrypoint",
                "python3",
                self.s.image,
                "-m",
                "racer_policy.check",
                host_dir,
            ],
            timeout=60,
        )

    def _gate_drive(self, override: bool, reason: str) -> bool:
        """Apply the pre-drive rules. Returns True when an override was used."""
        report = self.report(fresh=True)
        hard = report.hard_failures
        if hard:
            msg = "refused: " + "; ".join(f"{c.label}: {c.detail}" for c in hard)
            raise ActionError(msg, 409, {"failing": [c.id for c in hard], "overridable": False})
        bad = report.not_green_required
        if not bad:
            return False
        names = "; ".join(f"{c.label} ({c.level}): {c.detail}" for c in bad)
        if not override:
            raise ActionError(
                f"pre-drive checks are not all green: {names}",
                409,
                {"failing": [c.id for c in bad], "overridable": True},
            )
        if len(reason.strip()) < 3:
            raise ActionError("an override needs a reason", 400)
        return True

    def _tracker_precondition(self, snap: Snapshot) -> None:
        pose_hz = snap.ros.rate("/pose") if snap.ros else 0.0
        if snap.stack_flags.get("localization") != "true" or pose_hz <= 0.0:
            raise ActionError(
                "the tracker needs the stack running with localisation and a live /pose", 409
            )

    def start_drive(
        self,
        mode: str,
        profile: str = "",
        raceline: str = "",
        override: bool = False,
        override_reason: str = "",
        actor: str = "",
        patrol: str = "",
        rule: str = "arms_raised",
        speed_mps: float = cmd.SPIDERBOT_DEFAULT_SPEED_MPS,
        policy: str = "",
        pnp_perception: str = "",
        pnp_dynamics: str = "",
        pnp_backend: str = "net",
        pnp_v_max_mps: float = cmd.PNP_DEFAULT_SPEED_MPS,
        pnp_params: str = "",
        max_speed_mps: float | None = None,
        min_speed_mps: float | None = None,
    ) -> dict[str, Any]:
        action = f"drive.start.{mode}"
        params: dict[str, Any] = {"mode": mode, "profile": profile, "raceline": raceline}
        if mode == "gap_follow":
            params["max_speed_mps"] = max_speed_mps
            params["min_speed_mps"] = min_speed_mps
        if mode == "spiderbot":
            params.update({"patrol": patrol, "rule": rule, "speed_mps": speed_mps})
        if mode == "ml_policy":
            params["policy"] = policy
        if mode == "pnp":
            params.update(
                {
                    "perception": pnp_perception,
                    "dynamics": pnp_dynamics,
                    "backend": pnp_backend,
                    "v_max_mps": pnp_v_max_mps,
                    "params_file": pnp_params,
                }
            )
        if mode not in cmd.DRIVE_MODES:
            raise ActionError(f"unknown driving mode {mode!r}", 400)
        with self._drive_lock:
            with self._gen_lock:
                generation = self._stop_generation
            used_override = False
            try:
                ros_cmd, label, resolved = self._prepare_drive(
                    mode,
                    profile,
                    raceline,
                    patrol,
                    rule,
                    speed_mps,
                    policy,
                    (pnp_perception, pnp_dynamics, pnp_backend, pnp_v_max_mps, pnp_params),
                    max_speed_mps,
                    min_speed_mps,
                )
                params.update(resolved)
                used_override = self._gate_drive(override, override_reason)
                already = self.drive_mode()
                if already is not None:
                    raise ActionError(
                        f"a driving launch is already running ({already}): stop it first "
                        "(STOP ALL stops launches started from a terminal too)",
                        409,
                    )
                if mode in ("tracker", "ml_policy", "spiderbot"):
                    self._tracker_precondition(self.snapshot(fresh=True))
                if mode == "spiderbot":
                    self._spiderbot_precondition(self.snapshot(fresh=True))
                self.runner.run(
                    [
                        "docker",
                        "exec",
                        "-d",
                        self.s.stack_container,
                        "bash",
                        "-c",
                        cmd.drive_launch_script(label, ros_cmd),
                    ],
                    timeout=20,
                )
                self._sleep(1.5)
                running = self.drive_mode()
                if running is None:
                    tail = self.runner.run(cmd.tail_argv(self.s.stack_container, 8), timeout=10)
                    raise ActionError(
                        f"the launch exited right away: {tail.stdout.strip()[-400:]}", 500
                    )
            except ActionError as exc:
                self._log(action, False, actor, exc.message, params=params)
                raise
            except cmd.CommandError as exc:
                self._log(action, False, actor, str(exc), params=params)
                raise ActionError(str(exc), 400) from exc
            with self._gen_lock:
                interrupted = generation != self._stop_generation
            if interrupted:
                self._kill_drive()
                self._log(action, False, actor, "STOP ALL arrived while starting", params=params)
                raise ActionError("STOP ALL arrived while the launch was starting: stopped it", 409)
            self._log(
                action,
                True,
                actor,
                f"running as {running}" + _describe(mode, params),
                params=params,
                override=used_override,
                override_reason=override_reason if used_override else "",
            )
            return {"message": f"{running} started", "mode": running, "override": used_override}

    def _prepare_drive(
        self,
        mode: str,
        profile: str,
        raceline: str,
        patrol: str = "",
        rule: str = "arms_raised",
        speed_mps: float = cmd.SPIDERBOT_DEFAULT_SPEED_MPS,
        policy: str = "",
        pnp: tuple[str, str, str, float, str] = ("", "", "net", cmd.PNP_DEFAULT_SPEED_MPS, ""),
        max_speed_mps: float | None = None,
        min_speed_mps: float | None = None,
    ) -> tuple[str, str, dict[str, Any]]:
        """Validate a driving request against the whitelist.

        Returns (ros2 command, mode label, the choices as resolved here, for the log).
        """
        if mode == "pnp":
            return self._prepare_pnp(*pnp)
        if mode == "spiderbot":
            status = self.spiderbot_status()
            if not status["available"]:
                raise ActionError("the spiderbot is disabled: " + status["reason"], 409)
            if patrol not in self.patrols():
                raise ActionError(f"unknown patrol {patrol!r}", 400)
            path = self.s.patrols_dir / f"{patrol}.yaml"
            container_path = cmd.container_path(self.s.repo_dir, path)
            return (
                cmd.spiderbot_cmd(container_path, rule, speed_mps),
                f"spiderbot:{patrol}",
                {},
            )
        if mode == "ml_policy":
            status = self.policy_status()
            if not status["available"]:
                raise ActionError("ML policy is disabled: " + status["reason"], 409)
            valid: list[str] = status["valid"]
            if not policy:
                if len(valid) != 1:
                    raise ActionError(
                        f"choose a policy: {', '.join(valid)} all pass the deploy checks", 400
                    )
                policy = valid[0]
            if policy not in valid:
                raise ActionError(
                    f"policy {policy!r} is not one the deploy checks accept "
                    f"(accepted: {', '.join(valid)})",
                    400,
                )
            match = [r for r in self.racelines() if r.id == raceline]
            if not match:
                raise ActionError(f"unknown raceline {raceline!r}", 400)
            return (
                cmd.ros_cmd_for(
                    "ml_policy",
                    raceline_container_path=cmd.container_path(self.s.repo_dir, match[0].path),
                    policy_container_path=cmd.container_path(
                        self.s.repo_dir, self.s.policies_dir / policy
                    ),
                ),
                f"ml_policy:{policy}",
                {"policy": policy},
            )
        if mode == "tracker":
            match = [r for r in self.racelines() if r.id == raceline]
            if not match:
                raise ActionError(f"unknown raceline {raceline!r}", 400)
            path = cmd.container_path(self.s.repo_dir, match[0].path)
            return (
                cmd.ros_cmd_for("tracker", raceline_container_path=path),
                f"tracker:{raceline}",
                {},
            )
        if mode in ("gap_follow", "park") and not profile:
            profile = "lane-2026-10-25" if mode == "gap_follow" else "floor-2026-10-07"
        label = f"{mode}:{profile}" if profile else mode
        resolved: dict[str, Any] = {"profile": profile}
        if mode == "gap_follow" and max_speed_mps is not None:
            low, high = self.gap_follow_speed_limits(profile)
            if (
                isinstance(max_speed_mps, bool)
                or not isinstance(max_speed_mps, (int, float))
                or not (low <= float(max_speed_mps) <= high)
            ):
                raise ActionError(
                    f"gap follow max speed for profile {profile} must be between {low:g} and "
                    f"{high:g} m/s (the profile's ceiling, and vehicle_params' speed cap)",
                    400,
                )
        elif mode != "gap_follow" and max_speed_mps is not None:
            raise ActionError("a max speed is only used with gap follow", 400)
        if mode == "gap_follow" and min_speed_mps is not None:
            # The node refuses min > max, so the floor is checked against the max speed that
            # will actually be in effect: the caller's cap, else the profile's ceiling (clipped
            # by vehicle_params' speed cap).
            if max_speed_mps is not None:
                effective, source = float(max_speed_mps), "the max speed set here"
            else:
                effective = self.gap_follow_speed_limits(profile)[1]
                source = f"profile {profile}'s max speed"
            if (
                isinstance(min_speed_mps, bool)
                or not isinstance(min_speed_mps, (int, float))
                or not (0.0 <= float(min_speed_mps) <= effective)
            ):
                raise ActionError(
                    f"gap follow min speed must be between 0 and {effective:g} m/s "
                    f"(it cannot exceed the max speed in effect, {source})",
                    400,
                )
        elif mode != "gap_follow" and min_speed_mps is not None:
            raise ActionError("a min speed is only used with gap follow", 400)
        gap = mode == "gap_follow"
        return (
            cmd.ros_cmd_for(
                mode,
                profile=profile,
                max_speed_mps=max_speed_mps if gap else None,
                min_speed_mps=min_speed_mps if gap else None,
            ),
            label,
            resolved,
        )

    def gap_follow_speed_limits(self, profile: str) -> tuple[float, float]:
        """(lowest, highest) max speed in m/s the app will pass to gap follow for `profile`.

        The highest is the smaller of the profile's own ceiling and vehicle_params'
        limits.global_speed_cap_mps, read from the single source of truth (null or unreadable
        leaves just the profile's ceiling).
        """
        ceiling = cmd.GAP_FOLLOW_PROFILE_MAX_SPEED_MPS.get(
            profile, cmd.GAP_FOLLOW_PROFILE_MAX_SPEED_MPS["none"]
        )
        cap = self._vehicle_speed_cap()
        high = min(ceiling, cap) if cap is not None else ceiling
        return min(cmd.GAP_FOLLOW_MIN_SPEED_MPS, high), high

    def _vehicle_speed_cap(self) -> float | None:
        path = self.s.repo_dir / "config" / "vehicle_params.yaml"
        try:
            raw = yaml.safe_load(path.read_text(encoding="utf-8")) or {}
            cap = (raw.get("limits") or {}).get("global_speed_cap_mps")
        except (OSError, yaml.YAMLError, AttributeError):
            return None
        return float(cap) if isinstance(cap, (int, float)) and not isinstance(cap, bool) else None

    def _prepare_pnp(
        self, perception: str, dynamics: str, backend: str, v_max_mps: float, params_file: str = ""
    ) -> tuple[str, str, dict[str, Any]]:
        status = self.pnp_status()
        if not status["available"]:
            raise ActionError("Plug-and-Play is disabled: " + status["reason"], 409)
        if backend not in cmd.PNP_BACKENDS:
            raise ActionError(f"backend must be one of {list(cmd.PNP_BACKENDS)}", 400)
        # The bundle is always named in the request, never guessed from "there is only one": an
        # empty name once reached the log (and the owner could not tell which bundle drove).
        if not dynamics:
            raise ActionError(
                "no dynamics bundle in the request: choose one "
                f"(available: {', '.join(status['dynamics']) or 'none'})",
                400,
            )
        if dynamics not in status["dynamics"]:
            raise ActionError(
                f"unknown dynamics bundle {dynamics!r} "
                f"(available: {', '.join(status['dynamics']) or 'none'})",
                400,
            )
        if params_file and params_file not in status["params_files"]:
            raise ActionError(
                f"unknown planner params file {params_file!r} "
                f"(available: {', '.join(status['params_files']) or 'none'})",
                400,
            )
        if backend == "net":
            if perception not in status["perception"]:
                raise ActionError(
                    "backend net needs a perception bundle "
                    f"(available: {', '.join(status['perception']) or 'none'})",
                    400,
                )
        else:
            perception = ""
        models = self.s.pnp_models_dir
        return (
            cmd.ros_cmd_for(
                "pnp",
                pnp_perception_container_path=(
                    cmd.container_path(self.s.repo_dir, models / perception) if perception else ""
                ),
                pnp_dynamics_container_path=cmd.container_path(self.s.repo_dir, models / dynamics),
                pnp_backend=backend,
                pnp_v_max_mps=v_max_mps,
                pnp_params_container_path=(
                    cmd.container_path(self.s.repo_dir, self.s.pnp_dir / params_file)
                    if params_file
                    else ""
                ),
            ),
            f"pnp:{backend}",
            {
                "dynamics": dynamics,
                "perception": perception,
                "backend": backend,
                "v_max_mps": v_max_mps,
                "params_file": params_file,
            },
        )

    def _signal_drive(self, signal: str) -> bool:
        """Send `signal` to the driving launch. True if a live process was found."""
        res = self.runner.run(
            cmd.exec_bash(self.s.stack_container, cmd.drive_signal_script(signal)), timeout=10
        )
        return res.ok

    def _signal_external(self, drive: DriveState, signal: str) -> bool:
        """Send `signal` to the driving launches the app did not start. True if any exist."""
        if not drive.external:
            return False
        self.runner.run(
            cmd.exec_bash(
                self.s.stack_container,
                cmd.drive_pids_signal_script([p.pid for p in drive.external], signal),
            ),
            timeout=10,
        )
        return True

    def _kill_drive(self) -> bool:
        """SIGINT every driving launch (like Ctrl-C), escalate if one lingers. No locks taken.

        That is the launch the app started and any started from a terminal in the container.
        """
        self.last_stopped_external = 0
        if self.container_states().get(self.s.stack_container) != "running":
            return False
        drive = self.drive_state()
        found = self._signal_drive("INT") if drive.label else False
        self.last_stopped_external = len(drive.external)
        found = self._signal_external(drive, "INT") or found
        if not found:
            return False
        for signal in ("TERM", "KILL"):
            for _ in range(6):
                self._sleep(0.5)
                if self.drive_mode() is None:
                    return True
            left = self.drive_state()
            if left.label:
                self._signal_drive(signal)
            self._signal_external(left, signal)
        self._sleep(0.5)
        return True

    def stop_all(self, actor: str = "") -> dict[str, Any]:
        """STOP ALL driving. The stack (and its recording) keeps running.

        safety_node then sees /drive_raw go silent and brakes. This is a software stop; Wi-Fi
        can drop and the process can lag. The kill knob is the stop that always works.
        """
        with self._gen_lock:
            self._stop_generation += 1
        killed = self._kill_drive()
        external = self.last_stopped_external
        # A start that was mid-flight holds the drive lock; wait for it, then kill again.
        got = self._drive_lock.acquire(timeout=10)
        try:
            if got:
                killed = self._kill_drive() or killed
                external = max(external, self.last_stopped_external)
        finally:
            if got:
                self._drive_lock.release()
        self._invalidate()
        detail = "stopped the driving launch" if killed else "no driving launch was running"
        if external:
            detail += f" ({external} started from a terminal)"
        self._log("stop_all", True, actor, detail)
        return {"message": detail, "was_running": killed}

    def park_command(
        self, name: str, override: bool = False, override_reason: str = "", actor: str = ""
    ) -> dict[str, Any]:
        action = f"park.{name}"
        service = cmd.PARK_SERVICES.get(name)
        if service is None:
            raise ActionError(f"unknown park command {name!r}", 400)
        mode = self.drive_mode()
        if mode is None or not mode.startswith("park"):
            self._log(action, False, actor, "park launch is not running")
            raise ActionError("start the park launch first", 409)
        used_override = False
        if name != "abort":  # abort always goes through, with no checks in the way
            try:
                used_override = self._gate_drive(override, override_reason)
            except ActionError as exc:
                self._log(action, False, actor, exc.message)
                raise
        res = self.runner.run(
            cmd.exec_bash(self.s.stack_container, cmd.park_service_script(service), login=True),
            timeout=30,
        )
        ok = res.ok and "success=True" in res.stdout.replace(" ", "")
        msg = "ok" if ok else f"service call failed: {res.tail()}"
        self._log(
            action,
            ok,
            actor,
            msg,
            override=used_override,
            override_reason=override_reason if used_override else "",
        )
        if not ok:
            raise ActionError(msg, 500)
        return {"message": f"{name}: ok"}

    # ------------------------------------------------------------------------------------
    # maps
    # ------------------------------------------------------------------------------------

    def save_map(self, name: str, actor: str = "") -> dict[str, Any]:
        if not cmd.NAME_RE.match(name):
            raise ActionError("map name: letters, digits, dot, dash and underscore only", 400)
        snap = self.snapshot(fresh=True)
        if not snap.stack_running or snap.stack_flags.get("mapping") != "true":
            self._log("map.save", False, actor, "mapping is not running", params={"name": name})
            raise ActionError("start the stack with mapping first", 409)
        if (self.s.maps / f"{name}.yaml").exists():
            raise ActionError(f"a map called {name} already exists: pick another name", 409)
        self.s.maps.mkdir(parents=True, exist_ok=True)
        container_base = cmd.container_path(self.s.repo_dir, self.s.maps / name)
        res = self.runner.run(
            cmd.exec_bash(self.s.stack_container, cmd.save_map_script(container_base), login=True),
            timeout=60,
        )
        saved = (self.s.maps / f"{name}.yaml").exists()
        if not res.ok or not saved:
            msg = f"save_map failed: {res.tail() or 'no map file appeared'}"
            self._log("map.save", False, actor, msg, params={"name": name})
            raise ActionError(msg, 500)
        self._log("map.save", True, actor, f"saved {name}", params={"name": name})
        return {"message": f"saved {name}.yaml", "name": name}

    # ------------------------------------------------------------------------------------
    # bags
    # ------------------------------------------------------------------------------------

    @staticmethod
    def _dir_size(path: Path) -> int:
        total = 0
        for entry in path.rglob("*"):
            try:
                if entry.is_file() and not entry.is_symlink():
                    total += entry.stat().st_size
            except OSError:
                continue
        return total

    def _growth_mb_s(self, name: str | None, size_bytes: int) -> float | None:
        """How fast the recording bag is growing, MB/s, over the last GROWTH_WINDOW_S seconds.

        None when nothing is recording, or until two samples at least GROWTH_MIN_SPAN_S apart
        exist (the first poll after the recorder starts, or after the agent restarts).
        """
        now = self._clock()
        with self._growth_lock:
            if name is None or name != self._growth_bag:
                self._growth_bag = name
                self._growth_samples.clear()
            if name is None:
                return None
            samples = self._growth_samples
            if samples and now - samples[-1][0] > GROWTH_WINDOW_S:
                samples.clear()  # a long gap (agent idle, clock jump): do not average across it
            if not samples or now - samples[-1][0] >= 1.0:  # two web clients must not skew it
                samples.append((now, size_bytes))
            while len(samples) > 2 and now - samples[0][0] > GROWTH_WINDOW_S:
                samples.popleft()
            (t0, b0), (t1, b1) = samples[0], samples[-1]
            if t1 - t0 < GROWTH_MIN_SPAN_S:
                return None
            return round(max(b1 - b0, 0) / (t1 - t0) / 1e6, 2)

    def list_bags(self) -> list[dict[str, Any]]:
        base = self.s.bags
        if not base.is_dir():
            return []
        bags = [p for p in base.iterdir() if p.is_dir() and not p.is_symlink()]
        bags.sort(key=lambda p: p.stat().st_mtime, reverse=True)
        recording = self.snapshot().recorder_running
        out = []
        active_name = bags[0].name if recording and bags else None
        for p in bags:
            size = self._dir_size(p)
            entry: dict[str, Any] = {
                "name": p.name,
                "size_mb": round(size / 1e6, 1),
                "mtime": p.stat().st_mtime,
                "active": p.name == active_name,
            }
            if entry["active"]:
                entry["growth_mb_s"] = self._growth_mb_s(p.name, size)
            out.append(entry)
        if active_name is None:
            self._growth_mb_s(None, 0)
        return out

    def delete_bag(self, name: str, confirm: str, actor: str = "") -> dict[str, Any]:
        if not cmd.BAG_RE.match(name) or name in (".", ".."):
            raise ActionError("bad bag name", 400)
        if confirm != name:
            raise ActionError("confirmation must repeat the bag name", 400)
        target = self.s.bags / name
        if not target.is_dir() or target.is_symlink():
            raise ActionError("no such bag", 404)
        if target.resolve().parent != self.s.bags.resolve():
            raise ActionError("bag is outside the bag directory", 400)
        active = [b for b in self.list_bags() if b["active"]]
        if active and active[0]["name"] == name:
            self._log("bag.delete", False, actor, "bag is being recorded", params={"name": name})
            raise ActionError("that bag is being recorded right now", 409)
        shutil.rmtree(target)
        self._invalidate()
        self._log("bag.delete", True, actor, "deleted", params={"name": name})
        return {"message": f"deleted {name}"}

    # ------------------------------------------------------------------------------------
    # system
    # ------------------------------------------------------------------------------------

    def power(self, kind: str, confirm: str, actor: str = "") -> dict[str, Any]:
        if kind not in ("reboot", "shutdown"):
            raise ActionError("unknown power action", 400)
        if confirm != kind:
            raise ActionError(f"type {kind!r} to confirm", 400)
        self._log(f"system.{kind}", True, actor, "stopping the stack cleanly first")
        # Each stop SIGINTs and waits (the bag has to be closed before the power goes), and writes
        # its own action log line saying what happened.
        with self._lock:
            self.stop_all(actor)
            self.stop_perception(actor)
            self.stop_cameras(actor)
            stopped = self.stop_stack(actor)
        verb = "reboot" if kind == "reboot" else "poweroff"

        def go() -> None:
            self.runner.run(["sudo", "-n", "/usr/bin/systemctl", verb], timeout=30)

        self._defer(2.0, go)
        return {"message": f"{kind} in 2 s: {stopped['message']}"}

    # ------------------------------------------------------------------------------------
    # autostart and boot bookkeeping
    # ------------------------------------------------------------------------------------

    def autostart_enabled(self) -> bool:
        return self.s.autostart_flag.exists()

    def set_autostart(self, enabled: bool, actor: str = "") -> dict[str, Any]:
        flag = self.s.autostart_flag
        flag.parent.mkdir(parents=True, exist_ok=True)
        if enabled:
            flag.touch()
        else:
            flag.unlink(missing_ok=True)
        self._log("settings.autostart", True, actor, "on" if enabled else "off")
        return {"autostart": enabled}

    def autostart_boot(self, wait_s: float = 45.0) -> dict[str, Any]:
        """What racer-stack.service runs at boot: lidar + vesc, no cameras, if the disk allows."""
        if not self.autostart_enabled():
            self._log("autostart", True, "boot", "disabled, doing nothing")
            return {"message": "autostart is off"}
        if self.container_states().get(self.s.stack_container) == "running":
            return {"message": "stack already running"}
        waited = 0.0
        while waited < wait_s and not (
            self.host.exists(cmd.VESC_DEVICE) and self.host.exists(cmd.LIDAR_DEVICE)
        ):
            self._sleep(1.0)
            waited += 1.0
        lidar = self.host.exists(cmd.LIDAR_DEVICE)
        vesc = self.host.exists(cmd.VESC_DEVICE)
        flags = cmd.StackFlags(lidar=lidar, vesc=vesc)
        if not (lidar and vesc):
            self._log(
                "autostart",
                True,
                "boot",
                f"starting degraded: lidar {'present' if lidar else 'MISSING'}, "
                f"vesc {'present' if vesc else 'MISSING'}",
            )
        return self.start_stack(flags, actor="boot")

    def startup_check(self) -> str | None:
        """Called when the agent starts. Returns a message if the last run did not end cleanly."""
        marker = self.s.run_marker
        marker.parent.mkdir(parents=True, exist_ok=True)
        previous = marker.read_text().strip() if marker.exists() else ""
        marker.write_text("running\n")
        uptime = self.host.uptime_s()
        note = None
        if previous == "running":
            note = (
                "unclean shutdown: the agent was not stopped cleanly last time "
                "(power loss, brown-out, crash or kill)"
            )
        self._log(
            "agent.start",
            True,
            "agent",
            note or "clean start",
            params={"uptime_s": round(uptime)} if uptime else None,
        )
        return note

    def shutdown_mark(self) -> None:
        try:
            self.s.run_marker.write_text("stopped\n")
        except OSError:
            log.warning("could not write the shutdown marker")

    def drive_log(self, lines: int = 60) -> str:
        res = self.runner.run(cmd.tail_argv(self.s.stack_container, lines), timeout=10)
        return res.stdout

    def _invalidate(self) -> None:
        with self._snap_lock:
            self._snap = None
