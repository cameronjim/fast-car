"""A fake docker / systemd / ROS layer for the agent tests.

`World` keeps the state the real machine would: which containers exist and whether they run,
whether the stack has a bag recorder, whether a driving launch is alive inside car-stack, a
fake disk. `World.responder` plugs into `FakeRunner` and answers the exact argv the agent
builds, so the tests exercise the real command construction and nothing is mocked inside it.
"""

from __future__ import annotations

import json
import re
from dataclasses import dataclass, field
from pathlib import Path

from racer_agent import commands as cmd
from racer_agent.hostinfo import GIB, HostInfo
from racer_agent.rosstate import RosState
from racer_agent.runner import CmdResult

APP_DRIVE_PID = 4242


def terminal_drive(pid: int, *words: str) -> dict:
    """A driving launch started from a terminal: `python3 /opt/ros/humble/bin/ros2 launch ...`."""
    return {"pid": pid, "argv": ["/usr/bin/python3", "/opt/ros/humble/bin/ros2", *words]}


@dataclass
class World:
    repo: Path
    free_gb: float = 100.0
    image_present: bool = True
    perception_image_present: bool = True  # car:perception, the GPU image
    heartbeat: str = "active"
    containers: dict[str, dict] = field(default_factory=dict)
    recorder: bool = True
    drive_label: str | None = None
    ignore_int: bool = False  # a driving launch that needs a TERM
    stack_dies_on_start: bool = False
    launch_dies_on_start: bool = False
    docker_run_fails: str = ""
    stuck_on_int: bool = False  # the stack ignores SIGINT until killed
    stuck_on_term: bool = False  # ... and SIGTERM too, until SIGKILL
    slow_stop_polls: int = 0  # after SIGINT the container takes this many polls to exit
    #: driving launches started from a terminal: {"pid": int, "argv": [str, ...]} (container pids)
    external_drives: list = field(default_factory=list)
    ignore_int_external: bool = False
    drive_cmd: str = ""  # the ros2 command of the app-started launch
    signals: list = field(default_factory=list)  # (container, signal) of every docker kill
    prune_fails: str = ""  # stderr of a failing docker image/builder prune
    reclaim: dict = field(default_factory=lambda: {"image": "1.2GB", "builder": "640MB"})
    park_reply: str = "response:\nstd_srvs.srv.Trigger_Response(success=True, message='ok')"

    def disk_usage(self, _path: str) -> tuple[int, int, int]:
        total = 200 * GIB
        free = int(self.free_gb * GIB)
        return total, total - free, free

    # -- the responder ---------------------------------------------------------------------
    def responder(self, argv: tuple[str, ...]):
        if argv[:2] == ("systemctl", "is-active"):
            return (0 if self.heartbeat == "active" else 3, self.heartbeat + "\n")
        if argv[:3] == ("sudo", "-n", "/usr/bin/systemctl"):
            return (0, "")
        if argv[0] != "docker":
            return None
        sub = argv[1]
        if sub == "ps":
            lines = [f"{n}\t{c['state']}" for n, c in self.containers.items()]
            return (0, "\n".join(lines) + "\n")
        if argv[1:3] in (("image", "prune"), ("builder", "prune")):
            if self.prune_fails:
                return CmdResult(argv, 1, "", self.prune_fails)
            kind = "Deleted Images" if argv[1] == "image" else "Deleted build cache objects"
            return (0, f"{kind}:\nsha256:abc\n\nTotal reclaimed space: {self.reclaim[argv[1]]}\n")
        if sub == "image":
            if argv[-1] == "car:perception":
                return (0 if self.perception_image_present else 1, "")
            return (0 if self.image_present else 1, "")
        if sub == "inspect":
            return self._inspect(argv)
        if sub == "run":
            return self._run(argv)
        if sub == "rm":
            c = self.containers.get(argv[2])
            if c is None:
                return (1, "")
            if c["state"] == "running":
                return (1, "")
            del self.containers[argv[2]]
            return (0, "")
        if sub == "kill":
            return self._kill(argv)
        if sub == "top":
            name = argv[2]
            c = self.containers.get(name)
            if not c or c["state"] != "running":
                return (1, "")
            lines = [
                "ARGS",
                "/usr/bin/python3 /opt/ros/humble/bin/ros2 launch racer_bringup car_teleop",
            ]
            if self.recorder:
                lines.append("/usr/bin/python3 /opt/ros/humble/bin/ros2 bag record -o x --regex y")
            return (0, "\n".join(lines) + "\n")
        if sub == "logs":
            return (0, "some log text\n")
        if sub == "exec":
            return self._exec(argv)
        return None

    def _inspect(self, argv):
        if argv[2] == "-f" and argv[3] == "{{json .Config.Cmd}}":
            c = self.containers.get(argv[4])
            return (0, json.dumps(c["cmd"])) if c else (1, "")
        if argv[2] == "-f" and argv[3] == "{{.State.Running}}":
            c = self.containers.get(argv[4])
            if not c:
                return (1, "")
            if c["state"] == "stopping":
                c["polls"] = c.get("polls", 0) - 1
                if c["polls"] <= 0:
                    c["state"] = "exited"
                    c.get("on_exit", lambda: None)()
                    self.drive_label = None
                    self.external_drives.clear()
            return (0, "true" if c["state"] in ("running", "stopping") else "false")
        if argv[2] == "-f" and argv[3] == "{{.State.ExitCode}}":
            return (0, "0\n") if argv[4] in self.containers else (1, "")
        if argv[2] == "image":
            return None
        return None

    def _run(self, argv):
        if "--rm" in argv:  # the policy loader check, handled by tests with their own responder
            return None
        if self.docker_run_fails:
            return CmdResult(argv, 125, "", self.docker_run_fails)
        name = argv[argv.index("--name") + 1]
        if name in self.containers:
            return CmdResult(argv, 125, "", "name already in use")
        state = "exited" if self.stack_dies_on_start and name == "car-stack" else "running"
        self.containers[name] = {"state": state, "cmd": list(argv[-3:]), "argv": argv}
        return (0, "abc123\n")

    def _kill(self, argv):
        name = argv[-1]
        c = self.containers.get(name)
        if not c:
            return (1, "")
        sig = argv[argv.index("-s") + 1] if "-s" in argv else "KILL"
        self.signals.append((name, sig))
        if sig == "INT":
            if self.stuck_on_int:
                return (0, "")
            if self.slow_stop_polls:
                c["state"] = "stopping"
                c["polls"] = self.slow_stop_polls
                return (0, "")
        if sig == "TERM" and self.stuck_on_term:
            return (0, "")
        c["state"] = "exited"
        c.get("on_exit", lambda: None)()
        self.drive_label = None
        self.external_drives.clear()
        return (0, "")

    def _exec(self, argv):
        detached = argv[2] == "-d"
        rest = argv[3:] if detached else argv[2:]
        name, script = rest[0], rest[-1]
        c = self.containers.get(name)
        if not c or c["state"] != "running":
            return (1, "")
        if detached:
            if self.launch_dies_on_start:
                return (0, "")
            m = re.search(r"echo (\S+) > /tmp/racer-drive.mode", script)
            self.drive_label = m.group(1) if m else "unknown"
            m = re.search(r"exec (ros2 .*?) > /tmp/racer-drive.log", script)
            self.drive_cmd = m.group(1) if m else ""
            return (0, "")
        if script == cmd.DRIVE_STATUS_SCRIPT:
            return (0, f"{APP_DRIVE_PID}\n{self.drive_label}\n") if self.drive_label else (1, "")
        if script == cmd.DRIVE_SCAN_SCRIPT:
            return (0, self._scan())
        m = re.fullmatch(
            r"for p in ([0-9 ]+); do grep -aq ros2 /proc/\$p/cmdline 2>/dev/null && "
            r"\{ pkill -(INT|TERM|KILL) -P \$p; kill -\2 \$p; \}; done",
            script,
        )
        if m:
            pids = {int(x) for x in m.group(1).split()}
            sig = m.group(2)
            if sig == "INT" and self.ignore_int_external:
                return (0, "")
            self.external_drives[:] = [d for d in self.external_drives if d["pid"] not in pids]
            if APP_DRIVE_PID in pids:
                self.drive_label = None
            return (0, "")
        m = re.search(r"\{ pkill -(INT|TERM|KILL) -P \$p; kill -\1 \$p; \}$", script)
        if m:
            if self.drive_label is None:
                return (1, "")
            sig = m.group(1)
            if sig == "INT" and self.ignore_int:
                return (0, "")
            self.drive_label = None
            return (0, "")
        if script.startswith("tail "):
            return (0, "launch said something\n")
        if "ros2 service call /park_node" in script:
            return (0, self.park_reply)
        if "save_map" in script:
            m2 = re.search(r"data: (\S+?)\}\}", script)
            host_path = self.repo / m2.group(1).removeprefix("/workspace/")
            host_path.parent.mkdir(parents=True, exist_ok=True)
            Path(str(host_path) + ".yaml").write_text("image: x.pgm\n")
            return (0, "result: 0")
        return (1, "")

    def _scan(self) -> str:
        """What DRIVE_SCAN_SCRIPT prints: every process whose command line mentions ros2."""
        sep = "\x1f"
        lines = [
            f"1 python3{sep}/opt/ros/humble/bin/ros2{sep}launch{sep}racer_bringup{sep}car_teleop.launch.py"
        ]
        lines.append(
            f"88 bash{sep}-c{sep}ros2 launch racer_control gap_follow.launch.py"
        )  # a wrapper
        if self.drive_label:
            words = (self.drive_cmd or "ros2 launch racer_control gap_follow.launch.py").split()
            lines.append(f"{APP_DRIVE_PID} python3{sep}/opt/ros/humble/bin/{sep.join(words)}")
        for d in self.external_drives:
            lines.append(f"{d['pid']} " + sep.join(d["argv"]))
        return "\n".join(lines) + "\n"

    # -- convenience ----------------------------------------------------------------------
    def start_stack(self, flags: str = "lidar:=true vesc:=true", state: str = "running"):
        self.containers["car-stack"] = {
            "state": state,
            "cmd": ["bash", "-lc", f"exec ros2 launch racer_bringup car_teleop.launch.py {flags}"],
            "argv": (),
        }


def make_host(tmp_path: Path, world: World, devices: bool = True, camera: bool = True) -> HostInfo:
    root = tmp_path / "root"
    if devices:
        for dev in ("dev/ttyUSB0", "dev/ttyTHS1"):
            (root / dev).parent.mkdir(parents=True, exist_ok=True)
            (root / dev).write_text("")
    if camera:
        for idx, (name, index) in enumerate(
            (("ELP Global Shutter Camera", 0), ("ELP Global Shutter Camera", 1))
        ):
            d = root / "sys/class/video4linux" / f"video{idx + 1}"
            d.mkdir(parents=True, exist_ok=True)
            (d / "name").write_text(name + "\n")
            (d / "index").write_text(f"{index}\n")
    (root / "proc").mkdir(parents=True, exist_ok=True)
    (root / "proc/uptime").write_text("5000.00 4000.00\n")
    zone = root / "sys/class/thermal/thermal_zone0"
    zone.mkdir(parents=True, exist_ok=True)
    (zone / "type").write_text("cpu-thermal\n")
    (zone / "temp").write_text("48000\n")
    return HostInfo(
        root=root,
        uid=1000,
        gid=1000,
        group_lookup=lambda n: {"gpio": 997, "dialout": 20, "video": 44}.get(n),
        disk_usage=world.disk_usage,
    )


def healthy_ros(ros: RosState, scan_hz: float = 10.0, voltage: float = 12.0) -> None:
    """Fill a RosState as a healthy running car would."""
    ros.set_connected(True)
    ros.set_topics({"/drive", "/drive_raw", "/safety/events", "/scan", "/odom/wheel"})
    ros.set_value("voltage_v", voltage)
    ros.set_value("temp_fet_degc", 35.0)
    ros.set_value("temp_motor_degc", 0.0)
    ros.set_value("fault", "NONE")
    ros.set_value("current_input_a", 0.2)
    # a window's worth of scan stamps
    n = int(scan_hz * 4.0)
    for _ in range(n):
        ros.count("/scan")
