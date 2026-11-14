"""pnp.launch.py -- the Plug-and-Play mapless driver (claude-docs/13-plug-and-play.md).

    ros2 launch racer_bringup pnp.launch.py \
        perception_bundle:=/workspace/data/pnp/models/<perception model_id> \
        dynamics_bundle:=/workspace/data/pnp/models/<dynamics model_id> v_max_mps:=0.5

Started in a second terminal next to the running stack (`car_teleop.launch.py lidar:=true
vesc:=true`, recorder on), exactly like racer_control's gap_follow.launch.py. It starts the
driving nodes and nothing else that touches the command path:

    racer_track_perception/track_perception_node   /scan -> /pnp/local_track
    racer_planner/mppi_planner_node                /pnp/local_track + /odom/wheel -> /drive_raw

mppi_planner_node is the only publisher of /drive_raw and nothing here publishes /drive or remaps
anything onto it: the command still goes through safety_node and the hardware mux (CLAUDE.md
invariant 1). One driver at a time: never start this next to gap_follow or the tracker.

Speeds. `v_max_mps` is the absolute cap handed to the planner (the ceiling; the planner already
refuses a value above the dynamics bundle's validity). `v_lap1_mps` (default: equal to `v_max_mps`,
so a launch without lap memory behaves exactly as before) is the cap of lap 1: the lap memory's
start cap and, with `lap_memory:=true`, the planner's `v_no_guidance_mps`, the speed it uses when
the guidance is missing, stale or malformed. With lap memory a dead lap node therefore leaves the
car at the lap-1 cap, never at the ceiling.

Refusals (a launch that does not start is the safe outcome):

* `require_recorder` (default true): the launch refuses to start unless a rosbag2 recorder node
  is in the ROS graph (CLAUDE.md invariant 5: every run is logged). The check waits up to 5 s.
  Only sim tests pass false.
* `dynamics_bundle` is required, and `perception_bundle` is required with `backend:=net`.
* `lap_memory:=true` with `v_lap1_mps` above `v_max_mps`; `lap_mode:=v1` with
  `lap_source:=map_pose` (D v1 is indexed by station).
* `require_lap_counter:=true` (evaluation runs) refuses when no `lap_counter_node` is in the graph
  within 5 s (checked at once with `lap_counter:=false`, five seconds after start otherwise, when
  the launch shuts down).
* If any driving node or `lap_memory_node` exits for any reason the whole launch shuts down, so a
  node that refuses its bundle leaves nothing running; safety_node's watchdog then brakes on the
  silent /drive_raw.

Options that are measurements, not part of the command path:

* `lap_memory` (default false) adds racer_laps' lap_memory_node (`lap_source:=start_scan|map_pose`,
  `lap_mode:=v0|v1`) and sets the planner's `use_lap_guidance`.
* `lap_counter` (default true) starts racer_perception's camera lap counter unless a node named
  `lap_counter_node` is already in the graph, and calls `/lap_counter/reset` once so the
  reference frame is this run's start line. If the package or OpenCV is missing (the node exits
  with code 3) the launch logs one clear error and the driver keeps running. `/lap/event` and
  `/lap/status` are in the stack recorder's regex.
* `deskew` (default false) starts racer_track_perception's scan_deskew_node (`/scan`, `/odom/wheel`
  and the gated `/drive` steering in, `/pnp/scan_deskewed` out) and points the scan subscription
  of track_perception_node, mppi_planner_node, lap_memory_node and the shadow perception node at
  it. safety_node is not in this launch and keeps reading the raw `/scan`. The node's `stamp_is`
  and `beam_order` are what the measurement M-DSK chose on real bags (DESKEW_STAMP_IS and
  DESKEW_BEAM_ORDER below, docs/notes/scan-deskew-2026-11.md); until they are set the launch
  refuses `deskew:=true`. With `deskew:=false` the launch is the one it was before this option.
* `shadow_perception_bundle:=<dir>` starts a SECOND perception node, `track_perception_shadow`
  (`backend:=net`), publishing `/pnp/shadow/local_track` and `/pnp/shadow/perception/status`.
  Nothing subscribes to them; the recorder takes them (`/pnp/.*`). Its exit does not shut the
  launch down.
"""

from __future__ import annotations

import time
from pathlib import Path

from launch import LaunchDescription
from launch.actions import (
    DeclareLaunchArgument,
    EmitEvent,
    ExecuteProcess,
    IncludeLaunchDescription,
    LogInfo,
    OpaqueFunction,
    RegisterEventHandler,
    TimerAction,
)
from launch.event_handlers import OnProcessExit
from launch.events import Shutdown
from launch.launch_context import LaunchContext
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node

BACKENDS = ("net", "geometric")
LAP_SOURCES = ("start_scan", "map_pose")
LAP_MODES = ("v0", "v1")
RECORDER_NODE_SUBSTRING = "rosbag2_recorder"
RECORDER_WAIT_S = 5.0
RECORDER_REFUSAL = "refusing to drive: no rosbag recorder in the graph (every run is logged)"
LAP_COUNTER_NODE = "lap_counter_node"
LAP_COUNTER_WAIT_S = 5.0
LAP_COUNTER_PROBE_S = 1.0
LAP_COUNTER_REFUSAL = (
    "refusing to drive: require_lap_counter is true and no lap_counter_node is in the graph"
)
LAP_COUNTER_RESET_TIMEOUT_S = 30
LAP_COUNTER_OPENCV_EXIT = 3
DESKEW_NODE_NAME = "scan_deskew_node"
DESKEW_TOPIC = "/pnp/scan_deskewed"
DESKEW_REMAPPING = ("/scan", DESKEW_TOPIC)
# What M-DSK step A chose on the train role (docs/notes/scan-deskew-2026-11.md). None until then:
# the launch refuses deskew:=true, so a node never starts on a guess.
DESKEW_STAMP_IS: str | None = None
DESKEW_BEAM_ORDER: str | None = None
SHADOW_NODE_NAME = "track_perception_shadow"
SHADOW_REMAPPINGS = [
    ("/pnp/local_track", "/pnp/shadow/local_track"),
    ("/pnp/perception/status", "/pnp/shadow/perception/status"),
]

# Parameter names the two nodes declare for their bundle directories.
PERCEPTION_BUNDLE_PARAM = "bundle_dir"
DYNAMICS_BUNDLE_PARAM = "dynamics_bundle"


def _node_in_graph(matches, timeout_s: float) -> bool:
    """True once a node whose name satisfies `matches` is in the ROS graph, polling for up to
    `timeout_s`. Uses a short-lived rclpy node in its own context, so it neither disturbs nor
    depends on the launch's own rclpy state."""
    import rclpy

    context = rclpy.Context()
    rclpy.init(context=context)
    try:
        node = rclpy.create_node("pnp_launch_probe", context=context)
        try:
            deadline = time.monotonic() + timeout_s
            while True:
                if any(matches(name) for name in node.get_node_names()):
                    return True
                if time.monotonic() >= deadline:
                    return False
                time.sleep(0.1)
        finally:
            node.destroy_node()
    finally:
        rclpy.shutdown(context=context)


def recorder_in_graph(timeout_s: float = RECORDER_WAIT_S) -> bool:
    """True once a node whose name contains `rosbag2_recorder` is in the ROS graph, polling for
    up to `timeout_s`."""
    return _node_in_graph(lambda name: RECORDER_NODE_SUBSTRING in name, timeout_s)


def lap_counter_in_graph(timeout_s: float = LAP_COUNTER_PROBE_S) -> bool:
    """True once a node named `lap_counter_node` is in the ROS graph, polling for up to
    `timeout_s`."""
    return _node_in_graph(lambda name: name == LAP_COUNTER_NODE, timeout_s)


def parameters_for(
    perception_bundle: str,
    dynamics_bundle: str,
    backend: str,
    v_max_mps: float,
    laser_yaw_from_vehicle_params: bool,
) -> tuple[dict, dict]:
    """(perception parameters, planner parameters) as plain dicts, so they can be checked.
    Raises, so the launch fails before any node starts, on a missing bundle or a bad backend."""
    if backend not in BACKENDS:
        raise RuntimeError(
            f"pnp.launch.py: backend must be one of {list(BACKENDS)}, got {backend!r}"
        )
    if not dynamics_bundle:
        raise RuntimeError(
            "pnp.launch.py: dynamics_bundle:=<dynamics bundle directory> is required"
        )
    if backend == "net" and not perception_bundle:
        raise RuntimeError(
            "pnp.launch.py: perception_bundle:=<track_perception bundle directory> is required "
            "with backend:=net"
        )
    perception: dict = {
        "backend": backend,
        "laser_yaw_from_vehicle_params": laser_yaw_from_vehicle_params,
    }
    if perception_bundle:
        perception[PERCEPTION_BUNDLE_PARAM] = perception_bundle
    planner: dict = {
        DYNAMICS_BUNDLE_PARAM: dynamics_bundle,
        "v_max_mps": float(v_max_mps),
        "laser_yaw_from_vehicle_params": laser_yaw_from_vehicle_params,
    }
    return perception, planner


def deskew_parameters() -> dict:
    """Parameters of scan_deskew_node: the choices of M-DSK step A. Refuses (so the launch fails
    before any node starts) while they are not set."""
    if DESKEW_STAMP_IS is None or DESKEW_BEAM_ORDER is None:
        raise RuntimeError(
            "pnp.launch.py: deskew:=true needs DESKEW_STAMP_IS and DESKEW_BEAM_ORDER, the "
            "choices of the M-DSK selection step, and they are not set"
        )
    return {"stamp_is": DESKEW_STAMP_IS, "beam_order": DESKEW_BEAM_ORDER}


def deskew_node_for(params_file: str) -> Node:
    base = [params_file] if params_file else []
    return Node(
        package="racer_track_perception",
        executable="scan_deskew_node",
        name=DESKEW_NODE_NAME,
        output="screen",
        parameters=[*base, deskew_parameters()],
    )


def nodes_for(
    perception_params: dict,
    planner_params: dict,
    params_file: str,
    scan_remapping: tuple[str, str] | None = None,
) -> tuple[Node, Node]:
    # A params file comes first so the explicit launch values above it win. `scan_remapping`
    # (deskew:=true) points each node's /scan subscription at /pnp/scan_deskewed; None leaves the
    # node exactly as it was.
    base = [params_file] if params_file else []
    remappings = [scan_remapping] if scan_remapping else None
    perception = Node(
        package="racer_track_perception",
        executable="track_perception_node",
        name="track_perception_node",
        output="screen",
        parameters=[*base, perception_params],
        remappings=remappings,
    )
    planner = Node(
        package="racer_planner",
        executable="mppi_planner_node",
        name="mppi_planner_node",
        output="screen",
        parameters=[*base, planner_params],
        remappings=remappings,
    )
    return perception, planner


def shadow_node_for(
    shadow_bundle: str,
    laser_yaw_from_vehicle_params: bool,
    params_file: str,
    scan_remapping: tuple[str, str] | None = None,
) -> Node:
    """A second perception node (net backend) whose outputs nothing subscribes to."""
    base = [params_file] if params_file else []
    return Node(
        package="racer_track_perception",
        executable="track_perception_node",
        name=SHADOW_NODE_NAME,
        output="screen",
        parameters=[
            *base,
            {
                "backend": "net",
                PERCEPTION_BUNDLE_PARAM: shadow_bundle,
                "laser_yaw_from_vehicle_params": laser_yaw_from_vehicle_params,
            },
        ],
        remappings=[*SHADOW_REMAPPINGS, *([scan_remapping] if scan_remapping else [])],
    )


def validity_v_max_mps(dynamics_bundle: str) -> float:
    """`io.validity.v_max_mps` of the dynamics bundle's manifest. Raises (so the launch fails
    before any node starts) if it cannot be read: the lap memory cap ceiling comes from here."""
    import yaml

    path = Path(dynamics_bundle) / "manifest.yaml"
    try:
        manifest = yaml.safe_load(path.read_text(encoding="utf-8"))
        return float(manifest["io"]["validity"]["v_max_mps"])
    except (OSError, KeyError, TypeError, ValueError, yaml.YAMLError) as exc:
        raise RuntimeError(f"pnp.launch.py: no io.validity.v_max_mps in {path}") from exc


def lap_memory_parameters(
    dynamics_bundle: str,
    v_max_mps: float,
    v_lap1_mps: float | None = None,
    lap_mode: str = "v0",
    lap_source: str = "start_scan",
) -> dict:
    """Parameters of racer_laps' lap_memory_node. The cap starts at `v_lap1_mps` (default
    `v_max_mps`) and can rise to, never above, the lower of `v_max_mps` and the dynamics
    bundle's validity range. Refuses `v_lap1_mps` above `v_max_mps` and `lap_mode:=v1` with the
    map-pose source."""
    if lap_source not in LAP_SOURCES:
        raise RuntimeError(f"pnp.launch.py: lap_source must be one of {list(LAP_SOURCES)}")
    if lap_mode not in LAP_MODES:
        raise RuntimeError(f"pnp.launch.py: lap_mode must be one of {list(LAP_MODES)}")
    if lap_mode == "v1" and lap_source != "start_scan":
        raise RuntimeError(
            "pnp.launch.py: lap_mode:=v1 needs lap_source:=start_scan (D v1 is indexed by station)"
        )
    lap1 = float(v_max_mps) if v_lap1_mps is None else float(v_lap1_mps)
    if lap1 > float(v_max_mps):
        raise RuntimeError(
            f"pnp.launch.py: v_lap1_mps {lap1} is above v_max_mps {v_max_mps} "
            "(lap 1 can not be faster than the ceiling)"
        )
    ceiling = min(float(v_max_mps), validity_v_max_mps(dynamics_bundle))
    return {
        "v_cap_ceiling_mps": ceiling,
        "v_cap_start_mps": min(lap1, ceiling),
        "lap_mode": lap_mode,
        "lap_source": lap_source,
    }


def _bool(value: str, name: str) -> bool:
    value = value.strip().lower()
    if value not in ("true", "false"):
        raise RuntimeError(f"pnp.launch.py: {name} must be true or false, got {value!r}")
    return value == "true"


def _log_exit(label: str, note: str):
    """An exit handler that logs and does NOT shut the launch down."""

    def handler(event, context):
        name = getattr(event, "process_name", "")
        code = getattr(event, "returncode", None)
        return [LogInfo(msg=f"[pnp.launch.py] {label} ({name}) exited with code {code}: {note}")]

    return handler


def lap_counter_actions(require: bool) -> list:
    """Start the camera lap counter and reset it once. A failure of the counter (OpenCV missing,
    exit code 3, or the package absent) is logged and never stops the driver."""
    from ament_index_python.packages import PackageNotFoundError, get_package_share_directory

    try:
        share = get_package_share_directory("racer_perception")
    except PackageNotFoundError:
        return [
            LogInfo(
                msg="[pnp.launch.py] ERROR: racer_perception is not installed, so there is no "
                "camera lap counter in this run; the driver keeps running without one"
            )
        ]
    include = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(str(Path(share) / "launch" / "lap_counter.launch.py"))
    )

    def on_exit(event, context):
        if "lap_counter_node" not in getattr(event, "process_name", ""):
            return []
        code = getattr(event, "returncode", None)
        why = (
            "OpenCV is missing (install python3-opencv)"
            if code == LAP_COUNTER_OPENCV_EXIT
            else "see its log"
        )
        return [
            LogInfo(
                msg=f"[pnp.launch.py] ERROR: lap_counter_node exited with code {code}: {why}. "
                "The counter is a measurement, not part of the command path: the driver keeps "
                "running without lap times from the camera"
            )
        ]

    return [include, RegisterEventHandler(OnProcessExit(on_exit=on_exit))]


def lap_counter_reset_action() -> ExecuteProcess:
    """One `/lap_counter/reset` call: `ros2 service call` waits for the service, `timeout` stops
    it waiting for ever when the counter never comes up."""
    return ExecuteProcess(
        cmd=[
            "timeout",
            str(LAP_COUNTER_RESET_TIMEOUT_S),
            "ros2",
            "service",
            "call",
            "/lap_counter/reset",
            "std_srvs/srv/Trigger",
            "{}",
        ],
        output="screen",
    )


def require_counter_check(context: LaunchContext):
    """After the counter had its five seconds: shut the launch down if it is not in the graph."""
    if lap_counter_in_graph(0.5):
        return []
    return [
        LogInfo(msg=f"[pnp.launch.py] ERROR: {LAP_COUNTER_REFUSAL}"),
        EmitEvent(event=Shutdown(reason=LAP_COUNTER_REFUSAL)),
    ]


def _make_actions(context: LaunchContext) -> list:
    def arg(name: str) -> str:
        return LaunchConfiguration(name).perform(context).strip()

    v_max = float(arg("v_max_mps"))
    perception_params, planner_params = parameters_for(
        arg("perception_bundle"),
        arg("dynamics_bundle"),
        arg("backend"),
        v_max,
        _bool(arg("laser_yaw_from_vehicle_params"), "laser_yaw_from_vehicle_params"),
    )
    lap_memory = _bool(arg("lap_memory"), "lap_memory")
    deskew_on = _bool(arg("deskew"), "deskew")
    scan_remapping = DESKEW_REMAPPING if deskew_on else None
    deskew_node = deskew_node_for(arg("params_file")) if deskew_on else None
    lap_counter = _bool(arg("lap_counter"), "lap_counter")
    require_counter = _bool(arg("require_lap_counter"), "require_lap_counter")
    v_lap1_text = arg("v_lap1_mps")
    v_lap1 = float(v_lap1_text) if v_lap1_text else None
    lap_params = None
    if lap_memory:
        lap_params = lap_memory_parameters(
            arg("dynamics_bundle"), v_max, v_lap1, arg("lap_mode"), arg("lap_source")
        )
        planner_params["use_lap_guidance"] = True
        # missing, stale or malformed guidance: the lap-1 cap, never the ceiling
        planner_params["v_no_guidance_mps"] = float(lap_params["v_cap_start_mps"])
        lap_params["laser_yaw_from_vehicle_params"] = perception_params[
            "laser_yaw_from_vehicle_params"
        ]
    if _bool(arg("require_recorder"), "require_recorder") and not recorder_in_graph():
        raise RuntimeError(RECORDER_REFUSAL)
    if require_counter and not lap_counter and not lap_counter_in_graph(LAP_COUNTER_WAIT_S):
        raise RuntimeError(LAP_COUNTER_REFUSAL)
    perception, planner = nodes_for(
        perception_params, planner_params, arg("params_file"), scan_remapping
    )
    stop_all = [
        RegisterEventHandler(
            OnProcessExit(
                target_action=node,
                on_exit=[EmitEvent(event=Shutdown(reason=f"{name} exited"))],
            )
        )
        for node, name in ((perception, "track_perception_node"), (planner, "mppi_planner_node"))
    ]
    actions = [perception, planner, *stop_all]
    if deskew_node is not None:
        actions += [
            deskew_node,
            RegisterEventHandler(
                OnProcessExit(
                    target_action=deskew_node,
                    on_exit=[EmitEvent(event=Shutdown(reason="scan_deskew_node exited"))],
                )
            ),
        ]
    if lap_params is not None:
        base = [arg("params_file")] if arg("params_file") else []
        laps = Node(
            package="racer_laps",
            executable="lap_memory_node",
            name="lap_memory_node",
            output="screen",
            parameters=[*base, lap_params],
            remappings=[scan_remapping] if scan_remapping else None,
        )
        actions += [
            laps,
            RegisterEventHandler(
                OnProcessExit(
                    target_action=laps,
                    on_exit=[EmitEvent(event=Shutdown(reason="lap_memory_node exited"))],
                )
            ),
        ]
    shadow_bundle = arg("shadow_perception_bundle")
    if shadow_bundle:
        shadow = shadow_node_for(
            shadow_bundle,
            perception_params["laser_yaw_from_vehicle_params"],
            arg("params_file"),
            scan_remapping,
        )
        actions += [
            shadow,
            RegisterEventHandler(
                OnProcessExit(
                    target_action=shadow,
                    on_exit=_log_exit(
                        "shadow perception",
                        "a measurement only, nothing reads it: the driver keeps running",
                    ),
                )
            ),
        ]
    if lap_counter:
        if lap_counter_in_graph():
            actions.append(
                LogInfo(msg="[pnp.launch.py] a lap_counter_node is already running: not started")
            )
        else:
            actions += lap_counter_actions(require_counter)
        actions.append(lap_counter_reset_action())
        if require_counter:
            actions.append(
                TimerAction(
                    period=LAP_COUNTER_WAIT_S,
                    actions=[OpaqueFunction(function=require_counter_check)],
                )
            )
    return actions


def generate_launch_description() -> LaunchDescription:
    return LaunchDescription(
        [
            DeclareLaunchArgument(
                "perception_bundle",
                default_value="",
                description="track_perception bundle directory. Required with backend:=net.",
            ),
            DeclareLaunchArgument(
                "dynamics_bundle",
                default_value="",
                description="dynamics bundle directory (required).",
            ),
            DeclareLaunchArgument(
                "backend",
                default_value="geometric",
                choices=list(BACKENDS),
                description=(
                    "Track perception backend: geometric (default: the net failed the real-bag "
                    "gate, 2026-11-11) or net (needs perception_bundle)."
                ),
            ),
            DeclareLaunchArgument(
                "v_max_mps",
                default_value="0.5",
                description=(
                    "The absolute speed cap in m/s passed to the planner: the ceiling. With "
                    "lap_memory the lap cap rises towards it."
                ),
            ),
            DeclareLaunchArgument(
                "v_lap1_mps",
                default_value="",
                description=(
                    "Cap of lap 1 in m/s: the lap memory's start cap and the planner's "
                    "v_no_guidance_mps. Empty: equal to v_max_mps. Refused above v_max_mps "
                    "with lap_memory:=true."
                ),
            ),
            DeclareLaunchArgument(
                "require_recorder",
                default_value="true",
                choices=["true", "false"],
                description=(
                    "Refuse to start unless a rosbag recorder node is in the ROS graph. "
                    "false only in sim tests."
                ),
            ),
            DeclareLaunchArgument(
                "laser_yaw_from_vehicle_params",
                default_value="true",
                choices=["true", "false"],
                description=(
                    "true on the car: the LiDAR mount yaw comes from vehicle_params. false only "
                    "in the simulator, whose scan is already in the vehicle frame."
                ),
            ),
            DeclareLaunchArgument(
                "lap_memory",
                default_value="false",
                choices=["true", "false"],
                description=(
                    "Also start racer_laps' lap_memory_node and let the planner use its speed "
                    "cap. The cap ceiling is the lower of v_max_mps and the dynamics bundle's "
                    "validity v_max_mps. Off for a first run."
                ),
            ),
            DeclareLaunchArgument(
                "lap_mode",
                default_value="v0",
                choices=list(LAP_MODES),
                description="D v0 (cap schedule) or v1 (station set); v1 needs start_scan.",
            ),
            DeclareLaunchArgument(
                "lap_source",
                default_value="start_scan",
                choices=list(LAP_SOURCES),
                description=(
                    "How a lap closes: start_scan (start-scan gate plus wheel odometry, no map "
                    "after lap 1) or map_pose (TF map -> base_link, needs mapping or "
                    "localisation)."
                ),
            ),
            DeclareLaunchArgument(
                "lap_counter",
                default_value="true",
                choices=["true", "false"],
                description=(
                    "Start the camera lap counter (racer_perception) unless one is already in "
                    "the graph, and reset it once. A measurement, not part of the command path."
                ),
            ),
            DeclareLaunchArgument(
                "require_lap_counter",
                default_value="false",
                choices=["true", "false"],
                description=(
                    "Refuse to drive when no lap_counter_node is in the graph within 5 s "
                    "(evaluation runs)."
                ),
            ),
            DeclareLaunchArgument(
                "deskew",
                default_value="false",
                choices=["true", "false"],
                description=(
                    "Start scan_deskew_node and point perception, the planner, lap memory and "
                    "the shadow perception at /pnp/scan_deskewed. safety_node keeps the raw "
                    "/scan. Off unless M-DSK passed (docs/notes/scan-deskew-2026-11.md)."
                ),
            ),
            DeclareLaunchArgument(
                "shadow_perception_bundle",
                default_value="",
                description=(
                    "A perception bundle for a SECOND, shadow perception node (net backend) "
                    "publishing /pnp/shadow/local_track. Nothing subscribes to it."
                ),
            ),
            DeclareLaunchArgument(
                "params_file",
                default_value="",
                description="Optional ROS parameter YAML applied to all nodes.",
            ),
            OpaqueFunction(function=_make_actions),
        ]
    )
