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
  `lap_source:=map_pose` (D v1 is indexed by station: start_scan or camera); an unknown
  `dv1_variant`.
* `lap_source:=truth` always: it is a simulator source (the sim runner starts its own lap node
  with it) and the car has no truth. `lap_source:=camera` with `lap_counter:=false` refuses when no
  `lap_counter_node` is in the graph (the wording of the `require_lap_counter` refusal), and with
  `lap_counter:=true` the planner waits for the counter exactly as with `require_lap_counter`.
  `lap_shadow` equal to `lap_source`, or `lap_shadow` without `lap_memory:=true`.
* `require_lap_counter:=true` (evaluation runs) refuses when no `lap_counter_node` is in the graph
  (checked for 5 s before anything starts with `lap_counter:=false`). With `lap_counter:=true` the
  planner is NOT started until the counter is in the graph (it takes about 4 s to come up on the
  Jetson); the wait is bounded (30 s) and ends in a refusal and a shutdown, with the planner never
  having been started.
* If any driving node or `lap_memory_node` exits for any reason the whole launch shuts down, so a
  node that refuses its bundle leaves nothing running; safety_node's watchdog then brakes on the
  silent /drive_raw.

Prototype switch (owner decision 29, default off):

* `dynamics_adaptation` (default false): the planner's online speed correction
  (racer_planner/speed_adaptation.hpp): a bounded bias and gain on the model's speed channel,
  fitted at run time from /odom/wheel, status on `/pnp/dynamics/adaptation`, reset by an empty
  message on `/pnp/dynamics/adaptation_reset`. With false the planner gets no extra parameter and
  runs exactly as before. Independent of which `dynamics_bundle` is loaded.
* `dynamics_fast_mlp` (default false): the planner evaluates the bundle's learned MLP (the
  dyn-v0.7p residual) with a vectorised tanh, a few 1e-16 from the exact pass, checked against the
  bundle's golden rollouts at load; about 2.6x faster solves with dyn-v0.7p. No effect on a
  bundle without a learned block (dyn-v0.1). With false the planner gets no extra parameter.
* `driving_reference` (default false, S4) and `lap_gate_ratio` (0.85): passed to the lap counter this
  launch starts (not to one already in the graph). The peak frame of lap 1 becomes a second
  reference and the counter's distance gate is the ratio times the lap length (alias guard);
  events say which reference scored. With false nothing extra is passed. Not part of any profile.
* `dynamics_mlp_float32` (default false, owner decision 30): the planner evaluates the learned MLP
  in single precision, about 1e-7 relative from the double pass, checked against the bundle's
  golden rollouts at load. Combines with `dynamics_fast_mlp`. The next fallback when the solve
  time on the Jetson is over budget. No effect on a bundle without a learned block. With false the
  planner gets no extra parameter and runs bit for bit as before. Not part of any profile.

Prototype profile (owner decision 29, default none):

* `profile:=prototype` turns every learned part on together as development runs: `backend:=net`
  with trackperc-v2, dyn-v0.7p with `dynamics_fast_mlp:=true` (`dynamics_adaptation` stays false),
  `lap_mode:=v1` with `dv1_variant:=K6`, `lap_source:=camera` and the session 2 lap settings
  (`lap_rate_hz` 30, ratio 0.75 / 1.5, `v_lap1_mps` 0.5, `v_max_mps` 1.3). It only changes the
  DEFAULTS of those arguments (PROFILES below); any argument given on the command line wins.
  `profile:=none` (default) changes nothing. `rt_sched` is not part of it.

Options that are measurements, not part of the command path:

* `lap_memory` (default false) adds racer_laps' lap_memory_node (`lap_source:=start_scan|map_pose|
  camera`, `lap_mode:=v0|v1|observe`) and sets the planner's `use_lap_guidance`. With
  `lap_source:=camera` D takes its laps from the camera lap counter (`/lap/event`, `/lap/status`,
  design 13 section 21) and says when it does not believe one; D9-P0: it logs READY once the
  counter has a reference, 0 laps and a similarity of at least 0.90. With `lap_mode:=observe`
  the node publishes `/pnp/lap/event` and `/pnp/lap/status` and no guidance, and the planner is
  launched exactly as with `lap_memory:=false` (fixed cap `v_max_mps`); `v_lap1_mps` is ignored
  and the launch log says so.
* `lap_counter` (default true) starts racer_perception's camera lap counter unless a node named
  `lap_counter_node` is already in the graph, and calls `/lap_counter/reset` once so the
  reference frame is this run's start line. If the package or OpenCV is missing (the node exits
  with code 3) the launch logs one clear error and the driver keeps running. `/lap/event` and
  `/lap/status` are in the stack recorder's regex.
* `lap_shadow:=camera|start_scan` (default empty: none) starts a SECOND lap node,
  `lap_memory_shadow_node`, always `lap_mode:=observe`, publishing `/pnp/lap_shadow/event` and
  `/pnp/lap_shadow/status` (no guidance publisher, so it cannot reach the planner) and writing to
  its own `shadow` output folder. A measurement: its exit is logged and does not stop the launch.
  One observe run therefore gives the lap gate's laps and the camera's.
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

import importlib.util
import math
import re
import sys
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
from launch.event_handlers import OnProcessExit, OnProcessIO
from launch.events import Shutdown
from launch.launch_context import LaunchContext
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, TextSubstitution
from launch_ros.actions import Node

_RT_SCHED_FILE = Path(__file__).resolve().with_name("rt_sched.py")


# profile:=prototype (decision 29): the defaults of the arguments below, nothing else. D v1 K6 replaces K3s (decision 36)
# by the coordinator's rule of decision 37b (it beat K3s on speed at 0 TTC warnings on the dev runs,
# docs/notes/pnp-dv1-k4-dev-2026-11.md); it runs capped (v_max_mps) with the owner on the kill switch.
PROFILE_NONE = "none"
PROFILES: dict[str, dict[str, str]] = {
    PROFILE_NONE: {},
    "prototype": {
        "backend": "net",
        "perception_bundle": "/workspace/data/pnp/models/trackperc-v2-2026-11-14",
        "dynamics_bundle": "/workspace/data/pnp/models/dyn-v0.7p-2026-11-24",
        "dynamics_fast_mlp": "true",
        "dynamics_adaptation": "false",
        "lap_memory": "true",
        "lap_mode": "v1",
        "dv1_variant": "K6",
        "lap_source": "camera",
        "lap_counter": "true",
        "lap_rate_hz": "30.0",
        "lap_length_ratio_min": "0.75",
        "lap_length_ratio_max": "1.5",
        "v_lap1_mps": "0.5",
        "v_max_mps": "1.3",
    },
}


class _ProfileDefault(TextSubstitution):
    """An argument default that the chosen profile may replace.

    `text` is the plain default (profile none), so the declared default reads as before. An
    argument given on the command line is already in the launch context when its
    DeclareLaunchArgument runs and this default is never evaluated, so it wins over the profile.
    """

    def __init__(self, name: str, default: str):
        super().__init__(text=default)
        self._name = name

    def perform(self, context: LaunchContext) -> str:
        profile = context.launch_configurations.get("profile", PROFILE_NONE)
        return PROFILES.get(profile, {}).get(self._name, self.text)


def _load_rt_sched():
    # rt_sched.py (brief 52), loaded by path: launch files are not a package
    name = "racer_bringup_rt_sched"
    if name not in sys.modules:
        spec = importlib.util.spec_from_file_location(name, _RT_SCHED_FILE)
        module = importlib.util.module_from_spec(spec)
        sys.modules[name] = module
        spec.loader.exec_module(module)
    return sys.modules[name]


BACKENDS = ("net", "geometric")
LAP_SOURCES = ("start_scan", "map_pose", "camera", "truth")
#: `truth` is a simulator source: lap_memory_parameters accepts it (the sim runner calls it) and
#: the launch refuses it.
LAUNCH_LAP_SOURCES = ("start_scan", "map_pose", "camera")
LAP_SHADOW_SOURCES = ("camera", "start_scan")
LAP_SHADOW_NODE_NAME = "lap_memory_shadow_node"
LAP_SHADOW_PREFIX = "/pnp/lap_shadow"
#: lap_memory_node's default `output_dir`; the shadow node writes to a `shadow` folder below it.
LAP_OUTPUT_DIR = "/workspace/data/pnp/laps"
TRUTH_REFUSAL = (
    "pnp.launch.py: lap_source:=truth is a simulator source (ground-truth line crossings) and "
    "the car has no truth: use start_scan or camera"
)
LAP_MODES = ("v0", "v1", "observe")
RECORDER_NODE_SUBSTRING = "rosbag2_recorder"
RECORDER_WAIT_S = 5.0
RECORDER_REFUSAL = "refusing to drive: no rosbag recorder in the graph (every run is logged)"
LAP_COUNTER_NODE = "lap_counter_node"
LAP_COUNTER_WAIT_S = 5.0
LAP_COUNTER_PROBE_S = 1.0
# require_lap_counter with the counter started here: the planner is not started until the counter
# is in the graph; the wait is bounded and ends in a refusal (the counter takes ~4 s on the Jetson).
LAP_COUNTER_GATE_S = 30.0
LAP_COUNTER_GATE_PERIOD_S = 1.0
LAP_COUNTER_GATE_PROBE_S = 0.5
LAP_COUNTER_REFUSAL = (
    "refusing to drive: require_lap_counter is true and no lap_counter_node is in the graph"
)
LAP_COUNTER_RESET_TIMEOUT_S = 30
#: The last log line of `racer_laps`' lap_source_ready_wait on a refusal is this and the reason (a
#: test pins it to the waiter's own constant).
WAITER_LAST_LINE_PREFIX = "lap source not ready: "
CAMERA_REFUSAL = "refusing to drive: the camera lap source is not ready:"
LAP_COUNTER_OPENCV_EXIT = 3
DESKEW_NODE_NAME = "scan_deskew_node"
DESKEW_TOPIC = "/pnp/scan_deskewed"
DESKEW_REMAPPING = ("/scan", DESKEW_TOPIC)
# What M-DSK step A chose on the train role (docs/notes/scan-deskew-2026-11.md). None until then:
# the launch refuses deskew:=true, so a node never starts on a guess.
DESKEW_STAMP_IS: str | None = "sweep_end"
DESKEW_BEAM_ORDER: str | None = "index_descending"
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


ADAPTATION_PARAM = "dynamics_adaptation"


def adaptation_parameters(enabled: bool) -> dict:
    """Planner parameters of the online speed adaptation: none when off, so a launch without it
    starts the planner exactly as before."""
    return {ADAPTATION_PARAM: True} if enabled else {}


FAST_MLP_PARAM = "dynamics_fast_mlp"


def fast_mlp_parameters(enabled: bool) -> dict:
    """Planner parameter of the fast MLP mode: none when off, so a launch without it starts the
    planner exactly as before."""
    return {FAST_MLP_PARAM: True} if enabled else {}


MLP_FLOAT32_PARAM = "dynamics_mlp_float32"


def mlp_float32_parameters(enabled: bool) -> dict:
    """Planner parameter of the float32 MLP mode: none when off, so a launch without it starts the
    planner exactly as before."""
    return {MLP_FLOAT32_PARAM: True} if enabled else {}


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
    planner_prefix: list | None = None,
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
        prefix=planner_prefix,  # None (rt_sched off) leaves the node exactly as it was
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
    length_ratio: tuple[float, float] = (0.0, 0.0),
) -> dict:
    """Parameters of racer_laps' lap_memory_node. The cap starts at `v_lap1_mps` (default
    `v_max_mps`) and can rise to, never above, the lower of `v_max_mps` and the dynamics
    bundle's validity range. Refuses `v_lap1_mps` above `v_max_mps` and `lap_mode:=v1` with the
    map-pose source. `length_ratio` (min, max), owner decision 21: the camera gate's length check
    as a ratio of the reference; (0, 0) is the registered band and adds no parameter."""
    ratio = _length_ratio_params(length_ratio, lap_source)
    if lap_source not in LAP_SOURCES:
        raise RuntimeError(f"pnp.launch.py: lap_source must be one of {list(LAP_SOURCES)}")
    if lap_mode not in LAP_MODES:
        raise RuntimeError(f"pnp.launch.py: lap_mode must be one of {list(LAP_MODES)}")
    if lap_mode == "v1" and lap_source == "map_pose":
        raise RuntimeError(
            "pnp.launch.py: lap_mode:=v1 needs lap_source:=start_scan or camera (D v1 is indexed "
            "by station)"
        )
    if lap_mode == "observe":
        # The gate runs, events and status are published, no guidance: the planner keeps its
        # fixed cap `v_max_mps`. `v_lap1_mps` has no meaning here and is ignored.
        ceiling = min(float(v_max_mps), validity_v_max_mps(dynamics_bundle))
        return {
            "v_cap_ceiling_mps": ceiling,
            "v_cap_start_mps": ceiling,
            "lap_mode": lap_mode,
            "lap_source": lap_source,
            **ratio,
        }
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
        **ratio,
    }


#: D v1 variants of brief 53 (owner decision 29: runnable on the car behind a switch, default
#: off; unregistered development runs). `registered` is D v1 as brief 26 built it. K1 to K3 are the
#: M-D1b candidates (cap wins in caution zones, station uncertainty, reset on change, the lap
#: station along the track), with the values chosen before M-D1b. Each maps to (lap node
#: parameters, planner parameters); every key left out keeps its node default.
_DV1_COMMON_LAP = {
    "zone_floor_mode": "cap_wins",
    "station_uncertainty": True,
    "station_uncertainty_rate": 0.02,
    "station_uncertainty_cap_m": 0.5,
    "reset_on_change": True,
    "reset_length_share": 0.08,
}
#: K3s values, chosen on development cases only (docs/notes/pnp-dv1-k3s-dev-2026-11.md): the
#: planner's gate aware cost weight (default 3.0) and TTC margin (default 1.1, times the gate's
#: warning time), and how many laps a TTC warning zone holds (100: the rest of a run).
K3S_W_GATE = 9.0
K3S_GATE_TTC_MARGIN = 1.3
K3S_TTC_ZONE_LAPS = 100
#: K4 values, chosen on development cases only (docs/notes/pnp-dv1-k4-dev-2026-11.md): K3s's lap
#: node parts (TTC warning zones, bound hold) with the planner's gate barrier in place of K3s's
#: heavier soft gate cost (w_gate and gate_ttc_margin stay at the planner defaults).
K4_W_GATE_BARRIER = 10.0
K4_GATE_BARRIER_MARGIN = 1.0
K4_GATE_BARRIER_STEPS = 6
#: K6 (docs/notes/pnp-dv1-k4-dev-2026-11.md): K5 with the barrier on the first 4 plan steps (0.2 s).
K6_GATE_BARRIER_STEPS = 4
DV1_VARIANTS: dict[str, tuple[dict, dict]] = {
    "registered": ({}, {}),
    "K1": (
        dict(_DV1_COMMON_LAP),
        {"lap_station_mode": "track", "lap_reward_mode": "keep_progress"},
    ),
    "K2": (
        {**_DV1_COMMON_LAP, "value_profile_mode": "update"},
        {"lap_station_mode": "track", "lap_reward_mode": "replace"},
    ),
    "K3": (
        {**_DV1_COMMON_LAP, "value_profile_mode": "freeze_after_first_clean_lap"},
        {"lap_station_mode": "track", "lap_reward_mode": "replace"},
    ),
    # K3s (development fix candidate after B11, unregistered): K3 with a heavier gate aware cost
    # and a wider TTC margin (the value term drove the car into the safety gate's TTC warning
    # band in corners), a caution zone held for the run where a TTC warning still happens (the
    # bound stops rising there), and the station bound of the lap after a doubtful lap held at
    # that lap's.
    "K3s": (
        {
            **_DV1_COMMON_LAP,
            "value_profile_mode": "freeze_after_first_clean_lap",
            "bound_hold_after_doubtful": True,
            "ttc_warning_zone_laps": K3S_TTC_ZONE_LAPS,
        },
        {
            "lap_station_mode": "track",
            "lap_reward_mode": "replace",
            "w_gate": K3S_W_GATE,
            "gate_ttc_margin": K3S_GATE_TTC_MARGIN,
        },
    ),
    # K4 (development candidate after K3s, unregistered): K3s's lap node parts, and in place of
    # its heavier soft gate cost a barrier that charges only the plan steps (of the first 0.3 s)
    # whose predicted time to collision is inside the gate's warning band.
    "K4": (
        {
            **_DV1_COMMON_LAP,
            "value_profile_mode": "freeze_after_first_clean_lap",
            "bound_hold_after_doubtful": True,
            "ttc_warning_zone_laps": K3S_TTC_ZONE_LAPS,
        },
        {
            "lap_station_mode": "track",
            "lap_reward_mode": "replace",
            "w_gate_barrier": K4_W_GATE_BARRIER,
            "gate_barrier_margin": K4_GATE_BARRIER_MARGIN,
            "gate_barrier_steps": K4_GATE_BARRIER_STEPS,
        },
    ),
    # K5 (development candidate after K4, unregistered): K4 with the barrier's later steps judged
    # from the predicted pose of each step (the commanded arc in the planner's distance field)
    # instead of the current pose's scan table less the path travelled.
    "K5": (
        {
            **_DV1_COMMON_LAP,
            "value_profile_mode": "freeze_after_first_clean_lap",
            "bound_hold_after_doubtful": True,
            "ttc_warning_zone_laps": K3S_TTC_ZONE_LAPS,
        },
        {
            "lap_station_mode": "track",
            "lap_reward_mode": "replace",
            "w_gate_barrier": K4_W_GATE_BARRIER,
            "gate_barrier_margin": K4_GATE_BARRIER_MARGIN,
            "gate_barrier_steps": K4_GATE_BARRIER_STEPS,
            "gate_barrier_predicted": True,
        },
    ),
    # K6 (development candidate after K5, unregistered): K5 with the barrier on the first 4 plan
    # steps only (later steps are replanned and over-charged speed on wide lanes).
    "K6": (
        {
            **_DV1_COMMON_LAP,
            "value_profile_mode": "freeze_after_first_clean_lap",
            "bound_hold_after_doubtful": True,
            "ttc_warning_zone_laps": K3S_TTC_ZONE_LAPS,
        },
        {
            "lap_station_mode": "track",
            "lap_reward_mode": "replace",
            "w_gate_barrier": K4_W_GATE_BARRIER,
            "gate_barrier_margin": K4_GATE_BARRIER_MARGIN,
            "gate_barrier_steps": K6_GATE_BARRIER_STEPS,
            "gate_barrier_predicted": True,
        },
    ),
}


def dv1_variant_parameters(variant: str, lap_mode: str) -> tuple[dict, dict]:
    """(lap node, planner) parameters of a D v1 variant. Nothing unless `lap_mode` is v1, so the
    D v0 and observe paths are unchanged whatever `dv1_variant` says."""
    if variant not in DV1_VARIANTS:
        raise RuntimeError(f"pnp.launch.py: dv1_variant must be one of {list(DV1_VARIANTS)}")
    if lap_mode != "v1":
        return {}, {}
    lap, planner = DV1_VARIANTS[variant]
    return dict(lap), dict(planner)


def _length_ratio_params(length_ratio: tuple[float, float], lap_source: str) -> dict:
    """The lap node's `length_ratio_min/max`, or nothing for the registered band (0, 0). Refuses
    a ratio that would accept a double lap or reject the reference itself, and a ratio with a
    source other than the camera or the sim truth (the scan gate has its own band)."""
    lo, hi = float(length_ratio[0]), float(length_ratio[1])
    if (lo, hi) == (0.0, 0.0):
        return {}
    if not (0.0 < lo < 1.0 < hi < 2.0):
        raise RuntimeError(
            f"pnp.launch.py: lap_length_ratio_min/max must be 0 < min < 1 < max < 2, got {lo}, {hi}"
        )
    if lap_source not in ("camera", "truth"):
        raise RuntimeError(
            "pnp.launch.py: lap_length_ratio_min/max apply to lap_source:=camera or truth only"
        )
    return {"length_ratio_min": lo, "length_ratio_max": hi}


def lap_shadow_parameters(
    dynamics_bundle: str, v_max_mps: float, lap_shadow: str, laser_yaw_from_vehicle_params: bool
) -> dict:
    """Parameters of the shadow lap node: `observe` always (events and status, no guidance), its
    own topics under `/pnp/lap_shadow` and its own output folder. It has no guidance publisher, so
    it cannot reach the planner."""
    params = lap_memory_parameters(dynamics_bundle, v_max_mps, None, "observe", lap_shadow)
    params["output_prefix"] = LAP_SHADOW_PREFIX
    params["output_dir"] = f"{LAP_OUTPUT_DIR}/shadow"
    params["laser_yaw_from_vehicle_params"] = laser_yaw_from_vehicle_params
    return params


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


DRIVING_REFERENCE_ARG = "driving_reference"
LAP_GATE_RATIO_ARG = "lap_gate_ratio"


def lap_counter_actions(
    require: bool,
    rate_hz: float = 15.0,
    driving_reference: bool = False,
    lap_gate_ratio: float = 0.85,
) -> list:
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
        PythonLaunchDescriptionSource(str(Path(share) / "launch" / "lap_counter.launch.py")),
        # S4 (default off): nothing extra is passed unless the switch is on, so the counter this
        # launch starts is the registered one.
        launch_arguments=[
            ("rate_hz", repr(float(rate_hz))),
            *(
                [
                    ("driving_reference", "true"),
                    ("lap_gate_ratio", repr(float(lap_gate_ratio))),
                ]
                if driving_reference
                else []
            ),
        ],
    )

    def on_exit(event, context):
        if "lap_counter_node" not in getattr(event, "process_name", ""):
            return []
        code = getattr(event, "returncode", None)
        if code == 0:  # a clean exit (ctrl-c, an orderly shutdown) is not an error
            return [LogInfo(msg=f"[pnp.launch.py] lap_counter_node exited cleanly (code {code})")]
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


def camera_status_topic(lap_memory: bool, lap_source: str, lap_shadow: str) -> str | None:
    """The status topic of the lap node of this launch that has the camera source (brief 51c): the
    primary's `/pnp/lap/status` with `lap_source:=camera`, else the shadow's with
    `lap_shadow:=camera`, else None (no camera lap node: no start gate). Without lap memory there
    is no lap node at all."""
    if not lap_memory:
        return None
    if lap_source == "camera":
        return "/pnp/lap/status"
    if lap_shadow == "camera":
        return f"{LAP_SHADOW_PREFIX}/status"
    return None


def lap_source_ready_waiter(status_topic: str, min_reset_id: int | None = None) -> ExecuteProcess:
    """The start gate's node: exits 0 when the lap node on `status_topic` has said the source is
    ready on two consecutive statuses, 3 when it did not within 30 s (racer_laps/start_ready.py).
    With `min_reset_id` it ignores every status of an older counter reset."""
    reset = ["-p", f"min_reset_id:={min_reset_id}"] if min_reset_id else []
    return ExecuteProcess(
        cmd=[
            "ros2",
            "run",
            "racer_laps",
            "lap_source_ready_wait",
            "--ros-args",
            "-p",
            f"status_topic:={status_topic}",
            *reset,
        ],
        output="screen",
    )


class CameraStartChain:
    """The planner of a launch with a camera lap node starts only at the end of this chain
    (brief 51c, D10-P0): the counter is in the graph (the caller's gate), the reset call exited 0,
    `lap_source_ready_wait` exited 0, then the planner. Any other exit is one ERROR line starting
    "refusing to drive: the camera lap source is not ready:" and a shutdown, the planner never
    started. A method per step, so that each is a plain function of an exit code."""

    def __init__(
        self,
        planner: Node,
        status_topic: str,
        reset: ExecuteProcess | None = None,
        waiter: ExecuteProcess | None = None,
    ) -> None:
        self.planner = planner
        self.status_topic = status_topic
        self.reset = reset
        self._own_waiter = waiter is None
        self.waiter = waiter if waiter is not None else lap_source_ready_waiter(status_topic)
        self.reason: str | None = None
        self.reset_id: int | None = None

    def capture_line(self, text: str) -> None:
        """Remember the waiter's last refusal line (its reason is the lap node's own)."""
        for line in text.splitlines():
            index = line.rfind(WAITER_LAST_LINE_PREFIX)
            if index >= 0:
                self.reason = line[index + len(WAITER_LAST_LINE_PREFIX) :].strip()

    def capture_reset_text(self, text: str) -> None:
        """The reset call's answer carries `reset_id=N`: the counter's acknowledgement."""
        found = re.findall(r"reset_id=(\d+)", text)
        if found:
            self.reset_id = int(found[-1])

    def _capture_reset(self, event) -> None:
        self.capture_reset_text(bytes(getattr(event, "text", b"")).decode(errors="replace"))

    def _capture(self, event) -> None:
        self.capture_line(bytes(getattr(event, "text", b"")).decode(errors="replace"))

    def refuse(self, reason: str) -> list:
        return [
            LogInfo(msg=f"[pnp.launch.py] ERROR: {CAMERA_REFUSAL} {reason}"),
            EmitEvent(event=Shutdown(reason=f"{CAMERA_REFUSAL} {reason}")),
        ]

    def after_counter(self) -> list:
        """The counter is in the graph: the reset call, or the waiter when this launch makes none."""
        if self.reset is None:
            return self.start_waiter()
        return [
            LogInfo(
                msg="[pnp.launch.py] lap_counter_node is in the graph: resetting its reference"
            ),
            RegisterEventHandler(
                OnProcessIO(
                    target_action=self.reset,
                    on_stdout=lambda event: self._capture_reset(event),
                    on_stderr=lambda event: self._capture_reset(event),
                )
            ),
            RegisterEventHandler(
                OnProcessExit(
                    target_action=self.reset,
                    on_exit=lambda event, context: self.reset_exit(event.returncode),
                )
            ),
            self.reset,
        ]

    def reset_exit(self, code: int | None) -> list:
        if code != 0:
            return self.refuse(f"the counter reset failed or timed out (exit code {code})")
        notes: list = []
        if self.reset_id is not None and self._own_waiter:
            # the counter acknowledged reset N: only statuses of reset N or later count
            self.waiter = lap_source_ready_waiter(self.status_topic, self.reset_id)
        elif self.reset_id is None:
            notes.append(
                LogInfo(
                    msg="[pnp.launch.py] WARN: the reset call named no reset_id (an older "
                    "lap_counter_node?): statuses are not tied to this reset"
                )
            )
        return [*notes, *self.start_waiter()]

    def start_waiter(self) -> list:
        return [
            LogInfo(
                msg=f"[pnp.launch.py] waiting for the lap node on {self.status_topic} to say the "
                "source is ready, the planner is not started yet"
            ),
            RegisterEventHandler(
                OnProcessExit(
                    target_action=self.waiter,
                    on_exit=lambda event, context: self.waiter_exit(event.returncode),
                )
            ),
            RegisterEventHandler(
                OnProcessIO(
                    target_action=self.waiter,
                    on_stdout=lambda event: self._capture(event),
                    on_stderr=lambda event: self._capture(event),
                )
            ),
            self.waiter,
        ]

    def waiter_exit(self, code: int | None) -> list:
        if code != 0:
            return self.refuse(
                self.reason or f"lap_source_ready_wait ended without ready (exit code {code})"
            )
        return [
            LogInfo(msg="[pnp.launch.py] the lap source is ready: starting the planner"),
            self.planner,
        ]


def require_counter_gate(
    planner: Node,
    wait_s: float = LAP_COUNTER_GATE_S,
    clock=time.monotonic,
    chain: CameraStartChain | None = None,
):
    """The action that starts `planner` only once `lap_counter_node` is in the graph. With a
    `chain` (a camera lap node, brief 51c) it hands over to `chain.after_counter()` at that point
    and the planner waits for the rest of the chain.

    Polls (a short probe, then a one second timer) for up to `wait_s`; if the counter never
    appears it logs the refusal and shuts the launch down. The planner, the only publisher of
    /drive_raw, is never started before the requirement is met."""
    state: dict[str, float] = {}

    def gate(context: LaunchContext):
        started = state.setdefault("t0", clock())
        if lap_counter_in_graph(LAP_COUNTER_GATE_PROBE_S):
            if chain is not None:
                return chain.after_counter()
            return [
                LogInfo(
                    msg="[pnp.launch.py] lap_counter_node is in the graph: starting the planner"
                ),
                planner,
            ]
        if clock() - started >= wait_s:
            return [
                LogInfo(msg=f"[pnp.launch.py] ERROR: {LAP_COUNTER_REFUSAL} within {wait_s:g} s"),
                EmitEvent(event=Shutdown(reason=LAP_COUNTER_REFUSAL)),
            ]
        return [
            TimerAction(period=LAP_COUNTER_GATE_PERIOD_S, actions=[OpaqueFunction(function=gate)])
        ]

    action = OpaqueFunction(function=gate)
    action.chain = chain  # type: ignore[attr-defined]  # for the tests: the held-back planner
    return action


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
    adaptation = LaunchConfiguration(ADAPTATION_PARAM, default="false").perform(context).strip()
    planner_params.update(adaptation_parameters(_bool(adaptation, ADAPTATION_PARAM)))
    fast_mlp = LaunchConfiguration(FAST_MLP_PARAM, default="false").perform(context).strip()
    planner_params.update(fast_mlp_parameters(_bool(fast_mlp, FAST_MLP_PARAM)))
    mlp_float32 = LaunchConfiguration(MLP_FLOAT32_PARAM, default="false").perform(context).strip()
    planner_params.update(mlp_float32_parameters(_bool(mlp_float32, MLP_FLOAT32_PARAM)))
    lap_memory = _bool(arg("lap_memory"), "lap_memory")
    deskew_on = _bool(arg("deskew"), "deskew")
    scan_remapping = DESKEW_REMAPPING if deskew_on else None
    deskew_node = deskew_node_for(arg("params_file")) if deskew_on else None
    lap_counter = _bool(arg("lap_counter"), "lap_counter")
    require_counter = _bool(arg("require_lap_counter"), "require_lap_counter")
    driving_reference = _bool(
        LaunchConfiguration(DRIVING_REFERENCE_ARG, default="false").perform(context).strip(),
        DRIVING_REFERENCE_ARG,
    )
    lap_gate_ratio = float(
        LaunchConfiguration(LAP_GATE_RATIO_ARG, default="0.85").perform(context).strip()
    )
    if not 0.0 < lap_gate_ratio <= 1.0:
        raise RuntimeError(f"pnp.launch.py: lap_gate_ratio must be in (0, 1], got {lap_gate_ratio}")
    lap_rate_hz = float(arg("lap_rate_hz"))
    if not math.isfinite(lap_rate_hz):
        raise RuntimeError(f"pnp.launch.py: lap_rate_hz must be finite, got {lap_rate_hz}")
    lap_source = arg("lap_source")
    lap_shadow = LaunchConfiguration("lap_shadow", default="").perform(context).strip()
    if lap_source == "truth":
        raise RuntimeError(TRUTH_REFUSAL)
    if lap_shadow:
        if lap_shadow not in LAP_SHADOW_SOURCES:
            raise RuntimeError(
                f"pnp.launch.py: lap_shadow must be one of {list(LAP_SHADOW_SOURCES)} or empty"
            )
        if lap_shadow == lap_source:
            raise RuntimeError(
                f"pnp.launch.py: lap_shadow:={lap_shadow} equals lap_source: the shadow lap node "
                "is there to read the other source"
            )
        if not lap_memory:
            raise RuntimeError("pnp.launch.py: lap_shadow needs lap_memory:=true")
    # D on the camera source depends on the counter being there: the same requirement as
    # require_lap_counter (refusal wording, and the planner waits for the counter)
    camera_topic = camera_status_topic(lap_memory, lap_source, lap_shadow)
    if camera_topic is not None:
        # brief 51c: a camera lap node (primary or shadow) needs the counter, and the planner
        # waits for the counter, the reset and the lap node's ready statuses (CameraStartChain)
        require_counter = True
    v_lap1_text = arg("v_lap1_mps")
    v_lap1 = float(v_lap1_text) if v_lap1_text else None
    lap_params = None
    log: list[str] = []
    profile = LaunchConfiguration("profile", default=PROFILE_NONE).perform(context).strip()
    if profile != PROFILE_NONE:
        log.append(
            f"profile:={profile}: development defaults {PROFILES[profile]} (arguments given "
            "on the command line win). D v1 K3 was unsafe in sim: owner on the kill switch."
        )
    if lap_memory:
        lap_params = lap_memory_parameters(
            arg("dynamics_bundle"),
            v_max,
            v_lap1,
            arg("lap_mode"),
            lap_source,
            (float(arg("lap_length_ratio_min")), float(arg("lap_length_ratio_max"))),
        )
        variant = LaunchConfiguration("dv1_variant", default="registered").perform(context)
        dv1_lap, dv1_planner = dv1_variant_parameters(variant.strip(), arg("lap_mode"))
        lap_params.update(dv1_lap)
        planner_params.update(dv1_planner)
        if dv1_lap or dv1_planner:
            log.append(f"D v1 variant {variant}: lap {dv1_lap} planner {dv1_planner}")
        if arg("lap_mode") == "observe":
            # the planner is launched exactly as with lap_memory:=false: fixed cap, no guidance
            # expected, no no-guidance fallback
            log.append(
                "lap_mode:=observe: the lap node publishes events and status only, the planner "
                f"runs at its fixed cap v_max_mps {v_max} m/s; v_lap1_mps is ignored"
            )
        else:
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
    rt = _load_rt_sched()
    rt_mode = rt.check_mode(LaunchConfiguration("rt_sched", default="off").perform(context))
    perception, planner = nodes_for(
        perception_params,
        planner_params,
        arg("params_file"),
        scan_remapping,
        rt.prefix_for("mppi_planner_node") if rt_mode == "on" else None,
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
    # require_lap_counter with the counter started here: the planner waits for it (gate below)
    gated = (require_counter and lap_counter) or camera_topic is not None
    driver = [perception] if gated else [perception, planner]
    actions = [*driver, *stop_all, *[LogInfo(msg=f"[pnp.launch.py] {m}") for m in log]]
    if rt_mode == "on":
        # brief 52, rt_sched:=on only (nothing is added with off): the planner's thread check and
        # the refusal line; the planner's own exit already shuts the launch down (stop_all). First
        # in the list: the start handler must exist before the planner's start event.
        actions = [
            *rt.all_actions(planner, "mppi_planner_node", "mppi_planner_node", False),
            *actions,
        ]
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
    if lap_shadow:
        shadow_laps = Node(
            package="racer_laps",
            executable="lap_memory_node",
            name=LAP_SHADOW_NODE_NAME,
            output="screen",
            parameters=[
                *([arg("params_file")] if arg("params_file") else []),
                lap_shadow_parameters(
                    arg("dynamics_bundle"),
                    v_max,
                    lap_shadow,
                    perception_params["laser_yaw_from_vehicle_params"],
                ),
            ],
            remappings=[scan_remapping] if scan_remapping else None,
        )
        actions += [
            shadow_laps,
            RegisterEventHandler(
                OnProcessExit(
                    target_action=shadow_laps,
                    on_exit=_log_exit(
                        "shadow lap node",
                        "a measurement only, nothing reads it: the driver keeps running",
                    ),
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
            actions += lap_counter_actions(
                require_counter, lap_rate_hz, driving_reference, lap_gate_ratio
            )
        if camera_topic is None:
            actions.append(lap_counter_reset_action())
    chain = None
    if camera_topic is not None:
        # the reset call is the chain's second step, not a call at launch
        chain = CameraStartChain(
            planner, camera_topic, reset=lap_counter_reset_action() if lap_counter else None
        )
    if gated:
        actions.append(require_counter_gate(planner, chain=chain))
    return actions


def generate_launch_description() -> LaunchDescription:
    return LaunchDescription(
        [
            DeclareLaunchArgument(
                "profile",
                default_value=PROFILE_NONE,
                choices=list(PROFILES),
                description=(
                    "prototype: the defaults of every learned part together (net perception with "
                    "trackperc-v2, dyn-v0.7p with dynamics_fast_mlp, lap_mode v1 K6 on the camera "
                    "source, the session 2 lap settings, v_max_mps 1.3); development runs "
                    "(owner decision 29). An argument given explicitly overrides it. none "
                    "(default): nothing changes."
                ),
            ),
            DeclareLaunchArgument(
                "perception_bundle",
                default_value=_ProfileDefault("perception_bundle", ""),
                description="track_perception bundle directory. Required with backend:=net.",
            ),
            DeclareLaunchArgument(
                "dynamics_bundle",
                default_value=_ProfileDefault("dynamics_bundle", ""),
                description="dynamics bundle directory (required).",
            ),
            DeclareLaunchArgument(
                "backend",
                default_value=_ProfileDefault("backend", "geometric"),
                choices=list(BACKENDS),
                description=(
                    "Track perception backend: geometric (default: the net failed the real-bag "
                    "gate, 2026-11-11) or net (needs perception_bundle)."
                ),
            ),
            DeclareLaunchArgument(
                "v_max_mps",
                default_value=_ProfileDefault("v_max_mps", "0.5"),
                description=(
                    "The absolute speed cap in m/s passed to the planner: the ceiling. With "
                    "lap_memory the lap cap rises towards it."
                ),
            ),
            DeclareLaunchArgument(
                "v_lap1_mps",
                default_value=_ProfileDefault("v_lap1_mps", ""),
                description=(
                    "Cap of lap 1 in m/s: the lap memory's start cap and the planner's "
                    "v_no_guidance_mps. Empty: equal to v_max_mps. Refused above v_max_mps "
                    "with lap_memory:=true."
                ),
            ),
            DeclareLaunchArgument(
                "lap_length_ratio_min",
                default_value=_ProfileDefault("lap_length_ratio_min", "0.0"),
                description=(
                    "Owner decision 21: with lap_source:=camera a lap counts if its wheel length "
                    "is at least this times the reference. 0 with lap_length_ratio_max 0: the "
                    "registered band (0.5 m or 8 percent). The car run of 2026-11-22 used 0.75."
                ),
            ),
            DeclareLaunchArgument(
                "lap_length_ratio_max",
                default_value=_ProfileDefault("lap_length_ratio_max", "0.0"),
                description=(
                    "Owner decision 21: at most this times the reference (below 2, so a missed "
                    "crossing, a double lap, stays doubtful). 0: the registered band."
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
                default_value=_ProfileDefault("lap_memory", "false"),
                choices=["true", "false"],
                description=(
                    "Also start racer_laps' lap_memory_node and let the planner use its speed "
                    "cap. The cap ceiling is the lower of v_max_mps and the dynamics bundle's "
                    "validity v_max_mps. Off for a first run."
                ),
            ),
            DeclareLaunchArgument(
                "lap_mode",
                default_value=_ProfileDefault("lap_mode", "v0"),
                choices=list(LAP_MODES),
                description=(
                    "D v0 (cap schedule), v1 (station set; start_scan or camera) or observe (the lap "
                    "node publishes events and status and no guidance; the planner runs at its "
                    "fixed cap v_max_mps and v_lap1_mps is ignored)."
                ),
            ),
            DeclareLaunchArgument(
                "dv1_variant",
                default_value=_ProfileDefault("dv1_variant", "registered"),
                choices=list(DV1_VARIANTS),
                description=(
                    "lap_mode:=v1 only: registered (D v1 of brief 26), a brief 53 candidate K1, "
                    "K2, K3, K3s (K3 with a heavier gate cost and margin, TTC warning zones "
                    "and the bound held after a doubtful lap, a development fix after B11), or "
                    "K4 (K3s's lap node parts with a gate barrier in the planner instead of the "
                    "heavier gate cost), K5 (K4 with the barrier judged from the predicted "
                    "pose of each plan step), or K6 (K5 on the first 4 plan steps). "
                    "Ignored for v0 and observe. Development runs, not a registered result "
                    "(owner decision 29)."
                ),
            ),
            DeclareLaunchArgument(
                "lap_source",
                default_value=_ProfileDefault("lap_source", "start_scan"),
                choices=list(LAP_SOURCES),
                description=(
                    "How a lap closes: start_scan (start-scan gate plus wheel odometry, no map "
                    "after lap 1), map_pose (TF map -> base_link, needs mapping or "
                    "localisation) or camera (the camera lap counter's events, judged by the lap "
                    "node; needs the counter). truth is a simulator source and is refused."
                ),
            ),
            DeclareLaunchArgument(
                "lap_shadow",
                default_value="",
                description=(
                    "camera or start_scan (not the lap_source): also start a second lap node, "
                    "lap_memory_shadow_node, in observe mode on /pnp/lap_shadow/*. Empty: none."
                ),
            ),
            DeclareLaunchArgument(
                "lap_counter",
                default_value=_ProfileDefault("lap_counter", "true"),
                choices=["true", "false"],
                description=(
                    "Start the camera lap counter (racer_perception) unless one is already in "
                    "the graph, and reset it once. A measurement, not part of the command path."
                ),
            ),
            DeclareLaunchArgument(
                "lap_rate_hz",
                default_value=_ProfileDefault("lap_rate_hz", "15.0"),
                description=(
                    "Frames per second the camera lap counter processes (a fixed phase, so a "
                    "camera at least this fast delivers exactly this). The car session uses 30. "
                    "Only for the counter this launch starts; one already in the graph keeps "
                    "its own."
                ),
            ),
            DeclareLaunchArgument(
                "require_lap_counter",
                default_value="false",
                choices=["true", "false"],
                description=(
                    "Refuse to drive when no lap_counter_node is in the graph: the planner starts only "
                    "once it is there, waiting up to 30 s (evaluation runs)."
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
                "rt_sched",
                default_value="off",
                choices=["off", "on"],
                description=(
                    "Brief 52. on: start mppi_planner_node behind `chrt -f <priority>` "
                    "(config/scheduling.yaml) and check every thread of it after start. The "
                    "launch REFUSES to start (no fallback to SCHED_OTHER) when the policy is not "
                    "granted, so the stack container needs --ulimit rtprio=<n>. off (default): "
                    "the planner starts exactly as before, and the launch only logs its policy."
                ),
            ),
            DeclareLaunchArgument(
                ADAPTATION_PARAM,
                default_value=_ProfileDefault("dynamics_adaptation", "false"),
                choices=["true", "false"],
                description=(
                    "PROTOTYPE: online bias and gain correction of the dynamics model's speed "
                    "channel from the wheel speed (default off: the planner is unchanged)."
                ),
            ),
            DeclareLaunchArgument(
                FAST_MLP_PARAM,
                default_value=_ProfileDefault("dynamics_fast_mlp", "false"),
                choices=["true", "false"],
                description=(
                    "true: the planner evaluates the dynamics bundle's learned MLP with a "
                    "vectorised tanh (golden rollouts checked at load), for a bundle with a "
                    "residual such as dyn-v0.7p. false (default): the planner is unchanged."
                ),
            ),
            DeclareLaunchArgument(
                DRIVING_REFERENCE_ARG,
                default_value="false",
                choices=["true", "false"],
                description=(
                    "true: the camera lap counter this launch starts adds the peak frame of lap 1 "
                    "as a second reference and gates a lap at lap_gate_ratio times the lap length "
                    "(S4 alias guard). Only for the counter started here. false (default): the "
                    "registered counter, nothing extra passed."
                ),
            ),
            DeclareLaunchArgument(
                LAP_GATE_RATIO_ARG,
                default_value="0.85",
                description="With driving_reference: the distance gate as a share of the lap length.",
            ),
            DeclareLaunchArgument(
                MLP_FLOAT32_PARAM,
                default_value="false",
                choices=["true", "false"],
                description=(
                    "true: the planner evaluates the dynamics bundle's learned MLP in single "
                    "precision (about 1e-7 relative from the double pass, golden rollouts checked "
                    "at load), for a bundle with a residual such as dyn-v0.7p. Combines with "
                    "dynamics_fast_mlp. false (default): the planner is unchanged."
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
