#!/usr/bin/env python3
"""Driver benchmark on the track family: one track, one driver, one JSON result.

Like reactive_bench_runner.py (same metrics, same real command path: driver -> /drive_raw ->
safety_node -> /drive -> bridge_node, nothing remapped) but for any of the bridge's synthetic
tracks including the family tracks (stadium, rounded_rect, and `map`, a saved SLAM map as the
world), with the car as measured and three drivers:

  gap_follow    gap_follow_node on a launch profile (default lane-2026-10-25) plus parameters
  tracker       tracker_node on a raceline made from the track's map by tools/track_session
                (the same pipeline a track session runs), following GROUND TRUTH pose
  tracker_loc   the same raceline, following the LOCALISED pose: the car's odometry (wheel
                speed, modelled steering, EKF) and AMCL on the track's map, sensor errors on
  tracker_policy the tracker in HANDOFF mode (drive_topic /tracker/command, lookahead from the
                policy directory's policy_meta.yaml) plus racer_policy's policy_node, which adds
                the bounded residual and is the only publisher of /drive_raw (--policy-dir)
  pnp           the Plug-and-Play driver, no map and no pose: racer_track_perception's
                track_perception_node and racer_planner's mppi_planner_node (--backend,
                --perception-bundle, --dynamics-bundle, --v-max-mps), with the car's LiDAR,
                scan noise and 50 Hz /odom/wheel; parameters come from pnp.launch.py.
                --lap-memory none|off|v0|v1 adds racer_laps' lap_memory_node (start-scan lap
                source): `off` runs it as an observer only (the planner ignores its guidance, so
                the run is "D off" but the system's laps are still counted), v0 and v1 let the
                planner use it. --v-max-mps is then the ceiling and --v-lap1-mps the cap of lap
                1. The result carries the system's LapEvents, the guidance cap per lap and the
                interventions per ground-truth lap by source.

The car (--car): `params` (default) gives the simulator's steering the limits of vehicle_params
(0.449 rad each way, schema 0.14.0: commands are true angles and true speeds); `gym` is the
simulator's own 0.4189 rad. The LiDAR is cast from the real mount.

Run with the workspace sourced (racer_gym_bridge, racer_safety, racer_control, racer_state,
racer_bringup, racer_tools built), e.g. in the ros-dev container:

    python3 tests/l5_reactive_bench/track_family_runner.py --track stadium \\
        --track-params-json '{"lane_width_m": 1.0, "straight_length_m": 2.0, "island_radius_m": 0.0}' \\
        --driver tracker --stop-after-laps 3 --out result.json

Exit status: 0 with a result, 3 when a node died (the JSON carries "error").
"""

from __future__ import annotations

import argparse
import json
import math
import os
import subprocess
import sys
import tempfile
import time
from itertools import pairwise
from pathlib import Path

import numpy as np
import yaml

HERE = Path(__file__).resolve().parent
REPO_ROOT = HERE.parents[1]
sys.path.insert(0, str(HERE))
sys.path.insert(0, str(REPO_ROOT / "tools" / "track_session"))

import dv1_measures
import pnp_tick_summary as pts
import reactive_bench_runner as base
from truth_lap_source import FaultPlan, TruthLapSource

sys.path.insert(0, str(REPO_ROOT / "ros_ws" / "src" / "racer_laps" / "src"))
from racer_laps.start_ready import StartGateConfig, StartReadyGate

TRACKS = (
    "lane_square",
    "lane_round",
    "lane_open_side",
    "chicane",
    "stadium",
    "rounded_rect",
    "map",
)
DRIVERS = ("gap_follow", "tracker", "tracker_loc", "tracker_policy", "pnp")
CARS = ("gym", "params")
PNP_SCAN_BEAMS = 720
#: Ground-truth yaw change over the integral of v tan(delta) / L of the gated steering command, sim
#: plant of the PnP canaries (dyn-car-2026-11-12 actuation: steer_gain 0.917 left, 1.011 right),
#: measured on the tuning stadium, two laps each way: 0.903 counter-clockwise (left turns), 1.008
#: clockwise (right turns).
SIM_YAW_GAIN_LEFT = 0.90
SIM_YAW_GAIN_RIGHT = 1.01
LOCALISATION_LAUNCH = REPO_ROOT / "ros_ws" / "src" / "racer_bringup" / "launch"
# The sensor errors the localisation canaries use (sim_autopilot.launch.py SIM_SENSOR_NOISE).
LANE_TRACKER_DEFAULTS = {
    "lookahead_min_m": 0.35,
    "lookahead_max_m": 0.7,
    "lookahead_curvature_ref_1pm": 1.25,
}
_STRING_TRACK_KEYS = {
    "map_yaml": "synthetic_map_yaml",
    "map_centreline_csv": "synthetic_map_centreline",
}
_FLOAT_TRACK_KEYS = {
    **base._BRIDGE_TRACK_KEYS,
    "straight_length_m": "synthetic_straight_length_m",
    "island_width_m": "synthetic_island_width_m",
    "island_radius_m": "synthetic_island_radius_m",
}


#: The output the bridge's lockstep gate watches in the PnP stack: one message per scan, stamped
#: with that scan (track_perception_node), so a starved perception holds sim time back.
PNP_LOCKSTEP_ACK = "racer_pnp_msgs/msg/LocalTrack@/pnp/local_track"
#: The oracle track node of measurement M-A2 (test-only code under tests/, run by path).
ORACLE_DIR = REPO_ROOT / "tests" / "l5_pnp_oracle"
ORACLE_NODE = ORACLE_DIR / "oracle_track_node.py"
LAP_NODE_COUNTING = REPO_ROOT / "tests" / "l5_pnp_laps" / "lap_node_counting.py"
#: The truth oracle's topics (brief 51): the lap node reads these instead of the camera's.
TRUTH_EVENT_TOPIC = "/sim/lap/truth_event"
TRUTH_STATUS_TOPIC = "/sim/lap/truth_status"


def write_oracle_ground_truth(synthetic, path: Path) -> None:
    """The file oracle_track_node reads: the driving-direction midline (the free lane's, bending
    round obstacles) and the occupancy grid the simulator itself ray casts."""
    from racer_gym_bridge import track_obstacles

    sys.path.insert(0, str(ORACLE_DIR))
    import oracle_core

    grid, origin, resolution = track_obstacles.ground_truth_grid(synthetic)
    oracle_core.write_ground_truth(
        path, synthetic.centreline_x, synthetic.centreline_y, grid, origin, resolution
    )


def car_parameters(car: str) -> dict:
    """Bridge parameters for the car model: `params` gives the gym the steering limits of
    vehicle_params (0.449 rad each way), `gym` leaves its own 0.4189."""
    from pipeline import steering_limits

    if car != "params":
        return {}
    left, right = steering_limits()
    return {"steering_limit_left_rad": left, "steering_limit_right_rad": right}


def plant_parameters(bundle_dir: Path) -> dict:
    """Bridge plant-realism parameters from a dynamics bundle's io.nominal: the sim car then
    actuates the way the planner's model says the real car does."""
    nominal = yaml.safe_load((bundle_dir / "manifest.yaml").read_text())["io"]["nominal"]
    return {
        "plant_steer_gain_left": float(nominal["steer_gain_left"]),
        "plant_steer_gain_right": float(nominal["steer_gain_right"]),
        "plant_speed_gain": float(nominal["speed_gain"]),
        "plant_accel_max_mps2": float(nominal["accel_max_mps2"]),
        "plant_decel_max_mps2": float(nominal["decel_max_mps2"]),
        # Absent in manifests made before the dead band was fitted: no dead band.
        "plant_speed_deadband_mps": float(nominal.get("speed_deadband_mps", 0.0)),
    }


def plant_overrides(args: argparse.Namespace) -> dict:
    """Bridge plant-realism parameters given by runner flags; they win over --plant-bundle."""
    flags = (
        (args.plant_speed_deadband, "plant_speed_deadband_mps"),
        (args.plant_steer_tau, "plant_steer_tau_s"),
        (args.plant_command_delay, "plant_command_delay_s"),
    )
    return {key: float(value) for value, key in flags if value is not None}


def bridge_track_parameters(track: str, track_params: dict, seed: int, reverse: bool) -> dict:
    unknown = set(track_params) - set(_FLOAT_TRACK_KEYS) - set(_STRING_TRACK_KEYS) - {"obstacles"}
    if unknown:
        raise ValueError(f"unknown track parameter(s) {sorted(unknown)}")
    params: dict = {"seed": seed, "synthetic_track": track, "reverse_direction": reverse}
    for key, value in track_params.items():
        if key == "obstacles":
            # Solid boxes against a wall (sim/bridge track_obstacles): the bridge draws them.
            params["synthetic_obstacles_json"] = json.dumps(value)
        elif key in _STRING_TRACK_KEYS:
            params[_STRING_TRACK_KEYS[key]] = str(value)
        else:
            params[_FLOAT_TRACK_KEYS[key]] = float(value)
    return params


def build_world(track: str, track_params: dict, work: Path):
    """The track, its map yaml (surfaces only, for AMCL and the raceline tool) and the
    centreline CSV the `map` track needs. Returns (synthetic, map_yaml, track_params with the
    generated centreline path filled in)."""
    from pipeline import raceline_from_map, write_surface_map
    from racer_gym_bridge import track_loader

    params = dict(track_params)
    if track == "map":
        map_yaml = Path(params["map_yaml"])
        if "map_centreline_csv" not in params:
            centre_csv = work / "map_centreline.csv"
            raceline_from_map(
                map_yaml, work / "unused_line.csv", centre_only=True, centre_csv=centre_csv
            )
            params["map_centreline_csv"] = str(centre_csv)
        synthetic = track_loader.build_synthetic_track(track, **params)
        return synthetic, map_yaml, params
    synthetic = track_loader.build_synthetic_track(track, **params)
    occupancy, origin = track_loader.rasterize_walls(
        synthetic.walls, synthetic.bounds(), synthetic.occupancy_resolution_m
    )
    map_yaml = write_surface_map(
        occupancy, (origin[0], origin[1]), synthetic.occupancy_resolution_m, work / "world"
    )
    return synthetic, map_yaml, params


def make_raceline(args, synthetic, map_yaml: Path, work: Path) -> tuple[Path, dict]:
    """tools/track_session's pipeline on the track's own map, in the driving direction."""
    from pipeline import raceline_from_map

    options = json.loads(args.raceline_json)
    if args.clearances:
        options["clearance_candidates"] = [float(v) for v in args.clearances.split(",")]
    if "clearance_candidates" in options:
        options["clearance_candidates"] = tuple(options["clearance_candidates"])
    seed_xy = (float(synthetic.centreline_x[0]), float(synthetic.centreline_y[0]))
    if args.track == "map":
        seed_xy = (0.0, 0.0)
    out = work / "raceline.csv"
    result = raceline_from_map(
        map_yaml,
        out,
        direction="cw" if args.reverse else "ccw",
        seed_xy=seed_xy,
        **options,
    )
    return out, result.report


#: The simulated LiDAR's sweep (bridge scan_time: scan_divider / step rate at 10 Hz). The lap node
#: stamps a scan at its stamp plus half the sweep; the simulator's scans are instantaneous.
SIM_SCAN_SWEEP_S = 0.1
#: Ground truth validity (pre-registration pnp-v0.5-laps.yaml, lap_battery_b7.validity): a scan
#: stamped within this long after the first DRIVING report must exist, or crossing 0 is not defined.
MAX_SCAN_WAIT_S = 0.25
#: The nodes started before the startup reset of a pnp run (the rest follow the reset): the
#: simulator and the safety node, which only publish what the bridge feeds them.
PHASE_ONE_NODES = ("bridge_node", "safety_node")
#: Started last of all, once the lap node answers: a lap node still importing its modules when the
#: planner reports DRIVING starts lap 1 seconds late (its start scan is the first one after ITS first
#: DRIVING report), and its lines are then not the ground truth's. The planner is the one node whose
#: DRIVING report starts the clock of both.
PLANNER_NODE = "mppi_planner_node"
LAP_NODE = "lap_memory_node"


def rescore_from_start_scan(
    odom: list[tuple],
    xs,
    ys,
    lap_events: list[dict],
    sweep_s: float = SIM_SCAN_SWEEP_S,
):
    """Ground-truth laps counted from the pose of the system's own start scan.

    The simulator's lap tracker counts from the first odometry sample. From a standstill a few
    centimetres of start line are a few tenths of a second, so lap 1 of the truth and lap 1 of a
    system whose start line is the pose of its start scan differ by that, without either being
    wrong. The start scan's stamp comes from the system's first LapEvent (crossing time minus lap
    time is the start's effective time, the stamp plus half a sweep); the truth is then re-counted
    over the recorded ground-truth odometry from that instant. Returns (lap times, crossing
    stamps, start stamp) or None when the system closed no lap."""
    import bench_metrics as bm

    if not lap_events:
        return None
    first = lap_events[0]
    start_stamp = first["t_cross_s"] - first["lap_time_s"] - 0.5 * sweep_s
    tracker = bm.LapTracker(xs, ys)
    previous = None
    for sample in odom:
        t, x, y = sample[0], sample[1], sample[2]
        if t < start_stamp:
            continue
        if previous is not None and bm.is_teleport(previous, (x, y)):
            tracker.reset()
        tracker.update(x, y, t)
        previous = (x, y)
    return list(tracker.lap_times_s), list(getattr(tracker, "_crossings", [])), start_stamp


def first_scan_at_or_after(scan_stamps: list[float], t_s: float | None) -> float | None:
    """The header stamp of the first /scan stamped at or after `t_s`, or None. The start line of
    the ground truth is the pose at this stamp (pre-registration 1.3: the first /scan after the
    planner first reported DRIVING, `t_s` being the wall time this process received that report; every
    node and the simulator's scans share that clock). Defined by stamps, not by the order in which
    this process happened to receive the scans: its own spin loop lags the scan subscription by
    seconds in a busy run (a_S1_ccw_13 of the 2026-11-14 acceptance run saw the first scan 1.9 s
    late). The report's own header stamp is not used: the planner stamps its first DRIVING status
    about 2 s after it was received (a_T2_ccw_12)."""
    if t_s is None:
        return None
    later = [t for t in scan_stamps if t >= t_s]
    return min(later) if later else None


def crossing_lap_times(crossings: list[float], start_stamp: float | None) -> list[float]:
    """Ground-truth lap k is crossing stamp k minus crossing stamp k-1, crossing 0 being the stamp
    of the first /scan after the planner first reported DRIVING (pre-registration 1.3). It does
    not depend on where the simulator's own lap tracker started its clock: a tracker reset
    (a teleport in the recorded odometry) moves ITS first lap, not these differences."""
    if start_stamp is None or not crossings:
        return []
    edges = [float(start_stamp), *[float(c) for c in crossings]]
    return [round(b - a, 3) for a, b in pairwise(edges)]


def registered_truth(
    odom: list[tuple],
    scan_stamps: list[float],
    driving_receipt_s: float | None,
    centreline,
    startup_reset_done_s: float | None = None,
    max_scan_wait_s: float = MAX_SCAN_WAIT_S,
) -> dict:
    """The ground truth of a lap battery run, by the registered rule (pre-registration
    pnp-v0.5-laps section 2): crossing 0 is the stamp of the first /scan stamped at or after the
    receipt of the planner's first DRIVING report, the start line is the ground-truth pose at that
    stamp, crossings come from `bench_metrics.LapTracker` started at that sample, and lap k is
    crossing k minus crossing k-1 (lap 1 is the launch lap, from crossing 0). Nothing here reads
    the system's LapEvents: the system is the thing under test.

    `odom` is the recorded ground-truth odometry (stamp, x, y, yaw, v), `centreline` is the
    (xs, ys) of the loop the tracker counts progress on. The result says whether the truth is
    VALID. It is not when the recording does not cover crossing 0 (`truth_starts_after_crossing_0`:
    the first odometry stamp is later, so the line would be the pose at a later instant), when no
    scan is stamped within `max_scan_wait_s` after the report (`no_scan_at_start`) or when a
    startup reset completed after the report (`reset_after_driving`, `startup_reset_done_s` on
    the report's clock). An invalid run is rerun, never scored."""
    import bench_metrics as bm

    xs, ys = centreline
    crossing0 = first_scan_at_or_after(scan_stamps, driving_receipt_s)
    first_odom = float(odom[0][0]) if odom else None
    reason = ""
    if crossing0 is None or crossing0 > driving_receipt_s + max_scan_wait_s:
        reason = "no_scan_at_start"
    elif first_odom is None or first_odom > crossing0:
        reason = "truth_starts_after_crossing_0"
    elif startup_reset_done_s is not None and startup_reset_done_s > driving_receipt_s:
        reason = "reset_after_driving"
    crossings: list[float] = []
    if crossing0 is not None:
        tracker = bm.LapTracker(xs, ys)
        previous = None
        for sample in odom:
            t, x, y = sample[0], sample[1], sample[2]
            if t < crossing0:
                continue
            if previous is not None and bm.is_teleport(previous, (x, y)):
                tracker.reset()
            tracker.update(x, y, t)
            previous = (x, y)
        crossings = list(getattr(tracker, "_crossings", []))
    return {
        "truth_valid": reason == "",
        "truth_invalid_reason": reason,
        "truth_first_odom_s": first_odom,
        # brief 50: where the truth recording ends (B8 judges a count window that must be inside)
        "truth_last_odom_s": float(odom[-1][0]) if odom else None,
        "crossing0_s": crossing0,
        "crossings_s": crossings,
        "lap_times_s": crossing_lap_times(crossings, crossing0),
    }


def oracle_status_at(
    base: dict, now_s: float, lap_first_status_s: float | None, ready_after_s: float
) -> dict:
    """What the oracle's status says at node time `now_s` (brief 51c): until `ready_after_s`
    seconds after the runner received the lap node's first status it says `waiting` with a
    similarity of 0.0 (the rest of the status is the oracle's own); then, and always with a delay
    of 0, `base` as it is. `inf` never becomes ready. This is how the battery exercises the start
    gate."""
    if ready_after_s == 0.0:
        return base
    if lap_first_status_s is None or now_s < lap_first_status_s + ready_after_s:
        return {**base, "state": "waiting", "similarity": 0.0}
    return base


class OracleStatusPublisher:
    """The oracle's status, once a second of ground-truth time from the first sample it is fed,
    in every phase of the runner: the runner calls `on_time` from its ground-truth odometry
    callback, as brief 51 section 4 said (brief 51c: the first build fed it from the main loop,
    which begins after the planner's first /drive_raw)."""

    def __init__(self, status_fn, publish_fn, ready_after_s: float = 0.0, period_s: float = 1.0):
        self._status_fn = status_fn
        self._publish_fn = publish_fn
        self._ready_after_s = ready_after_s
        self._period_s = period_s
        self._due = -math.inf
        self._samples = 0
        self.first_publish_s: float | None = None

    def camera_keys(self) -> dict:
        """The camera counter's freshness keys as the oracle can speak them (outside review of
        brief 51c): the ground-truth samples seen so far are its images, so the sequence advances
        while the simulator runs, in every phase, and the camera is always fresh."""
        n = self._samples
        return {
            "camera": "fresh",
            "camera_fresh": True,
            "frame_seq": n,
            "frame_age_s": 0.0,
            "frames_since_reset": n,
            "reset_id": 1,
            "odom_age_s": 0.0,
            "odom_fresh": True,
        }

    def on_time(self, t_s: float, now_s: float, lap_first_status_s: float | None) -> None:
        self._samples += 1
        if t_s < self._due:
            return
        self._due = t_s + self._period_s
        if self.first_publish_s is None:
            self.first_publish_s = now_s
        body = {**self._status_fn(), **self.camera_keys()}
        self._publish_fn(oracle_status_at(body, now_s, lap_first_status_s, self._ready_after_s))


def refusal_result_keys(
    verdict: tuple[str, str], gate_trace: dict, refused_at_s: float, drive_raw_messages: int
) -> dict:
    """The result keys of a run the starter refused (`stopped_because` `start_refused`): the
    reason, when, the gate's trace and how many messages reached /drive_raw (none, the planner was
    never started)."""
    return {
        "stopped_because": "start_refused",
        "start_refusal": {"reason": verdict[1], "refused_at_s": refused_at_s, "gate": gate_trace},
        "drive_raw_messages": int(drive_raw_messages),
    }


class StartRefused(Exception):
    """The starter refused: the planner was never started (a result, not an error)."""

    def __init__(self, reason: str) -> None:
        super().__init__(reason)
        self.reason = reason


def ready_delay(text: str) -> float:
    """An argparse type: seconds, zero or more, or `inf`."""
    value = float(text)
    if math.isnan(value) or value < 0.0:
        raise argparse.ArgumentTypeError(f"{text!r} is not a number of seconds, zero or more")
    return value


def _gate_trace(gate_box: list) -> dict:
    return gate_box[0].trace() if gate_box else {}


def _finite_or_none(value: float | None) -> float | None:
    if value is None or not math.isfinite(value):
        return None
    return float(value)


def start_trace(
    lap_node_first_status_s: float | None = None,
    ready_first_s: float | None = None,
    gate_start_s: float | None = None,
    planner_spawn_s: float | None = None,
    driving_first_s: float | None = None,
    lap1_start_s: float | None = None,
    oracle_first_status_s: float | None = None,
    gate_verdict: str | None = None,
    gate_reason: str | None = None,
) -> dict:
    """The start of a run as the report prints it (brief 51c): when the lap node's first status
    reached the runner, when one first said `source_ready_now`, when the start gate said start,
    when the planner was spawned, when its first DRIVING report arrived, when the lap node's lap 1
    started, when the oracle first published a status (all node time, seconds), and the gate's
    verdict and reason. None for what did not happen; a non-finite time is None, so the JSON
    stays valid."""
    return {
        "lap_node_first_status_s": _finite_or_none(lap_node_first_status_s),
        "ready_first_s": _finite_or_none(ready_first_s),
        "gate_start_s": _finite_or_none(gate_start_s),
        "planner_spawn_s": _finite_or_none(planner_spawn_s),
        "driving_first_s": _finite_or_none(driving_first_s),
        "lap1_start_s": _finite_or_none(lap1_start_s),
        "oracle_first_status_s": _finite_or_none(oracle_first_status_s),
        "gate_verdict": gate_verdict,
        "gate_reason": gate_reason,
    }


def run_startup(
    stack: list[tuple[str, list[str]]],
    two_phase: bool,
    spawn,
    ready,
    spin,
    reset,
    after_reset,
    wait_timeout_polls: int,
    lap_ready=None,
    start_decide=None,
    gate_polls: int | None = None,
) -> tuple[str, str]:
    """The runner's startup order. `spawn(name, command)` starts a node, `ready()` says the
    reset service is there (and what the caller also needs before the reset), `spin()` serves
    the runner's own callbacks for one poll, `reset()` performs the startup reset and returns when
    it completed, `after_reset()` clears the recording, `lap_ready()` says the lap node answers
    (its status arrived).

    `two_phase` (driver pnp): only `PHASE_ONE_NODES` start first; the rest start after the reset
    and the clearing of the recording, so the startup reset can no longer land on a car the
    planner has already set moving (brief 48: it did in 4 of 90 B4 runs, 2.3 s after the planner
    reported DRIVING, and the ground truth began there). The wait is a spinning one: the blocking
    `wait_for_service` served no callback while it waited. Of the rest, the perception and the lap
    node start first and the planner only once the lap node answers (`lap_ready`, when the stack
    has a lap node and the caller gives one), so the lap node never starts lap 1 after the
    planner's first DRIVING report. Otherwise every node starts at once, as before.

    `start_decide` (brief 51c, lap source `truth`) replaces `lap_ready`: it returns the start
    rule's `(verdict, reason)` (`racer_laps.start_ready.StartReadyGate`, the rule the car's launch
    uses); `gate_polls` bounds that wait in polls (default `wait_timeout_polls`; the runner passes a
    very large number and bounds the wait in wall time instead, since a spin returns at every
    message and 30 s of simulated time is many thousands of polls). The planner, the only
    publisher of /drive_raw, is spawned on `start` and never on
    `refuse`. Returns `("start", "")`, or `("refuse", reason)` with the planner not started."""
    first = [item for item in stack if item[0] in PHASE_ONE_NODES] if two_phase else list(stack)
    later = [item for item in stack if item not in first]
    for name, command in first:
        spawn(name, command)
    for _ in range(wait_timeout_polls):
        if ready():
            break
        spin()
    else:
        raise RuntimeError("/sim/reset not available")
    reset()
    after_reset()
    planner = [item for item in later if item[0] == PLANNER_NODE]
    others = [item for item in later if item[0] != PLANNER_NODE]
    for name, command in others:
        spawn(name, command)
    has_lap_node = any(item[0] == LAP_NODE for item in others)
    if planner and start_decide is not None and has_lap_node:
        for _ in range(wait_timeout_polls if gate_polls is None else gate_polls):
            verdict, reason = start_decide()
            if verdict != "wait":
                break
            spin()
        else:
            raise RuntimeError("lap_memory_node did not answer before the planner was due")
        if verdict == "refuse":
            return verdict, reason
    elif planner and lap_ready is not None and has_lap_node:
        for _ in range(wait_timeout_polls):
            if lap_ready():
                break
            spin()
        else:
            raise RuntimeError("lap_memory_node did not answer before the planner was due")
    for name, command in planner:
        spawn(name, command)
    return "start", ""


def note_guidance_floor(counters: dict, v_ref_mps: list[float]) -> None:
    """Count one guidance message: it has stations, and the floor was its active bound when the
    lowest per station bound sits at the D v1 floor (`counters["floor_mps"]`)."""
    if not len(v_ref_mps):
        return
    counters["messages_with_stations"] += 1
    if min(v_ref_mps) <= counters["floor_mps"] + 1e-4:
        counters["messages_floor_active"] += 1


def lap_results(
    lap_tracker,
    events: list[tuple],
    lap_events: list[dict],
    guidance_log: list[tuple],
    lap_status: dict,
    driving_first_s: float | None,
    lap1_start_s: float | None = None,
    rescored: tuple | None = None,
    truth: dict | None = None,
) -> dict:
    """Ground truth against the system for a run with the lap memory: the ground-truth crossing
    stamps, lap times, the system's LapEvents, the interventions per ground-truth lap by source
    (every /safety/events ENGAGE, and the BRAKE severity ones separately) and the guidance cap
    per system lap.

    Lap 1 is counted from the system's own start line: the stamp of the first /scan after the
    planner first reported DRIVING (`lap1_start_s`; the report instant itself, `driving_first_s`,
    when no scan was seen). The simulator's lap tracker counts from the first odometry sample, and
    from a standstill a few centimetres of start line are a few tenths of a second.

    `truth` is `registered_truth`'s result (brief 48): when given it IS the ground truth of the
    result, with its validity, and `rescored` (the numbers `rescore_from_start_scan` takes from the
    system's own first LapEvent) is only reported as `gt_lap_times_rescored_s`."""
    crossings = list(getattr(lap_tracker, "_crossings", []))
    gt_times = list(lap_tracker.lap_times_s)
    start_s = lap1_start_s if lap1_start_s is not None else driving_first_s
    first_odom_times = list(gt_times)
    if truth is not None:
        gt_times = truth["lap_times_s"]
        crossings = truth["crossings_s"]
        start_s = truth["crossing0_s"]
    elif rescored is not None:
        # truth counted from the pose of the system's start scan (see rescore_from_start_scan)
        gt_times, crossings, start_s = rescored
    elif gt_times and crossings and start_s is not None:
        gt_times[0] = crossings[0] - start_s
    edges = [start_s if start_s is not None else -math.inf, *crossings]
    engage: list[dict] = []
    brake: list[dict] = []
    for k in range(len(crossings)):
        lo, hi = edges[k], edges[k + 1]
        in_lap = [e for e in events if e[2] == 0 and lo < e[0] <= hi]
        engage.append(_count_by_source(in_lap))
        brake.append(_count_by_source([e for e in in_lap if e[3] == 2]))
    caps: dict[int, dict] = {}
    for t, lap_index, cap, n_arrays in guidance_log:
        entry = caps.setdefault(
            lap_index, {"first_cap_mps": cap, "max_cap_mps": cap, "max_stations": n_arrays}
        )
        entry["max_cap_mps"] = max(entry["max_cap_mps"], cap)
        entry["max_stations"] = max(entry["max_stations"], n_arrays)
        entry["last_cap_mps"] = cap
    by_crossing_start = lap1_start_s if lap1_start_s is not None else driving_first_s
    if truth is not None:
        by_crossing_start = truth["crossing0_s"]
    by_crossing = crossing_lap_times(crossings, by_crossing_start)
    out: dict = {
        "gt_lap_crossings_s": [round(c, 3) for c in crossings],
        "gt_lap_times_s": [round(t, 3) for t in gt_times],
        "gt_lap_times_by_crossing_s": by_crossing,
        "gt_lap1_start_s": start_s,
        "gt_lap_times_from_first_odom_s": [round(t, 3) for t in first_odom_times],
        "gt_rescored_from_start_scan": rescored is not None and truth is None,
        "driving_first_s": driving_first_s,
        "system_lap_events": lap_events,
        "system_laps": len(lap_events),
        "engage_per_lap": engage,
        "brake_per_lap": brake,
        "guidance_caps_by_lap": {str(k): v for k, v in sorted(caps.items())},
        "lap_status": dict(lap_status),
    }
    if truth is not None:
        out.update(
            {
                # the registered truth (brief 48), under the names the decisions read
                "gt_lap_times_registered_s": list(by_crossing),
                "gt_lap_crossings_registered_s": [round(c, 3) for c in crossings],
                "truth_valid": bool(truth["truth_valid"]),
                "truth_invalid_reason": str(truth["truth_invalid_reason"]),
                "truth_first_odom_s": truth["truth_first_odom_s"],
                "truth_last_odom_s": truth.get("truth_last_odom_s"),
                # what rescore_from_start_scan made of the system's first LapEvent: reported only
                "gt_lap_times_rescored_s": (
                    [round(t, 3) for t in rescored[0]] if rescored is not None else []
                ),
            }
        )
    return out


def _count_by_source(events: list[tuple]) -> dict:
    out: dict[str, int] = {}
    for e in events:
        out[e[1]] = out.get(e[1], 0) + 1
    return out


def write_event_trace(args, work: Path, trace: dict, t_first: float) -> None:
    """With RACER_EVENT_TRACE_DIR set, writes <dir>/<case>.json.gz: /drive_raw and /drive as
    [stamp, received, steering, speed], /safety/events as [stamp, received, source, phase,
    severity, detail] (the rate_limit detail carries the gate's dt) and the planner status as
    [stamp, received, state, start_missing], all in seconds with t_first (the first odometry) given so a time can
    be read the way the result's safety_events are. Diagnostics only: nothing here is asserted,
    and without the variable nothing is written. The case name is the pytest temp directory the
    runner was started under (one per test case)."""
    directory = os.environ.get("RACER_EVENT_TRACE_DIR")
    if not directory:
        return
    import gzip

    out_dir = Path(directory)
    out_dir.mkdir(parents=True, exist_ok=True)
    case = work.parent.name if work.parent.name else "case"
    stalls = (
        "stalls"
        if "debug_stall_profile" in str(getattr(args, "planner_params_json", ""))
        else "nostalls"
    )
    label = (
        f"{args.track}-{'cw' if args.reverse else 'ccw'}-v{args.v_max_mps}-{stalls}"
        f"-{getattr(args, 'car', '')}-{getattr(args, 'backend', '')}"
    )
    payload = {"case": case, "label": label, "t_first_s": t_first, **trace}
    stamp = time.strftime("%H%M%S")
    with gzip.open(out_dir / f"{label}-{case}-{stamp}.json.gz", "wt") as handle:
        json.dump(payload, handle, separators=(",", ":"))


def pnp_timing_summary(recording, planner_status: list[tuple[float, int]]) -> dict:
    """What the planner's timing acceptance counts (pre-registration 4.1 and 4.2):

    safety_event_counts   /safety/events PHASE_ENGAGE records by source, stamped between the first
                          non-zero /drive_raw command and the last /pnp/planner/status (the
                          watchdog engagement after the planner has stopped is the stop mechanism
                          and is not counted).
    drive_raw             the command stream: the largest gap between two messages, the largest
                          rate at which the speed magnitude rose over a window of at least 50 ms
                          (m/s^2, the planner's allowance is its rate_limit_fraction times
                          max_acceleration),
                          and the largest first non-zero speed after the command had been 0.
    """
    out: dict = {}
    # (stamp, speed, steering); a planner on use_sim_time stamps 0 until its first /clock arrives
    drive = [d for d in recording.drive_raw if d[0] > 0.0]
    first_nonzero = next((t for t, v, _ in drive if v != 0.0), None)
    last_status = planner_status[-1][0] if planner_status else None
    if first_nonzero is not None and last_status is not None:
        counts: dict[str, int] = {}
        for stamp, source, phase, _severity in recording.events:
            if phase == 0 and first_nonzero <= stamp <= last_status:
                counts[source] = counts.get(source, 0) + 1
        out["safety_event_counts"] = counts
        out["pnp_window_s"] = round(last_status - first_nonzero, 2)
    if len(drive) >= 2:
        t = np.array([d[0] for d in drive])
        v = np.array([d[1] for d in drive])
        mag = np.abs(v)  # the planner limits the growth of the magnitude (a reverse is slowing)
        gaps = np.diff(t)
        max_rate = 0.0
        j0 = 0
        for j in range(1, len(t)):
            while j0 < j and t[j] - t[j0] >= 0.05:
                max_rate = max(max_rate, (mag[j] - mag[j0]) / (t[j] - t[j0]))
                j0 += 1
        starts = [float(v[i]) for i in range(1, len(v)) if v[i - 1] == 0.0 and v[i] > 0.0]
        out["drive_raw"] = {
            "messages": len(drive),
            "max_gap_s": round(float(gaps.max()), 4),
            "gaps_over_50ms": int((gaps > 0.05).sum()),
            "max_speed_rise_mps2": round(float(max_rate), 3),
            "max_first_speed_after_zero_mps": round(max(starts), 4) if starts else 0.0,
            "starts_from_zero": len(starts),
        }
    # safety_node's own output. It publishes /drive on a 50 Hz WALL timer and stamps with the sim
    # clock, and the sim never runs faster than the wall clock, so two /drive stamps more than a
    # few cycles apart mean the safety_node process was not scheduled for that long (a loaded
    # runner), not that anything the planner did was late. Its watchdog then reads the wall time
    # since the last /drive_raw it received as a stale command and brakes, and the release is a
    # ramp the rate limit clips (a brake plus a rate_limit engagement, the whole signature).
    drive_out = [d for d in recording.drive if d[0] > 0.0]
    if len(drive_out) >= 2:
        out_gaps = np.diff(np.array([d[0] for d in drive_out]))
        out["safety_drive"] = {
            "messages": len(drive_out),
            "max_gap_s": round(float(out_gaps.max()), 4),
            "gaps_over_50ms": int((out_gaps > 0.05).sum()),
        }
    return out


def run_benchmark(args: argparse.Namespace) -> dict:
    os.environ["ROS_DOMAIN_ID"] = str(args.domain_id)
    import bench_metrics as bm
    import rclpy
    from ackermann_msgs.msg import AckermannDriveStamped
    from geometry_msgs.msg import PoseWithCovarianceStamped
    from nav_msgs.msg import Odometry
    from racer_msgs.msg import SafetyEvent
    from rclpy.qos import HistoryPolicy, QoSProfile, ReliabilityPolicy
    from std_srvs.srv import Trigger
    from tf2_ros import Buffer, TransformListener

    work = Path(args.work_dir) if args.work_dir else Path(tempfile.mkdtemp(prefix="family_bench_"))
    work.mkdir(parents=True, exist_ok=True)
    track_params = json.loads(args.track_params_json)
    overrides = json.loads(args.params_json)
    tracker_overrides = json.loads(args.tracker_params_json)
    result: dict = {
        "track": args.track,
        "track_params": track_params,
        "reverse": args.reverse,
        "driver": args.driver,
        "car": args.car,
        "seed": args.seed,
        "lidar": args.lidar,
    }
    synthetic, map_yaml, track_params = build_world(args.track, track_params, work)
    walls_synthetic = synthetic
    if args.reverse:
        synthetic = synthetic.reversed()
    vehicle = yaml.safe_load(base.VEHICLE_PARAMS_PATH.read_text())
    box = bm.BodyBox.from_vehicle_params(vehicle)
    cg_to_rear = float(vehicle["chassis"]["cg_to_rear_axle_m"])
    lap_tracker = bm.LapTracker(synthetic.centreline_x, synthetic.centreline_y)
    result["loop_length_m"] = round(lap_tracker.length_m, 3)

    localised = args.driver == "tracker_loc"
    bridge = bridge_track_parameters(args.track, track_params, args.seed, args.reverse)
    bridge.update(car_parameters(args.car))
    if args.plant_bundle:
        bridge.update(plant_parameters(Path(args.plant_bundle)))
    bridge.update(plant_overrides(args))
    mounted = args.lidar == "mounted" or localised
    if mounted:
        from racer_tools.vehicle_params_loader import load_vehicle_params

        helpers = base._load_module(
            "sim_autopilot_launch", LOCALISATION_LAUNCH / "sim_autopilot.launch.py"
        )
        car_like = helpers.car_like_bridge_parameters(load_vehicle_params(), args.noise)
        if localised:
            bridge.update(car_like)
            bridge.update({k: float(v) for k, v in (kv.split("=") for kv in args.noise_set)})
        else:
            lidar_keys = (
                "lidar_offset_x_m",
                "lidar_yaw_rad",
                "lidar_fov_rad",
                "lidar_num_beams",
                "scan_rate_hz",
                "scan_range_max_m",
            )
            bridge.update({k: car_like[k] for k in lidar_keys})
    if args.driver in ("tracker", "tracker_policy"):
        bridge["ground_truth_rear_odom_offset_x_m"] = -cg_to_rear
    pnp_launch = None
    if args.driver == "pnp":
        pnp_launch = base._load_module("pnp_launch", LOCALISATION_LAUNCH / "pnp.launch.py")
        bridge.update({k: car_like[k] for k in ("publish_wheel_odom", "wheel_odom_rate_hz")})
        bridge["scan_noise_std_m"] = helpers.SIM_SENSOR_NOISE["scan_noise_std_m"]
        # The C1 as it runs on the car: 720 beams over the full circle.
        bridge["lidar_num_beams"] = PNP_SCAN_BEAMS
        bridge["lidar_fov_rad"] = 2.0 * math.pi * (PNP_SCAN_BEAMS - 1) / PNP_SCAN_BEAMS

    nodes: dict[str, dict] = {
        "bridge_node": bridge,
        "safety_node": {"laser_yaw_from_vehicle_params": mounted},
    }
    stack = [
        ("bridge_node", [base._executable("racer_gym_bridge", "bridge_node")]),
        ("safety_node", [base._executable("racer_safety", "safety_node")]),
    ]
    if args.driver == "gap_follow":
        profiles = base.load_profiles()
        gap = base.gap_follow_parameters(args.profile, overrides, profiles)
        gap["laser_yaw_from_vehicle_params"] = mounted
        nodes["gap_follow_node"] = gap
        stack.append(("gap_follow_node", [base._executable("racer_control", "gap_follow_node")]))
        result.update({"profile": args.profile, "overrides": overrides, "gap_follow_params": gap})
    elif args.driver == "pnp":
        perception, planner = pnp_launch.parameters_for(
            args.perception_bundle,
            args.dynamics_bundle,
            "geometric" if args.backend == "oracle" else args.backend,
            args.v_max_mps,
            mounted,
        )
        planner.update(json.loads(args.planner_params_json))
        if args.wall_clock:
            planner["use_sim_time"] = True
        nodes["track_perception_node"], nodes["mppi_planner_node"] = perception, planner
        pnp_nodes = [
            ("racer_track_perception", "track_perception_node"),
            ("racer_planner", "mppi_planner_node"),
        ]
        if args.backend == "oracle":
            # Measurement M-A2 (tests/l5_pnp_oracle, never part of the car's launch): a ground
            # truth LocalTrack in place of the perception node, same topic, rate and stamps.
            if args.no_perception:
                raise ValueError("--backend oracle with --no-perception makes no sense")
            ground_truth = work / "ground_truth_track.npz"
            write_oracle_ground_truth(synthetic, ground_truth)
            del nodes["track_perception_node"]
            nodes["oracle_track_node"] = {"ground_truth_track": str(ground_truth)}
            pnp_nodes = pnp_nodes[1:]
            stack.append(("oracle_track_node", [sys.executable, str(ORACLE_NODE)]))
        if args.no_perception:
            # The scan-only fault case: the planner starts and no track ever arrives.
            del nodes["track_perception_node"]
            pnp_nodes = pnp_nodes[1:]
        if args.lap_memory != "none":
            # brief 53: D v1 on the sim truth source. pnp.launch.py (single-implementer path,
            # untouched) refuses v1 with any source but start_scan, so its v0 parameters are taken
            # and the mode set here; the lap node itself allows v1 with truth (never the camera).
            v1_on_truth = args.lap_memory == "v1" and args.lap_source == "truth"
            lap_params = pnp_launch.lap_memory_parameters(
                args.dynamics_bundle,
                args.v_max_mps,
                args.v_lap1_mps,
                "v1" if args.lap_memory == "v1" and not v1_on_truth else "v0",
                args.lap_source,
            )
            if v1_on_truth:
                lap_params["lap_mode"] = "v1"
            lap_params.update(
                {
                    # The heading the gate integrates from the gated steering command: what the
                    # sim plant does with it, per side (the real car's gain is vehicle_params'
                    # steering.yaw_rate_gain_table).
                    "yaw_gain_override": SIM_YAW_GAIN_LEFT,
                    "yaw_gain_override_right": SIM_YAW_GAIN_RIGHT,
                    "laser_yaw_from_vehicle_params": mounted,
                    "save_lap1_map": False,
                    "output_dir": str(work / "laps"),
                }
            )
            if args.lap_source == "truth":
                # the simulator's oracle (brief 51): the lap node reads its two topics, on
                # simulated time (the node refuses `truth` without it)
                if not args.sim_time:
                    raise ValueError("--lap-source truth needs --sim-time")
                lap_params.update(
                    {
                        "lap_event_topic": TRUTH_EVENT_TOPIC,
                        "lap_source_status_topic": TRUTH_STATUS_TOPIC,
                    }
                )
            # D v1 variants (brief 29): the defaults are the node's own.
            lap_params["zone_merge"] = bool(args.lap_zone_merge)
            if args.lap_bound_floor_mps is not None:
                lap_params["v_bound_floor_mps"] = float(args.lap_bound_floor_mps)
            if os.environ.get("RUNNER_LAP_DUMP_DIR"):
                # Diagnostics only: the lap node's inputs, to replay a run through the gate.
                lap_params["debug_dump_path"] = str(
                    Path(os.environ["RUNNER_LAP_DUMP_DIR"]) / f"{work.name}.npz"
                )
            lap_params.update(json.loads(args.lap_params_json))
            nodes["lap_memory_node"] = lap_params
            pnp_nodes.append(("racer_laps", "lap_memory_node"))
            if args.lap_memory != "off":
                planner["use_lap_guidance"] = True
                # missing, stale or malformed guidance: the lap-1 cap, never the ceiling
                planner["v_no_guidance_mps"] = float(lap_params["v_cap_start_mps"])
            result["lap_params"] = lap_params
        if args.wall_clock:
            stack.append(("wall_clock_node", [sys.executable, str(HERE / "wall_clock_node.py")]))
        for package, name in pnp_nodes:
            if name == "lap_memory_node" and args.count_innovation:
                # Test-only entry point that counts the innovation gate's rejections (brief 40).
                os.environ["LAP_INNOVATION_OUT"] = str(work / "innovation.json")
                stack.append((name, [sys.executable, str(LAP_NODE_COUNTING)]))
                continue
            stack.append((name, [base._executable(package, name)]))
        result.update(
            {
                "backend": args.backend,
                "v_max_mps": args.v_max_mps,
                "v_lap1_mps": args.v_lap1_mps,
                "lap_memory": args.lap_memory,
                "lap_source": args.lap_source,
                "pnp": nodes,
            }
        )
    else:
        raceline_csv, report = make_raceline(args, walls_synthetic, map_yaml, work)
        tracker = dict(LANE_TRACKER_DEFAULTS)
        tracker.update(tracker_overrides)
        tracker["raceline_path"] = str(raceline_csv)
        if localised:
            tracker.update({"odom_topic": "/odom", "pose_frame": "map"})
        else:
            tracker["odom_topic"] = "/sim/ground_truth_rear_odom"
        if args.driver == "tracker_policy":
            if not args.policy_dir:
                raise ValueError("--driver tracker_policy needs --policy-dir")
            policy_dir = Path(args.policy_dir)
            meta = yaml.safe_load((policy_dir / "policy_meta.yaml").read_text())
            base_settings = meta["must_match_at_deploy"]["base_controller"]
            tracker.update(
                {
                    k: base_settings[k]
                    for k in ("lookahead_min_m", "lookahead_max_m", "lookahead_curvature_ref_1pm")
                }
            )
            tracker["drive_topic"] = "/tracker/command"
            nodes["policy_node"] = {
                "policy_dir": str(policy_dir),
                "raceline_path": str(raceline_csv),
                "odom_topic": tracker["odom_topic"],
                "pose_frame": "",
                "tracker_command_topic": "/tracker/command",
                "laser_yaw_from_vehicle_params": mounted,
            }
        nodes["tracker_node"] = tracker
        stack.append(("tracker_node", [base._executable("racer_control", "tracker_node")]))
        if args.driver == "tracker_policy":
            stack.append(("policy_node", [base._executable("racer_policy", "policy_node")]))
            result["policy_dir"] = str(args.policy_dir)
        result.update({"tracker_params": tracker, "raceline": report})
        if args.track == "map":
            # The imported map's start pose comes from its centreline file: give it the raceline
            # (already in the driving direction), so the car starts on the line it follows.
            from racer_gym_bridge import track_loader

            track_params["map_centreline_csv"] = str(raceline_csv)
            synthetic = track_loader.build_synthetic_track(args.track, **track_params)
            bridge["synthetic_map_centreline"] = str(raceline_csv)
            bridge["reverse_direction"] = False
            lap_tracker = bm.LapTracker(synthetic.centreline_x, synthetic.centreline_y)
            result["loop_length_m"] = round(lap_tracker.length_m, 3)
        if localised:
            stack.append(
                (
                    "localization",
                    [
                        "ros2",
                        "launch",
                        str(LOCALISATION_LAUNCH / "localization.launch.py"),
                        f"map:={map_yaml}",
                        "set_initial_pose:=false",
                        # the sim's car steers exactly as commanded (vehicle_params'
                        # steering.yaw_rate_gain_table is the real car's measured value)
                        "yaw_gain_override:=1.0",
                        *([f"amcl_params_file:={args.amcl_params}"] if args.amcl_params else []),
                        *([f"ekf_params_file:={args.ekf_params}"] if args.ekf_params else []),
                    ],
                )
            )
    if args.sim_time:
        # Simulated time (GitHub issue 156): the bridge owns /clock and paces its steps, every
        # node has use_sim_time:=true, and sim time may not lead the perception's per-scan
        # /pnp/local_track (stamped with its scan) by more than the bridge's lockstep lag.
        if args.driver != "pnp" or args.wall_clock:
            raise ValueError("--sim-time is for driver pnp, and not with --wall-clock")
        bridge.update({"sim_time": True, "lockstep_ack": [PNP_LOCKSTEP_ACK]})
        for node_name, node_params in nodes.items():
            if node_name != "bridge_node":
                node_params["use_sim_time"] = True
        result["sim_time"] = True
    params_file = work / "params.yaml"
    base._write_params_file(params_file, nodes)

    processes: list[tuple[str, subprocess.Popen]] = []

    planner_spawned: list[float] = []  # node time of the planner's spawn (start_trace)

    def spawn(name: str, command: list[str]) -> None:
        if name == PLANNER_NODE:
            planner_spawned.append(received_s())
        log = (work / f"{name}.log").open("w")
        if name == "localization":
            cmd = command
        else:
            cmd = [*command, "--ros-args", "--params-file", str(params_file)]
        processes.append(
            (
                name,
                subprocess.Popen(cmd, stdout=log, stderr=subprocess.STDOUT, env=os.environ.copy()),
            )
        )

    def died() -> str | None:
        for name, p in processes:
            if p.poll() is not None:
                tail = (work / f"{name}.log").read_text()[-1500:]
                return f"{name} exited with {p.returncode}: {tail}"
        return None

    duration_s = args.duration
    result["requested_duration_s"] = duration_s
    rclpy.init()
    if args.sim_time:
        from rclpy.parameter import Parameter

        node = rclpy.create_node(
            "track_family_runner",
            parameter_overrides=[Parameter("use_sim_time", Parameter.Type.BOOL, True)],
        )
    else:
        node = rclpy.create_node("track_family_runner")
    recording = bm.Recording()
    reliable = QoSProfile(
        reliability=ReliabilityPolicy.RELIABLE, history=HistoryPolicy.KEEP_LAST, depth=200
    )
    rear_pose: list = [None]
    loc_errors: list[float] = []
    loc_series: list[tuple] = []
    tf_buffer = Buffer()
    TransformListener(tf_buffer, node, spin_thread=False)

    def odom_yaw() -> float:
        """Heading of the EKF's odom -> base_link (the odometry alone, before AMCL's correction)."""
        if not tf_buffer.can_transform("odom", "base_link", rclpy.time.Time()):
            return float("nan")
        q = tf_buffer.lookup_transform("odom", "base_link", rclpy.time.Time()).transform.rotation
        return math.atan2(2.0 * (q.w * q.z + q.x * q.y), 1.0 - 2.0 * (q.y * q.y + q.z * q.z))

    def on_odom(msg) -> None:
        q = msg.pose.pose.orientation
        yaw = float(np.arctan2(2.0 * (q.w * q.z + q.x * q.y), 1.0 - 2.0 * (q.y * q.y + q.z * q.z)))
        x, y = msg.pose.pose.position.x, msg.pose.pose.position.y
        recording.odom.append(
            (base._stamp_s(msg.header.stamp), x, y, yaw, msg.twist.twist.linear.x)
        )
        rear_pose[0] = (
            x - cg_to_rear * math.cos(yaw),
            y - cg_to_rear * math.sin(yaw),
            yaw,
        )
        if oracle_status is not None:
            # the oracle's status, in every phase of the run (brief 51c)
            oracle_status.on_time(
                base._stamp_s(msg.header.stamp),
                received_s(),
                lap_first_status[0] if lap_first_status else None,
            )
        if (
            localised
            and len(recording.odom) % 5 == 0
            and tf_buffer.can_transform("map", "base_link", rclpy.time.Time())
        ):
            tf = tf_buffer.lookup_transform("map", "base_link", rclpy.time.Time())
            tq = tf.transform.rotation
            est_yaw = math.atan2(
                2.0 * (tq.w * tq.z + tq.x * tq.y), 1.0 - 2.0 * (tq.y * tq.y + tq.z * tq.z)
            )
            ex, ey = tf.transform.translation.x, tf.transform.translation.y
            gx, gy, gyaw = rear_pose[0]
            dx, dy = ex - gx, ey - gy
            loc_errors.append(math.hypot(dx, dy))
            loc_series.append(
                (
                    base._stamp_s(msg.header.stamp),
                    math.cos(gyaw) * dx + math.sin(gyaw) * dy,  # along the car's heading
                    -math.sin(gyaw) * dx + math.cos(gyaw) * dy,  # across it, left positive
                    math.atan2(math.sin(est_yaw - gyaw), math.cos(est_yaw - gyaw)),
                    gyaw,
                    odom_yaw(),
                )
            )

    # Arrival trace for offline diagnosis (RACER_EVENT_TRACE_DIR, see write_event_trace): the stamp
    # of each message next to the time it reached this node, on the node's clock (sim time).
    trace: dict[str, list] = {"drive_raw": [], "drive": [], "events": [], "planner": []}

    def received_s() -> float:
        return node.get_clock().now().nanoseconds * 1e-9

    def on_drive_raw(msg) -> None:
        recording.drive_raw.append(
            (base._stamp_s(msg.header.stamp), msg.drive.speed, msg.drive.steering_angle)
        )
        trace["drive_raw"].append(
            (
                base._stamp_s(msg.header.stamp),
                received_s(),
                msg.drive.steering_angle,
                msg.drive.speed,
            )
        )

    def on_drive(msg) -> None:
        recording.drive.append(
            (base._stamp_s(msg.header.stamp), msg.drive.speed, msg.drive.steering_angle)
        )
        trace["drive"].append(
            (
                base._stamp_s(msg.header.stamp),
                received_s(),
                msg.drive.steering_angle,
                msg.drive.speed,
            )
        )

    event_details: list[dict] = []

    def on_event(msg) -> None:
        recording.events.append((base._stamp_s(msg.stamp), msg.source, msg.phase, msg.severity))
        trace["events"].append(
            (
                base._stamp_s(msg.stamp),
                received_s(),
                msg.source,
                int(msg.phase),
                int(msg.severity),
                msg.detail,
            )
        )
        if msg.phase == 0 and len(event_details) < 40:
            event_details.append(
                {"t_s": base._stamp_s(msg.stamp), "source": msg.source, "detail": msg.detail}
            )

    node.create_subscription(Odometry, "/sim/ground_truth_odom", on_odom, reliable)
    node.create_subscription(AckermannDriveStamped, "/drive_raw", on_drive_raw, reliable)
    node.create_subscription(AckermannDriveStamped, "/drive", on_drive, reliable)
    node.create_subscription(SafetyEvent, "/safety/events", on_event, reliable)
    policy_status: dict = {}
    policy_events: list[dict] = []
    if args.driver == "tracker_policy":
        from std_msgs.msg import String

        def on_policy_status(msg) -> None:
            policy_status.clear()
            policy_status.update(json.loads(msg.data))

        def on_policy_event(msg) -> None:
            policy_events.append(json.loads(msg.data))

        node.create_subscription(String, "/policy/status", on_policy_status, reliable)
        node.create_subscription(String, "/policy/events", on_policy_event, reliable)
    planner_states: list[int] = []
    tracker_resets: list[float] = []  # stamps of ground-truth teleports (the tracker restarts)
    lap_events: list[dict] = []
    guidance_log: list[tuple] = []
    bound_log = dv1_measures.BoundLog()  # brief 53: the published bound per lap and station
    cap_series: list[tuple[float, float]] = []  # (t, cap) at every change of the guidance cap
    cap_last = [math.nan]
    oracle: TruthLapSource | None = (
        TruthLapSource(FaultPlan.from_dict(json.loads(args.lap_truth_faults_json)))
        if args.driver == "pnp" and args.lap_memory != "none" and args.lap_source == "truth"
        else None
    )
    oracle_events: list[dict] = []
    oracle_pubs: dict = {}
    lap_status: dict = {}
    lap_first_status: list[float] = []  # node time the lap node's first status arrived
    lap_ready_first: list[float] = []  # node time the first status with source_ready_now arrived
    oracle_status: OracleStatusPublisher | None = None  # set where the oracle's publishers exist
    gate_box: list = []  # the start gate (lap source truth), created when the wait begins
    gate_wall_start: list[float] = []  # wall time the wait began (it is bounded in wall time too)
    # B4 report item: guidance messages with stations, and those whose lowest per station bound
    # sits at the D v1 floor (the floor was the active bound). Counted here, not in the node.
    guidance_floor: dict = {
        "floor_mps": float(
            args.lap_bound_floor_mps if args.lap_bound_floor_mps is not None else 0.3
        ),
        "messages_with_stations": 0,
        "messages_floor_active": 0,
    }
    driving_first: list[float] = []
    first_scan_after_driving: list[float] = []  # by arrival order (kept for the record only)
    scan_stamps: list[float] = []
    planner_status: list[tuple[float, int]] = []
    planner_ticks: list[pts.Tick] = []

    # While STARTING, the planner's first missing warm start condition (PlannerStatus.start_missing)
    # per tick, counted by value: a run that never leaves STARTING says which condition held it.
    start_missing: list[int] = []

    def on_planner_status(msg) -> None:
        planner_states.append(msg.state)
        planner_status.append((base._stamp_s(msg.header.stamp), msg.state))
        missing = int(getattr(msg, "start_missing", 0))
        if msg.state == 0:
            start_missing.append(missing)
        trace["planner"].append(
            (base._stamp_s(msg.header.stamp), received_s(), int(msg.state), missing)
        )

    if args.driver == "pnp":
        from racer_pnp_msgs.msg import LapEvent, LapGuidance, PlannerStatus

        def on_planner(m) -> None:
            on_planner_status(m)
            planner_ticks.append(
                pts.Tick(
                    base._stamp_s(m.header.stamp),
                    int(m.state),
                    float(m.cmd_speed_mps),
                    float(m.v_cap_mps),
                    float(m.track_confidence),
                    float(m.plan_min_clearance_m),
                    int(m.block_reason),
                )
            )
            if m.state == 1 and not driving_first:
                driving_first.append(node.get_clock().now().nanoseconds * 1e-9)

        node.create_subscription(PlannerStatus, "/pnp/planner/status", on_planner, reliable)
        if args.lap_memory != "none":
            from std_msgs.msg import String

            def on_lap_event(m) -> None:
                lap_events.append(
                    {
                        "lap_index": int(m.lap_index),
                        "t_cross_s": base._stamp_s(m.header.stamp),
                        "frame_id": m.header.frame_id,
                        "lap_time_s": float(m.lap_time_s),
                        "length_m": float(m.lap_length_m),
                        "cap_mps": float(m.v_cap_mps),
                        "safety_brakes": int(m.safety_brakes),
                        "min_clearance_m": float(m.min_clearance_m),
                        "clean": bool(m.clean),
                    }
                )

            def on_guidance(m) -> None:
                note_guidance_floor(guidance_floor, list(m.v_ref_mps))
                now_s = node.get_clock().now().nanoseconds * 1e-9
                if not cap_series or abs(cap_series[-1][1] - float(m.v_cap_mps)) > 1e-9:
                    cap_series.append((now_s, float(m.v_cap_mps)))
                cap_last[0] = now_s
                guidance_log.append(
                    (
                        node.get_clock().now().nanoseconds * 1e-9,
                        int(m.lap_index),
                        float(m.v_cap_mps),
                        len(m.s_m),
                    )
                )
                bound_log.add(int(m.lap_index), float(m.v_cap_mps), list(m.s_m), list(m.v_ref_mps))

            def on_lap_status(m) -> None:
                lap_status.clear()
                lap_status.update(json.loads(m.data))
                now_s = received_s()
                if gate_box:
                    gate_box[0].on_status(now_s, lap_status)
                if not lap_first_status:
                    lap_first_status.append(now_s)
                if not lap_ready_first and lap_status.get("source_ready_now") is True:
                    lap_ready_first.append(now_s)

            from sensor_msgs.msg import LaserScan

            def on_scan_seen(m) -> None:
                scan_stamps.append(base._stamp_s(m.header.stamp))
                if (
                    oracle is not None
                    and driving_first
                    and base._stamp_s(m.header.stamp) >= driving_first[0]
                ):
                    # the registered crossing 0: the first /scan stamped at or after the report
                    oracle.arm(base._stamp_s(m.header.stamp))
                if driving_first and not first_scan_after_driving:
                    first_scan_after_driving.append(base._stamp_s(m.header.stamp))

            node.create_subscription(
                LaserScan,
                "/scan",
                on_scan_seen,
                QoSProfile(
                    reliability=ReliabilityPolicy.BEST_EFFORT,
                    history=HistoryPolicy.KEEP_LAST,
                    depth=5,
                ),
            )
            if oracle is not None:
                oracle_pubs["event"] = node.create_publisher(String, TRUTH_EVENT_TOPIC, reliable)
                oracle_pubs["status"] = node.create_publisher(String, TRUTH_STATUS_TOPIC, reliable)
                oracle_status = OracleStatusPublisher(
                    oracle.status,
                    lambda body: oracle_pubs["status"].publish(String(data=json.dumps(body))),
                    ready_after_s=args.lap_truth_ready_after_s,
                )
            node.create_subscription(LapEvent, "/pnp/lap/event", on_lap_event, reliable)
            node.create_subscription(LapGuidance, "/pnp/lap/guidance", on_guidance, reliable)
            node.create_subscription(String, "/pnp/lap/status", on_lap_status, reliable)
    reset_client = node.create_client(Trigger, "/sim/reset")
    initial_pose_pub = node.create_publisher(PoseWithCovarianceStamped, "/initialpose", reliable)

    def publish_initial_pose() -> None:
        if rear_pose[0] is None:
            return
        x, y, yaw = rear_pose[0]
        msg = PoseWithCovarianceStamped()
        msg.header.frame_id = "map"
        msg.header.stamp = node.get_clock().now().to_msg()
        msg.pose.pose.position.x, msg.pose.pose.position.y = x, y
        msg.pose.pose.orientation.z, msg.pose.pose.orientation.w = (
            math.sin(yaw / 2),
            math.cos(yaw / 2),
        )
        msg.pose.covariance[0] = msg.pose.covariance[7] = 0.05**2
        msg.pose.covariance[35] = 0.05**2
        initial_pose_pub.publish(msg)

    def publish_oracle(t: float, x: float, y: float, yaw: float) -> None:
        """Feed the oracle a ground-truth pose and publish the events due (brief 51). Its status
        is not published here: the odometry callback feeds `oracle_status` (brief 51c)."""
        from std_msgs.msg import String

        assert oracle is not None
        for ev in oracle.on_truth_odom(t, x, y, yaw):
            oracle_events.append(
                {
                    "kind": ev.kind,
                    "truth_crossing": ev.truth_crossing,
                    "publish_t": ev.publish_t,
                    **ev.payload,
                }
            )
            oracle_pubs["event"].publish(String(data=json.dumps(ev.payload)))

    stopped_because = "duration"
    # Brief 48: a pnp run starts the bridge and safety_node, resets the simulator, clears the
    # recording and only then starts the perception, the planner and the lap node, so the startup
    # reset can never land on a car that is already moving (the truth recording would begin after
    # crossing 0). Other drivers start every node at once, as before.
    two_phase = args.driver == "pnp"
    reset_done: list[float] = []  # the report's clock (node time) when the startup reset returned
    stall_from = [-math.inf]  # odometry before this stamp starts neither the duration nor the stall

    def request_reset():
        return reset_client.call_async(Trigger.Request())

    def stack_ready() -> bool:
        if not reset_client.service_is_ready():
            return False
        if two_phase:
            return bool(recording.odom)
        return bool(recording.odom and (recording.drive_raw or localised))

    def startup_reset() -> None:
        future = request_reset()
        rclpy.spin_until_future_complete(node, future, timeout_sec=30.0)
        if future.result() is None:
            raise RuntimeError("/sim/reset did not complete")
        reset_done.append(node.get_clock().now().nanoseconds * 1e-9)

    def after_startup_reset() -> None:
        if localised:
            # AMCL starts from the true pose, as an operator's click in Foxglove would, and
            # tracker_node cannot move until map -> odom exists, so the car waits here.
            end = time.monotonic() + 60.0
            next_pub = 0.0
            while time.monotonic() < end:
                if time.monotonic() >= next_pub:
                    publish_initial_pose()
                    next_pub = time.monotonic() + 1.0
                rclpy.spin_once(node, timeout_sec=0.05)
                if tf_buffer.can_transform("map", "base_link", rclpy.time.Time()):
                    break
            else:
                raise RuntimeError("AMCL published no map -> odom within 60 s of /initialpose")
            for _ in range(20):
                rclpy.spin_once(node, timeout_sec=0.05)

    try:
        # a spinning wait for the bridge's odometry and the reset service (the blocking
        # wait_for_service of the earlier runner served no callback while it waited)
        polls = round(base.STARTUP_TIMEOUT_S / 0.05)

        def spin_poll() -> None:
            rclpy.spin_once(node, timeout_sec=0.05)
            if why := died():
                raise RuntimeError(why)

        # clear the recording once the stack is reset, before the rest of a two-phase stack runs
        def clear_recording() -> None:
            after_startup_reset()
            recording.clear()
            loc_errors.clear()
            loc_series.clear()

        def start_decide() -> tuple[str, str]:
            """The start gate of brief 51c, fed by the lap status subscription on the node clock;
            created when the wait begins so that its 30 s count from there."""
            if not gate_box:
                gate_box.append(StartReadyGate(StartGateConfig(), received_s()))
                gate_wall_start.append(time.monotonic())
            if time.monotonic() - gate_wall_start[0] > base.STARTUP_TIMEOUT_S:
                raise RuntimeError(
                    f"lap_memory_node did not answer before the planner was due: the start gate "
                    f"was undecided after {base.STARTUP_TIMEOUT_S:g} s of wall time"
                )
            return gate_box[0].decide(received_s())

        verdict = run_startup(
            stack,
            two_phase,
            spawn,
            stack_ready,
            spin_poll,
            startup_reset,
            clear_recording,
            polls,
            lap_ready=lambda: bool(lap_status),
            start_decide=start_decide
            if args.lap_source == "truth" and oracle is not None
            else None,
            gate_polls=10**9,
        )
        if verdict[0] == "refuse":
            raise StartRefused(verdict[1])
        if two_phase:
            # the rest of the stack is starting: wait until the planner talks (the single phase
            # start waited for this before the reset) and count the duration and the stall from
            # there. The recording keeps everything since the clearing, so crossing 0 is covered.
            deadline = time.monotonic() + base.STARTUP_TIMEOUT_S
            while not recording.drive_raw:
                rclpy.spin_once(node, timeout_sec=0.05)
                if time.monotonic() > deadline or (why := died()):
                    raise RuntimeError(why or f"stack not up after {base.STARTUP_TIMEOUT_S} s")
            stall_from[0] = recording.odom[-1][0]
        pending_reset = None
        initial_pose_until = 0.0
        consumed = 0
        previous = None
        cooldown_until = 0.0
        start_stamp = None
        wall_deadline = time.monotonic() + 3.0 * duration_s + 60.0
        last_check = time.monotonic()
        next_initial = 0.0
        last_motion = None
        while True:
            rclpy.spin_once(node, timeout_sec=0.01)
            # One callback per spin_once: with 50 Hz odometry, scans and commands the runner fell
            # behind by up to half a second and stamped the DRIVING report late (a_T2 CI run). Drain
            # what is ready so the receipt times it records are the arrival times.
            for _ in range(200):
                rclpy.spin_once(node, timeout_sec=0.0)
            if pending_reset is not None and pending_reset.done():
                pending_reset = None
            if (
                localised
                and time.monotonic() < initial_pose_until
                and time.monotonic() >= next_initial
            ):
                publish_initial_pose()
                next_initial = time.monotonic() + 0.4
            while consumed < len(recording.odom):
                sample = recording.odom[consumed]
                consumed += 1
                t, x, y, yaw, v = sample
                if start_stamp is None and t >= stall_from[0]:
                    start_stamp = t
                if previous is not None and bm.is_teleport(previous[1:3], (x, y)):
                    lap_tracker.reset()
                    tracker_resets.append(round(t, 3))
                    cooldown_until = 0.0
                    if localised:
                        initial_pose_until = time.monotonic() + 3.0
                elif previous is not None and t >= cooldown_until:
                    reason = bm.gym_contact_signature(previous[3], yaw, previous[4], v)
                    if (
                        reason is None
                        and consumed % base.CLEARANCE_CHECK_STRIDE == 0
                        and bm.body_clearance_m(
                            x, y, yaw, box, synthetic.walls, synthetic.wall_thickness_m
                        )
                        <= 0.0
                    ):
                        reason = "body_overlaps_wall"
                    if reason is not None:
                        recording.contacts.append((t, x, y, reason))
                        cooldown_until = t + base.CONTACT_COOLDOWN_S
                        if pending_reset is None:
                            pending_reset = request_reset()
                if t >= cooldown_until:
                    lap_tracker.update(x, y, t)
                if oracle is not None:
                    publish_oracle(t, x, y, yaw)
                if abs(v) > 0.05 or (last_motion is None and t >= stall_from[0]):
                    last_motion = t
                previous = sample
            if start_stamp is not None and previous is not None:
                if previous[0] - start_stamp >= duration_s:
                    break
                if args.stop_after_laps and lap_tracker.laps >= args.stop_after_laps:
                    stopped_because = "laps"
                    break
                if args.stop_after_contacts and len(recording.contacts) >= args.stop_after_contacts:
                    stopped_because = "contacts"
                    break
                if (
                    args.stall_s
                    and last_motion is not None
                    and previous[0] - last_motion > args.stall_s
                ):
                    stopped_because = "stalled"
                    break
            if time.monotonic() > wall_deadline:
                stopped_because = "wall_timeout"
                break
            if time.monotonic() - last_check > 1.0:
                last_check = time.monotonic()
                if why := died():
                    raise RuntimeError(why)
        if args.driver == "pnp" and args.lap_memory != "none":
            # the system's last LapEvent follows the ground-truth crossing by a scan or two
            grace_end = time.monotonic() + args.grace_s
            while time.monotonic() < grace_end:
                rclpy.spin_once(node, timeout_sec=0.01)
        metrics = bm.compute_metrics(
            recording,
            box=box,
            walls=synthetic.walls,
            lap_tracker=lap_tracker,
            wall_thickness_m=synthetic.wall_thickness_m,
        )
        result.update(metrics)
        result["stopped_because"] = stopped_because
        result["gt_tracker_resets_s"] = tracker_resets
        if args.driver == "tracker_policy":
            for _ in range(150):  # let the 1 Hz status of the last second arrive
                rclpy.spin_once(node, timeout_sec=0.01)
            result["policy_status"] = dict(policy_status)
            # CLAUDE.md invariant 1 / 04-architecture.md: policy_node is the only publisher of
            # /drive_raw in a policy run, safety_node the only publisher of /drive.
            result["publishers"] = {
                "/drive_raw": node.count_publishers("/drive_raw"),
                "/drive": node.count_publishers("/drive"),
                "/tracker/command": node.count_publishers("/tracker/command"),
            }
            result["policy_events"] = list(policy_events)
        if args.driver == "pnp":
            result["planner_states"] = {
                str(s): planner_states.count(s) for s in set(planner_states)
            }
            result["start_missing"] = {
                str(m): start_missing.count(m) for m in sorted(set(start_missing))
            }
            # Every /safety/events PHASE_ENGAGE with SEVERITY_BRAKE, whatever gate raised it.
            result["safety_brakes"] = sum(1 for e in recording.events if e[2] == 0 and e[3] == 2)
            # Every engagement of safety_node's rate limit: the planner's own output rate limit
            # is meant to keep it inside safety_node's, so a pass has none.
            result["rate_limit_events"] = sum(
                1 for e in recording.events if e[1] == "rate_limit" and e[2] == 0
            )
            result.update(pnp_timing_summary(recording, planner_status))
            # Brief 40: confidence, cap limiting and planned clearance, no threshold.
            if args.count_innovation:
                counts = work / "innovation.json"
                result["innovation_rejections"] = (
                    json.loads(counts.read_text()) if counts.exists() else {"error": "no file"}
                )
            result["pnp_ticks"] = pts.tick_summary(planner_ticks)
            result["obstacle_passes"] = pts.obstacle_passes(
                planner_ticks,
                recording.odom,
                [(o.centre_x, o.centre_y, o.length_m) for o in synthetic.obstacles],
            )
            # The command path: only the planner on /drive_raw, only safety_node on /drive.
            result["publisher_nodes"] = {
                topic: sorted(i.node_name for i in node.get_publishers_info_by_topic(topic))
                for topic in ("/drive_raw", "/drive")
            }
            if args.lap_memory != "none":
                result["guidance_floor"] = dict(guidance_floor)
                driving_receipt = driving_first[0] if driving_first else None
                truth = registered_truth(
                    recording.odom,
                    scan_stamps,
                    driving_receipt,
                    (lap_tracker._xs, lap_tracker._ys),
                    reset_done[0] if reset_done else None,
                )
                result.update(
                    lap_results(
                        lap_tracker,
                        recording.events,
                        lap_events,
                        guidance_log,
                        lap_status,
                        driving_receipt,
                        first_scan_at_or_after(scan_stamps, driving_receipt),
                        rescore_from_start_scan(
                            recording.odom, lap_tracker._xs, lap_tracker._ys, lap_events
                        ),
                        truth,
                    )
                )
                result["startup_reset_done_s"] = reset_done[0] if reset_done else None
                # brief 53 (battery B11): per truth lap clearance and speed command, the longest
                # stop, the published bound per lap and station
                c0 = result.get("gt_lap1_start_s")
                crossings = result.get("gt_lap_crossings_s") or []
                result["min_body_clearance_by_lap_m"] = dv1_measures.clearance_by_lap(
                    recording.odom,
                    c0,
                    crossings,
                    lambda x, y, yaw: bm.body_clearance_m(
                        x, y, yaw, box, synthetic.walls, synthetic.wall_thickness_m
                    ),
                    stride=base.CLEARANCE_CHECK_STRIDE,
                )
                by_lap, top = dv1_measures.max_command_by_lap(recording.drive_raw, c0, crossings)
                result["max_cmd_speed_by_lap_mps"] = by_lap
                result["max_cmd_speed_mps"] = top
                result["longest_stop_s"] = dv1_measures.longest_stop_s(
                    recording.odom, c0, crossings[-1] if crossings else None
                )
                result["bound_by_lap"] = bound_log.to_json()
                # brief 51c: the start of the run, so a start failure is explained by the report
                result["start_trace"] = start_trace(
                    lap_node_first_status_s=lap_first_status[0] if lap_first_status else None,
                    ready_first_s=lap_ready_first[0] if lap_ready_first else None,
                    gate_start_s=_gate_trace(gate_box).get("start_s"),
                    planner_spawn_s=planner_spawned[0] if planner_spawned else None,
                    driving_first_s=driving_receipt,
                    lap1_start_s=lap_status.get("lap1_start_stamp_s"),
                    oracle_first_status_s=oracle_status.first_publish_s if oracle_status else None,
                    gate_verdict=_gate_trace(gate_box).get("verdict"),
                    gate_reason=_gate_trace(gate_box).get("reason"),
                )
                result["source_ready"] = lap_status.get("source_ready")
                result["guidance_cap_series"] = [[round(a, 3), b] for a, b in cap_series]
                if oracle is not None:
                    result["oracle_events"] = oracle_events
                    result["oracle_truth_crossings_s"] = list(oracle.truth_crossings)
                    result["oracle_start_s"] = oracle.truth_start_t
                    result["lap_truth_faults"] = json.loads(args.lap_truth_faults_json)
        t_first = recording.odom[0][0] if recording.odom else 0.0
        write_event_trace(args, work, trace, t_first)
        if os.environ.get("RUNNER_TRACE_NPZ"):
            # Diagnostics only: the command stream and the events, for timing analysis offline.
            np.savez(
                os.environ["RUNNER_TRACE_NPZ"],
                drive_raw=np.array(recording.drive_raw),
                drive=np.array(recording.drive),
                events=np.array([(e[0], e[2], e[3]) for e in recording.events]),
                event_sources=np.array([e[1] for e in recording.events]),
                t_first=t_first,
            )
        result["safety_events"] = [
            {**e, "t_s": round(e["t_s"] - t_first, 2)} for e in event_details
        ]
        if loc_errors:
            err = np.array(loc_errors)
            result["localisation_error_m"] = {
                "mean": round(float(err.mean()), 4),
                "p95": round(float(np.percentile(err, 95)), 4),
                "max": round(float(err.max()), 4),
            }
            ser = np.array(loc_series)
            result["localisation_error_split"] = {
                "along_mean_m": round(float(ser[:, 1].mean()), 4),
                "along_p95_abs_m": round(float(np.percentile(np.abs(ser[:, 1]), 95)), 4),
                "cross_mean_m": round(float(ser[:, 2].mean()), 4),
                "cross_p95_abs_m": round(float(np.percentile(np.abs(ser[:, 2]), 95)), 4),
                "heading_p95_abs_rad": round(float(np.percentile(np.abs(ser[:, 3]), 95)), 4),
            }
        if args.trace_out:
            np.savez_compressed(
                args.trace_out,
                odom=np.array(recording.odom),
                drive_raw=np.array(recording.drive_raw),
                drive=np.array(recording.drive),
                loc=np.array(loc_series),
                events=np.array([e["t_s"] for e in event_details]),
            )
    except StartRefused as refusal:
        # not an error: the starter refused and the planner was never started (brief 51c)
        trace_of_gate = _gate_trace(gate_box)
        try:
            result.update(
                bm.compute_metrics(
                    recording,
                    box=box,
                    walls=synthetic.walls,
                    lap_tracker=lap_tracker,
                    wall_thickness_m=synthetic.wall_thickness_m,
                )
            )
        except ValueError as exc:  # the bridge never published: say so, it is an error
            result["error"] = str(exc)
        result.update(
            refusal_result_keys(
                ("refuse", refusal.reason),
                trace_of_gate,
                trace_of_gate.get("refuse_s") or received_s(),
                len(recording.drive_raw),
            )
        )
        result["start_trace"] = start_trace(
            lap_node_first_status_s=lap_first_status[0] if lap_first_status else None,
            ready_first_s=lap_ready_first[0] if lap_ready_first else None,
            planner_spawn_s=planner_spawned[0] if planner_spawned else None,
            oracle_first_status_s=oracle_status.first_publish_s if oracle_status else None,
            gate_verdict=trace_of_gate.get("verdict"),
            gate_reason=trace_of_gate.get("reason"),
        )
        result["start_trace"]["gate"] = trace_of_gate
        result["lap_status"] = dict(lap_status)
    except RuntimeError as e:
        result["error"] = str(e)
    finally:
        node.destroy_node()
        rclpy.shutdown()
        base._stop([p for _, p in processes])
    return result


def build_parser() -> argparse.ArgumentParser:
    """The runner's argument parser (importable, so a battery command can be parsed in a test)."""
    parser = argparse.ArgumentParser(
        description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter
    )
    parser.add_argument("--track", required=True, choices=TRACKS)
    parser.add_argument("--track-params-json", default="{}")
    parser.add_argument("--reverse", action="store_true", help="drive the loop clockwise")
    parser.add_argument("--driver", default="gap_follow", choices=DRIVERS)
    parser.add_argument("--policy-dir", help="policy directory (driver tracker_policy)")
    parser.add_argument("--perception-bundle", default="", help="track_perception bundle (pnp)")
    parser.add_argument("--dynamics-bundle", default="", help="dynamics bundle (pnp)")
    parser.add_argument(
        "--plant-bundle",
        default="",
        help="dynamics bundle whose io.nominal sets the sim car's actuation (steer gain, speed "
        "gain, accel and decel limits, and speed_deadband_mps when it has one); default: the "
        "gym's own plant",
    )
    parser.add_argument(
        "--plant-speed-deadband",
        type=float,
        default=None,
        help="a /drive speed under this magnitude (m/s) leaves the car standing; overrides "
        "--plant-bundle",
    )
    parser.add_argument(
        "--plant-steer-tau",
        type=float,
        default=None,
        help="first order lag of the wheel angle (s); overrides --plant-bundle",
    )
    parser.add_argument(
        "--plant-command-delay",
        type=float,
        default=None,
        help="the /drive command acts this long after it arrives (s); overrides --plant-bundle",
    )
    parser.add_argument(
        "--no-perception",
        action="store_true",
        help="driver pnp: start the planner without track_perception_node (the scan-only fault "
        "case: no track ever arrives)",
    )
    parser.add_argument(
        "--backend",
        default="geometric",
        choices=("net", "geometric", "oracle"),
        help="oracle: tests/l5_pnp_oracle (ground truth LocalTrack, measurement M-A2 only)",
    )
    parser.add_argument(
        "--count-innovation",
        action="store_true",
        help="pnp with a lap memory: start the lap node through tests/l5_pnp_laps/"
        "lap_node_counting.py, which counts the innovation gate's rejected crossing candidates per "
        "lap (reported, no threshold)",
    )
    parser.add_argument("--v-max-mps", type=float, default=0.6, help="pnp planner speed cap")
    parser.add_argument(
        "--lap-memory",
        default="none",
        choices=("none", "off", "v0", "v1"),
        help="driver pnp: none = no lap_memory_node; off = the node observes only (the planner "
        "ignores its guidance); v0 = cap schedule; v1 = station set. --v-max-mps is then the "
        "ceiling",
    )
    parser.add_argument(
        "--lap-source",
        default="start_scan",
        choices=("start_scan", "map_pose", "truth"),
        help="lap source of the lap memory (map_pose needs a map -> base_link transform, which "
        "this runner does not provide; truth is the simulator's oracle of brief 51, with "
        "--sim-time: it publishes the ground-truth line crossing on /sim/lap/truth_event)",
    )
    parser.add_argument(
        "--lap-truth-faults-json",
        default="{}",
        help="lap source truth: the fault plan (the yaml's lap_battery_b9.fault_plan as JSON: "
        "drop_truth_crossings, false_event, duplicate_truth_crossing); {} is the clean oracle",
    )
    parser.add_argument(
        "--lap-truth-ready-after-s",
        type=ready_delay,
        default=0.0,
        help="lap source truth (brief 51c): until this many seconds of sim time after the runner "
        "receives the lap node's first status the oracle's status says `waiting`; `inf` never "
        "becomes ready. 0 (default): as before. How the battery exercises the start gate",
    )
    parser.add_argument(
        "--v-lap1-mps",
        type=float,
        default=None,
        help="cap of lap 1 (default: --v-max-mps); the planner's v_no_guidance_mps",
    )
    parser.add_argument(
        "--lap-zone-merge",
        action=argparse.BooleanOptionalAction,
        default=True,
        help="D v1: merge a flickering state into one caution zone (--no-lap-zone-merge: the "
        "variant of pre-registration v0.2)",
    )
    parser.add_argument(
        "--lap-bound-floor-mps",
        type=float,
        default=None,
        help="D v1: no speed bound below this (default: the node's v_bound_floor_mps)",
    )
    parser.add_argument(
        "--lap-params-json", default="{}", help="lap_memory_node parameters on top (tuning)"
    )
    parser.add_argument(
        "--grace-s",
        type=float,
        default=1.5,
        help="with a lap memory: keep listening this long after the run ends so the system's "
        "last LapEvent arrives (s)",
    )
    parser.add_argument(
        "--planner-params-json",
        default="{}",
        help="driver pnp: mppi_planner_node parameters on top of pnp.launch.py's (debug_stall_profile, "
        "start_timeout_s, ...)",
    )
    parser.add_argument(
        "--wall-clock",
        action="store_true",
        help="driver pnp: start wall_clock_node.py (publishes /clock from the system clock) and "
        "give the planner use_sim_time:=true, which its debug_stall_profile requires",
    )
    parser.add_argument(
        "--sim-time",
        action="store_true",
        help="driver pnp: run on simulated time (bridge_node sim_time, /clock, use_sim_time:=true "
        "on every node, lockstep on /pnp/local_track), independent of CPU scheduling",
    )
    parser.add_argument("--profile", default="lane-2026-10-25", help="gap_follow profile")
    parser.add_argument("--params-json", default="{}", help="gap_follow parameters on top")
    parser.add_argument("--tracker-params-json", default="{}", help="tracker_node parameters")
    parser.add_argument(
        "--raceline-json",
        default="{}",
        help="tools/track_session pipeline.raceline_from_map options (margin, v_max_mps, ...)",
    )
    parser.add_argument("--car", default="params", choices=CARS)
    parser.add_argument("--lidar", default="mounted", choices=("mounted", "aligned"))
    parser.add_argument("--noise", action=argparse.BooleanOptionalAction, default=True)
    parser.add_argument(
        "--noise-set",
        action="append",
        default=[],
        help="override one bridge sensor noise parameter on a localised run, name=value "
        "(wheel_speed_scale=1.0, steering_offset_rad=0 ...); repeatable",
    )
    parser.add_argument(
        "--clearances",
        help="body clearance candidates for the raceline, widest first, comma separated "
        "(overrides clearance_candidates in --raceline-json)",
    )
    parser.add_argument("--amcl-params", help="amcl_params_file for a localised run (tuning)")
    parser.add_argument("--ekf-params", help="ekf_params_file for a localised run (tuning)")
    parser.add_argument("--duration", type=float, default=150.0, help="run time cap (s)")
    parser.add_argument("--stop-after-laps", type=int, default=3)
    parser.add_argument("--stop-after-contacts", type=int, default=0)
    parser.add_argument(
        "--stall-s",
        type=float,
        default=12.0,
        help="end the run once the car has been still this long (s), e.g. latched by safety_node "
        "with a driver that has no way out; 0 = never",
    )
    parser.add_argument("--seed", type=int, default=base.SEED)
    parser.add_argument("--domain-id", type=int, default=int(os.environ.get("ROS_DOMAIN_ID", "0")))
    parser.add_argument("--work-dir", type=Path, help="keep the generated map, raceline and logs")
    parser.add_argument("--out", type=Path)
    parser.add_argument("--trace-out", type=Path)
    return parser


def main(argv: list[str] | None = None) -> int:
    args = build_parser().parse_args(argv)
    result = run_benchmark(args)
    text = json.dumps(result, indent=2, sort_keys=True, default=str)
    print(text)
    if args.out:
        args.out.write_text(text + "\n")
    return 3 if "error" in result else 0


if __name__ == "__main__":
    sys.exit(main())
