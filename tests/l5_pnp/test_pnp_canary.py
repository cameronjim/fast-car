"""L5 canary: the Plug-and-Play driver laps tracks through the real command path.

Each case runs tests/l5_reactive_bench/track_family_runner.py with driver pnp: bridge_node on the
track with the car's LiDAR (mounted at the vehicle_params pose, 720 beams over 360 degrees,
10 Hz, 2 cm range noise) and a 50 Hz /odom/wheel, then track_perception_node,
mppi_planner_node and safety_node, nothing remapped. No map, no pose and no raceline: the driver
sees only the scan and the wheel speed.

Cases, each 3 laps at 0.6 m/s, car `params`, seed 11:
  stadium (lane 1.0 m, straight 2.0 m, island radius 0.4 m)   ccw and cw   geometric backend
  rounded rectangle (lane 1.0 m, 3 x 2 m island, radius 0.4 m)   ccw        geometric backend
  stadium, ccw, net backend with the released bundle from racer_track_perception's fixtures
      (skipped, with the reason, when that fixture directory is absent)

Pass criteria per case, never loosened to get green: 3 laps, 0 wall contacts, at most 3 safety
brake engagements (/safety/events PHASE_ENGAGE with SEVERITY_BRAKE), no rate_limit event from
safety_node (the planner's output rate limit stays inside it), the only publisher on
/drive_raw is mppi_planner_node, the only publisher on /drive is safety_node, and no STATE_FAULT in
/pnp/planner/status. Lap times and the brake count go to the test output. Skipped without a
sourced ROS workspace.
"""

from __future__ import annotations

import json
import math
import os
import random
import shutil
import statistics
import subprocess
import sys
from pathlib import Path

import pytest
import yaml

pytest.importorskip("rclpy")
pytest.importorskip("f1tenth_gym")

REPO_ROOT = Path(__file__).resolve().parents[2]
sys.path.insert(0, str(REPO_ROOT / "tests" / "l5_reactive_bench"))
import starvation
from starvation import MAX_ATTEMPTS

RUNNER = REPO_ROOT / "tests" / "l5_reactive_bench" / "track_family_runner.py"
DYNAMICS_BUNDLE = REPO_ROOT / "config" / "pnp" / "fixtures" / "dyn-nominal-v0"
# The planner's model: the fitted car fixture, which carries the speed dead zone (0.22 m/s), so the
# planner's speed floor sits above the dead zone the car-like plant has. The sim plant itself
# keeps the nominal fixture's actuation (DYNAMICS_BUNDLE) with its own 0.20 m/s dead zone.
PLANNER_BUNDLE = REPO_ROOT / "config" / "pnp" / "fixtures" / "dyn-car-2026-11-12"
PERCEPTION_BUNDLE = (
    REPO_ROOT / "ros_ws" / "src" / "racer_track_perception" / "test" / "fixtures" / "bundle_release"
)
LAPS = 3
V_MAX_MPS = 0.6
SEED = 11
MAX_SAFETY_BRAKES = 3
STATE_FAULT = 6

# A run that fails and shows a gap in the planner's /drive_raw or in safety_node's /drive was starved
# by the machine and is run again; see tests/l5_reactive_bench/starvation.py for the argument and
# the one place the rule lives.
TIMING_MAX_ATTEMPTS = MAX_ATTEMPTS


def run_judged(command, out, report, judge, capsys, timeout_s=500):
    """Run the runner and judge its result, running it again when a failing attempt was starved."""

    def run_once():
        out.unlink(missing_ok=True)
        proc = subprocess.run(
            command,
            env=dict(os.environ),
            capture_output=True,
            text=True,
            timeout=timeout_s,
            check=False,
        )
        assert out.exists(), proc.stdout[-2000:] + proc.stderr[-2000:]
        result = json.loads(out.read_text())
        assert "error" not in result, result["error"]
        return result

    def say(line):
        with capsys.disabled():
            print(line)

    return starvation.run_judged(run_once, judge, report, say)


STADIUM = (
    "stadium",
    {"lane_width_m": 1.0, "straight_length_m": 2.0, "island_radius_m": 0.4},
)
ROUNDED_RECT = (
    "rounded_rect",
    {
        "lane_width_m": 1.0,
        "straight_length_m": 3.0,
        "island_width_m": 2.0,
        "island_radius_m": 0.4,
    },
)

CASES = [
    pytest.param(STADIUM, False, "geometric", id="stadium-ccw-geometric"),
    pytest.param(STADIUM, True, "geometric", id="stadium-cw-geometric"),
    pytest.param(ROUNDED_RECT, False, "geometric", id="rounded_rect-ccw-geometric"),
    pytest.param(STADIUM, False, "net", id="stadium-ccw-net"),
]


@pytest.mark.parametrize(("track", "reverse", "backend"), CASES)
def test_pnp_laps_the_track_through_the_real_command_path(
    track, reverse, backend, tmp_path, capsys
):
    if backend == "net" and not (PERCEPTION_BUNDLE / "manifest.yaml").is_file():
        pytest.skip(f"released perception bundle not committed yet: {PERCEPTION_BUNDLE}")
    assert (DYNAMICS_BUNDLE / "manifest.yaml").is_file(), DYNAMICS_BUNDLE
    assert (PLANNER_BUNDLE / "manifest.yaml").is_file(), PLANNER_BUNDLE
    name, params = track
    out = tmp_path / "result.json"
    command = [
        sys.executable,
        str(RUNNER),
        "--track",
        name,
        "--track-params-json",
        json.dumps(params),
        "--driver",
        "pnp",
        "--backend",
        backend,
        "--dynamics-bundle",
        str(PLANNER_BUNDLE),
        "--v-max-mps",
        str(V_MAX_MPS),
        "--car",
        "params",
        "--seed",
        str(SEED),
        "--stop-after-laps",
        str(LAPS),
        "--sim-time",
        "--duration",
        "150",
        "--domain-id",
        "81",
        "--work-dir",
        str(tmp_path / "work"),
        "--out",
        str(out),
    ]
    if backend == "net":
        command += ["--perception-bundle", str(PERCEPTION_BUNDLE)]
    if reverse:
        command.append("--reverse")

    def report(result):
        return (
            f"\nl5_pnp {name} {'cw' if reverse else 'ccw'} {backend}: "
            f"laps {result['laps']} lap_times_s {result['lap_times_s']} "
            f"safety_brakes {result['safety_brakes']} "
            f"rate_limit_events {result['rate_limit_events']} "
            f"wall_contacts {result['wall_contacts']} "
            f"min_clearance_m {result['min_body_clearance_m']} "
            f"planner_states {result['planner_states']} "
            f"start_missing {result.get('start_missing')}"
        )

    def judge(result):
        assert result["laps"] >= LAPS, result.get("stopped_because")
        assert result["wall_contacts"] == 0
        assert result["safety_brakes"] <= MAX_SAFETY_BRAKES
        assert result["rate_limit_events"] == 0, (
            "safety_node rate-limited the planner's command: "
            f"{[e for e in result['safety_events'] if e['source'] == 'rate_limit']}"
        )
        assert result["publisher_nodes"] == {
            "/drive_raw": ["mppi_planner_node"],
            "/drive": ["safety_node"],
        }
        assert str(STATE_FAULT) not in result["planner_states"], result["planner_states"]

    run_judged(command, out, report, judge, capsys)


# --- the car-like plant (2026-11-12) ----------------------------------------------------------
# The cases above run on the gym's own plant, which is nicer than the car: no speed dead zone and
# no steering lag. These run the same driver, perception and planner against a plant like the
# car's: the dynamics bundle's actuation (gains, accel and decel limits), a 0.20 m/s speed dead
# zone (a /drive speed under it leaves the car standing) and a 0.10 s steering lag. The planner's
# model does not know about either of the last two, which is the situation on the car. Stadium
# with island radius 0.4 m and straight 2.0 m, lane 1.0 m and 1.3 m, both ways, geometric
# backend, 0.5 m/s, and the scan-only fault case. Same pass criteria as above.
#
# Measured on the v0 driver (main at 8766bec, 2026-11-12, car `params`, seed 11), per case:
#   lane 1.0 ccw  passes in 6 of 7 runs (0 to 2 safety brakes); the seventh stalled after one lap
#                 (94 brakes, BLOCKED and NO_ODOM)
#   lane 1.0 cw, lane 1.3 ccw, lane 1.3 cw   3 laps, 0 brakes, 0 contacts
#   lane 1.3 ccw with --no-perception: 0 laps, planner_states NO_TRACK about 600 ticks (and NO_ODOM
#                 at the start), stalled; this case was a strict xfail until the planner drove on
#                 the scan alone
# With the planner v0.1 of main (free-space field, no NO_TRACK state) all five pass: 3 laps,
# 0 brakes, 0 contacts, laps of about 20 s (about 28 s for the scan-only case).
CAR_LIKE_V_MAX_MPS = 0.5
CAR_LIKE_PLANT = [
    "--plant-bundle",
    str(DYNAMICS_BUNDLE),
    "--plant-speed-deadband",
    "0.20",
    "--plant-steer-tau",
    "0.10",
]


def stadium(lane_width_m: float) -> tuple[str, dict]:
    return (
        "stadium",
        {"lane_width_m": lane_width_m, "straight_length_m": 2.0, "island_radius_m": 0.4},
    )


CAR_LIKE_CASES = [
    pytest.param(stadium(1.0), False, True, id="car-like-stadium-lane1.0-ccw"),
    pytest.param(stadium(1.0), True, True, id="car-like-stadium-lane1.0-cw"),
    pytest.param(stadium(1.3), False, True, id="car-like-stadium-lane1.3-ccw"),
    pytest.param(stadium(1.3), True, True, id="car-like-stadium-lane1.3-cw"),
    pytest.param(stadium(1.3), False, False, id="car-like-stadium-lane1.3-ccw-no-perception"),
]


@pytest.mark.parametrize(("track", "reverse", "perception"), CAR_LIKE_CASES)
def test_pnp_laps_with_the_car_like_plant(track, reverse, perception, tmp_path, capsys):
    assert (DYNAMICS_BUNDLE / "manifest.yaml").is_file(), DYNAMICS_BUNDLE
    assert (PLANNER_BUNDLE / "manifest.yaml").is_file(), PLANNER_BUNDLE
    name, params = track
    out = tmp_path / "result.json"
    command = [
        sys.executable,
        str(RUNNER),
        "--track",
        name,
        "--track-params-json",
        json.dumps(params),
        "--driver",
        "pnp",
        "--backend",
        "geometric",
        "--dynamics-bundle",
        str(PLANNER_BUNDLE),
        "--v-max-mps",
        str(CAR_LIKE_V_MAX_MPS),
        "--car",
        "params",
        *CAR_LIKE_PLANT,
        "--seed",
        str(SEED),
        "--stop-after-laps",
        str(LAPS),
        "--sim-time",
        "--duration",
        "150",
        "--domain-id",
        "82",
        "--work-dir",
        str(tmp_path / "work"),
        "--out",
        str(out),
    ]
    if not perception:
        command.append("--no-perception")
    if reverse:
        command.append("--reverse")

    def report(result):
        return (
            f"\nl5_pnp car-like {name} lane {params['lane_width_m']} "
            f"{'cw' if reverse else 'ccw'}{'' if perception else ' no-perception'}: "
            f"laps {result['laps']} lap_times_s {result['lap_times_s']} "
            f"safety_brakes {result['safety_brakes']} "
            f"rate_limit_events {result['rate_limit_events']} "
            f"wall_contacts {result['wall_contacts']} "
            f"min_clearance_m {result['min_body_clearance_m']} "
            f"planner_states {result['planner_states']} "
            f"start_missing {result.get('start_missing')}"
        )

    def judge(result):
        assert result["laps"] >= LAPS, result.get("stopped_because")
        assert result["wall_contacts"] == 0
        assert result["safety_brakes"] <= MAX_SAFETY_BRAKES
        assert result["rate_limit_events"] == 0, (
            "safety_node rate-limited the planner's command: "
            f"{[e for e in result['safety_events'] if e['source'] == 'rate_limit']}"
        )
        assert result["publisher_nodes"] == {
            "/drive_raw": ["mppi_planner_node"],
            "/drive": ["safety_node"],
        }
        assert str(STATE_FAULT) not in result["planner_states"], result["planner_states"]

    run_judged(command, out, report, judge, capsys)


# --- step contract 2 (brief 43) ---------------------------------------------------------------
# The same car-like stadium run with the planner on a contract 2 bundle exported for the sim plant
# (config/pnp/fixtures/dyn-car-2026-11-12-contract2-sim: the car bundle's nominal, ground table 1,
# learned block off; the sim plant moves by wheel speed, so ground over wheel distance is 1).
# Next to the 1.x run above (the CI default stays the 1.x bundle): 3 laps, 0 wall contacts, same
# criteria. Nothing selects this bundle but this test.
CONTRACT2_PLANNER_BUNDLE = (
    REPO_ROOT / "config" / "pnp" / "fixtures" / "dyn-car-2026-11-12-contract2-sim"
)


@pytest.mark.parametrize("reverse", [False, True], ids=["ccw", "cw"])
def test_pnp_laps_with_a_contract_2_planner_bundle(reverse, tmp_path, capsys):
    assert (CONTRACT2_PLANNER_BUNDLE / "manifest.yaml").is_file(), CONTRACT2_PLANNER_BUNDLE
    name, params = stadium(1.0)
    out = tmp_path / "result.json"
    command = [
        sys.executable,
        str(RUNNER),
        "--track",
        name,
        "--track-params-json",
        json.dumps(params),
        "--driver",
        "pnp",
        "--backend",
        "geometric",
        "--dynamics-bundle",
        str(CONTRACT2_PLANNER_BUNDLE),
        "--v-max-mps",
        str(CAR_LIKE_V_MAX_MPS),
        "--car",
        "params",
        *CAR_LIKE_PLANT,
        "--seed",
        str(SEED),
        "--stop-after-laps",
        str(LAPS),
        "--sim-time",
        "--duration",
        "150",
        "--domain-id",
        "83",
        "--work-dir",
        str(tmp_path / "work"),
        "--out",
        str(out),
    ]
    if reverse:
        command.append("--reverse")

    def report(result):
        return (
            f"\nl5_pnp contract 2 stadium lane 1.0 {'cw' if reverse else 'ccw'}: "
            f"laps {result['laps']} lap_times_s {result['lap_times_s']} "
            f"safety_brakes {result['safety_brakes']} wall_contacts {result['wall_contacts']} "
            f"rate_limit_events {result['rate_limit_events']} "
            f"min_clearance_m {result['min_body_clearance_m']} "
            f"planner_states {result['planner_states']} "
            f"start_missing {result.get('start_missing')}"
        )

    def judge(result):
        assert result["laps"] >= LAPS, result.get("stopped_because")
        assert result["wall_contacts"] == 0
        assert result["safety_brakes"] <= MAX_SAFETY_BRAKES
        assert result["rate_limit_events"] == 0
        assert result["publisher_nodes"] == {
            "/drive_raw": ["mppi_planner_node"],
            "/drive": ["safety_node"],
        }
        assert str(STATE_FAULT) not in result["planner_states"], result["planner_states"]

    run_judged(command, out, report, judge, capsys)


# --- planner timing (2026-11-13 night, design 14.7, pre-registration 4.1) ----------------------
# The planner's output timing against the real safety_node, on the car-like plant: no rate_limit
# engagement, no watchdog engagement between the first non-zero /drive_raw command and the last
# planner status, no speed command jump after a stop. The same cases with stalls injected into the
# planner's SOLVE thread (debug_stall_profile, a 400 ms stall in the first second after DRIVING,
# then 40 to 90 ms stalls at about 0.2 per second, seeded): the publisher thread keeps /drive_raw
# coming, a plan older than 0.12 s stops the car and the ramp restarts from zero, so at most one
# watchdog engagement per run and the same slew behaviour. The stall profile needs the planner on
# use_sim_time, which the runner gives it with --wall-clock (a /clock from the system clock).
#
# Per push: stadium lane 1.0 m both ways at 0.6 and 1.0 m/s, with and without stalls, 3 laps.
# The full matrix (stadium lane 1.0 and 1.3 m and the rounded rectangle, both ways, 0.6 and 1.0 m/s
# and the planner bundle's validity ceiling, with and without stalls, 10 laps) runs only with
# RACER_L5_PNP_FULL=1: about 36 runs of one to four minutes each.
TIMING_SEED = 13
TIMING_STALL_SEED = 2026
RATE_FRACTION = 0.45  # the planner's rate_limit_fraction (its default; the cases leave it alone)
# Runs in parallel containers on one docker network see each other's DDS traffic unless their
# domain ids differ: RACER_L5_PNP_DOMAIN_OFFSET shifts the ids of the timing cases.
DOMAIN_OFFSET = int(os.environ.get("RACER_L5_PNP_DOMAIN_OFFSET", "0"))
FULL = os.environ.get("RACER_L5_PNP_FULL") == "1"
FULL_SKIP = pytest.mark.skipif(
    not FULL,
    reason="the full planner timing matrix (about 36 runs of 10 laps) runs only with "
    "RACER_L5_PNP_FULL=1",
)


def bundle_ceiling_mps() -> float:
    manifest = yaml.safe_load((PLANNER_BUNDLE / "manifest.yaml").read_text())
    return float(manifest["io"]["validity"]["v_max_mps"])


def stall_profile(seed: int, horizon_s: float = 600.0) -> str:
    """'T:D,...': T seconds after the first DRIVING the solve thread sleeps D seconds. One stall
    of 400 ms at 0.5 s, then 40 to 90 ms stalls with exponential gaps of mean 5 s (0.2 per s)."""
    rng = random.Random(seed)
    items = [(0.5, 0.4)]
    t = 0.5
    while t < horizon_s:
        t += max(1.0, rng.expovariate(0.2))
        items.append((t, rng.uniform(0.04, 0.09)))
    return ",".join(f"{a:.3f}:{b:.3f}" for a, b in items)


def timing_id(track, reverse, v_max, stalls, laps) -> str:
    name, params = track
    return (
        f"{name}-lane{params['lane_width_m']}-{'cw' if reverse else 'ccw'}-v{v_max}"
        f"-{'stalls' if stalls else 'nostalls'}-{laps}laps"
    )


TIMING_PUSH_CASES = [
    pytest.param(
        stadium(1.0), reverse, v, stalls, 3, id=timing_id(stadium(1.0), reverse, v, stalls, 3)
    )
    for v in (0.6, 1.0)
    for stalls in (False, True)
    for reverse in (False, True)
]
TIMING_FULL_CASES = [
    pytest.param(
        track, reverse, v, stalls, 10, id=timing_id(track, reverse, v, stalls, 10), marks=FULL_SKIP
    )
    for track in (stadium(1.0), ROUNDED_RECT, stadium(1.3))
    for v in (0.6, 1.0, "ceiling")
    for stalls in (False, True)
    for reverse in (False, True)
]


def run_timing_case(track, reverse, v_max, stalls, laps, tmp_path, capsys, domain_id):
    if v_max == "ceiling":
        v_max = bundle_ceiling_mps()
    name, params = track
    out = tmp_path / "result.json"
    command = [
        sys.executable,
        str(RUNNER),
        "--track",
        name,
        "--track-params-json",
        json.dumps(params),
        "--driver",
        "pnp",
        "--backend",
        "geometric",
        "--dynamics-bundle",
        str(PLANNER_BUNDLE),
        "--v-max-mps",
        str(v_max),
        "--car",
        "params",
        *CAR_LIKE_PLANT,
        "--seed",
        str(TIMING_SEED),
        "--stop-after-laps",
        str(laps),
        "--duration",
        str(60 + 40 * laps),
        "--domain-id",
        str(domain_id),
        "--work-dir",
        str(tmp_path / "work"),
        "--out",
        str(out),
    ]
    if stalls:
        command += [
            "--wall-clock",
            "--planner-params-json",
            json.dumps({"debug_stall_profile": stall_profile(TIMING_STALL_SEED)}),
        ]
    if reverse:
        command.append("--reverse")
    from racer_tools.vehicle_params_loader import load_vehicle_params

    vehicle = load_vehicle_params()
    accel_allow = RATE_FRACTION * vehicle.actuation.max_acceleration_mps2
    label = (
        f"{name} lane {params['lane_width_m']} {'cw' if reverse else 'ccw'} {v_max} m/s "
        f"{'with stalls' if stalls else 'no stalls'}"
    )

    def run_once():
        out.unlink(missing_ok=True)
        proc = subprocess.run(
            command, env=dict(os.environ), capture_output=True, text=True, timeout=1500, check=False
        )
        assert out.exists(), proc.stdout[-2000:] + proc.stderr[-2000:]
        result = json.loads(out.read_text())
        assert "error" not in result, result["error"]
        return result

    def report(result):
        return (
            f"\nl5_pnp timing {label}: laps {result['laps']} "
            f"lap_times_s {result['lap_times_s']} wall_contacts {result['wall_contacts']} "
            f"min_clearance_m {result['min_body_clearance_m']} "
            f"events {result.get('safety_event_counts', {})} "
            f"rate_limit_events_whole_run {result['rate_limit_events']} "
            f"drive_raw {result.get('drive_raw', {})} "
            f"safety_drive {result.get('safety_drive', {})} "
            f"planner_states {result['planner_states']}"
        )

    def judge(result):
        failures = timing_failures(
            result,
            result.get("safety_event_counts", {}),
            result.get("drive_raw", {}),
            stalls,
            laps,
            accel_allow,
        )
        assert not failures, failures

    def say(line):
        with capsys.disabled():
            print(line)

    # A failing attempt with a gap in the planner's /drive_raw or in safety_node's /drive was starved
    # by the machine and is run again (starvation.py); the judgement is the same every attempt.
    result = starvation.run_judged(run_once, judge, report, say, max_attempts=TIMING_MAX_ATTEMPTS)
    assert result["publisher_nodes"] == {
        "/drive_raw": ["mppi_planner_node"],
        "/drive": ["safety_node"],
    }
    if not stalls:
        assert str(STATE_FAULT) not in result["planner_states"], result["planner_states"]


def timing_failures(result, counts, drive, stalls, laps, accel_allow) -> list[str]:
    """The pass criteria of one timing run (pre-registration 4.1, counting window of 4.2: events
    stamped between the first non-zero /drive_raw command and the last planner status)."""
    failures = []
    if result["laps"] < laps:
        failures.append(f"laps {result['laps']} of {laps}: {result.get('stopped_because')}")
    if result["wall_contacts"] != 0:
        failures.append(f"wall contacts {result['wall_contacts']}")
    if counts.get("rate_limit", 0) != 0:
        failures.append(f"rate_limit engagements {counts['rate_limit']}")
    if counts.get("watchdog", 0) > (1 if stalls else 0):
        failures.append(f"watchdog engagements {counts['watchdog']}")
    ttc = sum(n for source, n in counts.items() if source.startswith("ttc"))
    if ttc > math.ceil(laps / 10):
        failures.append(f"TTC or obstacle brakes {ttc}")
    # no speed command jump after a stop: the first speed after the command was 0 is one ramp step
    if drive["max_first_speed_after_zero_mps"] > accel_allow * 0.02 + 1e-6:
        failures.append(f"speed jump after a stop {drive['max_first_speed_after_zero_mps']}")
    # and the speed never rose faster than the planner's share of the acceleration limit (the
    # window is at least 50 ms, so the 1 ms resolution of stamps is a few percent at most)
    if drive["max_speed_rise_mps2"] > 1.1 * accel_allow:
        failures.append(f"speed rise {drive['max_speed_rise_mps2']} m/s^2")
    return failures


@pytest.mark.parametrize(("track", "reverse", "v_max", "stalls", "laps"), TIMING_PUSH_CASES)
def test_pnp_planner_timing(track, reverse, v_max, stalls, laps, tmp_path, capsys):
    run_timing_case(
        track, reverse, v_max, stalls, laps, tmp_path, capsys, domain_id=83 + DOMAIN_OFFSET
    )


@pytest.mark.parametrize(("track", "reverse", "v_max", "stalls", "laps"), TIMING_FULL_CASES)
def test_pnp_planner_timing_full_matrix(track, reverse, v_max, stalls, laps, tmp_path, capsys):
    run_timing_case(
        track, reverse, v_max, stalls, laps, tmp_path, capsys, domain_id=84 + DOMAIN_OFFSET
    )


# --- fused-v1 against the geometric backend (brief 38, pre-registration v0.4 section 3.2 item 8) ---
# The same cases and seed, once with the geometric backend and once with `backend net` and a bundle
# that declares `io.arch.type: circ_conv1d_v0_fused_v1`. Every case must complete with 0 wall
# contacts and the fused median lap time must be at most 1.03 times the geometric one.
#
# The bundle: the frozen `trackperc-v2f-*` weights are not in git (car-data), so CI builds a fused
# bundle from the committed released v0 weights, which exercises the whole fused path (tracer, net,
# refinement, fusion, bundle contract) on the real node. Set RACER_L5_PNP_FUSED_BUNDLE to the
# directory of the real v2f bundle to run the same comparison with it. The geometric runs above
# stay the default canary.
FUSED_MAX_LAP_RATIO = (
    1.03  # evaluation/prereg/pnp-v0.4.yaml perception.sim_canary_max_lap_time_ratio
)
FUSED_CASES = [
    pytest.param(STADIUM, False, id="stadium-ccw"),
    pytest.param(STADIUM, True, id="stadium-cw"),
    pytest.param(ROUNDED_RECT, False, id="rounded_rect-ccw"),
]


def fused_bundle(tmp_path: Path) -> Path:
    override = os.environ.get("RACER_L5_PNP_FUSED_BUNDLE")
    if override:
        return Path(override)
    if not (PERCEPTION_BUNDLE / "manifest.yaml").is_file():
        pytest.skip(f"released perception bundle not committed yet: {PERCEPTION_BUNDLE}")
    target = tmp_path / "fused_bundle"
    shutil.copytree(PERCEPTION_BUNDLE, target)
    manifest = yaml.safe_load((target / "manifest.yaml").read_text())
    manifest["io"]["arch"] = {
        "type": "circ_conv1d_v0_fused_v1",
        "fuse": {
            "reach_m": 0.35,
            "edge_margin_m": 0.15,
            "blend": 1.0,
            "append_far": True,
            "max_shift_slope": 0.30,
        },
    }
    (target / "manifest.yaml").write_text(yaml.safe_dump(manifest, sort_keys=False))
    return target


def run_comparison_case(track, reverse, backend, bundle, tmp_path, domain_id):
    name, params = track
    out = tmp_path / f"result-{backend}.json"
    command = [
        sys.executable,
        str(RUNNER),
        "--track",
        name,
        "--track-params-json",
        json.dumps(params),
        "--driver",
        "pnp",
        "--backend",
        backend,
        "--dynamics-bundle",
        str(PLANNER_BUNDLE),
        "--v-max-mps",
        str(V_MAX_MPS),
        "--car",
        "params",
        "--seed",
        str(SEED),
        "--stop-after-laps",
        str(LAPS),
        "--sim-time",
        "--duration",
        "150",
        "--domain-id",
        str(domain_id),
        "--work-dir",
        str(tmp_path / f"work-{backend}"),
        "--out",
        str(out),
    ]
    if bundle is not None:
        command += ["--perception-bundle", str(bundle)]
    if reverse:
        command.append("--reverse")
    proc = subprocess.run(
        command, env=dict(os.environ), capture_output=True, text=True, timeout=500, check=False
    )
    assert out.exists(), proc.stdout[-2000:] + proc.stderr[-2000:]
    result = json.loads(out.read_text())
    assert "error" not in result, result["error"]
    return result


@pytest.mark.parametrize(("track", "reverse"), FUSED_CASES)
def test_pnp_fused_backend_laps_as_fast_as_geometric(track, reverse, tmp_path, capsys):
    assert (DYNAMICS_BUNDLE / "manifest.yaml").is_file(), DYNAMICS_BUNDLE
    assert (PLANNER_BUNDLE / "manifest.yaml").is_file(), PLANNER_BUNDLE
    bundle = fused_bundle(tmp_path)
    geo = run_comparison_case(track, reverse, "geometric", None, tmp_path, 86)
    fused = run_comparison_case(track, reverse, "net", bundle, tmp_path, 87)
    name = track[0]
    with capsys.disabled():
        print(
            f"\nl5_pnp fused {name} {'cw' if reverse else 'ccw'}: "
            f"geometric lap_times_s {geo['lap_times_s']} wall_contacts {geo['wall_contacts']}; "
            f"fused lap_times_s {fused['lap_times_s']} wall_contacts {fused['wall_contacts']} "
            f"safety_brakes {fused['safety_brakes']} planner_states {fused['planner_states']}"
        )
    for result in (geo, fused):
        assert result["laps"] >= LAPS, result.get("stopped_because")
        assert result["wall_contacts"] == 0
    assert fused["publisher_nodes"] == {
        "/drive_raw": ["mppi_planner_node"],
        "/drive": ["safety_node"],
    }
    ratio = statistics.median(fused["lap_times_s"]) / statistics.median(geo["lap_times_s"])
    assert ratio <= FUSED_MAX_LAP_RATIO, f"fused median lap time is {ratio:.3f} times geometric"
