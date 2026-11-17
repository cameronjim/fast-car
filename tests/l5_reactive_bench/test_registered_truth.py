"""L1 tests of the registered ground truth of the lap battery (brief 48 item 1, pre-registration
`docs/notes/preregistration-pnp-v0.5-laps.md` section 2): crossing 0 is the stamp of the first
`/scan` at or after the planner's first DRIVING report, the start line is the ground-truth pose at
that stamp, truth never reads the system's laps, and a recording that does not cover crossing 0 is
invalid, not a result. Also the two-phase start of the runner. No ROS."""

from __future__ import annotations

import inspect
import math
from pathlib import Path
from types import SimpleNamespace

import numpy as np
import pytest
import track_family_runner as runner
import yaml

REPO_ROOT = Path(__file__).resolve().parents[2]
PREREG = REPO_ROOT / "evaluation" / "prereg" / "pnp-v0.5-laps.yaml"

PERIOD_S = 10.0


def centreline():
    a = np.linspace(0, 2 * math.pi, 400, endpoint=False)
    return np.cos(a), np.sin(a)


def car_odom(t_from: float, t_to: float, t_go: float = 103.0, hz: float = 50.0):
    """Ground-truth odometry (t, x, y, yaw, v) of a car on a unit circle that stands until `t_go`
    and then goes round once per PERIOD_S, recorded from `t_from` to `t_to`."""
    out = []
    n = int((t_to - t_from) * hz)
    for i in range(n):
        t = t_from + i / hz
        a = 0.0 if t < t_go else 2 * math.pi * (t - t_go) / PERIOD_S
        out.append((t, math.cos(a), math.sin(a), a + math.pi / 2, 0.0))
    return out


def scans(t0: float, t1: float, hz: float = 10.0) -> list[float]:
    return [round(t0 + k / hz, 6) for k in range(int((t1 - t0) * hz) + 1)]


def test_a_recording_that_starts_after_crossing_0_is_invalid_not_a_short_lap() -> None:
    """The four B4 runs of 2.3 s: the planner was DRIVING at 100.0, the startup reset finished at
    102.3 and the recording began there. The tool's lap 1 was 2.3 s short and nothing said so."""
    odom = car_odom(102.3, 150.0)
    out = runner.registered_truth(odom, scans(99.0, 150.0), 100.0, centreline())
    assert out["truth_valid"] is False
    assert out["truth_invalid_reason"] == "truth_starts_after_crossing_0"
    assert out["crossing0_s"] == pytest.approx(100.0)
    assert out["truth_first_odom_s"] == pytest.approx(102.3)


def test_a_recording_that_covers_crossing_0_gives_lap_1_from_the_first_scan() -> None:
    odom = car_odom(99.2, 150.0)  # recorded before the DRIVING report, as in the 86 good runs
    out = runner.registered_truth(odom, scans(99.0, 150.0), 100.04, centreline())
    assert out["truth_valid"] is True and out["truth_invalid_reason"] == ""
    assert out["crossing0_s"] == pytest.approx(100.1)  # the first scan at or after the report
    # the car stands until 103.0 on the line of the pose at 100.1: one lap after that
    assert out["lap_times_s"][0] == pytest.approx(3.0 - 0.1 + PERIOD_S, abs=0.1)
    assert all(t == pytest.approx(PERIOD_S, abs=0.1) for t in out["lap_times_s"][1:])
    assert out["crossings_s"][0] == pytest.approx(113.0, abs=0.1)
    assert len(out["lap_times_s"]) == len(out["crossings_s"]) == 4


def test_no_scan_within_the_wait_after_the_report_is_invalid() -> None:
    odom = car_odom(99.2, 150.0)
    late = [s for s in scans(99.0, 150.0) if not 100.0 <= s < 100.5]
    out = runner.registered_truth(odom, late, 100.0, centreline())
    assert out["truth_valid"] is False and out["truth_invalid_reason"] == "no_scan_at_start"
    none = runner.registered_truth(odom, [], 100.0, centreline())
    assert none["truth_invalid_reason"] == "no_scan_at_start"
    assert none["lap_times_s"] == [] and none["crossing0_s"] is None
    ok = runner.registered_truth(
        odom, [s for s in scans(99.0, 150.0) if s != 100.0], 100.0, centreline()
    )
    assert ok["truth_valid"] is True  # the next scan, 0.1 s later, is inside the 0.25 s


def test_a_startup_reset_that_completed_after_driving_is_invalid() -> None:
    odom = car_odom(99.2, 150.0)
    args = (odom, scans(99.0, 150.0), 100.0, centreline())
    late = runner.registered_truth(*args, startup_reset_done_s=100.4)
    assert late["truth_valid"] is False and late["truth_invalid_reason"] == "reset_after_driving"
    early = runner.registered_truth(*args, startup_reset_done_s=99.5)
    assert early["truth_valid"] is True
    unknown = runner.registered_truth(*args)
    assert unknown["truth_valid"] is True


def test_no_driving_report_or_no_odometry_is_invalid() -> None:
    odom = car_odom(99.2, 150.0)
    out = runner.registered_truth(odom, scans(99.0, 150.0), None, centreline())
    assert out["truth_valid"] is False and out["truth_invalid_reason"] == "no_scan_at_start"
    empty = runner.registered_truth([], scans(99.0, 150.0), 100.0, centreline())
    assert empty["truth_valid"] is False
    assert empty["truth_invalid_reason"] == "truth_starts_after_crossing_0"


def test_truth_takes_nothing_from_the_systems_laps() -> None:
    """By its signature, and by what lap_results does with two different sets of LapEvents."""
    params = set(inspect.signature(runner.registered_truth).parameters)
    assert params == {
        "odom",
        "scan_stamps",
        "driving_receipt_s",
        "centreline",
        "startup_reset_done_s",
        "max_scan_wait_s",
    }
    odom = car_odom(99.2, 150.0)
    truth = runner.registered_truth(odom, scans(99.0, 150.0), 100.0, centreline())

    tracker = SimpleNamespace(_crossings=[1.0], lap_times_s=[1.0])

    def fields(lap_events: list[dict]) -> dict:
        out = runner.lap_results(
            tracker, [], lap_events, [], {}, 100.0, 100.0, rescored=None, truth=truth
        )
        return {k: v for k, v in out.items() if k.startswith("gt_lap") and "rescored" not in k}

    first = {"lap_index": 1, "t_cross_s": 120.0, "lap_time_s": 17.0}
    other = {"lap_index": 1, "t_cross_s": 111.0, "lap_time_s": 3.0}
    assert fields([first]) == fields([other, first]) == fields([])
    assert fields([])["gt_lap_times_registered_s"] == pytest.approx(truth["lap_times_s"], abs=1e-3)


def test_lap_results_reports_validity_and_keeps_the_rescored_numbers_as_a_reported_field() -> None:
    odom = car_odom(102.3, 150.0)
    truth = runner.registered_truth(odom, scans(99.0, 150.0), 100.0, centreline())

    tracker = SimpleNamespace(_crossings=[], lap_times_s=[])

    out = runner.lap_results(
        tracker,
        [],
        [],
        [],
        {},
        100.0,
        100.0,
        rescored=([9.9], [110.3], 100.4),
        truth=truth,
    )
    assert out["truth_valid"] is False
    assert out["truth_invalid_reason"] == "truth_starts_after_crossing_0"
    assert out["gt_lap_times_rescored_s"] == [9.9]  # reported, not used
    assert out["gt_lap_times_by_crossing_s"] == out["gt_lap_times_registered_s"]
    assert out["truth_first_odom_s"] == pytest.approx(102.3)


def test_the_scan_wait_is_the_registered_constant() -> None:
    data = yaml.safe_load(PREREG.read_text())
    assert runner.MAX_SCAN_WAIT_S == data["lap_battery_b7"]["validity"]["max_scan_wait_s"]


# -- the two-phase start ----------------------------------------------------------------------


def stack():
    return [
        ("bridge_node", ["bridge"]),
        ("safety_node", ["safety"]),
        ("track_perception_node", ["perception"]),
        ("mppi_planner_node", ["planner"]),
        ("lap_memory_node", ["lap"]),
    ]


def test_the_planner_is_not_started_before_the_reset_even_with_a_slow_reset_service() -> None:
    """The service appears 25 polls (2.5 s) after the bridge starts: the old blocking
    `wait_for_service` did not spin, so the startup reset landed 2.3 s after a planner that had
    been started with the bridge. The perception, planner and lap node are spawned only after
    the reset completed and the recording was cleared."""
    log: list[str] = []
    polls = {"n": 0}

    def spawn(name, _command):
        log.append(f"spawn {name}")

    def ready() -> bool:
        polls["n"] += 1
        log.append("poll")
        return polls["n"] >= 25

    runner.run_startup(
        stack(),
        two_phase=True,
        spawn=spawn,
        ready=ready,
        spin=lambda: log.append("spin"),
        reset=lambda: log.append("reset"),
        after_reset=lambda: log.append("clear"),
        wait_timeout_polls=100,
    )
    spawned = [e for e in log if e.startswith("spawn")]
    assert spawned == [
        "spawn bridge_node",
        "spawn safety_node",
        "spawn track_perception_node",
        "spawn mppi_planner_node",
        "spawn lap_memory_node",
    ]
    i_reset, i_clear = log.index("reset"), log.index("clear")
    assert log.index("spawn safety_node") < log.index("poll") < i_reset < i_clear
    assert i_clear < log.index("spawn track_perception_node")
    assert log.count("poll") == 25 and log.count("spin") >= 24  # it spun while it waited
    assert log.index("spawn mppi_planner_node") > i_reset


def test_a_reset_service_that_never_appears_is_an_error_and_starts_nothing_else() -> None:
    log: list[str] = []
    with pytest.raises(RuntimeError, match="/sim/reset not available"):
        runner.run_startup(
            stack(),
            two_phase=True,
            spawn=lambda name, _c: log.append(name),
            ready=lambda: False,
            spin=lambda: None,
            reset=lambda: log.append("reset"),
            after_reset=lambda: log.append("clear"),
            wait_timeout_polls=10,
        )
    assert log == ["bridge_node", "safety_node"]


def test_other_drivers_keep_the_single_phase_start() -> None:
    log: list[str] = []
    runner.run_startup(
        stack(),
        two_phase=False,
        spawn=lambda name, _c: log.append(name),
        ready=lambda: True,
        spin=lambda: None,
        reset=lambda: log.append("reset"),
        after_reset=lambda: log.append("clear"),
        wait_timeout_polls=10,
    )
    assert log[:5] == [n for n, _ in stack()] and log[5:] == ["reset", "clear"]
