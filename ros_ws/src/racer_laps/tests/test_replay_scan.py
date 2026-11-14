"""L1: `racer_laps.replay --lap-source start_scan` on a synthetic recording (a stadium driven by a
simulated car), the start-time rule, the split-file segments and the report."""

from __future__ import annotations

import math
from pathlib import Path
from types import SimpleNamespace as NS

import numpy as np
import pytest
from racer_laps import replay
from racer_laps.lap_record import STATE_DRIVING
from racer_laps.vehicle import LapVehicle
from synth import BODY, MOUNT, WHEELBASE_M, Stadium, drive_stadium, scan_at


def stamp(t: float) -> NS:
    return NS(sec=int(t), nanosec=int((t % 1.0) * 1e9))


def messages(laps: float, standstill_s: float = 0.0, planner: bool = False):
    stadium = Stadium()
    walls = stadium.walls()
    rng = np.random.default_rng(5)
    out = []
    t_log0 = 1000.0
    # a standstill first: odometry at zero, scans of the first pose
    x0, y0, yaw0, _ = stadium.pose_at(0.0)
    for i in range(int(standstill_s * 50)):
        t = t_log0 + i * 0.02
        out.append(
            (t, "/odom/wheel", NS(header=NS(stamp=stamp(t)), twist=NS(twist=NS(linear=NS(x=0.0)))))
        )
        if i % 5 == 0:
            ranges, a0, da = scan_at(walls, x0, y0, yaw0, rng)
            out.append((t + 0.1, "/scan", scan_msg(t, ranges, a0, da)))
    shift = t_log0 + standstill_s
    for tick in drive_stadium(stadium, laps, speed_mps=1.0, t0=0.0):
        t = shift + tick.t
        if tick.kind == "odom":
            out.append(
                (
                    t,
                    "/odom/wheel",
                    NS(header=NS(stamp=stamp(t)), twist=NS(twist=NS(linear=NS(x=tick.payload[0])))),
                )
            )
        elif tick.kind == "drive":
            out.append((t, "/drive", NS(drive=NS(steering_angle=tick.payload[0], speed=1.0))))
        else:
            x, y, yaw, _ = tick.payload
            ranges, a0, da = scan_at(walls, x, y, yaw, rng)
            out.append((t + 0.1, "/scan", scan_msg(t, ranges, a0, da)))
        if planner and tick.kind == "odom" and abs(tick.t) < 1e-9:
            out.append(
                (t, "/pnp/planner/status", NS(state=STATE_DRIVING, plan_min_clearance_m=0.3))
            )
    out.sort(key=lambda m: m[0])
    return out


def scan_msg(t: float, ranges: np.ndarray, a0: float, da: float) -> NS:
    return NS(
        header=NS(stamp=stamp(t)),
        ranges=ranges,
        angle_min=a0,
        angle_increment=da,
        range_min=0.05,
        range_max=16.0,
        scan_time=0.08,
    )


VEHICLE = LapVehicle(MOUNT, BODY, WHEELBASE_M, ())


def test_find_start_time_rule() -> None:
    assert replay.find_start_time([]) is None
    # already moving: the first sample
    assert replay.find_start_time([(5.0, 0.4), (5.02, 0.5)]) == 5.0
    # standing, then moving: the last standstill sample
    assert replay.find_start_time([(1.0, 0.0), (1.02, 0.01), (1.04, 0.2), (1.06, 0.0)]) == 1.02
    assert replay.find_start_time([(1.0, -0.04), (1.02, 0.3)]) == 1.0


def test_start_scan_replay_counts_the_laps_of_a_synthetic_recording() -> None:
    msgs = messages(3.3, standstill_s=1.0)
    result = replay.run_replay_start_scan(msgs, VEHICLE)
    assert [lap.lap_index for lap in result.laps] == [1, 2, 3]
    period = Stadium().length_m
    for lap in result.laps[1:]:
        assert lap.lap_time_s == pytest.approx(period, abs=0.25)
    assert result.start_t_s == pytest.approx(1000.0 + 0.98, abs=0.03)
    assert result.armed_scans > 0 and 0 < result.matched_scans <= result.armed_scans
    assert result.wheel_distance_m == pytest.approx(3.3 * period, rel=0.1)
    assert result.match_times_s
    report = replay.format_scan_report(result)
    assert "lap 3:" in report and "percent of armed" in report and "on_scan cost" in report
    assert result.bag_time(result.laps[0].t_cross_s) == pytest.approx(
        result.laps[0].t_cross_s - 1000.1, abs=0.2
    )


def test_a_window_cuts_the_recording_in_bag_seconds() -> None:
    msgs = messages(3.3)
    first_scan_log = next(m[0] for m in msgs if m[1] == "/scan")
    period = Stadium().length_m
    result = replay.run_replay_start_scan(msgs, VEHICLE, window_s=(0.0, 1.5 * period + 0.5))
    assert [lap.lap_index for lap in result.laps] == [1]
    late = replay.run_replay_start_scan(msgs, VEHICLE, window_s=(period, math.inf))
    assert len(late.laps) >= 1
    assert late.start_t_s is not None and late.start_t_s - first_scan_log >= period - 0.1


def test_planner_status_in_the_bag_starts_the_lap_and_judges_it_clean() -> None:
    msgs = messages(2.4, planner=True)
    result = replay.run_replay_start_scan(msgs, VEHICLE)
    assert len(result.laps) >= 1
    assert result.laps[0].clean


def test_no_scan_in_the_bag_is_an_error() -> None:
    with pytest.raises(ValueError):
        replay.run_replay_start_scan([(1.0, "/odom/wheel", NS())], VEHICLE)


def test_segment_ranges_joins_consecutive_segments(tmp_path: Path) -> None:
    split = tmp_path / "split.yaml"
    split.write_text(
        "bags:\n  bagA:\n    segments:\n"
        "      - {name: A1, from: 10.0, to: 20.0}\n"
        "      - {name: A2, from: 22.0, to: 30.0}\n"
        "      - {name: A3, from: 40.0, to: 50.0}\n"
    )
    assert replay.segment_ranges(split, "bagA", "A2") == (22.0, 30.0)
    assert replay.segment_ranges(split, "bagA", "A1+A2") == (10.0, 30.0)
    with pytest.raises(KeyError):
        replay.segment_ranges(split, "bagA", "A9")


def test_cli_start_scan_and_compare_pose(
    monkeypatch: pytest.MonkeyPatch, capsys: pytest.CaptureFixture[str], tmp_path: Path
) -> None:
    msgs = messages(2.4)
    monkeypatch.setattr(replay, "read_messages", lambda path: iter(msgs))
    monkeypatch.setattr(replay, "load_live_vehicle", lambda: VEHICLE)
    code = replay.main(["--bag", str(tmp_path), "--lap-source", "start_scan", "--compare-pose"])
    out = capsys.readouterr().out
    assert code == 0
    assert "lap 1:" in out and "map pose, same segment and start" in out
    assert replay.main(["--bag", str(tmp_path), "--lap-source", "start_scan", "--from-s", "0"]) == 0
    with pytest.raises(SystemExit):
        replay.main(["--bag", str(tmp_path), "--segment", "X"])


def test_cli_map_pose_with_a_window(
    monkeypatch: pytest.MonkeyPatch, capsys: pytest.CaptureFixture[str], tmp_path: Path
) -> None:
    msgs = messages(1.0)
    monkeypatch.setattr(replay, "read_messages", lambda path: iter(msgs))
    code = replay.main(["--bag", str(tmp_path), "--from-s", "0", "--to-s", "3"])
    assert code == 1  # no TF in the synthetic recording
    assert "no usable map -> base_link transform" in capsys.readouterr().out
