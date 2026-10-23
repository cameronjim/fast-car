"""L1 tests for tools/reactive_bench.py (the reactive benchmark's battery CLI), no ROS.

The runner (tests/l5_reactive_bench) is replaced by fakes: what is under test here is the
CLI's own logic: how runs are set up and retried, how a parameter sweep is expanded and ranked,
the CI criteria, and the tables and the note.
"""

from __future__ import annotations

import json
import math
import queue
import threading

import pytest
import reactive_bench as rb


def result(track="lane_square", **kw) -> dict:
    """A clean run's result; keyword arguments override fields."""
    base = {
        "track": track,
        "profile": "p",
        "overrides": {},
        "laps": 2,
        "lap_time_s": 30.0,
        "lap_times_s": [30.0, 30.0],
        "obstacle_brakes": 0,
        "escapes": 0,
        "min_body_clearance_m": 0.12,
        "wall_contacts": 0,
        "contact_points": [],
        "mean_abs_steering_rad": 0.2,
        "steering_reversals_per_min": 40.0,
        "duration_s": 90.0,
        "progress_m": 40.0,
        "loop_length_m": 19.0,
    }
    base.update(kw)
    return base


# -- run_one ------------------------------------------------------------------------------------


def _fake_runner(payloads):
    """An invoke() that records its command and writes the next payload as the runner would
    (a dict -> the JSON result file, None -> no file, as when the runner dies)."""
    calls = []

    def invoke(command, timeout_s):
        calls.append((command, timeout_s))
        payload = payloads[min(len(calls) - 1, len(payloads) - 1)]
        if payload is not None:
            out = command[command.index("--out") + 1]
            with open(out, "w") as f:
                json.dump(payload, f)
        return (0 if payload is not None else 1), "stderr tail"

    return invoke, calls


def test_run_one_builds_the_runner_command():
    invoke, calls = _fake_runner([result()])
    r = rb.run_one(
        "lane_square",
        "floor-2026-10-06",
        {"steering_gain": 2.4},
        duration_s=45.0,
        stop_after_laps=1,
        stop_after_contacts=3,
        track_params={"box_gap_m": 1.0},
        domain_id=62,
        invoke=invoke,
    )
    command, timeout = calls[0]
    assert str(rb.RUNNER) in command
    pairs = dict(zip(command[2::2], command[3::2], strict=False))
    assert pairs["--track"] == "lane_square"
    assert pairs["--profile"] == "floor-2026-10-06"
    assert json.loads(pairs["--params-json"]) == {"steering_gain": 2.4}
    assert pairs["--duration"] == "45.0"
    assert pairs["--stop-after-laps"] == "1"
    assert pairs["--stop-after-contacts"] == "3"
    assert json.loads(pairs["--track-params-json"]) == {"box_gap_m": 1.0}
    assert pairs["--domain-id"] == "62"
    assert timeout > 3 * 45.0
    assert r["laps"] == 2


def test_run_one_retries_a_run_that_produced_no_result():
    invoke, calls = _fake_runner([None, result()])
    r = rb.run_one("lane_square", "p", {}, invoke=invoke)
    assert len(calls) == 2
    assert "error" not in r


def test_run_one_retries_a_run_that_reports_an_error():
    invoke, calls = _fake_runner([{"error": "stack not up"}, result()])
    assert "error" not in rb.run_one("lane_square", "p", {}, invoke=invoke)
    assert len(calls) == 2


def test_run_one_reports_a_run_that_keeps_failing_with_its_setup():
    invoke, calls = _fake_runner([None])
    r = rb.run_one("chicane", "floor-2026-10-06", {"a": 1.0}, invoke=invoke)
    assert len(calls) == 2
    assert "without a result" in r["error"] and "stderr tail" in r["error"]
    assert (r["track"], r["profile"], r["overrides"]) == ("chicane", "floor-2026-10-06", {"a": 1.0})


def test_run_one_survives_a_runner_timeout():
    import subprocess

    def invoke(command, timeout_s):
        raise subprocess.TimeoutExpired(command, timeout_s)

    r = rb.run_one("chicane", "p", {}, invoke=invoke)
    assert "timed out" in r["error"]


# -- run_batteries ------------------------------------------------------------------------------


def test_run_batteries_scales_durations_and_keeps_order():
    seen = []

    def runner(track, profile, overrides, **kw):
        seen.append(
            (track, overrides, kw["duration_s"], kw["stop_after_laps"], kw["stop_after_contacts"])
        )
        return result(track, overrides=overrides)

    domains: queue.Queue[int] = queue.Queue()
    domains.put(60)
    out = rb.run_batteries(
        "p",
        [{"a": 1.0}, {"a": 2.0}],
        [("lane_square", 150.0), ("chicane", 90.0), ("open_room", 0.0)],
        duration_scale=0.5,
        stop_after_laps=1,
        stop_after_contacts=2,
        domain_ids=domains,
        runner=runner,
    )
    # One list of results per parameter set, in track order.
    tracks = ["lane_square", "chicane", "open_room"]
    assert [[r["track"] for r in battery] for battery in out] == [tracks, tracks]
    assert [[r["overrides"] for r in battery] for battery in out] == [
        [{"a": 1.0}] * 3,
        [{"a": 2.0}] * 3,
    ]
    assert seen[:3] == [
        ("lane_square", {"a": 1.0}, 75.0, 1, 2),
        ("chicane", {"a": 1.0}, 45.0, 1, 2),
        ("open_room", {"a": 1.0}, 0.0, 1, 2),  # 0 = the runner's own default, not scaled
    ]
    assert domains.qsize() == 1  # the domain id went back


def test_run_batteries_never_runs_two_jobs_on_one_domain_id():
    lock = threading.Lock()
    in_use: set[int] = set()
    clashes = []
    barrier = threading.Barrier(3, timeout=10)

    def runner(track, profile, overrides, **kw):
        with lock:
            if kw["domain_id"] in in_use:
                clashes.append(kw["domain_id"])
            in_use.add(kw["domain_id"])
        barrier.wait()  # all three overlap
        with lock:
            in_use.discard(kw["domain_id"])
        return result(track)

    domains: queue.Queue[int] = queue.Queue()
    for d in (70, 71, 72):
        domains.put(d)
    out = rb.run_batteries(
        "p",
        [{}],
        [("lane_square", 1.0), ("chicane", 1.0), ("open_room", 1.0)],
        domain_ids=domains,
        jobs=3,
        runner=runner,
    )
    assert clashes == []
    assert len(out) == 1 and len(out[0]) == 3


def test_run_batteries_pools_the_runs_of_all_parameter_sets():
    """Two sets of one track on two jobs: both runs overlap, not set after set."""
    barrier = threading.Barrier(2, timeout=10)
    started = []

    def runner(track, profile, overrides, **kw):
        started.append(overrides)
        barrier.wait()
        return result(track)

    domains: queue.Queue[int] = queue.Queue()
    for d in (70, 71):
        domains.put(d)
    rb.run_batteries(
        "p",
        [{"a": 1.0}, {"a": 2.0}],
        [("lane_square", 1.0)],
        domain_ids=domains,
        jobs=2,
        runner=runner,
    )
    assert sorted(o["a"] for o in started) == [1.0, 2.0]


# -- Judging ------------------------------------------------------------------------------------


def test_ci_criteria_pass_for_clean_laps():
    assert rb.ci_failures([result("lane_square"), result("chicane", laps=1)]) == []


def test_ci_criteria_fail_on_a_wall_contact():
    points = [{"t_s": 3.0, "x_m": 1.0, "y_m": 0.0, "reason": "gym_yaw_jump"}]
    failures = rb.ci_failures([result(wall_contacts=1, contact_points=points)])
    assert len(failures) == 1 and "wall contact" in failures[0]


def test_ci_criteria_fail_on_a_loop_without_a_lap():
    failures = rb.ci_failures([result(laps=0, lap_time_s=None, progress_m=12.0)])
    assert len(failures) == 1 and "no lap" in failures[0]


def test_ci_criteria_fail_on_an_errored_run_and_ignore_no_loop_tracks():
    failures = rb.ci_failures([{"track": "chicane", "error": "gap_follow_node exited with 1"}])
    assert len(failures) == 1 and "run failed" in failures[0]
    assert rb.ci_failures([result("open_room", laps=None, lap_time_s=None, progress_m=None)]) == []


def test_ci_battery_is_the_three_tracks_the_floor_profile_must_lap():
    assert [t for t, _ in rb.CI_BATTERY] == ["lane_square", "lane_open_side", "chicane"]
    assert all(duration > 0 for _, duration in rb.CI_BATTERY)


def test_aggregate_sums_over_tracks():
    agg = rb.aggregate(
        [
            result("lane_square", laps=2, lap_time_s=40.0, obstacle_brakes=3, escapes=4),
            result(
                "chicane", laps=3, lap_time_s=20.0, obstacle_brakes=1, min_body_clearance_m=0.05
            ),
            result("open_room", laps=None, lap_time_s=None, obstacle_brakes=2),
        ]
    )
    assert agg["laps"] == 5
    assert agg["obstacle_brakes"] == 6  # every track
    assert agg["loop_brakes"] == 4  # the open room has no laps: its brakes are not per lap
    assert agg["brakes_per_lap"] == pytest.approx(4 / 5)
    assert agg["lap_time_sum_s"] == pytest.approx(60.0)
    assert agg["escapes"] == 4
    assert agg["min_clearance_m"] == 0.05
    assert agg["wall_contacts"] == 0 and agg["incomplete_loops"] == 0 and agg["errors"] == 0


def test_aggregate_lap_time_is_per_track_mean_so_repeats_do_not_inflate_it():
    once = rb.aggregate(
        [result("lane_square", lap_time_s=40.0), result("chicane", lap_time_s=20.0)]
    )
    thrice = rb.aggregate(
        [result("lane_square", lap_time_s=t) for t in (39.0, 40.0, 41.0)]
        + [result("chicane", lap_time_s=t) for t in (19.0, 20.0, 21.0)]
    )
    assert once["lap_time_sum_s"] == thrice["lap_time_sum_s"] == pytest.approx(60.0)
    assert thrice["laps"] == 12


def test_aggregate_counts_loop_contacts_apart_from_the_open_rooms():
    agg = rb.aggregate(
        [
            result("lane_square", wall_contacts=1),
            result("open_room", laps=None, lap_time_s=None, wall_contacts=2),
        ]
    )
    assert agg["wall_contacts"] == 3 and agg["loop_wall_contacts"] == 1


def test_aggregate_counts_contacts_missing_laps_and_errors():
    agg = rb.aggregate(
        [
            result("lane_square", wall_contacts=2),
            result("chicane", laps=0, lap_time_s=None),
            {"track": "lane_round", "error": "boom"},
        ]
    )
    assert agg["wall_contacts"] == 2
    assert agg["incomplete_loops"] == 2  # chicane lapped nothing, lane_round never ran
    assert agg["errors"] == 1


def test_aggregate_with_no_laps_has_infinite_brakes_per_lap():
    agg = rb.aggregate([result(laps=0, lap_time_s=None, obstacle_brakes=4)])
    assert math.isinf(agg["brakes_per_lap"])


def _agg(contacts=0, incomplete=0, brakes_per_lap=1.0, lap_time=100.0):
    return {
        "wall_contacts": contacts,
        "incomplete_loops": incomplete,
        "brakes_per_lap": brakes_per_lap,
        "lap_time_sum_s": lap_time,
    }


def test_rank_order_is_contacts_then_laps_then_brakes_per_lap_then_lap_time():
    ordered = [
        _agg(contacts=0, brakes_per_lap=2.0, lap_time=200.0),
        _agg(contacts=0, brakes_per_lap=2.0, lap_time=300.0),  # slower, same brakes
        _agg(contacts=0, brakes_per_lap=5.0, lap_time=50.0),  # fast but brakes more
        _agg(contacts=0, incomplete=1, brakes_per_lap=0.0, lap_time=10.0),  # a loop never lapped
        _agg(contacts=1, brakes_per_lap=0.0, lap_time=10.0),  # touched a wall: behind all clean
        _agg(contacts=3, brakes_per_lap=0.0, lap_time=10.0),
    ]
    shuffled = [ordered[i] for i in (4, 2, 0, 5, 3, 1)]
    assert sorted(shuffled, key=rb.rank_key) == ordered


# -- Sweep --------------------------------------------------------------------------------------


def test_parse_sweep_types():
    grid = rb.parse_sweep(
        [
            "steering_gain=1.0,1.6,2.4",
            "centering_gain=0,0.4",
            "reverse_escape=true,false",
            "target_range_median_scans=3,5",
        ]
    )
    assert grid["steering_gain"] == [1.0, 1.6, 2.4]
    # A double parameter must not be handed an int: "0" becomes 0.0.
    assert grid["centering_gain"] == [0.0, 0.4]
    assert all(isinstance(v, float) for v in grid["centering_gain"])
    assert grid["reverse_escape"] == [True, False]
    assert grid["target_range_median_scans"] == [3, 5]
    assert all(isinstance(v, int) for v in grid["target_range_median_scans"])


@pytest.mark.parametrize("spec", ["steering_gain", "=1,2", "steering_gain=", "steering_gain=a"])
def test_parse_sweep_rejects_bad_specs(spec):
    with pytest.raises(ValueError):
        rb.parse_sweep([spec])


def test_grid_points_is_the_full_product():
    points = rb.grid_points({"a": [1.0, 2.0], "b": [0.1, 0.2, 0.3]})
    assert len(points) == 6
    assert {"a": 2.0, "b": 0.3} in points
    assert len({tuple(sorted(p.items())) for p in points}) == 6


def test_the_task_grid_has_thirty_six_points():
    grid = rb.parse_sweep(
        [
            "steering_gain=1.0,1.6,2.4",
            "cone_half_angle_rad=0.8,1.2",
            "centering_gain=0.0,0.4,0.6",
            "safety_margin_m=0.05,0.1",
        ]
    )
    assert len(rb.grid_points(grid)) == 36


# -- Formatting ---------------------------------------------------------------------------------


def test_results_table_has_a_row_per_track_and_dashes_for_missing_values():
    table = rb.results_table(
        [
            result("lane_square", obstacle_brakes=3, escapes=4),
            result("open_room", laps=None, lap_time_s=None),
            {"track": "chicane", "error": "x | y"},
        ]
    )
    lines = table.splitlines()
    assert lines[0].startswith("| Track |") and lines[1].startswith("|---")
    assert "| lane_square | 2 | 30.0 | 3 | 4 | 0.120 | 0 | 0.200 | 40.0 | 90 |" in lines
    assert "| open_room | - | - |" in table
    assert "ERROR" in lines[-1] and "x / y" in lines[-1]  # a pipe must not break the table
    assert len({line.count("|") for line in lines[:-1]}) == 1  # every full row has every column


def test_sweep_table_lists_the_parameters_and_the_totals():
    agg = rb.aggregate([result("lane_square", obstacle_brakes=2, laps=2, lap_time_s=40.0)])
    table = rb.sweep_table(
        [({"steering_gain": 1.6, "centering_gain": 0.4}, agg)], ["steering_gain", "centering_gain"]
    )
    lines = table.splitlines()
    assert lines[0].startswith("| # | steering_gain | centering_gain |")
    assert lines[2].startswith("| 1 | 1.6 | 0.4 | 0 | 0 | 0 | 2 | 2 | 1.00 | 40.0 |")
    assert lines[0].count("|") == lines[1].count("|") == lines[2].count("|")


def test_contact_lines_name_the_track_and_time():
    point = {"t_s": 12.5, "x_m": 3.0, "y_m": 0.5, "reason": "body_overlaps_wall"}
    lines = rb.contact_lines([result("chicane", wall_contacts=1, contact_points=[point])])
    assert lines == ["- chicane: contact at t = 12.5 s, (3.0, 0.5) (body_overlaps_wall)"]


# -- main ---------------------------------------------------------------------------------------


def _patch_battery(monkeypatch, per_track):
    calls = []

    def fake_batteries(profile, override_sets, tracks, **kw):
        calls.append((profile, override_sets, tracks, kw))
        return [[per_track(profile, o, t) for t, _ in tracks] for o in override_sets]

    monkeypatch.setattr(rb, "run_batteries", fake_batteries)
    return calls


def test_main_ci_battery_passes_and_stops_at_the_first_lap(monkeypatch, capsys):
    calls = _patch_battery(monkeypatch, lambda p, o, t: result(t))
    assert rb.main(["--battery", "ci", "--check", "--no-note"]) == 0
    profile, _, tracks, kw = calls[0]
    assert profile == "floor-2026-10-06"
    assert [t for t, _ in tracks] == ["lane_square", "lane_open_side", "chicane"]
    assert kw["stop_after_laps"] == 1 and kw["stop_after_contacts"] == 1
    assert "| lane_square |" in capsys.readouterr().out


def test_main_check_fails_on_a_contact(monkeypatch, capsys):
    point = {"t_s": 1.0, "x_m": 0.0, "y_m": 0.0, "reason": "gym_yaw_jump"}
    _patch_battery(
        monkeypatch,
        lambda p, o, t: (
            result(t, wall_contacts=1, contact_points=[point]) if t == "chicane" else result(t)
        ),
    )
    assert rb.main(["--battery", "ci", "--check", "--no-note"]) == 1
    assert "chicane" in capsys.readouterr().err
    # Without --check the same run is only reported.
    assert rb.main(["--battery", "ci", "--no-note"]) == 0


def test_main_full_battery_writes_a_note_for_each_profile(monkeypatch, tmp_path):
    calls = _patch_battery(monkeypatch, lambda p, o, t: result(t, profile=p))
    note = tmp_path / "note.md"
    assert rb.main(["--profile", "floor-2026-10-06,none", "--note", str(note)]) == 0
    assert [c[0] for c in calls] == ["floor-2026-10-06", "none"]
    assert [t for t, _ in calls[0][2]] == list(rb.TRACKS)
    text = note.read_text()
    assert "### Profile `floor-2026-10-06`" in text and "### Profile `none`" in text
    assert text.count("| lane_square |") == 2


def test_main_tracks_subset_duration_and_scale(monkeypatch):
    calls = _patch_battery(monkeypatch, lambda p, o, t: result(t))
    rb.main(
        [
            "--tracks",
            "chicane,open_room",
            "--duration",
            "30",
            "--duration-scale",
            "0.5",
            "--no-note",
        ]
    )
    _, _, tracks, kw = calls[0]
    assert tracks == [("chicane", 30.0), ("open_room", 30.0)]
    assert kw["duration_scale"] == 0.5 and kw["stop_after_laps"] == 0


def test_main_rejects_an_unknown_track():
    with pytest.raises(SystemExit):
        rb.main(["--tracks", "oval", "--no-note"])


def test_main_sweep_ranks_and_reports_the_best(monkeypatch, tmp_path, capsys):
    def per_track(profile, overrides, track):
        # centering_gain 0.4 never touches a wall; 0.0 does; 0.6 is clean but brakes more.
        gain = overrides["centering_gain"]
        return result(
            track,
            wall_contacts=1 if gain == 0.0 else 0,
            contact_points=[] if gain != 0.0 else [{"t_s": 1.0, "x_m": 0, "y_m": 0, "reason": "x"}],
            obstacle_brakes=4 if gain == 0.6 else 1,
        )

    calls = _patch_battery(monkeypatch, per_track)
    note = tmp_path / "sweep.md"
    assert (
        rb.main(
            [
                "--sweep",
                "centering_gain=0.0,0.4,0.6",
                "--params-json",
                '{"steering_gain": 1.6}',
                "--stop-after-contacts",
                "3",
                "--tracks",
                "lane_square",
                "--note",
                str(note),
            ]
        )
        == 0
    )
    assert len(calls) == 1  # one pooled call for the whole grid
    assert calls[0][1] == [{"steering_gain": 1.6, "centering_gain": g} for g in (0.0, 0.4, 0.6)]
    assert calls[0][3]["stop_after_contacts"] == 3
    text = note.read_text()
    rows = [line for line in text.splitlines() if line.startswith("| ") and line[2].isdigit()]
    assert [row.split("|")[2].strip() for row in rows] == ["0.4", "0.6", "0.0"]
    assert "Best: {'centering_gain': 0.4}" in text
    assert "per track:" in text and "| lane_square |" in text
    assert "centering_gain" in capsys.readouterr().out


# -- Regrouping and repeats ---------------------------------------------------------------------


def test_group_runs_finds_the_swept_parameters_and_groups_by_them():
    runs = [
        result(track, overrides={"a": a, "b": 5.0})
        for a in (1.0, 2.0)
        for track in ("lane_square", "chicane")
    ]
    names, groups = rb.group_runs(runs)
    assert names == ["a"]  # b never varies
    assert [point for point, _ in groups] == [{"a": 1.0}, {"a": 2.0}]
    assert [[r["track"] for r in results] for _, results in groups] == [
        ["lane_square", "chicane"]
    ] * 2


def test_group_runs_with_explicit_names_and_repeats():
    runs = [result(overrides={"a": 1.0, "b": b}) for b in (0.1, 0.1, 0.2)]
    _, groups = rb.group_runs(runs, ["a", "b"])
    assert [(point, len(results)) for point, results in groups] == [
        ({"a": 1.0, "b": 0.1}, 2),
        ({"a": 1.0, "b": 0.2}, 1),
    ]


def test_main_from_json_ranks_an_earlier_sweep_without_running(monkeypatch, tmp_path):
    def boom(*a, **kw):
        raise AssertionError("must not run anything")

    monkeypatch.setattr(rb, "run_batteries", boom)
    runs = [
        result("lane_square", profile="floor", overrides={"g": g}, wall_contacts=int(g == 1.0))
        for g in (1.0, 2.0)
    ]
    source = tmp_path / "runs.json"
    source.write_text(json.dumps(runs))
    note = tmp_path / "n.md"
    assert rb.main(["--from-json", str(source), "--note", str(note)]) == 0
    text = note.read_text()
    assert "Sweep on profile `floor`" in text
    rows = [line for line in text.splitlines() if line.startswith("| ") and line[2].isdigit()]
    assert [row.split("|")[2].strip() for row in rows] == ["2.0", "1.0"]  # the clean one first


def test_main_sweep_repeats_each_point_and_sums_the_repeats(monkeypatch, tmp_path):
    calls = _patch_battery(
        monkeypatch,
        lambda p, o, t: result(t, overrides=o, obstacle_brakes=2, laps=1, lap_time_s=10.0),
    )
    note = tmp_path / "n.md"
    rb.main(
        [
            "--sweep",
            "g=1.0,2.0",
            "--repeats",
            "3",
            "--tracks",
            "lane_square",
            "--note",
            str(note),
        ]
    )
    # Every point three times, point after point.
    assert calls[0][1] == [{"g": 1.0}] * 3 + [{"g": 2.0}] * 3
    rows = [
        line
        for line in note.read_text().splitlines()
        if line.startswith("| ") and line[2].isdigit()
    ]
    assert len(rows) == 2
    cells = [c.strip() for c in rows[0].split("|")]
    # Laps 3, loop brakes 6, 2.00 per lap, lap time 10 (a mean, not a sum of the repeats).
    assert cells[6:10] == ["3", "6", "2.00", "10.0"]


def test_main_passes_track_parameters_to_every_battery(monkeypatch):
    calls = _patch_battery(monkeypatch, lambda p, o, t: result(t))
    rb.main(
        ["--track-params-json", '{"box_gap_m": 1.0}', "--tracks", "lane_open_side", "--no-note"]
    )
    assert calls[0][3]["track_params"] == {"box_gap_m": 1.0}
    calls.clear()
    rb.main(["--sweep", "g=1.0,2.0", "--track-params-json", '{"lane_width_m": 1.4}', "--no-note"])
    assert calls[0][3]["track_params"] == {"lane_width_m": 1.4}
