"""L1 tests for the race auto-tune harness (tools/race_tuning/, tools/race_tune.py).

Everything here is pure: no simulator, no ROS. The harness's runner needs the ROS workspace, so
`runs.execute` is exercised with a fake subprocess; the selection rule, the track sampler, the
statistics and the replay checks run on hand-built records. The statistics are checked against
known answers AND against synthetic data with a known effect (the harness must recover it).
"""

from __future__ import annotations

import json
import math
import random
from pathlib import Path

import pytest
from race_tuning import (
    replay_check,
    report,
    runs,
    selection,
    selection_gate,
    space,
    stats,
    tracks,
)

# -- tracks ----------------------------------------------------------------------------------------


def test_sample_track_is_deterministic_and_uses_no_global_random_state():
    random.seed(123)
    before = random.random()
    random.seed(123)
    a = tracks.sample_track(7)
    b = tracks.sample_track(7)
    assert a == b
    assert random.random() == before  # the sampler did not advance the global generator


def test_sampled_tracks_stay_inside_the_family_and_are_valid_bridge_tracks():
    for seed in list(tracks.TUNING_SEEDS) + list(tracks.HELD_OUT_SEEDS) + list(range(2000, 2200)):
        t = tracks.sample_track(seed)
        p = t.params_dict()
        assert t.track in ("stadium", "rounded_rect")
        assert 0.85 <= p["lane_width_m"] <= 1.2
        assert 1.0 <= p["straight_length_m"] <= 3.0
        assert 0.0 <= p["island_radius_m"] <= 0.6
        assert ("island_width_m" in p) == (t.track == "rounded_rect")
        assert t.seed == seed
        assert t.name  # a file-name-safe id
        assert "/" not in t.name and " " not in t.name


def test_tuning_and_held_out_seeds_are_disjoint_and_so_are_their_tracks():
    assert set(tracks.TUNING_SEEDS).isdisjoint(tracks.HELD_OUT_SEEDS)
    tuning = tracks.tuning_tracks()
    held = tracks.held_out_tracks()
    assert len(tuning) >= 5 and len(held) == len(tracks.HELD_OUT_SEEDS)
    tuning_keys = {t.key() for t in tuning}
    held_keys = [t.key() for t in held]
    assert tuning_keys.isdisjoint(held_keys)
    assert len(set(held_keys)) == len(held_keys)  # no held-out track twice
    assert len({t.name for t in tuning}) == len(tuning)


def test_held_out_tracks_can_be_asked_for_fewer_or_more():
    assert len(tracks.held_out_tracks(3)) == 3
    many = tracks.held_out_tracks(20)
    assert len(many) == 20
    assert len({t.key() for t in many}) == 20


def test_the_family_is_varied_enough_to_matter():
    kinds = {tracks.sample_track(s).track for s in range(1, 60)}
    radii = {tracks.sample_track(s).params_dict()["island_radius_m"] for s in range(1, 60)}
    assert kinds == {"stadium", "rounded_rect"}
    assert 0.0 in radii and max(radii) >= 0.4  # thin islands and round ends both appear


# -- stats -----------------------------------------------------------------------------------------


def test_wilson_interval_known_answers():
    low, high = stats.wilson_interval(8, 10)
    assert low == pytest.approx(0.4902, abs=2e-3)
    assert high == pytest.approx(0.9433, abs=2e-3)
    low, high = stats.wilson_interval(10, 10)
    assert high == pytest.approx(1.0, abs=1e-9) and low == pytest.approx(0.7225, abs=2e-3)
    low, high = stats.wilson_interval(0, 10)
    assert low == 0.0 and high == pytest.approx(0.2775, abs=2e-3)
    assert stats.wilson_interval(0, 0) == (0.0, 1.0)


def test_mean_ci_contains_the_mean_and_shrinks_with_more_data():
    rng = random.Random(1)
    small = [rng.gauss(10.0, 1.0) for _ in range(8)]
    large = [rng.gauss(10.0, 1.0) for _ in range(200)]
    m_small, lo_small, hi_small = stats.mean_ci(small)
    m_large, lo_large, hi_large = stats.mean_ci(large)
    assert lo_small <= m_small <= hi_small and lo_large <= m_large <= hi_large
    assert hi_large - lo_large < hi_small - lo_small
    assert stats.mean_ci([3.0]) == (3.0, 3.0, 3.0)
    assert all(math.isnan(v) for v in stats.mean_ci([]))
    assert stats.mean_ci(small, seed=5) == stats.mean_ci(small, seed=5)  # reproducible


def test_paired_ratio_recovers_a_known_effect():
    rng = random.Random(2)
    base = [rng.uniform(15.0, 35.0) for _ in range(40)]
    race = [b * 0.62 * rng.uniform(0.97, 1.03) for b in base]  # a 38 percent faster car
    mean, low, high = stats.paired_ratio_ci(race, base)
    assert mean == pytest.approx(0.62, abs=0.01)
    assert low <= mean <= high and low > 0.60
    assert high < 0.65  # and the interval excludes "no effect" by a mile


def test_paired_ratio_drops_bad_denominators_and_checks_lengths():
    mean, _, _ = stats.paired_ratio_ci([2.0, 5.0, 1.0], [4.0, 0.0, math.nan])
    assert mean == pytest.approx(0.5)
    with pytest.raises(ValueError):
        stats.paired_ratio_ci([1.0], [1.0, 2.0])


# -- selection -------------------------------------------------------------------------------------


def _rec(
    track="t",
    direction="ccw",
    lap=20.0,
    laps=2,
    brakes=0,
    escapes=0,
    contacts=0,
    clr=0.1,
    completed=True,
):
    return selection.RunRecord(
        track, direction, 2, laps, lap, brakes, escapes, contacts, clr, completed
    )


def _runs(lap, **kw):
    return [
        _rec(track=f"t{i}", direction=d, lap=lap, **kw) for i in range(3) for d in ("ccw", "cw")
    ]


def test_run_record_reduces_a_runner_result():
    result = {
        "laps": 2,
        "lap_time_s": 12.5,
        "obstacle_brakes": 3,
        "escapes": 1,
        "wall_contacts": 0,
        "min_body_clearance_m": 0.07,
        "stopped_because": "laps",
    }
    r = selection.run_record(result, "tr", "cw", 2)
    assert (r.laps, r.lap_time_s, r.brakes, r.escapes, r.contacts) == (2, 12.5, 3, 1, 0)
    assert r.completed and r.track == "tr" and r.direction == "cw"
    assert not selection.run_record(
        {**result, "stopped_because": "stalled"}, "t", "cw", 2
    ).completed
    assert not selection.run_record({**result, "laps": 1}, "t", "cw", 2).completed
    err = selection.run_record({"error": "node died"}, "t", "cw", 2)
    assert not err.completed and math.isnan(err.lap_time_s)


def test_aggregate_and_admissibility_boundaries():
    base = selection.aggregate(_runs(20.0, brakes=2))
    assert base.runs == 6 and base.completed == 6 and base.contacts == 0
    assert base.brakes_per_lap == pytest.approx(2 / 2)
    good = selection.aggregate(_runs(12.0, brakes=2), _runs(20.0, brakes=2))
    assert good.mean_ratio == pytest.approx(0.6)
    assert selection.admissible(good, base) == (True, [])

    def violates(**kw):
        agg = selection.aggregate(_runs(12.0, **{"brakes": 2, **kw}), _runs(20.0, brakes=2))
        return selection.admissible(agg, base)

    assert not violates(contacts=1)[0]
    assert not violates(completed=False)[0]
    assert not violates(clr=0.039)[0]
    assert violates(clr=0.04)[0]  # exactly at the minimum is allowed
    assert not violates(brakes=4)[0]  # 2.0 per lap vs baseline 1.0 + 0.5
    assert violates(brakes=3)[0]  # exactly 1.5 per lap is allowed
    assert not violates(escapes=4)[0]  # 2.0 escapes per lap vs 0.0 + 0.5
    assert violates(escapes=1)[0]  # exactly 0.5 per lap is allowed
    why = violates(contacts=1, clr=0.0)[1]
    assert len(why) >= 2  # every violated clause is named


def test_a_candidate_with_no_comparable_baseline_run_is_not_admissible():
    base = selection.aggregate(_runs(20.0))
    other = [_rec(track="elsewhere", lap=10.0)]
    agg = selection.aggregate(other, _runs(20.0))
    ok, why = selection.admissible(agg, base)
    assert not ok and any("baseline" in w for w in why)


def test_rank_orders_by_ratio_and_breaks_ties_by_brakes_then_speed():
    base_records = _runs(20.0, brakes=2)
    base = selection.aggregate(base_records)

    def entry(lap, brakes, speed):
        recs = _runs(lap, brakes=brakes)
        return ({"max_speed_mps": speed}, selection.aggregate(recs, base_records))

    candidates = {
        "slow": entry(14.0, 2, 0.64),
        "fast": entry(10.0, 2, 1.0),
        "fast_but_unsafe": entry(8.0, 20, 1.0),  # inadmissible: far more brakes
        "tied_more_brakes": entry(10.05, 3, 0.8),
        "tied_fewer_brakes": entry(10.08, 1, 0.8),
    }
    ranked = [cid for cid, _, _ in selection.rank(candidates, base)]
    assert "fast_but_unsafe" not in ranked
    # fast (ratio 0.50), tied_more_brakes (0.5025) and tied_fewer_brakes (0.504) are within the
    # tie band of each other: fewest brakes first.
    assert ranked[0] == "tied_fewer_brakes"
    assert ranked.index("fast") < ranked.index("slow")
    assert ranked[-1] == "slow"


def test_rank_is_empty_when_nothing_is_admissible():
    base = selection.aggregate(_runs(20.0))
    bad = selection.aggregate(_runs(10.0, contacts=1), _runs(20.0))
    assert selection.rank({"bad": ({}, bad)}, base) == []


# -- the wide family and the gate rule (2026-11-12 retune) --------------------------------------------


def test_wide_tracks_are_deterministic_wide_and_valid_bridge_tracks():
    assert tracks.sample_wide_track(2003) == tracks.sample_wide_track(2003)
    for seed in list(tracks.WIDE_TUNING_SEEDS) + list(tracks.WIDE_HELD_OUT_SEEDS):
        t = tracks.sample_wide_track(seed)
        p = t.params_dict()
        assert t.track == "stadium" and t.seed == seed
        assert 1.3 <= p["lane_width_m"] <= 1.5
        assert 1.5 <= p["straight_length_m"] <= 3.0
        assert 0.0 <= p["island_radius_m"] <= 0.6
        assert "/" not in t.name and " " not in t.name
    # None of them is a narrow-family lane.
    assert min(t.params_dict()["lane_width_m"] for t in tracks.wide_tuning_tracks()) >= 1.3


def test_wide_tuning_and_held_out_tracks_are_disjoint_and_distinct():
    tuning = tracks.wide_tuning_tracks()
    held = tracks.wide_held_out_tracks()
    assert len(tuning) >= 5 and len(held) == len(tracks.WIDE_HELD_OUT_SEEDS)
    assert {t.key() for t in tuning}.isdisjoint(t.key() for t in held)
    assert len({t.key() for t in held}) == len(held)
    assert len(tracks.wide_tuning_tracks(3)) == 3 and len(tracks.wide_held_out_tracks(4)) == 4
    assert set(tracks.WIDE_TUNING_SEEDS).isdisjoint(tracks.WIDE_HELD_OUT_SEEDS)
    assert set(tracks.WIDE_TUNING_SEEDS).isdisjoint(tracks.TUNING_SEEDS + tracks.HELD_OUT_SEEDS)


def test_family_tracks_dispatches_and_refuses_an_unknown_family():
    assert tracks.family_tracks("narrow", True) == tracks.tuning_tracks()
    assert tracks.family_tracks("narrow", False, 3) == tracks.held_out_tracks(3)
    assert tracks.family_tracks("wide", True) == tracks.wide_tuning_tracks()
    assert tracks.family_tracks("wide", False, 2) == tracks.wide_held_out_tracks(2)
    with pytest.raises(ValueError, match="unknown track family"):
        tracks.family_tracks("huge", True)


def test_gate_rule_is_absolute_not_relative_to_the_baseline():
    assert selection_gate.MAX_BRAKES_PER_LAP == 0.25 and selection_gate.MAX_ESCAPES_PER_LAP == 0.25
    base_records = _runs(20.0, brakes=40)  # a baseline that brakes constantly

    def agg(lap, **kw):
        return selection.aggregate(_runs(lap, **kw), base_records)

    assert selection_gate.admissible(agg(12.0, brakes=0)) == (True, [])
    # 0.25 per lap exactly is allowed: 6 runs x 2 laps = 12 laps, 3 brakes.
    assert selection_gate.admissible(agg(12.0, brakes=0))[0]
    runs_3 = [
        _rec(track=f"t{i}", direction=d, lap=12.0, brakes=0)
        for i in range(3)
        for d in ("ccw", "cw")
    ]
    runs_3[0] = _rec(track="t0", direction="ccw", lap=12.0, brakes=3)
    assert selection_gate.admissible(selection.aggregate(runs_3, base_records))[0]
    runs_4 = list(runs_3)
    runs_4[0] = _rec(track="t0", direction="ccw", lap=12.0, brakes=4)
    ok, why = selection_gate.admissible(selection.aggregate(runs_4, base_records))
    assert not ok and any("brakes per lap" in w for w in why)
    # The baseline's own brakes do not excuse the candidate (the relative rule would).
    assert selection.admissible(
        selection.aggregate(runs_4, base_records), selection.aggregate(base_records)
    )[0]


def test_gate_rule_names_every_violated_clause_and_needs_a_baseline_run():
    base_records = _runs(20.0)
    bad = selection.aggregate(
        _runs(10.0, contacts=1, completed=False, clr=0.0, brakes=5, escapes=5), base_records
    )
    ok, why = selection_gate.admissible(bad)
    assert not ok
    for word in ("completed", "wall contacts", "clearance", "brakes per lap", "escapes per lap"):
        assert any(word in w for w in why), word
    elsewhere = selection.aggregate([_rec(track="elsewhere")], base_records)
    assert any("baseline" in w for w in selection_gate.admissible(elsewhere)[1])


def test_gate_rank_orders_by_ratio_and_breaks_ties_by_brakes_escapes_then_speed():
    base_records = _runs(20.0)

    def entry(lap, speed, **kw):
        return ({"max_speed_mps": speed}, selection.aggregate(_runs(lap, **kw), base_records))

    candidates = {
        "slow": entry(14.0, 1.0),
        "fast": entry(10.0, 1.6),
        "fast_but_brakes": entry(8.0, 1.6, brakes=6),  # inadmissible: 1.0 brake per lap
        "tie_escape": entry(10.1, 1.3, escapes=1),
        "tie_clean_slower": entry(10.2, 1.0),
        "tie_clean_faster": entry(10.3, 1.3),
    }
    ranked = [cid for cid, _, _ in selection_gate.rank(candidates)]
    assert "fast_but_brakes" not in ranked
    # fast (0.50), tie_escape (0.505), tie_clean_slower (0.51), tie_clean_faster (0.515) are within
    # the tie band: fewest brakes, then fewest escapes, then lower top speed.
    assert ranked[0] == "tie_clean_slower"  # 0 brakes, 0 escapes, the lowest top speed
    assert ranked[-1] == "slow"
    assert selection_gate.rank({"bad": ({}, selection.aggregate(_runs(10.0, contacts=1)))}) == []


# -- space -----------------------------------------------------------------------------------------


def test_sample_candidates_is_seeded_distinct_and_starts_at_the_reference_point():
    a = space.sample_candidates(20, seed=3)
    assert a == space.sample_candidates(20, seed=3)
    assert a != space.sample_candidates(20, seed=4)
    assert len({json.dumps(c, sort_keys=True) for c in a}) == 20
    assert a[0]["apex_offset_m"] == 0.0 and a[0]["track_out_offset_m"] == 0.0
    for cand in a:
        for key, value in cand.items():
            assert value in space.SPACE[key]


def test_sample_candidates_cannot_exceed_the_space():
    size = math.prod(len(v) for v in space.SPACE.values())
    assert len(space.sample_candidates(size + 50, seed=1)) <= size


def test_neighbours_move_one_parameter_one_step():
    params = {k: v[1] for k, v in space.SPACE.items()}
    near = space.neighbours(params)
    assert len(near) == 2 * len(space.SPACE)  # every middle value has both neighbours
    for n in near:
        diff = [k for k in params if n[k] != params[k]]
        assert len(diff) == 1
        values = space.SPACE[diff[0]]
        assert abs(values.index(n[diff[0]]) - values.index(params[diff[0]])) == 1
    edge = {k: v[0] for k, v in space.SPACE.items()}
    assert len(space.neighbours(edge)) == len(space.SPACE)
    assert space.neighbours({"max_speed_mps": 123.0}) == []


def test_candidate_id_is_stable_and_distinct():
    a = {k: v[0] for k, v in space.SPACE.items()}
    b = {**a, "apex_offset_m": 0.1}
    assert space.candidate_id(a) == space.candidate_id(dict(a))
    assert space.candidate_id(a) != space.candidate_id(b)
    assert "/" not in space.candidate_id(a)


def test_the_speed_ceiling_of_the_space_is_one_metre_per_second():
    assert max(space.SPACE["max_speed_mps"]) == 1.0
    assert space.BASE["physics_speed"] is True
    assert space.BASE["speed_time_constant_s"] == 0.0  # a low-pass would delay the stopping limit


# -- runs ------------------------------------------------------------------------------------------


def test_make_specs_covers_every_track_in_both_directions():
    tr = tracks.tuning_tracks()[:3]
    specs = runs.make_specs("v", {"a": 1}, tr, laps=2)
    assert len(specs) == 6
    assert {s.direction for s in specs} == {"ccw", "cw"}
    assert len({s.run_id for s in specs}) == 6


def test_command_carries_the_track_direction_profile_and_params(tmp_path):
    spec = runs.make_specs("v", {"physics_speed": True}, [tracks.sample_track(1)], 3, "race")[1]
    assert spec.direction == "cw"
    cmd = runs.command(spec, 207, tmp_path, python="py")
    assert cmd[0] == "py" and cmd[1].endswith("track_family_runner.py")
    assert "--reverse" in cmd
    assert cmd[cmd.index("--profile") + 1] == "race"
    assert json.loads(cmd[cmd.index("--params-json") + 1]) == {"physics_speed": True}
    assert cmd[cmd.index("--stop-after-laps") + 1] == "3"
    assert cmd[cmd.index("--domain-id") + 1] == "207"
    assert cmd[cmd.index("--seed") + 1] == str(spec.track.seed)
    ccw = runs.command(runs.make_specs("v", {}, [tracks.sample_track(1)], 3)[0], 1, tmp_path)
    assert "--reverse" not in ccw


def test_run_one_resumes_from_an_existing_result_and_records_wall_time(tmp_path, monkeypatch):
    spec = runs.make_specs("v", {}, [tracks.sample_track(1)], 2)[0]
    path = spec.path(tmp_path)
    path.parent.mkdir(parents=True)
    path.write_text(json.dumps({"laps": 2, "marker": "old"}))
    called = []
    monkeypatch.setattr(runs.subprocess, "run", lambda *a, **k: called.append(a))
    assert runs.run_one(spec, 200, tmp_path)["marker"] == "old"
    assert not called  # a finished run is not run again


def test_run_one_runs_a_missing_result_and_reports_a_dead_runner(tmp_path, monkeypatch):
    spec = runs.make_specs("v", {}, [tracks.sample_track(1)], 2)[0]

    class Proc:
        returncode = 3
        stderr = "boom"

    monkeypatch.setattr(runs.subprocess, "run", lambda *a, **k: Proc())
    result = runs.run_one(spec, 200, tmp_path)
    assert "boom" in result["error"] and "wall_s" in result
    record = runs.load_records(tmp_path, [spec])[0]
    assert not record.completed


def test_execute_runs_every_spec_once_with_distinct_concurrent_domains(tmp_path, monkeypatch):
    specs = runs.make_specs("v", {}, tracks.tuning_tracks()[:4], 2)
    seen: list[tuple[str, int]] = []

    def fake(spec, domain, out):
        seen.append((spec.run_id, domain))
        return {"laps": 2}

    monkeypatch.setattr(runs, "run_one", fake)
    done = []
    runs.execute(specs, tmp_path, jobs=3, domain_base=300, progress=lambda s, r: done.append(s))
    assert sorted(r for r, _ in seen) == sorted(s.run_id for s in specs)
    assert {d for _, d in seen} <= {300, 301, 302}
    assert len(done) == len(specs)


def test_load_records_skips_runs_that_have_not_finished(tmp_path):
    specs = runs.make_specs("v", {}, tracks.tuning_tracks()[:2], 2)
    p = specs[0].path(tmp_path)
    p.parent.mkdir(parents=True)
    p.write_text(
        json.dumps(
            {
                "laps": 2,
                "lap_time_s": 11.0,
                "stopped_because": "laps",
                "obstacle_brakes": 0,
                "escapes": 0,
                "wall_contacts": 0,
                "min_body_clearance_m": 0.1,
            }
        )
    )
    records = runs.load_records(tmp_path, specs)
    assert len(records) == 1 and records[0].completed and records[0].lap_time_s == 11.0


# -- report ----------------------------------------------------------------------------------------


def test_comparison_table_has_a_row_per_variant_and_direction_with_intervals():
    base = _runs(20.0, brakes=2)
    race = _runs(12.0, brakes=1)
    table = report.comparison_table({"baseline": base, "race": race}, "baseline")
    lines = table.splitlines()
    assert len(lines) == 2 + 2 * 3  # header + rule + (both, ccw, cw) per variant
    assert "| baseline | both |" in table and "| race | cw |" in table
    assert "0.600 [0.600, 0.600]" in table  # a constant 12/20 ratio: zero-width interval
    assert "(61% to 100%)" in table  # Wilson, 6 of 6
    track_table = report.per_track_table({"baseline": base, "race": race})
    assert "| t0 | ccw |" in track_table and "12.0s" in track_table


def test_per_track_table_marks_a_dnf():
    dnf = _rec(lap=math.nan, laps=1, completed=False)
    table = report.per_track_table({"v": [dnf]})
    assert "DNF" in table


# -- replay checks ---------------------------------------------------------------------------------

GEOMETRY = {
    "max_steering_rad": 0.449,
    "max_acceleration_mps2": 9.51,
    "min_forward_clearance_m": 0.1,
    "stop_deceleration_mps2": 1.0,
    "stop_latency_s": 0.2,
}


def _row(steer=0.0, speed=0.5, target=None, free=None, valid=True, clamped=False):
    return {
        "valid": valid,
        "filtered_steering_rad": steer,
        "speed_mps": speed,
        "target_speed_mps": target,
        "free_path_m": free,
        "clamped": clamped,
    }


def test_reversals_counts_swings_not_wiggles():
    assert replay_check.reversals([0, 0.3, 0, 0.3, 0, 0.3]) == 4
    assert replay_check.reversals([0, 0.05, 0, 0.05, 0]) == 0  # under the swing
    assert replay_check.reversals([0.0, 0.2, 0.4, 0.5]) == 0  # monotone
    assert replay_check.reversals([]) == 0


def test_stopping_limit_matches_the_c_plus_plus_hand_case():
    assert replay_check.stopping_limit(0.65, 0.15, 1.0, 0.2) == pytest.approx(math.sqrt(1.04) - 0.2)
    assert replay_check.stopping_limit(0.1, 0.15, 1.0, 0.2) == 0.0


def test_analyse_flags_each_kind_of_failure():
    params = {
        "max_speed_mps": 1.0,
        "min_speed_mps": 0.36,
        "physics_speed": True,
        "speed_rate_limit_margin_fraction": 0.1,
        "stopping_margin_m": 0.05,
    }
    stamps = [i * 0.1 for i in range(20)]
    clean = [_row(0.1 * math.sin(i / 3), 0.5, target=0.5, free=3.0) for i in range(20)]
    base = replay_check.analyse(clean, stamps, GEOMETRY, params)
    checks = replay_check.hard_checks(base, base)
    assert all(checks.values()), checks

    over_limit = [dict(r) for r in clean]
    over_limit[5]["filtered_steering_rad"] = 0.6
    assert not replay_check.hard_checks(
        replay_check.analyse(over_limit, stamps, GEOMETRY, params), base
    )["finite_and_within_limits"]

    too_fast = [dict(r) for r in clean]
    too_fast[5]["speed_mps"] = 1.5
    assert not replay_check.hard_checks(
        replay_check.analyse(too_fast, stamps, GEOMETRY, params), base
    )["finite_and_within_limits"]

    nan_row = [dict(r) for r in clean]
    nan_row[3]["speed_mps"] = math.nan
    assert not replay_check.analyse(nan_row, stamps, GEOMETRY, params)["finite"]

    jumpy = [dict(r) for r in clean]
    jumpy[7]["filtered_steering_rad"] = 0.44
    assert not replay_check.hard_checks(
        replay_check.analyse(jumpy, stamps, GEOMETRY, params), base
    )["no_steering_jumps"]

    # A jump the baseline makes too (the real bags where the car left the lane) is not the race
    # profile's fault; a bigger one is.
    jumpy_base = replay_check.analyse(jumpy, stamps, GEOMETRY, params)
    assert replay_check.hard_checks(jumpy_base, jumpy_base)["no_steering_jumps"]
    worse = [dict(r) for r in jumpy]
    worse[7]["filtered_steering_rad"] = 0.449
    worse[8]["filtered_steering_rad"] = -0.2  # a 0.65 rad step in one scan
    assert not replay_check.hard_checks(
        replay_check.analyse(worse, stamps, GEOMETRY, params), jumpy_base
    )["no_steering_jumps"]

    surging = [dict(r) for r in clean]
    surging[10]["speed_mps"] = 0.95  # +0.45 m/s in 0.1 s against 5 ticks of a 0.951 bound (0.095)
    assert not replay_check.hard_checks(
        replay_check.analyse(surging, stamps, GEOMETRY, params), base
    )["acceleration_within_limiter"]

    violating = [dict(r) for r in clean]
    violating[4] = _row(0.0, 0.9, target=0.9, free=0.3)  # 0.9 m/s asked with 0.15 m of room
    result = replay_check.analyse(violating, stamps, GEOMETRY, params)
    assert result["stop_violations"] == 1
    assert not replay_check.hard_checks(result, base)["stopping_law_holds"]

    twitchy = [_row(0.3 * (i % 2), 0.5, target=0.5, free=3.0) for i in range(20)]
    assert not replay_check.hard_checks(
        replay_check.analyse(twitchy, stamps, GEOMETRY, params), base
    )["not_twitchier_than_baseline"]


def test_a_speed_ramp_exactly_at_the_limiter_passes_whatever_the_scan_spacing():
    # 0.951 m/s^2 is 0.01902 per 50 Hz tick. Scans 0.1 s apart get 5 ticks (0.0951), scans 0.03 s
    # apart get 2 ticks (the CLI steps at least one, rounds to the nearest): both are exactly at
    # the limiter's bound, so a check against accel * scan interval alone would call them
    # violations, and the real violation below (twice the bound) still fails.
    params = {"max_speed_mps": 1.0, "min_speed_mps": 0.0, "speed_rate_limit_margin_fraction": 0.1}
    step = 0.951 / 50.0
    gaps = [0.1, 0.1, 0.03, 0.03, 0.1, 0.045, 0.1]
    stamps = [0.0]
    speeds = [0.0]
    for gap in gaps:
        stamps.append(stamps[-1] + gap)
        speeds.append(speeds[-1] + step * max(1, round(gap * 50)))
    rows = [_row(0.0, v) for v in speeds]
    result = replay_check.analyse(rows, stamps, GEOMETRY, params)
    assert replay_check.hard_checks(result, result)["acceleration_within_limiter"]
    rows[-1] = _row(0.0, speeds[-2] + 2.0 * step * 5)
    result = replay_check.analyse(rows, stamps, GEOMETRY, params)
    assert not replay_check.hard_checks(result, result)["acceleration_within_limiter"]


def test_the_stopping_check_is_skipped_for_the_range_law():
    params = {"max_speed_mps": 1.0, "min_speed_mps": 0.36, "physics_speed": False}
    rows = [_row(0.0, 0.5, target=None, free=None) for _ in range(5)]
    result = replay_check.analyse(rows, [i * 0.1 for i in range(5)], GEOMETRY, params)
    assert result["stop_checked"] == 0 and result["stop_violations"] == 0


def test_steering_departure_and_pre_brake_speeds():
    a = [_row(0.0), _row(0.2), _row(-0.1)]
    b = [_row(0.0), _row(0.1), _row(0.1)]
    d = replay_check.steering_departure(a, b)
    assert d["max_rad"] == pytest.approx(0.2) and d["median_rad"] == pytest.approx(0.1)
    assert math.isnan(replay_check.steering_departure([], [])["median_rad"])

    from follower_replay import bag

    rows = [_row(speed=0.5 + 0.1 * i) for i in range(30)]
    stamps = [i * 0.1 for i in range(30)]
    events = [
        bag.EventRecord(2.0, bag.SEVERITY_BRAKE, bag.PHASE_ENGAGE, 0.0, "ttc", ""),
        bag.EventRecord(2.5, bag.SEVERITY_WARNING, bag.PHASE_ENGAGE, 0.0, "ttc", ""),  # not a brake
        bag.EventRecord(2.6, bag.SEVERITY_BRAKE, bag.PHASE_RELEASE, 0.1, "ttc", ""),  # a release
    ]
    recorded = [bag.DriveRecord(i * 0.05, 0.0, 0.4, 0.0) for i in range(60)]
    got = replay_check.pre_brake_speeds(rows, stamps, events, recorded)
    assert got is not None and got["events"] == 1.0
    assert got["replayed_median_mps"] == pytest.approx(0.5 + 0.1 * 14.5)  # scans 10 to 19
    assert got["recorded_median_mps"] == pytest.approx(0.4)
    assert replay_check.pre_brake_speeds(rows, stamps, [], recorded) is None


def test_bag_paths_finds_files_and_directories_of_mcap(tmp_path):
    (tmp_path / "a_bag").mkdir()
    (tmp_path / "a_bag" / "x.mcap").write_bytes(b"")
    (tmp_path / "empty").mkdir()
    (tmp_path / "loose.mcap").write_bytes(b"")
    (tmp_path / "notes.txt").write_text("x")
    found = {p.name for p in replay_check.bag_paths(tmp_path)}
    assert found == {"a_bag", "loose.mcap"}
    assert replay_check.bag_paths(tmp_path / "a_bag") == [tmp_path / "a_bag"]


def test_format_report_names_failed_checks():
    result = {
        "bag": "b",
        "duration_s": 60.0,
        "race": {
            "scans": 100,
            "speed_mean_mps": 0.7,
            "speed_max_mps": 0.9,
            "reversals_per_min": 10.0,
            "max_steering_step_rad": 0.1,
            "stop_violations": 1,
            "stop_checked": 90,
        },
        "baseline": {"speed_mean_mps": 0.5, "speed_max_mps": 0.6, "reversals_per_min": 9.0},
        "steering_departure": {"p99_rad": 0.2},
        "checks": {"stopping_law_holds": False, "no_steering_jumps": True},
        "passed": False,
    }
    text = replay_check.format_report([result])
    assert "FAIL: stopping_law_holds" in text and "1 of 90" in text


# -- replay's profile loader keeps the base of a profile built with ** ---------------------------


def test_replay_profile_loader_resolves_splatted_base_profiles():
    from follower_replay import replay

    profiles = replay.load_profiles()
    floor = profiles["floor-2026-10-06"]
    lane = profiles["lane-2026-10-25"]
    assert lane["target_mode"] == "lane_centre"
    for key, value in floor.items():
        assert lane[key] == value, key  # the lane profile used to lose all of these
    race = profiles["race-2026-11-12"]
    for key, value in lane.items():
        if key not in race:
            raise AssertionError(key)
    assert race["physics_speed"] is True
    assert race["lane_lookahead_m"] == 0.6 and race["outside_entry_offset_m"] == 0.25


def test_race_profile_only_adds_what_it_says_over_the_lane_profile():
    from follower_replay import replay

    profiles = replay.load_profiles()
    lane, race = profiles["lane-2026-10-25"], profiles["race-2026-11-12"]
    # The old profiles do not mention any race parameter: old behaviour by default.
    race_only = {
        "physics_speed",
        "a_lat_max_mps2",
        "apex_offset_m",
        "track_out_offset_m",
        "side_cap_distance_m",
        "stopping_margin_m",
        "compute_on_scan",
    }
    assert race_only.isdisjoint(lane)
    assert race_only.isdisjoint(profiles["floor-2026-10-06"])
    assert {"physics_speed"} <= set(race)


def test_the_harness_baseline_is_the_recommended_pre_race_setup():
    assert space.BASELINE == {
        "max_speed_mps": 0.64,
        "lane_lookahead_m": 0.8,
        "outside_entry_offset_m": 0.25,
    }
    assert replay_check.BASELINE_OVERRIDES == {
        "lane_lookahead_m": 0.8,
        "outside_entry_offset_m": 0.25,
    }
    assert "physics_speed" not in space.BASELINE  # the baseline is the old behaviour
    assert Path(runs.RUNNER).name == "track_family_runner.py"
