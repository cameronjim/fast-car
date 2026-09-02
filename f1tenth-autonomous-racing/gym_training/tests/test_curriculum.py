# speed-cap promotion logic, driven by a fake eval history instead of the simulator

import pytest

from f1rl.envs.curriculum import EvalRound, SpeedCapSchedule, racing_score


def feed(schedule, rates):
    """record a run of eval rounds, returning the cap after each one."""
    return [schedule.record(rate) or schedule.cap_mps for rate in rates]


def test_starts_at_the_configured_cap():
    schedule = SpeedCapSchedule(start_cap_mps=4.5)
    assert schedule.cap_mps == pytest.approx(4.5)
    assert not schedule.at_target


def test_one_clean_round_is_not_enough():
    schedule = SpeedCapSchedule(window_rounds=2, cooldown_rounds=2)
    assert schedule.record(1.0) is None
    assert schedule.cap_mps == pytest.approx(4.5)


def test_a_full_clean_window_promotes():
    schedule = SpeedCapSchedule(window_rounds=2, cooldown_rounds=2)
    schedule.record(1.0)
    assert schedule.record(1.0) == pytest.approx(5.0)


def test_a_dirty_round_blocks_the_promotion():
    schedule = SpeedCapSchedule(promote_rate=0.9)
    feed(schedule, [1.0, 0.8, 1.0])
    assert schedule.cap_mps == pytest.approx(4.5)
    schedule.record(1.0)
    assert schedule.cap_mps == pytest.approx(5.0)


def test_the_threshold_is_inclusive():
    schedule = SpeedCapSchedule(promote_rate=0.9)
    feed(schedule, [0.9, 0.9])
    assert schedule.cap_mps == pytest.approx(5.0)


def test_cooldown_stops_two_promotions_in_a_row():
    schedule = SpeedCapSchedule(window_rounds=1, cooldown_rounds=3)
    caps = feed(schedule, [1.0, 1.0, 1.0, 1.0, 1.0, 1.0])
    assert caps == [
        pytest.approx(4.5),
        pytest.approx(4.5),
        pytest.approx(5.0),
        pytest.approx(5.0),
        pytest.approx(5.0),
        pytest.approx(5.5),
    ]


def test_the_deciding_window_only_holds_rounds_at_the_current_cap():
    schedule = SpeedCapSchedule(window_rounds=2, cooldown_rounds=2)
    feed(schedule, [1.0, 1.0])
    assert schedule.cap_mps == pytest.approx(5.0)
    # the pre-promotion 1.0s must not carry over and promote again on one clean round
    assert schedule.record(1.0) is None
    assert schedule.record(1.0) == pytest.approx(5.5)


def test_the_cap_stops_at_the_target():
    schedule = SpeedCapSchedule(start_cap_mps=7.5, max_cap_mps=8.0, window_rounds=1, cooldown_rounds=1)
    feed(schedule, [1.0] * 8)
    assert schedule.cap_mps == pytest.approx(8.0)
    assert schedule.at_target
    assert schedule.record(1.0) is None


def test_a_partial_step_is_clamped_to_the_target():
    schedule = SpeedCapSchedule(start_cap_mps=7.8, max_cap_mps=8.0, window_rounds=1, cooldown_rounds=1)
    assert schedule.record(1.0) == pytest.approx(8.0)


def test_a_stalled_run_never_promotes():
    schedule = SpeedCapSchedule()
    feed(schedule, [0.5, 0.7, 0.0, 0.89, 0.6])
    assert schedule.cap_mps == pytest.approx(4.5)


def test_bad_settings_fail_loudly():
    with pytest.raises(ValueError):
        SpeedCapSchedule(step_mps=0.0)
    with pytest.raises(ValueError):
        SpeedCapSchedule(start_cap_mps=6.0, max_cap_mps=5.0)
    with pytest.raises(ValueError):
        SpeedCapSchedule(promote_rate=1.5)
    with pytest.raises(ValueError):
        SpeedCapSchedule(window_rounds=0)


def test_from_config_rejects_typos():
    with pytest.raises(ValueError):
        SpeedCapSchedule.from_config({"start_cap": 4.5})


def test_from_config_rejects_derived_state():
    with pytest.raises(ValueError):
        SpeedCapSchedule.from_config({"cap_mps": 6.0})


def test_from_config_defaults_when_absent():
    assert SpeedCapSchedule.from_config(None).cap_mps == pytest.approx(4.5)


def _round(rate, laps=2, lap_times=(60.0,), overtake_rate=0.0):
    return EvalRound(
        speed_cap_mps=5.0,
        episodes=10,
        collision_free_rate=rate,
        laps=laps,
        lap_times_sec=list(lap_times),
        overtake_rate=overtake_rate,
    )


def test_reliability_outranks_lap_time():
    slow_and_clean = _round(1.0, lap_times=(110.0,))
    fast_and_dirty = _round(0.9, lap_times=(40.0,))
    assert racing_score(slow_and_clean) > racing_score(fast_and_dirty)


def test_lap_time_breaks_ties_at_equal_reliability():
    assert racing_score(_round(1.0, lap_times=(50.0,))) > racing_score(_round(1.0, lap_times=(70.0,)))


def test_a_round_without_a_lap_scores_below_one_with_a_lap():
    assert racing_score(_round(1.0, laps=0, lap_times=())) < racing_score(_round(1.0))


def test_overtakes_break_ties_at_equal_reliability():
    assert racing_score(_round(1.0, overtake_rate=0.9)) > racing_score(
        _round(1.0, overtake_rate=0.2)
    )


def test_overtakes_never_outrank_surviving():
    # a whole round of overtakes moves the score by OVERTAKE_WEIGHT, less than 25% of clean rate
    passer = _round(0.75, overtake_rate=1.0)
    survivor = _round(1.0, overtake_rate=0.0)
    assert racing_score(survivor) > racing_score(passer)


def test_overtakes_outrank_lap_time():
    slow_passer = _round(1.0, lap_times=(90.0,), overtake_rate=1.0)
    fast_blocker = _round(1.0, lap_times=(35.0,), overtake_rate=0.0)
    assert racing_score(slow_passer) > racing_score(fast_blocker)


def test_a_run_with_no_opponent_scores_exactly_as_before():
    # overtake_rate defaults to zero, so every pre-m7 round keeps its old score
    assert racing_score(_round(1.0)) == pytest.approx(100.0 + 300.0 / 60.0)


def test_eval_round_summarizes_its_lap_times():
    finished = _round(1.0, lap_times=(70.0, 50.0, 60.0))
    assert finished.best_lap_time_sec == pytest.approx(50.0)
    assert finished.mean_lap_time_sec == pytest.approx(60.0)
    assert _round(1.0, laps=0, lap_times=()).best_lap_time_sec is None
