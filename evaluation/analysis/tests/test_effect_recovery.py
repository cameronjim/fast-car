"""Known effect recovery on seeded synthetic sessions (brief 21, section 4).

A session is 5 D-on runs of 10 laps. Lap k of a run takes mu_k + run offset + noise, with a known
linear fall in lap time. The 95 percent intervals must cover the truth at about their stated rate
(92 to 98 percent of 500 sessions) and a null effect must almost never pass item 2 of GP4."""

from __future__ import annotations

import numpy as np
from racer_eval.curves import RunLaps, lap_curve
from racer_eval.gates import gp4_decision

N_SESSIONS = 500
N_RUNS = 5
OFFSET_SD = 0.3
NOISE_SD = 0.2


def session(rng: np.random.Generator, slope: float, t0: float = 10.0) -> list[RunLaps]:
    """mu_k = t0 - slope * (k - 1). The lap 8 to 10 times are symmetric about mu_9, so the true
    Delta = T1 - median(T8..T10) is exactly 8 * slope."""
    runs = []
    for i in range(N_RUNS):
        offset = rng.normal(0.0, OFFSET_SD)
        times = [t0 - slope * k + offset + rng.normal(0.0, NOISE_SD) for k in range(10)]
        runs.append(RunLaps(f"r{i}", "d_on", tuple(times)))
    return runs


def test_curve_intervals_cover_the_true_mean_lap_time() -> None:
    rng = np.random.default_rng(20261113)
    slope = 0.15
    hits = {1: 0, 5: 0, 10: 0}
    for _ in range(N_SESSIONS):
        curve = lap_curve(session(rng, slope))
        for p in curve.points:
            if p.lap in hits:
                truth = 10.0 - slope * (p.lap - 1)
                assert p.interval is not None
                hits[p.lap] += p.interval.contains(truth)
    for lap, n in hits.items():
        assert 0.92 <= n / N_SESSIONS <= 0.98, (lap, n / N_SESSIONS)


def test_gp4_mean_delta_interval_covers_the_true_delta() -> None:
    rng = np.random.default_rng(20261114)
    slope = 0.15
    truth = 8 * slope
    hits = 0
    for _ in range(N_SESSIONS):
        reported = gp4_decision(session(rng, slope), session(rng, 0.0)).reported["mean_delta_s"]
        hits += reported["lo"] <= truth <= reported["hi"]
    assert 0.92 <= hits / N_SESSIONS <= 0.98, hits / N_SESSIONS


def test_a_real_improvement_passes_item_2_and_a_null_effect_does_not() -> None:
    rng = np.random.default_rng(20261115)
    item2 = ("5.2-2a", "5.2-2b", "5.2-2c")
    strong_pass = null_pass = null_interval_excludes_zero = 0
    for _ in range(N_SESSIONS):
        # t0 = 10, slope 0.45: Delta 3.6 s, r about 0.64
        strong = gp4_decision(session(rng, 0.45), session(rng, 0.0))
        strong_pass += all(strong.item(i).passed for i in item2)
        null = gp4_decision(session(rng, 0.0), session(rng, 0.0))
        null_pass += all(null.item(i).passed for i in item2)
        null_interval_excludes_zero += null.item("5.2-2b").passed
    assert strong_pass / N_SESSIONS >= 0.99
    assert null_pass / N_SESSIONS <= 0.05
    # the interval item alone is a two sided 95 percent test: at most about 2.5 percent
    assert null_interval_excludes_zero / N_SESSIONS <= 0.05
