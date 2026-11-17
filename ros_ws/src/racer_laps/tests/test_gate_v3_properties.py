"""L2 (hypothesis): gate v3 on a ring and on a stadium under generated disturbances (brief 50).

* Ring, any heading gain error in plus or minus 3 percent, any speed 0.4 to 2.0 m/s: after N true
  laps the gate has N crossings within half a lap of the truth's, each lap within 4 percent.
* Stadium, any pattern of dropped scans (each scan lost with a probability up to 0.3: beyond that
  the line passage is lost outright, by gate v2.1 as well, and lap 1 cannot be flagged missed with
  no length estimate yet) and any odometry lag up to 0.2 s: never more crossings than true laps,
  and every lap not flagged missed has the truth's wheel length within the band.

Few examples: every example drives several laps through the ICP."""

from __future__ import annotations

import numpy as np
from hypothesis import HealthCheck, given, settings
from hypothesis import strategies as st
from racer_laps.engine import LapResult
from racer_laps.gate import BAND_ABS_M, BAND_REL
from synth import drive_stadium
from v3_support import RING, STADIUM, deliver, make_mem, run

SLOW = settings(
    max_examples=6,
    deadline=None,
    suppress_health_check=[HealthCheck.too_slow, HealthCheck.data_too_large],
    derandomize=True,
)


@SLOW
@given(
    gain=st.floats(min_value=0.95, max_value=1.05),
    speed=st.floats(min_value=0.4, max_value=2.0),
    launch_yaw=st.floats(min_value=-0.4, max_value=0.4),
)
def test_a_ring_has_n_crossings_each_within_4_percent_of_a_lap(
    gain: float, speed: float, launch_yaw: float
) -> None:
    """Brief 50b: any steering gain error within 5 percent, any speed 0.4 to 2.0 m/s, any start
    attitude within 0.4 rad: N crossings for N true laps, each lap within 4 percent."""
    n = 4
    truth_lap_s = RING.length_m / speed
    _mem, laps = run(RING, n + 0.45, speed, steer_scale=gain, launch_yaw_rad=launch_yaw)
    assert len(laps) == n, [(r.lap_index, round(r.lap_time_s, 2)) for r in laps]
    for k, r in enumerate(laps, start=1):
        assert abs(r.lap_time_s / truth_lap_s - 1.0) <= 0.04, (k, r.lap_time_s, truth_lap_s)
        assert abs(r.t_cross_s - k * truth_lap_s) <= 0.5 * truth_lap_s


@SLOW
@given(
    drop=st.floats(min_value=0.0, max_value=0.3),
    lag=st.floats(min_value=0.0, max_value=0.2),
    seed=st.integers(min_value=0, max_value=1000),
)
def test_a_stadium_never_gets_more_crossings_than_true_laps(
    drop: float, lag: float, seed: int
) -> None:
    laps_driven = 3.4
    mem = make_mem()
    rng = np.random.default_rng(3)
    pick = np.random.default_rng(seed)
    walls = STADIUM.walls()
    events = []
    for tick in drive_stadium(STADIUM, laps_driven, speed_mps=1.0, t0=0.0):
        if tick.kind == "scan" and pick.random() < drop:
            continue
        late = lag if tick.kind == "odom" else 0.0
        events.append((tick.t + late, 0 if tick.kind != "scan" else 1, tick))
    events.sort(key=lambda e: (e[0], e[1]))
    out: list[LapResult] = []
    for _t, _k, tick in events:
        deliver(mem, tick, walls, rng, out)
    true_laps = int(laps_driven)
    assert len(out) <= true_laps, [(r.lap_index, r.t_cross_s) for r in out]
    band = max(BAND_ABS_M, BAND_REL * STADIUM.length_m)
    for r in out:
        if not r.missed_gate:
            assert abs(r.length_m - STADIUM.length_m) <= band, r
