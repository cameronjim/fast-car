"""L1: the lap node switches of brief 53 (brief 44 section 1): every default is the behaviour before
them, and each non-default behaviour on a built case. `value_profile_mode`, `station_uncertainty`,
`reset_on_change` and `zone_floor_mode`, plus D v1 on the sim truth source (the camera stays
refused)."""

from __future__ import annotations

import math

import numpy as np
import pytest
from external_support import DT, LAP_M, odom_model, steering_for
from racer_laps.engine import LapConfig, LapMemory
from racer_laps.lap_record import STATE_DRIVING
from racer_laps.station_set import StationSet

FLOOR = 0.3


# -- a station set with two built laps -------------------------------------------------------------


def lap_samples(speed_of, length: float = 6.0, n: int = 600):
    s = np.linspace(0.0, length, n)
    v = np.array([speed_of(x) for x in s])
    t = np.concatenate([[0.0], np.cumsum(np.diff(s) / np.maximum(v[1:], 1e-3))])
    return s, t, v, float(t[-1])


def built_set(**kw) -> StationSet:
    ss = StationSet(floor_mps=FLOOR, **kw)
    ss.set_reference_length(6.0)
    s, t, v, T = lap_samples(lambda x: 0.6)
    assert ss.add_lap(1, s, t, v, T, 6.0, 0.6)
    s, t, v, T = lap_samples(lambda x: 0.9 if x < 3.0 else 0.7)
    assert ss.add_lap(2, s, t, v, T, 6.0, 0.9)
    return ss


GRID = np.arange(0.0, 12.0, 0.1)


def test_station_set_defaults_are_update_and_floor_wins() -> None:
    a, b = built_set(), built_set(value_profile_mode="update", zone_floor_mode="floor_wins")
    assert a.value_profile_mode == "update" and a.zone_floor_mode == "floor_wins"
    for x, y in ((a, b),):
        assert np.array_equal(x.v_bound(GRID, 3, 0.2), y.v_bound(GRID, 3, 0.2))
        assert np.array_equal(x.cost_to_go(GRID), y.cost_to_go(GRID))
    # arrays() without the uncertainty argument is the old expression exactly
    s, v, q = a.arrays(1.0, 6.0, 3, 0.2)
    assert np.array_equal(v, np.maximum(a.v_bound(s, 3, 0.2), 0.0))
    assert np.array_equal(q, a.cost_to_go(s))


def test_unknown_modes_are_refused() -> None:
    with pytest.raises(ValueError):
        StationSet(value_profile_mode="sometimes")
    with pytest.raises(ValueError):
        StationSet(zone_floor_mode="maybe")
    with pytest.raises(ValueError):
        LapConfig(value_profile_mode="x")
    with pytest.raises(ValueError):
        LapConfig(zone_floor_mode="x")
    with pytest.raises(ValueError):
        LapConfig(reset_length_share=0.0)


def test_freeze_keeps_the_first_laps_value_and_still_learns_speed() -> None:
    upd = built_set()
    frz = built_set(value_profile_mode="freeze_after_first_clean_lap")
    first = StationSet(floor_mps=FLOOR)
    first.set_reference_length(6.0)
    s, t, v, T = lap_samples(lambda x: 0.6)
    first.add_lap(1, s, t, v, T, 6.0, 0.6)
    # Q and T_best are the first lap's; the update mode took the faster second lap
    assert np.allclose(frz.cost_to_go(GRID), first.cost_to_go(GRID))
    assert frz.best_lap_time_s == pytest.approx(first.best_lap_time_s)
    assert upd.best_lap_time_s < frz.best_lap_time_s
    # the speed profile learns from both laps in both modes
    assert np.array_equal(frz.v_safe(GRID), upd.v_safe(GRID))


def test_freeze_moves_to_the_next_lap_when_the_first_is_removed() -> None:
    frz = built_set(value_profile_mode="freeze_after_first_clean_lap")
    t2 = frz.laps[1].lap_time_s
    frz.remove_lap(1)
    assert frz.best_lap_time_s == pytest.approx(t2)


def test_cap_wins_holds_below_the_floor_and_never_lifts_a_zone() -> None:
    fw = built_set()
    cw = built_set(zone_floor_mode="cap_wins")
    for ss in (fw, cw):
        ss.add_zone(2.0, 0.35, created_lap=2, reason="brake")  # zone cap 0.25 < floor
        ss.add_zone(5.0, 0.55, created_lap=2, reason="brake")  # zone cap 0.45 > floor
    s = np.array([1.5, 4.5, 0.5])  # inside zone 1, inside zone 2, outside both
    b_fw = fw.v_bound(s, 3, 0.2)
    b_cw = cw.v_bound(s, 3, 0.2)
    assert b_fw[0] == pytest.approx(FLOOR)  # floor wins lifts the zone to the floor
    assert b_cw[0] == 0.0  # cap wins: below the floor is a hold
    assert b_cw[1] == pytest.approx(0.45) and b_fw[1] == pytest.approx(0.45)
    assert b_cw[2] == pytest.approx(b_fw[2])  # outside zones: unchanged
    assert np.all(cw.v_bound(GRID, 3, 0.2) >= 0.0)


def test_cap_wins_lifts_a_low_profile_outside_zones_to_the_floor() -> None:
    cw = StationSet(floor_mps=FLOOR, zone_floor_mode="cap_wins")
    cw.set_reference_length(6.0)
    s, t, v, T = lap_samples(lambda x: 0.05 if 2.0 < x < 2.5 else 0.6)
    cw.add_lap(1, s, t, v, T, 6.0, 0.6)
    assert np.min(cw.v_bound(GRID, 2, 0.0)) == pytest.approx(FLOOR)


def _dense_min(ss: StationSet, lo: float, hi: float) -> float:
    return float(np.min(ss.v_bound(np.linspace(lo, hi, 20001), 3, 0.2)))


def test_station_uncertainty_is_the_lowest_bound_over_the_interval() -> None:
    ss = built_set()
    ss.add_zone(4.6, 0.75, created_lap=2, reason="brake")  # a step inside the lap
    s = np.array([2.2, 3.2, 1.0, 4.9, 4.0])
    base = ss.v_bound(s, 3, 0.2)
    assert np.array_equal(ss.v_bound_uncertain(s, 3, 0.2, 0.0), base)
    w = np.array([0.6, 0.6, 0.6, 0.33, 0.07])
    wide = ss.v_bound_uncertain(s, 3, 0.2, w)
    for i in range(s.shape[0]):
        assert wide[i] == pytest.approx(_dense_min(ss, s[i] - w[i], s[i] + w[i]), abs=1e-9)
    assert wide[0] < base[0]  # 2.2 + 0.6 reaches the slow half of the profile
    assert wide[3] < base[3]  # 4.9 - 0.33 reaches into the zone that ends at 4.6
    assert np.all(wide <= base + 1e-12)


def test_station_uncertainty_grows_with_station_and_stops_at_the_cap() -> None:
    ss = built_set()
    rate, cap = 0.1, 0.4
    s, v_plain, _ = ss.arrays(0.0, 8.0, 3, 0.2)
    _, v_unc, _ = ss.arrays(0.0, 8.0, 3, 0.2, uncertainty=(rate, cap))
    assert np.all(v_unc <= v_plain + 1e-12)
    assert v_unc[0] == pytest.approx(v_plain[0])  # half width 0 at the crossing
    for i in range(s.shape[0]):
        half = min(rate * s[i], cap)
        assert v_unc[i] == pytest.approx(_dense_min(ss, s[i] - half, s[i] + half), abs=1e-9)
    assert np.any(v_unc < v_plain - 1e-6)


# -- the engine: reset on change and D v1 on the truth source --------------------------------------


class Rig:
    """`LapMemory` on the truth source with the given switches, a car on a loop, clean laps."""

    def __init__(self, mode: str = "v1", speed: float = 1.0, **kw: object) -> None:
        cfg = LapConfig(
            lap_source="truth",
            lap_mode=mode,
            gate=odom_model(),
            v_cap_start_mps=0.5,
            v_cap_ceiling_mps=1.3,
            **kw,  # type: ignore[arg-type]
        )
        self.mem = LapMemory(cfg)
        self.speed = speed
        self.k = 0
        self.next_status_s = 0.0
        self.mem.on_drive(steering_for(LAP_M))
        self.mem.on_lap_source_status(0.0, "running", 0, 1.0)
        self.mem.on_planner_status(STATE_DRIVING, 0.3)
        self.mem.on_wheel_odom(0.0, speed)
        self.results: list = []  # type: ignore[type-arg]

    def drive_until(self, t_end: float) -> None:
        while (self.k + 1) * DT <= t_end + 1e-9:
            self.k += 1
            t = self.k * DT
            if t >= self.next_status_s - 1e-9:
                self.mem.on_lap_source_status(t, "running", 0, 1.0)
                self.next_status_s = t + 1.0
            self.mem.on_wheel_odom(t, self.speed)
            self.mem.on_planner_status(STATE_DRIVING, 0.3)

    def event(self, t_cross: float, lap_no: int):  # type: ignore[no-untyped-def]
        arrival = t_cross + 0.05
        self.drive_until(arrival)
        r = self.mem.on_lap_source_event(arrival, lap_no, t_cross)
        if r is not None:
            self.results.append(r)
        return r

    def laps(self, n: int) -> None:
        for _ in range(n):
            k = len(self.results) + 1
            self.event(LAP_M / self.speed * k, k)


def test_v1_on_truth_learns_and_raises_the_cap_by_the_bound() -> None:
    rig = Rig()
    rig.laps(2)
    assert rig.mem.station_set.n_laps == 2
    g = rig.mem.guidance()
    assert g.s_m.shape[0] > 2 and g.v_cap_mps > 0.5
    assert "resets" not in rig.mem.status()  # reset_on_change off: no new status key


def test_reset_on_change_off_by_default_changes_nothing() -> None:
    a, b = Rig(), Rig(reset_on_change=False)
    a.laps(3)
    b.laps(3)
    ga, gb = a.mem.guidance(), b.mem.guidance()
    assert ga.v_cap_mps == gb.v_cap_mps
    assert np.array_equal(ga.v_ref_mps, gb.v_ref_mps)
    assert a.mem.resets == [] and b.mem.resets == []


def test_reset_after_two_missed_crossings_running() -> None:
    rig = Rig(reset_on_change=True)
    rig.laps(3)
    assert rig.mem.station_set.n_laps == 3
    r = rig.event(LAP_M * 4, 5)  # a skipped number: doubtful (missed)
    assert r is not None and r.missed_gate
    assert rig.mem.resets == []
    r = rig.event(LAP_M * 5, 7)  # doubtful again: the second running
    assert r is not None and r.missed_gate
    assert rig.mem.resets and rig.mem.resets[-1][1] in (
        "crossing_not_matched_twice",
        "length_out_of_band_twice",
    )
    assert rig.mem.station_set.n_laps == 0
    assert rig.mem.guidance().v_cap_mps == 0.5
    assert rig.mem.status()["resets"][-1]["lap"] == r.lap_index


def test_reset_after_two_laps_out_of_the_length_band() -> None:
    rig = Rig(reset_on_change=True, reset_length_share=0.08)
    rig.laps(2)
    # two laps 20 percent long, confirmed numbers: the length leaves the band twice running
    t = rig.results[-1].t_cross_s
    for k in (3, 4):
        t += 1.2 * LAP_M
        rig.event(t, k)
    reasons = [why for _, why in rig.mem.resets]
    assert "length_out_of_band_twice" in reasons or "crossing_not_matched_twice" in reasons
    assert rig.mem.station_set.n_laps == 0


def test_one_bad_lap_is_not_a_reset() -> None:
    rig = Rig(reset_on_change=True)
    rig.laps(3)
    rig.event(LAP_M * 4, 5)  # one doubtful lap
    rig.event(LAP_M * 5, 6)  # then a confirmed one
    assert rig.mem.resets == []


def test_reset_on_an_obstacle_brake_where_the_bound_was_above_the_lap1_cap() -> None:
    rig = Rig(reset_on_change=True)
    rig.laps(2)
    rig.drive_until(rig.results[-1].t_cross_s + 1.0)
    b = rig.mem.station_set.v_bound(rig.mem.station_m, rig.mem.lap_in_progress, 0.2)[0]
    assert b > 0.5
    rig.mem.on_safety_event(0, 2, "ttc")
    assert rig.mem.resets[-1][1] == "brake_where_bound_above_lap1_cap"
    assert rig.mem.guidance().v_cap_mps == 0.5
    # a timing event is never a reset trigger
    rig2 = Rig(reset_on_change=True)
    rig2.laps(2)
    rig2.drive_until(rig2.results[-1].t_cross_s + 1.0)
    rig2.mem.on_safety_event(0, 2, "watchdog")
    assert rig2.mem.resets == []


def test_no_brake_reset_without_the_switch_or_in_v0() -> None:
    for kw in ({"mode": "v1"}, {"mode": "v0", "reset_on_change": True}):
        rig = Rig(**kw)  # type: ignore[arg-type]
        rig.laps(2)
        rig.drive_until(rig.results[-1].t_cross_s + 1.0)
        rig.mem.on_safety_event(0, 2, "ttc")
        assert rig.mem.resets == []


def test_operator_reset_drops_the_laps_and_returns_to_the_lap1_cap() -> None:
    for mode in ("v0", "v1"):
        rig = Rig(mode=mode, reset_on_change=True)
        rig.laps(3)
        assert rig.mem.guidance().v_cap_mps > 0.5
        rig.mem.reset("operator")
        assert rig.mem.guidance().v_cap_mps == 0.5
        assert rig.mem.station_set.n_laps == 0
        assert rig.mem.resets[-1][1] == "operator"
        # learning starts again from there
        rig.laps(1)
        assert rig.mem.station_set.n_laps == 1


def test_station_uncertainty_in_the_engine_lowers_the_published_bound_only() -> None:
    a, b = (
        Rig(),
        Rig(station_uncertainty=True, station_uncertainty_rate=0.2, station_uncertainty_cap_m=1.0),
    )
    for rig in (a, b):
        rig.laps(2)
        rig.drive_until(rig.results[-1].t_cross_s + 2.0)
    ga, gb = a.mem.guidance(), b.mem.guidance()
    assert np.all(gb.v_ref_mps <= ga.v_ref_mps + 1e-12)
    assert np.array_equal(ga.cost_to_go_s, gb.cost_to_go_s)
    assert ga.v_cap_mps == gb.v_cap_mps
    assert not math.isnan(a.mem.station_m)


# -- K3s: the bound of the lap after a doubtful lap is held (development, owner decision 29) -------


def _drive_and_log(rig: Rig, t_end: float, log: dict) -> None:
    """Drive to `t_end` polling the guidance every 0.1 s; log the highest min(cap, bound) per
    0.10 m station bin per lap in progress, and the highest cap of a guidance with no arrays."""
    while (rig.k + 1) * DT <= t_end + 1e-9:
        rig.drive_until(min(t_end, rig.k * DT + 0.1))
        g = rig.mem.guidance()
        entry = log.setdefault(g.lap_index, {"cap": None, "bins": {}})
        if g.s_m.shape[0] == 0:
            entry["cap"] = g.v_cap_mps if entry["cap"] is None else max(entry["cap"], g.v_cap_mps)
            continue
        for s, v in zip(g.s_m, g.v_ref_mps, strict=True):
            b = math.floor(s / 0.1 + 1e-6)
            entry["bins"][b] = max(entry["bins"].get(b, -math.inf), min(g.v_cap_mps, v))


def _above(doubtful: dict, after: dict) -> list[int]:
    out = []
    for b, v in after["bins"].items():
        refs = [x for x in (doubtful["bins"].get(b), doubtful["cap"]) if x is not None]
        if not refs or v > max(refs) + 1e-9:
            out.append(b)
    return out


def _doubtful_run(**kw: object) -> tuple[dict, Rig]:
    rig = Rig(
        station_uncertainty=True,
        station_uncertainty_rate=0.2,
        station_uncertainty_cap_m=1.0,
        **kw,  # type: ignore[arg-type]
    )
    log: dict = {}
    period = LAP_M / rig.speed
    for k in (1, 2, 3):
        if k == 2:
            # a low plan clearance mid lap 2: a caution zone, active on laps 3 and 4, gone on 5
            _drive_and_log(rig, period * 1.5, log)
            rig.mem.on_planner_status(STATE_DRIVING, 0.01)
        _drive_and_log(rig, period * k + 0.05, log)
        rig.event(period * k, k)
    # the crossing of lap 4 is dropped: lap 4 is a double lap closed by event 5 (doubtful)
    _drive_and_log(rig, period * 5 + 0.05, log)
    r = rig.event(period * 5, 5)
    assert r is not None and r.missed_gate
    _drive_and_log(rig, period * 6 + 0.05, log)
    r = rig.event(period * 6, 6)
    assert r is not None and not r.missed_gate
    return log, rig


def test_bound_hold_is_off_by_default_and_changes_nothing() -> None:
    assert LapConfig().bound_hold_after_doubtful is False
    a, _ = _doubtful_run()
    b, _ = _doubtful_run(bound_hold_after_doubtful=False)
    assert a == b


def test_bound_hold_keeps_the_lap_after_a_doubtful_lap_at_or_below_it() -> None:
    off, _ = _doubtful_run()
    on, rig = _doubtful_run(bound_hold_after_doubtful=True)
    # without the hold the lap after publishes higher bounds where the zone expired after the
    # doubtful lap; with it, none
    assert _above(off[4], off[5])
    assert on[4]["bins"] and on[5]["bins"]
    assert _above(on[4], on[5]) == []
    # laps before the doubtful one are unchanged, and the hold ends with the lap after
    for lap in (1, 2, 3, 4):
        assert on[lap] == off[lap]
    assert rig.mem._bound_hold is None


def test_bound_hold_does_nothing_in_v0() -> None:
    a, _ = _doubtful_run(mode="v0")
    b, _ = _doubtful_run(mode="v0", bound_hold_after_doubtful=True)
    assert a == b


# -- K3s: a TTC warning makes a held caution zone (development, owner decision 29) -----------------

SEVERITY_INFO = 0


def _warned_rig(**kw: object) -> Rig:
    rig = Rig(**kw)  # type: ignore[arg-type]
    rig.laps(2)
    rig.drive_until(rig.results[-1].t_cross_s + 2.0)
    rig.mem.on_safety_event(0, SEVERITY_INFO, "ttc")  # advisory warning, no command change
    return rig


def test_ttc_warning_zone_is_off_by_default() -> None:
    assert LapConfig().ttc_warning_zone_laps == 0
    a, b = _warned_rig(), _warned_rig(ttc_warning_zone_laps=0)
    assert a.mem.station_set.zones == () and b.mem.station_set.zones == ()
    a.laps(1)
    b.laps(1)
    assert np.array_equal(a.mem.guidance().v_ref_mps, b.mem.guidance().v_ref_mps)
    with pytest.raises(ValueError):
        LapConfig(ttc_warning_zone_laps=-1)


def test_ttc_warning_makes_a_zone_held_for_the_laps_given() -> None:
    rig = _warned_rig(ttc_warning_zone_laps=5)
    (zone,) = rig.mem.station_set.zones
    assert zone.reason == "ttc_warning" and zone.hold_laps == 5
    centre = zone.centre_m
    ss = rig.mem.station_set
    lap = rig.mem.lap_in_progress
    # not on the lap it was made, then the speed driven there less 0.1 for 5 laps, then gone
    assert ss.v_bound(centre, lap, 0.2)[0] == pytest.approx(1.2)
    for k in range(1, 6):
        assert ss.v_bound(centre, lap + k, 0.2)[0] == pytest.approx(zone.speed_mps - 0.1)
    assert ss.v_bound(centre, lap + 6, 0.2)[0] == pytest.approx(1.2)
    # a brake is still not a zone for the ttc gate's warning path, and the default zones keep
    # their own hold
    rig.mem.on_planner_status(STATE_DRIVING, 0.01)
    assert rig.mem.station_set.zones[-1].hold_laps == rig.mem.cfg.caution_laps


def test_ttc_warning_zone_ignores_other_sources_and_v0() -> None:
    rig = Rig(ttc_warning_zone_laps=5)
    rig.laps(2)
    rig.drive_until(rig.results[-1].t_cross_s + 2.0)
    rig.mem.on_safety_event(0, SEVERITY_INFO, "watchdog")
    rig.mem.on_safety_event(1, SEVERITY_INFO, "ttc")  # a release
    assert rig.mem.station_set.zones == ()
    v0 = _warned_rig(mode="v0", ttc_warning_zone_laps=5)
    assert v0.mem.station_set.zones == ()
