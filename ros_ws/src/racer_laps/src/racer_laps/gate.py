"""The start-scan gate: when a lap closes and how far along the lap the car is, without a map.
Numpy only, no ROS.

There is no IMU on the car and a slam_toolbox map built over several laps smears, so lap closure
and lap position must not depend on a metric map after lap 1. The gate needs two things only:

* Station `s`: wheel distance (the integral of the forward wheel speed) since the last lap
  closure, metres. The planner defines its rollout stations the same way, so no pose is exchanged.
* The start scan: the LiDAR scan (points in base_link) taken when driving starts. It defines the
  start pose.

Arming. The gate is armed when both hold: the wheel distance since the last closure is at least
`min_lap_length_m` (lap 1) or `arm_fraction` times the reference lap length (later laps), and the
heading integrated from wheel odometry has turned at least 2 pi minus `heading_slack_rad` in
either direction since the last closure. The heading rate is `gain(|delta|) v tan(delta) / L`
with the gated steering `delta` (the yaw-rate gain table of vehicle_params when the caller has
it). The heading condition is what stops a symmetric track (a stadium looks the same half a lap
later) from closing at half a lap. Only the lower bound is used: odometry that over-reads the
turn must not disarm the gate.

Match. While armed every scan is matched to the start scan by point-to-point ICP, seeded with the
identity and, after a first success, with the previous result advanced by wheel odometry. A match
is accepted when RMS <= `rms_max_m`, inlier share >= `inlier_min_share`, |lateral offset| <=
`lateral_max_m` and |relative yaw| <= `yaw_max_rad`.

Crossing. An accepted match is the car's pose in the start frame. The lap closes when the
longitudinal offset goes from negative to non-negative between two accepted matches at most
`max_match_gap_s` apart. Crossing time: linear interpolation between the two scans' effective
times (header stamp plus half the sweep). One addition: the views a metre or more before the
start pose are often too unlike the start scan to pass the thresholds, so the first accepted
match of an approach can already be ahead of the line. If it is at most `late_match_max_m`
(0.4 m) ahead and no match was accepted in the previous `max_match_gap_s`, the crossing is placed
that far back by wheel distance (an error of centimetres of path) and the lap closes. Beyond
0.4 m nothing is invented. Lap time is crossing to crossing, lap length the wheel
distance between crossings. The first closure fixes the reference lap length. A crossing is
checked before it counts: at least `min_along_support` inlier matches of the later scan must
sit on surfaces whose normal has a large component along the start heading (corners, island
ends, the far wall of a bend). In a straight lane the along-lane offset is not observable, ICP
then keeps whatever x it was seeded with, and a gate without the check would "cross" wherever the
odometry says; the check refuses exactly that.

Missed gate. If the wheel distance since the last closure exceeds `missed_factor` times the
reference lap length without a crossing, the station is uncertain: the gate never invents a lap,
stays armed, and the next real crossing closes a lap that is flagged `missed` (the caller marks it
unclean and does not store it, its length is not a lap).

The gate is a measurement of lap position, not a safety function.
"""

from __future__ import annotations

import math
from collections import deque
from dataclasses import dataclass
from itertools import pairwise

import numpy as np

from racer_laps.scan_match import (
    DENSE_MAX_POINTS,
    DENSE_VOXEL_M,
    FloatArray,
    MatchResult,
    along_support,
    match_scans,
    overlap_share,
    surface_normals,
    thin,
    wrap,
)

TWO_PI = 2.0 * math.pi
#: A dropout of the wheel odometry longer than this is not integrated across.
MAX_ODOM_DT_S = 1.0
HISTORY_S = 4.0
#: How far a scan time may run ahead of the last odometry sample and still be extrapolated.
MAX_EXTRAPOLATION_S = 0.2


@dataclass(frozen=True)
class GateConfig:
    wheelbase_m: float  # vehicle_params chassis.wheelbase_m, from the generated binding
    min_lap_length_m: float = 3.0
    arm_fraction: float = 0.6
    heading_slack_rad: float = 1.2
    rms_max_m: float = 0.03
    inlier_min_share: float = 0.6
    inlier_dist_m: float = 0.05
    lateral_max_m: float = 0.75
    yaw_max_rad: float = math.radians(60.0)
    max_match_gap_s: float = 0.5
    missed_factor: float = 1.25
    #: (|delta| rad, gain) pairs from vehicle_params steering.yaw_rate_gain_table; empty = 1.0.
    yaw_gain_table: tuple[tuple[float, float], ...] = ()
    #: A crossing counts only if at least this many inlier matches pin down the along-lane offset
    #: (0 turns the check off). See `along_support` in scan_match.py.
    min_along_support: int = 8
    #: When the first accepted match of an approach (none in the previous `max_match_gap_s`) is
    #: already up to this far ahead of the line, close the lap with the crossing placed that
    #: wheel distance back. 0 turns it off: then only a negative-to-non-negative pair counts.
    late_match_max_m: float = 0.4
    #: Also seed the ICP with the dead-reckoned pose of the car in the start frame (see _seeds).
    dead_reckoning_seed: bool = True
    #: Around the dead-reckoned seed, rank a grid of along-lane and yaw offsets by overlap and
    #: refine the best `refine_top` with ICP (odometry over-reads or under-reads a lap by up to
    #: about a tenth: 0.7 m and 0.6 rad on the 2026-11-13 bags).
    seed_search: bool = True
    seed_grid_dx_m: tuple[float, ...] = (-0.8, -0.4, 0.0, 0.4, 0.8)
    seed_grid_dyaw_rad: tuple[float, ...] = (-0.35, 0.0, 0.35)
    refine_top: int = 2

    def __post_init__(self) -> None:
        if not (math.isfinite(self.wheelbase_m) and self.wheelbase_m > 0.0):
            raise ValueError(f"wheelbase_m must be positive and finite, got {self.wheelbase_m}")
        if self.min_lap_length_m <= 0.0 or not (0.0 < self.arm_fraction <= 1.0):
            raise ValueError("min_lap_length_m must be positive, arm_fraction in (0, 1]")
        if self.missed_factor <= 1.0:
            raise ValueError("missed_factor must exceed 1")


@dataclass(frozen=True)
class GateCrossing:
    lap_index: int  # 1 for the first closure
    t_cross_s: float
    lap_time_s: float
    length_m: float  # wheel distance between the two crossings
    missed: bool  # the previous crossing was missed: this is not a lap
    lateral_m: float
    yaw_rad: float
    heading_rad: float = math.nan  # heading integrated from odometry over the lap


@dataclass
class _Sample:
    t: float
    total_m: float
    heading: float  # integrated heading, unwrapped
    x: float  # dead-reckoned pose in the odometry frame
    y: float
    theta: float


def yaw_gain(table: tuple[tuple[float, float], ...], abs_delta: float) -> float:
    """Piecewise linear in |delta|, held at the first and last gain outside the knots (the same
    rule racer_state's wheel_twist_node applies)."""
    if not table:
        return 1.0
    xs = [k[0] for k in table]
    gs = [k[1] for k in table]
    return float(np.interp(abs_delta, xs, gs))


def _compose(
    a: tuple[float, float, float], b: tuple[float, float, float]
) -> tuple[float, float, float]:
    c, s = math.cos(a[2]), math.sin(a[2])
    return (a[0] + c * b[0] - s * b[1], a[1] + s * b[0] + c * b[1], wrap(a[2] + b[2]))


def _inverse(a: tuple[float, float, float]) -> tuple[float, float, float]:
    c, s = math.cos(a[2]), math.sin(a[2])
    return (-(c * a[0] + s * a[1]), -(-s * a[0] + c * a[1]), -a[2])


class StartScanGate:
    def __init__(self, cfg: GateConfig) -> None:
        self.cfg = cfg
        self._started = False
        self._hist: deque[_Sample] = deque()
        self._steer = 0.0
        self._speed = 0.0
        self._laps = 0
        self._l_ref = math.nan
        self._origin_m = 0.0
        self._heading_origin = 0.0
        self._t_lap_start = 0.0
        self._start_pts: FloatArray = np.zeros((0, 2))
        self._start_normals: FloatArray = np.zeros((0, 2))
        self._start_dense: FloatArray = np.zeros((0, 2))
        self._source_dense: FloatArray = np.zeros((0, 2))
        self._start_dr: tuple[float, float, float] = (0.0, 0.0, 0.0)
        self._prev: tuple[float, MatchResult] | None = None  # t_eff, accepted match
        self._last_good: tuple[float, MatchResult, tuple[float, float, float]] | None = None
        self._uncertain_lap = False  # a crossing was missed during this lap
        self.n_armed_scans = 0
        self.n_matched_scans = 0
        self.last_match: MatchResult | None = None
        self.last_arming_state = "idle"
        self._odom_t: float | None = None

    # -- state -----------------------------------------------------------------------------

    @property
    def started(self) -> bool:
        return self._started

    @property
    def laps_completed(self) -> int:
        return self._laps

    @property
    def l_ref_m(self) -> float:
        """Reference lap length (NaN before the first closure)."""
        return self._l_ref

    @property
    def total_m(self) -> float:
        return self._hist[-1].total_m if self._hist else 0.0

    @property
    def station_m(self) -> float:
        """Wheel distance since the last closure (since the start before the first)."""
        return self.total_m - self._origin_m

    @property
    def heading_since_closure_rad(self) -> float:
        return (self._hist[-1].heading if self._hist else 0.0) - self._heading_origin

    @property
    def station_uncertain(self) -> bool:
        """No crossing although more than `missed_factor` reference laps were driven."""
        return math.isfinite(self._l_ref) and (
            self.station_m > self.cfg.missed_factor * self._l_ref
        )

    @property
    def armed(self) -> bool:
        need = (
            self.cfg.min_lap_length_m
            if not math.isfinite(self._l_ref)
            else self.cfg.arm_fraction * self._l_ref
        )
        return (
            self._started
            and self.station_m >= need
            and abs(self.heading_since_closure_rad) >= TWO_PI - self.cfg.heading_slack_rad
        )

    # -- inputs ----------------------------------------------------------------------------

    def on_steering(self, steer_rad: float) -> None:
        """The gated steering angle (zero-order hold until the next one)."""
        if math.isfinite(steer_rad):
            self._steer = float(steer_rad)

    def on_odom(self, t_s: float, speed_mps: float) -> None:
        """One wheel odometry sample. Forward distance only: a negative speed (recovery) adds
        nothing to the station and nothing to the heading."""
        if not math.isfinite(speed_mps) or not math.isfinite(t_s):
            return
        v = max(0.0, float(speed_mps))
        last = self._hist[-1] if self._hist else None
        if last is None or self._odom_t is None:
            self._hist.append(_Sample(t_s, 0.0, 0.0, 0.0, 0.0, 0.0))
            self._odom_t = t_s
            self._speed = v
            return
        dt = t_s - last.t
        if dt <= 0.0:
            return
        dt = min(dt, MAX_ODOM_DT_S)
        # trapezoid on speed
        v_mid = 0.5 * (self._speed + v)
        delta = self._steer
        rate = (
            yaw_gain(self.cfg.yaw_gain_table, abs(delta))
            * v_mid
            * math.tan(delta)
            / (self.cfg.wheelbase_m)
        )
        dth = rate * dt
        step = v_mid * dt
        th_mid = last.theta + 0.5 * dth
        self._hist.append(
            _Sample(
                t_s,
                last.total_m + step,
                last.heading + dth,
                last.x + step * math.cos(th_mid),
                last.y + step * math.sin(th_mid),
                last.theta + dth,
            )
        )
        self._speed = v
        self._odom_t = t_s
        while len(self._hist) > 2 and self._hist[0].t < t_s - HISTORY_S:
            self._hist.popleft()

    def set_start(self, t_eff_s: float, points: FloatArray) -> bool:
        """Store the start scan (base_link points) and begin lap 1. False, and nothing started,
        if the scan has too few points to match against."""
        pts = thin(points)
        if pts.shape[0] < 12 or not self._hist:
            return False
        self._start_pts = pts
        self._start_normals = surface_normals(pts)
        self._start_dense = thin(points, DENSE_VOXEL_M, DENSE_MAX_POINTS)
        self._started = True
        self._laps = 0
        self._l_ref = math.nan
        self._origin_m = self._hist[-1].total_m
        self._heading_origin = self._hist[-1].heading
        self._t_lap_start = float(t_eff_s)
        self._prev = None
        self._last_good = None
        self._uncertain_lap = False
        last = self._hist[-1]
        self._start_dr = (last.x, last.y, last.theta)
        self.n_armed_scans = self.n_matched_scans = 0
        return True

    # -- the match -------------------------------------------------------------------------

    def _at(self, t_s: float) -> _Sample:
        """Odometry state at time t (linear in between, held or extrapolated at the ends)."""
        h = self._hist
        if not h:
            return _Sample(t_s, 0.0, 0.0, 0.0, 0.0, 0.0)
        if t_s >= h[-1].t:
            last = h[-1]
            dt = min(t_s - last.t, MAX_EXTRAPOLATION_S)
            step = self._speed * dt
            return _Sample(
                t_s,
                last.total_m + step,
                last.heading,
                last.x + step * math.cos(last.theta),
                last.y + step * math.sin(last.theta),
                last.theta,
            )
        if t_s <= h[0].t:
            return h[0]
        prev = h[0]
        for cur in h:
            if cur.t >= t_s:
                w = (t_s - prev.t) / (cur.t - prev.t) if cur.t > prev.t else 1.0
                return _Sample(
                    t_s,
                    prev.total_m + w * (cur.total_m - prev.total_m),
                    prev.heading + w * (cur.heading - prev.heading),
                    prev.x + w * (cur.x - prev.x),
                    prev.y + w * (cur.y - prev.y),
                    prev.theta + w * wrap(cur.theta - prev.theta),
                )
            prev = cur
        return h[-1]

    def _seeds(self, now: _Sample) -> list[tuple[float, float, float]]:
        """Where to start the ICP from: the previous accepted result advanced by wheel odometry
        while it is fresh, else the dead-reckoned pose of the car in the start frame (the start
        pose is re-anchored at every closure, so that estimate drifts over one lap only), and
        always the identity."""
        seeds: list[tuple[float, float, float]] = []
        if self._last_good is not None:
            t_prev, res, dr_prev = self._last_good
            if now.t - t_prev <= 1.0:
                rel = _compose(_inverse(dr_prev), (now.x, now.y, now.theta))
                seeds.append(_compose((res.x_m, res.y_m, res.yaw_rad), rel))
        if not seeds and self.cfg.dead_reckoning_seed:
            dr = _compose(_inverse(self._start_dr), (now.x, now.y, now.theta))
            seeds.append(dr)
        seeds.append((0.0, 0.0, 0.0))
        return seeds

    def _grid_around(
        self, source: FloatArray, seed: tuple[float, float, float]
    ) -> list[tuple[float, float, float]]:
        """The `refine_top` best of the seed's along-lane and yaw offset grid, by overlap."""
        scored = []
        for dx in self.cfg.seed_grid_dx_m:
            for dyaw in self.cfg.seed_grid_dyaw_rad:
                cand = (seed[0] + dx, seed[1], seed[2] + dyaw)
                scored.append(
                    (
                        overlap_share(source, self._start_pts, cand, self.cfg.inlier_dist_m * 1.6),
                        cand,
                    )
                )
        scored.sort(key=lambda sc: -sc[0])
        return [cand for _score, cand in scored[: max(1, self.cfg.refine_top)]]

    def _last_good_age(self, now: _Sample) -> float:
        return math.inf if self._last_good is None else now.t - self._last_good[0]

    def _accept(self, res: MatchResult) -> bool:
        c = self.cfg
        return (
            res.rms_m <= c.rms_max_m
            and res.inlier_share >= c.inlier_min_share
            and abs(res.y_m) <= c.lateral_max_m
            and abs(res.yaw_rad) <= c.yaw_max_rad
        )

    def _supported(self, source: FloatArray, res: MatchResult) -> bool:
        """The along-lane offset of `res` is observable (see `along_support`)."""
        if self.cfg.min_along_support <= 0:
            return True
        n = along_support(source, self._start_pts, self._start_normals, res, self.cfg.inlier_dist_m)
        return n >= self.cfg.min_along_support

    def _best_match(self, source: FloatArray, now: _Sample) -> MatchResult | None:
        """The ICP result of the seed that gives the highest inlier share among the accepted
        ones, else of the one with the highest share."""
        best: MatchResult | None = None
        best_ok = False
        seeds = self._seeds(now)
        if self.cfg.seed_search and self.cfg.dead_reckoning_seed and self._last_good_age(now) > 1.0:
            seeds = [*self._grid_around(source, seeds[0]), *seeds[1:]]
        for seed in seeds:
            res = match_scans(
                source,
                self._start_pts,
                seed,
                inlier_m=self.cfg.inlier_dist_m,
                score_source=self._source_dense,
                score_target=self._start_dense,
            )
            if res is None:
                continue
            ok = self._accept(res)
            if best is None or (ok, res.inlier_share) > (best_ok, best.inlier_share):
                best, best_ok = res, ok
        return best

    def on_scan(self, t_eff_s: float, points: FloatArray) -> GateCrossing | None:
        """One scan (base_link points, body filtered) at its effective time. Returns the lap it
        closed, if any. Costs one ICP while armed, nothing otherwise."""
        if not self._started:
            return None
        if not self.armed:
            self.last_arming_state = "waiting"
            self._prev = None
            return None
        self.last_arming_state = "armed"
        self.n_armed_scans += 1
        source = thin(points)
        self._source_dense = thin(points, DENSE_VOXEL_M, DENSE_MAX_POINTS)
        now = self._at(t_eff_s)
        res = self._best_match(source, now)
        self.last_match = res
        if res is None or not self._accept(res):
            return None
        self.n_matched_scans += 1
        self._last_good = (t_eff_s, res, (now.x, now.y, now.theta))
        crossing: GateCrossing | None = None
        prev = self._prev
        if prev is not None and 0.0 < t_eff_s - prev[0] <= self.cfg.max_match_gap_s:
            t0, m0 = prev
            if m0.x_m < 0.0 <= res.x_m and self._supported(source, res):
                crossing = self._close(t0, m0, t_eff_s, res)
        elif (
            self.cfg.late_match_max_m > 0.0
            and 0.0 <= res.x_m <= self.cfg.late_match_max_m
            and self._supported(source, res)
        ):
            # The first accepted match of the approach is already past the line (the views
            # before it are too unlike the start scan to match): place the crossing the wheel
            # distance x back instead of interpolating.
            t_cross = self._time_at_total(now.total_m - res.x_m)
            if t_cross is not None and t_cross > self._t_lap_start:
                crossing = self._finish(t_cross, res.y_m, res.yaw_rad)
        self._prev = None if crossing is not None else (t_eff_s, res)
        return crossing

    def _time_at_total(self, total_m: float) -> float | None:
        """Time at which the wheel distance was `total_m` (linear in the odometry history)."""
        h = list(self._hist)
        if not h or total_m < h[0].total_m:
            return None
        for a, b in pairwise(h):
            if a.total_m <= total_m <= b.total_m:
                span = b.total_m - a.total_m
                w = (total_m - a.total_m) / span if span > 0.0 else 0.0
                return a.t + w * (b.t - a.t)
        return h[-1].t if total_m >= h[-1].total_m else None

    def _close(self, t0: float, m0: MatchResult, t1: float, m1: MatchResult) -> GateCrossing:
        f = -m0.x_m / (m1.x_m - m0.x_m)
        return self._finish(
            t0 + f * (t1 - t0),
            m0.y_m + f * (m1.y_m - m0.y_m),
            m0.yaw_rad + f * wrap(m1.yaw_rad - m0.yaw_rad),
        )

    def _finish(self, t_cross: float, lateral_m: float, yaw_rad: float) -> GateCrossing:
        at = self._at(t_cross)
        length = at.total_m - self._origin_m
        missed = self._uncertain_lap or self.station_uncertain_at(at.total_m)
        self._laps += 1
        crossing = GateCrossing(
            lap_index=self._laps,
            t_cross_s=t_cross,
            lap_time_s=t_cross - self._t_lap_start,
            length_m=length,
            missed=missed,
            lateral_m=lateral_m,
            yaw_rad=yaw_rad,
            heading_rad=at.heading - self._heading_origin,
        )
        if not math.isfinite(self._l_ref) and not missed:
            self._l_ref = length
        # The start pose in the odometry frame, from this crossing: the car is at (0, lateral,
        # yaw) in the start frame, so the start frame is that pose's inverse from the car.
        self._start_dr = _compose(
            (at.x, at.y, at.theta), _inverse((0.0, crossing.lateral_m, crossing.yaw_rad))
        )
        self._origin_m = at.total_m
        self._heading_origin = at.heading
        self._t_lap_start = t_cross
        self._uncertain_lap = False
        return crossing

    def station_uncertain_at(self, total_m: float) -> bool:
        return math.isfinite(self._l_ref) and (
            total_m - self._origin_m > self.cfg.missed_factor * self._l_ref
        )

    def poll_uncertain(self) -> bool:
        """Latch the missed-gate condition (call from the node's tick) and return it."""
        if self.station_uncertain:
            self._uncertain_lap = True
        return self._uncertain_lap

    def status(self) -> dict[str, object]:
        return {
            "gate_armed": self.armed,
            "gate_state": self.last_arming_state,
            "station_m": self.station_m,
            "heading_since_closure_rad": self.heading_since_closure_rad,
            "l_ref_m": self._l_ref,
            "station_uncertain": self.poll_uncertain(),
            "armed_scans": self.n_armed_scans,
            "matched_scans": self.n_matched_scans,
        }
