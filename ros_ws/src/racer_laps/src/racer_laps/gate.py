"""The start-scan gate (v2): when a lap closes and how far along the lap the car is, without a
map. Numpy only, no ROS. Design: claude-docs/13-plug-and-play.md section 15.2.

There is no IMU on the car and a slam_toolbox map built over several laps smears, so lap closure
and lap position must not depend on a metric map after lap 1. The gate needs two things only:

* Station `s`: wheel distance (the integral of the forward wheel speed) since the last lap
  closure, metres. The planner defines its rollout stations the same way, so no pose is exchanged.
* The start scan: the LiDAR scan (points in base_link) taken when driving starts. It defines the
  start pose.

The gate is a small estimator of one number, the wheel distance at which the car crosses the
start line. Odometry proposes it, scans confirm it, and a closure needs both.

Arming. The gate is armed when both hold: the wheel distance since the last closure is at least
`min_lap_length_m` (no lap length estimate yet) or `arm_fraction` times the estimated lap length,
and the heading integrated from wheel odometry has turned at least 2 pi minus
`heading_slack_rad` in either direction since the last closure. The heading rate is
`gain(|delta|) v tan(delta) / L` with the gated steering `delta` (the yaw-rate gain table of
vehicle_params when the caller has it). The heading condition is what stops a symmetric track (a
stadium looks the same half a lap later) from closing at half a lap. Only the lower bound is used:
odometry that over-reads the turn must not disarm the gate.

Match. While tracking (about 60 percent of a lap round, so the matches chain on the approach) and
while armed, every scan is matched to the start scan by point-to-point ICP. Seeds: the previous
accepted result advanced by wheel odometry while it is fresh, else a grid of along-lane and yaw
offsets around the dead-reckoned pose of the car in the start frame (the start pose is re-anchored
at every closure), and the identity. The RMS and inlier share are scored on denser point sets than
the ICP iterates on. A match within 2.5 m of the start pose is then re-scored shifted along the
start heading: ICP slides along a straight lane (the walls match at any shift) and could be a
metre off; only the lane ends and bends decide the shift, and a clearly better one replaces it. A
match is accepted when RMS <= `rms_max_m`, inlier share >= `inlier_min_share`, |lateral offset| <=
`lateral_max_m` and |relative yaw| <= `yaw_max_rad`. Accepted matches keep the tracking seed
alive. They place a crossing only when STRONG.

STRONG. An accepted match whose along-lane offset is observable (`along_support`) and whose
free-space violation share is at most `strong_max_violation` (0.02): under the match each scan is moved
into the other's frame and a point that lies more than `freespace_margin_m` inside the other
scan's nearest range in its bearing bin is a violation (`scan_match.freespace_violation_share`;
the larger of the two directions counts). A straight lane matches itself at any shift and the
side walls alone pass every other test, but the lane ends of an alias sit in the other scan's
free space.

Candidate and consensus crossing. A STRONG match within `consensus_window_m` of the line, whose
heading since the last closure is a lap's, is one estimate of the line's wheel distance: the wheel
total at the scan minus the match's along offset. With a trusted lap length estimate the estimate
must also lie within the band of the last line position plus the estimated length (innovation
gate: an alias metres early is metres of innovation and is ignored); without one it must be at
least `min_lap_length_m` from the last line. The lap closes when the car is `consensus_past_m`
past the median estimate by wheel distance, or `consensus_wait_s` after the last candidate once
one was at or past the line, and only if two estimates agree within `consensus_spread_m` or one
is of late-match quality (inlier share >= `quality_min_share`, RMS <= `quality_rms_max_m`). The
crossing is at wheel total `median`, its time from the odometry history. After a closure no
candidate is accepted until the gate arms again.

Lap length estimate (`LapLengthEstimate`): the median of the last five closed, not-missed laps,
trusted once the last two agree. A lap that differs from a trusted estimate by more than the band
is flagged suspect (a LapEvent cannot be recalled; the caller drops the lap from what it learned).

Missed gate. If the wheel distance since the last closure exceeds the estimated length plus the
band (trusted) or `missed_factor` times it (not yet trusted) without a crossing, the station is
uncertain: the gate never invents a lap, stays armed, and the next real crossing closes a lap that
is flagged `missed` (the caller marks it unclean and does not store it, its length is not a lap).

The gate is a measurement of lap position, not a safety function.
"""

from __future__ import annotations

import math
from collections import deque
from dataclasses import dataclass, replace
from itertools import pairwise

import numpy as np

from racer_laps.scan_match import (
    DENSE_MAX_POINTS,
    DENSE_VOXEL_M,
    FloatArray,
    MatchResult,
    along_support,
    freespace_violation_share,
    match_scans,
    overlap_share,
    polar_profile,
    surface_normals,
    thin,
    wrap,
)

TWO_PI = 2.0 * math.pi
#: A dropout of the wheel odometry longer than this is not integrated across.
MAX_ODOM_DT_S = 1.0
#: Odometry history kept: a crossing is placed back in it by wheel distance (at most the 0.3 m
#: past the line plus the `consensus_wait_s` after the last candidate, a few seconds at the
#: slowest speed the planner drives).
HISTORY_S = 6.0
#: Lap length estimate (pre-registration of the design, 13-plug-and-play.md 15.2): the median of
#: the last LENGTH_MEDIAN_LAPS laps; the last two agree within max(LENGTH_AGREE_ABS_M,
#: LENGTH_AGREE_REL * value); the suspect band is max(BAND_ABS_M, BAND_REL * value).
LENGTH_MEDIAN_LAPS = 5
LENGTH_AGREE_ABS_M = 0.30
LENGTH_AGREE_REL = 0.04
BAND_ABS_M = 0.5
BAND_REL = 0.08
#: How far a scan time may run ahead of the last odometry sample and still be extrapolated.
MAX_EXTRAPOLATION_S = 0.2


@dataclass(frozen=True)
class GateConfig:
    wheelbase_m: float  # vehicle_params chassis.wheelbase_m, from the generated binding
    min_lap_length_m: float = 3.0
    arm_fraction: float = 0.6
    heading_slack_rad: float = 1.2
    #: Matching starts (without permission to close a lap) at this share of the arming distance
    #: and with this heading slack: about 60 percent of a lap.
    #: A closure needs the heading since the last one to be at least 2 pi minus this (tighter
    #: than `heading_slack_rad`, which only arms the gate).
    closure_heading_slack_rad: float = 0.9
    track_distance_fraction: float = 0.6
    track_slack_rad: float = 2.4
    rms_max_m: float = 0.03
    inlier_min_share: float = 0.6
    inlier_dist_m: float = 0.05
    lateral_max_m: float = 0.75
    yaw_max_rad: float = math.radians(60.0)
    missed_factor: float = 1.25
    #: (|delta| rad, gain) pairs from vehicle_params steering.yaw_rate_gain_table; empty = 1.0.
    yaw_gain_table: tuple[tuple[float, float], ...] = ()
    #: Multiplies the gain for a RIGHT turn (negative steering). 1.0 for the car, whose table is
    #: symmetric; the simulator's plant turns differently each way.
    yaw_gain_right_scale: float = 1.0
    #: A crossing counts only if at least this many inlier matches pin down the along-lane offset
    #: (0 turns the check off). See `along_support` in scan_match.py.
    min_along_support: int = 8
    #: A match within this distance of the start pose is re-scored shifted along the start
    #: heading in steps of `along_step_m`; a shift whose dense inlier share is better by
    #: `along_min_gain` replaces it (0 turns the search off). See `_refine_along`.
    along_search_m: float = 2.5
    along_step_m: float = 0.1
    along_min_gain: float = 0.03
    #: The LiDAR head's position in base_link (vehicle_params sensors.lidar.mount_x_m, mount_y_m;
    #: the node and the replay pass the binding's numbers, 0.0 is for synthetic tests).
    lidar_x_m: float = 0.0
    lidar_y_m: float = 0.0
    #: STRONG match: free-space violation share at most this. The design's starting value 0.04
    #: let an alias 2 m early through on development case T2 (5 m straight; a_T2_cw_12: violation
    #: 0.032 to 0.075 in the alias zone, 0.000 at the true line); 0.02 sits between.
    strong_max_violation: float = 0.02
    freespace_margin_m: float = 0.15
    freespace_max_range_m: float = 6.0
    #: Consensus crossing (design 15.2): only STRONG matches within `consensus_window_m` of the
    #: line are candidates; close when the car is `consensus_past_m` past the median line estimate
    #: or `consensus_wait_s` after the last candidate once one was at or past the line; two
    #: estimates must agree within `consensus_spread_m`, or one is of late-match quality.
    consensus_window_m: float = 0.6
    consensus_past_m: float = 0.3
    consensus_wait_s: float = 0.5
    consensus_spread_m: float = 0.15
    quality_min_share: float = 0.75
    quality_rms_max_m: float = 0.027
    #: A candidate not confirmed within this long is dropped.
    candidate_ttl_s: float = 1.5
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
        if not (0.0 <= self.strong_max_violation <= 1.0) or self.consensus_window_m <= 0.0:
            raise ValueError("strong_max_violation must be in [0, 1], consensus_window_m positive")
        if self.consensus_past_m < 0.0 or self.consensus_wait_s < 0.0:
            raise ValueError("consensus_past_m and consensus_wait_s must not be negative")


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
    n_candidates: int = 0  # STRONG matches the crossing was placed from
    spread_m: float = 0.0  # max minus min of their line estimates, wheel distance


@dataclass
class _Sample:
    t: float
    total_m: float
    heading: float  # integrated heading, unwrapped
    x: float  # dead-reckoned pose in the odometry frame
    y: float
    theta: float


@dataclass(frozen=True)
class _Candidate:
    """One STRONG match near the line: an estimate of the line's wheel distance."""

    t_eff_s: float
    line_m: float  # wheel total at the scan minus the match's along offset
    match: MatchResult


class LapLengthEstimate:
    """The lap length in wheel distance, revisable. `value_m` is the median of the last
    `LENGTH_MEDIAN_LAPS` closed, not-missed lap lengths; it is `trusted` once at least two laps
    were added and the last two agree within `max(LENGTH_AGREE_ABS_M, LENGTH_AGREE_REL * value)`.
    A first lap that closed early heals by the third lap: two laps that disagree are not trusted,
    the third makes the median the right length and the first is then suspect."""

    def __init__(self) -> None:
        self._laps: list[float] = []

    def add(self, length_m: float) -> None:
        """A closed, not-missed lap."""
        if math.isfinite(length_m) and length_m > 0.0:
            self._laps.append(float(length_m))

    def clear(self) -> None:
        self._laps = []

    @property
    def n_laps(self) -> int:
        return len(self._laps)

    @property
    def value_m(self) -> float:
        if not self._laps:
            return math.nan
        return float(np.median(self._laps[-LENGTH_MEDIAN_LAPS:]))

    @property
    def trusted(self) -> bool:
        if len(self._laps) < 2:
            return False
        tol = max(LENGTH_AGREE_ABS_M, LENGTH_AGREE_REL * self.value_m)
        return abs(self._laps[-1] - self._laps[-2]) <= tol

    def band_m(self) -> float:
        value = self.value_m
        if not math.isfinite(value):
            return BAND_ABS_M
        return max(BAND_ABS_M, BAND_REL * value)

    def suspect(self, length_m: float) -> bool:
        return self.trusted and abs(length_m - self.value_m) > self.band_m()


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
        self._est = LapLengthEstimate()
        self._lengths: dict[int, float] = {}
        self._flagged: set[int] = set()  # suspect when they closed
        self._suspect: list[int] = []
        self._origin_m = 0.0
        self._heading_origin = 0.0
        self._t_lap_start = 0.0
        self._start_pts: FloatArray = np.zeros((0, 2))
        self._start_normals: FloatArray = np.zeros((0, 2))
        self._start_dense: FloatArray = np.zeros((0, 2))
        self._start_profile: FloatArray = np.zeros(0)
        self._source_dense: FloatArray = np.zeros((0, 2))
        self._source_thin: FloatArray = np.zeros((0, 2))
        self._start_dr: tuple[float, float, float] = (0.0, 0.0, 0.0)
        self._cands: list[_Candidate] = []
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
        """Estimated lap length (NaN before the first closure)."""
        return self._est.value_m

    @property
    def l_ref_trusted(self) -> bool:
        return self._est.trusted

    @property
    def suspect_laps(self) -> list[int]:
        """Indices of closed laps whose length differs from the trusted estimate by more than the
        band (set later than the LapEvent, which cannot be recalled)."""
        return list(self._suspect)

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

    def _uncertain_after_m(self) -> float:
        value = self._est.value_m
        if not math.isfinite(value):
            return math.inf
        if self._est.trusted:
            return value + self._est.band_m()
        return self.cfg.missed_factor * value

    @property
    def station_uncertain(self) -> bool:
        """No crossing although the estimated lap length (plus its band when trusted, else
        `missed_factor` times it) was driven."""
        return self.station_m > self._uncertain_after_m()

    def _need_m(self) -> float:
        value = self._est.value_m
        if not math.isfinite(value):
            return self.cfg.min_lap_length_m
        return self.cfg.arm_fraction * value

    @property
    def tracking(self) -> bool:
        """Matching runs: far enough round the lap (distance and heading, with a wider heading
        slack than arming) that the start scan can be followed on the approach. A closure still
        needs `armed`."""
        if not self._started:
            return False
        return self.station_m >= self.cfg.track_distance_fraction * self._need_m() and (
            abs(self.heading_since_closure_rad) >= TWO_PI - self.cfg.track_slack_rad
        )

    @property
    def armed(self) -> bool:
        return (
            self._started
            and self.station_m >= self._need_m()
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
        gain = yaw_gain(self.cfg.yaw_gain_table, abs(delta))
        if delta < 0.0:
            gain *= self.cfg.yaw_gain_right_scale
        rate = gain * v_mid * math.tan(delta) / self.cfg.wheelbase_m
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
        self._start_profile = polar_profile(points, (self.cfg.lidar_x_m, self.cfg.lidar_y_m))
        self._started = True
        self._laps = 0
        self._est = LapLengthEstimate()
        self._lengths = {}
        self._flagged = set()
        self._suspect = []
        self._origin_m = self._hist[-1].total_m
        self._heading_origin = self._hist[-1].heading
        self._t_lap_start = float(t_eff_s)
        self._cands = []
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
        if (
            best is not None
            and self.cfg.along_search_m > 0.0
            and abs(best.x_m) <= self.cfg.along_search_m
            and self._accept(best)
        ):
            best = self._refine_along(best)
        return best

    def _refine_along(self, res: MatchResult) -> MatchResult:
        """Point-to-point ICP slides along a straight lane: the walls match at any shift and the
        solution stays near its seed, so a lane end can be missed by a metre. Score the same
        match shifted along the start heading (the dense inlier share, which only the lane ends
        and bends depend on) and, if another shift is clearly better, polish from there."""
        c = self.cfg
        n = round(2.0 * c.along_search_m / c.along_step_m) + 1
        shifts = -c.along_search_m + c.along_step_m * np.arange(n)
        cos_y, sin_y = math.cos(res.yaw_rad), math.sin(res.yaw_rad)
        moved = self._source_dense @ np.array([[cos_y, -sin_y], [sin_y, cos_y]]).T
        target = self._start_dense
        t2 = (target * target).sum(axis=1)[None, :]
        shares = np.empty(n)
        for k, dx in enumerate(shifts):
            p = moved + np.array([res.x_m + dx, res.y_m])
            d2 = (p * p).sum(axis=1)[:, None] + t2 - 2.0 * (p @ target.T)
            shares[k] = float(np.mean(d2.min(axis=1) < c.inlier_dist_m**2))
        k0 = int(np.argmin(np.abs(shifts)))
        k_best = int(np.argmax(shares))
        if shares[k_best] < shares[k0] + c.along_min_gain or k_best == k0:
            return res
        again = match_scans(
            self._source_thin,
            self._start_pts,
            (res.x_m + float(shifts[k_best]), res.y_m, res.yaw_rad),
            inlier_m=c.inlier_dist_m,
            score_source=self._source_dense,
            score_target=self._start_dense,
        )
        if again is not None and again.inlier_share > res.inlier_share:
            return again
        return res

    def _freespace(self, res: MatchResult, points: FloatArray) -> float:
        """Larger free-space violation share of the match, both ways (see scan_match)."""
        c = self.cfg
        origin = (c.lidar_x_m, c.lidar_y_m)
        cos_y, sin_y = math.cos(res.yaw_rad), math.sin(res.yaw_rad)
        rot = np.array([[cos_y, -sin_y], [sin_y, cos_y]])
        trans = np.array([res.x_m, res.y_m])
        cur_in_start = self._source_dense @ rot.T + trans
        a = freespace_violation_share(
            cur_in_start, self._start_profile, origin, c.freespace_margin_m, c.freespace_max_range_m
        )
        start_in_cur = (self._start_dense - trans) @ rot
        b = freespace_violation_share(
            start_in_cur,
            polar_profile(points, origin),
            origin,
            c.freespace_margin_m,
            c.freespace_max_range_m,
        )
        return max(a, b)

    def on_scan(self, t_eff_s: float, points: FloatArray) -> GateCrossing | None:
        """One scan (base_link points, body filtered) at its effective time. Returns the lap it
        closed, if any. Costs one ICP while armed, nothing otherwise."""
        if not self._started:
            return None
        if not self.tracking:
            self.last_arming_state = "waiting"
            self._cands = []
            return None
        armed = self.armed
        self.last_arming_state = "armed" if armed else "tracking"
        if armed:
            self.n_armed_scans += 1
        source = thin(points)
        self._source_thin = source
        self._source_dense = thin(points, DENSE_VOXEL_M, DENSE_MAX_POINTS)
        now = self._at(t_eff_s)
        res = self._best_match(source, now)
        if res is not None and self._accept(res):
            if armed:
                self.n_matched_scans += 1
                res = replace(res, freespace_violation=self._freespace(res, points))
            self._last_good = (t_eff_s, res, (now.x, now.y, now.theta))
            if armed:
                self._consider(t_eff_s, res, source, now)
        self.last_match = res
        if not armed:
            return None
        return self._try_close(t_eff_s, now)

    def _consider(self, t_eff_s: float, res: MatchResult, source: FloatArray, now: _Sample) -> None:
        """Keep a STRONG match near the line as one estimate of the line's wheel distance."""
        c = self.cfg
        if abs(res.x_m) > c.consensus_window_m or res.freespace_violation > c.strong_max_violation:
            return
        if abs(now.heading - self._heading_origin) < TWO_PI - c.closure_heading_slack_rad:
            return
        line = now.total_m - res.x_m
        if line - self._origin_m < c.min_lap_length_m:
            return
        # innovation gate: with a trusted length an alias metres early is metres of innovation
        if self._est.trusted and abs(line - (self._origin_m + self._est.value_m)) > (
            self._est.band_m()
        ):
            return
        if not self._supported(source, res):
            return
        self._cands.append(_Candidate(t_eff_s, line, res))

    def _try_close(self, t_eff_s: float, now: _Sample) -> GateCrossing | None:
        c = self.cfg
        self._cands = [k for k in self._cands if t_eff_s - k.t_eff_s <= c.candidate_ttl_s]
        if not self._cands:
            return None
        lines = sorted(k.line_m for k in self._cands)
        median = float(np.median(lines))
        agree = any(b - a_ <= c.consensus_spread_m for a_, b in pairwise(lines))
        quality = any(
            k.match.inlier_share >= c.quality_min_share and k.match.rms_m <= c.quality_rms_max_m
            for k in self._cands
        )
        if not (agree or quality):
            return None
        past = now.total_m >= median + c.consensus_past_m
        wait = t_eff_s - self._cands[-1].t_eff_s >= c.consensus_wait_s and any(
            k.match.x_m >= 0.0 for k in self._cands
        )
        if not (past or wait):
            return None
        t_cross = self._time_at_total(median)
        if t_cross is None:
            return None
        t_cross = min(t_cross, t_eff_s)
        if t_cross <= self._t_lap_start:
            return None
        nearest = min(self._cands, key=lambda k: abs(k.match.x_m))
        m = nearest.match
        lateral = m.y_m - m.x_m * math.tan(m.yaw_rad)
        return self._finish(t_cross, lateral, m.yaw_rad, len(self._cands), lines[-1] - lines[0])

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

    def _finish(
        self,
        t_cross: float,
        lateral_m: float,
        yaw_rad: float,
        n_candidates: int = 0,
        spread_m: float = 0.0,
    ) -> GateCrossing:
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
            n_candidates=n_candidates,
            spread_m=spread_m,
        )
        if not missed:
            # judged against the estimate as it stood: adding a deviant lap would itself make the
            # last two disagree and the estimate untrusted
            if self._est.suspect(length):
                self._flagged.add(self._laps)
            self._est.add(length)
            self._lengths[self._laps] = length
        # and every earlier lap against the estimate as it is now (a poisoned first lap is
        # flagged once the later laps make the estimate trusted)
        self._suspect = sorted(
            self._flagged | {i for i, n in self._lengths.items() if self._est.suspect(n)}
        )
        # The start pose in the odometry frame, from this crossing: the car is at (0, lateral,
        # yaw) in the start frame, so the start frame is that pose's inverse from the car.
        self._start_dr = _compose(
            (at.x, at.y, at.theta), _inverse((0.0, crossing.lateral_m, crossing.yaw_rad))
        )
        self._origin_m = at.total_m
        self._heading_origin = at.heading
        self._t_lap_start = t_cross
        self._uncertain_lap = False
        self._cands = []
        return crossing

    def station_uncertain_at(self, total_m: float) -> bool:
        return total_m - self._origin_m > self._uncertain_after_m()

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
            "l_ref_m": self._est.value_m,
            "l_ref_trusted": self._est.trusted,
            "suspect_laps": list(self._suspect),
            "station_uncertain": self.poll_uncertain(),
            "armed_scans": self.n_armed_scans,
            "matched_scans": self.n_matched_scans,
        }
