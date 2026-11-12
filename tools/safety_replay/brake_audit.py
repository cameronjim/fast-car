"""Audit of safety_node's TTC engagements on a race bag (docs/notes/safety-race-retune-2026-11-12.md).

PRE-REGISTERED METHOD. This section, the constants below and the classification rule were
committed BEFORE the audit was run on the 2026-11-12 bag, so the result cannot have chosen them.

What is classified. Every `/safety/events` ENGAGE record with source `ttc`: severity BRAKE (the
output speed was zeroed) and severity INFO (an advisory warning, "no command change"). Nothing
else (rate limit, watchdog) is a TTC engagement.

For each engagement, at the gate time t (the record's log time):
  * the SCENE is the last /scan that had arrived by t (the scan the gate judged), taken as static
    for the next second, in the rear-axle frame, minus the parts of the car the LiDAR sees in
    every scan;
  * REQUEST: the last /drive_raw by t (the follower's steering and speed), WHEEL SPEED: the last
    /odom/wheel by t, TRUE SPEED = wheel speed x the measured slip table (below);
  * BODY CLEARANCE: the arc length the rear axle travels along the requested steering before the
    body rectangle (wheelbase + front_overhang, rear_overhang, width / 2, no margin) touches a
    return (counterfactual.swept_distance_m), the geometric distance the gate is meant to measure;
  * CORNER if |requested steering| > CORNER_STEERING_RAD, else STRAIGHT.

THE COUNTERFACTUAL. "Would it have hit without the brake" is answered by running the UNBRAKED plan,
i.e. the follower's own /drive_raw steering and speed over the scan's time to one second after the
engagement, through the measured plant (counterfactual.simulate: wheel-speed transport delay,
first-order lag, acceleration limit, steering servo lag, the tyre slip tables) from the true state
at the scan, and taking the smallest distance from the body rectangle to any return over that
time. A BRAKE is

  NECESSARY   if that smallest clearance is below NECESSARY_CLEARANCE_M (0.04 m: the figure
              docs/notes/safety-floor-review-2026-11-02.md already uses as the least acceptable
              end clearance, about the model's own error, so a pass this thin is a contact);
  MARGINAL    if it is from 0.04 m up to MARGINAL_CLEARANCE_M (0.10 m): the unbraked car would
              have missed by a hand's breadth; kept, not counted as a false alarm;
  FALSE ALARM if it is at least 0.10 m.

A class is reported with a SENSITIVITY flag when it changes under either bracketing plant: the
"hold" plant (the car keeps its speed for the whole second, the worst case for the brake) and the
"agile" plant (the car obeys the request at once and has 15 percent less true speed, the best).
Advisories took no action; they are tabulated with what the car then did (the real smallest
clearance in the next second, from the real scans) and whether a brake followed within 1 s.

STOPPING. For each brake the wheel-speed trace around the gate time gives the latency (gate time to
the first 10 percent drop of the wheel speed below its pre-trip value) and the deceleration (the
steepest 0.1 s slope, and the mean from 90 to 20 percent of the pre-trip speed); the scan-matched
travel from the judged scan to the lowest speed within 1.2 s is compared with the stopping model
travel v * stop_latency_s + v^2 / (2 stop_deceleration_mps2) at the same wheel speed.

LIMITS, stated up front. One scan is the scene (a wall the scan misses is not in it); the
follower would have steered differently without the brake (its recorded request is the
counterfactual's only input); the slip and plant numbers are fitted on this bag; a hit means the
body rectangle reaches a return, which a foam wall tolerates better than a person would.
"""

from __future__ import annotations

import bisect
import math
from collections.abc import Sequence
from dataclasses import dataclass, field

import numpy as np
from follower_replay.bag import (
    PHASE_ENGAGE,
    PHASE_RELEASE,
    SEVERITY_BRAKE,
    SEVERITY_INFO,
    DriveRecord,
    EventRecord,
    OdomRecord,
    ScanRecord,
)

from safety_replay.counterfactual import BodyBox, Plant, Trajectory, simulate, swept_distance_m
from safety_replay.motion import Mount, car_motion, scan_points

CORNER_STEERING_RAD = 0.2
NECESSARY_CLEARANCE_M = 0.04
MARGINAL_CLEARANCE_M = 0.10
HORIZON_AFTER_S = 1.0
SWEPT_HORIZON_M = 2.0

# Steering bins of the slip tables (|angle| rad, upper edges).
STEERING_BINS = (0.1, 0.2, 0.3, 0.4, 0.5)


def arrival(record: object) -> float:
    log_time = float(getattr(record, "log_time_s", 0.0))
    return log_time if log_time > 0.0 else float(record.stamp_s)  # type: ignore[attr-defined]


@dataclass(frozen=True)
class Engagement:
    t_s: float  # arrival time of the ENGAGE record
    severity: int
    detail: str
    release_s: float | None  # arrival time of the matching release, None if none
    kind: str  # "brake" or "advisory"

    @property
    def duration_s(self) -> float | None:
        return None if self.release_s is None else self.release_s - self.t_s


def ttc_engagements(events: Sequence[EventRecord]) -> list[Engagement]:
    """Every `ttc` ENGAGE record with its release (the next release of the same severity).
    Severity BRAKE is a brake, INFO an advisory; the steering-hold and other INFO notes that
    share the source are advisories only when their detail is the TTC warning."""
    ordered = sorted(events, key=arrival)
    out: list[Engagement] = []
    for i, e in enumerate(ordered):
        if e.source != "ttc" or e.phase != PHASE_ENGAGE:
            continue
        if e.severity == SEVERITY_BRAKE:
            kind = "brake"
        elif e.severity == SEVERITY_INFO and e.detail.startswith("time-to-collision"):
            kind = "advisory"
        else:
            continue
        release = None
        for later in ordered[i + 1 :]:
            if (
                later.source == "ttc"
                and later.severity == e.severity
                and later.phase == PHASE_RELEASE
            ):
                release = arrival(later)
                break
        out.append(Engagement(arrival(e), e.severity, e.detail, release, kind))
    return out


# --- measured tables ----------------------------------------------------------------------------


@dataclass(frozen=True)
class SlipTable:
    """What scan matching says the car does, by |steering| bin: the true speed as a share of the
    wheel speed, and the true path curvature as a share of tan(delta) / wheelbase."""

    edges: tuple[float, ...]
    speed_ratio: tuple[float, ...]
    curvature_ratio: tuple[float, ...]
    counts: tuple[int, ...]

    def _bin(self, steering_rad: float) -> int:
        a = abs(steering_rad)
        for i, edge in enumerate(self.edges):
            if a < edge:
                return i
        return len(self.edges) - 1

    def speed_gain(self, steering_rad: float) -> float:
        return self.speed_ratio[self._bin(steering_rad)]

    def curvature_gain(self, steering_rad: float) -> float:
        return self.curvature_ratio[self._bin(steering_rad)]


def build_slip_table(
    pairs: np.ndarray,
    wheel_speed: np.ndarray,
    steering: np.ndarray,
    wheelbase_m: float,
    max_rms_m: float = 0.03,
    min_wheel_speed_mps: float = 0.2,
    min_count: int = 8,
) -> SlipTable:
    """From scan-matched pairs (columns: t_a, t_b, x, y, heading change, rms), the mean wheel speed
    over each pair and the steering command a servo-lag earlier: the median ratios per steering
    bin. Bins with fewer than min_count usable pairs take the nearest bin below (or above)."""
    dt = pairs[:, 1] - pairs[:, 0]
    dist = np.hypot(pairs[:, 2], pairs[:, 3])
    v_true = np.sign(pairs[:, 2] + 1e-12) * dist / dt
    wz = pairs[:, 4] / dt
    ok = (pairs[:, 5] < max_rms_m) & (wheel_speed > min_wheel_speed_mps) & np.isfinite(wheel_speed)
    ratios: list[float] = []
    kappas: list[float] = []
    counts: list[int] = []
    lo = 0.0
    for hi in STEERING_BINS:
        m = ok & (np.abs(steering) >= lo) & (np.abs(steering) < hi)
        counts.append(int(m.sum()))
        if m.sum() >= min_count:
            ratios.append(float(np.median(v_true[m] / wheel_speed[m])))
            kin = np.tan(steering[m]) / wheelbase_m
            usable = np.abs(kin) > 0.2
            if usable.sum() >= min_count:
                kap = wz[m][usable] / np.maximum(v_true[m][usable], 0.05)
                kappas.append(float(np.median(kap / kin[usable])))
            else:
                kappas.append(math.nan)
        else:
            ratios.append(math.nan)
            kappas.append(math.nan)
        lo = hi
    return SlipTable(
        STEERING_BINS, tuple(_fill(ratios, 1.0)), tuple(_fill(kappas, 1.0)), tuple(counts)
    )


def _fill(values: list[float], default: float) -> list[float]:
    """Replace NaN with the nearest earlier value, then the nearest later, then the default."""
    out = list(values)
    for i, v in enumerate(out):
        if math.isnan(v):
            earlier = [x for x in out[:i] if not math.isnan(x)]
            later = [x for x in out[i + 1 :] if not math.isnan(x)]
            out[i] = earlier[-1] if earlier else (later[0] if later else default)
    return out


def interp_series(t: np.ndarray, v: np.ndarray, at: float) -> float:
    return float(np.interp(at, t, v))


def fit_speed_plant(
    t: np.ndarray, command: np.ndarray, wheel: np.ndarray, dt_s: float = 0.02
) -> tuple[float, float, float, float, float]:
    """(delay_s, tau_s, max_accel, max_decel, command_gain) minimising the RMS error of a delayed,
    rate-limited first-order response of the wheel speed to gain x command, over a grid, the
    model re-anchored to the measurement every 3 s so one miss does not run away. Fit on the
    whole drive, not a clean step: the follower never drives a clean step."""
    grid = np.array(
        [
            (L, tau, aa, ad, g)
            for L in (0.0, 0.04, 0.08, 0.12, 0.16)
            for tau in (0.05, 0.1, 0.15, 0.2, 0.3, 0.45)
            for aa in (1.0, 1.5, 2.5, 4.0, 8.0)
            for ad in (2.5, 4.0, 6.0, 10.0, 16.0)
            for g in (0.8, 0.85, 0.9, 0.95, 1.0)
        ]
    )
    grid_t = np.arange(t[0], t[-1], dt_s)
    u = np.interp(grid_t, t, command)
    v = np.interp(grid_t, t, wheel)
    k = np.round(grid[:, 0] / dt_s).astype(int)
    cur = np.full(len(grid), v[0])
    err2 = np.zeros(len(grid))
    for i in range(1, len(grid_t)):
        uu = u[np.maximum(i - 1 - k, 0)] * grid[:, 4]
        a = np.clip((uu - cur) / grid[:, 1], -grid[:, 3], grid[:, 2])
        cur = cur + a * dt_s
        err2 += (cur - v[i]) ** 2
        if i % 150 == 0:
            cur = np.full(len(grid), v[i])
    best = int(np.argmin(err2))
    return tuple(float(x) for x in grid[best])  # type: ignore[return-value]


# --- per-engagement analysis --------------------------------------------------------------------


def classify(clearance_m: float) -> str:
    if clearance_m < NECESSARY_CLEARANCE_M:
        return "necessary"
    if clearance_m < MARGINAL_CLEARANCE_M:
        return "marginal"
    return "false alarm"


@dataclass
class Row:
    engagement: Engagement
    scan_stamp_s: float = math.nan
    request_steering_rad: float = math.nan
    request_speed_mps: float = math.nan
    wheel_speed_mps: float = math.nan
    true_speed_mps: float = math.nan
    body_clearance_m: float = math.inf
    gate_distance_m: float | None = None
    corner: bool = False
    # counterfactual (nominal, hold, agile)
    cf_min_clearance_m: float = math.nan
    cf_hit_s: float | None = None
    cf_hold_clearance_m: float = math.nan
    cf_agile_clearance_m: float = math.nan
    classification: str = ""
    sensitive: bool = False
    # what the car really did in the next second
    real_min_clearance_m: float = math.nan
    brake_followed_s: float | None = None
    stop: dict[str, float] = field(default_factory=dict)


def last_before(times: Sequence[float], t: float) -> int:
    return bisect.bisect_right(times, t) - 1


@dataclass
class Streams:
    scans: Sequence[ScanRecord]
    scan_times: Sequence[float]
    raw: Sequence[DriveRecord]
    raw_times: Sequence[float]
    drive: Sequence[DriveRecord]
    drive_times: Sequence[float]
    odom: Sequence[OdomRecord]
    odom_times: Sequence[float]

    @staticmethod
    def of(scans, raw, drive, odom) -> Streams:
        s = sorted(scans, key=arrival)
        r = sorted(raw, key=arrival)
        d = sorted(drive, key=arrival)
        o = sorted(odom, key=arrival)
        return Streams(
            s,
            [arrival(x) for x in s],
            r,
            [arrival(x) for x in r],
            d,
            [arrival(x) for x in d],
            o,
            [arrival(x) for x in o],
        )

    def wheel_speed_at(self, t: float) -> float:
        i = last_before(self.odom_times, t)
        return self.odom[i].speed_mps if i >= 0 else math.nan

    def request_at(self, t: float) -> DriveRecord | None:
        i = last_before(self.raw_times, t)
        return self.raw[i] if i >= 0 else None

    def drive_steering_at(self, t: float) -> float:
        i = last_before(self.drive_times, t)
        return self.drive[i].steering_rad if i >= 0 else 0.0


@dataclass(frozen=True)
class AuditModel:
    body: BodyBox
    mount: Mount
    plant_base: dict[str, float]  # delay_s, tau_s, max_accel, max_decel, cmd_gain, steer_tau_s
    slip: SlipTable
    wheelbase_m: float
    ignore: frozenset | None = None


def _plant(model: AuditModel, variant: str) -> Plant:
    """nominal: the fitted plant and the measured slip tables. hold: the car keeps its speed for
    the whole second (nothing slows it), the worst case for a brake's necessity. agile: the car
    obeys the request in half the delay and lag, with 15 percent less true speed, the best case."""
    b = model.plant_base
    slip = model.slip
    if variant == "hold":
        return Plant(
            model.wheelbase_m,
            b["delay_s"],
            b["tau_s"],
            b["steer_tau_s"],
            0.0,
            0.0,
            curvature_gain=slip.curvature_gain,
            speed_gain=slip.speed_gain,
        )
    agile = variant == "agile"
    scale = 0.85 if agile else 1.0
    return Plant(
        model.wheelbase_m,
        b["delay_s"] * (0.5 if agile else 1.0),
        b["tau_s"] * (0.5 if agile else 1.0),
        b["steer_tau_s"],
        b["max_decel"],
        b["max_accel"],
        curvature_gain=slip.curvature_gain,
        speed_gain=lambda d, s=slip, k=scale: s.speed_gain(d) * k,
    )


def counterfactual(
    row: Row, streams: Streams, model: AuditModel, variant: str
) -> Trajectory | None:
    """The unbraked plan through the scan the gate judged, from the scan's stamp to
    HORIZON_AFTER_S after the engagement."""
    t = row.engagement.t_s
    i = last_before(streams.scan_times, t)
    if i < 0:
        return None
    scan = streams.scans[i]
    ts = arrival(scan) - 0.0  # scan state is judged at its arrival; the car moved since its stamp
    t_state = scan.stamp_s if scan.stamp_s > 0.0 else ts
    pts = scan_points(scan, model.mount, ignore=model.ignore)
    plant = _plant(model, variant)
    delta0 = streams.drive_steering_at(t_state)
    wheel0 = streams.wheel_speed_at(t_state)
    v0 = wheel0 * plant.speed_gain(delta0)

    def speed_cmd(tau: float) -> float:
        rec = streams.request_at(t_state + tau)
        return (rec.speed_mps if rec else 0.0) * model.plant_base["cmd_gain"]

    def steer_cmd(tau: float) -> float:
        rec = streams.request_at(t_state + tau)
        return rec.steering_rad if rec else delta0

    horizon = (t - t_state) + HORIZON_AFTER_S
    return simulate(pts, speed_cmd, steer_cmd, v0, delta0, plant, model.body, horizon)


def analyse_engagement(engagement: Engagement, streams: Streams, model: AuditModel) -> Row:
    row = Row(engagement)
    t = engagement.t_s
    i = last_before(streams.scan_times, t)
    req = streams.request_at(t)
    if i < 0 or req is None:
        return row
    scan = streams.scans[i]
    row.scan_stamp_s = scan.stamp_s
    row.request_steering_rad = req.steering_rad
    row.request_speed_mps = req.speed_mps
    row.wheel_speed_mps = streams.wheel_speed_at(t)
    row.true_speed_mps = row.wheel_speed_mps * model.slip.speed_gain(req.steering_rad)
    row.corner = abs(req.steering_rad) > CORNER_STEERING_RAD
    pts = scan_points(scan, model.mount, ignore=model.ignore)
    row.body_clearance_m = swept_distance_m(
        pts, req.steering_rad, model.wheelbase_m, model.body, SWEPT_HORIZON_M
    )
    results = {}
    for variant in ("nominal", "hold", "agile"):
        traj = counterfactual(row, streams, model, variant)
        results[variant] = traj
    nominal = results["nominal"]
    if nominal is not None:
        row.cf_min_clearance_m = nominal.min_clearance_m
        row.cf_hit_s = nominal.first_hit_s
        row.cf_hold_clearance_m = results["hold"].min_clearance_m  # type: ignore[union-attr]
        row.cf_agile_clearance_m = results["agile"].min_clearance_m  # type: ignore[union-attr]
        row.classification = classify(row.cf_min_clearance_m)
        row.sensitive = (
            classify(row.cf_hold_clearance_m) != row.classification
            or classify(row.cf_agile_clearance_m) != row.classification
        )
    # What the car really did: the smallest body clearance in the scans of the next second.
    ends = []
    for j in range(i, len(streams.scans)):
        tj = streams.scan_times[j]
        if tj > t + HORIZON_AFTER_S:
            break
        if tj >= t:
            from safety_replay.motion import Body, box_clearance

            body = Body(model.body.front_x_m, model.body.rear_x_m, model.body.half_width_m)
            ends.append(
                box_clearance(scan_points(streams.scans[j], model.mount, ignore=model.ignore), body)
            )
    if ends:
        row.real_min_clearance_m = min(ends)
    return row


# --- stopping -----------------------------------------------------------------------------------


def stop_response(
    t: np.ndarray, v: np.ndarray, t_trip: float, window_s: float = 1.2
) -> dict[str, float]:
    """Latency and deceleration of the wheel speed after a trip at t_trip.

    v0 is the median wheel speed in the 0.1 s before the trip. The ONSET is the first time at or
    after t_trip the speed is 10 percent below v0 (latency = onset - t_trip). The window ends at the
    lowest speed within window_s of the trip (the car may be released and re-accelerate). The mean
    deceleration runs from the first 90 percent crossing to the first 20 percent crossing (NaN if
    the speed never gets down to 20 percent); the peak is the steepest 0.1 s slope.
    """
    pre = (t >= t_trip - 0.1) & (t <= t_trip)
    if not pre.any():
        return {}
    v0 = float(np.median(v[pre]))
    w = (t >= t_trip) & (t <= t_trip + window_s)
    tw, vw = t[w], v[w]
    if len(tw) < 3 or v0 < 0.05:
        return {"v0": v0}
    k_min = int(np.argmin(vw))
    out: dict[str, float] = {
        "v0": v0,
        "v_min": float(vw[k_min]),
        "t_min_s": float(tw[k_min] - t_trip),
    }
    below = np.where(vw <= 0.9 * v0)[0]
    out["latency_s"] = float(tw[below[0]] - t_trip) if len(below) else math.nan
    on = np.where(vw <= 0.8 * v0)[0]
    off = np.where(vw <= 0.2 * v0)[0]
    if len(on) and len(off) and off[0] > on[0]:
        out["mean_decel_mps2"] = float((vw[on[0]] - vw[off[0]]) / (tw[off[0]] - tw[on[0]]))
    else:
        out["mean_decel_mps2"] = math.nan
    span = 5  # samples, ~0.1 s at 50 Hz
    if len(tw) > span:
        slopes = (vw[:-span] - vw[span:]) / (tw[span:] - tw[:-span])
        out["peak_decel_mps2"] = float(slopes.max())
    else:
        out["peak_decel_mps2"] = math.nan
    # wheel-integrated travel from the trip to the lowest speed
    seg = tw <= tw[k_min]
    out["wheel_travel_m"] = float(np.trapezoid(vw[seg], tw[seg]))
    return out


def scan_travel_m(
    scans: Sequence[ScanRecord],
    times: Sequence[float],
    mount: Mount,
    t_from: float,
    t_to: float,
    ignore: frozenset | None,
) -> float:
    """Scan-matched displacement of the rear axle between the last scan at or before t_from and
    the last scan at or before t_to. NaN when matching fails."""
    a = last_before(times, t_from)
    b = last_before(times, t_to)
    if a < 0 or b <= a:
        return 0.0 if a >= 0 and b == a else math.nan
    try:
        x, y, _, _ = car_motion(scans[a], scans[b], mount, ignore=ignore)
    except ValueError:
        return math.nan
    return math.hypot(x, y)


def model_stop_travel_m(v_mps: float, latency_s: float, decel_mps2: float) -> float:
    """The stopping model of vehicle_params actuation: travel = v * latency + v^2 / (2 decel)."""
    return v_mps * latency_s + v_mps * v_mps / (2.0 * decel_mps2)
