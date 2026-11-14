"""What a fault-injected replay must and must not show, per consumer (roadmap P3.4).

Perception: `perception_violations` over the tracks `TrackPerceiver.process` returned for the
scans of a mutated recording. Planner: `planner_violations` over the CSV of planner_replay_cli
run on the same recording. Both return a list of plain sentences, empty when every rule holds.

The planner rules, with the bounds of the v0.2 design (the HARD timeouts, so they hold for the
planner with a step at 0.35 s and 0.10 s as well as for the one that makes staleness gradual):

  always       the command is finite, the speed is in [-reverse limit, v_cap] and never above the
               configured v_max, the steering within the vehicle limits, no forward command
               strictly between 0 and the speed floor outside a start ramp
  timeouts     a scan stream that gives no good scan for more than SCAN_HARD_S (drop, frozen,
               stamp jumps): from SCAN_HARD_S plus two ticks after the last good scan the speed is
               0 and the state NO_SCAN until a good scan comes back; the same for wheel odometry
               with ODOM_HARD_S and NO_ODOM (dropped or NaN samples: the planner node does not
               let a non finite sample refresh the staleness clock); within RETURN_S of the end
               of the fault the state is DRIVING again (when the unfaulted run was driving)
  short        a gap of at most the hard bound: the speed never exceeds what it was before the
               fault plus one ramp step, and only DRIVING, NO_SCAN and NO_ODOM appear
  nan scan     every beam NaN, inf or 0: no usable scan; BLOCKED or NO_SCAN with speed 0, never
               DRIVING at speed on an empty distance field
  track        a garbage /pnp/local_track: never NO_TRACK, track confidence 0 within one tick,
               still DRIVING where the unfaulted run was
  out of order no negative age (a stamp from the future) in the CSV

What the planner CANNOT see is not asserted: a frozen scan with fresh stamps (plausible ranges, a
fresh header) and, for wheel odometry, anything but silence, because the node stamps odometry with
its own arrival time (`on_odom`): a repeated or mis-stamped sample looks alive. Those cases get the
`always` rules and a recorded outcome, nothing more. They are caught, if at all, by layer 3 (the
safety node's stale and TTC checks) and layer 1 (the hardware mux).
"""

from __future__ import annotations

import math
from dataclasses import dataclass
from pathlib import Path
from typing import Any

import numpy as np
from numpy.typing import NDArray

from pnp_replay import metrics
from pnp_replay.vehicle import VehicleLimits

SCAN_HARD_S = 0.60
ODOM_HARD_S = 0.20
RETURN_S = 1.0
HARD_TIMEOUT_TICKS = 2
STATE_NO_SCAN = 3
STATE_NO_ODOM = 4
STATE_NO_TRACK = 2
STATE_DRIVING = metrics.STATE_DRIVING
STATE_BLOCKED = metrics.STATE_BLOCKED
SHORT_FAULT_STATES = {STATE_DRIVING, STATE_NO_SCAN, STATE_NO_ODOM}
EPS = 1e-6


# --- perception ------------------------------------------------------------------------------


def perception_violations(
    tracks: list[Any], scan_is_all_nan: list[bool] | None = None
) -> list[str]:
    """The rules for the tracks `TrackPerceiver.process` returned, one per scan: arrays of 16, a
    valid_count of 0 to 16 with the first valid_count entries finite, the self-check numbers
    (boundary residual, intrusion fraction) never infinite, and finite when the track has at least
    two valid stations. `scan_is_all_nan[i]` True means scan i had no usable beam: valid_count
    must be 0."""
    out: list[str] = []
    for i, t in enumerate(tracks):
        arrays = {
            "x": t.x_m,
            "y": t.y_m,
            "hl": t.half_width_left_m,
            "hr": t.half_width_right_m,
            "sigma": t.sigma_m,
        }
        for name, a in arrays.items():
            if len(a) != 16:
                out.append(f"scan {i}: {name} has {len(a)} entries, not 16")
        n = int(t.valid_count)
        if not 0 <= n <= 16:
            out.append(f"scan {i}: valid_count {n} is not in 0..16")
            continue
        for name, a in arrays.items():
            if len(a) == 16 and not np.isfinite(np.asarray(a[:n], dtype=np.float64)).all():
                out.append(f"scan {i}: {name} is not finite within valid_count {n}")
        for name, v in (("boundary_residual_m", t.boundary_residual_m),
                        ("intrusion_fraction", t.intrusion_fraction)):  # fmt: skip
            if math.isinf(v):
                out.append(f"scan {i}: {name} is infinite")
            elif n >= 2 and math.isnan(v):
                out.append(f"scan {i}: {name} is NaN on a track of {n} stations")
        if scan_is_all_nan is not None and scan_is_all_nan[i] and n != 0:
            out.append(f"scan {i}: every beam unusable but valid_count is {n}")
    return out


# --- planner ---------------------------------------------------------------------------------


@dataclass(frozen=True)
class PlannerLimits:
    """The bounds the always rules check against."""

    v_max_mps: float
    reverse_limit_mps: float
    floor_mps: float
    steering_min_rad: float
    steering_max_rad: float
    max_speed_up_step_mps: float
    tick_s: float

    @classmethod
    def from_config(
        cls, describe: dict[str, Any], floor_mps: float, vehicle: VehicleLimits
    ) -> PlannerLimits:
        """From `planner_replay_cli --describe`, the speed floor of the bundle and the vehicle
        limits: the planner's reverse limit is max(recover_speed_mps, floor), its speed-up step
        rate_limit_fraction of the acceleration limit per tick."""
        p = describe["parameters"]
        tick = float(describe["tick_s"])
        return cls(
            v_max_mps=float(p["v_max_mps"]),
            reverse_limit_mps=max(float(p["recover_speed_mps"]), floor_mps),
            floor_mps=floor_mps,
            steering_min_rad=vehicle.steering_min_rad,
            steering_max_rad=vehicle.steering_max_rad,
            max_speed_up_step_mps=float(p["rate_limit_fraction"])
            * vehicle.max_acceleration_mps2
            * tick,
            tick_s=tick,
        )


def speed_floor_from_bundle(bundle: Path, describe: dict[str, Any]) -> float:
    """The planner's speed floor: the bundle's speed dead band plus speed_floor_margin_mps."""
    import yaml

    manifest = yaml.safe_load((bundle / "manifest.yaml").read_text(encoding="utf-8"))
    band = max(0.0, float(manifest["io"]["nominal"]["speed_deadband_mps"]))
    return band + float(describe["parameters"]["speed_floor_margin_mps"])


def always_violations(rows: dict[str, NDArray[Any]], lim: PlannerLimits) -> list[str]:
    """The rules that hold under every fault (see the module docstring)."""
    out: list[str] = []
    now, v, steer = rows["now_s"], rows["cmd_speed_mps"], rows["cmd_steering_rad"]
    cap = rows["v_cap_mps"]
    bad = np.nonzero(~np.isfinite(v) | ~np.isfinite(steer))[0]
    if len(bad):
        out.append(f"non finite command at t={now[bad[0]]:.2f} s ({len(bad)} ticks)")
    finite = np.isfinite(v) & np.isfinite(steer)
    low = np.nonzero(finite & (v < -lim.reverse_limit_mps - EPS))[0]
    if len(low):
        out.append(
            f"speed {v[low[0]]:.3f} below -{lim.reverse_limit_mps:.3f} at t={now[low[0]]:.2f}"
        )
    over = np.nonzero(finite & (v > lim.v_max_mps + EPS))[0]
    if len(over):
        out.append(f"speed {v[over[0]]:.3f} above v_max {lim.v_max_mps} at t={now[over[0]]:.2f}")
    capped = np.nonzero(finite & np.isfinite(cap) & (v > cap + EPS))[0]
    if len(capped):
        k = capped[0]
        out.append(f"speed {v[k]:.3f} above the cap {cap[k]:.3f} at t={now[k]:.2f}")
    wild = np.nonzero(
        finite & ((steer < lim.steering_min_rad - EPS) | (steer > lim.steering_max_rad + EPS))
    )[0]
    if len(wild):
        out.append(f"steering {steer[wild[0]]:.3f} outside the limits at t={now[wild[0]]:.2f}")
    out.extend(_dead_zone_violations(now, v, lim))
    return out


def _dead_zone_violations(
    now: NDArray[np.float64], v: NDArray[np.float64], lim: PlannerLimits
) -> list[str]:
    """No forward command strictly between 0 and the floor except the ramp of a start from rest:
    a run that begins from a zero (or the start of the replay), rises on every tick by at most one
    speed-up step, and has fewer ticks than ceil(floor / step)."""
    out: list[str] = []
    step = lim.max_speed_up_step_mps
    longest = math.ceil(lim.floor_mps / step - 1e-9) - 1
    run = 0
    for i, x in enumerate(v):
        if not (0.0 < x < lim.floor_mps - EPS):
            run = 0
            continue
        before = float(v[i - 1]) if i > 0 else 0.0
        run += 1
        if run > longest:
            out.append(f"{run} ticks in the dead zone at t={now[i]:.2f} s (v {x:.3f})")
        elif run == 1 and before != 0.0:
            out.append(f"dead zone entered from {before:.3f}, not from rest, at t={now[i]:.2f} s")
        elif not before < x <= before + step + EPS:
            out.append(f"dead zone step {before:.3f} -> {x:.3f} at t={now[i]:.2f} s is not a ramp")
        if out:
            return out
    return out


@dataclass(frozen=True)
class Fault:
    """A fault as it was injected: what, on which stream, when (arrival times, bag seconds) and
    the arrival times of the last good message before it and the first good one after it."""

    name: str
    target: str  # "scan", "odom" or "track"
    t0: float
    t1: float
    last_good_t: float
    resume_t: float
    first_bad_t: float = math.nan  # arrival of the first affected message in the window
    fraction: float = 0.5
    fill: str = "nan"

    @property
    def first_t(self) -> float:
        return self.t0 if math.isnan(self.first_bad_t) else self.first_bad_t

    @property
    def gap_s(self) -> float:
        return self.resume_t - self.last_good_t

    @property
    def hard_s(self) -> float:
        return ODOM_HARD_S if self.target == "odom" else SCAN_HARD_S

    @property
    def detectable_silence(self) -> bool:
        """True when, for the planner, this fault is the absence of messages it can see: a
        dropped scan, a scan frozen or stamped from another time, dropped or NaN odometry."""
        if self.target == "scan":
            return self.name in ("drop", "frozen", "stamp_jump_forward", "stamp_jump_backward")
        if self.target == "odom":
            return self.name in ("drop", "nan_ranges")
        return False


def fault_for(
    rec_scan_t: NDArray[np.float64],
    rec_odom_t: NDArray[np.float64],
    rec_track_t: NDArray[np.float64],
    name: str,
    target: str,
    t0: float,
    t1: float,
    **params: Any,
) -> Fault:
    """The Fault record of an injection, from the arrival times of the UNMUTATED recording."""
    times = {"scan": rec_scan_t, "odom": rec_odom_t, "track": rec_track_t}.get(target)
    if times is None:
        return Fault(name, target, t0, t1, t0, t1, **params)
    before = times[times < t0]
    after = times[times > t1]
    inside = times[(times >= t0) & (times <= t1)]
    last_good = float(before[-1]) if len(before) else t0
    resume = float(after[0]) if len(after) else t1
    first = float(inside[0]) if len(inside) else t0
    return Fault(name, target, t0, t1, last_good, resume, first, **params)


def _ticks_in(now: NDArray[np.float64], a: float, b: float) -> NDArray[np.bool_]:
    return (now >= a) & (now < b)


def planner_violations(
    rows: dict[str, NDArray[Any]],
    fault: Fault,
    lim: PlannerLimits,
    baseline: dict[str, NDArray[Any]] | None = None,
    baseline_speed_slack: bool = False,
) -> list[str]:
    """Every rule that applies to this fault (module docstring). `baseline` is the CSV of the
    same replay without the fault; the return-to-DRIVING and track rules apply only where it was
    driving. With `baseline_speed_slack` the short-fault speed bound is
    max(speed before, the baseline's speed at that tick) plus one step, which is the same rule
    when the car is at a steady speed and does not blame a fault for an acceleration."""
    out = always_violations(rows, lim)
    now, state, v = rows["now_s"], rows["state"], rows["cmd_speed_mps"]
    tick = lim.tick_s
    base_driving_before = True
    if baseline is not None:
        bnow = baseline["now_s"]
        k = np.nonzero(bnow < fault.t0)[0]
        base_driving_before = bool(len(k)) and int(baseline["state"][k[-1]]) == STATE_DRIVING

    if fault.detectable_silence and fault.gap_s > fault.hard_s:
        want = STATE_NO_ODOM if fault.target == "odom" else STATE_NO_SCAN
        label = "NO_ODOM" if fault.target == "odom" else "NO_SCAN"
        start = fault.last_good_t + fault.hard_s + HARD_TIMEOUT_TICKS * tick
        sel = _ticks_in(now, start, fault.resume_t)
        late = np.nonzero(sel & (v != 0.0))[0]
        if len(late):
            k = late[0]
            out.append(
                f"speed {v[k]:.3f} at t={now[k]:.2f} s, {now[k] - fault.last_good_t:.2f} s after "
                f"the last good {fault.target} (hard bound {fault.hard_s} s plus two ticks)"
            )
        wrong = np.nonzero(sel & (state != want))[0]
        if len(wrong):
            k = wrong[0]
            out.append(f"state {rows['state_name'][k]} at t={now[k]:.2f} s, expected {label}")
        if base_driving_before:
            after = (now > fault.t1) & (now <= fault.t1 + RETURN_S)
            if not (state[after] == STATE_DRIVING).any():
                out.append(f"not DRIVING within {RETURN_S} s of the end of the fault")
    elif fault.detectable_silence:  # a short gap
        out.extend(_short_fault(rows, fault, lim, baseline, baseline_speed_slack))

    if fault.target == "scan" and fault.name == "nan_ranges" and fault.fraction >= 1.0:
        start = fault.first_t + 2 * tick
        sel = _ticks_in(now, start, fault.resume_t)
        wrong = np.nonzero(sel & ((v != 0.0) | ~np.isin(state, (STATE_NO_SCAN, STATE_BLOCKED))))[0]
        if len(wrong):
            k = wrong[0]
            out.append(
                f"no usable beam but {rows['state_name'][k]} at speed {v[k]:.3f} at "
                f"t={now[k]:.2f} s (expected BLOCKED or NO_SCAN with speed 0)"
            )
    if fault.target == "scan" and fault.name == "out_of_order":
        age = rows["scan_age_s"]
        neg = np.nonzero((age < 0.0) & (age != -1.0))[0]  # -1 is "no scan yet"
        if len(neg):
            out.append(f"negative scan age {age[neg[0]]:.3f} s at t={now[neg[0]]:.2f} s")
    if fault.target == "track":
        if (state == STATE_NO_TRACK).any():
            out.append("state NO_TRACK appeared (it is not produced since v0.1)")
        sel = _ticks_in(now, fault.first_t + tick, fault.resume_t)
        conf = rows["track_confidence"]
        bad = np.nonzero(sel & ~(conf == 0.0))[0]
        if len(bad):
            out.append(f"track confidence {conf[bad[0]]:.3f} at t={now[bad[0]]:.2f} s, expected 0")
        if baseline is not None and len(baseline["now_s"]) == len(now):
            was = baseline["state"] == STATE_DRIVING
            lost = np.nonzero(sel & was & (state != STATE_DRIVING))[0]
            if len(lost):
                out.append(
                    f"left DRIVING ({rows['state_name'][lost[0]]}) at t={now[lost[0]]:.2f} s "
                    "where the unfaulted run was driving"
                )
    return out


def _short_fault(
    rows: dict[str, NDArray[Any]],
    fault: Fault,
    lim: PlannerLimits,
    baseline: dict[str, NDArray[Any]] | None,
    slack: bool,
) -> list[str]:
    out: list[str] = []
    now, state, v = rows["now_s"], rows["state"], rows["cmd_speed_mps"]
    before = np.nonzero(now < fault.t0)[0]
    if not len(before):
        return out
    v_before = float(v[before[-1]])
    sel = _ticks_in(now, fault.t0, fault.resume_t + lim.tick_s)
    bound = np.full(len(now), v_before)
    if slack and baseline is not None and len(baseline["now_s"]) == len(now):
        bound = np.maximum(bound, baseline["cmd_speed_mps"])
    bound = bound + lim.max_speed_up_step_mps + EPS
    over = np.nonzero(sel & (v > bound))[0]
    if len(over):
        k = over[0]
        out.append(
            f"speed {v[k]:.3f} at t={now[k]:.2f} s exceeds {bound[k]:.3f} "
            f"(before the fault {v_before:.3f} plus one ramp step)"
        )
    # Planner v0.2 ramps the speed cap down while the scan is 0.35 to 0.60 s old; when the ramp
    # reaches the speed floor the plan cannot move: BLOCKED with reason 2 (BelowSpeedFloor) on a
    # scan older than the soft bound is that stop, not a fault of the planner.
    reason = rows.get("block_reason")
    ramp_stop = (
        np.zeros(len(now), dtype=bool)
        if reason is None
        else (state == STATE_BLOCKED) & (reason == 2) & (rows["scan_age_s"] > 0.35)
    )
    odd = np.nonzero(sel & ~np.isin(state, tuple(SHORT_FAULT_STATES)) & ~ramp_stop)[0]
    if len(odd):
        out.append(f"state {rows['state_name'][odd[0]]} at t={now[odd[0]]:.2f} s in a short fault")
    return out
