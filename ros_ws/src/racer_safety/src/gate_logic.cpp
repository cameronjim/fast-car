#include "racer_safety/gate_logic.hpp"

#include <array>
#include <cmath>
#include <cstddef>
#include <limits>
#include <optional>
#include <string>
#include <utility>

#include "racer_safety/gate_logic_formatting.hpp"

namespace racer_safety {

namespace {

// Plain, explicit-branch helpers (deliberately NOT std::clamp/std::min) so every decision
// this file makes is a literal `if` in this translation unit -- the 100% branch-coverage
// gate (claude-docs/12-testing.md) needs to be satisfiable predictably regardless of
// optimization level or how a standard-library template happens to get inlined.

double clamp_value(double value, double lo, double hi) {
  if (value < lo) {
    return lo;
  }
  if (value > hi) {
    return hi;
  }
  return value;
}

bool is_finite_command(const DriveCommand& cmd) {
  if (!std::isfinite(cmd.steering_angle_rad)) {
    return false;
  }
  if (!std::isfinite(cmd.speed_mps)) {
    return false;
  }
  return true;
}

// Garbage/negative timing is the least-permissive (safest) input: no time is treated as
// having elapsed, so no rate-limited change is allowed this cycle, rather than guessing a
// nominal period or letting NaN/Inf propagate into a rate-limit bound.
double safe_dt(double dt_s) {
  if (!std::isfinite(dt_s)) {
    return 0.0;
  }
  if (dt_s < 0.0) {
    return 0.0;
  }
  return dt_s;
}

// How long an engagement lasted, defensively: a non-finite or backwards interval (a clock
// that did something impossible) is reported as 0.0 rather than propagating garbage into an
// evaluation metric (claude-docs/09-evaluation.md counts interventions off this stream).
double engagement_duration_s(double engaged_at_s, double now_s) {
  const double duration_s = now_s - engaged_at_s;
  if (!std::isfinite(duration_s)) {
    return 0.0;
  }
  if (duration_s < 0.0) {
    return 0.0;
  }
  return duration_s;
}

// A /scan return only counts for TTC if it is a finite, positive distance. Garbage
// (NaN/Inf/non-positive) is treated as "no valid range this cycle" -- ignored, not
// hallucinated into an obstacle -- rather than forcing a brake off of nonsense sensor data.
bool is_valid_range(double range_m) {
  if (!std::isfinite(range_m)) {
    return false;
  }
  if (range_m <= 0.0) {
    return false;
  }
  return true;
}

// Below this a request is "not moving forward" for TTC purposes (unchanged since the TTC
// gate was written; a numerical zero guard, not a physical constant).
constexpr double kMinForwardSpeedMps = 1e-6;

// Garbage range: NaN, zero, negative or -inf. +inf is NOT garbage (it is "nothing in the
// path"), and neither is a finite positive distance. Written as !(r > 0) so NaN
// lands here without its own branch.
bool is_garbage_range(double range_m) { return !(range_m > 0.0); }

// The steering-hold timer for a cycle on which the latch holds the output speed at zero
// (gate_logic.hpp, "STEERING HOLD WHILE PARKED ON THE OBSTACLE LATCH"): 0.0 on the first such
// cycle, previous + dt after that. A garbage previous value (non-finite or negative, which no
// path in this file produces) restarts the timer: steering stays live, today's behaviour,
// rather than holding on nonsense. `dt_s` has already been through safe_dt.
double advance_hold_timer_s(const std::optional<double>& previous_s, double dt_s) {
  if (!previous_s.has_value()) {
    return 0.0;
  }
  if (!std::isfinite(*previous_s)) {
    return 0.0;
  }
  if (*previous_s < 0.0) {
    return 0.0;
  }
  return *previous_s + dt_s;
}

}  // namespace

// The output of a gate that forces zero speed: zero speed, and the steering angle this logic
// last COMMANDED, held. See gate_logic.hpp's "STEERING ON A ZERO-THROTTLE GATE" for the
// reasoning (centring is an unrate-limited step input and straightens a cornering car). The
// held angle is defended against a non-finite previous output -- which no path in this file
// can produce, since every output is clamped -- because centring is still better than
// emitting a NaN steering angle onto /drive.
DriveCommand zero_throttle_command(const DriveCommand& previous_output) {
  if (!std::isfinite(previous_output.steering_angle_rad)) {
    return DriveCommand{0.0, 0.0};
  }
  return DriveCommand{previous_output.steering_angle_rad, 0.0};
}

CovarianceGateResult evaluate_covariance_gate(bool has_pose_input, double pose_covariance_trace) {
  // TODO(roadmap task 2.6): once /pose (PoseWithCovarianceStamped) exists, derate
  // `speed_fraction` from `pose_covariance_trace` against a tuned threshold
  // (claude-docs/04-architecture.md: "Pose covariance above gate threshold -> safety_node
  // ramps speed cap down"). Until then this is a deliberate no-op stub: `has_pose_input` is
  // always false in the real graph (no /pose publisher exists), and this function must never
  // be the thing that decides whether the OTHER gates in SafetyGateLogic::evaluate() run --
  // see that function's covariance-gate step, which applies this result unconditionally
  // (multiplying by speed_fraction, a no-op at 1.0) rather than branching on it.
  (void)pose_covariance_trace;  // unused until roadmap task 2.6 wires a real derate
  if (!has_pose_input) {
    return CovarianceGateResult{/*speed_fraction=*/1.0, /*engaged=*/false};
  }
  // Reachable only by a caller that explicitly sets has_pose_input=true (no production code
  // path does this yet -- test_gate_logic.cpp exercises it directly to prove the stub, even
  // if "engaged", still fails safe by not inventing a derate it has no data to justify).
  return CovarianceGateResult{/*speed_fraction=*/1.0, /*engaged=*/true};
}

DriveCommand SafetyGateLogic::clamp_to_bounds(const DriveCommand& cmd) const {
  DriveCommand out = cmd;
  out.steering_angle_rad =
      clamp_value(out.steering_angle_rad, limits_.steering_min_rad, limits_.steering_max_rad);
  out.speed_mps = clamp_value(out.speed_mps, limits_.speed_min_mps, limits_.speed_max_mps);
  return out;
}

DriveCommand SafetyGateLogic::rate_limit(const DriveCommand& cmd,
                                         const DriveCommand& previous_output, double dt_s) const {
  DriveCommand out = cmd;

  const double steer_delta = cmd.steering_angle_rad - previous_output.steering_angle_rad;
  const double steer_delta_min = limits_.steering_rate_min_rad_per_s * dt_s;
  const double steer_delta_max = limits_.steering_rate_max_rad_per_s * dt_s;
  const double clamped_steer_delta = clamp_value(steer_delta, steer_delta_min, steer_delta_max);
  out.steering_angle_rad = previous_output.steering_angle_rad + clamped_steer_delta;

  const double speed_delta = cmd.speed_mps - previous_output.speed_mps;
  if (speed_delta > 0.0) {
    // Only accelerating (increasing commanded speed) is rate-limited; decelerating/braking is
    // never rate-limited (claude-docs/05-safety.md never asks for slower braking).
    const double max_speed_delta = limits_.max_acceleration_mps2 * dt_s;
    if (speed_delta > max_speed_delta) {
      out.speed_mps = previous_output.speed_mps + max_speed_delta;
    } else {
      out.speed_mps = previous_output.speed_mps + speed_delta;
    }
  }
  return out;
}

bool SafetyGateLogic::steering_hold_engaged(const std::optional<double>& hold_timer_s) const {
  if (!hold_timer_s.has_value()) {
    return false;
  }
  // Written as a literal `if` (not `return a >= b`) for the branch-coverage gate. A NaN
  // configured hold time never engages.
  if (*hold_timer_s >= limits_.obstacle_steering_hold_after_s) {
    return true;
  }
  return false;
}

GateActivation SafetyGateLogic::steering_hold_activation(double held_steering_angle_rad) const {
  return GateActivation{GateSource::kTtc, EventSeverity::kInfo,
                        formatting::steering_hold_detail(held_steering_angle_rad,
                                                         limits_.obstacle_steering_hold_after_s)};
}

void SafetyGateLogic::hold_obstacle_state(const GateInput& input,
                                          const DriveCommand& previous_output,
                                          GateResult& result) const {
  // result.ttc_brake_latched already carries the input's latch. Without a latch the hold timer
  // stays cleared (GateResult's default) and the hold does not apply.
  if (input.ttc_brake_latched) {
    result.activations.push_back(GateActivation{GateSource::kTtc, EventSeverity::kBrake,
                                                formatting::ttc_latch_held_detail()});
    result.obstacle_hold_timer_s = input.obstacle_hold_timer_s;
    if (steering_hold_engaged(input.obstacle_hold_timer_s)) {
      // The short-circuit output already holds the previous steering (zero_throttle_command),
      // which is the held angle; this only keeps the engagement from being split.
      result.activations.push_back(
          steering_hold_activation(zero_throttle_command(previous_output).steering_angle_rad));
    }
  }
}

void SafetyGateLogic::apply_steering_hold(const GateInput& input,
                                          const DriveCommand& previous_output, double dt_s,
                                          DriveCommand& target, DriveCommand& output,
                                          GateResult& result) const {
  std::optional<double> timer_s;  // nullopt: the latch is not holding the output at zero
  if (result.ttc_brake_latched) {
    if (output.speed_mps == 0.0) {
      timer_s = advance_hold_timer_s(input.obstacle_hold_timer_s, dt_s);
    }
  }
  result.obstacle_hold_timer_s = timer_s;

  if (steering_hold_engaged(timer_s)) {
    // Frozen at the angle the output had when the hold started: the previous output, which
    // on every later held cycle is that same angle again. Bounds-clamped and NaN-defended
    // (zero_throttle_command) although no path here can produce either case.
    const double held_rad = clamp_value(zero_throttle_command(previous_output).steering_angle_rad,
                                        limits_.steering_min_rad, limits_.steering_max_rad);
    // `target` too, so the rate-limit detection below does not count the hold as a clamp.
    target.steering_angle_rad = held_rad;
    output.steering_angle_rad = held_rad;
    result.activations.push_back(steering_hold_activation(held_rad));
  } else if (steering_hold_engaged(input.obstacle_hold_timer_s)) {
    // The hold ended on this cycle (latch released, or a reverse request moved the car): the
    // note that becomes the PHASE_RELEASE record's detail.
    result.releases.push_back(GateActivation{GateSource::kTtc, EventSeverity::kInfo,
                                             formatting::steering_hold_release_detail()});
  }
}

double SafetyGateLogic::ttc_release_threshold_s() const {
  if (limits_.ttc_warning_s.has_value()) {
    if (*limits_.ttc_warning_s > *limits_.ttc_brake_s) {
      return *limits_.ttc_warning_s;
    }
  }
  return *limits_.ttc_brake_s * limits_.ttc_release_hysteresis_factor;
}

void SafetyGateLogic::apply_obstacle_gate(const DriveCommand& requested, const GateInput& input,
                                          DriveCommand& target, GateResult& result) const {
  // No state estimator (/odom) feeds safety_node yet (the EKF is roadmap phase 2), so the
  // REQUESTED speed is the forward-speed estimate. Reversing/stopped is never TTC-braked.
  const double range_m = input.min_scan_range_m;
  const double forward_speed_mps = requested.speed_mps > 0.0 ? requested.speed_mps : 0.0;
  bool moving_forward = false;
  if (forward_speed_mps > kMinForwardSpeedMps) {
    moving_forward = true;
  }
  const bool obstacle = is_valid_range(range_m);  // finite and > 0

  // TTC of the REQUEST. +inf when there is no obstacle or the request is not forward.
  double ttc_s = std::numeric_limits<double>::infinity();
  if (obstacle) {
    if (moving_forward) {
      ttc_s = range_m / forward_speed_mps;
    }
  }

  // Trip tests. A disabled half (unset threshold) never trips.
  bool ttc_trip = false;
  if (limits_.ttc_brake_s.has_value()) {
    if (ttc_s <= *limits_.ttc_brake_s) {
      ttc_trip = true;
    }
  }
  bool floor_trip = false;
  if (limits_.min_forward_clearance_m.has_value()) {
    if (obstacle) {
      if (moving_forward) {
        if (range_m < *limits_.min_forward_clearance_m) {
          floor_trip = true;
        }
      }
    }
  }

  // Release test (only consulted when the latch came in set). Garbage range never releases.
  double release_ttc_s = std::numeric_limits<double>::quiet_NaN();
  double release_clearance_m = std::numeric_limits<double>::quiet_NaN();
  bool release_ok = true;
  if (is_garbage_range(range_m)) {
    release_ok = false;
  } else {
    if (limits_.ttc_brake_s.has_value()) {
      release_ttc_s = ttc_release_threshold_s();
      if (!(ttc_s > release_ttc_s)) {
        release_ok = false;
      }
    }
    if (limits_.min_forward_clearance_m.has_value()) {
      release_clearance_m = *limits_.min_forward_clearance_m * limits_.clearance_release_factor;
      if (!(range_m > release_clearance_m)) {
        release_ok = false;
      }
    }
  }

  // Trip always wins over release (gate_logic.hpp: a misconfigured release factor must never
  // produce an output pulse).
  bool latched = false;
  if (ttc_trip) {
    latched = true;
  } else if (floor_trip) {
    latched = true;
  } else if (input.ttc_brake_latched) {
    if (!release_ok) {
      latched = true;
    }
  }

  if (latched) {
    // Only forward throttle is held; a zero/reverse request passes (gate_logic.hpp).
    if (target.speed_mps > 0.0) {
      target.speed_mps = 0.0;
    }
    result.zero_throttle = true;
    std::string detail;
    if (ttc_trip) {
      detail = formatting::ttc_brake_detail(ttc_s, *limits_.ttc_brake_s);
    } else if (floor_trip) {
      detail = formatting::clearance_brake_detail(range_m, *limits_.min_forward_clearance_m);
    } else {
      detail = formatting::ttc_latch_held_detail();
    }
    result.activations.push_back(
        GateActivation{GateSource::kTtc, EventSeverity::kBrake, std::move(detail)});
  } else if (input.ttc_brake_latched) {
    // The latch released on this cycle: the "ttc brake released" note that becomes the
    // PHASE_RELEASE record's detail.
    result.releases.push_back(GateActivation{
        GateSource::kTtc, EventSeverity::kBrake,
        formatting::ttc_release_detail(ttc_s, release_ttc_s, range_m, release_clearance_m)});
  }
  result.ttc_brake_latched = latched;

  // Advisory warning zone: only when not latched, and only when the TTC brake is configured
  // (an unconfigured TTC gate is a documented no-op, CLAUDE.md invariant 2).
  if (!latched) {
    if (limits_.ttc_brake_s.has_value()) {
      if (limits_.ttc_warning_s.has_value()) {
        if (ttc_s <= *limits_.ttc_warning_s) {
          result.activations.push_back(
              GateActivation{GateSource::kTtc, EventSeverity::kInfo,
                             formatting::ttc_warning_detail(ttc_s, *limits_.ttc_warning_s)});
        }
      }
    }
  }
}

GateResult SafetyGateLogic::evaluate(const GateInput& input,
                                     const DriveCommand& previous_output) const {
  GateResult result;
  // Every return path below either recomputes the latch (step 3b) or holds it as it came in.
  result.ttc_brake_latched = input.ttc_brake_latched;

  // 1. Watchdog (claude-docs/04-architecture.md: "missing /drive_raw for 3 cycles -> brake
  // command") -- short-circuits everything else below; a stale/garbage age means there is no
  // fresh command to reason about further.
  const double watchdog_timeout_s =
      static_cast<double>(limits_.watchdog_missed_cycles) * limits_.control_period_s;
  bool watchdog_tripped = false;
  if (!std::isfinite(input.drive_raw_age_s)) {
    watchdog_tripped = true;
  } else if (input.drive_raw_age_s < 0.0) {
    watchdog_tripped = true;
  } else if (input.drive_raw_age_s >= watchdog_timeout_s) {
    watchdog_tripped = true;
  }
  if (watchdog_tripped) {
    result.output = zero_throttle_command(previous_output);
    result.zero_throttle = true;
    result.activations.push_back(
        GateActivation{GateSource::kWatchdog, EventSeverity::kBrake,
                       formatting::watchdog_detail(watchdog_timeout_s, input.drive_raw_age_s)});
    hold_obstacle_state(input, previous_output, result);
    return result;
  }

  // 2. Command sanity: non-finite (NaN/Inf) input is garbage, not a bounds violation -- it
  // also short-circuits to a hard brake rather than being clamped into something that looks
  // sane and passed through (claude-docs/05-safety.md fail-closed).
  if (!is_finite_command(input.command)) {
    result.output = zero_throttle_command(previous_output);
    result.zero_throttle = true;
    result.activations.push_back(GateActivation{GateSource::kCommandSanity, EventSeverity::kBrake,
                                                formatting::command_sanity_detail()});
    hold_obstacle_state(input, previous_output, result);
    return result;
  }

  // 3a. Absolute bounds clamp against vehicle_params limits.
  DriveCommand cmd = clamp_to_bounds(input.command);
  bool bounds_clamped = false;
  if (cmd.steering_angle_rad != input.command.steering_angle_rad) {
    bounds_clamped = true;
  } else if (cmd.speed_mps != input.command.speed_mps) {
    bounds_clamped = true;
  }
  if (bounds_clamped) {
    result.activations.push_back(GateActivation{
        GateSource::kBoundsClamp, EventSeverity::kWarning,
        formatting::bounds_clamp_detail(limits_.steering_min_rad, limits_.steering_max_rad,
                                        limits_.speed_min_mps, limits_.speed_max_mps)});
  }

  // 3b. Obstacle gate (TTC brake on the REQUESTED speed + distance floor, one latch). See
  // gate_logic.hpp "THE OBSTACLE GATE AND ITS LATCH" for why this runs BEFORE the rate
  // limiter and on the request, not the output (the 2026-10-06 limit cycle).
  const DriveCommand requested = cmd;
  DriveCommand target = requested;
  apply_obstacle_gate(requested, input, target, result);

  // 3c. Rate-limit clamp relative to the previous OUTPUT (what was actually commanded last
  // cycle, not what was requested), applied to whatever the obstacle gate left, then
  // re-clamp to absolute bounds defensively. Braking to zero is a decrease and is never
  // rate-limited, so a latched brake reaches the output on the cycle it engages.
  const double dt_s = safe_dt(input.dt_s);
  DriveCommand rate_limited = rate_limit(target, previous_output, dt_s);
  rate_limited = clamp_to_bounds(rate_limited);

  // 3d. Steering hold while parked on the obstacle latch (gate_logic.hpp, "STEERING HOLD WHILE
  // PARKED ON THE OBSTACLE LATCH"). Runs on the final speed, so "held at zero" means what the
  // car is actually sent; may freeze the steering of `target` and `rate_limited`.
  apply_steering_hold(input, previous_output, dt_s, target, rate_limited, result);

  bool rate_clamped = false;
  if (rate_limited.steering_angle_rad != target.steering_angle_rad) {
    rate_clamped = true;
  } else if (rate_limited.speed_mps != target.speed_mps) {
    rate_clamped = true;
  }
  if (rate_clamped) {
    result.activations.push_back(GateActivation{GateSource::kRateLimit, EventSeverity::kWarning,
                                                formatting::rate_limit_detail(dt_s)});
  }
  cmd = rate_limited;

  // 4. Covariance gate stub (roadmap task 2.6). Evaluated unconditionally and applied
  // unconditionally (multiplying by speed_fraction, a no-op at the stub's fixed 1.0) --
  // per this milestone's instructions, the stub must fail SAFE, meaning the ABSENCE of a
  // real /pose source must never disable the watchdog/sanity/bounds/rate/TTC gates above,
  // which have already run unconditionally by this point regardless of this step's outcome.
  const CovarianceGateResult covariance =
      evaluate_covariance_gate(input.has_pose_input, input.pose_covariance_trace);
  cmd.speed_mps *= covariance.speed_fraction;
  if (covariance.engaged) {
    result.activations.push_back(
        GateActivation{GateSource::kCovariance, EventSeverity::kWarning,
                       formatting::covariance_detail(covariance.speed_fraction)});
  }

  result.output = cmd;
  return result;
}

std::vector<SafetyEventRecord> GateEventTracker::update(
    const std::vector<GateActivation>& activations, double now_s,
    const std::vector<GateActivation>& release_notes) {
  std::vector<SafetyEventRecord> records;

  // Release reasons by slot (first note per slot wins). Pointers into `release_notes`, which
  // outlives every use of them below.
  std::array<const std::string*, kGateSourceCount * kEventSeverityCount> notes{};
  for (const GateActivation& note : release_notes) {
    const std::size_t slot = static_cast<std::size_t>(note.source) * kEventSeverityCount +
                             static_cast<std::size_t>(note.severity);
    if (notes[slot] == nullptr) {
      notes[slot] = &note.detail;
    }
  }

  // Index arithmetic, not a lookup or a switch: an engagement's identity is its
  // (source, severity) pair (see GateEventTracker's doc comment for why severity is part of
  // the identity), and both enums are contiguous from zero.
  std::array<bool, kGateSourceCount * kEventSeverityCount> active_this_cycle{};

  for (const GateActivation& activation : activations) {
    const std::size_t slot = static_cast<std::size_t>(activation.source) * kEventSeverityCount +
                             static_cast<std::size_t>(activation.severity);
    if (active_this_cycle[slot]) {
      // The same (source, severity) reported twice in one cycle is still ONE engagement.
      // No gate does this today; treating it as one engagement rather than trusting the
      // caller keeps the record stream's "one engage per engagement" promise unconditional.
      continue;
    }
    active_this_cycle[slot] = true;
    if (engagements_[slot].engaged) {
      // Already engaged on a previous cycle: emit NOTHING. This one line is GitHub issue
      // #37 -- the sustained state is the interval between the engage record and its
      // release record, not a record per cycle.
      continue;
    }
    engagements_[slot].engaged = true;
    engagements_[slot].engaged_at_s = now_s;
    records.push_back(SafetyEventRecord{activation.source, activation.severity, EventPhase::kEngage,
                                        activation.detail,
                                        /*duration_s=*/0.0});
  }

  for (std::size_t slot = 0; slot < engagements_.size(); ++slot) {
    if (!engagements_[slot].engaged) {
      continue;
    }
    if (active_this_cycle[slot]) {
      continue;
    }
    const double duration_s = engagement_duration_s(engagements_[slot].engaged_at_s, now_s);
    engagements_[slot].engaged = false;
    std::string detail;
    if (notes[slot] != nullptr) {
      detail = formatting::release_detail_with_reason(duration_s, *notes[slot]);
    } else {
      detail = formatting::release_detail(duration_s);
    }
    records.push_back(SafetyEventRecord{static_cast<GateSource>(slot / kEventSeverityCount),
                                        static_cast<EventSeverity>(slot % kEventSeverityCount),
                                        EventPhase::kRelease, std::move(detail), duration_s});
  }

  return records;
}

}  // namespace racer_safety
