// The decision layer: staleness, the state machine, the speed cap and what is published. Pure
// functions and small value classes, no ROS, every branch covered by test_planner_logic.cpp.
#ifndef RACER_PLANNER_PLANNER_LOGIC_HPP_
#define RACER_PLANNER_PLANNER_LOGIC_HPP_

#include <cstdint>
#include <limits>

#include "racer_planner/dynamics_model.hpp"
#include "racer_planner/local_track.hpp"

namespace racer_planner {

// Values match racer_pnp_msgs/PlannerStatus.
enum class PlannerState : std::uint8_t {
  Starting = 0,
  Driving = 1,
  NoTrack = 2,  // not produced since v0.1: a distrusted track lowers the confidence instead
  NoScan = 3,
  NoOdom = 4,
  Blocked = 5,
  Fault = 6,
  Recovering = 7,
};

struct LogicParams {
  double v_max_mps = 0.5;
  // Graceful staleness. A scan up to scan_timeout_s old gives the full cap; from there to
  // scan_hard_timeout_s the cap is multiplied by a linear ramp 1 to 0; beyond it: NO_SCAN. A wheel
  // speed up to odom_timeout_s old is used as it is; from there to odom_hard_timeout_s the planner
  // keeps solving with its own speed estimate and the cap may not exceed the last measured wheel
  // speed plus odom_stale_cap_margin_mps; beyond it: NO_ODOM. A stamp from the future by more than
  // the first timeout, or one that is not finite, is beyond both at once.
  double scan_timeout_s = 0.35;
  double scan_hard_timeout_s = 0.60;
  double odom_timeout_s = 0.10;
  double odom_hard_timeout_s = 0.20;
  double odom_stale_cap_margin_mps = 0.05;
  bool use_lap_guidance = false;
  double guidance_timeout_s = 2.0;
  // With use_lap_guidance and no fresh, well formed guidance the cap is min(v_max_mps, this): lap
  // learning makes v_max_mps the ceiling, and a dead lap memory node must not send the car there.
  // The default (no limit beyond v_max_mps) is what the node and the replay tool replace with
  // v_max_mps itself.
  double v_no_guidance_mps = std::numeric_limits<double>::infinity();
  // Plan snapshots (Planner::publish): a snapshot older than this while the state would be Driving
  // or Recovering gives speed 0 and Fault (s).
  double plan_timeout_s = 0.12;
  // Warm start gate (start_gate): ticks that must each arrive within start_tick_gap_s of the
  // previous one, how long STARTING may last before the missing condition is logged, how long the
  // published speed must have been 0 before a return to Driving goes through STARTING again, and
  // whether the gated /drive must have been seen (false: sim tests that run without safety_node).
  int start_ticks = 25;
  double start_tick_gap_s = 0.03;
  double start_timeout_s = 5.0;
  double start_stop_hold_s = 1.0;
  bool require_gated_for_start = true;
  // Fewer usable scan points than this make the scan unusable (NO_SCAN, speed 0): an empty distance
  // field must never read as free space. A scan of the 720 beam lidar in a lane has hundreds.
  int min_scan_points = 10;
  // While the gated /drive has not been seen yet (DDS discovery of safety_node takes a few ticks),
  // STARTING does not publish /drive_raw for up to this long: the steering it would start from is
  // not known, and a guess of 0 against safety_node's held steering is a rate limit clip.
  double start_gated_grace_s = 0.5;
  // A gap between two planner ticks longer than this (s) means the planner was stalled long
  // enough for safety_node's /drive_raw watchdog (60 ms) to have braked: the speed ramp then
  // restarts from zero, as from rest, instead of asking safety for a jump from 0 to the old
  // speed (which it rate limits). 0 turns it off.
  double gap_reset_s = 0.05;
  double gated_timeout_s = 0.2;  // the gated /drive older than this never counts
  // Output shaping of a driving command (Planner): first-order low-pass time constant on the
  // steering, and the share of the vehicle's steering rate and acceleration limits the planner's
  // own rate limit may use (safety_node limits at the full values, so the planner stays inside).
  double steer_output_tau_s = 0.04;
  double rate_limit_fraction = 0.45;
  // The published steering holds its value until the low-passed target is this far from it (rad):
  // microradian wiggles of a static scene are not commands (a servo cannot follow them anyway).
  double steer_output_hysteresis_rad = 0.0015;
  // Track confidence (see track_confidence_raw): the ends of the four ramps and how fast it may
  // rise. It falls at once.
  double conf_intrusion_full = 0.03;
  double conf_intrusion_zero = 0.15;
  double conf_residual_full_m = 0.10;
  double conf_residual_zero_m = 0.30;
  double conf_valid_zero = 3.0;
  double conf_valid_full = 8.0;
  double conf_age_full_s = 0.25;
  double conf_age_zero_s = 0.60;
  double conf_rise_per_s = 1.0;
  // Share of the speed cap kept at track confidence 0: cap * (f + (1 - f) * c).
  double freespace_speed_fraction = 0.7;
};

struct TrackSummary {
  bool present = false;
  double stamp_s = 0.0;
  TrackValidity validity = TrackValidity::NoTrack;
  int valid_count = 0;
  double boundary_residual_m = 0.0;
  double intrusion_fraction = 0.0;
};

struct LogicInputs {
  double now_s = 0.0;
  bool internal_error = false;
  bool has_odom = false;
  double odom_stamp_s = 0.0;
  double wheel_speed_mps = 0.0;  // the last measured one, used by speed_cap while it is stale
  bool has_scan = false;
  double scan_stamp_s = 0.0;
  TrackSummary track;
  bool has_guidance = false;
  double guidance_stamp_s = 0.0;
  double guidance_v_cap_mps = 0.0;
  // Set by the lap memory plumbing when a non-empty lap profile is malformed or too old: forces
  // the same fallback as a missing guidance.
  bool guidance_profile_bad = false;
};

// now - stamp, NaN when either is not finite.
double age_s(double now_s, double stamp_s);

// True when the data is older than the timeout, from the future by more than the timeout (a
// timestamp jump), or its time is not finite.
bool is_stale(double now_s, double stamp_s, double timeout_s);

// 1 for x <= full, 0 for x >= zero, linear between. A non finite x gives 0. Degenerate ends
// (zero <= full) make a step at `full`.
double ramp_down(double x, double full, double zero);
// 0 for x <= zero, 1 for x >= full, linear between. A non finite x gives 0. Degenerate ends
// (full <= zero) make a step at `zero`.
double ramp_up(double x, double zero, double full);

// How far the track is trusted before the rise limit, 0 to 1:
//   0 when the track is missing, fails the contract, or any of intrusion fraction, boundary
//   residual, track age (or a timestamp from the future by more than the age ramp's zero end) is
//   not finite; otherwise the minimum of
//   ramp_down(intrusion, full, zero), ramp_down(residual, full, zero),
//   ramp_up(valid_count, zero, full), ramp_down(age, full, zero).
double track_confidence_raw(const TrackSummary& t, double now_s, const LogicParams& p);

// c = min(c_raw, c_prev + rise_per_s * tick_s): falls at once, rises slowly. A non finite
// c_prev counts as 0. The result is within [0, 1].
double track_confidence(double c_raw, double c_prev, double tick_s, double rise_per_s);

struct Gate {
  PlannerState state = PlannerState::Starting;
  bool may_solve = false;  // true only for Driving (the planner may still report Blocked)
};

// The first rows of the decision table, checked in order: FAULT, NO_ODOM (wheel speed older than
// odom_hard_timeout_s, missing, or a bad stamp), NO_SCAN (scan older than scan_hard_timeout_s,
// missing, or a bad stamp), otherwise the planner may solve (it then decides between DRIVING and
// BLOCKED). The track never gates.
Gate evaluate_gate(const LogicInputs& in, const LogicParams& p);

// How an input's age is judged: Fresh up to the first timeout (a stamp ahead of now by up to the
// timeout is clock skew, still Fresh), Soft between the timeouts, Hard beyond the second, ahead of
// now by more than the first, or not finite.
enum class Age : std::uint8_t { Fresh = 0, Soft = 1, Hard = 2 };
Age classify_age(double now_s, double stamp_s, double timeout_s, double hard_timeout_s);

// min(v_max_mps, guidance cap) scaled by (f + (1 - f) * confidence) with f =
// freespace_speed_fraction, then by the scan staleness ramp (1 up to scan_timeout_s, linear to 0 at
// scan_hard_timeout_s, 0 for a missing scan or a bad stamp), then at most the last wheel speed plus
// odom_stale_cap_margin_mps while the wheel speed is between its two timeouts (0 when it is
// missing or beyond the second). With use_lap_guidance the guidance cap applies when the guidance
// is fresh, finite and its profile is not bad; otherwise the cap is min(v_max_mps,
// v_no_guidance_mps). Never below 0, never above v_max_mps, a NaN confidence counts as 0.
double speed_cap(const LogicParams& p, const LogicInputs& in, double confidence = 1.0);

struct Command {
  double speed_mps = 0.0;
  double steering_rad = 0.0;
};

// What goes on /drive_raw for a state. For Driving the solver command is sanitised (finite,
// speed in [0, v_cap], steering in [steer_min, steer_max]); Blocked takes the given steering with
// speed 0; Recovering takes the recovery command with the speed in [-reverse_limit, v_cap] (the
// only state with a negative speed); Fault is 0, 0; the NO_* states are speed 0 with the last
// steering.
Command command_for_state(PlannerState state, const Command& solver_cmd, double last_steering_rad,
                          double v_cap_mps, double steer_min_rad, double steer_max_rad,
                          double reverse_limit_mps = 0.0);

// Output rate limits: steering by max_steer_step, the speed's magnitude by max_speed_up_step when
// it grows, forward or backward (slowing, braking to zero included, is never limited; a request
// across zero goes to zero at once and grows from there). The dead zone: with speed_floor_mps > 0 a
// forward speed below the floor that is asked to go to the floor ramps in equal steps of floor /
// ceil(floor / max_speed_up_step_mps), never larger than max_speed_up_step_mps, so a start from
// rest at a 0.23 m/s floor is three ticks of 0.077 m/s: at most two ticks inside the dead zone,
// then the floor itself.
Command rate_limit(const Command& wanted, const Command& previous, double max_steer_step_rad,
                   double max_speed_up_step_mps, double speed_floor_mps = 0.0);

// Slew limits by elapsed time (the planner publishes from a timer that can be late or arrive just
// after a late one, so the limit follows the measured time, not the nominal tick).
struct SlewLimits {
  double tick_s = 0.02;  // the nominal tick: the upper bound of one step's elapsed time
  double steering_rate_rad_per_s = 0.0;
  double max_acceleration_mps2 = 0.0;
  double fraction = 0.45;  // share of the two limits the planner may use
  double speed_floor_mps = 0.0;
};

// elapsed clamped to [0, tick_s]; NaN gives 0 (no movement allowed).
double slew_elapsed_s(double elapsed_s, double tick_s);

// rate_limit with steps of f * rate * e and f * accel * e, e = slew_elapsed_s(elapsed_s). The dead
// zone ramp keeps its shape for an on-time tick (equal steps of floor / ceil(floor / (f * accel *
// tick))) and scales its step by e / tick otherwise, so the ramp takes the same wall time however
// the ticks land and no step exceeds the limit of its own elapsed time.
Command slew_limit(const Command& wanted, const Command& previous, double elapsed_s,
                   const SlewLimits& limits);

// The control `age_s` after the first of n controls spaced dt_s apart: the first for age <= 0 or a
// NaN age, the last beyond the end, linear in between.
Control interpolate_plan(const Control* seq, int n, double dt_s, double age_s);

// Why STARTING has not ended yet, first missing condition in this order.
enum class StartMissing : std::uint8_t {
  None = 0,
  NoPlan = 1,        // no plan snapshot, or one older than plan_timeout_s
  Scan = 2,          // scan not fresh
  Odom = 3,          // wheel speed not fresh
  Track = 4,         // a track has arrived once and is not fresh now
  TickTiming = 5,    // fewer than start_ticks consecutive publish ticks within start_tick_gap_s
  SolverWarmup = 6,  // fewer than start_ticks consecutive solves on the real inputs
  Gated = 7,         // the gated /drive has not been seen (and it is required)
};
const char* start_missing_name(StartMissing m);

struct StartGateInputs {
  bool plan_fresh = false;
  bool scan_fresh = false;
  bool odom_fresh = false;
  bool track_ever = false;  // /pnp/local_track has arrived at least once
  bool track_fresh = false;
  int publish_good_ticks = 0;  // consecutive publishes each within start_tick_gap_s of the last
  int solve_good_ticks = 0;    // consecutive solves on the real inputs, same gap rule
  bool gated_seen = false;
};

struct StartGateResult {
  bool leave = false;  // STARTING may end
  StartMissing missing = StartMissing::None;
};

// The warm start gate: leave STARTING only when every condition holds (design 14.7). A pure
// decision; the planner keeps the counters. Not a safety function: safety_node gates /drive_raw.
StartGateResult start_gate(const StartGateInputs& in, const LogicParams& p);

// One step of a first-order low-pass: previous + (wanted - previous) * (1 - exp(-tick / tau)).
// tau <= 0 or a non finite input passes `wanted` (or `previous` when wanted is not finite).
double lowpass_step(double previous, double wanted, double tick_s, double tau_s);

// A double rounded to float32 that never lies outside [lo, hi] (0.449f is above 0.449). Non
// finite input gives 0.
float float32_within(double x, double lo, double hi);

// Why the planner has nothing drivable (values match PlannerStatus.block_reason).
enum class BlockReason : std::uint8_t {
  None = 0,
  AllCollide = 1,
  BelowSpeedFloor = 2,
  WheelStalled = 3,
  GateRefusing = 4,
  RecoveryExhausted = 5,
};

struct DrivabilityParams {
  double stall_wheel_speed_mps = 0.03;  // wheel speed under this ...
  double stall_hold_s = 1.5;            // ... for this long while the published speed was at least
                                        // the floor: WheelStalled
  double gate_hold_s = 1.0;             // gated /drive speed 0 for this long while the published
                                        // speed was positive: GateRefusing
};

struct DrivabilityInputs {
  bool no_plan = false;  // nothing to plan toward (no track weight and no free-space field)
  bool all_collide = false;
  double plan_speed_mps = 0.0;  // mean speed command of the plan over its first 0.5 s
  double speed_floor_mps = 0.0;
  double wheel_speed_mps = 0.0;
  double published_speed_mps = 0.0;  // last published, before this tick
  bool gate_fresh = false;           // a fresh /drive was seen; a stale one never counts
  double gated_speed_mps = 0.0;
};

// Nothing drivable, as the decision table needs it (claude-docs section 3.6):
//   1 AllCollide: every sample collides (immediate).
//   2 BelowSpeedFloor: there is no plan, or its mean speed over the first 0.5 s is under the
//     floor (immediate).
//   3 WheelStalled: wheel speed under 0.03 m/s for stall_hold_s while the published speed was at
//     least the floor.
//   4 GateRefusing: the gated /drive speed is 0 for gate_hold_s while the published speed was
//     positive.
// The monitor keeps how long each condition has held continuously, which recovery also uses.
class DrivabilityMonitor {
 public:
  explicit DrivabilityMonitor(const DrivabilityParams& p = DrivabilityParams{}) : p_(p) {}
  // Updates the timers with this tick and returns the reason that makes it "nothing drivable"
  // (first of 1 to 4 in order), or None.
  BlockReason update(const DrivabilityInputs& in, double now_s);
  // How long reason r (1 to 4) has held continuously as of the last update, 0 when not.
  double held_s(BlockReason r) const;
  bool active(BlockReason r) const;
  void reset();

 private:
  static constexpr int kReasons = 4;
  DrivabilityParams p_;
  bool active_[kReasons] = {false, false, false, false};
  double since_s_[kReasons] = {0.0, 0.0, 0.0, 0.0};
  double now_s_ = 0.0;
};

struct SampleBudgetParams {
  int max_samples = 768;
  int min_samples = 128;
  double halve_over_s = 0.015;
  double restore_under_s = 0.006;
  int restore_after_ticks = 25;
};

// Halves the sample count after a slow solve and restores it gradually after fast ones.
class SampleBudget {
 public:
  explicit SampleBudget(const SampleBudgetParams& p);
  int current() const { return current_; }
  // Feed the latest solve time. Returns the sample count for the next tick. `halved` (optional)
  // is set true on the tick the count was halved.
  int update(double solve_time_s, bool* halved);

 private:
  SampleBudgetParams p_;
  int floor_;
  int current_;
  int fast_ticks_ = 0;
};

}  // namespace racer_planner

#endif  // RACER_PLANNER_PLANNER_LOGIC_HPP_
