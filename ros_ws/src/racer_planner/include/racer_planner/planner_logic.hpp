// The decision layer: staleness, the state machine, the speed cap and what is published. Pure
// functions and small value classes, no ROS, every branch covered by test_planner_logic.cpp.
#ifndef RACER_PLANNER_PLANNER_LOGIC_HPP_
#define RACER_PLANNER_PLANNER_LOGIC_HPP_

#include <cstdint>

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
  double scan_timeout_s = 0.35;
  double odom_timeout_s = 0.10;
  bool use_lap_guidance = false;
  double guidance_timeout_s = 2.0;
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
  bool has_scan = false;
  double scan_stamp_s = 0.0;
  TrackSummary track;
  bool has_guidance = false;
  double guidance_stamp_s = 0.0;
  double guidance_v_cap_mps = 0.0;
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

// The first rows of the decision table, checked in order: FAULT, NO_ODOM, NO_SCAN, otherwise the
// planner may solve (it then decides between DRIVING and BLOCKED). The track never gates.
Gate evaluate_gate(const LogicInputs& in, const LogicParams& p);

// min(v_max_mps, guidance cap with a fresh guidance message when enabled) scaled by
// (f + (1 - f) * confidence) with f = freespace_speed_fraction. Never below 0, never above
// v_max_mps, NaN guidance ignored, a NaN confidence counts as 0.
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
