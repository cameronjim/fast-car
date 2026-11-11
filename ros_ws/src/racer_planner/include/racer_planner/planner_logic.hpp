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
  NoTrack = 2,
  NoScan = 3,
  NoOdom = 4,
  Blocked = 5,
  Fault = 6,
};

struct LogicParams {
  double v_max_mps = 0.5;
  double track_timeout_s = 0.30;
  double scan_timeout_s = 0.35;
  double odom_timeout_s = 0.10;
  int min_valid_stations = 6;
  double max_boundary_residual_m = 0.25;
  double max_intrusion_fraction = 0.10;
  double selfcheck_hold_s = 0.5;
  bool use_lap_guidance = false;
  double guidance_timeout_s = 2.0;
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

// True when the label free self-check is outside its limits. NaN in either number counts as over
// the limit (fail closed: an unverifiable track is not trusted for more than selfcheck_hold_s).
bool selfcheck_over_limits(double boundary_residual_m, double intrusion_fraction,
                           const LogicParams& p);

// Remembers since when the self-check has been continuously over its limits.
class SelfCheckHold {
 public:
  // Returns true when `over` has held continuously for at least hold_s.
  bool update(bool over, double now_s, double hold_s);
  void reset() { over_since_valid_ = false; }

 private:
  bool over_since_valid_ = false;
  double over_since_s_ = 0.0;
};

struct Gate {
  PlannerState state = PlannerState::Starting;
  bool may_solve = false;  // true only for Driving (the solver may still report Blocked)
};

// The decision table, checked in order: FAULT, NO_ODOM, NO_SCAN, NO_TRACK, otherwise solve.
Gate evaluate_gate(const LogicInputs& in, const LogicParams& p, SelfCheckHold* hold);

// v_max_mps, or min(v_max_mps, guidance cap) with a fresh guidance message when enabled. Never
// below 0, never above v_max_mps, NaN guidance ignored.
double speed_cap(const LogicParams& p, const LogicInputs& in);

struct Command {
  double speed_mps = 0.0;
  double steering_rad = 0.0;
};

// What goes on /drive_raw for a state. For Driving the solver command is sanitised (finite,
// speed in [0, v_cap], steering in [steer_min, steer_max]); Blocked takes the solver's steering
// with speed 0; Fault is 0, 0; the NO_* states are speed 0 with the last steering.
Command command_for_state(PlannerState state, const Command& solver_cmd, double last_steering_rad,
                          double v_cap_mps, double steer_min_rad, double steer_max_rad);

// Output rate limits: steering by max_steer_step, speed upward by max_speed_up_step (slowing is
// never limited). Both relative to the previous published command.
Command rate_limit(const Command& wanted, const Command& previous, double max_steer_step_rad,
                   double max_speed_up_step_mps);

// A double rounded to float32 that never lies outside [lo, hi] (0.449f is above 0.449). Non
// finite input gives 0.
float float32_within(double x, double lo, double hi);

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
