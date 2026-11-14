// The what-if replay core: racer_safety's UNCHANGED gate (SafetyGateLogic::evaluate, the same
// rate limiter and watchdog safety_node runs) driven cycle by cycle from a recorded /drive_raw
// stream. ROS-free. Nothing here publishes or drives anything; it is an offline measurement
// (pre-registration PnP v0.3 section 5, brief 34).
//
// What it models, and what it leaves out, so a count is never read as more than it is:
//   * The gate is built exactly as safety_node builds it (default limits from the generated
//     vehicle_params binding, the same defaults safety_replay_cli uses).
//   * One cycle per recorded safety cycle. The cycle time, the command in force and its age come
//     from the stream; the caller chooses the dt that is passed to the gate (the recorded one, or
//     the nominal period).
//   * There is NO scan: the forward and rear path distances are +infinity every cycle, so the
//     obstacle gate never trips and no latch or steering hold is ever set. A TTC brake in the
//     real run therefore has no counterpart here. In the 2026-11-13 PnP runs there was none
//     inside the counting window (pre-registration v0.2 section 4.2); outside it this is a
//     stated difference, not an error.
//   * No wheel odometry: the measured speed is absent, as when /odom/wheel is stale.
#pragma once

#include <string>
#include <vector>

#include "racer_safety/gate_logic.hpp"

namespace racer_session_tools {

// The gate limits exactly as safety_node's build_limits() sets them at its parameter defaults.
racer_safety::SafetyLimits default_limits();

// The nominal gate period, limits.control_period_s (1 / 50 Hz).
double nominal_period_s(const racer_safety::SafetyLimits& limits);

struct CycleInput {
  double stamp_s = 0.0;  // the cycle's time, used only to label events
  double dt_s = 0.0;     // what the gate is told about the time since the previous cycle
  racer_safety::DriveCommand command;  // the /drive_raw command in force
  double age_s = 0.0;    // seconds since that command arrived (+inf: none yet)
};

struct EventOut {
  double t_s = 0.0;
  bool engage = true;
  std::string source;
  std::string severity;
  std::string detail;
  double duration_s = 0.0;
};

struct CycleOutput {
  racer_safety::DriveCommand output;
  bool rate_limit_clipped = false;  // a rate_limit activation this cycle (a clip, one per cycle)
  bool watchdog_tripped = false;    // a watchdog activation this cycle
  std::vector<EventOut> events;     // engage and release records, as safety_node would publish
};

// safety_node's threaded state: previous output, latches, hold timer, speed bounds, event tracker.
class Replayer {
 public:
  explicit Replayer(const racer_safety::SafetyLimits& limits);
  CycleOutput step(const CycleInput& in);
  // Start a segment from a known output: the previous output becomes `output`, latches, hold
  // timer, speed bounds and the event tracker are cleared. Used per run, because the recorded
  // /drive_raw can lack the messages that led to the output a run starts from.
  void seed(const racer_safety::DriveCommand& output);
  // Teacher forcing: replace only the previous output (latches and tracker are kept). Judging a
  // cycle from the recorded output of the cycle before isolates the limiter's decision from
  // any drift of a free-running chain.
  void force_previous(const racer_safety::DriveCommand& output) { previous_ = output; }

 private:
  racer_safety::SafetyGateLogic gate_;
  racer_safety::DriveCommand previous_;
  bool latched_ = false;
  bool reverse_latched_ = false;
  std::optional<double> hold_timer_s_;
  double bound_forward_ = 0.0;
  double bound_reverse_ = 0.0;
  racer_safety::GateEventTracker tracker_;
};

}  // namespace racer_session_tools
