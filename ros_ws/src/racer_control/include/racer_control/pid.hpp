// PID controller for the wall follower (GitHub issue 26 item g), ported from the old Python
// reactive_control/pid.py with its defects removed. ROS-free; the caller supplies dt, which
// wall_follow_node measures on the steady clock (never the ROS clock, see that node's CLOCK
// POLICY).
//
// Changes from the old version:
//   * No magic first-step dt (the old code assumed 0.01 s for the first call) and no 1e-6 s
//     floor for a non-positive dt (which turned any clock hiccup into a derivative spike of a
//     million times the error change). The first sample after construction or reset() is
//     proportional only plus whatever integral is already stored; a later sample with a
//     non-finite or non-positive dt is treated the same way (P + stored I, no derivative,
//     no integration).
//   * Derivative on the error (not the measurement), skipped on the first sample.
//   * The integral clamp is a parameter (units: error units times seconds, here m*s), not a
//     hard-coded 100.
//
// The gap follower deliberately does NOT use this: a PID on a target bearing is not
// meaningful (its I term winds up on any steady curve). See gap_follow.hpp.
#ifndef RACER_CONTROL_PID_HPP_
#define RACER_CONTROL_PID_HPP_

namespace racer_control {

struct PidGains {
  double kp = 0.0;
  double ki = 0.0;
  double kd = 0.0;
  double integral_limit = 0.0;  // |integral of error| is clamped to this (>= 0)
};

class Pid {
 public:
  explicit Pid(PidGains gains) : gains_(gains) {}

  // Returns kp * e + ki * I + kd * de/dt. Non-finite error returns 0 and leaves state alone.
  double update(double error, double dt_s);
  void reset();

  double integral() const { return integral_; }

 private:
  PidGains gains_;
  double integral_ = 0.0;
  double previous_error_ = 0.0;
  bool has_previous_ = false;
};

}  // namespace racer_control

#endif  // RACER_CONTROL_PID_HPP_
