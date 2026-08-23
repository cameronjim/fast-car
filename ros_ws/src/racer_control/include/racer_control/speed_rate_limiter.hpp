// SpeedRateLimiter: ramps a raw commanded speed toward the vehicle's own acceleration bound
// (roadmap milestone 3).
//
// Motivation (found empirically while building tests/e2e_sim_safety/test_sim_autopilot_e2e.py):
// racer_control::PurePursuitController::compute_command commands the raceline's raw target
// speed at the vehicle's nearest point, with no ramp of its own (see pure_pursuit.cpp). That
// target-speed profile was generated (tools/raceline) as a function of ARC LENGTH, friction-
// limited, not a function of TIME -- so successive nearest points visited one control cycle
// apart do not, in general, imply a speed change that respects
// vehicle_params.actuation.max_acceleration_mps2 over that cycle's actual dt. Without this
// class, tracker_node's raw /drive_raw speed continuously trips
// racer_safety::SafetyGateLogic's rate-limit gate in completely ordinary driving -- not a
// brief startup transient, but throughout a run -- which is exactly the "tracker and gate
// disagree" failure claude-docs/12-testing.md's milestone-3 e2e test exists to catch.
//
// This is intentionally a SEPARATE, small implementation from
// racer_safety::SafetyGateLogic::rate_limit, not a shared dependency: racer_control and
// racer_safety are different packages with different testing/coverage requirements
// (claude-docs/12-testing.md gates racer_safety's gate logic at 100% branch coverage; this is
// "normal package" tracker math), and the asymmetry/semantics this class copies (only
// accelerating is rate-limited; braking/holding is never rate-limited; no elapsed time means
// no increase is allowed) are re-implemented here rather than imported.
//
// Deliberately NOT part of PurePursuitController/compute_command's public API: that shared
// core is exercised by pure_pursuit_cli and cross-checked byte-for-byte against
// training/racer_train's Python port (test/divergence/compare_divergence.py) -- changing its
// output would require updating that Python port and its committed reference fixture in
// lockstep, which is out of this milestone's scope. tracker_node.cpp applies this limiter to
// the CORE's raw output as a node-level, ROS-plumbing-adjacent step instead (still ROS-free
// itself, gtest-unit-testable with no ROS install per claude-docs/12-testing.md L1).
#ifndef RACER_CONTROL_SPEED_RATE_LIMITER_HPP_
#define RACER_CONTROL_SPEED_RATE_LIMITER_HPP_

namespace racer_control {

class SpeedRateLimiter {
 public:
  explicit SpeedRateLimiter(double max_acceleration_mps2)
      : max_acceleration_mps2_(max_acceleration_mps2) {}

  // raw_speed_mps: this cycle's raw, unramped target speed. dt_s: seconds since the previous
  // call; non-finite or <= 0.0 is treated as "no time elapsed" (no increase is allowed that
  // cycle) -- the same fail-safe convention racer_safety::SafetyGateLogic uses for garbage
  // dt. Returns the rate-limited speed and updates internal state to that returned value (an
  // "output relative to the previous OUTPUT" contract, not the raw input -- same contract as
  // racer_safety::SafetyGateLogic::rate_limit).
  double limit(double raw_speed_mps, double dt_s);

  double previous_speed_mps() const { return previous_speed_mps_; }

 private:
  double max_acceleration_mps2_;
  double previous_speed_mps_ = 0.0;
};

}  // namespace racer_control

#endif  // RACER_CONTROL_SPEED_RATE_LIMITER_HPP_
