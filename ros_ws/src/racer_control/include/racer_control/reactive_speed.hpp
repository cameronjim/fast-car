// Speed laws shared by the reactive controllers (GitHub issue 26 item f). ROS-free, SI.
//
// The old Python nodes took their speed from the old safety node over /speed. That path is
// gone (racer_safety/safety_node owns braking and the speed cap); speed is now a pure function
// of what the controller sees and how hard it is steering, and the nodes then ramp it through
// racer_control::SpeedRateLimiter exactly like tracker_node.
//
// Smoothing (2026-10-06 floor checkpoint, docs/notes/reactive-control-port-2026-10-05.md): the
// target range flickers from scan to scan at 10 Hz and the range-based speed followed it, so
// the car surged and slowed. Two optional stages, both off by default:
//   * RollingMedian replaces each scan's target range with the median of the last N scans'
//     target ranges (gap_follow_node `target_range_median_scans`, 1 = off).
//   * ReactiveSpeedCommand low-pass filters the speed command BEFORE the SpeedRateLimiter
//     (`speed_time_constant_s`, 0 = off), so the limiter still bounds the acceleration of the
//     command that is actually published.
#ifndef RACER_CONTROL_REACTIVE_SPEED_HPP_
#define RACER_CONTROL_REACTIVE_SPEED_HPP_

#include <cstddef>
#include <vector>

#include "racer_control/gap_follow.hpp"
#include "racer_control/speed_rate_limiter.hpp"

namespace racer_control {

// clamp(k_speed * range_m, min_speed_mps, max_speed_mps). A non-finite or negative range
// returns min_speed_mps (the sanitised ranges the cores feed in are always finite, this is
// belt and braces so a NaN can never reach a published speed).
double range_based_speed(double range_m, double k_speed_per_s, double min_speed_mps,
                         double max_speed_mps);

// speed * (1 - k_steer * min(1, |steering| / max_steering)), floored at 0. k_steer in [0, 1]:
// 0 disables the slowdown, 1 stops the car at full lock. Non-finite inputs return 0.
double apply_steering_slowdown(double speed_mps, double steering_rad, double k_steer,
                               double max_steering_rad);

// Median of the last `window` finite values pushed (window 0 is treated as 1). Storage is
// allocated once in the constructor; push() never allocates. Before the window has filled,
// the median is over the values pushed so far; with an even count it is the mean of the two
// middle values. A window of 1 returns each pushed value unchanged, bit for bit.
class RollingMedian {
 public:
  explicit RollingMedian(std::size_t window);
  // Adds `value` and returns the median of the window. A non-finite value is not stored: the
  // median of what is already held is returned, or `value` itself when nothing is held (the
  // speed law maps a non-finite range to min speed).
  double push(double value);
  void reset();
  std::size_t window() const { return ring_.size(); }
  std::size_t size() const { return count_; }

 private:
  std::vector<double> ring_;
  std::vector<double> scratch_;
  std::size_t next_ = 0;
  std::size_t count_ = 0;
};

struct ReactiveSpeedConfig {
  double k_speed_per_s = 1.0;
  double min_speed_mps = 0.0;
  double max_speed_mps = 0.0;
  double k_steer = 0.0;
  double max_steering_rad = 0.0;  // steering.max_angle_rad, from the binding in the node
  // First-order low-pass time constant on the speed command (s). <= 0 skips the filter
  // entirely, so the output is exactly the range speed -> slowdown -> rate limiter chain.
  double speed_time_constant_s = 0.0;
  // SpeedRateLimiter's ramp limit (m/s^2), from the binding in the node.
  double max_acceleration_mps2 = 0.0;
};

// The per-cycle speed chain of gap_follow_node:
//   range_based_speed(target range) -> apply_steering_slowdown -> FirstOrderLowPass
//   -> SpeedRateLimiter.
// The low-pass sits before the limiter on purpose: the limiter's bound then applies to the
// published command whatever the filter does.
class ReactiveSpeedCommand {
 public:
  explicit ReactiveSpeedCommand(const ReactiveSpeedConfig& config);
  // target_range_m: range along the target bearing (already median filtered, if enabled).
  // dt_s: the configured control period, or 0 on the first cycle after a reset.
  double update(double target_range_m, double steering_rad, double dt_s);
  // Back to rest: filter output and limiter state 0 (safety_node braked the car meanwhile).
  void reset();
  const ReactiveSpeedConfig& config() const { return config_; }

 private:
  ReactiveSpeedConfig config_;
  FirstOrderLowPass filter_;
  SpeedRateLimiter limiter_;
};

}  // namespace racer_control

#endif  // RACER_CONTROL_REACTIVE_SPEED_HPP_
