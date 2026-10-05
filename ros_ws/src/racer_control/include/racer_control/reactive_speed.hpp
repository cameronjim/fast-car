// Speed laws shared by the reactive controllers (GitHub issue 26 item f). ROS-free, SI.
//
// The old Python nodes took their speed from the old safety node over /speed. That path is
// gone (racer_safety/safety_node owns braking and the speed cap); speed is now a pure function
// of what the controller sees and how hard it is steering, and the nodes then ramp it through
// racer_control::SpeedRateLimiter exactly like tracker_node.
#ifndef RACER_CONTROL_REACTIVE_SPEED_HPP_
#define RACER_CONTROL_REACTIVE_SPEED_HPP_

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

}  // namespace racer_control

#endif  // RACER_CONTROL_REACTIVE_SPEED_HPP_
