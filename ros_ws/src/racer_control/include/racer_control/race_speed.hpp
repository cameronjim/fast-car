// Physics-based speed law for gap_follow_node's race profile (2026-11-12), plus the small
// latency statistics the node publishes. ROS-free, SI, gtest-covered (test/test_race_speed.cpp).
// Off unless gap_follow_node's `physics_speed` is true: the old range-based law
// (reactive_speed.hpp) is untouched and stays the default.
//
// TARGET SPEED = max(min_speed, min(max_speed, v_lateral, v_stop, v_side)).
//
//   v_lateral  the speed at which the commanded steering arc needs a_lat_max of lateral
//              acceleration. The bicycle model turns a steering angle delta into a rear-axle turn
//              radius R = L / tan|delta| (L = chassis.wheelbase_m from the binding), and a car on
//              that circle at speed v needs v^2 / R, so v_lateral = sqrt(a_lat_max * R). Straight
//              ahead (delta 0) has no limit. a_lat_max is a node parameter, NOT a vehicle
//              constant: the schema has no friction limit (CLAUDE.md invariant 2 forbids
//              inventing one), and the sim's tyre model is not the floor's.
//   v_stop     the speed from which the car can stop inside the free distance D along the swept
//              path (free_arc_travel): stopping takes latency * v (the car keeps its speed for
//              actuation.stop_latency_s) plus v^2 / (2 a) (a = actuation.stop_deceleration_mps2,
//              both from the binding, both DERIVED from the floor bags), and it must finish
//              `standoff_m` short of the return (limits.min_forward_clearance_m plus a margin).
//              Solving latency * v + v^2 / (2 a) = D - standoff for v:
//                  v = a * (sqrt(latency^2 + 2 (D - standoff) / a) - latency),  0 when D <=
//                  standoff.
//   v_side     a cap while a wall is close beside the car, smooth in the clearance c to the nearer
//              side wall (the body, not the LiDAR): no cap at c >= side_cap_distance_m, a straight
//              line down to side_cap_min_speed at c = 0.
//
// The floor is min_speed_mps (as in the range law): the car keeps creeping so that safety_node's
// TTC gate and the reverse escape, not a speed of exactly zero, decide what happens at a wall. The
// stopping limit therefore never commands less than the floor; below it the safety layer is the
// authority (05-safety.md: this is layer-3 risk reduction at best, never a guarantee).
//
// The caller (ReactiveSpeedCommand::update_target) then low-passes and rate limits the target
// exactly like the range law, so the acceleration bound still holds on what is published.
#ifndef RACER_CONTROL_RACE_SPEED_HPP_
#define RACER_CONTROL_RACE_SPEED_HPP_

#include <cstddef>
#include <optional>
#include <vector>

namespace racer_control {

// sqrt(a_lat_max * L / tan|steering|). +infinity for a straight (|tan| below 1e-9). 0 for a
// non-finite steering, a non-positive wheelbase or a non-positive / non-finite a_lat_max (fail
// slow, like apply_steering_slowdown).
double lateral_limit_speed(double steering_rad, double wheelbase_m, double a_lat_max_mps2);

// The speed that stops within free_path_m - standoff_m (see above). 0 when that distance is not
// positive, and for any non-finite or non-positive input (fail slow). +infinity is not returned:
// a long free path gives a large finite speed.
double stopping_limit_speed(double free_path_m, double standoff_m, double decel_mps2,
                            double latency_s);

// Distance the car covers stopping from speed_mps: latency * v + v^2 / (2 a). The inverse of
// stopping_limit_speed (without the standoff). 0 for a non-finite or non-positive speed; a
// non-positive decel returns +infinity.
double stopping_distance(double speed_mps, double decel_mps2, double latency_s);

// Smooth cap from the body clearance beside the car: +infinity when `clearance_m` is empty, the
// distance is not positive (off) or c >= distance_m; otherwise
//     min_speed + (max_speed - min_speed) * c / distance_m      (c clamped at 0).
double side_proximity_cap(std::optional<double> clearance_m, double distance_m,
                          double min_speed_mps, double max_speed_mps);

// How far along the swept path the free travel is measured, for a top speed: the distance that
// stops from max_speed (stopping_distance) plus the standoff and 0.3 m of margin, at most 3 m.
// Shared by gap_follow_node and gap_follow_replay_cli so they size the probe alike. 0 for a
// non-positive or non-finite top speed.
double free_path_horizon(double max_speed_mps, double decel_mps2, double latency_s,
                         double standoff_m);

struct RaceSpeedConfig {
  double wheelbase_m = 0.0;       // chassis.wheelbase_m
  double a_lat_max_mps2 = 2.0;    // node parameter
  double brake_decel_mps2 = 1.0;  // actuation.stop_deceleration_mps2
  double stop_latency_s = 0.0;    // actuation.stop_latency_s
  double standoff_m = 0.0;        // limits.min_forward_clearance_m + the node's margin
  double max_speed_mps = 0.0;
  double min_speed_mps = 0.0;        // the floor
  double side_cap_distance_m = 0.0;  // 0 = no side cap
  double side_cap_min_speed_mps = 0.0;
};

struct RaceSpeedInputs {
  double steering_rad = 0.0;               // the commanded (filtered) steering
  std::optional<double> free_path_m;       // free_arc_travel; empty = no stopping limit
  std::optional<double> side_clearance_m;  // empty = no side cap
};

struct RaceSpeedTarget {
  double lateral_mps = 0.0;   // +infinity when straight
  double stopping_mps = 0.0;  // +infinity when no free path was given
  double side_mps = 0.0;      // +infinity when no cap applies
  double target_mps = 0.0;    // after the min and the floor
};

RaceSpeedTarget race_target_speed(const RaceSpeedInputs& inputs, const RaceSpeedConfig& config);

// Fixed-capacity window of the last N samples with percentiles, for the scan-to-command latency
// and per-scan CPU cost the node publishes. Storage is allocated once in the constructor; push()
// and percentile() never allocate. Non-finite samples are ignored.
class PercentileWindow {
 public:
  explicit PercentileWindow(std::size_t capacity);
  void push(double value);
  void clear();
  std::size_t size() const { return count_; }
  // Nearest-rank percentile, q in [0, 1] (clamped): the value at rank ceil(q * n) of the sorted
  // window (rank at least 1). 0 for an empty window.
  double percentile(double q);
  double max() const;

 private:
  std::vector<double> ring_;
  std::vector<double> scratch_;
  std::size_t next_ = 0;
  std::size_t count_ = 0;
};

}  // namespace racer_control

#endif  // RACER_CONTROL_RACE_SPEED_HPP_
