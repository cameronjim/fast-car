// Follow-the-gap reactive controller core (GitHub issue 26), ported from the old Python
// reactive_control/gap_logic.py with the maths fixes the owner asked for. ROS-free and
// gtest-covered (test/test_gap_follow.cpp); gap_follow_node owns the ROS plumbing.
//
// Pipeline, once per /scan:
//   1. sanitize_ranges (laser_scan.hpp): invalid-return policy, clip to clip_max_range_m.
//   2. extend_disparities: at every range discontinuity larger than disparity_threshold_m,
//      overwrite the rays on the FAR side with the near range over the angle the car's half
//      width (plus a safety margin) subtends at that near range, so a gap that survives is
//      wide enough for the car.
//   3. select_gap: the widest run of rays above free_space_threshold_m inside a cone of
//      +/- cone_half_angle_rad around the VEHICLE's forward direction, tie-broken toward
//      forward. Target its centre ray (default) or its deepest ray.
//   4. steering_from_bearing: steering = clamp(gain * target_bearing, +/- max_angle).
//   5. corner_blocked: zero the steering if the whole side sector the car would turn into is
//      within min_clearance (the car is hugging that wall and would clip it).
// The node then low-pass filters the steering (FirstOrderLowPass) and derives speed from the
// range along the target bearing (reactive_speed.hpp) before rate limiting it.
//
// Sign convention (claude-docs/06-vehicle-params.md): bearings and steering are LEFT
// positive. A gap to the left of the car gives a positive bearing and a positive steering
// command (test/test_gap_follow.cpp pins this against a scan in racer_gym_bridge's ray
// ordering).
//
// Heap: GapFollower owns its working buffers and only grows them, so once it has seen a
// scan of the running size, process() allocates nothing (claude-docs/10-conventions.md).
#ifndef RACER_CONTROL_GAP_FOLLOW_HPP_
#define RACER_CONTROL_GAP_FOLLOW_HPP_

#include <cstddef>
#include <optional>
#include <vector>

#include "racer_control/laser_scan.hpp"

namespace racer_control {

// Disparity extension (step 2). `ranges` are sanitised ranges (finite, >= 0). Writes the
// extended ranges into `out` (resized to ranges.size()). Discontinuities are detected on the
// INPUT ranges, never on partially extended output, so the result does not depend on the
// order edges are processed in. For an edge between rays i and i+1 whose nearer side is
// `near`, the bubble covers
//     ceil(atan2(half_width_m, near) / angle_increment)
// rays starting at the first ray on the far side (the near ray already reads `near`), and
// only ever lowers a range (out[k] = min(out[k], near)). ceil rather than the old int()
// truncation: the true obstacle edge lies somewhere inside the near ray's angular bin, up to
// one increment beyond the ray itself, so rounding up is the conservative choice. near == 0
// gives atan2(w, 0) = pi/2, a full quarter turn, which is the right answer for an obstacle at
// the lens. Work is O(n + total bubble length). A non-positive or non-finite angle_increment
// or half width leaves the ranges unchanged.
void extend_disparities(const std::vector<double>& ranges, double disparity_threshold_m,
                        double half_width_m, double angle_increment, std::vector<double>& out);

struct GapSelection {
  // False when no ray in the cone is above the free-space threshold; the target is then the
  // forward ray (the old code's "aim straight ahead" fallback).
  bool gap_found = false;
  std::size_t target_index = 0;  // index into the scan
  // Inclusive scan indices of the chosen gap's ends (the cone may wrap on a 360 degree
  // scan, so gap_first can be numerically greater than gap_last). Equal to target_index when
  // gap_found is false.
  std::size_t gap_first = 0;
  std::size_t gap_last = 0;
  double target_vehicle_bearing_rad = 0.0;  // left positive, wrapped to [-pi, pi)
};

enum class GapTarget {
  kCentre,   // centre ray of the chosen gap (the old behaviour, default)
  kDeepest,  // deepest ray of the chosen gap, ties broken toward forward
};

// Gap selection (step 3). `geometry` supplies angle_min / angle_increment / ray count;
// `ranges` are the extended ranges (same length). The cone is +/- cone_half_angle_rad around
// VEHICLE bearing 0 (laser bearing -laser_yaw_offset_rad), wrapping around the seam of a full
// circle scan and clipped to the scan's coverage otherwise. Ties between equally wide gaps
// go to the gap whose centre is nearest forward. Returns nullopt when the forward direction
// itself is outside the scan (nothing sensible to aim at) or the scan is unusable.
std::optional<GapSelection> select_gap(const ScanInput& geometry, const std::vector<double>& ranges,
                                       double free_space_threshold_m, double cone_half_angle_rad,
                                       double laser_yaw_offset_rad, GapTarget target);

// Corner override (step 5). True when steering is non-zero and every ray whose VEHICLE
// bearing lies in the turn-in side sector has range < min_clearance_m. The sector is
// [sector_inner_rad, sector_outer_rad] on the LEFT for a left (positive) command and its
// mirror [-sector_outer_rad, -sector_inner_rad] for a right one. A sector containing no rays
// (e.g. beyond a narrow scan's coverage) is never blocked. `ranges` are sanitised ranges.
//
// Note on the old code: it checked the LAST sixth of the rays for a right turn, but in
// LaserScan ordering the last rays are on the LEFT, so it guarded the wrong side. This
// version checks the side the car actually turns into.
bool corner_blocked(const ScanInput& geometry, const std::vector<double>& ranges,
                    double steering_rad, double sector_inner_rad, double sector_outer_rad,
                    double min_clearance_m, double laser_yaw_offset_rad);

// Step 4: clamp(gain * bearing, +/- max_steering_rad). Non-finite input returns 0.
double steering_from_bearing(double bearing_rad, double gain, double max_steering_rad);

// Discrete first-order low-pass y += dt / (tau + dt) * (x - y), stable for any dt > 0. A
// non-finite or non-positive dt holds the previous output (same "no elapsed time, no change"
// convention as SpeedRateLimiter), so the first sample after construction or reset() holds
// at 0, the straight-ahead servo position. tau <= 0 passes the input straight through
// (when dt > 0). A non-finite input is ignored (output held).
class FirstOrderLowPass {
 public:
  explicit FirstOrderLowPass(double time_constant_s) : time_constant_s_(time_constant_s) {}
  double update(double input, double dt_s);
  void reset() { output_ = 0.0; }
  double output() const { return output_; }

 private:
  double time_constant_s_;
  double output_ = 0.0;
};

struct GapFollowConfig {
  // Physical (from the generated vehicle_params binding in the node, never typed in here).
  double half_width_m = 0.0;            // chassis.width_m / 2
  double max_steering_angle_rad = 0.0;  // steering.max_angle_rad
  // Tuning.
  double safety_margin_m = 0.0;  // added to half_width_m for the disparity bubble
  double clip_max_range_m = 0.0;
  double disparity_threshold_m = 0.0;
  double free_space_threshold_m = 0.0;
  double cone_half_angle_rad = 0.0;
  double laser_yaw_offset_rad = 0.0;
  GapTarget target = GapTarget::kCentre;
  double steering_gain = 1.0;
  double corner_sector_inner_rad = 0.0;
  double corner_sector_outer_rad = 0.0;
  double corner_min_clearance_m = 0.0;
};

struct GapFollowResult {
  // False when the scan is unusable, has no valid returns, or does not cover forward. The
  // node must not produce a command from an invalid result.
  bool valid = false;
  bool gap_found = false;
  bool corner_blocked = false;
  double target_vehicle_bearing_rad = 0.0;
  double target_range_m = 0.0;  // extended range along the target ray (input to speed)
  double steering_rad = 0.0;    // after clamp and corner override, BEFORE low-pass
};

class GapFollower {
 public:
  explicit GapFollower(GapFollowConfig config) : config_(config) {}
  GapFollowResult process(const ScanInput& scan);
  const GapFollowConfig& config() const { return config_; }

 private:
  GapFollowConfig config_;
  std::vector<double> sanitized_;
  std::vector<double> extended_;
};

}  // namespace racer_control

#endif  // RACER_CONTROL_GAP_FOLLOW_HPP_
