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
//   3. select_gap: the best-scoring run of rays above free_space_threshold_m inside a cone
//      of +/- cone_half_angle_rad around the VEHICLE's forward direction, tie-broken toward
//      forward. The score is the gap's angular width, optionally weighted toward forward
//      (forward_preference) and with optional switching hysteresis (gap_switch_margin); both
//      default to off, which is the plain widest gap. Target its centre ray (default) or its
//      deepest ray.
//   4. steering_from_bearing: steering = clamp(gain * target_bearing, +/- max_angle).
//   4a. lane centring (optional, centering_gain, default 0 = off): a push away from the
//      nearer side wall is added to gain * target_bearing BEFORE that clamp, so the sum
//      saturates at the steering limit (2026-10-06 floor finding (a), see below).
//   4b. clamp_steering_to_swept_path (optional, default on): reduce |steering| until the
//      car's swept area over the next swept_path_lookahead_m of arc is clear of every return
//      beside and ahead of it on the turn-in side (2026-10-06 floor finding, see below).
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

// Gap scoring and switching (step 3). Added after the 2026-10-06 floor test (bag
// 2026-10-06T22-12-40_car_teleop): on a lane with a continuous wall on one side and
// scattered objects on the other, "widest run of free rays" picked a gap between the
// objects, into the open room, in 62 of 63 scans under every parameter set tried, because a
// side opening close to the car subtends a wider angle than the lane ahead.
//
// forward_preference p in [0, 1]. Each candidate gap scores
//     score = angular_width_rad * max(0, 1 - p * (1 - cos(centre_bearing)))
// where angular_width_rad = (ray count) * angle_increment and centre_bearing is the VEHICLE
// frame bearing of the gap's midpoint (laser yaw applied, left positive). The highest score
// wins; exact ties still go to the gap whose centre is nearest forward. Why this weighting:
//   * p = 0 makes the factor exactly 1, so the score is the angular width and the choice is
//     bit-identical to the plain widest gap (pinned in test/test_gap_follow.cpp against a
//     copy of the old code).
//   * p = 1 makes the factor cos(bearing): a gap at 90 degrees scores zero, one at 60
//     degrees counts half its width.
//   * The factor is even in the bearing (left and right are treated alike) and does not
//     increase with |bearing| on [0, pi], so for a fixed width a gap nearer forward always
//     scores at least as well.
//   * 1 - cos is quadratic near 0, so a lane that is only a few degrees off axis is barely
//     penalised (10 degrees costs p * 1.5 percent), while side openings are penalised hard.
//   * The clamp at 0 only matters for p > 0.5 and bearings past 90 degrees (a cone wider
//     than +/- pi/2), where 1 - p * (1 - cos) would go negative; those gaps all score 0 and
//     the forward tie-break picks among them.
// A side opening still wins when it is wide enough: with p = 0.6, a gap centred at 60
// degrees must be 1 / 0.7 = 1.43 times wider than a gap dead ahead to beat it.
//
// gap_switch_margin m in [0, 1] (0 = off). If a gap's span contains the previous cycle's
// target bearing (the ray nearest that bearing lies inside the gap), that gap is kept unless
// the best-scoring gap's score exceeds its score by more than the fraction m, i.e. unless
// best_score > incumbent_score * (1 + m). This stops the target flipping between two
// similar gaps on alternate scans. With m = 0, or no previous target, or no gap containing
// it, selection is the plain best score above.
struct GapPreference {
  double forward_preference = 0.0;
  double switch_margin = 0.0;
  // Vehicle-frame bearing of the previous cycle's target, when the previous cycle found a
  // gap. Ignored when switch_margin is 0.
  std::optional<double> previous_target_vehicle_bearing_rad;
};

enum class GapTarget {
  kCentre,   // centre ray of the chosen gap (the old behaviour, default)
  kDeepest,  // deepest ray of the chosen gap, ties broken toward forward
};

// Gap selection (step 3). `geometry` supplies angle_min / angle_increment / ray count;
// `ranges` are the extended ranges (same length). The cone is +/- cone_half_angle_rad around
// VEHICLE bearing 0 (laser bearing -laser_yaw_offset_rad), wrapping around the seam of a full
// circle scan and clipped to the scan's coverage otherwise. Gaps are scored and chosen as
// described above GapPreference; the default preference is the plain widest gap. Ties go to
// the gap whose centre is nearest forward. Returns nullopt when the forward direction itself
// is outside the scan (nothing sensible to aim at), the scan is unusable, or a preference
// field is outside [0, 1] or not finite.
std::optional<GapSelection> select_gap(const ScanInput& geometry, const std::vector<double>& ranges,
                                       double free_space_threshold_m, double cone_half_angle_rad,
                                       double laser_yaw_offset_rad, GapTarget target,
                                       const GapPreference& preference = {});

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

// Lane centring (step 4a). Added after the 2026-10-06 night floor test (bags
// 2026-10-06T23-19-41 and 23-25-38, replayed through this core): the follower is EDGE-BIASED.
// It aims at the angular centre of the widest run of free rays, and rays grazing the wall the
// car is already near stay long and count as free, so the gap centre drags toward that wall.
// The owner watched it hug the edges in both directions. Centring adds a push away from the
// nearer side wall, measured directly from the scan.
//
// MEASUREMENT (measure_lane_walls). Returns are taken from the RAW scan (invalid returns, as
// for the swept-path clamp, are ignored, never filled), turned to the VEHICLE frame with the
// laser yaw. A return at vehicle bearing b and range r is on the LEFT if 0 < b <=
// sector_half_angle_rad and on the RIGHT if -sector_half_angle_rad <= b < 0 (b = 0 is on
// neither side), and counts only if r <= max_range_m. Its perpendicular distance from the
// car's centreline is r sin(b) + lidar_mount_y_m on the left and -(r sin(b) + lidar_mount_y_m)
// on the right (only positive values count). The side's wall distance is the MEDIAN of those
// perpendicular distances, not the nearest return's. Why the median (the "cleaner estimate"):
//   * a wall parallel to the car gives the same perpendicular distance on every ray, so the
//     median, the minimum and the nearest return all agree on a straight lane;
//   * a wall ACROSS the lane ahead (the outside wall of the next corner) also falls in the
//     front sector, and its rays just off the centreline have a perpendicular distance near
//     zero. The nearest return (or the minimum r |sin b|) then reads a wall at about 0 m on
//     whichever side that ray lands, and the push saturates the steering away from it, which
//     on the way into a corner is away from the turn. The median of a side that holds both a
//     side wall and part of a wall ahead stays near the side wall's distance, and a side that
//     sees only the wall ahead reads a typical distance across that wall rather than 0, so
//     the push into a corner stays small (the gap steering is large there anyway);
//   * one near object (a chair leg) does not decide the push; avoiding objects is the job of
//     the gap, the swept-path clamp and safety_node, not of centring.
// A side with no counted return is OPEN (nullopt).
//
// PUSH (centering_steering). With both sides measured,
//     centering = centering_gain * (d_left - d_right) / (d_left + d_right)
// so the push is positive (LEFT) when the right wall is nearer, zero when centred, odd in the
// offset, and bounded by +/- centering_gain. (The task text wrote (d_right - d_left); with the
// left-positive steering convention that steers TOWARD the nearer wall, the opposite of its
// own "positive = steer left when the right wall is nearer". The tests pin the stated intent:
// a car offset to the right steers left.) If either side is OPEN the push is ZERO: centring
// is relative to a lane with two walls, and with one side open (a doorway, the room beyond a
// row of scattered objects) there is no centre to aim at. Substituting max_range_m for the
// open side would push the car toward the opening, which is the "into the open room" failure
// of the first floor test. In a straight lane of width W, an offset e from the centre gives
// (d_left - d_right) / (d_left + d_right) = 2 e / W, i.e. a push of 2 * gain / W rad per metre.
//
// Garbage (non-finite or non-positive sector or max range, unusable scan, non-finite yaw or
// mount) measures both sides open, so centring contributes nothing.
struct LaneWalls {
  std::optional<double> left_m;   // median perpendicular distance of the left wall, m
  std::optional<double> right_m;  // same for the right wall (a positive distance)
};

// `left_scratch` / `right_scratch` are working buffers (grown, never shrunk), so a caller that
// keeps them allocates nothing once they have reached the scan size.
LaneWalls measure_lane_walls(const ScanInput& scan, double laser_yaw_offset_rad,
                             double sector_half_angle_rad, double max_range_m,
                             double lidar_mount_y_m, std::vector<double>& left_scratch,
                             std::vector<double>& right_scratch);

// centering_gain * (d_left - d_right) / (d_left + d_right); 0 when either side is open, the
// gain is 0 or not finite, or a distance is not finite and positive.
double centering_steering(const LaneWalls& walls, double centering_gain);

// Swept-path steering clamp (step 4b). Added after the 2026-10-06 floor test (bag
// 2026-10-06T22-12-40_car_teleop, counter-clockwise loop with loose objects on the inside of
// every left corner): the follower steered to a geometrically correct gap, but the car's
// inside flank swept across the apex object (62 of 63 replayed scans chose a gap whose turn
// passed within the car's width of an object beside the car). The disparity bubble only
// inflates obstacle edges angularly inside the search cone; an object alongside the car, 60
// to 100 degrees off the LiDAR's forward axis, never constrains the turn.
//
// GEOMETRY. Rear-axle frame, x forward, y left (REP-103). For a steering delta != 0 the rear
// axle follows a circle of signed radius R = L / tan(delta), L = wheelbase, about the turn
// centre (0, R), the same model racer_safety's arc corridor uses. The working below is for a
// LEFT turn; a right turn is its mirror image (y -> -y), so with c = half_width + margin:
//   * inside flank: the body point nearest the centre is (0, half_width) (the rear axle lies
//     inside the body), so the inside of the swept area is the circle of radius R - c.
//   * outside front corner: (body_front_x, -half_width) sweeps the outer circle,
//     R_out = hypot(body_front_x, R + c) (margin on this side too, which is conservative).
//   * a return at (x, y) (head-relative x, y from range and VEHICLE bearing, then shifted by
//     the LiDAR mount: x += lidar_mount_x, y += lidar_mount_y) is considered only if it is on
//     the turn-in side and outside the car's width, y > half_width (a return straight ahead
//     inside the car's width is the TTC gate's job, not this clamp's), and ahead of the car
//     along the turn: arc angle phi = atan2(x, R - y) in (0, pi/2), with the rear axle's arc
//     length R * phi <= lookahead_m.
//   * it is in the swept area if its distance from the centre rho = hypot(x, R - y) satisfies
//     R - c < rho <= R_out.
//   * it is clear on the INSIDE (rho <= R - c) exactly when the curvature k = 1 / R satisfies
//         k <= k_max = 2 (y - c) / (x^2 + y^2 - c^2)     (y > c; k_max = 0 when y <= c)
//     (square rho <= R - c and solve for R: R >= (x^2 + y^2 - c^2) / (2 (y - c))). A return
//     within the margin of the car's side (half_width < y <= c) therefore forbids any turn
//     toward it.
// ALGORITHM. Start from the wanted curvature k = tan(|delta|) / L. Every return in the
// swept area at k (with the window above evaluated at k) is a violator; set k to the
// smallest k_max among the violators and repeat until there are none. Each pass strictly
// lowers k and a return is never a violator again once k <= its k_max, so this ends in at
// most one pass per return (usually one or two). The result is delta = sign * atan(k L): the
// inside flank circle passes every considered return at a distance of at least the margin
// (exactly the margin for the binding return). The wanted steering is returned unchanged,
// bit for bit, when it is zero or non-finite, when nothing violates, or when the geometry is
// unusable (non-positive or non-finite wheelbase, half width or lookahead; negative or
// non-finite margin; non-finite mount or front x; unusable scan).
//
// Invalid returns (non-finite, <= 0, below range_min or above range_max) are ignored, never
// filled from neighbours: this reads the RAW scan, not the sanitised or extended ranges.
// When the clamp drives the steering to (near) zero while the target bearing is large, that
// is intended: the corner override and the safety node take it from there.
struct SweptPathGeometry {
  double wheelbase_m = 0.0;      // chassis.wheelbase_m
  double half_width_m = 0.0;     // chassis.width_m / 2
  double margin_m = 0.0;         // the follower's safety_margin_m
  double body_front_x_m = 0.0;   // rear axle to the front of the body, for the outer circle
  double lidar_mount_x_m = 0.0;  // sensors.lidar.mount_x_m (rear axle to head, forward)
  double lidar_mount_y_m = 0.0;  // sensors.lidar.mount_y_m (left)
  double lookahead_m = 0.0;      // swept_path_lookahead_m, rear-axle arc length
};

struct SweptPathClamp {
  double steering_rad = 0.0;
  bool clamped = false;  // true only when |steering| was reduced
};

SweptPathClamp clamp_steering_to_swept_path(const ScanInput& scan, double laser_yaw_offset_rad,
                                            double steering_rad, const SweptPathGeometry& geometry);

// Forward arc probe for the reverse escape (reverse_escape.hpp; 2026-10-06 night floor finding
// (b): nose-in to a corner tighter than the turning circle). "Is there ANY steering that would
// let the car drive forward `travel_m` without its body touching a return?" If not, the forward
// path is blocked and backing out is the only way on.
//
// GEOMETRY. The same kinematic model as the swept-path clamp: for steering delta the rear axle
// follows a circle of curvature k = tan(delta) / L (rear-axle frame, x forward, y left). The
// body is the rectangle x in [-rear_x_m - margin_m, front_x_m + margin_m], |y| <= half_width_m +
// margin_m about the rear axle, i.e. the bounding box inflated by the margin on every side.
// Returns are taken from the RAW scan (invalid returns ignored, never filled), turned to the
// vehicle frame with the laser yaw and shifted by the LiDAR mount into the rear-axle frame.
// A return already inside the inflated body at the start pose is ignored: it is either the car
// itself or contact the probe cannot judge (safety_node's job); everything else must stay out
// of the body at every pose along the arc. The arc is sampled every kArcProbeStepM of
// rear-axle travel up to and including travel_m; between samples a body corner moves at most
// that step times hypot(front_x, R + c) / R, about 1.3 times the step at full lock, so a
// return can only slip between samples within a couple of centimetres of the body, inside
// the margin.
//
// CANDIDATES. kArcProbeCandidates steering angles evenly spaced over [-max, +max] (9: every
// 0.105 rad at the 0.4189 rad lock), including straight ahead. This is a numerical resolution,
// not a vehicle constant.
//
// forward_arc_clear: true when that one steering's arc is clear for travel_m.
// any_forward_arc_clear: true when at least one candidate is.
// Unusable input (scan, yaw, non-positive wheelbase / half width / travel / max steering at or
// beyond pi/2, negative or non-finite margin / extents / mounts) reports CLEAR: the probe only
// ever ADDS a reason to back up, so garbage must not invent one (the node refuses to start
// without the geometry, so this is belt and braces).
inline constexpr std::size_t kArcProbeCandidates = 9;
inline constexpr double kArcProbeStepM = 0.02;

struct ArcProbeGeometry {
  double wheelbase_m = 0.0;      // chassis.wheelbase_m
  double half_width_m = 0.0;     // chassis.width_m / 2
  double margin_m = 0.0;         // the follower's safety_margin_m
  double front_x_m = 0.0;        // rear axle to the front of the body (as for the clamp)
  double rear_x_m = 0.0;         // rear axle to the rear bumper line, chassis.rear_overhang_m
  double lidar_mount_x_m = 0.0;  // sensors.lidar.mount_x_m
  double lidar_mount_y_m = 0.0;  // sensors.lidar.mount_y_m
};

// `points` is a working buffer (grown, never shrunk) for the returns in the rear-axle frame.
bool forward_arc_clear(const ScanInput& scan, double laser_yaw_offset_rad, double steering_rad,
                       double travel_m, const ArcProbeGeometry& geometry,
                       std::vector<double>& points);
bool any_forward_arc_clear(const ScanInput& scan, double laser_yaw_offset_rad,
                           double max_steering_rad, double travel_m,
                           const ArcProbeGeometry& geometry, std::vector<double>& points);

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
  double forward_preference = 0.0;  // [0, 1], see GapPreference; 0 = plain widest gap
  double gap_switch_margin = 0.0;   // [0, 1], see GapPreference; 0 = off
  double steering_gain = 1.0;
  double corner_sector_inner_rad = 0.0;
  double corner_sector_outer_rad = 0.0;
  double corner_min_clearance_m = 0.0;
  // Swept-path clamp (step 4b). Off in a default-constructed config, so the core's default is
  // the pipeline without it; gap_follow_node's swept_path_clamp parameter defaults to true.
  // The geometry's half width and margin are taken from half_width_m and safety_margin_m
  // above (whatever swept_path holds for them is ignored); the node fills the rest from the
  // binding.
  bool swept_path_clamp = false;
  SweptPathGeometry swept_path;
  // Lane centring (step 4a, see measure_lane_walls / centering_steering). centering_gain 0
  // (the default) skips the measurement entirely and the pipeline is bit-identical to the
  // one without centring. The LiDAR's lateral mount is taken from swept_path.lidar_mount_y_m.
  double centering_gain = 0.0;
  double centering_sector_half_angle_rad = 1.0;
  double centering_max_range_m = 1.5;
};

struct GapFollowResult {
  // False when the scan is unusable, has no valid returns, or does not cover forward. The
  // node must not produce a command from an invalid result.
  bool valid = false;
  bool gap_found = false;
  bool corner_blocked = false;
  double target_vehicle_bearing_rad = 0.0;
  double target_range_m = 0.0;  // extended range along the target ray (input to speed)
  // Lane centring (step 4a): the walls it measured (both open when centring is off) and the
  // push it added before the clamp (0 when off or when a side is open).
  LaneWalls lane_walls;
  double centering_steering_rad = 0.0;
  // clamp(steering_gain * bearing + centering, +/- max): the steering the follower wants,
  // before the swept-path clamp and the corner override.
  double wanted_steering_rad = 0.0;
  bool swept_path_clamped = false;  // the swept-path clamp reduced |steering|
  double steering_rad = 0.0;        // after clamps and corner override, BEFORE low-pass
};

class GapFollower {
 public:
  explicit GapFollower(GapFollowConfig config) : config_(config) {}
  GapFollowResult process(const ScanInput& scan);
  const GapFollowConfig& config() const { return config_; }

 private:
  GapFollowConfig config_;
  // Target bearing of the last scan that found a gap, for gap_switch_margin. Cleared when a
  // processed scan finds no gap; kept across scans process() rejects as unusable.
  std::optional<double> previous_target_vehicle_bearing_rad_;
  std::vector<double> sanitized_;
  std::vector<double> extended_;
  std::vector<double> lane_left_;
  std::vector<double> lane_right_;
};

}  // namespace racer_control

#endif  // RACER_CONTROL_GAP_FOLLOW_HPP_
