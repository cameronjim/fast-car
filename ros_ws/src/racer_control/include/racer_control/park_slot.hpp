// Parking-slot detection and lane measurement from one LaserScan, for park_node (roadmap 2.9).
// ROS-free and gtest-covered (test/test_park_slot.cpp).
//
// FRAME. Every result is in the vehicle frame at the time of the scan: the REAR AXLE centre,
// x forward, y left (REP-103; park_geometry.hpp). A return at range r and LASER bearing b is
// turned to the vehicle frame with the mount yaw (vehicle bearing = b + laser_yaw_offset_rad,
// pi on this car) and shifted by the LiDAR mount (x = r cos + mount_x, y = r sin + mount_y),
// the same arithmetic as laser_scan.hpp / gap_follow.hpp. Invalid returns (non-finite, <= 0,
// below range_min or above range_max) are ignored, never filled (the RPLIDAR's "no return" is
// +inf or 0).
//
// SLOT. The car drives slowly forward along a row of obstacles on the parking side
// (`side`, right by default). The LATERAL DISTANCE of a return is its distance from the car's
// centreline toward the parking side (-y on the right, +y on the left). The returns on the
// parking side with a lateral distance in [lateral_min_m, lateral_max_m] and x in
// [-window_back_m, window_ahead_m] are put into bins of bin_m along x, each bin keeping its
// SMALLEST lateral distance (the nearest obstacle at that x). Then:
//   1. EMPTY BINS. A bin with no return is OPEN (lateral distance +inf): nothing is there within
//      the band, or the LiDAR cannot see it, which next to a pocket is the pocket's interior in
//      the shadow of the obstacle the LiDAR is looking past. A run of empty bins at most
//      gap_fill_m long between two occupied bins takes the larger of its neighbours' values: at
//      a grazing angle far ahead the rays hit the row face more than one bin apart, and those
//      holes must not read as pockets.
//   2. FACE LEVEL. The reference face is the 25th percentile of the occupied bins' lateral
//      distances: the row face as long as the pockets cover less than three quarters of the
//      window.
//   3. CANDIDATES. Maximal runs of bins at least jump_min_m beyond the reference face (open bins
//      included). A run that touches either end of the window is not bounded and is not a slot
//      yet (its far end has not been seen).
//   4. EDGES AND DEPTH. For a bounded run, the face on each side is the median of up to three
//      bins next to it; the ROW lateral distance is the larger of the two (the car parks fully
//      inside both faces). The slot's NEAR edge is the run's first bin's lower boundary, its FAR
//      edge the last bin's upper boundary: the obstacle bins next to the run had a return at the
//      face, the run's bins had none, so the obstacles end inside the neighbouring bins. The
//      DEPTH is the smallest lateral distance inside the run (an open bin counts as
//      lateral_max_m) minus the row lateral distance; back_lateral_m = row + depth, and
//      back_seen is false when no bin of the run had a return (open, or beyond the band).
//   5. ACCEPTANCE. depth >= depth_min_m and length (far - near) >= length_min_m. The first
//      accepted run in x is reported. Otherwise the longest bounded candidate is reported as the
//      reason: kTooShort when it is shorter than length_min_m, else kTooShallow.
//   6. ROW ANGLE (checked before 3 to 5). A least-squares line through the face bins (every
//      occupied bin that is not a candidate) gives the row's heading relative to the car
//      (row_angle_rad, counter-clockwise). With at least four face bins and |angle| >
//      max_row_angle_rad the scan is refused (kRowAngle) before any pocket is looked at: the
//      planner assumes the car is parallel to the row.
// The nearest return on the OPPOSITE side inside the same x window and band is reported as
// far_side_lateral_m (a lane wall or another row), so the planner can keep the body's swing-out
// off it.
//
// LANE. measure_lane reports the nearest return on each side (perpendicular distance from the
// centreline) among the returns with x in [x_min_m, x_max_m] and |y| in [lateral_min_m,
// lateral_max_m]: the lane the car stands in, for the three-point turn.
//
// Heap: SlotDetector keeps its bin buffers and only grows them.
#ifndef RACER_CONTROL_PARK_SLOT_HPP_
#define RACER_CONTROL_PARK_SLOT_HPP_

#include <cstddef>
#include <optional>
#include <vector>

#include "racer_control/laser_scan.hpp"

namespace racer_control {

enum class ParkSide { kRight, kLeft };

// +1 for left, -1 for right: lateral distance on the parking side = side_sign * y.
double side_sign(ParkSide side);
const char* park_side_name(ParkSide side);

struct SlotDetectorConfig {
  ParkSide side = ParkSide::kRight;
  double laser_yaw_offset_rad = 0.0;  // vehicle bearing = laser bearing + this
  double lidar_mount_x_m = 0.0;       // sensors.lidar.mount_x_m
  double lidar_mount_y_m = 0.0;       // sensors.lidar.mount_y_m
  double lateral_min_m = 0.0;         // chassis.width_m / 2: nothing nearer is a row
  double lateral_max_m = 2.0;         // outer edge of the band
  double window_back_m = 1.5;         // x from -window_back_m ...
  double window_ahead_m = 2.5;        // ... to +window_ahead_m (rear-axle frame)
  double bin_m = 0.05;
  double jump_min_m = 0.1;    // a candidate is at least this beyond the face
  double depth_min_m = 0.0;   // park_node slot_depth_min_m
  double length_min_m = 0.0;  // park_node slot_length_min_m
  double gap_fill_m = 0.15;
  double max_row_angle_rad = 0.087;
};

// True when the numbers make sense (finite; positive bin, band and window; lateral_min <
// lateral_max; depth and length minima positive; jump_min positive).
bool is_usable_detector_config(const SlotDetectorConfig& config);

enum class SlotReject {
  kNone,
  kBadInput,    // unusable scan or configuration
  kNoReturns,   // nothing on the parking side inside the window and band
  kNoPocket,    // no bounded run beyond the face
  kTooShort,    // longest bounded pocket shorter than length_min_m
  kTooShallow,  // long enough but not deep enough
  kRowAngle,    // the row is not parallel to the car
};
const char* slot_reject_name(SlotReject reject);

struct SlotDetection {
  bool found = false;
  SlotReject reject = SlotReject::kNoPocket;
  double near_x_m = 0.0;  // rear-axle frame at scan time
  double far_x_m = 0.0;
  double row_lateral_m = 0.0;   // face of the row, positive toward the parking side
  double back_lateral_m = 0.0;  // back of the pocket (row + depth)
  double depth_m = 0.0;
  double length_m = 0.0;
  bool back_seen = false;
  double row_angle_rad = 0.0;
  std::optional<double> far_side_lateral_m;  // nearest return on the opposite side
  // The candidate the reject reason describes (also filled for a found slot).
  double candidate_length_m = 0.0;
  double candidate_depth_m = 0.0;
};

class SlotDetector {
 public:
  explicit SlotDetector(const SlotDetectorConfig& config) : config_(config) {}
  SlotDetection detect(const ScanInput& scan);
  const SlotDetectorConfig& config() const { return config_; }

 private:
  SlotDetectorConfig config_;
  std::vector<double> bins_;
  std::vector<double> occupied_;
};

struct LaneMeasurement {
  std::optional<double> left_m;   // perpendicular distance to the nearest return on the left
  std::optional<double> right_m;  // ... and on the right (both positive)
};

LaneMeasurement measure_lane(const ScanInput& scan, double laser_yaw_offset_rad,
                             double lidar_mount_x_m, double lidar_mount_y_m, double x_min_m,
                             double x_max_m, double lateral_min_m, double lateral_max_m);

}  // namespace racer_control

#endif  // RACER_CONTROL_PARK_SLOT_HPP_
