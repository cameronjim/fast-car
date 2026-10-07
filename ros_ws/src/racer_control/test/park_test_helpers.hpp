// Test helpers for the park_node core suites (test_park_slot.cpp, test_park_planner.cpp,
// test_park_controller.cpp): the committed car's geometry and a 360 degree synthetic LiDAR that
// ray-casts line segments.
//
// THE CAR. The numbers below are the committed config/vehicle_params.yaml values (schema 0.12.0)
// typed into the TEST fixture on purpose, like test_gap_follow.cpp's "real C1 geometry": the
// hand-computed expectations in test_park_planner.cpp are derived from them, so a change to the
// params file must come with a deliberate change here and to those expectations. The node itself
// only ever reads them from the generated binding (CLAUDE.md invariant 2).
//
// THE LIDAR. Like the RPLIDAR C1 on the car: 360 degrees at 0.72 degree steps (500 rays,
// sensors.lidar_spec.angular_resolution_rad), angle_min -pi, mounted facing BACKWARDS
// (sensors.lidar.mount_yaw_rad pi, vehicle bearing = laser bearing + pi) at mount_x 0.285 m ahead
// of the rear axle on the centreline. Rays that hit nothing within range_max are +inf, the
// driver's "no return".
#ifndef RACER_CONTROL_TEST_PARK_TEST_HELPERS_HPP_
#define RACER_CONTROL_TEST_PARK_TEST_HELPERS_HPP_

#include <cmath>
#include <limits>
#include <ostream>
#include <vector>

#include "racer_control/laser_scan.hpp"
#include "racer_control/park_controller.hpp"
#include "racer_control/park_geometry.hpp"
#include "racer_control/park_planner.hpp"
#include "racer_control/park_slot.hpp"

namespace racer_control {

// Readable gtest failure messages.
inline std::ostream& operator<<(std::ostream& os, ParkPhase phase) {
  return os << park_phase_name(phase);
}
inline std::ostream& operator<<(std::ostream& os, SlotReject reject) {
  return os << slot_reject_name(reject);
}
inline std::ostream& operator<<(std::ostream& os, PlanReject reject) {
  return os << plan_reject_name(reject);
}

namespace park_test {

constexpr double kWheelbase = 0.3302;          // chassis.wheelbase_m
constexpr double kWidth = 0.31;                // chassis.width_m
constexpr double kLength = 0.58;               // chassis.length_m
constexpr double kFrontOverhang = 0.13;        // chassis.front_overhang_m (PROVISIONAL)
constexpr double kRearOverhang = 0.12;         // chassis.rear_overhang_m (PROVISIONAL)
constexpr double kMaxSteering = 0.4189;        // steering.max_angle_rad
constexpr double kMaxSteeringRate = 3.2;       // steering.max_rate_rad_per_s
constexpr double kMaxAcceleration = 9.51;      // actuation.max_acceleration_mps2
constexpr double kLidarX = 0.285;              // sensors.lidar.mount_x_m
constexpr double kLidarY = 0.0;                // sensors.lidar.mount_y_m
constexpr double kLidarYaw = M_PI;             // sensors.lidar.mount_yaw_rad (3.141593)
constexpr double kLidarResolution = 0.012566;  // sensors.lidar_spec.angular_resolution_rad
constexpr double kRangeMin = 0.05;             // sensors.lidar_spec.range_min_m
constexpr double kRangeMax = 12.0;             // sensors.lidar_spec.range_max_m
constexpr double kCorridorMargin = 0.05;       // limits.obstacle_corridor_margin_m
constexpr double kSectorHalfAngle = 1.2;       // limits.ttc_forward_sector_half_angle_rad
constexpr double kOuterCornerHorizon = 0.45;   // limits.outer_corner_horizon_m
constexpr double kFloor = 0.25;                // limits.min_forward_clearance_m
constexpr double kTtcBrake = 0.3;              // limits.ttc_brake_s

// The floor-2026-10-07 profile's tuning (park.launch.py).
constexpr double kMargin = 0.1;
constexpr double kSteeringFraction = 0.95;
constexpr double kSearchSpeed = 0.5;
constexpr double kParkSpeed = -0.5;
constexpr double kSettle = 0.3;
constexpr double kGateMargin = 0.03;

inline ParkBody car_body() {
  ParkBody b;
  b.wheelbase_m = kWheelbase;
  b.half_width_m = kWidth / 2.0;
  b.front_x_m = kWheelbase + kFrontOverhang;
  b.rear_x_m = kRearOverhang;
  b.lidar_x_m = kLidarX;
  b.lidar_y_m = kLidarY;
  return b;
}

inline ParkGateModel car_gate() {
  ParkGateModel g;
  g.enabled = true;
  g.corridor_half_width_m = kWidth / 2.0 + kCorridorMargin;
  g.sector_half_angle_rad = kSectorHalfAngle;
  g.outer_corner_horizon_m = kOuterCornerHorizon;
  g.max_steering_rad = kMaxSteering;
  g.floor_m = kFloor;
  g.ttc_brake_s = kTtcBrake;
  g.margin_m = kGateMargin;
  return g;
}

inline ParkPlannerConfig car_planner() {
  ParkPlannerConfig c;
  c.body = car_body();
  c.max_steering_rad = kMaxSteering;
  c.steering_fraction = kSteeringFraction;
  c.margin_m = kMargin;
  c.final_forward_max_m = 0.15;
  c.forward_speed_mps = kSearchSpeed;
  c.reverse_speed_mps = -kParkSpeed;
  c.gate = car_gate();
  return c;
}

inline SlotDetectorConfig car_detector(ParkSide side) {
  SlotDetectorConfig d;
  d.side = side;
  d.laser_yaw_offset_rad = kLidarYaw;
  d.lidar_mount_x_m = kLidarX;
  d.lidar_mount_y_m = kLidarY;
  d.lateral_min_m = kWidth / 2.0;
  d.lateral_max_m = 2.0;
  d.window_back_m = 1.5;
  d.window_ahead_m = 2.5;
  d.bin_m = 0.05;
  d.jump_min_m = 0.1;
  d.depth_min_m = kWidth + 2.0 * kMargin;  // slot_depth_min_m default
  d.length_min_m = kLength + 0.35;         // slot_length_min_m default
  return d;
}

inline ParkControllerConfig car_controller(ParkSide side) {
  ParkControllerConfig c;
  c.detector = car_detector(side);
  c.planner = car_planner();
  c.search_speed_mps = kSearchSpeed;
  c.park_speed_mps = kParkSpeed;
  c.settle_s = kSettle;
  c.steering_rate_rad_per_s = 0.35 * kMaxSteeringRate;  // park_node defaults
  c.acceleration_mps2 = 0.25 * kMaxAcceleration;
  c.require_odometry = true;
  c.synchronous_planning = true;
  return c;
}

// Ray-segment intersection distance along a unit ray from (ox, oy); +inf when none.
inline double ray_hit(double ox, double oy, double dx, double dy, const Segment2& s) {
  const double ex = s.b.x - s.a.x;
  const double ey = s.b.y - s.a.y;
  const double denom = dx * ey - dy * ex;
  if (std::abs(denom) < 1e-12) {
    return std::numeric_limits<double>::infinity();
  }
  const double wx = s.a.x - ox;
  const double wy = s.a.y - oy;
  const double t = (wx * ey - wy * ex) / denom;
  const double u = (wx * dy - wy * dx) / denom;
  if (t > 0.0 && u >= 0.0 && u <= 1.0) {
    return t;
  }
  return std::numeric_limits<double>::infinity();
}

// The car's 360 degree scan of `world` with the rear axle at `pose`.
inline ScanInput synthetic_scan(const Pose2& pose, const std::vector<Segment2>& world) {
  ScanInput scan;
  const std::size_t n = static_cast<std::size_t>(std::lround(2.0 * M_PI / kLidarResolution));
  scan.angle_min = -M_PI;
  scan.angle_increment = 2.0 * M_PI / static_cast<double>(n);
  scan.range_min = kRangeMin;
  scan.range_max = kRangeMax;
  scan.ranges.resize(n);
  const Point2 head = to_world(pose, {kLidarX, kLidarY});
  for (std::size_t i = 0; i < n; ++i) {
    const double laser = scan.angle_min + static_cast<double>(i) * scan.angle_increment;
    const double world_bearing = pose.yaw + laser + kLidarYaw;
    const double dx = std::cos(world_bearing);
    const double dy = std::sin(world_bearing);
    double best = std::numeric_limits<double>::infinity();
    for (const Segment2& s : world) {
      best = std::min(best, ray_hit(head.x, head.y, dx, dy, s));
    }
    scan.ranges[i] =
        best > kRangeMax ? std::numeric_limits<float>::infinity() : static_cast<float>(best);
  }
  return scan;
}

// A row of parked obstacles along y = side * row_lateral with a pocket from near_x to far_x,
// depth deep, plus an optional wall on the other side. Same segment layout as
// parallel_park_obstacles, extended 4 m each way.
inline std::vector<Segment2> pocket_world(ParkSide side, double row_lateral, double near_x,
                                          double far_x, double depth,
                                          double opposite_wall_lateral = 0.0) {
  ParkSlot slot;
  slot.side = side;
  const double s = side_sign(side);
  slot.near_x_m = near_x;
  slot.far_x_m = far_x;
  slot.row_y_m = s * row_lateral;
  slot.back_y_m = s * (row_lateral + depth);
  if (opposite_wall_lateral > 0.0) {
    slot.far_wall_y_m = -s * opposite_wall_lateral;
  }
  return parallel_park_obstacles(slot, 4.0);
}

}  // namespace park_test
}  // namespace racer_control

#endif  // RACER_CONTROL_TEST_PARK_TEST_HELPERS_HPP_
