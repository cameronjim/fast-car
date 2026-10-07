// L1 tests for the rear corridor and the rear obstacle gate (2026-10-06 night floor finding:
// bags 2026-10-06T23-19-41 and 23-25-38, the car nose-in to a corner tighter than its turning
// circle in both lap directions, with no way to back out). forward_sector.hpp "REAR CORRIDOR"
// and gate_logic.hpp "THE REAR OBSTACLE GATE" / "SPEED RATE LIMIT IN BOTH DIRECTIONS".
//
// The cases mirror the forward suites (test_forward_sector.cpp, test_arc_corridor.cpp,
// test_obstacle_gate.cpp): reversing into a wall brakes, reversing away is free, the arc is
// mirrored for left and right, mount yaw pi and 0 agree, plus the two-latch properties (the
// forward latch never blocks a reverse request and the rear latch never blocks a forward one).
//
// Inputs are plain fixtures in the shape of config/vehicle_params.yaml (schema 0.10.0), NOT
// read from it (racer_safety_core never includes the generated binding).
#include <gtest/gtest.h>

#include <algorithm>
#include <cmath>
#include <cstddef>
#include <limits>
#include <optional>
#include <string>
#include <vector>

#include "racer_safety/forward_sector.hpp"
#include "racer_safety/gate_logic.hpp"

namespace racer_safety {
namespace {

constexpr double kPi = 3.14159265358979323846;
constexpr double kNan = std::numeric_limits<double>::quiet_NaN();
constexpr double kInf = std::numeric_limits<double>::infinity();
constexpr float kFloatNan = std::numeric_limits<float>::quiet_NaN();

constexpr double kWheelbase = 0.3302;   // chassis.wheelbase_m
constexpr double kMaxSteer = 0.4189;    // steering.max_angle_rad
constexpr double kMountX = 0.285;       // sensors.lidar.mount_x_m
constexpr double kRearOverhang = 0.12;  // chassis.rear_overhang_m (PROVISIONAL)
constexpr double kHalfAngle = 1.2;      // limits.ttc_forward_sector_half_angle_rad
constexpr double kHalfWidth = 0.205;    // chassis.width_m / 2 + limits.obstacle_corridor_margin_m
constexpr double kCarYaw = kPi;         // sensors.lidar.mount_yaw_rad
// Rear axle to the front bumper line: chassis.wheelbase_m + chassis.front_overhang_m
// (PROVISIONAL, schema 0.11.0). Forward only; the rear corridor's leading edge is the rear
// bumper line.
constexpr double kBodyFrontX = kWheelbase + 0.13;
// The rear bumper line, in the head frame (x ahead of the head).
constexpr double kBumperX = -(kMountX + kRearOverhang);

PathGeometry car_path() {
  PathGeometry path;
  path.wheelbase_m = kWheelbase;
  path.max_steering_angle_rad = kMaxSteer;
  path.lidar_mount_x_m = kMountX;
  path.lidar_mount_y_m = 0.0;
  path.body_front_x_m = kBodyFrontX;
  return path;
}

// Head at the rear axle and 0.3 rad gives a 1 m radius exactly (L = tan(0.3)).
constexpr double kUnitSteer = 0.3;
PathGeometry unit_path() {
  PathGeometry path;
  path.wheelbase_m = std::tan(kUnitSteer);
  path.max_steering_angle_rad = 0.4;
  path.lidar_mount_x_m = 0.0;
  path.lidar_mount_y_m = 0.0;
  return path;
}

// A scan holding exactly one return at head-relative (x ahead, y left).
struct OneReturn {
  OneReturn(double x_m, double y_m, double yaw_rad)
      : ranges{static_cast<float>(std::hypot(x_m, y_m))} {
    geometry.angle_min_rad = std::atan2(y_m, x_m) - yaw_rad;
    geometry.angle_increment_rad = 0.01;
    geometry.range_min_m = 0.05;
    geometry.range_max_m = 12.0;
  }
  ScanGeometry geometry;
  std::vector<float> ranges;
};

double rear(const OneReturn& r, double steering_rad, const PathGeometry& path = car_path(),
            double overhang_m = kRearOverhang, double yaw_rad = kCarYaw,
            double half_angle_rad = kHalfAngle) {
  return min_rear_path_distance_m(r.geometry, r.ranges, yaw_rad, half_angle_rad, kHalfWidth, path,
                                  overhang_m, steering_rad);
}

double forward(const OneReturn& r, double steering_rad, const PathGeometry& path = car_path(),
               double yaw_rad = kCarYaw) {
  return min_path_distance_m(r.geometry, r.ranges, yaw_rad, kHalfAngle, kHalfWidth, path,
                             steering_rad);
}

// A point `phi` of arc angle round a turn of radius `radius_m`, `offset_m` radially outward,
// in the MIRRORED rear-axle frame (xm behind the rear axle, ym left; left turn when `left`).
struct Xy {
  double x;
  double y;
};
Xy on_mirrored_arc(double radius_m, double phi, double offset_m, bool left) {
  const double r = radius_m + offset_m;
  const double sign = left ? 1.0 : -1.0;
  return Xy{r * std::sin(phi), sign * (radius_m - r * std::cos(phi))};
}

// Head-relative (x, y) of a mirrored rear-axle point, for a path whose head sits `mount_x` ahead
// of the rear axle on the centreline.
Xy head_of_mirrored(const Xy& m, double mount_x) { return Xy{-m.x - mount_x, m.y}; }

// A C1-shaped scan (720 rays, car yaw pi) ray-cast from segments in the HEAD frame. Rays that
// hit nothing read 0.0 (the driver's "no return").
struct Segment {
  double x0, y0, x1, y1;
};
struct CastScan {
  explicit CastScan(const std::vector<Segment>& segments, double yaw_rad = kCarYaw)
      : yaw(yaw_rad), ranges(720, 0.0f) {
    geometry.angle_min_rad = -kPi - yaw_rad;
    geometry.angle_increment_rad = 2.0 * kPi / 720.0;
    geometry.range_min_m = 0.05;
    geometry.range_max_m = 12.0;
    for (std::size_t i = 0; i < ranges.size(); ++i) {
      const double bearing = -kPi + static_cast<double>(i) * geometry.angle_increment_rad;
      const double dx = std::cos(bearing);
      const double dy = std::sin(bearing);
      double best = kInf;
      for (const Segment& s : segments) {
        const double ex = s.x1 - s.x0;
        const double ey = s.y1 - s.y0;
        const double denom = dx * ey - dy * ex;
        if (std::abs(denom) < 1e-12) {
          continue;
        }
        const double t = (s.x0 * ey - s.y0 * ex) / denom;
        const double u = (s.x0 * dy - s.y0 * dx) / denom;
        if (t > 0.0 && u >= 0.0 && u <= 1.0) {
          best = std::min(best, t);
        }
      }
      ranges[i] = std::isfinite(best) ? static_cast<float>(best) : 0.0f;
    }
  }
  double rear_distance(double steering_rad) const {
    return min_rear_path_distance_m(geometry, ranges, yaw, kHalfAngle, kHalfWidth, car_path(),
                                    kRearOverhang, steering_rad);
  }
  double forward_distance(double steering_rad) const {
    return min_path_distance_m(geometry, ranges, yaw, kHalfAngle, kHalfWidth, car_path(),
                               steering_rad);
  }
  double yaw;
  ScanGeometry geometry;
  std::vector<float> ranges;
};

double full_lock_radius() { return kWheelbase / std::tan(kMaxSteer); }

// ---------------------------------------------------------------------------------------
// Rear corridor geometry (forward_sector.hpp "REAR CORRIDOR").
// ---------------------------------------------------------------------------------------

TEST(RearCorridorStraight, TheDistanceIsFromTheRearBumperLine) {
  // 0.30 m behind the bumper, on the centreline and at the corridor's edge.
  EXPECT_NEAR(rear(OneReturn(kBumperX - 0.30, 0.0, kCarYaw), 0.0), 0.30, 1e-6);
  EXPECT_NEAR(rear(OneReturn(kBumperX - 0.30, 0.20, kCarYaw), 0.0), 0.30, 1e-6);
  EXPECT_NEAR(rear(OneReturn(kBumperX - 0.30, -0.20, kCarYaw), 0.0), 0.30, 1e-6);
  // A wall across the path behind: every in-corridor return is 0.30 m behind the bumper.
  const CastScan wall({Segment{kBumperX - 0.30, -0.6, kBumperX - 0.30, 0.6}});
  EXPECT_NEAR(wall.rear_distance(0.0), 0.30, 1e-5);
}

TEST(RearCorridorStraight, JustOutsideTheCorridorIsIgnoredJustInsideIsCounted) {
  EXPECT_NEAR(rear(OneReturn(kBumperX - 0.5, kHalfWidth - 1e-4, kCarYaw), 0.0), 0.5, 1e-6);
  EXPECT_EQ(rear(OneReturn(kBumperX - 0.5, kHalfWidth + 1e-3, kCarYaw), 0.0), kInf);
  EXPECT_EQ(rear(OneReturn(kBumperX - 0.5, -kHalfWidth - 1e-3, kCarYaw), 0.0), kInf);
}

TEST(RearCorridorStraight, TheCarsOwnFootprintBehindTheHeadIsIgnored) {
  // Between the head and the bumper (the body, cables): never an obstacle.
  EXPECT_EQ(rear(OneReturn(-0.10, 0.0, kCarYaw), 0.0), kInf);
  EXPECT_EQ(rear(OneReturn(kBumperX + 0.005, 0.1, kCarYaw), 0.0), kInf);
  EXPECT_EQ(rear(OneReturn(-0.10, 0.0, kCarYaw), kMaxSteer), kInf);
  EXPECT_EQ(rear(OneReturn(-0.10, 0.0, kCarYaw), -kMaxSteer), kInf);
}

TEST(RearCorridorStraight, AheadIsIgnoredBehindAndTheForwardCorridorIgnoresBehind) {
  const OneReturn ahead(0.5, 0.0, kCarYaw);
  const OneReturn behind(kBumperX - 0.3, 0.0, kCarYaw);
  EXPECT_EQ(rear(ahead, 0.0), kInf);
  EXPECT_NEAR(forward(ahead, 0.0), 0.5, 1e-6);
  EXPECT_EQ(forward(behind, 0.0), kInf);
  EXPECT_NEAR(rear(behind, 0.0), 0.3, 1e-6);
}

TEST(RearCorridorStraight, OverhangZeroMeasuresFromTheRearAxle) {
  EXPECT_NEAR(rear(OneReturn(-kMountX - 0.5, 0.0, kCarYaw), 0.0, car_path(), 0.0), 0.5, 1e-6);
}

TEST(RearCorridorArc, TheDistanceIsTheReverseArcLengthFromTheBumperLine) {
  // Unit path (R = 1, head at the rear axle). A point 0.5 rad round the reverse-left arc.
  const Xy m = on_mirrored_arc(1.0, 0.5, 0.0, true);
  const Xy h = head_of_mirrored(m, 0.0);
  const OneReturn r(h.x, h.y, kCarYaw);
  EXPECT_NEAR(rear(r, kUnitSteer, unit_path(), 0.0), 0.5, 1e-6);
  EXPECT_NEAR(rear(r, kUnitSteer, unit_path(), 0.1), 0.5 - std::atan2(0.1, 1.0), 1e-6);
  // Never the straight-line range.
  EXPECT_GT(std::abs(rear(r, kUnitSteer, unit_path(), 0.0) - std::hypot(h.x, h.y)), 1e-3);
}

TEST(RearCorridorArc, ReversingLeftSwingsTheTailLeftAndRightIsTheMirror) {
  // Backing up with left lock, the rear axle moves back and to the LEFT (it rides the same
  // circle about (0, R) as driving forward, the other way round). A point on that path behind
  // and to the left is in the reverse-left path, and its mirror image is in the reverse-right
  // path at exactly the same distance.
  const double radius = full_lock_radius();
  const Xy m = on_mirrored_arc(radius, 0.9, 0.15, true);
  ASSERT_GT(m.y, 0.0);  // left of the centreline
  const Xy h = head_of_mirrored(m, kMountX);
  const double left = rear(OneReturn(h.x, h.y, kCarYaw), kMaxSteer);
  const double right = rear(OneReturn(h.x, -h.y, kCarYaw), -kMaxSteer);
  ASSERT_TRUE(std::isfinite(left));
  EXPECT_NEAR(left, right, 1e-6);
  EXPECT_NEAR(left, radius * (0.9 - std::atan2(kRearOverhang, radius)), 1e-5);
  // The other lock sweeps the other way: the same point is out of the path.
  EXPECT_EQ(rear(OneReturn(h.x, h.y, kCarYaw), -kMaxSteer), kInf);
  EXPECT_EQ(rear(OneReturn(h.x, -h.y, kCarYaw), kMaxSteer), kInf);
}

TEST(RearCorridorArc, APostStraightBehindIsOutOfThePathAtFullLock) {
  // 0.5 m behind the bumper on the centreline: in the straight reverse path, and outside the
  // swept band at full lock either way (rho - R = 0.225 m > 0.205 m).
  const OneReturn post(kBumperX - 0.5, 0.0, kCarYaw);
  EXPECT_NEAR(rear(post, 0.0), 0.5, 1e-6);
  EXPECT_EQ(rear(post, kMaxSteer), kInf);
  EXPECT_EQ(rear(post, -kMaxSteer), kInf);
}

TEST(RearCorridorArc, TheOuterEdgeIsTheOuterRearCornersSweep) {
  // Backing up, the rear bumper line leads, so the band's outer edge is the outer REAR corner's
  // sweep, hypot(rear_overhang_m, R + half width) (forward_sector.hpp "OUTER BOUNDARY"), not
  // R + half width and not the front corner's. At full lock: 0.954 m against 0.947 m.
  constexpr double kEps = 5e-4;
  const double radius = full_lock_radius();
  const double old_outer = radius + kHalfWidth;
  const double corner_outer = std::hypot(kRearOverhang, radius + kHalfWidth);
  ASSERT_NEAR(corner_outer, 0.9542, 1e-4);
  const double between = 0.5 * (old_outer + corner_outer);
  for (const bool left : {true, false}) {
    const double steer = left ? kMaxSteer : -kMaxSteer;
    // Inside the rear corner's sweep but outside the old band: in the path, at the reverse arc
    // length from the bumper line.
    const Xy in = head_of_mirrored(on_mirrored_arc(radius, 0.9, between - radius, left), kMountX);
    EXPECT_NEAR(rear(OneReturn(in.x, in.y, kCarYaw), steer),
                radius * (0.9 - std::atan2(kRearOverhang, radius)), 1e-5)
        << left;
    // With no overhang the leading corner is on the axle line: the old band, and it is out.
    EXPECT_EQ(rear(OneReturn(in.x, in.y, kCarYaw), steer, car_path(), 0.0), kInf) << left;
    // Just outside the rear corner's sweep: out.
    const Xy out =
        head_of_mirrored(on_mirrored_arc(radius, 0.9, corner_outer - radius + kEps, left), kMountX);
    EXPECT_EQ(rear(OneReturn(out.x, out.y, kCarYaw), steer), kInf) << left;
    // The FRONT bumper's x plays no part behind the car, however large.
    PathGeometry long_nose = car_path();
    long_nose.body_front_x_m = 2.0;
    EXPECT_EQ(rear(OneReturn(out.x, out.y, kCarYaw), steer, long_nose), kInf) << left;
    EXPECT_NEAR(rear(OneReturn(in.x, in.y, kCarYaw), steer, long_nose),
                rear(OneReturn(in.x, in.y, kCarYaw), steer), 0.0)
        << left;
  }
}

TEST(RearCorridorArc, BeyondAQuarterTurnAndAheadOfTheRearAxleAreIgnored) {
  const Xy past = head_of_mirrored(on_mirrored_arc(1.0, 1.7, 0.0, true), 0.0);
  EXPECT_EQ(rear(OneReturn(past.x, past.y, kCarYaw), kUnitSteer, unit_path(), 0.0), kInf);
  // Ahead of the rear axle (phi <= 0 in the mirrored frame): the car is driving away from it.
  const Xy ahead = head_of_mirrored(on_mirrored_arc(1.0, -0.3, 0.0, true), 0.0);
  EXPECT_EQ(rear(OneReturn(ahead.x, ahead.y, kCarYaw), kUnitSteer, unit_path(), 0.0), kInf);
}

TEST(RearCorridorArc, MountYawPiAndZeroAgree) {
  const Xy m = on_mirrored_arc(full_lock_radius(), 0.5, -0.05, false);
  const Xy h = head_of_mirrored(m, kMountX);
  const double yaw_pi = rear(OneReturn(h.x, h.y, kPi), -kMaxSteer, car_path(), kRearOverhang, kPi);
  const double yaw_0 = rear(OneReturn(h.x, h.y, 0.0), -kMaxSteer, car_path(), kRearOverhang, 0.0);
  ASSERT_TRUE(std::isfinite(yaw_pi));
  EXPECT_NEAR(yaw_pi, yaw_0, 1e-6);
  // A wall behind on a ray-cast scan, both yaws.
  const std::vector<Segment> wall{Segment{kBumperX - 0.35, -0.6, kBumperX - 0.35, 0.6}};
  EXPECT_NEAR(CastScan(wall, kPi).rear_distance(0.0), CastScan(wall, 0.0).rear_distance(0.0), 1e-5);
  EXPECT_NEAR(CastScan(wall, 0.0).rear_distance(0.0), 0.35, 1e-5);
}

TEST(RearCorridorArc, TheRearSectorBoundIsCentredBehindTheCar) {
  // With a 0.3 rad bound: a return 0.245 rad off the -x axis counts, one 0.32 rad off does
  // not, although both are inside the corridor's half width.
  EXPECT_TRUE(std::isfinite(
      rear(OneReturn(-0.8, 0.2, kCarYaw), 0.0, car_path(), kRearOverhang, kCarYaw, 0.3)));
  EXPECT_EQ(rear(OneReturn(-0.6, 0.2, kCarYaw), 0.0, car_path(), kRearOverhang, kCarYaw, 0.3),
            kInf);
}

TEST(RearCorridorArc, TakesTheNearestOfSeveralReturnsAndIgnoresInvalidOnes) {
  const CastScan two({Segment{kBumperX - 0.9, -0.6, kBumperX - 0.9, 0.6},
                      Segment{kBumperX - 0.4, -0.05, kBumperX - 0.4, 0.05}});
  EXPECT_NEAR(two.rear_distance(0.0), 0.4, 1e-5);
  CastScan bad = two;
  for (float& r : bad.ranges) {
    if (r > 0.0f && r < 1.0f) {
      r = kFloatNan;  // knock out the near post
    }
  }
  EXPECT_NEAR(bad.rear_distance(0.0), 0.9, 1e-5);
  // Nothing behind at all.
  EXPECT_EQ(CastScan({Segment{1.0, -0.6, 1.0, 0.6}}).rear_distance(0.0), kInf);
}

TEST(RearCorridorGarbage, GarbageInputFallsBackToTheWholeScanConservatively) {
  // One return ahead at 0.8 m and one behind; the fallback is the minimum slant range over
  // every usable return, wherever it is.
  const CastScan scene({Segment{0.8, -0.05, 0.8, 0.05}, Segment{-2.0, -0.05, -2.0, 0.05}});
  const double whole = 0.8;
  for (const double overhang : {kNan, -0.01, kInf}) {
    EXPECT_NEAR(min_rear_path_distance_m(scene.geometry, scene.ranges, kCarYaw, kHalfAngle,
                                         kHalfWidth, car_path(), overhang, 0.0),
                whole, 1e-5);
  }
  PathGeometry bad_path = car_path();
  bad_path.wheelbase_m = 0.0;
  EXPECT_NEAR(min_rear_path_distance_m(scene.geometry, scene.ranges, kCarYaw, kHalfAngle,
                                       kHalfWidth, bad_path, kRearOverhang, 0.0),
              whole, 1e-5);
  bad_path = car_path();
  bad_path.body_front_x_m = kNan;
  EXPECT_NEAR(min_rear_path_distance_m(scene.geometry, scene.ranges, kCarYaw, kHalfAngle,
                                       kHalfWidth, bad_path, kRearOverhang, 0.0),
              whole, 1e-5);
  EXPECT_NEAR(min_rear_path_distance_m(scene.geometry, scene.ranges, kCarYaw, kHalfAngle, kNan,
                                       car_path(), kRearOverhang, 0.0),
              whole, 1e-5);
  EXPECT_NEAR(min_rear_path_distance_m(scene.geometry, scene.ranges, kCarYaw, kHalfAngle,
                                       kHalfWidth, car_path(), kRearOverhang, kNan),
              whole, 1e-5);
  // The good inputs see only the return behind.
  EXPECT_NEAR(scene.rear_distance(0.0), 2.0 - kMountX - kRearOverhang, 1e-5);
}

// ---------------------------------------------------------------------------------------
// The rear obstacle gate (gate_logic.hpp "THE REAR OBSTACLE GATE").
// ---------------------------------------------------------------------------------------

constexpr double kDt = 0.02;  // 50 Hz

// The committed obstacle-gate values (schema 0.9.4 / 0.10.0).
SafetyLimits committed_limits() {
  SafetyLimits limits;
  limits.steering_min_rad = -kMaxSteer;
  limits.steering_max_rad = kMaxSteer;
  limits.steering_rate_min_rad_per_s = -3.2;
  limits.steering_rate_max_rad_per_s = 3.2;
  limits.speed_min_mps = -5.0;
  limits.speed_max_mps = 20.0;
  limits.max_acceleration_mps2 = 9.51;  // 0.19 m/s per cycle
  limits.ttc_warning_s = 0.75;          // the release line
  limits.ttc_brake_s = 0.6;
  limits.min_forward_clearance_m = 0.40;  // release above 0.60 m
  limits.obstacle_steering_hold_after_s = 0.5;
  limits.watchdog_missed_cycles = 3;
  limits.control_period_s = kDt;
  return limits;
}

bool has_activation(const GateResult& result, GateSource source, EventSeverity severity) {
  for (const auto& a : result.activations) {
    if (a.source == source && a.severity == severity) {
      return true;
    }
  }
  return false;
}

const GateActivation* find_activation(const GateResult& result, GateSource source,
                                      EventSeverity severity) {
  for (const auto& a : result.activations) {
    if (a.source == source && a.severity == severity) {
      return &a;
    }
  }
  return nullptr;
}

std::size_t count_records(const std::vector<SafetyEventRecord>& records, GateSource source,
                          EventSeverity severity, EventPhase phase) {
  std::size_t n = 0;
  for (const auto& r : records) {
    if (r.source == source && r.severity == severity && r.phase == phase) {
      ++n;
    }
  }
  return n;
}

// safety_node's cycle with explicit forward and rear distances; both latches, the hold timer
// and the event tracker threaded from cycle to cycle.
struct Loop {
  explicit Loop(SafetyLimits limits = committed_limits()) : gate(limits) {}

  GateResult step(double speed_mps, double forward_m, double rear_m, double steering_rad = 0.0) {
    GateInput input;
    input.command = DriveCommand{steering_rad, speed_mps};
    input.drive_raw_age_s = 0.0;
    input.dt_s = kDt;
    input.min_scan_range_m = forward_m;
    input.min_rear_scan_range_m = rear_m;
    input.ttc_brake_latched = latched;
    input.reverse_brake_latched = reverse_latched;
    input.obstacle_hold_timer_s = hold_timer_s;
    return finish(gate.evaluate(input, previous));
  }

  GateResult finish(GateResult result) {
    previous = result.output;
    latched = result.ttc_brake_latched;
    reverse_latched = result.reverse_brake_latched;
    hold_timer_s = result.obstacle_hold_timer_s;
    now_s += kDt;
    for (auto& r : tracker.update(result.activations, now_s, result.releases)) {
      records.push_back(r);
    }
    return result;
  }

  SafetyGateLogic gate;
  GateEventTracker tracker;
  DriveCommand previous{0.0, 0.0};
  bool latched = false;
  bool reverse_latched = false;
  std::optional<double> hold_timer_s;
  double now_s = 0.0;
  std::vector<SafetyEventRecord> records;
};

TEST(RearGate, ReversingIntoAWallInsideTheFloorBrakesAndLatches) {
  // -0.3 m/s at 0.25 m: TTC 0.83 s is past the brake threshold, so the floor is what trips.
  Loop loop;
  const GateResult r = loop.step(-0.3, kInf, 0.25);
  EXPECT_TRUE(r.reverse_brake_latched);
  EXPECT_FALSE(r.ttc_brake_latched);
  EXPECT_TRUE(r.zero_throttle);
  EXPECT_EQ(r.output.speed_mps, 0.0);
  const GateActivation* a = find_activation(r, GateSource::kTtcReverse, EventSeverity::kBrake);
  ASSERT_NE(a, nullptr);
  EXPECT_NE(a->detail.find("rear clearance"), std::string::npos);
  EXPECT_FALSE(has_activation(r, GateSource::kTtc, EventSeverity::kBrake));
  // Held on every following cycle, one engagement.
  for (int i = 0; i < 100; ++i) {
    EXPECT_EQ(loop.step(-0.3, kInf, 0.25).output.speed_mps, 0.0);
  }
  EXPECT_EQ(count_records(loop.records, GateSource::kTtcReverse, EventSeverity::kBrake,
                          EventPhase::kEngage),
            1u);
  EXPECT_EQ(
      count_records(loop.records, GateSource::kTtc, EventSeverity::kBrake, EventPhase::kEngage),
      0u);
}

TEST(RearGate, ReverseTtcOnTheRequestedSpeedTrips) {
  // 0.5 m behind (above the floor) at -1.0 m/s: TTC 0.5 s <= 0.6 s.
  Loop loop;
  const GateResult r = loop.step(-1.0, kInf, 0.5);
  EXPECT_TRUE(r.reverse_brake_latched);
  EXPECT_EQ(r.output.speed_mps, 0.0);
  const GateActivation* a = find_activation(r, GateSource::kTtcReverse, EventSeverity::kBrake);
  ASSERT_NE(a, nullptr);
  EXPECT_NE(a->detail.find("reverse time-to-collision"), std::string::npos);
}

TEST(RearGate, ReversingWithNothingBehindIsFreeAndRateLimited) {
  Loop loop;
  const GateResult first = loop.step(-0.5, 0.25, kInf);  // a wall ahead does not matter
  EXPECT_FALSE(first.reverse_brake_latched);
  EXPECT_FALSE(first.ttc_brake_latched);                    // not moving toward it
  EXPECT_NEAR(first.output.speed_mps, -9.51 * kDt, 1e-12);  // reverse growth is rate-limited
  GateResult last = first;
  for (int i = 0; i < 10; ++i) {
    last = loop.step(-0.5, 0.25, kInf);
  }
  EXPECT_DOUBLE_EQ(last.output.speed_mps, -0.5);
  EXPECT_EQ(count_records(loop.records, GateSource::kTtcReverse, EventSeverity::kBrake,
                          EventPhase::kEngage),
            0u);
}

TEST(RearGate, TheForwardLatchNeverBlocksAReverseRequest) {
  // Parked nose-in: the forward latch is set and the wall ahead is inside the floor. Backing
  // out with the rear clear passes (rate-limited); the forward latch stays until the forward
  // clearance is back.
  Loop loop;
  ASSERT_TRUE(loop.step(0.5, 0.25, kInf).ttc_brake_latched);
  for (int i = 0; i < 10; ++i) {
    const GateResult r = loop.step(-0.5, 0.25, 2.0);
    EXPECT_TRUE(r.ttc_brake_latched);
    EXPECT_FALSE(r.reverse_brake_latched);
    EXPECT_LT(r.output.speed_mps, 0.0);
  }
  EXPECT_DOUBLE_EQ(loop.previous.speed_mps, -0.5);
}

TEST(RearGate, TheRearLatchNeverBlocksAForwardRequest) {
  Loop loop;
  ASSERT_TRUE(loop.step(-0.5, kInf, 0.25).reverse_brake_latched);
  const GateResult r = loop.step(0.5, 2.0, 0.25);
  EXPECT_NEAR(r.output.speed_mps, 9.51 * kDt, 1e-12);
  EXPECT_FALSE(r.ttc_brake_latched);
  // The rear latch holds on its floor half while the wall behind is inside the release
  // clearance (0.60 m), and keeps being one engagement.
  EXPECT_TRUE(r.reverse_brake_latched);
  EXPECT_TRUE(has_activation(r, GateSource::kTtcReverse, EventSeverity::kBrake));
}

TEST(RearGate, ReleasesOnlyPastTheHysteresisWithOneReleaseRecord) {
  Loop loop;
  ASSERT_TRUE(loop.step(-0.5, kInf, 0.25).reverse_brake_latched);
  // 0.55 m: TTC 1.1 s is past the release line, but the clearance is not past 0.60 m.
  EXPECT_TRUE(loop.step(-0.5, kInf, 0.55).reverse_brake_latched);
  EXPECT_EQ(loop.previous.speed_mps, 0.0);
  // 0.65 m at -1.0 m/s: TTC 0.65 s is under the 0.75 s release line.
  EXPECT_TRUE(loop.step(-1.0, kInf, 0.65).reverse_brake_latched);
  // 0.65 m at -0.5 m/s: TTC 1.3 s and clearance 0.65 m: released, the speed ramps.
  const GateResult released = loop.step(-0.5, kInf, 0.65);
  EXPECT_FALSE(released.reverse_brake_latched);
  EXPECT_NEAR(released.output.speed_mps, -9.51 * kDt, 1e-12);
  ASSERT_EQ(released.releases.size(), 1u);
  EXPECT_EQ(released.releases[0].source, GateSource::kTtcReverse);
  EXPECT_NE(released.releases[0].detail.find("reverse ttc brake released"), std::string::npos);
  EXPECT_EQ(count_records(loop.records, GateSource::kTtcReverse, EventSeverity::kBrake,
                          EventPhase::kEngage),
            1u);
  EXPECT_EQ(count_records(loop.records, GateSource::kTtcReverse, EventSeverity::kBrake,
                          EventPhase::kRelease),
            1u);
}

TEST(RearGate, AReverseBrakeFromSpeedReachesTheOutputOnTheSameCycle) {
  Loop loop;
  for (int i = 0; i < 10; ++i) {
    loop.step(-0.5, kInf, 3.0);
  }
  ASSERT_DOUBLE_EQ(loop.previous.speed_mps, -0.5);
  const GateResult braked = loop.step(-0.5, kInf, 0.25);
  EXPECT_EQ(braked.output.speed_mps, 0.0);
  EXPECT_FALSE(has_activation(braked, GateSource::kRateLimit, EventSeverity::kWarning));
}

TEST(RearGate, BoxedInFrontAndBackIsTwoSeparateEngagements) {
  Loop loop;
  ASSERT_TRUE(loop.step(0.5, 0.25, 0.25).ttc_brake_latched);
  const GateResult both = loop.step(-0.5, 0.25, 0.25);
  EXPECT_TRUE(both.ttc_brake_latched);
  EXPECT_TRUE(both.reverse_brake_latched);
  EXPECT_EQ(both.output.speed_mps, 0.0);
  EXPECT_TRUE(has_activation(both, GateSource::kTtc, EventSeverity::kBrake));
  EXPECT_TRUE(has_activation(both, GateSource::kTtcReverse, EventSeverity::kBrake));
  // The obstacle ahead goes away: the forward latch releases (its own record) while the rear
  // one stays engaged.
  const GateResult front_clear = loop.step(0.5, 3.0, 0.25);
  EXPECT_FALSE(front_clear.ttc_brake_latched);
  EXPECT_TRUE(front_clear.reverse_brake_latched);
  EXPECT_GT(front_clear.output.speed_mps, 0.0);
  EXPECT_EQ(
      count_records(loop.records, GateSource::kTtc, EventSeverity::kBrake, EventPhase::kEngage),
      1u);
  EXPECT_EQ(
      count_records(loop.records, GateSource::kTtc, EventSeverity::kBrake, EventPhase::kRelease),
      1u);
  EXPECT_EQ(count_records(loop.records, GateSource::kTtcReverse, EventSeverity::kBrake,
                          EventPhase::kEngage),
            1u);
  EXPECT_EQ(count_records(loop.records, GateSource::kTtcReverse, EventSeverity::kBrake,
                          EventPhase::kRelease),
            0u);
}

TEST(RearGate, ShortCircuitsHoldTheRearLatchAndKeepReportingIt) {
  SafetyGateLogic gate(committed_limits());
  GateInput stale;
  stale.command = DriveCommand{0.0, -0.5};
  stale.drive_raw_age_s = 1.0;  // watchdog
  stale.dt_s = kDt;
  stale.min_rear_scan_range_m = 3.0;  // clear, but the gate cannot judge a release here
  stale.reverse_brake_latched = true;
  const GateResult watchdog = gate.evaluate(stale, DriveCommand{0.0, 0.0});
  EXPECT_TRUE(watchdog.reverse_brake_latched);
  EXPECT_TRUE(has_activation(watchdog, GateSource::kTtcReverse, EventSeverity::kBrake));
  EXPECT_TRUE(has_activation(watchdog, GateSource::kWatchdog, EventSeverity::kBrake));

  GateInput garbage = stale;
  garbage.drive_raw_age_s = 0.0;
  garbage.command = DriveCommand{kNan, -0.5};  // command sanity
  const GateResult sanity = gate.evaluate(garbage, DriveCommand{0.0, 0.0});
  EXPECT_TRUE(sanity.reverse_brake_latched);
  EXPECT_TRUE(has_activation(sanity, GateSource::kTtcReverse, EventSeverity::kBrake));
  EXPECT_FALSE(sanity.ttc_brake_latched);
  EXPECT_FALSE(has_activation(sanity, GateSource::kTtc, EventSeverity::kBrake));
}

TEST(RearGate, GarbageRearRangeNeverTripsAndNeverReleases) {
  SafetyGateLogic gate(committed_limits());
  for (const double garbage : {kNan, 0.0, -1.0, -kInf}) {
    GateInput input;
    input.command = DriveCommand{0.0, -0.5};
    input.dt_s = 1e6;  // keep the rate limiter out of it
    input.min_rear_scan_range_m = garbage;
    const GateResult fresh = gate.evaluate(input, DriveCommand{});
    EXPECT_FALSE(fresh.reverse_brake_latched) << garbage;
    EXPECT_DOUBLE_EQ(fresh.output.speed_mps, -0.5) << garbage;
    input.reverse_brake_latched = true;
    const GateResult held = gate.evaluate(input, DriveCommand{});
    EXPECT_TRUE(held.reverse_brake_latched) << garbage;
    EXPECT_EQ(held.output.speed_mps, 0.0) << garbage;
  }
}

TEST(RearGate, TheReverseWarningZoneIsAdvisoryOnly) {
  // 0.5 m behind at -0.7 m/s: TTC 0.714 s, past the 0.6 s brake, inside the 0.75 s warning.
  SafetyGateLogic gate(committed_limits());
  GateInput input;
  input.command = DriveCommand{0.0, -0.7};
  input.dt_s = 1e6;
  input.min_rear_scan_range_m = 0.5;
  const GateResult r = gate.evaluate(input, DriveCommand{});
  EXPECT_FALSE(r.reverse_brake_latched);
  EXPECT_DOUBLE_EQ(r.output.speed_mps, -0.7);
  const GateActivation* a = find_activation(r, GateSource::kTtcReverse, EventSeverity::kInfo);
  ASSERT_NE(a, nullptr);
  EXPECT_NE(a->detail.find("reverse time-to-collision"), std::string::npos);
  EXPECT_FALSE(has_activation(r, GateSource::kTtc, EventSeverity::kInfo));
}

TEST(RearGate, TheSteeringHoldStaysTiedToTheForwardLatch) {
  // Parked by the rear latch alone for 2 s: no hold, the steering follows the request.
  Loop loop;
  for (int i = 0; i < 100; ++i) {
    const GateResult r = loop.step(-0.5, kInf, 0.25, 0.2);
    ASSERT_EQ(r.output.speed_mps, 0.0);
    EXPECT_FALSE(r.obstacle_hold_timer_s.has_value());
    EXPECT_FALSE(has_activation(r, GateSource::kTtc, EventSeverity::kInfo));
  }
  EXPECT_DOUBLE_EQ(loop.previous.steering_angle_rad, 0.2);
}

// ---------------------------------------------------------------------------------------
// The speed rate limit in both directions (gate_logic.hpp "SPEED RATE LIMIT IN BOTH
// DIRECTIONS"). dt 0.1 s: max change 0.951 m/s.
// ---------------------------------------------------------------------------------------

GateResult rate_step(double previous_speed_mps, double request_mps) {
  SafetyGateLogic gate(committed_limits());
  GateInput input;
  input.command = DriveCommand{0.0, request_mps};
  input.dt_s = 0.1;
  input.min_scan_range_m = kInf;
  return gate.evaluate(input, DriveCommand{0.0, previous_speed_mps});
}

TEST(RateLimitBothDirections, ReverseAccelerationFromRestIsLimited) {
  const GateResult r = rate_step(0.0, -5.0);
  EXPECT_NEAR(r.output.speed_mps, -0.951, 1e-12);
  EXPECT_TRUE(has_activation(r, GateSource::kRateLimit, EventSeverity::kWarning));
  const GateResult marginal = rate_step(0.0, -0.951);
  EXPECT_NEAR(marginal.output.speed_mps, -0.951, 1e-12);
  EXPECT_FALSE(has_activation(marginal, GateSource::kRateLimit, EventSeverity::kWarning));
  const GateResult growing = rate_step(-1.0, -3.0);
  EXPECT_NEAR(growing.output.speed_mps, -1.951, 1e-12);
}

TEST(RateLimitBothDirections, BrakingInReverseIsNeverLimited) {
  for (const double request : {-0.5, 0.0}) {
    const GateResult r = rate_step(-2.0, request);
    EXPECT_DOUBLE_EQ(r.output.speed_mps, request);
    EXPECT_FALSE(has_activation(r, GateSource::kRateLimit, EventSeverity::kWarning));
  }
}

TEST(RateLimitBothDirections, CrossingZeroBrakesFreeThenGrowsFromZero) {
  EXPECT_NEAR(rate_step(-1.0, 5.0).output.speed_mps, 0.951, 1e-12);
  EXPECT_NEAR(rate_step(1.0, -5.0).output.speed_mps, -0.951, 1e-12);
  EXPECT_NEAR(rate_step(2.0, -0.5).output.speed_mps, -0.5, 1e-12);
  EXPECT_NEAR(rate_step(-2.0, 0.5).output.speed_mps, 0.5, 1e-12);
}

TEST(RateLimitBothDirections, ForwardIsUnchanged) {
  EXPECT_NEAR(rate_step(0.0, 5.0).output.speed_mps, 0.951, 1e-12);
  EXPECT_NEAR(rate_step(1.0, 5.0).output.speed_mps, 1.951, 1e-12);
  EXPECT_DOUBLE_EQ(rate_step(3.0, 1.0).output.speed_mps, 1.0);
  EXPECT_DOUBLE_EQ(rate_step(3.0, 0.0).output.speed_mps, 0.0);
  EXPECT_DOUBLE_EQ(rate_step(0.2, 0.5).output.speed_mps, 0.2 + (0.5 - 0.2));
}

TEST(RearGate, SourceStringIsTtcReverse) {
  EXPECT_EQ(gate_source_to_string(GateSource::kTtcReverse), "ttc_reverse");
  EXPECT_EQ(kGateSourceCount, 8u);
}

// ---------------------------------------------------------------------------------------
// Composed the way safety_node composes it: the last scan reduced every cycle along the
// request's arc, forward and rear.
// ---------------------------------------------------------------------------------------

struct NodeLoop {
  GateResult step(const CastScan& scan, double steering_rad, double speed_mps) {
    return loop.step(speed_mps, scan.forward_distance(steering_rad),
                     scan.rear_distance(steering_rad), steering_rad);
  }
  Loop loop;
};

TEST(RearCorridorThroughTheGate, NoseInThenBackOutUntilTheWallBehind) {
  // The night's trap, schematically: a wall 0.25 m ahead of the head (forward latch) and a
  // wall 0.8 m behind the bumper. Backing out passes until the rear clearance would drop
  // under the floor; with the car not actually moving in this L1 loop, the walls are moved
  // by hand to stand in for the motion.
  NodeLoop n;
  const std::vector<Segment> ahead{Segment{0.25, -0.6, 0.25, 0.6}};
  ASSERT_TRUE(
      n.step(CastScan({ahead[0], Segment{kBumperX - 0.8, -0.6, kBumperX - 0.8, 0.6}}), 0.0, 0.5)
          .ttc_brake_latched);
  // Reverse at full lock away from the corner: passes (the forward latch never blocks it).
  const GateResult backing = n.step(
      CastScan({ahead[0], Segment{kBumperX - 0.8, -0.6, kBumperX - 0.8, 0.6}}), -kMaxSteer, -0.5);
  EXPECT_LT(backing.output.speed_mps, 0.0);
  EXPECT_FALSE(backing.reverse_brake_latched);
  // The car has backed up: the wall behind is now 0.3 m from the bumper, inside the floor.
  const GateResult stopped = n.step(
      CastScan({Segment{0.5, -0.6, 0.5, 0.6}, Segment{kBumperX - 0.3, -0.8, kBumperX - 0.3, 0.8}}),
      -kMaxSteer, -0.5);
  EXPECT_TRUE(stopped.reverse_brake_latched);
  EXPECT_EQ(stopped.output.speed_mps, 0.0);
  // And forward again is allowed by the rear latch (the forward latch decides on its own).
  const GateResult forward_again = n.step(
      CastScan({Segment{1.5, -0.6, 1.5, 0.6}, Segment{kBumperX - 0.3, -0.8, kBumperX - 0.3, 0.8}}),
      0.0, 0.5);
  EXPECT_GT(forward_again.output.speed_mps, 0.0);
  EXPECT_FALSE(forward_again.ttc_brake_latched);
}

}  // namespace
}  // namespace racer_safety
