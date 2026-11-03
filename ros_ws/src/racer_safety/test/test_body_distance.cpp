// L1 tests for "DISTANCE FROM THE BODY" (forward_sector.hpp, 2026-11-02, schema 0.13.0): with
// PathGeometry::distance_from_body the in-path distance is measured from the swept body's leading
// edge (front bumper / leading corner forward, rear bumper / leading corner in reverse) instead of
// the LiDAR head. Also the floor-bag geometry through the gate, composed the way safety_node
// composes it. docs/notes/safety-floor-review-2026-11-02.md has the bag evidence.
//
// Inputs are plain fixtures in the shape of config/vehicle_params.yaml, NOT read from it
// (racer_safety_core never includes the generated binding), same pattern as the other suites.
#include <gtest/gtest.h>

#include <cmath>
#include <cstddef>
#include <limits>
#include <optional>
#include <vector>

#include "racer_safety/forward_sector.hpp"
#include "racer_safety/gate_logic.hpp"

namespace racer_safety {
namespace {

constexpr double kPi = 3.14159265358979323846;
constexpr double kInf = std::numeric_limits<double>::infinity();

// Shaped like the committed config (schema 0.13.0).
constexpr double kWheelbase = 0.3302;
constexpr double kMaxSteer = 0.4189;
constexpr double kMountX = 0.285;
constexpr double kHalfAngle = 1.2;
constexpr double kHalfWidth = 0.205;               // 0.31 / 2 + 0.05 margin
constexpr double kCarYaw = kPi;                    // sensors.lidar.mount_yaw_rad
constexpr double kBodyFrontX = kWheelbase + 0.13;  // wheelbase + PROVISIONAL front_overhang_m
constexpr double kRearOverhang = 0.12;             // PROVISIONAL rear_overhang_m
constexpr double kHorizon = 0.45;

PathGeometry car_path(bool from_body) {
  PathGeometry path;
  path.wheelbase_m = kWheelbase;
  path.max_steering_angle_rad = kMaxSteer;
  path.lidar_mount_x_m = kMountX;
  path.lidar_mount_y_m = 0.0;
  path.body_front_x_m = kBodyFrontX;
  path.outer_corner_horizon_m = kHorizon;
  path.distance_from_body = from_body;
  return path;
}

// Head at the rear axle and 0.3 rad giving a 1 m radius exactly (L = tan 0.3): hand geometry.
constexpr double kUnitSteer = 0.3;
constexpr double kUnitBodyX = 0.5;
constexpr double kUnitHalfWidth = 0.2;
PathGeometry unit_path(bool from_body) {
  PathGeometry path;
  path.wheelbase_m = std::tan(kUnitSteer);
  path.max_steering_angle_rad = 0.4;
  path.body_front_x_m = kUnitBodyX;
  path.distance_from_body = from_body;
  return path;
}

// A scan holding exactly one return at head-relative (x ahead, y left).
struct OneReturn {
  OneReturn(double x_m, double y_m, double yaw_rad = kCarYaw)
      : ranges{static_cast<float>(std::hypot(x_m, y_m))} {
    geometry.angle_min_rad = std::atan2(y_m, x_m) - yaw_rad;
    geometry.angle_increment_rad = 0.01;
    geometry.range_min_m = 0.05;
    geometry.range_max_m = 12.0;
  }
  ScanGeometry geometry;
  std::vector<float> ranges;
};

double forward(const OneReturn& r, double steer, const PathGeometry& path,
               double half_width_m = kHalfWidth, double yaw_rad = kCarYaw) {
  return min_path_distance_m(r.geometry, r.ranges, yaw_rad, kHalfAngle, half_width_m, path, steer);
}

double rear(const OneReturn& r, double steer, const PathGeometry& path,
            double half_width_m = kHalfWidth) {
  return min_rear_path_distance_m(r.geometry, r.ranges, kCarYaw, kHalfAngle, half_width_m, path,
                                  kRearOverhang, steer);
}

// A point at arc angle phi and radius rho about the centre of a LEFT turn of radius R (rear-axle
// frame): x = rho sin(phi), y = R - rho cos(phi). Mirrored for a right turn.
struct Xy {
  double x;
  double y;
};
Xy polar(double radius_m, double rho_m, double phi, bool left) {
  const double sign = left ? 1.0 : -1.0;
  return Xy{rho_m * std::sin(phi), sign * (radius_m - rho_m * std::cos(phi))};
}

// ---------------------------------------------------------------------------------------
// Straight requests.
// ---------------------------------------------------------------------------------------

TEST(BodyDistanceStraight, MeasuredFromTheFrontBumperLine) {
  // 0.5 m ahead of the head is 0.5 + 0.285 - 0.4602 = 0.3248 m ahead of the bumper.
  const OneReturn r(0.5, 0.05);
  EXPECT_NEAR(forward(r, 0.0, car_path(false)), 0.5, 1e-6);
  EXPECT_NEAR(forward(r, 0.0, car_path(true)), 0.5 + kMountX - kBodyFrontX, 1e-6);
}

TEST(BodyDistanceStraight, AReturnBetweenTheHeadAndTheBumperIsInsideTheCarAndIgnored) {
  // 0.10 m ahead of the head is 0.075 m behind the bumper: inside the footprint. The legacy head
  // reference counted it as an obstacle 0.10 m away.
  const OneReturn r(0.10, 0.0);
  EXPECT_NEAR(forward(r, 0.0, car_path(false)), 0.10, 1e-6);
  EXPECT_EQ(forward(r, 0.0, car_path(true)), kInf);
}

TEST(BodyDistanceStraight, ExactlyAtTheBumperIsNotAhead) {
  // Built to land on the bumper line within float rounding of the range: either side is fine,
  // but it must never come out negative.
  const OneReturn r(kBodyFrontX - kMountX, 0.0);
  const double d = forward(r, 0.0, car_path(true));
  EXPECT_TRUE(d == kInf || (d > 0.0 && d < 1e-6)) << d;
}

TEST(BodyDistanceStraight, TheCorridorWidthIsUnchanged) {
  EXPECT_EQ(forward(OneReturn(0.6, 0.21), 0.0, car_path(true)), kInf);
  EXPECT_LT(forward(OneReturn(0.6, 0.20), 0.0, car_path(true)), kInf);
}

// ---------------------------------------------------------------------------------------
// Arc requests: the leading point of the body at the return's radius.
// ---------------------------------------------------------------------------------------

TEST(BodyDistanceArc, OnTheRearAxlePathTheBumperLeads) {
  // R = 1, body x 0.5, half width 0.2: at rho = 1 the inner flank's circle (radius 0.8) is at
  // x = 0.6, past the bumper, so the bumper line leads at x = 0.5, phi_lead = asin(0.5).
  for (const bool left : {true, false}) {
    const Xy p = polar(1.0, 1.0, 1.0, left);
    const OneReturn r(p.x, p.y);
    const double steer = left ? kUnitSteer : -kUnitSteer;
    EXPECT_NEAR(forward(r, steer, unit_path(true), kUnitHalfWidth), 1.0 - std::asin(0.5), 1e-6);
    // The legacy head reference (the head is at the rear axle here): R phi.
    EXPECT_NEAR(forward(r, steer, unit_path(false), kUnitHalfWidth), 1.0, 1e-6);
  }
}

TEST(BodyDistanceArc, TowardTheInsideTheInnerFlankLeads) {
  // rho = 0.85: the inner flank (radius 0.8) reaches it at x = sqrt(0.85^2 - 0.8^2) = 0.287,
  // before the bumper line, so that point leads: phi_lead = asin(0.287 / 0.85).
  const double flank_x = std::sqrt(0.85 * 0.85 - 0.8 * 0.8);
  const double phi_lead = std::asin(flank_x / 0.85);
  const Xy p = polar(1.0, 0.85, 0.9, true);
  const OneReturn r(p.x, p.y);
  EXPECT_NEAR(forward(r, kUnitSteer, unit_path(true), kUnitHalfWidth), 0.9 - phi_lead, 1e-6);
}

TEST(BodyDistanceArc, InTheOuterCornerBandTheOuterFrontCornerLeads) {
  // rho = 1.25 is beyond the body band (1.2) and inside the outer corner's sweep, hypot(0.5,
  // 1.2) = 1.3: the bumper line leads, phi_lead = asin(0.5 / 1.25). Within the horizon.
  PathGeometry path = unit_path(true);
  path.outer_corner_horizon_m = 0.45;
  const double phi = std::asin(0.5 / 1.25) + 0.3;
  const Xy p = polar(1.0, 1.25, phi, true);
  EXPECT_NEAR(forward(OneReturn(p.x, p.y), kUnitSteer, path, kUnitHalfWidth), 0.3, 1e-6);
  // The horizon is tested against the same (body) distance: 0.3 m beyond a 0.25 m horizon.
  path.outer_corner_horizon_m = 0.25;
  EXPECT_EQ(forward(OneReturn(p.x, p.y), kUnitSteer, path, kUnitHalfWidth), kInf);
}

TEST(BodyDistanceArc, AReturnTheLeadingEdgeHasPassedIsNeverReached) {
  // Beside the body (behind its leading edge at that radius): a rigid car on this arc never
  // reaches it, so it does not count. The head reference counted it (it is ahead of the head).
  const double phi = std::asin(0.5 / 1.25) - 0.05;  // just behind the outer front corner
  const Xy p = polar(1.0, 1.25, phi, true);
  const OneReturn r(p.x, p.y);
  EXPECT_EQ(forward(r, kUnitSteer, unit_path(true), kUnitHalfWidth), kInf);
  EXPECT_GT(forward(r, kUnitSteer, unit_path(false), kUnitHalfWidth), 0.0);
}

TEST(BodyDistanceArc, ATurnTighterThanTheHalfWidthKeepsTheInnerRadiusAtZero) {
  // A garbage-but-finite geometry the car cannot have (R 0.1 m, half width 0.2 m): the inner
  // radius clamps at 0 and the result stays finite and non-negative.
  PathGeometry path = unit_path(true);
  path.wheelbase_m = 0.1 * std::tan(kUnitSteer);
  const Xy p = polar(0.1, 0.15, 0.8, true);
  const double d = forward(OneReturn(p.x, p.y), kUnitSteer, path, kUnitHalfWidth);
  EXPECT_TRUE(d == kInf || (std::isfinite(d) && d > 0.0)) << d;
}

TEST(BodyDistanceArc, LeftAndRightAreMirrorImages) {
  const PathGeometry path = car_path(true);
  for (const double x : {0.25, 0.4, 0.6, 0.8}) {
    for (const double y : {0.05, 0.15, 0.3}) {
      for (const double steer : {0.1, 0.25, kMaxSteer}) {
        const double left = forward(OneReturn(x, y), steer, path);
        const double right = forward(OneReturn(x, -y), -steer, path);
        EXPECT_EQ(std::isinf(left), std::isinf(right)) << x << " " << y << " " << steer;
        if (std::isfinite(left)) {
          EXPECT_NEAR(left, right, 1e-6);
        }
      }
    }
  }
}

TEST(BodyDistanceArc, OnAndOutsideTheRearAxlePathNeverFartherThanTheHeadDistance) {
  // Straight ahead and on the outer half of the band the bumper / outer corner leads the head,
  // so the body distance is the smaller: the new rule can only judge such an obstacle closer.
  const PathGeometry head = car_path(false);
  const PathGeometry body = car_path(true);
  std::size_t compared = 0;
  for (double x = 0.05; x < 1.5; x += 0.05) {
    for (double y = -0.6; y <= 0.6; y += 0.05) {
      for (const double steer : {-kMaxSteer, -0.2, -0.05, 0.0, 0.05, 0.2, kMaxSteer}) {
        if (steer != 0.0) {
          const double radius = kWheelbase / std::tan(steer);
          if (std::hypot(x + kMountX, y - radius) < std::abs(radius)) {
            continue;  // toward the inside: next test
          }
        }
        const double d_head = forward(OneReturn(x, y), steer, head);
        const double d_body = forward(OneReturn(x, y), steer, body);
        if (std::isfinite(d_body) && std::isfinite(d_head)) {
          ++compared;
          EXPECT_LE(d_body, d_head + 1e-9) << x << " " << y << " " << steer;
        }
      }
    }
  }
  EXPECT_GT(compared, 100u);
}

TEST(BodyDistanceArc, TowardTheInsideTheFlankCanTrailTheHeadAndThenTheBodyDistanceIsLarger) {
  // Stated, not hidden: a return just inside the band's inner edge is reached only by the inner
  // flank close to the rear axle, which trails the head's arc angle. The legacy reference used
  // the head's angle at every radius and so read it closer than any part of the body can reach
  // it; the body distance is the true one. Steering 0.2 rad (R = 1.63 m), a return 1 cm inside
  // the band's inner edge, 0.6 rad round.
  const double radius = kWheelbase / std::tan(0.2);
  const double inner = radius - kHalfWidth;
  const Xy p = polar(radius, inner + 0.01, 0.6, true);
  const OneReturn r(p.x - kMountX, p.y);
  const double flank_x = std::sqrt((inner + 0.01) * (inner + 0.01) - inner * inner);
  const double phi_lead = std::atan2(flank_x, inner);
  EXPECT_NEAR(forward(r, 0.2, car_path(true)), radius * (0.6 - phi_lead), 1e-6);
  EXPECT_LT(forward(r, 0.2, car_path(false)), forward(r, 0.2, car_path(true)));
}

TEST(BodyDistanceArc, AtFullLockTheOuterCornerLeadsTheHeadByAboutEightCentimetres) {
  // The review's number: a return on the outer corner's own circle, 0.3 m of arc round, is
  // about 0.08 m closer measured from the corner than from the head.
  const double radius = kWheelbase / std::tan(kMaxSteer);
  const double rho = std::hypot(kBodyFrontX, radius + 0.155);  // the actual outer corner's circle
  const double phi_corner = std::asin(kBodyFrontX / rho);
  const Xy p = polar(radius, rho, phi_corner + 0.3 / radius, true);
  const OneReturn r(p.x - kMountX, p.y);
  const double d_body = forward(r, kMaxSteer, car_path(true));
  const double d_head = forward(r, kMaxSteer, car_path(false));
  EXPECT_NEAR(d_body, 0.3, 1e-6);
  EXPECT_NEAR(d_head - d_body, 0.08, 0.01);
}

// ---------------------------------------------------------------------------------------
// Reverse: the rear bumper line's leading point.
// ---------------------------------------------------------------------------------------

TEST(BodyDistanceRear, StraightIsUnchangedFromTheBumperLine) {
  // 0.4 m behind the rear axle (head coordinates x = -(0.4 + 0.285)) is 0.28 m behind the
  // rear bumper line, with either reference.
  const OneReturn r(-(0.4 + kMountX), 0.0);
  EXPECT_NEAR(rear(r, 0.0, car_path(false)), 0.4 - kRearOverhang, 1e-6);
  EXPECT_NEAR(rear(r, 0.0, car_path(true)), 0.4 - kRearOverhang, 1e-6);
}

TEST(BodyDistanceRear, JustBehindTheOuterRearCornerIsMeasuredFromThatCorner) {
  // Backing round a full-lock arc: a return just behind the OUTER rear corner, at the corner's
  // own radius. Measured from the bumper's centre it reads as already passed (the corner sits at
  // a smaller arc angle than the centre at that radius) and is ignored; from the body it is 2 cm
  // of arc away.
  const double radius = kWheelbase / std::tan(kMaxSteer);
  const double rho = std::hypot(kRearOverhang, radius + 0.2);
  const double phi = std::asin(kRearOverhang / rho) + 0.02 / radius;
  // Mirrored rear frame (xm behind the axle, ym left): a left-steered reverse arc.
  const Xy pm = polar(radius, rho, phi, true);
  const OneReturn r(-(pm.x + kMountX), pm.y);
  EXPECT_EQ(rear(r, kMaxSteer, car_path(false)), kInf);
  EXPECT_NEAR(rear(r, kMaxSteer, car_path(true)), 0.02, 1e-6);
}

// ---------------------------------------------------------------------------------------
// The floor bags, through the gate.
// ---------------------------------------------------------------------------------------

constexpr double kDt = 0.02;

SafetyLimits limits_for(double ttc_brake_s, double ttc_warning_s, double floor_m) {
  SafetyLimits limits;
  limits.steering_min_rad = -kMaxSteer;
  limits.steering_max_rad = kMaxSteer;
  limits.steering_rate_min_rad_per_s = -3.2;
  limits.steering_rate_max_rad_per_s = 3.2;
  limits.speed_min_mps = -5.0;
  limits.speed_max_mps = 20.0;
  limits.max_acceleration_mps2 = 9.51;
  limits.ttc_brake_s = ttc_brake_s;
  limits.ttc_warning_s = ttc_warning_s;
  limits.min_forward_clearance_m = floor_m;
  limits.speed_bound_deceleration_mps2 = 1.0;
  limits.watchdog_missed_cycles = 3;
  limits.control_period_s = kDt;
  return limits;
}

// A straight wall across the path `bumper_gap_m` ahead of the bumper, seen from the head.
OneReturn wall_ahead(double bumper_gap_m) {
  return OneReturn(kBodyFrontX - kMountX + bumper_gap_m, 0.0);
}

// The first bumper gap, approaching at `speed` in 1 mm steps (one gate cycle each, so the
// steady-state output equals the request), at which the gate trips.
double trip_gap(const SafetyLimits& limits, const PathGeometry& path, double speed) {
  const SafetyGateLogic gate(limits);
  DriveCommand previous{0.0, speed};
  bool latched = false;
  double bound_f = speed;
  double bound_r = 0.0;
  for (double gap = 1.0; gap > 0.0; gap -= 0.001) {
    GateInput input;
    input.command = DriveCommand{0.0, speed};
    input.dt_s = kDt;
    input.min_scan_range_m = forward(wall_ahead(gap), 0.0, path);
    input.ttc_brake_latched = latched;
    input.speed_bound_forward_mps = bound_f;
    input.speed_bound_reverse_mps = bound_r;
    const GateResult result = gate.evaluate(input, previous);
    if (result.ttc_brake_latched) {
      return gap;
    }
    previous = result.output;
    bound_f = result.speed_bound_forward_mps;
    bound_r = result.speed_bound_reverse_mps;
  }
  return 0.0;
}

TEST(BodyDistanceFloorBags, TheOldRuleLeftLessThanTheStoppingTravelStraightAhead) {
  // Schema 0.12.0 on the head distance at 0.8 m/s: the 0.25 m floor (above TTC 0.3 x 0.8 =
  // 0.24 m) trips 0.25 m from the head, 0.075 m from the bumper. The measured stopping travel at
  // that request was about 0.15 m (median, LiDAR scan matching): a straight approach could not
  // stop clear.
  const double gap = trip_gap(limits_for(0.3, 0.45, 0.25), car_path(false), 0.8);
  EXPECT_NEAR(gap, 0.25 - (kBodyFrontX - kMountX), 0.002);
}

TEST(BodyDistanceFloorBags, TheCommittedRuleTripsAtTheTtcDistanceFromTheBumper) {
  // Schema 0.13.0 on the body distance at 0.8 m/s: 0.34 x 0.8 = 0.272 m off the bumper.
  const double gap = trip_gap(limits_for(0.34, 0.45, 0.05), car_path(true), 0.8);
  EXPECT_NEAR(gap, 0.272, 0.002);
}

TEST(BodyDistanceFloorBags, AtCrawlSpeedTheFloorIsMeasuredFromTheBumperToo) {
  // 0.1 m/s: TTC wants only 0.034 m, the 0.05 m floor trips first, 0.05 m off the bumper (the
  // old 0.25 m floor from the head was 0.075 m off the bumper).
  EXPECT_NEAR(trip_gap(limits_for(0.34, 0.45, 0.05), car_path(true), 0.1), 0.05, 0.002);
  EXPECT_NEAR(trip_gap(limits_for(0.3, 0.45, 0.25), car_path(false), 0.1),
              0.25 - (kBodyFrontX - kMountX), 0.002);
}

}  // namespace
}  // namespace racer_safety
