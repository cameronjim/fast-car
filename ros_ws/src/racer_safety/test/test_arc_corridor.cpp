// L1 tests for the arc corridor (forward_sector.hpp "ARC CORRIDOR", 2026-10-06 late floor
// test): the in-path test follows the REQUESTED steering arc, so a car latched on a wall
// releases when it is asked to steer away and the arc is clear. Also the bag reproduction
// (2026-10-06T22-12-40_car_teleop) through the gate, composed the way safety_node composes it:
// the last scan reduced on every gate cycle with that cycle's requested steering.
//
// Inputs are plain fixtures in the shape of config/vehicle_params.yaml, NOT read from it
// (racer_safety_core never includes the generated binding), same pattern as the other suites.
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
constexpr float kFloatInf = std::numeric_limits<float>::infinity();

// Shaped like the committed config (schema 0.9.2).
constexpr double kWheelbase = 0.3302;  // chassis.wheelbase_m
constexpr double kMaxSteer = 0.4189;   // steering.max_angle_rad
constexpr double kMountX = 0.285;      // sensors.lidar.mount_x_m
constexpr double kHalfAngle = 1.2;     // limits.ttc_forward_sector_half_angle_rad
constexpr double kHalfWidth = 0.205;   // chassis.width_m / 2 + limits.obstacle_corridor_margin_m
constexpr double kCarYaw = kPi;        // sensors.lidar.mount_yaw_rad
// Rear axle to the front bumper line: chassis.wheelbase_m + chassis.front_overhang_m
// (PROVISIONAL 0.13 m, schema 0.11.0).
constexpr double kBodyFrontX = kWheelbase + 0.13;

PathGeometry car_path() {
  PathGeometry path;
  path.wheelbase_m = kWheelbase;
  path.max_steering_angle_rad = kMaxSteer;
  path.lidar_mount_x_m = kMountX;
  path.lidar_mount_y_m = 0.0;
  path.body_front_x_m = kBodyFrontX;
  return path;
}

// A path model whose geometry is easy to compute by hand: head at the rear axle, and the
// request 0.3 rad gives a 1 m radius exactly (L = tan(0.3)).
constexpr double kUnitSteer = 0.3;
PathGeometry unit_path() {
  PathGeometry path;
  path.wheelbase_m = std::tan(kUnitSteer);
  path.max_steering_angle_rad = 0.4;
  path.lidar_mount_x_m = 0.0;
  path.lidar_mount_y_m = 0.0;
  return path;
}

// A scan holding exactly one return at head-relative (x ahead, y left), with angle_min chosen
// so ray 0 lands exactly on its vehicle bearing after `yaw_rad`.
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

double path_distance(const OneReturn& r, double steering_rad, const PathGeometry& path,
                     double yaw_rad = kCarYaw, double half_angle_rad = kHalfAngle,
                     double half_width_m = kHalfWidth) {
  return min_path_distance_m(r.geometry, r.ranges, yaw_rad, half_angle_rad, half_width_m, path,
                             steering_rad);
}

// A point `phi` of arc angle round a turn of radius `radius_m` from the rear axle, `offset_m`
// radially outward from the rear-axle path (left turn when `left`). Rear-axle frame.
struct Xy {
  double x;
  double y;
};
Xy on_arc(double radius_m, double phi, double offset_m, bool left) {
  const double r = radius_m + offset_m;
  const double sign = left ? 1.0 : -1.0;
  return Xy{r * std::sin(phi), sign * (radius_m - r * std::cos(phi))};
}

// A C1-shaped scan (720 rays over a full turn, car yaw pi) built by ray-casting line segments
// given in the HEAD frame (x ahead, y left). `first_vehicle_bearing_rad` places ray 0 exactly.
struct Segment {
  double x0, y0, x1, y1;
};
struct CastScan {
  explicit CastScan(const std::vector<Segment>& segments, double first_vehicle_bearing_rad = -kPi)
      : ranges(720, 0.0f) {
    geometry.angle_min_rad = first_vehicle_bearing_rad - kCarYaw;
    geometry.angle_increment_rad = 2.0 * kPi / 720.0;
    geometry.range_min_m = 0.05;
    geometry.range_max_m = 12.0;
    for (std::size_t i = 0; i < ranges.size(); ++i) {
      const double bearing =
          first_vehicle_bearing_rad + static_cast<double>(i) * geometry.angle_increment_rad;
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
        const double t = (s.x0 * ey - s.y0 * ex) / denom;  // along the ray
        const double u = (s.x0 * dy - s.y0 * dx) / denom;  // along the segment
        if (t > 0.0 && u >= 0.0 && u <= 1.0) {
          best = std::min(best, t);
        }
      }
      ranges[i] = std::isfinite(best) ? static_cast<float>(best) : 0.0f;  // 0.0: no return
    }
  }
  double distance(double steering_rad) const {
    return min_path_distance_m(geometry, ranges, kCarYaw, kHalfAngle, kHalfWidth, car_path(),
                               steering_rad);
  }
  ScanGeometry geometry;
  std::vector<float> ranges;
};

double full_lock_radius() { return kWheelbase / std::tan(kMaxSteer); }

// The floor-test wall: 0.33 m ahead of the head, from 2 cm right of the centreline out to
// 0.6 m on the right. Straight ahead it is in the corridor; full lock right (into it) it is
// too. Full lock LEFT (away from it) cleared it with the pre-0.11.0 band, but the car's outer
// (right) front corner sweeps 1.008 m from the turn centre, and the wall's near end, 2 to 5.7
// cm right of the centreline, is inside that circle: steering away, the corner would clip it.
// Since the band's outer edge is that corner's sweep (forward_sector.hpp "OUTER BOUNDARY") the
// near end is in the full-lock-left path too.
std::vector<Segment> bag_wall() { return {Segment{0.33, -0.02, 0.33, -0.60}}; }

// The same wall with its near end 15 cm right of the centreline: still in the straight
// corridor (half width 0.205 m) at 0.33 m, and clear of the outer front corner's sweep with
// the margin (1.053 m) at full lock left, its nearest point 1.083 m from the turn centre.
std::vector<Segment> clear_wall() { return {Segment{0.33, -0.15, 0.33, -0.60}}; }

// ---------------------------------------------------------------------------------------
// Straight request: the arc corridor reduces to the straight corridor.
// ---------------------------------------------------------------------------------------

TEST(ArcCorridorStraight, AStraightRequestEqualsTheStraightCorridor) {
  std::vector<Segment> scene = bag_wall();
  scene.push_back(Segment{0.8, 0.5, 0.8, 0.1});     // ahead-left, partly in the corridor
  scene.push_back(Segment{-0.3, 0.4, -0.3, -0.4});  // behind the head
  const CastScan scan(scene);
  const double reference =
      min_corridor_distance_m(scan.geometry, scan.ranges, kCarYaw, kHalfAngle, kHalfWidth);
  EXPECT_NEAR(reference, 0.33, 1e-6);
  for (const double steer : {0.0, 5e-4, -5e-4, kStraightSteeringEpsilonRad * 0.999}) {
    EXPECT_EQ(scan.distance(steer), reference) << "steering " << steer;
  }
}

TEST(ArcCorridorStraight, AlmostStraightArcsTendToTheStraightDistance) {
  // Just past the epsilon the arc model is used; its distance must be the straight one to
  // within the arc's sag over half a metre (R = 165 m: well under a millimetre).
  for (const double steer : {2e-3, -2e-3}) {
    const OneReturn r(0.5, 0.05, kCarYaw);
    EXPECT_NEAR(path_distance(r, steer, car_path()), 0.5, 1e-3) << steer;
  }
}

TEST(ArcCorridorStraight, StraightCorridorIsCentredOnTheCentrelineNotTheHead) {
  // A head mounted 5 cm left of the centreline: a return 0.17 m left of the head is 0.22 m
  // left of the centreline (out), one 0.24 m right of the head is 0.19 m right of it (in).
  PathGeometry path = car_path();
  path.lidar_mount_y_m = 0.05;
  EXPECT_EQ(path_distance(OneReturn(0.5, 0.17, kCarYaw), 0.0, path), kInf);
  EXPECT_NEAR(path_distance(OneReturn(0.5, -0.24, kCarYaw), 0.0, path), 0.5, 1e-6);
}

// ---------------------------------------------------------------------------------------
// The arc: geometry, distance, turn direction, quarter turn, behind, margin.
// ---------------------------------------------------------------------------------------

TEST(ArcCorridorGeometry, TheDistanceIsTheArcLengthNotTheStraightLineRange) {
  // Head at the rear axle, R = 1 m. A return on the path one radian round the turn is 1.0 m
  // of arc away; its straight-line range is the chord, 2 sin(0.5) = 0.959 m.
  for (const bool left : {true, false}) {
    const Xy p = on_arc(1.0, 1.0, 0.0, left);
    const OneReturn r(p.x, p.y, kCarYaw);
    const double steer = left ? kUnitSteer : -kUnitSteer;
    EXPECT_NEAR(path_distance(r, steer, unit_path()), 1.0, 1e-6);
    EXPECT_NEAR(static_cast<double>(r.ranges[0]), 2.0 * std::sin(0.5), 1e-6);
  }
}

TEST(ArcCorridorGeometry, TheArcIsMeasuredFromTheHeadLikeTheStraightCorridor) {
  // Head 0.285 m ahead of the rear axle (the car). A return on the full-lock arc at arc angle
  // phi is R (phi - phi_head) from the head, phi_head = atan2(0.285, R).
  const double radius = full_lock_radius();
  const double phi_head = std::atan2(kMountX, radius);
  for (const bool left : {true, false}) {
    const Xy p = on_arc(radius, 0.9, 0.0, left);
    const OneReturn r(p.x - kMountX, p.y, kCarYaw);
    const double steer = left ? kMaxSteer : -kMaxSteer;
    EXPECT_NEAR(path_distance(r, steer, car_path()), radius * (0.9 - phi_head), 1e-6);
  }
}

TEST(ArcCorridorGeometry, LeftAndRightTurnsAreMirrorImages) {
  const PathGeometry path = car_path();
  for (const double x : {0.15, 0.3, 0.5}) {
    for (const double y : {0.05, 0.15, 0.3}) {
      for (const double steer : {0.1, 0.25, kMaxSteer}) {
        const double left = path_distance(OneReturn(x, y, kCarYaw), steer, path);
        const double right = path_distance(OneReturn(x, -y, kCarYaw), -steer, path);
        EXPECT_EQ(std::isinf(left), std::isinf(right)) << x << " " << y << " " << steer;
        if (std::isfinite(left)) {
          EXPECT_NEAR(left, right, 1e-6);
        }
      }
    }
  }
}

TEST(ArcCorridorGeometry, AWallStraightAheadIsOutOfThePathAtFullLockAwayFromIt) {
  // In the straight corridor at 0.33 m, gone from the full-lock-left arc once the outer front
  // corner clears its near end.
  const CastScan scan(clear_wall());
  EXPECT_NEAR(scan.distance(0.0), 0.33, 1e-6);
  EXPECT_EQ(scan.distance(kMaxSteer), kInf);
  // A request beyond the lock is clamped to it, so it sees the same path.
  EXPECT_EQ(scan.distance(1.0), kInf);
}

TEST(ArcCorridorGeometry, TheBagWallsNearEndIsInTheFullLockAwayPathBecauseTheCornerWouldClipIt) {
  // The bag geometry itself: the right front corner, steering left, sweeps through the wall's
  // near end, so it is in the path, closer than the straight-ahead 0.33 m. With the pre-0.11.0
  // band it was not.
  const CastScan scan(bag_wall());
  const double left = scan.distance(kMaxSteer);
  EXPECT_TRUE(std::isfinite(left));
  EXPECT_GT(left, 0.0);
  EXPECT_LT(left, 0.33);
  PathGeometry old_band = car_path();
  old_band.body_front_x_m = 0.0;
  EXPECT_EQ(min_path_distance_m(scan.geometry, scan.ranges, kCarYaw, kHalfAngle, kHalfWidth,
                                old_band, kMaxSteer),
            kInf);
}

TEST(ArcCorridorGeometry, AWallOnTheInsideOfTheTurnIsInThePath) {
  // Same wall, full lock RIGHT: the wall is on the inside of the turn and the car would drive
  // into it, sooner than going straight (the arc reaches the near edge of the wall first).
  const CastScan scan(bag_wall());
  const double right = scan.distance(-kMaxSteer);
  EXPECT_TRUE(std::isfinite(right));
  EXPECT_GT(right, 0.0);
  EXPECT_LT(right, 0.33);
  // A request beyond the lock the other way is clamped too.
  EXPECT_EQ(scan.distance(-1.0), right);
  // Mirror image: a wall on the left, full lock left.
  const CastScan left_scan({Segment{0.33, 0.02, 0.33, 0.60}});
  EXPECT_NEAR(left_scan.distance(kMaxSteer), right, 2e-3);  // 720-ray sampling
  // Full lock right, away from it: its near end is inside the outer (left) front corner's
  // sweep (see bag_wall), the clear wall's mirror image is not.
  EXPECT_NEAR(left_scan.distance(-kMaxSteer), scan.distance(kMaxSteer), 2e-3);
  EXPECT_EQ(CastScan({Segment{0.33, 0.15, 0.33, 0.60}}).distance(-kMaxSteer), kInf);
}

TEST(ArcCorridorGeometry, BeyondAQuarterTurnIsIgnored) {
  // R = 1 m, head at the rear axle, a sector wide enough to see round the turn.
  for (const bool left : {true, false}) {
    const double steer = left ? kUnitSteer : -kUnitSteer;
    const Xy inside = on_arc(1.0, 1.5, 0.0, left);  // 1.5 rad < pi/2: counts
    EXPECT_NEAR(
        path_distance(OneReturn(inside.x, inside.y, kCarYaw), steer, unit_path(), kCarYaw, kPi),
        1.5, 1e-6);
    const Xy beyond = on_arc(1.0, 1.65, 0.0, left);  // past a quarter turn: ignored
    EXPECT_EQ(
        path_distance(OneReturn(beyond.x, beyond.y, kCarYaw), steer, unit_path(), kCarYaw, kPi),
        kInf);
  }
}

TEST(ArcCorridorGeometry, BehindTheCarIsIgnored) {
  // With a sector of pi the outer bound lets everything through; the arc angle keeps out a
  // return behind the rear axle that sits right on the swept band.
  const double radius = full_lock_radius();
  for (const bool left : {true, false}) {
    const double steer = left ? kMaxSteer : -kMaxSteer;
    const Xy behind = on_arc(radius, -0.3, 0.0, left);  // rear-axle frame, behind the axle
    EXPECT_EQ(path_distance(OneReturn(behind.x - kMountX, behind.y, kCarYaw), steer, car_path(),
                            kCarYaw, kPi),
              kInf);
    // Ahead of the rear axle but not yet at the head along the arc (beside the car): ignored,
    // as x <= 0 is in the straight corridor.
    const Xy beside = on_arc(radius, 0.2, 0.0, left);
    EXPECT_EQ(path_distance(OneReturn(beside.x - kMountX, beside.y, kCarYaw), steer, car_path(),
                            kCarYaw, kPi),
              kInf);
  }
}

TEST(ArcCorridorGeometry, TheSectorBoundStillApplies) {
  // On the R = 1 m path at 1.5 rad the return is at bearing 0.75 rad from the head: a 0.7 rad
  // sector drops it.
  const Xy p = on_arc(1.0, 1.5, 0.0, true);
  EXPECT_EQ(path_distance(OneReturn(p.x, p.y, kCarYaw), kUnitSteer, unit_path(), kCarYaw, 0.7),
            kInf);
}

TEST(ArcCorridorGeometry, TheCorridorMarginWidensTheSweptBand) {
  // R = 1 m. Just inside / just outside the band on the outer and inner edges.
  constexpr double kEps = 1e-3;
  for (const double margin : {0.0, 0.05, 0.10}) {
    const double half_width = corridor_half_width_m(0.31, margin);
    for (const double edge : {half_width, -half_width}) {
      const double in_offset = edge > 0.0 ? edge - kEps : edge + kEps;
      const double out_offset = edge > 0.0 ? edge + kEps : edge - kEps;
      const Xy in = on_arc(1.0, 0.8, in_offset, true);
      const Xy out = on_arc(1.0, 0.8, out_offset, true);
      EXPECT_NEAR(path_distance(OneReturn(in.x, in.y, kCarYaw), kUnitSteer, unit_path(), kCarYaw,
                                kHalfAngle, half_width),
                  0.8, 1e-6)
          << margin << " " << edge;
      EXPECT_EQ(path_distance(OneReturn(out.x, out.y, kCarYaw), kUnitSteer, unit_path(), kCarYaw,
                              kHalfAngle, half_width),
                kInf)
          << margin << " " << edge;
    }
  }
}

TEST(ArcCorridorGeometry, MountYawPiAndZeroAgree) {
  // The same head-frame return seen through the car's yaw (pi) and the sim's (0).
  const double radius = full_lock_radius();
  const Xy p = on_arc(radius, 0.8, 0.05, true);
  const double expected = radius * (0.8 - std::atan2(kMountX, radius));
  for (const double yaw : {kCarYaw, 0.0}) {
    EXPECT_NEAR(path_distance(OneReturn(p.x - kMountX, p.y, yaw), kMaxSteer, car_path(), yaw),
                expected, 1e-6)
        << yaw;
  }
  // And a real-car scan read with the wrong yaw looks the other way: the arc ahead is empty.
  const OneReturn ahead(p.x - kMountX, p.y, kCarYaw);
  EXPECT_EQ(path_distance(ahead, kMaxSteer, car_path(), 0.0), kInf);
}

TEST(ArcCorridorGeometry, TakesTheNearestOfSeveralReturnsOnThePath) {
  const double radius = full_lock_radius();
  std::vector<Segment> scene;
  for (const double phi : {1.2, 0.7, 1.0}) {
    const Xy p = on_arc(radius, phi, 0.0, true);
    scene.push_back(Segment{p.x - kMountX - 0.01, p.y, p.x - kMountX + 0.01, p.y});
  }
  const CastScan scan(scene);
  const double expected = radius * (0.7 - std::atan2(kMountX, radius));
  EXPECT_NEAR(scan.distance(kMaxSteer), expected, 0.02);
}

// ---------------------------------------------------------------------------------------
// The outer boundary: the outer front corner's sweep (forward_sector.hpp "OUTER BOUNDARY",
// 2026-10-06 night sim escape scenario on a round corner).
// ---------------------------------------------------------------------------------------

// The band's edges at full lock with the committed geometry.
double old_outer_radius() { return full_lock_radius() + kHalfWidth; }
double corner_outer_radius() { return std::hypot(kBodyFrontX, full_lock_radius() + kHalfWidth); }

TEST(ArcCorridorOuterCorner, TheCommittedNumbers) {
  // R = 0.742 m at full lock. The old band ended at R + 0.205 = 0.947 m; the outer front
  // corner sweeps hypot(0.4602, 0.897) = 1.008 m without the margin and 1.053 m with it.
  EXPECT_NEAR(full_lock_radius(), 0.7416, 1e-4);
  EXPECT_NEAR(old_outer_radius(), 0.9466, 1e-4);
  EXPECT_NEAR(std::hypot(kBodyFrontX, full_lock_radius() + 0.155), 1.0078, 1e-4);
  EXPECT_NEAR(corner_outer_radius(), 1.0525, 1e-4);
}

TEST(ArcCorridorOuterCorner, AReturnInsideTheCornerSweepButOutsideTheOldBandIsNowInThePath) {
  // The round-corner wall: 1.0 m from the turn centre at full lock, 5 cm outside the old band
  // and inside the corner's sweep, at arc angle 0.8 rad. It is in the path, at the usual arc
  // length from the head; with the pre-0.11.0 band (body_front_x 0) it was not.
  const double radius = full_lock_radius();
  const double phi_head = std::atan2(kMountX, radius);
  for (const bool left : {true, false}) {
    const double steer = left ? kMaxSteer : -kMaxSteer;
    const Xy p = on_arc(radius, 0.8, 1.0 - radius, left);
    const OneReturn r(p.x - kMountX, p.y, kCarYaw);
    EXPECT_NEAR(path_distance(r, steer, car_path()), radius * (0.8 - phi_head), 1e-6) << left;
    PathGeometry old_band = car_path();
    old_band.body_front_x_m = 0.0;
    EXPECT_EQ(path_distance(r, steer, old_band), kInf) << left;
  }
}

TEST(ArcCorridorOuterCorner, TheOuterEdgeIsExactlyTheCornerSweepWithTheMargin) {
  constexpr double kEps = 1e-3;
  const double radius = full_lock_radius();
  for (const bool left : {true, false}) {
    const double steer = left ? kMaxSteer : -kMaxSteer;
    const Xy in = on_arc(radius, 0.9, corner_outer_radius() - radius - kEps, left);
    const Xy out = on_arc(radius, 0.9, corner_outer_radius() - radius + kEps, left);
    EXPECT_TRUE(
        std::isfinite(path_distance(OneReturn(in.x - kMountX, in.y, kCarYaw), steer, car_path())))
        << left;
    EXPECT_EQ(path_distance(OneReturn(out.x - kMountX, out.y, kCarYaw), steer, car_path()), kInf)
        << left;
  }
}

TEST(ArcCorridorOuterCorner, TheInnerEdgeIsUnchanged) {
  // The inside flank, R - half width, whatever the front x. (A sector of pi: the inner edge
  // at 1 rad of arc is about 1.22 rad off the head's axis.)
  constexpr double kEps = 1e-3;
  const double radius = full_lock_radius();
  for (const double front_x : {0.0, kBodyFrontX, 1.0}) {
    PathGeometry path = car_path();
    path.body_front_x_m = front_x;
    const Xy in = on_arc(radius, 1.0, -kHalfWidth + kEps, true);
    const Xy out = on_arc(radius, 1.0, -kHalfWidth - kEps, true);
    EXPECT_TRUE(std::isfinite(
        path_distance(OneReturn(in.x - kMountX, in.y, kCarYaw), kMaxSteer, path, kCarYaw, kPi)))
        << front_x;
    EXPECT_EQ(
        path_distance(OneReturn(out.x - kMountX, out.y, kCarYaw), kMaxSteer, path, kCarYaw, kPi),
        kInf)
        << front_x;
  }
}

TEST(ArcCorridorOuterCorner, TheStraightCorridorIsUnchanged) {
  // Going straight the corner runs along the side line, inside |y| <= half width already: the
  // front x changes nothing, edge for edge.
  constexpr double kEps = 1e-3;
  for (const double front_x : {0.0, kBodyFrontX, 1.0}) {
    PathGeometry path = car_path();
    path.body_front_x_m = front_x;
    for (const double steer : {0.0, 5e-4, -5e-4}) {
      EXPECT_NEAR(path_distance(OneReturn(0.5, kHalfWidth - kEps, kCarYaw), steer, path), 0.5,
                  1e-6);
      EXPECT_NEAR(path_distance(OneReturn(0.5, -kHalfWidth + kEps, kCarYaw), steer, path), 0.5,
                  1e-6);
      EXPECT_EQ(path_distance(OneReturn(0.5, kHalfWidth + kEps, kCarYaw), steer, path), kInf);
      EXPECT_EQ(path_distance(OneReturn(0.5, -kHalfWidth - kEps, kCarYaw), steer, path), kInf);
    }
    const CastScan scan(bag_wall());
    EXPECT_EQ(
        min_path_distance_m(scan.geometry, scan.ranges, kCarYaw, kHalfAngle, kHalfWidth, path, 0.0),
        min_corridor_distance_m(scan.geometry, scan.ranges, kCarYaw, kHalfAngle, kHalfWidth))
        << front_x;
  }
}

TEST(ArcCorridorOuterCorner, ARoundCornerWallIsSeenBeforeTheCornerReachesIt) {
  // The sim escape scenario: full lock left round a corner whose outside wall is a circle
  // about the turn centre (ray-cast, 720 rays). With the wall at the corner's no-margin sweep
  // (1.008 m) the car would scrape it; it must be in the path, ahead of the head. The
  // pre-0.11.0 band (0.947 m) never saw it.
  const double radius = full_lock_radius();
  const double wall_r = std::hypot(kBodyFrontX, radius + 0.155);
  std::vector<Segment> wall;
  for (int i = 0; i < 40; ++i) {
    const double a0 = 0.3 + 0.03 * i;
    const double a1 = a0 + 0.03;
    // On the circle about (0, R) in the rear-axle frame, then to the head frame.
    wall.push_back(Segment{wall_r * std::sin(a0) - kMountX, radius - wall_r * std::cos(a0),
                           wall_r * std::sin(a1) - kMountX, radius - wall_r * std::cos(a1)});
  }
  const CastScan scan(wall);
  const double d = scan.distance(kMaxSteer);
  EXPECT_TRUE(std::isfinite(d));
  EXPECT_GT(d, 0.0);
  PathGeometry old_band = car_path();
  old_band.body_front_x_m = 0.0;
  EXPECT_EQ(min_path_distance_m(scan.geometry, scan.ranges, kCarYaw, kHalfAngle, kHalfWidth,
                                old_band, kMaxSteer),
            kInf);
}

// ---------------------------------------------------------------------------------------
// Invalid returns and garbage inputs.
// ---------------------------------------------------------------------------------------

TEST(ArcCorridorGarbage, InvalidReturnsOnThePathAreIgnored) {
  const Xy p = on_arc(1.0, 0.8, 0.0, true);
  for (const float bad : {kFloatNan, kFloatInf, 0.0f, -0.5f, 0.03f, 13.0f}) {
    OneReturn r(p.x, p.y, kCarYaw);
    r.ranges[0] = bad;
    EXPECT_EQ(path_distance(r, kUnitSteer, unit_path()), kInf) << bad;
  }
}

TEST(ArcCorridorGarbage, GarbagePathInputFallsBackToTheWholeScanConservatively) {
  // Whole-scan minimum: a 0.10 m return straight behind the head, which no corridor counts.
  CastScan scan(clear_wall());
  scan.ranges[0] = 0.10f;  // ray 0 points straight back (vehicle bearing -pi)
  auto with = [&](auto mutate, double steer) {
    PathGeometry path = car_path();
    mutate(path);
    return min_path_distance_m(scan.geometry, scan.ranges, kCarYaw, kHalfAngle, kHalfWidth, path,
                               steer);
  };
  auto keep = [](PathGeometry&) {};
  EXPECT_EQ(with(keep, kMaxSteer), kInf);  // sanity: the full-lock-left arc is clear
  EXPECT_FLOAT_EQ(static_cast<float>(with(keep, kNan)), 0.10f);
  EXPECT_FLOAT_EQ(static_cast<float>(with(keep, kInf)), 0.10f);
  EXPECT_FLOAT_EQ(static_cast<float>(with([](PathGeometry& p) { p.wheelbase_m = kNan; }, 0.2)),
                  0.10f);
  EXPECT_FLOAT_EQ(static_cast<float>(with([](PathGeometry& p) { p.wheelbase_m = 0.0; }, 0.2)),
                  0.10f);
  EXPECT_FLOAT_EQ(static_cast<float>(with([](PathGeometry& p) { p.wheelbase_m = -0.33; }, 0.2)),
                  0.10f);
  EXPECT_FLOAT_EQ(
      static_cast<float>(with([](PathGeometry& p) { p.max_steering_angle_rad = kInf; }, 0.2)),
      0.10f);
  EXPECT_FLOAT_EQ(
      static_cast<float>(with([](PathGeometry& p) { p.max_steering_angle_rad = -0.1; }, 0.2)),
      0.10f);
  EXPECT_FLOAT_EQ(
      static_cast<float>(with([](PathGeometry& p) { p.max_steering_angle_rad = kPi / 2.0; }, 0.2)),
      0.10f);
  EXPECT_FLOAT_EQ(static_cast<float>(with([](PathGeometry& p) { p.lidar_mount_x_m = kNan; }, 0.2)),
                  0.10f);
  EXPECT_FLOAT_EQ(static_cast<float>(with([](PathGeometry& p) { p.lidar_mount_y_m = kInf; }, 0.2)),
                  0.10f);
  EXPECT_FLOAT_EQ(static_cast<float>(with([](PathGeometry& p) { p.body_front_x_m = kNan; }, 0.2)),
                  0.10f);
  EXPECT_FLOAT_EQ(static_cast<float>(with([](PathGeometry& p) { p.body_front_x_m = kInf; }, 0.2)),
                  0.10f);
  EXPECT_FLOAT_EQ(static_cast<float>(with([](PathGeometry& p) { p.body_front_x_m = -0.01; }, 0.2)),
                  0.10f);
  // Garbage scan geometry still falls back too, whatever the path.
  ScanGeometry bad_geometry = scan.geometry;
  bad_geometry.angle_increment_rad = 0.0;
  EXPECT_FLOAT_EQ(
      static_cast<float>(min_path_distance_m(bad_geometry, scan.ranges, kCarYaw, kHalfAngle,
                                             kHalfWidth, car_path(), kMaxSteer)),
      0.10f);
}

TEST(ArcCorridorGarbage, ZeroMaxSteeringMeansAlwaysStraight) {
  PathGeometry path = car_path();
  path.max_steering_angle_rad = 0.0;
  const CastScan scan(bag_wall());
  EXPECT_NEAR(min_path_distance_m(scan.geometry, scan.ranges, kCarYaw, kHalfAngle, kHalfWidth, path,
                                  kMaxSteer),
              0.33, 1e-6);
}

// ---------------------------------------------------------------------------------------
// The bag, through the gate (2026-10-06T22-12-40_car_teleop).
// ---------------------------------------------------------------------------------------

constexpr double kDt = 0.02;          // 50 Hz
constexpr double kBagSpeedMps = 1.0;  // gap_follow_node's request on the floor test

// The committed obstacle-gate values after this change (schema 0.9.2).
SafetyLimits committed_limits() {
  SafetyLimits limits;
  limits.steering_min_rad = -kMaxSteer;
  limits.steering_max_rad = kMaxSteer;
  limits.steering_rate_min_rad_per_s = -3.2;
  limits.steering_rate_max_rad_per_s = 3.2;
  limits.speed_min_mps = -5.0;
  limits.speed_max_mps = 20.0;
  limits.max_acceleration_mps2 = 9.51;
  limits.ttc_warning_s = 0.36;  // the release line
  limits.ttc_brake_s = 0.35;
  limits.min_forward_clearance_m = 0.20;
  limits.obstacle_steering_hold_after_s = 0.5;
  limits.watchdog_missed_cycles = 3;
  limits.control_period_s = kDt;
  return limits;
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

// safety_node's cycle: the last scan is reduced with THIS cycle's requested steering, then the
// gate runs; output, latch, hold timer and event tracker threaded from cycle to cycle.
struct NodeLoop {
  NodeLoop() : gate(committed_limits()) {}

  GateResult step(const CastScan& scan, double steering_rad, double speed_mps) {
    GateInput input;
    input.command = DriveCommand{steering_rad, speed_mps};
    input.drive_raw_age_s = 0.0;
    input.dt_s = kDt;
    input.min_scan_range_m = scan.distance(input.command.steering_angle_rad);
    input.ttc_brake_latched = latched;
    input.obstacle_hold_timer_s = hold_timer_s;
    GateResult result = gate.evaluate(input, previous);
    previous = result.output;
    latched = result.ttc_brake_latched;
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
  std::optional<double> hold_timer_s;
  double now_s = 0.0;
  std::vector<SafetyEventRecord> records;
};

// Latches on the wall at 1.0 m/s with a straight request (TTC 0.33 s at 0.33 m) and parks
// long enough for the steering hold to engage.
void latch_on_the_wall(NodeLoop& loop, const CastScan& scan) {
  for (int i = 0; i < 100; ++i) {  // 2 s
    const GateResult r = loop.step(scan, 0.0, kBagSpeedMps);
    ASSERT_TRUE(r.ttc_brake_latched) << "cycle " << i;
    ASSERT_EQ(r.output.speed_mps, 0.0) << "cycle " << i;
  }
  ASSERT_TRUE(loop.gate.steering_hold_engaged(loop.hold_timer_s));
  ASSERT_EQ(
      count_records(loop.records, GateSource::kTtc, EventSeverity::kBrake, EventPhase::kEngage),
      1u);
}

TEST(ArcCorridorBag, LatchedOnAWallReleasesOnTheCycleTheRequestSteersAwayAndTheArcIsClear) {
  // The bag's wall with its near end 15 cm right of the centreline (clear_wall): the bag's own
  // near end, 2 cm right, is inside the outer front corner's sweep (next test).
  const CastScan scan(clear_wall());
  NodeLoop loop;
  latch_on_the_wall(loop, scan);
  EXPECT_DOUBLE_EQ(loop.previous.steering_angle_rad, 0.0);  // held at the hold-start angle

  // gap_follow_node goes to full lock away from the wall. The held OUTPUT steering is still 0,
  // but the obstacle check uses the request's arc, which is clear: TTC +inf above the 0.36 s
  // release line, distance +inf above 1.5 x the 0.20 m floor. Release on the first cycle.
  const GateResult first = loop.step(scan, kMaxSteer, kBagSpeedMps);
  EXPECT_FALSE(first.ttc_brake_latched);
  EXPECT_FALSE(first.obstacle_hold_timer_s.has_value());
  EXPECT_EQ(
      count_records(loop.records, GateSource::kTtc, EventSeverity::kBrake, EventPhase::kRelease),
      1u);
  EXPECT_EQ(
      count_records(loop.records, GateSource::kTtc, EventSeverity::kInfo, EventPhase::kRelease),
      1u);  // "steering hold released"
  bool saw_release_detail = false;
  for (const auto& r : loop.records) {
    if (r.phase == EventPhase::kRelease && r.severity == EventSeverity::kBrake) {
      saw_release_detail = r.detail.find("ttc brake released") != std::string::npos;
    }
  }
  EXPECT_TRUE(saw_release_detail);
  // Rate-limited out of the latch: one acceleration step of speed, one rate step of steering.
  EXPECT_NEAR(first.output.speed_mps, 9.51 * kDt, 1e-9);
  EXPECT_NEAR(first.output.steering_angle_rad, 3.2 * kDt, 1e-9);

  // And it stays released while the request keeps steering away: the wheels reach full lock
  // and the speed ramps to the request.
  GateResult last = first;
  for (int i = 0; i < 25; ++i) {
    last = loop.step(scan, kMaxSteer, kBagSpeedMps);
    ASSERT_FALSE(last.ttc_brake_latched) << "cycle " << i;
  }
  EXPECT_NEAR(last.output.steering_angle_rad, kMaxSteer, 1e-9);
  EXPECT_NEAR(last.output.speed_mps, kBagSpeedMps, 1e-9);
  EXPECT_EQ(
      count_records(loop.records, GateSource::kTtc, EventSeverity::kBrake, EventPhase::kEngage),
      1u);
}

TEST(ArcCorridorBag, TheBagsOwnWallHoldsTheLatchAtFullLockAwayBecauseTheCornerWouldClipIt) {
  // Since schema 0.11.0 the bag geometry no longer releases by steering away: the right front
  // corner would sweep through the wall's near end (bag_wall). The car has to back out first
  // (gap_follow_node's reverse escape, through the rear corridor).
  const CastScan scan(bag_wall());
  NodeLoop loop;
  latch_on_the_wall(loop, scan);
  for (int i = 0; i < 10 * 50; ++i) {
    const GateResult r = loop.step(scan, kMaxSteer, kBagSpeedMps);
    ASSERT_TRUE(r.ttc_brake_latched) << "cycle " << i;
    ASSERT_EQ(r.output.speed_mps, 0.0);
  }
}

TEST(ArcCorridorBag, AStraightRequestOrFullLockIntoTheWallStillNeverReleases) {
  // Control: a straight request (what the straight corridor assumed whatever the steering) or
  // full lock INTO the wall keeps the latch for the bag's longest latch, 37 s.
  for (const double steer : {0.0, -kMaxSteer}) {
    const CastScan scan(bag_wall());
    NodeLoop loop;
    latch_on_the_wall(loop, scan);
    for (int i = 0; i < 37 * 50; ++i) {
      const GateResult r = loop.step(scan, steer, kBagSpeedMps);
      ASSERT_TRUE(r.ttc_brake_latched) << "steer " << steer << " cycle " << i;
      ASSERT_EQ(r.output.speed_mps, 0.0);
    }
  }
}

TEST(ArcCorridorBag, AnObstacleFurtherRoundTheArcReleasesOnlyPastTheHysteresis) {
  // The arc is not empty: one return on the full-lock-left path. Release needs the request's
  // TTC above 0.36 s and the distance above 0.30 m; between 0.35 s and 0.36 s it neither trips
  // nor releases, and the latch holds.
  const double radius = full_lock_radius();
  const double phi_head = std::atan2(kMountX, radius);
  auto scan_with_return_at = [&](double arc_m) {
    const Xy p = on_arc(radius, phi_head + arc_m / radius, 0.0, true);
    const double x = p.x - kMountX;
    const double y = p.y;
    CastScan scan(clear_wall(), std::atan2(y, x));  // ray 0 exactly on the return's bearing
    scan.ranges[0] = static_cast<float>(std::hypot(x, y));
    return scan;
  };

  const CastScan held_scan = scan_with_return_at(0.355);  // TTC 0.355 s at 1.0 m/s
  ASSERT_NEAR(held_scan.distance(kMaxSteer), 0.355, 1e-6);
  NodeLoop loop;
  latch_on_the_wall(loop, held_scan);
  for (int i = 0; i < 50; ++i) {
    const GateResult r = loop.step(held_scan, kMaxSteer, kBagSpeedMps);
    ASSERT_TRUE(r.ttc_brake_latched) << "cycle " << i;
    ASSERT_EQ(r.output.speed_mps, 0.0);
  }

  const CastScan clear_scan = scan_with_return_at(0.40);  // TTC 0.40 s, distance 0.40 m
  ASSERT_NEAR(clear_scan.distance(kMaxSteer), 0.40, 1e-6);
  const GateResult released = loop.step(clear_scan, kMaxSteer, kBagSpeedMps);
  EXPECT_FALSE(released.ttc_brake_latched);
  EXPECT_GT(released.output.speed_mps, 0.0);
}

}  // namespace
}  // namespace racer_safety
