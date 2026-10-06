// L1 tests for racer_safety's in-path obstacle distance (forward_sector.hpp): which /scan
// returns count, the car-width corridor (the primary filter, "corridor, not wedge", 2026-10-06
// floor test) inside the forward sector (the outer bound), the along-track distance x rather
// than the slant range, the real car's mount yaw (pi) and the simulator's (0), the
// conservative fallback on garbage geometry, and the yaw resolution rules shared with
// racer_control.
#include <gtest/gtest.h>

#include <cmath>
#include <cstddef>
#include <limits>
#include <optional>
#include <vector>

#include "racer_safety/forward_sector.hpp"

namespace racer_safety {
namespace {

constexpr double kPi = 3.14159265358979323846;
constexpr double kNan = std::numeric_limits<double>::quiet_NaN();
constexpr double kInf = std::numeric_limits<double>::infinity();
constexpr float kFloatNan = std::numeric_limits<float>::quiet_NaN();
constexpr float kFloatInf = std::numeric_limits<float>::infinity();

// Test inputs shaped like the committed config (the code under test takes them as arguments;
// safety_node feeds the real ones from the generated binding).
constexpr double kHalfAngle = 0.6;    // limits.ttc_forward_sector_half_angle_rad, the outer bound
constexpr double kHalfWidth = 0.205;  // chassis.width_m / 2 + limits.obstacle_corridor_margin_m
constexpr double kCarYaw = kPi;       // vehicle_params sensors.lidar.mount_yaw_rad on the car

double deg(double degrees) { return degrees * kPi / 180.0; }

// RPLIDAR C1 as seen on the 2026-10-06 bench: 720 beams over a full turn, angle_min -pi,
// range_min 0.05 m. Every ray starts as "no return" (0.0, the driver's encoding).
struct C1Scan {
  C1Scan() : ranges(720, 0.0f) {
    geometry.angle_min_rad = -kPi;
    geometry.angle_increment_rad = 2.0 * kPi / 720.0;
    geometry.range_min_m = 0.05;
    geometry.range_max_m = 12.0;
  }
  // Puts a return at the ray nearest a LASER bearing.
  void set(double laser_bearing_rad, float range_m) {
    const double offset = laser_bearing_rad - geometry.angle_min_rad;
    const auto i = static_cast<std::size_t>(std::llround(offset / geometry.angle_increment_rad));
    ranges[i % ranges.size()] = range_m;
  }
  ScanGeometry geometry;
  std::vector<float> ranges;
};

// The bench run's three things in view: a box straight ahead of the nose (read at laser
// bearing 177 deg, 0.51 m), a box on the driver's left (laser -100 deg, 0.37 m), and
// something close BEHIND the car (laser 0 deg, i.e. behind on a head rotated by pi), 0.10 m.
C1Scan bench_scene() {
  C1Scan scan;
  scan.set(deg(177.0), 0.51f);
  scan.set(deg(-100.0), 0.37f);
  scan.set(deg(0.0), 0.10f);
  return scan;
}

// ---------------------------------------------------------------------------------------
// wrap_angle_rad
// ---------------------------------------------------------------------------------------

TEST(WrapAngle, WrapsIntoMinusPiToPi) {
  EXPECT_DOUBLE_EQ(wrap_angle_rad(0.0), 0.0);
  EXPECT_NEAR(wrap_angle_rad(kPi), -kPi, 1e-12);
  EXPECT_NEAR(wrap_angle_rad(1.5 * kPi), -0.5 * kPi, 1e-12);
  EXPECT_NEAR(wrap_angle_rad(-1.5 * kPi), 0.5 * kPi, 1e-12);
  EXPECT_NEAR(wrap_angle_rad(deg(177.0) + kPi), deg(-3.0), 1e-12);
  EXPECT_TRUE(std::isnan(wrap_angle_rad(kNan)));
}

// ---------------------------------------------------------------------------------------
// is_usable_return: the invalid-return policy.
// ---------------------------------------------------------------------------------------

TEST(UsableReturn, InvalidReturnsAreIgnored) {
  const ScanGeometry g = C1Scan().geometry;
  EXPECT_FALSE(is_usable_return(kFloatNan, g));
  EXPECT_FALSE(is_usable_return(kFloatInf, g));
  EXPECT_FALSE(is_usable_return(-kFloatInf, g));
  EXPECT_FALSE(is_usable_return(0.0f, g));
  EXPECT_FALSE(is_usable_return(-0.5f, g));
  EXPECT_FALSE(is_usable_return(0.03f, g));  // below range_min 0.05
  EXPECT_FALSE(is_usable_return(12.5f, g));  // above range_max 12
  EXPECT_TRUE(is_usable_return(0.05f, g));   // exactly range_min counts
  EXPECT_TRUE(is_usable_return(12.0f, g));   // exactly range_max counts
  EXPECT_TRUE(is_usable_return(0.22f, g));
}

TEST(UsableReturn, NonFiniteBoundsAreNotApplied) {
  ScanGeometry g = C1Scan().geometry;
  g.range_min_m = kNan;
  g.range_max_m = kInf;
  EXPECT_TRUE(is_usable_return(0.01f, g));
  EXPECT_TRUE(is_usable_return(500.0f, g));
  EXPECT_FALSE(is_usable_return(0.0f, g));
}

// ---------------------------------------------------------------------------------------
// corridor_half_width_m and min_corridor_distance_m: the in-path test (corridor, not wedge),
// with the real mount yaw (pi) and the sim's (0).
// ---------------------------------------------------------------------------------------

// A scan holding exactly one return at an exact VEHICLE bearing: angle_min is chosen so that
// ray 0 lands on `vehicle_bearing_rad` after `yaw_rad`, so the edge tests below are not at the
// mercy of the C1's 0.5 degree ray spacing.
struct OneReturn {
  OneReturn(double vehicle_bearing_rad, float range_m, double yaw_rad) : ranges{range_m} {
    geometry.angle_min_rad = vehicle_bearing_rad - yaw_rad;
    geometry.angle_increment_rad = 0.01;
    geometry.range_min_m = 0.05;
    geometry.range_max_m = 12.0;
  }
  ScanGeometry geometry;
  std::vector<float> ranges;
};

// A return at vehicle-frame (x ahead, y left), as range and bearing.
OneReturn at_xy(double x_m, double y_m, double yaw_rad) {
  return OneReturn(std::atan2(y_m, x_m), static_cast<float>(std::hypot(x_m, y_m)), yaw_rad);
}

double corridor(const OneReturn& r, double yaw_rad, double half_angle_rad, double half_width_m) {
  return min_corridor_distance_m(r.geometry, r.ranges, yaw_rad, half_angle_rad, half_width_m);
}

TEST(CorridorHalfWidth, IsHalfTheChassisWidthPlusTheMargin) {
  // Test inputs shaped like the committed chassis.width_m and the PROVISIONAL margin.
  EXPECT_DOUBLE_EQ(corridor_half_width_m(0.31, 0.05), 0.205);
  EXPECT_DOUBLE_EQ(corridor_half_width_m(0.31, 0.0), 0.155);
  EXPECT_DOUBLE_EQ(corridor_half_width_m(0.20, 0.10), 0.20);
}

TEST(Corridor, TheFloorTestBagBesideThePathIsIgnored) {
  // 2026-10-06 floor test: a bag 0.3 m to the side of the car's path. At 0.6 m ahead and
  // 0.3 m left of the centreline it is at bearing 0.46 rad, inside the 0.6 rad wedge, so the
  // old wedge braked on its 0.67 m slant range. It is outside the 0.205 m corridor.
  for (const double yaw : {kCarYaw, 0.0}) {
    EXPECT_EQ(corridor(at_xy(0.6, 0.3, yaw), yaw, kHalfAngle, kHalfWidth), kInf);
    EXPECT_EQ(corridor(at_xy(0.6, -0.3, yaw), yaw, kHalfAngle, kHalfWidth), kInf);
    // The same bag straight ahead is counted.
    EXPECT_NEAR(corridor(at_xy(0.6, 0.0, yaw), yaw, kHalfAngle, kHalfWidth), 0.6, 1e-6);
  }
}

TEST(Corridor, JustOutsideTheCorridorIsIgnoredJustInsideIsCounted) {
  constexpr double kEps = 1e-3;
  for (const double yaw : {kCarYaw, 0.0}) {
    for (const double side : {1.0, -1.0}) {  // left and right
      EXPECT_EQ(corridor(at_xy(0.5, side * (kHalfWidth + kEps), yaw), yaw, kHalfAngle, kHalfWidth),
                kInf);
      EXPECT_NEAR(
          corridor(at_xy(0.5, side * (kHalfWidth - kEps), yaw), yaw, kHalfAngle, kHalfWidth), 0.5,
          1e-6);
    }
  }
}

TEST(Corridor, TheDistanceIsAlongTrackXNotSlantRange) {
  // x 0.40, y 0.15: slant range 0.427 m. The gate must see 0.40.
  const OneReturn r = at_xy(0.40, 0.15, kCarYaw);
  const double distance = corridor(r, kCarYaw, kHalfAngle, kHalfWidth);
  EXPECT_NEAR(distance, 0.40, 1e-6);
  EXPECT_GT(static_cast<double>(r.ranges[0]) - distance, 0.02);
}

TEST(Corridor, CarMountYawPiSeesTheBoxAheadAndIgnoresBehindAndBeside) {
  // Bench scene: box ahead at laser 177 deg (vehicle -3 deg, y = -0.027 m: in the corridor),
  // box on the left at vehicle +80 deg (outside the sector), 0.10 m return behind the car.
  const C1Scan scan = bench_scene();
  const double distance =
      min_corridor_distance_m(scan.geometry, scan.ranges, kCarYaw, kHalfAngle, kHalfWidth);
  EXPECT_NEAR(distance, 0.51f * std::cos(deg(3.0)), 1e-6);
}

TEST(Corridor, YawZeroOnTheSameScanLooksTheOtherWay) {
  // With yaw 0 (the simulator's convention) laser bearing 0 IS ahead, so the 0.10 m return
  // that is behind the real car is what counts. This is why the sim must not use the car's yaw.
  const C1Scan scan = bench_scene();
  EXPECT_NEAR(min_corridor_distance_m(scan.geometry, scan.ranges, 0.0, kHalfAngle, kHalfWidth),
              0.10, 1e-6);
}

TEST(Corridor, SimStyleScanWithYawZeroSeesStraightAhead) {
  // racer_gym_bridge-shaped scan: 1080 beams over 4.7 rad centred on the vehicle's +x axis.
  ScanGeometry g;
  g.angle_min_rad = -2.35;
  g.angle_increment_rad = 4.7 / 1079.0;
  g.range_min_m = 0.0;
  g.range_max_m = 30.0;
  std::vector<float> ranges(1080, 10.0f);
  ranges[540] = 0.4f;  // straight ahead (bearing 0.002 rad)
  ranges[0] = 0.2f;    // far right (-2.35 rad), outside the sector
  EXPECT_NEAR(min_corridor_distance_m(g, ranges, 0.0, kHalfAngle, kHalfWidth), 0.4, 1e-5);
}

TEST(Corridor, BehindTheCarIsIgnoredEvenWithAWideSectorAndAHugeMargin) {
  // A sector wider than pi/2 lets bearings behind the head through the outer bound; x > 0 is
  // what keeps them out. Half width 10 m would take in anything at all beside the car.
  for (const double yaw : {kCarYaw, 0.0}) {
    EXPECT_EQ(corridor(OneReturn(2.5, 0.3f, yaw), yaw, 3.0, 10.0), kInf);
    EXPECT_EQ(corridor(OneReturn(-2.5, 0.3f, yaw), yaw, 3.0, 10.0), kInf);
    EXPECT_EQ(corridor(at_xy(-0.2, 0.0, yaw), yaw, 3.0, 10.0), kInf);
  }
  // Nearly beside but still ahead (bearing 1.5 rad, r 0.1: x 0.007, y 0.0997) is in the path.
  EXPECT_NEAR(corridor(OneReturn(1.5, 0.1f, kCarYaw), kCarYaw, 3.0, kHalfWidth),
              0.1f * std::cos(1.5), 1e-6);
}

TEST(Corridor, TheSectorBoundStillAppliesWhateverTheMargin) {
  // Half width 10 m: the corridor alone would take everything ahead. The 0.6 rad outer
  // bound still drops a return at 0.7 rad and keeps one at 0.55 rad.
  EXPECT_EQ(corridor(OneReturn(0.7, 0.5f, kCarYaw), kCarYaw, kHalfAngle, 10.0), kInf);
  EXPECT_EQ(corridor(OneReturn(-0.7, 0.5f, kCarYaw), kCarYaw, kHalfAngle, 10.0), kInf);
  EXPECT_NEAR(corridor(OneReturn(0.55, 0.5f, kCarYaw), kCarYaw, kHalfAngle, 10.0),
              0.5f * std::cos(0.55), 1e-6);
}

TEST(Corridor, CloseToTheHeadTheSectorNotTheCorridorIsTheLimit) {
  // Documented limitation of the committed values (forward_sector.hpp, "CORRIDOR, NOT WEDGE"):
  // x 0.20, y 0.15 is inside the 0.205 m corridor but at bearing 0.64 rad, outside 0.6, so it
  // is not seen. The crossover is x = 0.205 / tan(0.6), about 0.30 m.
  EXPECT_EQ(corridor(at_xy(0.20, 0.15, kCarYaw), kCarYaw, kHalfAngle, kHalfWidth), kInf);
  EXPECT_NEAR(corridor(at_xy(0.32, 0.20, kCarYaw), kCarYaw, kHalfAngle, kHalfWidth), 0.32, 1e-6);
}

TEST(Corridor, TheDriversLeftBoxLandsAtEightyDegreesFromAhead) {
  // Laser -100 deg on the car is vehicle +80 deg (the driver's left). With a corridor wide
  // enough to hold it (1 m), a +/- 1.5 rad (86 deg) sector includes it and a +/- 70 deg sector
  // does not, which pins the yaw arithmetic. Its distance is x = 0.37 cos(80 deg).
  C1Scan scan;
  scan.set(deg(-100.0), 0.37f);
  EXPECT_NEAR(min_corridor_distance_m(scan.geometry, scan.ranges, kCarYaw, 1.5, 1.0),
              0.37f * std::cos(deg(80.0)), 1e-6);
  EXPECT_EQ(min_corridor_distance_m(scan.geometry, scan.ranges, kCarYaw, deg(70.0), 1.0), kInf);
  // With the car-width corridor it is beside the car (y = 0.364 m) and never counts.
  EXPECT_EQ(min_corridor_distance_m(scan.geometry, scan.ranges, kCarYaw, 1.5, kHalfWidth), kInf);
}

TEST(Corridor, InvalidReturnsInThePathAreIgnored) {
  C1Scan scan;
  scan.set(deg(180.0), 0.0f);  // driver "no return"
  scan.set(deg(181.0), kFloatNan);
  scan.set(deg(182.0), kFloatInf);
  scan.set(deg(183.0), 0.03f);  // below range_min
  scan.set(deg(184.0), -1.0f);
  scan.set(deg(185.0), 0.8f);  // the only real return ahead (vehicle 5 deg, y 0.07 m)
  EXPECT_NEAR(min_corridor_distance_m(scan.geometry, scan.ranges, kCarYaw, kHalfAngle, kHalfWidth),
              0.8f * std::cos(deg(5.0)), 1e-6);
}

TEST(Corridor, NothingUsableInThePathIsInfinity) {
  const C1Scan empty_returns;
  EXPECT_EQ(min_corridor_distance_m(empty_returns.geometry, empty_returns.ranges, kCarYaw,
                                    kHalfAngle, kHalfWidth),
            kInf);
  EXPECT_EQ(min_corridor_distance_m(empty_returns.geometry, {}, kCarYaw, kHalfAngle, kHalfWidth),
            kInf);
}

TEST(Corridor, TakesTheNearestAlongTrackOfSeveralReturnsInThePath) {
  C1Scan scan;
  scan.set(deg(176.0), 0.9f);  // vehicle -4 deg
  scan.set(deg(182.0), 0.4f);  // vehicle +2 deg: nearest
  scan.set(deg(185.0), 0.7f);  // vehicle +5 deg
  EXPECT_NEAR(min_corridor_distance_m(scan.geometry, scan.ranges, kCarYaw, kHalfAngle, kHalfWidth),
              0.4f * std::cos(deg(2.0)), 1e-6);
}

TEST(Corridor, GarbageGeometryFallsBackToTheWholeScanConservatively) {
  const C1Scan scene = bench_scene();  // whole-scan minimum is the 0.10 m return behind
  auto with = [&](auto mutate, double yaw, double half_angle, double half_width) {
    C1Scan s = scene;
    mutate(s.geometry);
    return static_cast<float>(
        min_corridor_distance_m(s.geometry, s.ranges, yaw, half_angle, half_width));
  };
  auto keep = [](ScanGeometry&) {};
  EXPECT_FLOAT_EQ(
      with([](ScanGeometry& g) { g.angle_min_rad = kNan; }, kCarYaw, kHalfAngle, kHalfWidth),
      0.10f);
  EXPECT_FLOAT_EQ(
      with([](ScanGeometry& g) { g.angle_increment_rad = kInf; }, kCarYaw, kHalfAngle, kHalfWidth),
      0.10f);
  EXPECT_FLOAT_EQ(
      with([](ScanGeometry& g) { g.angle_increment_rad = 0.0; }, kCarYaw, kHalfAngle, kHalfWidth),
      0.10f);
  EXPECT_FLOAT_EQ(
      with([](ScanGeometry& g) { g.angle_increment_rad = -0.01; }, kCarYaw, kHalfAngle, kHalfWidth),
      0.10f);
  EXPECT_FLOAT_EQ(with(keep, kNan, kHalfAngle, kHalfWidth), 0.10f);
  EXPECT_FLOAT_EQ(with(keep, kCarYaw, kNan, kHalfWidth), 0.10f);
  EXPECT_FLOAT_EQ(with(keep, kCarYaw, 0.0, kHalfWidth), 0.10f);
  EXPECT_FLOAT_EQ(with(keep, kCarYaw, -1.0, kHalfWidth), 0.10f);
  EXPECT_FLOAT_EQ(with(keep, kCarYaw, kHalfAngle, kNan), 0.10f);
  EXPECT_FLOAT_EQ(with(keep, kCarYaw, kHalfAngle, kInf), 0.10f);
  EXPECT_FLOAT_EQ(with(keep, kCarYaw, kHalfAngle, 0.0), 0.10f);
  EXPECT_FLOAT_EQ(with(keep, kCarYaw, kHalfAngle, -0.2), 0.10f);
}

// ---------------------------------------------------------------------------------------
// resolve_laser_yaw_rad: same rules as racer_control's resolve_laser_yaw_offset.
// ---------------------------------------------------------------------------------------

TEST(ResolveLaserYaw, CarDefaultUsesTheBinding) {
  EXPECT_EQ(resolve_laser_yaw_rad(kPi, 0.0, true), std::optional<double>(kPi));
}

TEST(ResolveLaserYaw, AgreeingParameterIsAccepted) {
  EXPECT_EQ(resolve_laser_yaw_rad(kPi, kPi, true), std::optional<double>(kPi));
}

TEST(ResolveLaserYaw, DisagreeingParameterRefusesToStart) {
  EXPECT_FALSE(resolve_laser_yaw_rad(kPi, 0.5, true).has_value());
}

TEST(ResolveLaserYaw, UnsetBindingUsesTheParameter) {
  EXPECT_EQ(resolve_laser_yaw_rad(std::nullopt, 0.5, true), std::optional<double>(0.5));
}

TEST(ResolveLaserYaw, SimFixtureIgnoresTheBinding) {
  EXPECT_EQ(resolve_laser_yaw_rad(kPi, 0.0, false), std::optional<double>(0.0));
}

TEST(ResolveLaserYaw, NonFiniteValuesRefuse) {
  EXPECT_FALSE(resolve_laser_yaw_rad(kPi, kNan, true).has_value());
  EXPECT_FALSE(resolve_laser_yaw_rad(kPi, kInf, false).has_value());
  EXPECT_FALSE(resolve_laser_yaw_rad(kNan, 0.0, true).has_value());
}

}  // namespace
}  // namespace racer_safety
